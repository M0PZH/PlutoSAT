// Five programmable narrowband IQ channels from the AD9363 receive samples,
// written to DDR ring buffers (one per channel). AXI-Lite registers, an
// AXI3 write master of DW = 32 or 64 bits (S_AXI_GP / S_AXI_HP), a periodic
// interrupt, and the FIR
// output stream of all channels as a tap (for the PL FFT).
//
// Channel: source (rx0 / rx1) -> NCO + mixer -> CIC /R -> FIR /2 -> ring.
// Output rate fs / (2R), R = 2..128; model and filter design: model/nbiq.py.
//
// Registers (32-bit, byte addresses; unlisted addresses read 0, writes are
// ignored):
//   0x000 VERSION     0x0002_0000
//   0x004 ID          "NBIQ" (0x4E424951)
//   0x008 CTRL        [0] interrupt enable
//   0x00C IRQ_PERIOD  interrupt period in clocks (0: off)
//   0x010 IRQ_STATUS  [0] period elapsed; write 1 to clear
//   0x014 ADC_CNT_LO  ADC samples received (reading it latches ADC_CNT_HI)
//   0x018 ADC_CNT_HI
//   0x01C INFO        (ro) [7:0] channels (5), [15:8] max taps/2 (127), [23:16] R max (128)
//   channel c = 0..4 at 0x100 + 0x40 c:
//   +0x00 CTRL        [0] enable (0 clears the channel), [1] source rx1
//   +0x04 FINC        NCO step: frequency finc / 2^32 * fs goes to DC
//   +0x08 CIC         [7:0] R (2..128), [12:8] cshift    (change while disabled)
//   +0x0C FIR         [6:0] npairs (1..127), [13:8] oshift (change while disabled)
//   +0x10 RING_BASE   physical address, aligned to RING_MASK + 1
//   +0x14 RING_MASK   ring size - 1 (size a power of two >= 4096)
//   +0x18 WPTR        (ro) bytes written to the ring since enable, wraps at 2^32
//   +0x1C T0_LO       (ro) ADC_CNT at enable: the first ADC sample of the channel
//   +0x20 T0_HI       (ro)
//   +0x24 DROPS       (ro) [15:0] FIR overruns, [31:16] staging overflows
//   0x2000 + 0x400 c + 4 i : FIR tap h[i] of channel c, i = 0..253 (18-bit signed, wo)
//
// Ring contents: 32-bit words {Q[15:0], I[15:0]}, little-endian. The k-th
// sample of a channel is FIR output m = k + npairs - 1, i.e. it ends at ADC
// sample T0 + 2 R m + 2 R - 1.
//
// Tap (s_axi_aclk domain): every sample handed to the ring writer, one
// clock each with tap_valid, its channel in tap_ch, {tap_q, tap_i} as in the
// ring (a sample the writer drops on a staging overflow is still on the
// tap). At most one sample per clock; per channel one per 2 R ADC samples.

module nbiq_axi #(
  parameter string LUT_FILE = "nbiq_lut.mem",
  parameter int    NCH = 5,
  parameter int    DW  = 32
) (
  input  logic        s_axi_aclk,
  input  logic        s_axi_aresetn,
  input  logic [15:0] s_axi_awaddr,
  input  logic        s_axi_awvalid,
  output logic        s_axi_awready,
  input  logic [31:0] s_axi_wdata,
  input  logic        s_axi_wvalid,
  output logic        s_axi_wready,
  output logic [1:0]  s_axi_bresp,
  output logic        s_axi_bvalid,
  input  logic        s_axi_bready,
  input  logic [15:0] s_axi_araddr,
  input  logic        s_axi_arvalid,
  output logic        s_axi_arready,
  output logic [31:0] s_axi_rdata,
  output logic [1:0]  s_axi_rresp,
  output logic        s_axi_rvalid,
  input  logic        s_axi_rready,

  output logic [31:0] m_axi_awaddr,
  output logic [3:0]  m_axi_awlen,
  output logic [2:0]  m_axi_awsize,
  output logic [1:0]  m_axi_awburst,
  output logic [3:0]  m_axi_awcache,
  output logic        m_axi_awvalid,
  input  logic        m_axi_awready,
  output logic [DW-1:0]   m_axi_wdata,
  output logic [DW/8-1:0] m_axi_wstrb,
  output logic        m_axi_wlast,
  output logic        m_axi_wvalid,
  input  logic        m_axi_wready,
  input  logic [1:0]  m_axi_bresp,
  input  logic        m_axi_bvalid,
  output logic        m_axi_bready,

  output logic        irq,

  output logic        tap_valid,
  output logic [2:0]  tap_ch,
  output logic [15:0] tap_i,
  output logic [15:0] tap_q,

  input  logic        adc_clk,
  input  logic        adc_valid,
  input  logic [15:0] adc_data_i0,
  input  logic [15:0] adc_data_q0,
  input  logic [15:0] adc_data_i1,
  input  logic [15:0] adc_data_q1
);

  wire clk = s_axi_aclk;
  assign s_axi_bresp = 2'b00;
  assign s_axi_rresp = 2'b00;

  // ---------------------------------------------------------------- reset
  logic [3:0] srst_cnt;
  logic       rst, aresetn_q;
  always_ff @(posedge clk) begin
    aresetn_q <= s_axi_aresetn;
    if (!aresetn_q)         srst_cnt <= '1;
    else if (srst_cnt != 0) srst_cnt <= srst_cnt - 1'b1;
    rst <= !aresetn_q || (srst_cnt != 0);
  end

  // ---------------------------------------------------------------- registers
  logic [NCH-1:0] en, src;
  logic [31:0]    finc [NCH], base [NCH], mask [NCH];
  logic [7:0]     R [NCH];
  logic [6:0]     npairs [NCH];
  logic [4:0]     cshift [NCH];
  logic [5:0]     oshift [NCH];
  logic [63:0]    t0 [NCH];
  logic           irq_en, irq_st;
  logic [31:0]    irq_period, irq_cnt;
  logic [63:0]    adc_cnt;
  logic           coef_we;
  logic [2:0]     coef_ch;
  logic [7:0]     coef_idx;
  logic [17:0]    coef_data;

  logic        a_valid;
  logic [63:0] a_data;

  logic        aw_hold, w_hold;
  logic [15:0] awaddr_q;
  logic [31:0] wdata_q;

  // channel c at 0x100 + 0x40 c: address bits [9:6] = c + 4
  logic [3:0]  wcx;
  logic [2:0]  wch;
  logic        wch_ok;
  always_comb begin
    wcx = awaddr_q[9:6] - 4'd4;
    wch = wcx[2:0];
    wch_ok = awaddr_q[15:10] == 6'd0 && awaddr_q[9:6] >= 4'd4 && wcx < 4'(NCH);
  end
  assign s_axi_awready = !aw_hold;
  assign s_axi_wready  = !w_hold;

  always_ff @(posedge clk) begin
    coef_we <= 1'b0;
    if (rst) begin
      aw_hold <= 1'b0; w_hold <= 1'b0; s_axi_bvalid <= 1'b0;
      en <= '0; src <= '0;
      irq_en <= 1'b0; irq_st <= 1'b0; irq_period <= '0; irq_cnt <= '0;
      for (int c = 0; c < NCH; c++) begin
        finc[c] <= '0; base[c] <= '0; mask[c] <= 32'hFFF;
        R[c] <= 8'd6; cshift[c] <= '0; npairs[c] <= 7'd1; oshift[c] <= '0;
      end
    end else begin
      // interrupt timer
      if (irq_period == 0) irq_cnt <= '0;
      else if (irq_cnt + 32'd1 >= irq_period) begin irq_cnt <= '0; irq_st <= 1'b1; end
      else irq_cnt <= irq_cnt + 32'd1;

      if (s_axi_awvalid && s_axi_awready) begin aw_hold <= 1'b1; awaddr_q <= s_axi_awaddr; end
      if (s_axi_wvalid && s_axi_wready)   begin w_hold  <= 1'b1; wdata_q  <= s_axi_wdata;  end
      if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
      if (aw_hold && w_hold && !s_axi_bvalid) begin
        aw_hold <= 1'b0; w_hold <= 1'b0; s_axi_bvalid <= 1'b1;
        if (awaddr_q[15:13] == 3'b001) begin
          coef_we <= awaddr_q[12:10] < 3'(NCH);
          coef_ch <= awaddr_q[12:10];
          coef_idx <= awaddr_q[9:2];
          coef_data <= wdata_q[17:0];
        end else if (wch_ok) begin : chw
          automatic logic [2:0] c = wch;
          case (awaddr_q[5:2])
            4'h0: begin
              en[c] <= wdata_q[0];
              src[c] <= wdata_q[1];
              if (wdata_q[0] && !en[c]) t0[c] <= adc_cnt + 64'(a_valid);
            end
            4'h1: finc[c] <= wdata_q;
            4'h2: begin R[c] <= wdata_q[7:0]; cshift[c] <= wdata_q[12:8]; end
            4'h3: begin npairs[c] <= wdata_q[6:0]; oshift[c] <= wdata_q[13:8]; end
            4'h4: base[c] <= wdata_q;
            4'h5: mask[c] <= wdata_q;
            default: ;
          endcase
        end else if (awaddr_q[15:8] == 8'd0)
          case (awaddr_q[7:2])
            6'h02: irq_en <= wdata_q[0];
            6'h03: irq_period <= wdata_q;
            6'h04: if (wdata_q[0]) irq_st <= 1'b0;
            default: ;
          endcase
      end
    end
  end

  always_ff @(posedge clk) irq <= irq_st && irq_en;

  // ---------------------------------------------------------------- ADC -> clk
  (* ASYNC_REG = "TRUE" *) logic [1:0] rst_s;
  always_ff @(posedge adc_clk) rst_s <= {rst_s[0], rst};

  // popped at most every other clock (the channels take I and Q in turn);
  // the FIFO evens out any bunching of the ADC samples
  logic a_avail, a_last;
  dvbs2_async_fifo #(.W(64), .AW(4)) u_adc_fifo (
    .wclk(adc_clk), .wrst(rst_s[1]), .w_valid(adc_valid), .w_ready(),
    .w_data({adc_data_q1, adc_data_i1, adc_data_q0, adc_data_i0}),
    .rclk(clk), .rrst(rst), .r_valid(a_avail), .r_ready(!a_last), .r_data(a_data));
  assign a_valid = a_avail && !a_last;
  always_ff @(posedge clk) a_last <= !rst && a_valid;

  always_ff @(posedge clk)
    if (rst) adc_cnt <= '0;
    else if (a_valid) adc_cnt <= adc_cnt + 64'd1;

  // ---------------------------------------------------------------- channels
  logic [NCH-1:0]     c_v;
  logic signed [52:0] c_i [NCH], c_q [NCH];

  for (genvar c = 0; c < NCH; c++) begin : g_ch
    // the source select and the enable apply to the same ADC samples:
    // a sample popped while en (and src) is set goes through
    logic               x_v, x_en;
    logic signed [15:0] x_i, x_q;
    always_ff @(posedge clk) begin
      x_v <= a_valid;
      x_en <= en[c];
      x_i <= src[c] ? a_data[47:32] : a_data[15:0];
      x_q <= src[c] ? a_data[63:48] : a_data[31:16];
    end
    nbiq_chan #(.LUT_FILE(LUT_FILE), .W(53)) u_chan (
      .clk, .en(x_en), .finc(finc[c]), .R(R[c]),
      .s_valid(x_v), .s_i(x_i), .s_q(x_q),
      .m_valid(c_v[c]), .m_i(c_i[c]), .m_q(c_q[c]));
  end

  logic               k_v;
  logic [2:0]         k_ch;
  logic signed [23:0] k_i, k_q;
  nbiq_comb #(.NCH(NCH), .W(53)) u_comb (
    .clk, .en, .cshift,
    .s_valid(c_v), .s_i(c_i), .s_q(c_q),
    .m_valid(k_v), .m_ch(k_ch), .m_i(k_i), .m_q(k_q));

  logic               f_v;
  logic [2:0]         f_ch;
  logic signed [15:0] f_i, f_q;
  logic [15:0]        st_drop [NCH], st_ovf [NCH];
  logic [31:0]        wptr [NCH];

  nbiq_fir #(.NCH(NCH)) u_fir (
    .clk, .en, .npairs, .oshift,
    .coef_we, .coef_ch, .coef_idx, .coef_data(coef_data),
    .s_valid(k_v), .s_ch(k_ch), .s_i(k_i), .s_q(k_q),
    .m_valid(f_v), .m_ch(f_ch), .m_i(f_i), .m_q(f_q), .st_drop);

  // tap: the samples the ring writer takes (it ignores those of a channel
  // being disabled)
  always_ff @(posedge clk) begin
    tap_valid <= f_v && en[f_ch];
    tap_ch <= f_ch;
    tap_i <= f_i;
    tap_q <= f_q;
  end

  nbiq_wr #(.NCH(NCH), .DW(DW)) u_wr (
    .clk, .rst, .en, .base, .mask,
    .s_valid(f_v), .s_ch(f_ch), .s_data({f_q, f_i}),
    .wptr, .st_ovf,
    .m_axi_awaddr, .m_axi_awlen, .m_axi_awsize, .m_axi_awburst, .m_axi_awcache,
    .m_axi_awvalid, .m_axi_awready, .m_axi_wdata, .m_axi_wstrb, .m_axi_wlast,
    .m_axi_wvalid, .m_axi_wready, .m_axi_bresp, .m_axi_bvalid, .m_axi_bready);

  // ---------------------------------------------------------------- AXI-Lite read
  logic        ar_p;
  logic [15:0] ar_q;
  logic [31:0] cnt_hi;
  logic [3:0]  rcx;
  logic [2:0]  rch;
  logic        rch_ok;
  always_comb begin
    rcx = ar_q[9:6] - 4'd4;
    rch = rcx[2:0];
    rch_ok = ar_q[15:10] == 6'd0 && ar_q[9:6] >= 4'd4 && rcx < 4'(NCH);
  end
  always_ff @(posedge clk) begin
    if (rst) begin
      s_axi_arready <= 1'b1; s_axi_rvalid <= 1'b0;
      ar_p <= 1'b0;
    end else begin
      ar_p <= 1'b0;
      if (s_axi_arvalid && s_axi_arready) begin
        s_axi_arready <= 1'b0;
        ar_p <= 1'b1;
        ar_q <= s_axi_araddr;
      end
      if (ar_p) begin : rd
        automatic logic [2:0] c = rch;
        s_axi_rvalid <= 1'b1;
        s_axi_rdata <= '0;
        if (rch_ok)
          case (ar_q[5:2])
            4'h0: s_axi_rdata <= {30'd0, src[c], en[c]};
            4'h1: s_axi_rdata <= finc[c];
            4'h2: s_axi_rdata <= {19'd0, cshift[c], R[c]};
            4'h3: s_axi_rdata <= {18'd0, oshift[c], 1'b0, npairs[c]};
            4'h4: s_axi_rdata <= base[c];
            4'h5: s_axi_rdata <= mask[c];
            4'h6: s_axi_rdata <= wptr[c];
            4'h7: s_axi_rdata <= t0[c][31:0];
            4'h8: s_axi_rdata <= t0[c][63:32];
            4'h9: s_axi_rdata <= {st_ovf[c], st_drop[c]};
            default: ;
          endcase
        else if (ar_q[15:8] == 8'd0)
          case (ar_q[7:2])
            6'h00: s_axi_rdata <= 32'h0002_0000;
            6'h01: s_axi_rdata <= 32'h4E42_4951;
            6'h02: s_axi_rdata <= {31'd0, irq_en};
            6'h03: s_axi_rdata <= irq_period;
            6'h04: s_axi_rdata <= {31'd0, irq_st};
            6'h05: begin s_axi_rdata <= adc_cnt[31:0]; cnt_hi <= adc_cnt[63:32]; end
            6'h06: s_axi_rdata <= cnt_hi;
            6'h07: s_axi_rdata <= {8'd0, 8'd128, 8'd127, 8'(NCH)};
            default: ;
          endcase
      end
      if (s_axi_rvalid && s_axi_rready) begin
        s_axi_rvalid <= 1'b0;
        s_axi_arready <= 1'b1;
      end
    end
  end

endmodule
