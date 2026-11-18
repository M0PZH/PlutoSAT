// Narrowband IQ transmitter: complex baseband samples from a DDR ring
// (AXI3 read master, 64-bit, S_AXI_HP0) -> envelope (gain, start/stop ramp)
// -> FIR x5 -> CIC x RATE -> NCO -> DAC channel 0, and the TX source mux in
// front of the AD9361 DAC (DVB-S2 TX, IQ TX or off). Model: model/iqtx.py.
//
// Registers (32-bit, byte addresses; unlisted addresses read 0, writes are
// ignored):
//   0x000 VERSION     0x0001_0000
//   0x004 ID          "IQTX" (0x49515458)
//   0x008 CTRL        [0] enable: 0 -> 1 starts (once IQ TX is the active
//                     source), 1 -> 0 ramps down, flushes and stops
//   0x00C TX_SRC      [1:0] requested DAC source: 0 DVB-S2 (reset), 1 IQ TX,
//                     2 off. Becomes active only while the active and the
//                     requested source are both silent (STATUS)
//   0x010 STATUS      (ro) [1:0] active source, [2] switch pending, [3] busy,
//                     [4] waiting for the IQ source, [5] stopping,
//                     [6] DVB-S2 output silent, [7] started (first sample)
//   0x014 RATE        [8:0] CIC factor R, 8..400: input rate 12 MS/s / (5 R)
//                     (latched at the start)
//   0x018 GAIN        [15:0] Q2.14, 0x4000 = 1.0 (full scale in -> out)
//   0x01C FINC        NCO step: +FINC / 2^32 * 12 MHz (signed)
//   0x020 RAMP        [3:0] RLOG: ramp length 2^RLOG input samples (latched)
//   0x024 RING_BASE   physical address, aligned to RING_MASK + 1
//   0x028 RING_MASK   ring size - 1 (size a power of two >= 4096)
//   0x02C WPTR        bytes written by software, wraps at 2^32 (bits [5:0] ignored)
//   0x030 RPTR        (ro) bytes fetched by the core, wraps at 2^32
//   0x034 LOWATER     low-water mark in bytes (IRQ_STATUS.LOW)
//   0x038 IRQ_EN      [2:0] interrupt enables (as IRQ_STATUS)
//   0x03C IRQ_STATUS  [0] LOW: fill fell below LOWATER, [1] IDLE: stopped,
//                     [2] UNDERRUN; write 1 to clear
//   0x040 SAMPLES     (ro) input samples played since the start
//   0x044 UNDERRUNS   (ro) input samples replaced by zero since the start
//   0x048 DAC_UNDERRUNS (ro) DAC samples with no IQ sample at hand
//   0x04C INFO        (ro) [7:0] FIR factor (5), [15:8] RATE min (8), [31:16] RATE max (400)
//
// Ring contents: 32-bit words {Q[15:0], I[15:0]}, little-endian.

module iqtx_axi #(
  parameter string FIR_FILE  = "iqtx_fir.mem",
  parameter string NCO_FILE  = "iqtx_nco.mem",
  parameter string NORM_FILE = "iqtx_norm.mem",
  parameter string RAMP_FILE = "iqtx_ramp.mem"
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

  output logic [31:0] m_axi_araddr,
  output logic [3:0]  m_axi_arlen,
  output logic [2:0]  m_axi_arsize,
  output logic [1:0]  m_axi_arburst,
  output logic [3:0]  m_axi_arcache,
  output logic        m_axi_arvalid,
  input  logic        m_axi_arready,
  input  logic [63:0] m_axi_rdata,
  input  logic [1:0]  m_axi_rresp,
  input  logic        m_axi_rlast,
  input  logic        m_axi_rvalid,
  output logic        m_axi_rready,

  output logic        irq,

  input  logic        dac_clk,
  input  logic        dac_valid,
  input  logic [15:0] dvb_i,            // DVB-S2 TX output (dvbs2_tx dac_out)
  input  logic [15:0] dvb_q,
  output logic [15:0] dac_out_i,        // to axi_ad9361 dac_data_i0 / q0
  output logic [15:0] dac_out_q
);

  wire clk = s_axi_aclk;
  assign s_axi_bresp = 2'b00;
  assign s_axi_rresp = 2'b00;

  localparam int FLUSH = 32;
  localparam logic [1:0] SRC_DVB = 2'd0, SRC_IQ = 2'd1, SRC_OFF = 2'd2;

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
  logic        en;
  logic [1:0]  tx_src;
  logic [8:0]  rate;
  logic [15:0] gain;
  logic [31:0] finc, base, mask, wptr, lowater;
  logic [3:0]  rlog;
  logic [2:0]  irq_en, irq_st, irq_set;
  logic [31:0] samples, underruns, dac_unf;

  logic        aw_hold, w_hold;
  logic [15:0] awaddr_q;
  logic [31:0] wdata_q;
  assign s_axi_awready = !aw_hold;
  assign s_axi_wready  = !w_hold;

  always_ff @(posedge clk) begin
    if (rst) begin
      aw_hold <= 1'b0; w_hold <= 1'b0; s_axi_bvalid <= 1'b0;
      en <= 1'b0; tx_src <= SRC_DVB; rate <= 9'd50; gain <= 16'h4000; finc <= '0;
      rlog <= 4'd8; base <= '0; mask <= 32'hFFF; wptr <= '0; lowater <= '0;
      irq_en <= '0; irq_st <= '0;
    end else begin
      irq_st <= irq_st | irq_set;
      if (s_axi_awvalid && s_axi_awready) begin aw_hold <= 1'b1; awaddr_q <= s_axi_awaddr; end
      if (s_axi_wvalid && s_axi_wready)   begin w_hold  <= 1'b1; wdata_q  <= s_axi_wdata;  end
      if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
      if (aw_hold && w_hold && !s_axi_bvalid) begin
        aw_hold <= 1'b0; w_hold <= 1'b0; s_axi_bvalid <= 1'b1;
        if (awaddr_q[15:8] == 8'd0)
          case (awaddr_q[7:2])
            6'h02: en <= wdata_q[0];
            6'h03: tx_src <= (wdata_q[1:0] == 2'd3) ? SRC_OFF : wdata_q[1:0];
            6'h05: rate <= wdata_q[8:0];
            6'h06: gain <= wdata_q[15:0];
            6'h07: finc <= wdata_q;
            6'h08: rlog <= wdata_q[3:0];
            6'h09: base <= wdata_q;
            6'h0A: mask <= wdata_q;
            6'h0B: wptr <= wdata_q;
            6'h0D: lowater <= wdata_q;
            6'h0E: irq_en <= wdata_q[2:0];
            6'h0F: irq_st <= (irq_st | irq_set) & ~wdata_q[2:0];
            default: ;
          endcase
      end
    end
  end

  always_ff @(posedge clk) irq <= |(irq_st & irq_en);

  // ---------------------------------------------------------------- tables
  (* rom_style = "distributed" *) logic [21:0] norm_rom [512];
  (* rom_style = "distributed" *) logic [15:0] ramp_rom [256];
  initial $readmemh(NORM_FILE, norm_rom);
  initial $readmemh(RAMP_FILE, ramp_rom);

  logic [21:0] norm_q;
  always_ff @(posedge clk) norm_q <= norm_rom[rate];

  // ---------------------------------------------------------------- control
  typedef enum logic [1:0] {S_IDLE, S_RUN, S_DOWN, S_FLUSH} st_t;
  st_t         st;
  logic        started, act_iq, dvb_silent_c;
  logic [1:0]  act_c;
  logic [8:0]  c_rate;
  logic [4:0]  c_shift;
  logic [16:0] c_norm;
  logic [3:0]  c_rlog;
  logic [16:0] kg;
  logic [16:0] lvl;
  logic [5:0]  fcnt;
  logic        drop;

  wire         clr = st == S_IDLE;
  wire  [16:0] mm  = 17'd1 << c_rlog;

  // ring reader
  logic        rd_valid, rd_pop, rd_busy;
  logic [31:0] rd_data, rptr;

  iqtx_rd u_rd (
    .clk, .rst, .run(st == S_RUN || st == S_DOWN), .drop,
    .base, .mask, .wptr, .rptr, .busy(rd_busy),
    .s_valid(rd_valid), .s_ready(rd_pop), .s_data(rd_data),
    .m_axi_araddr, .m_axi_arlen, .m_axi_arsize, .m_axi_arburst, .m_axi_arcache,
    .m_axi_arvalid, .m_axi_arready, .m_axi_rdata, .m_axi_rlast, .m_axi_rvalid,
    .m_axi_rready);

  // envelope pipeline: e0 decision -> e1 ramp value -> e2 e -> e3 products
  // -> x' (ev); one sample at a time
  logic               ev_valid, ev_ready;
  logic signed [17:0] ev_i, ev_q;
  logic [3:0]         ep;                 // pipeline stage flags e1..e4
  logic [31:0]        x0;
  logic               rfull0, rzero0;
  logic [7:0]         ridx0;
  logic [16:0]        r1;
  logic [16:0]        e2;
  logic signed [33:0] pi3, pq3;

  wire can = !ev_valid && ep == 4'd0;

  function automatic logic [7:0] ridx(input logic [16:0] l, input logic [3:0] lg);
    return (lg >= 4'd8) ? 8'(l >> (lg - 4'd8)) : 8'(l << (4'd8 - lg));
  endfunction

  function automatic logic signed [17:0] sat18r(input logic signed [33:0] a);
    logic signed [33:0] r;
    r = (a + 34'sd8192) >>> 14;
    if (r > 34'sd131071)  return 18'sd131071;
    if (r < -34'sd131072) return -18'sd131072;
    return r[17:0];
  endfunction

  logic [31:0] fill;
  logic        low_q, idle_c, dsil_d;
  assign fill = {wptr[31:6], 6'd0} - rptr;

  always_ff @(posedge clk) begin
    logic prod, low;                       // e0 makes a sample this clock
    kg <= 17'((33'(gain) * 33'(c_norm)) >> 16);
    idle_c <= st == S_IDLE;
    drop <= 1'b0;
    prod = 1'b0;
    rd_pop <= 1'b0;
    irq_set <= '0;
    if (rst) begin
      st <= S_IDLE; started <= 1'b0; ep <= '0; ev_valid <= 1'b0;
      samples <= '0; underruns <= '0; low_q <= 1'b0;
    end else begin
      // e0
      case (st)
        S_IDLE: if (en && act_iq) begin
          st <= S_RUN; started <= 1'b0; lvl <= '0;
          samples <= '0; underruns <= '0;
          c_rate <= rate; c_norm <= norm_q[16:0]; c_shift <= norm_q[21:17]; c_rlog <= rlog;
        end
        S_RUN: begin
          if (!en) begin
            if (started) st <= S_DOWN;
            else begin st <= S_FLUSH; fcnt <= 6'(FLUSH); end
          end else if (can && !rd_pop && (started || rd_valid)) begin
            prod = 1'b1;
            started <= 1'b1;
            x0 <= rd_valid ? rd_data : 32'd0;
            rd_pop <= rd_valid;
            if (!rd_valid) begin
              underruns <= underruns + 32'd1;
              irq_set[2] <= 1'b1;
            end
            rfull0 <= lvl == mm;
            rzero0 <= 1'b0;
            ridx0 <= ridx(lvl, c_rlog);
            if (lvl != mm) lvl <= lvl + 17'd1;
          end
        end
        S_DOWN: if (can && !rd_pop) begin
          if (lvl == 17'd0) begin
            st <= S_FLUSH; fcnt <= '0;
          end else begin
            prod = 1'b1;
            x0 <= rd_valid ? rd_data : 32'd0;
            rd_pop <= rd_valid;
            rfull0 <= 1'b0;
            rzero0 <= 1'b0;
            ridx0 <= ridx(lvl - 17'd1, c_rlog);
            lvl <= lvl - 17'd1;
          end
        end
        S_FLUSH: begin
          if (fcnt != 6'(FLUSH)) begin
            if (can) begin
              prod = 1'b1;
              x0 <= '0; rfull0 <= 1'b0; rzero0 <= 1'b1;
              fcnt <= fcnt + 6'd1;
            end
          end else if (can && !rd_busy) begin
            st <= S_IDLE;
            drop <= 1'b1;
            irq_set[1] <= 1'b1;
          end
        end
        default: st <= S_IDLE;
      endcase
      if (prod && st != S_FLUSH) samples <= samples + 32'd1;

      // e1..e4
      ep <= {ep[2:0], prod};
      if (ep[0]) r1 <= rfull0 ? 17'd32768 : rzero0 ? 17'd0 : 17'(ramp_rom[ridx0]);
      if (ep[1]) e2 <= 17'((34'(kg) * 34'(r1)) >> 15);
      if (ep[2]) begin
        pi3 <= $signed(x0[15:0]) * $signed({1'b0, e2});
        pq3 <= $signed(x0[31:16]) * $signed({1'b0, e2});
      end
      if (ep[3]) begin
        ev_i <= sat18r(pi3);
        ev_q <= sat18r(pq3);
        ev_valid <= 1'b1;
      end
      if (ev_valid && ev_ready) ev_valid <= 1'b0;

      // low-water mark: rising edge while playing
      low = st == S_RUN && started && fill < lowater;
      low_q <= low;
      if (low && !low_q) irq_set[0] <= 1'b1;
    end
  end

  // ---------------------------------------------------------------- filters, NCO
  logic               v_valid, v_ready;
  logic signed [19:0] v_i, v_q;

  iqtx_fir #(.FIR_FILE(FIR_FILE)) u_fir (
    .clk, .clr,
    .s_valid(ev_valid), .s_ready(ev_ready), .s_i(ev_i), .s_q(ev_q),
    .m_valid(v_valid), .m_ready(v_ready), .m_i(v_i), .m_q(v_q));

  logic        o_valid, o_ready;
  logic [31:0] o_data;

  iqtx_cic #(.NCO_FILE(NCO_FILE)) u_cic (
    .clk, .clr, .rate(c_rate), .shift(c_shift), .finc,
    .s_valid(v_valid), .s_ready(v_ready), .s_i(v_i), .s_q(v_q),
    .m_valid(o_valid), .m_ready(o_ready), .m_data(o_data));

  // ---------------------------------------------------------------- DAC side
  (* ASYNC_REG = "TRUE" *) logic [1:0] rst_s;
  wire drst = rst_s[1];
  always_ff @(posedge dac_clk) rst_s <= {rst_s[0], rst};

  logic        d_valid, d_pop;
  logic [31:0] d_data;
  dvbs2_async_fifo #(.W(32), .AW(4)) u_cdc (
    .wclk(clk), .wrst(rst), .w_valid(o_valid), .w_ready(o_ready), .w_data(o_data),
    .rclk(dac_clk), .rrst(drst), .r_valid(d_valid), .r_ready(d_pop), .r_data(d_data));

  (* ASYNC_REG = "TRUE" *) logic [1:0] req_s, req_s2, idle_s;
  logic [1:0] req_q, act;
  logic [4:0] zc;
  logic       dvb_silent, armed, unf_toggle;

  wire iq_idle = idle_s[1];
  wire req_ok  = req_s2 == req_q;

  function automatic logic silent(input logic [1:0] s, input logic dvb_sil, input logic iq_sil);
    return s == SRC_DVB ? dvb_sil : s == SRC_IQ ? iq_sil : 1'b1;
  endfunction

  assign d_pop = dac_valid && act == SRC_IQ;

  always_ff @(posedge dac_clk) begin
    req_s <= tx_src; req_s2 <= req_s; req_q <= req_s2;
    idle_s <= {idle_s[0], idle_c};
    dsil_d <= zc == 5'd16;
    if (drst) begin
      act <= SRC_DVB; zc <= '0; armed <= 1'b0; unf_toggle <= 1'b0;
      dac_out_i <= '0; dac_out_q <= '0;
    end else begin
      if (dac_valid) begin
        if (dvb_i == 16'd0 && dvb_q == 16'd0) begin
          if (zc != 5'd16) zc <= zc + 5'd1;
        end else
          zc <= '0;
        case (act)
          SRC_DVB: begin dac_out_i <= dvb_i; dac_out_q <= dvb_q; end
          SRC_IQ: begin
            dac_out_i <= d_valid ? d_data[15:0] : 16'd0;
            dac_out_q <= d_valid ? d_data[31:16] : 16'd0;
          end
          default: begin dac_out_i <= '0; dac_out_q <= '0; end
        endcase
      end
      if (req_ok && req_q != act && silent(act, dvb_silent, iq_idle) && silent(req_q, dvb_silent, iq_idle))
        act <= req_q;
      if (iq_idle) armed <= 1'b0;
      else if (d_pop && d_valid) armed <= 1'b1;
      if (d_pop && !d_valid && armed && !iq_idle) unf_toggle <= !unf_toggle;
    end
  end
  assign dvb_silent = dsil_d;

  // DAC side -> clk: active source, DVB-S2 silence, underrun toggle
  (* ASYNC_REG = "TRUE" *) logic [1:0] act_s, act_s2, dsil_s, unf_s;
  logic [1:0] act_q;
  logic       unf_prev;
  always_ff @(posedge clk) begin
    act_s <= act; act_s2 <= act_s; act_q <= act_s2;
    if (act_s2 == act_q) act_c <= act_q;
    dsil_s <= {dsil_s[0], dvb_silent};
    unf_s <= {unf_s[0], unf_toggle};
    unf_prev <= unf_s[1];
    if (rst) begin
      dac_unf <= '0; act_c <= SRC_DVB;
    end else if (unf_prev ^ unf_s[1]) dac_unf <= dac_unf + 32'd1;
  end
  assign act_iq = act_c == SRC_IQ;
  assign dvb_silent_c = dsil_s[1];

  // ---------------------------------------------------------------- AXI-Lite read
  logic        ar_p;
  logic [15:0] ar_q;
  always_ff @(posedge clk) begin
    if (rst) begin
      s_axi_arready <= 1'b1; s_axi_rvalid <= 1'b0; ar_p <= 1'b0;
    end else begin
      ar_p <= 1'b0;
      if (s_axi_arvalid && s_axi_arready) begin
        s_axi_arready <= 1'b0;
        ar_p <= 1'b1;
        ar_q <= s_axi_araddr;
      end
      if (ar_p) begin
        s_axi_rvalid <= 1'b1;
        s_axi_rdata <= '0;
        if (ar_q[15:8] == 8'd0)
          case (ar_q[7:2])
            6'h00: s_axi_rdata <= 32'h0001_0000;
            6'h01: s_axi_rdata <= 32'h4951_5458;
            6'h02: s_axi_rdata <= {31'd0, en};
            6'h03: s_axi_rdata <= {30'd0, tx_src};
            6'h04: s_axi_rdata <= {24'd0, started && st != S_IDLE, dvb_silent_c,
                                   st == S_DOWN || st == S_FLUSH, en && st == S_IDLE,
                                   st != S_IDLE, tx_src != act_c, act_c};
            6'h05: s_axi_rdata <= {23'd0, rate};
            6'h06: s_axi_rdata <= {16'd0, gain};
            6'h07: s_axi_rdata <= finc;
            6'h08: s_axi_rdata <= {28'd0, rlog};
            6'h09: s_axi_rdata <= base;
            6'h0A: s_axi_rdata <= mask;
            6'h0B: s_axi_rdata <= wptr;
            6'h0C: s_axi_rdata <= rptr;
            6'h0D: s_axi_rdata <= lowater;
            6'h0E: s_axi_rdata <= {29'd0, irq_en};
            6'h0F: s_axi_rdata <= {29'd0, irq_st};
            6'h10: s_axi_rdata <= samples;
            6'h11: s_axi_rdata <= underruns;
            6'h12: s_axi_rdata <= dac_unf;
            6'h13: s_axi_rdata <= {16'd400, 8'd8, 8'd5};
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
