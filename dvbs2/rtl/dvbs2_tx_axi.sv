// DVB-S2 TX for the Zynq block design: AXI4-Lite registers, DMA packet input,
// clock-domain crossing to the DAC and a selector in front of the AD9361
// core's channel 0 DAC input.
//
// Register map (byte offsets, 32-bit):
//   0x00 VERSION      RO  0x0001_0000 (1.0)
//   0x04 ID           RO  "DVS2" (0x44565332)
//   0x08 SCRATCH      RW
//   0x10 CTRL         RW  [0] enable PLFRAME generation
//                         [1] soft reset (core, FIFOs, counters)
//                         [2] DAC select: 1 = DVB-S2 drives DAC channel 0
//                         [3] NCO bypass
//                         [4] carrier only: constant symbols (a test tone at the
//                             NCO offset; framing keeps running)
//   0x14 SPS          RW  [6:0] samples per symbol (2..64)
//   0x18 GOLD_XA      RW  [17:0]
//   0x1C GOLD_XB      RW  [17:0]
//   0x20 NCO_FINC     RW  phase increment per sample, 2^32 = sample rate
//   0x24 COEF_ADDR    RW  [9:0] RRC coefficient address (p*16 + j)
//   0x28 COEF_DATA    WO  [17:0] write coefficient at COEF_ADDR, then COEF_ADDR++
//   0x30 DATA_FRAMES  RO  data PLFRAMEs sent
//   0x34 DUMMY_FRAMES RO  dummy PLFRAMEs sent
//   0x38 LEN_ERRORS   RO  bad-length packets (adapter) + core length errors
//   0x3C HDR_ERRORS   RO  packets with a bad header word
//   0x40 FIFO_LEVEL   RO  FECFRAME buffer level in symbols
//   0x44 DAC_UNDERRUN RO  DAC samples with no DVB-S2 sample available
//   0x48 FIFO_FRAMES  RO  [2:0] complete FECFRAMEs waiting

module dvbs2_tx_axi #(
  parameter string LDPC_ROM_FILE = "ldpc_rom.mem",
  parameter string NCO_LUT_FILE  = "nco_lut.mem"
) (
  // AXI4-Lite + DMA stream + core clock
  input  logic        s_axi_aclk,
  input  logic        s_axi_aresetn,
  input  logic [7:0]  s_axi_awaddr,
  input  logic        s_axi_awvalid,
  output logic        s_axi_awready,
  input  logic [31:0] s_axi_wdata,
  input  logic [3:0]  s_axi_wstrb,
  input  logic        s_axi_wvalid,
  output logic        s_axi_wready,
  output logic [1:0]  s_axi_bresp,
  output logic        s_axi_bvalid,
  input  logic        s_axi_bready,
  input  logic [7:0]  s_axi_araddr,
  input  logic        s_axi_arvalid,
  output logic        s_axi_arready,
  output logic [31:0] s_axi_rdata,
  output logic [1:0]  s_axi_rresp,
  output logic        s_axi_rvalid,
  input  logic        s_axi_rready,

  input  logic [31:0] s_axis_tdata,
  input  logic        s_axis_tvalid,
  output logic        s_axis_tready,
  input  logic        s_axis_tlast,

  // DAC side (axi_ad9361 l_clk domain)
  input  logic        dac_clk,
  input  logic        dac_valid,
  input  logic [15:0] dac_in_i,     // normal path (FIR interpolator output)
  input  logic [15:0] dac_in_q,
  output logic [15:0] dac_out_i,    // to axi_ad9361 dac_data_i0/q0
  output logic [15:0] dac_out_q
);

  wire clk = s_axi_aclk;

  // ---------------------------------------------------------------- registers
  logic [31:0] scratch, nco_finc;
  logic [4:0]  ctrl;
  logic [6:0]  sps;
  logic [17:0] gold_xa, gold_xb;
  logic [9:0]  coef_addr;
  logic        coef_we;
  logic [17:0] coef_data;
  logic [31:0] cnt_data, cnt_dummy, cnt_len_core, cnt_len_adp, cnt_hdr, cnt_unf;
  logic [16:0] fifo_level;

  // soft reset: hold for 16 cycles so the DAC side of the CDC FIFO sees it
  logic [3:0] srst_cnt;
  logic       core_rst;
  always_ff @(posedge clk) begin
    if (!s_axi_aresetn) begin
      srst_cnt <= '1;
    end else if (ctrl[1]) begin
      srst_cnt <= '1;
    end else if (srst_cnt != 0) begin
      srst_cnt <= srst_cnt - 1'b1;
    end
    core_rst <= !s_axi_aresetn || ctrl[1] || (srst_cnt != 0);
  end

  // AXI-Lite write: accept address and data together
  logic        aw_hold, w_hold;
  logic [7:0]  awaddr_q;
  logic [31:0] wdata_q;
  assign s_axi_awready = !aw_hold;
  assign s_axi_wready  = !w_hold;
  assign s_axi_bresp   = 2'b00;
  assign s_axi_rresp   = 2'b00;

  always_ff @(posedge clk) begin
    coef_we <= 1'b0;
    if (!s_axi_aresetn) begin
      aw_hold <= 1'b0; w_hold <= 1'b0; s_axi_bvalid <= 1'b0;
      ctrl <= 5'b00000; sps <= 7'd8; scratch <= '0; nco_finc <= '0;
      gold_xa <= 18'h00001; gold_xb <= 18'h01008; coef_addr <= '0;
    end else begin
      if (s_axi_awvalid && s_axi_awready) begin aw_hold <= 1'b1; awaddr_q <= s_axi_awaddr; end
      if (s_axi_wvalid && s_axi_wready)   begin w_hold  <= 1'b1; wdata_q  <= s_axi_wdata;  end
      if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
      if (aw_hold && w_hold && !s_axi_bvalid) begin
        aw_hold <= 1'b0; w_hold <= 1'b0; s_axi_bvalid <= 1'b1;
        case (awaddr_q[7:2])
          6'h02: scratch   <= wdata_q;
          6'h04: ctrl      <= wdata_q[4:0];
          6'h05: sps       <= wdata_q[6:0];
          6'h06: gold_xa   <= wdata_q[17:0];
          6'h07: gold_xb   <= wdata_q[17:0];
          6'h08: nco_finc  <= wdata_q;
          6'h09: coef_addr <= wdata_q[9:0];
          6'h0A: begin coef_data <= wdata_q[17:0]; coef_we <= 1'b1; end
          default: ;
        endcase
      end
      if (coef_we) coef_addr <= coef_addr + 10'd1;
      if (ctrl[1] && srst_cnt == 4'hF) ctrl[1] <= 1'b0;   // self-clearing
    end
  end

  // AXI-Lite read
  always_ff @(posedge clk) begin
    if (!s_axi_aresetn) begin
      s_axi_arready <= 1'b1; s_axi_rvalid <= 1'b0;
    end else begin
      if (s_axi_arvalid && s_axi_arready) begin
        s_axi_arready <= 1'b0;
        s_axi_rvalid  <= 1'b1;
        case (s_axi_araddr[7:2])
          6'h00: s_axi_rdata <= 32'h0001_0000;
          6'h01: s_axi_rdata <= 32'h4456_5332;
          6'h02: s_axi_rdata <= scratch;
          6'h04: s_axi_rdata <= {27'd0, ctrl};
          6'h05: s_axi_rdata <= {25'd0, sps};
          6'h06: s_axi_rdata <= {14'd0, gold_xa};
          6'h07: s_axi_rdata <= {14'd0, gold_xb};
          6'h08: s_axi_rdata <= nco_finc;
          6'h09: s_axi_rdata <= {22'd0, coef_addr};
          6'h0C: s_axi_rdata <= cnt_data;
          6'h0D: s_axi_rdata <= cnt_dummy;
          6'h0E: s_axi_rdata <= cnt_len_core + cnt_len_adp;
          6'h0F: s_axi_rdata <= cnt_hdr;
          6'h10: s_axi_rdata <= {15'd0, fifo_level};
          6'h11: s_axi_rdata <= cnt_unf;
          default: s_axi_rdata <= '0;
        endcase
      end
      if (s_axi_rvalid && s_axi_rready) begin
        s_axi_rvalid  <= 1'b0;
        s_axi_arready <= 1'b1;
      end
    end
  end

  // ---------------------------------------------------------------- packet adapter + core
  logic       b_valid, b_ready, b_last, err_hdr, err_len;
  logic [7:0] b_data;
  logic [4:0] b_user;

  dvbs2_bb_adapter i_adapter (
    .clk, .rst(core_rst),
    .s_valid(s_axis_tvalid), .s_ready(s_axis_tready), .s_data(s_axis_tdata), .s_last(s_axis_tlast),
    .m_valid(b_valid), .m_ready(b_ready), .m_data(b_data), .m_user(b_user), .m_last(b_last),
    .err_hdr, .err_len);

  always_ff @(posedge clk) begin
    if (core_rst) begin
      cnt_hdr <= '0; cnt_len_adp <= '0;
    end else begin
      if (err_hdr) cnt_hdr <= cnt_hdr + 1;
      if (err_len) cnt_len_adp <= cnt_len_adp + 1;
    end
  end

  logic        t_valid, t_ready;
  logic [15:0] t_i, t_q;

  dvbs2_tx #(.LDPC_ROM_FILE(LDPC_ROM_FILE), .NCO_LUT_FILE(NCO_LUT_FILE)) i_tx (
    .clk, .rst(core_rst),
    .enable(ctrl[0]), .sps, .gold_xa, .gold_xb, .nco_finc, .nco_bypass(ctrl[3]), .carrier_only(ctrl[4]),
    .coef_we, .coef_addr, .coef_data,
    .s_valid(b_valid), .s_ready(b_ready), .s_data(b_data), .s_user(b_user), .s_last(b_last),
    .m_valid(t_valid), .m_ready(t_ready), .m_i(t_i), .m_q(t_q),
    .cnt_data_frames(cnt_data), .cnt_dummy_frames(cnt_dummy), .cnt_len_errors(cnt_len_core),
    .fifo_level,
    .sym_valid(), .sym_si(), .sym_sq(), .sym_sof(), .sym_dummy());

  // ---------------------------------------------------------------- CDC to the DAC clock
  (* ASYNC_REG = "TRUE" *) logic [2:0] drst_sync;
  (* ASYNC_REG = "TRUE" *) logic [1:0] dsel_sync;
  wire        dac_rst = drst_sync[2];
  wire        dac_sel = dsel_sync[1];
  always_ff @(posedge dac_clk) begin
    drst_sync <= {drst_sync[1:0], core_rst};
    dsel_sync <= {dsel_sync[0], ctrl[2]};
  end

  logic        d_valid, d_take;
  logic [31:0] d_data;

  dvbs2_async_fifo #(.W(32), .AW(5)) i_cdc (
    .wclk(clk), .wrst(core_rst),
    .w_valid(t_valid), .w_ready(t_ready), .w_data({t_q, t_i}),
    .rclk(dac_clk), .rrst(dac_rst),
    .r_valid(d_valid), .r_ready(d_take), .r_data(d_data));

  assign d_take = dac_sel && dac_valid;

  logic unf_toggle;
  always_ff @(posedge dac_clk) begin
    if (dac_rst) unf_toggle <= 1'b0;
    else if (d_take && !d_valid) unf_toggle <= !unf_toggle;
    if (dac_valid) begin
      if (!dac_sel) begin
        dac_out_i <= dac_in_i;
        dac_out_q <= dac_in_q;
      end else if (d_valid) begin
        dac_out_i <= d_data[15:0];
        dac_out_q <= d_data[31:16];
      end else begin
        dac_out_i <= '0;
        dac_out_q <= '0;
      end
    end
  end

  // underrun counter (toggle synchronised into the AXI domain)
  (* ASYNC_REG = "TRUE" *) logic [1:0] unf_sync;
  logic       unf_prev;
  always_ff @(posedge clk) begin
    unf_sync <= {unf_sync[0], unf_toggle};
    unf_prev <= unf_sync[1];
    if (core_rst) cnt_unf <= '0;
    else if (unf_prev ^ unf_sync[1]) cnt_unf <= cnt_unf + 1;
  end

endmodule
