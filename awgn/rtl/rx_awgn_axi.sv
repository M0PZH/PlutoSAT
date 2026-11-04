// Registers and clock crossing for rx_awgn (the self-test noise source
// between the AD9363 interface and everything that uses its receive data).
//
//   0x00 VERSION  0x0001_0000
//   0x04 ID       "AWGN" (0x4157474E)
//   0x08 CTRL     [0] add noise to rx0 (and apply GAIN)
//   0x0C GAIN     [15:0] rx0 signal gain, Q2.14 (16384 = 1)
//   0x10 NSCALE   [16:0] noise: sigma per component = 0.018042 * NSCALE ADC LSB
//   0x14 P_IN     (ro) mean I^2 + Q^2 of rx0 before the noise, 2^16 samples
//   0x18 P_OUT    (ro) the same after it
//   0x1C P_COUNT  (ro) measurements done
//   0x20 SIGMA_G  (ro) noise generator sigma * 2^16 (295.6)
// CTRL / GAIN / NSCALE reach the ADC clock domain a few clocks after the
// last write.

module rx_awgn_axi (
  input  logic        s_axi_aclk,
  input  logic        s_axi_aresetn,
  input  logic [7:0]  s_axi_awaddr,
  input  logic        s_axi_awvalid,
  output logic        s_axi_awready,
  input  logic [31:0] s_axi_wdata,
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

  input  logic        adc_clk,
  input  logic [3:0]  in_valid,
  input  logic [15:0] in_i0, in_q0, in_i1, in_q1,
  output logic [3:0]  out_valid,
  output logic [15:0] out_i0, out_q0, out_i1, out_q1
);

  wire clk = s_axi_aclk;
  assign s_axi_bresp = 2'b00;
  assign s_axi_rresp = 2'b00;

  logic rst;
  always_ff @(posedge clk) rst <= !s_axi_aresetn;

  // ---------------------------------------------------------------- registers
  // kept as plain registers: they are the clock-crossing endpoints the
  // constraints name (synthesis must not merge them into the DSPs)
  (* DONT_TOUCH = "true" *) logic        en;
  (* DONT_TOUCH = "true" *) logic [15:0] gain;
  (* DONT_TOUCH = "true" *) logic [16:0] nscale;
  logic        cfg_t;                          // toggles on every configuration write

  logic        aw_hold, w_hold;
  logic [7:0]  awaddr_q;
  logic [31:0] wdata_q;
  assign s_axi_awready = !aw_hold;
  assign s_axi_wready  = !w_hold;

  always_ff @(posedge clk) begin
    if (rst) begin
      aw_hold <= 1'b0; w_hold <= 1'b0; s_axi_bvalid <= 1'b0;
      en <= 1'b0; gain <= 16'd16384; nscale <= '0; cfg_t <= 1'b0;
    end else begin
      if (s_axi_awvalid && s_axi_awready) begin aw_hold <= 1'b1; awaddr_q <= s_axi_awaddr; end
      if (s_axi_wvalid && s_axi_wready)   begin w_hold  <= 1'b1; wdata_q  <= s_axi_wdata;  end
      if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
      if (aw_hold && w_hold && !s_axi_bvalid) begin
        aw_hold <= 1'b0; w_hold <= 1'b0; s_axi_bvalid <= 1'b1;
        case (awaddr_q[7:2])
          6'h02: begin en <= wdata_q[0]; cfg_t <= !cfg_t; end
          6'h03: begin gain <= wdata_q[15:0]; cfg_t <= !cfg_t; end
          6'h04: begin nscale <= wdata_q[16:0]; cfg_t <= !cfg_t; end
          default: ;
        endcase
      end
    end
  end

  // ---------------------------------------------------------------- to the ADC domain
  (* ASYNC_REG = "TRUE" *) logic [1:0] arst_s;
  (* ASYNC_REG = "TRUE" *) logic [1:0] cfg_s;
  logic        cfg_q, a_rst;
  (* DONT_TOUCH = "true" *) logic        a_en;
  (* DONT_TOUCH = "true" *) logic [15:0] a_gain;
  (* DONT_TOUCH = "true" *) logic [16:0] a_nscale;
  always_ff @(posedge adc_clk) begin
    arst_s <= {arst_s[0], rst};
    a_rst <= arst_s[1];
    cfg_s <= {cfg_s[0], cfg_t};
    cfg_q <= cfg_s[1];
    if (a_rst) begin
      a_en <= 1'b0; a_gain <= 16'd16384; a_nscale <= '0;
    end else if (cfg_q != cfg_s[1]) begin    // quasi-static: stable since the toggle
      a_en <= en; a_gain <= gain; a_nscale <= nscale;
    end
  end

  logic        p_done;
  logic [31:0] p_in, p_out;
  rx_awgn u_awgn (
    .clk(adc_clk), .rst(a_rst), .en(a_en), .gain(a_gain), .nscale(a_nscale),
    .in_valid, .in_i0, .in_q0, .in_i1, .in_q1,
    .out_valid, .out_i0, .out_q0, .out_i1, .out_q1,
    .p_done, .p_in, .p_out);

  // ---------------------------------------------------------------- meters back
  logic        p_t;
  logic [31:0] h_in, h_out;
  always_ff @(posedge adc_clk)
    if (a_rst) p_t <= 1'b0;
    else if (p_done) begin h_in <= p_in; h_out <= p_out; p_t <= !p_t; end

  (* ASYNC_REG = "TRUE" *) logic [1:0] p_s;
  logic        p_q;
  logic [31:0] r_in, r_out, r_cnt;
  always_ff @(posedge clk) begin
    p_s <= {p_s[0], p_t};
    p_q <= p_s[1];
    if (rst) r_cnt <= '0;
    else if (p_q != p_s[1]) begin            // h_* stable for 2^16 samples
      r_in <= h_in; r_out <= h_out; r_cnt <= r_cnt + 32'd1;
    end
  end

  // ---------------------------------------------------------------- read
  logic       ar_p;
  logic [5:0] ar_q;
  always_ff @(posedge clk) begin
    if (rst) begin
      s_axi_arready <= 1'b1; s_axi_rvalid <= 1'b0; ar_p <= 1'b0;
    end else begin
      ar_p <= 1'b0;
      if (s_axi_arvalid && s_axi_arready) begin
        s_axi_arready <= 1'b0; ar_p <= 1'b1; ar_q <= s_axi_araddr[7:2];
      end
      if (ar_p) begin
        s_axi_rvalid <= 1'b1;
        case (ar_q)
          6'h00: s_axi_rdata <= 32'h0001_0000;
          6'h01: s_axi_rdata <= 32'h4157_474E;
          6'h02: s_axi_rdata <= {31'd0, en};
          6'h03: s_axi_rdata <= {16'd0, gain};
          6'h04: s_axi_rdata <= {15'd0, nscale};
          6'h05: s_axi_rdata <= r_in;
          6'h06: s_axi_rdata <= r_out;
          6'h07: s_axi_rdata <= r_cnt;
          6'h08: s_axi_rdata <= 32'd19372513;   // round(295.601 * 2^16)
          default: s_axi_rdata <= '0;
        endcase
      end
      if (s_axi_rvalid && s_axi_rready) begin
        s_axi_rvalid <= 1'b0; s_axi_arready <= 1'b1;
      end
    end
  end

endmodule
