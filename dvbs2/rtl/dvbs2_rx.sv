// DVB-S2 receiver core, QPSK normal frames: 2 samples/symbol in, BBFRAMEs out.
//
//   in_clk samples -> async FIFO (512) -> dvbs2_rx_timing -> dvbs2_rx_framesync
//   -> dvbs2_rx_carrier (64800 LLRs per data frame) -> dvbs2_ldpc_dec
//   -> dvbs2_bch_dec -> BBFRAME bytes (Kbch/8 per frame, BB-descrambled),
//   m_ok: BCH codeword (after <= t corrections), m_ldpc_ok: LDPC converged
//
// Models: model/rxfixed.py (receive()), model/ldpcdec.py. The sample input
// has no backpressure; samples arriving with the FIFO full are dropped and
// counted. dvbs2_rx_pls decodes the PLS code of every PLHEADER: status, and
// the frame sync's header confirmation (frame type, loss of lock).
//
// LLR queue (LLRQ = 1, dvbs2_rx_llrq, docs/rx2_design.md 6.1): with llrq_en
// the carrier's LLR frames go through a ring in DDR (m_axi) to the LDPC
// decoder, as slot 0 (the path of the second receive slot, testable with
// one); without, straight to the decoder as before. Either way the decoded
// frames are the same.

module dvbs2_rx #(
  parameter string LUT_FILE = "nco_lut.mem",
  parameter string LDPC_ROM_FILE = "ldpc_dec_rom.mem",
  parameter bit    LLRQ = 1'b1               // include the LLR queue
) (
  input  logic               in_clk,
  input  logic               in_rst,
  input  logic               in_valid,
  input  logic signed [15:0] in_i,
  input  logic signed [15:0] in_q,
  output logic [15:0]        in_overflows,   // in_clk domain

  input  logic               clk,
  input  logic               rst,

  // configuration (static while running; change under rst)
  input  logic               coef_we,
  input  logic [9:0]         coef_addr,
  input  logic signed [17:0] coef_data,
  input  logic [15:0]        timing_kp,
  input  logic [15:0]        timing_ki,
  input  logic signed [31:0] timing_integ0,   // timing loop integrator at reset
  input  logic [4:0]         modcod,
  input  logic               pilots,
  input  logic [31:0]        fs_thresh,
  input  logic [17:0]        gold_xa,
  input  logic [17:0]        gold_xb,
  input  logic [17:0]        pll_kp,
  input  logic [17:0]        pll_ki,
  input  logic [5:0]         ldpc_max_it,
  input  logic [5:0]         pls_h,        // PLS decoder grid -H..H
  input  logic [9:0]         pls_step,     // ... step, turns * 2^16 per symbol
  input  logic [31:0]        pls_seen_clr, // w1c of st_pls_seen
  // LLR queue (static, change under rst); axi_rst: the AXI bus reset
  input  logic               axi_rst,
  input  logic               llrq_en,
  input  logic [31:0]        llrq_base,    // 2 MB aligned
  input  logic [4:0]         llrq_nfr,     // frames per slot ring, 1..16
  input  logic [5:0]         llrq_qhi,     // iteration cap while more than qhi frames wait
  input  logic [5:0]         llrq_it_lo,   // ... at it_lo iterations

  output logic               m_valid,
  input  logic               m_ready,
  output logic [7:0]         m_data,
  output logic               m_last,
  output logic               m_ok,
  output logic               m_ldpc_ok,
  output logic [5:0]         m_iters,
  output logic [4:0]         m_nerr,       // BCH errors located
  output logic [15:0]        m_raw_err,    // pre-FEC bit errors of the frame (if m_ok)
  // the frame's own parameters (taken with its first LLR word), constant in a frame
  output logic [3:0]         m_rate,       // code rate 0..10
  output logic [4:0]         m_modcod,
  output logic               m_pilots,
  output logic               m_slot,       // receive slot (0 for now)

  // constellation snapshot sources: carrier output (data symbols, phase
  // corrected, before the LLR gain), timing loop output (all symbols)
  output logic               snap_c_v,
  output logic signed [15:0] snap_c_i,
  output logic signed [15:0] snap_c_q,
  output logic               snap_t_v,
  output logic signed [15:0] snap_t_i,
  output logic signed [15:0] snap_t_q,

  output logic [16:0]        st_agc_gain,
  output logic signed [31:0] st_timing_integ,
  output logic [1:0]         st_fs_state,
  output logic [15:0]        st_fs_found,
  output logic [15:0]        st_fs_flywheel,
  output logic [15:0]        st_fs_lost,
  output logic [31:0]        st_freq,
  output logic [31:0]        st_amp,
  output logic [35:0]        st_energy,
  output logic [17:0]        st_gain,
  output logic [15:0]        st_frames,
  output logic [15:0]        st_acq,
  output logic [15:0]        st_fa,
  output logic [15:0]        st_retype,
  output logic [15:0]        st_ldpc_frames,
  output logic [15:0]        st_ldpc_failed,
  output logic [15:0]        st_bch_corrected,
  output logic [15:0]        st_bch_failed,

  // PLS decoder (dvbs2_rx_pls): the last decode, the last confident data decode, counters
  output logic               st_pls_v,       // pulse: a decode
  output logic [6:0]         st_pls_code,    // {MODCOD, short frame, pilots}
  output logic [3:0]         st_pls_flags,   // {valid, grid searched, carrier hypothesis, confident}
  output logic [17:0]        st_pls_best,
  output logic [17:0]        st_pls_second,
  output logic [15:0]        st_pls_freq,    // turns * 2^16 per symbol
  output logic [15:0]        st_pls_count,
  output logic [6:0]         st_pls_dcode,
  output logic [3:0]         st_pls_dflags,
  output logic [15:0]        st_pls_dcount,
  output logic [15:0]        st_pls_diff_modcod,
  output logic [15:0]        st_pls_diff_pilots,
  output logic [31:0]        st_pls_seen,
  output logic [15:0]        st_pls_skipped,

  // LLR queue status
  output logic [5:0]         st_llrq_depth,
  output logic [5:0]         st_llrq_depth_max,
  output logic [15:0]        st_llrq_frames,
  output logic [15:0]        st_llrq_dropped0,
  output logic [15:0]        st_llrq_dropped1,
  output logic [15:0]        st_llrq_axi_err,
  output logic               st_llrq_busy,

  // LLR queue: AXI3 master, 64 bits (Zynq S_AXI_HP1)
  output logic [31:0]        m_axi_awaddr,
  output logic [3:0]         m_axi_awlen,
  output logic [2:0]         m_axi_awsize,
  output logic [1:0]         m_axi_awburst,
  output logic [3:0]         m_axi_awcache,
  output logic               m_axi_awvalid,
  input  logic               m_axi_awready,
  output logic [63:0]        m_axi_wdata,
  output logic [7:0]         m_axi_wstrb,
  output logic               m_axi_wlast,
  output logic               m_axi_wvalid,
  input  logic               m_axi_wready,
  input  logic [1:0]         m_axi_bresp,
  input  logic               m_axi_bvalid,
  output logic               m_axi_bready,
  output logic [31:0]        m_axi_araddr,
  output logic [3:0]         m_axi_arlen,
  output logic [2:0]         m_axi_arsize,
  output logic [1:0]         m_axi_arburst,
  output logic [3:0]         m_axi_arcache,
  output logic               m_axi_arvalid,
  input  logic               m_axi_arready,
  input  logic [63:0]        m_axi_rdata,
  input  logic [1:0]         m_axi_rresp,
  input  logic               m_axi_rlast,
  input  logic               m_axi_rvalid,
  output logic               m_axi_rready
);

  logic w_ready;
  always_ff @(posedge in_clk)
    if (in_rst) in_overflows <= '0;
    else if (in_valid && !w_ready && in_overflows != 16'hFFFF) in_overflows <= in_overflows + 16'd1;

  logic               f_valid, f_ready;
  logic [31:0]        f_data;

  dvbs2_async_fifo #(.W(32), .AW(9)) u_in_fifo (
    .wclk(in_clk), .wrst(in_rst), .w_valid(in_valid), .w_ready(w_ready), .w_data({in_q, in_i}),
    .rclk(clk), .rrst(rst), .r_valid(f_valid), .r_ready(f_ready), .r_data(f_data));

  logic               t_valid, t_ready;
  logic signed [15:0] t_i, t_q;

  dvbs2_rx_timing u_timing (
    .clk, .rst, .kp(timing_kp), .ki(timing_ki), .integ_init(timing_integ0),
    .coef_we, .coef_addr, .coef_data,
    .s_valid(f_valid), .s_ready(f_ready), .s_i($signed(f_data[15:0])), .s_q($signed(f_data[31:16])),
    .m_valid(t_valid), .m_ready(t_ready), .m_i(t_i), .m_q(t_q),
    .agc_gain(st_agc_gain), .loop_integ(st_timing_integ));

  logic               y_valid, y_ready, y_sof, y_dummy, y_found;
  logic               ty_v, ty_dummy;               // carrier -> frame sync: frame type
  logic               pls_busy;                     // PLS decoder -> frame sync: header confirmation
  logic signed [15:0] y_i, y_q;

  dvbs2_rx_framesync u_fs (
    .clk, .rst, .modcod, .pilots, .thresh(fs_thresh),
    .s_valid(t_valid), .s_ready(t_ready), .s_i(t_i), .s_q(t_q),
    .m_valid(y_valid), .m_ready(y_ready), .m_i(y_i), .m_q(y_q), .m_sof(y_sof), .m_dummy(y_dummy), .m_found(y_found),
    .ty_v, .ty_dummy,
    .pls_v(st_pls_v), .pls_conf(st_pls_flags[0]), .pls_code(st_pls_code), .pls_busy,
    .fs_state(st_fs_state), .cnt_found(st_fs_found), .cnt_flywheel(st_fs_flywheel), .cnt_lost(st_fs_lost));

  logic       l_valid, l_ready, l_first, l_last;
  logic       c_acquired;
  logic [7:0] l_i, l_q;

  dvbs2_rx_carrier #(.LUT_FILE(LUT_FILE)) u_carrier (
    .clk, .rst, .modcod, .pilots, .gold_xa, .gold_xb, .pll_kp, .pll_ki,
    .s_valid(y_valid), .s_ready(y_ready), .s_i(y_i), .s_q(y_q), .s_sof(y_sof), .s_dummy(y_dummy), .s_found(y_found),
    .ty_v, .ty_dummy,
    .m_valid(l_valid), .m_ready(l_ready), .m_llr_i(l_i), .m_llr_q(l_q), .m_first(l_first), .m_last(l_last),
    .st_freq, .st_amp, .st_energy, .st_gain, .st_frames, .st_acq, .st_fa, .st_retype, .st_acquired(c_acquired),
    .snap_v(snap_c_v), .snap_i(snap_c_i), .snap_q(snap_c_q));

  // PLS decoder on the carrier's input
  dvbs2_rx_pls u_pls (
    .clk, .rst, .modcod, .pilots, .cfg_h(pls_h), .cfg_step(pls_step),
    .s_v(y_valid && y_ready), .s_i(y_i), .s_q(y_q), .s_sof(y_sof), .fhat(st_freq), .acquired(c_acquired),
    .seen_clr(pls_seen_clr),
    .res_v(st_pls_v), .res_code(st_pls_code), .res_conf(st_pls_flags[0]), .res_car(st_pls_flags[1]),
    .res_grid(st_pls_flags[2]), .res_best(st_pls_best), .res_second(st_pls_second), .res_freq(st_pls_freq),
    .res_valid(st_pls_flags[3]), .st_count(st_pls_count),
    .data_code(st_pls_dcode), .data_conf(st_pls_dflags[0]), .data_car(st_pls_dflags[1]),
    .data_grid(st_pls_dflags[2]), .data_valid(st_pls_dflags[3]), .st_data(st_pls_dcount),
    .st_diff_modcod(st_pls_diff_modcod), .st_diff_pilots(st_pls_diff_pilots),
    .st_seen(st_pls_seen), .st_skipped(st_pls_skipped), .dec_busy(pls_busy));

  assign snap_t_v = t_valid && t_ready;
  assign snap_t_i = t_i;
  assign snap_t_q = t_q;

  // ---------------------------------------------------------------- LLR queue
  // d_*: the decoder's input, from the carrier or from the queue (slot 0)
  logic       d_valid, d_ready, d_first;
  logic [7:0] d_i, d_q;
  logic [3:0] d_rate;
  logic [5:0] d_max_it;
  logic [7:0] d_tag;
  logic       cap_en;
  logic [5:0] cap;

  if (LLRQ) begin : g_llrq
    logic       q_valid, q_ready, q_first, q_slot, q_pilots;
    logic [7:0] q_i, q_q;
    logic [3:0] q_rate;
    logic [5:0] q_max_it;
    logic [4:0] q_modcod;

    dvbs2_rx_llrq u_llrq (
      .clk, .arst(axi_rst), .rst, .en(llrq_en), .base(llrq_base), .nfr(llrq_nfr),
      .max_it0(ldpc_max_it), .max_it1(ldpc_max_it), .qhi(llrq_qhi), .it_lo(llrq_it_lo),
      .s_valid(llrq_en && l_valid), .s_ready(q_ready), .s_llr_i(l_i), .s_llr_q(l_q), .s_first(l_first),
      .s_slot(1'b0), .s_modcod(modcod), .s_pilots(pilots),
      .m_valid(q_valid), .m_ready(llrq_en && d_ready), .m_llr_i(q_i), .m_llr_q(q_q), .m_first(q_first),
      .m_rate(q_rate), .m_max_it(q_max_it), .m_slot(q_slot), .m_modcod(q_modcod), .m_pilots(q_pilots),
      .it_cap_en(cap_en), .it_cap(cap),
      .st_depth(st_llrq_depth), .st_depth_max(st_llrq_depth_max), .st_frames(st_llrq_frames),
      .st_dropped0(st_llrq_dropped0), .st_dropped1(st_llrq_dropped1), .st_axi_err(st_llrq_axi_err),
      .st_busy(st_llrq_busy),
      .m_axi_awaddr, .m_axi_awlen, .m_axi_awsize, .m_axi_awburst, .m_axi_awcache, .m_axi_awvalid,
      .m_axi_awready, .m_axi_wdata, .m_axi_wstrb, .m_axi_wlast, .m_axi_wvalid, .m_axi_wready,
      .m_axi_bresp, .m_axi_bvalid, .m_axi_bready, .m_axi_araddr, .m_axi_arlen, .m_axi_arsize,
      .m_axi_arburst, .m_axi_arcache, .m_axi_arvalid, .m_axi_arready, .m_axi_rdata, .m_axi_rresp,
      .m_axi_rlast, .m_axi_rvalid, .m_axi_rready);

    assign l_ready  = llrq_en ? q_ready : d_ready;
    assign d_valid  = llrq_en ? q_valid : l_valid;
    assign d_i      = llrq_en ? q_i : l_i;
    assign d_q      = llrq_en ? q_q : l_q;
    assign d_first  = llrq_en ? q_first : l_first;
    assign d_rate   = llrq_en ? q_rate : 4'(modcod - 5'd1);
    assign d_max_it = llrq_en ? q_max_it : ldpc_max_it;
    assign d_tag    = llrq_en ? {1'b0, q_slot, q_pilots, q_modcod} : {2'b00, pilots, modcod};
  end else begin : g_no_llrq
    assign l_ready  = d_ready;
    assign d_valid  = l_valid;
    assign d_i      = l_i;
    assign d_q      = l_q;
    assign d_first  = l_first;
    assign d_rate   = 4'(modcod - 5'd1);
    assign d_max_it = ldpc_max_it;
    assign d_tag    = {2'b00, pilots, modcod};
    assign cap_en   = 1'b0;
    assign cap      = '0;
    assign {st_llrq_depth, st_llrq_depth_max, st_llrq_frames, st_llrq_dropped0, st_llrq_dropped1,
            st_llrq_axi_err, st_llrq_busy} = '0;
    assign {m_axi_awaddr, m_axi_awlen, m_axi_awvalid, m_axi_wdata, m_axi_wlast, m_axi_wvalid,
            m_axi_araddr, m_axi_arlen, m_axi_arvalid} = '0;
    assign m_axi_awsize = 3'd3; assign m_axi_arsize = 3'd3;
    assign m_axi_awburst = 2'd1; assign m_axi_arburst = 2'd1;
    assign m_axi_awcache = 4'b0011; assign m_axi_arcache = 4'b0011;
    assign m_axi_wstrb = 8'hFF;
    assign m_axi_bready = 1'b1; assign m_axi_rready = 1'b1;
  end

  // ---------------------------------------------------------------- LDPC, BCH
  // per-frame tag through the LDPC and BCH decoders: {0, slot, pilots, MODCOD}
  logic       c_valid, c_ready, c_last, c_ok;   // LDPC -> BCH
  logic [7:0] c_data, c_chan;
  logic [5:0] c_iters;
  logic [3:0] c_rate;
  logic [7:0] c_tag, m_tag;

  dvbs2_ldpc_dec #(.ROM_FILE(LDPC_ROM_FILE)) u_ldpc (
    .clk, .rst,
    .s_valid(d_valid), .s_ready(d_ready), .s_llr_i(d_i), .s_llr_q(d_q), .s_first(d_first),
    .s_rate(d_rate), .s_max_it(d_max_it), .s_tag(d_tag),
    .it_cap_en(cap_en), .it_cap(cap),
    .m_valid(c_valid), .m_ready(c_ready), .m_data(c_data), .m_last(c_last), .m_ok(c_ok), .m_iters(c_iters), .m_chan(c_chan),
    .m_rate(c_rate), .m_tag(c_tag),
    .st_frames(st_ldpc_frames), .st_failed(st_ldpc_failed));

  dvbs2_bch_dec u_bch (
    .clk, .rst,
    .s_valid(c_valid), .s_ready(c_ready), .s_data(c_data), .s_last(c_last), .s_ok(c_ok), .s_iters(c_iters), .s_chan(c_chan),
    .s_rate(c_rate), .s_tag(c_tag),
    .m_valid, .m_ready, .m_data, .m_last, .m_ok, .m_ldpc_ok, .m_iters, .m_nerr, .m_raw_err,
    .m_rate, .m_tag,
    .st_frames(), .st_corrected(st_bch_corrected), .st_failed(st_bch_failed));

  assign m_modcod = m_tag[4:0];
  assign m_pilots = m_tag[5];
  assign m_slot   = m_tag[6];

endmodule
