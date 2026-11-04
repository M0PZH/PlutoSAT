// DVB-S2 receiver core, QPSK normal frames: 2 samples/symbol in, BBFRAMEs out.
//
//   in_clk samples -> async FIFO (512) -> dvbs2_rx_timing -> dvbs2_rx_framesync
//   -> dvbs2_rx_carrier (64800 LLRs per data frame) -> dvbs2_ldpc_dec
//   -> dvbs2_bch_dec -> BBFRAME bytes (Kbch/8 per frame, BB-descrambled),
//   m_ok: BCH codeword (after <= t corrections), m_ldpc_ok: LDPC converged
//
// Models: model/rxfixed.py (receive()), model/ldpcdec.py. The sample input
// has no backpressure; samples arriving with the FIFO full are dropped and
// counted. dvbs2_rx_pls decodes the PLS code of every PLHEADER (status only).

module dvbs2_rx #(
  parameter string LUT_FILE = "nco_lut.mem",
  parameter string LDPC_ROM_FILE = "ldpc_dec_rom.mem"
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

  output logic               m_valid,
  input  logic               m_ready,
  output logic [7:0]         m_data,
  output logic               m_last,
  output logic               m_ok,
  output logic               m_ldpc_ok,
  output logic [5:0]         m_iters,
  output logic [4:0]         m_nerr,       // BCH errors located
  output logic [15:0]        m_raw_err,    // pre-FEC bit errors of the frame (if m_ok)

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
  output logic [15:0]        st_pls_skipped
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
  logic signed [15:0] y_i, y_q;

  dvbs2_rx_framesync u_fs (
    .clk, .rst, .modcod, .pilots, .thresh(fs_thresh),
    .s_valid(t_valid), .s_ready(t_ready), .s_i(t_i), .s_q(t_q),
    .m_valid(y_valid), .m_ready(y_ready), .m_i(y_i), .m_q(y_q), .m_sof(y_sof), .m_dummy(y_dummy), .m_found(y_found),
    .ty_v, .ty_dummy,
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
    .st_seen(st_pls_seen), .st_skipped(st_pls_skipped));

  assign snap_t_v = t_valid && t_ready;
  assign snap_t_i = t_i;
  assign snap_t_q = t_q;

  logic       c_valid, c_ready, c_last, c_ok;   // LDPC -> BCH
  logic [7:0] c_data, c_chan;
  logic [5:0] c_iters;

  dvbs2_ldpc_dec #(.ROM_FILE(LDPC_ROM_FILE)) u_ldpc (
    .clk, .rst, .rate_idx(4'(modcod - 5'd1)), .max_it(ldpc_max_it),
    .s_valid(l_valid), .s_ready(l_ready), .s_llr_i(l_i), .s_llr_q(l_q), .s_first(l_first),
    .m_valid(c_valid), .m_ready(c_ready), .m_data(c_data), .m_last(c_last), .m_ok(c_ok), .m_iters(c_iters), .m_chan(c_chan),
    .st_frames(st_ldpc_frames), .st_failed(st_ldpc_failed));

  dvbs2_bch_dec u_bch (
    .clk, .rst, .rate_idx(4'(modcod - 5'd1)),
    .s_valid(c_valid), .s_ready(c_ready), .s_data(c_data), .s_last(c_last), .s_ok(c_ok), .s_iters(c_iters), .s_chan(c_chan),
    .m_valid, .m_ready, .m_data, .m_last, .m_ok, .m_ldpc_ok, .m_iters, .m_nerr, .m_raw_err,
    .st_frames(), .st_corrected(st_bch_corrected), .st_failed(st_bch_failed));

endmodule
