// DVB-S2 transmitter core: BBFRAME bytes in, complex baseband samples out.
//
//   s_axis (BBFRAME bytes + per-frame {psk8, short, pilots, rate_idx})
//     -> BB scrambler -> BCH -> LDPC                (dvbs2_fec)
//     -> FECFRAME buffer, 8PSK interleaver, mapping (dvbs2_frame_fifo)
//     -> PL framing, pilots, PL scrambling, dummies (dvbs2_plframer)
//     -> symbol amplitudes, RRC polyphase interpolation by P (dvbs2_rrc_interp)
//     -> CIC interpolation by R, off for R <= 1     (dvbs2_cic_interp)
//     -> NCO frequency shift                        (dvbs2_nco)
//   m_axis (16-bit I/Q at P * R * symbol rate, e.g. 12 MS/s)
//
// Single clock domain; the sample output is pulled by the DAC side (m_ready).
// Samples per symbol = P (sps, 2..MAX_SPS) times R (cic_r, 2..32); with
// R = 0 or 1 (and cic_shift = 0) the CIC passes the RRC output unchanged.
// QPSK (all 11 code rates) and 8PSK (3/5 .. 9/10), normal and short
// FECFRAMEs (short: no 9/10), pilots on/off, all per frame.

module dvbs2_tx #(
  parameter string LDPC_ROM_FILE = "ldpc_rom.mem",
  parameter string NCO_LUT_FILE  = "nco_lut.mem",
  parameter int    TPP           = 16,
  parameter int    MACS          = 4,
  parameter int    MAX_SPS       = 64
) (
  input  logic        clk,
  input  logic        rst,

  // configuration (static while enabled, except coefficients)
  input  logic        enable,        // start emitting PLFRAMEs
  input  logic [6:0]  sps,           // RRC interpolation factor P
  input  logic [5:0]  cic_r,         // CIC interpolation factor R (0, 1: off)
  input  logic [3:0]  cic_shift,     // CIC output shift, floor(log2 R^2)
  input  logic [17:0] gold_xa,
  input  logic [17:0] gold_xb,
  input  logic [31:0] nco_finc,
  input  logic        nco_bypass,
  input  logic        carrier_only,  // constant symbols: a tone at the NCO offset
  input  logic        coef_we,
  input  logic [$clog2(MAX_SPS * TPP)-1:0] coef_addr,
  input  logic [17:0] coef_data,

  // BBFRAME input
  input  logic        s_valid,
  output logic        s_ready,
  input  logic [7:0]  s_data,
  input  logic [6:0]  s_user,       // {psk8, short, pilots, rate_idx[3:0]}
  input  logic        s_last,

  // sample output {Q, I}
  output logic        m_valid,
  input  logic        m_ready,
  output logic [15:0] m_i,
  output logic [15:0] m_q,

  // status
  output logic [31:0] cnt_data_frames,
  output logic [31:0] cnt_dummy_frames,
  output logic [31:0] cnt_len_errors,
  output logic [16:0] fifo_level,
  output logic [2:0]  fifo_frames,

  // symbol tap (before shaping), for test and debug
  output logic        sym_valid,
  output logic [2:0]  sym_ph,        // phase index p: symbol exp(j pi p / 4)
  output logic        sym_sof,
  output logic        sym_dummy
);

  localparam logic signed [15:0] SYM_A = 16'sd23170;  // 1/sqrt(2) in Q1.15
  localparam logic signed [15:0] SYM_1 = 16'sd32767;  // 1 in Q1.15 (8PSK axis points)

  // symbol exp(j pi p / 4) in Q1.15 (model: txmodel.PHASE_IQ)
  function automatic logic signed [31:0] phase_iq(input logic [2:0] p);   // {Q, I}
    case (p)
      3'd0:    phase_iq = {16'sd0, SYM_1};
      3'd1:    phase_iq = {SYM_A, SYM_A};
      3'd2:    phase_iq = {SYM_1, 16'sd0};
      3'd3:    phase_iq = {SYM_A, -SYM_A};
      3'd4:    phase_iq = {16'sd0, -SYM_1};
      3'd5:    phase_iq = {-SYM_A, -SYM_A};
      3'd6:    phase_iq = {-SYM_1, 16'sd0};
      default: phase_iq = {-SYM_A, SYM_A};
    endcase
  endfunction

  // ---------------------------------------------------------------- FEC
  logic       f_valid, f_ready, f_data, f_last, err_len;
  logic [6:0] f_user;

  dvbs2_fec #(.LDPC_ROM_FILE(LDPC_ROM_FILE)) i_fec (
    .clk, .rst,
    .s_valid, .s_ready, .s_data, .s_user, .s_last,
    .m_valid(f_valid), .m_ready(f_ready), .m_data(f_data), .m_user(f_user), .m_last(f_last),
    .err_len);

  // ---------------------------------------------------------------- frame buffer
  logic       frame_avail, frame_pop, fs_valid, fs_rd;
  logic [6:0] frame_cfg;
  logic [2:0] fs_data;
  logic [16:0] level;

  dvbs2_frame_fifo i_fifo (
    .clk, .rst,
    .s_valid(f_valid), .s_ready(f_ready), .s_data(f_data), .s_user(f_user), .s_last(f_last),
    .frame_avail, .frame_cfg, .frame_pop,
    .sym_valid(fs_valid), .sym_data(fs_data), .sym_rd(fs_rd),
    .level, .frames(fifo_frames));

  assign fifo_level = 17'(level);

  // ---------------------------------------------------------------- PL framer
  logic       p_valid, p_ready, p_sof, p_dummy, st_data, st_dummy;
  logic [2:0] p_ph;

  dvbs2_plframer i_framer (
    .clk, .rst, .enable, .gold_xa, .gold_xb,
    .frame_avail, .frame_cfg, .frame_pop,
    .sym_valid(fs_valid), .sym_data(fs_data), .sym_rd(fs_rd),
    .m_valid(p_valid), .m_ready(p_ready), .m_ph(p_ph),
    .m_sof(p_sof), .m_dummy(p_dummy),
    .stat_data_frame(st_data), .stat_dummy_frame(st_dummy));

  assign sym_valid = p_valid && p_ready;
  assign sym_ph    = p_ph;
  assign sym_sof   = p_sof;
  assign sym_dummy = p_dummy;

  // ---------------------------------------------------------------- shaping + NCO
  logic               r_valid, r_ready, c_valid, c_ready;
  wire  [31:0]        sym_iq = phase_iq(carrier_only ? 3'd1 : p_ph);
  logic signed [15:0] r_i, r_q, c_i, c_q;

  dvbs2_rrc_interp #(.TPP(TPP), .MACS(MACS), .MAX_SPS(MAX_SPS)) i_rrc (
    .clk, .rst, .sps,
    .coef_we, .coef_addr, .coef_data,
    .s_valid(p_valid), .s_ready(p_ready),
    .s_i(sym_iq[15:0]), .s_q(sym_iq[31:16]),
    .m_valid(r_valid), .m_ready(r_ready), .m_i(r_i), .m_q(r_q));

  dvbs2_cic_interp i_cic (
    .clk, .rst, .r(cic_r), .cshift(cic_shift),
    .s_valid(r_valid), .s_ready(r_ready), .s_i(r_i), .s_q(r_q),
    .m_valid(c_valid), .m_ready(c_ready), .m_i(c_i), .m_q(c_q));

  logic signed [15:0] n_i, n_q;
  dvbs2_nco #(.LUT_FILE(NCO_LUT_FILE)) i_nco (
    .clk, .rst, .finc(nco_finc), .bypass(nco_bypass),
    .s_valid(c_valid), .s_ready(c_ready), .s_i(c_i), .s_q(c_q),
    .m_valid, .m_ready, .m_i(n_i), .m_q(n_q));

  assign m_i = n_i;
  assign m_q = n_q;

  // ---------------------------------------------------------------- counters
  always_ff @(posedge clk) begin
    if (rst) begin
      cnt_data_frames  <= '0;
      cnt_dummy_frames <= '0;
      cnt_len_errors   <= '0;
    end else begin
      if (st_data)  cnt_data_frames  <= cnt_data_frames + 1;
      if (st_dummy) cnt_dummy_frames <= cnt_dummy_frames + 1;
      if (err_len)  cnt_len_errors   <= cnt_len_errors + 1;
    end
  end

endmodule
