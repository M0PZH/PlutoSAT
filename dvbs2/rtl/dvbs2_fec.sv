// BBFRAME bytes -> FECFRAME bits: BB scrambler, BCH, LDPC.

module dvbs2_fec #(
  parameter string LDPC_ROM_FILE = "ldpc_rom.mem"
) (
  input  logic       clk,
  input  logic       rst,

  input  logic       s_valid,
  output logic       s_ready,
  input  logic [7:0] s_data,
  input  logic [4:0] s_user,     // {pilots, rate_idx}
  input  logic       s_last,

  output logic       m_valid,
  input  logic       m_ready,
  output logic       m_data,
  output logic [4:0] m_user,
  output logic       m_last,

  output logic       err_len
);

  logic       a_valid, a_ready, a_data, a_last;
  logic [4:0] a_user;
  logic       b_valid, b_ready, b_data, b_last;
  logic [4:0] b_user;
  logic       c_valid, c_ready, c_data, c_last;
  logic [4:0] c_user;
  logic       d_valid, d_ready, d_data, d_last;
  logic [4:0] d_user;

  dvbs2_bbscrambler i_bbscr (
    .clk, .rst,
    .s_valid, .s_ready, .s_data, .s_user, .s_last,
    .m_valid(a_valid), .m_ready(a_ready), .m_data(a_data), .m_user(a_user), .m_last(a_last),
    .err_len);

  dvbs2_axis_skid #(.W(7)) i_skid_a (
    .clk, .rst,
    .s_valid(a_valid), .s_ready(a_ready), .s_data({a_last, a_user, a_data}),
    .m_valid(b_valid), .m_ready(b_ready), .m_data({b_last, b_user, b_data}));

  dvbs2_bch_enc i_bch (
    .clk, .rst,
    .s_valid(b_valid), .s_ready(b_ready), .s_data(b_data), .s_user(b_user), .s_last(b_last),
    .m_valid(c_valid), .m_ready(c_ready), .m_data(c_data), .m_user(c_user), .m_last(c_last));

  dvbs2_axis_skid #(.W(7)) i_skid_c (
    .clk, .rst,
    .s_valid(c_valid), .s_ready(c_ready), .s_data({c_last, c_user, c_data}),
    .m_valid(d_valid), .m_ready(d_ready), .m_data({d_last, d_user, d_data}));

  dvbs2_ldpc_enc #(.ROM_FILE(LDPC_ROM_FILE)) i_ldpc (
    .clk, .rst,
    .s_valid(d_valid), .s_ready(d_ready), .s_data(d_data), .s_user(d_user), .s_last(d_last),
    .m_valid, .m_ready, .m_data, .m_user, .m_last);

endmodule
