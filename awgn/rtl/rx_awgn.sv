// Self-test noise source on the AD9363 receive samples (ADC clock domain;
// model: model/awgn.py). All four lanes (rx0 I/Q, rx1 I/Q) and their valids
// are delayed by the same 6 clocks; rx0 gets
//
//   y = sat16((x * gain + g * nscale + 2^13) >> 14)     while en
//   y = x                                               otherwise
//
// g: per component the sum of 16 random bytes - 2040 (eight xorshift32
// generators, one step per clock), sigma 295.6. Power meters: mean of
// I^2 + Q^2 over 2^16 rx0 samples of the input and of the output.

module rx_awgn (
  input  logic               clk,
  input  logic               rst,

  input  logic               en,
  input  logic [15:0]        gain,        // Q2.14
  input  logic [16:0]        nscale,

  input  logic [3:0]         in_valid,    // i0, q0, i1, q1
  input  logic signed [15:0] in_i0, in_q0, in_i1, in_q1,
  output logic [3:0]         out_valid,
  output logic signed [15:0] out_i0, out_q0, out_i1, out_q1,

  output logic               p_done,      // pulse: p_in / p_out updated
  output logic [31:0]        p_in,
  output logic [31:0]        p_out
);

  // ------------------------------------------------------------ noise
  localparam logic [31:0] SEED [8] = '{32'h9E3779B9, 32'h7F4A7C15, 32'h85EBCA6B, 32'hC2B2AE35,
                                       32'h27D4EB2F, 32'h165667B1, 32'hD3A2646C, 32'hFD7046C5};
  logic [31:0] st [8];

  function automatic logic [31:0] xs32(input logic [31:0] x);
    x = x ^ (x << 13);
    x = x ^ (x >> 17);
    x = x ^ (x << 5);
    return x;
  endfunction

  always_ff @(posedge clk)
    for (int k = 0; k < 8; k++) st[k] <= rst ? SEED[k] : xs32(st[k]);

  // adder trees: 16 bytes -> 8 -> 4 -> 2 -> 1, one level per clock
  logic [8:0]  a1i [8], a1q [8];
  logic [9:0]  a2i [4], a2q [4];
  logic [10:0] a3i [2], a3q [2];
  logic [11:0] a4i, a4q;
  logic signed [12:0] gi, gq;
  always_ff @(posedge clk) begin
    for (int k = 0; k < 8; k++) begin
      a1i[k] <= 9'(st[k / 2][(k % 2) * 16 +: 8]) + 9'(st[k / 2][(k % 2) * 16 + 8 +: 8]);
      a1q[k] <= 9'(st[4 + k / 2][(k % 2) * 16 +: 8]) + 9'(st[4 + k / 2][(k % 2) * 16 + 8 +: 8]);
    end
    for (int k = 0; k < 4; k++) begin
      a2i[k] <= 10'(a1i[2 * k]) + 10'(a1i[2 * k + 1]);
      a2q[k] <= 10'(a1q[2 * k]) + 10'(a1q[2 * k + 1]);
    end
    for (int k = 0; k < 2; k++) begin
      a3i[k] <= 11'(a2i[2 * k]) + 11'(a2i[2 * k + 1]);
      a3q[k] <= 11'(a2q[2 * k]) + 11'(a2q[2 * k + 1]);
    end
    a4i <= 12'(a3i[0]) + 12'(a3i[1]);
    a4q <= 12'(a3q[0]) + 12'(a3q[1]);
    gi <= $signed({1'b0, a4i}) - 13'sd2040;
    gq <= $signed({1'b0, a4q}) - 13'sd2040;
  end

  // ------------------------------------------------------------ data path
  logic [3:0]  v0, v1, v2, v3, v4;
  logic signed [15:0] x0 [4], x1 [4], x2 [4], x3 [4], x4 [4];
  logic signed [20:0] ri, rq;
  logic signed [32:0] pai, paq, pai2, paq2;
  logic signed [30:0] pni, pnq, pni2, pnq2;
  logic signed [34:0] si, sq;
  logic        en_q;
  logic signed [16:0] g17;
  logic signed [17:0] n18;

  assign g17 = {1'b0, gain};
  assign n18 = {1'b0, nscale};

  function automatic logic signed [15:0] sat16(input logic signed [20:0] r);
    if (r > 21'sd32767) return 16'sd32767;
    if (r < -21'sd32768) return -16'sd32768;
    return 16'(r);
  endfunction

  always_ff @(posedge clk) begin
    en_q <= en;
    v0 <= in_valid;
    x0 <= '{in_i0, in_q0, in_i1, in_q1};
    // products (2 clocks: DSP input + multiplier registers)
    v1 <= v0; x1 <= x0;
    pai <= x0[0] * g17;
    paq <= x0[1] * g17;
    pni <= gi * n18;
    pnq <= gq * n18;
    v2 <= v1; x2 <= x1;
    pai2 <= pai; paq2 <= paq; pni2 <= pni; pnq2 <= pnq;
    // sum
    v3 <= v2; x3 <= x2;
    si <= 35'(pai2) + 35'(pni2) + 35'sd8192;
    sq <= 35'(paq2) + 35'(pnq2) + 35'sd8192;
    // shift
    v4 <= v3; x4 <= x3;
    ri <= 21'(si >>> 14);
    rq <= 21'(sq >>> 14);
    // saturate, out
    out_valid <= v4;
    out_i0 <= en_q ? sat16(ri) : x4[0];
    out_q0 <= en_q ? sat16(rq) : x4[1];
    out_i1 <= x4[2];
    out_q1 <= x4[3];
  end

  // ------------------------------------------------------------ power meters
  // the rx0 input aligned with out_*0 (for the input meter)
  logic signed [15:0] x3_d [2];
  always_ff @(posedge clk) x3_d <= '{x4[0], x4[1]};

  logic        m_v0, m_v1, m_v, m_v2;
  logic signed [15:0] ma, mb, mc, md;
  logic [31:0] sqa0, sqb0, sqc0, sqd0, sqa, sqb, sqc, sqd;
  logic [32:0] e_in, e_out;
  logic [48:0] acc_in, acc_out;
  logic [15:0] mcnt;
  always_ff @(posedge clk) begin
    // DSP input, multiplier and output registers
    m_v0 <= out_valid[0];
    ma <= out_i0; mb <= out_q0; mc <= x3_d[0]; md <= x3_d[1];
    m_v1 <= m_v0;
    sqa0 <= 32'(ma * ma); sqb0 <= 32'(mb * mb); sqc0 <= 32'(mc * mc); sqd0 <= 32'(md * md);
    m_v <= m_v1;
    sqa <= sqa0; sqb <= sqb0; sqc <= sqc0; sqd <= sqd0;
    m_v2 <= m_v;
    e_out <= 33'(sqa) + 33'(sqb);
    e_in <= 33'(sqc) + 33'(sqd);
    p_done <= 1'b0;
    if (rst) begin
      acc_in <= '0; acc_out <= '0; mcnt <= '0;
    end else if (m_v2) begin
      mcnt <= mcnt + 16'd1;
      if (mcnt == 16'hFFFF) begin
        p_in <= 32'((acc_in + 49'(e_in)) >> 16);
        p_out <= 32'((acc_out + 49'(e_out)) >> 16);
        p_done <= 1'b1;
        acc_in <= '0; acc_out <= '0;
      end else begin
        acc_in <= acc_in + 49'(e_in);
        acc_out <= acc_out + 49'(e_out);
      end
    end
  end

endmodule
