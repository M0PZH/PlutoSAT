// The CIC combs and output scaling, shared by the channels (model:
// model/nbiq.py cic()): the decimated integrator outputs of the channels are
// taken one channel per clock through 5 pipelined comb stages (per-channel
// delay registers) and sat24(round(. / 2^cshift)).
//
// Each channel delivers at most one sample per NCH clocks (fs / R <= clk / NCH).

module nbiq_comb #(
  parameter int NCH = 4
) (
  input  logic               clk,
  input  logic [NCH-1:0]     en,
  input  logic [4:0]         cshift [NCH],

  input  logic [NCH-1:0]     s_valid,
  input  logic signed [47:0] s_i [NCH],
  input  logic signed [47:0] s_q [NCH],

  output logic               m_valid,
  output logic [1:0]         m_ch,
  output logic signed [23:0] m_i,
  output logic signed [23:0] m_q
);

  // ------------------------------------------------------------ one channel per clock
  logic [NCH-1:0] pend;
  logic [47:0]    hi [NCH], hq [NCH];
  logic           take;
  logic [1:0]     tc;

  always_comb begin
    take = 1'b0; tc = '0;
    for (int c = NCH - 1; c >= 0; c--)
      if (pend[c]) begin take = 1'b1; tc = 2'(c); end
  end

  logic        v [6];
  logic [1:0]  ch [6];
  logic signed [47:0] ci [6], cq [6];
  logic signed [47:0] di [5][NCH], dq [5][NCH];

  always_ff @(posedge clk) begin
    for (int c = 0; c < NCH; c++) begin
      if (s_valid[c]) begin
        pend[c] <= 1'b1;
        hi[c] <= s_i[c];
        hq[c] <= s_q[c];
      end else if (take && tc == 2'(c)) pend[c] <= 1'b0;
      if (!en[c]) pend[c] <= 1'b0;
    end

    v[0]  <= take;
    ch[0] <= tc;
    ci[0] <= hi[tc];
    cq[0] <= hq[tc];
    for (int k = 0; k < 5; k++) begin
      v[k + 1]  <= v[k] && en[ch[k]];
      ch[k + 1] <= ch[k];
      ci[k + 1] <= ci[k] - di[k][ch[k]];
      cq[k + 1] <= cq[k] - dq[k][ch[k]];
      if (v[k]) begin
        di[k][ch[k]] <= ci[k];
        dq[k][ch[k]] <= cq[k];
      end
    end
    for (int c = 0; c < NCH; c++)
      if (!en[c]) for (int k = 0; k < 5; k++) begin di[k][c] <= '0; dq[k][c] <= '0; end
  end

  // ------------------------------------------------------------ round and shift, saturate
  function automatic logic signed [48:0] rsh(input logic signed [47:0] x, input logic [4:0] sh);
    return (sh == 0) ? 49'(x) : (49'(x) + (49'sd1 <<< (sh - 5'd1))) >>> sh;
  endfunction
  function automatic logic signed [23:0] sat24(input logic signed [48:0] r);
    if (r > 49'sd8388607) return 24'sd8388607;
    if (r < -49'sd8388608) return -24'sd8388608;
    return 24'(r);
  endfunction

  logic [4:0]  sh5;
  logic        o_v;
  logic [1:0]  o_ch;
  logic signed [48:0] o_i, o_q;
  always_ff @(posedge clk) begin
    sh5 <= cshift[ch[4]];                    // for the sample in stage 5 next clock
    o_v <= v[5] && en[ch[5]];
    o_ch <= ch[5];
    o_i <= rsh(ci[5], sh5);
    o_q <= rsh(cq[5], sh5);
    m_valid <= o_v && en[o_ch];
    m_ch <= o_ch;
    m_i <= sat24(o_i);
    m_q <= sat24(o_q);
  end

endmodule
