// The CIC combs and output scaling, shared by the channels (model:
// model/nbiq.py cic()): the decimated integrator outputs of the channels are
// taken one channel per clock through 5 pipelined comb stages and
// sat24(round(. / 2^cshift)).
//
// The comb delays of stage k are a distributed RAM addressed by the channel
// (nbiq_spram8 below; read and written at the same address in the same
// clock, the read gives the old value). A channel's delays are not cleared
// on disable; instead a valid flag per stage and channel, cleared while the
// channel is disabled, set by the first sample through the stage, makes the
// stage read 0 until then (the same results as cleared delays).
//
// Each channel delivers at most one sample per NCH clocks (fs / R <= clk /
// NCH) and holds it (s_i/s_q) until its next s_valid; the lowest pending
// channel goes first.

module nbiq_comb #(
  parameter int NCH = 5,
  parameter int W = 53
) (
  input  logic               clk,
  input  logic [NCH-1:0]     en,
  input  logic [4:0]         cshift [NCH],

  input  logic [NCH-1:0]     s_valid,
  input  logic signed [W-1:0] s_i [NCH],
  input  logic signed [W-1:0] s_q [NCH],

  output logic               m_valid,
  output logic [2:0]         m_ch,
  output logic signed [23:0] m_i,
  output logic signed [23:0] m_q
);

  // ------------------------------------------------------------ one channel per clock
  logic [NCH-1:0] pend;
  logic           take;
  logic [2:0]     tc;

  always_comb begin
    take = 1'b0; tc = '0;
    for (int c = NCH - 1; c >= 0; c--)
      if (pend[c]) begin take = 1'b1; tc = 3'(c); end
  end

  always_ff @(posedge clk)
    for (int c = 0; c < NCH; c++) begin
      if (s_valid[c]) pend[c] <= 1'b1;
      else if (take && tc == 3'(c)) pend[c] <= 1'b0;
      if (!en[c]) pend[c] <= 1'b0;
    end

  logic        v [6];
  (* max_fanout = 32 *) logic [2:0] ch [6];
  (* max_fanout = 32 *) logic       dok [5];   // stage k's delay holds a sample of ch[k]
  logic [NCH-1:0] dv [5];                  // ... of channel c
  logic signed [W-1:0] ci [6], cq [6];

  always_ff @(posedge clk) begin
    v[0]  <= take;
    ch[0] <= tc;
    ci[0] <= s_i[tc];
    cq[0] <= s_q[tc];
  end

  // dok[k] is looked up as the sample enters stage k: dv[k] can only change
  // for the channel in stage k, and the same channel is never in two
  // consecutive slots (or is being disabled, and its sample is dropped)
  always_ff @(posedge clk) dok[0] <= dv[0][tc];

  for (genvar k = 0; k < 5; k++) begin : g_st
    wire  [2*W-1:0] d;
    nbiq_spram8 #(.W(2 * W)) u_dl (.clk, .we(v[k]), .a(ch[k]), .d({cq[k], ci[k]}), .q(d));
    always_ff @(posedge clk) begin
      v[k + 1]  <= v[k] && en[ch[k]];
      ch[k + 1] <= ch[k];
      ci[k + 1] <= ci[k] - (dok[k] ? $signed(d[W-1:0]) : W'(0));
      cq[k + 1] <= cq[k] - (dok[k] ? $signed(d[2*W-1:W]) : W'(0));
    end
    if (k < 4) begin : g_ok
      always_ff @(posedge clk) dok[k + 1] <= dv[k + 1][ch[k]];
    end
    always_ff @(posedge clk)
      for (int c = 0; c < NCH; c++) begin
        if (v[k] && ch[k] == 3'(c)) dv[k][c] <= 1'b1;
        if (!en[c]) dv[k][c] <= 1'b0;
      end
  end

  // ------------------------------------------------------------ round and shift, saturate
  function automatic logic signed [W:0] rsh(input logic signed [W-1:0] x, input logic [4:0] sh);
    return (sh == 0) ? (W+1)'(x) : ((W+1)'(x) + ((W+1)'(1) <<< (sh - 5'd1))) >>> sh;
  endfunction
  function automatic logic signed [23:0] sat24(input logic signed [W:0] r);
    if (r > (W+1)'(8388607)) return 24'sd8388607;
    if (r < -(W+1)'(8388608)) return -24'sd8388608;
    return 24'(r);
  endfunction

  logic [4:0]  sh5;
  logic        o_v;
  logic [2:0]  o_ch;
  logic signed [W:0] o_i, o_q;
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


// 8 x W single-port distributed RAM, asynchronous read at the write
// address. For synthesis RAM32M with all four address ports together: 8
// bits in 4 LUTs (inferred, Vivado takes one LUT per bit).
module nbiq_spram8 #(
  parameter int W = 106
) (
  input  logic         clk,
  input  logic         we,
  input  logic [2:0]   a,
  input  logic [W-1:0] d,
  output logic [W-1:0] q
);
`ifdef SYNTHESIS
  localparam int N = (W + 7) / 8;
  logic [8*N-1:0] dp, qp;
  assign dp = (8*N)'(d);
  assign q = qp[W-1:0];
  for (genvar k = 0; k < N; k++) begin : g
    RAM32M u (
      .DOA(qp[8*k +: 2]), .DOB(qp[8*k+2 +: 2]), .DOC(qp[8*k+4 +: 2]), .DOD(qp[8*k+6 +: 2]),
      .ADDRA({2'b00, a}), .ADDRB({2'b00, a}), .ADDRC({2'b00, a}), .ADDRD({2'b00, a}),
      .DIA(dp[8*k +: 2]), .DIB(dp[8*k+2 +: 2]), .DIC(dp[8*k+4 +: 2]), .DID(dp[8*k+6 +: 2]),
      .WCLK(clk), .WE(we));
  end
`else
  logic [W-1:0] mem [8];
  always_ff @(posedge clk) if (we) mem[a] <= d;
  assign q = mem[a];
`endif
endmodule
