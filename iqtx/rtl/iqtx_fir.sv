// Interpolating FIR of the IQ transmitter, x5 (model: model/iqtx.py fir()):
//
//   v[5n + p] = sat20(rnd(sum_{j=0..23} h[5j + p] x[n - j], 16)),  p = 0..4
//
// 120 fixed taps (rtl/gen/iqtx_fir.mem, Q2.16, 18-bit) in a LUT ROM, the last
// 24 input samples in LUTRAM; samples before the start (clr) count as 0. One
// MAC per clock for I and one for Q (2 DSP48), one output about every 29
// clocks; the outputs go to a 4-entry FIFO (first-word fall-through), and a
// new output is started only when the FIFO has room for it. A new input
// sample is taken at p = 0.

module iqtx_fir #(
  parameter string FIR_FILE = "iqtx_fir.mem"
) (
  input  logic               clk,
  input  logic               clr,          // synchronous: empty state

  input  logic               s_valid,
  output logic               s_ready,
  input  logic signed [17:0] s_i,
  input  logic signed [17:0] s_q,

  output logic               m_valid,
  input  logic               m_ready,
  output logic signed [19:0] m_i,
  output logic signed [19:0] m_q
);

  localparam int TPP = 24;

  (* rom_style = "distributed" *) logic [17:0] rom [128];
  initial $readmemh(FIR_FILE, rom);

  (* ram_style = "distributed" *) logic [35:0] dl [32];

  logic [4:0] wp, base, fill, j;
  logic [2:0] p;
  logic [6:0] hi;                      // 5 j + p
  logic       busy, issuing;

  // output FIFO
  logic [39:0] of [4];
  logic [2:0]  ocnt;
  logic [1:0]  owp, orp;

  wire room  = !busy && ocnt < 3'd4;
  wire start = room && (p != 3'd0 || s_valid);
  assign s_ready = room && p == 3'd0;

  always_ff @(posedge clk)
    if (start && p == 3'd0) dl[5'(wp + 5'd1)] <= {s_q, s_i};

  // MAC pipeline: issue (j) -> read -> multiply -> accumulate
  logic               vr, fr, lr, vm, fm, lm, done;
  logic signed [17:0] di, dq, hh;
  logic signed [35:0] mi, mq;
  logic signed [47:0] acc_i, acc_q;

  function automatic logic signed [19:0] sat20r(input logic signed [47:0] a);
    logic signed [47:0] r;
    r = (a + 48'sd32768) >>> 16;
    if (r > 48'sd524287)  return 20'sd524287;
    if (r < -48'sd524288) return -20'sd524288;
    return r[19:0];
  endfunction

  always_ff @(posedge clk) begin
    logic [35:0] d;
    // read: sample x[n - j] (0 before the start), tap h[5 j + p]
    d = dl[5'(base - j)];
    di <= (j < fill) ? $signed(d[17:0]) : 18'sd0;
    dq <= (j < fill) ? $signed(d[35:18]) : 18'sd0;
    hh <= $signed(rom[hi]);
    // multiply
    mi <= di * hh;
    mq <= dq * hh;
    // accumulate
    if (vm) begin
      acc_i <= (fm ? 48'sd0 : acc_i) + 48'(mi);
      acc_q <= (fm ? 48'sd0 : acc_q) + 48'(mq);
    end
  end

  always_ff @(posedge clk) begin
    if (clr) begin
      wp <= '0; fill <= '0; p <= '0; busy <= 1'b0; issuing <= 1'b0;
      vr <= 1'b0; vm <= 1'b0; done <= 1'b0;
    end else begin
      if (start) begin
        busy <= 1'b1;
        issuing <= 1'b1;
        j <= '0;
        hi <= 7'(p);
        if (p == 3'd0) begin
          wp <= wp + 5'd1;
          base <= wp + 5'd1;
          if (fill != 5'(TPP)) fill <= fill + 5'd1;
        end else
          base <= wp;
      end else if (issuing) begin
        j <= j + 5'd1;
        hi <= hi + 7'd5;
        if (j == 5'(TPP - 1)) issuing <= 1'b0;
      end
      vr <= issuing; fr <= issuing && j == 5'd0; lr <= issuing && j == 5'(TPP - 1);
      vm <= vr;      fm <= fr;                   lm <= vr && lr;
      done <= vm && lm;
      if (done) begin
        busy <= 1'b0;
        p <= (p == 3'd4) ? 3'd0 : p + 3'd1;
      end
    end
  end

  // output FIFO
  assign m_valid = ocnt != 3'd0;
  assign m_i = $signed(of[orp][19:0]);
  assign m_q = $signed(of[orp][39:20]);
  always_ff @(posedge clk) begin
    if (clr) begin
      ocnt <= '0; owp <= '0; orp <= '0;
    end else begin
      if (done) begin
        of[owp] <= {sat20r(acc_q), sat20r(acc_i)};
        owp <= owp + 2'd1;
      end
      if (m_valid && m_ready) orp <= orp + 2'd1;
      ocnt <= ocnt + 3'(done) - 3'(m_valid && m_ready);
    end
  end

endmodule
