// One narrowband channel front end: NCO + mixer + the integrators and the
// decimation of the 5-stage CIC (model: model/nbiq.py mixer(), cic()); the
// combs follow in nbiq_comb, shared by the channels. Held cleared while !en.
//
//   in : s_valid with s_i/s_q (16-bit ADC samples), at most one per 2 clocks
//   out: m_valid with m_i/m_q (W-bit, wrapping), one per R inputs; m_i/m_q
//        hold their value until the next m_valid (the comb reads them later)
//
// I and Q share the mixer's two multipliers and the integrators' adders:
// a sample's I goes through in one clock, its Q in the next. Each
// integrator keeps its two states in a pair of registers that rotates on
// every step (a = the state of the phase being added, b = the other one;
// the sum goes to b, b to a), so no multiplexers are needed.
//
// W = 53: the CIC gain R^5 <= 2^35 times the 18-bit input range (-2^17 ..
// 2^17 - 1) fits 53-bit two's complement, so the wrap-around arithmetic is
// exact up to R = 128.

module nbiq_chan #(
  parameter string LUT_FILE = "nbiq_lut.mem",
  parameter int    W = 53
) (
  input  logic               clk,
  input  logic               en,          // 0: state cleared

  input  logic [31:0]        finc,
  input  logic [7:0]         R,           // 2..128, static while en

  input  logic               s_valid,
  input  logic signed [15:0] s_i,
  input  logic signed [15:0] s_q,

  output logic               m_valid,
  output logic signed [W-1:0] m_i,
  output logic signed [W-1:0] m_q
);

  // ------------------------------------------------------------ NCO table
  (* rom_style = "block" *) logic [15:0] lut [1024];
  initial $readmemh(LUT_FILE, lut);

  logic [31:0] ph;
  logic [9:0]  ka, kb;
  logic [15:0] la, lb;
  logic [1:0]  q0, q1;
  logic        v0, v1, v2, v3, v4, v5;
  logic        f3, f4, f5;                   // phase of the stage: 0 I, 1 Q
  logic signed [15:0] xi0, xq0, xi1, xq1, xi2, xq2, xi3, xq3;
  logic signed [16:0] sn, cs, sn3, cs3;
  logic signed [32:0] pa, pb;
  logic signed [33:0] mx_s;
  logic signed [17:0] mx;
  logic        mx_v, mx_f;

  wire               qn   = v3 && !f3;          // stage 3 takes the Q products next
  wire signed [15:0] ma_x = qn ? xq3 : xi2;
  wire signed [16:0] ma_c = qn ? cs3 : cs;
  wire signed [15:0] mb_x = qn ? xi3 : xq2;
  wire signed [16:0] mb_s = qn ? sn3 : sn;

  function automatic logic signed [17:0] sat18(input logic signed [33:0] v);
    if (v > 34'sd131071) return 18'sd131071;
    if (v < -34'sd131072) return -18'sd131072;
    return 18'(v);
  endfunction

  always_ff @(posedge clk) begin
    // stage 0: phase -> table addresses
    v0 <= en && s_valid;
    if (!en) ph <= '0;
    else if (s_valid) ph <= ph + finc;
    ka  <= ph[29:20];
    kb  <= 10'd1023 - ph[29:20];
    q0  <= ph[31:30];
    xi0 <= s_i; xq0 <= s_q;
    // stage 1: table read
    v1 <= v0 && en;
    la <= lut[ka];
    lb <= lut[kb];
    q1 <= q0;
    xi1 <= xi0; xq1 <= xq0;
    // stage 2: quadrant
    v2 <= v1 && en;
    unique case (q1)
      2'd0: begin sn <=  17'(la); cs <=  17'(lb); end
      2'd1: begin sn <=  17'(lb); cs <= -17'(la); end
      2'd2: begin sn <= -17'(la); cs <= -17'(lb); end
      default: begin sn <= -17'(lb); cs <= 17'(la); end
    endcase
    xi2 <= xi1; xq2 <= xq1;
    // stage 3: products, I (x_i c, x_q s) in the sample's first clock, Q
    // (x_q c, x_i s) in the next, from the stage-2 values kept a clock
    v3 <= (v2 || qn) && en;
    f3 <= qn;
    xi3 <= xi2; xq3 <= xq2; sn3 <= sn; cs3 <= cs;
    pa <= ma_x * ma_c;
    pb <= mb_x * mb_s;
    // stage 4: sum + rounding (I: pa + pb, Q: pa - pb)
    v4 <= v3 && en;
    f4 <= f3;
    mx_s <= ((f3 ? 34'(pa) - 34'(pb) : 34'(pa) + 34'(pb)) + 34'sd4096) >>> 13;
    // stage 5: saturation
    mx_v <= v4 && en;
    mx_f <= f4;
    mx <= sat18(mx_s);
  end

  // ------------------------------------------------------------ CIC
  logic signed [W-1:0] sa [5], sb [5];       // rotating I/Q state pairs
  logic signed [W-1:0] sum [5];
  logic [7:0]  dcnt;
  logic        dec;                          // this sample is decimated
  logic        dv;                           // m_i/m_q hold a new sample

  always_comb begin
    sum[0] = sa[0] + W'(mx);
    for (int k = 1; k < 5; k++) sum[k] = sa[k] + sa[k - 1];
  end

  always_ff @(posedge clk) begin
    dv <= 1'b0;
    if (!en) begin
      for (int k = 0; k < 5; k++) begin sa[k] <= '0; sb[k] <= '0; end
      dcnt <= '0;
      dec <= 1'b0;
    end else if (mx_v) begin
      for (int k = 0; k < 5; k++) begin
        sb[k] <= sum[k];
        sa[k] <= sb[k];
      end
      if (!mx_f) begin
        dec <= dcnt == R - 8'd1;
        if (dcnt == R - 8'd1) begin
          dcnt <= '0;
          m_i <= sum[4];
        end else dcnt <= dcnt + 8'd1;
      end else if (dec) begin
        m_q <= sum[4];
        dv <= 1'b1;
      end
    end
  end

  // the decimated last integrator is held in m_i/m_q; the combs are shared
  // (nbiq_comb)
  always_ff @(posedge clk) m_valid <= dv && en;

endmodule
