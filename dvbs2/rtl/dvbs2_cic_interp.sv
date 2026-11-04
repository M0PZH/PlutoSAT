// 3-stage CIC interpolator by R behind the RRC interpolator (model:
// model/txmodel.py cic_interp()). With x[m] the input at the RRC rate:
//
//   d[m] = x[m] - 2 x[m-1] + x[m-2]              two combs at the input rate
//   u[n] = d[floor(n / R)]                       hold (= the third comb, zero
//                                                stuffing and the first integrator)
//   v[n] = v[n-1] + u[n],  w[n] = w[n-1] + v[n]  two integrators at the output rate
//   y[n] = sat16((w[n] + rc) >> cshift),  rc = 2^(cshift-1), 0 for cshift = 0
//
// (samples before the first one are zero). Transfer function
// ((1 - z^-R) / (1 - z^-1))^3, gain R^2 / 2^cshift on every output phase.
// Software uses cshift = floor(log2 R^2) and scales the RRC taps by
// 2^cshift / R^2 (<= 1), so the output level does not depend on R.
// The integrators wrap around at W = 26 bits; that is exact because the
// result fits (|y| <= 2^15 R^2 <= 2^25 for R <= 32). The rounding constant
// is the start value of the last integrator.
// R = 0 or 1 with cshift = 0 passes samples through unchanged.
//
// One output per clock at most; a stall-able pipeline (all stages move
// when the output register is free or taken), like dvbs2_nco. r and cshift
// are static (change them in reset).

module dvbs2_cic_interp #(
  parameter int W = 26
) (
  input  logic               clk,
  input  logic               rst,
  input  logic [5:0]         r,          // 0, 1: no interpolation; 2..32
  input  logic [3:0]         cshift,     // 0..10

  input  logic               s_valid,
  output logic               s_ready,
  input  logic signed [15:0] s_i,
  input  logic signed [15:0] s_q,

  output logic               m_valid,
  input  logic               m_ready,
  output logic signed [15:0] m_i,
  output logic signed [15:0] m_q
);

  logic [5:0]   rm1;                     // R - 1 (0 for R = 0, 1)
  logic [W-1:0] rc;
  always_ff @(posedge clk) begin
    rm1 <= (r > 6'd1) ? r - 6'd1 : 6'd0;
    rc  <= (cshift == 4'd0) ? '0 : W'(1) << (cshift - 4'd1);
  end

  wire ce = !m_valid || m_ready;

  // ------------------------------------------------------------ combs + hold
  logic [5:0]         ph;                // output phase within an input sample
  logic signed [15:0] xi1, xq1;          // x[m-1]
  logic signed [16:0] ci1, cq1;          // x[m-1] - x[m-2]
  logic signed [17:0] di, dq;            // d[m], held for R outputs
  logic               a_v, b_v, c_v;

  assign s_ready = ce && (ph == 6'd0);
  wire take = s_valid && s_ready;
  wire step = ce && ((ph != 6'd0) || s_valid);

  wire signed [16:0] ci0 = 17'(s_i) - 17'(xi1);
  wire signed [16:0] cq0 = 17'(s_q) - 17'(xq1);

  always_ff @(posedge clk) begin
    if (rst) begin
      ph <= '0;
      xi1 <= '0; xq1 <= '0; ci1 <= '0; cq1 <= '0; di <= '0; dq <= '0;
      a_v <= 1'b0;
    end else if (ce) begin
      a_v <= step;
      if (step) ph <= (ph == rm1) ? 6'd0 : ph + 6'd1;
      if (take) begin
        xi1 <= s_i;
        xq1 <= s_q;
        ci1 <= ci0;
        cq1 <= cq0;
        di  <= 18'(ci0) - 18'(ci1);
        dq  <= 18'(cq0) - 18'(cq1);
      end
    end
  end

  // ------------------------------------------------------------ integrators
  logic signed [W-1:0] vi, vq, wi, wq;
  logic                first;            // the last integrator starts from rc

  always_ff @(posedge clk) begin
    if (rst) begin
      vi <= '0; vq <= '0; wi <= '0; wq <= '0;
      first <= 1'b1;
      b_v <= 1'b0; c_v <= 1'b0;
    end else if (ce) begin
      b_v <= a_v;
      c_v <= b_v;
      if (a_v) begin
        vi <= vi + W'(di);
        vq <= vq + W'(dq);
      end
      if (b_v) begin
        wi <= (first ? rc : wi) + vi;
        wq <= (first ? rc : wq) + vq;
        first <= 1'b0;
      end
    end
  end

  // ------------------------------------------------------------ shift, saturate
  function automatic logic signed [15:0] sat16(input logic signed [W-1:0] a);
    if (a > 32767)       sat16 = 16'sd32767;
    else if (a < -32768) sat16 = -16'sd32768;
    else                 sat16 = a[15:0];
  endfunction

  logic signed [W-1:0] si, sq;
  logic                d_v;
  always_ff @(posedge clk) begin
    if (rst) begin
      d_v <= 1'b0;
      m_valid <= 1'b0;
    end else if (ce) begin
      d_v <= c_v;
      if (c_v) begin
        si <= wi >>> cshift;
        sq <= wq >>> cshift;
      end
      m_valid <= d_v;
      if (d_v) begin
        m_i <= sat16(si);
        m_q <= sat16(sq);
      end
    end
  end

endmodule
