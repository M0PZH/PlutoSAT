// One narrowband channel front end: NCO + mixer + the integrators and the
// decimation of the 5-stage CIC (model: model/nbiq.py mixer(), cic()); the
// combs follow in nbiq_comb, shared by the channels. Held cleared while !en.
//
//   in : s_valid with s_i/s_q (16-bit ADC samples), at most one per clock
//   out: m_valid with m_i/m_q (48-bit, wrapping), one per R inputs

module nbiq_chan #(
  parameter string LUT_FILE = "nbiq_lut.mem"
) (
  input  logic               clk,
  input  logic               en,          // 0: state cleared

  input  logic [31:0]        finc,
  input  logic [6:0]         R,           // 2..64, static while en

  input  logic               s_valid,
  input  logic signed [15:0] s_i,
  input  logic signed [15:0] s_q,

  output logic               m_valid,
  output logic signed [47:0] m_i,
  output logic signed [47:0] m_q
);

  // ------------------------------------------------------------ NCO table
  (* rom_style = "block" *) logic [15:0] lut [1024];
  initial $readmemh(LUT_FILE, lut);

  logic [31:0] ph;
  logic [9:0]  ka, kb;
  logic [15:0] la, lb;
  logic [1:0]  q0, q1;
  logic        v0, v1, v2, v3, v4;
  logic signed [15:0] xi0, xq0, xi1, xq1, xi2, xq2;
  logic signed [16:0] sn, cs;
  logic signed [32:0] p_ic, p_qs, p_qc, p_is;
  logic signed [33:0] mi_s, mq_s;
  logic signed [17:0] mx_i, mx_q;
  logic        mx_v;

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
    // stage 3: products
    v3 <= v2 && en;
    p_ic <= xi2 * cs;
    p_qs <= xq2 * sn;
    p_qc <= xq2 * cs;
    p_is <= xi2 * sn;
    // stage 4: sums + rounding
    v4 <= v3 && en;
    mi_s <= ((34'(p_ic) + 34'(p_qs)) + 34'sd4096) >>> 13;
    mq_s <= ((34'(p_qc) - 34'(p_is)) + 34'sd4096) >>> 13;
    // stage 5: saturation
    mx_v <= v4 && en;
    mx_i <= sat18(mi_s);
    mx_q <= sat18(mq_s);
  end

  // ------------------------------------------------------------ CIC
  logic signed [47:0] ii [5], iq [5];
  logic [6:0]  dcnt;
  logic        dv;                           // decimated sample in ii[4]/iq[4]

  always_ff @(posedge clk) begin
    dv <= 1'b0;
    if (!en) begin
      for (int k = 0; k < 5; k++) begin ii[k] <= '0; iq[k] <= '0; end
      dcnt <= '0;
    end else if (mx_v) begin
      ii[0] <= ii[0] + 48'(mx_i);
      iq[0] <= iq[0] + 48'(mx_q);
      for (int k = 1; k < 5; k++) begin
        ii[k] <= ii[k] + ii[k - 1];
        iq[k] <= iq[k] + iq[k - 1];
      end
      if (dcnt == R - 7'd1) begin
        dcnt <= '0;
        dv <= 1'b1;
      end else dcnt <= dcnt + 7'd1;
    end
  end

  // the decimated last integrator; the combs are shared (nbiq_comb)
  always_ff @(posedge clk) begin
    m_valid <= dv && en;
    m_i <= ii[4];
    m_q <= iq[4];
  end

endmodule
