// Complex NCO mixer: y = x * exp(j*phase), phase advancing by finc per sample.
// 32-bit phase accumulator, 1024-entry sin/cos ROM (Q1.15), result rounded
// by 2^15 and saturated. bypass passes samples through with the same latency.

module dvbs2_nco #(
  parameter string LUT_FILE = "nco_lut.mem",
  parameter int    LUT_AW   = 10
) (
  input  logic               clk,
  input  logic               rst,
  input  logic [31:0]        finc,
  input  logic               bypass,

  input  logic               s_valid,
  output logic               s_ready,
  input  logic signed [15:0] s_i,
  input  logic signed [15:0] s_q,

  output logic               m_valid,
  input  logic               m_ready,
  output logic signed [15:0] m_i,
  output logic signed [15:0] m_q
);

  (* rom_style = "block" *) logic [31:0] lut [1 << LUT_AW];   // {sin, cos}
  initial $readmemh(LUT_FILE, lut);

  // stall-able fixed-latency pipeline: all stages move when ce is high
  wire ce = !m_valid || m_ready;
  assign s_ready = ce;

  logic [31:0] phase;

  logic               v1, v2, v3;
  logic signed [15:0] i1, q1, i2, q2;
  logic [31:0]        cs2;
  logic signed [32:0] re3, im3;
  logic signed [15:0] bi3, bq3;
  logic               byp1, byp2, byp3;

  function automatic logic signed [15:0] rs15(input logic signed [32:0] a);
    logic signed [32:0] r;
    r = (a + 33'sd16384) >>> 15;
    if (r > 32767)       rs15 = 16'sd32767;
    else if (r < -32768) rs15 = -16'sd32768;
    else                 rs15 = r[15:0];
  endfunction

  logic [LUT_AW-1:0] lut_addr;
  always_ff @(posedge clk) if (ce) cs2 <= lut[lut_addr];

  always_ff @(posedge clk) begin
    if (rst) begin
      phase <= '0;
      v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; m_valid <= 1'b0;
    end else if (ce) begin
      // stage 1: take the sample, address the LUT with the current phase
      v1 <= s_valid;
      if (s_valid) begin
        i1 <= s_i; q1 <= s_q; byp1 <= bypass;
        lut_addr <= phase[31 -: LUT_AW];
        phase <= phase + finc;
      end
      // stage 2: LUT data valid
      v2 <= v1; i2 <= i1; q2 <= q1; byp2 <= byp1;
      // stage 3: complex multiply
      v3 <= v2; byp3 <= byp2; bi3 <= i2; bq3 <= q2;
      re3 <= i2 * $signed(cs2[15:0]) - q2 * $signed(cs2[31:16]);
      im3 <= i2 * $signed(cs2[31:16]) + q2 * $signed(cs2[15:0]);
      // stage 4: round / saturate
      m_valid <= v3;
      m_i <= byp3 ? bi3 : rs15(re3);
      m_q <= byp3 ? bq3 : rs15(im3);
    end
  end

endmodule
