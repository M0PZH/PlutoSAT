// CIC interpolator (x R), output scaling and NCO of the IQ transmitter
// (model: model/iqtx.py cic(), scale(), nco()).
//
//   combs        4 stages at the FIR rate, one per clock (24 bits)
//   integrators  4 stages at the output rate, 48-bit wrap-around, each with a
//                one-sample delay; input: the comb output at q = 0, else 0
//   scale        o = sat16(rnd(y, shift))
//   NCO          z = sat16(rnd(o (cos + j sin), 15)), phase advancing by finc
//                per sample from 0 (clr); 12-bit phase, quarter-wave table
//                of 1024 entries (LUT ROM, read twice per sample)
//
// An output sample ("step") is made at most every other clock, when the
// 16-entry output FIFO (first-word fall-through) has room for it and every
// sample in the pipeline, and at q = 0 only with a comb output at hand.

module iqtx_cic #(
  parameter string NCO_FILE = "iqtx_nco.mem"
) (
  input  logic               clk,
  input  logic               clr,
  input  logic [8:0]         rate,         // R, static while running
  input  logic [4:0]         shift,
  input  logic [31:0]        finc,

  input  logic               s_valid,
  output logic               s_ready,
  input  logic signed [19:0] s_i,
  input  logic signed [19:0] s_q,

  output logic               m_valid,
  input  logic               m_ready,
  output logic [31:0]        m_data        // {Q, I}
);

  localparam int W = 48;

  // ------------------------------------------------------------ combs
  logic               cbusy, cv;
  logic [1:0]         ck;
  logic signed [23:0] ci, cq, di [4], dq [4];
  logic signed [23:0] c4i, c4q;
  logic               c4v;
  logic               take;                 // the integrators take c4

  assign s_ready = !cbusy && !c4v;

  always_ff @(posedge clk) begin
    if (clr) begin
      cbusy <= 1'b0; c4v <= 1'b0; ck <= '0;
      for (int k = 0; k < 4; k++) begin di[k] <= '0; dq[k] <= '0; end
    end else begin
      if (s_valid && s_ready) begin
        cbusy <= 1'b1;
        ck <= '0;
        ci <= 24'(s_i);
        cq <= 24'(s_q);
      end else if (cbusy) begin
        ci <= ci - di[ck];
        cq <= cq - dq[ck];
        di[ck] <= ci;
        dq[ck] <= cq;
        ck <= ck + 2'd1;
        if (ck == 2'd3) begin
          cbusy <= 1'b0;
          c4v <= 1'b1;
        end
      end
      if (take) c4v <= 1'b0;
    end
  end
  assign c4i = ci;
  assign c4q = cq;

  // ------------------------------------------------------------ steps
  logic [8:0]          q;
  logic signed [W-1:0] i1, i2, i3, i4, j1, j2, j3, j4;
  logic [31:0]         phase;
  logic                last;                 // a step in the previous clock
  logic [4:0]          ocnt, inflight;
  logic                push;
  logic [31:0]         push_data;

  wire room = 6'(ocnt) + 6'(inflight) < 6'd16;
  wire step = !last && room && (q != 9'd0 || c4v);
  assign take = step && q == 9'd0;

  logic                v1, v2, v3, v4, v5, v6;
  logic signed [W-1:0] y1, yr2;
  logic signed [W-1:0] zi1, yq1, yq2;
  logic [31:0]         th1;

  always_ff @(posedge clk) begin
    if (clr) begin
      q <= '0; last <= 1'b0; phase <= '0;
      i1 <= '0; i2 <= '0; i3 <= '0; i4 <= '0;
      j1 <= '0; j2 <= '0; j3 <= '0; j4 <= '0;
      v1 <= 1'b0;
    end else begin
      last <= step;
      v1 <= step;
      if (step) begin
        y1 <= i4; yq1 <= j4; th1 <= phase;
        i4 <= i4 + i3; i3 <= i3 + i2; i2 <= i2 + i1;
        j4 <= j4 + j3; j3 <= j3 + j2; j2 <= j2 + j1;
        i1 <= i1 + (q == 9'd0 ? W'(c4i) : '0);
        j1 <= j1 + (q == 9'd0 ? W'(c4q) : '0);
        q <= (q == rate - 9'd1) ? 9'd0 : q + 9'd1;
        phase <= phase + finc;
      end
    end
  end

  // ------------------------------------------------------------ scale + NCO
  (* rom_style = "distributed" *) logic [15:0] lut [1024];
  initial $readmemh(NCO_FILE, lut);

  wire [W-1:0] half = (shift == 5'd0) ? '0 : W'(1) << (shift - 5'd1);

  logic [9:0]          ka2, kb3;
  logic [1:0]          qa2, qa3, qa4;
  logic [15:0]         la3, lb4, rom_q;
  logic signed [W-1:0] ys2, ysq2;
  logic signed [15:0]  oi3, oq3, oi4, oq4, oi5, oq5;
  logic signed [16:0]  sn5, cs5;
  logic signed [32:0]  p_ic, p_qs, p_is, p_qc;
  logic signed [15:0]  zi, zq;

  function automatic logic signed [15:0] sat16(input logic signed [W-1:0] a);
    if (a > W'(32767))  return 16'sd32767;
    if (a < -W'(32768)) return -16'sd32768;
    return a[15:0];
  endfunction

  function automatic logic signed [15:0] rs15(input logic signed [33:0] a);
    logic signed [33:0] r;
    r = (a + 34'sd16384) >>> 15;
    if (r > 34'sd32767)  return 16'sd32767;
    if (r < -34'sd32768) return -16'sd32768;
    return r[15:0];
  endfunction

  // the table is read for sample at stage 2 (sin index) or at stage 3
  // (cos index); samples are at least two clocks apart
  always_ff @(posedge clk) rom_q <= lut[v2 ? ka2 : kb3];

  always_ff @(posedge clk) begin
    // stage 2: rounding offset, table address
    v2 <= v1 && !clr;
    yr2  <= y1 + half;
    ysq2 <= yq1 + half;
    ka2 <= th1[29:20];
    qa2 <= th1[31:30];
    // stage 3: shift; sin-index entry
    v3 <= v2 && !clr;
    oi3 <= sat16(yr2 >>> shift);
    oq3 <= sat16(ysq2 >>> shift);
    kb3 <= 10'd1023 - ka2;
    qa3 <= qa2;
    // stage 4: cos-index entry (rom_q now holds lut[ka2] of this sample)
    v4 <= v3 && !clr;
    la3 <= rom_q;
    oi4 <= oi3; oq4 <= oq3; qa4 <= qa3;
    // stage 5: sin / cos by quadrant
    v5 <= v4 && !clr;
    lb4 <= rom_q;
    oi5 <= oi4; oq5 <= oq4;
    case (qa4)
      2'd0: begin sn5 <=  17'(la3);   cs5 <=  17'(rom_q); end
      2'd1: begin sn5 <=  17'(rom_q); cs5 <= -17'(la3);   end
      2'd2: begin sn5 <= -17'(la3);   cs5 <= -17'(rom_q); end
      default: begin sn5 <= -17'(rom_q); cs5 <= 17'(la3); end
    endcase
    // stage 6: products
    v6 <= v5 && !clr;
    p_ic <= oi5 * cs5;
    p_qs <= oq5 * sn5;
    p_is <= oi5 * sn5;
    p_qc <= oq5 * cs5;
    // stage 7: sums, round, saturate -> FIFO
    push <= v6 && !clr;
    push_data <= {rs15(34'(p_is) + 34'(p_qc)), rs15(34'(p_ic) - 34'(p_qs))};
  end

  // ------------------------------------------------------------ output FIFO
  (* ram_style = "distributed" *) logic [31:0] of [16];
  logic [3:0] owp, orp;
  assign m_valid = ocnt != 5'd0;
  assign m_data  = of[orp];
  always_ff @(posedge clk) if (push) of[owp] <= push_data;
  always_ff @(posedge clk) begin
    if (clr) begin
      ocnt <= '0; owp <= '0; orp <= '0; inflight <= '0;
    end else begin
      if (push) owp <= owp + 4'd1;
      if (m_valid && m_ready) orp <= orp + 4'd1;
      ocnt <= ocnt + 5'(push) - 5'(m_valid && m_ready);
      inflight <= inflight + 5'(step) - 5'(push);
    end
  end

endmodule
