// DDC for the DVB-S2 receiver (model: model/rxfixed.py, ddc()): NCO mix,
// an optional 3-stage CIC, then a decimating FIR; ADC rate -> 2 samples/symbol.
//
//   m[n] = sat16(round(x[n] * exp(j*2*pi*n*finc/2^32)))   (bypassed when finc = 0)
//   CIC (cic_r = R >= 2): 3 integrators at the input rate (integrator k > 0
//   adds the previous value of integrator k - 1), decimation by R (the value
//   after input R*j + R-1), 3 combs, 32-bit wrap-around arithmetic, then
//        c[j] = sat18(round(comb / 2^cic_shift));   CIC off (R = 0, 1): c = m
//   y[k] = sat16(round(sum_{i<ntaps} h[i] c[k*dec + dec-1 - i] / 2^shift))
//          (samples before the first one are zero; total decimation R * dec)
//
// Input: at most one sample per 2 clocks (s_ready is low in the clock after
// a sample is taken); the NCO, the CIC and the output scaling take I and Q
// in consecutive clocks through one datapath. 18-bit samples go into a
// 1024-sample buffer split into even and odd samples (two 512 x 36 RAMs),
// so taps 2m and 2m+1 are read in the same clock: an output takes ntaps/2
// clocks, outputs are computed back to back, so the input rate may be up to
// min(clk / 2, clk * R * dec / (ntaps/2)). m_* has no backpressure; an
// output request while two are pending is dropped and counted.
//
// DSPs: NCO 2, CIC integrators 6, FIR 6 (per axis: tap 2m, tap 2m+1 and
// the pair sum, the accumulator starting from the rounding constant).
// The configuration inputs are static (registered here; change them in
// reset).
// R <= 32 keeps the CIC output within 31 bits (R^3 * 2^15 <= 2^30), so the
// rounding constant can be added in 32 bits.

module dvbs2_rx_ddc #(
  parameter string LUT_FILE = "nco_lut.mem"
) (
  input  logic               clk,
  input  logic               rst,

  input  logic [31:0]        finc,
  input  logic [5:0]         dec,           // FIR decimation, 1..63
  input  logic [9:0]         ntaps,         // even, 2..512
  input  logic [4:0]         shift,
  input  logic [5:0]         cic_r,         // 0, 1: no CIC; 2..32
  input  logic [3:0]         cic_shift,
  input  logic               coef_we,
  input  logic [8:0]         coef_addr,
  input  logic signed [17:0] coef_data,

  input  logic               s_valid,
  output logic               s_ready,
  input  logic signed [15:0] s_i,
  input  logic signed [15:0] s_q,

  output logic               m_valid,
  output logic signed [15:0] m_i,
  output logic signed [15:0] m_q,
  output logic [15:0]        st_dropped
);

  localparam int CW = 32;                   // CIC width (the output and its rounding fit 32 bits)
  localparam int AW = 46;                   // FIR accumulator: 512 * 2^17 * 2^17 + rounding < 2^45

  // static configuration, registered
  logic        byp, cic_on;
  logic [5:0]  cic_rm1;
  logic [14:0] cic_rc;                      // CIC rounding constant
  logic [13:0] cic_mask;                    // bits 17 + j of the CIC result, j >= cic_shift
  logic [30:0] fir_rc;                      // FIR rounding constant
  logic [29:0] sat_mask;                    // bits 15 + j of the accumulator, j >= shift
  logic [7:0]  npairs_m1;
  always_ff @(posedge clk) begin
    byp      <= finc == 0;
    cic_on   <= cic_r >= 6'd2;
    cic_rm1  <= cic_r - 6'd1;
    // one 4- or 5-input function per bit
    for (int j = 0; j < 15; j++) cic_rc[j] <= cic_shift == 4'(j + 1);
    for (int j = 0; j < 14; j++) cic_mask[j] <= 4'(j) >= cic_shift;
    for (int j = 0; j < 31; j++) fir_rc[j] <= shift == 5'(j + 1);
    for (int j = 0; j < 30; j++) sat_mask[j] <= 5'(j) >= shift;
    npairs_m1 <= 8'((ntaps >> 1) - 10'd1);
  end

  // ------------------------------------------------------------ NCO mix
  // I = x_i c - x_q s in the first clock, Q = x_q c + x_i s in the next:
  // multiplier 1 takes x_i, x_q times c, multiplier 2 x_q, x_i times s.
  // Bypass: c = 2^15, s = 0 (exact).
  (* rom_style = "block" *) logic [31:0] lut [1024];
  initial $readmemh(LUT_FILE, lut);

  logic        tk, v0, vc, va, vm, vp, vo;
  logic        fa, fm, fp, fo;               // 0: I, 1: Q
  logic [31:0] ph, cs;
  logic [9:0]  la;
  logic signed [15:0] xi0, xq0, xi1, xq1;
  logic signed [15:0] c1, s2, s2d;          // cos, sin
  logic signed [16:0] d1;                    // 1 in bypass: cos(0) + 1 = 2^15
  logic signed [15:0] x1;
  logic signed [16:0] x2, x2d;
  logic signed [33:0] m1, m2;
  logic signed [35:0] p1, p2;

  assign s_ready = !rst && !tk;
  wire take = s_valid && s_ready;

  always_ff @(posedge clk) begin
    if (take) begin xi0 <= s_i; xq0 <= s_q; la <= ph[31:22]; end
    cs <= lut[la];
    if (v0) begin xi1 <= xi0; xq1 <= xq0; end
    if (vc) begin
      c1 <= $signed(cs[15:0]);
      s2 <= $signed(cs[31:16]);
      d1 <= 17'(byp);                        // (the phase stays 0: cos 32767, sin 0)
      x1 <= xi1;
      x2 <= 17'(xq1);
    end else if (fa == 1'b0 && va) begin
      x1 <= xq1;
      x2 <= 17'(xi1);
    end
    x2d <= x2; s2d <= s2;
    m1 <= (17'(c1) + d1) * x1;
    m2 <= s2d * x2d;
    p1 <= m1 + 36'sd16384;
    p2 <= fp ? p1 + 36'(m2) : p1 - 36'(m2);    // I: x_i c - x_q s, Q: x_q c + x_i s
    fm <= fa; fp <= fm; fo <= fp;
    if (rst) begin
      tk <= 1'b0; ph <= '0;
      v0 <= 1'b0; vc <= 1'b0; va <= 1'b0; vm <= 1'b0; vp <= 1'b0; vo <= 1'b0;
    end else begin
      tk <= take;
      if (take) ph <= ph + finc;
      v0 <= take;
      vc <= v0;
      va <= vc || (va && !fa);               // I, then Q
      fa <= va && !fa;
      vm <= va; vp <= vm; vo <= vp;
    end
  end

  // rounded product sum -> sat16 (p2 = I or Q * 2^15 + 2^14)
  logic               mx_v, mx_f;
  logic signed [15:0] mx_d;
  always_ff @(posedge clk) begin
    mx_v <= vo && !rst;
    mx_f <= fo;
    mx_d <= (p2[35:30] == '0 || p2[35:30] == '1) ? p2[30:15] : (p2[35] ? -16'sd32768 : 16'sd32767);
  end

  // ------------------------------------------------------------ CIC
  logic               cvi, cvq;
  logic signed [15:0] cx;
  (* use_dsp = "yes" *) logic signed [CW-1:0] i0, i1, i2, q0, q1, q2;
  logic [5:0]         rcnt;
  logic               dI, dQ;

  always_ff @(posedge clk) begin
    cx  <= mx_d;
    cvi <= mx_v && !mx_f && cic_on && !rst;
    cvq <= mx_v && mx_f && cic_on && !rst;
    dQ  <= dI;
    if (rst) begin
      i0 <= '0; i1 <= '0; i2 <= '0; q0 <= '0; q1 <= '0; q2 <= '0;
      rcnt <= '0; dI <= 1'b0; dQ <= 1'b0;
    end else begin
      if (cvi) begin
        i0 <= i0 + CW'(cx); i1 <= i1 + i0; i2 <= i2 + i1;
        rcnt <= rcnt == cic_rm1 ? 6'd0 : rcnt + 6'd1;
      end
      if (cvq) begin
        q0 <= q0 + CW'(cx); q1 <= q1 + q0; q2 <= q2 + q1;
      end
      dI <= cvi && rcnt == cic_rm1;          // i2 holds a decimated value next clock, q2 the clock after
    end
  end

  // combs: I and Q in consecutive clocks; stages 2 and 3 keep the previous I
  // and Q input in a pair of registers that rotates on every input
  logic                 kv [4];
  logic                 kf [4];
  logic signed [CW-1:0] kd [4];
  logic signed [CW-1:0] da [3], db [3];
  logic signed [CW-1:0] pi2, pq2;            // stage 0: the previous decimated i2, q2

  always_ff @(posedge clk) begin
    kv[1] <= (dI || dQ) && !rst;
    kf[1] <= dQ;
    kd[1] <= (dQ ? q2 : i2) - (dQ ? pq2 : pi2);
    if (rst) begin
      pi2 <= '0; pq2 <= '0;
    end else begin
      if (dI) pi2 <= i2;
      if (dQ) pq2 <= q2;
    end
    for (int k = 1; k < 3; k++) begin
      kv[k + 1] <= kv[k] && !rst;
      kf[k + 1] <= kf[k];
      kd[k + 1] <= kd[k] - da[k];
      if (rst) begin
        da[k] <= '0; db[k] <= '0;
      end else if (kv[k]) begin
        da[k] <= db[k]; db[k] <= kd[k];
      end
    end
  end

  // shift (by cic_shift[1:0], then [3:2]) and saturate to 18 bits
  logic               c0_v, c0_f, c1_v, c1_f, c1_ov, c1_s, co_v, co_f;
  logic signed [CW-1:0] c0_x;
  logic [29:0]        c1_x;
  logic signed [17:0] co_d;
  always_ff @(posedge clk) begin
    c0_v  <= kv[3] && !rst;
    c0_f  <= kf[3];
    c0_x  <= kd[3] + CW'(cic_rc);
    c1_v  <= c0_v && !rst;
    c1_f  <= c0_f;
    c1_x  <= 30'(c0_x >>> cic_shift[1:0]);
    c1_s  <= c0_x[CW-1];
    c1_ov <= |((c0_x[30:17] ^ {14{c0_x[CW-1]}}) & cic_mask);
    co_v  <= c1_v && !rst;
    co_f  <= c1_f;
    co_d  <= c1_ov ? (c1_s ? -18'sd131072 : 18'sd131071) : 18'(c1_x >> {cic_shift[3:2], 2'b00});
  end

  // ------------------------------------------------------------ sample buffer
  wire               w_v = cic_on ? co_v : mx_v;
  wire               w_f = cic_on ? co_f : mx_f;
  wire signed [17:0] w_d = cic_on ? co_d : 18'(mx_d);

  (* ram_style = "block" *) logic [35:0] be [512], bo [512];   // even, odd samples {q, i}
  (* ram_style = "block" *) logic [35:0] hc [256];             // {h[2m+1], h[2m]}
  logic [35:0]  be_q, bo_q, hc_q;
  logic [17:0]  wi;
  logic [9:0]   n;                           // next sample index
  logic [10:0]  nw;                          // samples written, up to 1024
  logic [5:0]   dcnt;
  logic         trig;
  logic [9:0]   t_base;
  logic [10:0]  t_rem;

  wire [35:0] cd36 = {coef_data, coef_data};
  always_ff @(posedge clk) begin
    if (coef_we)
      for (int b = 0; b < 4; b++)
        if ((b >= 2) == coef_addr[0]) hc[coef_addr[8:1]][b * 9 +: 9] <= cd36[b * 9 +: 9];
  end

  always_ff @(posedge clk) begin
    trig <= 1'b0;
    if (w_v && !w_f) wi <= w_d;
    if (w_v && w_f) begin
      if (n[0]) bo[n[9:1]] <= {w_d, wi};
      else      be[n[9:1]] <= {w_d, wi};
    end
    if (rst) begin
      n <= '0; nw <= '0; dcnt <= '0;
    end else if (w_v && w_f) begin
      n <= n + 10'd1;
      if (!nw[10]) nw <= nw + 11'd1;
      t_base <= n;
      t_rem  <= nw[10] ? nw : nw + 11'd1;
      if (dcnt + 6'd1 >= dec) begin
        dcnt <= '0;
        trig <= 1'b1;
      end else dcnt <= dcnt + 6'd1;
    end
  end

  // ------------------------------------------------------------ MAC engine
  // pending outputs: {newest sample index, valid samples up to it}
  logic [20:0] pend [2];
  logic [1:0]  npend;
  logic        busy;
  logic [7:0]  mi;                           // tap pair index
  logic [8:0]  ea, oa;                       // bank addresses of taps 2m, 2m+1 (swapped for an odd base)
  logic        par;                          // newest sample index odd
  logic signed [11:0] rem;                   // valid samples from tap 2m back
  logic        r_v, r_first, r_last, r_oka, r_okb, r_par;
  logic        x_v, x_first, x_last, x_oka, x_okb, x_par;
  logic [35:0] be_q2, bo_q2, hc_q2;

  always_ff @(posedge clk) begin
    if (rst) begin
      busy <= 1'b0;
      npend <= '0;
      r_v <= 1'b0;
      st_dropped <= '0;
    end else begin : eng
      logic [1:0]  np;
      logic [20:0] p0, p1, nb;
      logic        start;
      np = npend; p0 = pend[0]; p1 = pend[1];
      if (trig) begin
        if (np == 2'd2) st_dropped <= st_dropped + 16'd1;
        else if (np == 2'd1) begin p1 = {t_base, t_rem}; np = 2'd2; end
        else begin p0 = {t_base, t_rem}; np = 2'd1; end
      end
      r_v <= busy;
      r_first <= mi == 0;
      r_last  <= mi == npairs_m1;
      r_oka <= rem > 0;
      r_okb <= rem > 1;
      r_par <= par;
      start = 1'b0;
      if (busy) begin
        ea <= ea - 9'd1;
        oa <= oa - 9'd1;
        rem <= rem - 12'sd2;
        if (mi != npairs_m1) mi <= mi + 8'd1;
        else if (np != 0) start = 1'b1;      // next output back to back
        else busy <= 1'b0;
      end else if (np != 0) begin
        busy <= 1'b1;
        start = 1'b1;
      end
      if (start) begin
        nb = p0;
        ea  <= nb[20:12];
        oa  <= nb[20:12] - {8'd0, !nb[11]};
        par <= nb[11];
        rem <= 12'(nb[10:0]);
        mi  <= '0;
        p0 = p1;
        np = np - 2'd1;
      end
      npend <= np; pend[0] <= p0; pend[1] <= p1;
    end
  end

  always_ff @(posedge clk) begin
    be_q <= be[ea];
    bo_q <= bo[oa];
    hc_q <= hc[mi];
    be_q2 <= be_q; bo_q2 <= bo_q; hc_q2 <= hc_q;
    x_v <= r_v && !rst;
    x_first <= r_first; x_last <= r_last; x_oka <= r_oka; x_okb <= r_okb; x_par <= r_par;
  end

  // even bank: tap 2m (even base) or 2m+1 (odd base); odd bank the other one
  logic        k1_v, k1_f, k1_l, k2_v, k2_f, k2_l, k3_v, k3_f, k3_l, k4_v, k4_f, k4_l, a_v;
  logic signed [17:0] aei, aeq, aoi1, aoq1, aoi2, aoq2, be_c, bo1_c, bo2_c;
  logic signed [35:0] mei, meq, moi, moq;
  (* use_dsp = "yes" *) logic signed [AW-1:0] pei, peq, si, sq, acc_i, acc_q;

  always_ff @(posedge clk) begin
    // k1: operands (a sample before the first one gets a zero coefficient)
    aei <= be_q2[17:0]; aeq <= be_q2[35:18];
    aoi1 <= bo_q2[17:0]; aoq1 <= bo_q2[35:18];
    be_c  <= (x_par ? x_okb : x_oka) ? (x_par ? hc_q2[35:18] : hc_q2[17:0]) : 18'sd0;
    bo1_c <= (x_par ? x_oka : x_okb) ? (x_par ? hc_q2[17:0] : hc_q2[35:18]) : 18'sd0;
    k1_v <= x_v && !rst; k1_f <= x_first; k1_l <= x_last;
    // k2: even-bank products, odd-bank operands delayed
    mei <= aei * be_c;
    meq <= aeq * be_c;
    aoi2 <= aoi1; aoq2 <= aoq1; bo2_c <= bo1_c;
    k2_v <= k1_v && !rst; k2_f <= k1_f; k2_l <= k1_l;
    // k3: odd-bank products
    pei <= AW'(mei);
    peq <= AW'(meq);
    moi <= aoi2 * bo2_c;
    moq <= aoq2 * bo2_c;
    k3_v <= k2_v && !rst; k3_f <= k2_f; k3_l <= k2_l;
    // k4: pair sum
    si <= pei + AW'(moi);
    sq <= peq + AW'(moq);
    k4_v <= k3_v && !rst; k4_f <= k3_f; k4_l <= k3_l;
    // k5: accumulate, starting from the rounding constant
    if (k4_v) begin
      acc_i <= (k4_f ? AW'(fir_rc) : acc_i) + si;
      acc_q <= (k4_f ? AW'(fir_rc) : acc_q) + sq;
    end
    a_v <= k4_v && k4_l && !rst;
  end

  // ------------------------------------------------------------ output: shift, saturate
  // I and Q in consecutive clocks (outputs are >= 2 clocks apart):
  // o1 shift by shift[1:0] + overflow test, o2 by shift[3:2], o3 by shift[4]
  logic          a_vd, o0_v, o0_f, o1_v, o1_f, o2_v, o2_f, o3_v, o3_f;
  logic signed [AW-1:0] o0_x, hq;
  logic [43:0]   o1_x;
  logic [31:0]   o2_x;
  logic          o1_ov, o1_s, o2_ov, o2_s;
  logic signed [15:0] o3_d, oi;

  always_ff @(posedge clk) begin
    a_vd <= a_v && !rst;
    if (a_v) hq <= acc_q;
    o0_v <= (a_v || a_vd) && !rst;
    o0_f <= a_vd;
    o0_x <= a_v ? acc_i : hq;
    // o1
    o1_v <= o0_v && !rst; o1_f <= o0_f;
    o1_x <= 44'(o0_x >>> shift[1:0]);
    o1_s <= o0_x[AW-1];
    o1_ov <= |((o0_x[44:15] ^ {30{o0_x[AW-1]}}) & sat_mask);
    // o2
    o2_v <= o1_v && !rst; o2_f <= o1_f; o2_s <= o1_s; o2_ov <= o1_ov;
    o2_x <= 32'(o1_x >> {shift[3:2], 2'b00});
    // o3
    o3_v <= o2_v && !rst; o3_f <= o2_f;
    o3_d <= o2_ov ? (o2_s ? -16'sd32768 : 16'sd32767) : (shift[4] ? o2_x[31:16] : o2_x[15:0]);
    // output
    m_valid <= o3_v && o3_f && !rst;
    if (o3_v && !o3_f) oi <= o3_d;
    if (o3_v && o3_f) begin m_i <= oi; m_q <= o3_d; end
  end

endmodule
