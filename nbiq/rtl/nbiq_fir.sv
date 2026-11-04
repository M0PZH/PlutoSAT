// Shared decimate-by-2 FIR for the narrowband channels (model: model/nbiq.py
// fir()). Real 18-bit coefficients h[0..2*npairs-1] per channel; one tap
// pair per clock:
//
//   y[m] = sum_{j<npairs} h[2j] xo[m-j] + h[2j+1] xe[m-j],
//   xe[k] = x[2k], xo[k] = x[2k+1]  (x: the channel's CIC outputs)
//
// Jobs. Job m of a channel (output y[m]) becomes ready when xo[m] is
// written. Ready jobs go through one FIFO of channel numbers (JQ entries) in
// the order they became ready, and run back to back, npairs clocks each
// (oldest first). A channel's jobs are consecutive, so the FIFO entry only
// names the channel; per channel, jn counts its ready jobs and hm is the
// pair index m of the oldest. An entry left from before a disable finds
// jn = 0 and is dropped (one idle clock); so is a job m < npairs - 1 (its
// output is not produced: it would use samples from before the enable).
// A job that finds the FIFO full is counted in st_drop and runs with the
// channel's next job instead: the outputs stay complete but come one output
// period later (the driver's limits below make this impossible).
//
// Bounds. Let P_c = npairs of channel c, T_c its output period in clocks
// (2 R_c clk / fs) and S = sum_c P_c. If sum_c (P_c + 1) / T_c <= 1 (the
// driver keeps it below 0.9), the work ready at any time is at most S
// clocks, so a job waits less than S - P_c clocks and is done less than S
// clocks after it became ready (oldest first). Hence:
//   - FIFO: at most sum_c (floor((S - P_c + 2) / T_c) + 1) entries; must be
//     <= JQ = 32.
//   - Sample buffers: 128 pairs per channel. Job m reads pairs m .. m - P_c
//     + 1 (the oldest last); pair m - P_c + 1 + 128 overwrites the oldest,
//     its xe half (128.5 - P_c) T_c clocks after xo[m]. Safe when
//     S <= (128 - P_c) T_c (the remaining T_c / 2 >= 16 clocks cover the
//     pipeline).
// With the driver's per-channel share fout (npairs + 1) <= 0.9 clk / k
// (k channels in use) both hold for every rate mix (at most 27 entries).

module nbiq_fir #(
  parameter int NCH = 5
) (
  input  logic               clk,
  input  logic [NCH-1:0]     en,

  input  logic [6:0]         npairs [NCH],   // 1..127
  input  logic [5:0]         oshift [NCH],

  input  logic               coef_we,
  input  logic [2:0]         coef_ch,
  input  logic [7:0]         coef_idx,       // tap 0..253
  input  logic signed [17:0] coef_data,

  input  logic               s_valid,       // CIC outputs, one channel at a time
  input  logic [2:0]         s_ch,
  input  logic signed [23:0] s_i,
  input  logic signed [23:0] s_q,

  output logic               m_valid,
  output logic [2:0]         m_ch,
  output logic signed [15:0] m_i,
  output logic signed [15:0] m_q,

  output logic [15:0]        st_drop [NCH]
);

  localparam int JQ = 32;

  // ------------------------------------------------------------ memories
  // samples {q, i} of 128 pairs per channel, split 36 + 12 bits (1K x 36 in
  // a RAMB36, 1K x 12 in a RAMB18)
  logic [47:0] xo_q, xe_q;
  logic signed [17:0] he_q, ho_q;
  logic [9:0]  i_xa, i_ha;

  wire        w_en  = s_valid && en[s_ch];
  logic [7:0] n [NCH];                       // CIC outputs since enable (mod 256)
  wire        w_odd = n[s_ch][0];
  wire [9:0]  w_a   = {s_ch, n[s_ch][7:1]};
  wire [47:0] w_d   = {s_q, s_i};

  nbiq_sdpram #(.W(36), .AW(10)) u_xe0 (.clk, .we(w_en && !w_odd), .wa(w_a), .wd(w_d[35:0]),  .ra(i_xa), .rq(xe_q[35:0]));
  nbiq_sdpram #(.W(12), .AW(10)) u_xe1 (.clk, .we(w_en && !w_odd), .wa(w_a), .wd(w_d[47:36]), .ra(i_xa), .rq(xe_q[47:36]));
  nbiq_sdpram #(.W(36), .AW(10)) u_xo0 (.clk, .we(w_en && w_odd),  .wa(w_a), .wd(w_d[35:0]),  .ra(i_xa), .rq(xo_q[35:0]));
  nbiq_sdpram #(.W(12), .AW(10)) u_xo1 (.clk, .we(w_en && w_odd),  .wa(w_a), .wd(w_d[47:36]), .ra(i_xa), .rq(xo_q[47:36]));
  nbiq_sdpram #(.W(18), .AW(10)) u_he (.clk, .we(coef_we && !coef_idx[0]), .wa({coef_ch, coef_idx[7:1]}),
                                       .wd(coef_data), .ra(i_ha), .rq(he_q));
  nbiq_sdpram #(.W(18), .AW(10)) u_ho (.clk, .we(coef_we && coef_idx[0]), .wa({coef_ch, coef_idx[7:1]}),
                                       .wd(coef_data), .ra(i_ha), .rq(ho_q));

  // ------------------------------------------------------------ job FIFO
  (* ram_style = "distributed" *) logic [2:0] jq [JQ];
  logic [4:0]     jwp, jrp;
  logic [5:0]     jcnt;
  logic [5:0]     jn [NCH];                  // ready jobs per channel
  logic [6:0]     hm [NCH];                  // pair index of the oldest (mod 128)
  logic [6:0]     hpc [NCH];                 // the same, saturating at 127
  logic [1:0]     gen [NCH];
  logic [NCH-1:0] en_q;

  wire        jset  = w_en && w_odd;         // job n[s_ch][7:1] ready
  wire [2:0]  hc    = jq[jrp];               // channel of the oldest entry

  logic       busy;
  logic [6:0] ej, enp;
  wire        jpop  = jcnt != 0 && (!busy || ej == enp - 7'd1);
  wire        hlive = jn[hc] != 6'd0;        // else: left from before a disable
  wire        hskip = 8'(hpc[hc]) + 8'd1 < 8'(npairs[hc]);   // m < npairs - 1
  wire        start = jpop && hlive && !hskip && en[hc];
  wire        jpush = jset && (jcnt != 6'(JQ) || jpop);
  logic [NCH-1:0] jtake;                     // a ready job of channel c leaves the FIFO
  always_comb
    for (int c = 0; c < NCH; c++) jtake[c] = jpop && hlive && hc == 3'(c);

  always_ff @(posedge clk) begin
    en_q <= en;
    if (jpush) begin
      jq[jwp] <= s_ch;
      jwp <= jwp + 5'd1;
    end
    if (jpop) jrp <= jrp + 5'd1;
    jcnt <= jcnt + 6'(jpush) - 6'(jpop);
    for (int c = 0; c < NCH; c++) begin
      if (!en[c]) begin
        n[c] <= '0; jn[c] <= '0; hm[c] <= '0; hpc[c] <= '0;
      end else begin
        if (w_en && s_ch == 3'(c)) n[c] <= n[c] + 8'd1;
        jn[c] <= jn[c] + 6'(jpush && s_ch == 3'(c)) - 6'(jtake[c]);
        if (jtake[c]) begin
          hm[c] <= hm[c] + 7'd1;
          if (hpc[c] != 7'd127) hpc[c] <= hpc[c] + 7'd1;
        end
        if (jset && !jpush && s_ch == 3'(c)) st_drop[c] <= st_drop[c] + 16'd1;
      end
      if (en[c] && !en_q[c]) gen[c] <= gen[c] + 2'd1;
    end
  end

  // ------------------------------------------------------------ MAC engine
  logic [2:0]  ec;
  logic [6:0]  em;
  logic [1:0]  egen;

  // issue stage outputs
  logic        i_v, i_first, i_last;
  logic [2:0]  i_ch;
  logic [1:0]  i_gen;

  always_ff @(posedge clk) begin
    i_v <= 1'b0;
    if (start) begin
      busy <= 1'b1;
      ec <= hc;
      ej <= '0; em <= hm[hc]; enp <= npairs[hc];
      egen <= gen[hc];
    end else if (busy) begin
      if (ej == enp - 7'd1) busy <= 1'b0;
      else ej <= ej + 7'd1;
    end
    if (busy) begin
      i_v <= 1'b1;
      i_first <= ej == 7'd0;
      i_last <= ej == enp - 7'd1;
      i_ch <= ec; i_gen <= egen;
      i_xa <= {ec, 7'(em - ej)};
      i_ha <= {ec, ej};
    end
  end

  // read
  logic        r_v, r_first, r_last;
  logic [2:0]  r_ch;
  logic [1:0]  r_gen;
  always_ff @(posedge clk) begin
    r_v <= i_v; r_first <= i_first; r_last <= i_last;
    r_ch <= i_ch; r_gen <= i_gen;
  end

  // second register (the block RAMs keep their output register; this one
  // may become the DSP input register)
  logic        q_v, q_first, q_last;
  logic [2:0]  q_ch;
  logic [1:0]  q_gen;
  logic [47:0] xo_r, xe_r;
  logic signed [17:0] he_r, ho_r;
  always_ff @(posedge clk) begin
    q_v <= r_v; q_first <= r_first; q_last <= r_last;
    q_ch <= r_ch; q_gen <= r_gen;
    xo_r <= xo_q; xe_r <= xe_q; he_r <= he_q; ho_r <= ho_q;
  end

  // multiply
  logic        p_v, p_first, p_last;
  logic [2:0]  p_ch;
  logic [1:0]  p_gen;
  logic signed [41:0] pai, paq, pbi, pbq;
  always_ff @(posedge clk) begin
    p_v <= q_v; p_first <= q_first; p_last <= q_last;
    p_ch <= q_ch; p_gen <= q_gen;
    pai <= $signed(xo_r[23:0])  * he_r;
    paq <= $signed(xo_r[47:24]) * he_r;
    pbi <= $signed(xe_r[23:0])  * ho_r;
    pbq <= $signed(xe_r[47:24]) * ho_r;
  end

  // pair sum
  logic        u_v, u_first, u_last;
  logic [2:0]  u_ch;
  logic [1:0]  u_gen;
  logic signed [42:0] ui, uq;
  always_ff @(posedge clk) begin
    u_v <= p_v; u_first <= p_first; u_last <= p_last;
    u_ch <= p_ch; u_gen <= p_gen;
    ui <= 43'(pai) + 43'(pbi);
    uq <= 43'(paq) + 43'(pbq);
  end

  // accumulate
  logic        a_v;
  logic [2:0]  a_ch;
  logic [1:0]  a_gen;
  logic [5:0]  a_osh;
  logic signed [47:0] acc_i, acc_q;
  always_ff @(posedge clk) begin
    a_v <= 1'b0;
    if (u_v) begin
      acc_i <= (u_first ? 48'sd0 : acc_i) + 48'(ui);
      acc_q <= (u_first ? 48'sd0 : acc_q) + 48'(uq);
      if (u_last) begin
        a_v <= 1'b1;
        a_ch <= u_ch; a_gen <= u_gen;
        a_osh <= oshift[u_ch];
      end
    end
  end

  // round and shift, then saturate
  function automatic logic signed [48:0] rsh(input logic signed [47:0] v, input logic [5:0] sh);
    return (sh == 0) ? 49'(v) : (49'(v) + (49'sd1 <<< (sh - 6'd1))) >>> sh;
  endfunction
  function automatic logic signed [15:0] sat16(input logic signed [48:0] r);
    if (r > 49'sd32767) return 16'sd32767;
    if (r < -49'sd32768) return -16'sd32768;
    return 16'(r);
  endfunction

  logic        o_v;
  logic [2:0]  o_ch;
  logic signed [48:0] o_i, o_q;
  always_ff @(posedge clk) begin
    o_v <= a_v && en[a_ch] && a_gen == gen[a_ch];
    o_ch <= a_ch;
    o_i <= rsh(acc_i, a_osh);
    o_q <= rsh(acc_q, a_osh);
    m_valid <= o_v && en[o_ch];
    m_ch <= o_ch;
    m_i <= sat16(o_i);
    m_q <= sat16(o_q);
  end

endmodule
