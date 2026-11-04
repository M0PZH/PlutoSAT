// Shared decimate-by-2 FIR for the narrowband channels (model: model/nbiq.py
// fir()). Real 18-bit coefficients h[0..2*npairs-1] per channel; one tap
// pair per clock:
//
//   y[m] = sum_{j<npairs} h[2j] xo[m-j] + h[2j+1] xe[m-j],
//   xe[k] = x[2k], xo[k] = x[2k+1]  (x: the channel's CIC outputs)
//
// A job (one output) is queued when xo[m] is written (up to 4 per channel);
// channels are served round-robin, jobs back to back. A job that finds its
// channel's queue full is dropped and counted (overload: the sum over the
// channels of fout * (npairs + 1) must stay below the clock). A queued job
// reads samples up to 128 pairs back, so npairs + (jobs queued behind it)
// must stay below 128: true when fout * (npairs + 1) <= clk / 4 per channel.
// Outputs m < npairs - 1 after enabling a channel are not produced.

module nbiq_fir #(
  parameter int NCH = 4
) (
  input  logic               clk,
  input  logic [NCH-1:0]     en,

  input  logic [6:0]         npairs [NCH],   // 1..127
  input  logic [5:0]         oshift [NCH],

  input  logic               coef_we,
  input  logic [1:0]         coef_ch,
  input  logic [7:0]         coef_idx,       // tap 0..253
  input  logic signed [17:0] coef_data,

  input  logic               s_valid,       // CIC outputs, one channel at a time
  input  logic [1:0]         s_ch,
  input  logic signed [23:0] s_i,
  input  logic signed [23:0] s_q,

  output logic               m_valid,
  output logic [1:0]         m_ch,
  output logic signed [15:0] m_i,
  output logic signed [15:0] m_q,

  output logic [15:0]        st_drop [NCH]
);

  // ------------------------------------------------------------ memories
  logic [47:0] xo_q, xe_q;
  logic signed [17:0] he_q, ho_q;
  logic        w_en_x, w_odd_x;
  logic [8:0]  w_a_x, i_xa, i_ha;
  logic [47:0] w_d_x;

  nbiq_sdpram #(.W(48), .AW(9)) u_xe (.clk, .we(w_en_x && !w_odd_x), .wa(w_a_x), .wd(w_d_x), .ra(i_xa), .rq(xe_q));
  nbiq_sdpram #(.W(48), .AW(9)) u_xo (.clk, .we(w_en_x && w_odd_x),  .wa(w_a_x), .wd(w_d_x), .ra(i_xa), .rq(xo_q));
  nbiq_sdpram #(.W(18), .AW(9)) u_he (.clk, .we(coef_we && !coef_idx[0]), .wa({coef_ch, coef_idx[7:1]}),
                                      .wd(coef_data), .ra(i_ha), .rq(he_q));
  nbiq_sdpram #(.W(18), .AW(9)) u_ho (.clk, .we(coef_we && coef_idx[0]), .wa({coef_ch, coef_idx[7:1]}),
                                      .wd(coef_data), .ra(i_ha), .rq(ho_q));

  // ------------------------------------------------------------ sample writes
  logic [7:0]     n  [NCH];                  // CIC outputs since enable (mod 256)
  logic [6:0]     pc [NCH];                  // pairs since enable, saturating
  // pending jobs: up to 4 per channel (a long job of another channel may
  // keep the engine busy for several of this channel's output periods)
  logic [6:0]     jqm [NCH][4];
  logic [NCH-1:0] jqs [4];
  logic [1:0]     jwr [NCH], jrd [NCH];
  logic [2:0]     jn  [NCH];
  logic [NCH-1:0] jp;
  logic [6:0]     jm [NCH];                  // head of each queue
  logic [NCH-1:0] jskip;
  always_comb
    for (int c = 0; c < NCH; c++) begin
      jp[c] = jn[c] != 3'd0;
      jm[c] = jqm[c][jrd[c]];
      jskip[c] = jqs[jrd[c]][c];
    end
  logic [1:0]     gen [NCH];
  logic [NCH-1:0] en_q;

  wire        w_en  = s_valid && en[s_ch];
  wire        w_odd = n[s_ch][0];
  wire [8:0]  w_a   = {s_ch, n[s_ch][7:1]};
  logic [NCH-1:0] jset, jtake;
  always_comb
    for (int c = 0; c < NCH; c++) jset[c] = w_en && s_ch == 2'(c) && n[c][0];

  assign w_en_x = w_en;
  assign w_odd_x = w_odd;
  assign w_a_x = w_a;
  assign w_d_x = {s_q, s_i};

  always_ff @(posedge clk) begin
    en_q <= en;
    for (int c = 0; c < NCH; c++) begin
      if (!en[c]) begin
        n[c] <= '0; pc[c] <= '0; jn[c] <= '0; jwr[c] <= '0; jrd[c] <= '0;
      end else begin
        if (w_en && s_ch == 2'(c)) begin
          n[c] <= n[c] + 8'd1;
          if (n[c][0] && pc[c] != 7'd127) pc[c] <= pc[c] + 7'd1;
        end
        if (jset[c] && (jn[c] != 3'd4 || jtake[c])) begin
          jqm[c][jwr[c]] <= n[c][7:1];
          jqs[jwr[c]][c] <= 8'(pc[c]) + 8'd1 < 8'(npairs[c]);   // m < npairs - 1
          jwr[c] <= jwr[c] + 2'd1;
        end else if (jset[c]) st_drop[c] <= st_drop[c] + 16'd1;
        if (jtake[c]) jrd[c] <= jrd[c] + 2'd1;
        jn[c] <= jn[c] + 3'(jset[c] && (jn[c] != 3'd4 || jtake[c])) - 3'(jtake[c]);
      end
      if (en[c] && !en_q[c]) gen[c] <= gen[c] + 2'd1;
    end
  end

  // ------------------------------------------------------------ MAC engine
  logic        busy;
  logic [1:0]  ec, rr;
  logic [6:0]  ej, em, enp;
  logic        eskip;
  logic [1:0]  egen;

  // issue stage outputs
  logic        i_v, i_first, i_last, i_skip;
  logic [1:0]  i_ch, i_gen;

  logic        pick;
  logic [1:0]  pc_ch;
  always_comb begin
    pick = 1'b0; pc_ch = '0; jtake = '0;
    if (!busy || ej == enp - 7'd1) begin
      for (int k = NCH - 1; k >= 0; k--)
        if (jp[2'(rr + 2'(k))]) begin pick = 1'b1; pc_ch = 2'(rr + 2'(k)); end
      if (pick) jtake[pc_ch] = 1'b1;
    end
  end

  always_ff @(posedge clk) begin
    i_v <= 1'b0;
    if (pick) begin
      busy <= 1'b1;
      ec <= pc_ch; rr <= pc_ch + 2'd1;
      ej <= '0; em <= jm[pc_ch]; enp <= npairs[pc_ch];
      eskip <= jskip[pc_ch]; egen <= gen[pc_ch];
    end else if (busy) begin
      if (ej == enp - 7'd1) busy <= 1'b0;
      else ej <= ej + 7'd1;
    end
    if (busy) begin
      i_v <= 1'b1;
      i_first <= ej == 7'd0;
      i_last <= ej == enp - 7'd1;
      i_skip <= eskip;
      i_ch <= ec; i_gen <= egen;
      i_xa <= {ec, 7'(em - ej)};
      i_ha <= {ec, ej};
    end
  end

  // read
  logic        r_v, r_first, r_last, r_skip;
  logic [1:0]  r_ch, r_gen;
  always_ff @(posedge clk) begin
    r_v <= i_v; r_first <= i_first; r_last <= i_last; r_skip <= i_skip;
    r_ch <= i_ch; r_gen <= i_gen;
  end

  // second register (the block RAMs keep their output register; this one
  // may become the DSP input register)
  logic        q_v, q_first, q_last, q_skip;
  logic [1:0]  q_ch, q_gen;
  logic [47:0] xo_r, xe_r;
  logic signed [17:0] he_r, ho_r;
  always_ff @(posedge clk) begin
    q_v <= r_v; q_first <= r_first; q_last <= r_last; q_skip <= r_skip;
    q_ch <= r_ch; q_gen <= r_gen;
    xo_r <= xo_q; xe_r <= xe_q; he_r <= he_q; ho_r <= ho_q;
  end

  // multiply
  logic        p_v, p_first, p_last, p_skip;
  logic [1:0]  p_ch, p_gen;
  logic signed [41:0] pai, paq, pbi, pbq;
  always_ff @(posedge clk) begin
    p_v <= q_v; p_first <= q_first; p_last <= q_last; p_skip <= q_skip;
    p_ch <= q_ch; p_gen <= q_gen;
    pai <= $signed(xo_r[23:0])  * he_r;
    paq <= $signed(xo_r[47:24]) * he_r;
    pbi <= $signed(xe_r[23:0])  * ho_r;
    pbq <= $signed(xe_r[47:24]) * ho_r;
  end

  // pair sum
  logic        u_v, u_first, u_last, u_skip;
  logic [1:0]  u_ch, u_gen;
  logic signed [42:0] ui, uq;
  always_ff @(posedge clk) begin
    u_v <= p_v; u_first <= p_first; u_last <= p_last; u_skip <= p_skip;
    u_ch <= p_ch; u_gen <= p_gen;
    ui <= 43'(pai) + 43'(pbi);
    uq <= 43'(paq) + 43'(pbq);
  end

  // accumulate
  logic        a_v, a_skip;
  logic [1:0]  a_ch, a_gen;
  logic [5:0]  a_osh;
  logic signed [47:0] acc_i, acc_q;
  always_ff @(posedge clk) begin
    a_v <= 1'b0;
    if (u_v) begin
      acc_i <= (u_first ? 48'sd0 : acc_i) + 48'(ui);
      acc_q <= (u_first ? 48'sd0 : acc_q) + 48'(uq);
      if (u_last) begin
        a_v <= 1'b1;
        a_skip <= u_skip;
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
  logic [1:0]  o_ch;
  logic signed [48:0] o_i, o_q;
  always_ff @(posedge clk) begin
    o_v <= a_v && !a_skip && en[a_ch] && a_gen == gen[a_ch];
    o_ch <= a_ch;
    o_i <= rsh(acc_i, a_osh);
    o_q <= rsh(acc_q, a_osh);
    m_valid <= o_v && en[o_ch];
    m_ch <= o_ch;
    m_i <= sat16(o_i);
    m_q <= sat16(o_q);
  end

endmodule
