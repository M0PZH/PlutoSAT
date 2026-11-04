// DVB-S2 RX PLS decoder: MODCOD, frame size and pilots of every PLHEADER
// (model: model/rxfixed.py, pls_decode() / PlsDecoder; see there for the
// algorithm). Detection only: it taps the carrier's input and changes
// nothing in the receive path.
//
// The 90 header symbols (s_sof on the first) are stored as 6-bit values;
// then, for each frequency hypothesis (the carrier's frequency if it has
// acquired, then the grid h * step, h = -H..H):
//   rotation : 90 symbols, 4 clocks each (one multiplier): stripped of the
//              PLS-code-0 header (SOF, PLS scrambling, pi/2), rotated by
//              -k f (64-phase table) -> S (SOF sum) and Y[0..63] (PLS)
//   passes   : q = 0..15, 64 clocks each: two accumulators (pilots 0/1)
//              over the two 16-pair halves -> P1, P2
//   metrics  : 8 per pass (second multiplier, overlapping the next pass):
//              T = S +- (P1 +- P2), M = Ti^2 + Tq^2; running best / second
//              best of the hypothesis, then the best hypothesis
// ~1420 clocks per hypothesis (~94k with H = 32 and the carrier
// hypothesis); a header that starts while a decode runs is skipped
// (st_skipped). The carrier hypothesis alone decides when it gives the
// configured PLS code or the dummy code with a metric > 2^8. Confident:
// best > 2 x second (another code, same hypothesis) and best > 2^8.
// Widths: r 6, x 8, y 7, S and P 12, T 14 bits (no overflow), M = (Ti^2 +
// Tq^2) >> 8, 18 bits.
//
// Status: res_* = the last decode (res_v pulses), data_* = the last
// confident decode of a data header (MODCOD != 0); counters of confident
// data decodes whose MODCOD (or frame size) / pilots differ from the
// configured ones; a mask of the MODCODs of confident normal-frame decodes
// (bit 0: dummy frames), cleared by seen_clr bits.

module dvbs2_rx_pls
  import dvbs2_pl_pkg::*;
(
  input  logic               clk,
  input  logic               rst,

  input  logic [4:0]         modcod,         // configured
  input  logic               pilots,
  input  logic [5:0]         cfg_h,          // grid -H..H
  input  logic [9:0]         cfg_step,       // turns * 2^16 per symbol

  input  logic               s_v,            // symbol taken by the carrier
  input  logic signed [15:0] s_i,
  input  logic signed [15:0] s_q,
  input  logic               s_sof,
  input  logic [31:0]        fhat,           // carrier frequency, turns * 2^32
  input  logic               acquired,

  input  logic [31:0]        seen_clr,       // w1c

  output logic               res_v,
  output logic [6:0]         res_code,       // {MODCOD, short, pilots}
  output logic               res_conf,
  output logic               res_car,        // from the carrier hypothesis
  output logic               res_grid,       // the grid was searched
  output logic [17:0]        res_best,
  output logic [17:0]        res_second,
  output logic [15:0]        res_freq,       // hypothesis, turns * 2^16
  output logic               res_valid,      // a decode since reset
  output logic [15:0]        st_count,
  output logic [6:0]         data_code,
  output logic               data_conf,
  output logic               data_car,
  output logic               data_grid,
  output logic               data_valid,
  output logic [15:0]        st_data,
  output logic [15:0]        st_diff_modcod,
  output logic [15:0]        st_diff_pilots,
  output logic [31:0]        st_seen,
  output logic [15:0]        st_skipped
);

  // ---------------------------------------------------------------- constants
  localparam logic [89:0] H0 = plheader(5'd0, 1'b0);        // PLS code 0

  // 64-phase table {-256 sin, 256 sin, 256 cos}: dvbs2_rx_pkg::PLS_TAB

  // ---------------------------------------------------------------- header capture
  (* ram_style = "distributed" *) logic [11:0] rbuf [128];    // {q, i}, 6 bit each
  logic        col, busy, go;
  logic [6:0]  ck;
  logic [15:0] f_car;
  logic        acq_l;
  logic [4:0]  cmc;
  logic        cpil;
  logic [5:0]  ch;
  logic [9:0]  cstep;

  // sat6(v >> 9)
  function automatic logic [5:0] q6(input logic signed [15:0] v);
    if (v[15] != v[14]) q6 = v[15] ? 6'h20 : 6'h1F;
    else                q6 = v[14:9];
  endfunction

  // one port: written while collecting, read while decoding (busy)
  logic        rb_we;
  logic [6:0]  rb_wa, rk;
  logic [11:0] rb_wd;
  wire  [6:0]  rb_a = busy ? rk : rb_wa;
  always_ff @(posedge clk) if (rb_we) rbuf[rb_a] <= rb_wd;

  always_ff @(posedge clk) begin
    go <= 1'b0;
    rb_we <= 1'b0;
    if (rst) begin
      col <= 1'b0;
      st_skipped <= '0;
    end else if (s_v) begin
      if (s_sof && busy) begin
        col <= 1'b0;
        st_skipped <= st_skipped + 16'd1;
      end else if (s_sof || col) begin
        logic [6:0] k;
        k = s_sof ? 7'd0 : ck;
        rb_we <= 1'b1;
        rb_wa <= k;
        rb_wd <= {q6(s_q), q6(s_i)};
        ck <= k + 7'd1;
        col <= k != 7'd89;
        go  <= k == 7'd89;          // (busy rises after the write)
        if (s_sof) begin
          f_car <= fhat[31:16];
          acq_l <= acquired;
          cmc   <= modcod;
          cpil  <= pilots;
          ch    <= cfg_h;
          cstep <= cfg_step;
        end
      end
    end
  end

  // ---------------------------------------------------------------- control
  typedef enum logic [3:0] {D_IDLE, D_INIT, D_HYP, D_ROT, D_ROTW, D_PASS, D_MTAIL, D_HEND, D_DONE} st_t;
  st_t st;

  logic        hcar;                         // current hypothesis: the carrier's
  logic [15:0] f, fg;                        // its frequency, the grid's next
  logic [5:0]  hc;
  logic [6:0]  gleft;                        // grid hypotheses left after this one
  logic        gfirst, used_grid;

  // rotation
  logic [1:0]  sub;
  logic [15:0] phi;
  logic signed [7:0] xi_r, xq_r;
  logic signed [9:0] c_r, s_r, ns_r;
  logic [6:0]  k_r;
  logic        x_v;                          // xi_r.. hold a symbol
  // multiplier R: issue -> operands -> product -> accumulator
  logic              i_v, i_first, i_last, i_q;
  logic [6:0]        i_k;
  logic signed [7:0] ma;
  logic signed [9:0] mb;
  logic              a_v, a_first, a_last, a_q;
  logic [6:0]        a_k;
  (* use_dsp = "yes" *) logic signed [17:0] mp;
  (* use_dsp = "yes" *) logic signed [17:0] racc;
  logic              r_v, r_q;
  logic [6:0]        r_k;
  logic signed [6:0] yi_h;
  logic signed [11:0] Si, Sq;
  (* ram_style = "distributed" *) logic [13:0] ybuf [64];     // {yq, yi}
  logic              y_we;
  logic [5:0]        y_wa;
  logic [13:0]       y_wd;
  logic [5:0]        pj;
  wire  [5:0]        y_a = y_we ? y_wa : pj;          // one port
  always_ff @(posedge clk) if (y_we) ybuf[y_a] <= y_wd;

  // passes
  logic [3:0]  pq;
  logic        pr_v;
  logic [5:0]  pr_j;
  logic [3:0]  pr_q;
  logic [13:0] yrd;
  logic signed [11:0] A0i, A0q, A1i, A1q, T0i, T0q, T1i, T1q;
  logic        l1, l2;
  logic [3:0]  lq;
  logic signed [11:0] P10i, P10q, P11i, P11q, P20i, P20q, P21i, P21q;

  // metrics
  logic        m_run;
  logic [3:0]  mi;
  logic [3:0]  mqq;
  logic              t_v, t_c;
  logic [6:0]        t_code;
  logic signed [13:0] mt;
  logic              u_v, u_c;
  logic [6:0]        u_code;
  logic signed [13:0] mu;
  logic              w_v, w_c;
  logic [6:0]        w_code;
  logic signed [25:0] mprod;
  logic              z_v;
  logic [6:0]        z_code;
  logic [25:0]       macc;
  wire  [17:0]       mm = macc[25:8];
  logic [17:0]       lbest, lsecond, gbest, gsecond;
  logic [6:0]        lcode, gcode;
  logic [15:0]       gfreq;
  logic              gcar;
  wire               m_busy = m_run || t_v || u_v || w_v || z_v;

  wire [6:0] cfg_code = {cmc, 1'b0, cpil};
  // confident: best > 2 second and best > 2^16 (M < 2^25.1)
  wire       gconf = 19'(gbest) > {gsecond, 1'b0} && gbest > 18'h100;
  // grid frequency: -H steps at the start, then +1 step per hypothesis
  wire [15:0] fg_next = (st == D_INIT) ? fg - 16'(cstep) : fg + 16'(cstep);

  always_ff @(posedge clk) begin
    i_v <= 1'b0;
    y_we <= 1'b0;
    pr_v <= 1'b0;
    res_v <= 1'b0;
    if (rst) begin
      st <= D_IDLE;
      busy <= 1'b0;
      x_v <= 1'b0;
      a_v <= 1'b0; r_v <= 1'b0;
      l1 <= 1'b0; l2 <= 1'b0;
      m_run <= 1'b0; t_v <= 1'b0; u_v <= 1'b0; w_v <= 1'b0; z_v <= 1'b0;
      res_valid <= 1'b0; data_valid <= 1'b0;
      st_count <= '0; st_data <= '0; st_diff_modcod <= '0; st_diff_pilots <= '0;
    end else begin
      // ------------------------------------------------ multiplier R pipeline
      // (ma, mb: issue; mp: product, aligned with a_*; racc: sum)
      a_v <= i_v; a_first <= i_first; a_last <= i_last; a_q <= i_q; a_k <= i_k;
      mp  <= ma * mb;
      if (a_v) racc <= a_first ? mp + 18'sd256 : racc + mp;
      r_v <= a_v && a_last; r_q <= a_q; r_k <= a_k;
      if (r_v) begin : ystore
        logic signed [6:0] y;
        y = racc[15:9];
        if (!r_q) begin
          yi_h <= y;
          if (r_k < 7'd26) Si <= Si + 12'(y);
        end else begin
          if (r_k < 7'd26) Sq <= Sq + 12'(y);
          else begin
            y_we <= 1'b1;
            y_wa <= 6'(r_k - 7'd26);
            y_wd <= {y, yi_h};
          end
        end
      end

      // ------------------------------------------------ pass accumulators
      // (yrd is Y[pr_j])
      yrd <= ybuf[y_a];
      if (pr_v) begin : pacc
        logic s0, s1;
        logic signed [6:0] yi, yq;
        yi = $signed(yrd[6:0]);
        yq = $signed(yrd[13:7]);
        s0 = ^(pr_j[4:1] & pr_q);
        s1 = s0 ^ pr_j[0];
        A0i <= (pr_j[4:0] == 0 ? 12'sd0 : A0i) + (s0 ? -12'(yi) : 12'(yi));
        A0q <= (pr_j[4:0] == 0 ? 12'sd0 : A0q) + (s0 ? -12'(yq) : 12'(yq));
        A1i <= (pr_j[4:0] == 0 ? 12'sd0 : A1i) + (s1 ? -12'(yi) : 12'(yi));
        A1q <= (pr_j[4:0] == 0 ? 12'sd0 : A1q) + (s1 ? -12'(yq) : 12'(yq));
      end
      l1 <= pr_v && pr_j == 6'd31;
      l2 <= pr_v && pr_j == 6'd63;
      lq <= pr_q;
      if (l1) begin
        T0i <= A0i; T0q <= A0q; T1i <= A1i; T1q <= A1q;
      end
      if (l2) begin
        P10i <= T0i; P10q <= T0q; P11i <= T1i; P11q <= T1q;
        P20i <= A0i; P20q <= A0q; P21i <= A1i; P21q <= A1q;
        mqq <= lq;
        m_run <= 1'b1;
        mi <= '0;
      end

      // ------------------------------------------------ metrics: T, then T^2 summed
      t_v <= m_run;
      if (m_run) begin : met
        logic b6, a4, b5, c;
        logic signed [13:0] sc, p1, p2, w;
        c  = mi[0];
        b5 = mi[1];
        a4 = mi[2];
        b6 = mi[3];
        sc = c ? 14'(Sq) : 14'(Si);
        p1 = c ? (b6 ? 14'(P11q) : 14'(P10q)) : (b6 ? 14'(P11i) : 14'(P10i));
        p2 = c ? (b6 ? 14'(P21q) : 14'(P20q)) : (b6 ? 14'(P21i) : 14'(P20i));
        w  = a4 ? p1 - p2 : p1 + p2;
        mt <= b5 ? sc - w : sc + w;
        t_c <= c;
        // MODCOD = bit-reversed {a4, q}
        t_code <= {mqq[0], mqq[1], mqq[2], mqq[3], a4, b5, b6};
        mi <= mi + 4'd1;
        if (mi == 4'd15) m_run <= 1'b0;
      end
      u_v <= t_v; u_c <= t_c; u_code <= t_code; mu <= mt;
      w_v <= u_v; w_c <= u_c; w_code <= u_code;
      mprod <= mu * mu;
      z_v <= w_v && w_c; z_code <= w_code;
      if (w_v) macc <= w_c ? macc + mprod : mprod;
      if (z_v) begin
        if (mm > lbest) begin
          lsecond <= lbest;
          lbest <= mm;
          lcode <= z_code;
        end else if (mm > lsecond) lsecond <= mm;
      end

      // ------------------------------------------------ sequencer
      case (st)
        D_IDLE: if (go) begin
          busy <= 1'b1;
          fg <= '0;
          hc <= ch;
          gfirst <= 1'b1;
          used_grid <= !acq_l;
          st <= D_INIT;
        end

        D_INIT: begin                        // fg = -H * step
          if (hc != 0) begin
            fg <= fg_next;
            hc <= hc - 6'd1;
          end else begin
            hcar  <= acq_l;
            f     <= acq_l ? f_car : fg;
            gleft <= acq_l ? 7'({ch, 1'b0}) + 7'd1 : 7'({ch, 1'b0});
            if (!acq_l) fg <= fg_next;
            st <= D_HYP;
          end
        end

        D_HYP: begin
          rk <= '0; sub <= '0; phi <= '0; x_v <= 1'b0;
          Si <= '0; Sq <= '0;
          lbest <= '0; lsecond <= '0; lcode <= '0;
          st <= D_ROT;
        end

        D_ROT: begin
          // slot of 4 clocks per symbol: products xi c, xq s (I), xq c,
          // xi (-s) (Q) of the symbol in x_r; the next symbol is fetched at sub 0
          sub <= sub + 2'd1;
          if (x_v) begin
            i_v <= 1'b1;
            i_k <= k_r;
            i_first <= sub == 2'd1 || sub == 2'd3;
            i_last  <= sub == 2'd2 || sub == 2'd0;
            i_q     <= sub == 2'd3 || sub == 2'd0;
            case (sub)
              2'd1: begin ma <= xi_r; mb <= c_r;  end
              2'd2: begin ma <= xq_r; mb <= s_r;  end
              2'd3: begin ma <= xq_r; mb <= c_r;  end
              default: begin ma <= xi_r; mb <= ns_r; end
            endcase
          end
          if (sub == 2'd0) begin
            if (rk == 7'd90) begin
              x_v <= 1'b0;
              st <= D_ROTW;
            end else begin : fetch
              logic [11:0] r;
              logic signed [7:0] ri, rq, xi, xq;
              logic [29:0] t;
              r  = rbuf[rk];
              ri = 8'($signed(r[5:0]));
              rq = 8'($signed(r[11:6]));
              if (!rk[0]) begin xi = ri + rq; xq = rq - ri; end
              else        begin xi = rq - ri; xq = -ri - rq; end
              if (H0[7'd89 - rk]) begin xi = -xi; xq = -xq; end
              t = dvbs2_rx_pkg::PLS_TAB[6'((phi + 16'd512) >> 10)];
              xi_r <= xi; xq_r <= xq;
              c_r <= $signed(t[9:0]); s_r <= $signed(t[19:10]); ns_r <= $signed(t[29:20]);
              k_r <= rk;
              x_v <= 1'b1;
              phi <= phi + f;
              rk <= rk + 7'd1;
            end
          end
        end

        D_ROTW: if (!i_v && !a_v && !r_v && !y_we) begin
          pq <= '0; pj <= '0;
          st <= D_PASS;
        end

        D_PASS: begin
          pr_v <= 1'b1; pr_j <= pj; pr_q <= pq;
          pj <= pj + 6'd1;
          if (pj == 6'd63) begin
            pq <= pq + 4'd1;
            if (pq == 4'd15) st <= D_MTAIL;
          end
        end

        D_MTAIL: if (!pr_v && !l1 && !l2 && !m_busy) st <= D_HEND;

        D_HEND: begin : hend
          logic take, stop;
          take = gfirst || lbest > gbest;
          gfirst <= 1'b0;
          if (take) begin
            gbest <= lbest; gsecond <= lsecond; gcode <= lcode; gfreq <= f; gcar <= hcar;
          end
          // the carrier hypothesis decides alone when it gives the expected code
          stop = hcar && lbest > 18'h100 && (lcode == cfg_code || lcode == 7'd0);
          if (stop || gleft == 0) st <= D_DONE;
          else begin
            if (hcar) used_grid <= 1'b1;
            hcar  <= 1'b0;
            f     <= fg;
            fg    <= fg_next;
            gleft <= gleft - 7'd1;
            st <= D_HYP;
          end
        end

        D_DONE: begin : done
          logic conf;
          conf = gconf;
          busy <= 1'b0;
          res_v <= 1'b1;
          res_valid <= 1'b1;
          res_code <= gcode; res_conf <= conf; res_car <= gcar; res_grid <= used_grid;
          res_best <= gbest; res_second <= gsecond; res_freq <= gfreq;
          st_count <= st_count + 16'd1;
          if (conf && gcode[6:2] != 5'd0) begin
            data_code <= gcode; data_conf <= conf; data_car <= gcar; data_grid <= used_grid;
            data_valid <= 1'b1;
            st_data <= st_data + 16'd1;
            if (gcode[6:1] != {cmc, 1'b0}) st_diff_modcod <= st_diff_modcod + 16'd1;
            if (gcode[0] != cpil)          st_diff_pilots <= st_diff_pilots + 16'd1;
          end
          st <= D_IDLE;
        end

        default: st <= D_IDLE;
      endcase

    end
  end

  // seen mask (a clear wins over a decode in the same clock)
  always_ff @(posedge clk)
    for (int b = 0; b < 32; b++)
      if (rst || seen_clr[b]) st_seen[b] <= 1'b0;
      else if (st == D_DONE && gconf && !gcode[1] && gcode[6:2] == 5'(b)) st_seen[b] <= 1'b1;

endmodule
