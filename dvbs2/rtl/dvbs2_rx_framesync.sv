// DVB-S2 RX PLHEADER detection (model: model/rxfixed.py, class FrameSync).
//
// d[n] = s[n] * conj(s[n-1]) (12 bit) is correlated with the 89 header
// differentials of two candidates, the data MODCOD (+pilots) and the dummy
// header: A = sum c_i d[n-89+i], c_i = +-1, metric |A|^2 >> 8. This is
// insensitive to the carrier frequency (the header differentials are +-j).
//
//   SEARCH : keep the largest metric seen (the search peak); its next
//            header is CHECKed at the dummy length first (the type is
//            unreliable at low SNR), for a data candidate then at the data
//            length
//   CHECK  : max over +-W around the prediction; NCHECK (2) headers in a
//            row, each above max(thresh, 4 nm), both together above
//            10 nm (nm: mean metric, the noise level) -> LOCKED, else SEARCH
//            (at -2 dB a header is ~8 nm, the largest noise metric of a
//            frame ~7 nm)
//   LOCKED : max over +-W around each predicted header (but the predicted
//            position unless the max is more than 2 x the metric there)
//            above max(thresh, 1/4 of the running average of the found
//            peaks); a miss keeps the predicted position (flywheel), more
//            than MAXMISS -> SEARCH
// ty_v: the carrier's frame type of the last tagged frame (coherent once it
// has acquired the frequency); the next header is predicted from it.
//
// Symbols leave DELAY symbols late, m_sof on the first header symbol of each
// frame found (m_dummy: dummy frame). 6 correlator lanes: ~34 clocks/symbol.

module dvbs2_rx_framesync
  import dvbs2_pl_pkg::*;
#(
  parameter int W       = 2,
  parameter int DELAY   = 96,
  parameter int MAXMISS = 3,
  parameter int NCHECK  = 2
) (
  input  logic               clk,
  input  logic               rst,

  input  logic [4:0]         modcod,
  input  logic               pilots,
  input  logic [31:0]        thresh,

  input  logic               s_valid,
  output logic               s_ready,
  input  logic signed [15:0] s_i,
  input  logic signed [15:0] s_q,

  output logic               m_valid,
  input  logic               m_ready,
  output logic signed [15:0] m_i,
  output logic signed [15:0] m_q,
  output logic               m_sof,
  output logic               m_dummy,
  output logic               m_found,       // with m_sof: detected (not flywheel)

  input  logic               ty_v,
  input  logic               ty_dummy,

  output logic [1:0]         fs_state,      // 0 search, 1 check, 2 locked
  output logic [15:0]        cnt_found,
  output logic [15:0]        cnt_flywheel,
  output logic [15:0]        cnt_lost
);

  localparam int LANES = 6;
  localparam int TL    = 15;                 // LANES * TL >= 89
  localparam logic [1:0] SEARCH = 2'd0, CHECK = 2'd1, LOCKED = 2'd2;

  // ------------------------------------------------------------ coefficients
  logic [89:0] hdr_d, hdr_z;
  logic [89:1] neg_d, neg_z;                 // c_i = -1
  logic [31:0] len_d;
  localparam logic [31:0] LEN_Z = 32'd3330;

  always_ff @(posedge clk) begin
    hdr_d <= plheader(modcod, pilots);
    hdr_z <= plheader(5'd0, 1'b0);
    len_d <= pilots ? 32'd33282 : 32'd32490;
    for (int i = 1; i < 90; i++) begin
      // y_k = hdr[89 - k]; c_i = b_i b_(i-1) (-1 if i odd)
      neg_d[i] <= hdr_d[89 - i] ^ hdr_d[90 - i] ^ i[0];
      neg_z[i] <= hdr_z[89 - i] ^ hdr_z[90 - i] ^ i[0];
    end
  end

  // ------------------------------------------------------------ storage
  logic [31:0] sbuf [128];                   // symbols {q, i}
  // differentials {q, i}: one LUTRAM copy per correlator lane
  logic        d_we;
  logic [23:0] d_wdata;
  logic [23:0] rd [LANES];

  for (genvar l = 0; l < LANES; l++) begin : g_lane
    logic [23:0] mem [128];
    wire  [6:0]  ra = 7'(n - 32'd88 + 32'(l * TL) + 32'(t));   // d[n - 89 + i], i = 1 + l*TL + t
    always_ff @(posedge clk) begin
      if (d_we) mem[n[6:0]] <= d_wdata;
      rd[l] <= mem[ra];
    end
  end

  function automatic logic signed [11:0] sat12r(input logic signed [32:0] a);
    logic signed [32:0] r;
    r = (a + 33'sd32768) >>> 16;
    if (r > 33'sd2047)       sat12r = 12'sd2047;
    else if (r < -33'sd2048) sat12r = -12'sd2048;
    else                     sat12r = r[11:0];
  endfunction

  typedef enum logic [4:0] {S_IN, S_D0, S_D1, S_D2, S_COR, S_MET1, S_MET2, S_MET3,
                            S_DEC1, S_DEC2, S_DEC3, S_DEC4, S_DEC5, S_DEC6, S_DEC65, S_DEC7, S_OUT} st_t;
  st_t st;

  logic [31:0]        n;                     // index of the current symbol
  logic signed [15:0] ci, cq, pi_, pq;
  logic signed [32:0] pdi, pdq;
  logic signed [31:0] p1, p2, p3, p4;
  logic [37:0]        sqd_i, sqd_q, sqz_i, sqz_q;
  logic               inwin;
  logic [4:0]         t;
  logic               iss, rd_v;
  logic               rd_ok [LANES];
  logic               rd_nd [LANES], rd_nz [LANES];
  logic signed [18:0] ad_i [LANES], ad_q [LANES], az_i [LANES], az_q [LANES];
  logic signed [18:0] Ad_i, Ad_q, Az_i, Az_q;
  logic [31:0]        md, mz;

  logic [1:0]  fst;
  logic        best_v, win_v;
  logic [31:0] best_m, best_n, win_m, win_n, pred;
  logic        best_c, win_c;
  logic [31:0] cwin_m;                       // window: best at the predicted position
  logic        cwin_c;
  logic [3:0]  miss;
  logic [31:0] pk;                           // average found peak, 0 = none
  logic [31:0] thr, thresh_q;                 // registered (S_DEC2)
  logic [33:0] nm4;
  logic [35:0] nm10;
  logic        pend_v, pend_c, pend_f;
  logic [31:0] pend_hs;
  logic        nchk;                         // CHECK: headers found so far (NCHECK = 2)
  logic        chk_alt;                      // CHECK at the dummy length of a data candidate
  logic [32:0] csum;                         // CHECK: sum of the metrics found
  logic [31:0] nm;                           // mean metric, IIR 2^-10 per symbol
  logic        have_last;                    // last_n: end of the last tagged header
  logic [31:0] last_n;
  logic        ty_p, ty_pc;                  // carrier report pending

  assign s_ready  = (st == S_IN) && !rst;
  assign fs_state = fst;

  function automatic logic [31:0] flen(input logic c, input logic [31:0] ld);
    flen = c ? LEN_Z : ld;
  endfunction

  always_ff @(posedge clk) begin
    if (rst) begin
      st <= S_IN;
      n <= '0;
      pi_ <= '0; pq <= '0;
      iss <= 1'b0; rd_v <= 1'b0;
      fst <= SEARCH;
      best_v <= 1'b0; win_v <= 1'b0;
      miss <= '0;
      pk <= '0;
      nm <= '0;
      pend_v <= 1'b0;
      have_last <= 1'b0;
      ty_p <= 1'b0;
      m_valid <= 1'b0;
      cnt_found <= '0; cnt_flywheel <= '0; cnt_lost <= '0;
    end else begin
      if (m_valid && m_ready) m_valid <= 1'b0;

      d_we <= 1'b0;
      if (ty_v) begin
        ty_p <= 1'b1; ty_pc <= ty_dummy;
      end
      // correlator read stage (rd[] is read in g_lane)
      rd_v <= iss;
      for (int l = 0; l < LANES; l++) begin
        automatic int i = 1 + l * TL + int'(t);
        rd_ok[l] <= iss && (i <= 89);
        rd_nd[l] <= (i <= 89) ? neg_d[i] : 1'b0;
        rd_nz[l] <= (i <= 89) ? neg_z[i] : 1'b0;
      end

      case (st)
        S_IN: if (s_valid) begin
          ci <= s_i; cq <= s_q;
          sbuf[n[6:0]] <= {s_q, s_i};
          st <= S_D0;
        end

        S_D0: begin
          p1 <= 32'(ci) * 32'(pi_);
          p2 <= 32'(cq) * 32'(pq);
          p3 <= 32'(cq) * 32'(pi_);
          p4 <= 32'(ci) * 32'(pq);
          pi_ <= ci; pq <= cq;
          st <= S_D1;
        end

        S_D1: begin
          pdi <= 33'(p1) + 33'(p2);
          pdq <= 33'(p3) - 33'(p4);
          st <= S_D2;
        end

        S_D2: begin
          d_we    <= 1'b1;
          d_wdata <= {sat12r(pdq), sat12r(pdi)};
          for (int l = 0; l < LANES; l++) begin
            ad_i[l] <= '0; ad_q[l] <= '0; az_i[l] <= '0; az_q[l] <= '0;
          end
          t <= '0;
          iss <= 1'b1;
          st <= S_COR;
        end

        S_COR: begin
          if (iss) begin
            if (t == 5'(TL - 1)) iss <= 1'b0;
            else                 t <= t + 5'd1;
          end
          if (rd_v) begin
            for (int l = 0; l < LANES; l++) if (rd_ok[l]) begin
              automatic logic signed [11:0] di = $signed(rd[l][11:0]);
              automatic logic signed [11:0] dq = $signed(rd[l][23:12]);
              ad_i[l] <= rd_nd[l] ? ad_i[l] - 19'(di) : ad_i[l] + 19'(di);
              ad_q[l] <= rd_nd[l] ? ad_q[l] - 19'(dq) : ad_q[l] + 19'(dq);
              az_i[l] <= rd_nz[l] ? az_i[l] - 19'(di) : az_i[l] + 19'(di);
              az_q[l] <= rd_nz[l] ? az_q[l] - 19'(dq) : az_q[l] + 19'(dq);
            end
          end
          if (!iss && !rd_v) st <= S_MET1;
        end

        S_MET1: begin
          Ad_i <= ad_i[0] + ad_i[1] + ad_i[2] + ad_i[3] + ad_i[4] + ad_i[5];
          Ad_q <= ad_q[0] + ad_q[1] + ad_q[2] + ad_q[3] + ad_q[4] + ad_q[5];
          Az_i <= az_i[0] + az_i[1] + az_i[2] + az_i[3] + az_i[4] + az_i[5];
          Az_q <= az_q[0] + az_q[1] + az_q[2] + az_q[3] + az_q[4] + az_q[5];
          st <= S_MET2;
        end

        S_MET2: begin
          sqd_i <= 38'(38'(Ad_i) * 38'(Ad_i));
          sqd_q <= 38'(38'(Ad_q) * 38'(Ad_q));
          sqz_i <= 38'(38'(Az_i) * 38'(Az_i));
          sqz_q <= 38'(38'(Az_q) * 38'(Az_q));
          st <= S_MET3;
        end

        S_MET3: begin
          // metrics are 0 until a full header fits (n >= 88)
          md <= (n < 32'd88) ? '0 : 32'((sqd_i + sqd_q) >> 8);
          mz <= (n < 32'd88) ? '0 : 32'((sqz_i + sqz_q) >> 8);
          st <= S_DEC1;
        end

        // decision, one step per clock (same order as the model)
        S_DEC1: begin                                // search: data candidate
          // the carrier's frame type of the last tagged frame
          if (ty_p && !ty_v) begin
            ty_p <= 1'b0;
            if (fst != SEARCH && have_last) pred <= last_n + flen(ty_pc, len_d);
          end
          if (fst == SEARCH && (!best_v || md > best_m)) begin
            best_v <= 1'b1; best_m <= md; best_n <= n; best_c <= 1'b0;
          end
          nm <= 32'($signed({2'b0, nm}) + (($signed({2'b0, 32'((33'(md) + 33'(mz)) >> 1)}) - $signed({2'b0, nm})) >>> 10));
          st <= S_DEC2;
        end

        S_DEC2: begin                                // search: dummy candidate
          if (fst == SEARCH && mz > best_m) begin
            best_m <= mz; best_n <= n; best_c <= 1'b1;
          end
          thresh_q <= thresh;
          thr  <= (pk >> 2) > thresh ? (pk >> 2) : thresh;
          nm4  <= {nm, 2'b00};
          nm10 <= 36'(nm) * 36'd10;
          st <= S_DEC3;
        end

        S_DEC3: begin                                // next header of the best due: check it
          if (fst == SEARCH && n == best_n + LEN_Z - 32'(W)) begin
            fst <= CHECK;
            nchk <= 1'b0;
            chk_alt <= !best_c;
            pred <= best_n + LEN_Z;
            win_v <= 1'b0;
          end else if (fst == SEARCH && !best_c && n == best_n + len_d - 32'(W)) begin
            fst <= CHECK;
            nchk <= 1'b0;
            chk_alt <= 1'b0;
            pred <= best_n + len_d;
            win_v <= 1'b0;
          end
          st <= S_DEC4;
        end

        S_DEC4: begin
          inwin <= fst != SEARCH && n + 32'(W) >= pred && n <= pred + 32'(W);
          st <= S_DEC5;
        end

        S_DEC5: begin
          if (inwin && (!win_v || md > win_m)) begin
            win_v <= 1'b1; win_m <= md; win_n <= n; win_c <= 1'b0;
          end
          if (inwin && n == pred) begin
            cwin_m <= md; cwin_c <= 1'b0;
          end
          st <= S_DEC6;
        end

        S_DEC6: begin
          if (inwin && mz > win_m) begin
            win_m <= mz; win_n <= n; win_c <= 1'b1;
          end
          if (inwin && n == pred && mz > cwin_m) begin
            cwin_m <= mz; cwin_c <= 1'b1;
          end
          st <= S_DEC65;
        end

        S_DEC65: begin
          // LOCKED: stay at the predicted position unless clearly beaten
          if (inwin && n == pred + 32'(W) && fst == LOCKED && 33'(win_m) <= {cwin_m, 1'b0}) begin
            win_m <= cwin_m; win_n <= pred; win_c <= cwin_c;
          end
          st <= S_DEC7;
        end

        S_DEC7: begin
          if (inwin && n == pred + 32'(W)) begin : dec
            logic ok;
            if (fst == CHECK) begin
              if (!nchk) ok = win_m > thresh_q && 34'(win_m) > nm4;
              else       ok = win_m > thresh_q && 34'(win_m) > nm4 && 36'(csum) + 36'(win_m) > nm10;
              csum <= nchk ? csum + 33'(win_m) : 33'(win_m);
            end else ok = win_m > thr;
            if (ok) begin
              pend_v <= 1'b1; pend_hs <= win_n - 32'd89; pend_c <= win_c; pend_f <= 1'b1;
              if (fst == LOCKED || nchk) fst <= LOCKED;
              nchk <= 1'b1;
              chk_alt <= 1'b0;
              miss <= '0;
              pk <= (pk == 0) ? win_m : 32'($signed({1'b0, pk}) + ($signed({1'b0, win_m}) - $signed({1'b0, pk})) >>> 3);
              pred <= win_n + flen(win_c, len_d);
              have_last <= 1'b1; last_n <= win_n;
              cnt_found <= cnt_found + 16'd1;
            end else if (fst == CHECK) begin
              fst <= SEARCH;
              if (!chk_alt) best_v <= 1'b0;   // else: wait for the data length
              chk_alt <= 1'b0;
              have_last <= 1'b0;
              pk <= '0;
            end else if (miss + 4'd1 > 4'(MAXMISS)) begin
              miss <= miss + 4'd1;
              fst <= SEARCH;
              best_v <= 1'b0;
              pk <= '0;
              have_last <= 1'b0;
              cnt_lost <= cnt_lost + 16'd1;
            end else begin
              miss <= miss + 4'd1;
              pend_v <= 1'b1; pend_hs <= pred - 32'd89; pend_c <= win_c; pend_f <= 1'b0;
              pred <= pred + flen(win_c, len_d);
              have_last <= 1'b1; last_n <= pred;
              cnt_flywheel <= cnt_flywheel + 16'd1;
            end
            win_v <= 1'b0;
          end
          st <= S_OUT;
        end

        S_OUT: begin
          if (n < 32'(DELAY)) begin
            n <= n + 32'd1;
            st <= S_IN;
          end else if (!m_valid || m_ready) begin : out
            logic [31:0] o, sv;
            o  = n - 32'(DELAY);
            sv = sbuf[o[6:0]];
            m_valid <= 1'b1;
            m_i <= $signed(sv[15:0]);
            m_q <= $signed(sv[31:16]);
            m_sof   <= pend_v && (pend_hs == o);
            m_dummy <= pend_c;
            m_found <= pend_f;
            if (pend_v && pend_hs == o) pend_v <= 1'b0;
            n <= n + 32'd1;
            st <= S_IN;
          end
        end

        default: st <= S_IN;
      endcase
    end
  end

endmodule
