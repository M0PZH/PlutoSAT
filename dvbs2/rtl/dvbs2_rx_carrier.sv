// DVB-S2 RX carrier recovery, PL descrambling and QPSK soft demapping
// (model: model/rxfixed.py, class Carrier; see there for the algorithm).
//
// Input : symbols from dvbs2_rx_framesync, s_sof on the first header symbol
//         of every frame (s_dummy: dummy frame, s_found: detected, not
//         flywheel). After each PLHEADER, ty_v pulses with the frame type
//         decided here (coherently, once the frequency is acquired) for the
//         frame sync's next prediction.
// Output: one word per data symbol, LLR(b0) from I and LLR(b1) from Q, int8,
//         4 LSBs per unit LLR; exactly 32400 words per data frame (symbols
//         missing from a frame cut short are sent as 0), m_first / m_last.
//
// Sequential datapath: ~8 clocks per symbol, ~900 more per PLHEADER
// (frequency estimate) and ~100 per known block. With pilots each data
// segment (<= 1440 symbols) is sent when the block after it has been
// measured, about one word per clock. The input needs a FIFO that covers
// these stalls.

module dvbs2_rx_carrier
  import dvbs2_pl_pkg::*;
  import dvbs2_rx_pkg::*;
#(
  parameter string LUT_FILE = "nco_lut.mem"
) (
  input  logic               clk,
  input  logic               rst,

  input  logic [4:0]         modcod,
  input  logic               pilots,
  input  logic [17:0]        gold_xa,
  input  logic [17:0]        gold_xb,
  input  logic [17:0]        pll_kp,
  input  logic [17:0]        pll_ki,

  input  logic               s_valid,
  output logic               s_ready,
  input  logic signed [15:0] s_i,
  input  logic signed [15:0] s_q,
  input  logic               s_sof,
  input  logic               s_dummy,
  input  logic               s_found,

  output logic               ty_v,
  output logic               ty_dummy,

  output logic               m_valid,
  input  logic               m_ready,
  output logic [7:0]         m_llr_i,
  output logic [7:0]         m_llr_q,
  output logic               m_first,
  output logic               m_last,

  output logic [31:0]        st_freq,      // NCO frequency, turns * 2^32 per symbol
  output logic [31:0]        st_amp,       // symbol amplitude estimate
  output logic [35:0]        st_energy,    // symbol energy estimate
  output logic [17:0]        st_gain,      // LLR gain
  output logic [15:0]        st_frames,    // data frames sent
  output logic [15:0]        st_acq,       // frequency acquisitions
  output logic [15:0]        st_fa,        // 1/1476 false-lock corrections
  output logic [15:0]        st_retype,    // frame types changed by the coherent decision
  output logic               st_acquired,  // frequency acquired

  output logic               snap_v,       // each data symbol (phase corrected, before the LLR gain)
  output logic signed [15:0] snap_i,
  output logic signed [15:0] snap_q
);

  localparam logic [8:0]  SEG_TIMEOUT = 9'd256;
  localparam logic [15:0] FLEN_DATA_P = 16'd33282, FLEN_DATA_N = 16'd32490, FLEN_DUMMY = 16'd3330;
  localparam logic [17:0] YA_INIT = dvbs2_tables_pkg::PLS_YA_INIT;
  localparam logic [17:0] YB_INIT = dvbs2_tables_pkg::PLS_YB_INIT;

  // ---------------------------------------------------------------- config
  logic [89:0] hdr_d, hdr_z;
  always_ff @(posedge clk) begin
    hdr_d <= plheader(modcod, pilots);
    hdr_z <= plheader(5'd0, 1'b0);
  end
  assign hdiff = hdr_d ^ hdr_z;

  // ---------------------------------------------------------------- units
  logic               rot_v, rot_ov;
  logic signed [15:0] rot_i, rot_q, rot_oi, rot_oq;
  logic [31:0]        rot_ph;
  logic [7:0]         rot_tag, rot_otag;

  dvbs2_rx_rot #(.LUT_FILE(LUT_FILE), .TW(8)) u_rot (
    .clk, .in_v(rot_v), .in_i(rot_i), .in_q(rot_q), .in_ph(rot_ph), .in_tag(rot_tag),
    .out_v(rot_ov), .out_i(rot_oi), .out_q(rot_oq), .out_tag(rot_otag));

  logic               cor_start, cor_done;
  logic signed [49:0] cor_x, cor_y;
  logic signed [15:0] cor_ang;
  logic [31:0]        cor_mag;

  dvbs2_rx_cordic u_cordic (
    .clk, .rst, .start(cor_start), .x(cor_x), .y(cor_y), .done(cor_done), .ang(cor_ang), .mag(cor_mag));

  logic        div_start, div_done;
  logic [43:0] div_n, div_q;
  logic [35:0] div_d;

  dvbs2_udiv #(.NW(44), .DW(36)) u_div (
    .clk, .rst, .start(div_start), .n(div_n), .d(div_d), .done(div_done), .q(div_q));

  // ---------------------------------------------------------------- buffers
  logic [31:0] zbuf [640];                  // 5 banks of header s1, then z / 2 (16-bit pairs)
  logic [31:0] zrbuf [128];                 // derotated by the current FLL stage
  // seg and zbuf have their own write and read processes (registered address,
  // data, enable), so they map to block RAM instead of ~2.3k LUTs of LUTRAM
  (* ram_style = "block" *) logic [31:0] seg [2048];   // pending data symbols
  logic [31:0] seg_rd;
  logic        sw_v;                       // seg write, one clock after the symbol
  logic [10:0] sw_a;
  logic [31:0] sw_d;
  always_ff @(posedge clk) if (sw_v) seg[sw_a] <= sw_d;

  // ---------------------------------------------------------------- output FIFO
  logic [15:0] ofifo [32];
  logic [5:0]  of_wp, of_rp;
  wire  [5:0]  of_cnt = of_wp - of_rp;
  logic        of_push;
  logic [15:0] of_din;
  logic [14:0] oc_out;

  always_ff @(posedge clk) begin
    if (rst) begin
      of_wp <= '0;
      of_rp <= '0;
      m_valid <= 1'b0;
      oc_out <= '0;
      st_frames <= '0;
    end else begin
      if (of_push) begin
        ofifo[of_wp[4:0]] <= of_din;
        of_wp <= of_wp + 6'd1;
      end
      if (m_valid && m_ready) m_valid <= 1'b0;
      if ((!m_valid || m_ready) && of_cnt != 0) begin
        m_valid <= 1'b1;
        {m_llr_q, m_llr_i} <= ofifo[of_rp[4:0]];
        m_first <= (oc_out == 0);
        m_last  <= (oc_out == 15'd32399);
        of_rp <= of_rp + 6'd1;
        if (oc_out == 15'd32399) begin
          oc_out <= '0;
          st_frames <= st_frames + 16'd1;
        end else oc_out <= oc_out + 15'd1;
      end
    end
  end

  // ---------------------------------------------------------------- helpers
  function automatic logic signed [15:0] sat16(input logic signed [47:0] a);
    if (a > 48'sd32767)       sat16 = 16'sd32767;
    else if (a < -48'sd32768) sat16 = -16'sd32768;
    else                      sat16 = a[15:0];
  endfunction

  function automatic logic signed [15:0] neg16(input logic signed [15:0] a);
    neg16 = (a == -16'sd32768) ? 16'sd32767 : -a;
  endfunction

  // round(p / 2^20), saturated to int8
  function automatic logic [7:0] llr8(input logic signed [35:0] p);
    logic signed [35:0] pr;
    pr = (p + 36'sd524288) >>> 20;
    if (pr > 36'sd127)       llr8 = 8'd127;
    else if (pr < -36'sd128) llr8 = 8'h80;
    else                     llr8 = pr[7:0];
  endfunction

  // LLR pipeline (all output words go through it, in order):
  // symbol * gain -> round/saturate -> output FIFO
  logic               lp_v, lp_zero;
  logic signed [15:0] lp_i, lp_q;
  logic [17:0]        lp_g;
  logic               l1_v, l1_zero;
  logic signed [35:0] l1_pi, l1_pq;
  // pilots off, a data frame that began before the frequency acquisition is not sent
  // (model: _finish): its words go through the pipeline but are not pushed,
  // and its padding is skipped
  logic               lp_mute, l1_mute;
  logic               acquired;

  assign snap_v = lp_v && !lp_zero;
  assign snap_i = lp_i;
  assign snap_q = lp_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      l1_v    <= 1'b0;
      of_push <= 1'b0;
    end else begin
      l1_mute <= lp_mute;
      l1_v    <= lp_v;
      l1_zero <= lp_zero;
      l1_pi   <= 36'(lp_i) * 36'($signed({1'b0, lp_g}));
      l1_pq   <= 36'(lp_q) * 36'($signed({1'b0, lp_g}));
      of_push <= l1_v && !l1_mute;
      of_din  <= l1_zero ? 16'd0 : {llr8(l1_pq), llr8(l1_pi)};
    end
  end

  // ---------------------------------------------------------------- state
  typedef enum logic [5:0] {
    M_IDLE, M_START, M_INIT, M_NOFR, M_DESC, M_ROTW, M_SYM, M_SYM2, M_SYM3, M_PLL, M_PLL2, M_NEXT,
    M_FLL_ROT, M_FLL_ACC0, M_FLL_ACC1, M_FLL_ACC2, M_FLL_COR, M_FLL_CORW, M_FLL_CORW2,
    M_FLL_END, M_FLL_END2, M_FLL_NOUPD,
    M_BLK_COR, M_BLK_CORW, M_BLK_ST1, M_BLK_ST1B, M_BLK_ST2, M_BLK_ST3, M_BLK_ST3B, M_BLK_DIVW,
    M_SEG_DIV, M_SEG_DIVW, M_SEG_DIVW2, M_SEG_SET, M_SEG_EMIT, M_SEG_DONE, M_PAD, M_BLK_END,
    M_TYP1, M_TYP3, M_HZ, M_HZ2, M_DUM_COR, M_DUM_CORW, M_DUM2,
    M_SEG_FA1, M_SEG_FA2, M_SEG_FA3
  } st_t;
  st_t st, ret_pad, ret_blk, seg_cont;

  logic signed [15:0] si, sq, s1i, s1q;
  logic               ssof, sdum, sfnd;

  logic [31:0] phi, fhat, phip, fp;
  logic [3:0]  nhdr;
  logic        in_frame, typ;                // typ 1: dummy frame
  logic [15:0] p, flen;
  logic [10:0] r_body;                       // body position modulo 1476 (pilots)
  logic [17:0] xa, xb, ya, yb;
  logic [8:0]  idle;

  logic signed [23:0] Hi, Hq, Hai, Haq, Pi, Pq;
  logic [35:0]        Esum, Pe;
  logic signed [17:0] zi, zq, zai, zaq;
  logic               z_hdr, z_pil, z_dum;
  // pilots off, data symbol: soft-decision phase detector (model: PLL_SOFT_SH)
  //   e = (s1q * c(s1i) - s1i * c(s1q)) >>> SH, c = clamp to +-(2^SH - 1),
  // the products on the squarers of M_SYM2 (zi = s1q, zq = s1i)
  localparam logic signed [15:0] SOFT_C = 16'sd2047;
  logic               z_soft;
  logic signed [11:0] ci, cq;
  logic signed [17:0] pll_e;

  // estimators
  logic [31:0]        A;
  logic [35:0]        E;
  logic [1:0]         nblk;                  // known blocks averaged (saturates at 3): IIR gain 2^-nblk
  logic               stat_rst;              // acquisition: restart the A/E average (model: stat_rst)
  logic [17:0]        G;
  logic signed [15:0] theta;
  logic [31:0]        mag;
  logic [35:0]        bsum;
  logic [6:0]         blen;
  logic               blk_opens;
  logic [31:0]        a_new;
  logic [35:0]        e_new;

  // pilots: pending data segment
  logic               seg_pend, owner_ended;
  logic [10:0]        nd;
  logic signed [15:0] th0;
  logic [6:0]         len0;
  logic signed [22:0] incr;
  logic [31:0]        sph;
  logic [10:0]        sk, sk_done;
  logic               seg_rv;
  logic [31:0]        seg_rph;
  logic [14:0]        oc_push;               // words pushed for the current output frame
  logic               fmute;                 // the current output frame is not sent
  logic               smute;                 // the data frame being received began before the acquisition
  wire                w_mute = (oc_push == '0) ? smute : fmute;

  // FLL
  logic [1:0]         fst;
  logic signed [35:0] ff;
  logic [6:0]         fk, fk_got;
  logic [31:0]        fph;
  logic signed [47:0] Ri, Rq;
  logic signed [22:0] Si, Sq;
  logic signed [15:0] zki, zkq, zmi, zmq;
  logic               zm_ok;
  logic signed [38:0] pr_a, pr_b, pr_c, pr_d;

  // pipeline registers of the multi-clock steps
  logic [35:0]        sq_zi, sq_zq;
  logic signed [35:0] p_ki, p_kp;
  logic signed [33:0] p_ang;
  logic signed [35:0] f_c;
  logic [49:0]        p_a;
  logic [53:0]        p_e;
  logic [39:0]        p_aa;
  logic [43:0]        p_acg;
  logic signed [39:0] p_dl;

  // frequency acquisition and 1/1476 ambiguity correction (model:
  // FLL_ACQ_N, FLL_REACQ_N, FA_*): the pilot loop cannot tell frequencies
  // 1/1476 cycle/symbol apart, the averaged header estimates can
  // (FA_THRESH, FA_STEP, FA_AVG_CLAMP, DUMMY_K: dvbs2_rx_pkg)
  localparam logic [1:0]         ACQ_N_M1     = 2'd3;             // FLL_ACQ_N - 1
  localparam logic [3:0]         ACQ_NHDR     = 4'd2;             // FLL_ACQ_NHDR
  localparam logic [3:0]         REACQ_N      = 4'd8;
  localparam logic [3:0]         EARLY_N      = 4'd2;             // FLL_EARLY_N
  localparam logic [3:0]         EARLY_HDRS   = 4'd4;             // FLL_EARLY_HDRS
  logic signed [15:0] dl_q;
  logic signed [43:0] fa_kp;
  logic signed [11:0] fa_k;
  logic signed [31:0] fa_stepk;
  logic               fa_big, fa_jump;
  logic [1:0]         fsh;
  logic               fkick;                 // the header estimate moves the NCO
  // pilots off: PLL gears once acquired (model: PLL_GEAR, pilots on the
  // PLL is off); the gains are registered (nhdr changes only at headers,
  // the next PLL update is many clocks later)
  logic [17:0]        kp_g, ki_g;
  always_ff @(posedge clk) begin : gear
    logic [2:0] skp, ski;
    skp = '0;
    ski = '0;
    for (int g = 0; g < PLL_NGEAR; g++)
      if (nhdr >= PLL_GEAR_N[g]) begin
        skp = PLL_GEAR_KP_SH[g];
        ski = PLL_GEAR_KI_SH[g];
      end
    kp_g <= pll_kp >> skp;
    ki_g <= pll_ki >> ski;
  end
  logic [1:0]         nconf;                 // confirmed headers (banks 0 .. nconf-1)
  logic [3:0]         big;
  logic signed [31:0] favg;
  logic [4:0]         navg;
  logic [15:0]        n_fa, n_acq, n_retype;
  // acquisition from confirmed headers: the last header (bank nconf of
  // zbuf) is confirmed when the next one is detected exactly its frame
  // length later; FLL_ACQ_N of them give the first estimate (all stages'
  // correlations summed over banks 0..3)
  logic [2:0]         wb;                    // zbuf bank of the current header
  logic [2:0]         fh, fh_last;           // FLL: bank being correlated, last one
  logic               ap_v, ap_typ, prev_ok;
  logic [15:0]        since;
  // a confirmed header whose next one came at the other type's frame length
  // was of that type: its stored z is negated where the two headers differ
  // (read into the FLL); one bit per confirmed bank
  logic [3:0]         cflip;
  logic [89:0]        hdiff;
  // coherent frame type decision
  logic [47:0]        sq1 [4], sq2 [4];
  logic [48:0]        hh, hha;
  // dummy body blocks (pilots on)
  logic [6:0]         c90;
  logic signed [15:0] dth;
  logic signed [42:0] p_dd;

  assign st_freq   = fhat;
  assign st_amp    = A;
  assign st_energy = E;
  assign st_gain   = G;
  assign st_acq    = n_acq;
  assign st_fa     = n_fa;
  assign st_retype = n_retype;
  assign st_acquired = acquired;

  assign s_ready = (st == M_IDLE) && !rst;

  wire [1:0] rsc = {xb[0] ^ yb[0], xa[0] ^ ya[0]};
  wire       is_hdr   = p < 16'd90;
  wire       is_pilot = pilots && !typ && !is_hdr && r_body >= 11'd1440;
  wire [6:0] pilot_j  = 7'(r_body - 11'd1440);
  wire       hy   = typ ? hdr_z[7'd89 - 7'(p)] : hdr_d[7'd89 - 7'(p)];
  wire       h_si = hy ^ p[0];
  wire       h_sq = hy;
  wire       hya  = typ ? hdr_d[7'd89 - 7'(p)] : hdr_z[7'd89 - 7'(p)];   // the other type
  wire       ha_si = hya ^ p[0];
  // header bits at FLL index fk (decided type), for z of the stored s1
  wire       hyk  = typ ? hdr_z[7'd89 - fk] : hdr_d[7'd89 - fk];
  wire       hk_si = hyk ^ fk[0];
  wire       hk_sq = hyk;

  // |H|^2, |Ha|^2: squarers, 2 pipeline stages, always running
  always_ff @(posedge clk) begin
    sq1[0] <= 48'(48'(Hi) * 48'(Hi));
    sq1[1] <= 48'(48'(Hq) * 48'(Hq));
    sq1[2] <= 48'(48'(Hai) * 48'(Hai));
    sq1[3] <= 48'(48'(Haq) * 48'(Haq));
    sq2 <= sq1;
  end

  // zbuf: one registered write port
  logic        zw_v;
  logic [9:0]  zw_a;
  logic [31:0] zw_d;
  always_ff @(posedge clk) if (zw_v) zbuf[zw_a] <= zw_d;

  // zbuf read port: M_HZ reads the header bank, M_FLL_ROT the bank being
  // correlated. Two clocks of read latency: zb_q0 is the array output, zb_q
  // the block RAM's output register (DOB_REG), so the logic after the read
  // starts from a register instead of the RAM's slow clock-to-out. zb_q holds
  // the word two clocks after the read: the first stage of the header z
  // pipeline below (one clock after M_HZ2), or the rotator input issued from
  // zr2_* two clocks after M_FLL_ROT.
  logic [31:0] zb_q0, zb_q;
  logic        zr_v, zr2_v;
  logic [31:0] zr_ph, zr2_ph;
  logic [6:0]  zr_tag, zr2_tag;
  wire  [9:0]  zb_ra = (st == M_HZ) ? 10'({wb, fk}) : 10'({fh, fk});
  always_ff @(posedge clk) begin
    zb_q0 <= zbuf[zb_ra];
    zb_q  <= zb_q0;
  end
  // header z pipeline (stored s1 -> z / 2 of the decided type, written back
  // in place): M_HZ2 starts one word, hz1 = zb_q valid, hz2 = z valid, then
  // zw_*. It runs beside the FSM, so the last word or two are still in
  // flight during the first clocks of M_FLL_ROT / M_FLL_NOUPD. That is safe:
  // M_FLL_ROT reads that bank from k = 0 up (the words written last, k = 88
  // and 89, are read ~88 clocks later), and the next zbuf write from M_SYM2
  // is many clocks away.
  logic               hz1_v, hz1_si, hz1_sq, hz2_v;
  logic [9:0]         hz1_a, hz2_a;
  logic signed [17:0] hz2_i, hz2_q;
  logic [5:0] inflight;                      // segment words issued, not yet out of the rotator
  // seg read port: the word M_SEG_EMIT issues (same condition as there)
  wire seg_issue = (st == M_SEG_EMIT) && (sk != nd) && (6'(of_cnt) + inflight < 6'd22);
  always_ff @(posedge clk) if (seg_issue) seg_rd <= seg[sk];
  wire        room1 = of_cnt < 6'd26;            // FIFO 32 - LLR pipeline in flight

  always_ff @(posedge clk) begin
    rot_v     <= 1'b0;
    cor_start <= 1'b0;
    div_start <= 1'b0;
    lp_v      <= 1'b0;
    seg_rv    <= 1'b0;
    ty_v      <= 1'b0;
    zw_v      <= 1'b0;
    sw_v      <= 1'b0;
    zr_v      <= 1'b0;
    hz1_v     <= 1'b0;

    if (rst) begin
      st <= M_IDLE;
      phi <= '0; fhat <= '0; phip <= '0; fp <= '0;
      nhdr <= '0;
      in_frame <= 1'b0;
      typ <= 1'b1;
      idle <= '0;
      A <= '0; E <= '0; nblk <= '0; G <= '0; stat_rst <= 1'b0;
      seg_pend <= 1'b0; owner_ended <= 1'b0;
      th0 <= '0; len0 <= '0;
      oc_push <= '0;
      fmute <= 1'b0;
      smute <= 1'b1;
      acquired <= 1'b0;
      nconf <= '0;
      big <= '0;
      favg <= '0;
      navg <= '0;
      n_fa <= '0;
      n_acq <= '0;
      n_retype <= '0;
      wb <= '0;
      ap_v <= 1'b0;
      prev_ok <= 1'b0;
      cflip <= '0;
      since <= '0;
      dth <= '0;
    end else begin
      case (st)
        // ------------------------------------------------------------ input
        M_IDLE: if (s_valid) begin
          si <= s_i; sq <= s_q; ssof <= s_sof; sdum <= s_dummy; sfnd <= s_found;
          st <= M_START;
        end

        M_START: begin
          // acquisition: the last header is confirmed by this one
          if (ssof) begin : conf
            logic ok, other;
            // either frame length confirms it; the length says the type
            // (before the acquisition the tag's type is the frame sync's
            // differential guess)
            other = since == (ap_typ ? (pilots ? FLEN_DATA_P : FLEN_DATA_N) : FLEN_DUMMY);
            ok = ap_v && sfnd && (other || since == (ap_typ ? FLEN_DUMMY : (pilots ? FLEN_DATA_P : FLEN_DATA_N)));
            prev_ok <= ok;
            if (ok) cflip[nconf] <= other;
            // bank of this header: after the confirmed ones (acquired: 0)
            wb <= acquired ? 3'd0 : 3'(nconf) + 3'(ok);
            since <= 16'd1;
          end else if (since != 16'hFFFF) since <= since + 16'd1;
          if (ssof) begin
            if (in_frame && !typ) begin
              // the data frame being received is cut short
              if (pilots && seg_pend) begin
                owner_ended <= 1'b1;
                st <= M_INIT;
              end else begin
                ret_pad <= M_INIT;
                st <= M_PAD;
              end
            end else st <= M_INIT;
          end else st <= in_frame ? M_DESC : M_NOFR;
        end

        M_INIT: begin
          in_frame <= 1'b1;
          smute <= !acquired && !pilots;
          typ  <= sdum;
          flen <= sdum ? FLEN_DUMMY : (pilots ? FLEN_DATA_P : FLEN_DATA_N);
          p <= '0;
          r_body <= '0;
          Hi <= '0; Hq <= '0; Hai <= '0; Haq <= '0; Esum <= '0;
          c90 <= '0;
          Pi <= '0; Pq <= '0; Pe <= '0;
          xa <= gold_xa; xb <= gold_xb; ya <= YA_INIT; yb <= YB_INIT;
          st <= M_DESC;
        end

        M_NOFR: begin
          idle <= idle + 9'd1;
          phi  <= phi + fhat;
          phip <= phip + fp;
          if (pilots && seg_pend && idle + 9'd1 == SEG_TIMEOUT) begin
            incr     <= '0;
            seg_cont <= M_IDLE;
            st <= M_SEG_SET;
          end else st <= M_IDLE;
        end

        // ------------------------------------------------------------ one symbol
        M_DESC: begin : desc
          logic signed [15:0] di, dq;
          di = si; dq = sq;
          if (!is_hdr) begin
            case (rsc)                       // * j^-r
              2'd0:    begin di = si;        dq = sq;        end
              2'd1:    begin di = sq;        dq = neg16(si); end
              2'd2:    begin di = neg16(si); dq = neg16(sq); end
              default: begin di = neg16(sq); dq = si;        end
            endcase
            xa <= {xa[0] ^ xa[7], xa[17:1]};
            xb <= {xb[0] ^ xb[7], xb[17:1]};
            ya <= {ya[0] ^ ya[5] ^ ya[7] ^ ya[10], ya[17:1]};
            yb <= {yb[0] ^ yb[5] ^ yb[7] ^ yb[10], yb[17:1]};
          end
          rot_v   <= 1'b1;
          rot_i   <= di;
          rot_q   <= dq;
          rot_ph  <= -(phi + phip);
          rot_tag <= 8'd0;
          phi <= phi + fhat;
          st <= M_ROTW;
        end

        M_ROTW: if (rot_ov) begin
          s1i <= rot_oi;
          s1q <= rot_oq;
          st <= M_SYM;
        end

        M_SYM: begin
          z_hdr <= 1'b0;
          z_pil <= 1'b0;
          z_dum <= 1'b0;
          z_soft <= 1'b0;
          pll_e <= '0;
          if (is_hdr) begin : hdr
            logic signed [17:0] a_i, a_q;
            // z = s1 * conj(h) * sqrt2 = s1 * (a - jb), a, b = +-1
            a_i = h_si ? -18'(s1i) : 18'(s1i);
            a_q = h_si ? -18'(s1q) : 18'(s1q);
            zi <= a_i + (h_sq ? -18'(s1q) : 18'(s1q));
            zq <= a_q - (h_sq ? -18'(s1i) : 18'(s1i));
            // the same for the other frame type's header
            a_i = ha_si ? -18'(s1i) : 18'(s1i);
            a_q = ha_si ? -18'(s1q) : 18'(s1q);
            zai <= a_i + (hya ? -18'(s1q) : 18'(s1q));
            zaq <= a_q - (hya ? -18'(s1i) : 18'(s1i));
            z_hdr <= 1'b1;
            st <= M_SYM2;
          end else if (typ) begin
            pll_e <= 18'(s1q) - 18'(s1i);   // dummy body: (1+j)/sqrt2
            zi <= 18'(s1i) + 18'(s1q);       // z = s1 * (1 - j)
            zq <= 18'(s1q) - 18'(s1i);
            z_dum <= pilots && acquired;
            st <= M_SYM2;
          end else if (is_pilot) begin
            zi <= 18'(s1i) + 18'(s1q);       // z = s1 * (1 - j)
            zq <= 18'(s1q) - 18'(s1i);
            z_pil <= 1'b1;
            st <= M_SYM2;
          end else if (pilots) begin
            sw_v <= 1'b1; sw_a <= nd; sw_d <= {s1q, s1i};
            nd <= nd + 11'd1;
            st <= M_SYM2;
          end else if (room1) begin
            lp_v    <= 1'b1;
            lp_zero <= 1'b0;
            lp_i    <= s1i;
            lp_q    <= s1q;
            lp_g    <= G;
            oc_push <= oc_push + 15'd1;
            lp_mute <= w_mute;
            if (oc_push == '0) fmute <= smute;
            zi <= 18'(s1q);
            zq <= 18'(s1i);
            ci <= s1i > SOFT_C ? 12'(SOFT_C) : s1i < -SOFT_C ? 12'(-SOFT_C) : 12'(s1i);
            cq <= s1q > SOFT_C ? 12'(SOFT_C) : s1q < -SOFT_C ? 12'(-SOFT_C) : 12'(s1q);
            z_soft <= 1'b1;
            st <= M_SYM2;
          end
        end

        M_SYM2: begin
          sq_zi <= 36'(36'(zi) * (z_soft ? 36'(ci) : 36'(zi)));
          sq_zq <= 36'(36'(zq) * (z_soft ? 36'(cq) : 36'(zq)));
          if (z_hdr) begin
            zw_v <= 1'b1; zw_a <= 10'({wb, 7'(p)}); zw_d <= {s1q, s1i};
            Hi <= Hi + 24'(zi);
            Hq <= Hq + 24'(zq);
            Hai <= Hai + 24'(zai);
            Haq <= Haq + 24'(zaq);
            pll_e <= zq;
          end
          if (z_pil || z_dum) begin
            Pi <= Pi + 24'(zi);
            Pq <= Pq + 24'(zq);
          end
          st <= M_SYM3;
        end

        M_SYM3: begin : sym3
          logic [35:0] e2;
          e2 = 36'((37'(sq_zi) + 37'(sq_zq)) >> 8);
          if (z_hdr) Esum <= Esum + e2;
          if (z_pil) Pe <= Pe + e2;
          if (z_soft) pll_e <= 18'(($signed(sq_zi) - $signed(sq_zq)) >>> PLL_SOFT_SH);
          st <= M_PLL;
        end

        M_PLL: begin : pll
          logic signed [17:0] e;
          e = (pilots || !acquired) ? 18'sd0 : pll_e;
          p_ki <= 36'(e) * 36'($signed({1'b0, ki_g}));
          p_kp <= 36'(e) * 36'($signed({1'b0, kp_g}));
          st <= M_PLL2;
        end

        M_PLL2: begin : pll2
          logic [31:0] fpn;
          fpn = fp + 32'(p_ki >>> 16);
          fp  <= fpn;
          phip <= phip + fpn + 32'(p_kp >>> 8);
          if (is_hdr && p == 16'd89) begin
            st <= M_TYP1;
          end else if (z_dum && c90 == 7'd89) begin
            cor_x <= 50'(Pi);
            cor_y <= 50'(Pq);
            st <= M_DUM_COR;
          end else if (is_pilot && pilot_j == 7'd35) begin
            cor_x <= 50'(Pi);
            cor_y <= 50'(Pq);
            bsum  <= Pe;
            blen  <= 7'd36;
            blk_opens <= 1'b1;
            ret_blk <= M_NEXT;
            st <= M_BLK_COR;
          end else st <= M_NEXT;
        end

        M_NEXT: begin
          p <= p + 16'd1;
          if (!is_hdr) begin
            r_body <= (r_body == 11'd1475) ? 11'd0 : r_body + 11'd1;
            c90 <= (c90 == 7'd89) ? 7'd0 : c90 + 7'd1;
          end
          if (p + 16'd1 == flen) begin
            in_frame <= 1'b0;
            idle <= '0;
            if (!typ) begin
              if (pilots && seg_pend) begin
                owner_ended <= 1'b1;
                st <= M_IDLE;
              end else begin
                ret_pad <= M_IDLE;
                st <= M_PAD;
              end
            end else st <= M_IDLE;
          end else st <= M_IDLE;
        end

        // ------------------------------------------------------------ header type (model: _header_type)
        M_TYP1: begin
          // squares of the header sums (pipeline sq*, valid 2 clocks after M_SYM2)
          hh   <= 49'(sq2[0]) + 49'(sq2[1]);
          hha  <= 49'(sq2[2]) + 49'(sq2[3]);
          st <= M_TYP3;
        end

        M_TYP3: begin : typ3
          logic nt;
          nt = typ;
          // acquired: the type whose header correlates better (coherently)
          if (acquired && hha > hh) begin
            nt = !typ;
            Hi <= Hai; Hq <= Haq;
            n_retype <= n_retype + 16'd1;
          end
          typ  <= nt;
          flen <= nt ? FLEN_DUMMY : (pilots ? FLEN_DATA_P : FLEN_DATA_N);
          ty_v <= 1'b1;
          ty_dummy <= nt;
          fk <= '0;
          st <= M_HZ;
        end

        // stored s1 -> z / 2 of the decided type, in place
        M_HZ: st <= M_HZ2;                  // read {wb, fk}

        M_HZ2: begin
          // the word read in M_HZ goes through the header z pipeline (below)
          hz1_v  <= 1'b1;
          hz1_si <= hk_si;
          hz1_sq <= hk_sq;
          hz1_a  <= 10'({wb, fk});
          if (fk == 7'd89) begin
            fst <= '0;
            ff  <= '0;
            fk  <= '0;
            fk_got <= '0;
            fph <= '0;
            if (acquired) begin
              fh <= wb;                      // this header
              fh_last <= wb;
              st <= M_FLL_ROT;
            end else if (prev_ok && nconf == ACQ_N_M1) begin
              fh <= '0;                      // the confirmed headers
              fh_last <= 3'd3;
              st <= M_FLL_ROT;
            end else begin
              // acquisition: keep this header (bank wb) until it is confirmed
              if (prev_ok) nconf <= nconf + 2'd1;
              ap_v   <= 1'b1;
              ap_typ <= typ;
              f_c <= '0;
              st  <= M_FLL_NOUPD;
            end
          end else begin
            fk <= fk + 7'd1;
            st <= M_HZ;
          end
        end

        // ------------------------------------------------------------ dummy body block (pilots, acquired)
        M_DUM_COR: begin
          cor_start <= 1'b1;
          st <= M_DUM_CORW;
        end

        M_DUM_CORW: if (cor_done) begin : dcw
          logic signed [15:0] dl;
          dl = cor_ang - dth;
          dth <= cor_ang;
          p_dd <= 43'(dl) * 43'($signed({1'b0, DUMMY_K}));
          Pi <= '0; Pq <= '0;
          st <= M_DUM2;
        end

        M_DUM2: begin : dum2
          logic signed [31:0] d;
          d = 32'(p_dd >>> 19);              // (dl * 2^32/90 >> 16) >> DUMMY_SH
          fhat <= fhat + d;
          favg <= favg - d;
          st <= M_NEXT;
        end

        // ------------------------------------------------------------ header frequency estimate
        M_FLL_ROT: begin
          // derotate zbuf by the estimate so far, one per clock
          if (fk != 7'd90) begin
            zr_v    <= 1'b1;
            zr_ph   <= fph;
            zr_tag  <= fk;
            fph <= fph - ff[31:0];
            fk  <= fk + 7'd1;
          end
          if (rot_ov) begin
            zrbuf[rot_otag[6:0]] <= {rot_oq, rot_oi};
            fk_got <= fk_got + 7'd1;
            if (fk_got == 7'd89) st <= M_FLL_ACC0;
          end
        end

        M_FLL_ACC0: begin
          // R summed over the banks of the stage, S per header
          if (fh == wb && acquired || fh == 3'd0 && !acquired) begin
            Ri <= '0; Rq <= '0;
          end
          Si <= '0; Sq <= '0;
          fk <= '0;
          st <= M_FLL_ACC1;
        end

        M_FLL_ACC1: begin : acc1
          logic signed [15:0] ai, aq;
          logic [6:0]         km;
          ai = $signed(zrbuf[fk][15:0]);
          aq = $signed(zrbuf[fk][31:16]);
          km = fk - 7'(FLL_M[fst]);
          zki <= ai; zkq <= aq;
          zmi <= $signed(zrbuf[km][15:0]);
          zmq <= $signed(zrbuf[km][31:16]);
          zm_ok <= fk >= 7'(FLL_M[fst]);
          // R += z_k * conj(S_k)
          pr_a <= 39'(ai) * 39'(Si);
          pr_b <= 39'(aq) * 39'(Sq);
          pr_c <= 39'(aq) * 39'(Si);
          pr_d <= 39'(ai) * 39'(Sq);
          st <= M_FLL_ACC2;
        end

        M_FLL_ACC2: begin
          Ri <= Ri + 48'(pr_a) + 48'(pr_b);
          Rq <= Rq + 48'(pr_c) - 48'(pr_d);
          Si <= Si + 23'(zki) - (zm_ok ? 23'(zmi) : 23'sd0);
          Sq <= Sq + 23'(zkq) - (zm_ok ? 23'(zmq) : 23'sd0);
          if (fk == 7'd89) begin
            if (fh == fh_last) st <= M_FLL_COR;
            else begin
              fh <= fh + 3'd1;
              fk <= '0;
              fk_got <= '0;
              fph <= '0;
              st <= M_FLL_ROT;
            end
          end else begin
            fk <= fk + 7'd1;
            st <= M_FLL_ACC1;
          end
        end

        M_FLL_COR: begin
          cor_x <= 50'(Ri);
          cor_y <= 50'(Rq);
          cor_start <= 1'b1;
          st <= M_FLL_CORW;
        end

        M_FLL_CORW: if (cor_done) begin
          p_ang <= 34'(cor_ang) * 34'($signed({1'b0, FLL_MUL[fst]}));
          st <= M_FLL_CORW2;
        end

        M_FLL_CORW2: begin
          ff <= ff + 36'(p_ang);
          if (fst == 2'd2) st <= M_FLL_END;
          else begin
            fst <= fst + 2'd1;
            fh <= acquired ? wb : 3'd0;
            fk <= '0;
            fk_got <= '0;
            fph <= '0;
            st <= M_FLL_ROT;
          end
        end

        M_FLL_END: begin : fend
          logic signed [35:0] f, fa;
          logic [4:0]         na;
          logic [2:0]         ash;
          f = ff;
          if (!acquired) begin
            // acquisition done: full update
            acquired <= 1'b1;
            stat_rst <= 1'b1;
            ap_v <= 1'b0;
            nhdr <= ACQ_NHDR;
            nconf <= '0;
            big <= '0;
            favg <= '0;
            navg <= '0;
            n_acq <= n_acq + 16'd1;
            f_c <= f;
            fsh <= '0;
            fkick <= 1'b1;
            st <= M_FLL_END2;
          end else if ((f > 36'sd4194304 || f < -36'sd4194304) &&
                       (big == REACQ_N - 4'd1 || (pilots && big == EARLY_N - 4'd1 && nhdr <= EARLY_HDRS))) begin
            // the estimates keep saying we are far off (or right after the
            // acquisition: it was bad): acquire again
            acquired <= 1'b0;
            big <= '0;
            f_c <= '0;
            st <= M_FLL_NOUPD;
          end else begin
            big <= (f > 36'sd4194304 || f < -36'sd4194304) ? big + 4'd1 : 4'd0;
            fa = f > FA_AVG_CLAMP ? FA_AVG_CLAMP : f < -FA_AVG_CLAMP ? -FA_AVG_CLAMP : f;
            // gain 2^-floor(log2(n)), n = headers averaged (1..16)
            na = navg == 5'd16 ? 5'd16 : navg + 5'd1;
            ash = na[4] ? 3'd4 : na[3] ? 3'd3 : na[2] ? 3'd2 : na[1] ? 3'd1 : 3'd0;
            navg <= na;
            favg <= favg + 32'((fa - 36'(favg)) >>> ash);
            if (nhdr > 4'd3) begin
              if (f > 36'sd4194304)       f = 36'sd4194304;
              else if (f < -36'sd4194304) f = -36'sd4194304;
            end
            f_c <= f;
            fsh <= nhdr > 4'd3 ? 2'd3 : nhdr[1:0];
            // pilots off: none after the acquisition (model: PLL_KICK_NHDR,
            // nhdr >= 2 here), the PLL integrator is the estimate
            fkick <= pilots || nhdr < PLL_KICK_NHDR;
            if (nhdr != 4'hF) nhdr <= nhdr + 4'd1;
            st <= M_FLL_END2;
          end
        end

        M_FLL_END2, M_FLL_NOUPD: begin
          ff   <= f_c;
          // update with gain 2^-fsh (0 for the acquisition update); pilots
          // off: the PLL's frequency moves into the NCO (same total), so
          // fhat does not random-walk against fp
          if (!pilots) begin
            fhat <= fhat + (st == M_FLL_END2 && fkick ? 32'(f_c >>> fsh) : 32'd0) + fp;
            fp   <= '0;
          end else if (st == M_FLL_END2) fhat <= fhat + 32'(f_c >>> fsh);
          cor_x <= 50'(Hi);
          cor_y <= 50'(Hq);
          bsum  <= Esum;
          blen  <= 7'd90;
          blk_opens <= pilots && !typ;
          ret_blk <= M_NEXT;
          st <= M_BLK_COR;
        end

        // ------------------------------------------------------------ known block end
        M_BLK_COR: begin
          cor_start <= 1'b1;
          st <= M_BLK_CORW;
        end

        M_BLK_CORW: if (cor_done) begin
          theta <= cor_ang;
          mag   <= cor_mag;
          st <= M_BLK_ST1;
        end

        M_BLK_ST1: begin
          p_a <= 50'(mag) * 50'(blen == 7'd36 ? STAT_RA36 : STAT_RA90);
          p_e <= 54'(bsum) * 54'(blen == 7'd36 ? STAT_RE36 : STAT_RE90);
          st <= M_BLK_ST1B;
        end

        M_BLK_ST1B: begin
          a_new <= 32'(p_a >> 24);
          e_new <= 36'(p_e >> 16);
          st <= M_BLK_ST2;
        end

        M_BLK_ST2: begin
          if (stat_rst) begin
            // the acquisition header (summed before the new frequency) is
            // skipped, the next known block starts the average again
            stat_rst <= 1'b0;
            nblk <= '0;
          end else begin
            // gain 1, 1/2, 1/4, then 1/8 (model: _stats)
            A <= 32'($signed({1'b0, A}) + (($signed({1'b0, a_new}) - $signed({1'b0, A})) >>> nblk));
            E <= 36'($signed({1'b0, E}) + (($signed({1'b0, e_new}) - $signed({1'b0, E})) >>> nblk));
            if (nblk != 2'd3) nblk <= nblk + 2'd1;
          end
          st <= M_BLK_ST3;
        end

        M_BLK_ST3: begin
          // A < 2^19 (|P| < 2^25.3, RA < 2^17.7)
          p_aa  <= 40'(A[19:0]) * 40'(A[19:0]);
          p_acg <= 44'(A[19:0]) * 44'(LLR_CG);
          st <= M_BLK_ST3B;
        end

        M_BLK_ST3B: begin : st3
          logic signed [41:0] s2;
          s2 = 42'(E) - 42'(p_aa);
          if (s2 < 42'sd1024) s2 = 42'sd1024;
          div_n <= p_acg;
          div_d <= 36'(s2);
          div_start <= 1'b1;
          st <= M_BLK_DIVW;
        end

        M_BLK_DIVW: if (div_done) begin
          G <= (div_q > 44'h3FFFF) ? 18'h3FFFF : div_q[17:0];
          if (pilots && seg_pend) begin
            seg_cont <= M_BLK_END;
            st <= M_SEG_DIV;
          end else st <= M_BLK_END;
        end

        M_BLK_END: begin
          // pilots off: the PLHEADER phase resets the PLL phase (not
          // extrapolated: the PLL has removed the residual frequency)
          if (!pilots && blen == 7'd90) phip <= phip + {theta, 16'd0};
          if (blen == 7'd90) dth <= theta;
          if (pilots) begin
            th0  <= theta;
            len0 <= blen;
            if (blk_opens) begin
              seg_pend <= 1'b1;
              owner_ended <= 1'b0;
              nd <= '0;
            end
          end
          Pi <= '0; Pq <= '0; Pe <= '0;
          st <= ret_blk;
        end

        // ------------------------------------------------------------ data segment (pilots)
        M_SEG_DIV: begin
          div_n <= 44'h1_0000_0000;
          div_d <= 36'(nd) + 36'((len0 + blen) >> 1);
          div_start <= 1'b1;
          st <= M_SEG_DIVW;
        end

        M_SEG_DIVW: if (div_done) begin : sdw
          logic signed [15:0] dl;
          dl  = theta - th0;
          dl_q <= dl;
          p_dl <= 40'(dl) * 40'($signed({1'b0, div_q[23:0]}));
          st <= M_SEG_DIVW2;
        end

        // 1/1476 correction, 4 steps (model: Carrier._close_segment)
        M_SEG_DIVW2: begin
          incr <= 23'(p_dl >>> 16);
          fa_kp <= 44'(favg) * 44'sd1476;
          fa_big <= (dl_q > FA_THRESH || dl_q < -FA_THRESH) && favg != 0;
          st <= M_SEG_FA1;
        end

        M_SEG_FA1: begin
          // k = round(favg * 1476 / 2^32)
          fa_k <= 12'((fa_kp + 44'sd2147483648) >>> 32);
          st <= M_SEG_FA2;
        end

        M_SEG_FA2: begin
          fa_stepk <= 32'(fa_k) * FA_STEP;
          // odd k (783-symbol residual near half a turn): sign of the average; |k| >= 2: k
          fa_jump <= len0 == 7'd36 && blen == 7'd90 && (fa_big || fa_k >= 12'sd2 || fa_k <= -12'sd2);
          st <= M_SEG_FA3;
        end

        M_SEG_FA3: begin : fa3
          logic signed [31:0] step;
          step = fa_big ? (favg > 0 ? FA_STEP : -FA_STEP) : fa_stepk;
          if (acquired) begin
            if (fa_jump) begin
              fhat <= fhat + 32'(step);
              favg <= favg - step;
              n_fa <= n_fa + 16'd1;
            end else fhat <= fhat + 32'(incr >>> 2);
          end
          st <= M_SEG_SET;
        end

        M_SEG_SET: begin : sset
          logic signed [29:0] l1, t;
          l1 = 30'($signed({1'b0, len0})) + 30'sd1;
          t  = (30'(incr) * l1) >>> 1;
          sph <= {th0, 16'd0} + 32'(t);
          sk <= '0;
          sk_done <= '0;
          inflight <= '0;
          st <= M_SEG_EMIT;
        end

        M_SEG_EMIT: begin : emit
          logic issue, push;
          issue = (sk != nd) && (6'(of_cnt) + inflight < 6'd22);
          push  = rot_ov && rot_otag == 8'h80;
          if (issue) begin
            seg_rv  <= 1'b1;
            seg_rph <= -sph;
            sph <= sph + 32'(incr);
            sk  <= sk + 11'd1;
          end
          if (seg_rv) begin
            rot_v   <= 1'b1;
            rot_i   <= $signed(seg_rd[15:0]);
            rot_q   <= $signed(seg_rd[31:16]);
            rot_ph  <= seg_rph;
            rot_tag <= 8'h80;
          end
          if (push) begin
            lp_v    <= 1'b1;
            lp_zero <= 1'b0;
            lp_i    <= rot_oi;
            lp_q    <= rot_oq;
            lp_g    <= G;
            oc_push <= oc_push + 15'd1;
            lp_mute <= w_mute;
            if (oc_push == '0) fmute <= smute;
            sk_done <= sk_done + 11'd1;
          end
          inflight <= inflight + 6'(issue) - 6'(push);
          if (push && sk_done + 11'd1 == nd || nd == 0) st <= M_SEG_DONE;
        end

        M_SEG_DONE: begin
          seg_pend <= 1'b0;
          if (owner_ended) begin
            ret_pad <= seg_cont;
            st <= M_PAD;
          end else st <= seg_cont;
        end

        // ------------------------------------------------------------ finish an output frame
        M_PAD: begin
          if (oc_push == 15'd32400 || w_mute) begin
            // (a frame that is not sent needs no padding)
            oc_push <= '0;
            st <= ret_pad;
          end else if (room1) begin
            lp_v    <= 1'b1;
            lp_zero <= 1'b1;
            oc_push <= oc_push + 15'd1;
            lp_mute <= w_mute;
            if (oc_push == '0) fmute <= smute;
          end
        end

        default: st <= M_IDLE;
      endcase
    end
    // M_FLL_ROT: the zbuf word read two clocks earlier goes to the rotator
    zr2_v   <= zr_v;
    zr2_ph  <= zr_ph;
    zr2_tag <= zr_tag;
    if (zr2_v) begin : zr2
      logic neg;
      neg = !acquired && cflip[fh[1:0]] && hdiff[7'd89 - zr2_tag];
      rot_v   <= 1'b1;
      rot_i   <= neg ? neg16(zb_q[15:0])  : $signed(zb_q[15:0]);
      rot_q   <= neg ? neg16(zb_q[31:16]) : $signed(zb_q[31:16]);
      rot_ph  <= zr2_ph;
      rot_tag <= {1'b0, zr2_tag};
    end
    // header z pipeline, stage 1: z = s1 * conj(h) of the stored s1 (zb_q)
    hz2_v <= hz1_v;
    hz2_a <= hz1_a;
    if (hz1_v) begin : hzp1
      logic signed [17:0] a_i, a_q;
      a_i = hz1_si ? -18'($signed(zb_q[15:0]))  : 18'($signed(zb_q[15:0]));
      a_q = hz1_si ? -18'($signed(zb_q[31:16])) : 18'($signed(zb_q[31:16]));
      hz2_i <= a_i + (hz1_sq ? -18'($signed(zb_q[31:16])) : 18'($signed(zb_q[31:16])));
      hz2_q <= a_q - (hz1_sq ? -18'($signed(zb_q[15:0]))  : 18'($signed(zb_q[15:0])));
    end
    // stage 2: z / 2 rounded, saturated, written back in place
    if (hz2_v) begin
      zw_v <= 1'b1; zw_a <= hz2_a;
      zw_d <= {sat16((48'(hz2_q) + 48'sd1) >>> 1), sat16((48'(hz2_i) + 48'sd1) >>> 1)};
    end
  end

endmodule
