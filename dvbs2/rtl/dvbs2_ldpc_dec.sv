// DVB-S2 LDPC decoder, normal FECFRAMEs, all 11 rates (model: model/ldpcdec.py).
//
// Layered offset min-sum, P = 18 lanes, edges serial; see the model for the
// schedule. Two APP memories (18 lanes x 4096 x 9 bit each) ping-pong between
// the LLR input and the decoder. Output: the first Kldpc hard decisions
// (BB-scrambled BBFRAME + BCH parity, for dvbs2_bch_dec) as bytes, MSB
// first; m_ok on the last byte says whether the LDPC decoder converged;
// m_chan: the channel's hard decisions (input LLR signs) of the same bits.
//
// Input: 32400 words of two LLRs (bit 2i from I, 2i+1 from Q) per frame, as
// produced by dvbs2_rx_carrier; s_first restarts a frame.
//
// Clocks per iteration: sum over the q*S sub-steps of 2*deg + ~14, about
// 50k for every rate (e.g. 1/2: 1800 sub-steps x 28).

module dvbs2_ldpc_dec
  import dvbs2_ldpc_dec_pkg::*;
#(
  parameter string ROM_FILE = "ldpc_dec_rom.mem"
) (
  input  logic        clk,
  input  logic        rst,

  input  logic [3:0]  rate_idx,      // static while running
  input  logic [5:0]  max_it,

  input  logic        s_valid,
  output logic        s_ready,
  input  logic [7:0]  s_llr_i,
  input  logic [7:0]  s_llr_q,
  input  logic        s_first,

  output logic        m_valid,
  input  logic        m_ready,
  output logic [7:0]  m_data,
  output logic        m_last,
  output logic        m_ok,
  output logic [5:0]  m_iters,
  output logic [7:0]  m_chan,

  output logic [15:0] st_frames,
  output logic [15:0] st_failed
);

  localparam int P  = LD_P;
  localparam int S  = LD_S;
  localparam int W  = 9;
  localparam int AW = 12;
  localparam int SD = 15840;         // sign memory depth: max q * S * deg

  // ------------------------------------------------------------ rate parameters
  logic [7:0]  q, ng;
  logic [4:0]  deg;
  logic [12:0] rbase;
  logic [15:0] kbch, kldpc;
  always_ff @(posedge clk) begin
    q     <= 8'(LD_Q[rate_idx]);
    ng    <= 8'(LD_NG[rate_idx]);
    deg   <= 5'(LD_DEG[rate_idx]);
    rbase <= 13'(LD_BASE[rate_idx]);
    kbch  <= LD_KBCH[rate_idx];
    kldpc <= LD_KLDPC[rate_idx];
  end

  function automatic logic [AW-1:0] gs(input logic [7:0] g);   // g * S
    gs = AW'({g, 4'd0}) + AW'({g, 2'd0});
  endfunction

  function automatic logic [5:0] norm(input logic [5:0] x);
    norm = (x > 6'(LD_OFFSET)) ? x - 6'(LD_OFFSET) : 6'd0;
  endfunction

  function automatic logic signed [W-1:0] sat9(input logic signed [10:0] x);
    if (x > 11'sd255)       sat9 = 9'sd255;
    else if (x < -11'sd256) sat9 = -9'sd256;
    else                    sat9 = x[W-1:0];
  endfunction

  // ------------------------------------------------------------ declarations
  // APP memories: copy c, lane l; one write port (lane enables), one read port
  logic [P-1:0]  l_we    [2];
  logic [AW-1:0] l_waddr [2];
  logic [W-1:0]  l_wdata [2][P];
  logic [AW-1:0] l_raddr [2];
  logic [W-1:0]  l_rdata [2][P];

  logic [17:0] erom [LD_ROM_DEPTH];          // {k0[4:0], w0[4:0], G[7:0]}
  initial $readmemh(ROM_FILE, erom);
  logic [17:0] erom_q;
  // R memory: 9 slices of 2 lanes (36 bits) -> 1024 x 36 BRAMs
  logic [P*18-1:0] rmem_q, rmem_d;
  logic            rmem_we;
  (* ram_style = "block" *) logic [P-1:0] smem [SD];
  logic [P-1:0] smem_q;
  (* ram_style = "distributed" *) logic [P*8-1:0] fifo [32];   // phase A -> B: {sign, R_old[6:0]} / lane

  typedef enum logic [1:0] {B_FREE, B_FULL, B_DEC} bst_t;
  bst_t bst [2];
  logic ib, db;
  logic in_done, dec_done;

  typedef enum logic [3:0] {D_IDLE, D_ITER, D_LOAD, D_SUB, D_SUB2, D_A, D_AEND, D_B, D_NEXT,
                            D_ITEREND, D_OUT, D_OUT2, D_DONE} dst_t;
  dst_t dst;

  // input writer
  logic          iw_phase;
  logic [7:0]    iw_q;
  logic [16:0]   iw_n;
  logic          iw_par;
  logic [7:0]    iw_g;
  logic [8:0]    iw_m;
  logic [4:0]    iw_w, iw_lane;
  logic [AW-1:0] iw_base;
  logic          iw_we;
  logic [4:0]    iw_wlane;
  logic [AW-1:0] iw_waddr;
  logic [W-1:0]  iw_wdata;

  // decoder
  logic [5:0]  it;
  logic        conv, first_it;
  logic        unsat, flips;
  logic [7:0]  j;
  logic [4:0]  b, e, ld_e;
  logic [11:0] r_addr;
  logic [13:0] sg_base;
  logic [13:0] sg_wa5;
  logic [12:0] ed_base;
  logic [7:0]  ed_g  [32];
  logic [4:0]  ed_w0 [32], ed_k0 [32];
  logic [5:0]  o_m1 [P], o_m2 [P], n_m1 [P], n_m2 [P];
  (* max_fanout = 24 *) logic o_ld;            // D_SUB2: load o_* from the R memory
  logic [6:0]  a_m1 [P], a_m2 [P];
  logic [4:0]  o_ix [P], n_ix [P], a_ix [P];
  logic        o_st [P], n_st [P], a_st [P];
  logic        hpar [P];

  logic          phase_b;
  logic          v2, v3, v4, v5, v6;
  logic [4:0]    e2, e3, e4, e5;
  logic [4:0]    k2, k3, k4, k5;
  logic          inv2, inv3, inv4, inv5;
  logic [AW-1:0] a2, a3, a4, a5, a6;
  logic [P-1:0]  sg3, sg4;
  logic signed [W-1:0] lv4 [P], q5 [P], ro5 [P], nw5 [P], lvp5 [P];
  logic [P-1:0]  hs5;
  logic [W-1:0]  wd6 [P];
  logic [4:0]    fifo_wp, fifo_rp;
  // APP read address. It loads every clock from registers only (no enable,
  // one 2:1 mux), because it fans out to the 36 APP memories across the die:
  // ia0 in the decode phases, o_addr in D_OUT (rd_out). The words read while
  // neither issues are not used (v3, o_r2).
  logic [AW-1:0] rd_addr;
  logic [AW-1:0] ia0, ia1;                   // issue address of edge e, e + 1 (lookahead)
  logic [AW-1:0] o_addr;                     // gs(og) + ow
  (* max_fanout = 16 *) logic rd_out;        // D_OUT: rd_addr from o_addr
  logic [W-1:0]  ld_q [P];

  // output
  logic [7:0]   og;
  logic [4:0]   ow, ow1, ow2;
  logic         o_r1, o_r2;
  logic [359:0] hb;
  logic [383:0] hc;                          // channel decisions around the group

  // ------------------------------------------------------------ channel decisions
  // input LLR signs in natural order, 32 per word, one copy per input buffer
  logic [31:0] sg_sr, sg_wd;
  logic        sg_we, sg_wb;
  logic [10:0] sg_wa, sg_ra;
  logic [31:0] sg_q [2];
  for (genvar c = 0; c < 2; c++) begin : g_sgn
    (* ram_style = "block" *) logic [31:0] mem [2048];
    always_ff @(posedge clk) begin
      if (sg_we && sg_wb == c[0]) mem[sg_wa] <= sg_wd;
      sg_q[c] <= mem[sg_ra];
    end
  end
  logic [5:0]   obyte;
  logic [15:0]  obits;

  // ------------------------------------------------------------ memories
  // 9 slices of 2 lanes x 3 banks of 1024 x 36: one BRAM36 each (27)
  logic [1:0] rbank_q;
  always_ff @(posedge clk) rbank_q <= r_addr[11:10];
  for (genvar r = 0; r < P / 2; r++) begin : g_rmem
    logic [35:0] bq [3];
    for (genvar k = 0; k < 3; k++) begin : g_bank
      (* ram_style = "block" *) logic [35:0] mem [1024];
      always_ff @(posedge clk) begin
        if (rmem_we && r_addr[11:10] == k) mem[r_addr[9:0]] <= rmem_d[r*36 +: 36];
        bq[k] <= mem[r_addr[9:0]];
      end
    end
    assign rmem_q[r*36 +: 36] = bq[rbank_q];
  end

  for (genvar c = 0; c < 2; c++) begin : g_copy
    for (genvar l = 0; l < P; l++) begin : g_lane
      (* ram_style = "block" *) logic [W-1:0] mem [1 << AW];
      always_ff @(posedge clk) begin
        if (l_we[c][l]) mem[l_waddr[c]] <= l_wdata[c][l];
        l_rdata[c][l] <= mem[l_raddr[c]];
      end
    end
  end

  always_comb begin
    for (int c = 0; c < 2; c++) begin
      l_we[c]    = '0;
      l_waddr[c] = iw_waddr;
      l_raddr[c] = rd_addr;
      for (int l = 0; l < P; l++) l_wdata[c][l] = iw_wdata;
      if (iw_we && ib == c[0]) l_we[c][iw_wlane] = 1'b1;
    end
    // the decoder's copy is never the input copy while decoding
    if (v6) begin
      l_we[db]    = '1;
      l_waddr[db] = a6;
      for (int l = 0; l < P; l++) l_wdata[db][l] = wd6[l];
    end
    for (int l = 0; l < P; l++) ld_q[l] = l_rdata[db][l];
  end

  always_ff @(posedge clk) begin
    o_ld <= dst == D_SUB;
    if (o_ld)
      for (int l = 0; l < P; l++) begin
        o_m1[l] <= rmem_q[l*18 +: 6];
        o_m2[l] <= rmem_q[l*18 + 6 +: 6];
        o_ix[l] <= rmem_q[l*18 + 12 +: 5];
        o_st[l] <= rmem_q[l*18 + 17];
      end
  end

  // ------------------------------------------------------------ buffer states
  always_ff @(posedge clk) begin
    if (rst) begin
      bst[0] <= B_FREE; bst[1] <= B_FREE;
      ib <= 1'b0; db <= 1'b0;
    end else begin
      if (in_done) begin
        bst[ib] <= B_FULL;
        ib <= !ib;
      end
      if (dec_done) begin
        bst[db] <= B_FREE;
        db <= !db;
      end else if (bst[db] == B_FULL && dst == D_IDLE && !(in_done && ib == db))
        bst[db] <= B_DEC;
    end
  end

  // ------------------------------------------------------------ LLR input
  assign s_ready = !rst && bst[ib] == B_FREE && !iw_phase && !in_done;

  always_ff @(posedge clk) begin
    in_done <= 1'b0;
    iw_we   <= 1'b0;
    sg_we   <= 1'b0;
    if (rst) begin
      iw_phase <= 1'b0;
      iw_n <= '0; iw_par <= 1'b0; iw_g <= '0; iw_m <= '0; iw_w <= '0; iw_lane <= '0; iw_base <= '0;
    end else begin : wr
      logic          take, restart;
      logic [7:0]    llr;
      logic          par;
      logic [7:0]    g;
      logic [8:0]    mm;
      logic [4:0]    w, ln;
      logic [AW-1:0] base;
      logic [16:0]   n;
      take = 1'b0;
      restart = 1'b0;
      llr = iw_q;
      if (!iw_phase && s_valid && s_ready) begin
        take = 1'b1;
        llr = s_llr_i;
        iw_q <= s_llr_q;
        iw_phase <= 1'b1;
        restart = s_first;
      end else if (iw_phase) begin
        take = 1'b1;
        iw_phase <= 1'b0;
      end
      n = restart ? 17'd0 : iw_n;
      par = restart ? 1'b0 : iw_par;
      g = restart ? 8'd0 : iw_g;
      mm = restart ? 9'd0 : iw_m;
      w = restart ? 5'd0 : iw_w;
      ln = restart ? 5'd0 : iw_lane;
      base = restart ? '0 : iw_base;
      if (take) begin
        iw_we    <= 1'b1;
        iw_wlane <= ln;
        iw_waddr <= base + AW'(w);
        iw_wdata <= {llr[7], llr};
        begin : sgw
          logic [31:0] wv;
          wv = sg_sr;
          wv[n[4:0]] = llr[7];
          sg_sr <= wv;
          sg_we <= n[4:0] == 5'd31 || n == 17'd64799;
          sg_wb <= ib;
          sg_wa <= n[15:5];
          sg_wd <= wv;
        end
        if (n == 17'd64799) begin
          iw_n <= '0; iw_par <= 1'b0; iw_g <= '0; iw_m <= '0; iw_w <= '0; iw_lane <= '0; iw_base <= '0;
          in_done <= 1'b1;
        end else begin
          iw_n <= n + 17'd1;
          if (!par && n + 17'd1 == 17'(kldpc)) begin
            // first parity bit: parity group 0, m = 0
            iw_par <= 1'b1; iw_g <= '0; iw_m <= '0; iw_w <= '0; iw_lane <= '0; iw_base <= gs(ng);
          end else if (!par && mm == 9'd359) begin
            iw_par <= 1'b0; iw_g <= g + 8'd1; iw_m <= '0; iw_w <= '0; iw_lane <= '0; iw_base <= base + AW'(S);
          end else if (!par || g == q - 8'd1) begin
            // next m (info: same group; parity: back to parity group 0)
            iw_par <= par;
            iw_g <= par ? 8'd0 : g;
            iw_m <= mm + 9'd1;
            iw_base <= par ? gs(ng) : base;
            if (w == 5'(S - 1)) begin iw_w <= '0; iw_lane <= ln + 5'd1; end
            else begin iw_w <= w + 5'd1; iw_lane <= ln; end
          end else begin
            // parity: next group, same m
            iw_par <= 1'b1; iw_g <= g + 8'd1; iw_m <= mm; iw_w <= w; iw_lane <= ln;
            iw_base <= base + AW'(S);
          end
        end
      end
    end
  end

  // ------------------------------------------------------------ decoder
  // APP address of edge ee of the current layer in sub-step b
  function automatic logic [AW-1:0] eaddr(input logic [4:0] ee);
    logic [4:0] w;
    w = (b < ed_w0[ee]) ? b + 5'(S) - ed_w0[ee] : b - ed_w0[ee];
    eaddr = gs(ed_g[ee]) + AW'(w);
  endfunction

  // candidate edge at the issue stage; its address c_a = eaddr(e) is ia0,
  // registered one clock ahead (ia1 = eaddr(e + 1)), as b and ed_* are
  // static from D_SUB2 to the end of D_B
  wire          c_brw = b < ed_w0[e];
  wire [5:0]    c_kt  = 6'(P) - 6'(ed_k0[e]) - 6'(c_brw);
  wire [4:0]    c_k   = (c_kt == 6'(P)) ? 5'd0 : c_kt[4:0];
  wire [AW-1:0] c_a   = ia0;
  // the p_(c-1) edge of layer 0 (parity group q-1, shift 1) is absent in row 0
  wire          c_inv = j == 8'd0 && ed_g[e] == ng + q - 8'd1 && b == 5'd0;
  wire          hazard = phase_b && ((v2 && a2 == c_a) || (v3 && a3 == c_a) || (v4 && a4 == c_a) ||
                                     (v5 && a5 == c_a) || (v6 && a6 == c_a));
  wire          issuing = (dst == D_A || dst == D_B) && e != deg && !hazard;
  wire          pipe_busy = v2 || v3 || v4 || v5 || v6;

  always_ff @(posedge clk) begin
    dec_done <= 1'b0;
    rmem_we  <= 1'b0;
    if (rst) begin
      dst <= D_IDLE;
      v2 <= 1'b0; v3 <= 1'b0; v4 <= 1'b0; v5 <= 1'b0; v6 <= 1'b0;
      o_r1 <= 1'b0; o_r2 <= 1'b0;
      m_valid <= 1'b0;
      st_frames <= '0;
      st_failed <= '0;
    end else begin
      if (m_valid && m_ready) m_valid <= 1'b0;
      // ---------------------------------------------------- edge pipeline
      // T: issue -> read address, sign memory read
      v2 <= issuing;
      if (issuing) begin
        e2 <= e; k2 <= c_k; inv2 <= c_inv; a2 <= c_a;
        smem_q <= smem[sg_base + 14'(e)];
        e <= e + 5'd1;
        ia0 <= ia1;
        ia1 <= eaddr(e + 5'd2);
      end
      // T+1: APP memory read
      v3 <= v2; e3 <= e2; k3 <= k2; inv3 <= inv2; a3 <= a2; sg3 <= smem_q;
      // T+2: data valid, rotate left by k (check lane l <- memory lane (l + k) mod P)
      v4 <= v3; e4 <= e3; k4 <= k3; inv4 <= inv3; a4 <= a3; sg4 <= sg3;
      for (int l = 0; l < P; l++) lv4[l] <= $signed(ld_q[(l + int'(k3)) % P]);
      // T+3: phase A: R_old, Q; phase B: L + R_new - R_old
      v5 <= v4; e5 <= e4; k5 <= k4; inv5 <= inv4; a5 <= a4;
      sg_wa5 <= sg_base + 14'(e4);           // sign write address (sg_base is static in phase A)
      if (v4 && !phase_b) begin
        for (int l = 0; l < P; l++) begin : pa
          logic [5:0] mg;
          logic signed [W-1:0] ro;
          mg = first_it ? 6'd0 : norm((e4 == o_ix[l]) ? o_m2[l] : o_m1[l]);
          ro = (o_st[l] ^ sg4[l]) ? -$signed({3'b0, mg}) : $signed({3'b0, mg});
          ro5[l] <= ro;
          q5[l]  <= sat9(11'(lv4[l]) - 11'(ro));
          hs5[l] <= lv4[l] < 0;
        end
      end
      if (v4 && phase_b) begin : pb
        logic [P*8-1:0] fw;
        fw = fifo[fifo_rp];
        fifo_rp <= fifo_rp + 5'd1;
        for (int l = 0; l < P; l++) begin : pbl
          logic                sn;
          logic signed [6:0]   rold;
          logic [5:0]          mg;
          logic signed [W-1:0] rn, nv;
          sn   = fw[l*8+7];
          rold = $signed(fw[l*8 +: 7]);
          mg   = norm((e4 == n_ix[l]) ? n_m2[l] : n_m1[l]);
          rn   = (n_st[l] ^ sn) ? -$signed({3'b0, mg}) : $signed({3'b0, mg});
          nv   = sat9(11'(lv4[l]) + 11'(rn) - 11'(rold));
          if (inv4 && l == 0) nv = lv4[l];
          nw5[l]  <= nv;
          lvp5[l] <= lv4[l];
        end
      end
      // T+4: phase A: min tracking, signs, FIFO; phase B: rotate right
      v6 <= v5 && phase_b; a6 <= a5;
      if (v5 && !phase_b) begin : a5s
        logic [P*8-1:0] fw;
        logic [P-1:0]   sgn;
        for (int l = 0; l < P; l++) begin : a5l
          logic       neg, s;
          logic [8:0] aq;
          logic [6:0] mg;
          neg = q5[l] < 0;
          aq  = neg ? 9'(-q5[l]) : 9'(q5[l]);
          mg  = (inv5 && l == 0) ? 7'd64 : (aq > 9'd63 ? 7'd63 : 7'(aq));
          s   = (inv5 && l == 0) ? 1'b0 : neg;
          sgn[l] = s;
          fw[l*8 +: 8] = {s, 7'(ro5[l])};
          if (mg < a_m1[l]) begin
            a_m2[l] <= a_m1[l]; a_m1[l] <= mg; a_ix[l] <= e5;
          end else if (mg < a_m2[l]) a_m2[l] <= mg;
          a_st[l] <= a_st[l] ^ s;
          if (!(inv5 && l == 0)) hpar[l] <= hpar[l] ^ hs5[l];
        end
        fifo[fifo_wp] <= fw;
        fifo_wp <= fifo_wp + 5'd1;
        smem[sg_wa5] <= sgn;
      end
      if (v5 && phase_b) begin
        for (int l = 0; l < P; l++) begin
          wd6[(l + int'(k5)) % P] <= nw5[l];
          if (!(inv5 && l == 0) && ((lvp5[l] < 0) != (nw5[l] < 0))) flips <= 1'b1;
        end
      end
      // T+5: write (l_we in the memory block)

      // ---------------------------------------------------- control
      case (dst)
        D_IDLE: if (bst[db] == B_DEC && !dec_done) begin
          it <= '0;
          dst <= D_ITER;
        end

        D_ITER: begin
          it <= it + 6'd1;
          first_it <= it == 0;
          unsat <= 1'b0;
          flips <= 1'b0;
          j <= '0;
          b <= '0;
          r_addr <= '0;
          sg_base <= '0;
          ed_base <= rbase;
          ld_e <= '0;
          dst <= D_LOAD;
        end

        // edges of layer j into registers (ROM latency 1)
        D_LOAD: begin
          erom_q <= erom[ed_base + 13'(ld_e)];
          if (ld_e != 0) begin
            ed_g[ld_e - 5'd1]  <= erom_q[7:0];
            ed_w0[ld_e - 5'd1] <= erom_q[12:8];
            ed_k0[ld_e - 5'd1] <= erom_q[17:13];
          end
          if (ld_e == deg) dst <= D_SUB;
          else ld_e <= ld_e + 5'd1;
        end

        D_SUB: dst <= D_SUB2;                     // R memory read (g_rmem)

        D_SUB2: begin
          for (int l = 0; l < P; l++) begin
            a_m1[l] <= 7'd64; a_m2[l] <= 7'd64; a_ix[l] <= '0; a_st[l] <= 1'b0;
            hpar[l] <= 1'b0;
          end
          e <= '0;
          ia0 <= eaddr(5'd0);
          ia1 <= eaddr(5'd1);
          phase_b <= 1'b0;
          fifo_wp <= '0;
          fifo_rp <= '0;
          dst <= D_A;
        end

        D_A: if (e == deg && !pipe_busy) dst <= D_AEND;

        D_AEND: begin : aend
          logic [P*18-1:0] rw;
          for (int l = 0; l < P; l++) begin : al
            logic [5:0] m2c;
            m2c = a_m2[l] > 7'd63 ? 6'd63 : a_m2[l][5:0];
            n_m1[l] <= a_m1[l][5:0];
            n_m2[l] <= m2c;
            n_ix[l] <= a_ix[l];
            n_st[l] <= a_st[l];
            rw[l*18 +: 18] = {a_st[l], a_ix[l], m2c, a_m1[l][5:0]};
            if (hpar[l]) unsat <= 1'b1;
          end
          rmem_d  <= rw;
          rmem_we <= 1'b1;
          e <= '0;
          ia0 <= eaddr(5'd0);
          ia1 <= eaddr(5'd1);
          phase_b <= 1'b1;
          dst <= D_B;
        end

        D_B: if (e == deg && !pipe_busy) dst <= D_NEXT;

        D_NEXT: begin
          r_addr  <= r_addr + 12'd1;
          sg_base <= sg_base + 14'(deg);
          if (b == 5'(S - 1)) begin
            b <= '0;
            if (j == q - 8'd1) dst <= D_ITEREND;
            else begin
              j <= j + 8'd1;
              ed_base <= ed_base + 13'(deg);
              ld_e <= '0;
              dst <= D_LOAD;
            end
          end else begin
            b <= b + 5'd1;
            dst <= D_SUB;
          end
        end

        D_ITEREND: begin
          if (!unsat && !flips) begin
            conv <= 1'b1;
            dst <= D_OUT;
          end else if (it == max_it) begin
            conv <= 1'b0;
            dst <= D_OUT;
          end else dst <= D_ITER;
          og <= '0; ow <= '0;
          o_addr <= '0;
          obits <= '0;
        end

        // ---------------------------------------------------- output
        // read the S words of info group og (2 clocks latency), then send its bits
        D_OUT: begin
          o_r1 <= 1'b0;
          if (ow != 5'(S)) begin
            sg_ra <= 11'((16'(og) * 16'd360) >> 5) + 11'(ow);
            o_r1 <= 1'b1;
            ow1 <= ow;
            ow <= ow + 5'd1;
            o_addr <= o_addr + AW'(1);
          end
          o_r2 <= o_r1;
          ow2 <= ow1;
          if (o_r2 && ow2 < 5'd12) hc[int'(ow2) * 32 +: 32] <= sg_q[db];
          if (o_r2) begin
            for (int l = 0; l < P; l++) hb[int'(ow2) + S * l] <= ld_q[l][W-1];
            if (ow2 == 5'(S - 1)) begin
              obyte <= '0;
              o_r1 <= 1'b0;
              o_r2 <= 1'b0;
              dst <= D_OUT2;
            end
          end
        end

        D_OUT2: begin
          if (!m_valid || m_ready) begin : ob
            logic [7:0]  d;
            logic        last;
            for (int i = 0; i < 8; i++) d[7 - i] = hb[int'(obyte) * 8 + i];
            last = obits + 16'd8 == kldpc;
            m_valid <= 1'b1;
            m_data  <= d;
            begin : ch
              logic [8:0] x;
              x = {og[1:0], 3'b000} + 9'(obyte) * 9'd8;     // (360 og) mod 32 + 8 obyte
              for (int i = 0; i < 8; i++) m_chan[7 - i] <= hc[int'(x) + i];
            end
            m_last  <= last;
            m_ok    <= conv;
            m_iters <= it;
            obits <= obits + 16'd8;
            if (last) begin
              st_frames <= st_frames + 16'd1;
              if (!conv) st_failed <= st_failed + 16'd1;
              dst <= D_DONE;
            end else if (obyte == 6'd44) begin
              og <= og + 8'd1;
              ow <= '0;
              o_addr <= gs(og + 8'd1);
              dst <= D_OUT;
            end else obyte <= obyte + 6'd1;
          end
        end

        D_DONE: begin
          if (!m_valid || m_ready) begin
            dec_done <= 1'b1;
            dst <= D_IDLE;
          end
        end

        default: dst <= D_IDLE;
      endcase
    end
    // D_OUT is only entered from these states, D_A / D_B never
    rd_out  <= dst == D_ITEREND || dst == D_OUT || dst == D_OUT2;
    rd_addr <= rd_out ? o_addr : ia0;
  end

endmodule
