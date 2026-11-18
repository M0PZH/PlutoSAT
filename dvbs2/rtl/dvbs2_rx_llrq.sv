// DDR LLR frame queue between the receive front end and the LDPC decoder
// (second receive slot, dvbs2/docs/rx2_design.md sections 6.1, 6.2, 7.3).
//
//   front end -> packer -> staging (per slot) -> AXI write bursts -> DDR ring[slot][frame]
//   DDR -> AXI read bursts -> read FIFO -> LDPC decoder (one frame after the other)
//
// Input: LLR words (as from dvbs2_rx_carrier), tagged with their slot; the
// words of the two slots may interleave in any way. QPSK: {s_llr_q, s_llr_i},
// two LLRs; 8PSK (MODCOD 12..17): {s_llr_z, s_llr_q, s_llr_i}, the LLRs of
// the symbol's bits (b0 b1 b2) in that order. A slot's frame is exactly
// Nldpc / 2 (QPSK) or Nldpc / 3 (8PSK) words (Nldpc = 64800, short 16200),
// s_first on the first; with it s_modcod / s_pilots / s_short (taken
// there). An s_first before a frame's last word abandons that frame (it is
// not decoded; its ring entry is reused).
//
// DDR layout (base 2 MB aligned, base[20:0] ignored): slot s, frame f at
// base + s * 1 MB + f * 64 KB, f = 0 .. nfr-1, holding the frame's Nldpc
// LLR bytes in arrival order (QPSK: bit order; 8PSK: symbol order, 3 bytes a
// symbol; a 64-bit beat = 8 bytes, little endian). Writes: 16-beat bursts
// (128 bytes), the last of a frame 4 beats (short frames: 9). A frame is
// complete when the write response of its last burst is back; its
// descriptor {slot, frame} then joins one FIFO, so frames are decoded in
// completion order. The reader streams the frames into the decoder (m_*, two
// LLRs a word in bit order, with the frame's code, iteration limit and tag)
// and frees each ring entry once its last read beat is in. 8PSK: the bit
// deinterleaver (EN 302 307-1 5.3.3, column c = bits c N/3 .. (c+1) N/3 - 1
// = byte c of each symbol, byte 2 - c for rate 3/5): the frame is read three
// times, pass c taking every third byte, one LLR per clock.
//
// A frame that starts while its slot's ring is full (nfr frames written or
// being written, not yet read back) is dropped and counted (st_dropped).
// Shared iteration budget (rx2_design.md 6.2): while more than qhi frames
// wait (st_depth: complete frames not yet read, plus the one being read),
// it_cap_en tells the decoder to stop each frame at it_lo iterations.
//
// rst (the core's soft reset): the AXI transfers in flight are finished
// first (write bursts complete, responses and read data taken and ignored),
// then the queue is cleared; meanwhile s_ready and m_valid are low. arst
// (the AXI bus reset) resets everything at once. en (static, change under
// rst): 0 = the queue is idle (dvbs2_rx then bypasses it).
//
// AXI3 master, 64 bits (a Zynq S_AXI_HP port), ID 0, INCR bursts, at most
// WO write bursts and RD read beats in flight. Bandwidth: 2 bytes per
// symbol written and read, e.g. 2 x 2.38 MBd x 2 B = 9.5 MB/s; 8PSK 3 bytes
// written and 9 read per symbol (2.38 MBd: 29 MB/s).

module dvbs2_rx_llrq #(
  parameter int SD = 32,                     // staging beats per slot (power of 2, >= 16)
  parameter int RD = 64,                     // read FIFO beats (power of 2, >= 32)
  parameter int WO = 4                       // write bursts in flight (power of 2)
) (
  input  logic        clk,
  input  logic        arst,                  // AXI bus reset
  input  logic        rst,                   // soft reset (drains, then clears)

  // configuration (static; change under rst)
  input  logic        en,
  input  logic [31:0] base,
  input  logic [4:0]  nfr,                   // frames per slot ring, 1..16 (0: 16)
  input  logic [5:0]  max_it0,               // LDPC iteration limit, slot 0
  input  logic [5:0]  max_it1,               // ... slot 1
  input  logic [5:0]  qhi,                   // cap while st_depth > qhi
  input  logic [5:0]  it_lo,                 // ... at it_lo iterations
  input  logic [1:0]  clr_dropped,           // clear st_dropped<s> (the slot's reset)

  // LLR words from the front end
  input  logic        s_valid,
  output logic        s_ready,
  input  logic [7:0]  s_llr_i,
  input  logic [7:0]  s_llr_q,
  input  logic [7:0]  s_llr_z,               // 8PSK: the third LLR
  input  logic        s_first,
  input  logic        s_slot,
  input  logic [4:0]  s_modcod,
  input  logic        s_pilots,
  input  logic        s_short,
  input  logic        s_epoch,               // opaque per-frame bit (taken with s_first), out as m_epoch

  // to the LDPC decoder (per-frame values constant within a frame)
  output logic        m_valid,
  input  logic        m_ready,
  output logic [7:0]  m_llr_i,
  output logic [7:0]  m_llr_q,
  output logic        m_first,
  output logic [4:0]  m_rate,                // {short, rate_idx}
  output logic [5:0]  m_max_it,
  output logic        m_slot,
  output logic [4:0]  m_modcod,
  output logic        m_pilots,
  output logic        m_short,
  output logic        m_epoch,
  output logic        it_cap_en,
  output logic [5:0]  it_cap,

  // status
  output logic [5:0]  st_depth,
  output logic [5:0]  st_depth_max,
  output logic [15:0] st_frames,             // frames complete in DDR
  output logic [15:0] st_dropped0,           // frames dropped, ring full: slot 0
  output logic [15:0] st_dropped1,           // ... slot 1
  output logic [15:0] st_axi_err,            // SLVERR / DECERR responses
  output logic        st_busy,               // frames queued or in flight

  // AXI3 master
  output logic [31:0] m_axi_awaddr,
  output logic [3:0]  m_axi_awlen,
  output logic [2:0]  m_axi_awsize,
  output logic [1:0]  m_axi_awburst,
  output logic [3:0]  m_axi_awcache,
  output logic        m_axi_awvalid,
  input  logic        m_axi_awready,
  output logic [63:0] m_axi_wdata,
  output logic [7:0]  m_axi_wstrb,
  output logic        m_axi_wlast,
  output logic        m_axi_wvalid,
  input  logic        m_axi_wready,
  input  logic [1:0]  m_axi_bresp,
  input  logic        m_axi_bvalid,
  output logic        m_axi_bready,
  output logic [31:0] m_axi_araddr,
  output logic [3:0]  m_axi_arlen,
  output logic [2:0]  m_axi_arsize,
  output logic [1:0]  m_axi_arburst,
  output logic [3:0]  m_axi_arcache,
  output logic        m_axi_arvalid,
  input  logic        m_axi_arready,
  input  logic [63:0] m_axi_rdata,
  input  logic [1:0]  m_axi_rresp,
  input  logic        m_axi_rlast,
  input  logic        m_axi_rvalid,
  output logic        m_axi_rready
);

  localparam int SA = $clog2(SD);
  localparam int RA = $clog2(RD);
  localparam int WA = $clog2(WO);
  // per frame size: beats (8100 / 2025), the start and length of the last
  // burst (8096, 4 / 2016, 9)
  function automatic logic [12:0] nbeat(input logic sh);
    nbeat = sh ? 13'd2025 : 13'd8100;
  endfunction
  function automatic logic [12:0] lburst(input logic sh);
    lburst = sh ? 13'd2016 : 13'd8096;
  endfunction
  function automatic logic [4:0] lblen(input logic sh);
    lblen = sh ? 5'd9 : 5'd4;
  endfunction
  // last word of a frame: Nldpc / 2 - 1 or Nldpc / 3 - 1
  function automatic logic [14:0] last_w(input logic psk8, input logic sh);
    case ({psk8, sh})
      2'b00:   last_w = 15'd32399;
      2'b01:   last_w = 15'd8099;
      2'b10:   last_w = 15'd21599;
      default: last_w = 15'd5399;
    endcase
  endfunction
  function automatic logic is_psk8(input logic [4:0] mc);
    is_psk8 = mc >= 5'd12;
  endfunction
  // code index {short, rate_idx}: QPSK MODCOD 1..11 -> 0..10, 8PSK 12..17 ->
  // 3/5, 2/3, 3/4, 5/6, 8/9, 9/10
  function automatic logic [4:0] rate_of(input logic [4:0] mc, input logic sh);
    logic [3:0] r;
    case (mc)
      5'd12:   r = 4'd4;
      5'd13:   r = 4'd5;
      5'd14:   r = 4'd6;
      5'd15:   r = 4'd8;
      5'd16:   r = 4'd9;
      5'd17:   r = 4'd10;
      default: r = 4'(mc - 5'd1);
    endcase
    rate_of = {sh, r};
  endfunction

  initial assert (SD >= 16 && (1 << SA) == SD && RD >= 32 && (1 << RA) == RD && (1 << WA) == WO)
    else $fatal(1, "dvbs2_rx_llrq: SD, RD, WO");

  assign m_axi_awsize  = 3'd3;
  assign m_axi_awburst = 2'd1;
  assign m_axi_awcache = 4'b0011;
  assign m_axi_wstrb   = 8'hFF;
  assign m_axi_bready  = 1'b1;
  assign m_axi_arsize  = 3'd3;
  assign m_axi_arburst = 2'd1;
  assign m_axi_arcache = 4'b0011;
  assign m_axi_rready  = 1'b1;

  wire [4:0] nf = (nfr == 0 || nfr > 5'd16) ? 5'd16 : nfr;

  function automatic logic [31:0] ring_addr(input logic s, input logic [3:0] f, input logic [12:0] beat);
    ring_addr = {base[31:21], s, f, beat, 3'b000};
  endfunction

  // ------------------------------------------------------------ resets
  // lrst: the queue's own state; the AXI channel state (bursts, responses
  // and read beats owed) is reset by arst only
  logic        flush;
  logic        we_busy;
  logic [WA:0] wout;
  logic [RA:0] r_owed;
  wire         axi_idle = !we_busy && wout == 0 && !m_axi_arvalid && r_owed == 0;
  always_ff @(posedge clk)
    if (arst)          flush <= 1'b0;
    else if (rst)      flush <= 1'b1;
    else if (axi_idle) flush <= 1'b0;
  wire lrst = arst || rst || flush || !en;

  // ------------------------------------------------------------ staging
  // per slot SD beats {data}, flags {short, abort, sof, frame}; slot = address MSB
  (* ram_style = "distributed" *) logic [63:0] stg_d [2 * SD];
  (* ram_style = "distributed" *) logic [6:0]  stg_f [2 * SD];
  logic [SA:0] st_wp [2], st_rp [2];
  logic        push;
  logic        push_s;
  logic [63:0] push_d;
  logic [6:0]  push_f;
  always_ff @(posedge clk)
    if (push) begin
      stg_d[{push_s, st_wp[push_s][SA-1:0]}] <= push_d;
      stg_f[{push_s, st_wp[push_s][SA-1:0]}] <= push_f;
    end
  wire [SA:0] st_cnt0 = st_wp[0] - st_rp[0];
  wire [SA:0] st_cnt1 = st_wp[1] - st_rp[1];
  // (a push is written one clock after it is decided: one beat in reserve)
  wire        room = st_cnt0 < (SA+1)'(SD - 1) && st_cnt1 < (SA+1)'(SD - 1);

  // ------------------------------------------------------------ packer state
  logic [14:0] pk_cnt  [2];                  // next word of the frame (0: between frames)
  logic        pk_act  [2];                  // the frame is stored (else dropped)
  logic        pk_reuse[2];                  // head is allocated (a frame was abandoned)
  logic [3:0]  pk_head [2];                  // ring entry of the frame being written
  logic [4:0]  occ     [2];                  // ring entries allocated, not yet read back
  logic [12:0] pk_beat [2];                  // beats pushed for the frame
  logic [55:0] pk_acc  [2];                  // bytes of the beat so far
  logic [2:0]  pk_nb   [2];                  // ... their number
  logic        pk_sh   [2];                  // the frame's size (short)
  logic [14:0] pk_lw   [2];                  // its last word
  (* ram_style = "distributed" *) logic [7:0] meta [32];   // {short, epoch, pilots, MODCOD} of [slot, frame]
  logic        meta_we;
  logic [4:0]  meta_wa;
  logic [7:0]  meta_wd;
  always_ff @(posedge clk) if (meta_we) meta[meta_wa] <= meta_wd;

  // abandoning a frame: the pad (s_ready low) fills the frame's last burst so
  // no burst holds beats of two frames; its last beat is flagged abort
  logic        pd_act, pd_s, pd_part;
  wire         abort_req = s_valid && s_first && pk_cnt[s_slot] != 0;
  assign s_ready = !lrst && !pd_act && !abort_req && room;
  wire   take = s_valid && s_ready;

  logic [1:0]  occ_dec;

  function automatic logic [3:0] next_f(input logic [3:0] f);
    next_f = (5'(f) + 5'd1 == nf) ? 4'd0 : f + 4'd1;
  endfunction

  // ------------------------------------------------------------ write engine
  logic [6:0]  hf [2];                       // flags of each slot's oldest staged beat
  assign hf[0] = stg_f[{1'b0, st_rp[0][SA-1:0]}];
  assign hf[1] = stg_f[{1'b1, st_rp[1][SA-1:0]}];
  logic [12:0] we_boff [2];                  // beat of the slot's next burst
  logic [12:0] boff_e  [2];
  logic [4:0]  blen    [2];
  logic [1:0]  rdy;
  always_comb
    for (int s = 0; s < 2; s++) begin
      boff_e[s] = hf[s][4] ? 13'd0 : we_boff[s];
      blen[s]   = (boff_e[s] == lburst(hf[s][6])) ? lblen(hf[s][6]) : 5'd16;
      rdy[s]    = (s == 0 ? st_cnt0 : st_cnt1) >= (SA+1)'(blen[s]);
    end

  logic        we_s, we_rr, we_lastb, we_ab, aw_done, w_done;
  // a new write burst: a slot with a whole burst staged (round robin)
  wire         we_ps    = (rdy == 2'b11) ? !we_rr : rdy[1];
  wire         we_start = !we_busy && !lrst && rdy != 0 && wout != (WA+1)'(WO);
  logic [3:0]  we_f;
  logic [4:0]  we_len, we_n;
  logic [6:0]  we_hf;                        // flags of the beat on the W channel
  assign m_axi_wdata = stg_d[{we_s, st_rp[we_s][SA-1:0]}];
  assign we_hf       = stg_f[{we_s, st_rp[we_s][SA-1:0]}];
  assign m_axi_wlast = we_n == we_len - 5'd1;

  // write responses owed: {commit, slot, frame}
  logic [5:0]  bq [WO];
  logic [WA:0] bq_wp, bq_rp;

  // descriptors of complete frames {slot, frame}
  logic [4:0]  dq [32];
  logic [5:0]  dq_wp, dq_rp;
  wire  [5:0]  dq_cnt = dq_wp - dq_rp;

  // ------------------------------------------------------------ reader
  (* ram_style = "distributed" *) logic [63:0] rf [RD];
  logic [RA:0] rf_wp, rf_rp;
  wire  [RA:0] rf_cnt = rf_wp - rf_rp;
  always_ff @(posedge clk) if (m_axi_rvalid && !lrst) rf[rf_wp[RA-1:0]] <= m_axi_rdata;
  wire  [63:0] rf_q = rf[rf_rp[RA-1:0]];

  logic        rd_act, rd_s;
  logic [3:0]  rd_f;
  logic [12:0] rd_ab;                        // next beat to request (of the pass)
  logic [14:0] rd_rcv;                       // beats received (all passes)
  logic [14:0] rd_wc;
  logic [1:0]  rd_ws;
  logic [4:0]  rd_modcod;
  logic        rd_pilots, rd_epoch, rd_sh;
  logic [5:0]  rd_maxit;
  // 8PSK: passes (column c) of the requests and of the consumption, the
  // beat consumed, the first byte of column c' in it (0..2) and the
  // selected byte in it (0..2), the held first LLR of a word
  logic        rd_p8;
  logic [1:0]  rd_pi, rd_pc;
  logic [12:0] rd_cb;
  logic [1:0]  rd_j0, rd_k;
  logic        rd_hv;
  logic [7:0]  rd_hb;
  wire  [4:0]  ar_len = (rd_ab == lburst(rd_sh)) ? lblen(rd_sh) : 5'd16;
  wire  [12:0] rd_nb  = nbeat(rd_sh);
  // the last beat of the frame read back (all passes)
  wire  [14:0] rd_rlast = rd_p8 ? 15'(3 * rd_nb) - 15'd1 : 15'(rd_nb) - 15'd1;

  assign st_depth  = dq_cnt + 6'(rd_act);
  assign it_cap_en = en && st_depth > qhi;
  assign it_cap    = it_lo;
  assign st_busy   = rd_act || dq_cnt != 0 || we_busy || wout != 0 || m_valid;

  // ------------------------------------------------------------ AXI channel state (arst only)
  always_ff @(posedge clk) begin
    if (arst) begin
      we_busy <= 1'b0;
      m_axi_awvalid <= 1'b0;
      m_axi_wvalid <= 1'b0;
      m_axi_arvalid <= 1'b0;
      wout <= '0;
      bq_wp <= '0; bq_rp <= '0;
      r_owed <= '0;
      we_rr <= 1'b0;
      st_axi_err <= '0;
    end else begin : axi
      logic       ps, aw_ok, wl_ok;
      logic [1:0] err;
      ps = we_ps;
      if (we_start) begin
        we_busy  <= 1'b1;
        we_s     <= ps;
        we_rr    <= ps;
        we_f     <= hf[ps][3:0];
        we_len   <= blen[ps];
        we_n     <= '0;
        we_lastb <= boff_e[ps] == lburst(hf[ps][6]);
        we_ab    <= 1'b0;
        aw_done  <= 1'b0;
        w_done   <= 1'b0;
        m_axi_awaddr  <= ring_addr(ps, hf[ps][3:0], boff_e[ps]);
        m_axi_awlen   <= 4'(blen[ps] - 5'd1);
        m_axi_awvalid <= 1'b1;
        m_axi_wvalid  <= 1'b1;
      end
      aw_ok = m_axi_awvalid && m_axi_awready;
      wl_ok = m_axi_wvalid && m_axi_wready && m_axi_wlast;
      if (aw_ok) begin m_axi_awvalid <= 1'b0; aw_done <= 1'b1; end
      if (m_axi_wvalid && m_axi_wready) begin
        we_n  <= we_n + 5'd1;
        we_ab <= we_ab || we_hf[5];
        if (m_axi_wlast) begin m_axi_wvalid <= 1'b0; w_done <= 1'b1; end
      end
      if (we_busy && (aw_done || aw_ok) && (w_done || wl_ok)) begin
        // burst done: its response is owed; a frame's last burst commits it
        we_busy <= 1'b0;
        bq[bq_wp[WA-1:0]] <= {we_lastb && !we_ab && !(wl_ok && we_hf[5]), we_s, we_f};
        bq_wp <= bq_wp + 1'b1;
      end
      if (m_axi_bvalid) bq_rp <= bq_rp + 1'b1;
      wout <= wout + (WA+1)'(we_start) - (WA+1)'(m_axi_bvalid);
      err = 2'(m_axi_bvalid && m_axi_bresp[1]) + 2'(m_axi_rvalid && m_axi_rresp[1]);
      st_axi_err <= (17'(st_axi_err) + 17'(err) > 17'hFFFF) ? 16'hFFFF : st_axi_err + 16'(err);

      // read bursts for the frame being read, while the read FIFO has room
      if (m_axi_arvalid && m_axi_arready) m_axi_arvalid <= 1'b0;
      if (!m_axi_arvalid && rd_act && !lrst && rd_ab != rd_nb &&
          32'(rf_cnt) + 32'(r_owed) + 32'(ar_len) <= RD) begin
        m_axi_araddr  <= ring_addr(rd_s, rd_f, rd_ab);
        m_axi_arlen   <= 4'(ar_len - 5'd1);
        m_axi_arvalid <= 1'b1;
        r_owed <= r_owed + (RA+1)'(ar_len) - (RA+1)'(m_axi_rvalid);
      end else
        r_owed <= r_owed - (RA+1)'(m_axi_rvalid);
    end
  end

  // ------------------------------------------------------------ queue state (lrst)
  always_ff @(posedge clk) begin
    push    <= 1'b0;                         // (default; set below, used the same clock)
    meta_we <= 1'b0;
    if (lrst) begin
      for (int s = 0; s < 2; s++) begin
        st_wp[s] <= '0; st_rp[s] <= '0;
        pk_cnt[s] <= '0; pk_act[s] <= 1'b0; pk_reuse[s] <= 1'b0; pk_head[s] <= '0;
        occ[s] <= '0; pk_beat[s] <= '0; we_boff[s] <= '0; pk_nb[s] <= '0;
      end
      pd_act <= 1'b0;
      dq_wp <= '0; dq_rp <= '0;
      rd_act <= 1'b0;
      rf_wp <= '0; rf_rp <= '0;
      m_valid <= 1'b0;
      if (arst || rst) begin
        st_frames <= '0; st_dropped0 <= '0; st_dropped1 <= '0; st_depth_max <= '0;
      end
    end else begin : q
      logic [1:0] occ_inc;
      // staging pointers (the write itself: the block above)
      if (push) st_wp[push_s] <= st_wp[push_s] + 1'b1;
      if (m_axi_wvalid && m_axi_wready) st_rp[we_s] <= st_rp[we_s] + 1'b1;
      if (we_start) we_boff[we_ps] <= boff_e[we_ps] + 13'(blen[we_ps]);

      // commits and descriptors
      if (m_axi_bvalid && bq[bq_rp[WA-1:0]][5]) begin
        dq[dq_wp[4:0]] <= bq[bq_rp[WA-1:0]][4:0];
        dq_wp <= dq_wp + 6'd1;
        st_frames <= st_frames + 16'd1;
      end
      if (st_depth > st_depth_max) st_depth_max <= st_depth;

      // reader: next frame, read beats, words out
      if (!rd_act && dq_cnt != 0) begin : disp
        logic [4:0] d;
        d = dq[dq_rp[4:0]];
        dq_rp <= dq_rp + 6'd1;
        rd_act <= 1'b1;
        rd_s <= d[4]; rd_f <= d[3:0];
        rd_ab <= '0; rd_rcv <= '0; rd_wc <= '0; rd_ws <= '0;
        {rd_sh, rd_epoch, rd_pilots, rd_modcod} <= meta[d];
        rd_p8 <= is_psk8(meta[d][4:0]);
        rd_pi <= '0; rd_pc <= '0; rd_cb <= '0; rd_k <= '0; rd_hv <= 1'b0;
        // column 0 is byte 0 of a symbol (rate 3/5: byte 2); byte n of the
        // frame is column n mod 3, beat B starts with column (8 B) mod 3
        rd_j0 <= (meta[d][4:0] == 5'd12) ? 2'd2 : 2'd0;
        rd_maxit <= d[4] ? max_it1 : max_it0;
      end
      if (m_axi_arvalid && m_axi_arready) begin
        rd_ab <= rd_ab + 13'(m_axi_arlen) + 13'd1;
        // 8PSK: the next pass's requests start once this pass's are out
        if (rd_p8 && rd_pi != 2'd2 && rd_ab + 13'(m_axi_arlen) + 13'd1 == rd_nb) begin
          rd_ab <= '0;
          rd_pi <= rd_pi + 2'd1;
        end
      end
      if (m_axi_rvalid) begin
        rf_wp <= rf_wp + 1'b1;
        rd_rcv <= rd_rcv + 15'd1;
      end
      if (!rd_p8) begin
        if (!m_valid || m_ready) begin
          if (rd_act && rf_cnt != 0) begin
            m_valid  <= 1'b1;
            m_llr_i  <= rf_q[16 * rd_ws +: 8];
            m_llr_q  <= rf_q[16 * rd_ws + 8 +: 8];
            m_first  <= rd_wc == 0;
            m_rate   <= rate_of(rd_modcod, rd_sh);
            m_max_it <= rd_maxit;
            m_slot   <= rd_s;
            m_modcod <= rd_modcod;
            m_pilots <= rd_pilots;
            m_short  <= rd_sh;
            m_epoch  <= rd_epoch;
            rd_ws <= rd_ws + 2'd1;
            if (rd_ws == 2'd3) rf_rp <= rf_rp + 1'b1;
            rd_wc <= rd_wc + 15'd1;
            if (rd_wc == last_w(1'b0, rd_sh)) rd_act <= 1'b0;
          end else
            m_valid <= 1'b0;
        end
      end else begin : rd8
        // 8PSK: one selected byte per clock; the first of a word is held,
        // the second completes it (only when the output is free)
        logic [3:0] j;
        logic       lastb, lastw;
        j = 4'(rd_j0) + 4'(3 * rd_k);
        lastb = j + 4'd3 > 4'd7;             // the beat's last byte of the column
        lastw = rd_wc == last_w(1'b0, rd_sh);  // Nldpc / 2 words, as for QPSK
        if (m_valid && m_ready) m_valid <= 1'b0;
        if (rd_act && rf_cnt != 0 && (!rd_hv || !m_valid || m_ready)) begin
          if (!rd_hv) begin
            rd_hv <= 1'b1;
            rd_hb <= rf_q[8 * j[2:0] +: 8];
          end else begin
            rd_hv    <= 1'b0;
            m_valid  <= 1'b1;
            m_llr_i  <= rd_hb;
            m_llr_q  <= rf_q[8 * j[2:0] +: 8];
            m_first  <= rd_wc == 0;
            m_rate   <= rate_of(rd_modcod, rd_sh);
            m_max_it <= rd_maxit;
            m_slot   <= rd_s;
            m_modcod <= rd_modcod;
            m_pilots <= rd_pilots;
            m_short  <= rd_sh;
            m_epoch  <= rd_epoch;
            rd_wc <= rd_wc + 15'd1;
            if (lastw) rd_act <= 1'b0;
          end
          if (lastb) begin
            rf_rp <= rf_rp + 1'b1;
            rd_k  <= '0;
            // next beat: its first byte of the column is 8 later, i.e. (j0 + 1) mod 3
            rd_j0 <= (rd_j0 == 2'd2) ? 2'd0 : rd_j0 + 2'd1;
            if (rd_cb == rd_nb - 13'd1) begin
              // the pass ends: the next one from beat 0, its column c' first
              rd_cb <= '0;
              rd_pc <= rd_pc + 2'd1;
              rd_j0 <= (rd_modcod == 5'd12) ? 2'd1 - 2'(rd_pc) : 2'(rd_pc) + 2'd1;
            end else
              rd_cb <= rd_cb + 13'd1;
          end else
            rd_k <= rd_k + 2'd1;
        end
      end

      // packer
      occ_inc = '0;
      if (pd_act) begin : pad
        logic last;
        last = (pk_beat[pd_s][3:0] == 4'd15) || (pk_beat[pd_s] + 13'd1 == nbeat(pk_sh[pd_s]));
        if (!pk_act[pd_s] || (!pd_part && pk_beat[pd_s][3:0] == 4'd0)) begin
          // nothing (more) to fill
          pd_act <= 1'b0;
          pk_cnt[pd_s] <= '0;
          pk_reuse[pd_s] <= pk_act[pd_s];
        end else if (room) begin
          push   <= 1'b1;
          push_s <= pd_s;
          push_d <= pd_part ? {8'd0, pk_acc[pd_s]} : 64'd0;
          push_f <= {pk_sh[pd_s], last, pk_beat[pd_s] == 0, pk_head[pd_s]};
          pd_part <= 1'b0;
          pk_beat[pd_s] <= pk_beat[pd_s] + 13'd1;
          if (last) begin
            pd_act <= 1'b0;
            pk_cnt[pd_s] <= '0;
            pk_reuse[pd_s] <= 1'b1;
          end
        end
      end else if (abort_req) begin
        pd_act  <= 1'b1;
        pd_s    <= s_slot;
        pd_part <= pk_nb[s_slot] != 3'd0;
      end else if (take) begin : pk
        logic        s, act, p8, sh;
        logic [12:0] bt;
        logic [2:0]  nb;
        logic [14:0] lw;
        logic [79:0] acc;
        s = s_slot;
        act = pk_act[s];
        bt = pk_beat[s];
        nb = pk_nb[s];
        sh = pk_sh[s];
        lw = pk_lw[s];
        p8 = is_psk8(s_modcod);
        if (pk_cnt[s] == 0) begin
          nb = '0;
          sh = s_short;
          lw = last_w(p8, s_short);
          pk_sh[s] <= sh;
          pk_lw[s] <= lw;
          // a frame starts: the head entry if the ring has room
          if (pk_reuse[s]) act = 1'b1;
          else if (occ[s] < nf) begin act = 1'b1; occ_inc[s] = 1'b1; end
          else begin
            act = 1'b0;
            if (s) begin if (st_dropped1 != 16'hFFFF) st_dropped1 <= st_dropped1 + 16'd1; end
            else   begin if (st_dropped0 != 16'hFFFF) st_dropped0 <= st_dropped0 + 16'd1; end
          end
          pk_reuse[s] <= 1'b0;
          bt = '0;
          if (act) begin
            meta_we <= 1'b1;
            meta_wa <= {s, pk_head[s]};
            meta_wd <= {s_short, s_epoch, s_pilots, s_modcod};
          end
        end
        pk_act[s] <= act;
        if (act) begin
          // append the word's 2 or 3 bytes; a full beat goes to staging
          logic [3:0] tot;
          acc = (nb == 0) ? 80'd0 : {24'd0, pk_acc[s]};   // (after an abort pk_acc is stale)
          acc = acc | ((p8 ? {56'd0, s_llr_z, s_llr_q, s_llr_i} : {64'd0, s_llr_q, s_llr_i}) << (8 * nb));
          tot = 4'(nb) + (p8 ? 4'd3 : 4'd2);
          nb = tot[2:0];                     // (mod 8)
          if (tot >= 4'd8) begin
            push   <= 1'b1;
            push_s <= s;
            push_d <= acc[63:0];
            push_f <= {sh, 1'b0, bt == 0, pk_head[s]};
            bt = bt + 13'd1;
            acc = acc >> 64;
          end
          pk_acc[s] <= acc[55:0];
          pk_nb[s]  <= nb;
        end
        pk_beat[s] <= bt;
        if (pk_cnt[s] == lw) begin
          pk_cnt[s] <= '0;
          if (act) pk_head[s] <= next_f(pk_head[s]);
        end else
          pk_cnt[s] <= pk_cnt[s] + 15'd1;
      end
      for (int s = 0; s < 2; s++) occ[s] <= occ[s] + 5'(occ_inc[s]) - 5'(occ_dec[s]);
    end
    if (clr_dropped[0]) st_dropped0 <= '0;
    if (clr_dropped[1]) st_dropped1 <= '0;
  end

  // a ring entry is free once its last beat is read
  assign occ_dec[0] = m_axi_rvalid && !lrst && rd_act && !rd_s && rd_rcv == rd_rlast;
  assign occ_dec[1] = m_axi_rvalid && !lrst && rd_act &&  rd_s && rd_rcv == rd_rlast;

endmodule
