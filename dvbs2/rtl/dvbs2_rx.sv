// DVB-S2 receiver core, QPSK normal frames, two receive slots
// (docs/rx2_design.md): per slot 2 samples/symbol in, BBFRAMEs of both
// slots out on one stream, each tagged with its slot.
//
//   slot s samples -> FIFO s (512) -> dvbs2_rx_timing2 -> dvbs2_rx_framesync2
//   -> dvbs2_rx_carrier2 (64800 LLRs per data frame, tagged with the slot)
//   -> dvbs2_rx_llrq (DDR frame queue, m_axi) -> dvbs2_ldpc_dec -> dvbs2_bch_dec
//   -> BBFRAME bytes (Kbch/8 per frame, BB-descrambled), m_slot / m_modcod /
//   m_pilots: the frame's own; m_ok: BCH codeword (after <= t corrections),
//   m_ldpc_ok: LDPC converged
//
// The timing, frame-sync and carrier engines serve both slots (one context
// per slot, swapped per symbol); each slot has its own FIFO and PLS decoder
// (dvbs2_rx_pls: status, the frame sync's header confirmation). Per slot
// the receiver equals the single-slot model (model/rxfixed.py receive(),
// model/ldpcdec.py) fed with that slot's samples, whatever the other slot
// does. The two symbol rates together must stay below 2.38 MBd (42 clocks
// per symbol in the timing engine; leave a few percent).
//
// The sample inputs have no backpressure; samples arriving with a slot's
// FIFO full are dropped and counted (in_overflows). rst resets everything;
// rst_s[s] (one clock or longer) resets slot s alone: its FIFO, its context
// in the shared engines (dvbs2_rx_timing2.sv, the README "Two receive slots"),
// its PLS decoder and its counters here (in_overflows, the LDPC/BCH counts,
// the LLR queue's drop count). No frame of the slot from before the reset
// comes out after it: each slot's frames carry an epoch bit (toggled by the
// slot's reset) through the queue and the decoders, and a frame whose epoch
// is not the slot's current one (or of a slot in reset) is dropped at the
// output, whole (decided at its first byte).
//
// The LLR queue is always in the path (llrq_en: on; off, nothing is decoded):
// the frames of both slots go through the DDR ring (llrq_base, 2 MB) and are
// decoded in completion order, each with its own rate, its slot's
// ldpc_max_it and its tag (dvbs2_rx_llrq.sv).

module dvbs2_rx #(
  parameter string LUT_FILE = "nco_lut.mem",
  parameter string LDPC_ROM_FILE = "ldpc_dec_rom.mem",
  parameter string FIFO_RAM = "block"          // input FIFOs: "block" or "distributed"
) (
  input  logic               clk,
  input  logic               rst,
  input  logic [1:0]         rst_s,

  input  logic [1:0]         in_valid,
  input  logic signed [15:0] in_i [2],
  input  logic signed [15:0] in_q [2],
  output logic [15:0]        in_overflows [2],

  // configuration per slot (static while the slot runs; change under its reset)
  input  logic               coef_we,
  input  logic               coef_slot,
  input  logic [9:0]         coef_addr,
  input  logic signed [17:0] coef_data,
  input  logic [15:0]        timing_kp [2],
  input  logic [15:0]        timing_ki [2],
  input  logic signed [31:0] timing_integ0 [2],   // timing loop integrator at reset
  input  logic [4:0]         modcod [2],
  input  logic [1:0]         pilots,
  input  logic [31:0]        fs_thresh [2],
  input  logic [17:0]        gold_xa [2],
  input  logic [17:0]        gold_xb [2],
  input  logic [17:0]        pll_kp [2],
  input  logic [17:0]        pll_ki [2],
  input  logic [5:0]         ldpc_max_it [2],
  input  logic [5:0]         pls_h [2],        // PLS decoder grid -H..H
  input  logic [9:0]         pls_step [2],     // ... step, turns * 2^16 per symbol
  input  logic [31:0]        pls_seen_clr [2], // w1c of st_pls_seen
  // LLR queue (en, base, nfr static, change under rst); axi_rst: the AXI bus reset;
  // !llrq_en: the queue is off, the carrier's LLR words are dropped (nothing decoded)
  input  logic               axi_rst,
  input  logic               llrq_en,
  input  logic [31:0]        llrq_base,    // 2 MB aligned
  input  logic [4:0]         llrq_nfr,     // frames per slot ring, 1..16
  input  logic [5:0]         llrq_qhi,     // iteration cap while more than qhi frames wait
  input  logic [5:0]         llrq_it_lo,   // ... at it_lo iterations

  output logic               m_valid,
  input  logic               m_ready,
  output logic [7:0]         m_data,
  output logic               m_last,
  output logic               m_ok,
  output logic               m_ldpc_ok,
  output logic [5:0]         m_iters,
  output logic [4:0]         m_nerr,       // BCH errors located
  output logic [15:0]        m_raw_err,    // pre-FEC bit errors of the frame (if m_ok)
  // the frame's own parameters (taken with its first LLR word), constant in a frame
  output logic [3:0]         m_rate,       // code rate 0..10
  output logic [4:0]         m_modcod,
  output logic               m_pilots,
  output logic               m_slot,       // receive slot

  // constellation snapshot sources: carrier output (data symbols, phase
  // corrected, before the LLR gain; snap_c_slot: whose), timing loop output
  // per slot (all symbols)
  output logic               snap_c_v,
  output logic               snap_c_slot,
  output logic signed [15:0] snap_c_i,
  output logic signed [15:0] snap_c_q,
  output logic [1:0]         snap_t_v,
  output logic signed [15:0] snap_t_i [2],
  output logic signed [15:0] snap_t_q [2],

  // status per slot
  output logic [16:0]        st_agc_gain [2],
  output logic signed [31:0] st_timing_integ [2],
  output logic [1:0]         st_fs_state [2],
  output logic [15:0]        st_fs_found [2],
  output logic [15:0]        st_fs_flywheel [2],
  output logic [15:0]        st_fs_lost [2],
  output logic [31:0]        st_freq [2],
  output logic [31:0]        st_amp [2],
  output logic [35:0]        st_energy [2],
  output logic [17:0]        st_gain [2],
  output logic [15:0]        st_frames [2],
  output logic [15:0]        st_acq [2],
  output logic [15:0]        st_fa [2],
  output logic [15:0]        st_retype [2],
  // decoder counts per slot (frames of the slot out of the BCH decoder)
  output logic [15:0]        st_ldpc_frames [2],
  output logic [15:0]        st_ldpc_failed [2],
  output logic [15:0]        st_bch_corrected [2],
  output logic [15:0]        st_bch_failed [2],

  // PLS decoders (dvbs2_rx_pls), per slot: the last decode, the last confident data decode, counters
  output logic [1:0]         st_pls_v,       // pulse: a decode
  output logic [6:0]         st_pls_code [2],  // {MODCOD, short frame, pilots}
  output logic [3:0]         st_pls_flags [2], // {valid, grid searched, carrier hypothesis, confident}
  output logic [17:0]        st_pls_best [2],
  output logic [17:0]        st_pls_second [2],
  output logic [15:0]        st_pls_freq [2],  // turns * 2^16 per symbol
  output logic [15:0]        st_pls_count [2],
  output logic [6:0]         st_pls_dcode [2],
  output logic [3:0]         st_pls_dflags [2],
  output logic [15:0]        st_pls_dcount [2],
  output logic [15:0]        st_pls_diff_modcod [2],
  output logic [15:0]        st_pls_diff_pilots [2],
  output logic [31:0]        st_pls_seen [2],
  output logic [15:0]        st_pls_skipped [2],

  // LLR queue status
  output logic [5:0]         st_llrq_depth,
  output logic [5:0]         st_llrq_depth_max,
  output logic [15:0]        st_llrq_frames,
  output logic [15:0]        st_llrq_dropped [2],
  output logic [15:0]        st_llrq_axi_err,
  output logic               st_llrq_busy,

  // LLR queue: AXI3 master, 64 bits (Zynq S_AXI_HP1)
  output logic [31:0]        m_axi_awaddr,
  output logic [3:0]         m_axi_awlen,
  output logic [2:0]         m_axi_awsize,
  output logic [1:0]         m_axi_awburst,
  output logic [3:0]         m_axi_awcache,
  output logic               m_axi_awvalid,
  input  logic               m_axi_awready,
  output logic [63:0]        m_axi_wdata,
  output logic [7:0]         m_axi_wstrb,
  output logic               m_axi_wlast,
  output logic               m_axi_wvalid,
  input  logic               m_axi_wready,
  input  logic [1:0]         m_axi_bresp,
  input  logic               m_axi_bvalid,
  output logic               m_axi_bready,
  output logic [31:0]        m_axi_araddr,
  output logic [3:0]         m_axi_arlen,
  output logic [2:0]         m_axi_arsize,
  output logic [1:0]         m_axi_arburst,
  output logic [3:0]         m_axi_arcache,
  output logic               m_axi_arvalid,
  input  logic               m_axi_arready,
  input  logic [63:0]        m_axi_rdata,
  input  logic [1:0]         m_axi_rresp,
  input  logic               m_axi_rlast,
  input  logic               m_axi_rvalid,
  output logic               m_axi_rready
);

  // per-slot reset of the slot's own units (FIFO, PLS decoder, counters)
  wire [1:0] srst = rst_s | {2{rst}};

  // ---------------------------------------------------------------- input FIFOs
  logic [1:0]         w_ready, f_valid, f_ready;
  logic [31:0]        f_data [2];
  logic signed [15:0] f_i [2], f_q [2];

  for (genvar s = 0; s < 2; s++) begin : g_in
    always_ff @(posedge clk)
      if (srst[s]) in_overflows[s] <= '0;
      else if (in_valid[s] && !w_ready[s] && in_overflows[s] != 16'hFFFF) in_overflows[s] <= in_overflows[s] + 16'd1;

    dvbs2_sync_fifo #(.W(32), .AW(9), .RAM_STYLE(FIFO_RAM)) u_in_fifo (
      .clk, .rst(srst[s]), .w_valid(in_valid[s]), .w_ready(w_ready[s]), .w_data({in_q[s], in_i[s]}),
      .r_valid(f_valid[s]), .r_ready(f_ready[s]), .r_data(f_data[s]));
    assign f_i[s] = $signed(f_data[s][15:0]);
    assign f_q[s] = $signed(f_data[s][31:16]);
  end

  // ---------------------------------------------------------------- timing
  logic [1:0]         t_valid, t_ready;
  logic signed [15:0] t_i [2], t_q [2];

  dvbs2_rx_timing2 u_timing (
    .clk, .rst, .rst_s, .kp(timing_kp), .ki(timing_ki), .integ_init(timing_integ0),
    .coef_we, .coef_slot, .coef_addr, .coef_data,
    .s_valid(f_valid), .s_ready(f_ready), .s_i(f_i), .s_q(f_q),
    .m_valid(t_valid), .m_ready(t_ready), .m_i(t_i), .m_q(t_q),
    .agc_gain(st_agc_gain), .loop_integ(st_timing_integ));

  assign snap_t_v = t_valid & t_ready;
  assign snap_t_i = t_i;
  assign snap_t_q = t_q;

  // ---------------------------------------------------------------- frame sync
  logic               y_valid, y_ready, y_sof, y_dummy, y_found, y_slot;
  logic signed [15:0] y_i, y_q;
  logic [1:0]         ty_v2, pls_busy2;               // carrier / PLS decoders -> frame sync
  logic               ty_dummy;
  logic [1:0]         pls_conf2;

  for (genvar s = 0; s < 2; s++) begin : g_pconf
    assign pls_conf2[s] = st_pls_flags[s][0];
  end

  dvbs2_rx_framesync2 u_fs (
    .clk, .rst, .rst_s, .modcod2(modcod), .pilots2(pilots), .thresh2(fs_thresh),
    .s_valid2(t_valid), .s_ready2(t_ready), .s_i2(t_i), .s_q2(t_q),
    .m_valid(y_valid), .m_ready(y_ready), .m_i(y_i), .m_q(y_q), .m_sof(y_sof), .m_dummy(y_dummy),
    .m_found(y_found), .m_slot(y_slot), .ty_v2, .ty_dummy,
    .pls_v2(st_pls_v), .pls_conf2, .pls_code2(st_pls_code), .pls_busy2,
    .fs_state2(st_fs_state), .cnt_found2(st_fs_found), .cnt_flywheel2(st_fs_flywheel), .cnt_lost2(st_fs_lost));

  // ---------------------------------------------------------------- carrier
  logic [1:0]         c_in_valid, c_in_ready, c_acquired;
  logic signed [15:0] y_i2 [2], y_q2 [2];
  logic               l_valid, l_ready, l_first, l_last, l_slot;
  logic [7:0]         l_i, l_q;

  assign y_i2[0] = y_i; assign y_i2[1] = y_i;
  assign y_q2[0] = y_q; assign y_q2[1] = y_q;
  assign c_in_valid = {y_valid && y_slot, y_valid && !y_slot};
  assign y_ready = c_in_ready[y_slot];

  dvbs2_rx_carrier2 #(.LUT_FILE(LUT_FILE)) u_carrier (
    .clk, .rst, .rst_s, .modcod2(modcod), .pilots2(pilots), .gold_xa2(gold_xa), .gold_xb2(gold_xb),
    .pll_kp2(pll_kp), .pll_ki2(pll_ki),
    .s_valid2(c_in_valid), .s_ready2(c_in_ready), .s_i2(y_i2), .s_q2(y_q2),
    .s_sof2({2{y_sof}}), .s_dummy2({2{y_dummy}}), .s_found2({2{y_found}}),
    .ty_v2, .ty_dummy,
    .m_valid(l_valid), .m_ready(l_ready), .m_llr_i(l_i), .m_llr_q(l_q), .m_first(l_first), .m_last(l_last),
    .m_slot(l_slot),
    .st_freq2(st_freq), .st_amp2(st_amp), .st_energy2(st_energy), .st_gain2(st_gain), .st_frames2(st_frames),
    .st_acq2(st_acq), .st_fa2(st_fa), .st_retype2(st_retype), .st_acquired2(c_acquired),
    .snap_v(snap_c_v), .snap_i(snap_c_i), .snap_q(snap_c_q), .snap_slot(snap_c_slot));

  // ---------------------------------------------------------------- PLS decoders, one per slot
  for (genvar s = 0; s < 2; s++) begin : g_pls
    dvbs2_rx_pls u_pls (
      .clk, .rst(srst[s]), .modcod(modcod[s]), .pilots(pilots[s]), .cfg_h(pls_h[s]), .cfg_step(pls_step[s]),
      .s_v(y_valid && y_ready && y_slot == s[0]), .s_i(y_i), .s_q(y_q), .s_sof(y_sof),
      .fhat(st_freq[s]), .acquired(c_acquired[s]), .seen_clr(pls_seen_clr[s]),
      .res_v(st_pls_v[s]), .res_code(st_pls_code[s]), .res_conf(st_pls_flags[s][0]),
      .res_car(st_pls_flags[s][1]), .res_grid(st_pls_flags[s][2]), .res_best(st_pls_best[s]),
      .res_second(st_pls_second[s]), .res_freq(st_pls_freq[s]), .res_valid(st_pls_flags[s][3]),
      .st_count(st_pls_count[s]),
      .data_code(st_pls_dcode[s]), .data_conf(st_pls_dflags[s][0]), .data_car(st_pls_dflags[s][1]),
      .data_grid(st_pls_dflags[s][2]), .data_valid(st_pls_dflags[s][3]), .st_data(st_pls_dcount[s]),
      .st_diff_modcod(st_pls_diff_modcod[s]), .st_diff_pilots(st_pls_diff_pilots[s]),
      .st_seen(st_pls_seen[s]), .st_skipped(st_pls_skipped[s]), .dec_busy(pls_busy2[s]));
  end

  // ---------------------------------------------------------------- LLR queue
  logic       d_valid, d_ready, d_first, d_slot, d_pilots;
  logic [7:0] d_i, d_q;
  logic [3:0] d_rate;
  logic [5:0] d_max_it;
  logic [4:0] d_modcod;
  logic       cap_en;
  logic [5:0] cap;
  logic       q_ready;
  logic       d_epoch;
  logic [1:0] epoch, rst_s_q;                 // per slot: toggled by its reset

  always_ff @(posedge clk) begin
    rst_s_q <= rst_s;
    if (rst) epoch <= '0;
    else epoch <= epoch ^ (rst_s & ~rst_s_q);
  end

  assign l_ready = q_ready || !llrq_en;

  dvbs2_rx_llrq u_llrq (
    .clk, .arst(axi_rst), .rst, .en(llrq_en), .base(llrq_base), .nfr(llrq_nfr),
    .max_it0(ldpc_max_it[0]), .max_it1(ldpc_max_it[1]), .qhi(llrq_qhi), .it_lo(llrq_it_lo),
    .clr_dropped(rst_s),
    .s_valid(l_valid && llrq_en), .s_ready(q_ready), .s_llr_i(l_i), .s_llr_q(l_q), .s_first(l_first),
    .s_slot(l_slot), .s_modcod(modcod[l_slot]), .s_pilots(pilots[l_slot]), .s_epoch(epoch[l_slot]),
    .m_valid(d_valid), .m_ready(d_ready), .m_llr_i(d_i), .m_llr_q(d_q), .m_first(d_first),
    .m_rate(d_rate), .m_max_it(d_max_it), .m_slot(d_slot), .m_modcod(d_modcod), .m_pilots(d_pilots), .m_epoch(d_epoch),
    .it_cap_en(cap_en), .it_cap(cap),
    .st_depth(st_llrq_depth), .st_depth_max(st_llrq_depth_max), .st_frames(st_llrq_frames),
    .st_dropped0(st_llrq_dropped[0]), .st_dropped1(st_llrq_dropped[1]), .st_axi_err(st_llrq_axi_err),
    .st_busy(st_llrq_busy),
    .m_axi_awaddr, .m_axi_awlen, .m_axi_awsize, .m_axi_awburst, .m_axi_awcache, .m_axi_awvalid,
    .m_axi_awready, .m_axi_wdata, .m_axi_wstrb, .m_axi_wlast, .m_axi_wvalid, .m_axi_wready,
    .m_axi_bresp, .m_axi_bvalid, .m_axi_bready, .m_axi_araddr, .m_axi_arlen, .m_axi_arsize,
    .m_axi_arburst, .m_axi_arcache, .m_axi_arvalid, .m_axi_arready, .m_axi_rdata, .m_axi_rresp,
    .m_axi_rlast, .m_axi_rvalid, .m_axi_rready);

  // ---------------------------------------------------------------- LDPC, BCH
  // per-frame tag through the LDPC and BCH decoders: {epoch, slot, pilots, MODCOD}
  logic       c_valid, c_ready, c_last, c_ok;   // LDPC -> BCH
  logic [7:0] c_data, c_chan;
  logic [5:0] c_iters;
  logic [3:0] c_rate;
  logic [7:0] c_tag, m_tag;
  logic       x_valid, x_ready;

  dvbs2_ldpc_dec #(.ROM_FILE(LDPC_ROM_FILE)) u_ldpc (
    .clk, .rst,
    .s_valid(d_valid), .s_ready(d_ready), .s_llr_i(d_i), .s_llr_q(d_q), .s_first(d_first),
    .s_rate(d_rate), .s_max_it(d_max_it), .s_tag({d_epoch, d_slot, d_pilots, d_modcod}),
    .it_cap_en(cap_en), .it_cap(cap),
    .m_valid(c_valid), .m_ready(c_ready), .m_data(c_data), .m_last(c_last), .m_ok(c_ok), .m_iters(c_iters), .m_chan(c_chan),
    .m_rate(c_rate), .m_tag(c_tag),
    .st_frames(), .st_failed());

  dvbs2_bch_dec u_bch (
    .clk, .rst,
    .s_valid(c_valid), .s_ready(c_ready), .s_data(c_data), .s_last(c_last), .s_ok(c_ok), .s_iters(c_iters), .s_chan(c_chan),
    .s_rate(c_rate), .s_tag(c_tag),
    .m_valid(x_valid), .m_ready(x_ready), .m_data, .m_last, .m_ok, .m_ldpc_ok, .m_iters, .m_nerr, .m_raw_err,
    .m_rate, .m_tag,
    .st_frames(), .st_corrected(), .st_failed());

  // frames of a slot from before its reset are dropped (whole)
  logic x_in, x_fresh;
  wire  x_fresh_now = m_tag[7] == epoch[m_tag[6]] && !rst_s[m_tag[6]];
  wire  x_keep = x_in ? x_fresh : x_fresh_now;
  assign m_valid = x_valid && x_keep;
  assign x_ready = m_ready || !x_keep;
  always_ff @(posedge clk)
    if (rst) x_in <= 1'b0;
    else if (x_valid && x_ready) begin
      if (!x_in) x_fresh <= x_fresh_now;
      x_in <= !m_last;
    end

  assign m_modcod = m_tag[4:0];
  assign m_pilots = m_tag[5];
  assign m_slot   = m_tag[6];

  // decoder counts per slot, at the BCH output (as the decoders count them)
  for (genvar s = 0; s < 2; s++) begin : g_cnt
    always_ff @(posedge clk)
      if (srst[s]) begin
        st_ldpc_frames[s] <= '0; st_ldpc_failed[s] <= '0; st_bch_corrected[s] <= '0; st_bch_failed[s] <= '0;
      end else if (m_valid && m_ready && m_last && m_slot == s[0]) begin
        st_ldpc_frames[s] <= st_ldpc_frames[s] + 16'd1;
        if (!m_ldpc_ok) st_ldpc_failed[s] <= st_ldpc_failed[s] + 16'd1;
        if (!m_ok) st_bch_failed[s] <= st_bch_failed[s] + 16'd1;
        else if (m_nerr != 0) st_bch_corrected[s] <= st_bch_corrected[s] + 16'd1;
      end
  end

endmodule
