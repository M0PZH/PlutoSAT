// DVB-S2 RX for the Zynq block design: AXI4-Lite registers, ADC input
// (axi_ad9361 channel 0, l_clk domain), DDC, receiver core, and one
// fixed-size DMA packet per decoded BBFRAME.
//
//   ADC (adc_clk) -> async FIFO -> dvbs2_rx_ddc -> dvbs2_rx -> packet buffer -> m_axis
//
// Packet (PKT_WORDS 64-bit words, tlast on the last; byte 0 = tdata[7:0]):
//   word 0  [15:0] BBFRAME length in bytes (Kbch/8), [31:16] frame counter,
//           [36:32] MODCOD, [37] pilots, [45:40] LDPC iterations,
//           [48] LDPC converged, [49] BCH ok (the BBFRAME is a codeword after
//           <= t corrections: use it), [54:50] BCH errors located, [63:56] 0xB5
//   word 1+ BBFRAME bytes (BB-descrambled), then zeros
// A frame that finds both packet slots full is dropped (PKT_DROPPED).
//
// Register map (byte offsets, 32-bit):
//   0x00 VERSION      RO  0x0001_0200: PLS decoder and TIMING_INTEG preset (0x0001_0100:
//                         neither, 0xB8.. read 0, a TIMING_INTEG write is ignored;
//                         0x0001_0000: no CIC, DDC_CIC reads 0)
//   0x04 ID           RO  "DVR2" (0x44565232)
//   0x08 SCRATCH      RW
//   0x10 CTRL         RW  [0] enable (take ADC samples), [1] soft reset (self-clearing)
//   0x14 MODCOD       RW  [4:0] MODCOD (QPSK: 1..11), [8] pilots
//   0x18 GOLD_XA      RW  [17:0]
//   0x1C GOLD_XB      RW  [17:0]
//   0x20 DDC_FINC     RW  NCO phase increment per ADC sample (2^32 = ADC rate)
//   0x24 DDC_CFG      RW  [5:0] FIR decimation D (1..63), [17:8] taps (even, <= 512), [28:24] shift
//   0x28 DDC_COEF_ADDR RW [8:0]
//   0x2C DDC_COEF_DATA WO [17:0], then DDC_COEF_ADDR++
//   0x30 RRC_COEF_ADDR RW [9:0] {phase[4:0], tap[4:0]}
//   0x34 RRC_COEF_DATA WO [17:0], then RRC_COEF_ADDR++
//   0x38 TIMING_GAINS RW  [15:0] kp, [31:16] ki
//   0x3C FS_THRESH    RW  header metric threshold
//   0x40 PLL_KP       RW  [17:0]
//   0x44 PLL_KI       RW  [17:0]
//   0x48 LDPC_MAX_IT  RW  [5:0]
//   0x4C DDC_CIC      RW  [5:0] CIC decimation R (0, 1: no CIC; 2..32), [11:8] CIC output shift;
//                         the total decimation is R * D (dvbs2_rx_ddc.sv; model/rxfixed.py ddc_config())
//   0x50 IN_OVERFLOWS RO  2-sps samples lost at the core input FIFO
//   0x54 DDC_DROPPED  RO  DDC outputs lost (input too fast for the taps)
//   0x58 AGC_GAIN     RO  Q5.12
//   0x5C TIMING_INTEG R   timing loop integrator (Q.24 samples/symbol beyond 2)
//                     W   its value at the core reset (default 0): a known
//                         symbol-rate offset, 2 (nominal / actual - 1) x 2^24
//                         (model/rxfixed.py timing_integ_preset())
//   0x60 FS_STATE     RO  [1:0] 0 search, 1 check, 2 locked
//   0x64 FS_FOUND     RO  0x68 FS_FLYWHEEL RO  0x6C FS_LOST RO
//   0x70 FREQ         RO  carrier frequency, turns * 2^32 per symbol
//   0x74 AMP          RO  0x78 ENERGY RO ([35:4]) 0x7C LLR_GAIN RO
//   0x80 FRAMES       RO  data frames demodulated
//   0x84 LDPC_FRAMES  RO  0x88 LDPC_FAILED RO
//   0x8C PKT_DROPPED  RO  0x90 PKT_SENT RO
//   0x94 CARRIER_ACQ  RO  [15:0] frequency acquisitions, [31:16] 1/1476 false-lock corrections
//   0x98 CARRIER_RETYPE RO [15:0] frame types changed by the coherent decision
//   0x9C BCH          RO  [15:0] frames corrected, [31:16] frames not decodable
//   0xA0 SNAP_CTRL    W   [0] arm: capture the next 1024 symbols, [1] source (0: carrier
//                         output, data symbols before the LLR gain; 1: timing loop output)
//                     R   [0] busy, [1] done, [2] source
//   0xA4 SNAP_ADDR    RW  [9:0] read address
//   0xA8 SNAP_DATA    RO  [15:0] I, [31:16] Q of symbol SNAP_ADDR, then SNAP_ADDR++
//   0xAC RAW_ERR      RO  pre-FEC bit errors (channel hard decisions vs. the decoded
//                         frame, Kldpc bits of each BCH-good frame), wraps
//   0xB0 RAW_BITS     RO  bits counted in RAW_ERR, wraps
//   0xB4 BCH_BITERR   RO  bit errors corrected by BCH (post-LDPC errors), wraps
//   PLS decoder (dvbs2_rx_pls.sv, model/rxfixed.py pls_decode()): the PLS code of
//   every PLHEADER the frame sync tags, whatever MODCOD/pilots are configured
//   (detection only). Confident: best > 2 x second and best > 2^8.
//   0xB8 PLS_LAST     RO  the last decode: [4:0] MODCOD, [8] pilots, [9] short frame,
//                         [12] confident, [13] from the carrier's frequency, [14] the
//                         frequency grid was searched, [15] valid (a decode since reset),
//                         [31:16] decodes (wraps)
//   0xBC PLS_DATA     RO  the last confident decode of a data header (MODCOD != 0), same
//                         fields, [15] valid, [31:16] confident data decodes (wraps)
//   0xC0 PLS_FREQ     RO  frequency of the last decode's hypothesis, turns * 2^32 per
//                         symbol (multiples of 2^16; relative to the DDC, like FREQ)
//   0xC4 PLS_BEST     RO  [17:0] metric of the last decode (|correlation|^2 / 256; a
//                         header at -3 dB Es/N0 gives >= ~300, noise alone <= ~220)
//   0xC8 PLS_SECOND   RO  [17:0] largest metric of another code at the same frequency
//   0xCC PLS_DIFF     RO  confident data decodes whose [15:0] MODCOD or frame size,
//                         [31:16] pilots differ from the configured ones (wrap)
//   0xD0 PLS_SEEN     R/W1C bit m: a confident normal-frame decode of MODCOD m (bit 0:
//                         dummy frames) since cleared (a clear wins over a decode in
//                         the same clock)
//   0xD4 PLS_CFG      RW  [5:0] H, [25:16] step (turns * 2^16 per symbol): frequency
//                         hypotheses h * step, h = -H..H (default 32, 262: +-0.128
//                         cycles/symbol), after the carrier's frequency once acquired;
//                         ~1420 clocks per hypothesis (2H + 2 of them) must fit
//                         between two headers (94k clocks: 2.38 MBd dummy frames)
//   0xD8 PLS_SKIPPED  RO  [15:0] headers not decoded (decoder still busy), wraps

module dvbs2_rx_axi #(
  parameter string LUT_FILE      = "nco_lut.mem",
  parameter string LDPC_ROM_FILE = "ldpc_dec_rom.mem",
  parameter int    PKT_WORDS     = 1024
) (
  input  logic        s_axi_aclk,
  input  logic        s_axi_aresetn,
  input  logic [7:0]  s_axi_awaddr,
  input  logic        s_axi_awvalid,
  output logic        s_axi_awready,
  input  logic [31:0] s_axi_wdata,
  input  logic [3:0]  s_axi_wstrb,
  input  logic        s_axi_wvalid,
  output logic        s_axi_wready,
  output logic [1:0]  s_axi_bresp,
  output logic        s_axi_bvalid,
  input  logic        s_axi_bready,
  input  logic [7:0]  s_axi_araddr,
  input  logic        s_axi_arvalid,
  output logic        s_axi_arready,
  output logic [31:0] s_axi_rdata,
  output logic [1:0]  s_axi_rresp,
  output logic        s_axi_rvalid,
  input  logic        s_axi_rready,

  output logic [63:0] m_axis_tdata,
  output logic        m_axis_tvalid,
  input  logic        m_axis_tready,
  output logic        m_axis_tlast,

  input  logic        adc_clk,
  input  logic        adc_valid,
  input  logic [15:0] adc_data_i,
  input  logic [15:0] adc_data_q
);

  wire clk = s_axi_aclk;

  // ---------------------------------------------------------------- registers
  logic [31:0] scratch, ddc_finc, fs_thresh;
  logic [1:0]  ctrl;
  logic [4:0]  modcod;
  logic        pilots;
  logic [17:0] gold_xa, gold_xb, pll_kp, pll_ki;
  logic [5:0]  ddc_dec, max_it;
  logic [9:0]  ddc_ntaps;
  logic [4:0]  ddc_shift;
  logic [5:0]  ddc_cic_r;
  logic [3:0]  ddc_cic_shift;
  logic [8:0]  ddc_caddr;
  logic [9:0]  rrc_caddr;
  logic        ddc_cwe, rrc_cwe;
  logic [17:0] cdata;
  logic [15:0] t_kp, t_ki;
  logic [5:0]  pls_h;
  logic [9:0]  pls_step;
  logic [31:0] pls_seen_clr;
  logic [31:0] t_integ0;

  logic [3:0] srst_cnt;
  logic       core_rst;
  logic       aresetn_q;                     // local copy (the core reset has a large fanout)
  always_ff @(posedge clk) begin
    aresetn_q <= s_axi_aresetn;
    if (!aresetn_q)         srst_cnt <= '1;
    else if (ctrl[1])       srst_cnt <= '1;
    else if (srst_cnt != 0) srst_cnt <= srst_cnt - 1'b1;
    core_rst <= !aresetn_q || ctrl[1] || (srst_cnt != 0);
  end

  logic        aw_hold, w_hold;
  logic [7:0]  awaddr_q;
  logic [31:0] wdata_q;
  assign s_axi_awready = !aw_hold;
  assign s_axi_wready  = !w_hold;
  assign s_axi_bresp   = 2'b00;
  assign s_axi_rresp   = 2'b00;

  logic        snap_arm, snap_src, snap_ra_we;
  logic [9:0]  snap_ra_d;

  always_ff @(posedge clk) begin
    ddc_cwe <= 1'b0;
    rrc_cwe <= 1'b0;
    snap_arm <= 1'b0;
    snap_ra_we <= 1'b0;
    pls_seen_clr <= '0;
    if (!s_axi_aresetn) begin
      aw_hold <= 1'b0; w_hold <= 1'b0; s_axi_bvalid <= 1'b0;
      ctrl <= '0; scratch <= '0;
      modcod <= 5'd4; pilots <= 1'b1;
      gold_xa <= 18'h00001; gold_xb <= 18'h01008;
      ddc_finc <= '0; ddc_dec <= 6'd1; ddc_ntaps <= 10'd2; ddc_shift <= 5'd16;
      ddc_cic_r <= '0; ddc_cic_shift <= '0;
      ddc_caddr <= '0; rrc_caddr <= '0;
      t_kp <= dvbs2_rx_pkg::TIMING_KP_DEFAULT;
      t_ki <= dvbs2_rx_pkg::TIMING_KI_DEFAULT;
      t_integ0 <= '0;
      fs_thresh <= dvbs2_rx_pkg::FS_THRESH_DEFAULT;
      pll_kp <= dvbs2_rx_pkg::PLL_KP_DEFAULT;
      pll_ki <= dvbs2_rx_pkg::PLL_KI_DEFAULT;
      max_it <= 6'd30;
      snap_src <= 1'b0;
      pls_h <= dvbs2_rx_pkg::PLS_H_DEFAULT;
      pls_step <= dvbs2_rx_pkg::PLS_STEP_DEFAULT;
    end else begin
      if (s_axi_awvalid && s_axi_awready) begin aw_hold <= 1'b1; awaddr_q <= s_axi_awaddr; end
      if (s_axi_wvalid && s_axi_wready)   begin w_hold  <= 1'b1; wdata_q  <= s_axi_wdata;  end
      if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
      if (aw_hold && w_hold && !s_axi_bvalid) begin
        aw_hold <= 1'b0; w_hold <= 1'b0; s_axi_bvalid <= 1'b1;
        case (awaddr_q[7:2])
          6'h02: scratch <= wdata_q;
          6'h04: ctrl <= wdata_q[1:0];
          6'h05: begin modcod <= wdata_q[4:0]; pilots <= wdata_q[8]; end
          6'h06: gold_xa <= wdata_q[17:0];
          6'h07: gold_xb <= wdata_q[17:0];
          6'h08: ddc_finc <= wdata_q;
          6'h09: begin ddc_dec <= wdata_q[5:0]; ddc_ntaps <= wdata_q[17:8]; ddc_shift <= wdata_q[28:24]; end
          6'h0A: ddc_caddr <= wdata_q[8:0];
          6'h0B: begin cdata <= wdata_q[17:0]; ddc_cwe <= 1'b1; end
          6'h0C: rrc_caddr <= wdata_q[9:0];
          6'h0D: begin cdata <= wdata_q[17:0]; rrc_cwe <= 1'b1; end
          6'h0E: begin t_kp <= wdata_q[15:0]; t_ki <= wdata_q[31:16]; end
          6'h0F: fs_thresh <= wdata_q;
          6'h10: pll_kp <= wdata_q[17:0];
          6'h11: pll_ki <= wdata_q[17:0];
          6'h12: max_it <= wdata_q[5:0];
          6'h13: begin ddc_cic_r <= wdata_q[5:0]; ddc_cic_shift <= wdata_q[11:8]; end
          6'h17: t_integ0 <= wdata_q;
          6'h28: begin snap_arm <= wdata_q[0]; snap_src <= wdata_q[1]; end
          6'h29: begin snap_ra_we <= 1'b1; snap_ra_d <= wdata_q[9:0]; end
          6'h34: pls_seen_clr <= wdata_q;
          6'h35: begin pls_h <= wdata_q[5:0]; pls_step <= wdata_q[25:16]; end
          default: ;
        endcase
      end
      if (ddc_cwe) ddc_caddr <= ddc_caddr + 9'd1;
      if (rrc_cwe) rrc_caddr <= rrc_caddr + 10'd1;
      if (ctrl[1] && srst_cnt == 4'hF) ctrl[1] <= 1'b0;
    end
  end

  // ---------------------------------------------------------------- ADC -> DDC
  // enable and reset cross into the ADC domain
  (* ASYNC_REG = "TRUE" *) logic [1:0] en_s, rst_s;
  always_ff @(posedge adc_clk) begin
    en_s  <= {en_s[0], ctrl[0]};
    rst_s <= {rst_s[0], core_rst};
  end

  logic        a_valid, a_ready;
  logic [31:0] a_data;
  dvbs2_async_fifo #(.W(32), .AW(5)) u_adc_fifo (
    .wclk(adc_clk), .wrst(rst_s[1]), .w_valid(adc_valid && en_s[1]), .w_ready(), .w_data({adc_data_q, adc_data_i}),
    .rclk(clk), .rrst(core_rst), .r_valid(a_valid), .r_ready(a_ready), .r_data(a_data));

  logic               d_valid;
  logic signed [15:0] d_i, d_q;
  logic [15:0]        ddc_dropped;

  dvbs2_rx_ddc #(.LUT_FILE(LUT_FILE)) u_ddc (
    .clk, .rst(core_rst), .finc(ddc_finc), .dec(ddc_dec), .ntaps(ddc_ntaps), .shift(ddc_shift),
    .cic_r(ddc_cic_r), .cic_shift(ddc_cic_shift),
    .coef_we(ddc_cwe), .coef_addr(ddc_caddr), .coef_data(cdata),
    .s_valid(a_valid), .s_ready(a_ready), .s_i($signed(a_data[15:0])), .s_q($signed(a_data[31:16])),
    .m_valid(d_valid), .m_i(d_i), .m_q(d_q), .st_dropped(ddc_dropped));

  // ---------------------------------------------------------------- receiver core
  logic        b_valid, b_last, b_ok, b_ldpc_ok;
  logic [4:0]  b_nerr;
  logic [15:0] b_raw_err;
  logic        snap_c_v, snap_t_v;
  logic signed [15:0] snap_c_i, snap_c_q, snap_t_i, snap_t_q;
  logic [15:0] bch_corr, bch_failed;
  logic [7:0]  b_data;
  logic [5:0]  b_iters;
  logic [15:0] in_ovf, fs_found, fs_fly, fs_lost, frames, ldpc_frames, ldpc_failed;
  logic [16:0] agc_gain;
  logic signed [31:0] t_integ;
  logic [1:0]  fs_state;
  logic [31:0] st_freq, st_amp;
  logic [35:0] st_energy;
  logic [17:0] st_gain;
  logic [15:0] st_acq, st_fa, st_retype;
  logic        pls_v;
  logic [6:0]  pls_code, pls_dcode;
  logic [3:0]  pls_flags, pls_dflags;
  logic [17:0] pls_best, pls_second;
  logic [31:0] pls_seen;
  logic [15:0] pls_freq, pls_count, pls_dcount, pls_diff_m, pls_diff_p, pls_skipped;

  dvbs2_rx #(.LUT_FILE(LUT_FILE), .LDPC_ROM_FILE(LDPC_ROM_FILE)) u_rx (
    .in_clk(clk), .in_rst(core_rst), .in_valid(d_valid), .in_i(d_i), .in_q(d_q), .in_overflows(in_ovf),
    .clk, .rst(core_rst),
    .coef_we(rrc_cwe), .coef_addr(rrc_caddr), .coef_data(cdata),
    .timing_kp(t_kp), .timing_ki(t_ki), .timing_integ0($signed(t_integ0)), .modcod, .pilots, .fs_thresh, .gold_xa, .gold_xb,
    .pll_kp, .pll_ki, .ldpc_max_it(max_it),
    .pls_h, .pls_step, .pls_seen_clr,
    .m_valid(b_valid), .m_ready(1'b1), .m_data(b_data), .m_last(b_last), .m_ok(b_ok), .m_ldpc_ok(b_ldpc_ok),
    .m_iters(b_iters), .m_nerr(b_nerr), .m_raw_err(b_raw_err),
    .snap_c_v, .snap_c_i, .snap_c_q, .snap_t_v, .snap_t_i, .snap_t_q,
    .st_agc_gain(agc_gain), .st_timing_integ(t_integ), .st_fs_state(fs_state),
    .st_fs_found(fs_found), .st_fs_flywheel(fs_fly), .st_fs_lost(fs_lost),
    .st_freq, .st_amp, .st_energy, .st_gain, .st_frames(frames), .st_acq, .st_fa, .st_retype,
    .st_ldpc_frames(ldpc_frames), .st_ldpc_failed(ldpc_failed),
    .st_bch_corrected(bch_corr), .st_bch_failed(bch_failed),
    .st_pls_v(pls_v), .st_pls_code(pls_code), .st_pls_flags(pls_flags), .st_pls_best(pls_best),
    .st_pls_second(pls_second), .st_pls_freq(pls_freq), .st_pls_count(pls_count),
    .st_pls_dcode(pls_dcode), .st_pls_dflags(pls_dflags), .st_pls_dcount(pls_dcount),
    .st_pls_diff_modcod(pls_diff_m), .st_pls_diff_pilots(pls_diff_p),
    .st_pls_seen(pls_seen), .st_pls_skipped(pls_skipped));

  // {MODCOD, short, pilots} + {valid, grid, carrier, confident} -> PLS_LAST / PLS_DATA fields
  function automatic logic [15:0] pls_word(input logic [6:0] c, input logic [3:0] fl);
    pls_word = {fl, 2'd0, c[1], c[0], 3'd0, c[6:2]};
  endfunction

  // ---------------------------------------------------------------- constellation snapshot
  (* ram_style = "block" *) logic [31:0] snap_mem [1024];
  logic        snap_busy, snap_done, snap_rd;
  logic [9:0]  snap_wa, snap_ra;
  logic [31:0] snap_q;
  wire         snap_v = snap_src ? snap_t_v : snap_c_v;
  wire  [31:0] snap_d = snap_src ? {snap_t_q, snap_t_i} : {snap_c_q, snap_c_i};

  always_ff @(posedge clk) begin
    if (snap_v && snap_busy) snap_mem[snap_wa] <= snap_d;
    snap_q <= snap_mem[snap_ra];
  end

  always_ff @(posedge clk) begin
    if (!s_axi_aresetn) begin
      snap_busy <= 1'b0; snap_done <= 1'b0; snap_wa <= '0; snap_ra <= '0;
    end else begin
      if (snap_arm) begin
        snap_busy <= 1'b1; snap_done <= 1'b0; snap_wa <= '0;
      end else if (snap_v && snap_busy) begin
        snap_wa <= snap_wa + 10'd1;
        if (snap_wa == 10'd1023) begin snap_busy <= 1'b0; snap_done <= 1'b1; end
      end
      if (snap_ra_we) snap_ra <= snap_ra_d;
      else if (snap_rd) snap_ra <= snap_ra + 10'd1;
    end
  end

  // ---------------------------------------------------------------- error counters
  logic [31:0] raw_err, raw_bits, bch_biterr;
  always_ff @(posedge clk) begin
    if (core_rst) begin
      raw_err <= '0; raw_bits <= '0; bch_biterr <= '0;
    end else if (b_valid && b_last && b_ok) begin
      raw_err    <= raw_err + 32'(b_raw_err);
      raw_bits   <= raw_bits + 32'(dvbs2_bch_pkg::BCH_N[4'(modcod - 5'd1)]);
      bch_biterr <= bch_biterr + 32'(b_nerr);
    end
  end

  // ---------------------------------------------------------------- packets
  localparam int PW = $clog2(PKT_WORDS);
  (* ram_style = "block" *) logic [63:0] pbuf [2 * PKT_WORDS];
  logic [1:0]    slot_full;
  logic [PW:0]   slot_nw [2];               // words with data (header + BBFRAME)
  logic          wslot, rslot;
  logic          w_act, w_drop;
  logic [PW-1:0] w_word;                    // next data word
  logic [2:0]    w_byte;
  logic [63:0]   w_acc;
  logic [15:0]   w_len, pkt_cnt, pkt_dropped, pkt_sent;
  logic          hdr_pending;
  logic [63:0]   hdr_word;
  logic          pb_we;
  logic [PW:0]   pb_waddr;
  logic [63:0]   pb_wdata;
  logic          clr_full, clr_slot;

  always_ff @(posedge clk) begin
    pb_we <= 1'b0;
    if (core_rst) begin
      w_act <= 1'b0; w_drop <= 1'b0; hdr_pending <= 1'b0;
      pkt_cnt <= '0; pkt_dropped <= '0;
      slot_full <= '0; wslot <= 1'b0;
    end else begin
      if (b_valid) begin : wr
        logic        act;
        logic [2:0]  bi;
        logic [63:0] acc;
        logic [15:0] len;
        logic [PW-1:0] wd;
        act = w_act;
        if (!w_act && !w_drop) begin
          // first byte: claim the write slot or drop the frame
          act = !slot_full[wslot];
          if (!act) begin
            w_drop <= 1'b1;
            pkt_dropped <= pkt_dropped + 16'd1;
          end
          bi = 3'd0; len = 16'd0; wd = PW'(1);
        end else begin
          bi = w_byte; len = w_len; wd = w_word;
        end
        acc = (bi == 0) ? 64'd0 : w_acc;
        acc[bi * 8 +: 8] = b_data;
        w_acc  <= acc;
        w_byte <= bi + 3'd1;
        w_len  <= len + 16'd1;
        w_word <= wd;
        w_act  <= act && !b_last;
        if (act && (bi == 3'd7 || b_last)) begin
          pb_we    <= 1'b1;
          pb_waddr <= {wslot, wd};
          pb_wdata <= acc;
          w_word   <= wd + PW'(1);
        end
        if (b_last) begin
          w_drop <= 1'b0;
          if (act) begin
            hdr_pending <= 1'b1;
            hdr_word <= {8'hB5, 1'b0, b_nerr, b_ok, b_ldpc_ok, 2'd0, b_iters, 2'd0, pilots, modcod, pkt_cnt, len + 16'd1};
            slot_nw[wslot] <= (PW+1)'(wd) + (PW+1)'(1);
            pkt_cnt <= pkt_cnt + 16'd1;
          end
        end
      end else if (hdr_pending) begin
        // header (word 0) last: length and decoder status are known now
        hdr_pending <= 1'b0;
        pb_we    <= 1'b1;
        pb_waddr <= {wslot, PW'(0)};
        pb_wdata <= hdr_word;
        slot_full[wslot] <= 1'b1;
        wslot <= !wslot;
      end
      if (clr_full) slot_full[clr_slot] <= 1'b0;
    end
    if (pb_we) pbuf[pb_waddr] <= pb_wdata;
  end

  // reader: words 0 .. PKT_WORDS-1 of the full slot, zeros after the data
  logic          r_act;
  logic [PW-1:0] r_word;
  logic [PW:0]   r_nw;
  logic          rd_v, rd_pad, rd_last;
  logic [63:0]   rd_q;
  logic [64:0]   ofifo [4];                 // {last, data}
  logic [2:0]    of_wp, of_rp;
  wire  [2:0]    of_cnt = of_wp - of_rp;

  assign m_axis_tvalid = of_cnt != 0;
  assign m_axis_tdata  = ofifo[of_rp[1:0]][63:0];
  assign m_axis_tlast  = ofifo[of_rp[1:0]][64];

  always_ff @(posedge clk) begin
    clr_full <= 1'b0;
    rd_v <= 1'b0;
    if (core_rst) begin
      r_act <= 1'b0;
      rslot <= 1'b0;
      of_wp <= '0; of_rp <= '0;
      pkt_sent <= '0;
    end else begin
      if (!r_act && slot_full[rslot] && !clr_full) begin
        r_act  <= 1'b1;
        r_word <= '0;
        r_nw   <= slot_nw[rslot];
      end else if (r_act && of_cnt + 3'(rd_v) < 3'd3) begin
        rd_q    <= pbuf[{rslot, r_word}];
        rd_v    <= 1'b1;
        rd_pad  <= (PW+1)'(r_word) >= r_nw;
        rd_last <= r_word == PW'(PKT_WORDS - 1);
        r_word  <= r_word + PW'(1);
        if (r_word == PW'(PKT_WORDS - 1)) begin
          r_act <= 1'b0;
          clr_full <= 1'b1;
          clr_slot <= rslot;
          rslot <= !rslot;
          pkt_sent <= pkt_sent + 16'd1;
        end
      end
      if (rd_v) begin
        ofifo[of_wp[1:0]] <= {rd_last, rd_pad ? 64'd0 : rd_q};
        of_wp <= of_wp + 3'd1;
      end
      if (m_axis_tvalid && m_axis_tready) of_rp <= of_rp + 3'd1;
    end
  end

  // ---------------------------------------------------------------- AXI-Lite read
  // the address is registered first (ar_p), the data one clock later
  logic       ar_p;
  logic [5:0] ar_q;
  always_ff @(posedge clk) begin
    if (!s_axi_aresetn) begin
      s_axi_arready <= 1'b1; s_axi_rvalid <= 1'b0;
      snap_rd <= 1'b0;
      ar_p <= 1'b0;
    end else begin
      snap_rd <= 1'b0;
      ar_p <= 1'b0;
      if (s_axi_arvalid && s_axi_arready) begin
        s_axi_arready <= 1'b0;
        ar_p <= 1'b1;
        ar_q <= s_axi_araddr[7:2];
      end
      if (ar_p) begin
        s_axi_rvalid  <= 1'b1;
        snap_rd <= ar_q == 6'h2A;
        case (ar_q)
          6'h00: s_axi_rdata <= 32'h0001_0200;
          6'h01: s_axi_rdata <= 32'h4456_5232;
          6'h02: s_axi_rdata <= scratch;
          6'h04: s_axi_rdata <= {30'd0, ctrl};
          6'h05: s_axi_rdata <= {23'd0, pilots, 3'd0, modcod};
          6'h06: s_axi_rdata <= {14'd0, gold_xa};
          6'h07: s_axi_rdata <= {14'd0, gold_xb};
          6'h08: s_axi_rdata <= ddc_finc;
          6'h09: s_axi_rdata <= {3'd0, ddc_shift, 6'd0, ddc_ntaps, 2'd0, ddc_dec};
          6'h0A: s_axi_rdata <= {23'd0, ddc_caddr};
          6'h0C: s_axi_rdata <= {22'd0, rrc_caddr};
          6'h0E: s_axi_rdata <= {t_ki, t_kp};
          6'h0F: s_axi_rdata <= fs_thresh;
          6'h10: s_axi_rdata <= {14'd0, pll_kp};
          6'h11: s_axi_rdata <= {14'd0, pll_ki};
          6'h12: s_axi_rdata <= {26'd0, max_it};
          6'h13: s_axi_rdata <= {20'd0, ddc_cic_shift, 2'd0, ddc_cic_r};
          6'h14: s_axi_rdata <= {16'd0, in_ovf};
          6'h15: s_axi_rdata <= {16'd0, ddc_dropped};
          6'h16: s_axi_rdata <= {15'd0, agc_gain};
          6'h17: s_axi_rdata <= t_integ;
          6'h18: s_axi_rdata <= {30'd0, fs_state};
          6'h19: s_axi_rdata <= {16'd0, fs_found};
          6'h1A: s_axi_rdata <= {16'd0, fs_fly};
          6'h1B: s_axi_rdata <= {16'd0, fs_lost};
          6'h1C: s_axi_rdata <= st_freq;
          6'h1D: s_axi_rdata <= st_amp;
          6'h1E: s_axi_rdata <= st_energy[35:4];
          6'h1F: s_axi_rdata <= {14'd0, st_gain};
          6'h20: s_axi_rdata <= {16'd0, frames};
          6'h21: s_axi_rdata <= {16'd0, ldpc_frames};
          6'h22: s_axi_rdata <= {16'd0, ldpc_failed};
          6'h23: s_axi_rdata <= {16'd0, pkt_dropped};
          6'h24: s_axi_rdata <= {16'd0, pkt_sent};
          6'h25: s_axi_rdata <= {st_fa, st_acq};
          6'h26: s_axi_rdata <= {16'd0, st_retype};
          6'h27: s_axi_rdata <= {bch_failed, bch_corr};
          6'h28: s_axi_rdata <= {29'd0, snap_src, snap_done, snap_busy};
          6'h29: s_axi_rdata <= {22'd0, snap_ra};
          6'h2A: s_axi_rdata <= snap_q;
          6'h2B: s_axi_rdata <= raw_err;
          6'h2C: s_axi_rdata <= raw_bits;
          6'h2D: s_axi_rdata <= bch_biterr;
          6'h2E: s_axi_rdata <= {pls_count, pls_word(pls_code, pls_flags)};
          6'h2F: s_axi_rdata <= {pls_dcount, pls_word(pls_dcode, pls_dflags)};
          6'h30: s_axi_rdata <= {pls_freq, 16'd0};
          6'h31: s_axi_rdata <= {14'd0, pls_best};
          6'h32: s_axi_rdata <= {14'd0, pls_second};
          6'h33: s_axi_rdata <= {pls_diff_p, pls_diff_m};
          6'h34: s_axi_rdata <= pls_seen;
          6'h35: s_axi_rdata <= {6'd0, pls_step, 10'd0, pls_h};
          6'h36: s_axi_rdata <= {16'd0, pls_skipped};
          default: s_axi_rdata <= '0;
        endcase
      end
      if (s_axi_rvalid && s_axi_rready) begin
        s_axi_rvalid  <= 1'b0;
        s_axi_arready <= 1'b1;
      end
    end
  end

endmodule
