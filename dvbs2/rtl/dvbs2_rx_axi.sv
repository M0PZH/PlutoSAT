// DVB-S2 RX for the Zynq block design, two receive slots
// (docs/rx2_design.md): AXI4-Lite registers, ADC input (axi_ad9361 channel
// 0, l_clk domain), one DDC per slot, the two-slot receiver core, and one
// fixed-size DMA packet per decoded BBFRAME of either slot.
//
//   ADC (adc_clk) -> async FIFO -> DDC slot 0 \                 (shared NCO ROM)
//                              \-> DDC slot 1 -> dvbs2_rx -> packet buffer -> m_axis
//
// Packet (PKT_WORDS 64-bit words, tlast on the last; byte 0 = tdata[7:0]):
//   word 0  [15:0] BBFRAME length in bytes (Kbch/8), [31:16] frame counter of
//           the slot, [36:32] MODCOD, [37] pilots (both: the frame's, as decoded),
//           [45:40] LDPC iterations, [48] LDPC converged, [49] BCH ok (the
//           BBFRAME is a codeword after <= t corrections: use it), [54:50] BCH
//           errors located, [55] receive slot, [63:56] 0xB5
//   word 1+ BBFRAME bytes (BB-descrambled), then zeros
// A frame that finds both packet slots full is dropped (PKT_DROPPED of its slot).
//
// Register map v2 (byte offsets, 32-bit; 10 address bits): a bank per receive
// slot (slot 0 from 0x000, slot 1 from 0x100), the shared registers at 0x200. A bank
// holds today's (1.x) single-slot map for its slot, so software for one slot
// adds 0x100 s to the 1.x offsets; VERSION, ID and SCRATCH read the same in
// every bank. The slot's settings are static while it runs: change them with
// the slot disabled or in its soft reset.
//   0x200 VERSION     RO  0x0002_0000: two receive slots, register map v2, the DDR LLR
//                         queue always on (0x0001_0500: DDR LLR queue (0xDC..0xF0), the
//                         packet header's MODCOD/pilots are the frame's own; 0x0001_0401:
//                         a whole-frame search after a PLS drop; 0x0001_0400: frame sync
//                         confirms headers by their PLS decode (no false lock
//                         on repeating frames), acquisition headers typed by the frame
//                         length, pilots off: no header kicks after the acquisition
//                         (0x0001_0300: pilots-off carrier tracking and the PLS fix: frame
//                         sync checks every search peak at the data length; 0x0001_0200:
//                         PLS decoder and TIMING_INTEG preset; 0x0001_0100: neither,
//                         0xB8.. read 0, a TIMING_INTEG write is ignored; 0x0001_0000: no
//                         CIC, DDC_CIC reads 0))
//   0x204 ID          RO  "DVR2" (0x44565232)
//   0x208 SCRATCH     RW
//   0x210 CTRL        RW  [1] soft reset of the whole receiver (both slots, the decoders,
//                         the LLR queue, the packet buffer; self-clearing, reads 1 while
//                         in progress), [9:8] enable of slot 1..0 (the slots' CTRL[0])
//   0x2A0 SNAP_CTRL   W   constellation snapshot (one buffer for both slots): [0] arm:
//                         capture the next 1024 symbols of slot [3], [1] source (0:
//                         carrier output, data symbols before the LLR gain; 1: timing
//                         loop output)
//                     R   [0] busy, [1] done, [2] source, [3] slot
//   0x2A4 SNAP_ADDR   RW  [9:0] read address
//   0x2A8 SNAP_DATA   RO  [15:0] I, [31:16] Q of symbol SNAP_ADDR, then SNAP_ADDR++
//   LLR queue (dvbs2_rx_llrq.sv; docs/rx2_design.md 6.1, 6.2): the frames of both
//   slots through a ring in DDR (m_axi, S_AXI_HP1) to the LDPC decoder
//   0x2E0 LLRQ_BASE   RW  [31:21] ring region, 2 MB aligned (default 0x3E80_0000, the
//                         device tree's dvbs2-llr-buffer): slot s frame f at base +
//                         s * 1 MB + f * 64 KB; takes effect at the next soft reset (0x210)
//   0x2E4 LLRQ_CFG    RW  [4:0] frames per slot ring (1..16, default 16; at the next soft
//                         reset), [13:8] QHI (default 63: no cap), [21:16] MAX_IT_LO
//                         (default 10): while more than QHI frames wait, frames stop at
//                         MAX_IT_LO iterations (both at once)
//   0x2E8 LLRQ_STAT   RO  [5:0] frames waiting (complete in DDR + the one being read),
//                         [13:8] the most since reset, [31:16] frames through the ring
//   0x2EC LLRQ_DROPPED RO frames dropped because the ring was full: [15:0] slot 0,
//                         [31:16] slot 1
//   0x2F0 LLRQ_AXI_ERR RO [15:0] AXI error responses (SLVERR/DECERR)
//   slot s = 0, 1 at 0x000 + 0x100 s:
//   +0x00 VERSION     RO  as 0x200
//   +0x04 ID          RO  as 0x204
//   +0x08 SCRATCH     RW  as 0x208
//   +0x10 CTRL        RW  [0] enable (the slot takes ADC samples), [1] soft reset of the
//                         slot (self-clearing): its DDC, input FIFO, state in the shared
//                         engines, PLS decoder and counters; the other slot runs on.
//                         Reads [1] 1 while in progress
//   +0x14 MODCOD      RW  [4:0] MODCOD (QPSK: 1..11), [8] pilots
//   +0x18 GOLD_XA     RW  [17:0]
//   +0x1C GOLD_XB     RW  [17:0]
//   +0x20 DDC_FINC    RW  NCO phase increment per ADC sample (2^32 = ADC rate)
//   +0x24 DDC_CFG     RW  [5:0] FIR decimation D (1..63), [17:8] taps (even, <= 512), [28:24] shift
//   +0x28 DDC_COEF_ADDR RW [8:0]
//   +0x2C DDC_COEF_DATA WO [17:0], then DDC_COEF_ADDR++
//   +0x30 RRC_COEF_ADDR RW [9:0] {phase[4:0], tap[4:0]}
//   +0x34 RRC_COEF_DATA WO [17:0], then RRC_COEF_ADDR++
//   +0x38 TIMING_GAINS RW [15:0] kp, [31:16] ki
//   +0x3C FS_THRESH   RW  header metric threshold
//   +0x40 PLL_KP      RW  [17:0]
//   +0x44 PLL_KI      RW  [17:0]
//   +0x48 LDPC_MAX_IT RW  [5:0]
//   +0x4C DDC_CIC     RW  [5:0] CIC decimation R (0, 1: no CIC; 2..32), [11:8] CIC output shift;
//                         the total decimation is R * D (dvbs2_rx_ddc.sv; model/rxfixed.py ddc_config())
//   +0x50 IN_OVERFLOWS RO 2-sps samples lost at the slot's input FIFO
//   +0x54 DDC_DROPPED RO  DDC outputs lost (input too fast for the taps)
//   +0x58 AGC_GAIN    RO  Q5.12
//   +0x5C TIMING_INTEG R  timing loop integrator (Q.24 samples/symbol beyond 2)
//                     W   its value at the slot's reset (default 0): a known
//                         symbol-rate offset, 2 (nominal / actual - 1) x 2^24
//                         (model/rxfixed.py timing_integ_preset())
//   +0x60 FS_STATE    RO  [1:0] 0 search, 1 check, 2 locked
//   +0x64 FS_FOUND    RO
//   +0x68 FS_FLYWHEEL RO
//   +0x6C FS_LOST     RO
//   +0x70 FREQ        RO  carrier frequency, turns * 2^32 per symbol
//   +0x74 AMP         RO
//   +0x78 ENERGY      RO  ([35:4])
//   +0x7C LLR_GAIN    RO
//   +0x80 FRAMES      RO  data frames demodulated
//   +0x84 LDPC_FRAMES RO  frames of the slot decoded
//   +0x88 LDPC_FAILED RO  ... not converged
//   +0x8C PKT_DROPPED RO  frames of the slot dropped (both packet slots full)
//   +0x90 PKT_SENT    RO  packets of the slot sent
//   +0x94 CARRIER_ACQ RO  [15:0] frequency acquisitions, [31:16] 1/1476 false-lock corrections
//   +0x98 CARRIER_RETYPE RO [15:0] frame types changed by the coherent decision
//   +0x9C BCH         RO  [15:0] frames corrected, [31:16] frames not decodable
//   +0xA0 SNAP_CTRL   RW  as 0x2A0, the slot being this bank's (a write sets SNAP_CTRL[3] = s)
//   +0xA4 SNAP_ADDR   RW  as 0x2A4
//   +0xA8 SNAP_DATA   RO  as 0x2A8
//   +0xAC RAW_ERR     RO  pre-FEC bit errors (channel hard decisions vs. the decoded
//                         frame, Kldpc bits of each BCH-good frame), wraps
//   +0xB0 RAW_BITS    RO  bits counted in RAW_ERR, wraps
//   +0xB4 BCH_BITERR  RO  bit errors corrected by BCH (post-LDPC errors), wraps
//   PLS decoder of the slot (dvbs2_rx_pls.sv, model/rxfixed.py pls_decode()): the PLS
//   code of every PLHEADER the frame sync tags, whatever MODCOD/pilots are configured
//   (detection; also the frame sync's header confirmation). Confident: best > 2 x
//   second and best > 2^8.
//   +0xB8 PLS_LAST    RO  the last decode: [4:0] MODCOD, [8] pilots, [9] short frame,
//                         [12] confident, [13] from the carrier's frequency, [14] the
//                         frequency grid was searched, [15] valid (a decode since reset),
//                         [31:16] decodes (wraps)
//   +0xBC PLS_DATA    RO  the last confident decode of a data header (MODCOD != 0), same
//                         fields, [15] valid, [31:16] confident data decodes (wraps)
//   +0xC0 PLS_FREQ    RO  frequency of the last decode's hypothesis, turns * 2^32 per
//                         symbol (multiples of 2^16; relative to the DDC, like FREQ)
//   +0xC4 PLS_BEST    RO  [17:0] metric of the last decode (|correlation|^2 / 256; a
//                         header at -3 dB Es/N0 gives >= ~300, noise alone <= ~220)
//   +0xC8 PLS_SECOND  RO  [17:0] largest metric of another code at the same frequency
//   +0xCC PLS_DIFF    RO  confident data decodes whose [15:0] MODCOD or frame size,
//                         [31:16] pilots differ from the configured ones (wrap)
//   +0xD0 PLS_SEEN    RW  (W1C) bit m: a confident normal-frame decode of MODCOD m (bit 0:
//                         dummy frames) since cleared (a clear wins over a decode in
//                         the same clock)
//   +0xD4 PLS_CFG     RW  [5:0] H, [25:16] step (turns * 2^16 per symbol): frequency
//                         hypotheses h * step, h = -H..H (default 32, 262: +-0.128
//                         cycles/symbol), after the carrier's frequency once acquired;
//                         ~1420 clocks per hypothesis (2H + 2 of them) must fit
//                         between two headers (94k clocks: 2.38 MBd dummy frames)
//   +0xD8 PLS_SKIPPED RO  [15:0] headers not decoded (decoder still busy), wraps
//   +0xEC LLRQ_DROPPED RO [15:0] frames of the slot dropped because its ring was full
//   (+0xDC..+0xE8 and +0xF0, the LLR queue's registers in 1.5, read 0.)

module dvbs2_rx_axi #(
  parameter string LUT_FILE      = "nco_lut.mem",
  parameter string LDPC_ROM_FILE = "ldpc_dec_rom.mem",
  parameter int    PKT_WORDS     = 1024,
  parameter string FIFO_RAM      = "block"     // the slots' input FIFOs
) (
  input  logic        s_axi_aclk,
  input  logic        s_axi_aresetn,
  input  logic [9:0]  s_axi_awaddr,
  input  logic        s_axi_awvalid,
  output logic        s_axi_awready,
  input  logic [31:0] s_axi_wdata,
  input  logic [3:0]  s_axi_wstrb,
  input  logic        s_axi_wvalid,
  output logic        s_axi_wready,
  output logic [1:0]  s_axi_bresp,
  output logic        s_axi_bvalid,
  input  logic        s_axi_bready,
  input  logic [9:0]  s_axi_araddr,
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
  input  logic [15:0] adc_data_q,

  // LLR queue: AXI3 master, 64 bits, ID 0 (S_AXI_HP1)
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

  localparam logic [31:0] VERSION = 32'h0002_0000;
  wire clk = s_axi_aclk;

  // ---------------------------------------------------------------- registers
  // shared
  logic [31:0] scratch;
  logic [1:0]  slot_en;
  logic [10:0] llrq_base, llrq_base_q;
  logic [4:0]  llrq_nfr, llrq_nfr_q;
  logic [5:0]  llrq_qhi, llrq_it_lo;
  logic [17:0] cdata;
  // per slot
  logic [31:0] ddc_finc [2], fs_thresh [2], t_integ0 [2], pls_seen_clr [2];
  logic [4:0]  modcod [2];
  logic [1:0]  pilots;
  logic [17:0] gold_xa [2], gold_xb [2], pll_kp [2], pll_ki [2];
  logic [5:0]  ddc_dec [2], max_it [2];
  logic [9:0]  ddc_ntaps [2];
  logic [4:0]  ddc_shift [2];
  logic [5:0]  ddc_cic_r [2];
  logic [3:0]  ddc_cic_shift [2];
  logic [8:0]  ddc_caddr [2];
  logic [9:0]  rrc_caddr [2];
  logic [1:0]  ddc_cwe;
  logic        rrc_cwe, rrc_cslot;
  logic [15:0] t_kp [2], t_ki [2];
  logic [5:0]  pls_h [2];
  logic [9:0]  pls_step [2];

  // resets: the whole receiver (core_rst) and per slot (slot_rst), 16 clocks each
  logic [3:0] srst_cnt, slot_cnt [2];
  logic       core_rst, srst_req;
  logic [1:0] slot_rst, slot_req;
  logic       aresetn_q;                     // local copy (the core reset has a large fanout)
  always_ff @(posedge clk) begin
    aresetn_q <= s_axi_aresetn;
    if (!aresetn_q || srst_req) srst_cnt <= '1;
    else if (srst_cnt != 0)     srst_cnt <= srst_cnt - 1'b1;
    core_rst <= !aresetn_q || srst_req || (srst_cnt != 0);
    for (int s = 0; s < 2; s++) begin
      if (!aresetn_q || slot_req[s]) slot_cnt[s] <= '1;
      else if (slot_cnt[s] != 0)     slot_cnt[s] <= slot_cnt[s] - 1'b1;
      slot_rst[s] <= slot_req[s] || (slot_cnt[s] != 0);
    end
    if (core_rst) begin                      // the queue's ring layout changes only at a reset
      llrq_base_q <= llrq_base;
      llrq_nfr_q  <= llrq_nfr;
    end
  end

  logic        aw_hold, w_hold;
  logic [9:0]  awaddr_q;
  logic [31:0] wdata_q;
  assign s_axi_awready = !aw_hold;
  assign s_axi_wready  = !w_hold;
  assign s_axi_bresp   = 2'b00;
  assign s_axi_rresp   = 2'b00;

  logic        snap_arm, snap_src, snap_sel, snap_ra_we;
  logic [9:0]  snap_ra_d;

  always_ff @(posedge clk) begin
    ddc_cwe <= '0;
    rrc_cwe <= 1'b0;
    snap_arm <= 1'b0;
    snap_ra_we <= 1'b0;
    srst_req <= 1'b0;
    slot_req <= '0;
    pls_seen_clr[0] <= '0;
    pls_seen_clr[1] <= '0;
    if (!s_axi_aresetn) begin
      aw_hold <= 1'b0; w_hold <= 1'b0; s_axi_bvalid <= 1'b0;
      scratch <= '0; slot_en <= '0;
      snap_src <= 1'b0; snap_sel <= 1'b0;
      llrq_base <= 11'(32'h3E80_0000 >> 21);
      llrq_nfr <= 5'd16; llrq_qhi <= 6'd63; llrq_it_lo <= 6'd10;
      for (int s = 0; s < 2; s++) begin
        modcod[s] <= 5'd4; pilots[s] <= 1'b1;
        gold_xa[s] <= 18'h00001; gold_xb[s] <= 18'h01008;
        ddc_finc[s] <= '0; ddc_dec[s] <= 6'd1; ddc_ntaps[s] <= 10'd2; ddc_shift[s] <= 5'd16;
        ddc_cic_r[s] <= '0; ddc_cic_shift[s] <= '0;
        ddc_caddr[s] <= '0; rrc_caddr[s] <= '0;
        t_kp[s] <= dvbs2_rx_pkg::TIMING_KP_DEFAULT;
        t_ki[s] <= dvbs2_rx_pkg::TIMING_KI_DEFAULT;
        t_integ0[s] <= '0;
        fs_thresh[s] <= dvbs2_rx_pkg::FS_THRESH_DEFAULT;
        pll_kp[s] <= dvbs2_rx_pkg::PLL_KP_DEFAULT;
        pll_ki[s] <= dvbs2_rx_pkg::PLL_KI_DEFAULT;
        max_it[s] <= 6'd30;
        pls_h[s] <= dvbs2_rx_pkg::PLS_H_DEFAULT;
        pls_step[s] <= dvbs2_rx_pkg::PLS_STEP_DEFAULT;
      end
    end else begin
      if (s_axi_awvalid && s_axi_awready) begin aw_hold <= 1'b1; awaddr_q <= s_axi_awaddr; end
      if (s_axi_wvalid && s_axi_wready)   begin w_hold  <= 1'b1; wdata_q  <= s_axi_wdata;  end
      if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
      if (aw_hold && w_hold && !s_axi_bvalid) begin : regw
        logic s;
        s = awaddr_q[8];
        aw_hold <= 1'b0; w_hold <= 1'b0; s_axi_bvalid <= 1'b1;
        if (awaddr_q[9:8] == 2'd2) begin
          // shared bank
          case (awaddr_q[7:2])
            6'h02: scratch <= wdata_q;
            6'h04: begin srst_req <= wdata_q[1]; slot_en <= wdata_q[9:8]; end
            6'h28: begin snap_arm <= wdata_q[0]; snap_src <= wdata_q[1]; snap_sel <= wdata_q[3]; end
            6'h29: begin snap_ra_we <= 1'b1; snap_ra_d <= wdata_q[9:0]; end
            6'h38: llrq_base <= wdata_q[31:21];
            6'h39: begin llrq_nfr <= wdata_q[4:0]; llrq_qhi <= wdata_q[13:8]; llrq_it_lo <= wdata_q[21:16]; end
            default: ;
          endcase
        end else if (!awaddr_q[9]) begin
          // slot s bank
          case (awaddr_q[7:2])
            6'h02: scratch <= wdata_q;
            6'h04: begin slot_en[s] <= wdata_q[0]; slot_req[s] <= wdata_q[1]; end
            6'h05: begin modcod[s] <= wdata_q[4:0]; pilots[s] <= wdata_q[8]; end
            6'h06: gold_xa[s] <= wdata_q[17:0];
            6'h07: gold_xb[s] <= wdata_q[17:0];
            6'h08: ddc_finc[s] <= wdata_q;
            6'h09: begin ddc_dec[s] <= wdata_q[5:0]; ddc_ntaps[s] <= wdata_q[17:8]; ddc_shift[s] <= wdata_q[28:24]; end
            6'h0A: ddc_caddr[s] <= wdata_q[8:0];
            6'h0B: begin cdata <= wdata_q[17:0]; ddc_cwe[s] <= 1'b1; end
            6'h0C: rrc_caddr[s] <= wdata_q[9:0];
            6'h0D: begin cdata <= wdata_q[17:0]; rrc_cwe <= 1'b1; rrc_cslot <= s; end
            6'h0E: begin t_kp[s] <= wdata_q[15:0]; t_ki[s] <= wdata_q[31:16]; end
            6'h0F: fs_thresh[s] <= wdata_q;
            6'h10: pll_kp[s] <= wdata_q[17:0];
            6'h11: pll_ki[s] <= wdata_q[17:0];
            6'h12: max_it[s] <= wdata_q[5:0];
            6'h13: begin ddc_cic_r[s] <= wdata_q[5:0]; ddc_cic_shift[s] <= wdata_q[11:8]; end
            6'h17: t_integ0[s] <= wdata_q;
            6'h28: begin snap_arm <= wdata_q[0]; snap_src <= wdata_q[1]; snap_sel <= s; end
            6'h29: begin snap_ra_we <= 1'b1; snap_ra_d <= wdata_q[9:0]; end
            6'h34: pls_seen_clr[s] <= wdata_q;
            6'h35: begin pls_h[s] <= wdata_q[5:0]; pls_step[s] <= wdata_q[25:16]; end
            default: ;
          endcase
        end
      end
      for (int s = 0; s < 2; s++)
        if (ddc_cwe[s]) ddc_caddr[s] <= ddc_caddr[s] + 9'd1;
      if (rrc_cwe) rrc_caddr[rrc_cslot] <= rrc_caddr[rrc_cslot] + 10'd1;
    end
  end

  // ---------------------------------------------------------------- ADC -> DDCs
  // enable and reset cross into the ADC domain
  (* ASYNC_REG = "TRUE" *) logic [1:0] en_a, rst_a;
  always_ff @(posedge adc_clk) begin
    en_a  <= {en_a[0], |slot_en};
    rst_a <= {rst_a[0], core_rst};
  end

  logic        a_valid, a_ready;
  logic [31:0] a_data;
  dvbs2_async_fifo #(.W(32), .AW(5)) u_adc_fifo (
    .wclk(adc_clk), .wrst(rst_a[1]), .w_valid(adc_valid && en_a[1]), .w_ready(), .w_data({adc_data_q, adc_data_i}),
    .rclk(clk), .rrst(core_rst), .r_valid(a_valid), .r_ready(a_ready), .r_data(a_data));

  // both DDCs take the same ADC sample (a slot disabled or in reset is skipped)
  logic [1:0]         ddc_rst, feed, k_ready, d_valid;
  logic signed [15:0] d_i [2], d_q [2];
  logic [15:0]        ddc_dropped [2];
  logic [9:0]         nco_a [2];
  logic [31:0]        nco_q [2];

  always_ff @(posedge clk)
    for (int s = 0; s < 2; s++) begin
      ddc_rst[s] <= core_rst || slot_rst[s];
      feed[s]    <= slot_en[s] && !core_rst && !slot_rst[s];
    end
  assign a_ready = (k_ready[0] || !feed[0]) && (k_ready[1] || !feed[1]);

  // the shared NCO ROM: one port per DDC
  (* rom_style = "block" *) logic [31:0] nco_lut [1024];
  initial $readmemh(LUT_FILE, nco_lut);
  always_ff @(posedge clk) begin
    nco_q[0] <= nco_lut[nco_a[0]];
    nco_q[1] <= nco_lut[nco_a[1]];
  end

  for (genvar s = 0; s < 2; s++) begin : g_ddc
    dvbs2_rx_ddc #(.LUT_FILE(LUT_FILE), .EXT_NCO(1'b1)) u_ddc (
      .clk, .rst(ddc_rst[s]), .finc(ddc_finc[s]), .dec(ddc_dec[s]), .ntaps(ddc_ntaps[s]), .shift(ddc_shift[s]),
      .cic_r(ddc_cic_r[s]), .cic_shift(ddc_cic_shift[s]),
      .coef_we(ddc_cwe[s]), .coef_addr(ddc_caddr[s]), .coef_data(cdata),
      .s_valid(a_valid && a_ready && feed[s]), .s_ready(k_ready[s]),
      .s_i($signed(a_data[15:0])), .s_q($signed(a_data[31:16])),
      .m_valid(d_valid[s]), .m_i(d_i[s]), .m_q(d_q[s]), .st_dropped(ddc_dropped[s]),
      .nco_addr(nco_a[s]), .nco_data(nco_q[s]));
  end

  // ---------------------------------------------------------------- receiver core
  logic        b_valid, b_last, b_ok, b_ldpc_ok, b_pilots, b_slot;
  logic [3:0]  b_rate;
  logic [4:0]  b_nerr, b_modcod;
  logic [15:0] b_raw_err;
  logic [7:0]  b_data;
  logic [5:0]  b_iters;
  logic        snap_c_v, snap_c_slot;
  logic [1:0]  snap_t_v;
  logic signed [15:0] snap_c_i, snap_c_q, snap_t_i [2], snap_t_q [2];
  logic [15:0] in_ovf [2], fs_found [2], fs_fly [2], fs_lost [2], frames [2], ldpc_frames [2], ldpc_failed [2];
  logic [15:0] bch_corr [2], bch_failed [2];
  logic [16:0] agc_gain [2];
  logic signed [31:0] t_integ [2];
  logic [1:0]  fs_state [2];
  logic [31:0] st_freq [2], st_amp [2];
  logic [35:0] st_energy [2];
  logic [17:0] st_gain [2];
  logic [15:0] st_acq [2], st_fa [2], st_retype [2];
  logic [1:0]  pls_v;
  logic [6:0]  pls_code [2], pls_dcode [2];
  logic [3:0]  pls_flags [2], pls_dflags [2];
  logic [17:0] pls_best [2], pls_second [2];
  logic [31:0] pls_seen [2];
  logic [15:0] pls_freq [2], pls_count [2], pls_dcount [2], pls_diff_m [2], pls_diff_p [2], pls_skipped [2];
  logic [5:0]  llrq_depth, llrq_depth_max;
  logic [15:0] llrq_frames, llrq_drop [2], llrq_axi_err;
  logic signed [31:0] t_integ0_s [2];
  assign t_integ0_s[0] = $signed(t_integ0[0]);
  assign t_integ0_s[1] = $signed(t_integ0[1]);

  dvbs2_rx #(.LUT_FILE(LUT_FILE), .LDPC_ROM_FILE(LDPC_ROM_FILE), .FIFO_RAM(FIFO_RAM)) u_rx (
    .clk, .rst(core_rst), .rst_s(slot_rst),
    .in_valid(d_valid), .in_i(d_i), .in_q(d_q), .in_overflows(in_ovf),
    .coef_we(rrc_cwe), .coef_slot(rrc_cslot), .coef_addr(rrc_caddr[rrc_cslot]), .coef_data(cdata),
    .timing_kp(t_kp), .timing_ki(t_ki), .timing_integ0(t_integ0_s), .modcod, .pilots, .fs_thresh, .gold_xa, .gold_xb,
    .pll_kp, .pll_ki, .ldpc_max_it(max_it),
    .pls_h, .pls_step, .pls_seen_clr,
    .axi_rst(!aresetn_q), .llrq_base({llrq_base_q, 21'd0}), .llrq_nfr(llrq_nfr_q), .llrq_qhi, .llrq_it_lo,
    .m_valid(b_valid), .m_ready(1'b1), .m_data(b_data), .m_last(b_last), .m_ok(b_ok), .m_ldpc_ok(b_ldpc_ok),
    .m_iters(b_iters), .m_nerr(b_nerr), .m_raw_err(b_raw_err),
    .m_rate(b_rate), .m_modcod(b_modcod), .m_pilots(b_pilots), .m_slot(b_slot),
    .snap_c_v, .snap_c_slot, .snap_c_i, .snap_c_q, .snap_t_v, .snap_t_i, .snap_t_q,
    .st_agc_gain(agc_gain), .st_timing_integ(t_integ), .st_fs_state(fs_state),
    .st_fs_found(fs_found), .st_fs_flywheel(fs_fly), .st_fs_lost(fs_lost),
    .st_freq, .st_amp, .st_energy, .st_gain, .st_frames(frames), .st_acq, .st_fa, .st_retype,
    .st_ldpc_frames(ldpc_frames), .st_ldpc_failed(ldpc_failed),
    .st_bch_corrected(bch_corr), .st_bch_failed(bch_failed),
    .st_pls_v(pls_v), .st_pls_code(pls_code), .st_pls_flags(pls_flags), .st_pls_best(pls_best),
    .st_pls_second(pls_second), .st_pls_freq(pls_freq), .st_pls_count(pls_count),
    .st_pls_dcode(pls_dcode), .st_pls_dflags(pls_dflags), .st_pls_dcount(pls_dcount),
    .st_pls_diff_modcod(pls_diff_m), .st_pls_diff_pilots(pls_diff_p),
    .st_pls_seen(pls_seen), .st_pls_skipped(pls_skipped),
    .st_llrq_depth(llrq_depth), .st_llrq_depth_max(llrq_depth_max), .st_llrq_frames(llrq_frames),
    .st_llrq_dropped(llrq_drop), .st_llrq_axi_err(llrq_axi_err), .st_llrq_busy(),
    .m_axi_awaddr, .m_axi_awlen, .m_axi_awsize, .m_axi_awburst, .m_axi_awcache, .m_axi_awvalid,
    .m_axi_awready, .m_axi_wdata, .m_axi_wstrb, .m_axi_wlast, .m_axi_wvalid, .m_axi_wready,
    .m_axi_bresp, .m_axi_bvalid, .m_axi_bready, .m_axi_araddr, .m_axi_arlen, .m_axi_arsize,
    .m_axi_arburst, .m_axi_arcache, .m_axi_arvalid, .m_axi_arready, .m_axi_rdata, .m_axi_rresp,
    .m_axi_rlast, .m_axi_rvalid, .m_axi_rready);

  // {MODCOD, short, pilots} + {valid, grid, carrier, confident} -> PLS_LAST / PLS_DATA fields
  function automatic logic [15:0] pls_word(input logic [6:0] c, input logic [3:0] fl);
    pls_word = {fl, 2'd0, c[1], c[0], 3'd0, c[6:2]};
  endfunction

  // ---------------------------------------------------------------- constellation snapshot
  // one buffer; SNAP_CTRL selects the slot
  (* ram_style = "block" *) logic [31:0] snap_mem [1024];
  logic        snap_busy, snap_done, snap_rd;
  logic [9:0]  snap_wa, snap_ra;
  logic [31:0] snap_q;
  logic        snap_v;
  logic [31:0] snap_d;
  always_ff @(posedge clk) begin               // (registered: the sources come from all over the core)
    snap_v <= snap_src ? snap_t_v[snap_sel] : snap_c_v && snap_c_slot == snap_sel;
    snap_d <= snap_src ? {snap_t_q[snap_sel], snap_t_i[snap_sel]} : {snap_c_q, snap_c_i};
  end

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

  // ---------------------------------------------------------------- error counters per slot
  logic [31:0] raw_err [2], raw_bits [2], bch_biterr [2];
  for (genvar s = 0; s < 2; s++) begin : g_err
    always_ff @(posedge clk) begin
      if (core_rst || slot_rst[s]) begin
        raw_err[s] <= '0; raw_bits[s] <= '0; bch_biterr[s] <= '0;
      end else if (b_valid && b_last && b_ok && b_slot == s[0]) begin
        raw_err[s]    <= raw_err[s] + 32'(b_raw_err);
        raw_bits[s]   <= raw_bits[s] + 32'(dvbs2_bch_pkg::BCH_N[b_rate]);
        bch_biterr[s] <= bch_biterr[s] + 32'(b_nerr);
      end
    end
  end

  // ---------------------------------------------------------------- packets
  localparam int PW = $clog2(PKT_WORDS);
  (* ram_style = "block" *) logic [63:0] pbuf [2 * PKT_WORDS];
  logic [1:0]    slot_full;
  logic [PW:0]   slot_nw [2];               // words with data (header + BBFRAME)
  logic [1:0]    slot_rx;                   // receive slot of each packet slot's frame
  logic          wslot, rslot;
  logic          w_act, w_drop;
  logic [PW-1:0] w_word;                    // next data word
  logic [2:0]    w_byte;
  logic [63:0]   w_acc;
  logic [15:0]   w_len;
  logic [15:0]   pkt_cnt [2], pkt_dropped [2], pkt_sent [2];
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
      pkt_cnt[0] <= '0; pkt_cnt[1] <= '0; pkt_dropped[0] <= '0; pkt_dropped[1] <= '0;
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
            pkt_dropped[b_slot] <= pkt_dropped[b_slot] + 16'd1;
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
            hdr_word <= {8'hB5, b_slot, b_nerr, b_ok, b_ldpc_ok, 2'd0, b_iters, 2'd0, b_pilots, b_modcod,
                         pkt_cnt[b_slot], len + 16'd1};
            slot_nw[wslot] <= (PW+1)'(wd) + (PW+1)'(1);
            slot_rx[wslot] <= b_slot;
            pkt_cnt[b_slot] <= pkt_cnt[b_slot] + 16'd1;
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
      for (int s = 0; s < 2; s++)
        if (slot_rst[s]) begin pkt_cnt[s] <= '0; pkt_dropped[s] <= '0; end
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
      pkt_sent[0] <= '0; pkt_sent[1] <= '0;
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
          pkt_sent[slot_rx[rslot]] <= pkt_sent[slot_rx[rslot]] + 16'd1;
        end
      end
      if (rd_v) begin
        ofifo[of_wp[1:0]] <= {rd_last, rd_pad ? 64'd0 : rd_q};
        of_wp <= of_wp + 3'd1;
      end
      if (m_axis_tvalid && m_axis_tready) of_rp <= of_rp + 3'd1;
      for (int s = 0; s < 2; s++)
        if (slot_rst[s]) pkt_sent[s] <= '0;
    end
  end

  // ---------------------------------------------------------------- AXI-Lite read
  // the address is registered first (ar_p), then a slot bank's word and the
  // shared word are selected (ar_p2), then the bank (rdata)
  logic       ar_p, ar_p2;
  logic [9:0] ar_q;
  logic [1:0] ar_bank;
  logic [31:0] rd_slot, rd_shared;

  wire [31:0] snap_ctrl_rd = {28'd0, snap_sel, snap_src, snap_done, snap_busy};

  always_ff @(posedge clk) begin
    if (!s_axi_aresetn) begin
      s_axi_arready <= 1'b1; s_axi_rvalid <= 1'b0;
      snap_rd <= 1'b0;
      ar_p <= 1'b0; ar_p2 <= 1'b0;
    end else begin
      snap_rd <= 1'b0;
      ar_p <= 1'b0;
      ar_p2 <= ar_p;
      if (s_axi_arvalid && s_axi_arready) begin
        s_axi_arready <= 1'b0;
        ar_p <= 1'b1;
        ar_q <= s_axi_araddr;
      end
      if (ar_p) begin : rd
        logic s;
        s = ar_q[8];
        ar_bank <= ar_q[9:8];
        snap_rd <= ar_q[7:2] == 6'h2A && ar_q[9:8] != 2'd3;
        case (ar_q[7:2])
          6'h00: rd_shared <= VERSION;
          6'h01: rd_shared <= 32'h4456_5232;
          6'h02: rd_shared <= scratch;
          6'h04: rd_shared <= {22'd0, slot_en, 6'd0, core_rst, 1'b0};
          6'h28: rd_shared <= snap_ctrl_rd;
          6'h29: rd_shared <= {22'd0, snap_ra};
          6'h2A: rd_shared <= snap_q;
          6'h38: rd_shared <= {llrq_base, 21'd0};
          6'h39: rd_shared <= {10'd0, llrq_it_lo, 2'd0, llrq_qhi, 3'd0, llrq_nfr};
          6'h3A: rd_shared <= {llrq_frames, 2'd0, llrq_depth_max, 2'd0, llrq_depth};
          6'h3B: rd_shared <= {llrq_drop[1], llrq_drop[0]};
          6'h3C: rd_shared <= {16'd0, llrq_axi_err};
          default: rd_shared <= '0;
        endcase
        case (ar_q[7:2])
          6'h00: rd_slot <= VERSION;
          6'h01: rd_slot <= 32'h4456_5232;
          6'h02: rd_slot <= scratch;
          6'h04: rd_slot <= {30'd0, slot_rst[s], slot_en[s]};
          6'h05: rd_slot <= {23'd0, pilots[s], 3'd0, modcod[s]};
          6'h06: rd_slot <= {14'd0, gold_xa[s]};
          6'h07: rd_slot <= {14'd0, gold_xb[s]};
          6'h08: rd_slot <= ddc_finc[s];
          6'h09: rd_slot <= {3'd0, ddc_shift[s], 6'd0, ddc_ntaps[s], 2'd0, ddc_dec[s]};
          6'h0A: rd_slot <= {23'd0, ddc_caddr[s]};
          6'h0C: rd_slot <= {22'd0, rrc_caddr[s]};
          6'h0E: rd_slot <= {t_ki[s], t_kp[s]};
          6'h0F: rd_slot <= fs_thresh[s];
          6'h10: rd_slot <= {14'd0, pll_kp[s]};
          6'h11: rd_slot <= {14'd0, pll_ki[s]};
          6'h12: rd_slot <= {26'd0, max_it[s]};
          6'h13: rd_slot <= {20'd0, ddc_cic_shift[s], 2'd0, ddc_cic_r[s]};
          6'h14: rd_slot <= {16'd0, in_ovf[s]};
          6'h15: rd_slot <= {16'd0, ddc_dropped[s]};
          6'h16: rd_slot <= {15'd0, agc_gain[s]};
          6'h17: rd_slot <= t_integ[s];
          6'h18: rd_slot <= {30'd0, fs_state[s]};
          6'h19: rd_slot <= {16'd0, fs_found[s]};
          6'h1A: rd_slot <= {16'd0, fs_fly[s]};
          6'h1B: rd_slot <= {16'd0, fs_lost[s]};
          6'h1C: rd_slot <= st_freq[s];
          6'h1D: rd_slot <= st_amp[s];
          6'h1E: rd_slot <= st_energy[s][35:4];
          6'h1F: rd_slot <= {14'd0, st_gain[s]};
          6'h20: rd_slot <= {16'd0, frames[s]};
          6'h21: rd_slot <= {16'd0, ldpc_frames[s]};
          6'h22: rd_slot <= {16'd0, ldpc_failed[s]};
          6'h23: rd_slot <= {16'd0, pkt_dropped[s]};
          6'h24: rd_slot <= {16'd0, pkt_sent[s]};
          6'h25: rd_slot <= {st_fa[s], st_acq[s]};
          6'h26: rd_slot <= {16'd0, st_retype[s]};
          6'h27: rd_slot <= {bch_failed[s], bch_corr[s]};
          6'h28: rd_slot <= snap_ctrl_rd;
          6'h29: rd_slot <= {22'd0, snap_ra};
          6'h2A: rd_slot <= snap_q;
          6'h2B: rd_slot <= raw_err[s];
          6'h2C: rd_slot <= raw_bits[s];
          6'h2D: rd_slot <= bch_biterr[s];
          6'h2E: rd_slot <= {pls_count[s], pls_word(pls_code[s], pls_flags[s])};
          6'h2F: rd_slot <= {pls_dcount[s], pls_word(pls_dcode[s], pls_dflags[s])};
          6'h30: rd_slot <= {pls_freq[s], 16'd0};
          6'h31: rd_slot <= {14'd0, pls_best[s]};
          6'h32: rd_slot <= {14'd0, pls_second[s]};
          6'h33: rd_slot <= {pls_diff_p[s], pls_diff_m[s]};
          6'h34: rd_slot <= pls_seen[s];
          6'h35: rd_slot <= {6'd0, pls_step[s], 10'd0, pls_h[s]};
          6'h36: rd_slot <= {16'd0, pls_skipped[s]};
          6'h3B: rd_slot <= {16'd0, llrq_drop[s]};
          default: rd_slot <= '0;
        endcase
      end
      if (ar_p2) begin
        s_axi_rvalid <= 1'b1;
        s_axi_rdata  <= ar_bank == 2'd2 ? rd_shared : ar_bank[1] ? 32'd0 : rd_slot;
      end
      if (s_axi_rvalid && s_axi_rready) begin
        s_axi_rvalid  <= 1'b0;
        s_axi_arready <= 1'b1;
      end
    end
  end

endmodule
