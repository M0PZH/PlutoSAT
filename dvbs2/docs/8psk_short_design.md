# 8PSK and short FECFRAMEs (study and design)

User decision (2026-11-18): add **8PSK** (only 8PSK: 3/5, 2/3, 3/4, 5/6, 8/9,
9/10, MODCOD 12..17) and **short FECFRAMEs** (16200 bits, QPSK and 8PSK) to
the transmitter and the two-slot receiver, if the FPGA still has room.
Nothing higher than 8PSK.

Status: see [Implementation status](#implementation-status) at the end.

## 1. What the standard asks for

| | QPSK normal | QPSK short | 8PSK normal | 8PSK short |
| --- | --- | --- | --- | --- |
| FECFRAME bits | 64800 | 16200 | 64800 | 16200 |
| symbols, slots of 90 | 32400, 360 | 8100, 90 | 21600, 240 | 5400, 60 |
| pilot blocks (pilots on) | 22 | 5 | 14 | 3 |
| PLFRAME symbols, pilots off / on | 32490 / 33282 | 8190 / 8370 | 21690 / 22194 | 5490 / 5598 |
| code rates | 1/4 .. 9/10 | 1/4 .. 8/9 | 3/5 .. 9/10 | 3/5 .. 8/9 |

* **Short codes** (EN 302 307-1 Tables 5b, 6b, B.x): Kldpc 3240 .. 14400,
  q = (16200 - Kldpc) / 360 = 36 .. 5, BCH over GF(2^14) (primitive
  polynomial x^14 + x^5 + x^3 + x + 1), t = 12 for every rate, 168 parity
  bits. The nominal rates are not the real ones (1/4 is 1/5, 1/2 is 4/9, 3/4
  is 11/15, 4/5 is 7/9, 5/6 is 37/45). Kbch: 3072, 5232, 6312, 7032, 9552,
  10632, 11712, 12432, 13152, 14232. The LDPC tables still use 360-column
  circulants, so the same encoder and decoder structures apply; but five of
  the short codes (1/4, 1/2, 3/4, 4/5, 5/6) have check nodes of different
  degrees (e.g. 1/2: 4..7, 3/4: 9..13), while every normal code is check-regular.
* **8PSK** uses the normal-frame LDPC/BCH codes of its rates (no new code
  tables). New: the bit interleaver (5.3.3: 3 columns of N/3 bits, written
  column-wise, read row-wise; the first column is the MSB of the symbol
  value, for rate 3/5 the LSB), the Gray mapping (5.4.2: value v = (b0 b1
  b2) -> phase pi/4 x {1, 0, 4, 5, 2, 7, 3, 6}), unit radius like QPSK.
  Checked against gr-dtv (`model/test_model.py::test_plframe_phases`).
* **PLHEADER**: the PLS code already carries MODCOD (5 bits), the frame size
  (the TYPE MSB) and pilots; the PL scrambling, pilots (36 symbols after
  every 16 slots), dummy frames and the header are modulation independent.

## 2. Transmitter

| Block | Change | Cost |
| --- | --- | --- |
| `dvbs2_bb_adapter` | DMA header word: bit 5 short, bit 6 8PSK; codes that do not exist (8PSK below 3/5 or 4/5, short 9/10) are bad headers | few LUT |
| `dvbs2_bbscrambler`, `dvbs2_bch_enc` | per-frame code index {short, rate_idx}; the BCH LFSR (192 bits, left aligned) takes the 168-bit short polynomial | ~10 LUT |
| `dvbs2_ldpc_enc` | short tables appended to the address ROM (+1087 entries, 6447 x 18), Nldpc per code; the parity RAM (49152 x 1) is large enough | ROM fits the same BRAM |
| `dvbs2_frame_fifo` | new: a ring of 98304 bits (1.5 normal frames, as before) in three 32K x 1 banks (bit a: bank a mod 3, row a / 3). QPSK writes bits in order; 8PSK writes bit c N/3 + k to ring bit 3k + c, so a symbol is one row of the three banks (the interleaver for free). It outputs the phase index (mapping included). A frame reserves its N bits when it starts | same 3 BRAM tiles |
| `dvbs2_plframer` | symbols are phase indices p (exp(j pi p / 4)): the header, pilots and dummy bodies are odd p, the scrambler's j^R adds 2R; slots per frame from the configuration; the header's short bit | ~30 LUT |
| `dvbs2_tx` | p -> (I, Q): odd p +-23170 per axis, even p 32767 on one axis | ~20 LUT |

The RRC taps are scaled so a worst-case QPSK sequence peaks at 0.8 of full
scale. 8PSK puts the whole radius on one axis, so the per-axis peak is
sqrt2 larger: model (random symbols, 8 samples/symbol), roll-off 0.35:
0.25 % of the samples clip, clipping error -52.7 dB below the signal (0.20:
-64 dB). Far below what 8PSK needs (~10-12 dB); left as is (a TX gain
register would be the cure if the spectrum regrowth ever matters).

TX VERSION 0x0001_0002 (1.2).

## 3. Receiver

### 3.1 Front end (timing, frame sync, PLS decoder)

* **Timing** (Gardner, AGC on the filter output): modulation independent.
* **Frame sync**: correlates the header differentials of the configured
  code and of the dummy header; it needs the configured header (now with the
  short bit) and the data frame length (now a function of MODCOD, frame
  size and pilots; table above). Its search, checks and flywheel do not
  change. The PLS confirmation 3072 symbols after a tag still fits: the
  shortest frame is still the dummy frame (3330). The header confirmation
  compares the decode with {MODCOD, short, pilots}.
* **PLS decoder**: already decodes all 128 codes (including the short and
  8PSK ones); only its "configured code" gains the short bit.

### 3.2 Carrier, phase tracking

* Frequency acquisition, header and pilot phase estimates, amplitude/noise
  estimates, dummy-frame tracking: all from known pi/2-BPSK/QPSK-like
  symbols, so modulation independent. Frame lengths and the data word count
  per frame become per-slot configuration (functions of MODCOD and size).
  The pilot positions (every 1476 symbols of the body) are unchanged; the
  last segment is 720 (QPSK), 1440 (8PSK), 900 (QPSK short) or 1080 (8PSK
  short) symbols.
* **Pilots on**: data symbols do not drive the loop (phase interpolated
  between known blocks), so 8PSK works as QPSK does. The residual phase error
  must stay well inside +-22.5 degrees instead of +-45; at the 8PSK operating
  points (Es/N0 >= ~5.5 dB for 3/5) the block estimates are 3-5x better
  than at QPSK 1/4's -2 dB, so this holds unless the phase noise is heavy.
* **1/1476 false lock** check: it uses the last segment (pilot block ->
  header), whose length is ~1476/2 for QPSK normal frames. For 8PSK normal
  it is 1440 + 63, almost a whole pilot period, so a k = +-1 false lock is
  not visible there (|k| >= 2 is still corrected from the header average).
  Such locks happen only near QPSK 1/4's threshold (-2 dB), never at 8PSK
  SNRs; not changed.
* **Pilots off, 8PSK**: the QPSK soft-decision detector has stable points
  every 45 degrees on 8PSK, so 8PSK gets its own decision-directed detector:
  the nearest point p (argmax of the 8 correlations below, i.e. the largest
  of |ci|, |u|, |cq|, |v| and its sign) and e = 2 C_(p+2) = 2 Im(s1
  conj(point)) / sqrt2 (the QPSK detector's gain at high SNR). The header
  phase reset per frame resolves the 8-fold ambiguity, as it does the
  4-fold one for QPSK. Model (`rxfixed_experiment.run_x`, 0.002-0.004
  cycles/symbol, 60 ppm): 8PSK 2/3 short at 8 dB and 9/10 at 12 dB without
  pilots decode every frame. A cycle slip of 45 degrees loses the frame, so
  pilots off is realistic only well above the threshold and with a clean
  LO (the README's phase-noise table halves in tolerated sigma). Use pilots
  with 8PSK; the receiver supports both.
* **8PSK soft demapper** (max-log, exact for the Gray mapping): with ci =
  rnd(I/sqrt2), cq = rnd(Q/sqrt2), u = (I + Q) / 2, v = (Q - I) / 2 (the
  correlations with the 8 points, scaled by 1/sqrt2):
  * D0 = max(|ci|, |u|) - max(|cq|, |v|)
  * D1 = max(X) + min(X), X = (u, ci, cq, -v)
  * D2 = max(Y) + min(Y), Y = (u, -ci, cq, v)
  * LLR_b = sat8(rnd(D_b x G, 20)), the same gain G as QPSK (QPSK's G I is
    the max-log difference sqrt2 I in the same units).

  Cost: two constant multipliers (DSP), two adds, comparator trees, a third
  G multiplier (DSP): about +3 DSP, +250 LUT, two pipeline stages in front
  of the existing LLR multiply (the output FIFO's room checks grow by two).
* **Output word**: QPSK {Q, I}, 8PSK {b2, b1, b0} (3 LLRs, the symbol's bits
  in order); per data frame Nldpc / 2 or Nldpc / 3 words (32400, 8100, 21600,
  5400).

### 3.3 LLR queue and deinterleaver

* The DDR ring keeps 64 KB per frame; a frame is Nldpc bytes (64800 or
  16200), written in arrival order (8PSK: symbol order, 3 bytes per
  symbol). The packer takes 2 or 3 bytes per word into 8-byte beats; bursts
  of 16 beats, the last of a frame 4 (normal) or 9 (short) beats; the frame
  size travels with each staged beat.
* **8PSK bit deinterleaver in the reader**: column c (bits c N/3 .. (c+1)
  N/3 - 1) is byte c of every symbol (byte 2 - c for rate 3/5). The reader
  reads the frame three times; pass c takes every third byte (2 or 3 per
  beat), one LLR per clock, paired into words of 2 LLRs in bit order. The
  LDPC decoder's input is unchanged (bit order, 2 LLRs a word; it takes one
  LLR per clock anyway). DDR traffic: 3 bytes written and 9 read per 8PSK
  symbol, 29 MB/s at 2.38 MBd (HP port: ~800 MB/s).
* Alternatives considered: deinterleaving in the LDPC input writer (three
  walkers; the channel-sign memory's 32-bit words straddle columns for short
  frames) or in the packer (three write streams per slot, 3x the staging
  memory). The reader is the cheapest.

### 3.4 LDPC and BCH decoders

* **LDPC**: the same layered decoder (18 lanes over 360 x 360 circulants);
  the code index {short, rate_idx} (5 bits) selects q, groups, degree, Kbch,
  Kldpc and Nldpc. Irregular short codes: each layer is padded to the code's
  largest check degree with absent edges (group 255 in the edge ROM: never
  the minimum, sign 0, no parity contribution, no write), so the decoder
  keeps one degree per code; model and RTL bit exact. The two parity edges
  of every layer (groups j - 1 / q - 1 shifted, and j) are now generated by
  the RTL instead of stored, so the edge ROM shrinks from 6828 to 6540
  entries with the short codes in it (8380 with the parity edges would have
  needed a 16K x 18 ROM: +4.5 BRAM tiles in a first OOC run). The APP,
  R-message and sign memories are sized for normal frames and fit short
  frames. Clocks per iteration: 10-14k for short codes (normal: 30-55k),
  so the iterations per frame time are about the same as for normal frames.
* **BCH**: short frames are GF(2^14), t = 12. The decoder gets a field mode:
  the syndromes keep 8 bits per clock (a second set of constant multipliers,
  selected per frame), the Berlekamp-Massey and Chien start multipliers
  reduce by either polynomial, and the Chien search for short frames runs
  1 position per clock (12 constant x alpha^j multipliers instead of a
  second 8-position array; at most 14400 clocks per short frame, well
  inside a short frame's time). Measured (OOC): +780 LUT.
* **8PSK** frames use the normal codes: nothing new in the decoders.

### 3.5 Frame tag, packets, registers, software

* The code index {short, rate_idx} goes from the LLR queue (computed from
  the slot's MODCOD and size) through the LDPC and BCH decoders (m_rate);
  the packet header gets SHORT at bit 38 (was 0).
* Per slot MODCOD register: MODCOD 1..17, SHORT at bit 9 (as in the PLS
  registers). RX VERSION 0x0002_0100 (2.1).
* Kbch per code: the BBFRAME packet is Kbch/8 bytes (short: 384..1779).
* Iteration budget (software, rx2_design 6.2): per frame clocks and frame
  times change with the code: 8PSK frames come 1.5x as often per symbol;
  short frames 4x, with ~1/4 of the clocks per iteration.

## 4. Resources

Out of context, xc7z020-2, 100 MHz (`syn/ooc.tcl`; "before" = master
f49cfa9 the same way; BRAM in tiles):

| Module | LUT before | LUT after | BRAM before / after | DSP before / after | WNS after (before) |
| --- | ---: | ---: | --- | --- | --- |
| `dvbs2_tx` | 1368 | 1582 | 13.5 / 12.5 | 16 / 16 | 3.02 (3.18) |
| `dvbs2_bch_dec` | 3802 | 4582 | 2 / 2 | 0 / 0 | 1.03 (1.75) |
| `dvbs2_ldpc_dec` | 7321 | 7458 | 80.5 / 80.5 | 0 / 0 | 1.12 (1.34) |
| `dvbs2_rx_axi` (whole RX) | 25646 | 27525 | 102.5 / 102.5 | 89 / 94 | 0.41 (0.80) |

Inside `dvbs2_rx_axi`: carrier2 6037 -> 7075 LUT (+5 DSP: the 8PSK
demapper's 1/sqrt2 products and the third gain multiplier; Vivado's
mapping of the wider products), BCH 4145 -> 4638, LLR queue 826 -> 1117,
frame sync 2383 -> 2558, LDPC 7657 -> 7450 (the smaller edge ROM).

A first version of the carrier failed OOC timing at -2.9 ns: the 8PSK phase
detector's argmax in one clock and the demapper's max/min trees feeding the
LLR multiplier. Both are now pipelined (detector: three states, M_SYM4..6,
for 8PSK data symbols without pilots only; demapper: stages a, b, c before
the multiplier, the output FIFO's room checks one word lower); the BCH's
Chien start multiplier takes registered operands. Result above.

Whole design, estimate from the OOC deltas: ~41.8k + 2.1k = ~43.9k LUT
(82.5 %), BRAM unchanged (135 of 140), DSP 155 + 5 = 160. No BRAM
fallback needed. Full build: section 6.

## 5. Software (station) changes needed

See the hand-back list; summary: MODCOD tables with 8PSK and the short
flag, Kbch/Kldpc per code, PLFRAME length per MODCOD/size/pilots, register
fields (RX MODCOD.SHORT, packet HDR.SHORT, TX HDR.SHORT/PSK8), VERSION
checks (TX >= 1.2, RX >= 2.1), the scan/detect lists, the iteration budget,
the `supported` flags in api.md.

## Implementation status

Implemented (model first, then RTL, bit exact), all simulation tests green:

* Model: `txmodel` (short codes, phase-index symbols, 8PSK interleaver and
  mapping; against gr-dtv), `ldpcdec` (short codes, padded layers),
  `bchdec` (GF(2^14)), `rxfixed` (frame lengths, short header bit, 8PSK
  demapper `psk8_metrics`, detector `psk8_pll_error`, `llr_bit_order`),
  `rxfixed_experiment.run_x` (any MODCOD and frame size, decoded with the
  LDPC/BCH models).
* TX 1.2, RX 2.1, `regs/` (TX HDR.SHORT/PSK8, VERSION_8PSK; RX MODCOD.SHORT,
  HDR.SHORT, VERSION_8PSK), `dvbs2-tx` (`-m 8psk..`, `-S`), `dvbs2-rx`
  (`-m 8psk..`, `--short`, `--detect` follows 8PSK and short codes, the
  iteration budget per code, packets with HDR.SHORT), `dvbs2-selftest -S`.
* Tests: see the README section "8PSK and short FECFRAMEs". The RX
  suites (`test_rx.py`, `test_rx2.py`, `test_llrq.py`, `test_rx_8psk.py`):
  131 passed. gr-dvbs2rx is not available for the host's Python 3.14, so
  `rxmodel.gr_ldpc_decode_llr` falls back to the LDPC/BCH models (the
  independent decoder check is lost until gr-dvbs2rx is rebuilt).

Not done / open:

* Board: nothing measured on the board (no bitstream load allowed in this
  work); the 8PSK thresholds, phase noise without pilots and the TX clipping
  are model results only.
* 1/1476 false-lock check for 8PSK normal frames (section 3.2).
* The station (Rust) side: see the hand-back list.
