# PlutoSAT: a second DATV receive slot (feasibility study and design)

Status: **final** study and prototypes, 2026-11-04. The carrier buffer fix
(section 2 item 5) is merged into master (commit 1349130, branch
`wt/carrier-bram`). Nothing else is integrated. The implementation is a
separate, later task, which starts once the PLS decoder and the AXI-port
re-plan have landed. All prototype RTL, tests and OOC runs are in this
directory (`rx2/`).

**Decided by the user (2026-11-04):**

* total symbol rate of both slots: **at most 2.38 MBd** (kept; no
  two-thread timing engine);
* **DDR LLR queue** with a 2 MB reserved region, on its own **HP port
  (HP1)**;
* **shared LDPC iteration budget**;
* **one DMA stream with a slot tag** (header word 0 bit 55);
* **per-slot register banks**;
* the two-slot **frame sync and carrier come after the PLS decoder merge**;
* the ADI FIR decimator/interpolator and the IIO DMA paths
  (axi_ad9361_adc_dma + cpack, axi_ad9361_dac_dma + upack) are removed from
  the design. That frees 2342 LUT, 4824 FF, 2 BRAM tiles, 44 DSP and the
  HP1/HP2 ports.

## 1. Requirements (as agreed)

* Two DATV RX slots on rx0 (12 MS/s complex). Each slot has its own frequency
  (DDC NCO), symbol rate (33.33 kS/s .. 2 MS/s, 6 MS/s / decimation), MODCOD
  (QPSK 1/4 .. 9/10, normal frames, pilots on or off) and Gold code.
* The two symbol rates together come to at most **2.38 MBd**, which is
  today's single-receiver maximum: 42 clocks/symbol at 100 MHz in the
  timing loop.
* Architecture: a time-multiplexed front end and a shared LDPC/BCH decoder.
* The TS of both slots reaches the PS through DMA: either one stream with
  tagged packets or a second DMA. Both options are compared below.

## 2. Summary

**Two slots fit, with trade-offs.** A plain copy of the front end does not
fit: it adds about 11.2k LUT and 70 DSP, with 40 DSP free and LUTs at 77 % in today's build. Even after the
FIR/IIO DMA removal (84 DSP free) the LUT and BRAM cost of a copy is too high.
The design that fits:

1. **Two DDCs.** They are small, and their FIR engine runs at up to 72 % load
   per slot, so a shared one would not keep up. The two DDCs share one NCO
   ROM (one port each).
2. **One timing engine, one frame-sync engine and one carrier engine for both
   slots.** Each slot's state lives in a second register set (a "shadow"),
   swapped in one clock between symbols. Each slot's buffers get the slot as
   the address MSB. Per slot, every engine does exactly what today's
   single-slot engine does. Prototypes for timing and carrier are **bit-exact
   per slot** to the model (Verilator), timing, frame sync and carrier alike.
3. **LLR frames queued in DDR.** A small AXI master on its own **HP port**
   (HP1, freed by the IIO DMA removal) writes each slot's LLR frames into a ring buffer and
   reads them back into the existing, unchanged 2-copy LDPC decoder. This
   needs no extra decoder BRAM. A frame queue on chip would need 18 tiles
   per frame store, which doesn't fit.
4. **One DMA stream with a slot bit** in the packet header, plus per-slot
   register banks.
5. **A separate, immediate win:** today's carrier puts its `seg` (2Kx32) and
   `zbuf` (1Kx32) buffers in LUTRAM, because the `ram_style = "block"`
   attribute is "infeasible" as written. Rewriting them so they infer BRAM
   saves **2429 LUT for +3 BRAM tiles**. It is bit-exact (7/7 carrier tests),
   and it pays for most of the second slot's LUTs.

Expected board totals, after the FIR/IIO DMA removal: LUT about 41.0k (77 %,
the same as today's build), BRAM about 133 of 140 tiles once some BRAM is
recovered (138.5 without), DSP 150 of 220 (68 %), clock still 100 MHz. Main
risks: BRAM headroom is about 7 tiles, and the carrier's critical path is
+0.04 ns (fixable).

## 3. Where the resources go today

Taken from the last full build's routed checkpoint
(`boards/fishball/hdl/fishball.runs/impl_1/system_top_routed.dcp`), opened
read-only. Reports are in `rx2/board/`.

Totals: 40770 LUT (76.6 %), 37808 FF (35.5 %), **slices 95.8 %**, BRAM 129
of 140 tiles (111 RAMB36 + 36 RAMB18), DSP 180 of 220. WNS +0.163 ns.

| Block | LUT | FF | BRAM tiles | DSP | Notes |
| --- | ---: | ---: | ---: | ---: | --- |
| **dvbs2_rx** total | 23186 | 11844 | 95 | 70 | |
| &nbsp; LDPC decoder | 7462 | 3594 | 80.5 | 0 | 2 APP copies 36, R memory 27, sign memory `smem` 9, channel signs 2x2, edge ROM 5 (+0.5) |
| &nbsp; carrier | 7391 | 2811 | 1 | 33 | 1 tile = rotator NCO ROM; 2036 LUTRAM (seg, zbuf, zrbuf) |
| &nbsp; BCH decoder | 4097 | 1643 | 2 | 0 | frame buffer |
| &nbsp; frame sync | 1955 | 1286 | 3.5 | 8 | seven RAMB18, each only 128 deep |
| &nbsp; timing | 738 | 424 | 0.5 | 15 | RRC coefficients |
| &nbsp; DDC | 665 | 1046 | 2.5 | 14 | samples 2x RAMB18, taps RAMB18, NCO ROM RAMB36 |
| &nbsp; input FIFO (async, LUTRAM) | 473 | 92 | 0 | 0 | |
| &nbsp; AXI wrapper | 286 | 854 | 5 | 0 | packet buffer 4 (2 x 8 KB), snapshot 1 |
| nbiq | 6288 | 7027 | 6.5 | 16 | 5 channels about 500 LUT each, comb 1606, FIR 901, ring writer `nbiq_wr` 668 |
| axi_ad9361 (incl. IQ/DC correction) | 3010 | 6172 | 0 | 20 | |
| dvbs2_tx | 1376 | 1674 | 13.5 | 16 | LDPC enc 6.5, frame FIFO 4, NCO 1, RRC 2 |
| fft (spectrum) | 1111 | 2088 | 10 | 6 | xfft 4.5, accumulator 4.5, window 1 |
| rx_awgn | 953 | 1359 | 0 | 8 | |
| ADI rx_fir_decimator | 628 | 1291 | 0 | 22 | SRL-based, **no BRAM**; removed (user decision) |
| ADI tx_fir_interpolator | 595 | 1915 | 0 | 22 | SRL-based, **no BRAM**; removed (user decision) |
| 4 x axi_dmac | 1829 | 2513 | 4 | 0 | one tile store-and-forward each; adc/dac DMAs (907 LUT, 2 tiles) are removed (user decision) |
| cpack + upack | 212 | 251 | 0 | 0 | removed (user decision) |
| GP0 interconnect, IIC, SPI, packers | about 1800 | about 1900 | 0 | 0 | |

What this means:

* The ADI FIR filters cost 1223 LUT and 44 DSP but **no BRAM**, so removing
  them does not give BRAM back. The user has decided to remove them anyway,
  together with the IIO DMAs (section 1). The IQ/DC correction stays.
* The free resources are 11 tiles, 40 DSP and about 12k LUT. Slices are
  already at 95.8 %, so LUT is the real limit: about 85 % LUT closed
  earlier (awgn build) with WNS +0.059 ns.
* 80.5 of the RX's 95 tiles are the LDPC decoder. Each frame store in the
  decoder's format (18 lanes x 4096 x 9 bit) costs 18 RAMB36. That rules out
  extra frame stores on chip.
* There is **no 9 Kb BRAM primitive on 7-series**. The smallest is RAMB18
  (half a tile), and any two RAMB18 can share a tile. The frame-sync and
  timing memories use only 128 of the 1024..2048 entries their RAMB18 offers,
  so the slot bit fits into the unused depth: **their second slot costs 0 or
  0.5 tiles.**
* BRAM vs LUTRAM: the profitable direction is LUTRAM to BRAM in the carrier
  (section 2, item 5). The reverse is possible but expensive: moving the
  frame-sync RAMB18s to LUTRAM costs about 470 LUT (one slot) or about 950
  LUT (two slots) for 3.5 tiles.

## 4. Block diagram

```
              rx0 (axi_ad9361 ch0 -> rx_awgn), 12 MS/s, l_clk
                 |                                   |
          +------+------+                     +------+------+
          | DDC slot 0  |   shared NCO ROM    | DDC slot 1  |      2 x (NCO, CIC3, FIR)
          +------+------+   (port A / B)      +------+------+
                 | 2 sps                             | 2 sps
          [ FIFO 512, RAMB18 ]                [ FIFO 512, RAMB18 ]          sys_cpu_clk (100 MHz) from here
                 \___________________   _____________/
                                     \ /
                       +---------------------------+
                       | timing (shared MAC, 42 clk/sym, per-slot k/mu/AGC/loop, coef bank per slot)
                       +---------------------------+   m_valid[2] -> s_valid2[2]
                       | frame sync (shared correlator; context swap in S_IN; RAMB18 address MSB = slot)
                       +---------------------------+   m_valid + m_slot
                       | carrier (shared FSM/DSPs/CORDIC/divider; context swap in M_IDLE;
                       |          seg 4Kx32 + zbuf 2Kx32 BRAM, MSB = slot)
                       +---------------------------+   LLR words + slot tag, 32400 per frame
                                     |
                       +---------------------------+     HP1 (64 bit)          +-------------------+
                       | LLR queue writer          |-------------------------->| DDR ring slot 0/1 |
                       | (per-slot 64 KB frames)   |                            | 2 x 8..16 x 64 KB |
                       | LLR queue reader (FIFO    |<---------------------------| 0x3E900000, 2 MB  |
                       |  order, frame descriptor) |                            +-------------------+
                       +---------------------------+
                                     | 64800 LLRs + {slot, modcod, pilots}
                       +---------------------------+
                       | LDPC (unchanged ping-pong, rate per frame) -> BCH (rate per frame)
                       +---------------------------+
                                     | BBFRAME + {slot, status}
                       +---------------------------+
                       | packet buffer (shared, 2 x 8 KB), header word 0 bit 55 = slot
                       +---------------------------+
                                     | AXI-Stream
                          axi_dvbs2_rx_dma (HP0, IRQ ps-9, unchanged)
```

## 5. Time multiplexing the front end

### 5.1 Throughput budget at 100 MHz

| Engine | Clocks per symbol | Limit on the sum of both rates | Measured |
| --- | --- | --- | --- |
| timing (shared) | 42 (41 busy + 1 pick) | **2.38 MBd** (the binding limit) | prototype: 46588 symbols of 2 slots in 1.963 M clocks (42.1/symbol), no switching overhead |
| frame sync (shared) | about 34 + 1 swap | about 2.9 MBd | from the RTL header comment, not measured |
| carrier (shared) | about 8 per symbol, plus ~900 per header and segment output; +1 per swap | about 7 MBd | prototype: 164.6k symbols (QPSK 1/2 pilots + 3/4 no pilots) in 2.25 M clocks = 13.7/symbol with a swap on every symbol |
| DDC (one per slot) | FIR load 72 % (FIR alone) or 96/R % (with the CIC) | per slot | a shared FIR would exceed 100 % for two FIR-alone slots, so it is duplicated |
| LDPC (shared) | 27k..59k per iteration | iteration budget, section 6 | |

**A faster clock is not realistic for the shared parts.** OOC WNS at 100 MHz
is +0.04..0.18 ns for the carrier, +0.33 ns for the LDPC and +2.27 ns for
the two-slot timing engine (about 129 MHz). If the user wants more than 2.38
MBd in total, the cheaper route is a two-thread timing engine: while slot A
runs its 17-clock loop update, slot B uses the MAC. That gives about 26
clocks per symbol, about 3.8 MBd total, for about +300 LUT and roughly a
day of work. The LDPC iteration budget then drops in proportion.

### 5.2 State per slot vs shared

| Engine | Shared | Per slot (duplicated) | Prototype |
| --- | --- | --- | --- |
| DDC | NCO ROM | the whole DDC (NCO phase, CIC, FIR buffers, taps) | measured: 640 LUT, 14 DSP, 2.5 tiles each (1.5 with the shared ROM) |
| timing | MAC pipeline, gain, TED, loop arithmetic (15 DSP) | sample buffer 64x32 (LUTRAM), coefficient bank (same RAMB36, MSB = slot), k, mu, integ, gacc, g, prev, kp/ki, output register | `rtl/dvbs2_rx_timing2.sv`, hand-written, bit-exact per slot |
| frame sync | correlator lanes, DSPs, FSM | 51 registers (1092 bits) in the shadow set, lane and symbol memories (MSB = slot), header pattern and length per MODCOD | `rtl/dvbs2_rx_framesync2.sv`, generated by `proto/make_fs2.py`, bit-exact per slot (OOC numbers from the generator version before the Verilator-compatible header-pattern rewrite; same logic) |
| carrier | FSM, 33 DSP, CORDIC, divider, rotator, zrbuf (scratch for each header), LLR pipeline, output FIFO (slot tag per word) | 122 registers (2359 bits, an upper bound: it includes temporaries), seg and zbuf (MSB = slot), header pattern, per-slot output counters | `rtl/dvbs2_rx_carrier2.sv`, generated by `proto/make_carrier2.py`, bit-exact per slot |

The swap is mechanical:

* every persistent register R gets R_sh;
* on `ctx_swap`: R <= R_sh and R_sh <= R;
* `ctx_swap` = the engine is idle, the other slot has a symbol, and the
  current slot has none or was just served.

So the engine alternates per symbol when both slots are busy. Because the
carrier and frame sync are purely symbol-driven (even the carrier's segment
timeout counts symbols, not clocks), the swap has no effect on the result.
Two RTL details:

* the swap's restore comes last in the FSM `always_ff`, so it wins over
  that block's other assignments in the swap clock;
* the slot tag of an LLR travels with the word through the LLR pipeline
  and the output FIFO.

The generators run on today's RTL, and can be re-run once the PLS decoder
lands (section 10).

Not yet in the prototypes:

* a per-slot reset of a shadowed engine (timing2 has one). It must reset
  the working set or the shadow set, depending on which slot is loaded, and
  costs a few dozen LUT;
* per-slot `ty_v` delivery into the shadow set. The frame-sync generator
  does it; the carrier only produces `ty_v` for the loaded slot, so it is
  correct by construction.

### 5.3 Prototype OOC results (xc7z020-2, 100 MHz, place and route)

| Module | LUT | FF | BRAM tiles | DSP | WNS ns | Verified |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| timing (today) | 698 | 424 | 0.5 | 15 | 3.17 | |
| **timing2** (2 slots) | 955 | 727 | 1 | 15 | 2.27 | bit-exact per slot vs `rxfixed.Timing` (2 cases, different signals, roll-offs, gains, stalls) |
| carrier (today) | 7151 | 2829 | 1 | 33 | 0.145 | |
| carrier, seg/zbuf as BRAM (1 slot) | **4722** | 2751 | 4 | 33 | 0.175 | `test_rx.py -k test_carrier`: 7/7 pass |
| **carrier2** (2 slots) | 6203 | 4941 | 7 | 33 | **0.037** | bit-exact per slot vs `rxfixed.Carrier` (3 cases: pilots + no pilots, 0 dB 1/4 with 20 % stalls, a gap in one slot) |
| frame sync (today) | 1930 | 1316 | 3.5 | 8 | 1.56 | |
| **framesync2** (2 slots) | 2687 | 2645 | 3.5 | 8 | 1.40 | bit-exact per slot vs `rxfixed.FrameSync` (2 cases: pilots + no pilots; 0 dB 1/4 with 25 % stalls and dropped symbols in the other slot: flywheel, lost, re-lock); measured 33 clocks/symbol incl. swaps |

Cost of the second slot: timing **+257 LUT, +0.5 tile**; carrier **+1481
LUT, +3 tiles** over the BRAM version; frame sync **+757 LUT, 0 tiles**. None
of them needs extra DSPs. Copying instead would cost +698 LUT/+15 DSP,
+7151 LUT/+33 DSP and +1930 LUT/+8 DSP.

The carrier2 worst path runs from the zbuf BRAM (no output register)
through the 16-level M_HZ2 negate/add/round/saturate into `zw_d`. Today the
same path ends at +0.145 ns. The fix is local: register `zb_q` once more
and add one state, which costs 90 clocks per header.

## 6. Sharing the LDPC/BCH decoder

### 6.1 Why the frame queue goes to DDR

The decoder decodes one frame in place in one APP copy while the next frame
is written into the other copy. With two streams, each slot must always
have somewhere to write its LLRs: 64800 per frame, and a carrier stall can
only be absorbed for about 250 symbols. Meanwhile frames from the other
slot wait for the decoder.

On chip, a store pool must hold 2 filling + 1 decoding + at least 2 waiting
frames. Otherwise the non-preemptive decoder either starves a frame (two
slots finishing frames nearly together leaves the second one ~0 time) or
caps the slow slot's iterations at T_fast - D_fast. Store sizes in the
lane layout the decoder needs (18 lanes x 3600, one lane written per clock):

| Store width | Per frame (tiles) | Comment |
| --- | ---: | --- |
| 9 bit (APP copy) | 18 | today's format |
| 8-bit LLR, lossless | 18 | 4Kx9 per lane |
| 6 bit | about 13.5 | 4Kx4 + shared 8Kx2 per lane pair |
| 4 bit | 9 | 4Kx4 RAMB18 per lane; LLR quantisation loss not measured, typically 0.2-0.3 dB |

Even 4-bit stores need +18..36 tiles, and only 11 are free. **Off-chip
is the only option that doesn't remove cores.**

DDR LLR queue:

* Writer: the carrier's tagged LLR words (2 x 8 bit) go into a per-slot
  staging FIFO, then out in 128-byte AXI bursts to `ring[slot][frame]`.
  Each frame slot is 64 KB (64800 bytes used). The ring has 8..16 frames per
  slot, 2 MB in total, e.g. reserved at 0x3E900000 just below nbiq's buffer
  at 0x3EB00000.
* Port: its own **HP port** (HP1, freed by removing axi_ad9361_adc_dma;
  64-bit AXI3). Bandwidth is 2 x 2.38 MBd x 2 bytes = 9.5 MB/s, well under
  1 % of the port. S_AXI_GP1 also remains free as a fallback.
* Reader: frames go out in completion order (a 16-entry descriptor FIFO
  of {slot, index, modcod, pilots}). They are read in natural order into
  the decoder's existing input writer: 64800 clocks, overlapped with
  decoding by the existing ping-pong.
* Ring full means the decoder fell behind. The frame is dropped and
  counted (`LLR_DROPPED[slot]`), as `PKT_DROPPED` is today.
* Cost, estimated from `nbiq_wr` (668 LUT, a 5-ring writer on GP0): writer
  about 700 + reader about 400 + descriptors/control about 200, so **about
  +1.3k LUT**, plus about +1.5 tiles of staging/read FIFOs, plus the HP port
  (already enabled today for the adc DMA) and a DT `reserved-memory` node
  (2 MB, agreed).
* LDPC changes: `rate_idx` is latched per buffer (one copy for the input
  writer, one for the decoder) instead of static, and {slot, modcod,
  pilots} travel with the frame. The BCH latches its rate per frame. About
  +200 LUT. This is also what VCM/ACM through the PLS decoder needs.

### 6.2 Iteration budget

Clocks per iteration are q x S x (2 deg + 14): 59.4k (1/4), 50.4k (1/2),
37.8k (3/4), 26.6k (9/10). The per-frame overhead (output and BCH wait) is
about Kldpc/8 + Kbch/4 + 500. The decoder holds at most 95 % of 100 MHz.
With the same iteration count for every frame (computed; normal frames with
pilots):

| Slots | Frames/s | Iterations per frame (cap 63) |
| --- | ---: | ---: |
| 2.38 MBd 1/4 (one slot, today's worst case) | 71.5 | 22.3 |
| 1.19 MBd 1/4 + 1.19 MBd 1/4 | 71.5 | 22.3 |
| 2 MBd 1/4 + 333 kBd 1/4 | 70.1 | 22.7 |
| 2 MBd 1/2 + 333 kBd 1/2 | 70.1 | 26.6 |
| 1.5 MBd 4/5 + 857 kBd 1/2 | 70.8 | 32.1 |
| 1.5 MBd 4/5 + 333 kBd 2/3 | 55.1 | 46.2 |
| 1 MBd 1/2 + 1 MBd 3/4 | 60.1 | 35.5 |
| 333 kBd 1/2 + 333 kBd 1/2 | 20.0 | 63 |

So **sharing costs no iterations** compared with one receiver at the summed
rate. Frames that converge early (most frames above threshold) leave their
time to the queue, which DDR makes possible.

Policy:

* software sets `LDPC_MAX_IT[slot]` from this formula (as `dvbs2-rx` sets
  today's single value);
* the hardware caps a frame at `LDPC_MAX_IT_LO` while the queue holds more
  than `QHI` frames (registers), so a run of non-converging frames at low
  SNR cannot fill the ring;
* frames are decoded in completion order (FIFO).

Queue depth: in any window of the shorter frame time at most one frame per
slot arrives. With caps that keep the load at or below 95 %, at most 2-3
frames are waiting at once, so 8 per slot (512 KB) leaves a large margin.

**BCH per frame:** about 16k clocks per frame (n/8 + Kbch/4 + 500), 1.1 %
of the clock at 71.5 frames/s. One BCH decoder serves both slots.

## 7. Interfaces

### 7.1 Registers: per-slot banks, not a slot-select register

The AXI address grows from 8 to 10 bits (the BD segment is 64 KB):

* `0x000-0x0FF`: today's map, which is slot 0. Old `dvbs2-rx` keeps working
  when VERSION is checked.
* `0x100-0x1FF`: slot 1, the same layout: MODCOD, GOLD, DDC_*, RRC coefs,
  gains, FS_THRESH, PLL, LDPC_MAX_IT, every status and counter register.
* `0x200-0x2FF`: shared. VERSION 0x0002_0000, ID, CTRL ([0] enable,
  per-slot enable and reset bits), SNAP (slot select bit), LLR ring base,
  size and frames per slot, LLR_DROPPED[2], QHI/MAX_IT_LO, queue depth,
  PKT counters.

A slot-select register would make two processes, or one process plus a
status poller, race each other. Banks cost about 700 LUT in total (the
second register set plus the read multiplexer). The constellation snapshot
buffer (1 RAMB36) is shared; a bit selects the slot.

### 7.2 DMA

| | One stream, slot tag (**decided**) | Second axi_dmac |
| --- | --- | --- |
| Hardware | 0: header word 0 bit 55 = slot (free today); packet buffer shared | +about 500 LUT, +1 tile, a second packet buffer (+4 tiles, or 0 if the existing 2 slots are split one per stream), IRQ ps-14 |
| AXI port | HP0 as today | HP2 (freed by removing the dac DMA) |
| Software | one owner of the DMA demultiplexes by the tag (`dvbs2-rx` handling both slots, or plutosatd) | two independent `dvbs2-rx` processes |

At most 71.5 packets/s of 8 KB, about 0.6 MB/s, so one stream has plenty
of room. With two slots, the two packet slots in the buffer suffice:
draining takes about 10 µs, and software keeps 4 transfers queued. A
second interrupt is not needed (ps-9 as today).

## 8. Resource budget, today vs after

Measured values come from OOC runs (`syn/out2_*`); estimates are marked "est.".

| Change | LUT | FF | BRAM tiles | DSP | Source |
| --- | ---: | ---: | ---: | ---: | --- |
| carrier seg/zbuf to BRAM (useful on its own) | -2429 | -78 | +3 | 0 | measured |
| carrier, second slot | +1481 | +2190 | +3 | 0 | measured |
| timing, second slot | +257 | +303 | +0.5 | 0 | measured |
| frame sync, second slot | +757 | +1329 | 0 | 0 | measured |
| DDC #2 (shared NCO ROM) | +640 | +1039 | +1.5 | +14 | measured (DDC OOC) |
| two synchronous BRAM input FIFOs instead of 1 LUTRAM async FIFO | -350 | +0 | +1 | 0 | est. (in_clk = clk on the board) |
| glue (fs to carrier demux, tags) | +50 | +50 | 0 | 0 | est. |
| LDPC/BCH rate and slot per frame | +200 | +150 | 0 | 0 | est. |
| DDR LLR queue (HP1) | +1300 | +1500 | +1.5 | 0 | est. from `nbiq_wr` |
| AXI: slot-1 register bank, map v2 | +700 | +800 | 0 | 0 | est. |
| **Sum** | **+2606** | **+7283** | **+11.5** | **+14** | |
| Board today | 40770 (76.6 %) | 37808 | 129 | 180 | routed |
| ADI FIRs, IIO DMAs, cpack/upack removed (user decision) | -2342 | -4824 | -2 | -44 | measured (routed hierarchy) |
| Board baseline for the 2-slot work | 38428 (72.2 %) | 32984 | 127 | 136 | |
| **Board after** | **about 41.0k (77.1 %)** | about 40.3k (38 %) | **138.5 (99 %)** | **150 (68 %)** | |
| BRAM recovery (pick about 6) | | | | | |
| &nbsp; LDPC edge ROM 6828x18 repacked (5 to 3.5 tiles) | +about 30 | | -1.5 | | est. |
| &nbsp; LDPC `smem` 15840x18 as 8 x 2Kx18 (9 to 8) | +about 20 | | -1 | | est. |
| &nbsp; frame sync: 2 lanes per RAMB18 via its 2nd port (7 to 4 RAMB18) | +about 50 | | -1.5 | | est. |
| &nbsp; carrier seg packed 2 x 1440 (4 to 3 RAMB36) | +about 20 | | -1 | | est. |
| &nbsp; snapshot 512 symbols (RAMB36 to RAMB18) | 0 | | -0.5 | | est. |
| **Board after, with recovery** | **about 41.1k (77 %)** | about 40.3k | **about 133 (95 %)** | 150 | |

Reserve still needed: the PLS decoder (in flight) and the TX CIC (in flight,
probably a few hundred LUT, no BRAM).

## 9. Timing risk

* The device is full: slices 95.8 % today. After the FIR/IIO DMA removal the
  2-slot design is back at about 77 % LUT, the level of today's build, which
  closes with WNS +0.163 ns. Mitigations if it gets tight:
  * the carrier BRAM fix first (-2.4k LUT);
  * pblocks for the RX engines if needed;
  * as a last resort, a second bitstream "2-slot" without nbiq (-6.3k LUT,
    -6.5 tiles, -16 DSP) and/or without the FFT (-1.1k LUT, -10 tiles).
* The carrier's worst path (header z computation) is +0.04 ns in carrier2
  OOC. Fixed with one pipeline stage (section 5.3).
* The LDPC (+0.33 ns OOC) is unchanged in the datapath. The per-frame rate
  registers are latched once, which should not hurt.
* The shared engines add one 2:1 mux level only at the shadow restore
  (FF D input), not on read paths, so logic depth is unchanged. timing2 is
  2.27 ns, framesync2 1.40 ns.
* BRAM 96 %: placement is constrained, and the LDPC's 80 tiles span many
  BRAM columns.

## 10. In-flight work by other workers

* **PLS decoder (frame sync, carrier, AXI).**
  * The two-slot frame sync and carrier must be generated from the
    post-PLS RTL. The generators re-run as long as the style stays the
    same: one FSM `always_ff`, memories written and read in their own
    processes or by address.
  * The PLS decoder's per-header state becomes part of the context, a few
    dozen bits. Its tables are shared.
  * Per-frame MODCOD from the PLS makes the LDPC/BCH per-frame rate (6.1)
    necessary anyway, so this work overlaps.
  * If the frame sync switches from known-MODCOD correlation to SOF+PLSC,
    its clocks/symbol change. Re-check that it stays at or below 42.
  * Please keep the carrier's new buffers BRAM-inferable (the seg/zbuf
    lesson). Plan the 2-slot work after the PLS decoder merges.
* **TX CIC** (`dvbs2_cic_interp.sv`, `dvbs2_tx*`): it doesn't touch RX
  files. It takes a few hundred LUT out of the reserve, and it makes the
  TX NCO ROM sharing idea (TX NCO + DDC NCO + rotator NCO: three identical
  1024x32 ROMs) a later 1-tile BRAM saving.

## 11. Verification plan

1. **Per-slot equivalence** (the prototype method): each two-slot engine
   gets two different signals (rates, MODCODs, pilots, CFO, ppm, gains,
   roll-offs, stalls, dropped symbols). Each slot must equal the
   single-slot model word for word. Done for timing and carrier
   (`sim/rx/test_rx2.py`, `test_rx2_carrier.py`, `test_rx2_framesync.py`); still needed for frame
   sync and for the post-PLS versions.
2. **Per-slot reset and reconfiguration:** reset or retune slot 1 in the
   middle of a run. Slot 0's output must be identical to a run without the
   reset.
3. **LDPC queue:** two interleaved frame streams with different rates go
   through the queue (a Verilator AXI slave memory model). Per frame: equal
   to `ldpcdec.py` in bits, iteration count and convergence, with the
   correct slot tag. Also overflow (drop and count) and the QHI/MAX_IT_LO cap.
4. **End to end `tb_rx_axi`:** one 12 MS/s ADC stream holding two DVB-S2
   signals at different offsets and rates (e.g. 1.5 MS/s 4/5 + 333 kS/s
   1/2, and 2 MS/s + 333 kS/s at the 2.38 MBd limit, 1/4 at low SNR).
   Every packet of each slot must equal the transmitted BBFRAMEs, the slot
   bit must be right and no frame may be lost.
5. **Software:** `dvbs2-rx` per-slot configuration matches the model
   (`test_ddc_sw_config` per slot); the packet demultiplexer is tested
   with recorded packets.
6. **Board:**
   * the loopback self-test with the TX signal received by both slots
     (same signal at two DDC offsets, or slot 1 on another rate with
     `dvbs2-tx` reconfigured), plus the PL noise source;
   * on air: the QO-100 beacon (1.5 MS/s) + one narrow DATV signal;
   * check the WNS of the full build and the per-slot counters
     (`LLR_DROPPED`, `IN_OVERFLOWS`, `PKT_DROPPED`).

## 12. Implementation plan (ordered, estimated effort)

| # | Step | Effort | Depends on |
| --- | --- | --- | --- |
| 0 | Carrier seg/zbuf as BRAM + the M_HZ2 pipeline stage, on today's single-slot RX; full build. Gives LUT headroom. | 0.5 d | none (can go now, coordinate with the PLS worker) |
| 1 | Wait for the PLS decoder to merge; re-run the RX tests | n/a | |
| 2 | LDPC/BCH: rate and {slot, modcod, pilots} per frame; tests | 1 d | 1 |
| 3 | DDR LLR queue (writer, reader, descriptors, drop counter, iteration cap), HP1 (after the IIO DMA removal), DT reserved-memory; Verilator AXI memory model; tests | 2.5-3 d | 2 |
| 4 | timing2 production (per-slot reset is in the prototype but untested; coefficient bank addressing; README) | 0.5 d | |
| 5 | framesync2 / carrier2 from the generators on post-PLS RTL; hand-review the context lists; per-slot reset of the shadow set; tests | 2.5-3 d | 1 |
| 6 | Second DDC (shared NCO ROM), synchronous input FIFOs, `dvbs2_rx2` core, `dvbs2_rx_axi` map v2, packet slot bit, snapshot slot select; `tb_rx_axi` with 2 signals | 2 d | 3-5 |
| 7 | BRAM recovery items as the build requires | 0.5-1 d | 6 |
| 8 | Software: `dvbs2-rx` per slot (`-S 0/1`), one DMA owner with demultiplexing (dvbs2-rx with two slots or plutosatd), regs TOML, README | 2 d | 6 |
| 9 | Full build, timing closure (pblocks if needed), board tests | 1-3 d | 7, 8 |
| | **Total** | **about 13-17 working days** | |

## 13. Decisions

Decided by the user (2026-11-04):

1. Total symbol rate at most **2.38 MBd** (the two-thread timing engine, about
   3.8 MBd, is not planned).
2. **DDR LLR queue**, 2 MB reserved (e.g. 0x3E900000), on its own **HP port
   (HP1)**. The AXI-port re-plan is in progress; the queue design does not
   depend on which HP port it gets.
3. **Shared LDPC iteration budget** (section 6.2).
4. **One DMA stream with a slot tag** (header word 0 bit 55), owned by one
   process that demultiplexes (dvbs2-rx with both slots, or plutosatd).
5. **Per-slot register banks** (0x000 slot 0, 0x100 slot 1, 0x200 shared).
6. **Frame sync and carrier** two-slot versions are generated **after the PLS
   decoder merge**.
7. The ADI FIR filters and IIO DMA paths are removed (frees LUT and DSP,
   2 BRAM tiles, HP1/HP2).
8. The carrier BRAM fix is done: merged into master (1349130).

Still open (only if needed during implementation): if timing or BRAM does
not close, the fallbacks are:

* the BRAM recovery items in section 8 (about 6 tiles);
* a slimmer BCH (Chien search 4 positions per clock instead of 8, an
  estimated -1..1.5k LUT);
* as a last resort, a separate "2-slot" bitstream without nbiq or without
  the FFT.

BRAM, with about 7 tiles of headroom, is the limit, not LUT or DSP.

## 14. Prototype files (branch proto/rx2, dvbs2/proto/rx2/)

* `rx2_design.md`: this document.
* `board/hier4.rpt`, `hier8.rpt`, `brams.txt`, `dsps.txt`, `util.rpt`:
  board reports from the routed DCP.
* `dvbs2/rtl/dvbs2_rx_timing2.sv`: the two-slot timing engine
  (hand-written).
* `dvbs2/rtl/dvbs2_rx_carrier.sv`: the carrier with seg/zbuf in BRAM (the
  original is `proto/dvbs2_rx_carrier.orig.sv`, patch
  `proto/patch_carrier_bram.py`).
* `dvbs2/rtl/dvbs2_rx_carrier2.sv` and `dvbs2_rx_framesync2.sv`: generated
  by `proto/make_carrier2.py` and `proto/make_fs2.py`.
* `dvbs2/sim/rx/test_rx2.py` + `tb_rx_timing2.sv`, `test_rx2_carrier.py` +
  `tb_rx_carrier2.sv`, and `test_rx2_framesync.py` + `tb_rx_framesync2.sv`:
  the per-slot equivalence tests (`cd dvbs2/sim/rx && ../../.venv/bin/python test_rx2.py`).
* `dvbs2/syn/ooc2.tcl` and `out2_*`: the OOC runs (env `RTLDIR`, `TAG`,
  `LUTF`; `rtl_orig/` holds the unmodified RTL for the baselines).
