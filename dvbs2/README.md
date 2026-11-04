# DVB-S2 transmitter and receiver (SystemVerilog)

QPSK, normal FECFRAMEs, all 11 code rates (1/4 to 9/10), pilots on or off per
frame, any Gold code, dummy PLFRAMEs when no data is queued, and polyphase RRC
interpolation to an integer multiple of the symbol rate (with a CIC behind it
for the low rates) followed by an NCO. Verified against GNU Radio gr-dtv.

```
BBFRAME bytes ─► BB scrambler ─► BCH ─► LDPC ─► FECFRAME FIFO ─► PL framer ─► RRC ×P ─► [CIC ×R] ─► NCO ─► I/Q
(+ {pilots, rate})   dvbs2_fec                  dvbs2_frame_fifo  dvbs2_plframer  dvbs2_rrc_interp  dvbs2_cic_interp  dvbs2_nco
```

| Path | Contents |
| --- | --- |
| `rtl/` | RTL; `dvbs2_tx.sv` is the top level |
| `rtl/gen/` | tables generated from gr-dtv: LDPC address ROM, BCH polynomials, NCO LUT |
| `model/` | `grref.py` (gr-dtv wrappers), `txmodel.py` (bit-exact model of the RTL), `gen_tables.py` |
| `sim/` | cocotb + Verilator testbenches |
| `syn/ooc.tcl` | out-of-context Vivado implementation for resource/timing numbers |
| `rtl/dvbs2_tx_axi.sv`, `ip/dvbs2_tx_bd.v` | block-design wrapper: AXI-Lite registers, DMA packet input, CDC + DAC selector |
| `sw/` | `dvbs2-tx` (TS over UDP/file -> BBFRAMEs -> DMA, AD9363 setup), `bbframe.c` |
| `rtl/dvbs2_rx*.sv` | receiver front end (timing, frame sync, carrier, soft demap); `dvbs2_rx.sv` is the top level |
| `model/rxfixed.py`, `model/rxfixed_experiment.py` | bit-exact model of the receiver RTL, end-to-end channel experiment |
| `model/rxmodel.py`, `model/rx_experiment.py` | floating-point receiver prototype |
| `sim/rx/` | file-driven Verilator testbenches for the receiver |

## Verification

```
python3 -m venv --system-site-packages .venv && .venv/bin/pip install cocotb==1.9.2 pytest
.venv/bin/python model/gen_tables.py                     # probe gr-dtv, write rtl/gen/*
cd model && ../.venv/bin/python -m pytest test_model.py  # model vs gr-dtv
cd ../sim && ./run.sh                                     # RTL vs gr-dtv and model
```

* `gen_tables.py` recovers the LDPC tables and BCH polynomials by probing
  gr-dtv's encoders, then cross-checks them on random data.
* `test_model.py`: FEC output of every rate and complete PLFRAMEs (header,
  pilots, PL scrambling, several Gold codes) equal gr-dtv; the fixed-point
  RRC is >60 dB SNR against GNU Radio's float filter.
* `test_fec.py`: the RTL FEC chain equals gr-dtv bit for bit for all 11
  rates, also with random input gaps and output backpressure.
* `test_tx.py`: RTL PLFRAME symbols equal gr-dtv; dummy frames and every
  output sample (RRC + CIC + NCO) equal the model, also at 33.33, 66.67 and
  125 kS/s (CIC) and 250 and 333.33 kS/s (RRC alone), on dummy-frame
  prefixes (a whole frame at 360 samples/symbol is too slow to simulate),
  and for the CIC at R = 32 with a full-scale input.
* `test_loopback.py`: RTL output samples -> gr-dvbs2rx (independent
  receiver) -> MPEG-TS; every packet equals the input, none missing. Once
  with the RRC alone (2 samples/symbol), once with RRC x2 + CIC x3.
* `test_tx_axi.py`: block-design wrapper: registers, DMA packet format
  (bad header / short / long packets), DAC-domain samples equal the model.
* `sw/test_bbframe.py`: the C BBFRAME builder equals gr-dtv's bbheader.

`dvbs2-rx` needs `--pl-acm-vcm` to follow dummy PLFRAMEs (in CCM mode it
assumes constant frame length), and more frames than the RTL simulation
produces to lock, so `test_loopback.py` plays whole data frames in a loop.

Note: gr-dtv 3.10's `dvbs2_physical_cc` outputs PLFRAMEs zero-stuffed to two
samples per symbol; the tests take every other sample.

## Interface

* Input: one BBFRAME per AXI-Stream packet, Kbch/8 bytes, MSB first.
  `s_user = {pilots, rate_idx}`, where rate_idx 0..10 is MODCOD 1..11. It is
  sampled on the first byte and can change every frame. A wrong `s_last`
  position is counted in `cnt_len_errors`.
* Output: 16-bit I/Q at P x R samples per symbol, pulled with `m_ready`.
* `sps` (RRC interpolation P): 2..64; `cic_r` (CIC interpolation R): 0 or 1
  = off, 2..32, with `cic_shift`. RRC coefficients go in through `coef_*`,
  phase-major (`addr = p*16 + j` holds `h[p + P*j]`);
  `model/txmodel.py:tx_config()` chooses P, R and generates them (see
  [Low symbol rates](#low-symbol-rates-cic-interpolator)).
* `gold_xa/gold_xb`: x-sequence states for Gold code n
  (`txmodel.gold_init(n)`). For n = 0 they are `PLS_XA_GOLD0/PLS_XB_GOLD0`.
* PlutoDVB2 (F5OEO) uses the ORI `dvb_fpga` VHDL modulator and changes the
  AD9363 rate per symbol rate. This core instead keeps the DAC at a fixed
  rate (12 MS/s in the design notes) and interpolates by an integer SPS.

## Low symbol rates: CIC interpolator

```
symbols ─► RRC ×P (16 taps/phase) ─► CIC3 ×R ─► NCO ─► 12 MS/s     samples/symbol = P x R
```

The DAC rate stays at 12 MS/s, so 33.33 kS/s needs 360 samples per symbol;
the RRC alone goes up to 64 (187.5 kS/s).

* `dvbs2_cic_interp` (CIC register, 0x2C): two combs at the RRC rate, the
  sample held for R outputs, two integrators at the output rate (the same
  as 3 combs, zero stuffing and 3 integrators), 26-bit wrap-around
  arithmetic (exact up to R = 32), output sat16(round(. / 2^shift)),
  shift = floor(log2 R^2). R = 0 or 1 (the reset value): the RRC output
  passes unchanged, so old software and coefficients give the same samples
  as before.
* Taps (`model/txmodel.py:rrc_coefs(P, ..., cic_r=R)`): the RRC at P
  samples/symbol convolved with [-a, 1 + 2a, -a], a = 3 (1 - 1/R^2) / 24,
  the second-order inverse of the CIC droop, and scaled by 2^shift / R^2
  (<= 1) against the CIC gain, so the output level is the same at every
  rate.
* Choice (`txmodel.tx_split()` / `tx_config()`, the same in
  `sw/dvbs2_tx.c`): up to 64 samples/symbol the RRC alone; above, the
  largest P <= 64 with R = sps / P in 2..32. That gives P >= 11 for every
  supported value (smaller P raises the CIC images: P = 4 gives -53 dB, P
  = 3 -44 dB). Supported: 2..64 and 753 of 65..2048, e.g. 96 (48 x 2), 180
  (60 x 3), 360 (60 x 6), 720, 2048 (64 x 32); not primes above 64 and
  e.g. 134 or 202. `dvbs2-tx --dump-coefs -s SR` prints P, R, the shift
  and the taps.

Measured on the bit-exact model (3000 random QPSK symbols, NCO off). MER:
ideal float RRC matched filter (64 symbols) at the symbol instants. PSD:
Welch, Blackman-Harris, resolution Rs/64, relative to the in-band PSD;
ACLR: power in [Rs, 2 Rs] (a neighbour at 1.5 Rs) over the wanted signal.
Roll-off 0.35 / 0.25 / 0.20:

| symbol rate | sps = P x R | MER (dB) | PSD at 0.75 Rs | max beyond Rs | max beyond 2 Rs | ACLR | CIC images |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 2 MS/s | 6 | 55.3 / 60.4 / 45.4 | -48.2 / -48.7 / -46.8 | -55.9 / -52.8 / -54.0 | -64.1 / -60.1 / -64.7 | -62.0 / -58.7 / -61.1 | - |
| 1 MS/s | 12 | 55.3 / 60.4 / 45.5 | -48.2 / -48.7 / -46.7 | -56.0 / -53.1 / -53.9 | -65.5 / -61.4 / -65.5 | -62.4 / -59.2 / -61.1 | - |
| 333.33 kS/s | 36 | 55.3 / 60.4 / 45.5 | -48.2 / -48.8 / -46.7 | -56.1 / -53.1 / -53.9 | -65.9 / -61.8 / -65.5 | -62.4 / -59.3 / -61.2 | - |
| 250 kS/s | 48 | 55.3 / 60.4 / 45.5 | -48.2 / -48.8 / -46.7 | -56.1 / -53.1 / -53.9 | -66.0 / -61.9 / -65.6 | -62.5 / -59.3 / -61.2 | - |
| 125 kS/s | 96 = 48 x 2 | 55.3 / 60.4 / 45.5 | -48.3 / -48.8 / -46.7 | -56.1 / -53.1 / -53.9 | -66.0 / -61.9 / -65.6 | -62.5 / -59.3 / -61.2 | -93.4 / -89.4 / -93.8 |
| 66.67 kS/s | 180 = 60 x 3 | 55.3 / 60.4 / 45.5 | -48.3 / -48.8 / -46.7 | -56.1 / -53.1 / -54.0 | -66.0 / -61.9 / -65.6 | -62.5 / -59.3 / -61.2 | -95.9 / -92.5 / -96.5 |
| 33.33 kS/s | 360 = 60 x 6 | 55.3 / 60.4 / 45.5 | -48.2 / -48.8 / -46.7 | -56.1 / -53.1 / -53.9 | -66.0 / -61.9 / -65.6 | -62.5 / -59.3 / -61.2 | -96.9 / -93.5 / -97.4 |

The low rates match the high ones to 0.1 dB: both are set by the 16-symbol
RRC (the MER by its truncation; the receiver works far below 45 dB, so its
MER does not change). The CIC images (the largest PSD beyond P/2 x Rs) are
>= 89 dB down, below the RRC's own floor (-75 dB beyond 5 Rs). Without
the droop compensation the MER would drop to 49 dB at P = 12 (0.1 dB at P
= 60). With an NCO offset the far floor is higher at every rate (-59 dB
beyond 2 Rs instead of -66 dB, at 333 kS/s and 33 kS/s alike): the NCO's
10-bit phase (-59.4 dB) and the per-axis clipping of the rotated 0.8 full
scale peak (0.25 % of the samples, -60.5 dB).

Resources (`syn/ooc.tcl 100 dvbs2_tx`, xc7z020-2): before 1024 LUTs, 1196
FFs, 13.5 BRAM tiles, 16 DSPs, WNS +3.21 ns; now 1368 LUTs, 1520 FFs,
13.5 BRAM, 16 DSPs, WNS +3.18 ns (the LDPC encoder's path, as before). The
CIC is 277 LUTs and 324 FFs; the RRC's output FIFO read mux adds ~66 LUTs
because it now feeds the comb subtractors. The alternative, the RRC alone
up to 512 samples/symbol, costs few LUTs but 2 more BRAM tiles (the
coefficient banks grow from 4 RAMB18 to 4 RAMB36); BRAM is the scarcest
resource of the full design (92 %).

Throughput: at 33.33 kS/s a PLFRAME with pilots lasts 1.0 s (without
pilots 0.98 s, a dummy frame 0.1 s); the FEC and framer idle and the DMA
waits on the core's backpressure for up to a frame. `dvbs2-tx` therefore
keeps at most max(2, 0.5 s of air time) BBFRAMEs submitted but not yet on
air (from DATA_FRAMES), sizes the UDP receive buffer to about 1 s of
channel capacity (8 kB .. 4 MB) so a too fast TS is dropped rather than
queued for minutes, and at the end of a file waits until every frame has
been sent. At high rates nothing changes (the DMA queue limits first).

## Fishball bitstream integration

`boards/fishball/hdl/system_bd.tcl` adds `dvbs2_tx` (module reference) at
0x7C500000 and its own `axi_dmac` (memory -> stream, 32-bit on S_AXI_GP1)
at 0x7C480000, IRQ ps-10. The core's DAC selector feeds `axi_ad9361`
channel 0; CTRL[2] switches it between DVB-S2 and `dac_in_i/q` (zero for now:
the input for a future IQ transmitter, see `boards/fishball/README.md`).
The core runs on `sys_cpu_clk` (100 MHz) and crosses to `l_clk` with a Gray
FIFO pulled by `dac_valid`.

DMA packet = one BBFRAME: header word `0xB500_0000 | pilots<<4 | rate_idx`,
then Kbch/8 bytes, padded to 8 bytes.

Linux: the device tree exports registers, DMA and a 16 MB reserved buffer
(0x3F000000) as UIO device `dvbs2-tx`; `dvbs2-tx` (buildroot package from
`boards/fishball/br2`) drives it from user space and sets the AD9363 sample
rate/LO/attenuation and DAC data source (`DAC_DDS_SEL = 2`) through IIO:

```
dvbs2-tx -s 500000 -m qpsk1/2 -p -f 2403.75e6 -a 10 -u 1234   # TS on UDP port 1234
```

## Receiver

```
ADC ─► DDC ─► timing ─► frame sync ─┬─► carrier + demap ─► LDPC ─► BCH ─► BBFRAME ─► DMA packet
                                    └─► PLS decoder (status only)
       dvbs2_rx_ddc  dvbs2_rx_timing  dvbs2_rx_framesync  dvbs2_rx_carrier  dvbs2_ldpc_dec  dvbs2_bch_dec
                                      dvbs2_rx_pls
       \_________________________________ dvbs2_rx (core) _________________________________/
       \_____________________________ dvbs2_rx_axi (registers, packets) _____________________/
```

| Block | Algorithm |
| --- | --- |
| `dvbs2_rx_timing` | 32-phase polyphase RRC matched filter (25 taps/phase, loadable), Gardner TED, PI loop; AGC on the filter output (target power 2^25) |
| `dvbs2_rx_framesync` | differential correlation with the 89 header differentials of the data MODCOD and of the dummy header (any carrier offset); search: the largest metric, checked at the dummy length first (the type is unreliable at low SNR) and then, for a data candidate, at the data length; lock after 2 checks in a row (each > 4x, both together > 10x the mean metric, i.e. the noise level: at -2 dB a header is only ~8x, the largest noise metric of a frame ~7x); locked: +-2 symbol window, but the predicted position unless beaten 2:1, threshold max(register, 1/4 of the average peak), flywheel; the next header is predicted from the carrier's frame type |
| `dvbs2_rx_ddc` | NCO, optional 3-stage CIC (decimation R = 2..32), decimating FIR (decimation D = 1..63, up to 512 loadable taps, 2 taps per clock): ADC rate -> 2 samples/symbol, total decimation R x D; see [DDC](#ddc) |
| `dvbs2_ldpc_dec` | layered offset min-sum (offset 1 LSB), 18 lanes over the 360x360 circulants, APP 9 bit, messages 6 bit; phase A reads a check's edges, phase B writes L += R_new - R_old (circulants of one column group sharing a layer, which the DVB-S2 tables have, update in turn); stops when an iteration has no unsatisfied check and no APP sign change; ping-pong APP memories; output the Kldpc hard decisions and the channel's hard decisions of the same bits |
| `dvbs2_bch_dec` | GF(2^16), t = 12/10/8: odd syndromes 8 bits per clock while the frame comes in, simplified inversionless Berlekamp-Massey, Chien search 8 positions per clock writing the corrected, BB-descrambled bits back into the frame buffer; ok = deg Lambda roots found; counts the pre-FEC bit errors (channel decisions vs. decoder output) |
| `dvbs2_rx_pls` | PLS code (MODCOD, frame size, pilots) of every tagged PLHEADER, whatever MODCOD/pilots are configured: maximum likelihood over the 128 codes for an unknown phase, per frequency hypothesis (the carrier's frequency, then a grid); see [PLS decoder](#pls-decoder) |
| `dvbs2_rx_carrier` | PL descrambling; frequency acquisition from 4 headers each confirmed by the next one being detected exactly a frame length later (staged Kay / Luise-Reggiannini with 1, 6 and 45 lags, correlations summed over the 4), re-acquisition if 2 of the next 3 or 8 later headers in a row are far off; then the NCO frequency from every PLHEADER (gain 1/4 .. 1/8); frame type (data/dummy) decided coherently from both headers and reported to the frame sync; pilots on: CORDIC phase per header/pilot block, linear phase interpolation across each data segment, pilot-to-pilot frequency loop (gain 1/4), dummy frame bodies as 36 known blocks (block-to-block frequency loop, no 1/1476 ambiguity), and the 1/1476 false lock of the pilot loop corrected from the averaged header estimates; pilots off: decision-directed PLL whose phase is reset from every header; amplitude/noise from the known blocks; max-log LLRs, 4 LSBs per unit |

`model/rxfixed.py`, `model/ldpcdec.py` and `model/bchdec.py` are the bit-exact models of this
RTL (`model/rxmodel.py` is the earlier floating-point prototype); they
generate `rtl/gen/dvbs2_rx_pkg.sv` (CORDIC table, estimator constants,
default gains), `rtl/gen/dvbs2_ldpc_dec_pkg.sv` and `rtl/gen/ldpc_dec_rom.mem`
(edges of every layer of every rate), `rtl/gen/dvbs2_bch_pkg.sv`:

```
cd model && ../.venv/bin/python rxfixed.py                  # rtl/gen/dvbs2_rx_pkg.sv
../.venv/bin/python ldpcdec.py                              # LDPC decoder tables
../.venv/bin/python bchdec.py                               # BCH decoder constants
../.venv/bin/python rxfixed_experiment.py                   # model + gr-dvbs2rx FEC, 7 channels
cd ../sim/rx && ../../.venv/bin/python -m pytest test_rx.py # RTL vs model (Verilator, file driven)
```

* `rxfixed_experiment.py`: TX model -> channel (carrier offset up to 0.01
  cycles/symbol, +-100 ppm clock, fractional delay, AWGN, int16) -> fixed
  model -> gr-dvbs2rx LDPC/BCH. All data frames decode at QPSK 1/4 -1 dB,
  1/2 2 dB (pilots) / 2.5 dB (no pilots), 4/5 6 dB, 9/10 8 dB Es/N0.
  QPSK 1/4 with pilots, 0.004 cycles/symbol, 60 ppm, the model + our LDPC
  decoder (96 seeds x 6 frames after 30 dummy frames; ideal sync + the same
  decoder in brackets): FER 31 % (7.8 %) at -2.4 dB, 0.2 % (0.2 %) at
  -2.2 dB, 0.2 % (0 %) at -2.0 dB. Start-up: SEARCH + 2 checks + 4
  confirmed headers, ~6-7 frames.
* `ldpcdec.compare()`: the decoder model decodes the same frames as
  gr-dvbs2rx's decoder at QPSK 1/4 -2.0 dB, 1/2 1.0 dB, 3/4 4.1 dB and 9/10
  6.6 dB (AWGN, 30 iterations max).
* `sim/rx/test_rx.py`: each block and the whole core equal the models word
  for word (the LDPC decoder also in iteration count and convergence flag), with random input/output stalls, low input level, dropped
  symbols (flywheel, loss of lock, re-acquisition, frames cut short and
  padded) and segment timeouts. End to end, samples -> RTL -> BBFRAMEs
  equal the transmitted BBFRAMEs (and gr-dvbs2rx decodes the RTL's LLRs to
  the same frames).
* The testbenches read and write files and run under `verilator --binary`
  (the RX needs millions of clocks per frame; cocotb's per-clock Python
  was too slow).

Throughput at 100 MHz: the timing block needs 42 clocks per symbol (2.4
MBd); the carrier block stalls for ~1000 clocks per header and while sending
a pilot-mode data segment, which the 512-sample input FIFO absorbs; the LDPC
decoder needs ~53k clocks per iteration (rate 1/2; all rates are 30-55k),
so at 2 MBd about 30 iterations fit in a frame time.

BCH matters where LDPC leaves a few errors: with few iterations (high symbol
rate), and the LDPC error floor; near the LDPC threshold the residual errors
of a failed frame are in the hundreds. The BCH decoder adds ~n/8 + Kbch/4 +
500 clocks per frame after its input, during which the LDPC output waits.
`dvbs2-rx` keeps the frames that are BCH codewords.

### PLS decoder

The receive path is told the MODCOD and pilots (MODCOD register); the PLS
decoder reports what the PLHEADERs say, so the software can detect them
(and changes of them) instead of trying hypotheses. It taps the carrier's
input (symbols + the frame sync's header tags) and decodes every tagged
header, data or dummy, in its own time (the next header is >= 3330
symbols away); nothing in the receive path depends on it.

* Metric: the 90 header symbols (6 bit) are multiplied by the conjugate of
  the PLS-code-0 header (SOF, PLS scrambling, pi/2), rotated by -2 pi f k
  (64-phase table), and correlated with the 128 codewords: SOF sum S, then
  16 passes over the 64 PLS symbols with two accumulators (pilots off/on)
  and the two 16-pair halves (the MODCOD LSB row), T = S +- (P1 +- P2) (the
  short-frame row flips all PLS symbols), M = |T|^2. That is the maximum
  likelihood decision for an unknown phase, if the frequency is known to
  about +-0.002 cycles/symbol over the header.
* Frequency hypotheses: the carrier's frequency (once acquired) first; if it
  gives the configured PLS code or the dummy code with a metric above the
  noise level, the decode ends there (normal operation: one hypothesis).
  Otherwise a grid h x step, h = -H..H around the DDC frequency (PLS_CFG,
  default 32 x 0.004: +-0.128 cycles/symbol, the carrier's acquisition
  range). The grid is what makes a detection with the wrong configuration
  work: the carrier's frequency acquisition uses the configured header and,
  with the wrong MODCOD or pilots, acquires a wrong frequency (in the model:
  +0.008 instead of +0.004, -0.127 instead of 0 cycles/symbol). The best
  grid frequency is reported too (PLS_FREQ, +-0.002 cycles/symbol).
* Confident: best > 2 x the best metric of another code at the same
  frequency, and best > 2^8 (with the AGC a header at -3 dB Es/N0 gives >=
  ~300, noise alone <= ~220). The counters and the seen mask take
  confident decodes only.
* Error rate (`model/rxfixed.py` pls_decode(), random codes and phases,
  AWGN, 10^5 headers per point; "carrier": the carrier's frequency within
  ~2e-4 cycles/symbol and the configured code; "grid": not acquired, the
  frequency uniform over the grid; errors, most of them confident):

  | Es/N0 | carrier | grid H = 32 (+-0.128) | H = 8 (+-0.032) | H = 16, step 0.002 |
  | --- | --- | --- | --- | --- |
  | -3 dB | 5e-5 | 9.0e-4 | | |
  | -2.4 dB | 1e-5 | 3.3e-4 | 2.2e-4 | 1.6e-4 |
  | -2 dB | 1e-5 | 2.1e-4 | | |
  | -1 dB | 0 | 2e-5 | | |
  | 0, 1, 3 dB | 0 | 0 | | |

  With the carrier's frequency the few errors come from the grid fallback
  (the carrier hypothesis below the noise level); a known frequency alone
  (one hypothesis, H = 0) gave no error at -2.4 dB. Not confident although
  right: 0.6 % (carrier) and 0.9 % (grid) of the headers at -2.4 dB, 1.5 %
  and 2 % at -3 dB. Noise alone: no confident decode in 10^5.

  A non-coherent alternative (correlating the 89 header differentials, as
  the frame sync does: `model/rxmodel.py` pls_decode_differential()) is
  useless here: 54 % code errors at -2.4 dB, 9 % at 3 dB, because codes whose
  (64,7) codewords differ only in the MODCOD LSB row (the 16-pair halves) or
  the short-frame row differ in only one or two differentials. The grid is
  the price of coherence without a frequency.
* A wrong code at a wrong grid frequency can beat the right one at low SNR
  and then looks confident: a detection should want a few equal decodes
  in a row (`dvbs2-rx --detect`: 3).
* The frame sync must still find the headers: with the right pilots it
  locks with any MODCOD (its correlation keeps the SOF and the pilot-bit
  differentials); with the wrong pilots it predicts the wrong frame length
  and finds data headers only next to dummy frames (with dummy frames the
  first data header is decoded right after them). Without dummy frames the
  software has to try pilots off and on: 2 hypotheses instead of 22.
* Time: ~1420 clocks per hypothesis (rotation 4 clocks per symbol on one
  DSP, 16 passes x 64 clocks, the metrics on a second DSP during the next
  pass), 94k clocks for 2H + 2 = 66 hypotheses: one dummy frame at 2.38
  MBd. A header that starts while a decode runs is skipped (PLS_SKIPPED).
* Resources (out of context, 100 MHz): 607 LUTs (50 of them distributed
  RAM: the 90 stored symbols and the 64 rotated ones), 828 FFs, 2 DSPs, no
  block RAM, WNS +3.6 ns. The status logic (counters, last decodes, seen
  mask) is ~70 of the LUTs.
* Tests (`sim/rx/test_rx.py`): `test_pls` (RTL = model on the model chain's
  carrier input: TX QPSK 3/4 pilots with the RX told QPSK 1/2 without
  pilots, a different MODCOD at 0.03 cycles/symbol without dummy frames,
  -2.4 dB matched (carrier hypothesis), -2 dB mismatched (grid), symbols
  too fast (skipped headers)); `test_rx_end_to_end` (every decode equals the
  model, the confident ones are the right codes); `test_rx_axi*` (the
  registers).

### DDC

```
ADC 12 MS/s ─► NCO ─► [CIC3 ÷R] ─► FIR ÷D (16-bit buffer -> 18-bit) ─► 2 samples/symbol, R x D = 6 MS/s / symbol rate
```

* NCO: 32-bit phase, the TX NCO's 1024-entry table, I and Q through two
  multipliers in consecutive clocks; bypassed (exactly) with DDC_FINC = 0.
* CIC (DDC_CIC, 0x4C): 3 integrators at the input rate, decimation R, 3
  combs, 32-bit arithmetic (exact up to R = 32), output
  sat18(round(. / 2^cic_shift)); `cic_shift` = ceil(log2 R^3) - 5 gives a
  gain of 16..32, so a 12-bit ADC sample cannot saturate.
* FIR: the old engine (2 taps per clock, 1024-sample buffer, now 18-bit
  samples in two 512 x 36 RAMs for the even and odd samples), output
  sat16(round(. / 2^shift)), rounding constant preloaded into the
  accumulator.
* Input: at most one sample per 2 clocks and clk x R x D / (taps/2).
* Choice (`model/rxfixed.py:ddc_config()`, the same in `sw/dvbs2_rx.c`):
  the CIC with the smallest R for which D = decimation / R <= 32, if D >= 8
  (or, above 32, D >= 4); then 16 FIR taps per step, Kaiser beta 8, cutoff
  at the symbol rate, CIC droop compensated (ideal response 1 + c f^2),
  scaled to the largest shift that keeps the taps below 2^17. Otherwise
  (decimations up to 32 without such a split) the FIR alone with the old
  design (12 taps per step, beta 5.65). Supported: 1..36, and the larger
  decimations (up to 1024) with such a split, e.g. 48, 90, 180, 240, 360,
  1024; not e.g. 37, 41, 74.

At 12 MS/s (passband ripple over the signal band (1 + roll-off) / 2 x
symbol rate; alias rejection = the smallest attenuation of anything that
folds into that band at 2 samples/symbol, roll-off 0.35 / 0.25 / 0.2):

| symbol rate | decimation | split R x D, taps | ripple | alias rejection |
| --- | --- | --- | --- | --- |
| 33.33 kS/s | 180 | 6 x 30, 480 | 0.002 dB | 81.2 / 81.2 / 81.2 dB |
| 66.67 kS/s | 90 | 3 x 30, 480 | 0.002 dB | 81.1 / 81.1 / 81.1 dB |
| 125 kS/s | 48 | 2 x 24, 384 | 0.002 dB | 81.3 / 81.5 / 81.5 dB |
| 250 kS/s | 24 | 2 x 12, 192 | 0.002 dB | 81.3 / 81.6 / 81.6 dB (FIR alone: 58.1 / 62.3 / 62.8) |
| 333.33 kS/s | 18 | 2 x 9, 144 | 0.002 dB | 73.8 / 75.8 / 76.8 dB (FIR alone: 58.3 / 62.8 / 62.8) |
| 500 kS/s | 12 | FIR, 144 | 0.012 dB | 58.4 / 62.5 / 62.6 dB |
| 1 MS/s | 6 | FIR, 72 | 0.012 dB | 58.5 / 61.6 / 63.2 dB |
| 1.5 MS/s | 4 | FIR, 48 | 0.013 dB | 58.3 / 60.9 / 62.7 dB |
| 2 MS/s | 3 | FIR, 36 | 0.013 dB | 59.0 / 61.1 / 62.8 dB |

Against an adjacent signal of power spectral density P dB above the
wanted one, the aliases stay 15 dB below the noise at Es/N0 = -2 dB while
P < rejection - 13 dB (45 dB with the FIR alone, 68 dB with the CIC).

Before (FIR alone, decimation 1..32) the DDC used 889 LUTs, 457 FFs, 4
BRAM tiles and 16 DSPs; now 640 LUTs, 1039 FFs, 2.5 BRAM tiles and 14 DSPs
(out of context, 100 MHz, WNS +4.0 ns instead of +0.3 ns).

Carrier acquisition (`model/rxfixed.py` receive(), CFO sweep through the
DDC, QPSK 1/2 pilots 3 dB, QPSK 1/4 pilots -1 dB, QPSK 1/2 without pilots
3 dB, 2 seeds): every frame decoded up to +-0.13 cycles/symbol, most up to
0.16, none at 0.2 and beyond. The range scales with the symbol rate: +-4.3
kHz at 33.33 kS/s, +-8.7 kHz at 66.67 kS/s, +-16 kHz at 125 kS/s, +-65 kHz
at 500 kS/s, +-260 kHz at 2 MS/s. Acquisition takes ~6-7 frames (SEARCH, 2
checks, 4 confirmed headers): ~7 s of data frames at 33.33 kS/s (a frame
with pilots lasts 1 s; dummy frames, 0.1 s, are faster). A larger LNB
offset needs a search by the software (DDC_FINC steps of ~0.2 x symbol
rate, ~10 s per step at 33.33 kS/s) or an offset known in advance: from
the QO-100 beacon (1.5 MS/s, acquires within +-195 kHz in ~0.2 s; its
FREQ register gives the LNB offset to a few Hz) or from the spectrum.

### Symbol-rate offset

At 12 MS/s the receiver's nominal rate is 6 MS/s / decimation, while
QO-100 stations transmit integer-kS rates: 333000 S/s is decimation 18
(333333 S/s) with the transmitter at e = -1000 ppm, 66000 and 33000 S/s
are decimation 91 and 182 (65934, 32967 S/s) at e = +1000 ppm.
`model/sr_offset_experiment.py` measures this: TX model, windowed-sinc
resampler with the rate error e, 0.004 cycles/symbol carrier offset, AWGN
over the ADC band, 12-bit ADC, the DDC as dvbs2-rx loads it, the receiver
model, gr-dvbs2rx FEC; QPSK 1/2, Es/N0 2 dB, data frames from the first
symbol on.

The Gardner loop (Bn T = 0.002) follows up to ~+-850 ppm without slipping
(2 zeta omega_n / 2 pi); a larger offset is pulled in by slipping symbols
first (12-45 at +-1000 ppm), which takes a time growing with e^2. The
integrator update rounds down (`>> 16`, -0.5 LSB per symbol), which the
loop balances with a small static timing error: the integrator settles
~187 below the expected 2 (nominal / actual - 1) x 2^24 at every offset,
and the pull-in is slower for e < 0 (transmitter slower than nominal).

| e (ppm) | pull-in, symbols (median) | data frames lost before the first decoded one (median of 12 runs, decimation 3) | after it (200 frames at decimation 3 and 18, 100 at 91 and 182; pilots on and off) |
| --- | --- | --- | --- |
| 0, +-200 | 4k | 5 | none, apart from single LDPC failures without pilots (also at e = 0) |
| +-500 | 7-11k | 5-6 | the same |
| -1000 / +1000 | 28k / 19k | 7 / 7 | the same, no slips |
| -2000 / +2000 | 120k / 50k | 10 / 8 (5 / 1 of 12 runs not locked within 12 frames) | the same |
| -5000 | none in 200 frames | - | no frame decoded |
| +5000 | 210-440k | 14-23 | the same |

So +-1000 ppm is tracked (integrator standard deviation ~1040 LSB, 31
ppm) and costs 1-2 frames of acquisition (1-2 s at 33 kS/s). A known
offset is removed by the integrator's start value: TIMING_INTEG (0x5C)
written before the soft reset is loaded at the reset
(`rxfixed.timing_integ_preset()`; 333000 S/s at decimation 18: 33588).
With it acquisition is the same as at e = 0 (median 5 frames) for every
tested offset up to +-2 % (`sr_offset_experiment.py --preset`).

### Fishball bitstream integration

`boards/fishball/hdl/system_bd.tcl` adds `dvbs2_rx` (module reference
`ip/dvbs2_rx_bd.v`) at 0x7C510000, fed from `axi_ad9361` channel 0 (through
`rx_awgn`), and `axi_dvbs2_rx_dma` (stream -> memory, 32-bit on S_AXI_GP0)
at 0x7C490000, IRQ ps-9. Everything after the ADC sample FIFO runs on
`sys_cpu_clk`. Register map and packet format: header of
`rtl/dvbs2_rx_axi.sv`. Each decoded BBFRAME becomes one 8 KB DMA packet
(status word + BBFRAME); with both packet slots full a frame is dropped and
counted instead of stalling the decoder.

`sim/rx/test_rx.py::test_rx_axi` runs the whole wrapper: ADC-rate samples
(6 per symbol, 48 MHz ADC clock) through registers written over AXI-Lite,
packets taken with random backpressure, every packet equal to the model
chain and to the transmitted BBFRAME. `test_rx_axi_lowrate` does the same
at 66.67 and 33.33 kS/s (12 MS/s, decimation 90 and 180 through the CIC,
the signal 0.8 / 1.5 MHz off the LO removed by the NCO; 33.33 kS/s with a
CW 30 dB above the signal in an alias band); `test_ddc` checks the DDC
alone (FIR alone and CIC, several R and D, NCO on/off, input at the
maximum rate, saturation of the CIC and FIR outputs) and
`test_ddc_sw_config` that `dvbs2-rx` chooses the model's configuration for
every decimation. `test_rx_axi_sr_offset` runs decimation 18 with the
transmitter at 333000 S/s (-1000 ppm, TIMING_INTEG preset) and at +1000 ppm
(pulled in from 0).

Linux: UIO device `dvbs2-rx` (registers, DMA, 1 MB buffer at 0x3EF00000)
and `dvbs2-rx` in the same buildroot package as `dvbs2-tx`:

```
dvbs2-rx -s 500000 -m qpsk1/2 -p -f 739.75e6 -u 192.168.2.10:1234   # QO-100 10489.75 MHz, 9750 MHz LNB
dvbs2-rx -s 33333 -m qpsk1/2 -p -f 748e6 -o 250e3                    # 33.33 kS/s, 250 kHz off the LO
```

It sets the AD9363 RX sample rate to 12 MS/s (the system's fixed rate;
the symbol rate must be 6 MS/s / a supported decimation, within 100 ppm:
33333 S/s means 12e6 / 360), or `-F` Hz, or with `-F auto` the lowest
rate >= 2.1 MS/s with a supported decimation, or 2 x symbol rate x N with
`-d N`; it loads the DDC (DDC_CFG, DDC_CIC, coefficients) and RRC
coefficients (the same as the model's, `sw/test_bbdeframe.py::test_rx_coefs`,
`sim/rx/test_rx.py::test_ddc_sw_config`), refuses a CIC configuration on a
core older than VERSION 0x0001_0100, keeps four DMA transfers queued and
turns the BBFRAMEs back into TS (`sw/bbdeframe.c`, checked against
gr-dtv's BBFRAMEs and with lost frames). A status line shows the lock state,
carrier offset, Es/N0 estimate, pre-FEC and post-LDPC bit error rates
(RAW_ERR / RAW_BITS / BCH_BITERR, over BCH-good frames) and frame/packet
counters; `-C FILE` writes a 1024-symbol constellation snapshot (carrier
output, or the timing loop output with `--constellation-raw`) every second.
With a core >= VERSION 0x0001_0200 the status line ends with the last
confident PLS decode of a data header (`, PLS qpsk3/4 pilots`, `!` when it
is not the configured MODCOD/pilots, `, PLS -` before the first).

`--detect` detects the MODCOD and pilots instead of receiving: it
configures the core as usual (any `-m`/`-p`), waits for `--detect-count`
(3) equal confident PLS decodes of data headers in a row, prints them
(stdout: `MODCOD name pilots short offset_Hz`, the offset being the
signal's frequency from the LO: `-o` plus the carrier's or the PLS
decoder's estimate) and exits 0, or exits 1 after `--detect-timeout` (30
s). The frame sync needs the right pilots (or dummy frames): without
dummy frames, try `-p` and no `-p`. `--pls-grid H` sets the grid (default
32; 2H + 2 hypotheses must fit in a dummy frame).

```
dvbs2-rx -s 333333 -f 739.75e6 --detect      # e.g. "7 qpsk3/4 1 0 -1250"
dvbs2-rx -s 333333 -f 739.75e6 -p --detect   # if the first one times out
```

## Loopback self-test on the board

`dvbs2-selftest` (package `selftest`, source `awgn/`) runs a PRBS transport
stream through `dvbs2-tx`, an AD9363 loopback and `dvbs2-rx`, with the PL
noise source in front of the receiver setting Es/N0, and prints the packet
error rate per point. `-L digital` uses the AD9363 data-port loopback
(nothing transmitted), `-L rf` an external TX1 → RX1 cable. See
[awgn/README.md](../awgn/README.md).
