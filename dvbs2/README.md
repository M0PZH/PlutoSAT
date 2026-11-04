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
                                    └─► PLS decoder (status; header confirmation for the frame sync)
       dvbs2_rx_ddc  dvbs2_rx_timing  dvbs2_rx_framesync  dvbs2_rx_carrier  dvbs2_ldpc_dec  dvbs2_bch_dec
                                      dvbs2_rx_pls
       \_________________________________ dvbs2_rx (core) _________________________________/
       \_____________________________ dvbs2_rx_axi (registers, packets) _____________________/
```

| Block | Algorithm |
| --- | --- |
| `dvbs2_rx_timing` | 32-phase polyphase RRC matched filter (25 taps/phase, loadable), Gardner TED, PI loop; AGC on the filter output (target power 2^25) |
| `dvbs2_rx_framesync` | differential correlation with the 89 header differentials of the data MODCOD and of the dummy header (any carrier offset); search: the largest metric, checked at the dummy length first and then at the data length, whichever candidate gave it (the type is unreliable at low SNR, and QPSK 1/4..2/5 headers are closer to the dummy header than to the QPSK 1/2 one); lock after 2 checks in a row (each > 4x, both together > 10x the mean metric, i.e. the noise level: at -2 dB a header is only ~8x, the largest noise metric of a frame ~7x); locked: +-2 symbol window, but the predicted position unless beaten 2:1, threshold max(register, 1/4 of the average peak), flywheel; the next header is predicted from the carrier's frame type, or from the PLS decode of the header (below); header confirmation: 3072 symbols after each tag the PLS decode of that header must be confident and the configured code or a dummy header, else CHECK -> SEARCH, LOCKED -> SEARCH after 3 such tags in a row; see [False lock on repeating frames](#false-lock-on-repeating-frames) |
| `dvbs2_rx_ddc` | NCO, optional 3-stage CIC (decimation R = 2..32), decimating FIR (decimation D = 1..63, up to 512 loadable taps, 2 taps per clock): ADC rate -> 2 samples/symbol, total decimation R x D; see [DDC](#ddc) |
| `dvbs2_ldpc_dec` | layered offset min-sum (offset 1 LSB), 18 lanes over the 360x360 circulants, APP 9 bit, messages 6 bit; phase A reads a check's edges, phase B writes L += R_new - R_old (circulants of one column group sharing a layer, which the DVB-S2 tables have, update in turn); stops when an iteration has no unsatisfied check and no APP sign change; ping-pong APP memories; output the Kldpc hard decisions and the channel's hard decisions of the same bits |
| `dvbs2_bch_dec` | GF(2^16), t = 12/10/8: odd syndromes 8 bits per clock while the frame comes in, simplified inversionless Berlekamp-Massey, Chien search 8 positions per clock writing the corrected, BB-descrambled bits back into the frame buffer; ok = deg Lambda roots found; counts the pre-FEC bit errors (channel decisions vs. decoder output) |
| `dvbs2_rx_pls` | PLS code (MODCOD, frame size, pilots) of every tagged PLHEADER, whatever MODCOD/pilots are configured: maximum likelihood over the 128 codes for an unknown phase, per frequency hypothesis (the carrier's frequency, then a grid); see [PLS decoder](#pls-decoder) |
| `dvbs2_rx_carrier` | PL descrambling; frequency acquisition from 4 headers each confirmed by the next one being detected exactly a data or dummy frame length later, which also gives the header's type (staged Kay / Luise-Reggiannini with 1, 6 and 45 lags, correlations summed over the 4), re-acquisition if 2 of the next 3 (pilots on) or 8 later headers in a row are far off; then the NCO frequency from every PLHEADER (gain 1/4 .. 1/8); frame type (data/dummy) decided coherently from both headers and reported to the frame sync; pilots on: CORDIC phase per header/pilot block, linear phase interpolation across each data segment, pilot-to-pilot frequency loop (gain 1/4), dummy frame bodies as 36 known blocks (block-to-block frequency loop, no 1/1476 ambiguity), and the 1/1476 false lock of the pilot loop corrected from the averaged header estimates; pilots off: decision-directed PLL (soft decisions, data-aided on headers and dummy bodies) whose phase is reset from every header, its integral gain narrowing to 1/4 and 1/16 two and four headers after the acquisition, the NCO frequency from its integrator (header estimates not used after the acquisition), see [Pilots off](#pilots-off); amplitude/noise from the known blocks; max-log LLRs, 4 LSBs per unit |

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
../.venv/bin/python pilotless_experiment.py --pilots 0,1    # carrier FER vs Es/N0, phase noise
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

### Pilots off

Without pilots the carrier phase between two PLHEADERs (32490 symbols)
comes from the decision-directed PLL alone, and one 90-degree slip loses
the frame. Up to VERSION 0x0001_0200 pilots off lost ~3 dB at QPSK 1/4
(board, digital loopback, 1 MS/s: PER 0.13 at 0 dB, 0.63 at -1 dB; pilots
on: 0 at -1 dB). The causes, found with `model/pilotless_experiment.py`:

* Each header's frequency estimate (90 symbols: ~5e-4 cycles/symbol rms at
  0 dB) moved the NCO by f/8 at every header, while the PLL integrator,
  which had the frequency, was cleared into the NCO: every frame began
  with a new ~6e-5 cycles/symbol error to pull in. At -1 dB 60 % of the
  frames had a slip, the rms phase error was 16 degrees, and a narrower
  loop could not pull in at all. Now the header estimates do not move the
  NCO after the acquisition (up to 1.3 they did for the first two headers,
  f/4 and f/8: at -1 dB a single header estimate is ~1e-3 cycles/symbol rms,
  and the kicks left the narrowed loop ~1.5e-4 to pull in over several
  frames); the PLL integrator (a frame of averaging) is the frequency
  estimate (still folded into the NCO at each header, so the reported
  frequency stays meaningful).
* Loop gain: the AGC holds signal + noise, and with decision errors the
  phase detector's gain at -2 dB is ~1/16 of the high-SNR value the gains
  were designed for: bandwidth and damping (zeta ~ 0.18) far below the
  design. Now kp is twice the bandwidth-0.002 design and ki narrows to 1/4
  and 1/16 two and four headers after the acquisition (`PLL_GEAR`): zeta
  ~ 1.4 at -2 dB. (Narrowing kp too, i.e. the whole bandwidth, slipped
  under phase noise.)
* Hard decisions: a limiter at half the per-axis RMS instead of sign()
  gives the detector twice the SNR at -2 dB.
* The header phase reset was extrapolated to the header's end with the
  header's own (noisy) frequency estimate (~11 degrees rms at -2 dB); the
  PLL has removed the residual frequency, so the mean phase is used.

A feed-forward 4th-power (Viterbi & Viterbi) estimator was tried and is
worse at these SNRs: the 4th power of a symbol at -2 dB has an SNR of
~1/740, so even a sliding 2970-symbol window slips in 4 % of the frames
without phase noise (82 % at 0.01 rad/symbol).

FER, model (carrier model fed with symbols at the AGC level and a tag on
every header, 0.001 cycles/symbol, Wiener phase noise sigma per symbol,
32 seeds x 6 data frames after 16 dummy frames, the LDPC decoder model;
"ideal": the same decoder on exact AWGN LLRs). sigma = 0.001 rad/symbol is
-76 dBc/Hz at 1 kHz at 1 MS/s, 0.003 is -66 dBc/Hz at 1 MS/s or -71 dBc/Hz
at 333 kS/s (L(f) = sigma^2 Rs / (4 pi^2 f^2)):

| QPSK 1/4, Es/N0 | -2.5 | -2.2 | -2.0 | -1.5 | -1.0 | 0 | +1 | +2 dB |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| ideal | 36 % | 1.0 % | 0.5 % | 0 | 0 | 0 | 0 | 0 |
| pilots on, sigma 0 / 0.001 / 0.003 | 70 / 73 / 69 % | 0.5 / 1.0 / 1.0 % | 0.5 / 0 / 0 % | 0 | 0 | 0 | 0 | 0 |
| pilots off, before, sigma 0 | 100 % | 98 % | 93 % | 80 % | 55 % | 18 % | 4.2 % | 0 |
| pilots off, before, sigma 0.001 | 100 % | 94 % | 95 % | 81 % | 66 % | 18 % | 3.1 % | 0 |
| pilots off, before, sigma 0.003 | 100 % | 98 % | 97 % | 85 % | 73 % | 26 % | 5.7 % | 0.5 % |
| pilots off, now, sigma 0 | 63 % | 1.6 % | 0 | 0 | 0 | 0 | 0 | 0 |
| pilots off, now, sigma 0.001 | 61 % | 1.0 % | 0 | 0 | 0 | 0 | 0 | 0 |
| pilots off, now, sigma 0.003 | 85 % | 3.1 % | 0.5 % | 0 | 0 | 0 | 0 | 0 |

| QPSK 1/2, Es/N0 | 0.5 | 1.0 | 1.5 | 2.0 | 2.5 dB |
| --- | --- | --- | --- | --- | --- |
| ideal | 100 % | 46 % | 0 | 0 | 0 |
| pilots on, sigma 0 / 0.001 / 0.003 | 100 % | 90 / 90 / 96 % | 0 | 0 | 0 |
| pilots off, before, sigma 0 / 0.001 / 0.003 | 100 % | 95 / 93 / 98 % | 1.6 / 1.0 / 1.0 % | 0.5 / 1.0 / 0 % | 0 |
| pilots off, now, sigma 0 / 0.001 / 0.003 | 100 % | 66 / 57 / 76 % | 0 | 0 | 0 |

So without pilots QPSK 1/4 now decodes as with pilots (both ~0.1-0.2 dB
from the ideal-LLR decoder at -2.2 dB; pilots on spends 2.4 % of the
symbols on pilots, which this table does not count), and QPSK 1/2 had
lost less (its threshold, ~1 dB, is above where hard decisions break
down). The RTL is bit exact to the model (`test_carrier_pilotless_pn`:
QPSK 1/4 at 0 and -1 dB with 0.002 rad/symbol through all gears, every
frame decoded).

Heavy phase noise still costs more without pilots: at 0.006 rad/symbol
(-60 dBc/Hz at 1 kHz at 1 MS/s) QPSK 1/4 loses 9/192 frames at -1 dB and
2/192 at 0 dB (pilots on: none at -1.5 dB); with half the default kp it
was 34/192 and 18/192. A larger PLL_KP is the knob for a worse LNB.

### PLS decoder

The receive path is told the MODCOD and pilots (MODCOD register); the PLS
decoder reports what the PLHEADERs say, so the software can detect them
(and changes of them) instead of trying hypotheses. It taps the carrier's
input (symbols + the frame sync's header tags) and decodes every tagged
header, data or dummy, in its own time (the next header is >= 3330
symbols away). From VERSION 0x0001_0400 the frame sync uses the decodes to
confirm its headers (see [False lock on repeating frames](#false-lock-on-repeating-frames));
the rest of the receive path does not depend on it.

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
  in a row (`dvbs2-rx --detect`: 3, with the core configured to them).
* The frame sync must still find the headers: with the right pilots it
  finds them with any QPSK MODCOD (its correlation keeps the SOF and the
  pilot-bit differentials: 59..87 of the 89 for another MODCOD, 7..37 for
  the other pilots). Without pilots the headers of some MODCODs are closer
  to the dummy header than to the configured one (QPSK 1/4 differs from it
  in one differential, so with QPSK 1/2 configured the search peak is a
  "dummy" one: it is checked at the data length too, and then every other
  header is found, the next one being predicted at the dummy length). With
  the wrong pilots it predicts the wrong frame length and finds data
  headers only next to dummy frames, and tags symbols that are not headers
  (noise-free QPSK data gives metrics up to ~260, a header ~2000, at -3 dB
  ~300). So `dvbs2-rx --detect` reconfigures the core to the decoded code
  and tries both pilots (below): with the right configuration the frame
  sync locks to the real headers.
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
  too fast (skipped headers), QPSK 1/4 and 2/5 without pilots or dummy
  frames with QPSK 1/2 configured, QPSK 1/4 pilots with the RX told no
  pilots); `test_framesync` (QPSK 1/4 without pilots, QPSK 1/2 configured); `test_rx_end_to_end` (every decode equals the
  model, the confident ones are the right codes); `test_rx_axi*` (the
  registers).

### False lock on repeating frames

Board finding with VERSION 0x0001_0300 (digital loopback, 1 MS/s, QPSK 9/10
pilots): with a TS of only null packets the receiver took 1-20 s to lock
(once > 30 s; QPSK 1/4 pilots > 8 s in 2 of 10 starts), with a PRBS payload
0.25 s. Meanwhile the frame sync said LOCKED, every frame failed LDPC/BCH
and the PLS decoder found no header of the configured code.

Cause (`model/falselock_experiment.py`: null-packet TS through the TX
model, the receiver started at a random point of the stream): null packets
give near-identical BBFRAMEs (the same BBHEADER and payload bytes, only the
packet headers moving with SYNCD), so consecutive PLFRAMEs share most of
their symbols (QPSK 9/10: ~90 %, the systematic part; QPSK 1/4: ~25 %), and
any data window correlates with itself exactly a frame length later. The
search keeps the largest metric since it started; while the AGC settles
from above, early metrics are many times a header's (2.7 x the final gain:
symbol 90 gave 10.4M, a header 8.3M), so that stale peak is never replaced,
and its check a frame length later lands on the same data again, ~4 x the
mean metric (> 4 x and > the threshold): two checks in a row pass and the
frame sync is LOCKED on data, where max(threshold, average peak / 4) keeps
it until the packet headers wander through the window. With a PRBS payload
the check fails and the search starts over. A random data window passing a
dummy-length check by chance (~18 % at high SNR) leads to the same lock.
Model, QPSK 9/10 pilots, 20 dB, 6 starts with the AGC from above: 1.3 locked
on data for the whole run in 3. The differential metric cannot tell such data from a header (it
repeats exactly as a header does), the header's content can.

Fix (1.4, model `FS_PLS_AT` / `FS_PLS_NBAD`, RTL `PLS_AT` / `PLS_NBAD`): the
PLS decoder decodes every tagged header coherently (128 codes, the frequency
grid before the carrier has acquired). 3072 symbols after each tag decision
(the decode is done by then: <= ~95k clocks at H = 32 against >= 2900 x 34
clocks; otherwise the frame sync input waits for `dec_busy`, so model and RTL
stay bit exact) the frame sync uses it: ok = confident and the configured
code or a dummy header (MODCOD 0). Not any code of the configured frame
length: repeating data can decode confidently as some code, the same every
frame (board, 1.3 false lock with qpsk9/10 pilots configured: steadily
qpsk5/6 pilots for 9.6 s, also 8psk3/5, reserved 31, qpsk9/10 without
pilots). With another MODCOD configured each CHECK tag is still decoded, so
`--detect` gets the code and reconfigures (its first confident other decode).
Not ok in CHECK: back to SEARCH; 3 tags in a row not ok in LOCKED:
lost (FS_LOST). ok: the next header is predicted from the decoded type.
Now each wrong CHECK tag (1-2 per start) is dropped 3072 symbols later and
the frame sync locks at the next headers (model: 6 of 6 starts for QPSK 9/10).
QPSK 1/4 pilots (only the systematic quarter repeats) did not lock on data in
the model in either version, but at high SNR random data passes a first check
(~18 % per dummy-length check) and the chain blinds the search until it
ends: 1-6 wrong CHECK tags, the lock 1-3 frames late (the same in 1.3 and
1.4; the board's > 8 s starts were locks on the repeating part, which the
PLS check now ends). At -3 dB a header's
decode is not confident ~1.5 % of the time, so a lock is lost by this rule
~3e-6 per header.

After a PLS drop the next search peak is checked at the data length only
(the search covers a whole data frame first; model `nodz`). Board finding
with the first 1.4 build (QPSK 1/4 without pilots, null packets, digital
loopback, 1 MS/s): 5 of 10 `dvbs2-rx` starts acquired a wrong frequency
(+374 kHz), retyped 6-7 headers, lost 1-8 frames (twice with input
overflows from the padding of retyped frames) and re-acquired. Reproduced
through the ADC and DDC (`falselock_experiment.adc_signal()`, 1 of 32
starts): after a drop the search found the same repeating data at the
dummy length again, a frame length after the last false tag, every frame;
each such tag was dropped, but the carrier took tags a frame length apart
for confirmed headers and acquired 0.374 cycles/symbol from four of them
(the search never reached the real header). With the data-length check
after a drop: 32 of 32 starts acquire once and decode every frame sent. It
costs a frame per drop at low SNR (QPSK 1/4 pilots, -2 dB, 8 dummy frames:
165 instead of 170 of 192 frames).

The same weakness, a lock that nothing about the header's content
confirms, cost the start-up of QPSK 1/4 without pilots (model finding:
`rxfixed_experiment.run(rate_idx=0, pilots=False, esn0=-1.0, seed=0, lead=8)`
decoded 0 of 6 frames, seed 1 6 of 6). Its header is one differential away
from the dummy header, so before the carrier has acquired the frame sync's
frame type is a coin toss: a dummy frame tagged as data sent the prediction
a data frame ahead, where the flywheel and noise passing max(threshold, peak
/ 4) kept it LOCKED; and the carrier's acquisition used only headers whose
next one came at the length of the tag's type. Three changes:

* the frame sync takes the type from the PLS decode (above);
* the carrier confirms an acquisition header when the next one comes at
  either frame length, which gives its type (the stored z of a header of the
  other type is negated where the two headers differ: `cflip`);
* pilots off: after the acquisition no header estimate kicks the NCO
  (`PLL_KICK_NHDR` 2, was 4: at -1 dB one header's estimate is ~1e-3
  cycles/symbol rms, the f/4 and f/8 kicks left the narrowed loop ~1.5e-4
  cycles/symbol to pull in over several frames), and the early
  re-acquisition (2 header estimates in a row beyond 2^-10 within 4 headers)
  is for pilots on only (without pilots it threw good acquisitions away).

Full chain (`rxfixed_experiment`, 0.004 cycles/symbol, 60 ppm, 32 seeds x 6
data frames, frames decoded by gr-dvbs2rx; lead = dummy frames before the
data):

| case | 1.3 | PLS type | + type from length | + no kicks | + no early re-acq | 1.4 |
| --- | --- | --- | --- | --- | --- | --- |
| QPSK 1/4 no pilots -1 dB, lead 8 | 50 | 120 | 159 | 181 | 190 | 187 |
| QPSK 1/4 no pilots -1 dB, lead 16 | 40 | | 192 | | 192 | 192 |
| QPSK 1/4 no pilots -2 dB, lead 8 | 5 | | 82 | | 137 | 144 |
| QPSK 1/4 no pilots -2 dB, lead 16 | | | 186 | | 190 | 189 |
| QPSK 1/2 no pilots 2.5 dB, lead 8 | 186 | | 192 | | 192 | 192 |
| QPSK 1/4 pilots -2 dB / -1 dB, lead 8 | 173 / 191 | | 173 / 191 | | 173 / 191 | 165 / 191 |

(1.4 = all of these plus the start-up details below: frames begun before the
acquisition not sent, pilots off, and the amplitude/noise average restarted
after the acquisition.) With pilots the frame sync and acquisition changes
changed nothing (the same frames decode; 1.4 loses 8 at -2 dB: 3 frames
right after the acquisition from the restarted LLR-gain average, 5 from the
full-frame search after a PLS drop); the
`rxfixed_experiment.py` 7 cases and `pilotless_experiment.py` (ideal tags, acquisition on 16 dummy
frames; pilots off, 0.003 rad/symbol: FER 164 / 6 / 0 of 192 at -2.5 / -2.2 /
-2.0 dB, before 160 / 6 / 1) give the same results as before. At -2 dB with 8 dummy frames the
acquisition itself (SEARCH, 2 checks, 4 confirmed headers) still often ends
inside the data.

Tests: `model/test_model.py` (test_rx_falselock_null_ts, test_rx_start_np14),
`sim/rx/test_rx.py` test_rx_falselock (null-packet TS, QPSK 9/10, RTL = model,
the wrong CHECK dropped, the frames decoded), test_rx_start_np14 (QPSK 1/4
without pilots at -1 dB after 8 dummy frames: 4 of 4 frames, 1.3: 0 of 4) and
test_framesync_pls (a PLS stand-in busy for 150k clocks: the frame sync waits,
tags equal the model; confirmed / not confident / other pilots), and
test_rx_esn0_restart / test_model.py test_rx_esn0_restart (below), and
test_rx_falselock_cycle (both: QPSK 1/4 without pilots, null packets, ADC and
DDC as on the board; RTL = model, one acquisition).

Board (pluto2, 2026-11-04, VERSION 0x0001_0400, AD9363 digital loopback;
plutosatd from station main, `examples/scanclient.rs`, tx0 sending null
packets, 20 starts per fixed case, 10 / 8 per auto case at 1 MS/s / 333 kS/s,
a start not locked within 25 s counted as 25 s; the 1.3 column is the
watchdog measurement in station `docs/api.md` section 4):

| case | 1.3, no watchdog | 1.3 + watchdog | 1.4, watchdog off | 1.4 + watchdog |
| --- | --- | --- | --- | --- |
| fixed qpsk9/10 pilots, 1 MS/s | 0.86 / 2.29 / 12.5 s | 1.07 / 1.04 / 3.9 s (15 restarts) | 0.25 / 0.25 / 0.29 s | 0.25 / 0.25 / 0.29 s (0) |
| fixed qpsk1/4 pilots, 1 MS/s | 0.46 / 0.65 / 3.2 s | 0.45 / 0.70 / 2.5 s (4) | 0.46 / 0.51 / 1.07 s | 0.37 / 0.46 / 1.11 s (0) |
| fixed qpsk9/10 pilots, 333 kS/s | 0.76 / 4.30 / > 25 s | 0.78 / 1.66 / 6.6 s (11) | 0.46 / 0.46 / 0.51 s | 0.46 / 0.47 / 0.66 s (0) |
| fixed qpsk1/4 pilots, 333 kS/s | | | 0.87 / 1.03 / 2.50 s | 0.86 / 0.95 / 1.89 s (0) |
| auto, qpsk9/10 pilots, 1 MS/s | 1.76 / 2.45 / 6.1 s | 1.77 / 1.93 / 4.3 s | | 0.65 / 0.73 / 1.46 s (0) |
| auto, qpsk1/4 pilots, 1 MS/s | 1.87 / 1.82 / 2.9 s | 1.58 / 1.81 / 3.9 s | | 0.85 / 0.89 / 1.26 s (0) |
| auto, qpsk9/10 pilots, 333 kS/s | 9.1 / 8.9 / 17.6 s | 2.6 / 4.4 / 9.4 s | | 1.87 / 1.87 / 1.88 s (0) |
| auto, qpsk1/4 pilots, 333 kS/s | 10.8 / 13.0 / > 25 s | 3.7 / 5.3 / 12.1 s | | 1.87 / 2.28 / 3.50 s (0) |

(median / mean / max; in brackets the watchdog's restarts.) No restart and
no loss of lock in the 3 s after each of the 160 fixed starts. Pilots
switched while locked (auto): 1.08 / 0.89 s (1 MS/s), 2.1 / 2.3 s (333 kS/s).
`dvbs2-selftest` (1 MS/s, 6 s per point): QPSK 1/4 at off, 0, -1, -2 dB,
1/2 at off, 2, 1.5, 3/4 at off, 5, 4.5, 9/10 at off, 8, 7 dB, pilots off and
on: PER 0 everywhere; LDPC failures only with pilots on (0-3 per run, the
frames before the acquisition, which are still sent with pilots on).
`dvbs2-rx --detect` from qpsk1/2 with the wrong pilots, TX qpsk3/4 pilots,
1/4, 9/10 pilots, 2/3, PRBS or null packets: all 16 right, 0.8-1.1 s from
the wrong start, 0.2-0.6 s from the right one. QPSK 2/3 without pilots at
125 kS/s: Es/N0 34-38 dB in the first second, 27-29 dB from the 5th on (1.3:
-21 dB, then ~0.6 dB per frame), no LDPC failure. QPSK 1/4 without pilots,
1 MS/s: PRBS no failure; null packets 5 of 10 starts with a wrong
acquisition (see above; fixed in 0x0001_0401, not yet measured on the
board).

### Start-up details (1.4)

* Frames before the acquisition (pilots off): a data frame that began before
  the frequency acquisition is not sent to the LDPC decoder (its LLRs are
  noise).
  Before the acquisition the frame type is the frame sync's guess, so dummy
  frames taken for data frames each cost the LDPC decoder its full 30
  iterations (QPSK 1/4: ~1.65M clocks) and the carrier the padding of a data
  frame cut short (~29k words, one per clock): at 80 clocks per symbol
  (1.25 MBd) the input FIFO overflowed (sim: 900-2100 samples lost in
  `test_rx_start_np14`, which then lost lock and every frame), and on the
  board each start showed LDPC failures. The frame whose header is the
  acquisition header is not sent either (its type was decided before the
  frequency was known), so a stream of data frames only loses that one frame
  more at the start. With pilots on the frame sync's type is reliable and the
  frame at the acquisition header often decodes, so nothing changes there.
* Amplitude/noise (AMP, ENERGY, the LLR gain): the 1/8 IIR per known block
  restarts after the acquisition (the acquisition header itself, measured
  before the new frequency, is skipped). Before, it began with headers
  measured at a wrong frequency (A far too small); without pilots there is
  one known block per frame, so the Es/N0 reading (software:
  A^2 / (E - A^2)) started at about -21 dB and took ~17 frames to come right
  (board: 125 kS/s QPSK 2/3 without pilots, ~17 s), and the LLRs of the
  first frames were scaled too small. Model, QPSK 2/3 without pilots, 20 dB:
  1.3 read 2.4 / -20.4 dB at the first header after the acquisition (0.004 /
  0.02 cycles/symbol), climbing ~0.6 dB per frame; 1.4 reads 20.3 / 20.5 dB
  there, at any of these offsets (the board's lower readings at larger
  offsets were this slow climb).
* Es/N0 ceiling: the estimate saturates at ~27-30 dB (model at 30 / 40 dB
  without pilots: ~27 / ~29 dB after a few frames), from the fixed-point A
  and E (A ~ 5000 LSB, the IIR's truncation, E - A^2 a small difference of
  large numbers) and the PLL's phase jitter within the 90 header symbols.
  Above ~25 dB use the MER from the constellation snapshot instead; not
  worth wider registers for a receiver that works below 10 dB.

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

`--detect` detects the MODCOD and pilots instead of receiving. The frame
sync needs the right pilots (they set the frame length, and its correlation
with the headers of the other pilots is weak) but finds the headers of any
QPSK MODCOD with them, and with the wrong ones it tags symbols that are not
headers, whose decodes can look confident. So `--detect` starts with the
configured `-m`/`-p` and reconfigures the core (soft reset) to what the
decodes say: to a confidently decoded QPSK normal-frame data header that is
not the configured one (the first such decode since the last
reconfiguration, or the same one twice in a row), and to the other pilots
after 10 frames (at least 0.5 s) without a decode of the configured one. It
prints the MODCOD and pilots after `--detect-count` (3) equal confident
decodes of the configured one in a row (stdout: `MODCOD name pilots short
offset_Hz`, the offset being the signal's frequency from the LO: `-o` plus
the carrier's or the PLS decoder's estimate) and exits 0, or exits 1 after
`--detect-timeout` (30 s). Codes the receiver cannot demodulate (8PSK..,
short frames) are reported after 2 x `--detect-count` equal decodes in a
row. `--pls-grid H` sets the grid (default 32; 2H + 2 hypotheses must fit
in a dummy frame).

```
dvbs2-rx -s 333333 -f 739.75e6 --detect      # e.g. "7 qpsk3/4 1 0 -1250"
```

## Loopback self-test on the board

`dvbs2-selftest` (package `selftest`, source `awgn/`) runs a PRBS transport
stream through `dvbs2-tx`, an AD9363 loopback and `dvbs2-rx`, with the PL
noise source in front of the receiver setting Es/N0, and prints the packet
error rate per point. `-L digital` uses the AD9363 data-port loopback
(nothing transmitted), `-L rf` an external TX1 → RX1 cable. See
[awgn/README.md](../awgn/README.md).
