# DVB-S2 transmitter and receiver (SystemVerilog)

QPSK, normal FECFRAMEs, all 11 code rates (1/4 to 9/10), pilots on or off per
frame, any Gold code, dummy PLFRAMEs when no data is queued, and polyphase RRC
interpolation to an integer multiple of the symbol rate followed by an NCO.
Verified against GNU Radio gr-dtv.

```
BBFRAME bytes ─► BB scrambler ─► BCH ─► LDPC ─► FECFRAME FIFO ─► PL framer ─► RRC ×SPS ─► NCO ─► I/Q
(+ {pilots, rate})   dvbs2_fec                  dvbs2_frame_fifo  dvbs2_plframer  dvbs2_rrc_interp  dvbs2_nco
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
  output sample (RRC + NCO) equal the model.
* `test_loopback.py`: RTL output samples -> gr-dvbs2rx (independent
  receiver) -> MPEG-TS; every packet equals the input, none missing.
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
* Output: 16-bit I/Q at SPS samples per symbol, pulled with `m_ready`.
* `sps`: 2..64. RRC coefficients go in through `coef_*`, phase-major
  (`addr = p*16 + j` holds `h[p + sps*j]`); `model/txmodel.py:rrc_coefs()`
  generates them.
* `gold_xa/gold_xb`: x-sequence states for Gold code n
  (`txmodel.gold_init(n)`). For n = 0 they are `PLS_XA_GOLD0/PLS_XB_GOLD0`.
* PlutoDVB2 (F5OEO) uses the ORI `dvb_fpga` VHDL modulator and changes the
  AD9363 rate per symbol rate. This core instead keeps the DAC at a fixed
  rate (12 MS/s in the design notes) and interpolates by an integer SPS.

## Fishball bitstream integration

`boards/fishball/hdl/system_bd.tcl` adds `dvbs2_tx` (module reference) at
0x7C500000 and its own `axi_dmac` (memory -> stream, HP3) at 0x7C480000,
IRQ ps-10. The ADI FIR interpolator output passes through the core's DAC
selector into `axi_ad9361` channel 0; CTRL[2] switches channel 0 to DVB-S2.
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
ADC ─► DDC ─► timing ─► frame sync ─► carrier + demap ─► LDPC ─► BCH ─► BBFRAME ─► DMA packet
       dvbs2_rx_ddc  dvbs2_rx_timing  dvbs2_rx_framesync  dvbs2_rx_carrier  dvbs2_ldpc_dec  dvbs2_bch_dec
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

### Fishball bitstream integration

`boards/fishball/hdl/system_bd.tcl` adds `dvbs2_rx` (module reference
`ip/dvbs2_rx_bd.v`) at 0x7C510000, fed from `axi_ad9361` channel 0 (before
the ADI FIR decimator), and `axi_dvbs2_rx_dma` (stream -> memory, HP0) at
0x7C490000, IRQ ps-9. Everything after the ADC sample FIFO runs on
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
every decimation.

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

## Loopback self-test on the board

`dvbs2-selftest` (package `selftest`, source `awgn/`) runs a PRBS transport
stream through `dvbs2-tx`, an AD9363 loopback and `dvbs2-rx`, with the PL
noise source in front of the receiver setting Es/N0, and prints the packet
error rate per point. `-L digital` uses the AD9363 data-port loopback
(nothing transmitted), `-L rf` an external TX1 → RX1 cable. See
[awgn/README.md](../awgn/README.md).
