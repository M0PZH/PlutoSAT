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
| `dvbs2_rx_ddc` | NCO + decimating FIR (decimation 1..32, loadable taps, 2 taps per clock): ADC rate -> 2 samples/symbol; `dvbs2-rx` uses a Kaiser-windowed sinc, 12 taps per decimation step (~58 dB) |
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
chain and to the transmitted BBFRAME.

Linux: UIO device `dvbs2-rx` (registers, DMA, 1 MB buffer at 0x3EF00000)
and `dvbs2-rx` in the same buildroot package as `dvbs2-tx`:

```
dvbs2-rx -s 500000 -m qpsk1/2 -p -f 739.75e6 -u 192.168.2.10:1234   # QO-100 10489.75 MHz, 9750 MHz LNB
```

It sets the AD9363 RX sample rate to 2 * symbol rate * decimation (at least
2.1 MS/s), loads the DDC and RRC coefficients (the same as the model's,
`sw/test_bbdeframe.py::test_rx_coefs`), keeps four DMA transfers queued and
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
