# Spectrum analyser (PL FFT)

Averaged 4096-bin power spectra computed in the PL; the ARM only reads the
result (4096 32-bit words over AXI-Lite) and converts to dB. One FFT core,
time-shared between the sources selected by a register: rx0 or rx1 at the
ADC rate (12 MSPS) or one nbiq channel (0..4; the 600 kS/s overview or a
48 kHz user channel).

```
 adc_clk (l_clk)          | s_axi_aclk (100 MHz)
 rx0/rx1 --> reg --> async FIFO --+
                                  +--> select --> 4096 consecutive samples
 nbiq tap (tap_ch == NB_CH) ------+        --> window (2048 x 18 table, Blackman-Harris)
                                           --> xfft (radix-2 Lite, 16-bit, scaled, convergent rounding)
                                           --> |X|^2 (32-bit) --> sum over NAVG frames (4096 x 40)
                                           --> >> RSHIFT, saturate --> RESULT (FFT-shifted)
```

| | |
|---|---|
| FFT | 4096 points, Xilinx xfft 9.1 radix-2 Lite burst I/O, 16-bit data and twiddles, run-time scale schedule, convergent rounding, natural order |
| Window | 2048-entry symmetric half table (w[n] = w[4095-n]), 18-bit unsigned, 2^17 = 1.0, writable; reset: 4-term Blackman-Harris (-92 dB sidelobes, ENBW 2.00 bins, coherent gain 0.35875) |
| Averaging | NAVG = 1..256 spectra, 40-bit sums (cannot overflow) |
| Result | 4096 x 32-bit: min(sum >> RSHIFT, 2^32 - 1), RSHIFT 0..8 (log2 NAVG gives the mean power) |
| Order | FFT-shifted: word j is frequency (j - 2048) fs / 4096 (j = 2048 is DC) |
| Throughput | 12 MSPS: 1144 spectra/s; 600 kSPS (overview): 136/s; 48 kSPS: 11.6/s (below) |
| Cost | 1.1k LUT, 2.1k FF, 10 BRAM tiles, 6 DSP (out of context; DOUBLE_BUF = 1: +4 BRAM tiles, +3 LUT) |

## Files

- `rtl/fft_axi.sv`: top: AXI-Lite registers (the map is in its header
  comment and in `regs/fft.toml`), ADC clock crossing, source select,
  sequencer, xfft instance. `rtl/fft_win.sv`: window table and multiply.
  `rtl/fft_acc.sv`: |X|^2, accumulator, result buffer.
  `rtl/gen/fft_win.mem`: window reset contents (`model/fftspec.py` writes it).
  Also needs `dvbs2/rtl/dvbs2_async_fifo.sv`.
- `ip/fft_xfft.tcl`: `fft_create_xfft ?dir?` creates the xfft instance
  `fft_xfft` in the current project. `ip/fft_bd.v`: Verilog-2001 wrapper
  for block-design module reference (parameters `WIN_FILE`, `DOUBLE_BUF`).
- `model/fftspec.py`: bit-exact model (window, the Xilinx bit-accurate FFT
  C model through ctypes, |X|^2, sums, shift, saturation, index map), the
  numpy reference and its tolerance, dB conversion.
- `sim/`: xsim testbench `tb_fft.sv`, `xsim.py` (build / run), `test_fft.py`
  (pytest), `build_ip.tcl`.
- `syn/ooc.tcl`, `syn/fft_cdc.xdc`: out-of-context synthesis and
  implementation.
- `sw/fft_spectrum.py`: board test (UIO or /dev/mem, numpy, dB); not yet
  run on hardware.

## Registers

64 KB (UIO map 0), `regs/fft.toml` / `regs/gen/fft_regs.h`:

| Offset | Name | |
|---|---|---|
| 0x000 | VERSION | 0x0001_0000 |
| 0x004 | ID | "FFTS" 0x46465453 |
| 0x008 | CTRL | [0] RUN, [1] IRQ_EN, [5:4] SRC (0 rx0, 1 rx1, 2 nbiq), [10:8] NB_CH |
| 0x00C | CMD | (wo) [0] START (clear DONE, start one average if idle), [1] ABORT |
| 0x010 | NAVG | [8:0] spectra per average 1..256 |
| 0x014 | SCALE | [23:0] xfft scale schedule, 2 bits per radix-2 stage, stage 0 in [1:0]; reset 0x555400 |
| 0x018 | RSHIFT | [3:0] 0..8 |
| 0x01C | STATUS | (ro) [0] BUSY, [1] DONE, [2] CAPT, [15:8] FRAME |
| 0x020 | DONE | [0] result ready, write 1 to clear; irq = DONE & IRQ_EN |
| 0x024 | FLAGS | sticky, w1c: [0] FFT_OVF, [1] SAT, [2] LOST, [3] DROP |
| 0x028 | FRAMES | (ro) frames accumulated |
| 0x02C | AVGS | (ro) results published (sequence number) |
| 0x030 | DROPS | (ro) averages dropped (DOUBLE_BUF) |
| 0x034 | RES_CFG | (ro) of the result: [1:0] SRC, [6:4] NB_CH, [11:8] RSHIFT, [24:16] NAVG |
| 0x038 | RES_SCALE | (ro) scale schedule of the result |
| 0x03C | INFO | (ro) [3:0] log2 N = 12, [4] DOUBLE_BUF |
| 0x2000 + 4n | WIN | (wo) window table, n < 2048 |
| 0x4000 + 4j | RESULT | (ro) result word j |

CTRL.SRC / NB_CH, NAVG, SCALE and RSHIFT are taken when an average starts
(RES_CFG / RES_SCALE tell what the buffer holds). Writing them during an
average affects the next one; for an immediate switch write CTRL, then
CMD.ABORT (with RUN set a new average starts at once).

Scale schedules (no overflow for any input within the stated range; the
C model was run on full-scale tones at 43 frequencies, real and complex,
clipped Gaussian noise, DC corners and square waves):

| Source | SCALE | Total shift | Full-scale tone peak |
|---|---|---|---|
| rx0 / rx1, 12-bit (±2048) | 0x555400 (reset) | 7 | 23505 per component |
| nbiq at its default gain 8 (±16384) | 0x555550 | 10 | 23506 |
| any 16-bit data | 0x555555 | 12 | 11752 |

The FFT's own rounding noise is ~0.41 LSB rms per output component
(measured on the C model; 0.336 LSB^2 per bin), independent of the
schedule: at 0x555400 that is 92 dB below a full-scale 12-bit tone in the
same bin, the level ADC noise of 1.6 LSB rms per component (-62 dBFS)
would give. The displayed noise floor is the ADC's as long as the receiver
gain puts the noise above that. One shift less than the table (6 dB lower
FFT floor) overflows on full-scale real tones (FLAGS.FFT_OVF).

## Using it from the ARM

Snapshot (one average):

1. CTRL = SRC/NB_CH (RUN = 0), NAVG, SCALE, RSHIFT = log2(NAVG).
2. CMD = 1 (START: clears DONE, starts).
3. Wait for DONE (poll DONE or STATUS, or the interrupt with IRQ_EN).
4. Read RES_CFG, RES_SCALE, FLAGS and the 4096 RESULT words, then repeat 2.

Continuous: CTRL.RUN = 1 (and IRQ_EN). After each DONE: read, then write
DONE = 1. With `DOUBLE_BUF = 0` (default build) the next average starts
when DONE is cleared, so the result is never overwritten while being read
(the ARM's read time is a gap between averages). With `DOUBLE_BUF = 1`
averages run back to back into the accumulator and only the result buffer
waits: an average that ends while DONE is still set is dropped (FLAGS.DROP,
DROPS), so a result is likewise never overwritten while DONE is set. AVGS
read before and after a copy tells whether a new result arrived.

Interrupt: level, DONE & IRQ_EN. With `generic-uio` the kernel masks the
line in its handler; clear DONE (or START) before writing 1 to the UIO fd.

Conversion to dB (`model/fftspec.py` `to_db`, `sw/fft_spectrum.py`):

    p[j]   = RESULT[j] * 2^RSHIFT / NAVG            mean |X|^2 of word j
    ref    = (A_fs * sum(w) / 2^17 / 2^shift)^2      a full-scale tone on a bin
    dBFS   = 10 log10(p[j] / ref)                    shift = sum of the SCALE stages
    f[j]   = (j - 2048) * fs / 4096                  (+ the LO / NCO frequency)

with A_fs = 2048 for the ADC, 16384 for nbiq at gain 8, and sum(w) / 2^17 =
1469.1 for the reset window (coherent gain 0.3587 x 4096). The noise
bandwidth of one bin is ENBW x fs / 4096 = 2.005 x fs / 4096 (5.87 kHz at
12 MSPS, 23.5 Hz at 48 kSPS).

`sw/fft_spectrum.py --src rx0 --navg 16 --count 100` prints the peak and the
median level per result.

## Performance

The radix-2 Lite core loads, processes and unloads one frame at a time (it
cannot load the next frame while processing or unloading; its input FIFO
would take ~14 samples early, so the sequencer waits until the previous
frame has left the core). Frame period:

    T = 4096 / fs + 49185 clocks (processing) + 4096 clocks (unloading) + ~10
      = 4096 / fs + 533 us at 100 MHz

| Source | fs | Resolution | Frame period | Spectra/s | Duty cycle |
|---|---|---|---|---|---|
| rx0 / rx1 | 12 MSPS | 2.93 kHz | 874 us (simulated: 874.25) | 1144 | 39 % of the samples |
| nbiq overview | 600 kSPS | 146 Hz | 7.36 ms | 136 | 93 % |
| nbiq user channel | 48 kSPS | 11.7 Hz | 85.9 ms | 11.6 | 99.4 % |

An average of NAVG frames takes NAVG x T (plus, with `DOUBLE_BUF = 0`, the
time until the ARM clears DONE; one RESULT read is ~4096 AXI-Lite reads,
about 1-2 ms from user space). At 12 MSPS the limit is the FFT (a faster
xfft architecture, radix-2 burst or radix-4, would cost more LUTs/DSPs);
at the nbiq rates it is the sample rate.

Not built, if a user channel needs more than 11.6 spectra/s: 50 %
overlapped frames (a 2048-sample capture buffer, ~2 BRAM36 and a few
dozen LUTs: 23 spectra/s at 48 kSPS), or a shorter transform (the xfft
run-time transform length option, e.g. 1024 points: 4x the rate at 4x
the bin width).

## Interfaces

- `s_axi_*`: AXI-Lite slave, 16-bit address (64 KB), `s_axi_aclk` (the
  100 MHz `sys_cpu_clk`), `s_axi_aresetn` active low.
- `irq`: level-high, "average done" (DONE & IRQ_EN).
- `adc_clk`, `adc_valid`, `adc_data_{i0,q0,i1,q1}`: the AD9363 receive
  samples in the l_clk domain (the outputs of `rx_awgn`: `out_valid_i0`
  qualifies all four lanes), 16-bit two's complement (12-bit samples
  sign-extended). Registered once, then written into a 16 x 64 Gray-code
  FIFO (`dvbs2_async_fifo`) to `s_axi_aclk`; the FIFO is drained every
  clock, so it never fills at any AD9363 rate.
- `tap_valid`, `tap_ch[2:0]`, `tap_i[15:0]`, `tap_q[15:0]`: the nbiq channel
  outputs in the `s_axi_aclk` domain, interleaved, at most one sample per
  clock (no back-pressure). The core takes the samples with
  `tap_ch == NB_CH`.

## Resources and timing (out of context)

`syn/ooc.tcl` (xc7z020clg400-2, s_axi_aclk 100 MHz, adc_clk 250 MHz,
implemented):

| Build | LUT | (LUTRAM / SRL) | FF | BRAM tiles | DSP | WNS 100 MHz | WNS 250 MHz |
|---|---|---|---|---|---|---|---|
| DOUBLE_BUF = 0 | 1101 | 44 / 154 | 2088 | 10 (5 RAMB36 + 10 RAMB18) | 6 | +2.95 ns | +0.83 ns |
| DOUBLE_BUF = 1 | 1104 | 44 / 154 | 2123 | 14 (9 RAMB36 + 10 RAMB18) | 6 | +2.91 ns | +0.98 ns |

Split (DOUBLE_BUF = 0): xfft 591 LUT / 4.5 BRAM / 2 DSP, accumulator 224 LUT /
4.5 BRAM (4096 x 40) / 2 DSP, window 30 LUT / 1 BRAM / 2 DSP, ADC FIFO 92 LUT,
registers and sequencer 165 LUT. The 250 MHz path is the FIFO write side
(Gray pointer compare -> LUTRAM write enable), as in nbiq and dvbs2_rx.
`report_cdc`: CDC-6 for the two Gray pointers (expected), CDC-1 for the
32 FIFO read-data bits (LUTRAM read port -> `s_i`/`s_q` registers; the
FIFO only presents a word after its write pointer has crossed, the path is
constrained by the 10 ns max_delay, as in nbiq and dvbs2_rx). Hold met.
The module-reference flow was checked separately: a block design with
`fft_bd` (xfft inside) validates and synthesises with project runs.

## Integration (boards/fishball, for the coordinator)

`system_bd.tcl`, after nbiq (which provides the tap):

```tcl
# Spectrum analyser (fft/): averaged 4096-bin spectra of rx0, rx1 or an
# nbiq channel, read by the ARM over AXI-Lite (UIO "fft")
set fft_dir [file normalize [file join $ad_hdl_dir .. fft]]
source [file join $fft_dir ip fft_xfft.tcl]
fft_create_xfft
add_files -norecurse [list \
  [file join $fft_dir rtl fft_win.sv] [file join $fft_dir rtl fft_acc.sv] \
  [file join $fft_dir rtl fft_axi.sv] [file join $fft_dir rtl gen fft_win.mem] \
  [file join $fft_dir ip fft_bd.v]]
update_compile_order -fileset sources_1
create_bd_cell -type module -reference fft_bd fft

ad_connect axi_ad9361/l_clk       fft/adc_clk
ad_connect rx_awgn/out_valid_i0   fft/adc_valid
ad_connect rx_awgn/out_data_i0    fft/adc_data_i0
ad_connect rx_awgn/out_data_q0    fft/adc_data_q0
ad_connect rx_awgn/out_data_i1    fft/adc_data_i1
ad_connect rx_awgn/out_data_q1    fft/adc_data_q1
ad_connect nbiq/tap_valid         fft/tap_valid
ad_connect nbiq/tap_ch            fft/tap_ch
ad_connect nbiq/tap_i             fft/tap_i
ad_connect nbiq/tap_q             fft/tap_q

ad_cpu_interconnect 0x7C540000 fft
ad_cpu_interrupt ps-7 mb-7 fft/irq
```

`dvbs2/rtl/dvbs2_async_fifo.sv` is already in the project (dvbs2_rx,
nbiq). Clock: `s_axi_aclk` = sys_cpu_clk (ad_cpu_interconnect connects it
and `s_axi_aresetn`), `adc_clk` = l_clk.

`system_constr.xdc`:

```tcl
# Spectrum analyser: AD9361 l_clk -> sys_cpu_clk ADC sample FIFO inside fft
set fft_cdc [get_cells -hier -filter {NAME =~ */fft/*/u_adc_fifo}]
set_max_delay -datapath_only 4.0 \
  -from [get_cells -hier -filter "NAME =~ ${fft_cdc}/wgray_reg*"] \
  -to   [get_cells -hier -filter "NAME =~ ${fft_cdc}/wgray_r1_reg*"]
set_max_delay -datapath_only 4.0 \
  -from [get_cells -hier -filter "NAME =~ ${fft_cdc}/rgray_r_reg*"] \
  -to   [get_cells -hier -filter "NAME =~ ${fft_cdc}/rgray_w1_reg*"]
set_max_delay -datapath_only 10.0 \
  -from [get_cells -hier -filter "NAME =~ ${fft_cdc}/mem_reg*"]
set_false_path -to [get_cells -hier -filter {NAME =~ */fft/*/rst_s_reg[0]}]
```

Device tree (ps-7 = IRQ_F2P[7] = GIC SPI 36):

```dts
	/* spectrum analyser (fft/) */
	fft@7c540000 {
		compatible = "generic-uio";
		reg = <0x7c540000 0x10000>;
		interrupt-parent = <&intc>;
		interrupts = <0 36 IRQ_TYPE_LEVEL_HIGH>;
	};
```

## Verification

`sim/test_fft.py` (pytest; runs the cases in parallel, about 8 minutes):

    cd fft/sim && ../../dvbs2/.venv/bin/python -m pytest -s test_fft.py

`xsim.py build` generates the xfft simulation model (Vivado batch,
`build_ip.tcl`), compiles the RTL, the testbench and the IP, and elaborates
two snapshots (DOUBLE_BUF = 0 and 1). Each case writes stimulus files and a
command script, runs xsim with the real xfft model, and checks the event
log. The ADC side runs at 250 MHz (4 ns) with a 12 MSPS valid strobe
(12 per 250 clocks); the nbiq tap carries all five channels interleaved in
random order, with same-channel runs (one sample per clock) and idle clocks.

Checks per result dump:

- every captured frame is 4096 consecutive samples of the selected source
  (located in the stimulus: proves the clock crossing, the source and
  channel select, and frame contiguity), frames in order, not overlapping;
- the 4096 words equal the bit-exact model (window, Xilinx bit-accurate
  xfft C model, |X|^2, sums, shift, saturation, FFT-shifted order);
- the words agree with numpy (float FFT of the same windowed samples
  divided by 2^shift, |.|^2, summed, shifted) within the derived tolerance;
- tones sit in the expected word (j = bin + 2048) at the expected level;
- RES_CFG / RES_SCALE match the configuration of the average.

Cases: rx0 (NAVG 1 and 4, RSHIFT 2, register reset values), rx1 (NAVG 3),
nbiq channels 3, 4, 0, RUN with acknowledge (both builds; back-to-back
timing with DOUBLE_BUF), dropped averages (DOUBLE_BUF: the result stays
intact while later averages run), source switching (CTRL during an average
applies to the next, ABORT restarts at once), FFT overflow flag and result
saturation, a window written over AXI (rectangular).

Results (2026-11-04): 19 result dumps in 9 cases, all bit-exact with the
model, largest error / numpy tolerance 0.22 (rectangular window) and
0.07..0.16 otherwise; tone levels within 0.012 dB, peaks in the expected
words; FFT overflow is flagged and still bit-exact with the C model;
saturated words and FLAGS.SAT match the model; LOST never set; simulated
frame period at 12 MSPS 874.2 us.

Tolerance (`fftspec.fft_err_sigma`, `power_tolerance`): radix-2 stage s
(0..11) rounds every butterfly output to the 16-bit grid after a shift of
b_s bits (convergent rounding: variance (1 - 4^-b_s)/12) and rounds the
twiddle product (1/12; not in stage 0). An error made in stage s reaches an
output through 2^(11-s) paths of the remaining stages, each scaled by
2^-(b_{s+1} + ... + b_11), so

    sigma^2 = sum_s [(1 - 4^-b_s)/12 + [s > 0]/12] * 2^(11-s) / 4^(b_{s+1} + ... + b_11)

per output component (0.54 LSB for 0x555400 and 0x555555; measured on the C
model: 0.41). The twiddles (16-bit) add a relative error below 12 x 2^-15.
With E = 7 sigma sqrt(2) + 12 x 2^-15 |X| per complex output, a power sum
may differ from numpy by at most sum_f (2 |X_f| E + E^2), divided by
2^RSHIFT, plus 1 for the floor of the shift. Truncation instead of
convergent rounding (the IP option) would add a -0.93 LSB bias per
component and errors up to 6 LSB (+48 LUT, +92 FF for rounding).
