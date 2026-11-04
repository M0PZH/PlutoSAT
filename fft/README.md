# Spectrum analyser (PL FFT)

Averaged power spectra (N = 4096, 2048, 1024 or 512 bins, chosen per
average) computed in the PL; the ARM reads the result (N 32-bit words) over
AXI-Lite, or the core streams each result as one packet to an axi_dmac
(AXI4-Stream master, 64-bit) that writes it into a DDR ring, and converts
to dB. One FFT core, time-shared between the sources selected by a
register: rx0 or rx1 at the ADC rate (12 MSPS) or one nbiq channel (0..4;
the 600 kS/s overview or a 48 kHz user channel).

```
 adc_clk (l_clk)          | s_axi_aclk (100 MHz)
 rx0/rx1 --> reg --> async FIFO --+
                                  +--> select --> N consecutive samples (N = 4096 >> LEN)
 nbiq tap (tap_ch == NB_CH) ------+        --> window (2048 x 18 table, Blackman-Harris; every 4096/N-th sample)
                                           --> xfft (radix-2 Lite, run-time N, 16-bit, scaled, convergent rounding)
                                           --> |X|^2 (32-bit) --> sum over NAVG frames (4096 x 40)
                                           --> >> RSHIFT, saturate --> RESULT (FFT-shifted)
                                                     |--> AXI-Lite RESULT window
                                                     '--> m_axis: header + N words per result (CTRL.STREAM)
```

| | |
|---|---|
| FFT | N = 4096 (reset), 2048, 1024 or 512 points (CTRL.LEN, per average), Xilinx xfft 9.1 radix-2 Lite burst I/O with run-time transform length, 16-bit data and twiddles, run-time scale schedule, convergent rounding, natural order |
| Window | 2048-entry symmetric half table of the 4096-point window (w[m] = w[4095-m]), 18-bit unsigned, 2^17 = 1.0, writable; reset: 4-term Blackman-Harris (-92 dB sidelobes, ENBW 2.00 bins, coherent gain 0.35875). Shorter frames use every 4096/N-th sample: the same sidelobes and ENBW (below) |
| Averaging | NAVG = 1..256 spectra, 40-bit sums (cannot overflow) |
| Result | N x 32-bit: min(sum >> RSHIFT, 2^32 - 1), RSHIFT 0..8 (log2 NAVG gives the mean power) |
| Order | FFT-shifted: word j is frequency (j - N/2) fs / N (j = N/2 is DC) |
| Readout | AXI-Lite (RESULT window), and/or the result stream (m_axis, one packet per result: 16-byte header + 4 N bytes, tlast at the end) |
| Throughput | 12 MSPS: 1142 spectra/s at N = 4096, 5006/s at 1024; 600 kSPS (overview): 136/s at 4096; 48 kSPS: 11.6/s at 4096, 46.6/s at 1024 (below) |
| Cost | 1.5k LUT, 2.4k FF, 10 BRAM tiles, 6 DSP (out of context; DOUBLE_BUF = 1: +4 BRAM tiles); the stream and the run-time length cost +388 LUT, +304 FF, no BRAM |

## Files

- `rtl/fft_axi.sv`: top: AXI-Lite registers (the map is in its header
  comment and in `regs/fft.toml`), ADC clock crossing, source select,
  sequencer, xfft instance, result stream. `rtl/fft_win.sv`: window table
  and multiply. `rtl/fft_acc.sv`: |X|^2, accumulator, result buffer.
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
- `sw/fft_spectrum.py`: board test (UIO or /dev/mem, AXI-Lite readout,
  numpy, dB, `--len`); not yet run on hardware.

## Registers

64 KB (UIO map 0), `regs/fft.toml` / `regs/gen/fft_regs.h`:

| Offset | Name | |
|---|---|---|
| 0x000 | VERSION | 0x0001_0100 (1.1; 1.0 had N = 4096 only and no stream) |
| 0x004 | ID | "FFTS" 0x46465453 |
| 0x008 | CTRL | [0] RUN, [1] IRQ_EN, [2] STREAM, [5:4] SRC (0 rx0, 1 rx1, 2 nbiq), [10:8] NB_CH, [13:12] LEN (N = 4096 >> LEN) |
| 0x00C | CMD | (wo) [0] START (clear DONE, start one average if idle), [1] ABORT, [2] STR_ABORT |
| 0x010 | NAVG | [8:0] spectra per average 1..256 |
| 0x014 | SCALE | [23:0] xfft scale schedule, 2 bits per radix-2 stage, stage 0 in [1:0] (stages 0..log2 N - 1 apply); reset 0x555400 |
| 0x018 | RSHIFT | [3:0] 0..8 |
| 0x01C | STATUS | (ro) [0] BUSY, [1] DONE, [2] CAPT, [3] STR_WAIT, [4] STR_SEND, [15:8] FRAME |
| 0x020 | DONE | [0] result ready, write 1 to clear; irq = DONE & IRQ_EN |
| 0x024 | FLAGS | sticky, w1c: [0] FFT_OVF, [1] SAT, [2] LOST, [3] DROP, [4] SDROP |
| 0x028 | FRAMES | (ro) frames accumulated |
| 0x02C | AVGS | (ro) results published (sequence number) |
| 0x030 | DROPS | (ro) averages dropped (DOUBLE_BUF) |
| 0x034 | RES_CFG | (ro) of the result: [1:0] SRC, [6:4] NB_CH, [11:8] RSHIFT, [15:12] LOG2N, [24:16] NAVG, [28] FFT_OVF, [29] SAT |
| 0x038 | RES_SCALE | (ro) scale schedule of the result |
| 0x03C | INFO | (ro) [3:0] log2 of the largest N = 12, [4] DOUBLE_BUF, [5] STREAM (m_axis present), [11:8] log2 of the smallest N = 9 |
| 0x040 | STR_SENT | (ro) stream packets sent |
| 0x044 | STR_DROPS | (ro) results not streamed |
| 0x2000 + 4n | WIN | (wo) window table, n < 2048 |
| 0x4000 + 4j | RESULT | (ro) result word j < N |

CTRL.SRC / NB_CH / LEN, NAVG, SCALE and RSHIFT are taken when an average
starts (RES_CFG / RES_SCALE tell what the buffer holds). Writing them during
an average affects the next one; for an immediate switch write CTRL, then
CMD.ABORT (with RUN set a new average starts at once). RES_CFG.FFT_OVF /
SAT are the flags of that result alone (FLAGS: sticky over all results).

Changes from 1.0 (compatible: software that writes 0 to the new fields gets
the 1.0 behaviour): CTRL.STREAM, CTRL.LEN, CMD.STR_ABORT, STATUS.STR_WAIT /
STR_SEND, FLAGS.SDROP, RES_CFG.LOG2N / FFT_OVF / SAT, INFO.STREAM /
LOG2N_MIN, STR_SENT, STR_DROPS; the stream packet header (`[[packet]]`
HDR0 / HDR1 in `regs/fft.toml`).

Scale schedules (no overflow for any input within the stated range; the
C model was run on full-scale tones at 43 frequencies, real and complex,
clipped Gaussian noise, DC corners and square waves; for N < 4096 on
tones at 23 frequencies, real and complex, noise, DC corners and square
waves at every N):

| Source | SCALE | Total shift (N = 4096 / 2048 / 1024 / 512) | Full-scale tone peak |
|---|---|---|---|
| rx0 / rx1, 12-bit (±2048) | 0x555400 (reset) | 7 / 6 / 5 / 4 | 23505 per component |
| nbiq at its default gain 8 (±16384) | 0x555550 | 10 / 9 / 8 / 7 | 23506 |
| any 16-bit data | 0x555555 | 12 / 11 / 10 / 9 | 11752 |

The same SCALE serves every N: a shorter transform drops the last (scaled)
stages, whose bits are ignored, and the peak of a full-scale tone stays the
same. The total shift is the sum over the first log2 N stages.

The FFT's own rounding noise is ~0.41 LSB rms per output component
(measured on the C model; 0.336 LSB^2 per bin), independent of the
schedule: at 0x555400 that is 92 dB below a full-scale 12-bit tone in the
same bin, the level ADC noise of 1.6 LSB rms per component (-62 dBFS)
would give. The displayed noise floor is the ADC's as long as the receiver
gain puts the noise above that. One shift less than the table (6 dB lower
FFT floor) overflows on full-scale real tones (FLAGS.FFT_OVF).

## Using it from the ARM

Snapshot (one average):

1. CTRL = SRC/NB_CH/LEN (RUN = 0), NAVG, SCALE, RSHIFT = log2(NAVG).
2. CMD = 1 (START: clears DONE, starts).
3. Wait for DONE (poll DONE or STATUS, or the interrupt with IRQ_EN).
4. Read RES_CFG, RES_SCALE, FLAGS and the N = 2^RES_CFG.LOG2N RESULT
   words, then repeat 2.

Continuous: CTRL.RUN = 1 (and IRQ_EN). After each DONE: read, then write
DONE = 1. With `DOUBLE_BUF = 0` (default build) the next average starts
when DONE is cleared, so the result is never overwritten while being read
(the ARM's read time is a gap between averages). With `DOUBLE_BUF = 1`
averages run back to back into the accumulator and only the result buffer
waits: an average that ends while DONE is still set is dropped (FLAGS.DROP,
DROPS), so a result is likewise never overwritten while DONE is set. AVGS
read before and after a copy tells whether a new result arrived.

Stream (DMA, see "Result stream" below): CTRL = STREAM | RUN | SRC/NB_CH/LEN
(and NAVG etc.); keep axi_dmac transfers queued (each one packet, FLAGS =
TLAST, X_LENGTH = PKT_BYTES_MAX - 1 = 16399 or the exact packet size of
the configured N); each finished transfer holds one spectrum with its
header. No AXI-Lite traffic per result, no acknowledge: with STREAM, RUN
does not wait for DONE. Snapshots work the same way with RUN = 0 (START per
result). DONE and the interrupt still mark each result, but in STREAM mode
DONE no longer keeps the next average from overwriting RESULT (DOUBLE_BUF
= 0: the accumulator is the result buffer); read RESULT over AXI-Lite only
with RUN = 0 or check AVGS around the copy.

Interrupt: level, DONE & IRQ_EN. With `generic-uio` the kernel masks the
line in its handler; clear DONE (or START) before writing 1 to the UIO fd.

Conversion to dB (`model/fftspec.py` `to_db`, `sw/fft_spectrum.py`):

    N      = 2^LOG2N                                 (RES_CFG or the packet header)
    p[j]   = RESULT[j] * 2^RSHIFT / NAVG            mean |X|^2 of word j
    ref    = (A_fs * sum(w) / 2^17 / 2^shift)^2      a full-scale tone on a bin
    dBFS   = 10 log10(p[j] / ref)                    shift = sum of the first log2 N SCALE stages
    f[j]   = (j - N/2) * fs / N                      (+ the LO / NCO frequency)

with A_fs = 2048 for the ADC, 16384 for nbiq at gain 8, and sum(w) / 2^17 =
1469.08 x N / 4096 for the reset window (coherent gain 0.3587 x N;
`regs/fft.toml` WIN_SUM_<N>: 1469.08, 734.54, 367.27, 183.64). With the
default schedules A_fs / 2^shift x 4096 / N is 16 for the ADC and nbiq
alike, so ref = (16 x 1469.08)^2 at every N. The noise bandwidth of one bin
is ENBW x fs / N = 2.005 x fs / N (5.87 kHz at 12 MSPS and N = 4096, 23.5
Hz at 48 kSPS and 4096, 94 Hz at 48 kSPS and 1024).

`sw/fft_spectrum.py --src rx0 --navg 16 --count 100` prints the peak and the
median level per result (`--len 1024` for 1024 points).

## Transform length

CTRL.LEN selects N = 4096 >> LEN per average: the xfft is built with the
run-time transform length option (config word NFFT field = log2 N), the
sequencer captures N samples, the window takes every 4096/N-th sample of
the 4096-point table (m = n x 4096 / N, mirrored: w[m] = WIN[m] for m <
2048, WIN[4095 - m] above), the accumulator uses words 0..N-1 with the
FFT shift j = k ^ N/2. RESULT words N..4095 are stale. 512 to 4096 are
offered (log2 N = 9..12; INFO.LOG2N_MIN); the xfft would do 8 points and up.

Window quality (reset Blackman-Harris table, 18-bit, out of `model/fftspec.py`
and the test `test_model_window_len`):

| N | Samples used | Highest sidelobe | ENBW | Coherent gain | Scalloping loss | sum(w) / 2^17 |
|---|---|---|---|---|---|---|
| 4096 | all | -92.0 dB | 2.0048 bins | 0.35866 | 0.83 dB | 1469.08 |
| 2048 | every 2nd | -92.0 dB | 2.0048 bins | 0.35866 | 0.83 dB | 734.54 |
| 1024 | every 4th | -92.0 dB | 2.0048 bins | 0.35866 | 0.83 dB | 367.27 |
| 512 | every 8th | -92.0 dB | 2.0048 bins | 0.35866 | 0.83 dB | 183.64 |

Decimating the 4096-point window keeps it a 4-term cosine sum sampled
uniformly over (almost) its whole length, so its spectrum is that of the
full window: the -92 dB sidelobes and the ENBW are unchanged (the frame is
not exactly symmetric, sample N-1 is the 4096-point sample 4096 - 4096/N,
which only adds a linear phase). Decimating the half table instead (the
first idea: w'[n] = WIN[n x 4096/N], mirrored at N/2) would put a kink at
the centre and raise the sidelobes to -83 dB (offset 4096/N/2 - 1 or
4096/N/2) or -66..-78 dB (other offsets); separate per-N tables would need
another RAMB36. A window written to WIN applies to all N the same way.

## Performance

The radix-2 Lite core loads, processes and unloads one frame at a time (it
cannot load the next frame while processing or unloading; its input FIFO
would take ~14 samples early, so the sequencer waits until the previous
frame has left the core). Frame period (processing (N/2) log2 N butterflies
at 2 clocks each, unloading N clocks; simulated):

    T = N / fs + (N log2 N + N + ~150..350) clocks
      = N / fs + 535 / 248 / 114 / 53 us at 100 MHz for N = 4096 / 2048 / 1024 / 512

| Source | fs | N | Resolution | Frame period | Spectra/s | Duty cycle |
|---|---|---|---|---|---|---|
| rx0 / rx1 | 12 MSPS | 4096 | 2.93 kHz | 876 us (simulated: 875.9) | 1142 | 39 % of the samples |
| rx0 / rx1 | 12 MSPS | 1024 | 11.7 kHz | 200 us (simulated: 199.75) | 5006 | 43 % |
| nbiq overview | 600 kSPS | 4096 | 146 Hz | 7.36 ms | 136 | 93 % |
| nbiq overview | 600 kSPS | 1024 | 586 Hz | 1.82 ms | 549 | 94 % |
| nbiq user channel | 48 kSPS | 4096 | 11.7 Hz | 85.9 ms | 11.6 | 99.4 % |
| nbiq user channel | 48 kSPS | 2048 | 23.4 Hz | 42.9 ms | 23.3 | 99.4 % |
| nbiq user channel | 48 kSPS | 1024 | 46.9 Hz | 21.45 ms (simulated at 47.96 kSPS: 21.45 ms) | 46.6 | 99.5 % |
| nbiq user channel | 48 kSPS | 512 | 93.8 Hz | 10.72 ms | 93.3 | 99.5 % |

An average of NAVG frames takes NAVG x T (plus, with `DOUBLE_BUF = 0` and
AXI-Lite readout, the time until the ARM clears DONE; one RESULT read is
~N AXI-Lite reads, about 1-2 ms from user space for 4096). With the stream
there is no gap: the next average starts when the previous one ends (the
packet goes out while the next frame is captured). At 12 MSPS the limit is
the FFT (a faster xfft architecture, radix-2 burst or radix-4, would cost
more LUTs/DSPs); at the nbiq rates it is the sample rate. For >= 30
spectra/s of a 48 kHz channel: N = 1024 (46.6/s, NAVG = 1) or 512.

Not built: 50 % overlapped frames (a 2048-sample capture buffer, ~2 BRAM36
and a few dozen LUTs: twice the rate at the same N).

## Result stream

With CTRL.STREAM each published result goes out on `m_axis` as one packet.
For the integrator:

| Port | Dir | |
|---|---|---|
| `m_axis_tdata[63:0]` | out | packet beat; byte 0 = [7:0] (little endian in memory) |
| `m_axis_tvalid` | out | |
| `m_axis_tready` | in | from the axi_dmac `s_axis_ready` (tie to 0 if unused) |
| `m_axis_tlast` | out | last beat of a packet |

Clock `s_axi_aclk` (the 100 MHz sys_cpu_clk), reset `s_axi_aresetn`; no
tkeep / tstrb / tuser (all bytes valid). In `fft_bd` the four ports form
the AXI4-Stream interface `m_axis` (associated with `s_axi_aclk`).

Packet (2 + N/2 beats, 16 + 4 N bytes: 4112 for N = 1024, 16400 for 4096):

| Beat | Bits | Field |
|---|---|---|
| 0 (HDR0) | [7:0] | MAGIC = 0xF5 |
| | [11:8] | LOG2N: N = 2^LOG2N result words follow |
| | [13:12] | SRC (0 rx0, 1 rx1, 2 nbiq) |
| | [18:16] | NB_CH |
| | [23:20] | RSHIFT |
| | [24] | FFT_OVF: FFT overflow in a frame of this average |
| | [25] | SAT: a word of this result saturated |
| | [26] | SKIP: results were dropped since the previous packet |
| | [63:32] | SEQ: sequence number (AVGS of this result; a gap = dropped results) |
| 1 (HDR1) | [23:0] | SCALE (RES_SCALE) |
| | [40:32] | NAVG (1..256) |
| 2 + m | [31:0] / [63:32] | RESULT[2m] / RESULT[2m + 1] |

Other bits are 0. In memory: u32 words 0..3 are the header (word 0:
MAGIC, LOG2N, ..., word 1: SEQ, word 2: SCALE, word 3: NAVG), word 4 + j
is RESULT[j] (FFT-shifted, as over AXI-Lite).

Back-pressure policy ("drop the waiting result, never cut a packet"):

- When a result is published, the core waits with tvalid low until it sees
  m_axis_tready high; then it sends the whole packet (tvalid held as AXI4-
  Stream requires, words read from the result buffer at up to 1 word per
  clock, 1 beat per 2 clocks, i.e. ~N clocks per packet: 41 us at 4096,
  10 us at 1024). axi_dmac raises s_axis_ready as soon as a transfer is
  queued, without waiting for tvalid; a slave in between (register slice,
  FIFO) must do the same.
- If tready stays low until the buffer is needed again, the result is
  dropped: STR_DROPS + 1, FLAGS.SDROP, SKIP in the next packet's header
  (SEQ jumps). The buffer is needed again with DOUBLE_BUF = 0 at the first
  FFT output of the next average (at least N / fs + the FFT processing
  after the end of this one: >= 200 us at 12 MSPS and N = 1024), with
  DOUBLE_BUF = 1 at the first FFT output of the next average's last frame.
  So an unqueued DMA costs results, never capture time: the averages run
  on at their normal rate (simulated).
- Once the first beat is out, the packet is completed. If the FFT output
  of the next average would overwrite the buffer before the last beat is
  accepted (a DMA stalled for longer than one frame), that output is held
  (xfft m_axis_data_tready low, the FFT pauses, the next frame is captured
  later); no data is lost or corrupted. In practice the packet takes 1-2 %
  of a frame period.
- CMD.STR_ABORT drops a waiting result, or ends a packet in progress at
  once without tlast (an AXI4-Stream violation: only with the DMA disabled,
  to recover). Both count as dropped. CMD.ABORT does not touch the stream.
- STATUS.STR_WAIT / STR_SEND show the state; STR_SENT counts packets.

AXI-Lite RESULT reads keep working during a packet: the result RAM's read
port is shared, an AXI-Lite read takes it for one clock (the packet skips
that clock). No extra block RAM: the packet is read from the result buffer
(the accumulator with DOUBLE_BUF = 0) into a 4-entry output FIFO (LUTs).

axi_dmac setup (stream -> memory, like `axi_dvbs2_rx_dma`): DMA_TYPE_SRC 1
(AXIS), DMA_TYPE_DEST 0 (MM), 64-bit both sides, CYCLIC 0,
SYNC_TRANSFER_START 0, `s_axis_aclk` = sys_cpu_clk. Software queues
transfers (up to the queue depth, typically 4) into a DDR ring with FLAGS
= TLAST (bit 1; optionally PARTIAL_REPORTING_EN, bit 2) and X_LENGTH =
16399 (PKT_BYTES_MAX - 1, room for N = 4096) or exactly 16 + 4 N - 1 when N
never changes. A packet shorter than the transfer (N < 4096) ends the
transfer early at tlast (axi_dmac partial transfer, `TRANSFER_DONE` set as
usual; the length is in the header, or PARTIAL_TRANSFER_LENGTH), so mixed
N need no reprogramming; slots of 16400 bytes (or 16 KiB + 16) per
transfer. Check MAGIC and use SEQ to detect drops.

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
- `m_axis_*`: the result stream (AXI4-Stream master, 64-bit, tlast,
  `s_axi_aclk`), see "Result stream".

## Resources and timing (out of context)

`syn/ooc.tcl` (xc7z020clg400-2, s_axi_aclk 100 MHz, adc_clk 250 MHz,
implemented):

| Build | LUT | (LUTRAM / SRL) | FF | BRAM tiles | DSP | WNS 100 MHz | WNS 250 MHz |
|---|---|---|---|---|---|---|---|
| DOUBLE_BUF = 0 | 1489 | 88 / 153 | 2392 | 10 (5 RAMB36 + 10 RAMB18) | 6 | +3.15 ns | +0.80 ns |
| DOUBLE_BUF = 1 | 1478 | 88 / 153 | 2428 | 14 (9 RAMB36 + 10 RAMB18) | 6 | +2.94 ns | +0.63 ns |
| 1.0 (4096 only, no stream), DOUBLE_BUF = 0 | 1101 | 44 / 154 | 2088 | 10 | 6 | +2.95 ns | +0.83 ns |
| 1.0, DOUBLE_BUF = 1 | 1104 | 44 / 154 | 2123 | 14 | 6 | +2.91 ns | +0.98 ns |

Split (DOUBLE_BUF = 0): xfft 750 LUT / 4.5 BRAM / 2 DSP (591 without the
run-time transform length), accumulator 266 LUT / 4.5 BRAM (4096 x 40) /
2 DSP, window 43 LUT / 1 BRAM / 2 DSP, ADC FIFO 92 LUT, registers,
sequencer and result stream 344 LUT (165 before; the stream's output FIFO
is 44 LUTRAM). The stream adds no BRAM: it reads the result buffer through
the port AXI-Lite uses. The 250 MHz path is the FIFO write side
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

Result stream (optional; without it tie `fft/m_axis_tready` to 0): an
axi_dmac stream -> memory as for dvbs2_rx. In the fishball design it is
`axi_fft_dma` at 0x7C4A0000, irq ps-13, with S_AXI_HP3 (64-bit) to itself
(`boards/fishball/README.md`, "AXI ports"):

```tcl
ad_ip_instance axi_dmac axi_fft_dma
ad_ip_parameter axi_fft_dma CONFIG.DMA_TYPE_SRC 1
ad_ip_parameter axi_fft_dma CONFIG.DMA_TYPE_DEST 0
ad_ip_parameter axi_fft_dma CONFIG.CYCLIC 0
ad_ip_parameter axi_fft_dma CONFIG.SYNC_TRANSFER_START 0
ad_ip_parameter axi_fft_dma CONFIG.AXI_SLICE_SRC 0
ad_ip_parameter axi_fft_dma CONFIG.AXI_SLICE_DEST 0
ad_ip_parameter axi_fft_dma CONFIG.DMA_2D_TRANSFER 0
ad_ip_parameter axi_fft_dma CONFIG.DMA_DATA_WIDTH_SRC 64
ad_ip_parameter axi_fft_dma CONFIG.DMA_DATA_WIDTH_DEST 64
ad_connect sys_cpu_clk axi_fft_dma/s_axis_aclk
ad_connect fft/m_axis axi_fft_dma/s_axis
ad_connect sys_cpu_clk axi_fft_dma/m_dest_axi_aclk
ad_connect sys_cpu_resetn axi_fft_dma/m_dest_axi_aresetn
# m_dest_axi -> S_AXI_HP3, address segment HP3_DDR_LOWOCM as for the
# other DMAs (see system_bd.tcl)
ad_cpu_interconnect 0x7C4A0000 axi_fft_dma
ad_cpu_interrupt ps-13 mb-13 axi_fft_dma/irq
```

The UIO node then gets the DMA registers and a reserved DDR ring as maps 1
and 2 (`dmac`, `ring` in regs/fft.toml; the fishball reserves 1 MB at
0x3EA00000); 16400 bytes per slot, e.g. 32 slots (525 KB). The
`axi_dmac` interrupt is not needed (software polls TRANSFER_DONE or uses
the fft DONE interrupt, which comes ~N clocks before the packet is in
memory).

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
		reg = <0x7c540000 0x10000>,     /* fft registers */
		      <0x7c4a0000 0x10000>,     /* axi_fft_dma */
		      <0x3ea00000 0x00100000>;  /* result ring (reserved-memory) */
		interrupt-parent = <&intc>;
		interrupts = <0 36 IRQ_TYPE_LEVEL_HIGH>;
	};
```

## Verification

`sim/test_fft.py` (pytest; runs the cases in parallel, about 16 minutes;
`FFT_CASES=len,str_snap` runs only those cases):

    cd fft/sim && ../../dvbs2/.venv/bin/python -m pytest -s test_fft.py

`xsim.py build` generates the xfft simulation model (Vivado batch,
`build_ip.tcl`), compiles the RTL, the testbench and the IP, and elaborates
two snapshots (DOUBLE_BUF = 0 and 1). Each case writes stimulus files and a
command script, runs xsim with the real xfft model, and checks the event
log. The ADC side runs at 250 MHz (4 ns) with a 12 MSPS valid strobe
(12 per 250 clocks); the nbiq tap carries all five channels interleaved in
random order, with same-channel runs (one sample per clock) and idle clocks,
or (case nb48) round robin with one beat every 417 clocks (47.96 kS/s per
channel). The m_axis sink takes beats with a set probability per clock
(command `K p c`: p %, for c packets; 0 packets = tready low as an axi_dmac
without a queued transfer) and checks the AXI4-Stream rules (tvalid held,
tdata / tlast stable until accepted).

Checks per result dump (AXI-Lite) and per stream packet:

- every captured frame is N consecutive samples of the selected source
  (located in the stimulus: proves the clock crossing, the source and
  channel select, and frame contiguity), frames in order, not overlapping;
- the N words equal the bit-exact model (window, Xilinx bit-accurate
  xfft C model with run-time length, |X|^2, sums, shift, saturation,
  FFT-shifted order);
- the words agree with numpy (float FFT of the same windowed samples
  divided by 2^shift, |.|^2, summed, shifted) within the derived tolerance;
- tones sit in the expected word (j = bin + N/2) at the expected level;
- RES_CFG (incl. LOG2N, per-result FFT_OVF / SAT) / RES_SCALE match the
  configuration of the average;
- packets: MAGIC, reserved bits 0, header = the configuration of the
  average with that SEQ, 2 + N/2 beats, FFT_OVF / SAT as the model, SKIP
  exactly where SEQ jumps, and the words equal an AXI-Lite dump of the same
  result where there is one.

Cases: rx0 (NAVG 1 and 4, RSHIFT 2, register reset values), rx1 (NAVG 3),
nbiq channels 3, 4, 0, RUN with acknowledge (both builds; back-to-back
timing with DOUBLE_BUF), dropped averages (DOUBLE_BUF: the result stays
intact while later averages run), source switching (CTRL during an average
applies to the next, ABORT restarts at once), FFT overflow flag and result
saturation, a window written over AXI (rectangular); and for 1.1:

- `len`: rx0 at N = 1024 (NAVG 1 and 4), rx1 at 2048 (NAVG 2), rx0 at 512,
  then back to 4096;
- `nb48`: nbiq channel 3 at 47.96 kS/s, N = 1024, RUN + STREAM, three
  packets, then an AXI-Lite dump of the last result: spectra/s >= 30;
- `str_snap`: STREAM snapshots at 4096 (sink 37 %) and 1024; the AXI-Lite
  dump of the same result runs while the packet is being sent (shared RAM
  read port);
- `str_run_sb`, `str_run_db` (both builds): RUN + STREAM at N = 1024, NAVG
  2: back to back; a slow sink (1-2 %: the FFT output is held, results
  stay exact); no sink (results dropped: STR_DROPS, FLAGS.SDROP, the
  averages go on back to back, LOST stays 0); the sink back (SKIP and the
  SEQ gap in the next header); STR_ABORT of a waiting result and of a
  packet in progress (cut without tlast, counted as dropped, the next
  packet intact with SKIP).

Results (2026-11-04): 18 tests, 14 cases, 29 AXI-Lite dumps and 23 stream
packets, all bit-exact with the model at every N; largest error / numpy
tolerance 0.22 (rectangular window) and 0.07..0.18 otherwise; tone levels
within 0.013 dB, peaks in the expected words; FFT overflow is flagged and
still bit-exact with the C model; saturated words and FLAGS.SAT match the
model; LOST never set; packets equal the AXI-Lite dumps of the same
result; no AXI4-Stream rule violated; the slow sink held the FFT output
for 17191 (DOUBLE_BUF = 0) / 26034 (1) clocks without any error; SEQ
1..5, 9..11, 14 sent (6..8 dropped without a sink, 12 and 13 aborted).
Simulated frame periods: 875.9 us at 12 MSPS and N = 4096 (874.25 with
the 1.0 core: the run-time length costs ~170 clocks per frame), 199.75 us
at N = 1024; nbiq at 47.96 kS/s and N = 1024: 21.45 ms = 46.6 spectra/s.

Tolerance (`fftspec.fft_err_sigma`, `power_tolerance`): radix-2 stage s
(0..11) rounds every butterfly output to the 16-bit grid after a shift of
b_s bits (convergent rounding: variance (1 - 4^-b_s)/12) and rounds the
twiddle product (1/12; not in stage 0). An error made in stage s reaches an
output through 2^(11-s) paths of the remaining stages, each scaled by
2^-(b_{s+1} + ... + b_11), so

    sigma^2 = sum_s [(1 - 4^-b_s)/12 + [s > 0]/12] * 2^(11-s) / 4^(b_{s+1} + ... + b_11)

per output component (0.54 LSB for 0x555400 and 0x555555; measured on the C
model: 0.41). For N < 4096 the same sum runs over the L = log2 N stages
(2^(L-1-s) paths; 0.54..0.59 LSB for the default schedules). The twiddles
(16-bit) add a relative error below L x 2^-15.
With E = 7 sigma sqrt(2) + 12 x 2^-15 |X| per complex output, a power sum
may differ from numpy by at most sum_f (2 |X_f| E + E^2), divided by
2^RSHIFT, plus 1 for the floor of the shift. Truncation instead of
convergent rounding (the IP option) would add a -0.93 LSB bias per
component and errors up to 6 LSB (+48 LUT, +92 FF for rounding).
