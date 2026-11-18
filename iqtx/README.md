# Narrowband IQ transmitter

A generic transmit path for narrowband modes made in software (SSB voice,
CW, FT8/FT4, test tones): the ARM writes complex baseband samples at a low
rate into a DDR ring, the PL interpolates them to the DAC rate (12 MS/s),
shifts them by an NCO and sends them to AD9363 TX1. It shares the DAC with
the DVB-S2 transmitter: one source at a time, chosen by a register in this
core.

```
 DDR ring --HP0--> prefetch --> envelope --> FIR x5 --> CIC x R --> NCO --> TX source mux --> DAC 0
 (int16 I/Q, 64-byte bursts)   (gain, ramp)  (120 taps) (4 stages)  (32-bit)    ^
                                                                                 dvbs2_tx output
```

| | |
|---|---|
| Input rate | 12 MS/s / (5 R), R = 8..400: 300 k (R 8), 240 k, 200 k, ..., 96 k (R 25), 60 k, 48 k (R 50), 40 k, 24 k (R 100), 12 k (R 200), ..., 6 k (R 400). Not 44.1 k or 192 k: resample in software |
| Passband | ±0.4 × input rate (80 %), flat to ±0.001 dB (all R) |
| Images | ≥ 0.6 × input rate from the carrier: below −78 dB (FIR stopband); CIC images below −84 dB. Worst spur of a full-scale passband tone at the DAC (images, NCO, rounding): −72 dBc, an NCO phase spur (`test_images` checks < −70 dBc at R 8..400) |
| NCO | 32-bit step, 12 MHz / 2^32 = 2.8 mHz resolution, phase-continuous retune; 4096-step phase (quarter-wave table of 1024), spurs about −72 dBc |
| Gain | Q2.14, 1.0 = an int16 full-scale input gives a 16-bit full-scale DAC word (the AD9363 takes the top 12 bits); up to 4.0 (clips at 18 bits before the FIR) |
| Ramp | raised cosine over 2^RLOG input samples on start and stop (default RLOG 8: 5.3 ms at 48 kS/s), from a 256-entry table |
| Underrun | empty ring: the sample is replaced by zero (band-limited by the filters, no clicks beyond the signal's band) and counted |
| Cost | out of context: 2.5k LUT, 1.8k FF, 10 DSP48, no BRAM (all tables in LUT ROM, buffers in LUTRAM), +2.57 ns at 100 MHz |

Arithmetic (bit-exact in `model/iqtx.py`):

1. envelope: `k = (GAIN * NORM[R]) >> 16`, `e = (k * r) >> 15`,
   `x' = sat18(rnd(x * e, 14))`; `r` is the ramp (0..32768), `NORM[R] =
   round(2^(16 + S) / R^3)` with `S = ceil(log2 R^3)` takes the CIC gain
   out, so the level does not depend on R;
2. FIR x5: `v[5n + p] = sat20(rnd(sum_j h[5j + p] x'[n - j], 16))`, 24
   taps per phase, Q2.16, linear phase, designed (least squares, Lawson
   iterations towards minimax) with 1/sinc^4 compensation of the CIC droop;
3. CIC x R: 4 combs at the FIR rate, zero insertion, 4 integrators (48-bit,
   wrap-around, each with a one-sample delay); `o = sat16(rnd(y, S))`;
4. NCO: `z = sat16(rnd(o (cos + j sin), 15))`, phase `t * FINC` from 0 at
   the start.

The engines are small and mostly idle: the FIR does one MAC per clock for I
and Q (one output per 29 clocks, at most 1.5 M outputs/s needed at R 8);
the integrators and the NCO make one output sample at most every other
clock (12 MS/s needs one per 8.3); a 16-entry FIFO and a 16-entry
dual-clock FIFO hand the samples to the DAC clock.

## Software interface

This is what the station daemon uses; keep it stable (bump VERSION on
changes). The register map is `regs/iqtx.toml` (generated: `regs/gen/iqtx.md`,
`regs/gen/iqtx_regs.h`).

**Device.** UIO device `iqtx` (`iqtx@7c550000`): map 0 = the registers
(0x7C550000, 64 KB), map 1 = the ring buffer (1 MB of reserved DDR at
0x3E700000, uncached), interrupt GIC SPI 56 (`ps-12`), level, from
`IRQ_STATUS & IRQ_EN`.

**Samples.** 32-bit words `{Q[15:0], I[15:0]}` (int16 I in the low half,
little-endian: an `int16_t iq[2]` pair), one word per input sample, in
ring order. A full-scale input is ±32767; with GAIN 1.0 it is full scale at
the DAC. Two tones: half the amplitude each.

**Ring.** `RING_BASE` = physical address (map 1's `addr`), `RING_MASK` =
size - 1 (the whole 1 MB: 0xFFFFF; any power of two ≥ 4096, base aligned
to it). `WPTR` (software) and `RPTR` (core) count bytes and wrap at 2^32; the
sample at byte count `c` is at `RING_BASE + (c & RING_MASK)`. The core
fetches 64-byte blocks (16 samples) while `(WPTR & ~63) - RPTR` is between
64 and the ring size, so

- write samples at `WPTR`, then advance `WPTR`; only whole 64-byte blocks
  are played (pad the end of a transmission with zeros);
- the fill is `WPTR - RPTR` bytes; everything below `RPTR` may be
  overwritten (the core holds up to 64 more samples internally);
- a `WPTR` behind `RPTR` or more than a ring ahead is ignored (nothing is
  fetched).

**Start.**

1. `CTRL = 0` and wait for `STATUS.BUSY = 0` (a previous run has ended);
2. `TX_SRC = 1` (IQ) and wait for `STATUS.ACTIVE = 1`. The switch waits
   until the active source (DVB-S2 TX: its output zero for 16 samples) and
   the requested one are both silent: stop the DVB-S2 transmitter first
   (dvbs2_tx `CTRL` 0, or its DAC select bit 2 cleared). While it is
   waiting, `STATUS.PENDING` is set;
3. `RATE`, `RAMP` (latched at the start), `GAIN`, `FINC`, `RING_BASE`,
   `RING_MASK`, `LOWATER`, `IRQ_EN`; the AD9361 core's DAC channel 0 must
   take its data from the PL (`DAC_DDS_SEL` = 2: write `0x418 0x2` and
   `0x458 0x2` to `direct_reg_access` of `cf-ad9361-dds-core-lpc`, as for
   DVB-S2 TX) and the AD9363 runs at 12 MS/s;
4. prefill: `RPTR` equals `WPTR & ~63` after a stop (0 after reset); write
   samples from there and advance `WPTR`;
5. `CTRL = 1`. The core starts at the first sample it fetches (ramping up
   over 2^RLOG samples; `STATUS.STARTED`). `CTRL = 1` before step 2 also
   works: the core waits (`STATUS.WAIT`) until IQ TX is the active source.

**Streaming.** Keep `WPTR - RPTR` above a margin (e.g. 100–250 ms of
samples): poll `RPTR` on a timer, or set `LOWATER` (bytes) and `IRQ_EN.LOW`
and refill on the interrupt (`IRQ_STATUS.LOW` is set when the fill falls
below `LOWATER`; write 1 to clear, then re-enable the UIO interrupt). An
empty ring gives zeros, `UNDERRUNS` counts them and `IRQ_STATUS.UNDERRUN` is
set; playing continues with the next samples written. `GAIN` and `FINC`
apply immediately (`FINC` phase-continuously). `SAMPLES` counts the input
samples played since the start (ramps and underrun zeros included).

**Stop.** `CTRL = 0`: the core ramps down over the next 2^RLOG samples from
the ring (zeros if it is empty, not counted), then plays 32 zero samples
(the filters' tail) and becomes idle: `STATUS.BUSY = 0`,
`IRQ_STATUS.IDLE`. Unplayed ring data is then dropped (`RPTR = WPTR &
~63`). To end a transmission exactly: write the last samples, wait until
`RPTR` reaches `WPTR`, then `CTRL = 0`. Afterwards give the DAC back with
`TX_SRC = 0` (DVB-S2), so DVB-S2 software that does not know this core
keeps working; `TX_SRC = 2` (off) sends zeros.

**Frequency.** The signal moves by `FINC / 2^32 * 12 MHz` (signed:
`FINC = round(f / 12e6 * 2^32)`), relative to the TX LO; keep the
signal plus offset within ±6 MHz (in practice within the AD9363's analog
bandwidth setting).

**Latency.** Ring to DAC: the fill, plus up to 64 samples in the prefetch
FIFO, plus about 12.5 input samples of filter delay (0.26 ms at 48 kS/s).

**Status** (`STATUS`): [1:0] ACTIVE source (0 DVB-S2, 1 IQ, 2 off), [2]
PENDING, [3] BUSY, [4] WAIT, [5] STOPPING, [6] DVB_SILENT, [7] STARTED.
`DAC_UNDERRUNS` should stay 0 (a sample missing at the DAC clock while
playing would be a core fault).

## TX source mux

The DAC channel 0 input is DVB-S2 TX (reset default, so existing software
works unchanged), IQ TX or zeros (`TX_SRC`). The selection changes only
while both the active and the requested source are silent: DVB-S2 TX when
its output has been zero for 16 DAC samples, IQ TX when idle (not running,
or enabled and waiting for the source), off always. A transmitting source
is therefore never cut and a running one never switched in mid-stream; a
request that cannot be honoured stays pending (`STATUS.PENDING`).

## Files

- `model/iqtx.py`: bit-exact model, the filter design and the tables
  (`python3 iqtx.py --gen` rewrites `rtl/gen/*.mem`; the FIR taps are
  committed and the model reads them back).
- `rtl/`: `iqtx_rd.sv` (ring reader, AXI3 64-bit read bursts, 64-sample
  prefetch FIFO), `iqtx_fir.sv` (FIR x5), `iqtx_cic.sv` (CIC, scaling, NCO,
  output FIFO), `iqtx_axi.sv` (registers, control and envelope, the DAC
  clock crossing and the TX source mux; register map in its header). `ip/iqtx_bd.v` is the
  block-design top. The clock-domain crossing uses `dvbs2/rtl/dvbs2_async_fifo.sv`.
- `sim/test_iqtx.py`: Verilator tests of the whole core against the model
  (every DAC sample of complete runs at R 8, 37, 50, 200 and 400 with AXI
  stalls; ramps; underrun and refill; a 4 KB ring refilled while playing,
  pointers across 2^32; the low-water, idle and underrun events; source
  switching with the DAC trace), and model checks (passband, images, NCO
  accuracy, ramp tables, keying splatter, NORM).
- `sw/iqtx.c`: board test tool: a tone, two tones or an int16 IQ file at an
  offset (`iqtx -r 48k -o 100k -T 700,1900 -n 10`).
- `syn/ooc.tcl`: out-of-context synthesis + implementation.

Tests: `python3 -m venv --without-pip --system-site-packages .venv &&
python3 -m pip --python .venv/bin/python install pytest` here, then
`.venv/bin/python -m pytest sim/test_iqtx.py` (about 10 s).

## Board

Registers 0x7C550000 (`ad_cpu_interconnect`), interrupt ps-12, ring reader
on S_AXI_HP0 (64-bit). Device tree: `iqtx-buffer@3e700000` (1 MB,
`no-map`) and the UIO node `iqtx@7c550000` (`boards/fishball/linux/`,
`boards/fishball/debian/dtb-add-plutosat.py`).
