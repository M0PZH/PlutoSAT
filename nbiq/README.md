# Narrowband IQ channels

Eight independent receive channels in the PL, each tuned and filtered on its
own and written to its own ring buffer in DDR. The ARM gets ready-made
narrowband IQ (SSB, CW, beacons, a transponder overview, ...) without
touching the 12 MSPS stream. The station plan: 48 kHz channels (R 125) for
the users and the background work (FT8, the recorder, the beacon lock) and a
600 kS/s overview of the QO-100 narrowband transponder (R 10) on channel 4,
all from rx1; any channel can take any rate and either receive channel.

```
 AD9363 rx0/rx1 --> NCO + mixer --> CIC (5 stages, /R) --> FIR (/2) --> DDR ring
   (per channel: source, frequency, R)        (shared engine)    `--> tap (PL FFT)
```

| | |
|---|---|
| Output rate | fs / (2R), R = 2..128. At 12 MSPS: 3 M (R 2), ..., 1 M (R 6), 750 k, 600 k, 500 k (R 12), ..., 100 k (R 60), ..., 48 k (R 125), 46.875 k (R 128) |
| Tuning | 32-bit NCO, anywhere in ±fs/2, phase-continuous retune |
| NCO | 4096-step phase (quarter-wave table of 1024 entries), spurs about −72 dBc |
| CIC | 5 stages, 53-bit state (exact up to R = 128) |
| Filter | CIC-compensated Kaiser lowpass (70 dB design), cutoff fout/2; taps from the FIR allocation (below) |
| Passband ripple | < 0.02 dB over 80 % of fout with 50 taps or more (0.17 dB with 34, 1.4 dB with 20) |
| Output | int16 I/Q pairs, gain 8 by default (12-bit full scale → 16384) |
| Cost | see "Resources" below; the fishball writes DDR through S_AXI_HP2 with `DW` 64 |
| Version | 2.1 (0x0002_0100): eight channels; the ring address is `RING_BASE + (WPTR & RING_MASK)` (2.0: `RING_BASE \| ...`, which needed `RING_BASE` aligned to the ring size) |

### FIR allocation

The FIR engine computes one tap pair per clock for all channels, jobs
(outputs) oldest first; a channel with P tap pairs (2P taps) at output rate
fout costs fout (P + 1) clocks per second, and the channels in use together
may take at most 90 % of the 100 MHz clock (`model/nbiq.py` `fir_check()`,
the bounds in the header of `rtl/nbiq_fir.sv`).

The filter quality depends on P only, not on the rate: 22 pairs (44 taps,
`FIR_NEED`) reach the design's full alias rejection over 80 % of the band
(60 dB; 49 dB at R 2, 55 dB at R 3, 57.5 dB at R 4, 59 dB at R 6, where the
CIC's own aliases limit it), about 45 pairs the full 53 dB over 90 %; more
taps buy little. So **every channel in use gets the same P, as many as
fit**: the narrow channels need few clocks for it, the wide ones (the
overview) take most of the engine. The rule (`allocate()` in the model,
`allocate()` in `sw/nbiq.c`; the station's driver must compute exactly the
same):

```
fouts = the output rates fs / (2 R) of the channels in use (any order)
P = floor(0.9 * 100e6 / sum(fouts) + 1e-9) - 1, clamped to 1..127
while P > 1 and not fir_check(fouts, [P] * k): P -= 1
error if not fir_check(fouts, [P] * k)
every channel in use: npairs = P, filter = design(fs, rate, npairs = P)

fir_check(fouts, npairs):                  # clk = 100e6, T_c = clk / fout_c
    S    = sum(npairs)
    load = sum(fout_c * (P_c + 1)) / clk           <= 0.9 (+1e-12)
    jq   = sum(floor((S - P_c + 2) / T_c) + 1)     <= 32
    buf  = S <= (128 - P_c) * T_c for every c
```

Only the sample-buffer bound can fail after the load bound, and only for P
above about 110 (e.g. 600 k + 48 k: 127 → 126). The FIFO then needs fewer
than 2k ≤ 16 entries (at most 13 over every mix of R). `nbiq -P
RATE,RATE,...` prints the allocation (`npairs`, `load`, `jq`) and is checked
against the model.

Taps and alias rejection (the worst alias landing within the given part of
the band, relative to DC, the CIC included: `alias_rejection()`; worst
channel of the plan; `model/nbiq.py` `plan_table()`):

| channels in use | taps | load | 70 % | 80 % | 90 % |
|---|---|---|---|---|---|
| 7 x 48 k + 600 k (station) | 190 | 90 % | 67 dB | 60 dB | 53 dB |
| 4 x 48 k + 600 k | 224 | 89 % | 67 dB | 60 dB | 53 dB |
| 7 x 48 k + 1000 k | 132 | 90 % | 66 dB | 59 dB | 53 dB |
| 7 x 48 k + 3000 k | 50 | 87 % | 55 dB | 49 dB | 25 dB |
| 4 x 48 k + 600 k + 1000 k + 250 k + 100 k | 82 | 90 % | 66 dB | 59 dB | 53 dB |
| 4 x 500 k + 4 x 48 k | 80 | 90 % | 67 dB | 60 dB | 53 dB |
| 8 x 600 k | 34 | 86 % | 67 dB | 38 dB | 17 dB |
| 5 x 1000 k | 34 | 90 % | 66 dB | 38 dB | 17 dB |
| 8 x 1000 k | 20 | 88 % | 33 dB | 21 dB | 13 dB |
| 8 x 48 k | 254 | 49 % | 67 dB | 60 dB | 54 dB |
| 600 k alone | 254 | 77 % | 67 dB | 60 dB | 53 dB |

(Up to 2.0 every one of the k channels in use got an equal share of the
clocks, `floor(0.9 × 100 MHz / k / fout) − 1` pairs: with eight channels
the overview would have had 34 taps and 38 dB at 80 %, a 3 MSPS channel
next to seven 48 k ones 4 taps.)

A driver that keeps a fixed plan (the station: channels are enabled and
disabled one at a time while the others run, so their filters must not
change) allocates once for the full plan and programs every channel with
that P; a subset of a plan always fits (fewer channels: less load, a
shorter FIFO, the same buffer bound or a looser one).

## Resources

Out of context (`syn/ooc.tcl`, 32-bit AXI master, 100 MHz), eight channels:
7995 LUT (508 as memory), 10350 FF, 8.5 BRAM tiles (2 RAMB36 + 13 RAMB18:
the FIR's sample and coefficient memories, one NCO table per channel, the
ring writer's staging buffer), 22 DSP, WNS +1.86 ns (five channels, 2.0:
6.3k LUT, 7.0k FF, 6.5 tiles, 16 DSP). Per channel about 520 LUT, 860 FF,
2 DSP and a RAMB18; the shared comb 1.7k LUT, FIR 1.0k, ring writer 0.6k
(its staging buffer in distributed RAM, as up to 2.0, cost 411 LUT more).

## Files

- `model/nbiq.py`: bit-exact model (vectorized, plus the scalar reference),
  the filter design and the FIR engine limits; writes `rtl/gen/nbiq_lut.mem`.
- `rtl/`: `nbiq_chan.sv` (NCO, mixer, CIC integrators; I and Q share the
  multipliers and adders on alternate clocks, so the ADC FIFO is popped at
  most every other clock), `nbiq_comb.sv` (the CIC combs and scaling, shared,
  delays in distributed RAM), `nbiq_fir.sv` (shared polyphase FIR),
  `nbiq_wr.sv` (staging, AXI3 bursts to the rings), `nbiq_axi.sv`
  (registers, ADC clock crossing, interrupt, tap; register map in its
  header, also in `regs/nbiq.toml`). `ip/nbiq_bd.v` is the block-design top.
- `sim/test_nbiq.py`: Verilator tests of the whole core (with an AXI3
  memory model, slave stalls, staggered enables, re-enable, full FIR load)
  against the model, the tap against the rings, the registers, and filter
  checks (tone gain, spurs, aliasing). The C and Python filter designs are
  checked against each other.
- `sw/nbiq.c`: the user-space tool.
- `syn/ooc.tcl`: out-of-context synthesis + implementation (resources, timing).

## Using it

```
nbiq -c 0:-205k:48k:rx1 -c 1:30k:48k:rx1 -c 2:95k:48k:rx1 -c 3:180k:48k:rx1 \
     -c 4:0:600k:rx1 -c 5:-120k:48k:rx1 -c 6:60k:48k:rx1 -c 7:250k:48k:rx1 \
     -o udp:192.168.2.10:7355
nbiq -P 48k,48k,48k,48k,600k,48k,48k,48k   # npairs 95 load 0.898560 jq 12
nbiq -c 2:0:1M -o - | ...          # one channel to stdout
nbiq -c 0:25k:200k -o /tmp/ch%d.cf32 -f cf32 -n 10
```

`OFFSET` is relative to the LO. Channel c goes to `PORT + c` with UDP
(1024 bytes per datagram for s16). The tool allocates the FIR taps and
designs the filters (`nbiq -D R` prints one), programs the channels, and copies each ring out every 10 ms
(interrupt); a reader that falls a whole ring behind skips ahead and counts
the lost samples.

Rings (UIO map 1, the 4 MB reserved buffer at 0x3EB00000, unchanged from
2.0): Q = the largest power of two ≤ size / 16 (256 KB), channels 0..3 a
ring of Q at c Q, channel 4 one of 8 Q (2 MB) at 4 Q, channels 5..7 one of
Q at (c + 7) Q; the last Q is spare. Put the widest channel on 4.

| ring | 48 k | 100 k | 250 k | 600 k | 1 M | 3 M |
|---|---|---|---|---|---|---|
| 256 KB (0..3, 5..7) | 1.4 s | 0.66 s | 0.26 s | 0.11 s | 66 ms | 22 ms |
| 2 MB (4) | 10.9 s | 5.2 s | 2.1 s | 0.87 s | 0.52 s | 0.17 s |

The readers copy out every 10 ms (interrupt), so these are the scheduling
margins. With the 4 MB base at 0x3EB00000 the rings happen to be aligned to
their size as well (the 2 MB ring at 0x3EC00000), so the layout would also
work with the 2.0 address rule; the 2.0 layout (channel 4's 2 MB ring at
0x3ED00000) was not aligned, so a 2.0 core wrote the upper half of that ring
over its lower half.

Timing: the k-th sample of a channel ends at ADC sample
`T0 + 2R (k + taps/2 - 1) + 2R - 1`, T0 being the channel's `T0` register
(the ADC sample counter when it was enabled).

## Tap

`tap_valid`, `tap_ch[2:0]` (channel 0..7), `tap_i[15:0]`, `tap_q[15:0]` (outputs of
`nbiq_axi` / `nbiq_bd`, `s_axi_aclk` domain, registered): every sample
that goes to the ring writer, one clock each, the same values as in the
ring. At most one sample per clock; channel c delivers one per 2 R_c ADC
samples. A consumer picks its channel by `tap_ch`; there is no
back-pressure.
