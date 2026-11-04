# Narrowband IQ channels

Five independent receive channels in the PL, each tuned and filtered on its
own and written to its own ring buffer in DDR. The ARM gets ready-made
narrowband IQ (SSB, CW, beacons, a transponder overview, ...) without
touching the 12 MSPS stream. The station plan: four 48 kHz user channels
(R 125) and a 500 kHz overview of the QO-100 narrowband transponder (R 12),
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
| Filter | CIC-compensated Kaiser lowpass, cutoff fout/2; taps from the FIR budget (below) |
| Passband ripple | < 0.03 dB over 80 % of fout (0.17 dB at 1 MSPS with five channels in use) |
| Output | int16 I/Q pairs, gain 8 by default (12-bit full scale → 16384) |
| Cost | 6.3k LUT, 7.0k FF, 16 DSP, 2 RAMB36 + 9 RAMB18 (out of context, +1.86 ns at 100 MHz; DDR writes through S_AXI_GP0) |

Taps and alias rejection (worst alias at the edge of the given part of the
band) with all five channels in use; with fewer channels the share is
larger (four in use: 42 taps at 1 MSPS, 88 at 500 k):

| fout | taps | 70 % | 80 % | 90 % |
|---|---|---|---|---|
| 1 MSPS (R 6) | 34 | 66 dB | 38 dB | 16 dB |
| 600 k (R 10) | 58 | 67 dB | 60 dB | 30 dB |
| 500 k (R 12) | 70 | 67 dB | 60 dB | 40 dB |
| 250 k (R 24) | 142 | 67 dB | 60 dB | 54 dB |
| ≤ 141 k (R ≥ 43) | 254 | 67 dB | 60 dB | 54 dB |

The FIR engine computes one tap pair per clock for all channels, jobs
(outputs) oldest first. The driver gives each of the k channels in use
`npairs = floor(0.9 × 100 MHz / k / fout) − 1` tap pairs (at most 127), so
the sum of `fout × (npairs + 1)` stays below 90 % of the clock, and checks
the limits of the job FIFO and the sample buffers (`model/nbiq.py`
`fir_check()`, explained in the header of `rtl/nbiq_fir.sv`); the share
always meets them. The station plan uses 43 % of the engine (4 × 48 k ×
128 + 500 k × 36 clocks per second).

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
     -c 4:0:500k:rx1 -o udp:192.168.2.10:7355
nbiq -c 2:0:1M -o - | ...          # one channel to stdout
nbiq -c 0:25k:200k -o /tmp/ch%d.cf32 -f cf32 -n 10
```

`OFFSET` is relative to the LO. Channel c goes to `PORT + c` with UDP
(1024 bytes per datagram for s16). The tool designs the filters (`nbiq -D R`
prints one), programs the channels, and copies each ring out every 10 ms
(interrupt); a reader that falls a whole ring behind skips ahead and counts
the lost samples.

Rings (UIO map 1, the 4 MB reserved buffer): channels 0..3 have 512 KB
each, channel 4 has 2 MB (put the widest channel there): 2.7 s at 48 kHz,
1 s at 500 kHz (131 ms for a 1 MSPS channel on 0..3). In general Q =
the largest power of two ≤ size / 8: 4 × Q, then 4 Q.

Timing: the k-th sample of a channel ends at ADC sample
`T0 + 2R (k + taps/2 - 1) + 2R - 1`, T0 being the channel's `T0` register
(the ADC sample counter when it was enabled).

## Tap

`tap_valid`, `tap_ch[2:0]`, `tap_i[15:0]`, `tap_q[15:0]` (outputs of
`nbiq_axi` / `nbiq_bd`, `s_axi_aclk` domain, registered): every sample
that goes to the ring writer, one clock each, the same values as in the
ring. At most one sample per clock; channel c delivers one per 2 R_c ADC
samples. A consumer picks its channel by `tap_ch`; there is no
back-pressure.
