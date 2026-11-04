# Narrowband IQ channels

Four independent receive channels in the PL, each tuned and filtered on its
own and written to its own ring buffer in DDR. The ARM gets ready-made
narrowband IQ (SSB, CW, beacons, a second DATV look, ...) without touching
the 12 MSPS stream.

```
 AD9363 rx0/rx1 --> NCO + mixer --> CIC (5 stages, /R) --> FIR (/2) --> DDR ring
   (per channel: source, frequency, R)        (shared engine, 4 channels)
```

| | |
|---|---|
| Output rate | fs / (2R), R = 2..64. At 12 MSPS: 1 MSPS (R 6), 857 k, 750 k, ..., 250 k (R 24), 200 k (R 30), ..., 100 k (R 60) |
| Tuning | 32-bit NCO, anywhere in ±fs/2, phase-continuous retune |
| NCO | 4096-step phase (quarter-wave table of 1024 entries), spurs about −72 dBc |
| Filter | CIC-compensated Kaiser lowpass, cutoff fout/2, 42 taps at 1 MSPS, 88 at 500 k, 178 at 250 k, 254 below 220 k |
| Alias rejection | 80 % of fout: ≥ 59 dB; 70 %: ≥ 66 dB (all rates); at 1 MSPS the band edge (90 %) is only 20 dB |
| Passband ripple | < 0.03 dB over 80 % of fout |
| Output | int16 I/Q pairs, gain 8 by default (12-bit full scale → 16384) |
| Cost | 6.3k LUT, 22 DSP, 2 RAMB36 + 6 RAMB18 (out of context; DDR writes through S_AXI_GP0) |

The FIR engine computes one tap pair per clock for all four channels;
the driver keeps the sum of `fout × (taps/2 + 1)` below 90 % of the
100 MHz clock, so every channel can run at 1 MSPS at the same time.

## Files

- `model/nbiq.py`: bit-exact model (vectorized, plus the scalar reference)
  and the filter design; writes `rtl/gen/nbiq_lut.mem`.
- `rtl/`: `nbiq_chan.sv` (NCO, mixer, CIC integrators), `nbiq_comb.sv` (the CIC
  combs and scaling, shared), `nbiq_fir.sv` (shared polyphase
  FIR), `nbiq_wr.sv` (staging, AXI3 bursts to the rings), `nbiq_axi.sv`
  (registers, ADC clock crossing, interrupt; register map in its header).
  `ip/nbiq_bd.v` is the block-design top.
- `sim/test_nbiq.py`: Verilator tests of the whole core (with an AXI3
  memory model, slave stalls, staggered enables, re-enable) against the
  model, and filter checks (tone gain, spurs, aliasing). The C and
  Python filter designs are checked against each other.
- `sw/nbiq.c`: the user-space tool.

## Using it

```
nbiq -c 0:+1.2M:250k -c 1:-350k:100k:rx1 -o udp:192.168.2.10:7355
nbiq -c 2:0:1M -o - | ...          # one channel to stdout
nbiq -c 0:25k:200k -o /tmp/ch%d.cf32 -f cf32 -n 10
```

`OFFSET` is relative to the LO. Channel c goes to `PORT + c` with UDP
(1024 bytes per datagram for s16). The tool designs the filters (`nbiq -D R`
prints one), programs the channels, and copies each ring out every 10 ms
(interrupt); a reader that falls a whole ring (1 MB = 262 ms at 1 MSPS)
behind skips ahead and counts the lost samples.

Timing: the k-th sample of a channel ends at ADC sample
`T0 + 2R (k + taps/2 - 1) + 2R - 1`, T0 being the channel's `T0` register
(the ADC sample counter when it was enabled).
