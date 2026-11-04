# Loopback self-test: PL noise source and test tools

A noise source in the PL between the AD9363 receive data and everything
that uses it (the IIO capture path, `dvbs2_rx`, `nbiq`), plus tools that
run the DVB-S2 transmitter into the receiver over an AD9363 loopback and
measure packet errors against a set Es/N0.

```
 ts-prbs gen -> dvbs2-tx -> DAC ch0 -> AD9363 loopback ----------------+
                                       (digital: data port TX1 -> RX1)  |
                                       (rf: TX1 cable -> RX1)           |
 ts-prbs check <- dvbs2-rx <- rx0 + noise (rx_awgn, Es/N0) <- ADC ch0 <-+
```

## Noise source (`rtl/rx_awgn.sv`, `rtl/rx_awgn_axi.sv`)

| | |
|---|---|
| Generator | 8 xorshift32, one step per ADC clock; per component the sum of 16 random bytes (Irwin–Hall, excess kurtosis −0.075, tails cut at 6.9 σ), white, I and Q independent |
| Level | σ = 0.018042 × NSCALE ADC LSB per component, NSCALE 0..131071 (σ up to 2365 LSB) |
| Signal | rx0 gain Q2.14 (0..3.99) applied with the noise; rx1 passes unchanged |
| Meters | mean I² + Q² of rx0 before and after the noise, 2^16-sample windows |
| Latency | all four lanes and valids delayed by 6 ADC clocks (on or off) |
| Cost | ~0.9k LUT, 1.4k FF, 8 DSP; 250 MHz out of context (+0.40 ns) |

Registers: see the header of `rtl/rx_awgn_axi.sv` (UIO `rx-awgn`,
0x7C530000). `model/awgn.py` is bit exact; `sim/test_awgn.py` checks the RTL
against it (noise, gain, saturation, bypass, meters) and the noise
statistics.

## Tools (`sw/`, installed by the `selftest` package)

- `rx-awgn -e ESN0 -r SYMRATE`: measures the clean rx0 power, then sets
  gain and noise for that Es/N0 with signal + noise at −12 dBFS (`-T`), the
  way the AD9363 AGC would hold the level. `rx-awgn -s SIGMA` sets the
  noise directly, `rx-awgn -x` turns it off, `rx-awgn` alone reports the
  state and the measured powers. Es/N0 = P · fs / (2σ² · Rs), the same
  definition as the receiver model (`dvbs2/model/rxmodel.py: channel()`).
- `ts-prbs gen | check`: transport stream with a sequence number and an
  xorshift payload per packet; `check` counts good, bad and lost packets.
- `dvbs2-selftest`: the whole loop, one Es/N0 point after another; the
  receiver is restarted on the noisy signal for each point, so acquisition
  is part of the test.

```
dvbs2-selftest                              # digital loopback, QPSK 1/4, 1 MS/s
dvbs2-selftest -p -m qpsk1/2 -s 500000 -e "off 3 2 1" -t 30
dvbs2-selftest -o 20000                     # 20 kHz carrier offset at the TX NCO
dvbs2-selftest -L rf -f 2400250000 -a 40 -G 20 -d 3000   # cable + attenuator, 3 kHz LO offset
```

Digital loopback (`-L digital`, AD9363 BIST loopback 1) returns the DAC
words exactly, at TX1 → RX1 only; nothing is transmitted. The AD9363 has no
internal RF loopback (its TX monitor input needs the board's TX_MON pins
wired to the TX outputs and is a power monitor), so `-L rf` means an
external loop: TX1 cabled to RX1 through 30 dB or more of attenuation. It
exercises the LOs, the analog filters, gain control and the I/Q and DC
corrections; the noise source still adds the set Es/N0 on top (the RF
path's own noise is not counted in it). Without a cable it radiates.
