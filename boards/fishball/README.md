# fishball-sdr-7020 board files

Zynq-7020 (xc7z020clg400-2) + AD9363 (LVDS, 2R2T), 1 GB DDR3.

| | |
|---|---|
| `hdl/` | the PlutoSAT block design (`system_bd.tcl`), top, constraints; `make -C boards/fishball/hdl` |
| `linux/`, `u-boot/`, `br2/`, `buildroot/` | buildroot firmware (installed into the submodules by `install.sh`) |
| `debian/` | deploying onto a board that keeps its own Debian (see `debian/README.md`) |

## PL design

The AD9363 runs at a fixed 12 MS/s. Its samples go through `axi_ad9361`
(digital interface, IQ/DC correction) and `rx_awgn` (self-test noise
source) to the PlutoSAT cores only: `dvbs2_rx`, `nbiq`, `fft`. The transmit
side is `dvbs2_tx` on DAC channel 0. There is no PL FIR
decimator/interpolator and no IIO streaming (no `axi_ad9361_adc_dma` /
`util_cpack2`, no `axi_ad9361_dac_dma` / `util_upack2`). The AD9363 itself
stays fully under Linux control: LO, gains, AGC, rates and RF bandwidth
through `ad9361-phy`, the core registers through `cf-ad9361-lpc` and
`cf-ad9361-dds-core-lpc` (debugfs `direct_reg_access`). Those two drivers
probe without DMAs when their device tree nodes have no `dmas` (the ADI
drivers create no IIO buffer then); the devicetrees in `linux/` and from
`debian/dtb-add-plutosat.py` have none.

## AXI ports

| Port | Use | Width |
|---|---|---|
| M_AXI_GP0 | all AXI-Lite registers (`axi_ad9361`, the PlutoSAT cores, their DMAs, SPI, I2C) | 32 |
| M_AXI_GP1 | spare | |
| S_AXI_HP0 | reserved: future IQ transmit DMA (memory -> DAC, up to 12 MS/s IQ, narrow- or wideband) | 64 |
| S_AXI_HP1 | reserved: the second DATV receiver's LDPC LLR frame queue (DDR ring, ~20 MB/s read + write, ~2 MB) | 64 |
| S_AXI_HP2 | `nbiq` ring writer (`DW` 64), 5 channels into 4 MB @ 0x3EB00000 | 64 |
| S_AXI_HP3 | FFT result DMA `axi_fft_dma` (stream -> memory, 0x7C4A0000, 1 MB @ 0x3EA00000) | 64 |
| S_AXI_GP0 | DVB-S2 RX DMA `axi_dvbs2_rx_dma` (stream -> memory, 0x7C490000) | 32 |
| S_AXI_GP1 | DVB-S2 TX DMA `axi_dvbs2_dma` (memory -> stream, 0x7C480000) | 32 |
| S_AXI_ACP | spare | |

Use the HP ports at 64 bits. Their width is a PS setting (the AFI
registers), and the FSBL sets it. A board that boots its own FSBL, like the
fishball Debian, leaves all four at the 64-bit default. A 32-bit master
on an HP port would need those registers changed at run time.

PL interrupts in use: ps-7 fft, ps-8 nbiq, ps-9 DVB-S2 RX DMA, ps-10 DVB-S2 TX
DMA, ps-11 SPI, ps-13 FFT DMA, ps-15 I2C. Free: ps-0 to ps-6, ps-12, ps-14.

### Future IQ transmit

`dvbs2_tx` selects DAC channel 0's data with CTRL bit 2 ("DAC select"):
its own output, or `dac_in_i/q`. Today `dac_in_i/q` are tied to zero (they
used to come from `util_upack2`). An IQ transmit core (memory -> DAC, up to
12 MS/s) plugs in there: its DMA on S_AXI_HP0, its samples into
`dvbs2_tx/dac_in_i/q`. Those inputs are in the `l_clk` domain, one sample per
`dac_valid`. Alternatively it gets a mux of its own between
`dvbs2_tx/dac_out_i/q` and `axi_ad9361/dac_data_i0/q0`. In both cases
`axi_ad9361` DAC channel 0 must take its data from the PL input
(`DAC_DDS_SEL` = 2, which `dvbs2-tx` sets through `direct_reg_access`).
