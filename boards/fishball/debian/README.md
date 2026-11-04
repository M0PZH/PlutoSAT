# PlutoSAT on a fishball board running its own Debian

For boards that keep their Debian system (kernel 5.15, their u-boot and
BOOT.bin) instead of the buildroot firmware. Tested on a "FISH Ball
PlutoSDR Rev.A (Z7020/AD9361)" with Debian 12.

```
boards/fishball/debian/deploy.sh --boot root@board   # once: boot files, then reboot
boards/fishball/debian/deploy.sh root@board          # bitstream + tools (again after changes)
ssh root@board plutosat-pl-load                      # after every boot
ssh root@board dvbs2-selftest -p                     # loopback self-test
```

What it changes:

| | |
|---|---|
| `/lib/firmware/plutosat.bit.bin` | the PL design (`build/system_top.bit` in FPGA-manager format) |
| `/usr/local/sbin/plutosat-pl-load` | loads it at run time: unbinds the ADI drivers on the PL (ADC core, DDS core, I2C), programs the PL, binds them again |
| `/usr/local/bin/` | `dvbs2-tx`, `dvbs2-rx`, `nbiq`, `rx-awgn`, `ts-prbs`, `dvbs2-selftest`, built on the board (Debian's glibc is older than buildroot's) |
| `devicetree.dtb` (`--boot`) | + reserved memory for the DMA buffers (top 22 MB of RAM), + UIO nodes for the cores, - the IIO streaming DMAs (`dtb-add-plutosat.py`) |
| `uEnv.txt` (`--boot`) | + `uio_pdrv_genirq.of_id=generic-uio` on the bootargs |

`--boot` keeps the original boot files on the card as `*.pre-plutosat`;
copying them back undoes it. The DMA buffers must be reserved at boot (the
kernel has no dma-heap and no overlays), hence the reboot. A reboot also
brings back the boot bitstream until the PlutoSAT bitstream goes into
BOOT.bin.

The board's PS setup (its FSBL) matches the design: FCLK0 100 MHz, FCLK1
200 MHz, the same PL addresses for axi_ad9361 and the I2C, the S_AXI_HP
ports in 64-bit mode.

The PlutoSAT PL has no IIO streaming DMAs (`axi_ad9361_adc_dma` at
0x7C400000, `axi_ad9361_dac_dma` at 0x7C420000; see `../README.md`).
`dtb-add-plutosat.py` removes them from the device tree, together with the
`dmas` of the ADC and DDS core nodes: `cf_axi_adc` and `cf_axi_dds` then
probe without an IIO buffer and still tune the AD9363 digital interface,
release the core from reset and give register access. A board with the
old device tree must run `deploy.sh --boot` and reboot first;
`plutosat-pl-load` refuses to load the bitstream while the device tree
still lists those DMAs (binding a driver to the empty address range would
hang the bus). `plutosat-pl-load --iio-dma FILE` loads an older bitstream
that still has them on a board whose device tree still lists them.
