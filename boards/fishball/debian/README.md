# PlutoSAT on a fishball board running its own Debian

For boards that keep their Debian system (kernel 5.15, their u-boot and
BOOT.bin) instead of the buildroot firmware. Tested on a "FISH Ball
PlutoSDR Rev.A (Z7020/AD9361)" with Debian 12.

```
boards/fishball/debian/deploy.sh --boot root@board   # once: boot files, then reboot
boards/fishball/debian/deploy.sh root@board          # bitstream + tools (again after changes)
boards/fishball/debian/deploy.sh --station root@board  # + plutosatd, web UI, systemd service
ssh root@board plutosat-pl-load                      # after every boot (unless BOOT.bin has the bitstream, below)
ssh root@board dvbs2-selftest -p                     # loopback self-test
```

What it changes:

| | |
|---|---|
| `/lib/firmware/plutosat.bit.bin` | the PL design (`build/system_top.bit` in FPGA-manager format) |
| `/usr/local/sbin/plutosat-pl-load` | loads it at run time: unbinds the ADI drivers on the PL (ADC core, DDS core, I2C), programs the PL, binds them again |
| `/usr/local/bin/` | `dvbs2-tx`, `dvbs2-rx`, `nbiq`, `rx-awgn`, `ts-prbs`, `dvbs2-selftest`, built on the board (Debian's glibc is older than buildroot's) |
| `devicetree.dtb` (`--boot`) | + reserved memory for the DMA buffers and the LLR queue (top 24 MB of RAM), + UIO nodes for the cores, - the IIO streaming DMAs (`dtb-add-plutosat.py`) |
| `uEnv.txt` (`--boot`) | + `uio_pdrv_genirq.of_id=generic-uio` on the bootargs |
| `/usr/local/bin/plutosatd`, `/usr/local/bin/plutosat` (`--station`) | the station daemon and CLI (static armv7 musl builds) |
| `/usr/share/plutosat/web`, `/usr/share/plutosat/api.md` (`--station`) | the web UI (`station/web`, `npm run build`) and the API reference |
| `/etc/systemd/system/plutosatd.service` (`--station`) | `plutosatd -c /etc/plutosat/plutosatd.toml`, enabled, restarted on failure (at most 5 starts a minute: a PL without the PlutoSAT cores faults every start) |
| `/etc/plutosat/plutosatd.toml` (`--station`, only if absent) | from `plutosatd.toml`: web UI on 127.0.0.1:8080 (ssh tunnel), QO-100 IF, a random admin password printed once |

`plutosat-pl-load` stops the plutosatd service while it reprograms the PL
(the daemon's register accesses would fault) and starts it again after.
Anything else that drives the cores (`dvbs2-rx`, `dvbs2-selftest`, a second
plutosatd) needs the service stopped first: `systemctl stop plutosatd`.

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

## The bitstream in BOOT.bin

Built in the top directory (`make TARGET=fishball` steps, by hand):

```
source /opt/Xilinx/2026.1/Vivado/settings64.sh
ADI_IGNORE_VERSION_CHECK=1 make -C boards/fishball/hdl
cp boards/fishball/hdl/fishball.runs/impl_1/system_top.bit build/
cp boards/fishball/hdl/fishball.sdk/system_top.xsa build/
scripts/create_fsbl.sh build/system_top.xsa build/sdk build/sdk/fsbl/Release/fsbl.elf
make -C u-boot ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zynq_fishball_defconfig all
cp u-boot/u-boot build/u-boot.elf
printf 'the_ROM_image:\n{\n  [bootloader] build/sdk/fsbl/Release/fsbl.elf\n  build/system_top.bit\n  build/u-boot.elf\n}\n' > build/boot-pl.bif
bootgen -arch zynq -image build/boot-pl.bif -w -o BOOT.bin
```

The FSBL loads the PL before u-boot, so the kernel probes the PlutoSAT
design: every PL node of the device tree must exist in it. The original tree's
`mwipcore@43c00000` does not (a boot with the PL loaded by u-boot and that
node in the tree hung; not confirmed as the cause); remove it with
`fdtput -r devicetree.dtb /fpga-axi@0/mwipcore@43c00000`. The FSBL from
the PlutoSAT XSA programs the PS like the board's original one (MIO, PLLs,
clocks, DDR and peripherals: the same 197 register writes).

The fishball u-boot boots the buildroot firmware (`sdboot` loads an .itb).
For Debian, `uEnv.txt` (the original) gets one more line, run by `uenvboot`
before that:

```
uenvcmd=echo Booting Debian from SD ...; load mmc 0 ${fit_load_address} ${kernel_image} && load mmc 0 ${devicetree_load_address} ${devicetree_image} && setenv bootargs console=ttyPS0,115200 root=/dev/mmcblk0p2 rootwait rw uio_pdrv_genirq.of_id=generic-uio && bootm ${fit_load_address} - ${devicetree_load_address}
```

That u-boot reads its environment from the QSPI flash (0x100000) when the
CRC matches. A board whose QSPI holds another firmware's environment then
runs that one (`modeboot` not set, `run` fails at the prompt): reset it
from the u-boot prompt (`env default -a; saveenv`) or erase the
environment sector.
