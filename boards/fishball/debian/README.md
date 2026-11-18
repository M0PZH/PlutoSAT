# PlutoSAT on a fishball board running its own Debian

For boards that keep their Debian system (kernel 5.15, their u-boot and
BOOT.bin) instead of the buildroot firmware. Tested on a "FISH Ball
PlutoSDR Rev.A (Z7020/AD9361)" with Debian 12.

## The Debian packages

`build-debs.sh` builds them (armhf, Debian 12) into `build/debs/`;
`deploy.sh --debs root@board` builds and installs them over ssh:

```
boards/fishball/debian/deploy.sh --debs --boot root@board   # first time: + device tree, then reboot
boards/fishball/debian/deploy.sh --debs root@board          # updates
ssh root@board dvbs2-selftest -p                            # loopback self-test (systemctl stop plutosatd first)
```

| package | installs | depends on |
|---|---|---|
| `plutosat-firmware` | `/lib/firmware/plutosat.bit.bin` (`build/system_top.bit`, FPGA-manager format), `/usr/sbin/plutosat-pl-load`, `plutosat-pl-load.service` (enabled: loads the bitstream at every boot, before plutosatd), `/usr/share/plutosat-firmware/cores.txt` (the cores' VERSION registers), `pl-axi.txt` (the bitstream's AXI address map), `/usr/share/doc/plutosat-firmware/` (README.md: the device-tree requirements; `dtb-add-plutosat.py`) | - |
| `plutosat-tools` | `/usr/bin/`: `dvbs2-tx`, `dvbs2-rx`, `nbiq`, `iqtx`, `rx-awgn`, `ts-prbs`, `dvbs2-selftest`, `fft-spectrum`, `fft-dmatest` | `plutosat-firmware`, the cores (below); recommends `python3-numpy` |
| `plutosatd` | `/usr/bin/plutosatd`, `/usr/bin/plutosat`, `/usr/share/plutosat/web`, `plutosatd.service` (enabled, restarted), `/etc/plutosat/plutosatd.toml` (conffile, from `plutosatd.toml`), `/usr/share/plutosatd/plutosatd.toml.default`, `/usr/share/doc/plutosatd/api.md` | `plutosat-firmware`, the cores |
| `plutosat-boot` | optional, risky: BOOT.bin and `plutosat-boot` / `plutosat-boot-rollback` (below) | `plutosat-firmware` of the same version, `python3`, `device-tree-compiler` |

**Versions** come from `git describe` of this repository
(`0.39+161.gf821df4`; `+dirty` with local changes); plutosatd's adds the
station's commit (`+st139.g5154eb4`). **Compatibility**: plutosat-firmware
`Provides: plutosat-core-NAME (= VERSION)` for each core of its bitstream
(`dvbs2-rx (= 2.0.0)`, ..., from `regs/gen/*_regs.h`, the major version in
[31:16]); plutosatd and plutosat-tools, built against the same `regs/`,
depend on each core's major version (`>= 2`, `<< 3`): a register layout
change (a new major) needs a new daemon, and apt refuses the mismatch. The
daemon checks the minor versions at run time (features such as the second
receive slot).

**plutosatd's config** `/etc/plutosat/plutosatd.toml` is a conffile:
HTTPS on 0.0.0.0:443 (`tls = "auto"`: a self-signed certificate made next
to it at the first start), port 80 redirecting, `web_root =
/usr/share/plutosat/web`, and **transmitting disabled** (`[tx] enabled =
false`, `[nbtx] enabled = false`: the daemon leaves the transmit cores
alone and refuses `tx.arm` / `nbtx.arm` with a message naming the switch;
set `true` and `systemctl restart plutosatd` to transmit). The maintainer
scripts:

- **fresh install**: postinst replaces the `ADMIN_PASSWORD` placeholder by
  a random password (mode 0600) and prints it once; enables and starts the
  service;
- **upgrade**: dpkg's conffile rules (an edited config is kept; `deploy.sh`
  passes `--force-confdef --force-confold`, so a changed default never
  prompts and lands as `.dpkg-dist`); the service restarts;
- **an existing config no package owns** (from `deploy.sh --station` or by
  hand): preinst moves it aside, postinst puts it back over the packaged
  one, byte for byte (same inode), so dpkg never asks and the admin's file
  wins; the default stays readable in `/usr/share/plutosatd/`;
- **remove**: stops and disables the service; the config stays;
- **purge**: removes `/etc/plutosat` (the config and the generated TLS
  certificate; other files of your own there are kept and listed);
  recordings in `/var/lib/plutosat` are kept.

**plutosat-firmware's postinst** enables `plutosat-pl-load.service` and
only checks `/proc/device-tree`: the reserved memory, the UIO nodes,
`uio_pdrv_genirq.of_id=generic-uio` on the command line, and no PL node
outside the bitstream's address map (IIO DMAs, `mwipcore`); it warns
loudly when something is missing and changes nothing. It does not reload
the PL: the new bitstream loads at the next boot, or now with
`plutosat-pl-load`. It never touches the boot partition.

**The C tools** are cross-built here with the host's
`arm-linux-gnueabihf-gcc`, whose glibc (2.43) is newer than Debian 12's
(2.36), and linked dynamically: `glibc-compat.c` pins the three symbols
that would need glibc >= 2.38 (`__isoc23_strtol` / `__isoc23_strtoul`, the
C23 redirects of `_GNU_SOURCE`, and libm's new `fmod`) to their GLIBC_2.4
versions, and `build-debs.sh` fails if any tool still needs a symbol above
GLIBC_2.36. (Static linking was tried: host lookups through NSS modules,
e.g. a DNS or mDNS name for `nbiq -o udp:HOST:PORT`, crash a static glibc
2.43 binary on the board.)

**build-debs.sh** needs `dpkg-deb`, `fakeroot`, `bootgen` (Vivado
settings), `arm-linux-gnueabihf-gcc`, and for plutosatd `cargo` (target
`armv7-unknown-linux-musleabihf`), `npm` and the RADE sources
(`station/rust/crates/plutosat-rade/fetch-sources.sh`, run by it).
`--no-station`, `--no-tools`, `--no-firmware`, `--no-boot` / `--boot`
select packages; the environment variables are in its header.

**Migrating from the file-copy deployment** (`deploy.sh --debs` does it):
the files an earlier `deploy.sh` put there by hand move to
`/root/pre-deb/DATE/` (`/usr/local/bin/{plutosatd,plutosat,dvbs2-tx,dvbs2-rx,nbiq,iqtx,rx-awgn,ts-prbs,dvbs2-selftest,fft-spectrum}`,
`/usr/local/sbin/plutosat-pl-load`, `/etc/systemd/system/plutosatd.service`
(it would override the packaged unit), `/usr/share/plutosat`), the service
is disabled, and apt installs the packages; `/etc/plutosat/plutosatd.toml`
stays.

## plutosat-boot (optional, risky)

Puts the PlutoSAT boot files on the SD card's boot partition, so that the
PL holds the PlutoSAT design from power-on: BOOT.bin (FSBL + bitstream +
the fishball u-boot), the board's **current** `devicetree.dtb` through
`dtb-add-plutosat.py --pl-ranges` (reserved memory, UIO nodes; no IIO DMAs
and no PL node the bitstream lacks, e.g. `mwipcore@43c00000`), and
`uEnv.txt` plus the Debian `uenvcmd=` line below when it has none. **A bad
boot file makes the board unreachable until someone takes the SD card
out**; nothing depends on this package (`plutosat-firmware` loads the
bitstream at every boot with any BOOT.bin).

Its postinst runs `plutosat-boot install`, which refuses (non-zero; the
package stays unconfigured, nothing is written) unless the board is a
fishball Zynq-7020; `/dev/mmcblk0p1` is FAT with `BOOT.bin`,
`devicetree.dtb` and `uEnv.txt` and room for the new files and backups;
every PL node of the resulting device tree is in the bitstream's address
map (`pl-axi.txt`, made from `system_bd.tcl` and the XSA); the u-boot
environment in QSPI (mtd `qspi-uboot-env`), which the fishball u-boot uses
whenever its CRC is valid, boots through `uEnv.txt` (else: `env default
-a; saveenv` at the u-boot prompt; the tool never writes the flash); and
an existing `uenvcmd=` boots Debian with this device tree and the
generic-uio bootargs. `PLUTOSAT_BOOT_FORCE=1` overrides, at your own risk.
The partition is mounted read-only for all this, read-write only to
write. Files whose content would not change are not touched; when none
changes, nothing happens. Otherwise the current three files are kept as
`NAME.pre-plutosat-VERSION-DATE` and `NAME.plutosat-prev`, and each new
one is written to a temporary name, fsynced, renamed and read back from
the card (SHA-256). It never reboots.

`plutosat-boot check` and `plutosat-boot install --dry-run` say what
would change; `plutosat-boot status` shows the hashes.

**Undo**: `plutosat-boot-rollback` restores the `*.plutosat-prev` set
(same safety; the replaced files kept as `*.pre-rollback-DATE`). Removing
or purging the package does not touch the boot partition. **When the board
does not boot**: take the SD card out, and on another computer copy, on
its first partition (FAT), `BOOT.bin.plutosat-prev` over `BOOT.bin`,
`devicetree.dtb.plutosat-prev` over `devicetree.dtb` and
`uEnv.txt.plutosat-prev` over `uEnv.txt` (or an older
`*.pre-plutosat-*` set).

Building it needs the XSA of the same build as the bitstream
(`build/system_top.xsa`; checked: the same configuration data). The FSBL
is built from it (`scripts/create_fsbl.sh`, in the fixed directory
`/tmp/plutosat-fsbl`: its path ends up in the FSBL's assert strings, a
fixed one keeps BOOT.bin reproducible); the XSA's PS setup must equal
`ps7_init.ref` (the 675 register operations of the known-good FSBL,
`ps7-writes.py`), and with `BOOT_REF=` a known-good BOOT.bin, its FSBL
must contain every ps7_init table of the new one (the partitions are
compared too). u-boot is built from `u-boot/` with
`zynq_fishball_defconfig` (or `UBOOT_ELF=`, checked to be the pinned
commit's fishball build).

## Without packages (older)

```
boards/fishball/debian/deploy.sh --boot root@board   # once: boot files, then reboot
boards/fishball/debian/deploy.sh root@board          # bitstream + tools (again after changes)
boards/fishball/debian/deploy.sh --station root@board  # + plutosatd, web UI, systemd service
ssh root@board plutosat-pl-load                      # after every boot (unless BOOT.bin has the bitstream, below)
```

What it changes:

| | |
|---|---|
| `/lib/firmware/plutosat.bit.bin` | the PL design (`build/system_top.bit` in FPGA-manager format) |
| `/usr/local/sbin/plutosat-pl-load` | loads it at run time: unbinds the ADI drivers on the PL (ADC core, DDS core, I2C), programs the PL, binds them again |
| `/usr/local/bin/` | `dvbs2-tx`, `dvbs2-rx`, `nbiq`, `iqtx`, `rx-awgn`, `ts-prbs`, `dvbs2-selftest`, built on the board (Debian's glibc is older than buildroot's) |
| `devicetree.dtb` (`--boot`) | + reserved memory for the DMA buffers and the LLR queue (top 24 MB of RAM), + UIO nodes for the cores, - the IIO streaming DMAs, - `mwipcore@43c00000` (`dtb-add-plutosat.py`) |
| `uEnv.txt` (`--boot`) | + `uio_pdrv_genirq.of_id=generic-uio` on the bootargs |
| `/usr/local/bin/plutosatd`, `/usr/local/bin/plutosat` (`--station`) | the station daemon and CLI (static armv7 musl builds) |
| `/usr/share/plutosat/web`, `/usr/share/plutosat/api.md` (`--station`) | the web UI (`station/web`, `npm run build`) and the API reference |
| `/etc/systemd/system/plutosatd.service` (`--station`) | `plutosatd -c /etc/plutosat/plutosatd.toml`, enabled, restarted on failure (at most 5 starts a minute: a PL without the PlutoSAT cores faults every start) |
| `/etc/plutosat/plutosatd.toml` (`--station`, only if absent) | from `plutosatd.toml`: web UI on https://board/ (port 443, self-signed certificate from `tls = "auto"`, port 80 redirects), QO-100 IF, transmitting disabled, a random admin password printed once |

## Notes

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
old device tree must run `deploy.sh --boot` (or install `plutosat-boot`)
and reboot first;
`plutosat-pl-load` refuses to load the bitstream while the device tree
still lists those DMAs (binding a driver to the empty address range would
hang the bus). `plutosat-pl-load --iio-dma FILE` loads an older bitstream
that still has them on a board whose device tree still lists them.

## The bitstream in BOOT.bin

The `plutosat-boot` package (above) does this with checks, backups and a
rollback. By hand, in the top directory (`make TARGET=fishball` steps):

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
node in the tree hung; not confirmed as the cause); `dtb-add-plutosat.py`
removes it (and with `--pl-ranges` every PL node outside the bitstream's
address map), or `fdtput -r devicetree.dtb /fpga-axi@0/mwipcore@43c00000`. The FSBL from
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
