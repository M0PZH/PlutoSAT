# Buildroot firmware for the fishball-sdr-7020

ADI's PlutoSDR firmware (busybox init, initramfs, libiio/iiod, the USB
gadget with RNDIS/NCM + mass storage) for the fishball board, with the whole
PlutoSAT stack:

- the PlutoSAT PL bitstream, loaded by u-boot from the FIT image before the
  kernel;
- the device tree `../linux/zynq-fishball-sdr-7020.dts` (reserved DMA
  memory, UIO nodes for the cores, no IIO streaming DMAs, no PL node the
  bitstream lacks);
- the C tools (`dvbs2-tx`, `dvbs2-rx`, `nbiq`, `iqtx`, `rx-awgn`,
  `ts-prbs`, `dvbs2-selftest`; br2 packages in `../br2/package`). Not the
  Python board tests `fft-spectrum` / `fft-dmatest` (no Python in the image);
- the station: `plutosatd` (web UI on `https://BOARD/`), the CLI
  `plutosat`, the web front end (br2 package `plutosat-station`).

It boots from the SD card. The full image does not fit the 14 MB QSPI
partition (see "Size").

## Build

In the top directory, with the submodules `linux`, `u-boot`, `buildroot`,
`br2-external` and `station` checked out (`hdl` only for a bitstream build):

```
make TARGET=fishball SKIP_LEGAL=1 build/fishball.itb
```

- **Bitstream:** `build/system_top.xsa` and `build/system_top.bit` from a
  PL build (`make -C boards/fishball/hdl`; the `.bit` is the XSA's). When
  both are there, make does not start Vivado.
- **Station:** make first runs `boards/fishball/br2/build-station.sh` into
  `build/station`, the recipe of the Debian package (`../debian/build-debs.sh`):
  `cargo build --release --target armv7-unknown-linux-musleabihf -p plutosatd
  -p plutosat-cli` with `PLUTOSAT_REGS`/`PLUTOSAT_FW` pointing here, then
  `npm ci && npm run build` in `station/web`. It needs cargo with the
  armv7 musl and wasm32 targets, `arm-linux-gnueabihf-gcc` and node
  (`NODE_DIR=`, default nvm v22.11.0). RADE: `RADE_DL=DIR` is the download
  cache of `fetch-sources.sh` (pinned SHA-256); `PLUTOSAT_RADE_REQUIRED=0`
  builds without RADE.
- **Buildroot:** `BR2_DL_DIR` may point at an existing download cache.
  The local packages (C tools, station) are rebuilt by every image build
  (`BOARD_BR2_LOCAL_PKGS`, `scripts/fishball.mk`).
- **Host:** on a host whose `dd` is uutils (Ubuntu 26.04), u-boot's SPL
  step (`dd conv=block,sync`) fails; put GNU coreutils first on `PATH`
  (Ubuntu's `gnu-coreutils` installs them as `gnudd` etc.: a directory of
  links without the prefix).
- `build/fishball.frm` (the USB mass-storage update file for the QSPI) is
  refused when the itb is larger than the partition (`FIT_MAX_SIZE`).

## SD card

The card's first partition (FAT) needs:

| file | |
|---|---|
| `BOOT.bin` | FSBL + u-boot (+ the bitstream, optional): any fishball BOOT.bin with the fishball u-boot, e.g. the one of the `plutosat-boot` package or `make sdimg` |
| `fishball.itb` | the FIT image: device tree, bitstream, kernel, initramfs |
| `uEnv.txt` | `uEnv.txt` here: its `uenvcmd` loads `fishball.itb` with the PlutoSAT kernel command line |

and the boot-mode straps on SD. The fishball u-boot uses the environment
saved in the QSPI flash (0x100000) when its CRC matches, else its default;
both `sdboot`s, its own and the Pluto-style one such a saved environment
may have (pluto2 has one), import `uEnv.txt` and run `uenvcmd` first, so
`uenvcmd` decides the boot either way. The kernel command line must
have `uio_pdrv_genirq.of_id=generic-uio` (UIO devices for the cores; the
fishball u-boot's own `sdboot` / `qspiboot` add it too). `uEnv.txt` does
not run `fishball_loadvals`: the u-boot variables `mode`, `attr_name` /
`attr_val` and `ad936x_ext_refclk_override` have no effect (the device tree
is 2R2T).

`make TARGET=fishball sdimg` puts `BOOT.bin` (FSBL + u-boot, made from
the XSA), `fishball.itb` and `uEnv.txt` into `build/sdimg/`.

## What runs

busybox init, ADI's Pluto scripts (`br2-external/board/adi/pluto`, copied
to `board/adi/fishball` by `../install.sh`), plus:

| script | |
|---|---|
| `S30sdcard` | mounts the SD card's FAT partition read-write at `/mnt/sd` (`flush`, root-only); adds `/mnt/sd/plutosat/authorized_keys` to root's ssh keys |
| `S40network` (Pluto) | `usb0` 192.168.2.1 (USB RNDIS/NCM), `eth0` (GEM0) with DHCP, or static with `fw_setenv ipaddr_eth` / `netmask_eth`; host name from `fw_printenv hostname` |
| `S41network` (Pluto) | udhcpd on `usb0`; busybox httpd with the Pluto info page, moved to **port 8080** (port 80 is plutosatd's redirect) |
| `S44eth0` | a background DHCP client on `eth0` when none got a lease at boot (late cable or server) |
| `S49ntpd` | busybox ntpd (no RTC on the board): `pool.ntp.org`, or `ntp.conf` in the PlutoSAT directory |
| `S95plutosatd` | the station daemon, see below |
| `S99tryboot` | `plutosat.reboot_after=SECONDS` on the kernel command line: reboot after that time unless `/tmp/plutosat-stay` exists (unattended test boots, below) |

Root login: `root` / `analog` (Pluto default), console on `ttyPS0`, ssh
(dropbear), iiod for libiio over USB and the network.

### plutosatd

`/etc/init.d/S95plutosatd` starts `/usr/sbin/plutosatd-run`, which waits
for the UIO device `dvbs2-rx` and the IIO device `ad9361-phy` (30 s), and
up to 20 s for an `eth0` address, then runs `plutosatd -c
PDIR/plutosatd.toml` with its output in syslog (`logread | grep plutosatd`)
and restarts it after a crash (at most 5 starts a minute). Stop it before
driving the cores with the C tools: `/etc/init.d/S95plutosatd stop`.

The persistent PlutoSAT directory `PDIR` is the first of:

| `PDIR` | when | recordings (`[recorder] dir`, `max_mb`) |
|---|---|---|
| `/mnt/sd/plutosat` | an SD card (also readable on a PC) | `/mnt/sd/plutosat/recordings`, 2000 MB |
| `/mnt/jffs2/plutosat` | no SD card: the QSPI `qspi-nvmfs` partition (jffs2, 896 KB, also Pluto's saved passwords and keys) | `/tmp/plutosat-recordings` (RAM, lost at reboot), 64 MB |
| `/var/lib/plutosat` | neither (RAM, lost at reboot) | as above |

`/etc/plutosat` links to it. It holds:

- `plutosatd.toml`, made at the first start from
  `/usr/share/plutosatd/plutosatd.toml.default`: the Debian package's
  configuration (`../debian/plutosatd.toml`: HTTPS on 0.0.0.0:443 with
  `tls = "auto"`, port 80 redirects, QO-100 IF, **transmit disabled**) plus
  a `[recorder]` section. The random admin password is printed on the
  console then and stays in the file (`[auth] admin_password`). After
  editing: `/etc/init.d/S95plutosatd restart`.
- `plutosatd-tls-cert.pem`, `plutosatd-tls-key.pem`,
  `plutosatd-tls-names.txt`: the self-signed certificate plutosatd makes
  (for the host name, `NAME.local` and the interface addresses).
- optional: `disabled` (plutosatd not started), `ntp.conf`,
  `authorized_keys`.

The recorder's channels are off by default. With them on, the overs go to
the SD card's boot partition: a crash while writing can damage that FAT
file system, so keep a backup of the boot files.

No other configuration change of the daemon is written anywhere: admin
changes over the API last until the daemon restarts (api.md).

## Size

The QSPI `qspi-linux` partition (0x200000..0x1000000) holds 14 MB
(0xE00000 = 14 680 064 bytes); u-boot's `read_sf` reads at most that. The
full image does not fit (2026-11-19, bitstream eca32cd, station ed776ac):

| part of `fishball.itb` | bytes |
|---|---|
| zImage (gzip) | 5 007 560 |
| bitstream (`.bit`, uncompressed) | 2 947 215 |
| device tree | 23 395 |
| initramfs (xz) | 10 551 048 |
| of which the station (plutosatd, plutosat, web UI; xz) | about 5 240 000 |
| **fishball.itb** | **18 531 367** |

## A QSPI variant (not built)

Ways to save, measured on that image:

| | saves |
|---|---|
| the station left out (`BR2_PACKAGE_PLUTOSAT_STATION` off, no `build/station` prerequisite): C tools only | 5.24 MB |
| the bitstream gzip-compressed in the FIT (`compression = "gzip"` for `fpga@1` in `scripts/fishball.its`; u-boot decompresses FPGA images to their load address) | 1.44 MB |
| the kernel XZ- instead of gzip-compressed (`CONFIG_KERNEL_XZ`) | about 1 MB (estimate) |
| no Wi-Fi firmware and wpa_supplicant (ADI's defconfig has them for USB dongles) | 0.5 MB |

Without the station the image is 13.3 MB and fits as it is (with the
compressed bitstream 11.8 MB). With the station, all of the others
together still leave it at about 15.6 MB: a QSPI image with plutosatd needs
the web UI (2 to 3 MB xz) or RADE out of it as well. The proposal is a
"tools" QSPI image (no station) next to the full SD card image.

## One unattended test boot from Debian

A board that boots Debian from the SD card (the fishball u-boot with the
`uenvcmd` of `../debian/README.md`) can boot this image once and come back
to Debian without anyone at the board. The trigger is the Zynq
REBOOT_STATUS register (0xF8000258): Linux's restart handler writes
`/sys/kernel/debug/zynq_rst/code` into its low 16 bits, which survive a
soft reset (BootROM and FSBL keep them; ADI's Pluto uses the same for its
DFU modes). Nothing is written to flash at boot.

1. Copy `fishball.itb` to the FAT partition, and Debian's
   `/root/.ssh/authorized_keys` to `plutosat/authorized_keys` on it.
2. Back up `uEnv.txt`, then replace its `uenvcmd=` line by (one line):

   ```
   uenvcmd=setexpr plutosat_rc *f8000258 % 10000; if test "${plutosat_rc}" = "b0"; then mw f8000008 df0d; mw f8000258 00400000; mw f8000004 767b; echo PlutoSAT: one boot of fishball.itb ...; load mmc 0 0x2080000 fishball.itb && setenv bootargs console=ttyPS0,115200 maxcpus=2 rootfstype=ramfs root=/dev/ram0 rw earlyprintk clk_ignore_unused uio_pdrv_genirq.of_id=generic-uio panic=10 plutosat.reboot_after=900 && bootm 0x2080000#config@0; echo fishball.itb did not boot; fi; echo Booting Debian from SD ...; load mmc 0 ${fit_load_address} ${kernel_image} && load mmc 0 ${devicetree_load_address} ${devicetree_image} && setenv bootargs console=ttyPS0,115200 root=/dev/mmcblk0p2 rootwait rw uio_pdrv_genirq.of_id=generic-uio && bootm ${fit_load_address} - ${devicetree_load_address}
   ```

   The Debian part is the line of `../debian/README.md` unchanged; without
   the code every boot (reboot, power cycle) is a Debian boot as before.
3. `echo 0xb0 > /sys/kernel/debug/zynq_rst/code && systemctl reboot`.
   u-boot sees `b0`, clears the register (as Pluto's `clear_reset_cause`)
   and boots `fishball.itb`; if loading or `bootm` fails it boots Debian.
4. The image comes up with the same MAC address (u-boot's `ethaddr`), so
   usually the same DHCP address, and u-boot's `hostname` (avahi:
   `NAME.local`); ssh as root with the key of step 1 (a new host key), or
   `analog`. It reboots into Debian after 900 s (`plutosat.reboot_after`;
   `touch /tmp/plutosat-stay` to stay, `reboot` to leave earlier). A kernel
   panic reboots after 10 s; a hang in user space is caught by the
   watchdog (`S15watchdog`, 10 s).
5. Afterwards restore `uEnv.txt`; `fishball.itb` and `plutosat/` on the
   FAT partition can stay or go.

Not covered: a kernel that hangs before `S15watchdog` starts (u-boot here
has no watchdog driver); then the board needs a power cycle, after which
it boots Debian. u-boot could arm the Zynq SWDT with `mw` before `bootm`
(not tried).
