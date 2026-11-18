# plutosat-boot (optional, risky)

Puts the PlutoSAT boot files on the fishball board's SD card boot
partition (`/dev/mmcblk0p1`, FAT), so that the PL holds the PlutoSAT
design from power-on:

| file | becomes |
|---|---|
| `BOOT.bin` | FSBL (built from the same XSA as the bitstream; its PS setup checked against the known-good one at build time) + the PlutoSAT bitstream + the fishball u-boot |
| `devicetree.dtb` | the board's current one through `dtb-add-plutosat.py --pl-ranges`: reserved DDR, UIO nodes, no IIO DMAs, no PL node the bitstream lacks (e.g. `mwipcore@43c00000`) |
| `uEnv.txt` | unchanged, plus the Debian `uenvcmd=` line if it has none |

**A bad boot file makes the board unreachable until someone takes the SD
card out.** Nothing needs this package: `plutosat-firmware` loads the
bitstream at every boot with any BOOT.bin. Install it only on a board you
can reach physically.

## What it checks first

`postinst` runs `plutosat-boot install`, which refuses (non-zero, the
package stays unconfigured, nothing written) unless all of these hold;
`PLUTOSAT_BOOT_FORCE=1` overrides at your own risk:

- the board is a fishball Zynq-7020 (`/proc/device-tree/model`);
- the boot partition is FAT, holds `BOOT.bin`, `devicetree.dtb` and
  `uEnv.txt`, and has room for the new files and two backups of each;
- every PL node (0x40000000..0xBFFFFFFF) of the resulting device tree lies
  in the bitstream's AXI address map (`/usr/share/plutosat-boot/pl-axi.txt`),
  the PlutoSAT nodes are there and the IIO DMAs are not;
- the u-boot environment in QSPI (mtd `qspi-uboot-env`, offset 0x100000),
  which the fishball u-boot uses instead of its own whenever its CRC is
  valid, boots through `uEnv.txt` (`bootcmd=run $modeboot`,
  `modeboot=sdboot`, `sdboot` runs `uenvboot`, `uenvboot` runs `uenvcmd`).
  If not (another firmware's environment), reset it at the u-boot prompt
  (serial console): `env default -a; saveenv`. The tool never writes the
  flash;
- an existing `uenvcmd=` line in `uEnv.txt` boots Debian with
  `${devicetree_image}` and `uio_pdrv_genirq.of_id=generic-uio`.

`plutosat-boot check` runs the same checks and says what would change;
`plutosat-boot install --dry-run` too.

## What it writes

Only the files whose content changes; nothing at all when all three are
already as they would be (e.g. on a reinstall). Before writing it keeps
the current three as `NAME.pre-plutosat-VERSION-DATE` and as
`NAME.plutosat-prev`. Each file goes to a temporary name, is fsynced,
renamed into place and read back from the card (page cache dropped) and
compared by SHA-256. The order is devicetree.dtb, uEnv.txt, BOOT.bin.
It never reboots: the change takes effect at the next boot.

## Undo

- On the board: `plutosat-boot-rollback` restores the `*.plutosat-prev`
  set (the files replaced are kept as `*.pre-rollback-DATE`).
- Removing or purging the package does **not** touch the boot partition;
  run `plutosat-boot-rollback` before removing it if you want the old
  files back.
- **By hand, when the board does not boot**: take the SD card out, open
  its first partition (FAT) on another computer and copy
  `BOOT.bin.plutosat-prev` over `BOOT.bin`,
  `devicetree.dtb.plutosat-prev` over `devicetree.dtb`,
  `uEnv.txt.plutosat-prev` over `uEnv.txt` (or an older set,
  `NAME.pre-plutosat-VERSION-DATE`).
