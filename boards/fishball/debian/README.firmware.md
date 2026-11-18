# plutosat-firmware

The PlutoSAT PL design for the fishball-sdr-7020 board (Zynq-7020 +
AD9363) running Debian 12:

| | |
|---|---|
| `/lib/firmware/plutosat.bit.bin` | the bitstream in FPGA-manager format |
| `/usr/sbin/plutosat-pl-load [FILE]` | loads it (or `/lib/firmware/FILE`) at run time: unbinds the ADI drivers on the PL, programs it, binds them again; stops plutosatd around it |
| `plutosat-pl-load.service` | runs `plutosat-pl-load` at every boot, before `plutosatd.service` |
| `/usr/share/plutosat-firmware/cores.txt` | the VERSION register of each core in this bitstream |
| `/usr/share/plutosat-firmware/pl-axi.txt` | the bitstream's AXI address map (`BASE HIGH NAME`) |

The package never touches the boot partition. Any BOOT.bin works: the
board's own (its PL design is replaced at boot by the service), or one
with the PlutoSAT bitstream (optional package `plutosat-boot`; loading the
same bitstream again at boot is harmless).

## The device tree

The kernel has no device-tree overlays: what the PlutoSAT software needs
must be in the board's `devicetree.dtb` on the SD card's boot partition.
`postinst` checks the running tree (`/proc/device-tree`) and warns when
something is missing; it changes nothing.

- **Reserved DDR** (`reserved-memory`, `no-map`), the DMA buffers the PL
  writes and reads:

  | node | size | use |
  |---|---|---|
  | `dvbs2-buffer@3f000000` | 16 MB | DVB-S2 TX |
  | `dvbs2-rx-buffer@3ef00000` | 1 MB | DVB-S2 RX DMA |
  | `nbiq-buffer@3eb00000` | 4 MB | narrowband IQ |
  | `fft-buffer@3ea00000` | 1 MB | FFT results |
  | `dvbs2-llr-buffer@3e800000` | 2 MB | RX LDPC LLR queue (PL only) |
  | `iqtx-buffer@3e700000` | 1 MB | narrowband TX ring |

- **UIO nodes** (`compatible = "generic-uio"`) under `fpga-axi@0` for
  `dvbs2-tx@7c500000`, `dvbs2-rx@7c510000`, `nbiq@7c520000`,
  `rx-awgn@7c530000`, `fft@7c540000`, `iqtx@7c550000`, and
  `uio_pdrv_genirq.of_id=generic-uio` on the kernel command line.
- **No IIO streaming DMAs**: the PlutoSAT PL has no `dma@7c400000` /
  `dma@7c420000`, and the ADC / DDS core nodes must not reference them
  (`dmas`). `plutosat-pl-load` refuses to load while they are listed
  (binding their driver to the empty address range would hang the bus).
- **No PL nodes the bitstream does not have** (`pl-axi.txt`), e.g. the
  original tree's `mwipcore@43c00000`. With a BOOT.bin that loads the PL
  before the kernel, such a node hung a boot (2026-11-05).

`dtb-add-plutosat.py` (here, needs `dtc`) makes such a tree from the
board's own:

```
python3 dtb-add-plutosat.py --pl-ranges /usr/share/plutosat-firmware/pl-axi.txt devicetree.dtb devicetree-new.dtb
```

Exit status 3: nothing to change. Copy the result over `devicetree.dtb`
on the boot partition (`mount /dev/mmcblk0p1 /mnt`), keeping the original,
and reboot. The optional package `plutosat-boot` does this (and BOOT.bin,
uEnv.txt) with checks, backups and a rollback command.

## Undo

`apt remove plutosat-firmware`: the PL keeps the design until the next
boot; after it, the board's BOOT.bin decides.
