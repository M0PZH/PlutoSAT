#!/usr/bin/env python3
"""Add the PlutoSAT nodes to a board's existing device tree blob:

  - reserved-memory (no-map) for the DMA buffers at the top of RAM:
      dvbs2-tx 16 MB @ 0x3F000000, dvbs2-rx 1 MB @ 0x3EF00000, nbiq 4 MB @ 0x3EB00000
  - generic-uio nodes under fpga-axi@0 for dvbs2-tx, dvbs2-rx, nbiq and
    rx-awgn (the same as boards/fishball/linux/zynq-fishball-sdr-7020.dts)

Everything else stays as it is. Needs dtc.

  dtb-add-plutosat.py IN.dtb OUT.dtb
"""
import re
import subprocess
import sys

RESERVED = """
	/* PlutoSAT (dvbs2/, nbiq/): uncached DMA buffers at the top of RAM */
	reserved-memory {
		#address-cells = <0x01>;
		#size-cells = <0x01>;
		ranges;

		dvbs2-buffer@3f000000 {
			reg = <0x3f000000 0x1000000>;
			no-map;
		};

		dvbs2-rx-buffer@3ef00000 {
			reg = <0x3ef00000 0x100000>;
			no-map;
		};

		nbiq-buffer@3eb00000 {
			reg = <0x3eb00000 0x400000>;
			no-map;
		};
	};
"""

UIO = """
		/* PlutoSAT cores, UIO (uio_pdrv_genirq.of_id=generic-uio) */
		dvbs2-tx@7c500000 {{
			compatible = "generic-uio";
			reg = <0x7c500000 0x10000 0x7c480000 0x10000 0x3f000000 0x1000000>;
			interrupt-parent = <{gic}>;
			interrupts = <0x00 0x36 0x04>;
		}};

		dvbs2-rx@7c510000 {{
			compatible = "generic-uio";
			reg = <0x7c510000 0x10000 0x7c490000 0x10000 0x3ef00000 0x100000>;
			interrupt-parent = <{gic}>;
			interrupts = <0x00 0x35 0x04>;
		}};

		nbiq@7c520000 {{
			compatible = "generic-uio";
			reg = <0x7c520000 0x10000 0x3eb00000 0x400000>;
			interrupt-parent = <{gic}>;
			interrupts = <0x00 0x34 0x04>;
		}};

		rx-awgn@7c530000 {{
			compatible = "generic-uio";
			reg = <0x7c530000 0x10000>;
		}};
"""


def main(src, dst):
    dts = subprocess.run(["dtc", "-q", "-I", "dtb", "-O", "dts", src], check=True,
                         capture_output=True, text=True).stdout
    if "dvbs2-rx@7c510000" in dts:
        sys.exit(f"{src} already has the PlutoSAT nodes")
    if "reserved-memory {" in dts:
        sys.exit(f"{src} has a reserved-memory node already: merge by hand")
    # the GIC's phandle
    m = re.search(r"interrupt-controller@f8f01000 \{.*?phandle = <(0x[0-9a-f]+)>;", dts, re.S)
    if not m:
        sys.exit("no interrupt-controller@f8f01000 with a phandle")
    gic = m.group(1)
    # used interrupts: ours (SPI 52..54) must be free
    used = {int(v, 16) for v in re.findall(r"interrupts = <0x00 (0x[0-9a-f]+) ", dts)}
    if used & {0x34, 0x35, 0x36}:
        sys.exit(f"SPI 52..54 already used: {sorted(used & {0x34, 0x35, 0x36})}")
    m = re.search(r"\n(\tmemory \{.*?\n\t\};\n)", dts, re.S)
    if not m:
        sys.exit("no top-level memory node")
    dts = dts.replace(m.group(1), m.group(1) + RESERVED, 1)
    m = re.search(r"(\tfpga-axi@0 \{\n(?:\t\t[^\n]*;\n)+)", dts)
    if not m:
        sys.exit("no fpga-axi@0 node")
    dts = dts.replace(m.group(1), m.group(1) + UIO.format(gic=gic), 1)
    subprocess.run(["dtc", "-q", "-I", "dts", "-O", "dtb", "-o", dst, "-"], input=dts, check=True, text=True)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
