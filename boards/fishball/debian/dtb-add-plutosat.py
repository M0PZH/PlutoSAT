#!/usr/bin/env python3
"""Add the PlutoSAT nodes to a board's existing device tree blob:

  - reserved-memory (no-map) for the DMA buffers at the top of RAM:
      dvbs2-tx 16 MB @ 0x3F000000, dvbs2-rx 1 MB @ 0x3EF00000,
      nbiq 4 MB @ 0x3EB00000, fft 1 MB @ 0x3EA00000, and the DVB-S2
      receiver's LDPC LLR frame queue 2 MB @ 0x3E800000 (PL only, S_AXI_HP1)
  - generic-uio nodes under fpga-axi@0 for dvbs2-tx, dvbs2-rx, nbiq,
    rx-awgn and fft (the same as boards/fishball/linux/zynq-fishball-sdr-7020.dts)
  - no IIO streaming: the PlutoSAT PL has no axi_ad9361 ADC/DAC DMAs and no
    FIR decimator/interpolator, so the dma@7c400000 / dma@7c420000 nodes go,
    and the ADC and DDS core nodes lose "dmas", "dma-names" and
    adi,axi-{decimation,interpolation}-core-available (their drivers then
    probe without an IIO buffer)

Everything else stays as it is. A blob patched by an earlier version gets
what it lacks (e.g. the fft buffer, the DMA removal, the LLR queue buffer).
Needs dtc.

  dtb-add-plutosat.py IN.dtb OUT.dtb
"""
import re
import subprocess
import sys

BUFFERS = [   # name, reg; in this order below the reserved-memory properties
    ("dvbs2-buffer@3f000000", "0x3f000000 0x1000000"),
    ("dvbs2-rx-buffer@3ef00000", "0x3ef00000 0x100000"),
    ("nbiq-buffer@3eb00000", "0x3eb00000 0x400000"),
    ("fft-buffer@3ea00000", "0x3ea00000 0x100000"),
    ("dvbs2-llr-buffer@3e800000", "0x3e800000 0x200000"),
]

UIO = """
		dvbs2-tx@7c500000 {{
			compatible = "generic-uio";
			reg = <0x7c500000 0x10000 0x7c480000 0x10000 0x3f000000 0x1000000>;
			interrupt-parent = <{gic}>;
			interrupts = <0x00 0x36 0x04>;
		}};

		dvbs2-rx@7c510000 {{
			compatible = "generic-uio";
			reg = <0x7c510000 0x10000 0x7c490000 0x10000 0x3ef00000 0x100000 0x3e800000 0x200000>;
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

		fft@7c540000 {{
			compatible = "generic-uio";
			reg = <0x7c540000 0x10000 0x7c4a0000 0x10000 0x3ea00000 0x100000>;
			interrupt-parent = <{gic}>;
			interrupts = <0x00 0x24 0x04>;
		}};
"""

# the fft node of earlier versions (registers only); now + its result DMA
# (axi_fft_dma, S_AXI_HP3) and buffer as map1 / map2
FFT_REG_OLD = "reg = <0x7c540000 0x10000>;"
FFT_REG_NEW = "reg = <0x7c540000 0x10000 0x7c4a0000 0x10000 0x3ea00000 0x100000>;"
# the dvbs2-rx node of earlier versions; now + the LLR queue rings as map3
RX_REG_OLD = "reg = <0x7c510000 0x10000 0x7c490000 0x10000 0x3ef00000 0x100000>;"
RX_REG_NEW = "reg = <0x7c510000 0x10000 0x7c490000 0x10000 0x3ef00000 0x100000 0x3e800000 0x200000>;"

# IIO streaming nodes / properties that go
DMA_NODES = ["dma@7c400000", "dma@7c420000"]
CORE_PROPS = {
    "cf-ad9361-lpc@79020000": ["dmas", "dma-names", "adi,axi-decimation-core-available"],
    "cf-ad9361-dds-core-lpc@79024000": ["dmas", "dma-names", "adi,axi-interpolation-core-available"],
}


def uio_nodes(gic):
    """The UIO nodes as (name@addr, text) pairs."""
    text = UIO.format(gic=gic)
    return [(m.group(1), m.group(0)) for m in
            re.finditer(r"\t\t(?:/\*[^\n]*\*/\n\t\t)?([\w-]+@[0-9a-f]+) \{.*?\n\t\t\};\n", text, re.S)]


def node_span(dts, name, depth):
    """(start, end) of the node "name {" at the given tab depth (with the
    blank line before it), or None."""
    t = "\t" * depth
    m = re.search(r"(?:\n(?=\n))?\n" + t + re.escape(name) + r" \{\n", dts)
    if not m:
        return None
    close = "\n" + t + "};\n"
    return m.start(), dts.index(close, m.end() - 1) + len(close) - 1


def main(src, dst):
    dts = subprocess.run(["dtc", "-q", "-I", "dtb", "-O", "dts", src], check=True,
                         capture_output=True, text=True).stdout
    done = []
    # the GIC's phandle
    m = re.search(r"interrupt-controller@f8f01000 \{.*?phandle = <(0x[0-9a-f]+)>;", dts, re.S)
    if not m:
        sys.exit("no interrupt-controller@f8f01000 with a phandle")
    gic = m.group(1)

    # reserved memory
    if "reserved-memory {" not in dts:
        m = re.search(r"\n(\tmemory \{.*?\n\t\};\n)", dts, re.S)
        if not m:
            sys.exit("no top-level memory node")
        rm = ("\n\t/* PlutoSAT (dvbs2/, nbiq/, fft/): uncached DMA buffers at the top of RAM */\n"
              "\treserved-memory {\n\t\t#address-cells = <0x01>;\n\t\t#size-cells = <0x01>;\n"
              "\t\tranges;\n\t};\n")
        dts = dts.replace(m.group(1), m.group(1) + rm, 1)
    elif "dvbs2-buffer@3f000000 {" not in dts:
        sys.exit(f"{src} has a reserved-memory node of its own: merge by hand")
    for name, reg in BUFFERS:
        if f"{name} {{" in dts:
            continue
        _, end = node_span(dts, "reserved-memory", 1)
        at = end - len("\t};")
        dts = dts[:at] + f"\n\t\t{name} {{\n\t\t\treg = <{reg}>;\n\t\t\tno-map;\n\t\t}};\n" + dts[at:]
        done.append(name)

    # UIO nodes; interrupts of the nodes to add must be free
    missing = [(n, t) for n, t in uio_nodes(gic) if f"{n} {{" not in dts]
    used = {int(v, 16) for v in re.findall(r"interrupts = <0x00 (0x[0-9a-f]+) ", dts)}
    ours = {int(v, 16) for _, t in missing for v in re.findall(r"interrupts = <0x00 (0x[0-9a-f]+) ", t)}
    if used & ours:
        sys.exit(f"GIC SPI already used: {sorted(used & ours)}")
    if missing:
        m = re.search(r"(\tfpga-axi@0 \{\n(?:\t\t[^\n]*;\n)+)", dts)
        if not m:
            sys.exit("no fpga-axi@0 node")
        add = "".join(t for _, t in missing)
        if "dvbs2-tx@7c500000 {" not in dts:
            add = "\n\t\t/* PlutoSAT cores, UIO (uio_pdrv_genirq.of_id=generic-uio) */\n" + add
        dts = dts.replace(m.group(1), m.group(1) + add, 1)
        done += [n for n, _ in missing]
    span = node_span(dts, "fft@7c540000", 2)
    if span and FFT_REG_OLD in dts[span[0]:span[1]]:
        dts = dts[:span[0]] + dts[span[0]:span[1]].replace(FFT_REG_OLD, FFT_REG_NEW) + dts[span[1]:]
        done.append("fft@7c540000:dma+buffer")
    span = node_span(dts, "dvbs2-rx@7c510000", 2)
    if span and RX_REG_OLD in dts[span[0]:span[1]]:
        dts = dts[:span[0]] + dts[span[0]:span[1]].replace(RX_REG_OLD, RX_REG_NEW) + dts[span[1]:]
        done.append("dvbs2-rx@7c510000:llr-buffer")

    # no IIO streaming
    for name in DMA_NODES:
        span = node_span(dts, name, 2)
        if span:
            dts = dts[:span[0]] + dts[span[1]:]
            done.append(f"-{name}")
    for name, props in CORE_PROPS.items():
        span = node_span(dts, name, 2)
        if not span:
            continue
        body = dts[span[0]:span[1]]
        for p in props:
            body, n = re.subn(r"\t\t\t" + re.escape(p) + r"(?: = [^\n]*)?;\n", "", body)
            if n:
                done.append(f"-{name}:{p}")
        dts = dts[:span[0]] + body + dts[span[1]:]

    if not done:
        sys.exit(f"{src} already has all the PlutoSAT changes")
    subprocess.run(["dtc", "-q", "-I", "dts", "-O", "dtb", "-o", dst, "-"], input=dts, check=True, text=True)
    print("changed: " + " ".join(done))


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
