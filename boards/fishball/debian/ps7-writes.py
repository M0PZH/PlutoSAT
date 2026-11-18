#!/usr/bin/env python3
"""Print the PS register writes of an XSA's ps7_init.c, one per line, as
"TABLE OP ADDR [MASK] VALUE" (comments and formatting dropped), so that two
FSBLs' PS setup (MIO, PLLs, clocks, DDR, peripherals) can be compared.

  ps7-writes.py system_top.xsa | ps7_init.c

build-debs.sh compares the XSA the plutosat-boot FSBL is built from with
ps7_init.ref (from the XSA of the known-good FSBL that boots pluto2, which
programs the PS like the board's original FSBL) and refuses to package a
BOOT.bin whose FSBL would set up the PS differently.
"""
import re
import sys
import zipfile


def writes(text):
    out = []
    table = None
    for line in text.splitlines():
        m = re.match(r"\s*unsigned long (ps7_\w+)\[\] = \{", line)
        if m:
            table = m.group(1)
            continue
        m = re.match(r"\s*(EMIT_\w+)\(([^)]*)\)", line)
        if m and table:
            args = " ".join("0x%08X" % int(a.strip().rstrip("Uu"), 16) for a in m.group(2).split(",") if a.strip())
            out.append(f"{table} {m.group(1)[5:]} {args}".rstrip())
    return out


def main(path):
    if path.endswith(".xsa"):
        with zipfile.ZipFile(path) as z:
            text = z.read("ps7_init.c").decode()
    else:
        text = open(path).read()
    w = writes(text)
    if not w:
        sys.exit(f"{path}: no ps7_init register writes found")
    print("\n".join(w))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
