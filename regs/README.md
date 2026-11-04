# Register descriptions

One TOML file per PL core: the single source of truth for its AXI-Lite
register map. `gen.py` turns them into C headers and Markdown docs; the Rust
crate `plutosat-regs` (in the `station/` submodule) reads them at build time.
The RTL is still written by hand: a check (`gen.py --check`) compares each
file with the register map in its RTL header comment.

## Format

```toml
[core]
name = "nbiq"                  # identifier: C prefix NBIQ_, Rust module nbiq
title = "Narrowband IQ channels"
uio = "nbiq"                   # UIO device name (device-tree node name)
size = 0x10000                 # bytes of register space (UIO map 0)
id = { offset = 0x004, value = 0x4E424951 }        # ID register and its value
version = { offset = 0x000, value = 0x00010000 }
rtl = "nbiq/rtl/nbiq_axi.sv"   # file whose header comment holds the map
maps = ["regs", "ring"]        # UIO maps in order (map0 = registers, ...)

[[reg]]
name = "CTRL"
offset = 0x008
access = "rw"                  # rw | ro | wo | w1c | rc (read clears)
reset = 0                      # optional
doc = "Control"
  [[reg.field]]
  name = "IRQ_EN"
  bits = [0, 0]                # [lsb, msb] inclusive
  doc = "Interrupt enable"
  # optional: signed = true, enum = { OFF = 0, ON = 1 }

[[array]]                      # a block of registers repeated per channel
name = "CH"
offset = 0x100                 # address of element 0
stride = 0x40
count = 4
  [[array.reg]]                # offsets relative to the element
  name = "FINC"
  offset = 0x04
  access = "rw"
  doc = "NCO step"

[[window]]                     # a memory region in the register space
name = "COEF"
offset = 0x1000
stride = 0x400                 # optional: one window per array element
count = 4                      # optional
words = 256                    # 32-bit words per window
access = "wo"
width = 18                     # meaningful bits per word
signed = true
doc = "FIR taps h[i]"
```

Rules: offsets are byte offsets, 4-byte aligned; names are UPPER_SNAKE;
every register and field has a `doc`. Fields not listed are reserved (write
0, ignore on read). A register without fields is one 32-bit value.

Optional additions (for things the tables above cannot say):

```toml
  [[reg.field]]                # in a register whose read and write differ
  name = "BUSY"
  bits = [0, 0]
  access = "ro"                # field access, overrides the register's;
  doc = "Capture running"      # a ro and a wo field may share bits

[[const]]                      # a number the software needs (sizes, limits)
name = "PKT_BYTES"
value = 8192
doc = "Bytes per DMA packet"

[[packet]]                     # a header word of the core's DMA packets
name = "HDR"
width = 64                     # 32 or 64 bits
doc = "Word 0 of a packet"
  [[packet.field]]             # same keys as reg.field, bits < width
  name = "LEN"
  bits = [0, 15]
  doc = "BBFRAME length in bytes"
```

## Tools

- `gen.py` writes `gen/<core>_regs.h` (C: `<CORE>_<REG>` offsets,
  `<CORE>_<REG>_<FIELD>_{SHIFT,WIDTH,MASK}`, enum values, `<CORE>_<ARRAY>_<REG>(i)`,
  `<CORE>_<WINDOW>(c, i)`, `<CORE>_<CONST>`, `<CORE>_PKT_<PACKET>_<FIELD>_*`) and
  `gen/<core>.md`.
- `gen.py --check` compares each file with the register table in the header
  comment of `core.rtl`: lines `0xNN NAME ...`, array headers `0xB + 0xS c`
  followed by `+0xNN NAME ...` lines, windows `0xB + 0xS c + 4 i`. Offsets
  and names must match; the rest of the line is free text.
- `station/rust/crates/plutosat-regs` generates Rust constants from the same
  files at build time (`PLUTOSAT_REGS` overrides the directory).
