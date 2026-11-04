#!/usr/bin/env python3
"""Register descriptions (regs/*.toml) -> C headers and Markdown docs.

    gen.py [FILE.toml ...]          write gen/<core>_regs.h and gen/<core>.md
    gen.py --check [FILE.toml ...]  compare each file with the register map in
                                    the header comment of its RTL (core.rtl)

Without files, all regs/*.toml are used. The format is described in
README.md. --check compares offsets and names only (wording, access columns
and field descriptions in the RTL comment are free text) and exits 1 on any
mismatch.
"""
import argparse
import pathlib
import re
import sys
import tomllib

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
ACCESS = ("rw", "ro", "wo", "w1c", "rc")
NAME_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")
CORE_RE = re.compile(r"^[a-z][a-z0-9_]*$")


class SpecError(Exception):
    pass


# ---------------------------------------------------------------- loading
def _need(d, keys, where):
    for k in keys:
        if k not in d:
            raise SpecError(f"{where}: missing '{k}'")


def _check_fields(fields, width, where, reg_access=None):
    seen = {}
    for f in fields:
        fw = f"{where}.{f.get('name', '?')}"
        _need(f, ("name", "bits", "doc"), fw)
        if not NAME_RE.match(f["name"]):
            raise SpecError(f"{fw}: name must be UPPER_SNAKE")
        if f["name"] in seen:
            raise SpecError(f"{fw}: duplicate field")
        lsb, msb = f["bits"]
        if not (0 <= lsb <= msb < width):
            raise SpecError(f"{fw}: bits [{lsb}, {msb}] outside 0..{width - 1}")
        acc = f.get("access", reg_access)
        if acc is not None and acc not in ACCESS:
            raise SpecError(f"{fw}: bad access '{acc}'")
        for g in seen.values():
            glsb, gmsb = g["bits"]
            if lsb <= gmsb and glsb <= msb:
                ga = g.get("access", reg_access)
                if {acc, ga} != {"ro", "wo"}:
                    raise SpecError(f"{fw}: overlaps {g['name']} (only a ro and a wo field may share bits)")
        for k, v in f.get("enum", {}).items():
            if not NAME_RE.match(k):
                raise SpecError(f"{fw}: enum name {k} must be UPPER_SNAKE")
            if not 0 <= v < (1 << (msb - lsb + 1)):
                raise SpecError(f"{fw}: enum {k} = {v} does not fit")
        seen[f["name"]] = f


def _check_reg(r, where, limit):
    _need(r, ("name", "offset", "access", "doc"), where)
    w = f"{where} {r['name']}"
    if not NAME_RE.match(r["name"]):
        raise SpecError(f"{w}: name must be UPPER_SNAKE")
    if r["offset"] % 4 or not 0 <= r["offset"] < limit:
        raise SpecError(f"{w}: bad offset {r['offset']:#x}")
    if r["access"] not in ACCESS:
        raise SpecError(f"{w}: bad access '{r['access']}'")
    _check_fields(r.get("field", []), 32, w, r["access"])


def load(path):
    path = pathlib.Path(path)
    with open(path, "rb") as f:
        spec = tomllib.load(f)
    where = path.name
    _need(spec, ("core",), where)
    core = spec["core"]
    _need(core, ("name", "title", "uio", "size", "id", "version", "rtl", "maps"), f"{where} [core]")
    if not CORE_RE.match(core["name"]):
        raise SpecError(f"{where}: core.name must be lower_snake")
    for k in ("id", "version"):
        _need(core[k], ("offset", "value"), f"{where} core.{k}")
    size = core["size"]
    spec.setdefault("reg", [])
    spec.setdefault("array", [])
    spec.setdefault("window", [])
    spec.setdefault("const", [])
    spec.setdefault("packet", [])

    words = {}          # word offset -> owner, to find overlaps

    def claim(off, owner):
        if off in words:
            raise SpecError(f"{where}: {owner} at {off:#x} overlaps {words[off]}")
        words[off] = owner

    for r in spec["reg"]:
        _check_reg(r, where, size)
        claim(r["offset"], r["name"])
    for a in spec["array"]:
        _need(a, ("name", "offset", "stride", "count", "reg"), f"{where} array")
        aw = f"{where} array {a['name']}"
        if not NAME_RE.match(a["name"]) or a["offset"] % 4 or a["stride"] % 4:
            raise SpecError(f"{aw}: bad name, offset or stride")
        if a["offset"] + a["stride"] * a["count"] > size:
            raise SpecError(f"{aw}: beyond the register space")
        a.setdefault("doc", "")
        names = set()
        for r in a["reg"]:
            _check_reg(r, aw, a["stride"])
            if r["name"] in names:
                raise SpecError(f"{aw}: duplicate {r['name']}")
            names.add(r["name"])
            for i in range(a["count"]):
                claim(a["offset"] + a["stride"] * i + r["offset"], f"{a['name']}[{i}].{r['name']}")
    for w in spec["window"]:
        _need(w, ("name", "offset", "words", "access", "doc"), f"{where} window")
        ww = f"{where} window {w['name']}"
        if not NAME_RE.match(w["name"]) or w["offset"] % 4:
            raise SpecError(f"{ww}: bad name or offset")
        if w["access"] not in ACCESS:
            raise SpecError(f"{ww}: bad access")
        count = w.get("count", 1)
        stride = w.get("stride", w["words"] * 4)
        if stride < w["words"] * 4 or stride % 4:
            raise SpecError(f"{ww}: stride smaller than the window")
        if w["offset"] + stride * (count - 1) + 4 * w["words"] > size:
            raise SpecError(f"{ww}: beyond the register space")
        if not 1 <= w.get("width", 32) <= 32:
            raise SpecError(f"{ww}: bad width")
        for i in range(count):
            for k in range(w["words"]):
                claim(w["offset"] + stride * i + 4 * k, f"{w['name']}[{i}][{k}]")
    for c in spec["const"]:
        _need(c, ("name", "value", "doc"), f"{where} const")
        if not NAME_RE.match(c["name"]):
            raise SpecError(f"{where} const {c['name']}: name must be UPPER_SNAKE")
    for p in spec["packet"]:
        _need(p, ("name", "width", "doc", "field"), f"{where} packet")
        if p["width"] not in (32, 64) or not NAME_RE.match(p["name"]):
            raise SpecError(f"{where} packet {p['name']}: width 32 or 64, UPPER_SNAKE name")
        _check_fields(p["field"], p["width"], f"{where} packet {p['name']}")
    for k in ("id", "version"):
        if core[k]["offset"] not in {r["offset"] for r in spec["reg"]}:
            raise SpecError(f"{where}: core.{k} offset is not a register")
    spec["_path"] = path
    return spec


# ---------------------------------------------------------------- C header
def _hex(v, width=32):
    return f"0x{v:0{width // 4}X}" + ("ULL" if width > 32 else "u")


def _field_lines(define, comment, prefix, fields, width=32):
    for f in fields:
        lsb, msb = f["bits"]
        n = msb - lsb + 1
        mask = ((1 << n) - 1) << lsb
        p = f"{prefix}_{f['name']}"
        acc = f" ({f['access']})" if "access" in f else ""
        sg = ", signed" if f.get("signed") else ""
        bits = f"{msb}" if lsb == msb else f"{msb}:{lsb}"
        comment(f"  [{bits}] {f['name']}{acc}{sg}: {f['doc']}")
        define(f"{p}_SHIFT", lsb)
        define(f"{p}_WIDTH", n)
        define(f"{p}_MASK", _hex(mask, width))
        for k, v in f.get("enum", {}).items():
            define(f"{p}_{k}", v)


def gen_c(spec):
    core = spec["core"]
    P = core["name"].upper()
    names = {}
    lines = [
        f"/* {core['title']}: register map of {core['name']}.",
        f" * Generated by regs/gen.py from regs/{spec['_path'].name}; do not edit. */",
        f"#ifndef {P}_REGS_H",
        f"#define {P}_REGS_H",
        "",
        f"#define {P}_UIO \"{core['uio']}\"",
        f"#define {P}_SIZE {_hex(core['size'])}",
        f"#define {P}_ID_VALUE {_hex(core['id']['value'])}",
        f"#define {P}_VERSION_VALUE {_hex(core['version']['value'])}",
    ]
    for i, m in enumerate(core["maps"]):
        lines.append(f"#define {P}_MAP_{m.upper()} {i}")

    def define(name, val, comment=None):
        if name in names:
            raise SpecError(f"{core['name']}: generated name {name} defined twice")
        names[name] = val
        if comment:
            lines.append(f"/* {comment} */")
        lines.append(f"#define {name} {val}")

    def comment(text):
        lines.append(f"/* {text} */")

    if spec["const"]:
        lines.append("")
        for c in spec["const"]:
            define(f"{P}_{c['name']}", c["value"], c["doc"])
    lines.append("")
    for r in spec["reg"]:
        define(f"{P}_{r['name']}", f"0x{r['offset']:03X}", f"{r['name']} ({r['access']}): {r['doc']}")
        _field_lines(define, comment, f"{P}_{r['name']}", r.get("field", []))
    for a in spec["array"]:
        A = f"{P}_{a['name']}"
        lines += ["", f"/* {a['name']}[0..{a['count'] - 1}]: {a['doc']} */"]
        define(f"{A}_BASE", f"0x{a['offset']:03X}")
        define(f"{A}_STRIDE", f"0x{a['stride']:03X}")
        define(f"{A}_COUNT", a["count"])
        for r in a["reg"]:
            R = f"{A}_{r['name']}"
            lines.append(f"/* +0x{r['offset']:02X} {r['name']} ({r['access']}): {r['doc']} */")
            define(f"{R}_OFF", f"0x{r['offset']:02X}")
            define(f"{R}(i)", f"({A}_BASE + {A}_STRIDE * (i) + {R}_OFF)")
            _field_lines(define, comment, R, r.get("field", []))
    for w in spec["window"]:
        W = f"{P}_{w['name']}"
        count = w.get("count", 1)
        lines += ["", f"/* {w['name']}: {w['doc']} ({w['access']}, {w.get('width', 32)} bits"
                  f"{', signed' if w.get('signed') else ''}) */"]
        define(f"{W}_BASE", f"0x{w['offset']:04X}")
        define(f"{W}_WORDS", w["words"])
        define(f"{W}_WIDTH", w.get("width", 32))
        if "count" in w:
            define(f"{W}_STRIDE", f"0x{w.get('stride', 4 * w['words']):04X}")
            define(f"{W}_COUNT", count)
            define(f"{W}(c, i)", f"({W}_BASE + {W}_STRIDE * (c) + 4 * (i))")
        else:
            define(f"{W}(i)", f"({W}_BASE + 4 * (i))")
    for p in spec["packet"]:
        lines += ["", f"/* DMA packet word {p['name']} ({p['width']} bits): {p['doc']} */"]
        _field_lines(define, comment, f"{P}_PKT_{p['name']}", p["field"], p["width"])
    lines += ["", f"#endif /* {P}_REGS_H */", ""]
    return "\n".join(lines)


# ---------------------------------------------------------------- Markdown
def _md(s):
    return str(s).replace("|", "\\|")


def _md_fields(fields, default_access):
    out = ["| Bits | Field | Access | Description |", "|---|---|---|---|"]
    for f in sorted(fields, key=lambda f: (f["bits"][0], f.get("access", ""))):
        lsb, msb = f["bits"]
        bits = f"{msb}" if lsb == msb else f"{msb}:{lsb}"
        d = f["doc"] + (" (signed)" if f.get("signed") else "")
        if f.get("enum"):
            d += ": " + ", ".join(f"{v} = {k}" for k, v in f["enum"].items())
        out.append(f"| [{bits}] | {f['name']} | {f.get('access', default_access)} | {_md(d)} |")
    return out


def _md_regs(regs, rel=False):
    out = ["| Offset | Name | Access | Reset | Description |", "|---|---|---|---|---|"]
    for r in sorted(regs, key=lambda r: r["offset"]):
        off = f"+0x{r['offset']:02X}" if rel else f"0x{r['offset']:03X}"
        reset = f"0x{r['reset']:X}" if "reset" in r else ""
        out.append(f"| {off} | {r['name']} | {r['access']} | {reset} | {_md(r['doc'])} |")
    return out


def gen_md(spec):
    core = spec["core"]
    L = [f"# {core['title']} (`{core['name']}`)", "",
         f"Generated by `regs/gen.py` from `regs/{spec['_path'].name}`; do not edit.", "",
         f"- UIO device: `{core['uio']}`, maps: "
         + ", ".join(f"{i} {m}" for i, m in enumerate(core["maps"])),
         f"- Register space: 0x{core['size']:X} bytes",
         f"- ID at 0x{core['id']['offset']:02X}: 0x{core['id']['value']:08X}, "
         f"version at 0x{core['version']['offset']:02X}: 0x{core['version']['value']:08X}",
         f"- RTL: `{core['rtl']}`", "", "## Registers", ""]
    L += _md_regs(spec["reg"])
    for r in spec["reg"]:
        if r.get("field"):
            L += ["", f"### {r['name']} (0x{r['offset']:03X})", ""] + _md_fields(r["field"], r["access"])
    for a in spec["array"]:
        L += ["", f"## {a['name']}[0..{a['count'] - 1}] at 0x{a['offset']:03X} + 0x{a['stride']:X} * i", ""]
        if a["doc"]:
            L += [a["doc"], ""]
        L += _md_regs(a["reg"], rel=True)
        for r in a["reg"]:
            if r.get("field"):
                L += ["", f"### {a['name']}.{r['name']} (+0x{r['offset']:02X})", ""] + _md_fields(r["field"], r["access"])
    if spec["window"]:
        L += ["", "## Windows", "", "| Offset | Name | Words | Access | Width | Description |", "|---|---|---|---|---|---|"]
        for w in spec["window"]:
            off = f"0x{w['offset']:04X}"
            if "count" in w:
                off += f" + 0x{w.get('stride', 4 * w['words']):X} * c (c < {w['count']})"
            width = f"{w.get('width', 32)}{' signed' if w.get('signed') else ''}"
            L.append(f"| {off} | {w['name']} | {w['words']} | {w['access']} | {width} | {_md(w['doc'])} |")
    if spec["const"]:
        L += ["", "## Constants", "", "| Name | Value | Description |", "|---|---|---|"]
        for c in spec["const"]:
            L.append(f"| {c['name']} | {c['value']} | {_md(c['doc'])} |")
    for p in spec["packet"]:
        L += ["", f"## DMA packet word {p['name']} ({p['width']} bits)", "", _md(p["doc"]), ""]
        L += _md_fields(p["field"], "")
    return "\n".join(L) + "\n"


# ---------------------------------------------------------------- RTL check
HEX = r"0x[0-9A-Fa-f_]+"
WIN_RE = re.compile(rf"({HEX})(?:\s*\+\s*({HEX})\s*\*?\s*[a-z])?\s*\+\s*4\s*\*?\s*[a-z]\b")
ARR_RE = re.compile(rf"({HEX})\s*\+\s*({HEX})\s*\*?\s*[a-z]\b")
REG_RE = re.compile(rf"(?<![\w+(])(\+?)({HEX})\s+([A-Z][A-Z0-9_]*)\b")


def _int(s):
    return int(s.replace("_", ""), 16)


def rtl_comment(path):
    """The header comment: leading // lines (blank lines allowed) before code."""
    lines = []
    for line in open(path, encoding="utf-8", errors="replace"):
        s = line.strip()
        if s.startswith("//"):
            lines.append(s[2:])
        elif s:
            break
    return lines


def parse_rtl(path, size):
    regs, arrays, windows = {}, [], []
    arr = None
    for line in rtl_comment(path):
        spans = []
        for m in WIN_RE.finditer(line):
            windows.append((_int(m.group(1)), _int(m.group(2)) if m.group(2) else None))
            spans.append(m.span())
        for m in ARR_RE.finditer(line):
            if any(a <= m.start() < b for a, b in spans):
                continue
            arr = {"offset": _int(m.group(1)), "stride": _int(m.group(2)), "regs": {}}
            arrays.append(arr)
            spans.append(m.span())
        for m in REG_RE.finditer(line):
            if any(a <= m.start() < b for a, b in spans):
                continue
            off = _int(m.group(2))
            if off % 4:
                continue
            if m.group(1):
                if arr is None:
                    continue
                arr["regs"][off] = m.group(3)
            elif off < size:
                regs.setdefault(off, m.group(3))
    return regs, arrays, windows


def check(spec):
    core = spec["core"]
    rtl = ROOT / core["rtl"]
    errs = []
    if not rtl.exists():
        return [f"{rtl}: not found"]
    regs, arrays, windows = parse_rtl(rtl, core["size"])
    if not regs and not arrays:
        return [f"{core['rtl']}: no register map found in the header comment"]

    def cmp(want, got, where):
        for off, name in sorted(want.items()):
            if off not in got:
                errs.append(f"{where} 0x{off:03X} {name}: not in the RTL comment")
            elif got[off].upper() != name.upper():
                errs.append(f"{where} 0x{off:03X}: {name} here, {got[off]} in the RTL comment")
        for off, name in sorted(got.items()):
            if off not in want:
                errs.append(f"{where} 0x{off:03X} {name}: in the RTL comment, not in the TOML")

    cmp({r["offset"]: r["name"] for r in spec["reg"]}, regs, core["name"])
    for a in spec["array"]:
        m = [x for x in arrays if x["offset"] == a["offset"]]
        if not m:
            errs.append(f"{core['name']} array {a['name']} at 0x{a['offset']:03X}: not in the RTL comment")
            continue
        if m[0]["stride"] != a["stride"]:
            errs.append(f"{core['name']} array {a['name']}: stride 0x{a['stride']:X} here, "
                        f"0x{m[0]['stride']:X} in the RTL comment")
        cmp({r["offset"]: r["name"] for r in a["reg"]}, m[0]["regs"], f"{core['name']} {a['name']} +")
    for x in arrays:
        if x["offset"] not in {a["offset"] for a in spec["array"]}:
            errs.append(f"{core['name']}: array at 0x{x['offset']:03X} in the RTL comment, not in the TOML")
    for w in spec["window"]:
        m = [x for x in windows if x[0] == w["offset"]]
        if not m:
            errs.append(f"{core['name']} window {w['name']} at 0x{w['offset']:04X}: not in the RTL comment")
        elif "count" in w and m[0][1] is not None and m[0][1] != w.get("stride", 4 * w["words"]):
            errs.append(f"{core['name']} window {w['name']}: stride differs from the RTL comment")
    for x in windows:
        if x[0] not in {w["offset"] for w in spec["window"]}:
            errs.append(f"{core['name']}: window at 0x{x[0]:04X} in the RTL comment, not in the TOML")
    return errs


# ---------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="compare with the RTL header comments")
    ap.add_argument("--out", default=str(HERE / "gen"), help="output directory (default regs/gen)")
    ap.add_argument("files", nargs="*")
    a = ap.parse_args(argv)
    files = a.files or sorted(str(p) for p in HERE.glob("*.toml"))
    bad = 0
    specs = []
    for f in files:
        try:
            specs.append(load(f))
        except (SpecError, tomllib.TOMLDecodeError, ValueError, TypeError) as e:
            print(f"{f}: {e}", file=sys.stderr)
            bad = 1
    if a.check:
        for s in specs:
            errs = check(s)
            for e in errs:
                print(e)
            print(f"{s['core']['name']}: {'FAIL' if errs else 'ok'} ({s['core']['rtl']})")
            bad |= bool(errs)
        return bad
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for s in specs:
        name = s["core"]["name"]
        try:
            h = gen_c(s)
        except SpecError as e:
            print(e, file=sys.stderr)
            bad = 1
            continue
        (out / f"{name}_regs.h").write_text(h)
        (out / f"{name}.md").write_text(gen_md(s))
    return bad


if __name__ == "__main__":
    sys.exit(main())
