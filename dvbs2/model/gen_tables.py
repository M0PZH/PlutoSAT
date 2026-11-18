#!/usr/bin/env python3
"""Derive the DVB-S2 normal-frame LDPC address tables and BCH generator
polynomials from gr-dtv, then emit them for the RTL.

The tables are recovered by probing GNU Radio's own encoders, so the HDL is
guaranteed to use exactly the same code as the golden reference:

* LDPC: encoding a frame with a single info bit set at 360*g gives parity
  bits p; undoing the final accumulator (d_i = p_i ^ p_(i-1)) leaves ones
  exactly at the table entries of group g.
* BCH: a message with only its last bit set has parity x^(n-k) mod g(x),
  i.e. g(x) without its leading term.

Outputs (in dvbs2/rtl/gen/):
  dvbs2_tables_pkg.sv   per-rate constants (Kbch, Kldpc, q, t, ROM bases)
  ldpc_rom.mem          LDPC address ROM, one 18-bit entry per line (hex)
and dvbs2/model/tables.json for the Python models.
"""

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import grref  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
GEN = os.path.join(HERE, "..", "rtl", "gen")


def code_params(rate, short):
    """(Kbch, Kldpc, t, Nldpc) of a normal or short FECFRAME code."""
    kbch, kldpc, t = (grref.SHORT_BCH if short else grref.NORMAL_BCH)[rate]
    return kbch, kldpc, t, grref.NLDPC_SHORT if short else grref.NLDPC


def probe_ldpc(rate, short=False):
    kbch, kldpc, _, nldpc = code_params(rate, short)
    nparity = nldpc - kldpc
    groups = kldpc // 360
    frames = np.zeros((groups, kldpc), dtype=np.uint8)
    for g in range(groups):
        frames[g, 360 * g] = 1
    out = grref.ldpc(frames.ravel(), rate, short).reshape(groups, nldpc)
    assert np.array_equal(out[:, :kldpc], frames), "LDPC not systematic?"
    table = []
    for g in range(groups):
        p = out[g, kldpc:]
        d = p.copy()
        d[1:] ^= p[:-1]
        table.append([int(x) for x in np.flatnonzero(d)])
    return table, nparity


def probe_bch(rate, short=False):
    kbch, nbch, t, _ = code_params(rate, short)
    msg = np.zeros(kbch, dtype=np.uint8)
    msg[-1] = 1
    out = grref.bch(msg, rate, short)
    parity = out[kbch:]
    assert len(parity) == nbch - kbch
    # parity bits MSB first = coefficients of x^(nk-1) .. x^0 of g(x) - x^nk
    return int("".join(str(b) for b in parity), 2)


def ldpc_encode(info, table, nparity):
    """Reference software encoder using the probed table (for self-check)."""
    q = nparity // 360
    p = np.zeros(nparity, dtype=np.uint8)
    for m, bit in enumerate(info):
        if bit:
            s = m % 360
            for x in table[m // 360]:
                p[(x + s * q) % nparity] ^= 1
    return np.bitwise_xor.accumulate(p)


def bch_encode(msg, poly, nk):
    reg = 0
    mask = (1 << nk) - 1
    for b in msg:
        fb = ((reg >> (nk - 1)) & 1) ^ int(b)
        reg = (reg << 1) & mask
        if fb:
            reg ^= poly
    return np.array([(reg >> (nk - 1 - i)) & 1 for i in range(nk)], dtype=np.uint8)


def y_advance(state, steps):
    for _ in range(steps):
        fb = (state ^ (state >> 5) ^ (state >> 7) ^ (state >> 10)) & 1
        state = (state >> 1) | (fb << 17)
    return state


def x_advance(state, steps):
    for _ in range(steps):
        fb = (state ^ (state >> 7)) & 1
        state = (state >> 1) | (fb << 17)
    return state


def extra_pkg_lines():
    ya = (1 << 18) - 1
    yb = y_advance(ya, 131072)
    xa = 1
    xb = x_advance(xa, 131072)
    return ("\n  // PL scrambling (Gold code n = 0 defaults; software computes x states for other n)\n"
            f"  localparam logic [17:0] PLS_YA_INIT = 18'h{ya:05x};\n"
            f"  localparam logic [17:0] PLS_YB_INIT = 18'h{yb:05x};\n"
            f"  localparam logic [17:0] PLS_XA_GOLD0 = 18'h{xa:05x};\n"
            f"  localparam logic [17:0] PLS_XB_GOLD0 = 18'h{xb:05x};\n")


def write_nco_lut(aw=10, amp=32767):
    ph = 2 * np.pi * np.arange(1 << aw) / (1 << aw)
    c = np.round(amp * np.cos(ph)).astype(np.int64)
    s = np.round(amp * np.sin(ph)).astype(np.int64)
    with open(os.path.join(GEN, "nco_lut.mem"), "w") as f:
        for ci, si in zip(c, s):  # {sin[15:0], cos[15:0]}
            f.write(f"{(int(si) & 0xffff):04x}{(int(ci) & 0xffff):04x}\n")


def table_index(rate, short):
    """RTL table index {short, rate_idx[3:0]}: rate_idx 0..10 = 1/4 .. 9/10."""
    return (16 if short else 0) + grref.QPSK_RATES.index(rate)


def main():
    rng = np.random.default_rng(1)
    tables = {}
    tables_short = {}
    polys = {}
    poly_short = None
    for short in (False, True):
        for rate in (grref.SHORT_RATES if short else grref.QPSK_RATES):
            table, nparity = probe_ldpc(rate, short)
            kbch, kldpc, t, _ = code_params(rate, short)
            poly = probe_bch(rate, short)
            nk = kldpc - kbch
            if short:
                poly_short = poly if poly_short is None else poly_short
                assert poly_short == poly, "short BCH polynomial differs between rates"
            else:
                polys[t] = poly if t not in polys else polys[t]
                assert polys[t] == poly, "BCH polynomial differs between rates with same t"

            # Cross-check against GR on random data.
            info = rng.integers(0, 2, kldpc, dtype=np.uint8)
            assert np.array_equal(ldpc_encode(info, table, nparity), grref.ldpc(info, rate, short)[kldpc:]), rate
            msg = rng.integers(0, 2, kbch, dtype=np.uint8)
            assert np.array_equal(bch_encode(msg, poly, nk), grref.bch(msg, rate, short)[kbch:]), rate

            (tables_short if short else tables)[rate] = {
                "kbch": kbch, "kldpc": kldpc, "t": t, "nparity": nparity,
                "q": nparity // 360, "table": table}
            print(f"{'short' if short else 'normal'} rate {rate:>4}: groups={len(table)} "
                  f"entries={sum(map(len, table))} q={nparity // 360} t={t}  OK vs gr-dtv")

    # Nested property of the BCH codes (g8 | g10 | g12) is not needed by the RTL,
    # but the three polynomials must be distinct.
    assert len(polys) == 3

    os.makedirs(GEN, exist_ok=True)
    with open(os.path.join(HERE, "tables.json"), "w") as f:
        json.dump({"ldpc": tables, "bch_poly": {str(k): hex(v) for k, v in polys.items()},
                   "ldpc_short": tables_short, "bch_poly_short": hex(poly_short)}, f)

    # LDPC ROM: entry = {last_in_group, r[7:0], c0[8:0]}; normal rates, then short
    rom = []
    bases = {}
    for short in (False, True):
        tb = tables_short if short else tables
        for rate in (grref.SHORT_RATES if short else grref.QPSK_RATES):
            q = tb[rate]["q"]
            bases[table_index(rate, short)] = len(rom)
            for grp in tb[rate]["table"]:
                for i, x in enumerate(grp):
                    r, c0 = x % q, x // q
                    assert r < 256 and c0 < 360
                    last = 1 if i == len(grp) - 1 else 0
                    rom.append((last << 17) | (r << 9) | c0)
    with open(os.path.join(GEN, "ldpc_rom.mem"), "w") as f:
        for e in rom:
            f.write(f"{e:05x}\n")

    # per-index constants; the unused indices (11..15, short 9/10 and above)
    # repeat the nearest code so that a wrong configuration cannot hang a block
    def entry(idx):
        short = idx >= 16
        r = idx & 15
        rates = grref.SHORT_RATES if short else grref.QPSK_RATES
        rate = rates[min(r, len(rates) - 1)]
        tb = (tables_short if short else tables)[rate]
        return tb, bases[table_index(rate, short)], short
    cols = {"KBCH": [], "KLDPC": [], "Q": [], "BCH_T": [], "LDPC_ROM_BASE": [], "NLDPC": []}
    for idx in range(32):
        tb, base, short = entry(idx)
        cols["KBCH"].append(tb["kbch"])
        cols["KLDPC"].append(tb["kldpc"])
        cols["Q"].append(tb["q"])
        cols["BCH_T"].append(tb["t"])
        cols["LDPC_ROM_BASE"].append(base)
        cols["NLDPC"].append(grref.NLDPC_SHORT if short else grref.NLDPC)

    rom_aw = int(np.ceil(np.log2(len(rom))))
    with open(os.path.join(GEN, "dvbs2_tables_pkg.sv"), "w") as f:
        f.write("// Generated by dvbs2/model/gen_tables.py from gr-dtv -- do not edit.\n")
        f.write("// DVB-S2 FECFRAME codes, indexed {short, rate_idx[3:0]}: rate_idx 0..10 =\n")
        f.write("//   " + " ".join(grref.QPSK_RATES) + "\n")
        f.write("// (normal: MODCOD 1..11 for QPSK; short frames have no 9/10). Unused\n")
        f.write("// indices repeat the nearest code.\n\n")
        f.write("package dvbs2_tables_pkg;\n\n")
        f.write("  localparam int NUM_RATES   = 32;\n")
        f.write(f"  localparam int LDPC_ROM_DEPTH = {len(rom)};\n")
        f.write(f"  localparam int LDPC_ROM_AW = {rom_aw};\n\n")
        for name, vals in cols.items():
            f.write(f"  localparam int {name} [NUM_RATES] = '{{{', '.join(map(str, vals))}}};\n")
        f.write("\n")
        for t in (8, 10, 12):
            nk = 16 * t
            f.write(f"  localparam logic [{nk - 1}:0] BCH_POLY_T{t} = {nk}'h{polys[t]:0{nk // 4}x};\n")
        f.write(f"  localparam logic [167:0] BCH_POLY_SHORT = 168'h{poly_short:042x};\n")
        f.write(extra_pkg_lines())
        f.write("\nendpackage\n")
    write_nco_lut()
    print(f"wrote {len(rom)} LDPC ROM entries ({rom_aw}-bit address) to {GEN}")


if __name__ == "__main__":
    main()
