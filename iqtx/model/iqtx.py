"""Narrowband IQ transmitter: bit-exact model of rtl/iqtx_*.sv, the filter
design and the generated tables (rtl/gen/*.mem).

Chain (one complex sample stream, 16-bit I/Q from a DDR ring):

  envelope   k  = (GAIN * NORM[RATE]) >> 16       GAIN Q2.14, NORM Q1.16 in [1, 2]
             e  = (k * r) >> 15                    r: ramp 0..32768 (raised cosine)
             x' = sat18(rnd(x * e, 14))            per input sample, I and Q
  FIR x5     v[5n + p] = sat20(rnd(sum_j h[5j + p] x'[n - j], 16)),  j = 0..23
             (120 taps, Q2.16, CIC compensated; x'[n] = 0 before the start)
  CIC xR     4 stages: combs at the FIR rate, zero insertion by R (RATE),
             4 integrators at 12 MS/s (48-bit, wrap-around), each with a
             one-sample delay: y[t] = sum ... (see cic()); gain R^3
  scale      o  = sat16(rnd(y, S[RATE])),          S = ceil(log2 R^3)
  NCO        z  = sat16(rnd(o * (cos + j sin), 15)), phase t * FINC (32-bit),
             4096-step phase, quarter-wave table of 1024 entries (amp 32767)

NORM[R] = round(2^16 2^S / R^3), so GAIN 1.0 (0x4000) gives z = x (a full-scale
input tone gives a full-scale output) for every RATE.

Input rate 12 MS/s / (5 RATE), RATE = 8..400: 300 kS/s .. 6 kS/s (48 kS/s at
RATE 50, 12 kS/s at 200). Passband +-0.4 x input rate (80 %), images
(>= 0.6 x input rate away) below -75 dB.

Ramp: 2^RLOG input samples, r = T[i], T[i] = round(32768 (1 - cos(pi (i +
0.5) / 256)) / 2), i = (l << 8) >> RLOG; up: l = 0, 1, .., M - 1 then 32768;
down: l = lvl - 1, .., 0 then the flush (FLUSH zero samples).

  python3 iqtx.py --gen   regenerate rtl/gen/*.mem (the FIR design is
                          committed: iqtx_fir.mem is what the RTL uses)
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GEN = os.path.join(HERE, "..", "rtl", "gen")

FS = 12e6
L = 5                     # FIR interpolation
TPP = 24                  # taps per phase
NTAP = L * TPP            # 120
CIC_N = 4
CIC_W = 48
R_MIN, R_MAX = 8, 400
X_W = 18                  # envelope output
V_W = 20                  # FIR output
H_FRAC = 16               # taps Q2.16
GAIN_ONE = 1 << 14
RAMP_ONE = 1 << 15
RAMP_N = 256
RLOG_MAX = 15
FLUSH = 32                # zero input samples after the ramp-down
LUT_N = 1024
LUT_AMP = 32767
NORM_N = 512


def sat(v, w):
    lo, hi = -(1 << (w - 1)), (1 << (w - 1)) - 1
    return np.clip(v, lo, hi)


def rnd(v, s):
    """Round half up: (v + 2^(s-1)) >> s (arithmetic), s >= 0."""
    v = np.asarray(v, dtype=np.int64)
    return (v + (1 << (s - 1))) >> s if s else v


def wrap(v, w):
    v = np.asarray(v, dtype=np.int64)
    return ((v + (1 << (w - 1))) & ((1 << w) - 1)) - (1 << (w - 1))


# ---------------------------------------------------------------- tables
def cic_shift(r):
    return math.ceil(math.log2(r ** 3)) if r > 1 else 0


def norm_table():
    """(NORM, S) per RATE index 0..511; entries outside R_MIN..R_MAX are 0."""
    out = []
    for r in range(NORM_N):
        if R_MIN <= r <= R_MAX:
            s = cic_shift(r)
            out.append((int(round(2 ** (16 + s) / r ** 3)), s))
        else:
            out.append((0, 0))
    return out


def ramp_table():
    return [int(round(RAMP_ONE * (1 - math.cos(math.pi * (i + 0.5) / RAMP_N)) / 2)) for i in range(RAMP_N)]


def nco_table():
    return [int(round(LUT_AMP * math.sin(math.pi / 2 * (k + 0.5) / LUT_N))) for k in range(LUT_N)]


def design_fir(it=40):
    """Linear-phase 120-tap interpolation filter (x5): passband 0..0.4 fs_in
    with 1/sinc^4 (CIC droop) compensation, stopband 0.6 fs_in ..; Lawson
    (IRLS) towards minimax. Returns float taps, each polyphase branch DC gain 1."""
    fp, fsb = 0.4 / L, 0.6 / L
    m = NTAP // 2
    fpg = np.linspace(0, fp, 400)
    fsg = np.linspace(fsb, 0.5, 2000)
    f = np.concatenate([fpg, fsg])
    d = np.concatenate([1 / np.sinc(fpg) ** CIC_N, np.zeros_like(fsg)])
    c = 2 * np.cos(2 * np.pi * np.outer(f, np.arange(m) + 0.5))
    w = np.concatenate([np.ones_like(fpg), 4 * np.ones_like(fsg)])
    for _ in range(it):
        sw = np.sqrt(w)
        b = np.linalg.lstsq(c * sw[:, None], d * sw, rcond=None)[0]
        e = np.abs(c @ b - d)
        w = w * (e + 1e-12)
        w /= w.max()
    return np.concatenate([b[::-1], b]) * L


def quant_fir(h):
    return [int(v) for v in np.round(h * 2 ** H_FRAC)]


def write_mem(name, vals, bits):
    os.makedirs(GEN, exist_ok=True)
    digits = (bits + 3) // 4
    with open(os.path.join(GEN, name), "w") as f:
        for v in vals:
            f.write("%0*x\n" % (digits, v & ((1 << bits) - 1)))


def read_mem(name, bits, signed):
    vals = []
    for line in open(os.path.join(GEN, name)):
        line = line.strip()
        if line:
            v = int(line, 16)
            if signed and v >> (bits - 1):
                v -= 1 << bits
            vals.append(v)
    return vals


def gen():
    h = quant_fir(design_fir())
    write_mem("iqtx_fir.mem", h + [0] * (128 - NTAP), 18)
    write_mem("iqtx_norm.mem", [n | s << 17 for n, s in norm_table()], 22)
    write_mem("iqtx_ramp.mem", ramp_table(), 16)
    write_mem("iqtx_nco.mem", nco_table(), 16)


def taps():
    return np.array(read_mem("iqtx_fir.mem", 18, True)[:NTAP], np.int64)


_S = None


def _nco():
    global _S
    if _S is None:
        _S = np.array(nco_table(), np.int64)
    return _S


# ---------------------------------------------------------------- envelope
def ramp_index(l, rlog):
    return (l << 8) >> rlog if rlog >= 8 else l << (8 - rlog)


def ramp_seq(n_total, n_stop, rlog):
    """r for n_total input samples; the ramp-down starts at sample n_stop
    (None: no stop) and the sequence ends with the ramp-down (n_total is then
    ignored beyond it). Returns r (len n_total or up to the end of the ramp)."""
    t = ramp_table()
    mm = 1 << rlog
    lvl = 0
    out = []
    for n in range(n_total):
        if n_stop is None or n < n_stop:
            if lvl < mm:
                out.append(t[ramp_index(lvl, rlog)])
                lvl += 1
            else:
                out.append(RAMP_ONE)
        else:
            if lvl == 0:
                break
            out.append(t[ramp_index(lvl - 1, rlog)])
            lvl -= 1
    return np.array(out, np.int64)


def envelope(xi, xq, r, gain, rate):
    norm, _ = norm_table()[rate]
    k = (gain * norm) >> 16
    e = (k * np.asarray(r, np.int64)) >> 15
    return sat(rnd(np.asarray(xi, np.int64) * e, 14), X_W), sat(rnd(np.asarray(xq, np.int64) * e, 14), X_W)


# ---------------------------------------------------------------- FIR, CIC, NCO
def fir(x, h=None):
    """x' (n samples) -> v (5 n samples); zero state before the start."""
    h = taps() if h is None else h
    x = np.asarray(x, np.int64)
    n = len(x)
    out = np.zeros(L * n, np.int64)
    xp = np.concatenate([np.zeros(TPP - 1, np.int64), x])
    for p in range(L):
        acc = np.zeros(n, np.int64)
        for j in range(TPP):
            acc += h[L * j + p] * xp[TPP - 1 - j: TPP - 1 - j + n]
        out[p::L] = sat(rnd(acc, H_FRAC), V_W)
    return out


def cic(v, rate):
    """v (FIR rate) -> y (rate x), 48-bit wrap, delayed integrators."""
    v = np.asarray(v, np.int64)
    c = v
    for _ in range(CIC_N):
        c = c - np.concatenate([[0], c[:-1]])
    u = np.zeros(len(c) * rate, np.int64)
    u[::rate] = c
    y = u
    for _ in range(CIC_N):
        y = np.cumsum(y)              # int64 wrap-around is exact mod 2^48
    y = np.concatenate([np.zeros(CIC_N, np.int64), y[:-CIC_N]])
    return wrap(y, CIC_W)


def scale(y, rate):
    return sat(rnd(y, cic_shift(rate)), 16)


def sincos(ph):
    """(sin, cos) arrays for 32-bit phases (top 12 bits)."""
    ph = np.asarray(ph, np.int64)
    s = _nco()
    q = (ph >> 30) & 3
    k = (ph >> 20) & (LUT_N - 1)
    a, b = s[k], s[LUT_N - 1 - k]
    sn = np.select([q == 0, q == 1, q == 2], [a, b, -a], -b)
    cs = np.select([q == 0, q == 1, q == 2], [b, -a, -b], a)
    return sn, cs


def nco(oi, oq, finc, ph0=0):
    n = len(oi)
    ph = (ph0 + np.arange(n, dtype=np.int64) * (finc & 0xFFFFFFFF)) & 0xFFFFFFFF
    sn, cs = sincos(ph)
    oi = np.asarray(oi, np.int64)
    oq = np.asarray(oq, np.int64)
    zi = sat(rnd(oi * cs - oq * sn, 15), 16)
    zq = sat(rnd(oi * sn + oq * cs, 15), 16)
    return zi, zq


def chain_env(xi_e, xq_e, rate, finc):
    """Envelope outputs x' -> DAC samples."""
    vi, vq = fir(xi_e), fir(xq_e)
    oi, oq = scale(cic(vi, rate), rate), scale(cic(vq, rate), rate)
    return nco(oi, oq, finc)


def run(xi, xq, rate, gain, finc, rlog, n_stop=None):
    """A whole run: input samples consumed (ring data, zeros for underruns),
    the ramp-down from sample n_stop (None: no stop, no flush). Returns
    (zi, zq, r): the DAC samples and the envelope used."""
    xi = np.asarray(xi, np.int64)
    xq = np.asarray(xq, np.int64)
    r = ramp_seq(len(xi), n_stop, rlog)
    m = len(r)
    ei, eq = envelope(xi[:m], xq[:m], r, gain, rate)
    if n_stop is not None:
        ei = np.concatenate([ei, np.zeros(FLUSH, np.int64)])
        eq = np.concatenate([eq, np.zeros(FLUSH, np.int64)])
    zi, zq = chain_env(ei, eq, rate, finc)
    return zi, zq, r


if __name__ == "__main__":
    if "--gen" in sys.argv:
        gen()
        print("wrote", GEN)
    else:
        sys.exit(__doc__)
