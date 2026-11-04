"""Narrowband IQ channels: bit-exact model of rtl/nbiq_*.sv, and the filter
design used by the driver (sw/nbiq.c does the same in C).

Per channel (4 channels, any of the two AD9363 receive channels):

  NCO / mixer   y[n] = x[n] * exp(-j theta[n]),  theta[n] = n * finc / 2^32 * 2 pi
                (frequency +finc/2^32 * fs moves to DC). 4096-step phase
                (12 bits), quarter-wave sine table of 1024 half-step entries,
                amplitude 32767. Out: sat18(round(x * (c, s) / 2^13)).
  CIC           5 stages, decimation R = 2..64, 48-bit wrap-around state.
                Out: sat24(round(comb / 2^cshift)).
  FIR           real coefficients h[0..2*npairs-1] (18-bit), decimation 2:
                  y[m] = sum_i h[i] x[2m + 1 - i]     (x: CIC outputs)
                computed as two polyphase halves (even taps on the odd
                samples, odd taps on the even samples), 48-bit accumulator.
                Out: sat16(round(acc / 2^oshift)). The first npairs - 1
                outputs after enabling a channel are not produced (they would
                use samples from before the enable).

Output rate fs / (2 R): 1 MSPS at R = 6 and 100 kSPS at R = 60 for 12 MSPS.
The k-th output sample of a channel is y[m], m = k + npairs - 1, the CIC
output index n of x[n] corresponds to ADC sample T0 + R n + R - 1 (T0: the
ADC sample counter when the channel was enabled).
"""
import math

import numpy as np

NCH = 4
LUT_N = 1024                 # quarter-wave entries
LUT_AMP = 32767
MIX_SH = 13
CIC_N = 5
CIC_W = 48
R_MIN, R_MAX = 2, 64
NPAIRS_MAX = 127             # FIR: up to 254 taps
ACC_W = 48
CLK_HZ = 100e6
FIR_BUDGET = 0.9 * CLK_HZ / NCH   # FIR engine clocks per second per channel


def lut():
    """Quarter-wave table: S[k] = round(32767 sin(pi/2 (k + 0.5) / 1024))."""
    return [int(round(LUT_AMP * math.sin(math.pi / 2 * (k + 0.5) / LUT_N))) for k in range(LUT_N)]


_S = lut()


def sincos(ph):
    """(sin, cos) for a 32-bit phase, using its top 12 bits."""
    q = (ph >> 30) & 3
    k = (ph >> 20) & (LUT_N - 1)
    a, b = _S[k], _S[LUT_N - 1 - k]
    if q == 0:
        return a, b
    if q == 1:
        return b, -a
    if q == 2:
        return -a, -b
    return -b, a


def sat(v, w):
    lo, hi = -(1 << (w - 1)), (1 << (w - 1)) - 1
    return lo if v < lo else hi if v > hi else v


def rshift_round(v, sh):
    return (v + (1 << (sh - 1))) >> sh if sh else v


def wrap(v, w):
    v &= (1 << w) - 1
    return v - (1 << w) if v >> (w - 1) else v


_SA = np.array(_S, np.int64)
_M48 = (1 << CIC_W) - 1


def _wrap48(v):
    return ((v + (1 << (CIC_W - 1))) & _M48) - (1 << (CIC_W - 1))


def _rshift_round(v, sh):
    return (v + (1 << (sh - 1))) >> sh if sh else v


def mixer(xi, xq, finc):
    """Vectorized mixer (same results as mixer_ref). Returns int64 arrays."""
    xi = np.asarray(xi, np.int64)
    xq = np.asarray(xq, np.int64)
    ph = (np.arange(len(xi), dtype=np.uint64) * np.uint64(finc)) & np.uint64(0xFFFFFFFF)
    q = (ph >> np.uint64(30)).astype(np.int64)
    k = ((ph >> np.uint64(20)) & np.uint64(LUT_N - 1)).astype(np.int64)
    a, b = _SA[k], _SA[LUT_N - 1 - k]
    s = np.select([q == 0, q == 1, q == 2], [a, b, -a], -b)
    c = np.select([q == 0, q == 1, q == 2], [b, -a, -b], a)
    rnd = 1 << (MIX_SH - 1)
    oi = np.clip((xi * c + xq * s + rnd) >> MIX_SH, -(1 << 17), (1 << 17) - 1)
    oq = np.clip((xq * c - xi * s + rnd) >> MIX_SH, -(1 << 17), (1 << 17) - 1)
    return oi, oq


def cic(x, R, cshift):
    """Vectorized CIC (same results as cic_ref): integrator k > 0 adds the
    previous value of integrator k - 1; int64 wrap-around is exact mod 2^48."""
    v = np.cumsum(np.asarray(x, np.int64))
    for _ in range(1, CIC_N):
        v = np.concatenate(([0], np.cumsum(v)[:-1]))
    u = _wrap48(v[R - 1::R])
    for _ in range(CIC_N):
        u = _wrap48(np.diff(u, prepend=0))
    return np.clip(_rshift_round(u, cshift), -(1 << 23), (1 << 23) - 1)


def fir(x, h, oshift):
    """Vectorized polyphase FIR (same results as fir_ref)."""
    x = np.asarray(x, np.int64)
    h = np.asarray(h, np.int64)
    np_ = len(h) // 2
    xe, xo = x[0::2], x[1::2]
    n = len(xo)
    if n < np_:
        return np.zeros(0, np.int64)
    acc = np.convolve(xo, h[0::2])[np_ - 1:n] + np.convolve(xe[:n], h[1::2])[np_ - 1:n]
    return np.clip(_rshift_round(_wrap48(acc), oshift), -32768, 32767)


def channel(xi, xq, cfg):
    """cfg: dict finc, R, cshift, h, oshift. Returns (I, Q) int64 arrays."""
    mi, mq = mixer(xi, xq, cfg["finc"])
    return (fir(cic(mi, cfg["R"], cfg["cshift"]), cfg["h"], cfg["oshift"]),
            fir(cic(mq, cfg["R"], cfg["cshift"]), cfg["h"], cfg["oshift"]))


# scalar reference versions (straight from the RTL's register behaviour)

def mixer_ref(xi, xq, finc):
    """xi, xq: int sequences (16-bit). Returns lists of 18-bit I, Q."""
    oi, oq = [], []
    ph = 0
    rnd = 1 << (MIX_SH - 1)
    for i, q in zip(xi, xq):
        s, c = sincos(ph)
        oi.append(sat((int(i) * c + int(q) * s + rnd) >> MIX_SH, 18))
        oq.append(sat((int(q) * c - int(i) * s + rnd) >> MIX_SH, 18))
        ph = (ph + finc) & 0xFFFFFFFF
    return oi, oq


def cic_ref(x, R, cshift):
    """x: 18-bit samples. Integrators update together from their old values
    (the RTL's registers); every R-th input the newest last integrator goes
    through the combs. Returns 24-bit outputs."""
    integ = [0] * CIC_N
    dly = [0] * CIC_N
    out = []
    cnt = 0
    for v in x:
        new = [wrap(integ[0] + v, CIC_W)] + [wrap(integ[k] + integ[k - 1], CIC_W) for k in range(1, CIC_N)]
        integ = new
        if cnt == R - 1:
            cnt = 0
            u = integ[-1]
            for k in range(CIC_N):
                u, dly[k] = wrap(u - dly[k], CIC_W), u
            out.append(sat(rshift_round(u, cshift), 24))
        else:
            cnt += 1
    return out


def fir_ref(x, h, oshift):
    """x: CIC outputs, h: 2*npairs coefficients. Returns 16-bit outputs y[m]
    for m >= npairs - 1."""
    np_ = len(h) // 2
    he, ho = h[0::2], h[1::2]
    xe, xo = x[0::2], x[1::2]
    out = []
    for m in range(np_ - 1, len(xo)):
        acc = 0
        for j in range(np_):
            acc += he[j] * xo[m - j] + ho[j] * xe[m - j]
        out.append(sat(rshift_round(wrap(acc, ACC_W), oshift), 16))
    return out


# ---------------------------------------------------------------- design

def cic_resp(f, R):
    """|H_cic| normalized to 1 at DC, f in cycles per CIC output sample."""
    f = np.asarray(f, float)
    num = np.sin(np.pi * f)
    den = R * np.sin(np.pi * f / R)
    with np.errstate(invalid="ignore", divide="ignore"):
        r = np.where(np.abs(f) < 1e-12, 1.0, num / den)
    return np.abs(r) ** CIC_N


def npairs_for(fout, budget=FIR_BUDGET):
    return int(max(1, min(NPAIRS_MAX, math.floor(budget / fout + 1e-9) - 1)))


def design(fs, rate, gain=8.0, npairs=None, atten=70.0):
    """Channel parameters for an output rate fs/(2R). gain: output LSBs per
    ADC LSB (12-bit full scale 2048 -> 16384 for gain 8). Returns a cfg
    dict without finc. Filter: CIC-compensated Kaiser-windowed lowpass,
    cutoff at fout/2 (the alias-protected band shrinks with fewer taps)."""
    R = int(round(fs / (2 * rate)))
    if not R_MIN <= R <= R_MAX or abs(fs / (2 * R) - rate) > 1e-6 * rate:
        raise ValueError(f"rate {rate} is not fs/(2R) with R in {R_MIN}..{R_MAX}")
    fout = fs / (2 * R)
    np_ = npairs or npairs_for(fout)
    ntaps = 2 * np_
    G = R ** CIC_N
    cshift = max(0, math.ceil(CIC_N * math.log2(R)) - 6)
    # ideal response 1/Hcic for |f| < 0.25 (per CIC output sample), windowed
    c = (ntaps - 1) / 2
    n = np.arange(ntaps) - c
    fgrid = (np.arange(4096) + 0.5) / 4096 * 0.25
    D = 1.0 / cic_resp(fgrid, R)
    hid = 2 * (D[None, :] * np.cos(2 * np.pi * fgrid[None, :] * n[:, None])).sum(1) * (0.25 / 4096)
    beta = 0.1102 * (atten - 8.7) if atten > 50 else 0.5842 * (atten - 21) ** 0.4 + 0.07886 * (atten - 21)
    h = hid * np.kaiser(ntaps, beta)
    h /= h.sum()
    # total gain = 4 (mixer) * G / 2^cshift * sum(h) / 2^oshift
    hsum = gain / (4 * G / 2 ** cshift)                # wanted sum(h) / 2^oshift
    oshift = int(math.floor(math.log2((1 << 17) * 0.95 / (np.abs(h).max() * hsum))))
    oshift = max(0, min(47, oshift))
    hq = [int(round(v)) for v in h * hsum * 2 ** oshift]
    return dict(R=R, cshift=cshift, h=hq, oshift=oshift, fout=fout, npairs=np_)


def finc_for(foff, fs):
    return int(round(foff / fs * 2 ** 32)) & 0xFFFFFFFF


def write_lut(path=None):
    """rtl/gen/nbiq_lut.mem: the quarter-wave table, 16-bit hex."""
    import os
    path = path or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rtl", "gen", "nbiq_lut.mem")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write("".join(f"{v:04x}\n" for v in _S))


if __name__ == "__main__":
    write_lut()
    for rate in (1e6, 500e3, 250e3, 200e3, 100e3):
        d = design(12e6, rate)
        print(f"{rate/1e3:7.1f} kSPS  R={d['R']:2d} cshift={d['cshift']:2d} taps={len(d['h']):3d} "
              f"oshift={d['oshift']:2d} max|h|={max(map(abs, d['h']))}")
