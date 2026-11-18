"""Bit-exact Python model of the DVB-S2 TX RTL (dvbs2/rtl).

Every function mirrors one RTL block and uses the same integer arithmetic,
so cocotb tests can compare the RTL against it sample-for-sample. The model
itself is checked against gr-dtv in test_model.py.
"""

import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "tables.json")) as _f:
    _T = json.load(_f)

RATES = ["1/4", "1/3", "2/5", "1/2", "3/5", "2/3", "3/4", "4/5", "5/6", "8/9", "9/10"]
LDPC = _T["ldpc"]
BCH_POLY = {int(k): int(v, 16) for k, v in _T["bch_poly"].items()}
NLDPC = 64800
# short FECFRAMEs (16200 bits): rates 1/4 .. 8/9 (rate_idx 0..9), BCH t = 12 over GF(2^14)
LDPC_SHORT = _T["ldpc_short"]
BCH_POLY_SHORT = int(_T["bch_poly_short"], 16)
NLDPC_SHORT = 16200
# 8PSK MODCODs 12..17: rate_idx of 3/5, 2/3, 3/4, 5/6, 8/9, 9/10
PSK8_RATE_IDX = [4, 5, 6, 8, 9, 10]

# Symbol amplitude for |I| = |Q| = 1/sqrt(2) in Q1.15 (GR value * 32767, rounded).
SYM_A = 23170


def rate_params(rate_idx, short=False):
    r = (LDPC_SHORT if short else LDPC)[RATES[rate_idx]]
    return r["kbch"], r["kldpc"], r["t"], r["q"], r["table"]


def nldpc(short=False):
    return NLDPC_SHORT if short else NLDPC


def modcod_info(modcod):
    """MODCOD 1..17 -> (rate_idx, psk8)."""
    if 1 <= modcod <= 11:
        return modcod - 1, False
    if 12 <= modcod <= 17:
        return PSK8_RATE_IDX[modcod - 12], True
    raise ValueError(f"MODCOD {modcod} not supported")


def modcod_of(rate_idx, psk8=False):
    return 12 + PSK8_RATE_IDX.index(rate_idx) if psk8 else rate_idx + 1


def valid_code(modcod, short):
    """QPSK 1..11 and 8PSK 12..17, normal or short; short frames have no 9/10."""
    try:
        r, _ = modcod_info(modcod)
    except ValueError:
        return False
    return not (short and r == 10)


# --------------------------------------------------------------------------
# FEC
# --------------------------------------------------------------------------

def bb_scramble(bits):
    """BB scrambler, PRBS 1 + x^14 + x^15, reset to 100101010000000 per BBFRAME."""
    reg = 0b100101010000000  # reg bit 14 = first PRBS stage (x^1) ... bit 0 = x^15
    out = np.empty_like(bits)
    for i, b in enumerate(bits):
        fb = ((reg >> 1) ^ reg) & 1  # x^14 xor x^15
        out[i] = b ^ fb
        reg = (reg >> 1) | (fb << 14)
    return out


def bch_encode(msg, t, short=False):
    nk = 168 if short else 16 * t
    poly = BCH_POLY_SHORT if short else BCH_POLY[t]
    mask = (1 << nk) - 1
    reg = 0
    for b in msg:
        fb = ((reg >> (nk - 1)) & 1) ^ int(b)
        reg = (reg << 1) & mask
        if fb:
            reg ^= poly
    par = np.array([(reg >> (nk - 1 - i)) & 1 for i in range(nk)], dtype=np.uint8)
    return np.concatenate([msg, par])


def ldpc_encode(info, rate_idx, short=False):
    _, kldpc, _, q, table = rate_params(rate_idx, short)
    nparity = nldpc(short) - kldpc
    p = np.zeros(nparity, dtype=np.uint8)
    idx = np.flatnonzero(info)
    for m in idx:
        s = m % 360
        for x in table[m // 360]:
            p[(x + s * q) % nparity] ^= 1
    return np.concatenate([info, np.bitwise_xor.accumulate(p)])


def fec_frame(bbframe_bits, rate_idx, short=False):
    kbch, _, t, _, _ = rate_params(rate_idx, short)
    assert len(bbframe_bits) == kbch
    return ldpc_encode(bch_encode(bb_scramble(bbframe_bits), t, short), rate_idx, short)


# --------------------------------------------------------------------------
# PL framing
# --------------------------------------------------------------------------

SOF = 0x18D2E82
PLSC_SCRAMBLE = 0x719D83C953422DFA
RM_G = [0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF, 0x0000FFFF, 0xFFFFFFFF]


def plsc_bits(modcod, short_frame, pilots):
    """64 PLS code bits (MSB first) for the 7-bit PLS field."""
    b = [(modcod >> 4) & 1, (modcod >> 3) & 1, (modcod >> 2) & 1, (modcod >> 1) & 1,
         modcod & 1, int(short_frame)]
    c = 0
    for k in range(6):
        if b[k]:
            c ^= RM_G[k]
    y = 0
    for i in range(32):
        ci = (c >> (31 - i)) & 1
        y = (y << 2) | (ci << 1) | (ci ^ int(pilots))
    y ^= PLSC_SCRAMBLE
    return y


def plheader_bits(modcod, pilots, short_frame=False):
    hdr = (SOF << 64) | plsc_bits(modcod, short_frame, pilots)
    return [(hdr >> (89 - i)) & 1 for i in range(90)]


def plheader_signs(modcod, pilots, short=False):
    """pi/2-BPSK header as (sign_I, sign_Q) per symbol; sign 1 means negative."""
    out = []
    for i, y in enumerate(plheader_bits(modcod, pilots, short)):
        # even index (1st, 3rd, ... symbol): (1+j)(1-2y)/sqrt2; odd: (-1+j)(1-2y)/sqrt2
        if i % 2 == 0:
            out.append((y, y))
        else:
            out.append((y ^ 1, y))
    return out


def gold_init(n):
    """(x_a, x_b) 18-bit LFSR states for Gold code n: x advanced by n and by n+131072.

    State bit k holds x(i+k); the RTL uses the same layout.
    """
    def advance(state, steps):
        for _ in range(steps):
            fb = (state & 1) ^ ((state >> 7) & 1)
            state = (state >> 1) | (fb << 17)
        return state
    xa = advance(1, n)
    xb = advance(xa, 131072)
    return xa, xb


Y_INIT_A = (1 << 18) - 1


def _y_advance(state, steps):
    for _ in range(steps):
        fb = (state ^ (state >> 5) ^ (state >> 7) ^ (state >> 10)) & 1
        state = (state >> 1) | (fb << 17)
    return state


Y_INIT_B = _y_advance(Y_INIT_A, 131072)


def pl_scramble_seq(n_syms, xa, xb, ya=Y_INIT_A, yb=Y_INIT_B):
    out = np.empty(n_syms, dtype=np.uint8)
    for i in range(n_syms):
        za = (xa ^ ya) & 1
        zb = (xb ^ yb) & 1
        out[i] = 2 * zb + za
        xa = (xa >> 1) | ((((xa >> 7) ^ xa) & 1) << 17)
        xb = (xb >> 1) | ((((xb >> 7) ^ xb) & 1) << 17)
        ya = (ya >> 1) | ((((ya >> 10) ^ (ya >> 7) ^ (ya >> 5) ^ ya) & 1) << 17)
        yb = (yb >> 1) | ((((yb >> 10) ^ (yb >> 7) ^ (yb >> 5) ^ yb) & 1) << 17)
    return out


def rotate_signs(si, sq, r):
    """Multiply (+-a, +-a) by j^r; signs are 1 for negative."""
    if r == 0:
        return si, sq
    if r == 1:  # j*(I + jQ) = -Q + jI
        return sq ^ 1, si
    if r == 2:
        return si ^ 1, sq ^ 1
    return sq, si ^ 1  # -j*(I + jQ) = Q - jI


def plframe_signs(fec_bits, modcod, pilots, gold=0):
    """One data PLFRAME (QPSK, normal) as an (N, 2) array of sign bits."""
    hdr = plheader_signs(modcod, pilots)
    data = fec_bits.reshape(-1, 2)  # QPSK: b0 -> I sign, b1 -> Q sign
    body = []
    for slot in range(len(data) // 90):
        if pilots and slot > 0 and slot % 16 == 0:
            body.extend([(0, 0)] * 36)
        body.extend([tuple(v) for v in data[slot * 90:(slot + 1) * 90]])
    xa, xb = gold_init(gold)
    rseq = pl_scramble_seq(len(body), xa, xb)
    body = [rotate_signs(si, sq, r) for (si, sq), r in zip(body, rseq)]
    return np.array(hdr + body, dtype=np.uint8)


def dummy_plframe_signs(gold=0):
    hdr = plheader_signs(0, False)
    body = [(0, 0)] * (36 * 90)
    xa, xb = gold_init(gold)
    rseq = pl_scramble_seq(len(body), xa, xb)
    body = [rotate_signs(si, sq, r) for (si, sq), r in zip(body, rseq)]
    return np.array(hdr + body, dtype=np.uint8)


# ---------------------------------------------------------------------------
# Symbols as phase indices (QPSK and 8PSK): symbol = exp(j pi p / 4), p = 0..7.
# QPSK points and the pi/2-BPSK header, pilots and dummy bodies are the odd
# p; the PL scrambler's j^R adds 2R. The RTL (dvbs2_plframer) carries p.
# ---------------------------------------------------------------------------

# 8PSK (EN 302 307-1 5.4.2): symbol value v = (b0 b1 b2) -> p
PSK8_PHASE = [1, 0, 4, 5, 2, 7, 3, 6]
# QPSK: (b0, b1) = (I sign, Q sign) -> p
QPSK_PHASE = [1, 7, 3, 5]
# (I, Q) per p, in Q1.15 (gr-dtv's unit-radius points * 32767, rounded)
PHASE_IQ = np.array([(32767, 0), (SYM_A, SYM_A), (0, 32767), (-SYM_A, SYM_A),
                     (-32767, 0), (-SYM_A, -SYM_A), (0, -32767), (SYM_A, -SYM_A)], dtype=np.int64)


def slots(short=False, psk8=False):
    """90-symbol slots of a data PLFRAME."""
    return nldpc(short) // (3 if psk8 else 2) // 90


def map_symbols(fec_bits, rate_idx, psk8=False, short=False):
    """FECFRAME bits -> phase indices (QPSK: pairs; 8PSK: the 3-column block
    interleaver, written column-wise and read row-wise, the first column
    being the MSB except for rate 3/5, where it is the LSB)."""
    b = np.asarray(fec_bits, dtype=np.int64)
    if not psk8:
        v = b.reshape(-1, 2) @ np.array([2, 1])
        return np.array(QPSK_PHASE, np.int64)[v]
    c = b.reshape(3, -1)
    if rate_idx == 4:
        v = c[2] * 4 + c[1] * 2 + c[0]
    else:
        v = c[0] * 4 + c[1] * 2 + c[2]
    return np.array(PSK8_PHASE, np.int64)[v]


def plheader_phases(modcod, pilots, short=False):
    """pi/2-BPSK header: even index (1+j)(1-2y)/sqrt2 -> 1 / 5, odd (-1+j)(1-2y)/sqrt2 -> 3 / 7."""
    return [(1 if i % 2 == 0 else 3) + 4 * y for i, y in enumerate(plheader_bits(modcod, pilots, short))]


def plframe_phases(fec_bits, modcod, pilots, gold=0, short=False):
    """One data PLFRAME (any supported MODCOD, normal or short) as phase indices."""
    rate_idx, psk8 = modcod_info(modcod)
    data = map_symbols(fec_bits, rate_idx, psk8, short)
    body = []
    for slot in range(len(data) // 90):
        if pilots and slot > 0 and slot % 16 == 0:
            body.extend([1] * 36)
        body.extend(data[slot * 90:(slot + 1) * 90].tolist())
    xa, xb = gold_init(gold)
    rseq = pl_scramble_seq(len(body), xa, xb)
    body = [(p + 2 * int(r)) & 7 for p, r in zip(body, rseq)]
    return np.array(plheader_phases(modcod, pilots, short) + body, dtype=np.int64)


def dummy_plframe_phases(gold=0):
    xa, xb = gold_init(gold)
    rseq = pl_scramble_seq(36 * 90, xa, xb)
    return np.array(plheader_phases(0, False) + [(1 + 2 * int(r)) & 7 for r in rseq], dtype=np.int64)


def phases_to_iq(ph):
    return PHASE_IQ[np.asarray(ph, dtype=np.int64)]


def plframe_len(modcod, pilots, short=False):
    _, psk8 = modcod_info(modcod)
    s = slots(short, psk8)
    return 90 + 90 * s + (36 * ((s - 1) // 16) if pilots else 0)


def signs_to_iq(signs):
    return np.where(signs == 0, SYM_A, -SYM_A).astype(np.int64)  # (N, 2): I, Q


# --------------------------------------------------------------------------
# Pulse shaping (polyphase RRC interpolator) and NCO
# --------------------------------------------------------------------------

COEF_W = 18
ACC_SHIFT = 17  # output = round(sum(coef * sym) / 2^ACC_SHIFT), saturated to 16 bit
P_MAX = 64      # RRC interpolation factor (polyphase branches)
CIC_N = 3       # CIC interpolator after the RRC (rtl/dvbs2_cic_interp.sv)
CIC_W = 26
CIC_RMAX = 32


def _rrc(t, rolloff):
    """Root-raised-cosine impulse response at times t (in symbols)."""
    h = np.empty(len(t))
    for i, ti in enumerate(t):
        if abs(ti) < 1e-12:
            h[i] = 1 - rolloff + 4 * rolloff / np.pi
        elif abs(abs(4 * rolloff * ti) - 1) < 1e-9:
            h[i] = rolloff / np.sqrt(2) * ((1 + 2 / np.pi) * np.sin(np.pi / (4 * rolloff))
                                           + (1 - 2 / np.pi) * np.cos(np.pi / (4 * rolloff)))
        else:
            h[i] = (np.sin(np.pi * ti * (1 - rolloff)) + 4 * rolloff * ti * np.cos(np.pi * ti * (1 + rolloff))) \
                / (np.pi * ti * (1 - (4 * rolloff * ti) ** 2))
    return h


def cic_shift_for(R):
    """CIC output shift floor(log2 R^2): gain R^2 / 2^shift in [1, 2)."""
    return (R * R).bit_length() - 1 if R >= 2 else 0


def tx_split(sps):
    """(P, R) for `sps` samples per symbol, or None if not supported: the RRC
    alone (R = 1) up to P_MAX, else the largest P <= P_MAX with R = sps / P
    an integer in 2..CIC_RMAX (the largest P keeps the CIC images and droop
    smallest)."""
    if 2 <= sps <= P_MAX:
        return sps, 1
    for P in range(P_MAX, 1, -1):
        if sps % P == 0 and 2 <= sps // P <= CIC_RMAX:
            return P, sps // P
    return None


def rrc_coefs(sps, rolloff, taps_per_phase, peak=0.80, cic_r=1):
    """Integer RRC coefficients, length sps*taps_per_phase (sps = the RRC's
    interpolation factor P).

    Scaled so a worst-case QPSK sequence stays below `peak` of full scale.
    With a CIC after the RRC (cic_r = R >= 2) the taps are at the RRC rate
    P x symbol rate; they then also
      * pre-compensate the CIC droop: convolved with [-a, 1 + 2a, -a],
        a = CIC_N (1 - 1/R^2) / 24, whose response 1 + 4a sin^2(pi f)
        matches 1 / H_cic(f) ~ 1 + CIC_N pi^2 (1 - 1/R^2) f^2 / 6 to second
        order (f in cycles per RRC-rate sample); the RRC is evaluated one
        sample beyond each end so the result keeps its length;
      * include 2^cic_shift / R^2 (<= 1), the inverse of the CIC gain after
        its shift, so the output level does not depend on R.
    """
    n = sps * taps_per_phase
    if cic_r >= 2:
        he = _rrc((np.arange(-1, n + 1) - (n - 1) / 2) / sps, rolloff)
        a = CIC_N * (1 - 1 / cic_r ** 2) / 24
        h = he[1:-1] + a * (2 * he[1:-1] - he[:-2] - he[2:])
        k = (1 << cic_shift_for(cic_r)) / cic_r ** 2
    else:
        h = _rrc((np.arange(n) - (n - 1) / 2) / sps, rolloff)
        k = 1.0
    # worst-case |output| for symbols +-SYM_A: max over phases of sum|h|
    # (the CIC's impulse response is positive, so it cannot raise the peak)
    worst = max(np.abs(h[p::sps]).sum() for p in range(sps)) * SYM_A
    scale = peak * 32767 * (1 << ACC_SHIFT) / worst * k
    c = np.round(h * scale).astype(np.int64)
    assert np.abs(c).max() < (1 << (COEF_W - 1))
    return c


def tx_config(sps, rolloff, taps_per_phase=16):
    """Shaping settings for `sps` samples per symbol at the DAC:
    dict(P, R, cic_shift, coefs) (SPS register = P, CIC register = R and
    cic_shift; R = 1 means no CIC). Raises ValueError if not supported.
    The same rules are in sw/dvbs2_tx.c."""
    sp = tx_split(sps)
    if sp is None:
        raise ValueError(f"{sps} samples per symbol is not supported")
    P, R = sp
    return dict(P=P, R=R, cic_shift=cic_shift_for(R),
                coefs=rrc_coefs(P, rolloff, taps_per_phase, cic_r=R))


def rrc_interp(iq, coefs, sps, taps_per_phase):
    """Polyphase interpolation: y[n*sps + p] = sum_j coefs[p + sps*j] * x[n - j]."""
    iq = np.asarray(iq, dtype=np.int64)
    nsym = len(iq)
    out = np.empty((nsym, sps, 2), dtype=np.int64)
    for p in range(sps):
        c = np.asarray(coefs[p::sps][:taps_per_phase], dtype=np.int64)
        for ax in range(2):
            out[:, p, ax] = np.convolve(iq[:, ax], c)[:nsym]
    out = np.clip((out + (1 << (ACC_SHIFT - 1))) >> ACC_SHIFT, -32768, 32767)
    return out.reshape(-1, 2)


def _wrapw(v, w):
    return ((v + (1 << (w - 1))) & ((1 << w) - 1)) - (1 << (w - 1))


def cic_interp(x, R, cshift):
    """3-stage CIC interpolator by R (rtl/dvbs2_cic_interp.sv):
      d[m] = x[m] - 2 x[m-1] + x[m-2], u[n] = d[n // R],
      w = u integrated twice, y[n] = sat16((w[n] + rc) >> cshift),
    rc = 2^(cshift-1) (0 for cshift 0); samples before the first are zero.
    R <= 1 with cshift 0 returns x unchanged."""
    x = np.asarray(x, dtype=np.int64).reshape(-1, 2)
    R = max(int(R), 1)
    d = np.diff(x, n=2, axis=0, prepend=np.zeros((2, 2), np.int64))
    w = np.cumsum(np.cumsum(np.repeat(d, R, axis=0), axis=0), axis=0)
    rc = (1 << (cshift - 1)) if cshift else 0
    return np.clip(_wrapw(w + rc, CIC_W) >> cshift, -32768, 32767)


def tx_shape(iq, cfg, taps_per_phase=16):
    """Symbols -> DAC-rate samples before the NCO, for a tx_config() dict."""
    y = rrc_interp(iq, cfg["coefs"], cfg["P"], taps_per_phase)
    return cic_interp(y, cfg["R"], cfg["cic_shift"]) if cfg["R"] >= 2 else y


NCO_LUT_AW = 10
NCO_AMP = 32767


def nco_lut():
    ph = 2 * np.pi * np.arange(1 << NCO_LUT_AW) / (1 << NCO_LUT_AW)
    return (np.round(NCO_AMP * np.cos(ph)).astype(np.int64),
            np.round(NCO_AMP * np.sin(ph)).astype(np.int64))


def nco_mix(iq, finc, phase0=0):
    """Complex mix by exp(j*phase), 32-bit phase accumulator, 10-bit LUT address.

    out = round((x * e) / 2^15), saturated.
    """
    cos_t, sin_t = nco_lut()
    out = np.empty_like(iq)
    ph = phase0
    for k, (i, q) in enumerate(iq):
        a = ph >> (32 - NCO_LUT_AW)
        c, s = cos_t[a], sin_t[a]
        re = i * c - q * s
        im = i * s + q * c
        out[k, 0] = np.clip((re + (1 << 14)) >> 15, -32768, 32767)
        out[k, 1] = np.clip((im + (1 << 14)) >> 15, -32768, 32767)
        ph = (ph + finc) & 0xFFFFFFFF
    return out
