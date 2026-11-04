"""DVB-S2 receiver, fixed-point model of the RTL (QPSK, normal FECFRAMEs).

Bit-exact reference for rtl/dvbs2_rx_*.sv. Three streaming stages:

  timing    2 samples/symbol in -> polyphase RRC matched filter (32 phases x
            25 taps) -> Gardner TED -> PI loop -> 1 symbol out. AGC on the
            matched-filter output (gain applied after the filter, power
            measured on the symbols).
  framesync differential correlation of the symbols with the 89 header
            differentials (data MODCOD and dummy), max search over one frame,
            verification one frame later, flywheel with a +-2 symbol window.
            Output: the symbols delayed by FS_DELAY with a tag on the first
            header symbol of every frame.
  carrier   PL descrambling, NCO (frequency from every PLHEADER: Kay, then
            Luise-Reggiannini with 6 and 45 lags on the stored header, each
            stage derotating by the previous estimate), then
              pilots on : phase per known block (header, pilot blocks) by
                          CORDIC, linear phase interpolation over the data
                          between two blocks, pilot-to-pilot frequency loop,
                          false-lock detector on the 783-symbol segment;
              pilots off: decision-directed 2nd-order PLL (data-aided on
                          headers and dummy frames, soft decisions on data),
                          phase set from every PLHEADER (removes the 90
                          degree ambiguity), the gains narrowing in gears
                          after the acquisition, the NCO frequency from the
                          PLL integrator (header estimates only right after
                          the acquisition).
            Amplitude/noise from the known blocks, QPSK max-log LLRs
            (int8, 4 LSBs per unit LLR), 64800 per data frame.

Conventions: rnd(a, s) = (a + 2^(s-1)) >> s (arithmetic), sat(a, w) clamps to
w-bit signed. Phases are 32-bit turns; angles out of the CORDIC 16-bit turns.
"""

import math

import numpy as np

import rxmodel as rf
import txmodel as m


def rnd(a, s):
    return (a + (1 << (s - 1))) >> s


def sat(a, w):
    lo = -(1 << (w - 1))
    hi = (1 << (w - 1)) - 1
    return lo if a < lo else hi if a > hi else a


def wrap(a, w):
    a &= (1 << w) - 1
    return a - (1 << w) if a >> (w - 1) else a


# ---------------------------------------------------------------------------
# shared arithmetic: sin/cos LUT rotator (same LUT as the TX NCO), CORDIC
# ---------------------------------------------------------------------------

_COS, _SIN = m.nco_lut()
_COS = [int(v) for v in _COS]
_SIN = [int(v) for v in _SIN]


def rot(i, q, phase):
    """(i + jq) * exp(j*2*pi*phase/2^32): 10-bit LUT, round by 2^15, saturate."""
    a = (phase & 0xFFFFFFFF) >> 22
    c, s = _COS[a], _SIN[a]
    return sat(rnd(i * c - q * s, 15), 16), sat(rnd(i * s + q * c, 15), 16)


CORDIC_IT = 18
CORDIC_AF = 20                    # angle accumulator: turns * 2^20
CORDIC_IN = 22                    # inputs normalised below 2^22
_ATAN = [round(math.atan(2.0 ** -i) / (2 * math.pi) * (1 << CORDIC_AF)) for i in range(CORDIC_IT)]
CORDIC_K = 1.0
for _i in range(CORDIC_IT):
    CORDIC_K *= math.sqrt(1 + 2.0 ** (-2 * _i))


def norm_shift(x, y):
    """Right shift that brings max(|x|, |y|) below 2^CORDIC_IN."""
    mx = max(abs(x), abs(y))
    return max(0, mx.bit_length() - CORDIC_IN)


def cordic(x, y):
    """Vectoring CORDIC. Returns (angle in turns * 2^16 (int16), magnitude * K).

    Inputs of any size are first shifted right (norm_shift); the magnitude is
    shifted back."""
    sh = norm_shift(x, y)
    x >>= sh
    y >>= sh
    ang = 0
    if x < 0:
        x, y = -x, -y
        ang = 1 << (CORDIC_AF - 1)
    x <<= 2
    y <<= 2
    for i in range(CORDIC_IT):
        if y >= 0:
            x, y = x + (y >> i), y - (x >> i)
            ang += _ATAN[i]
        else:
            x, y = x - (y >> i), y + (x >> i)
            ang -= _ATAN[i]
    return wrap(rnd(ang, CORDIC_AF - 16), 16), (x >> 2) << sh


# ---------------------------------------------------------------------------
# DDC: NCO + optional CIC + polyphase decimating FIR (ADC rate -> 2 samples/symbol)
# ---------------------------------------------------------------------------
#
#   m[n] = rot(x[n], n * finc)                       (bypassed when finc = 0)
#   CIC on (cic_r = R >= 2): 3 integrators at the input rate (integrator k > 0
#   adds the previous value of integrator k - 1), decimation by R (the value
#   after input R*j + R - 1), 3 combs; 32-bit wrap-around arithmetic (exact:
#   |output| <= R^3 * 2^15 <= 2^30 for R <= 32), then
#     c[j] = sat18(rnd(comb / 2^cic_shift))
#   CIC off: c[n] = m[n].
#   FIR, decimation D = dec, samples before the first one are zero:
#     y[k] = sat16(rnd(sum_{i<ntaps} h[i] c[k*D + D-1 - i] / 2^shift))
# Total decimation R * D.

DDC_NMAX = 512      # coefficients
DDC_CW = 18
DDC_DMAX = 63       # FIR decimation (6-bit field)
CIC_N = 3
CIC_W = 32
CIC_RMAX = 32
DDC_FIR_ONLY_MAX = 32   # ddc_config(): FIR alone up to this decimation ...
DDC_CIC_D_GOOD = 8      # ... unless the CIC path has a FIR decimation >= this
DDC_CIC_TPP = 16        # ddc_config(): FIR taps per decimation step after the CIC
DDC_CIC_BETA = 8.0      # ... and the Kaiser window's beta
DDC_CIC_DMIN = 4        # ... smallest FIR decimation after the CIC


def ddc_coefs(dec, taps_per_phase=12, gain=1.0):
    """Lowpass for decimation by `dec` to 2 samples/symbol: Kaiser-windowed
    sinc (beta 5.65, ~60 dB), cutoff at the symbol rate (half way between
    the band edge (1 + rolloff)/2 * Rs and the first alias 2 Rs - that).
    Integer coefficients, sum 2^16 * gain (use shift 16). The same design
    is in sw/dvbs2_rx.c."""
    n = dec * taps_per_phase if dec > 1 else 2
    if dec == 1:
        h = np.array([1.0, 0.0])
    else:
        fc = 1.0 / (2 * dec)                      # cutoff / input rate
        k = np.arange(n) - (n - 1) / 2
        beta = 5.65
        w = np.i0(beta * np.sqrt(1 - (2 * np.arange(n) / (n - 1) - 1) ** 2)) / np.i0(beta)
        h = 2 * fc * np.sinc(2 * fc * k) * w
        h /= h.sum()
    c = np.rint(h * gain * (1 << 16)).astype(np.int64)
    assert np.abs(c).max() < (1 << (DDC_CW - 1)) and n <= DDC_NMAX and n % 2 == 0
    return c


def cic_shift_for(R):
    """CIC output shift: gain R^3 / 2^shift in (16, 32] (R >= 4), so a 12-bit
    ADC sample (|I|, |Q| <= 2048, <= 2896 after the NCO) stays below 2^17."""
    return max(0, ((R ** CIC_N) - 1).bit_length() - 5)


def ddc_split(dec):
    """(R, D) for a total decimation `dec`, or None if not supported. CIC
    candidates: R = 2..CIC_RMAX dividing dec with D = dec / R <= 32; the
    one with the smallest R (largest D) is used if D >= DDC_CIC_D_GOOD
    (>= ~70 dB alias rejection), or, above 32, if D >= DDC_CIC_DMIN
    (>= ~62 dB). Otherwise decimations up to 32 use the FIR alone (R = 1,
    ~58 dB)."""
    cand = [R for R in range(2, CIC_RMAX + 1) if dec % R == 0 and dec // R <= DDC_FIR_ONLY_MAX]
    D = dec // cand[0] if cand else 0
    if D >= DDC_CIC_D_GOOD or (dec > DDC_FIR_ONLY_MAX and D >= DDC_CIC_DMIN):
        return cand[0], D
    if 1 <= dec <= DDC_FIR_ONLY_MAX:
        return 1, dec
    return None


def _kaiser(n, beta):
    return np.i0(beta * np.sqrt(1 - (2 * np.arange(n) / (n - 1) - 1) ** 2)) / np.i0(beta)


def ddc_cic_fir(R, D, taps_per_phase=DDC_CIC_TPP, beta=DDC_CIC_BETA):
    """FIR after the CIC (float, sum 1): Kaiser-windowed lowpass, cutoff at
    the symbol rate (1/(2D) of its input rate), with the CIC droop
    compensated: the ideal response is 1 + c f^2 (f in cycles per CIC
    output sample, c = 3 pi^2 (1 - 1/R^2) / 6, the second-order inverse of
    the CIC response) below the cutoff, integrated in closed form."""
    n = D * taps_per_phase
    fc = 1.0 / (2 * D)
    c = CIC_N * math.pi ** 2 * (1 - 1.0 / R ** 2) / 6
    k = np.arange(n) - (n - 1) / 2
    w = 2 * math.pi * k
    with np.errstate(divide="ignore", invalid="ignore"):
        g = 2 * (fc ** 2 * np.sin(w * fc) / w + 2 * fc * np.cos(w * fc) / w ** 2
                 - 2 * np.sin(w * fc) / w ** 3)
        sinc = np.sin(w * fc) / (math.pi * k)
    g = np.where(k == 0, 2 * fc ** 3 / 3, g)
    sinc = np.where(k == 0, 2 * fc, sinc)
    h = (sinc + c * g) * _kaiser(n, beta)
    return h / h.sum()


def ddc_config(dec, gain_log2=4):
    """DDC settings for a total decimation `dec` (ADC rate / (2 x symbol
    rate)) and an overall gain 2^gain_log2 (DC, CW). Returns dict(R, D,
    cic_shift, coefs, shift) or raises ValueError. FIR only (R = 1): the
    ddc_coefs() lowpass, shift 16 - gain_log2 (as before). With the CIC:
    ddc_cic_fir(), scaled to sum 2^shift * 2^gain_log2 / (R^3 / 2^cic_shift)
    with the largest shift that keeps every coefficient below 2^17. The same
    design is in sw/dvbs2_rx.c."""
    sp = ddc_split(dec)
    if sp is None:
        raise ValueError(f"decimation {dec} is not supported")
    R, D = sp
    if R == 1:
        return dict(R=1, D=D, cic_shift=0, coefs=ddc_coefs(D), shift=16 - gain_log2)
    cs = cic_shift_for(R)
    scale = float(1 << gain_log2) * (1 << cs) / R ** CIC_N    # wanted sum(h) / 2^shift
    h = ddc_cic_fir(R, D)
    mx = np.abs(h).max() * scale
    shift = 0
    while shift < 31 and mx * 2.0 ** (shift + 1) < (1 << (DDC_CW - 1)) - 1:
        shift += 1
    c = np.rint(h * scale * 2.0 ** shift).astype(np.int64)
    assert np.abs(c).max() < (1 << (DDC_CW - 1)) and len(c) <= DDC_NMAX and len(c) % 2 == 0
    return dict(R=R, D=D, cic_shift=cs, coefs=c, shift=shift)


_COSA = np.array(_COS, np.int64)
_SINA = np.array(_SIN, np.int64)


def mix(x, finc, ph0=0):
    """Vectorized rot() of x[n] by phase ph0 + n * finc (same results)."""
    x = np.asarray(x, np.int64)
    ph = (np.uint64(ph0) + np.arange(len(x), dtype=np.uint64) * np.uint64(finc)) & np.uint64(0xFFFFFFFF)
    a = (ph >> np.uint64(22)).astype(np.int64)
    c, s = _COSA[a], _SINA[a]
    i, q = x[:, 0], x[:, 1]
    oi = np.clip((i * c - q * s + (1 << 14)) >> 15, -32768, 32767)
    oq = np.clip((i * s + q * c + (1 << 14)) >> 15, -32768, 32767)
    return np.stack([oi, oq], 1)


def _wrapw(v, w):
    return ((v + (1 << (w - 1))) & ((1 << w) - 1)) - (1 << (w - 1))


def cic(x, R, cshift):
    """3-stage CIC, decimation R (see the DDC description above). x: (n, 2)
    int64; the int64 sums wrap mod 2^64, which is exact mod 2^CIC_W."""
    v = np.cumsum(np.asarray(x, np.int64), axis=0)
    for _ in range(1, CIC_N):
        v = np.concatenate([np.zeros((1, 2), np.int64), np.cumsum(v, axis=0)[:-1]])
    u = v[R - 1::R]
    for _ in range(CIC_N):
        u = np.diff(u, axis=0, prepend=np.zeros((1, 2), np.int64))
    u = _wrapw(u, CIC_W)
    if cshift:
        u = (u + (1 << (cshift - 1))) >> cshift
    return np.clip(u, -(1 << 17), (1 << 17) - 1)


def ddc(x, coefs, dec, finc=0, shift=16, cic_r=1, cic_shift=0):
    """The DDC (see above): y[k] = sat16(rnd(sum_i h[i] * c[k*dec + dec-1 - i],
    shift)), c = the CIC output (cic_r >= 2) or the NCO output m[n] =
    x[n] * exp(j*2*pi*n*finc/2^32) (rot(), same LUT as the NCOs)."""
    x = np.asarray(x, np.int64).reshape(-1, 2)
    if finc:
        x = mix(x, finc)
    if cic_r >= 2:
        x = cic(x, cic_r, cic_shift)
    h = np.asarray(coefs, np.int64)
    nt = len(h)
    xp = np.vstack([np.zeros((nt, 2), np.int64), x])       # zeros before the first sample
    K = len(x) // dec
    out = np.empty((K, 2), np.int64)
    hr = h[::-1]
    for k0 in range(0, K, 4096):
        k = np.arange(k0, min(K, k0 + 4096))
        n = k * dec + dec - 1 + nt                          # newest sample, index into xp
        seg = xp[n[:, None] - nt + 1 + np.arange(nt)[None, :]]   # (K, nt, 2): x[n-nt+1] .. x[n]
        acc = np.einsum("j,kjc->kc", hr, seg)
        r = (acc + (1 << (shift - 1))) >> shift if shift else acc
        out[k] = np.clip(r, -32768, 32767)
    return out


# ---------------------------------------------------------------------------
# timing recovery + AGC
# ---------------------------------------------------------------------------

NPH = 32            # fractional-delay phases
NT = 25             # taps per phase (span 12 symbols at 2 sps)
COEF_FRAC = 16
MF_SH = 14          # matched filter: round(acc / 2^14), 18 bit
G_FRAC = 12         # AGC gain Q5.12, 17 bit unsigned
G_MAX = (1 << 17) - 1
G_INIT = 1 << G_FRAC
P_TARGET = 1 << 25  # symbol power target (per axis RMS 4096)
AGC_SH = 12
MU_F = 24           # fractional delay mu: Q0.24
TED_SH = 12
KP_SH = 8
KI_SH = 16


def timing_coefs(rolloff):
    """(NPH, NT) int coefficients; phase p delays the output by p/NPH sample."""
    bank = rf.rrc_bank(rolloff, NPH, 12)
    assert bank.shape == (NPH, NT)
    c = np.round(bank * (1 << COEF_FRAC)).astype(np.int64)
    assert np.abs(c).max() < (1 << 17)
    return c


def loop_gains(bw, zeta=1 / math.sqrt(2)):
    theta = bw / (zeta + 0.25 / zeta)
    d = 1 + 2 * zeta * theta + theta ** 2
    return 4 * zeta * theta / d, 4 * theta ** 2 / d


TIMING_BW = 0.002           # Gardner loop bandwidth (0.005 wanders at -2 dB Es/N0)


def timing_gains(bw=None):
    bw = TIMING_BW if bw is None else bw
    """(kp, ki) for the Gardner loop, normalised to the AGC target."""
    kp, ki = loop_gains(bw)
    return round(kp / 2 * 2 ** 19), round(ki / 2 * 2 ** 27)


def timing_integ_preset(sym_rate, nominal):
    """Start value of the timing loop integrator (TIMING_INTEG write) for a
    transmitter at sym_rate when the receiver's nominal rate (ADC rate /
    (2 x decimation)) is `nominal`: the integrator holds the step of the
    sample position per symbol beyond 2 samples, 2 (nominal / sym_rate - 1)
    in Q.24 samples. E.g. 333000 S/s at decimation 18 (333333 S/s): +33588."""
    v = round(2 * (nominal / sym_rate - 1) * (1 << MU_F))
    return max(-(1 << 31), min((1 << 31) - 1, v))


class Timing:
    def __init__(self, coefs, kp, ki, g_init=G_INIT, integ0=0):
        """integ0: integrator start value (TIMING_INTEG, loaded at reset),
        timing_integ_preset() for a known symbol-rate offset."""
        self.c = np.asarray(coefs, np.int64)
        self.kp, self.ki = kp, ki
        self.g = g_init
        self.gacc = g_init << 15
        self.mu = 0
        self.integ = integ0
        self.prev = (0, 0)
        self.k = NT                     # newest sample of the on-time window
        self.trace = []

    def _filter(self, x, k, ph):
        c = self.c[ph]
        acc = c @ x[k - NT + 1:k + 1][::-1]
        accm = c @ x[k - NT:k][::-1]
        g = self.g
        out = []
        for a in (acc, accm):
            mi = sat(rnd(int(a[0]), MF_SH), 18)
            mq = sat(rnd(int(a[1]), MF_SH), 18)
            out.append((sat(rnd(mi * g, 14), 16), sat(rnd(mq * g, 14), 16)))
        return out

    def run(self, x, trace=False):
        """x: (N, 2) int samples at 2 sps. Returns (M, 2) int symbols."""
        x = np.asarray(x, np.int64)
        out = []
        n = len(x)
        while self.k < n:
            k = self.k
            ph = self.mu >> (MU_F - 5)
            on, mid = self._filter(x, k, ph)
            out.append(on)
            pi_, pq = self.prev
            e = (pi_ - on[0]) * mid[0] + (pq - on[1]) * mid[1]
            es = e >> TED_SH
            self.prev = on
            self.integ += (es * self.ki) >> KI_SH
            mu = self.mu + ((es * self.kp) >> KP_SH) + self.integ
            st = mu >> MU_F
            st = -1 if st < -1 else 1 if st > 1 else st
            self.mu = mu & ((1 << MU_F) - 1)
            self.k = k + 2 + st
            # AGC
            p = on[0] * on[0] + on[1] * on[1]
            err = sat((P_TARGET - p) >> 9, 18)
            self.gacc += (self.g * err) >> AGC_SH
            self.gacc = min(max(self.gacc, 1 << 15), (G_MAX << 15) | 0x7FFF)
            self.g = self.gacc >> 15
            if trace:
                self.trace.append((k, ph, self.g, self.integ))
        return np.array(out, np.int64).reshape(-1, 2)


# ---------------------------------------------------------------------------
# frame synchronisation
# ---------------------------------------------------------------------------

D_W = 12
FS_W = 2            # +-window around the predicted header
FS_DELAY = 96
FS_MAXMISS = 3
FS_NCHECK = 2
# CHECK thresholds relative to the mean metric (noise; IIR 2^-FS_NM_SH per
# symbol): each header > 4 x, both together > 10 x. At -2 dB a header is
# ~8 x the mean, the largest of a frame's 6660 noise metrics ~7 x.
FS_NM_SH = 10
# LOCKED: the header moves off the predicted position only if the metric
# there is more than FS_OFF_K x the one at the prediction
FS_OFF_K = 2
# Header confirmation by the PLS decoder. The differential metric cannot tell
# a header from data that repeats from frame to frame (a TS of null packets:
# consecutive PLFRAMEs share most of their symbols, so a data window found
# once is found again a frame length later, through CHECK and LOCKED). The
# PLS decoder decodes every tagged header coherently (128 codes, the frequency
# grid before the carrier has acquired); FS_PLS_AT symbols after a tag
# decision (the decode is done by then, the RTL waits for it otherwise) its
# result is used: "ok" = confident and the configured code or a dummy header
# (see FrameSync.pls_ok). ok: the
# next header is predicted from the decoded type (before the carrier has
# acquired, the frame sync's own type is unreliable: QPSK 1/4 without pilots
# differs from the dummy header in one differential); not ok in CHECK: back
# to SEARCH; FS_PLS_NBAD tags in a row not ok in LOCKED: lost.
FS_PLS_AT = 3072
FS_PLS_NBAD = 3
FS_CHK1 = 4
FS_CHK2 = 10
L_DUMMY = 90 + 36 * 90


def frame_len(pilots):
    return 90 + 32400 + (22 * 36 if pilots else 0)


def header_bits(modcod, pilots):
    return [sq for _, sq in m.plheader_signs(modcod, pilots)]


def diff_coefs(modcod, pilots):
    """c[i] (i = 1..89, +-1): |sum c_i d_(n-89+i)| is the header correlation."""
    b = [1 - 2 * y for y in header_bits(modcod, pilots)]
    return np.array([b[i] * b[i - 1] * (-1 if i & 1 else 1) for i in range(1, 90)], np.int64)


def fs_threshold(frac=0.2):
    """|A| threshold as a fraction of the noiseless peak (89 * 512)."""
    a = frac * 89 * (P_TARGET >> 16)
    return int(a * a) >> 8


def diffs(sym):
    s = np.asarray(sym, np.int64)
    p = np.vstack([[0, 0], s[:-1]])
    di = s[:, 0] * p[:, 0] + s[:, 1] * p[:, 1]
    dq = s[:, 1] * p[:, 0] - s[:, 0] * p[:, 1]
    lim = (1 << (D_W - 1))
    di = np.clip((di + (1 << 15)) >> 16, -lim, lim - 1)
    dq = np.clip((dq + (1 << 15)) >> 16, -lim, lim - 1)
    return di, dq


def fs_metrics(sym, coefs):
    """m[n] = |sum_i c_i d_(n-89+i)|^2 >> 8 (header ending at symbol n), 0 for n < 88."""
    di, dq = diffs(sym)
    n = len(di)
    out = np.zeros(n, np.int64)
    if n >= 89:
        # np.correlate(valid)[j] = sum_i coefs[i] d[j + i]: header ending at j + 88
        ai = np.correlate(di, coefs, "valid")
        aq = np.correlate(dq, coefs, "valid")
        out[88:] = (ai * ai + aq * aq) >> 8
    return out


class FrameSync:
    SEARCH, CHECK, LOCKED = range(3)

    def __init__(self, modcod, pilots, thresh=None):
        self.coefs = [diff_coefs(modcod, pilots), diff_coefs(0, False)]
        self.L = [frame_len(pilots), L_DUMMY]
        self.T = fs_threshold() if thresh is None else thresh
        self.code = pls_code(modcod, 0, pilots)

    def pls_ok(self, d):
        """(ok, dummy) of a PLS decode: confident, and the configured code or
        a dummy header. (Not any QPSK code of the configured length: data that
        repeats from frame to frame can decode confidently as some code, the
        same every frame; the board saw a false lock read as qpsk5/6 pilots for
        9.6 s with qpsk9/10 pilots configured.)"""
        dummy = d["modcod"] == 0
        ok = d["conf"] and (dummy or d["code"] == self.code)
        return ok, dummy

    def retype(self, typ):
        """The carrier's decision on the type of the frame tagged last
        (coherent once it has acquired the frequency): the next header is
        predicted from it."""
        if self.st != self.SEARCH and self.have_last:
            self.pred = self.last_n + self.L[typ]

    def run(self, sym, carrier=None):
        """Returns (tags, n_out): tags[i] = 0 (none), 1 (data frame starts at
        symbol i), 2 (dummy frame); self.found[i] = 1 if that tag is a
        detection (not flywheel); n_out symbols leave the delay line. With a
        carrier, every symbol leaving the delay line goes to carrier.step()
        right away (the carrier reports frame types back through retype())."""
        mets = [fs_metrics(sym, c) for c in self.coefs]
        n = len(sym)
        tags = np.zeros(n, np.int8)
        found = np.zeros(n, np.int8)
        self.found = found
        self.st = self.SEARCH
        best = None           # (m, n, c)
        self.pred = 0
        self.nchk = 0         # CHECK: headers found in a row
        self.csum = 0         # ... sum of their metrics
        chk_alt = False       # CHECK at the dummy length of a search peak
        self.have_last = False
        self.last_n = 0
        win = None
        cwin = None           # window: best at the predicted position
        miss = 0
        pk = 0                # running average of the found peaks (0: none)
        self.events = []
        if carrier is not None:
            carrier.fs = self
        self.nm = 0           # mean metric
        pls = carrier.pls if carrier is not None else None
        pls_at = None         # symbol at which the last tag's PLS decode is used
        pls_n = 0             # decodes before that tag
        nbad = 0              # LOCKED: tags in a row whose decode was not ok
        self.n_pls_drop = 0
        for k in range(n):
            if k == pls_at:
                pls_at = None
                if len(pls.results) > pls_n:
                    ok, dummy = self.pls_ok(pls.results[-1])
                    if ok and self.st != self.SEARCH and self.have_last:
                        self.pred = self.last_n + self.L[int(dummy)]
                    if self.st == self.CHECK and not ok:
                        self.st = self.SEARCH
                        best = None
                        chk_alt = False
                        self.have_last = False
                        pk = 0
                        self.n_pls_drop += 1
                        self.events.append(("pls_drop", k))
                    elif self.st == self.LOCKED:
                        nbad = 0 if ok else nbad + 1
                        if nbad == FS_PLS_NBAD:
                            self.st = self.SEARCH
                            best = None
                            pk = 0
                            self.have_last = False
                            self.n_pls_drop += 1
                            self.events.append(("lost", k))
            self.nm += (((int(mets[0][k]) + int(mets[1][k])) >> 1) - self.nm) >> FS_NM_SH
            if self.st == self.SEARCH:
                for c in (0, 1):
                    if best is None or mets[c][k] > best[0]:
                        best = (int(mets[c][k]), k, c)
                # the search peak's type is unreliable (at low SNR; and a
                # header of another MODCOD can be closer to the dummy header
                # than to the configured one: QPSK 1/4 differs from it in one
                # differential): its next header is looked for at the dummy
                # length first, then at the data length
                if k == best[1] + L_DUMMY - FS_W:
                    self.st = self.CHECK
                    self.nchk = 0
                    nbad = 0
                    chk_alt = True
                    self.pred = best[1] + L_DUMMY
                    win = None
                elif k == best[1] + self.L[0] - FS_W:
                    self.st = self.CHECK
                    self.nchk = 0
                    nbad = 0
                    chk_alt = False
                    self.pred = best[1] + self.L[0]
                    win = None
            if self.st != self.SEARCH and self.pred - FS_W <= k <= self.pred + FS_W:
                for c in (0, 1):
                    if win is None or mets[c][k] > win[0]:
                        win = (int(mets[c][k]), k, c)
                    if k == self.pred and (c == 0 or mets[c][k] > cwin[0]):
                        cwin = (int(mets[c][k]), k, c)
                if k == self.pred + FS_W:
                    if self.st == self.LOCKED and win[0] <= FS_OFF_K * cwin[0]:
                        win = cwin
                    # CHECK: FS_NCHECK headers in a row, each above the
                    # threshold, together above the search peak (at -2 dB the
                    # largest noise peak of a frame is as large as a header)
                    if self.st == self.CHECK:
                        ok = win[0] > max(self.T, FS_CHK1 * self.nm)
                        if self.nchk:
                            ok = ok and self.csum + win[0] > FS_CHK2 * self.nm
                        self.csum = self.csum + win[0] if self.nchk else win[0]
                    else:
                        ok = win[0] > max(self.T, pk >> 2)
                    if ok:
                        tags[win[1] - 89] = 1 + win[2]
                        if pls is not None:
                            pls_at, pls_n = k + FS_PLS_AT, len(pls.results)
                        found[win[1] - 89] = 1
                        self.events.append(("found", win[1] - 89, win[2], win[0]))
                        if self.st == self.CHECK:
                            self.nchk += 1
                            chk_alt = False
                        if self.st == self.LOCKED or self.nchk == FS_NCHECK:
                            self.st = self.LOCKED
                        miss = 0
                        pk = win[0] if pk == 0 else pk + ((win[0] - pk) >> 3)
                        self.last_n, self.have_last = win[1], True
                        self.pred = win[1] + self.L[win[2]]
                    elif self.st == self.CHECK:
                        self.st = self.SEARCH
                        if not chk_alt:
                            best = None         # else: wait for the data length
                        chk_alt = False
                        self.have_last = False
                        pk = 0
                    else:
                        miss += 1
                        if miss > FS_MAXMISS:
                            self.st = self.SEARCH
                            best = None
                            pk = 0
                            self.have_last = False
                            self.events.append(("lost", k))
                        else:
                            tags[self.pred - 89] = 1 + win[2]
                            if pls is not None:
                                pls_at, pls_n = k + FS_PLS_AT, len(pls.results)
                            self.events.append(("flywheel", self.pred - 89, win[2], win[0]))
                            self.last_n, self.have_last = self.pred, True
                            self.pred = self.pred + self.L[win[2]]
                    win = None
            if carrier is not None and k >= FS_DELAY:
                o = k - FS_DELAY
                carrier.step(int(sym[o][0]), int(sym[o][1]), int(tags[o]), int(found[o]))
        return tags, max(0, n - FS_DELAY)


# ---------------------------------------------------------------------------
# carrier recovery, descrambling, soft demapping
# ---------------------------------------------------------------------------

FLL_STAGES = (1, 6, 45)
FLL_ACQ_N = 4               # acquisition: FLL correlations summed over 4 headers
FLL_ACQ_NHDR = 2            # then header updates with gain 1/4, 1/8, ...
# Before acquisition the frame type comes from the frame sync's differential
# metric only, which errs at -2 dB; a header goes into the acquisition when
# the next header is detected (not flywheel) exactly its frame length later.
FLL_REACQ_N = 8             # 8 headers in a row beyond FLL_CLAMP: acquire again
FLL_HDR_SHMAX = 3           # header frequency update gain 2^-min(n, 3)
FLL_CLAMP = 1 << 22         # after acquisition: |header estimate| <= 2^-10 turn/symbol
PILOT_LOOP_SH = 2           # pilot-to-pilot frequency loop gain 1/4
# Pilot blocks are 1476 symbols apart, so the pilot loop cannot tell
# frequencies k/1476 cycles/symbol apart (k = +-1 happens at -2 dB). The
# header estimates can, averaged (each clamped to +-FA_AVG_CLAMP; gain
# 1, 1/2, 1/2, 1/4 x4, 1/8 x8, then 2^-FA_AVG_SH): k = round(average * 1476). The last data
# segment (pilot block 22 -> next PLHEADER, 783 ~ 1476/2 symbols) shows odd
# k as a phase residual near half a turn but cannot give its sign. At that
# segment the NCO is stepped by sign(average)/1476 if the residual exceeds
# FA_THRESH, or by k/1476 if |k| >= 2 (which the segment cannot see).
# Dummy frame bodies (pilots on) are 36 blocks of 90 known symbols: block to
# block phase steps (unambiguous to +-1/180 cycles/symbol) drive the NCO
# with gain 2^-DUMMY_SH, which also settles a k/1476 false lock.
DUMMY_SH = 3
DUMMY_K = (1 << 32) // 90
# Right after acquisition, FLL_EARLY_N header estimates in a row beyond
# FLL_CLAMP (the first FLL_EARLY_HDRS headers) mean a bad acquisition
# (a header of the wrong type in it): acquire again. Pilots on only: without
# pilots the header estimates are not used after the acquisition, and at -1
# dB (rms ~1e-3 cycles/symbol, FLL_CLAMP ~1e-3) two in a row beyond the clamp
# are common and threw good acquisitions away. (The confirmation of the
# acquisition headers now takes the type from the frame length, so a header
# of the wrong type no longer gets in.)
FLL_EARLY_N = 2
FLL_EARLY_HDRS = 4
FA_THRESH = round(0.3 * 65536)
FA_STEP = round((1 << 32) / 1476)
FA_AVG_SH = 4
FA_AVG_CLAMP = 4 * FA_STEP
PLL_KP_SH = 8
PLL_KI_SH = 16
# Pilots off. The PLL gains (kp, ki: the registers, designed for bandwidth
# 0.002 at high SNR) are the acquisition gains; once acquired, the loop
# narrows in gears: from nhdr >= n on (nhdr = 2 at the acquisition, +1 per
# header) the gains are kp >> a, ki >> b for each (n, a, b) of PLL_GEAR.
# Only ki narrows: the phase detector's gain falls with the SNR (AGC on
# signal + noise, decision errors: ~1/16 of the high-SNR gain at -2 dB
# Es/N0), which also lowers the damping (zeta ~ sqrt(gain)): kp / 1, ki / 16
# (with kp = PLL_KP_MUL x the design value) gives zeta ~ 1.4 at -2 dB
# (kp / 4, ki / 16, i.e. bandwidth / 4 at the design damping, is zeta ~ 0.18
# there and slipped under phase noise).
# Each header's frequency estimate (~5e-4
# cycles/symbol rms at 0 dB, ~1e-3 at -1 dB) moves the NCO only while nhdr <
# PLL_KICK_NHDR, i.e. never after the acquisition (nhdr = 2): the PLL
# integrator (much better, it averages a whole frame) is the frequency
# estimate; kicking the NCO by f/8 at every header made the loop pull in a new
# frequency error in every frame (slips), and the two kicks right after the
# acquisition (f/4, f/8; VERSION <= 0x0001_0300) left the narrowed loop
# ~1.5e-4 cycles/symbol to pull in over several frames at -1 dB (full chain,
# QPSK 1/4, 8 dummy frames before the data: 159 instead of 181 of 192 frames).
PLL_GEAR = ((4, 0, 2), (6, 0, 4))
# The default kp is twice the bandwidth-0.002 design (zeta 0.707 at high
# SNR): more damping at low SNR, which tracks phase noise better (QPSK 1/4,
# Wiener 0.003 rad/symbol, -2.2 dB: FER 3 % instead of 10 %; 0.006 rad/symbol,
# -1 dB: 5 % instead of 18 %) at the same FER without phase noise.
PLL_KP_MUL = 2
PLL_KICK_NHDR = 2
# Soft decisions for the decision-directed phase detector (pilots off):
#   e = (sq * c(si) - si * c(sq)) >> PLL_SOFT_SH,  c(x) = clamp(x, +-(2^SH - 1))
# (a limiter instead of sign(): tanh-like at low SNR, hard at high SNR;
# per-axis RMS 4096 at the AGC target)
PLL_SOFT_SH = 11
STAT_SH = 3                 # amplitude/noise IIR 1/8 per known block
SIG2_MIN = 1024
G_LLR_FRAC = 20
G_LLR_MAX = (1 << 18) - 1
CG = round(8 * math.sqrt(2) * (1 << G_LLR_FRAC))
SEG_TIMEOUT = 256           # flush a pending segment this many symbols after a frame
LLR_MAX = 127


def stat_consts(n):
    """(RA, RE): A = mag * RA >> 24, E = sum(|z|^2 >> 8) * RE >> 16."""
    return round((1 << 24) / (CORDIC_K * math.sqrt(2) * n)), round((1 << 24) / (2 * n))


def pll_gains(bw=0.002, kp_mul=PLL_KP_MUL):
    """PLL (kp, ki) for e = Im(s * conj(dec)), |s| ~ sqrt(P_TARGET): the loop
    of bandwidth bw at high SNR, kp times kp_mul."""
    kp, ki = loop_gains(bw)
    unit = math.sqrt(P_TARGET) * math.sqrt(2)          # e per radian
    sc = 2 ** 32 / (2 * math.pi * unit)
    return round(kp_mul * kp * sc * 2 ** PLL_KP_SH), round(ki * sc * 2 ** PLL_KI_SH)


def neg16(v):
    return 32767 if v == -32768 else -v


def descramble(i, q, r):
    """(i + jq) * j^-r."""
    if r == 0:
        return i, q
    if r == 1:
        return q, neg16(i)
    if r == 2:
        return neg16(i), neg16(q)
    return neg16(q), i


class Carrier:
    """Streaming carrier recovery + LLR output; see the module docstring."""

    def __init__(self, modcod, pilots, gold=0, pll=None):
        self.pilots = pilots
        self.hdr = [m.plheader_signs(modcod, pilots), m.plheader_signs(0, False)]
        self.hdiff = [a != b for a, b in zip(*self.hdr)]
        self.L = [frame_len(pilots), L_DUMMY]
        xa, xb = m.gold_init(gold)
        self.scr = m.pl_scramble_seq(frame_len(pilots) - 90, xa, xb).astype(int).tolist()
        self.kp_p, self.ki_p = pll_gains() if pll is None else pll
        self.phi = 0            # NCO phase / frequency (turns * 2^32)
        self.fhat = 0
        self.phip = 0           # PLL phase / frequency (pilots off)
        self.fp = 0
        self.nhdr = 0
        self.A = self.E = self.nblk = self.G = 0
        self.stat_rst = False       # acquisition: restart the amplitude/noise average
        self.in_frame = False
        self.typ = 1
        self.frames = []        # finished LLR frames (int8, 64800)
        self.n_muted = 0        # frames not sent (begun before the acquisition)
        self.cur = None         # output frame of the data frame being received
        self.seg = []           # pilots on: data symbols waiting for the next block
        self.seg_owner = None
        self.prev_blk = None    # (theta16, length) of the last known block
        self.idle = 0
        self.upd = 0
        self.fs = None              # frame sync to report frame types to
        self.n_retype = 0
        self.acquired = False       # frequency acquisition done
        self.conf = []              # acquisition: confirmed headers (z)
        self.ap = None              # acquisition: last header (z, type)
        self.prev_ok = False        # ... confirmed by the tag that started this frame
        self.since = 0              # symbols since the last tag
        self.big = 0                # headers in a row beyond FLL_CLAMP
        self.favg = 0               # average header frequency estimate
        self.navg = 0               # headers in it (saturates at 16)
        self.n_fa = 0               # 1/1476 steps
        self.n_acq = 0              # acquisitions
        self.n_dummy = 0            # dummy body blocks used
        self.dth = 0
        self.log = []
        self.pls = None             # PlsDecoder fed with the input symbols (detection, header confirmation)

    # ---- output ------------------------------------------------------------
    def _llr(self, buf, y):
        buf["llr"].append(sat(rnd(y[0] * self.G, G_LLR_FRAC), 8))
        buf["llr"].append(sat(rnd(y[1] * self.G, G_LLR_FRAC), 8))

    def _finish(self, buf):
        # pilots off, a data frame that began before the frequency acquisition
        # is not sent (pilots on: the frame sync's type is reliable, and the
        # frame at the acquisition header often decodes):
        # its LLRs are noise, and before the acquisition the frame type is the
        # frame sync's guess, so dummy frames taken for data frames cost the
        # LDPC decoder its full iterations and the carrier the padding of a
        # data frame (32400 words, one per clock: an input overflow at ~1 MS/s)
        llr = buf["llr"]
        llr.extend([0] * (64800 - len(llr)))
        if not buf["mute"]:
            self.frames.append(np.array(llr[:64800], np.int8))
        else:
            self.n_muted += 1

    def _frame_ended(self, buf):
        buf["ended"] = True
        if self.seg_owner is not buf:
            self._finish(buf)

    # ---- estimators --------------------------------------------------------
    def _stats(self, mag, e_sum, n):
        ra, re = stat_consts(n)
        a = (mag * ra) >> 24
        e = (e_sum * re) >> 16
        if self.stat_rst:
            # the acquisition header (summed before the new frequency): the
            # average restarts with the next known block. Before the
            # acquisition the blocks are measured at a wrong frequency (A
            # far too small); without pilots (one block per frame) the 1/8
            # IIR took ~17 frames to forget them (Es/N0 from -21 dB up).
            self.stat_rst = False
            self.nblk = 0
        else:
            # gain 1, 1/2, 1/4, then 1/8 (from one block a 1/8 IIR start
            # scales the first frames' LLRs by a single block's noise)
            sh = min(self.nblk, STAT_SH)
            self.A += (a - self.A) >> sh
            self.E += (e - self.E) >> sh
            self.nblk = min(self.nblk + 1, STAT_SH)
        sig2 = max(self.E - self.A * self.A, SIG2_MIN)
        self.G = min((self.A * CG) // sig2, G_LLR_MAX)

    @staticmethod
    def fll(zs):
        """Staged frequency estimate (turns * 2^32 per symbol) from the
        headers zs (lists of z); each stage's correlation is summed over them."""
        f = 0
        for M in FLL_STAGES:
            ri = rq = 0
            for z in zs:
                zr = []
                ph = 0
                for zi, zq in z:
                    zr.append(rot(zi, zq, ph))
                    ph = (ph - f) & 0xFFFFFFFF
                si = sq = 0
                for k, (zi, zq) in enumerate(zr):
                    # R += z_k * conj(S_k), S_k = z_(k-M) + ... + z_(k-1)
                    ri += zi * si + zq * sq
                    rq += zq * si - zi * sq
                    si += zi
                    sq += zq
                    if k >= M:
                        si -= zr[k - M][0]
                        sq -= zr[k - M][1]
            ang, _ = cordic(ri, rq)
            f += ang * round((1 << 17) / (M + 1))
        return f

    def _close_segment(self, theta, blen):
        """Phase-interpolate and output the pending data segment, now that the
        block after it is measured (theta None: timeout, constant phase)."""
        th0, len0 = self.prev_blk
        nd = len(self.seg)
        incr = 0
        if theta is not None:
            dl = wrap(theta - th0, 16)
            L = nd + (len0 + blen) // 2
            incr = (dl * ((1 << 32) // L)) >> 16
            if self.acquired:
                step = 0
                if len0 == 36 and blen == 90:
                    k = (self.favg * 1476 + (1 << 31)) >> 32
                    if abs(dl) > FA_THRESH and self.favg != 0:
                        step = FA_STEP if self.favg > 0 else -FA_STEP
                    elif abs(k) >= 2:
                        step = k * FA_STEP
                    if step:
                        self.favg -= step
                        self.n_fa += 1
                self.upd += step if step else incr >> PILOT_LOOP_SH
        ph = (th0 << 16) + ((incr * (len0 + 1)) >> 1)
        buf = self.seg_owner
        for s in self.seg:
            self._llr(buf, rot(s[0], s[1], -ph))
            ph += incr
        self.seg = []
        self.seg_owner = None
        if buf["ended"]:
            self._finish(buf)

    def _block_end(self, P, e_sum, blen, opens):
        theta, mag = cordic(P[0], P[1])
        self._stats(mag, e_sum, blen)
        if self.pilots:
            if self.seg_owner is not None:
                self._close_segment(theta, blen)
            self.prev_blk = (theta, blen)
            if opens is not None:
                self.seg_owner = opens
        return theta

    # ---- main loop ---------------------------------------------------------
    def run(self, sym, tags, found=None):
        for k in range(len(sym)):
            self.step(int(sym[k][0]), int(sym[k][1]), int(tags[k]), 0 if found is None else int(found[k]))
        return self.frames

    def step(self, si, sq, t, found=0):
        if self.pls is not None:
            # the decoder taps the carrier's input (state before this symbol)
            self.pls.step(si, sq, t, self.fhat, self.acquired)
        self.upd = 0
        if t:
            # confirmed: the next header detected a frame length later; the
            # length says the type (before the acquisition the tag's type is
            # the frame sync's differential guess, a coin toss between QPSK
            # 1/4 without pilots and the dummy header), and a header of the
            # other type has z negated where the two headers differ
            self.prev_ok = self.ap is not None and found == 1 and self.since in self.L
            if self.prev_ok and self.since != self.L[self.ap[1]]:
                z = [(neg16(zi), neg16(zq)) if self.hdiff[p] else (zi, zq)
                     for p, (zi, zq) in enumerate(self.ap[0])]
                self.ap = (z, 1 - self.ap[1])
            self.since = 0
            self._start(t - 1)
        self.since = min(self.since + 1, 65535)
        if self.in_frame:
            self._symbol(si, sq)
        else:
            self.idle += 1
            if self.seg_owner is not None and self.idle == SEG_TIMEOUT:
                self._close_segment(None, 0)
            self.phi = (self.phi + self.fhat) & 0xFFFFFFFF
            self.phip = (self.phip + self.fp) & 0xFFFFFFFF
        self.fhat += self.upd

    def _start(self, typ):
        if self.in_frame and self.typ == 0:
            self._frame_ended(self.cur)          # previous data frame cut short
        self.in_frame = True
        self.typ = typ
        self.p = 0
        self.H = [0, 0]
        self.Ha = [0, 0]            # the other frame type's header
        self.Esum = 0
        self.zbuf = []
        self.P = [0, 0]
        self.Pe = 0
        # mute: pilots off, the frame began before the frequency acquisition (not sent)
        self.cur = {"llr": [], "ended": False, "mute": not (self.acquired or self.pilots)} if typ == 0 else None

    def pll_shifts(self):
        """(kp, ki) right shifts of the PLL gains in the current gear."""
        skp = ski = 0
        for n, a, b in PLL_GEAR:
            if self.nhdr >= n:
                skp, ski = a, b
        return skp, ski

    def _pll(self, e):
        if self.pilots or not self.acquired:
            e = 0
        skp, ski = self.pll_shifts()
        self.fp += (e * (self.ki_p >> ski)) >> PLL_KI_SH
        self.phip = (self.phip + self.fp + ((e * (self.kp_p >> skp)) >> PLL_KP_SH)) & 0xFFFFFFFF

    def _symbol(self, si, sq):
        p = self.p
        typ = self.typ
        if p >= 90:
            si, sq = descramble(si, sq, self.scr[p - 90])
        s1 = rot(si, sq, -(self.phi + self.phip))
        self.phi = (self.phi + self.fhat) & 0xFFFFFFFF
        if p < 90:
            zi, zq = self._hz(s1, typ, p)           # z = s1 * conj(h) * sqrt2
            ai, aq = self._hz(s1, 1 - typ, p)
            self.zbuf.append(s1)
            self.H[0] += zi
            self.H[1] += zq
            self.Ha[0] += ai
            self.Ha[1] += aq
            self.Esum += (zi * zi + zq * zq) >> 8
            self._pll(zq)
            if p == 89:
                self._header_type()
                typ = self.typ
                f = self._header_freq()
                if not self.pilots:
                    # the decision-directed PLL's frequency goes into the NCO
                    # (same total): otherwise the two integrators share the
                    # same error and fhat random-walks, fp following it
                    self.upd += self.fp
                    self.fp = 0
                theta = self._block_end(self.H, self.Esum, 90, self.cur)
                self.dth = theta
                if not self.pilots:
                    # resolve the decision-directed PLL's 90 degree ambiguity
                    # with the header phase (not extrapolated by the header's
                    # noisy frequency estimate: the PLL has removed the
                    # residual frequency, so the phase is ~constant over it)
                    self.phip = (self.phip + (theta << 16)) & 0xFFFFFFFF
                self.log.append((self.nhdr, f, self.fhat, theta, self.A, self.E, self.G))
        elif typ == 1:
            if self.pilots and self.acquired:
                self._dummy_block(s1, p)
            self._pll(s1[1] - s1[0])               # dummy body: (1+j)/sqrt2
        else:
            j = self._body_pos(p - 90)
            if j < 0:
                zi = s1[0] + s1[1]                  # pilot: z = s1 * (1 - j)
                zq = s1[1] - s1[0]
                self.P[0] += zi
                self.P[1] += zq
                self.Pe += (zi * zi + zq * zq) >> 8
                if j == -36:
                    self._block_end(self.P, self.Pe, 36, self.cur)
                    self.P = [0, 0]
                    self.Pe = 0
            elif self.pilots:
                self.seg.append(s1)
            else:
                self._llr(self.cur, s1)
                c = (1 << PLL_SOFT_SH) - 1
                ci = max(-c, min(c, s1[0]))
                cq = max(-c, min(c, s1[1]))
                self._pll((s1[1] * ci - s1[0] * cq) >> PLL_SOFT_SH)
        self.p += 1
        if self.p == self.L[typ]:
            self.in_frame = False
            self.idle = 0
            if typ == 0:
                self._frame_ended(self.cur)

    def _dummy_block(self, s1, p):
        self.P[0] += s1[0] + s1[1]                  # z = s1 * (1 - j)
        self.P[1] += s1[1] - s1[0]
        if (p - 90) % 90 == 89:
            theta, _ = cordic(self.P[0], self.P[1])
            dl = wrap(theta - self.dth, 16)
            self.dth = theta
            self.P = [0, 0]
            d = ((dl * DUMMY_K) >> 16) >> DUMMY_SH
            self.upd += d
            self.favg -= d
            self.n_dummy += 1

    def _hz(self, s1, typ, p):
        hsi, hsq = self.hdr[typ][p]
        a, b = 1 - 2 * hsi, 1 - 2 * hsq
        return s1[0] * a + s1[1] * b, s1[1] * a - s1[0] * b

    def _header_type(self):
        """Once the frequency is acquired, the frame type (data / dummy) is
        decided by coherent correlation with both headers, and reported to
        the frame sync (which decided it from differentials)."""
        if self.acquired and self.Ha[0] ** 2 + self.Ha[1] ** 2 > self.H[0] ** 2 + self.H[1] ** 2:
            self.typ ^= 1
            self.H, self.Ha = self.Ha, self.H
            self.cur = {"llr": [], "ended": False, "mute": False} if self.typ == 0 else None
            self.n_retype += 1
        if self.fs is not None:
            self.fs.retype(self.typ)
        # header z of the decided type for the frequency estimate
        self.zh = []
        for p, s1 in enumerate(self.zbuf):
            zi, zq = self._hz(s1, self.typ, p)
            self.zh.append((sat(rnd(zi, 1), 16), sat(rnd(zq, 1), 16)))

    def _header_freq(self):
        """Frequency estimate of the header just received and NCO update."""
        if not self.acquired:
            if self.prev_ok:
                self.conf.append(self.ap[0])
            if len(self.conf) < FLL_ACQ_N:
                self.ap = (self.zh, self.typ)
                return 0
            # FLL_ACQ_N confirmed headers (the frequency has not been touched
            # since the first): full update
            f = self.fll(self.conf)
            self.upd += f
            self.acquired = True
            self.stat_rst = True
            self.ap = None
            self.conf = []
            self.nhdr = FLL_ACQ_NHDR
            self.big = 0
            self.favg = 0
            self.navg = 0
            self.n_acq += 1
            return f
        f = self.fll([self.zh])
        self.big = self.big + 1 if abs(f) > FLL_CLAMP else 0
        if self.big == FLL_REACQ_N or (self.pilots and self.big == FLL_EARLY_N and self.nhdr <= FLL_EARLY_HDRS):
            self.acquired = False               # estimates keep saying we are far off
            self.big = 0
            return 0
        self.navg = min(self.navg + 1, 16)
        ash = min(self.navg.bit_length() - 1, FA_AVG_SH)        # floor(log2(n))
        self.favg += (max(-FA_AVG_CLAMP, min(FA_AVG_CLAMP, f)) - self.favg) >> ash
        if self.nhdr > FLL_HDR_SHMAX:
            f = max(-FLL_CLAMP, min(FLL_CLAMP, f))
        if self.pilots or self.nhdr < PLL_KICK_NHDR:
            self.upd += f >> min(self.nhdr, FLL_HDR_SHMAX)
        if self.nhdr < 15:
            self.nhdr += 1
        return f

    def _body_pos(self, q):
        """Index in the slot (>= 0), or -(index + 1) inside a pilot block."""
        if not self.pilots:
            return q % 90
        r = q % (16 * 90 + 36)
        if r < 16 * 90:
            return r % 90
        return -(r - 16 * 90 + 1)


# ---------------------------------------------------------------------------
# PLS decoder: MODCOD, frame size and pilots of every header
# ---------------------------------------------------------------------------
#
# It taps the carrier's input (the frame sync output, tag on the first
# header symbol); the frame sync uses its results to confirm its headers
# (FS_PLS_AT), nothing else in the receive path does. The 90 header
# symbols are stored as r = sat6(s >> 9) (per axis RMS 8 at the AGC target)
# and decoded after the 90th (the next header is >= 3330 symbols away):
#
#   x_k = r_k * conj(h0_k) * sqrt2           h0: the PLHEADER of PLS code 0,
#                                            i.e. SOF, PLS scrambling, pi/2
#   per frequency hypothesis f (turns * 2^16 per symbol):
#     y_k = rnd(x_k * (C[n_k] - j S[n_k]), 9),  C, S = 256 cos, sin(2 pi n / 64),
#           n_k = rnd(phi_k, 10) mod 64, phi_k = k f mod 2^16
#     S   = sum_{k < 26} y_k                 (SOF)
#     Y_j = y_(26 + j), j = 0..63            (PLS; pair i = j >> 1)
#     pass q = 0..15 (bits 0..3 of a = the bit-reversed MODCOD), b6 = pilots:
#       P1, P2 = sum over j < 32 / j >= 32 of (-1)^(parity(i & q) ^ (b6 & j)) Y_j
#       a4 = bit 4 of a (the 16-pair halves), b5 = short frame (all pairs):
#         T = S + (-1)^b5 (P1 + (-1)^a4 P2),  M = (Ti^2 + Tq^2) >> 8
#   M ~ |sum_k conj(c_k) y_k|^2 over the 90 symbols of codeword c: the
#   maximum-likelihood decision for an unknown carrier phase (constant over
#   the header, i.e. a known frequency).
#
# Hypotheses: the carrier's frequency (fhat >> 16, if it has
# acquired) first; if that gives the configured PLS code or the dummy code
# with confidence, the decode ends there. Otherwise the grid h * step,
# h = -H..H, around 0 (the DDC frequency): the frequency must be known to
# ~+-0.002 cycles/symbol, which the carrier gets only with the right
# MODCOD and pilots (with the wrong ones it acquires a wrong frequency), so
# a detection without it needs the grid. Result: the first maximum (order:
# hypothesis, q, b6, a4, b5), second = the largest metric of another code
# at the same hypothesis (another code at another frequency gets ~1/2 of a
# noiseless header's metric, at the same frequency <= 0.18), confident =
# best > 2 x second and best > 2^8. The carrier hypothesis ends the
# decode when its code is the configured or the dummy one and best > 2^8.
# At low SNR a wrong code at a wrong grid frequency can win with a large
# metric (3e-4 of the headers at -2.4 dB with H = 32): such errors look
# confident, so a detection should want a few equal decodes in a row.

PLS_RB = 9                  # header symbols: sat6(s >> 9)
PLS_PHB = 6                 # rotation phases 2^6
PLS_H_DEFAULT = 32          # grid -32..32 x 0.004 = +-0.128 cycles/symbol
PLS_STEP_DEFAULT = 262      # 0.004 turns/symbol * 2^16
PLS_CONF_SH = 1             # confident: best > second << 1 ...
PLS_MIN_SH = 8              # ... and best > 2^8 (AGC level: a header at -3 dB Es/N0
                            # gives >= ~300, noise alone the largest of 128 x 65 <= ~220)
PLS_SOF = 26
PLS_C = [round(256 * math.cos(2 * math.pi * n / (1 << PLS_PHB))) for n in range(1 << PLS_PHB)]
PLS_S = [round(256 * math.sin(2 * math.pi * n / (1 << PLS_PHB))) for n in range(1 << PLS_PHB)]
PLS_H0 = m.plheader_bits(0, False)          # SOF + PLSC scrambling sequence


def rev5(v):
    return int(f"{v & 31:05b}"[::-1], 2)


def pls_code(modcod, short, pilots):
    """7-bit PLS code: MODCOD << 2 | short frame << 1 | pilots."""
    return (modcod << 2) | (int(short) << 1) | int(pilots)


def _pls_tables():
    j = np.arange(64)
    sg = np.empty((16, 2, 64), np.int64)
    for q in range(16):
        for b6 in range(2):
            par = np.array([bin((jj >> 1) & q).count("1") & 1 for jj in j])
            sg[q, b6] = 1 - 2 * (par ^ (b6 & j & 1))
    codes = np.empty((16, 2, 2, 2), np.int64)               # q, b6, a4, b5
    for q in range(16):
        for b6 in range(2):
            for a4 in range(2):
                for b5 in range(2):
                    codes[q, b6, a4, b5] = pls_code(rev5(q | (a4 << 4)), b5, b6)
    return sg, codes.reshape(-1)


_PLS_SG, _PLS_CODES = _pls_tables()
_PLS_CA = np.array(PLS_C, np.int64)
_PLS_SA = np.array(PLS_S, np.int64)
_PLS_K = np.arange(90, dtype=np.int64)


def pls_quant(s):
    """Header symbol (16-bit I or Q) -> 6 bit."""
    return sat(int(s) >> PLS_RB, 6)


def pls_strip(r):
    """x_k = r_k * conj(h0_k) * sqrt2 (r: (90, 2) 6-bit values)."""
    r = np.asarray(r, np.int64)
    ri, rq = r[:, 0], r[:, 1]
    ev = (_PLS_K & 1) == 0
    xi = np.where(ev, ri + rq, rq - ri)
    xq = np.where(ev, rq - ri, -ri - rq)
    sg = 1 - 2 * np.array(PLS_H0, np.int64)
    return xi * sg, xq * sg


def pls_metrics(r, freqs):
    """Metrics (len(freqs), 128) in decoding order (codes: _PLS_CODES) of the
    stored header r (90 x 2) for the frequency hypotheses (turns * 2^16)."""
    xi, xq = pls_strip(r)
    f = np.asarray(freqs, np.int64).reshape(-1, 1)
    ph = (_PLS_K[None, :] * f) & 0xFFFF
    n = ((ph + (1 << (15 - PLS_PHB))) >> (16 - PLS_PHB)) & ((1 << PLS_PHB) - 1)
    c, s = _PLS_CA[n], _PLS_SA[n]
    yi = (xi * c + xq * s + 256) >> 9
    yq = (xq * c - xi * s + 256) >> 9
    Si = yi[:, :PLS_SOF].sum(1)
    Sq = yq[:, :PLS_SOF].sum(1)
    Yi, Yq = yi[:, PLS_SOF:], yq[:, PLS_SOF:]
    # P[h, q, b6, half]
    P1i = np.einsum("qbj,hj->hqb", _PLS_SG[:, :, :32], Yi[:, :32])
    P1q = np.einsum("qbj,hj->hqb", _PLS_SG[:, :, :32], Yq[:, :32])
    P2i = np.einsum("qbj,hj->hqb", _PLS_SG[:, :, 32:], Yi[:, 32:])
    P2q = np.einsum("qbj,hj->hqb", _PLS_SG[:, :, 32:], Yq[:, 32:])
    sg = np.array([1, -1])
    # T[h, q, b6, a4, b5] = S + s5 (P1 + s4 P2)
    Ti = Si[:, None, None, None, None] + sg[None, None, None, None, :] * (
        P1i[:, :, :, None, None] + sg[None, None, None, :, None] * P2i[:, :, :, None, None])
    Tq = Sq[:, None, None, None, None] + sg[None, None, None, None, :] * (
        P1q[:, :, :, None, None] + sg[None, None, None, :, None] * P2q[:, :, :, None, None])
    return ((Ti * Ti + Tq * Tq) >> 8).reshape(len(f), 128)


def pls_select(mets):
    """mets (hypotheses, 128) -> (best, index, second): the first maximum,
    and the largest metric of another code at the same hypothesis (the
    RTL's running update gives the same)."""
    mets = np.asarray(mets)
    lb = mets.max(1)
    h = int(np.argmax(lb))
    row = mets[h]
    c = int(np.argmax(row))
    return int(row[c]), h * 128 + c, int(np.delete(row, c).max())


def pls_confident(best, second):
    return best > (second << PLS_CONF_SH) and best > (1 << PLS_MIN_SH)


def pls_decode(r, fhat=0, acquired=False, cfg_code=None, H=PLS_H_DEFAULT, step=PLS_STEP_DEFAULT):
    """Decode one stored header r (90 x 2, 6-bit values). Returns a dict."""
    hyps = []
    if acquired:
        hyps.append((fhat >> 16) & 0xFFFF)
    grid = [(h * step) & 0xFFFF for h in range(-H, H + 1)]
    mets = np.zeros((0, 128), np.int64)
    used = []
    grid_used = True
    if acquired:
        mets = pls_metrics(r, hyps)
        used = list(hyps)
        best, bi, second = pls_select(mets)
        c = int(_PLS_CODES[bi])
        if best > (1 << PLS_MIN_SH) and (c == cfg_code or c == 0):
            grid_used = False
    if grid_used:
        mets = np.vstack([mets, pls_metrics(r, grid)])
        used += grid
    best, bi, second = pls_select(mets)
    c = int(_PLS_CODES[bi % 128])
    f = used[bi // 128]
    return dict(code=c, modcod=c >> 2, short=(c >> 1) & 1, pilots=c & 1, best=best, second=second,
                conf=pls_confident(best, second), freq=f - 65536 if f >= 32768 else f,
                carrier_hyp=acquired and bi < 128, grid=grid_used, nhyp=len(used))


class PlsDecoder:
    """Streaming PLS decoder (see above); results in self.results."""

    def __init__(self, modcod, pilots, H=PLS_H_DEFAULT, step=PLS_STEP_DEFAULT):
        self.cfg = (modcod, int(pilots))
        self.H, self.step_ = H, step
        self.buf = None
        self.results = []

    def step(self, si, sq, t, fhat, acquired):
        if t:
            self.buf = []
            self.f0, self.acq = fhat, acquired
        if self.buf is not None:
            self.buf.append((pls_quant(si), pls_quant(sq)))
            if len(self.buf) == 90:
                d = pls_decode(self.buf, self.f0, self.acq, pls_code(self.cfg[0], 0, self.cfg[1]),
                               self.H, self.step_)
                self.results.append(d)
                self.buf = None

    def status(self, upto=None):
        """The RTL's status registers after the first `upto` decodes."""
        res = self.results[:upto]
        st = dict(count=len(res), data_count=0, diff_modcod=0, diff_pilots=0, seen=0,
                  last=res[-1] if res else None, data=None)
        for d in res:
            if not d["conf"]:
                continue
            if d["modcod"] != 0:
                st["data"] = d
                st["data_count"] += 1
                st["diff_modcod"] += (d["modcod"], d["short"]) != (self.cfg[0], 0)
                st["diff_pilots"] += d["pilots"] != self.cfg[1]
            if not d["short"]:
                st["seen"] |= 1 << d["modcod"]
        return st


# ---------------------------------------------------------------------------
# whole receiver
# ---------------------------------------------------------------------------

def receive(x, modcod, pilots, rolloff=0.35, gold=0, bw=None, pls=None, integ0=0):
    """pls: dict(H=, step=) of the PLS decoder (default: the defaults);
    integ0: timing integrator start value (TIMING_INTEG write)."""
    bw = TIMING_BW if bw is None else bw
    kp, ki = timing_gains(bw)
    tr = Timing(timing_coefs(rolloff), kp, ki, integ0=integ0)
    sym = tr.run(x)
    fs = FrameSync(modcod, pilots)
    cr = Carrier(modcod, pilots, gold)
    cr.pls = PlsDecoder(modcod, pilots, **(pls or {}))
    tags, nout = fs.run(sym, cr)
    return dict(sym=sym, tags=tags[:nout], frames=cr.frames, timing=tr, fs=fs, carrier=cr, pls=cr.pls)


# ---------------------------------------------------------------------------
# constants for the RTL
# ---------------------------------------------------------------------------

def write_rx_pkg(path=None):
    import os
    path = path or os.path.join(m.HERE, "..", "rtl", "gen", "dvbs2_rx_pkg.sv")
    assert PLL_SOFT_SH == 11            # the RTL clamp (SOFT_C, 12-bit ci, cq)
    ra36, re36 = stat_consts(36)
    ra90, re90 = stat_consts(90)
    mul = [round((1 << 17) / (M + 1)) for M in FLL_STAGES]
    kp, ki = timing_gains()
    pkp, pki = pll_gains()
    lines = ["// Generated by model/rxfixed.py write_rx_pkg(): receiver constants.",
             "package dvbs2_rx_pkg;"]
    lines.append(f"  localparam int CORDIC_IT = {CORDIC_IT};")
    lines.append("  localparam logic [19:0] CORDIC_ATAN [CORDIC_IT] = '{" +
                 ", ".join(f"20'd{a}" for a in _ATAN) + "};")
    lines += [f"  localparam logic [17:0] STAT_RA36 = 18'd{ra36};",
              f"  localparam logic [17:0] STAT_RE36 = 18'd{re36};",
              f"  localparam logic [17:0] STAT_RA90 = 18'd{ra90};",
              f"  localparam logic [17:0] STAT_RE90 = 18'd{re90};",
              f"  localparam logic [23:0] LLR_CG = 24'd{CG};",
              "  localparam logic [16:0] FLL_MUL [3] = '{" + ", ".join(f"17'd{v}" for v in mul) + "};",
              "  localparam int FLL_M [3] = '{" + ", ".join(str(v) for v in FLL_STAGES) + "};",
              f"  localparam logic [15:0] TIMING_KP_DEFAULT = 16'd{kp};",
              f"  localparam logic [15:0] TIMING_KI_DEFAULT = 16'd{ki};",
              f"  localparam logic [17:0] PLL_KP_DEFAULT = 18'd{pkp};",
              f"  localparam logic [17:0] PLL_KI_DEFAULT = 18'd{pki};",
              "  // pilots off: PLL gears (from nhdr >= N: kp >> KP_SH, ki >> KI_SH), header",
              "  // frequency updates while nhdr < PLL_KICK_NHDR, soft phase detector",
              f"  localparam int PLL_NGEAR = {len(PLL_GEAR)};",
              "  localparam logic [3:0] PLL_GEAR_N [PLL_NGEAR] = '{" + ", ".join(f"4'd{g[0]}" for g in PLL_GEAR) + "};",
              "  localparam logic [2:0] PLL_GEAR_KP_SH [PLL_NGEAR] = '{" + ", ".join(f"3'd{g[1]}" for g in PLL_GEAR) + "};",
              "  localparam logic [2:0] PLL_GEAR_KI_SH [PLL_NGEAR] = '{" + ", ".join(f"3'd{g[2]}" for g in PLL_GEAR) + "};",
              f"  localparam logic [3:0] PLL_KICK_NHDR = 4'd{PLL_KICK_NHDR};",
              f"  localparam int PLL_SOFT_SH = {PLL_SOFT_SH};",
              f"  localparam logic [31:0] FS_THRESH_DEFAULT = 32'd{fs_threshold()};",
              f"  localparam logic signed [15:0] FA_THRESH = 16'sd{FA_THRESH};",
              f"  localparam logic signed [31:0] FA_STEP = 32'sd{FA_STEP};",
              f"  localparam logic signed [35:0] FA_AVG_CLAMP = 36'sd{FA_AVG_CLAMP};",
              f"  localparam logic [26:0] DUMMY_K = 27'd{DUMMY_K};",
              f"  localparam logic [5:0] PLS_H_DEFAULT = 6'd{PLS_H_DEFAULT};",
              f"  localparam logic [9:0] PLS_STEP_DEFAULT = 10'd{PLS_STEP_DEFAULT};",
              "  // PLS decoder rotation table {-256 sin, 256 sin, 256 cos} (10 bit each)",
              "  localparam logic [29:0] PLS_TAB [64] = '{" + ", ".join(
                  f"30'h{((-s & 1023) << 20) | ((s & 1023) << 10) | (c & 1023):08x}"
                  for c, s in zip(PLS_C, PLS_S)) + "};",
              "endpackage", ""]
    with open(path, "w") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    write_rx_pkg()
