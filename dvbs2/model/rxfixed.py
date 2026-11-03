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
                          headers and dummy frames), phase set from every
                          PLHEADER (removes the 90 degree ambiguity).
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
# DDC: NCO + polyphase decimating FIR (ADC rate -> 2 samples/symbol)
# ---------------------------------------------------------------------------

DDC_NMAX = 512      # coefficients
DDC_CW = 18


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


def ddc(x, coefs, dec, finc=0, shift=16):
    """y[k] = sat16(rnd(sum_i h[i] * m[k*dec + dec-1 - i], shift)), where
    m[n] = x[n] * exp(j*2*pi*n*finc/2^32) (rot(), same LUT as the NCOs)."""
    x = np.asarray(x, np.int64)
    if finc:
        mx = np.empty_like(x)
        ph = 0
        for n, (i, q) in enumerate(x):
            mx[n] = rot(int(i), int(q), ph)
            ph = (ph + finc) & 0xFFFFFFFF
        x = mx
    h = np.asarray(coefs, np.int64)
    nt = len(h)
    xp = np.vstack([np.zeros((nt, 2), np.int64), x])       # zeros before the first sample
    out = []
    for k in range(len(x) // dec):
        n = k * dec + dec - 1 + nt
        seg = xp[n - nt + 1:n + 1][::-1]                    # x[n], x[n-1], ...
        acc = h @ seg
        r = [(int(a) + (1 << (shift - 1))) >> shift if shift else int(a) for a in acc]
        out.append([sat(r[0], 16), sat(r[1], 16)])
    return np.array(out, np.int64).reshape(-1, 2)


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


class Timing:
    def __init__(self, coefs, kp, ki, g_init=G_INIT):
        self.c = np.asarray(coefs, np.int64)
        self.kp, self.ki = kp, ki
        self.g = g_init
        self.gacc = g_init << 15
        self.mu = 0
        self.integ = 0
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
        chk_alt = False       # CHECK at the dummy length of a "data" search peak
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
        for k in range(n):
            self.nm += (((int(mets[0][k]) + int(mets[1][k])) >> 1) - self.nm) >> FS_NM_SH
            if self.st == self.SEARCH:
                for c in (0, 1):
                    if best is None or mets[c][k] > best[0]:
                        best = (int(mets[c][k]), k, c)
                # the search peak's type is unreliable at low SNR: its next
                # header is looked for at the dummy length first
                if k == best[1] + L_DUMMY - FS_W:
                    self.st = self.CHECK
                    self.nchk = 0
                    chk_alt = best[2] == 0
                    self.pred = best[1] + L_DUMMY
                    win = None
                elif best[2] == 0 and k == best[1] + self.L[0] - FS_W:
                    self.st = self.CHECK
                    self.nchk = 0
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
# (a header of the wrong type in it): acquire again.
FLL_EARLY_N = 2
FLL_EARLY_HDRS = 4
FA_THRESH = round(0.3 * 65536)
FA_STEP = round((1 << 32) / 1476)
FA_AVG_SH = 4
FA_AVG_CLAMP = 4 * FA_STEP
PLL_KP_SH = 8
PLL_KI_SH = 16
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


def pll_gains(bw=0.002):
    """PLL (kp, ki) for e = Im(s * conj(dec)), |s| ~ sqrt(P_TARGET)."""
    kp, ki = loop_gains(bw)
    unit = math.sqrt(P_TARGET) * math.sqrt(2)          # e per radian
    sc = 2 ** 32 / (2 * math.pi * unit)
    return round(kp * sc * 2 ** PLL_KP_SH), round(ki * sc * 2 ** PLL_KI_SH)


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


def sgn(v):
    return 1 if v >= 0 else -1


class Carrier:
    """Streaming carrier recovery + LLR output; see the module docstring."""

    def __init__(self, modcod, pilots, gold=0, pll=None):
        self.pilots = pilots
        self.hdr = [m.plheader_signs(modcod, pilots), m.plheader_signs(0, False)]
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
        self.in_frame = False
        self.typ = 1
        self.frames = []        # finished LLR frames (int8, 64800)
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

    # ---- output ------------------------------------------------------------
    def _llr(self, buf, y):
        buf["llr"].append(sat(rnd(y[0] * self.G, G_LLR_FRAC), 8))
        buf["llr"].append(sat(rnd(y[1] * self.G, G_LLR_FRAC), 8))

    def _finish(self, buf):
        llr = buf["llr"]
        llr.extend([0] * (64800 - len(llr)))
        self.frames.append(np.array(llr[:64800], np.int8))

    def _frame_ended(self, buf):
        buf["ended"] = True
        if self.seg_owner is not buf:
            self._finish(buf)

    # ---- estimators --------------------------------------------------------
    def _stats(self, mag, e_sum, n):
        ra, re = stat_consts(n)
        a = (mag * ra) >> 24
        e = (e_sum * re) >> 16
        if self.nblk == 0:
            self.A, self.E = a, e
        else:
            self.A += (a - self.A) >> STAT_SH
            self.E += (e - self.E) >> STAT_SH
        self.nblk += 1
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
        self.upd = 0
        if t:
            self.prev_ok = self.ap is not None and self.since == self.L[self.ap[1]] and found == 1
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
        self.cur = {"llr": [], "ended": False} if typ == 0 else None

    def _pll(self, e):
        if self.pilots or not self.acquired:
            e = 0
        self.fp += (e * self.ki_p) >> PLL_KI_SH
        self.phip = (self.phip + self.fp + ((e * self.kp_p) >> PLL_KP_SH)) & 0xFFFFFFFF

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
                theta = self._block_end(self.H, self.Esum, 90, self.cur)
                self.dth = theta
                if not self.pilots:
                    # resolve the decision-directed PLL's 90 degree ambiguity:
                    # header phase, extrapolated from its centre to its end
                    self.phip = (self.phip + (theta << 16) + ((f * 89) >> 1)) & 0xFFFFFFFF
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
                self._pll(s1[1] * sgn(s1[0]) - s1[0] * sgn(s1[1]))
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
            self.cur = {"llr": [], "ended": False} if self.typ == 0 else None
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
        if self.big == FLL_REACQ_N or (self.big == FLL_EARLY_N and self.nhdr <= FLL_EARLY_HDRS):
            self.acquired = False               # estimates keep saying we are far off
            self.big = 0
            return 0
        self.navg = min(self.navg + 1, 16)
        ash = min(self.navg.bit_length() - 1, FA_AVG_SH)        # floor(log2(n))
        self.favg += (max(-FA_AVG_CLAMP, min(FA_AVG_CLAMP, f)) - self.favg) >> ash
        if self.nhdr > FLL_HDR_SHMAX:
            f = max(-FLL_CLAMP, min(FLL_CLAMP, f))
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
# whole receiver
# ---------------------------------------------------------------------------

def receive(x, modcod, pilots, rolloff=0.35, gold=0, bw=None):
    bw = TIMING_BW if bw is None else bw
    kp, ki = timing_gains(bw)
    tr = Timing(timing_coefs(rolloff), kp, ki)
    sym = tr.run(x)
    fs = FrameSync(modcod, pilots)
    cr = Carrier(modcod, pilots, gold)
    tags, nout = fs.run(sym, cr)
    return dict(sym=sym, tags=tags[:nout], frames=cr.frames, timing=tr, fs=fs, carrier=cr)


# ---------------------------------------------------------------------------
# constants for the RTL
# ---------------------------------------------------------------------------

def write_rx_pkg(path=None):
    import os
    path = path or os.path.join(m.HERE, "..", "rtl", "gen", "dvbs2_rx_pkg.sv")
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
              f"  localparam logic [31:0] FS_THRESH_DEFAULT = 32'd{fs_threshold()};",
              f"  localparam logic signed [15:0] FA_THRESH = 16'sd{FA_THRESH};",
              f"  localparam logic signed [31:0] FA_STEP = 32'sd{FA_STEP};",
              f"  localparam logic signed [35:0] FA_AVG_CLAMP = 36'sd{FA_AVG_CLAMP};",
              f"  localparam logic [26:0] DUMMY_K = 27'd{DUMMY_K};",
              "endpackage", ""]
    with open(path, "w") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    write_rx_pkg()
