"""IQ transmitter: RTL (Verilator, tb_iqtx_axi.sv) against the bit-exact
model, and filter / NCO / ramp quality checks on the model."""
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
RTL = os.path.join(HERE, "..", "rtl")
BUILD = os.path.join(HERE, "build")
sys.path.insert(0, os.path.join(HERE, "..", "model"))
import iqtx as m  # noqa: E402

FS = m.FS
CLK = 100e6
RB = 0x1000_0000            # ring base in the memory model

# registers
CTRL, TX_SRC, STATUS, RATE, GAIN, FINC, RAMP = 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20
RING_BASE, RING_MASK, WPTR, RPTR, LOWATER = 0x24, 0x28, 0x2C, 0x30, 0x34
IRQ_EN, IRQ_STATUS, SAMPLES, UNDERRUNS, DAC_UNDERRUNS, INFO = 0x38, 0x3C, 0x40, 0x44, 0x48, 0x4C
ST_ACT, ST_PEND, ST_BUSY = 0x3, 0x4, 0x8


@pytest.fixture(scope="module")
def exe():
    obj = os.path.join(BUILD, "tb_iqtx_axi")
    os.makedirs(obj, exist_ok=True)
    src = [os.path.join(RTL, f) for f in ("iqtx_rd.sv", "iqtx_fir.sv", "iqtx_cic.sv", "iqtx_axi.sv")]
    src += [os.path.join(HERE, "..", "..", "dvbs2", "rtl", "dvbs2_async_fifo.sv"),
            os.path.join(HERE, "tb_iqtx_axi.sv")]
    subprocess.run(["verilator", "--binary", "--timing", "-j", "16", "-O3", "-Wno-fatal",
                    "-Wno-WIDTHTRUNC", "-Wno-WIDTHEXPAND", "--top-module", "tb_iqtx_axi",
                    "-Mdir", obj] + src, check=True, stdout=subprocess.DEVNULL)
    return os.path.join(obj, "Vtb_iqtx_axi")


class Cmd:
    def __init__(self):
        self.lines = []

    def w(self, a, d):
        self.lines.append(f"W {a:x} {d & 0xFFFFFFFF:x}")

    def r(self, a):
        self.lines.append(f"R {a:x}")

    def mem(self, a, d):
        self.lines.append(f"M {a:x} {d & 0xFFFFFFFF:x}")

    def wait(self, n):
        self.lines.append(f"D {int(n)}")

    def status(self, mask, val):
        self.lines.append(f"S {mask:x} {val:x}")

    def poll(self, a, v):
        self.lines.append(f"P {a:x} {v & 0xFFFFFFFF:x}")

    def dvb(self, on):
        self.lines.append(f"V {int(on)}")

    def ring(self, base, off, words):
        for k, wd in enumerate(words):
            self.mem(base + off + 4 * k, int(wd))


def run(exe, tmp_path, cmd, stall=0, trace=0):
    fc = tmp_path / "cmd.txt"
    fc.write_text("\n".join(cmd.lines + ["E"]) + "\n")
    fo = tmp_path / "out.txt"
    subprocess.run([exe, f"+cmd={fc}", f"+out={fo}", f"+stall={stall}", f"+trace={trace}"],
                   check=True, capture_output=True, cwd=HERE, timeout=1200)
    res = {"R": [], "O": [], "X": [], "T": []}
    for line in fo.read_text().split("\n"):
        p = line.split()
        if not p:
            continue
        if p[0] == "R":
            res["R"].append((int(p[1], 16), int(p[2], 16)))
        elif p[0] == "X":
            res["X"].append((int(p[1], 16), int(p[2], 16), int(p[3])))
        else:
            res[p[0]].append(int(p[1], 16))
    return res


def split_iq(w):
    w = np.asarray(w, np.int64)
    return (w & 0xFFFF).astype(np.int16).astype(np.int64), (w >> 16).astype(np.int16).astype(np.int64)


def pack(i, q):
    return (np.asarray(i, np.int64) & 0xFFFF) | ((np.asarray(q, np.int64) & 0xFFFF) << 16)


def signal(n, amp=0.9, seed=1, ntone=4, band=0.38):
    """Complex test signal: a few tones within +-band x input rate, no zero words."""
    rng = np.random.default_rng(seed)
    t = np.arange(n)
    x = np.zeros(n, complex)
    for _ in range(ntone):
        x += np.exp(1j * (2 * np.pi * rng.uniform(-band, band) * t + rng.uniform(0, 6.3)))
    x *= amp * 32767 / ntone
    i = np.round(x.real).astype(np.int64)
    q = np.round(x.imag).astype(np.int64)
    w = pack(i, q)
    w[w == 0] = 1
    return w


def clocks(nsamp, rate):
    """Clocks for nsamp input samples."""
    return int(nsamp * 5 * rate * CLK / FS)


def setup(c, rate, gain=0x4000, finc=0, rlog=4, mask=0xFFFF, lowater=0, irq_en=0):
    c.w(RATE, rate)
    c.w(GAIN, gain)
    c.w(FINC, finc)
    c.w(RAMP, rlog)
    c.w(RING_BASE, RB)
    c.w(RING_MASK, mask)
    c.w(LOWATER, lowater)
    c.w(IRQ_EN, irq_en)
    c.w(TX_SRC, 1)
    c.status(ST_ACT, 1)


def check(res, ring_words, rate, gain, finc, rlog):
    """X trace against the ring data and the model's ramp, the IQ samples
    against the model chain. Returns (n played, n underrun, n_stop)."""
    xs = np.array([x[0] for x in res["X"]], np.int64)
    rs = np.array([x[1] for x in res["X"]], np.int64)
    st = np.array([x[2] for x in res["X"]])
    play = st != 3                                 # not the flush
    assert (st[play] != 0).all()
    n = int(play.sum())
    assert (~play[n:]).all() and (st[n:] == 3).all()
    assert len(st) - n in (0, m.FLUSH)
    down = np.nonzero(st[:n] == 2)[0]
    n_stop = int(down[0]) if len(down) else None
    # ramp
    want_r = m.ramp_seq(n if n_stop is None else n + (1 << 20), n_stop, rlog)
    assert len(want_r) == n
    assert (rs[:n] == want_r).all()
    assert (rs[n:] == 0).all()
    # data: the ring in order, zeros for underruns
    taken = xs[:n] != 0
    k = int(taken.sum())
    assert (xs[:n][taken] == np.asarray(ring_words[:k], np.int64)).all()
    # samples
    xi, xq = split_iq(xs)
    ei, eq = m.envelope(xi, xq, rs, gain, rate)
    zi, zq = m.chain_env(ei, eq, rate, finc)
    oi, oq = split_iq(res["O"])
    assert len(oi) > 0
    lim = min(len(oi), len(zi))
    bad = np.nonzero((oi[:lim] != zi[:lim]) | (oq[:lim] != zq[:lim]))[0]
    assert len(bad) == 0, f"first mismatch at {bad[0]} of {lim}: rtl {oi[bad[0]]},{oq[bad[0]]} model {zi[bad[0]]},{zq[bad[0]]}"
    assert (oi[lim:] == 0).all() and (oq[lim:] == 0).all()
    if len(st) > n:       # a complete run: what is not played is zero
        assert (zi[lim:] == 0).all() and (zq[lim:] == 0).all()
    return n, n - k, n_stop


def regs(res):
    return {a: d for a, d in res["R"]}


# ---------------------------------------------------------------- RTL tests
def test_regs(exe, tmp_path):
    c = Cmd()
    for a, d in ((RATE, 123), (GAIN, 0x1234), (FINC, 0x89ABCDEF), (RAMP, 7), (RING_BASE, 0x3E700000),
                 (RING_MASK, 0xFFFFF), (WPTR, 0x40), (LOWATER, 0x8000), (IRQ_EN, 5), (TX_SRC, 3)):
        c.w(a, d)
    for a in (0x00, 0x04, CTRL, TX_SRC, STATUS, RATE, GAIN, FINC, RAMP, RING_BASE, RING_MASK, WPTR, RPTR,
              LOWATER, IRQ_EN, IRQ_STATUS, SAMPLES, UNDERRUNS, DAC_UNDERRUNS, INFO, 0x50, 0x100):
        c.r(a)
    c.status(ST_ACT, 2)
    c.r(STATUS)
    r = run(exe, tmp_path, c)
    v = r["R"]
    want = [0x00010000, 0x49515458, 0, 2, None, 123, 0x1234, 0x89ABCDEF, 7, 0x3E700000, 0xFFFFF, 0x40, 0,
            0x8000, 5, 0, 0, 0, 0, (400 << 16) | (8 << 8) | 5, 0, 0]
    for (a, d), w in zip(v, want):
        if w is not None:
            assert d == w, f"0x{a:x}: {d:#x} != {w:#x}"
    assert v[-1][1] & 0xFF == 0x42          # off active, DVB silent, nothing pending


@pytest.mark.parametrize("rate,rlog,gain,finc,stall", [
    (50, 6, 0x4000, 0x0A3D70A4, 0),        # 48 kS/s, +480 kHz
    (8, 8, 0x2C00, -0x23456789, 30),       # 300 kS/s, slave stalls
    (200, 3, 0x7FFF, 0x40000000, 0),       # 12 kS/s, gain 2 (clips), +3 MHz
    (37, 0, 0x4000, 0x1234567, 10),        # odd R, no ramp
    (400, 2, 0x4000, -0x0A3D70A4, 0),      # 6 kS/s, the widest CIC
])
def test_play(exe, tmp_path, rate, rlog, gain, finc, stall):
    """Prefilled ring, start, stop: ramp, data order, every DAC sample."""
    n = 160 if rate >= 400 else 400 if rate >= 50 else 896
    words = signal(n, seed=rate)
    c = Cmd()
    setup(c, rate, gain, finc, rlog)
    c.ring(RB, 0, words)
    c.w(WPTR, 4 * n)
    c.w(CTRL, 1)
    c.wait(clocks(n - (1 << rlog) - 80, rate))
    c.w(CTRL, 0)
    c.status(ST_BUSY, 0)
    c.r(SAMPLES)
    c.r(UNDERRUNS)
    c.r(RPTR)
    c.r(IRQ_STATUS)
    c.r(DAC_UNDERRUNS)
    res = run(exe, tmp_path, c, stall=stall)
    played, unf, n_stop = check(res, words, rate, gain, finc, rlog)
    assert unf == 0 and n_stop is not None
    v = [d for _, d in res["R"]]
    assert v[0] == played and v[1] == 0
    assert v[2] == 4 * n                    # stopped: the rest of the ring is dropped
    assert v[3] & 2                          # IDLE event
    assert v[4] == 0


def test_underrun_refill(exe, tmp_path):
    """The ring runs dry (zeros, counted, IRQ), then more data: played in order."""
    rate, rlog = 25, 4
    a = signal(256, seed=3)
    b = signal(320, seed=4)
    c = Cmd()
    setup(c, rate, rlog=rlog, irq_en=7)
    c.ring(RB, 0, a)
    c.w(WPTR, 4 * len(a))
    c.w(CTRL, 1)
    c.wait(clocks(len(a) + 50, rate))
    c.r(UNDERRUNS)
    c.r(IRQ_STATUS)
    c.ring(RB, 4 * len(a), b)
    c.w(WPTR, 4 * (len(a) + len(b)))
    c.wait(2000)
    c.w(IRQ_STATUS, 7)
    c.wait(clocks(200, rate))
    c.w(CTRL, 0)
    c.status(ST_BUSY, 0)
    c.r(UNDERRUNS)
    c.r(IRQ_STATUS)
    res = run(exe, tmp_path, c)
    played, unf, n_stop = check(res, np.concatenate([a, b]), rate, 0x4000, 0, rlog)
    v = [d for _, d in res["R"]]
    assert v[0] >= 40 and v[1] & 4
    assert v[2] == unf and unf >= v[0]
    assert v[3] & 2 and not v[3] & 4


def test_ring_wrap(exe, tmp_path):
    """4 KB ring refilled in halves as RPTR advances, pointers across 2^32;
    the low-water interrupt."""
    rate, rlog, half = 10, 5, 512
    words = signal(half * 6, seed=7)
    p0 = 0xFFFFE000
    c = Cmd()
    setup(c, rate, rlog=rlog, mask=0xFFF, lowater=1024, irq_en=1)
    # move the pointers: a start without data and a stop set RPTR to WPTR
    c.w(WPTR, p0)
    c.w(CTRL, 1)
    c.wait(200)
    c.w(CTRL, 0)
    c.status(ST_BUSY, 0)
    c.r(RPTR)
    c.ring(RB, 0, words[:2 * half])
    c.w(WPTR, p0 + 8 * half)
    c.w(CTRL, 1)
    for k in range(2, 6):
        c.poll(RPTR, p0 + 4 * half * (k - 1))
        c.ring(RB, 4 * half * (k % 2), words[k * half:(k + 1) * half])
        c.w(WPTR, p0 + 4 * half * (k + 1))
    c.r(UNDERRUNS)
    c.wait(clocks(half * 3, rate))
    c.r(IRQ_STATUS)
    c.r(UNDERRUNS)
    c.w(CTRL, 0)
    c.status(ST_BUSY, 0)
    c.r(RPTR)
    res = run(exe, tmp_path, c)
    v = [d for _, d in res["R"]]
    assert v[0] == p0
    assert v[1] == 0                         # no underrun while refilled
    assert v[2] & 1                          # low water
    assert v[4] == (p0 + 4 * half * 6) & 0xFFFFFFFF
    played, unf, _ = check(res, words, rate, 0x4000, 0, rlog)
    assert played - unf == len(words)


def test_mux(exe, tmp_path):
    """Source switching only while both sources are silent."""
    rate, rlog = 8, 3
    words = signal(300, seed=9)
    c = Cmd()
    c.dvb(1)
    c.w(RATE, rate)
    c.w(RAMP, rlog)
    c.w(RING_BASE, RB)
    c.ring(RB, 0, words)
    c.w(WPTR, 4 * len(words))
    c.wait(500)
    c.w(TX_SRC, 1)                           # DVB-S2 busy: pending
    c.wait(500)
    c.r(STATUS)
    c.w(CTRL, 1)                             # waits for the source
    c.wait(500)
    c.r(STATUS)
    c.dvb(0)
    c.status(ST_ACT | ST_BUSY, 1 | ST_BUSY)  # switched, started
    c.wait(clocks(150, rate))
    c.w(TX_SRC, 0)                           # IQ busy: pending
    c.dvb(1)
    c.wait(500)
    c.r(STATUS)
    c.w(CTRL, 0)
    c.status(ST_BUSY, 0)
    c.wait(500)
    c.r(STATUS)                              # IQ idle, DVB-S2 busy: still pending
    c.dvb(0)
    c.status(ST_ACT, 0)
    c.dvb(1)
    c.wait(500)
    c.w(TX_SRC, 2)
    c.dvb(0)
    c.status(ST_ACT, 2)
    c.dvb(1)
    c.wait(3000)
    res = run(exe, tmp_path, c, trace=1)
    v = [d for _, d in res["R"]]
    assert v[0] & 7 == 0b100                 # DVB active, pending
    assert v[1] & 0x1F == 0b10100            # waiting for the source
    assert v[2] & 0xF == 0b1101              # IQ active, pending, busy
    assert v[3] & 0xF == 0b0101              # IQ active, pending, idle
    check(res, words, rate, 0x4000, 0, rlog)
    # the DAC trace: DVB-S2 pattern (I odd, Q 0x100), IQ samples, zeros;
    # between the two sources at least 16 zeros
    t = np.array(res["T"], np.int64)
    iq = set(res["O"])
    kind = np.where(t == 0, 0, np.where((t >> 16) == 0x100, 1, 2))
    assert all(int(w) in iq for w in t[kind == 2])
    last, zeros = 0, 0
    for k in kind:
        if k == 0:
            zeros += 1
            continue
        if last and k != last:
            assert zeros >= 16
        last, zeros = k, 0
    seq = [k for k in kind if k]
    changes = sum(1 for a, b in zip(seq, seq[1:]) if a != b)
    assert changes == 2                      # DVB -> IQ -> DVB, then off
    assert kind[-300:].max() == 0             # off


# ---------------------------------------------------------------- model checks
def spectrum(zi, zq):
    x = (zi + 1j * zq).astype(complex)
    w = np.blackman(len(x))
    s = np.fft.fftshift(np.abs(np.fft.fft(x * w)) ** 2)
    f = np.fft.fftshift(np.fft.fftfreq(len(x), 1 / FS))
    return f, 10 * np.log10(s / s.max() + 1e-30)


def tone(n, f, fs, amp=0.9):
    t = np.arange(n)
    x = amp * 32767 * np.exp(2j * np.pi * f / fs * t)
    return np.round(x.real).astype(np.int64), np.round(x.imag).astype(np.int64)


@pytest.mark.parametrize("rate", [8, 25, 50, 200, 400])
def test_passband(rate):
    """Gain of a tone across +-0.4 x input rate: flat to 0.05 dB, 0 dB at GAIN 1."""
    fs = FS / (5 * rate)
    n = 120
    gains = []
    for f in np.linspace(-0.4, 0.4, 9) * fs:
        xi, xq = tone(n, f, fs, 0.5)
        ei, eq = m.envelope(xi, xq, np.full(n, m.RAMP_ONE), 0x4000, rate)
        zi, zq = m.chain_env(ei, eq, rate, 0)
        k = len(zi) // 2
        gains.append(np.sqrt(np.mean(zi[k:] ** 2 + zq[k:] ** 2)) / (0.5 * 32767))
    g = 20 * np.log10(gains)
    assert np.ptp(g) < 0.05, g
    assert abs(g.mean()) < 0.05, g


@pytest.mark.parametrize("rate,frac", [(8, 0.39), (8, -0.2), (50, 0.39), (50, -0.05), (400, 0.39)])
def test_images(rate, frac):
    """A tone in the passband: every other spectral component below -60 dBc
    (images, CIC, NCO, quantisation)."""
    fs = FS / (5 * rate)
    n = 2 ** 16 // (5 * rate) + 40
    f0 = frac * fs
    finc = 0x0A3D70A4
    xi, xq = tone(n, f0, fs)
    ei, eq = m.envelope(xi, xq, np.full(n, m.RAMP_ONE), 0x4000, rate)
    zi, zq = m.chain_env(ei, eq, rate, finc)
    zi, zq = zi[-2 ** 16:], zq[-2 ** 16:]
    f, s = spectrum(zi, zq)
    fc = f0 + finc / 2 ** 32 * FS
    far = np.abs(f - fc) > 6 * FS / 2 ** 16
    assert s[far].max() < -60, (s[far].max(), f[far][np.argmax(s[far])])


def test_nco_accuracy():
    """1 Hz steps or better; frequency from the phase slope; spurs."""
    assert FS / 2 ** 32 < 0.01
    finc = int(round(1234567.0 / FS * 2 ** 32))
    n = 2 ** 16
    zi, zq = m.nco(np.full(n, 20000), np.zeros(n, np.int64), finc)
    ph = np.unwrap(np.angle(zi + 1j * zq))
    fhat = np.polyfit(np.arange(n), ph, 1)[0] / (2 * np.pi) * FS
    assert abs(fhat - 1234567.0) < 0.01
    f, s = spectrum(zi, zq)
    far = np.abs(f - finc / 2 ** 32 * FS) > 6 * FS / n
    assert s[far].max() < -65


def test_ramp_tables():
    t = np.array(m.ramp_table())
    assert (np.diff(t) > 0).all() and t[0] >= 0 and t[-1] <= m.RAMP_ONE
    assert np.allclose(t + t[::-1], m.RAMP_ONE, atol=1)
    for rlog in (0, 3, 8, 12):
        up = m.ramp_seq(3 << rlog, 2 << rlog, rlog)
        mm = 1 << rlog
        assert len(up) == 3 * mm
        assert (up[mm:2 * mm] == m.RAMP_ONE).all()
        assert (up[2 * mm:] == up[:mm][::-1]).all()


def test_keying_splatter():
    """A keyed carrier (CW) at 12 kS/s with a 5.3 ms ramp: power more than
    500 Hz from the carrier below -60 dBc; without a ramp it is not."""
    rate = 200
    fs = FS / (5 * rate)

    def keyed(rlog):
        n_on = 1200
        xi = np.full(n_on + 200, 16000, np.int64)
        xq = np.zeros_like(xi)
        r = m.ramp_seq(len(xi), n_on, rlog)
        ei, eq = m.envelope(xi[:len(r)], xq[:len(r)], r, 0x4000, rate)
        ei = np.concatenate([np.zeros(300, np.int64), ei, np.zeros(400, np.int64)])
        eq = np.zeros_like(ei)
        # the envelope at the input rate: the spectrum of the baseband
        x = ei + 1j * eq
        s = np.abs(np.fft.fftshift(np.fft.fft(x, 1 << 16))) ** 2
        f = np.fft.fftshift(np.fft.fftfreq(1 << 16, 1 / fs))
        s = 10 * np.log10(s / s.max() + 1e-30)
        return s[np.abs(f) > 500].max()

    assert keyed(6) < -60
    assert keyed(0) > -50


def test_norm():
    for r in range(m.R_MIN, m.R_MAX + 1):
        nv, s = m.norm_table()[r]
        assert 1 << 16 <= nv < 1 << 17
        assert abs(nv * r ** 3 / 2 ** (16 + s) - 1) < 2 ** -15
