"""Spectrum analyser: RTL (xsim, tb_fft.sv, with the real xfft IP model)
against model/fftspec.py.

Every result dump is checked
  - bit-exact against the model (window, Xilinx bit-accurate FFT C model,
    |X|^2, sum, shift, saturation, FFT-shifted order), and
  - against numpy (float FFT of the same windowed samples / 2^shift) within
    the rounding tolerance fftspec.power_tolerance derives;
each captured frame must be 4096 consecutive samples of the selected source
(through the 250 MHz -> 100 MHz FIFO at 12 MSPS, or the nbiq tap), and tones
must appear in the expected word at the expected level.

Cases run in parallel in build/cases/<name> (xsim is slow: ~1 minute each).
"""
import concurrent.futures
import math
import os
import sys
import tomllib

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "model"))
sys.path.insert(0, HERE)
import fftspec as F  # noqa: E402
import xsim  # noqa: E402

F.ensure_cmodel_env()

N = F.N
FS = 12e6
SPEC = tomllib.load(open(os.path.join(HERE, "..", "..", "regs", "fft.toml"), "rb"))
REG = {r["name"]: r["offset"] for r in SPEC["reg"]}
WINDOW = {w["name"]: w["offset"] for w in SPEC["window"]}
CASES = os.path.join(xsim.BUILD, "cases")


def ctrl(run=0, irq=0, src=0, ch=0):
    return run | irq << 1 | src << 4 | ch << 8


# ---------------------------------------------------------------- stimulus
def tones(n, spec, noise, seed, amax):
    """spec: [(bin, amplitude)], bin = frequency in FFT bins (may be fractional)."""
    rng = np.random.default_rng(seed)
    t = np.arange(n)
    x = np.zeros(n, complex)
    for b, a in spec:
        x += a * np.exp(1j * (2 * np.pi * b / N * t + rng.uniform(0, 2 * np.pi)))
    x += rng.normal(0, noise, n) + 1j * rng.normal(0, noise, n)
    i = np.clip(np.round(x.real), -amax - 1, amax).astype(np.int64)
    q = np.clip(np.round(x.imag), -amax - 1, amax).astype(np.int64)
    return i, q


RX0_TONES = [(300, 1000), (-1000.5, 300), (1500, 2)]
RX1_TONES = [(-700, 800), (64, 50)]
NB_TONES = {0: [(100, 9000)], 1: [(-200, 12000)], 2: [(400, 6000)], 3: [(-1234, 15000), (50, 300)],
            4: [(1800, 10000), (-3, 4000)]}


class Case:
    def __init__(self, name, double_buf=0, adc_n=0, adc_tones=(RX0_TONES, RX1_TONES),
                 adc_noise=3.0, tap_n=0, tap_density=0.6, nb_tones=NB_TONES):
        self.name, self.db = name, double_buf
        self.dir = os.path.join(CASES, name)
        os.makedirs(self.dir, exist_ok=True)
        self.cmd = []
        self.src = {}
        self.adc_n = adc_n
        if adc_n:
            i0, q0 = tones(adc_n, adc_tones[0], adc_noise, 1, 2047)
            i1, q1 = tones(adc_n, adc_tones[1], adc_noise, 2, 2047)
            self.src[(0, 0)] = (i0, q0)
            self.src[(1, 0)] = (i1, q1)
            m = 0xFFFF
            with open(os.path.join(self.dir, "adc.hex"), "w") as f:
                for a, b, c, d in zip(i0 & m, q0 & m, i1 & m, q1 & m):
                    f.write(f"{d:04x}{c:04x}{b:04x}{a:04x}\n")
        self.tap_n = max(tap_n, 1)
        if tap_n:
            # interleaved channels, random order with runs of the same
            # channel (one sample per clock), idle clocks in between
            rng = np.random.default_rng(3)
            valid = rng.random(tap_n) < tap_density
            ch = np.zeros(tap_n, np.int64)
            c = 0
            for k in range(tap_n):
                if rng.random() < 0.4:
                    c = int(rng.integers(0, 5))
                ch[k] = c
            sig = {c: tones(tap_n, nb_tones[c], 20.0, 10 + c, 32767) for c in range(5)}
            cnt = {c: 0 for c in range(5)}
            words = []
            seq = {c: ([], []) for c in range(5)}
            for k in range(tap_n):
                if valid[k]:
                    c = int(ch[k])
                    i, q = sig[c][0][cnt[c]], sig[c][1][cnt[c]]
                    cnt[c] += 1
                    seq[c][0].append(i)
                    seq[c][1].append(q)
                    words.append((1 << 35) | (c << 32) | ((i & 0xFFFF) << 16) | (q & 0xFFFF))
                else:
                    words.append(0)
            for c in range(5):
                self.src[(2, c)] = (np.array(seq[c][0], np.int64), np.array(seq[c][1], np.int64))
            with open(os.path.join(self.dir, "tap.hex"), "w") as f:
                f.write("".join(f"{w:09x}\n" for w in words))
        self.win = F.window_half()

    # commands
    def w(self, reg, d):
        a = REG[reg] if isinstance(reg, str) else reg
        self.cmd.append(f"W {a:x} {d & 0xFFFFFFFF:x}")

    def r(self, reg):
        self.cmd.append(f"R {REG[reg]:x}")

    def poll(self, reg, mask, val):
        self.cmd.append(f"P {REG[reg]:x} {mask:x} {val:x}")

    def irq(self):
        self.cmd.append("I")

    def wait(self, n):
        self.cmd.append(f"C {n}")

    def dump(self):
        self.cmd.append("D")

    def regs(self, *names):
        for n in names:
            self.r(n)

    def snapshot(self, navg=1, rshift=0, scale=F.SCALE_RX, src=0, ch=0):
        self.w("NAVG", navg)
        self.w("RSHIFT", rshift)
        self.w("SCALE", scale)
        self.w("CTRL", ctrl(src=src, ch=ch))
        self.w("CMD", 1)
        self.poll("STATUS", 3, 2)          # idle and done
        self.regs("RES_CFG", "RES_SCALE", "FLAGS", "AVGS", "FRAMES")
        self.dump()

    def run(self):
        self.cmd.append("Q")
        with open(os.path.join(self.dir, "cmd.txt"), "w") as f:
            f.write("\n".join(self.cmd) + "\n")
        xsim.run(self.dir, self.db, self.adc_n, self.tap_n)
        return parse(os.path.join(self.dir, "ev.txt"))


def s16(v):
    v = int(v, 16)
    return v - 0x10000 if v & 0x8000 else v


def parse(path):
    """Event log -> averages (with their frames), dumps, register reads."""
    avgs, dumps, reads, irqs, aborts = [], [], [], [], []
    cur = None
    frame = None
    lines = open(path).read().split("\n")
    k = 0
    while k < len(lines):
        p = lines[k].split()
        k += 1
        if not p:
            continue
        t = p[0]
        if t == "G":
            cur = dict(t=int(p[1]), src=int(p[2]), ch=int(p[3]), navg=int(p[4]) + 1,
                       scale=int(p[5], 16), rsh=int(p[6]), frames=[], ftimes=[], pub=None, aborted=False)
            avgs.append(cur)
        elif t == "F":
            frame = ([], [])
            cur["frames"].append(frame)
            cur["ftimes"].append(int(p[1]))
        elif t == "S":
            frame[0].append(s16(p[1]))
            frame[1].append(s16(p[2]))
        elif t == "E":
            cur["pub"] = bool(int(p[2]))
            cur["tend"] = int(p[1])
        elif t == "X":
            aborts.append(int(p[1]))
            if cur is not None and cur["pub"] is None:
                cur["aborted"] = True
        elif t == "R":
            reads.append((int(p[1], 16), int(p[2], 16)))
        elif t == "I":
            irqs.append(int(p[1]))
        elif t == "D":
            pub = [a for a in avgs if a["pub"]]
            dumps.append(dict(t=int(p[1]), words=[], avg=pub[-1] if pub else None, nreads=len(reads)))
        elif t == "w":
            dumps[-1]["words"].append(int(p[1], 16))
    for d in dumps:
        assert len(d["words"]) == N
        d["words"] = np.array(d["words"], np.int64)
    for a in avgs:
        a["frames"] = [(np.array(i, np.int64), np.array(q, np.int64)) for i, q in a["frames"]]
    return dict(avgs=avgs, dumps=dumps, reads=reads, irqs=irqs, aborts=aborts)


# ---------------------------------------------------------------- checks
def locate(case, avg):
    """Each frame is 4096 consecutive samples of the selected source; returns
    the start indices."""
    key = (avg["src"], avg["ch"] if avg["src"] == 2 else 0)
    si, sq = case.src[key]
    starts = []
    for i, q in avg["frames"]:
        assert len(i) == N
        c = np.flatnonzero((si[:len(si) - N + 1] == i[0]) & (sq[:len(sq) - N + 1] == q[0]))
        c = [m for m in c if np.array_equal(si[m:m + N], i) and np.array_equal(sq[m:m + N], q)]
        assert len(c) == 1, f"{case.name}: frame not a unique consecutive slice of source {key}"
        starts.append(int(c[0]))
    assert starts == sorted(starts), "frames out of order"
    for a, b in zip(starts, starts[1:]):
        assert b >= a + N, "frames overlap"
    return starts


def check_dump(case, d, win=None, tones_=None, rtol_db=0.05):
    """Bit-exact and numpy comparison of one result dump; returns stats."""
    avg = d["avg"]
    assert avg is not None and avg["pub"]
    assert len(avg["frames"]) == avg["navg"]
    win = case.win if win is None else win
    starts = locate(case, avg)
    res, sat, ovf, acc = F.spectrum(avg["frames"], avg["scale"], avg["rsh"], win)
    exact = np.array_equal(res, d["words"])
    # numpy reference
    xref = []
    for i, q in avg["frames"]:
        yi, yq = F.apply_window(i, q, win)
        xref.append(F.fft_ref(yi, yq, avg["scale"]))
    pref = sum(np.abs(X) ** 2 for X in xref)
    tol = F.power_tolerance(xref, avg["scale"]) / 2.0 ** avg["rsh"] + 1
    exp = np.minimum(np.floor(pref / 2.0 ** avg["rsh"]), 0xFFFFFFFF)[F.shift_index()]
    tol = tol[F.shift_index()]
    unsat = d["words"] < 0xFFFFFFFF
    err = np.abs(d["words"] - exp)
    ratio = float(np.max(err[unsat] / tol[unsat])) if unsat.any() else 0.0
    # tones: word j = bin + 2048, level (A * sum(w) / 2^S)^2 * navg / 2^rsh
    tone_res = []
    for b, a in (tones_ or []):
        j = int(round(b)) % N ^ F.HALF
        lo, hi = max(j - 3, 0), min(j + 4, N)
        jm = lo + int(np.argmax(d["words"][lo:hi]))
        if float(b).is_integer() and a >= 100:
            lvl = (a * F.window_full(win).sum() / 2 ** F.WBITS / 2 ** F.sched_shift(avg["scale"])) ** 2
            lvl *= avg["navg"] / 2 ** avg["rsh"]
            db = 10 * math.log10(d["words"][j] / lvl)
            tone_res.append((b, j, jm, db))
        else:
            tone_res.append((b, j, jm, None))
    return dict(exact=exact, ratio=ratio, sat=sat, ovf=ovf, starts=starts, tones=tone_res,
                maxerr=float(err[unsat].max()) if unsat.any() else 0.0)


def assert_tones(st, rtol_db=0.05):
    for b, j, jm, db in st["tones"]:
        assert abs(jm - j) <= (0 if float(b).is_integer() else 1), f"tone at bin {b}: peak at word {jm}, expected {j}"
        if db is not None:
            assert abs(db) < rtol_db, f"tone at bin {b}: level {db:+.3f} dB off"


def regs_after(res, d):
    """Register reads logged just before dump d (RES_CFG, RES_SCALE, FLAGS, AVGS, FRAMES)."""
    return dict(res["reads"][d["nreads"] - 5:d["nreads"]])


# ---------------------------------------------------------------- cases
def case_rx0():
    c = Case("rx0", adc_n=60000)
    c.r("VERSION"); c.r("ID"); c.r("INFO"); c.r("SCALE"); c.r("NAVG")
    c.snapshot(navg=1, rshift=0)
    c.snapshot(navg=4, rshift=2)
    return c


def case_rx1():
    c = Case("rx1", adc_n=45000)
    c.snapshot(navg=3, rshift=0, scale=0x555500, src=1)
    return c


def case_nbiq():
    c = Case("nbiq", tap_n=400000, tap_density=0.6)
    c.snapshot(navg=2, rshift=1, scale=F.SCALE_16, src=2, ch=3)
    c.snapshot(navg=1, rshift=0, scale=F.SCALE_16, src=2, ch=4)
    c.snapshot(navg=1, rshift=0, scale=0x555556, src=2, ch=0)
    return c


def case_run(double_buf):
    """RUN: averages back to back; acknowledge each by clearing DONE."""
    c = Case("run_db" if double_buf else "run_sb", double_buf=double_buf, adc_n=110000)
    c.w("NAVG", 2)
    c.w("RSHIFT", 1)
    c.w("CTRL", ctrl(run=1, irq=1))
    for k in range(3):
        c.irq()
        c.regs("RES_CFG", "RES_SCALE", "FLAGS", "AVGS", "FRAMES")
        c.dump()
        c.w("DONE", 1)
    c.w("CTRL", 0)
    c.poll("STATUS", 1, 0)
    c.regs("AVGS", "DROPS", "FRAMES", "FLAGS")
    return c


def case_drop():
    """DOUBLE_BUF: the result stays while later averages run; one that ends
    before the acknowledge is dropped."""
    c = Case("drop", double_buf=1, adc_n=90000)
    c.w("NAVG", 1)
    c.w("CTRL", ctrl(run=1, irq=1))
    c.irq()
    c.regs("RES_CFG", "RES_SCALE", "FLAGS", "AVGS", "FRAMES")
    c.wait(250000)                       # > 2 frames: later averages are dropped
    c.dump()
    c.regs("DROPS", "FLAGS", "AVGS")
    c.w("FLAGS", 0xF)
    c.w("DONE", 1)
    c.irq()
    c.regs("RES_CFG", "RES_SCALE", "FLAGS", "AVGS", "FRAMES")
    c.dump()
    c.w("CTRL", 0)
    c.poll("STATUS", 1, 0)
    return c


def case_switch():
    """Source switching: CTRL changes apply at the next average; ABORT
    restarts at once with the new configuration."""
    c = Case("switch", adc_n=110000, tap_n=600000, tap_density=0.5)
    c.w("NAVG", 2)
    c.w("CTRL", ctrl(run=1, src=0))
    c.wait(30000)                        # inside the first average
    c.w("CTRL", ctrl(run=1, src=1))      # applies to the next one
    c.poll("DONE", 1, 1)
    c.regs("RES_CFG", "RES_SCALE", "FLAGS", "AVGS", "FRAMES")
    c.dump()
    c.w("DONE", 1)                       # the rx1 average starts
    c.wait(60000)
    c.w("SCALE", F.SCALE_16)
    c.w("CTRL", ctrl(run=1, src=2, ch=1))
    c.w("CMD", 2)                        # abort the rx1 average
    c.poll("DONE", 1, 1)
    c.regs("RES_CFG", "RES_SCALE", "FLAGS", "AVGS", "FRAMES")
    c.dump()
    c.w("CTRL", 0)
    c.w("DONE", 1)
    c.poll("STATUS", 1, 0)
    return c


def case_ovf():
    """FFT overflow flag (no scaling) and result saturation."""
    c = Case("ovf", adc_n=200000, adc_tones=([(300, 2040)], RX1_TONES), adc_noise=1.0)
    c.snapshot(navg=1, rshift=0, scale=0x000000)
    c.w("FLAGS", 0xF)
    c.r("FLAGS")
    c.snapshot(navg=16, rshift=0, scale=F.SCALE_RX)
    return c


def case_win():
    """Window table written over AXI (rectangular)."""
    c = Case("win", adc_n=30000)
    for n in range(F.HALF):
        c.w(WINDOW["WIN"] + 4 * n, 1 << F.WBITS)
    c.snapshot(navg=1, rshift=0, scale=0x555500)
    c.win = F.window_half("rect")
    return c


CASE_FUNS = {"rx0": case_rx0, "rx1": case_rx1, "nbiq": case_nbiq, "run_sb": lambda: case_run(0),
             "run_db": lambda: case_run(1), "drop": case_drop, "switch": case_switch,
             "ovf": case_ovf, "win": case_win}


@pytest.fixture(scope="session")
def results():
    xsim.build()
    cases = {n: f() for n, f in CASE_FUNS.items()}
    with concurrent.futures.ThreadPoolExecutor(len(cases)) as ex:
        fut = {n: ex.submit(c.run) for n, c in cases.items()}
        out = {}
        for n, f in fut.items():
            try:
                out[n] = (cases[n], f.result())
            except Exception as e:      # noqa: BLE001 - reported per test
                out[n] = (cases[n], e)
    return out


def get(results, name):
    c, r = results[name]
    if isinstance(r, Exception):
        raise r
    return c, r


def report(name, st):
    t = ", ".join(f"bin {b}: word {jm}" + (f" {db:+.3f} dB" if db is not None else "")
                  for b, _, jm, db in st["tones"])
    print(f"{name}: exact={st['exact']} max|err|={st['maxerr']:.0f} err/tol={st['ratio']:.3f} {t}")


def common(c, res, tones_by_src):
    stats = []
    for d in res["dumps"]:
        a = d["avg"]
        tn = tones_by_src(a)
        st = check_dump(c, d, tones_=tn)
        report(f"{c.name} src={a['src']} ch={a['ch']} navg={a['navg']}", st)
        assert st["exact"], "RTL result differs from the bit-exact model"
        assert st["ratio"] <= 1.0, "RTL result outside the numpy tolerance"
        assert_tones(st)
        rg = regs_after(res, d)
        if REG["RES_CFG"] in rg:
            v = rg[REG["RES_CFG"]]
            assert v & 3 == a["src"] and (v >> 8) & 15 == a["rsh"] and (v >> 16) & 0x1FF == a["navg"]
            if a["src"] == 2:
                assert (v >> 4) & 7 == a["ch"]
            assert rg[REG["RES_SCALE"]] == a["scale"]
        stats.append(st)
    return stats


def rx_tones(a):
    if a["src"] == 0:
        return RX0_TONES
    if a["src"] == 1:
        return RX1_TONES
    return NB_TONES[a["ch"]]


# ---------------------------------------------------------------- tests
def test_model_window():
    w = F.window_full(F.window_half())
    # Blackman-Harris: highest sidelobe below -90 dB
    W = np.abs(np.fft.fft(w / w.sum(), 16 * N))
    side = W[int(16 * 4.5):16 * N - int(16 * 4.5)]
    assert 20 * np.log10(side.max()) < -90


def test_model_vs_numpy():
    """Bit-exact model vs numpy within the derived tolerance (no RTL)."""
    rng = np.random.default_rng(7)
    for scale, amp in ((F.SCALE_RX, 2047), (F.SCALE_16, 32767), (0x555500, 2047)):
        i, q = tones(N, [(123, amp / 2), (-77.3, amp / 8)], amp / 64, int(rng.integers(1000)), amp)
        yi, yq = F.apply_window(i, q, F.window_half())
        xr, xi, ovf = F.xfft_cmodel(yi, yq, scale)
        assert not ovf
        X = F.fft_ref(yi, yq, scale)
        e = np.abs((xr + 1j * xi) - X)
        bound = 7 * F.fft_err_sigma(scale) * math.sqrt(2) + F.TWIDDLE_REL * np.abs(X)
        assert np.all(e <= bound)
        print(f"scale {scale:06x}: error rms {np.sqrt(np.mean(e ** 2) / 2):.3f} LSB per component, "
              f"sigma bound {F.fft_err_sigma(scale):.3f}")


def test_rx0(results):
    c, r = get(results, "rx0")
    rd = dict(r["reads"][:5])
    assert rd[REG["VERSION"]] == 0x00010000 and rd[REG["ID"]] == 0x46465453
    assert rd[REG["INFO"]] == 12 and rd[REG["SCALE"]] == F.SCALE_RX and rd[REG["NAVG"]] == 1
    st = common(c, r, rx_tones)
    assert len(r["dumps"]) == 2
    rg = regs_after(r, r["dumps"][1])
    assert rg[REG["AVGS"]] == 2 and rg[REG["FRAMES"]] == 5 and rg[REG["FLAGS"]] == 0
    # 12 MSPS: frame period inside an average
    ft = r["avgs"][1]["ftimes"]
    per = np.diff(ft).mean() * 1e-9
    print(f"rx 12 MSPS: frame period {per * 1e6:.1f} us = {1 / per:.0f} spectra/s")
    assert st


def test_rx1(results):
    c, r = get(results, "rx1")
    common(c, r, rx_tones)
    assert regs_after(r, r["dumps"][0])[REG["FLAGS"]] == 0


def test_nbiq(results):
    c, r = get(results, "nbiq")
    common(c, r, rx_tones)
    assert [d["avg"]["ch"] for d in r["dumps"]] == [3, 4, 0]
    for d in r["dumps"]:
        assert regs_after(r, d)[REG["FLAGS"]] == 0


@pytest.mark.parametrize("db", [0, 1])
def test_run(results, db):
    c, r = get(results, "run_db" if db else "run_sb")
    common(c, r, rx_tones)
    assert len(r["dumps"]) == 3
    avgs = [d["avg"] for d in r["dumps"]]
    assert len({id(a) for a in avgs}) == 3, "a result was dumped twice"
    tail = dict(r["reads"][-4:])
    pub = [a for a in r["avgs"] if a["pub"]]
    assert tail[REG["AVGS"]] == len(pub) and tail[REG["FLAGS"]] == 0
    assert tail[REG["FRAMES"]] == 2 * len([a for a in r["avgs"] if a["pub"] is not None])
    if db:
        # back to back: the next average starts when the previous one ends
        for a, b in zip(r["avgs"], r["avgs"][1:]):
            gap = (b["ftimes"][0] - a["tend"]) * 1e-9
            assert gap < 50e-6, f"gap between averages {gap * 1e6:.0f} us"
    else:
        # each average starts after the acknowledge
        assert len(r["avgs"]) >= 3


def test_drop(results):
    c, r = get(results, "drop")
    common(c, r, rx_tones)
    d0, d1 = r["dumps"]
    # the first dump is the first average although others ran meanwhile
    assert d0["avg"] is r["avgs"][0]
    rd = dict(r["reads"][d0["nreads"]:d0["nreads"] + 3])
    assert rd[REG["DROPS"]] >= 1 and rd[REG["FLAGS"]] & 8 and rd[REG["AVGS"]] == 1
    assert d1["avg"]["t"] > d0["t"]


def test_switch(results):
    c, r = get(results, "switch")
    common(c, r, rx_tones)
    d0, d1 = r["dumps"]
    assert d0["avg"]["src"] == 0                      # CTRL written mid-average
    assert d1["avg"]["src"] == 2 and d1["avg"]["ch"] == 1
    assert any(a["aborted"] and a["src"] == 1 for a in r["avgs"]), "the rx1 average was not aborted"
    # after ABORT the new average starts right away
    ab = r["aborts"][0]
    assert (d1["avg"]["t"] - ab) * 1e-9 < 5e-6


def test_ovf(results):
    c, r = get(results, "ovf")
    d0, d1 = r["dumps"]
    st0 = check_dump(c, d0)
    print(f"ovf: model overflow={st0['ovf']} exact={st0['exact']}")
    assert st0["ovf"] and st0["exact"]
    assert regs_after(r, d0)[REG["FLAGS"]] & 1
    assert r["reads"][d0["nreads"]] == (REG["FLAGS"], 0)    # cleared
    st1 = check_dump(c, d1, tones_=None)
    print(f"sat: model saturated={st1['sat']} exact={st1['exact']} words at max: {(d1['words'] == 0xFFFFFFFF).sum()}")
    assert st1["sat"] and st1["exact"] and st1["ratio"] <= 1.0
    assert regs_after(r, d1)[REG["FLAGS"]] == 2


def test_win(results):
    c, r = get(results, "win")
    st = check_dump(c, r["dumps"][0], win=F.window_half("rect"), tones_=[(300, 1000)])
    report("win rect", st)
    assert st["exact"] and st["ratio"] <= 1.0
    assert_tones(st)
