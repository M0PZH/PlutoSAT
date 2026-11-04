"""Narrowband IQ channels: RTL (Verilator, tb_nbiq_axi.sv) against the
bit-exact model, plus filter quality checks on the model."""
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
RTL = os.path.join(HERE, "..", "rtl")
BUILD = os.path.join(HERE, "build")
sys.path.insert(0, os.path.join(HERE, "..", "model"))
import nbiq as nb  # noqa: E402

FS = 12e6
ADC_PS = 20833          # adc_clk 48 MHz, one sample per 4 clocks: 12 MSPS
ADC_DIV = 4
CH_BASE = 0x100
COEF_BASE = 0x1000


@pytest.fixture(scope="module")
def exe():
    obj = os.path.join(BUILD, "tb_nbiq_axi")
    os.makedirs(obj, exist_ok=True)
    nb.write_lut()
    src = [os.path.join(RTL, f) for f in ("nbiq_chan.sv", "nbiq_comb.sv", "nbiq_fir.sv", "nbiq_sdpram.sv", "nbiq_wr.sv", "nbiq_axi.sv")]
    src += [os.path.join(HERE, "..", "..", "dvbs2", "rtl", "dvbs2_async_fifo.sv"),
            os.path.join(HERE, "tb_nbiq_axi.sv")]
    subprocess.run(["verilator", "--binary", "--timing", "-j", "16", "-O3", "-Wno-fatal",
                    "-Wno-WIDTHTRUNC", "-Wno-WIDTHEXPAND", "--top-module", "tb_nbiq_axi",
                    "-Mdir", obj] + src, check=True, stdout=subprocess.DEVNULL)
    return os.path.join(obj, "Vtb_nbiq_axi")


def adc_signal(n, seed):
    """Two receive channels: a few tones + noise, 12-bit."""
    rng = np.random.default_rng(seed)
    t = np.arange(n)
    out = []
    for _ in range(2):
        x = np.zeros(n, complex)
        for _ in range(6):
            f = rng.uniform(-0.45, 0.45)
            x += rng.uniform(50, 300) * np.exp(1j * (2 * np.pi * f * t + rng.uniform(0, 6.3)))
        x += rng.normal(0, 20, n) + 1j * rng.normal(0, 20, n)
        out.append(np.clip(np.round(x.real), -2048, 2047).astype(np.int64))
        out.append(np.clip(np.round(x.imag), -2048, 2047).astype(np.int64))
    return out                       # i0, q0, i1, q1


class Cmd:
    def __init__(self):
        self.lines = []

    def w(self, a, d):
        self.lines.append(f"W {a:x} {d & 0xFFFFFFFF:x}")

    def r(self, a):
        self.lines.append(f"R {a:x}")

    def adc(self, n):
        self.lines.append(f"A {n}")

    def wait(self, n):
        self.lines.append(f"D {n}")

    def end(self):
        self.lines.append("E")

    def config(self, c, cfg, src, base, size):
        a = CH_BASE + 0x40 * c
        self.w(a + 0x00, 0)
        self.w(a + 0x04, cfg["finc"])
        self.w(a + 0x08, cfg["R"] | cfg["cshift"] << 8)
        self.w(a + 0x0C, cfg["npairs"] | cfg["oshift"] << 8)
        self.w(a + 0x10, base)
        self.w(a + 0x14, size - 1)
        for i, h in enumerate(cfg["h"]):
            self.w(COEF_BASE + 0x400 * c + 4 * i, h & 0x3FFFF)

    def enable(self, c, src):
        self.w(CH_BASE + 0x40 * c, 1 | src << 1)

    def readback(self, c):
        a = CH_BASE + 0x40 * c
        for o in (0x18, 0x1C, 0x20, 0x24):
            self.r(a + o)


def run(exe, tmp_path, cmd, adc, stall=0):
    fa = tmp_path / "adc.txt"
    with open(fa, "w") as f:
        for row in zip(*[np.asarray(v) & 0xFFFF for v in adc]):
            f.write("%04x %04x %04x %04x\n" % row)
    fc = tmp_path / "cmd.txt"
    fc.write_text("\n".join(cmd.lines) + "\n")
    fo = tmp_path / "out.txt"
    subprocess.run([exe, f"+adc={fa}", f"+cmd={fc}", f"+out={fo}", f"+adc_ps={ADC_PS}",
                    f"+adc_div={ADC_DIV}", f"+stall={stall}"], check=True, capture_output=True, cwd=HERE, timeout=600)
    regs, mem = [], {}
    for line in fo.read_text().split("\n"):
        p = line.split()
        if not p:
            continue
        if p[0] == "R":
            regs.append((int(p[1], 16), int(p[2], 16)))
        else:
            mem[int(p[1], 16)] = int(p[2], 16)
    return regs, mem


def ring(mem, base, size, wptr):
    """(first sample index, int16 I, int16 Q) of what the ring holds."""
    n = wptr // 4
    k0 = max(0, n - size // 4)
    w = np.array([mem[base + (4 * k) % size] for k in range(k0, n)], np.int64)
    i = (w & 0xFFFF).astype(np.int16).astype(np.int64)
    q = (w >> 16).astype(np.int16).astype(np.int64)
    return k0, i, q


CHANNELS = [  # (source, offset Hz, rate, ring bytes)
    (0, 1.5e6, 1e6, 4096),
    (1, -2.2e6, 500e3, 65536),
    (0, 0.3e6, 250e3, 65536),
    (1, -4.0e6, 100e3, 65536),
]


@pytest.mark.parametrize("stall", [0, 40])
def test_channels(exe, tmp_path, stall):
    nadc = 96000
    adc = adc_signal(nadc, seed=stall + 1)
    cmd = Cmd()
    cmd.w(0x0C, 20000)                       # 200 us interrupt period
    cmd.w(0x08, 1)
    cfgs, bases = [], []
    base = 0x1000_0000
    for c, (src, foff, rate, size) in enumerate(CHANNELS):
        cfg = nb.design(FS, rate)
        cfg["finc"] = nb.finc_for(foff, FS)
        cfgs.append(cfg)
        bases.append(base)
        cmd.config(c, cfg, src, base, size)
        base += 0x10_0000
    cmd.adc(nadc)
    for c, (src, *_rest) in enumerate(CHANNELS):    # staggered enables while samples flow
        cmd.wait(1000 + 777 * c)
        cmd.enable(c, src)
    cmd.end()
    cmd.r(0x10)
    for c in range(4):
        cmd.readback(c)
    regs, mem = run(exe, tmp_path, cmd, adc, stall)
    assert regs[0][1] & 1, "interrupt status not set"
    for c, (src, foff, rate, size) in enumerate(CHANNELS):
        wptr, t0lo, t0hi, drops = [v for _, v in regs[1 + 4 * c: 5 + 4 * c]]
        t0 = t0lo | t0hi << 32
        assert drops == 0, f"ch{c} drops {drops:#x}"
        assert 0 < t0 < nadc
        xi, xq = adc[2 * src][t0:], adc[2 * src + 1][t0:]
        mi, mq = nb.channel(xi, xq, cfgs[c])
        k0, ri, rq = ring(mem, bases[c], size, wptr)
        n = wptr // 4
        assert len(mi) - 32 <= n <= len(mi), f"ch{c}: {n} samples in memory, model {len(mi)}"
        assert np.array_equal(ri, mi[k0:n]) and np.array_equal(rq, mq[k0:n]), f"ch{c} mismatch"


def test_reenable(exe, tmp_path):
    """Disable and re-enable a channel with another configuration while samples flow."""
    nadc = 60000
    adc = adc_signal(nadc, seed=7)
    cmd = Cmd()
    a = nb.design(FS, 1e6)
    a["finc"] = nb.finc_for(2e6, FS)
    b = nb.design(FS, 200e3)
    b["finc"] = nb.finc_for(-1e6, FS)
    cmd.config(2, a, 0, 0x2000_0000, 1 << 20)
    cmd.adc(nadc)
    cmd.wait(500)
    cmd.enable(2, 0)
    cmd.wait(30000)
    cmd.config(2, b, 1, 0x2000_0000, 1 << 20)     # writes CTRL = 0 first
    cmd.enable(2, 1)
    cmd.end()
    cmd.readback(2)
    regs, mem = run(exe, tmp_path, cmd, adc)
    wptr, t0lo, t0hi, drops = [v for _, v in regs]
    assert drops == 0
    mi, mq = nb.channel(adc[2][t0lo:], adc[3][t0lo:], b)
    k0, ri, rq = ring(mem, 0x2000_0000, 1 << 20, wptr)
    assert k0 == 0 and len(ri) > 300
    assert np.array_equal(ri, mi[:len(ri)]) and np.array_equal(rq, mq[:len(rq)])


# ---------------------------------------------------------------- model only

@pytest.mark.parametrize("rate", [1e6, 500e3, 250e3, 100e3])
def test_tone(rate):
    """A tone at offset + df comes out at df with the designed gain (8) and
    the image / spurs well below it."""
    cfg = nb.design(FS, rate)
    foff, amp = 1.234e6, 1000.0
    df = round(0.21 * 4096) / 4096 * cfg["fout"]       # on an FFT bin
    n = int(4096 * FS / rate) + 20000
    t = np.arange(n)
    x = amp * np.exp(2j * np.pi * (foff + df) / FS * t)
    xi, xq = np.round(x.real).astype(np.int64), np.round(x.imag).astype(np.int64)
    cfg["finc"] = nb.finc_for(foff, FS)
    yi, yq = nb.channel(xi, xq, cfg)
    y = (yi + 1j * yq)[-4096:]
    w = np.blackman(4096)
    Y = np.abs(np.fft.fft(y * w)) / w.sum()
    k = int(round(df / cfg["fout"] * 4096))
    assert abs(Y[k] / (8 * amp) - 1) < 0.01, Y[k]
    Y[k - 4:k + 5] = 0
    assert 20 * np.log10(Y.max() / (8 * amp)) < -65


@pytest.mark.parametrize("rate", [1e6, 250e3, 100e3])
def test_alias(rate):
    """A strong tone outside the channel that folds onto 0.3 fout is
    rejected by at least 60 dB (80 % of the band is alias-protected)."""
    cfg = nb.design(FS, rate)
    fo = cfg["fout"]
    worst = -200
    for k in (1, -1, 2, -2, 3):
        f = 0.3 * fo + k * fo
        n = int(2048 * FS / rate) + 20000
        t = np.arange(n)
        x = 1500 * np.exp(2j * np.pi * f / FS * t)
        xi, xq = np.round(x.real).astype(np.int64), np.round(x.imag).astype(np.int64)
        cfg["finc"] = 0
        yi, yq = nb.channel(xi, xq, cfg)
        y = (yi + 1j * yq)[-2048:]
        w = np.blackman(2048)
        Y = np.abs(np.fft.fft(y * w)) / w.sum()
        kk = int(round(0.3 * 2048)) % 2048
        worst = max(worst, 20 * np.log10(Y[kk - 2:kk + 3].max() / (8 * 1500) + 1e-12))
    assert worst < -58, worst


@pytest.mark.parametrize("R", [2, 6, 37, 64])
def test_model_vectorized(R):
    """The vectorized model equals the scalar reference (register behaviour)."""
    rng = np.random.default_rng(R)
    cfg = nb.design(FS, FS / (2 * R), npairs=min(127, 3 * R))
    xi, xq = rng.integers(-32768, 32768, 4000), rng.integers(-32768, 32768, 4000)
    f = int(rng.integers(0, 2 ** 32))
    a, b = nb.mixer(xi, xq, f), nb.mixer_ref(xi, xq, f)
    assert np.array_equal(a[0], b[0]) and np.array_equal(a[1], b[1])
    c = nb.cic(a[1], R, cfg["cshift"])
    assert np.array_equal(c, nb.cic_ref(b[1], R, cfg["cshift"]))
    big = rng.integers(-(1 << 23), 1 << 23, 1200)
    assert np.array_equal(nb.fir(big, cfg["h"], cfg["oshift"]), nb.fir_ref(list(big), cfg["h"], cfg["oshift"]))


def test_c_design():
    """sw/nbiq.c designs the same filters as model/nbiq.py (taps within 1 LSB)."""
    sw = os.path.join(HERE, "..", "sw")
    os.makedirs(BUILD, exist_ok=True)
    tool = os.path.join(BUILD, "nbiq-host")         # not in sw/: that is the package source
    subprocess.run(["gcc", "-O2", "-Wall", "-Wextra", "-std=gnu11", "-o", tool,
                    os.path.join(sw, "nbiq.c"), "-lm"], check=True)
    for R in range(2, 65):
        out = subprocess.run([tool, "-D", str(R)], check=True,
                             capture_output=True, text=True).stdout.split("\n")
        hdr = out[0].split()
        d = nb.design(FS, FS / (2 * R))
        assert [int(hdr[i]) for i in (1, 3, 5, 7)] == [d["R"], d["cshift"], d["npairs"], d["oshift"]], R
        h = np.array([int(v) for v in out[1:] if v])
        assert np.abs(h - np.array(d["h"])).max() <= 1, R


@pytest.mark.parametrize("rates", [(1e6, 1e6, 1e6, 1e6), (1e6, 100e3, 100e3, 100e3),
                                   (3e6 / 2, 600e3, 120e3, 1e6)])
def test_load(exe, tmp_path, rates):
    """Heavy FIR load (the driver's tap budget) and long jobs next to short
    ones: no drops, bit exact."""
    nadc = 72000
    adc = adc_signal(nadc, seed=11)
    cmd = Cmd()
    cfgs = []
    for c, rate in enumerate(rates):
        cfg = nb.design(FS, rate)
        cfg["finc"] = nb.finc_for(0.7e6 * (c - 1.5), FS)
        cfgs.append(cfg)
        cmd.config(c, cfg, c & 1, 0x1000_0000 + c * 0x10_0000, 1 << 20)
    cmd.adc(nadc)
    cmd.wait(300)
    for c in range(4):
        cmd.enable(c, c & 1)
    cmd.end()
    for c in range(4):
        cmd.readback(c)
    regs, mem = run(exe, tmp_path, cmd, adc, stall=20)
    for c in range(4):
        wptr, t0lo, _, drops = [v for _, v in regs[4 * c: 4 * c + 4]]
        assert drops == 0, f"ch{c} drops {drops:#x}"
        mi, mq = nb.channel(adc[2 * (c & 1)][t0lo:], adc[2 * (c & 1) + 1][t0lo:], cfgs[c])
        k0, ri, rq = ring(mem, 0x1000_0000 + c * 0x10_0000, 1 << 20, wptr)
        assert len(mi) - 32 <= len(ri) <= len(mi)
        assert np.array_equal(ri, mi[:len(ri)]) and np.array_equal(rq, mq[:len(rq)]), f"ch{c}"


@pytest.mark.parametrize("phase", range(9))
def test_enable_phase(exe, tmp_path, phase):
    """Enabling at any clock relative to the ADC samples: T0 names the first
    sample the channel used, for either source."""
    nadc = 8000
    adc = adc_signal(nadc, seed=phase)
    cmd = Cmd()
    cfgs = []
    for c in range(2):
        cfg = nb.design(FS, 1e6)
        cfg["finc"] = nb.finc_for(1.1e6 * (c + 1), FS)
        cfgs.append(cfg)
        cmd.config(c, cfg, c, 0x1000_0000 + c * 0x10_0000, 1 << 20)
    cmd.adc(nadc)
    cmd.wait(200 + phase)
    cmd.enable(0, 0)
    cmd.enable(1, 1)
    cmd.end()
    for c in range(2):
        cmd.readback(c)
    regs, mem = run(exe, tmp_path, cmd, adc)
    for c in range(2):
        wptr, t0, _, drops = [v for _, v in regs[4 * c: 4 * c + 4]]
        mi, mq = nb.channel(adc[2 * c][t0:], adc[2 * c + 1][t0:], cfgs[c])
        _, ri, rq = ring(mem, 0x1000_0000 + c * 0x10_0000, 1 << 20, wptr)
        assert drops == 0 and len(ri) > 300
        assert np.array_equal(ri, mi[:len(ri)]) and np.array_equal(rq, mq[:len(rq)]), f"ch{c}"
