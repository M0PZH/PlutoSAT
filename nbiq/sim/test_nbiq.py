"""Narrowband IQ channels: RTL (Verilator, tb_nbiq_axi.sv) against the
bit-exact model, plus filter quality checks on the model."""
import itertools
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
COEF_BASE = 0x2000


@pytest.fixture(scope="module", params=[64, 32], ids=["dw64", "dw32"])
def exe(request):
    """The testbench with a 64-bit (S_AXI_HP, as built) or 32-bit AXI master."""
    dw = request.param
    obj = os.path.join(BUILD, f"tb_nbiq_axi_dw{dw}")
    os.makedirs(obj, exist_ok=True)
    nb.write_lut()
    src = [os.path.join(RTL, f) for f in ("nbiq_chan.sv", "nbiq_comb.sv", "nbiq_fir.sv", "nbiq_sdpram.sv", "nbiq_wr.sv", "nbiq_axi.sv")]
    src += [os.path.join(HERE, "..", "..", "dvbs2", "rtl", "dvbs2_async_fifo.sv"),
            os.path.join(HERE, "tb_nbiq_axi.sv")]
    subprocess.run(["verilator", "--binary", "--timing", "-j", "16", "-O3", "-Wno-fatal",
                    "-Wno-WIDTHTRUNC", "-Wno-WIDTHEXPAND", "--top-module", "tb_nbiq_axi", f"-GDW={dw}",
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


def run(exe, tmp_path, cmd, adc, stall=0, adc_ps=ADC_PS, adc_div=ADC_DIV):
    fa = tmp_path / "adc.txt"
    with open(fa, "w") as f:
        for row in zip(*[np.asarray(v) & 0xFFFF for v in adc]):
            f.write("%04x %04x %04x %04x\n" % row)
    fc = tmp_path / "cmd.txt"
    fc.write_text("\n".join(cmd.lines) + "\n")
    fo = tmp_path / "out.txt"
    subprocess.run([exe, f"+adc={fa}", f"+cmd={fc}", f"+out={fo}", f"+adc_ps={adc_ps}",
                    f"+adc_div={adc_div}", f"+stall={stall}"], check=True, capture_output=True, cwd=HERE, timeout=600)
    regs, mem = [], {}
    tap = {c: [] for c in range(nb.NCH)}
    for line in fo.read_text().split("\n"):
        p = line.split()
        if not p:
            continue
        if p[0] == "R":
            regs.append((int(p[1], 16), int(p[2], 16)))
        elif p[0] == "T":
            tap[int(p[1])].append(int(p[2], 16))
        else:
            mem[int(p[1], 16)] = int(p[2], 16)
    run.tap = {c: split_iq(np.array(v, np.int64)) for c, v in tap.items()}
    return regs, mem


def split_iq(w):
    i = (w & 0xFFFF).astype(np.int16).astype(np.int64)
    q = (w >> 16).astype(np.int16).astype(np.int64)
    return i, q


def ring(mem, base, size, wptr):
    """(first sample index, int16 I, int16 Q) of what the ring holds."""
    n = wptr // 4
    k0 = max(0, n - size // 4)
    w = np.array([mem[base + (4 * k) % size] for k in range(k0, n)], np.int64)
    i, q = split_iq(w)
    return k0, i, q


def check_tap(c, k0, ri, rq, mi, mq):
    """The tap carries every sample of the channel (the model's, in order),
    the ring the same ones."""
    ti, tq = run.tap[c]
    n = k0 + len(ri)
    assert n <= len(ti) <= n + 80, f"ch{c}: {len(ti)} tap samples, {n} in the ring"
    assert np.array_equal(ti, mi[:len(ti)]) and np.array_equal(tq, mq[:len(tq)]), f"ch{c} tap"
    assert np.array_equal(ti[k0:n], ri) and np.array_equal(tq[k0:n], rq), f"ch{c} tap / ring"


def designs(rates, fs=FS):
    """The driver's designs: allocate() (the same tap pairs for every
    channel in use, as many as the FIR engine takes)."""
    fouts = [fs / (2 * round(fs / (2 * r))) for r in rates]
    return [nb.design(fs, r, npairs=p) for r, p in zip(rates, nb.allocate(fouts))]


CHANNELS = [  # (source, offset Hz, rate, ring bytes): the station's plan
    (1, -205e3, 48e3, 65536),
    (1, 30e3, 48e3, 65536),
    (0, 4.1e6, 48e3, 65536),
    (1, 180e3, 48e3, 65536),
    (1, 0.0, 600e3, 16384),             # overview of the transponder, wraps
    (1, 95e3, 48e3, 65536),             # background: FT8, recorder, beacon
    (0, -3.3e6, 48e3, 65536),
    (1, -120e3, 48e3, 65536),
]


@pytest.mark.parametrize("stall", [0, 40])
def test_channels(exe, tmp_path, stall):
    """Seven 48 kHz channels (R 125) and a 600 kHz overview (R 10) on
    channel 4, staggered enables while samples flow, rings not aligned to
    their size: bit exact, tap = ring = model."""
    nadc = 120000
    adc = adc_signal(nadc, seed=stall + 1)
    cmd = Cmd()
    cmd.w(0x0C, 20000)                       # 200 us interrupt period
    cmd.w(0x08, 1)
    cfgs = designs([r for _, _, r, _ in CHANNELS])
    assert [d["R"] for d in cfgs] == [125] * 4 + [10] + [125] * 3
    assert [d["npairs"] for d in cfgs] == [95] * 8
    load, jq, ok = nb.fir_check([d["fout"] for d in cfgs], [d["npairs"] for d in cfgs])
    assert ok and abs(load - 0.899) < 0.005 and jq <= 16, (load, jq)
    bases = []
    base = 0x1000_0000
    for c, (src, foff, rate, size) in enumerate(CHANNELS):
        cfgs[c]["finc"] = nb.finc_for(foff, FS)
        bases.append(base + 0x3040 * c)      # 64-byte aligned only (2.1: base + offset)
        cmd.config(c, cfgs[c], src, bases[c], size)
        base += 0x10_0000
    cmd.adc(nadc)
    for c, (src, *_rest) in enumerate(CHANNELS):    # staggered enables while samples flow
        cmd.wait(1000 + 777 * c)
        cmd.enable(c, src)
    cmd.end()
    cmd.r(0x10)
    for c in range(nb.NCH):
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
        assert n > 250, n
        assert np.array_equal(ri, mi[k0:n]) and np.array_equal(rq, mq[k0:n]), f"ch{c} mismatch"
        check_tap(c, k0, ri, rq, mi, mq)
    assert 4 * len(run.tap[4][0]) > CHANNELS[4][3], "the overview ring wraps"


def test_registers(exe, tmp_path):
    """ID, VERSION, INFO; every channel's registers; addresses past the
    channels and the coefficient windows change nothing."""
    cmd = Cmd()
    for a in (0x000, 0x004, 0x01C):
        cmd.r(a)
    for c in range(nb.NCH):
        a = CH_BASE + 0x40 * c
        cmd.w(a + 0x04, 0x1111_0000 + c)
        cmd.w(a + 0x08, 128 - c | (29 - c) << 8)
        cmd.w(a + 0x0C, 127 - c | (40 + c) << 8)
        cmd.w(a + 0x10, 0x2000_0000 + c * 0x1000)
        cmd.w(a + 0x14, 0xFFF + c * 0x1000)
    for a in (0x300, 0x304, 0x308, 0x30C, 0x3C4, 0x0404, 0x1104, 0x4104, 0x4400, 0x8104):
        cmd.w(a, 0xDEAD_BEEF)                # no channel there
    for c in range(nb.NCH):
        a = CH_BASE + 0x40 * c
        for o in (0x00, 0x04, 0x08, 0x0C, 0x10, 0x14):
            cmd.r(a + o)
    cmd.r(0x300)
    cmd.r(0x0104 + 0x400)
    cmd.adc(10)
    regs, _ = run(exe, tmp_path, cmd, adc_signal(10, 0))
    v = [d for _, d in regs]
    assert v[:3] == [0x0002_0100, 0x4E42_4951, 0x0080_7F08], [hex(x) for x in v[:3]]
    for c in range(nb.NCH):
        assert v[3 + 6 * c: 9 + 6 * c] == [0, 0x1111_0000 + c, 128 - c | (29 - c) << 8,
                                           127 - c | (40 + c) << 8, 0x2000_0000 + c * 0x1000,
                                           0xFFF + c * 0x1000], c
    assert v[3 + 6 * nb.NCH:] == [0, 0]


@pytest.mark.parametrize("c", [2, 7])
def test_reenable(exe, tmp_path, c):
    """Disable and re-enable a channel with another configuration while
    samples flow (another channel keeps the FIR busy). The ring holds the
    second configuration's samples, the tap both in turn."""
    nadc = 60000
    adc = adc_signal(nadc, seed=7)
    cmd = Cmd()
    a, o = designs((1e6, 1e6))               # two in use
    b = designs((200e3, 1e6))[0]
    a["finc"] = nb.finc_for(2e6, FS)
    b["finc"] = nb.finc_for(-1e6, FS)
    o["finc"] = nb.finc_for(-3e6, FS)
    other = 1 if c == 7 else 3
    cmd.config(c, a, 0, 0x2000_0000, 1 << 20)
    cmd.config(other, o, 0, 0x3000_0000, 1 << 20)
    cmd.adc(nadc)
    cmd.wait(500)
    cmd.enable(other, 0)
    cmd.enable(c, 0)
    cmd.r(CH_BASE + 0x40 * c + 0x1C)         # T0 of the first configuration
    cmd.wait(30000)
    cmd.config(c, b, 1, 0x2000_0000, 1 << 20)     # writes CTRL = 0 first
    cmd.enable(c, 1)
    cmd.end()
    cmd.readback(c)
    cmd.readback(other)
    regs, mem = run(exe, tmp_path, cmd, adc)
    t0a = regs[0][1]
    wptr, t0lo, t0hi, drops = [v for _, v in regs[1:5]]
    assert drops == 0
    mi, mq = nb.channel(adc[2][t0lo:], adc[3][t0lo:], b)
    k0, ri, rq = ring(mem, 0x2000_0000, 1 << 20, wptr)
    assert k0 == 0 and len(ri) > 300
    assert np.array_equal(ri, mi[:len(ri)]) and np.array_equal(rq, mq[:len(rq)])
    ti, tq = run.tap[c]
    ai, aq = nb.channel(adc[0][t0a:], adc[1][t0a:], a)
    n2 = [n for n in range(len(ri), len(ri) + 81)
          if np.array_equal(ti[len(ti) - n:], mi[:n]) and np.array_equal(tq[len(tq) - n:], mq[:n])]
    assert n2, "tap: the second configuration's samples"
    n1 = len(ti) - n2[0]
    assert n1 > 200 and np.array_equal(ti[:n1], ai[:n1]) and np.array_equal(tq[:n1], aq[:n1])
    wo, t0o, _, dro = [v for _, v in regs[5:9]]
    mo, moq = nb.channel(adc[0][t0o:], adc[1][t0o:], o)
    _, roi, roq = ring(mem, 0x3000_0000, 1 << 20, wo)
    assert dro == 0 and len(roi) > 1000
    assert np.array_equal(roi, mo[:len(roi)]) and np.array_equal(roq, moq[:len(roq)])


# ---------------------------------------------------------------- model only

def next_to_users(rate):
    """The design for a channel at rate next to seven 48 kHz channels."""
    fo = FS / (2 * round(FS / (2 * rate)))
    return nb.design(FS, rate, npairs=nb.allocate([fo] + [48e3] * 7)[0])


@pytest.mark.parametrize("rate", [1e6, 500e3, 250e3, 100e3, 48e3, FS / 256])
def test_tone(rate):
    """A tone at offset + df comes out at df with the designed gain (8) and
    the image / spurs well below it (the channel next to seven 48 kHz
    channels)."""
    cfg = next_to_users(rate)
    foff, amp = 1.234e6, 1000.0
    df = round(0.21 * 4096) / 4096 * cfg["fout"]       # on an FFT bin
    n = int(4096 * FS / rate) + 2 * cfg["R"] * (cfg["npairs"] + 60)
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


@pytest.mark.parametrize("rate", [1e6, 500e3, 250e3, 100e3, 48e3])
def test_alias(rate):
    """A strong tone outside the channel that folds onto 0.3 fout is
    rejected by at least 60 dB (80 % of the band is alias-protected)."""
    cfg = next_to_users(rate)
    fo = cfg["fout"]
    worst = -200
    for k in (1, -1, 2, -2, 3):
        f = 0.3 * fo + k * fo
        n = int(2048 * FS / rate) + 2 * cfg["R"] * (cfg["npairs"] + 60)
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


@pytest.mark.parametrize("R", [2, 6, 37, 64, 125, 128])
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
    """sw/nbiq.c designs the same filters as model/nbiq.py (taps within 1 LSB)
    and allocates the same tap pairs."""
    sw = os.path.join(HERE, "..", "sw")
    os.makedirs(BUILD, exist_ok=True)
    tool = os.path.join(BUILD, "nbiq-host")         # not in sw/: that is the package source
    subprocess.run(["gcc", "-O2", "-Wall", "-Wextra", "-std=gnu11", "-o", tool,
                    os.path.join(sw, "nbiq.c"), "-lm"], check=True)
    for R in range(2, 129):
        out = subprocess.run([tool, "-D", str(R)], check=True,
                             capture_output=True, text=True).stdout.split("\n")
        hdr = out[0].split()
        d = nb.design(FS, FS / (2 * R))
        assert [int(hdr[i]) for i in (1, 3, 5, 7)] == [d["R"], d["cshift"], d["npairs"], d["oshift"]], R
        h = np.array([int(v) for v in out[1:] if v])
        assert np.abs(h - np.array(d["h"])).max() <= 1, R
    # the FIR allocation (nbiq -P): the same as allocate() / fir_check()
    rng = np.random.default_rng(3)
    plans = [[48e3] * 7 + [600e3], [600e3, 48e3], [FS / 4] * 8, [1e6] * 5]
    plans += [list(FS / (2 * rng.integers(2, 129, rng.integers(1, nb.NCH + 1)))) for _ in range(300)]
    for rates in plans:
        out = subprocess.run([tool, "-P", ",".join(f"{r:.6f}" for r in rates)], check=True,
                             capture_output=True, text=True).stdout.split()
        fo = [FS / (2 * round(FS / (2 * r))) for r in rates]
        npairs = nb.allocate(fo)
        load, jq, _ = nb.fir_check(fo, npairs)
        assert (int(out[1]), int(out[5])) == (npairs[0], jq) and abs(float(out[3]) - load) < 1e-6, rates


LOADS = [  # rates of the channels in use (0.., in order)
    (1e6,) * 8,                              # 88 % load, 10-pair jobs
    (FS / 4,) * 8,                           # 3 MSPS (R 2) everywhere: 2-pair jobs
    (FS / 4,) + (48e3,) * 7,                 # 3 MSPS next to seven 25-pair jobs
    (600e3, 48e3),                           # the sample-buffer bound lowers 127 to 126 pairs
    (FS / 256, FS / 250, FS / 256, 3e6 / 2, FS / 24, 48e3, 1e6, FS / 4),  # R 128 .. 2
    (1e6, 1e6, 1e6, 1e6),                    # four in use
    (FS / 2 / 3, 600e3, 120e3, 1e6, 48e3),
]


@pytest.mark.parametrize("rates", LOADS)
def test_load(exe, tmp_path, rates):
    """Heavy FIR load (the driver's allocation) and long jobs next to short
    ones: no drops, bit exact, the tap equals the ring."""
    nadc = 72000
    adc = adc_signal(nadc, seed=11)
    cmd = Cmd()
    cfgs = designs(rates)
    load, jq, ok = nb.fir_check([d["fout"] for d in cfgs], [d["npairs"] for d in cfgs])
    assert ok and load > 0.5, (load, jq)
    nch = len(rates)
    for c, cfg in enumerate(cfgs):
        cfg["finc"] = nb.finc_for(0.7e6 * (c - 3.5), FS)
        cmd.config(c, cfg, c & 1, 0x1000_0000 + c * 0x10_0000, 1 << 20)
    cmd.adc(nadc)
    cmd.wait(300)
    for c in range(nch):
        cmd.enable(c, c & 1)
    cmd.end()
    for c in range(nch):
        cmd.readback(c)
    regs, mem = run(exe, tmp_path, cmd, adc, stall=20)
    for c in range(nch):
        wptr, t0lo, _, drops = [v for _, v in regs[4 * c: 4 * c + 4]]
        assert drops == 0, f"ch{c} drops {drops:#x}"
        mi, mq = nb.channel(adc[2 * (c & 1)][t0lo:], adc[2 * (c & 1) + 1][t0lo:], cfgs[c])
        k0, ri, rq = ring(mem, 0x1000_0000 + c * 0x10_0000, 1 << 20, wptr)
        assert len(mi) - 32 <= len(ri) <= len(mi) and len(ri) > 30
        assert np.array_equal(ri, mi[:len(ri)]) and np.array_equal(rq, mq[:len(rq)]), f"ch{c}"
        check_tap(c, k0, ri, rq, mi, mq)


def test_fir_bounds():
    """allocate() finds tap pairs within the FIR engine's limits (header of
    rtl/nbiq_fir.sv) for every mix of rates (1..8 channels in use): the
    largest common P, the FIFO below 16 entries."""
    Rs = [2, 3, 4, 5, 6, 8, 10, 12, 16, 24, 43, 44, 60, 86, 125, 128]
    worst = 0
    for k in range(1, nb.NCH + 1):
        for comb in itertools.combinations_with_replacement(Rs if k <= 6 else Rs[::2], k):
            fo = [FS / (2 * R) for R in comb]
            npairs = nb.allocate(fo)
            load, jq, ok = nb.fir_check(fo, npairs)
            assert ok and len(set(npairs)) == 1, (comb, load, jq)
            p = npairs[0]
            assert p == 127 or not nb.fir_check(fo, [p + 1] * k)[2], comb
            worst = max(worst, jq)
    assert worst < 2 * nb.NCH <= nb.FIR_JQ, worst


def test_allocation_quality():
    """With the station's plan (seven 48 kHz channels and the 600 kHz
    overview) every channel keeps the full rejection over 80 % of its band;
    FIR_NEED pairs are enough for it at any R >= 10."""
    fo = [48e3] * 7 + [600e3]
    p = nb.allocate(fo)[0]
    assert p == 95
    for r in (48e3, 600e3):
        assert nb.alias_rejection(nb.design(FS, r, npairs=p), 0.8) > 59.5
        assert nb.alias_rejection(nb.design(FS, r, npairs=p), 0.9) > 52.5
    for R in (10, 24, 125, 128):
        d = nb.design(FS, FS / (2 * R), npairs=nb.FIR_NEED)
        assert nb.alias_rejection(d, 0.8) > 59.5, R


def test_fast_adc(exe, tmp_path):
    """ADC samples 2..4 clocks apart (31.25 MSPS: the channels take I and Q
    in turn, the ADC FIFO is popped at most every other clock): bit exact."""
    fs = 62.5e6 / 2
    nadc = 60000
    adc = adc_signal(nadc, seed=5)
    cmd = Cmd()
    cfgs = []
    alloc = nb.allocate([fs / 4, fs / 12, fs / 32])
    for (c, R), p in zip(((1, 2), (3, 6), (7, 16)), alloc):
        cfg = nb.design(fs, fs / (2 * R), npairs=p)
        cfg["finc"] = nb.finc_for(1.3e6 * c, fs)
        cfgs.append((c, cfg))
        cmd.config(c, cfg, c & 1, 0x1000_0000 + c * 0x10_0000, 1 << 20)
    load, jq, ok = nb.fir_check([fs / (2 * d["R"]) for _, d in cfgs], [d["npairs"] for _, d in cfgs])
    assert ok, (load, jq)
    cmd.adc(nadc)
    cmd.wait(100)
    for c, _ in cfgs:
        cmd.enable(c, c & 1)
    cmd.end()
    cmd.r(0x14)
    for c, _ in cfgs:
        cmd.readback(c)
    regs, mem = run(exe, tmp_path, cmd, adc, adc_ps=16000, adc_div=2)
    assert regs[0][1] == nadc, "every ADC sample taken"
    for k, (c, cfg) in enumerate(cfgs):
        wptr, t0, _, drops = [v for _, v in regs[1 + 4 * k: 5 + 4 * k]]
        mi, mq = nb.channel(adc[2 * (c & 1)][t0:], adc[2 * (c & 1) + 1][t0:], cfg)
        k0, ri, rq = ring(mem, 0x1000_0000 + c * 0x10_0000, 1 << 20, wptr)
        assert drops == 0 and len(ri) > 500, (c, drops, len(ri))
        assert np.array_equal(ri, mi[:len(ri)]) and np.array_equal(rq, mq[:len(rq)]), f"ch{c}"
        check_tap(c, k0, ri, rq, mi, mq)


@pytest.mark.parametrize("phase", range(9))
def test_enable_phase(exe, tmp_path, phase):
    """Enabling at any clock relative to the ADC samples: T0 names the first
    sample the channel used, for either source."""
    nadc = 8000
    adc = adc_signal(nadc, seed=phase)
    cmd = Cmd()
    cfgs = []
    chans = (0, 7)
    for k, c in enumerate(chans):
        cfg = nb.design(FS, 1e6)
        cfg["finc"] = nb.finc_for(1.1e6 * (k + 1), FS)
        cfgs.append(cfg)
        cmd.config(c, cfg, k, 0x1000_0000 + k * 0x10_0000, 1 << 20)
    cmd.adc(nadc)
    cmd.wait(200 + phase)
    cmd.enable(chans[0], 0)
    cmd.enable(chans[1], 1)
    cmd.end()
    for c in chans:
        cmd.readback(c)
    regs, mem = run(exe, tmp_path, cmd, adc)
    for k, c in enumerate(chans):
        wptr, t0, _, drops = [v for _, v in regs[4 * k: 4 * k + 4]]
        mi, mq = nb.channel(adc[2 * k][t0:], adc[2 * k + 1][t0:], cfgs[k])
        _, ri, rq = ring(mem, 0x1000_0000 + k * 0x10_0000, 1 << 20, wptr)
        assert drops == 0 and len(ri) > 300
        assert np.array_equal(ri, mi[:len(ri)]) and np.array_equal(rq, mq[:len(rq)]), f"ch{c}"
