"""Self-test noise source: RTL (tb_rx_awgn.sv) against the model, and the
statistics of the noise."""
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
RTL = os.path.join(HERE, "..", "rtl")
sys.path.insert(0, os.path.join(HERE, "..", "model"))
import awgn  # noqa: E402


@pytest.fixture(scope="module")
def exe():
    obj = os.path.join(HERE, "build", "tb_rx_awgn")
    os.makedirs(obj, exist_ok=True)
    subprocess.run(["verilator", "--binary", "--timing", "-j", "16", "-O3", "-Wno-fatal",
                    "-Wno-WIDTHTRUNC", "-Wno-WIDTHEXPAND", "--top-module", "tb_rx_awgn", "-Mdir", obj,
                    os.path.join(RTL, "rx_awgn.sv"), os.path.join(RTL, "rx_awgn_axi.sv"),
                    os.path.join(HERE, "tb_rx_awgn.sv")], check=True, stdout=subprocess.DEVNULL)
    return os.path.join(obj, "Vtb_rx_awgn")


def run(exe, tmp_path, x, cmds):
    fa = tmp_path / "adc.txt"
    with open(fa, "w") as f:
        for row in zip(*[np.asarray(v) & 0xFFFF for v in x]):
            f.write("%04x %04x %04x %04x\n" % row)
    fc = tmp_path / "cmd.txt"
    fc.write_text("\n".join(cmds) + "\n")
    fo = tmp_path / "out.txt"
    subprocess.run([exe, f"+adc={fa}", f"+cmd={fc}", f"+out={fo}", "+adc_div=4"], check=True,
                   capture_output=True, cwd=HERE, timeout=600)
    regs, out = [], []
    for line in fo.read_text().split("\n"):
        p = line.split()
        if not p:
            continue
        if p[0] == "R":
            regs.append(int(p[2], 16))
        else:
            out.append([int(p[1])] + [int(v, 16) for v in p[2:]])
    o = np.array(out, np.int64)
    o[:, 1:] = np.where(o[:, 1:] >= 32768, o[:, 1:] - 65536, o[:, 1:])
    return regs, o


def signal(n, seed):
    rng = np.random.default_rng(seed)
    return [rng.integers(-2048, 2048, n) for _ in range(4)]


@pytest.mark.parametrize("gain,nscale", [(16384, 20000), (4096, 131071), (20000, 1)])
def test_bit_exact(exe, tmp_path, gain, nscale):
    n = 70000
    x = signal(n, gain)
    regs, o = run(exe, tmp_path, x, [f"W c {gain:x}", f"W 10 {nscale:x}", "W 8 1", "D 20",
                                    f"A {n}", "E", "R 14", "R 18", "R 1c"])
    assert len(o) == n
    gi, gq = awgn.noise(int(o[-1, 0]) + 1)
    # the generator runs 1 clock ahead of the sample it is added to plus the
    # pipeline: find that fixed offset
    for d in range(0, 16):
        yi = awgn.apply(x[0], gi[o[:, 0] - d], gain, nscale)
        if np.array_equal(yi, o[:, 1]):
            break
    else:
        pytest.fail("no alignment between RTL and model noise")
    assert np.array_equal(awgn.apply(x[1], gq[o[:, 0] - d], gain, nscale), o[:, 2])
    assert np.array_equal(o[:, 3], x[2]) and np.array_equal(o[:, 4], x[3]), "rx1 must pass unchanged"
    # power meters: the first full window (2^16 samples)
    p_in, p_out, cnt = regs
    assert cnt == 1
    w = slice(0, 1 << 16)
    assert p_in == (x[0][w] ** 2 + x[1][w] ** 2).sum() >> 16
    assert p_out == (o[w, 1] ** 2 + o[w, 2] ** 2).sum() >> 16


def test_bypass(exe, tmp_path):
    n = 3000
    x = signal(n, 5)
    regs, o = run(exe, tmp_path, x, ["W 10 30000", "W 8 0", "D 20", f"A {n}", "E"])
    for k in range(4):
        assert np.array_equal(o[:, 1 + k], x[k])


def test_statistics():
    gi, gq = awgn.noise(400000)
    for g in (gi, gq):
        assert abs(g.mean()) < 3 * awgn.SIGMA_G / np.sqrt(len(g))
        assert abs(g.std() / awgn.SIGMA_G - 1) < 0.005
        k = ((g / g.std()) ** 4).mean() - 3
        assert abs(k + 0.075) < 0.03, k             # Irwin-Hall(16)
        # white: normalized autocorrelation at lags 1..64 within noise
        r = np.array([np.dot(g[:-L], g[L:]) for L in range(1, 65)]) / np.dot(g, g)
        assert np.abs(r).max() < 5 / np.sqrt(len(g))
    gi, gq = gi.astype(float), gq.astype(float)
    assert abs(np.dot(gi, gq) / np.sqrt(np.dot(gi, gi) * np.dot(gq, gq))) < 5 / np.sqrt(len(gi))


def test_sigma_scale():
    assert abs(awgn.sigma(16384) - awgn.SIGMA_G) < 1e-9
    assert awgn.nscale_for(awgn.sigma(12345)) == 12345
    assert round(awgn.SIGMA_G * 65536) == 19372513
