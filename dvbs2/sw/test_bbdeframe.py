"""C BBFRAME de-adaptation (bbdeframe.c): BBFRAMEs from gr-dtv's bbheader and
from our own builder back to the original TS packets.
Run: ../.venv/bin/python -m pytest test_bbdeframe.py   (from dvbs2/sw)"""
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "model"))
import grref  # noqa: E402
import txmodel as m  # noqa: E402


def build(tmp, name, srcs):
    exe = tmp / name
    subprocess.run(["gcc", "-O2", "-Wall", "-Wextra", "-Werror", "-o", str(exe)] +
                   [os.path.join(HERE, s) for s in srcs], check=True)
    return str(exe)


@pytest.fixture(scope="module")
def tools(tmp_path_factory):
    t = tmp_path_factory.mktemp("bbd")
    return (build(t, "bbdeframe-cli", ["bbdeframe_cli.c", "bbdeframe.c", "bbframe.c"]),
            build(t, "bbframe-cli", ["bbframe_cli.c", "bbframe.c"]))


def ts_packets(n, seed):
    rng = np.random.default_rng(seed)
    ts = rng.integers(0, 256, (n, 188), dtype=np.uint8)
    ts[:, 0] = 0x47
    return ts


@pytest.mark.parametrize("rate_idx", [0, 3, 7, 10])
def test_gr_frames(tools, rate_idx):
    deframe, _ = tools
    rate = m.RATES[rate_idx]
    kb = m.LDPC[rate]["kbch"]
    ts = ts_packets(6 * kb // 1504 + 3, rate_idx)
    bits = grref.bbheader(ts.ravel(), rate)
    nf = len(bits) // kb
    frames = np.packbits(bits[:nf * kb]).tobytes()
    r = subprocess.run([deframe, str(rate_idx)], input=frames, capture_output=True, check=True)
    out = np.frombuffer(r.stdout, np.uint8).reshape(-1, 188)
    print(r.stderr.decode())
    assert len(out) >= nf * (kb - 80) // 1504 - 1
    assert np.array_equal(out, ts[:len(out)])
    assert b"crc_errors 0" in r.stderr and b"bad_header 0" in r.stderr


def test_lost_frame(tools):
    """A lost BBFRAME: the packets around it are dropped, the rest is exact."""
    deframe, frame = tools
    rate_idx = 3
    kb8 = m.LDPC[m.RATES[rate_idx]]["kbch"] // 8
    ts = ts_packets(200, 9)
    frames = subprocess.run([frame, str(rate_idx), "0.35"], input=ts.tobytes(),
                            capture_output=True, check=True).stdout
    nf = len(frames) // kb8
    assert nf >= 5
    r = subprocess.run([deframe, str(rate_idx), "2"], input=frames, capture_output=True, check=True)
    out = np.frombuffer(r.stdout, np.uint8).reshape(-1, 188)
    # every output packet is one of the inputs, in order, none duplicated
    idx = [int(np.nonzero(np.all(ts == p, axis=1))[0][0]) for p in out]
    assert idx == sorted(set(idx))
    lost = (kb8 - 10) * 1 // 188 + 2                   # about one frame's worth, plus the split ones
    assert len(ts[:idx[-1] + 1]) - len(out) <= lost + 1
    assert b"crc_errors 0" in r.stderr


@pytest.fixture(scope="module")
def rx_tool(tmp_path_factory):
    t = tmp_path_factory.mktemp("rx")
    exe = t / "dvbs2-rx"
    subprocess.run(["gcc", "-O2", "-Wall", "-Wextra", "-Werror", "-std=gnu11", "-o", str(exe),
                    os.path.join(HERE, "dvbs2_rx.c"), os.path.join(HERE, "bbdeframe.c"),
                    os.path.join(HERE, "bbframe.c"), "-lm"], check=True)
    return str(exe)


@pytest.mark.parametrize("sr,dec,rolloff,gold", [(500e3, 0, 0.35, 0), (2e6, 3, 0.25, 7), (125e3, 9, 0.20, 1000)])
def test_rx_coefs(rx_tool, sr, dec, rolloff, gold):
    """dvbs2-rx computes the same RRC bank, DDC filter and Gold states as the model."""
    import rxfixed as fx
    args = [rx_tool, "--dump-coefs", "-s", str(sr), "-r", str(rolloff), "-g", str(gold)]
    if dec:
        args += ["-d", str(dec)]
    out = subprocess.run(args, capture_output=True, text=True, check=True).stdout.split("\n")
    rrc = np.array([[int(v) for v in out[k].split()] for k in range(32)])
    assert np.array_equal(rrc, fx.timing_coefs(rolloff))
    d = dec or int(np.ceil(2.1e6 / (2 * sr)))
    assert np.array_equal(np.array([int(v) for v in out[32].split()]), fx.ddc_coefs(d))
    assert [int(v) for v in out[33].split()] == list(m.gold_init(gold))


def test_rx_defaults():
    """dvbs2-rx's loop gains / threshold equal the model's defaults (rtl/gen/dvbs2_rx_pkg.sv)."""
    import re
    import rxfixed as fx
    src = open(os.path.join(HERE, "dvbs2_rx.c")).read()
    d = {k: int(v) for k, v in re.findall(r"#define (TIMING_KP|TIMING_KI|PLL_KP|PLL_KI|FS_THRESH) (\d+)", src)}
    assert (d["TIMING_KP"], d["TIMING_KI"]) == fx.timing_gains()
    assert (d["PLL_KP"], d["PLL_KI"]) == fx.pll_gains()
    assert d["FS_THRESH"] == fx.fs_threshold()
