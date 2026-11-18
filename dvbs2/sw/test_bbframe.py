"""C BBFRAME builder (bbframe.c) vs gr-dtv dvb_bbheader_bb, bit exact.
Run: ../.venv/bin/python -m pytest test_bbframe.py   (from dvbs2/sw)"""
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "model"))
import grref  # noqa: E402
import txmodel as m  # noqa: E402


@pytest.fixture(scope="module")
def cli(tmp_path_factory):
    exe = tmp_path_factory.mktemp("bb") / "bbframe-cli"
    subprocess.run(["gcc", "-O2", "-o", str(exe), os.path.join(HERE, "bbframe_cli.c"),
                    os.path.join(HERE, "bbframe.c")], check=True)
    return str(exe)


@pytest.mark.parametrize("rate_idx,rolloff", [(r, ro) for r in range(11) for ro in (0.35, 0.25, 0.20)][::2])
def test_bbframe_vs_gr(cli, rate_idx, rolloff):
    rate = m.RATES[rate_idx]
    kb = m.LDPC[rate]["kbch"]
    rng = np.random.default_rng(rate_idx)
    ts = rng.integers(0, 256, (4 * kb // 1504 + 3, 188), dtype=np.uint8)
    ts[:, 0] = 0x47
    gr = grref.bbheader(ts.ravel(), rate, rolloff)
    c = subprocess.run([cli, str(rate_idx), str(rolloff)], input=ts.tobytes(),
                       capture_output=True, check=True).stdout
    cb = np.unpackbits(np.frombuffer(c, dtype=np.uint8))
    nf = min(len(gr), len(cb)) // kb
    assert nf >= 3
    assert np.array_equal(gr[:nf * kb], cb[:nf * kb])


@pytest.mark.parametrize("rate_idx", range(10))
def test_bbframe_short_vs_gr(cli, rate_idx):
    """Short FECFRAMEs (Kbch of Table 5b): the C BBFRAMEs equal gr-dtv's."""
    rate = m.RATES[rate_idx]
    kb = m.rate_params(rate_idx, True)[0]
    rng = np.random.default_rng(50 + rate_idx)
    ts = rng.integers(0, 256, (6 * kb // 1504 + 3, 188), dtype=np.uint8)
    ts[:, 0] = 0x47
    gr = grref.bbheader(ts.ravel(), rate, 0.35, short=True)
    c = subprocess.run([cli, str(rate_idx), "0.35", "1"], input=ts.tobytes(),
                       capture_output=True, check=True).stdout
    cb = np.unpackbits(np.frombuffer(c, dtype=np.uint8))
    nf = min(len(gr), len(cb)) // kb
    assert nf >= 4
    assert np.array_equal(gr[:nf * kb], cb[:nf * kb])
