"""8PSK (MODCOD 12..17) and short FECFRAMEs through the receiver RTL, bit
exact to the model (model/rxfixed.py; docs/8psk_short_design.md):

* the single-slot frame sync and carrier (the sources of the two-slot
  engines): tags, LLR words (3 LLRs a word for 8PSK), the 8PSK phase
  detector without pilots;
* the two-slot carrier with an 8PSK slot and a short-frame slot;
* the whole two-slot core (LLR queue with the 8PSK deinterleaver, the LDPC
  decoder's short codes, the GF(2^14) BCH decoder): every BBFRAME decoded.

    cd sim/rx && ../../.venv/bin/python -m pytest -v test_rx_8psk.py
"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "model"))
sys.path.insert(0, HERE)

import rxfixed as fx            # noqa: E402
import rxfixed_experiment as ex  # noqa: E402
import rxsim                     # noqa: E402
import txmodel as m              # noqa: E402
from test_rx import model_symbols, write_coefs, write_tagged, RX_SOURCES   # noqa: E402
from test_rx2 import all_decoded, core_run, llr_words, read_segments       # noqa: E402

WORK = os.path.join(rxsim.BUILD, "work")


def sig(modcod, short, pilots, nframes=2, lead=8, esn0=None, cfo=0.004, ppm=60.0, seed=1, rms=1500.0, pn=0.0):
    if esn0 is None:
        esn0 = 9.0 if modcod >= 12 else 4.0
    return ex.make_signal_x(modcod, short, pilots, nframes, cfo, ppm, 0.37, esn0, seed, lead=lead, rms=rms, pn=pn)


def decode(frames, modcod, short):
    return [bb for bb, _, ok in ex.decode_model(frames, modcod, short) if ok]


# ---------------------------------------------------------------------------
# single-slot frame sync and carrier
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def fs_exe():
    return rxsim.build("tb_rx_framesync", ["dvbs2_pl_pkg.sv", "dvbs2_rx_framesync.sv",
                                           os.path.join(HERE, "tb_rx_framesync.sv")])


FS_CASES = [("p8_35_p", 12, False, True, 0), ("q12_short_np", 4, True, False, 20),
            ("p8_23_short_p", 13, True, True, 10), ("p8_910_np", 17, False, False, 5)]


@pytest.mark.parametrize("name,modcod,short,pilots,stall", FS_CASES, ids=[c[0] for c in FS_CASES])
def test_framesync_x(fs_exe, name, modcod, short, pilots, stall):
    """The frame sync finds the headers of 8PSK and short frames (their
    header and frame length) as the model does."""
    os.makedirs(WORK, exist_ok=True)
    x, _ = sig(modcod, short, pilots, nframes=4, lead=3)
    sym = model_symbols(x)
    fs = fx.FrameSync(modcod, pilots, short=short)
    tags, nout = fs.run(sym)
    fin = os.path.join(WORK, f"fx_in_{name}.hex")
    fout = os.path.join(WORK, f"fx_out_{name}.hex")
    rxsim.write_iq(fin, sym)
    log = rxsim.run(fs_exe, modcod=modcod, pilots=int(pilots), short=int(short), thresh=fs.T, stall=stall,
                    **{"in": fin, "out": fout})
    print(log, fs.events)
    got = rxsim.read_hex_cols(fout, (-16, -16, 1, 1))
    assert len(got) == nout
    got_tag = np.where(got[:, 2] == 1, 1 + got[:, 3], 0)
    bad = np.nonzero(got_tag != tags[:nout])[0]
    assert len(bad) == 0, f"tag mismatch at {bad[:5]}"
    L = fx.frame_len(pilots, modcod, short)
    d = np.nonzero(tags[:nout] == 1)[0]
    assert len(d) >= 3 and np.all(np.diff(d) == L)


@pytest.fixture(scope="module")
def carrier_exe():
    return rxsim.build("tb_rx_carrier", ["gen/dvbs2_tables_pkg.sv", "dvbs2_pl_pkg.sv", "gen/dvbs2_rx_pkg.sv",
                                         "dvbs2_rx_rot.sv", "dvbs2_rx_cordic.sv", "dvbs2_udiv.sv",
                                         "dvbs2_rx_carrier.sv", os.path.join(HERE, "tb_rx_carrier.sv")])


def carrier_case(modcod, short, pilots, **kw):
    x, bb = sig(modcod, short, pilots, **kw)
    sym = model_symbols(x)
    fs = fx.FrameSync(modcod, pilots, short=short)
    tags, nout = fs.run(sym)
    sym, tags, found = sym[:nout], tags[:nout], fs.found[:nout]
    cr = fx.Carrier(modcod, pilots, short=short)
    frames = cr.run(sym, tags, found)
    return sym, tags, found, cr, frames, bb


CR_CASES = [
    # 8PSK 3/5 (bit order reversed) with pilots, 8PSK 9/10 without pilots (the
    # 8PSK detector), QPSK 1/4 short, 8PSK 2/3 short without pilots, 8PSK 5/6
    # with pilots and phase noise
    ("p8_35_p", 12, False, True, dict(), 10),
    ("p8_910_np", 17, False, False, dict(esn0=13.0, cfo=-0.002), 0),
    ("q14_short_p", 1, True, True, dict(esn0=1.0, nframes=4), 20),
    ("p8_23_short_np", 13, True, False, dict(esn0=9.0, nframes=5), 5),
    ("p8_56_p_pn", 15, False, True, dict(esn0=11.0, pn=0.002, seed=4), 0),
]


@pytest.mark.parametrize("name,modcod,short,pilots,kw,stall", CR_CASES, ids=[c[0] for c in CR_CASES])
def test_carrier_x(carrier_exe, name, modcod, short, pilots, kw, stall):
    """Carrier RTL = model, word for word (3 LLRs a word for 8PSK, Nldpc / 2
    or / 3 words a frame); every data frame decodes (LDPC/BCH models)."""
    os.makedirs(WORK, exist_ok=True)
    sym, tags, found, cr, frames, bb = carrier_case(modcod, short, pilots, **kw)
    want = llr_words(frames, modcod)
    nw = (16200 if short else 64800) // (3 if modcod >= 12 else 2)
    fin = os.path.join(WORK, f"cx_in_{name}.hex")
    fout = os.path.join(WORK, f"cx_out_{name}.hex")
    write_tagged(fin, sym, tags, found)
    xa, xb = m.gold_init(0)
    log = rxsim.run(carrier_exe, modcod=modcod, pilots=int(pilots), short=int(short), xa=xa, xb=xb, kp=cr.kp_p,
                    ki=cr.ki_p, stall=stall, wz=1, **{"in": fin, "out": fout})
    print(log, cr.n_acq, cr.n_fa, cr.fhat)
    assert f"acq {cr.n_acq} fa {cr.n_fa} retype {cr.n_retype}" in log
    got = rxsim.read_hex_cols(fout, (-8, -8, 1, 1, -8))
    assert len(got) == len(want) and len(want) >= nw, (len(got), len(want))
    bad = np.nonzero(np.any(got[:, [0, 1, 4]] != want, axis=1))[0]
    assert len(bad) == 0, f"{len(bad)} mismatches, first at word {bad[0]}: {got[bad[0]]} vs {want[bad[0]]}"
    assert np.array_equal(np.nonzero(got[:, 2])[0], np.arange(0, len(got), nw))
    assert np.array_equal(np.nonzero(got[:, 3])[0], np.arange(nw - 1, len(got), nw))
    dec = decode(frames, modcod, short)
    assert all(any(np.array_equal(d, b) for d in dec) for b in bb), f"{len(dec)} of {len(bb)} decoded"


# ---------------------------------------------------------------------------
# two-slot carrier and the whole core
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def carrier2_exe():
    return rxsim.build("tb_rx_carrier2", ["gen/dvbs2_tables_pkg.sv", "dvbs2_pl_pkg.sv", "gen/dvbs2_rx_pkg.sv",
                                          "dvbs2_rx_rot.sv", "dvbs2_rx_cordic.sv", "dvbs2_udiv.sv",
                                          "dvbs2_rx_carrier2.sv", os.path.join(HERE, "tb_rx_carrier2.sv")],
                       defines=["RX2_SCRAMBLE"])


def test_carrier2_x(carrier2_exe):
    """Two-slot carrier: 8PSK 3/4 without pilots in slot 0, QPSK 1/2 short
    with pilots in slot 1; each slot equals the single-slot model."""
    os.makedirs(WORK, exist_ok=True)
    cases = [(14, False, False, dict(esn0=11.0, seed=2)), (4, True, True, dict(esn0=3.0, nframes=4, seed=3))]
    plus, want = {"ostall": 5, "wz": 1}, []
    for s, (mc, short, pilots, kw) in enumerate(cases):
        sym, tags, found, cr, frames, _ = carrier_case(mc, short, pilots, **kw)
        fin = os.path.join(WORK, f"c2x_in{s}.hex")
        write_tagged(fin, sym, tags, found)
        xa, xb = m.gold_init(0)
        plus.update({f"in{s}": fin, f"out{s}": os.path.join(WORK, f"c2x_out{s}.hex"), f"modcod{s}": mc,
                     f"pilots{s}": int(pilots), f"short{s}": int(short), f"xa{s}": xa, f"xb{s}": xb,
                     f"kp{s}": cr.kp_p, f"ki{s}": cr.ki_p, f"stall{s}": 5 * s})
        want.append((cr, llr_words(frames, mc)))
    log = rxsim.run(carrier2_exe, **plus)
    print(log)
    for s in range(2):
        cr, w = want[s]
        line = log.split(f"slot {s}: ")[1].split("\n")[0]
        assert f"acq {cr.n_acq} fa {cr.n_fa} retype {cr.n_retype}" in line
        segs = read_segments(plus[f"out{s}"], (-8, -8, 1, 1, -8))
        got = segs[-1]
        assert len(got) == len(w) and len(w) > 0, (s, len(got), len(w))
        bad = np.nonzero(np.any(got[:, [0, 1, 4]] != w, axis=1))[0]
        assert len(bad) == 0, f"slot {s}: {len(bad)} mismatches, first at word {bad[0]}"


@pytest.fixture(scope="module")
def rx_exe():
    return rxsim.build("tb_rx", RX_SOURCES + [os.path.join(HERE, "tb_rx.sv")])


def test_core2_8psk(rx_exe):
    """Whole core: 8PSK 3/5 with pilots (normal frames) in slot 0 and 8PSK 8/9
    without pilots in slot 1, at once; LLRs, PLS decodes and status equal the
    model per slot, every BBFRAME decodes through the LLR queue (the reader's
    deinterleaver, rate 3/5's reversed bit order) and the decoders."""
    xa_, bba = sig(12, False, True, nframes=3, esn0=8.0, seed=5)
    xb_, bbb = sig(16, False, False, nframes=3, esn0=13.0, cfo=-0.002, ppm=-80.0, seed=6, rms=400.0)
    out, log = core_run(rx_exe, "p8", [dict(x=xa_, rate_idx=4, modcod=12, pilots=True, period=30),
                                        dict(x=xb_, rate_idx=9, modcod=16, pilots=False, period=75)])
    assert all(all_decoded(out[0]["dec"], bba)) and all(all_decoded(out[1]["dec"], bbb))


def test_core2_short(rx_exe):
    """Whole core with short FECFRAMEs: QPSK 1/4 short with pilots in slot 0
    (GF(2^14) BCH, an irregular short LDPC code) and 8PSK 2/3 short without
    pilots in slot 1; every BBFRAME decodes."""
    xa_, bba = sig(1, True, True, nframes=6, esn0=1.0, seed=7)
    xb_, bbb = sig(13, True, False, nframes=6, esn0=10.0, cfo=-0.003, seed=8, rms=400.0)
    out, log = core_run(rx_exe, "short", [dict(x=xa_, rate_idx=0, modcod=1, short=True, pilots=True, period=30),
                                           dict(x=xb_, rate_idx=5, modcod=13, short=True, pilots=False, period=75)])
    assert all(all_decoded(out[0]["dec"], bba)) and all(all_decoded(out[1]["dec"], bbb))
