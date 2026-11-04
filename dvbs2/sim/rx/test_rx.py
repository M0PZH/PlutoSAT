"""RX RTL blocks vs the fixed-point model (model/rxfixed.py), bit exact.

    cd sim/rx && ../../.venv/bin/python -m pytest -v test_rx.py
"""
import os
import re
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "model"))
sys.path.insert(0, HERE)

import rxfixed as fx            # noqa: E402
import rxfixed_experiment as ex  # noqa: E402
import rxsim                     # noqa: E402

WORK = os.path.join(rxsim.BUILD, "work")


def signal(nframes=1, lead=3, **kw):
    args = dict(rate_idx=3, pilots=True, cfo=0.004, ppm=60.0, delay=0.37, esn0=8.0, seed=1)
    args.update(kw)
    x, bb = ex.make_signal(args.pop("rate_idx"), args.pop("pilots"), nframes, args.pop("cfo"),
                           args.pop("ppm"), args.pop("delay"), args.pop("esn0"), args.pop("seed"),
                           lead=lead, **args)
    return x, bb


@pytest.fixture(scope="module")
def timing_exe():
    return rxsim.build("tb_rx_timing", ["dvbs2_rx_timing.sv", os.path.join(HERE, "tb_rx_timing.sv")])


@pytest.mark.parametrize("stall,rms,ppm,integ0", [(0, 1500.0, 60.0, 0), (30, 300.0, -100.0, 0),
                                                   (10, 800.0, -1000.0, 33588), (0, 1500.0, 2000.0, -70000)])
def test_timing(timing_exe, stall, rms, ppm, integ0):
    """integ0: integrator preset (TIMING_INTEG write), the last one far off."""
    os.makedirs(WORK, exist_ok=True)
    x, _ = signal(nframes=0, lead=6, rms=rms, ppm=ppm)
    coefs = fx.timing_coefs(0.35)
    kp, ki = fx.timing_gains()
    tr = fx.Timing(coefs, kp, ki, integ0=integ0)
    want = tr.run(x)
    cfile = os.path.join(WORK, "coef.hex")
    with open(cfile, "w") as f:
        for p in range(32):
            for j in range(32):
                f.write(f"{int(coefs[p, j]) & 0x3FFFF if j < fx.NT else 0:05x}\n")
    fin = os.path.join(WORK, f"t_in{stall}.hex")
    fout = os.path.join(WORK, f"t_out{stall}.hex")
    rxsim.write_iq(fin, x)
    log = rxsim.run(timing_exe, coef=cfile, kp=kp, ki=ki, integ0=integ0, stall=stall, **{"in": fin, "out": fout})
    print(log)
    got = rxsim.read_hex_cols(fout, (-16, -16))
    n = min(len(got), len(want))
    assert n > len(want) - 40, (len(got), len(want))
    bad = np.nonzero(np.any(got[:n] != want[:n], axis=1))[0]
    assert len(bad) == 0, f"first mismatch at symbol {bad[0]}: {got[bad[0]]} vs {want[bad[0]]}"


@pytest.fixture(scope="module")
def framesync_exe():
    return rxsim.build("tb_rx_framesync", ["dvbs2_pl_pkg.sv", "dvbs2_rx_framesync.sv",
                                           os.path.join(HERE, "tb_rx_framesync.sv")])


def model_symbols(x):
    kp, ki = fx.timing_gains()
    return fx.Timing(fx.timing_coefs(0.35), kp, ki).run(x)


@pytest.mark.parametrize("stall,pilots,esn0,drop,tx", [(0, True, 8.0, None, None), (25, False, 3.0, None, None),
                                                       (10, True, 8.0, (80000, 80100), None),
                                                       (15, False, 1.0, None, 0)])
def test_framesync(framesync_exe, stall, pilots, esn0, drop, tx):
    """drop: remove symbols [a, b) -> shifted headers: flywheel, lost, search, re-lock.
    tx: another MODCOD (rate index) than the configured QPSK 1/2, no dummy frames: QPSK
    1/4 without pilots is closer to the dummy header than to QPSK 1/2 (the search peak
    is a "dummy" one, checked at the data length too)."""
    os.makedirs(WORK, exist_ok=True)
    if tx is None:
        x, _ = signal(nframes=10 if drop else 3, lead=3, pilots=pilots, esn0=esn0)
    else:
        x, _ = signal(nframes=6, lead=0, rate_idx=tx, pilots=pilots, esn0=esn0)
    sym = model_symbols(x)
    if drop:
        sym = np.concatenate([sym[:drop[0]], sym[drop[1]:]])
    fs = fx.FrameSync(4, pilots)
    tags, nout = fs.run(sym)
    fin = os.path.join(WORK, f"f_in{stall}.hex")
    fout = os.path.join(WORK, f"f_out{stall}.hex")
    rxsim.write_iq(fin, sym)
    log = rxsim.run(framesync_exe, modcod=4, pilots=int(pilots), thresh=fs.T, stall=stall,
                    **{"in": fin, "out": fout})
    print(log, fs.events)
    got = rxsim.read_hex_cols(fout, (-16, -16, 1, 1))
    assert len(got) == nout
    assert np.array_equal(got[:, :2], sym[:nout])
    want_tag = tags[:nout]
    got_tag = np.where(got[:, 2] == 1, 1 + got[:, 3], 0)
    bad = np.nonzero(got_tag != want_tag)[0]
    assert len(bad) == 0, f"tag mismatch at {bad[:5]}: {got_tag[bad[:5]]} vs {want_tag[bad[:5]]}"
    assert ((want_tag == 1) if tx is None else (want_tag != 0)).sum() >= 2
    if tx is not None:
        # two of the headers found (a frame or two apart: the next one is predicted at
        # the dummy length again)
        L = fx.frame_len(pilots)
        f = [e[1] for e in fs.events if e[0] == "found"]
        assert any((b - a) % L == 0 and b - a <= 2 * L for a, b in zip(f, f[1:])), fs.events
    if drop:
        kinds = [e[0] for e in fs.events]
        assert "flywheel" in kinds and "lost" in kinds and kinds[-1] == "found"


class PlsStandIn:
    """Carrier stand-in for FrameSync.run(): a PLS decoder that reports the
    same decode for every tagged header (at its 90th symbol), as the
    testbench's stand-in does."""

    def __init__(self, code, conf):
        self.d = dict(code=code, modcod=code >> 2, short=(code >> 1) & 1, pilots=code & 1, conf=bool(conf))
        self.pls = type("Pls", (), {})()
        self.pls.results = []
        self.k = -1

    def step(self, si, sq, t, found):
        if t:
            self.k = 0
        elif self.k >= 0:
            self.k += 1
        if self.k == 89:
            self.k = -1
            self.pls.results.append(self.d)


@pytest.mark.parametrize("name,code,conf,stall", [("dummy_ok", 0, 1, 10), ("nack", fx.pls_code(4, 0, 1), 0, 0),
                                                  ("other_pilots", fx.pls_code(4, 0, 0), 1, 5),
                                                  ("other_modcod", fx.pls_code(7, 0, 1), 1, 0)])
def test_framesync_pls(framesync_exe, name, code, conf, stall):
    """Header confirmation by the PLS decoder, which is slower here than the
    frame sync needs it (busy 150k clocks after each header; the real one
    needs <= ~95k with H = 32): the frame sync waits for the decode, so the
    tags equal the model's. dummy_ok: dummy frames only, confirmed: locked,
    every header found. nack / other_pilots / other_modcod (not confident /
    not the configured code, even of the same frame length: repeating data can
    decode confidently as some code): each CHECK ends at the first decode,
    never LOCKED."""
    os.makedirs(WORK, exist_ok=True)
    x, _ = signal(nframes=0 if name == "dummy_ok" else 3, lead=14 if name == "dummy_ok" else 3,
                  pilots=True, esn0=8.0)
    sym = model_symbols(x)
    fs = fx.FrameSync(4, True)
    tags, nout = fs.run(sym, PlsStandIn(code, conf))
    fin = os.path.join(WORK, f"fp_in_{name}.hex")
    fout = os.path.join(WORK, f"fp_out_{name}.hex")
    rxsim.write_iq(fin, sym)
    log = rxsim.run(framesync_exe, modcod=4, pilots=1, thresh=fs.T, stall=stall, plsbusy=150000,
                    plscode=code, plsconf=conf, **{"in": fin, "out": fout})
    print(log, fs.events)
    got = rxsim.read_hex_cols(fout, (-16, -16, 1, 1))
    assert len(got) == nout
    assert np.array_equal(got[:, :2], sym[:nout])
    got_tag = np.where(got[:, 2] == 1, 1 + got[:, 3], 0)
    bad = np.nonzero(got_tag != tags[:nout])[0]
    assert len(bad) == 0, f"tag mismatch at {bad[:5]}: {got_tag[bad[:5]]} vs {tags[:nout][bad[:5]]}"
    kinds = [e[0] for e in fs.events]
    if name == "dummy_ok":
        assert "state 2" in log and kinds.count("found") >= 12 and "pls_drop" not in kinds
    else:
        assert kinds.count("pls_drop") >= 2 and "state 2" not in log
        assert all(a != "found" or b != "found" for a, b in zip(kinds, kinds[1:]))


@pytest.fixture(scope="module")
def carrier_exe():
    return rxsim.build("tb_rx_carrier", ["gen/dvbs2_tables_pkg.sv", "dvbs2_pl_pkg.sv", "gen/dvbs2_rx_pkg.sv",
                                         "dvbs2_rx_rot.sv", "dvbs2_rx_cordic.sv", "dvbs2_udiv.sv",
                                         "dvbs2_rx_carrier.sv", os.path.join(HERE, "tb_rx_carrier.sv")])


def write_tagged(path, sym, tags, found):
    s = np.asarray(sym, np.int64) & 0xFFFF
    with open(path, "w") as f:
        for (i, q), t, d in zip(s, tags, found):
            f.write(f"{i:04x} {q:04x} {int(t != 0)} {int(t == 2)} {int(d)}\n")


@pytest.mark.parametrize("name,stall,kw,drop", [
    ("pilots", 0, dict(pilots=True, esn0=6.0), None),
    ("nopilots", 20, dict(pilots=False, esn0=4.0, cfo=-0.003), None),
    ("lowsnr", 0, dict(pilots=True, esn0=0.0, rate_idx=0), None),
    ("gap", 10, dict(pilots=True, esn0=8.0), (52000, 52002)),
    ("gap_np", 0, dict(pilots=False, esn0=8.0), (52000, 52001)),
    ("timeout", 5, dict(pilots=True, esn0=8.0), "untag"),
    ("falselock", 0, dict(pilots=True, esn0=-2.2, rate_idx=0, seed=16, lead=8, nframes=4), None),
])
def test_carrier(carrier_exe, name, stall, kw, drop):
    """gap: drop symbols inside a data frame -> early header, cut frame, padding.
    untag: no header after the last data frame -> segment flushed on timeout."""
    os.makedirs(WORK, exist_ok=True)
    kw = dict(kw)
    pilots = kw["pilots"]
    modcod = kw.get("rate_idx", 3) + 1
    nfr = kw.pop("nframes", 2)
    # (data frames that begin before the acquisition are not sent: enough
    # dummy frames for SEARCH, 2 checks and 4 confirmed headers)
    lead = kw.pop("lead", 8)
    x, _ = signal(nframes=nfr, lead=lead, **kw)
    sym = model_symbols(x)
    if drop and drop != "untag":
        sym = np.concatenate([sym[:drop[0]], sym[drop[1]:]])
    fs = fx.FrameSync(modcod, pilots)
    tags, nout = fs.run(sym)
    sym, tags = sym[:nout], tags[:nout].copy()
    if drop == "untag":
        last = np.nonzero(tags == 1)[0][-1]
        tags[last + 1:] = 0
    found = fs.found[:nout]
    cr = fx.Carrier(modcod, pilots)
    want = cr.run(sym, tags, found)
    want = np.concatenate(want).reshape(-1, 2) if want else np.zeros((0, 2), np.int64)
    fin = os.path.join(WORK, f"c_in_{name}.hex")
    fout = os.path.join(WORK, f"c_out_{name}.hex")
    write_tagged(fin, sym, tags, found)
    import txmodel as m
    xa, xb = m.gold_init(0)
    log = rxsim.run(carrier_exe, modcod=modcod, pilots=int(pilots), xa=xa, xb=xb, kp=cr.kp_p, ki=cr.ki_p,
                    stall=stall, **{"in": fin, "out": fout})
    print(log, fs.events, cr.fhat, cr.A, cr.E, cr.G, "acq", cr.n_acq, "false-lock steps", cr.n_fa,
          "retype", cr.n_retype, "dummy blocks", cr.n_dummy)
    assert f"acq {cr.n_acq} fa {cr.n_fa} retype {cr.n_retype}" in log
    if name == "falselock":
        assert cr.n_fa >= 1            # the case must exercise the detector
    got = rxsim.read_hex_cols(fout, (-8, -8, 1, 1))
    assert len(want) >= 32400
    assert len(got) == len(want), (len(got), len(want))
    bad = np.nonzero(np.any(got[:, :2] != want, axis=1))[0]
    assert len(bad) == 0, f"{len(bad)} mismatches, first at word {bad[0]}: {got[bad[0], :2]} vs {want[bad[0]]}"
    assert np.array_equal(np.nonzero(got[:, 2])[0], np.arange(0, len(got), 32400))
    assert np.array_equal(np.nonzero(got[:, 3])[0], np.arange(32399, len(got), 32400))
    if drop:
        # the shortened frame ends with padding (LLR 0)
        assert any(np.all(f[-2 * (drop[1] - drop[0] if drop != "untag" else 1):] == 0) for f in
                   np.asarray(want).reshape(-1, 64800)) or drop == "untag"


@pytest.mark.parametrize("esn0,seed", [(0.0, 3), (-1.0, 4)])
def test_carrier_pilotless_pn(carrier_exe, esn0, seed):
    """Pilots off at low SNR with phase noise (QPSK 1/4, symbol-level
    channel: 0.001 cycles/symbol, Wiener phase noise 0.002 rad/symbol,
    ideal tags): the RTL equals the model through the PLL gears, and every
    data frame decodes (LDPC decoder model)."""
    import ldpcdec as ld
    import pilotless_experiment as pe
    import txmodel as m
    os.makedirs(WORK, exist_ok=True)
    sym, tags, found, cws, _ = pe.make_symbols(0, False, 3, esn0, 0.001, 0.002, seed, lead=8)
    cr = fx.Carrier(1, False)
    want = cr.run(sym, tags, found)
    assert cr.n_acq == 1 and cr.nhdr >= fx.PLL_GEAR[-1][0] and len(want) == len(cws)
    dec = ld.Decoder(0)
    assert all(np.array_equal(dec.decode(llr)[0], cw) for llr, cw in zip(want, cws))
    want = np.concatenate(want).reshape(-1, 2)
    name = f"pn_{esn0:+.0f}"
    fin = os.path.join(WORK, f"c_in_{name}.hex")
    fout = os.path.join(WORK, f"c_out_{name}.hex")
    write_tagged(fin, sym, tags, found)
    xa, xb = m.gold_init(0)
    log = rxsim.run(carrier_exe, modcod=1, pilots=0, xa=xa, xb=xb, kp=cr.kp_p, ki=cr.ki_p,
                    stall=10, **{"in": fin, "out": fout})
    assert f"acq {cr.n_acq} fa {cr.n_fa} retype {cr.n_retype}" in log
    got = rxsim.read_hex_cols(fout, (-8, -8, 1, 1))
    assert len(got) == len(want), (len(got), len(want))
    bad = np.nonzero(np.any(got[:, :2] != want, axis=1))[0]
    assert len(bad) == 0, f"{len(bad)} mismatches, first at word {bad[0]}: {got[bad[0], :2]} vs {want[bad[0]]}"


RX_SOURCES = ["gen/dvbs2_tables_pkg.sv", "dvbs2_pl_pkg.sv", "gen/dvbs2_rx_pkg.sv", "gen/dvbs2_ldpc_dec_pkg.sv",
              "dvbs2_async_fifo.sv", "dvbs2_rx_timing.sv", "dvbs2_rx_framesync.sv", "dvbs2_rx_rot.sv",
              "dvbs2_rx_cordic.sv", "dvbs2_udiv.sv", "dvbs2_rx_carrier.sv", "dvbs2_ldpc_dec.sv",
              "gen/dvbs2_bch_pkg.sv", "dvbs2_bch_dec.sv", "dvbs2_rx_pls.sv", "dvbs2_rx_llrq.sv", "dvbs2_rx.sv",
              os.path.join(HERE, "axi_mem.sv")]


@pytest.fixture(scope="module")
def rx_exe():
    return rxsim.build("tb_rx", RX_SOURCES + [os.path.join(HERE, "tb_rx.sv")])


def write_coefs(path, coefs):
    with open(path, "w") as f:
        for p in range(32):
            for j in range(32):
                f.write(f"{int(coefs[p, j]) & 0x3FFFF if j < fx.NT else 0:05x}\n")


@pytest.mark.parametrize("name,kw", [
    ("p_1_2", dict(rate_idx=3, pilots=True, esn0=3.0, cfo=0.004, ppm=60.0)),
    ("np_3_4", dict(rate_idx=6, pilots=False, esn0=6.0, cfo=-0.002, ppm=-80.0, rms=400.0)),
    ("p_1_2_llrq", dict(rate_idx=3, pilots=True, esn0=3.0, cfo=0.004, ppm=60.0, llrq=1)),
])
def test_rx_end_to_end(rx_exe, name, kw):
    """Samples -> RTL receiver: the LLRs equal the front-end model, the
    BBFRAMEs equal the LDPC model and the transmitted BBFRAMEs, and
    gr-dvbs2rx's LDPC/BCH decodes the RTL's LLRs to the same frames.
    llrq: the decoder's input through the DDR LLR queue (memory model with
    stalls and latency): the same result."""
    import ldpcdec as ld
    import rxmodel as rf
    import txmodel as m
    os.makedirs(WORK, exist_ok=True)
    kw = dict(kw)
    llrq = kw.pop("llrq", 0)
    rate_idx, pilots = kw["rate_idx"], kw["pilots"]
    x, bb = signal(nframes=4, lead=8, **kw)
    r = fx.receive(x, rate_idx + 1, pilots)
    want = np.concatenate(r["frames"]).reshape(-1, 2)
    cfile = os.path.join(WORK, "coef.hex")
    write_coefs(cfile, fx.timing_coefs(0.35))
    fin = os.path.join(WORK, f"rx_in_{name}.hex")
    fout = os.path.join(WORK, f"rx_out_{name}.hex")
    fbytes = os.path.join(WORK, f"rx_bytes_{name}.hex")
    fpls = os.path.join(WORK, f"rx_pls_{name}.hex")
    rxsim.write_iq(fin, x)
    kp, ki = fx.timing_gains()
    xa, xb = m.gold_init(0)
    log = rxsim.run(rx_exe, coef=cfile, period=22, stall=5, modcod=rate_idx + 1, pilots=int(pilots),
                    thresh=fx.fs_threshold(), xa=xa, xb=xb, tkp=kp, tki=ki,
                    pkp=r["carrier"].kp_p, pki=r["carrier"].ki_p, maxit=30, bytes=fbytes, pls=fpls,
                    llrq=llrq, axistall=30, axilat=200, **{"in": fin, "out": fout})
    print(log)
    assert "overflows 0," in log
    if llrq:
        assert "llrq frames 4 dropped 0 depth max 1 axi err 0," in log
    # PLS decodes: equal to the model, every confident one right
    got_p = [tuple(int(v, 16) for v in l.split()) for l in open(fpls)]
    want_p = [pls_line(d) for d in r["pls"].results]
    print("PLS", want_p)
    assert got_p == want_p and "skipped 0," in log
    codes = [d["code"] for d in r["pls"].results if d["conf"]]
    assert len(codes) >= 8 and set(codes) <= {0, fx.pls_code(rate_idx + 1, 0, pilots)}
    assert fx.pls_code(rate_idx + 1, 0, pilots) in codes
    got = rxsim.read_hex_cols(fout, (-8, -8, 1, 1))
    assert len(got) == len(want), (len(got), len(want))
    bad = np.nonzero(np.any(got[:, :2] != want, axis=1))[0]
    assert len(bad) == 0, f"{len(bad)} mismatches, first at word {bad[0]}"
    llr = got[:, :2].astype(np.int8).reshape(-1, 64800)
    kb = m.LDPC[m.RATES[rate_idx]]["kbch"]
    out = rf.gr_ldpc_decode_llr(llr, rate_idx)
    dec = [any(np.array_equal(out[k * kb:(k + 1) * kb], b) for k in range(len(llr))) for b in bb]
    print("gr-dvbs2rx decoded", dec)
    assert all(dec)
    # RTL LDPC + BCH decoder output
    import bchdec
    dec_m = ld.Decoder(rate_idx)
    got_b = rxsim.read_hex_cols(fbytes, (8, 1, 1, 8, 1, 8, 16, 4, 8, 1, 1))
    nb = kb // 8
    assert len(got_b) == nb * len(llr)
    assert np.all(got_b[:, 7:] == (rate_idx, rate_idx + 1, int(pilots), 0))       # the frames' own tags
    for k in range(len(llr)):
        hard, it, conv = dec_m.decode(llr[k])
        out, ok, L = bchdec.decode(hard[:dec_m.code.K], rate_idx)
        g = got_b[k * nb:(k + 1) * nb]
        assert np.array_equal(g[:, 0], np.packbits(out))
        assert (g[-1, 2], g[-1, 3], g[-1, 4], g[-1, 5]) == (int(ok), it, int(conv), L)
        K = dec_m.code.K
        assert g[-1, 6] == np.sum((llr[k][:K] < 0) != hard[:K])            # pre-FEC errors
        assert ok and np.array_equal(np.unpackbits(g[:, 0].astype(np.uint8)), bb[k]), f"frame {k} not decoded"


def rx_rtl(rx_exe, name, x, rate_idx, pilots, period=22):
    """Samples -> RTL receiver and model: the LLRs and the PLS decodes equal
    the model's. Returns the model result and the BBFRAMEs the RTL decoded
    (BCH ok), as bit arrays."""
    import txmodel as m
    os.makedirs(WORK, exist_ok=True)
    r = fx.receive(x, rate_idx + 1, pilots)
    cfile = os.path.join(WORK, "coef.hex")
    write_coefs(cfile, fx.timing_coefs(0.35))
    fin, fout, fbytes, fpls = (os.path.join(WORK, f"rx_{k}_{name}.hex") for k in ("in", "out", "bytes", "pls"))
    rxsim.write_iq(fin, x)
    kp, ki = fx.timing_gains()
    xa, xb = m.gold_init(0)
    log = rxsim.run(rx_exe, coef=cfile, period=period, stall=5, modcod=rate_idx + 1, pilots=int(pilots),
                    thresh=fx.fs_threshold(), xa=xa, xb=xb, tkp=kp, tki=ki,
                    pkp=r["carrier"].kp_p, pki=r["carrier"].ki_p, maxit=30, bytes=fbytes, pls=fpls,
                    **{"in": fin, "out": fout})
    print(log)
    assert "overflows 0," in log and "skipped 0," in log
    cr = r["carrier"]
    assert f"amp {cr.A}, energy {cr.E}, gain {cr.G}," in log
    got_p = [tuple(int(v, 16) for v in ln.split()) for ln in open(fpls)]
    assert got_p == [pls_line(d) for d in r["pls"].results]
    want = np.concatenate(r["frames"]).reshape(-1, 2) if r["frames"] else np.zeros((0, 2), np.int64)
    got = rxsim.read_hex_cols(fout, (-8, -8, 1, 1))
    # (the RTL also sends the start of a frame the input ends in)
    assert len(want) <= len(got) < len(want) + 32400, (len(got), len(want))
    bad = np.nonzero(np.any(got[:len(want), :2] != want, axis=1))[0]
    assert len(bad) == 0, f"{len(bad)} mismatches, first at word {bad[0]}"
    nb = m.LDPC[m.RATES[rate_idx]]["kbch"] // 8
    got_b = rxsim.read_hex_cols(fbytes, (8, 1, 1, 8, 1, 8, 16, 4, 8, 1, 1))
    assert len(got_b) == nb * len(r["frames"])
    assert np.all(got_b[:, 7:] == (rate_idx, rate_idx + 1, int(pilots), 0))
    dec = [np.unpackbits(got_b[k * nb:(k + 1) * nb, 0].astype(np.uint8))
           for k in range(len(r["frames"])) if got_b[(k + 1) * nb - 1, 2]]
    return r, dec


def test_rx_falselock(rx_exe):
    """Board finding (RX 1.3): a TS of null packets (consecutive PLFRAMEs
    nearly identical) and an RX started mid-frame with the AGC settling from
    above (early metrics up to 50x a header's): the stale search peak is
    checked a frame length later, where the same data correlates the same,
    and the frame sync locked there for good (every tag off the headers, no
    confident PLS decode). Now the PLS decode of the first CHECK tag sends it
    back to SEARCH; every later tag is a header, and the frames decode."""
    import falselock_experiment as fe
    L = fx.frame_len(True)
    tx, bb = fe.tx_signal(10, True, 10, "null")
    start = 28310
    x = fe.rx_input(tx, start, 20.0, 0.002, 20.0, 3, rms=8000)
    r, dec = rx_rtl(rx_exe, "falselock", x, 10, True)
    cl = fe.classify(r, L, start)
    print(cl, r["fs"].events)
    assert not cl[0][2] and all(ok for _, _, ok in cl[1:]) and len(cl) >= 7
    assert [e[0] for e in r["fs"].events].count("pls_drop") == 1
    got = [any(np.array_equal(d, b) for d in dec) for b in bb]
    print("RTL decoded", got)
    assert all(any(np.array_equal(d, b) for b in bb) for d in dec)
    # the carrier acquires at the header of frame 7 (the 4th confirmed one;
    # data frames only), which is not sent (it began before the acquisition):
    # frame 8 decodes (frame 9 is cut short by the end of the input)
    assert got[-2]


def test_rx_start_np14(rx_exe):
    """Model finding (RX 1.3): QPSK 1/4 without pilots at -1 dB, 8 dummy
    frames before the data. QPSK 1/4's header is a differential away from the
    dummy header, so the frame sync's frame type is a coin toss: a dummy
    frame tagged as data sent the prediction a data frame ahead (flywheel on
    noise), and the carrier confirmed only headers whose tag type was right.
    Now the PLS decodes give the frame sync the type, the carrier takes it
    from the frame length, and no header estimate kicks the NCO after the
    acquisition: all frames decode (before: none)."""
    x, bb = signal(nframes=4, lead=8, rate_idx=0, pilots=False, esn0=-1.0, seed=0)
    # 80 clocks per symbol: before the acquisition dummy frames tagged as data
    # go to the LDPC decoder too (30 iterations of QPSK 1/4: ~1.65M clocks)
    r, dec = rx_rtl(rx_exe, "start_np14", x, 0, False, period=40)
    got = [any(np.array_equal(d, b) for d in dec) for b in bb]
    print("RTL decoded", got, r["fs"].events)
    assert all(got)


def esn0_est(A, E):
    """The software's Es/N0 estimate (dB) from the AMP and ENERGY registers."""
    import math
    return 10 * math.log10(A * A / (E - A * A))


def test_rx_esn0_restart(rx_exe):
    """Board finding (RX 1.3, pilots off): the amplitude/noise average (1/8
    per known block, one block per frame without pilots) started from a
    header measured before the frequency acquisition and took ~17 frames to
    forget it (Es/N0 from -21 dB up). Now it restarts after the acquisition:
    QPSK 2/3 without pilots, 0.02 cycles/symbol, 20 dB Es/N0: right from the
    first header after the acquisition (1.3: -20 dB there)."""
    x, bb = signal(nframes=2, lead=8, rate_idx=5, pilots=False, esn0=20.0, cfo=0.02, seed=1)
    r, dec = rx_rtl(rx_exe, "esn0", x, 5, False)
    cr = r["carrier"]
    est = [esn0_est(A, E) for (nhdr, f, fh, th, A, E, G) in cr.log if nhdr >= 3]
    print("Es/N0 after the acquisition", [round(e, 1) for e in est])
    assert len(est) >= 3 and all(18.5 < e < 22 for e in est)
    assert all(any(np.array_equal(d, b) for d in dec) for b in bb)


# ---------------------------------------------------------------------------
# LDPC decoder
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def ldpc_exe():
    return rxsim.build("tb_ldpc_dec", ["gen/dvbs2_ldpc_dec_pkg.sv", "dvbs2_ldpc_dec.sv",
                                       os.path.join(HERE, "tb_ldpc_dec.sv")])


def ldpc_case(rate_idx, esn0, nframes, seed, max_it=30):
    import ldpcdec as ld
    import txmodel as m
    rng = np.random.default_rng(seed)
    dec = ld.Decoder(rate_idx)
    kb = dec.code.kbch
    llrs, want = [], []
    for _ in range(nframes):
        bb = rng.integers(0, 2, kb).astype(np.uint8)
        cw = m.fec_frame(bb, rate_idx)
        llr = ld.awgn_llr(cw, esn0, rng)
        hard, it, conv = dec.decode(llr, max_it=max_it)
        want.append((np.packbits(hard[:dec.code.K]), conv, it, np.array_equal(hard, cw),
                     np.packbits((llr[:dec.code.K] < 0).astype(np.uint8))))
        llrs.append(llr)
    return np.array(llrs), want


def write_ldpc_in(path, frames):
    """LLR words "ii qq first rate maxit tag" of frames [(llr, rate_idx, max_it, tag)]."""
    with open(path, "w") as f:
        for llr, rate_idx, max_it, tag in frames:
            u = llr.astype(np.int64) & 0xFF
            f.write("".join(f"{u[2 * k]:02x} {u[2 * k + 1]:02x} {int(k == 0)} {rate_idx} {max_it} {tag}\n"
                            for k in range(32400)))


def check_ldpc_out(got, want):
    """Decoder output rows (bytes, last, ok, iters, chan, rate, tag) = the
    model's frames [(data, conv, it, ok, chan, rate_idx, tag)], in order."""
    pos = 0
    for k, (data, conv, it, ok, chan, rate_idx, tag) in enumerate(want):
        nb = len(data)
        g = got[pos:pos + nb]
        pos += nb
        assert len(g) == nb, f"frame {k}: {len(g)} of {nb} bytes"
        assert np.array_equal(g[:, 0], data), f"frame {k}: data differs"
        assert np.array_equal(g[:, 4], chan), f"frame {k}: channel decisions differ"
        assert g[-1, 1] == 1 and g[:-1, 1].sum() == 0
        assert g[-1, 2] == int(conv) and g[-1, 3] == it, (k, g[-1], conv, it)
        assert np.all(g[:, 5] == rate_idx) and np.all(g[:, 6] == tag), (k, g[-1], rate_idx, tag)
    assert pos == len(got), (pos, len(got))


@pytest.mark.parametrize("name,rate_idx,esn0,nframes,max_it,stall,gap", [
    ("r1_2", 3, 1.2, 3, 30, 10, 0),
    ("r1_4", 0, -2.2, 2, 30, 0, 0),
    ("r9_10", 10, 6.5, 2, 30, 30, 1),
    ("r3_5_maxit", 4, 2.2, 2, 4, 0, 0),
])
def test_ldpc(ldpc_exe, name, rate_idx, esn0, nframes, max_it, stall, gap):
    """RTL decoder output (bytes, converged flag, iterations) equals the model."""
    os.makedirs(WORK, exist_ok=True)
    llrs, want = ldpc_case(rate_idx, esn0, nframes, seed=rate_idx + 7, max_it=max_it)
    fin = os.path.join(WORK, f"l_in_{name}.hex")
    fout = os.path.join(WORK, f"l_out_{name}.hex")
    write_ldpc_in(fin, [(llr, rate_idx, max_it, 0x40 + k) for k, llr in enumerate(llrs)])
    log = rxsim.run(ldpc_exe, stall=stall, gap=gap, **{"in": fin, "out": fout})
    print(log, [(w[1], w[2], w[3]) for w in want])
    got = rxsim.read_hex_cols(fout, (8, 1, 1, 6, 8, 4, 8))
    check_ldpc_out(got, [w + (rate_idx, 0x40 + k) for k, w in enumerate(want)])


def test_ldpc_mixed(ldpc_exe):
    """Per-frame rate, iteration limit and tag (RX2 step 2): consecutive
    frames of different rates and limits through the two APP buffers, each
    equal to the model at its own rate, with its tag."""
    os.makedirs(WORK, exist_ok=True)
    cases = [(3, 2.0, 30), (10, 7.5, 30), (0, -1.5, 30), (6, 4.5, 3), (6, 5.0, 30), (1, -0.5, 30), (3, 1.2, 6)]
    frames, want = [], []
    for k, (rate_idx, esn0, max_it) in enumerate(cases):
        llrs, w = ldpc_case(rate_idx, esn0, 1, seed=100 + k, max_it=max_it)
        tag = (k * 37 + 5) & 0xFF
        frames.append((llrs[0], rate_idx, max_it, tag))
        want.append(w[0] + (rate_idx, tag))
    fin = os.path.join(WORK, "l_in_mixed.hex")
    fout = os.path.join(WORK, "l_out_mixed.hex")
    write_ldpc_in(fin, frames)
    log = rxsim.run(ldpc_exe, stall=10, gap=1, **{"in": fin, "out": fout})
    print(log, [(w[1], w[2]) for w in want])
    got = rxsim.read_hex_cols(fout, (8, 1, 1, 6, 8, 4, 8))
    check_ldpc_out(got, want)


def test_ldpc_itcap(ldpc_exe):
    """it_cap (the LLR queue's shared-budget cap): a frame that would take
    more iterations stops at it_cap, equal to the model with that limit."""
    os.makedirs(WORK, exist_ok=True)
    llrs, want = ldpc_case(0, -2.3, 2, seed=11, max_it=5)
    fin = os.path.join(WORK, "l_in_itcap.hex")
    fout = os.path.join(WORK, "l_out_itcap.hex")
    write_ldpc_in(fin, [(llr, 0, 30, 1) for llr in llrs])
    log = rxsim.run(ldpc_exe, itcap=5, **{"in": fin, "out": fout})
    print(log, [(w[1], w[2]) for w in want])
    assert any(not w[1] for w in want), "a frame must hit the cap"
    got = rxsim.read_hex_cols(fout, (8, 1, 1, 6, 8, 4, 8))
    check_ldpc_out(got, [w + (0, 1) for w in want])


# ---------------------------------------------------------------------------
# DDC
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def ddc_exe():
    return rxsim.build("tb_rx_ddc", ["dvbs2_rx_rot.sv", "dvbs2_rx_ddc.sv", os.path.join(HERE, "tb_rx_ddc.sv")])


def ddc_case(R, D, gain_log2=4):
    """Coefficients/shifts of the DDC for the split R x D (ddc_config() for
    the total decimation when it chooses this split, else the same design
    for this R and D)."""
    if R <= 1:
        return dict(R=0, D=D, cic_shift=0, coefs=fx.ddc_coefs(D) if D > 1 else np.array([65536 - 1, 0]),
                    shift=16)
    c = fx.ddc_config(R * D, gain_log2) if fx.ddc_split(R * D) == (R, D) else None
    if c is None:
        cs = fx.cic_shift_for(R)
        scale = float(1 << gain_log2) * (1 << cs) / R ** 3
        h = fx.ddc_cic_fir(R, D)
        sh = int(np.floor(np.log2(((1 << 17) - 1) / (np.abs(h).max() * scale))))
        c = dict(R=R, D=D, cic_shift=cs, coefs=np.rint(h * scale * 2.0 ** sh).astype(np.int64), shift=sh)
    return c


# (name, R, D, finc, input period (0: as fast as s_ready allows), overrides)
DDC_CASES = [
    ("fir3", 1, 3, 0, 8, {}),
    ("fir3_nco", 1, 3, 123456789, 6, {}),
    ("fir18", 1, 18, -98765432, 8, {}),
    ("fir1", 1, 1, 55555555, 8, {}),
    ("fir1_fast", 1, 1, 0, 0, {}),
    ("cic6x30", 6, 30, 0, 8, {}),                         # 33.33 kS/s at 12 MS/s
    ("cic3x30_nco", 3, 30, -71582788, 8, {}),             # 66.67 kS/s, NCO at -0.2 MHz
    ("cic2x9_nco", 2, 9, 98765432, 5, {}),
    ("cic5x7", 5, 7, 0, 4, {}),
    ("cic32x32_nco", 32, 32, 12345678, 3, {}),            # 512 taps, R^3 = 2^15
    ("cic32x4_fast", 32, 4, -5555555, 0, {}),
    ("cic8x4_sat", 8, 4, 0, 4, dict(cic_shift=4, shift_add=-3)),   # CIC and FIR outputs saturate
]


@pytest.mark.parametrize("name,R,D,finc,period,ov", DDC_CASES, ids=[c[0] for c in DDC_CASES])
def test_ddc(ddc_exe, name, R, D, finc, period, ov):
    os.makedirs(WORK, exist_ok=True)
    rng = np.random.default_rng(R * 100 + D)
    nout = 60 if R > 1 else 2000
    n = max(6000, (nout + 2) * R * D)
    t = np.arange(n)
    x = rng.standard_normal((n, 2)) * 3000
    x[:, 0] += 6000 * np.cos(2 * np.pi * 0.0013 * t)               # a tone near DC (CIC passband)
    x[:, 1] += 6000 * np.sin(2 * np.pi * 0.0013 * t)
    x = np.clip(np.round(x), -32768, 32767).astype(np.int64)
    x[100] = [32767, -32768]                      # saturation corner
    x[n // 2:n // 2 + 3 * R * D] = [[-32768, 32767]] if ov else [[32767, 32767]]   # full-scale run
    c = ddc_case(R, D)
    cs = ov.get("cic_shift", c["cic_shift"])
    shift = c["shift"] + ov.get("shift_add", 0)
    coefs = c["coefs"]
    want = fx.ddc(x, coefs, D, finc & 0xFFFFFFFF, shift, R, cs)
    if ov:
        assert (np.abs(want) == 32768).any() or (want == 32767).any()
        assert (np.abs(fx.cic(fx.mix(x, finc & 0xFFFFFFFF) if finc else x, R, cs)) >= (1 << 17) - 1).any()
    cfile = os.path.join(WORK, f"ddc_coef_{name}.hex")
    with open(cfile, "w") as f:
        for v in coefs:
            f.write(f"{int(v) & 0x3FFFF:05x}\n")
    fin = os.path.join(WORK, f"ddc_in_{name}.hex")
    fout = os.path.join(WORK, f"ddc_out_{name}.hex")
    rxsim.write_iq(fin, x)
    log = rxsim.run(ddc_exe, coef=cfile, dec=D, ntaps=len(coefs), shift=shift, finc=finc & 0xFFFFFFFF,
                    cicr=R, cicshift=cs, period=max(period, 1), burst=int(period == 0),
                    **{"in": fin, "out": fout})
    print(log)
    assert "dropped 0" in log
    got = rxsim.read_hex_cols(fout, (-16, -16))
    assert len(got) == len(want)
    bad = np.nonzero(np.any(got != want, axis=1))[0]
    assert len(bad) == 0, f"first mismatch {bad[0]}: {got[bad[0]]} vs {want[bad[0]]}"


def test_ddc_sw_config(tmp_path):
    """sw/dvbs2_rx.c chooses the same DDC split, shifts and coefficients as
    model/rxfixed.py ddc_config() for every decimation (and none for the
    unsupported ones), and the default (no -d) decimation for low rates."""
    import subprocess
    sw = os.path.join(HERE, "..", "..", "sw")
    exe = str(tmp_path / "dvbs2-rx")
    subprocess.run(["gcc", "-O2", "-Wall", "-Wextra", "-Werror", "-std=gnu11", "-o", exe,
                    os.path.join(sw, "dvbs2_rx.c"), os.path.join(sw, "bbdeframe.c"),
                    os.path.join(sw, "bbframe.c"), "-lm"], check=True)

    def dump(*args):
        r = subprocess.run([exe, "--dump-coefs"] + [str(a) for a in args], capture_output=True, text=True)
        if r.returncode:
            return None
        out = r.stdout.split("\n")
        return [int(v) for v in out[32].split()], [int(v) for v in out[34].split()]

    for dec in range(1, fx.CIC_RMAX * fx.DDC_FIR_ONLY_MAX + 1):
        got = dump("-s", repr(6e6 / dec), "-F", "12e6", "-S", 4 + dec % 5 - 2)
        if fx.ddc_split(dec) is None:
            assert got is None, dec
            continue
        c = fx.ddc_config(dec, 4 + dec % 5 - 2)
        assert got is not None, dec
        assert got[1] == [c["R"], c["D"], c["cic_shift"], c["shift"]], dec
        assert got[0] == [int(v) for v in c["coefs"]], dec
    # default: 12 MS/s (symbol rates within 100 ppm); -F auto: ADC >= 2.1 MS/s,
    # the next supported decimation; -d alone: ADC rate 2 x sr x dec
    for sr, args, dec in [(33333, (), 180), (66666, (), 90), (500e3, (), 12), (2e6, (), 3),
                          (33333.0, ("-F", "auto"), 32), (25e3, ("-F", "0"), 42), (12e3, ("-F", "auto"), 88),
                          (66.67e3, ("-F", "auto"), 16), (1e6, ("-d", 3), 3)]:
        got = dump("-s", sr, *args)
        assert got[1][0] * got[1][1] == dec and fx.ddc_split(dec), (sr, args, got[1])
    assert dump("-s", 123456) is None and dump("-s", 123456, "-F", "auto") is not None


# ---------------------------------------------------------------------------
# block-design wrapper: ADC samples -> DDC -> receiver -> DMA packets
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def rx_axi_exe():
    return rxsim.build("tb_rx_axi", RX_SOURCES + ["dvbs2_rx_ddc.sv", "dvbs2_rx_axi.sv",
                                                  os.path.join(HERE, "tb_rx_axi.sv")])


@pytest.mark.parametrize("llrq", [0, 1])
def test_rx_axi(rx_axi_exe, llrq):
    """ADC-rate samples (6 per symbol, carrier offset partly removed by the
    DDC NCO) through the register-configured wrapper: every DMA packet
    carries a transmitted BBFRAME, equal to the model chain. llrq = 1: the
    same through the DDR LLR queue (LLRQ_CTRL, memory model with stalls and
    latency), the LLRQ registers counting the frames."""
    import bchdec
    import grref
    import ldpcdec as ld
    import rxmodel as rf
    import txmodel as m
    os.makedirs(WORK, exist_ok=True)
    rate_idx, pilots, dec, nframes = 3, True, 3, 3
    kb = m.LDPC[m.RATES[rate_idx]]["kbch"]
    rng = np.random.default_rng(4)
    ts = rng.integers(0, 256, ((nframes + 1) * kb // 1504 + 3, 188), dtype=np.uint8)
    ts[:, 0] = 0x47
    bb = grref.bbheader(ts.ravel(), m.RATES[rate_idx])[: nframes * kb].reshape(nframes, kb)
    frames = [m.dummy_plframe_signs()] * 8 + \
             [m.plframe_signs(m.fec_frame(b, rate_idx), rate_idx + 1, pilots) for b in bb] + \
             [m.dummy_plframe_signs()] * 2
    c = m.rrc_coefs(8, 0.35, 16)
    tx = m.rrc_interp(m.signs_to_iq(np.concatenate(frames)), c, 8, 16)
    y = rf.channel(tx, 8, 2 * dec, 0.006, 40.0, 0.2, 5.0, 4)       # 0.006 cycles/symbol
    y *= 300 / np.sqrt(np.mean(np.abs(y) ** 2) / 2)
    x = np.clip(np.round(np.stack([y.real, y.imag], 1)), -2048, 2047).astype(np.int64)
    finc = (-round(0.004 / (2 * dec) * 2 ** 32)) & 0xFFFFFFFF      # remove 0.004 of it
    dcoef = fx.ddc_coefs(dec)
    shift = 12                                                       # gain 16
    x2 = fx.ddc(x, dcoef, dec, finc, shift)
    snap = []                                  # the carrier's first 1024 data symbols
    llr0 = fx.Carrier._llr

    def llr_rec(self, buf, y):
        if len(snap) < 1024:
            snap.append((int(y[0]), int(y[1])))
        llr0(self, buf, y)
    fx.Carrier._llr = llr_rec
    try:
        r = fx.receive(x2, rate_idx + 1, pilots)
    finally:
        fx.Carrier._llr = llr0
    dm = ld.Decoder(rate_idx)
    want = []
    raw_err = raw_bits = bch_bits = 0
    for f in r["frames"]:
        hard, it, conv = dm.decode(f)
        K = dm.code.K
        out, ok, L = bchdec.decode(hard[:K], rate_idx)
        want.append((np.packbits(out), it, conv, ok, L))
        if ok:
            raw_err += int(np.sum((f[:K] < 0) != hard[:K]))
            raw_bits += K
            bch_bits += L
    fddc = os.path.join(WORK, "axi_ddc.hex")
    with open(fddc, "w") as f:
        for v in dcoef:
            f.write(f"{int(v) & 0x3FFFF:05x}\n")
    frrc = os.path.join(WORK, "axi_rrc.hex")
    write_coefs(frrc, fx.timing_coefs(0.35))
    fin = os.path.join(WORK, "axi_in.hex")
    fout = os.path.join(WORK, "axi_out.hex")
    fregs = os.path.join(WORK, "axi_regs.hex")
    rxsim.write_iq(fin, x)
    kp, ki = fx.timing_gains()
    xa, xb = m.gold_init(0)
    log = rxsim.run(rx_axi_exe, ddccoef=fddc, rrccoef=frrc, regs=fregs, adcdiv=4, stall=20,
                    modcod=rate_idx + 1, pilots=int(pilots), xa=xa, xb=xb, finc=finc, dec=dec,
                    ntaps=len(dcoef), shift=shift, tkp=kp, tki=ki, thresh=fx.fs_threshold(),
                    pkp=r["carrier"].kp_p, pki=r["carrier"].ki_p, maxit=30, llrq=llrq, axistall=30, axilat=200,
                    **{"in": fin, "out": fout})
    print(log)
    lines = [l.split() for l in open(fregs)]
    regs = {int(a, 16): int(v, 16) for a, v in lines if a[0] not in "sw"}
    snap_got = [int(v, 16) for a, v in lines if a.startswith("s")]
    print({hex(k): v for k, v in regs.items()})
    assert regs[0x50] == 0 and regs[0x54] == 0 and regs[0x8C] == 0     # nothing lost
    assert regs[0] == 0x0001_0500 and regs[0xE0] == 0x3E80_0000 and regs[0xE4] == 0x000A3F10
    assert regs[0xDC] == 3 * llrq and regs[0xEC] == 0 and regs[0xF0] == 0
    assert regs[0xE8] >> 16 == llrq * len(want) and regs[0xE8] & 0x3F == 0      # LLRQ_STAT
    check_pls_regs(regs, lines, r["pls"], rate_idx + 1, pilots)
    # error counters and the constellation snapshot
    print("raw BER", raw_err / max(raw_bits, 1), "BCH bits", bch_bits)
    assert (regs[0xAC], regs[0xB0], regs[0xB4]) == (raw_err, raw_bits, bch_bits)
    assert regs[0xA0] & 3 == 2                                          # done
    sg = [((v & 0xFFFF) - ((v & 0x8000) << 1), (v >> 16) - ((v >> 31) << 16)) for v in snap_got]
    assert sg == snap, "snapshot differs"
    words = [l.split() for l in open(fout)]
    data = np.array([int(w, 16) for w, _ in words], dtype=np.uint64)
    last = np.array([int(l) for _, l in words])
    assert len(data) == 1024 * len(want) and np.array_equal(np.nonzero(last)[0], np.arange(1023, len(data), 1024))
    for k, (wb, it, conv, ok, L) in enumerate(want):
        pkt = data[k * 1024:(k + 1) * 1024]
        h = int(pkt[0])
        assert h >> 56 == 0xB5 and h & 0xFFFF == kb // 8 and (h >> 16) & 0xFFFF == k
        assert (h >> 32) & 0x1F == rate_idx + 1 and (h >> 37) & 1 == int(pilots) and (h >> 55) & 1 == 0
        assert (h >> 40) & 0x3F == it and (h >> 48) & 1 == int(conv)
        assert (h >> 49) & 1 == int(ok) and (h >> 50) & 0x1F == L
        body = pkt[1:].view(np.uint8)[: kb // 8]                      # little endian: byte 0 first
        assert np.array_equal(body, wb), f"packet {k} body"
        assert np.array_equal(np.unpackbits(body), bb[k]), f"packet {k} is not the transmitted BBFRAME"
        nw = 1 + (kb // 8 + 7) // 8
        assert not pkt[nw:].any()


def pls_word(d, n):
    """PLS_LAST / PLS_DATA register value of decode d (None: no decode), count n."""
    if d is None:
        return n << 16
    return (n << 16) | (1 << 15) | (int(d["grid"]) << 14) | (int(d["carrier_hyp"]) << 13) | \
        (int(d["conf"]) << 12) | (d["short"] << 9) | (d["pilots"] << 8) | d["modcod"]


def check_pls_regs(regs, lines, pls, modcod, pilots):
    """The PLS registers of tb_rx_axi equal the model decoder's status, every
    confident decode is the transmitted code or a dummy, W1C works."""
    st = pls.status()
    last = st["last"]
    print("PLS", [(d["code"], d["conf"], d["carrier_hyp"], d["grid"]) for d in pls.results])
    assert regs[0x00] == 0x00010500
    assert regs[0xB8] == pls_word(last, st["count"])
    assert regs[0xBC] == pls_word(st["data"], st["data_count"])
    assert regs[0xC0] == (last["freq"] & 0xFFFF) << 16
    assert (regs[0xC4], regs[0xC8]) == (last["best"], last["second"])
    assert regs[0xCC] == (st["diff_pilots"] << 16) | st["diff_modcod"]
    assert regs[0xD0] == st["seen"]
    assert regs[0xD4] == (pls.step_ << 16) | pls.H
    assert regs[0xD8] == 0                                # no header skipped
    w = {a: int(v, 16) for a, v in lines if a[0] == "w"}
    assert (w["w0"], w["w1"]) == (st["seen"] & 1, 0) and st["seen"] & 1
    code = fx.pls_code(modcod, 0, pilots)
    assert all(d["code"] in (0, code) for d in pls.results if d["conf"])
    assert st["data_count"] >= 1 and st["data"]["code"] == code and st["diff_modcod"] == st["diff_pilots"] == 0


def lowrate_signal(dec, tone, nframes=2, lead=8, esn0=4.0, seed=7, rms=400.0, resid=0.003, ppm=40.0):
    """QPSK 1/2 pilots at 12 MS/s / (2 dec) x (1 + ppm 1e-6) symbols/s,
    2 dec samples/symbol: the signal 1.5 MHz (tone) or -0.8 MHz off the LO
    plus `resid` cycles/symbol, AWGN over the whole ADC band (Es/N0 in the
    signal band), 12-bit samples. tone: a CW 30 dB above the signal in the
    second alias band of the 2-samples/symbol output (2.3 x 2 Rs above
    it). Returns (x, BBFRAMEs, NCO increment that removes the offset)."""
    import grref
    import rxmodel as rf
    import txmodel as m
    rate_idx, pilots = 3, True
    kb = m.LDPC[m.RATES[rate_idx]]["kbch"]
    rng = np.random.default_rng(seed)
    ts = rng.integers(0, 256, ((nframes + 1) * kb // 1504 + 3, 188), dtype=np.uint8)
    ts[:, 0] = 0x47
    bb = grref.bbheader(ts.ravel(), m.RATES[rate_idx])[: nframes * kb].reshape(nframes, kb)
    frames = [m.dummy_plframe_signs()] * lead + \
             [m.plframe_signs(m.fec_frame(b, rate_idx), rate_idx + 1, pilots) for b in bb] + \
             [m.dummy_plframe_signs()] * 2
    tx = m.rrc_interp(m.signs_to_iq(np.concatenate(frames)), m.rrc_coefs(8, 0.35, 16), 8, 16)
    fs = 12e6
    Rs, sps = fs / (2 * dec), 2 * dec
    off = 1.5e6 if tone else -0.8e6
    y = rf.channel(tx, 8, sps, off / Rs + resid, ppm, 0.2, 300.0, seed)       # no noise yet
    ps = np.mean(np.abs(y) ** 2)
    n = len(y)
    y += np.sqrt(ps * sps / 10 ** (esn0 / 10) / 2) * (rng.standard_normal(n) + 1j * rng.standard_normal(n))
    if tone:
        y += np.sqrt(1000 * ps) * np.exp(2j * np.pi * (off + 2.3 * 2 * Rs) / fs * np.arange(n))
    y *= rms / np.sqrt(np.mean(np.abs(y) ** 2) / 2)
    x = np.clip(np.round(np.stack([y.real, y.imag], 1)), -2048, 2047).astype(np.int64)
    return x, bb, (-round(off / fs * 2 ** 32)) & 0xFFFFFFFF


# name, total decimation (12 MS/s / (2 x symbol rate)), testbench ADC clocks per sample
# (48 MHz ADC clock: the simulation runs the ADC 2-4x faster than 12 MS/s), CW interferer
LOWRATE_CASES = [("66k", 90, 2, False), ("33k", 180, 1, True)]


@pytest.mark.parametrize("name,dec,adcdiv,tone", LOWRATE_CASES, ids=[c[0] for c in LOWRATE_CASES])
def test_rx_axi_lowrate(rx_axi_exe, name, dec, adcdiv, tone):
    """66.67 and 33.33 kS/s (decimation 90 = 3 x 30 and 180 = 6 x 30: CIC +
    FIR, configured as dvbs2-rx does it, ddc_config()) through the wrapper,
    the offset from the LO removed by the DDC NCO: every DMA packet carries a
    transmitted BBFRAME, equal to the model chain."""
    x, bb, finc = lowrate_signal(dec, tone)
    lowrate_axi(rx_axi_exe, name, dec, adcdiv, x, bb, finc)


def lowrate_axi(rx_axi_exe, name, dec, adcdiv, x, bb, finc, integ0=0):
    """ADC samples x (QPSK 1/2 pilots) through the wrapper configured as
    dvbs2-rx does it (ddc_config(dec), TIMING_INTEG preset integ0): every DMA
    packet equals the model chain and carries a transmitted BBFRAME."""
    import bchdec
    import ldpcdec as ld
    import txmodel as m
    os.makedirs(WORK, exist_ok=True)
    rate_idx, pilots = 3, True
    kb = m.LDPC[m.RATES[rate_idx]]["kbch"]
    cfg = fx.ddc_config(dec)
    assert cfg["R"] > 1
    x2 = fx.ddc(x, cfg["coefs"], cfg["D"], finc, cfg["shift"], cfg["R"], cfg["cic_shift"])
    r = fx.receive(x2, rate_idx + 1, pilots, integ0=integ0)
    dm = ld.Decoder(rate_idx)
    want = []
    for f in r["frames"]:
        hard, it, conv = dm.decode(f)
        out, ok, L = bchdec.decode(hard[:dm.code.K], rate_idx)
        want.append((np.packbits(out), it, conv, ok, L))
    assert len(want) == len(bb)
    fddc = os.path.join(WORK, f"lr_ddc_{name}.hex")
    with open(fddc, "w") as f:
        for v in cfg["coefs"]:
            f.write(f"{int(v) & 0x3FFFF:05x}\n")
    frrc = os.path.join(WORK, f"lr_rrc_{name}.hex")
    write_coefs(frrc, fx.timing_coefs(0.35))
    fin = os.path.join(WORK, f"lr_in_{name}.hex")
    fout = os.path.join(WORK, f"lr_out_{name}.hex")
    fregs = os.path.join(WORK, f"lr_regs_{name}.hex")
    rxsim.write_iq(fin, x)
    kp, ki = fx.timing_gains()
    xa, xb = m.gold_init(0)
    log = rxsim.run(rx_axi_exe, ddccoef=fddc, rrccoef=frrc, regs=fregs, adcdiv=adcdiv, stall=20,
                    modcod=rate_idx + 1, pilots=int(pilots), xa=xa, xb=xb, finc=finc, dec=cfg["D"],
                    ntaps=len(cfg["coefs"]), shift=cfg["shift"], cicr=cfg["R"], cicshift=cfg["cic_shift"],
                    tkp=kp, tki=ki, tinteg0=integ0, thresh=fx.fs_threshold(),
                    pkp=r["carrier"].kp_p, pki=r["carrier"].ki_p, maxit=30, **{"in": fin, "out": fout})
    print(log)
    os.remove(fin)
    lines = [l.split() for l in open(fregs)]
    regs = {int(a, 16): int(v, 16) for a, v in lines if a[0] not in "sw"}
    assert regs[0x50] == 0 and regs[0x54] == 0 and regs[0x8C] == 0     # nothing lost
    check_pls_regs(regs, lines, r["pls"], rate_idx + 1, pilots)
    words = [l.split() for l in open(fout)]
    data = np.array([int(w, 16) for w, _ in words], dtype=np.uint64)
    assert len(data) == 1024 * len(want)
    for k, (wb, it, conv, ok, L) in enumerate(want):
        pkt = data[k * 1024:(k + 1) * 1024]
        h = int(pkt[0])
        assert h >> 56 == 0xB5 and h & 0xFFFF == kb // 8 and (h >> 16) & 0xFFFF == k
        assert (h >> 40) & 0x3F == it and (h >> 48) & 1 == int(conv)
        assert (h >> 49) & 1 == int(ok) and (h >> 50) & 0x1F == L
        body = pkt[1:].view(np.uint8)[: kb // 8]
        assert np.array_equal(body, wb), f"packet {k} body"
        assert np.array_equal(np.unpackbits(body), bb[k]), f"packet {k} is not the transmitted BBFRAME"


# name, symbol-rate error of the transmitter vs 12 MS/s / 36, TIMING_INTEG preset
SR_OFFSET_CASES = [("333000_preset", -1000.0, True), ("333667_pullin", 1000.0, False)]


@pytest.mark.parametrize("name,ppm,preset", SR_OFFSET_CASES, ids=[c[0] for c in SR_OFFSET_CASES])
def test_rx_axi_sr_offset(rx_axi_exe, name, ppm, preset):
    """Decimation 18 (333333 S/s nominal) with the transmitter 1000 ppm off:
    333000 S/s, the integer-kS rate QO-100 stations use, with the timing
    integrator preset from the known offset (TIMING_INTEG write,
    timing_integ_preset()), and +1000 ppm pulled in by the loop from 0
    (~20000 symbols: 12 dummy frames first). Every DMA packet carries a
    transmitted BBFRAME, equal to the model chain."""
    dec = 18
    x, bb, finc = lowrate_signal(dec, False, ppm=ppm, lead=6 if preset else 12)
    nominal = 12e6 / (2 * dec)
    integ0 = fx.timing_integ_preset(nominal * (1 + ppm * 1e-6), nominal) if preset else 0
    if preset:
        assert integ0 == fx.timing_integ_preset(333000.0, nominal) == 33588
    lowrate_axi(rx_axi_exe, f"sr_{name}", dec, 4, x, bb, finc, integ0)


# ---------------------------------------------------------------------------
# BCH decoder
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def bch_exe():
    return rxsim.build("tb_bch_dec", ["gen/dvbs2_bch_pkg.sv", "gen/dvbs2_ldpc_dec_pkg.sv", "dvbs2_bch_dec.sv",
                                      os.path.join(HERE, "tb_bch_dec.sv")])


@pytest.mark.parametrize("rate_idx,stall", [(0, 0), (5, 20), (10, 5)])
def test_bch(bch_exe, rate_idx, stall):
    """RTL BCH decoder = model: corrected, BB-descrambled BBFRAME, ok, deg
    Lambda, for 0 .. t + 6 errors (also in the parity part)."""
    import bchdec
    import txmodel as m
    os.makedirs(WORK, exist_ok=True)
    kbch, n, t = bchdec.params(rate_idx)
    rng = np.random.default_rng(rate_idx + 3)
    nerrs = [0, 1, 3, t, t + 1, t + 6]
    fin = os.path.join(WORK, f"bch_in{rate_idx}.hex")
    fout = os.path.join(WORK, f"bch_out{rate_idx}.hex")
    want = []
    with open(fin, "w") as f:
        for i, ne in enumerate(nerrs):
            bb = rng.integers(0, 2, kbch).astype(np.uint8)
            r = m.bch_encode(m.bb_scramble(bb), t)
            pos = rng.choice(n, ne, replace=False)
            if i == 1:
                pos = np.array([n - 1])                     # a parity bit
            r[pos] ^= 1
            out, ok, L = bchdec.decode(r, rate_idx)
            chan = r.copy()
            chan[rng.choice(n, 50 * i, replace=False)] ^= 1             # "channel" decisions
            want.append((np.packbits(out), ok, L, i % 2, 7 + i, ne <= t and np.array_equal(out, bb),
                         int(np.sum(chan != r))))
            for k, (v, c) in enumerate(zip(np.packbits(r), np.packbits(chan))):
                f.write(f"{v:02x} {int(k == n // 8 - 1)} {i % 2} {7 + i} {c:02x} {rate_idx} {i + 3}\n")
    log = rxsim.run(bch_exe, stall=stall, **{"in": fin, "out": fout})
    print(log, [(w[1], w[2], w[5]) for w in want])
    got = rxsim.read_hex_cols(fout, (8, 1, 1, 1, 6, 5, 16, 4, 8))
    nb = kbch // 8
    assert len(got) == nb * len(want)
    for i, (data, ok, L, lok, it, right, raw) in enumerate(want):
        g = got[i * nb:(i + 1) * nb]
        assert np.array_equal(g[:, 0], data), f"frame {i}: data differs"
        assert g[-1, 1] == 1 and g[:-1, 1].sum() == 0
        assert (g[-1, 2], g[-1, 3], g[-1, 4], g[-1, 5]) == (int(ok), lok, it, L), (i, g[-1], ok, L)
        assert right == (nerrs[i] <= t)                    # the model corrects <= t
        assert g[-1, 6] == raw
        assert np.all(g[:, 7] == rate_idx) and np.all(g[:, 8] == i + 3)


def test_bch_mixed(bch_exe):
    """Per-frame rate and tag (RX2 step 2): frames of different rates (and
    t = 12, 10, 8) back to back, each equal to the model at its rate."""
    import bchdec
    import txmodel as m
    os.makedirs(WORK, exist_ok=True)
    rng = np.random.default_rng(21)
    fin = os.path.join(WORK, "bch_in_mixed.hex")
    fout = os.path.join(WORK, "bch_out_mixed.hex")
    want = []
    with open(fin, "w") as f:
        for i, rate_idx in enumerate([10, 0, 5, 9, 3, 3, 7, 1]):
            kbch, n, t = bchdec.params(rate_idx)
            bb = rng.integers(0, 2, kbch).astype(np.uint8)
            r = m.bch_encode(m.bb_scramble(bb), t)
            ne = [0, t, t + 2, 1][i % 4]
            r[rng.choice(n, ne, replace=False)] ^= 1
            out, ok, L = bchdec.decode(r, rate_idx)
            want.append((np.packbits(out), ok, L, rate_idx, 0x80 + i))
            for k, v in enumerate(np.packbits(r)):
                f.write(f"{v:02x} {int(k == n // 8 - 1)} 1 {i} {v:02x} {rate_idx} {0x80 + i}\n")
    log = rxsim.run(bch_exe, stall=15, **{"in": fin, "out": fout})
    print(log, [(w[1], w[2]) for w in want])
    got = rxsim.read_hex_cols(fout, (8, 1, 1, 1, 6, 5, 16, 4, 8))
    pos = 0
    for i, (data, ok, L, rate_idx, tag) in enumerate(want):
        g = got[pos:pos + len(data)]
        pos += len(data)
        assert np.array_equal(g[:, 0], data), f"frame {i}: data differs"
        assert g[-1, 1] == 1 and g[:-1, 1].sum() == 0
        assert (g[-1, 2], g[-1, 5]) == (int(ok), L), (i, g[-1], ok, L)
        assert np.all(g[:, 7] == rate_idx) and np.all(g[:, 8] == tag)
    assert pos == len(got)


def test_rx_axi_hold(rx_axi_exe):
    """No DMA for the first frames (both packet slots fill, later frames are
    dropped), then the DMA starts: the two held frames and every later
    frame come out, in order (the board starts this way)."""
    import grref
    import rxmodel as rf
    import txmodel as m
    os.makedirs(WORK, exist_ok=True)
    rate_idx, pilots, dec, nframes = 3, True, 3, 8
    kb = m.LDPC[m.RATES[rate_idx]]["kbch"]
    rng = np.random.default_rng(5)
    ts = rng.integers(0, 256, ((nframes + 1) * kb // 1504 + 3, 188), dtype=np.uint8)
    ts[:, 0] = 0x47
    bb = grref.bbheader(ts.ravel(), m.RATES[rate_idx])[: nframes * kb].reshape(nframes, kb)
    frames = [m.dummy_plframe_signs()] * 8 + \
             [m.plframe_signs(m.fec_frame(b, rate_idx), rate_idx + 1, pilots) for b in bb] + \
             [m.dummy_plframe_signs()] * 2
    c = m.rrc_coefs(8, 0.35, 16)
    tx = m.rrc_interp(m.signs_to_iq(np.concatenate(frames)), c, 8, 16)
    y = rf.channel(tx, 8, 2 * dec, 0.002, 40.0, 0.2, 8.0, 5)
    y *= 300 / np.sqrt(np.mean(np.abs(y) ** 2) / 2)
    x = np.clip(np.round(np.stack([y.real, y.imag], 1)), -2048, 2047).astype(np.int64)
    dcoef = fx.ddc_coefs(dec)
    fddc = os.path.join(WORK, "hold_ddc.hex")
    with open(fddc, "w") as f:
        for v in dcoef:
            f.write(f"{int(v) & 0x3FFFF:05x}\n")
    frrc = os.path.join(WORK, "hold_rrc.hex")
    write_coefs(frrc, fx.timing_coefs(0.35))
    fin = os.path.join(WORK, "hold_in.hex")
    fout = os.path.join(WORK, "hold_out.hex")
    fregs = os.path.join(WORK, "hold_regs.hex")
    rxsim.write_iq(fin, x)
    kp, ki = fx.timing_gains()
    xa, xb = m.gold_init(0)
    frame_clk = (32400 + 90 + 792) * 2 * dec * 4                  # ADC samples x adcdiv
    hold = 2 * (8 * 3330 * 2 * dec * 4) + 11 * frame_clk          # ~5 frames decoded
    pkp, pki = fx.pll_gains()
    log = rxsim.run(rx_axi_exe, ddccoef=fddc, rrccoef=frrc, regs=fregs, adcdiv=4, stall=10, hold=hold,
              modcod=rate_idx + 1, pilots=int(pilots), xa=xa, xb=xb, finc=0, dec=dec,
              ntaps=len(dcoef), shift=12, tkp=kp, tki=ki, thresh=fx.fs_threshold(),
              pkp=pkp, pki=pki, maxit=30, **{"in": fin, "out": fout})
    print(log)
    lines = [l.split() for l in open(fregs)]
    regs = {int(a, 16): int(v, 16) for a, v in lines if a[0] not in "sw"}
    words = [int(l.split()[0], 16) for l in open(fout)]
    assert len(words) % 1024 == 0
    pk = [words[i:i + 1024] for i in range(0, len(words), 1024)]
    hdr = [p[0] for p in pk]
    assert all(h >> 56 == 0xB5 for h in hdr)
    cnts = [(h >> 16) & 0xFFFF for h in hdr]
    got = [np.frombuffer(np.array(p[1:], dtype="<u8").tobytes(), np.uint8)[: kb // 8] for p in pk]
    sent = [next((k for k, b in enumerate(bb) if np.array_equal(np.packbits(b), g)), -1) for g in got]
    print("packets", len(pk), "counters", cnts, "frames", sent, "dropped", regs[0x8C], "sent", regs[0x90])
    assert regs[0x8C] >= 1, "the hold must make the RX drop frames"
    assert cnts == list(range(len(pk))), "packet counter must run on"
    assert len(pk) + regs[0x8C] == nframes, (len(pk), regs[0x8C])
    assert len(pk) >= 3
    assert sent == sorted(sent) and -1 not in sent, sent


# ---------------------------------------------------------------------------
# PLS decoder
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def pls_exe():
    return rxsim.build("tb_rx_pls", ["gen/dvbs2_tables_pkg.sv", "dvbs2_pl_pkg.sv", "gen/dvbs2_rx_pkg.sv",
                                     "dvbs2_rx_pls.sv", os.path.join(HERE, "tb_rx_pls.sv")])


class _PlsRec(fx.PlsDecoder):
    """PlsDecoder that records its input (the carrier's input stream)."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.rec = []

    def step(self, si, sq, t, fhat, acquired):
        self.rec.append((si, sq, int(t != 0), fhat & 0xFFFFFFFF, int(acquired)))
        super().step(si, sq, t, fhat, acquired)


def pls_run(x, rx_modcod, rx_pilots, H=fx.PLS_H_DEFAULT, step=fx.PLS_STEP_DEFAULT):
    """Model chain with a recording PLS decoder: (receive() result, decoder)."""
    kp, ki = fx.timing_gains()
    sym = fx.Timing(fx.timing_coefs(0.35), kp, ki).run(x)
    fs = fx.FrameSync(rx_modcod, rx_pilots)
    cr = fx.Carrier(rx_modcod, rx_pilots)
    cr.pls = _PlsRec(rx_modcod, rx_pilots, H, step)
    fs.run(sym, cr)
    return cr, cr.pls


def pls_line(d):
    return (d["code"], int(d["conf"]), int(d["carrier_hyp"]), int(d["grid"]), d["best"], d["second"],
            d["freq"] & 0xFFFF)


# name, TX (rate_idx, pilots), RX (modcod, pilots), Es/N0, cfo, dummy lead frames, data frames,
# testbench clocks per symbol, grid H
PLS_CASES = [
    # the coordinator's case: TX QPSK 3/4 pilots, RX told QPSK 1/2 without pilots; the
    # frame sync finds the data header after the dummy frames (then misses: wrong length)
    ("mismatch", (6, True), (4, False), 3.0, 0.004, 3, 3, 40, 32),
    # MODCOD differs, pilots right, no dummy frames, 0.03 cycles/symbol: the carrier
    # acquires a wrong frequency, the grid finds the header
    ("modcod_cfo", (5, False), (4, False), 2.0, -0.03, 0, 4, 40, 32),
    # matched, -2.4 dB: after the acquisition the carrier hypothesis decides alone
    ("lowsnr", (0, True), (1, True), -2.4, 0.004, 8, 2, 40, 32),
    # MODCOD differs at -2 dB: grid decodes
    ("lowsnr_grid", (0, True), (4, True), -2.0, 0.002, 4, 3, 40, 16),
    # symbols faster than the decoder: headers skipped
    ("skip", (6, True), (6, True), 6.0, 0.004, 6, 1, 6, 32),
    # QPSK 1/4 and 2/5 without pilots and without dummy frames, RX told QPSK 1/2: their
    # headers are closer to the dummy header than to the QPSK 1/2 one, so the frame sync's
    # search peak is a "dummy" one, which it now also checks at the data length
    ("lowrate_np", (0, False), (4, False), 1.0, 0.02, 0, 5, 40, 32),
    ("lowrate_np25", (2, False), (4, False), 1.0, 0.01, 0, 5, 40, 32),
    # pilots on, RX not told, no dummy frames: the frame sync cannot find the headers
    # (dvbs2-rx --detect tries the other pilots), the decodes of what it tags are not them
    ("pilots_untold", (0, True), (4, False), 3.0, 0.01, 0, 5, 40, 32),
]


@pytest.mark.parametrize("name,tx,rx,esn0,cfo,lead,nfr,period,H", PLS_CASES, ids=[c[0] for c in PLS_CASES])
def test_pls(pls_exe, name, tx, rx, esn0, cfo, lead, nfr, period, H):
    """PLS decoder RTL = model on the carrier input of the model chain, with
    the configured MODCOD/pilots different from the transmitted ones."""
    os.makedirs(WORK, exist_ok=True)
    x, _ = signal(nframes=nfr, lead=lead, rate_idx=tx[0], pilots=tx[1], esn0=esn0, cfo=cfo, seed=2)
    cr, p = pls_run(x, rx[0], rx[1], H)
    fin = os.path.join(WORK, f"pls_in_{name}.hex")
    fout = os.path.join(WORK, f"pls_out_{name}.hex")
    with open(fin, "w") as f:
        for si, sq, t, fh, a in p.rec:
            f.write(f"{si & 0xFFFF:04x} {sq & 0xFFFF:04x} {t} {fh:08x} {a}\n")
    log = rxsim.run(pls_exe, modcod=rx[0], pilots=int(rx[1]), period=period, h=H, **{"in": fin, "out": fout})
    print(log)
    got = [tuple(int(v, 16) for v in l.split()) for l in open(fout)]
    want = [pls_line(d) for d in p.results]
    print("model", want, "acq", cr.n_acq, "fhat", cr.fhat / 2 ** 32)
    true_code = fx.pls_code(tx[0] + 1, 0, tx[1])
    if name == "skip":
        assert "skipped 0 " not in log + " "
        it = iter(want)
        assert all(any(g == w for w in it) for g in got), "not a subsequence of the model's decodes"
        return
    assert "skipped 0" in log
    assert got == want
    st = p.status()
    dcode = f"{st['data']['code']:02x}" if st["data"] else ".."
    assert re.search(f"count {st['count']} data {st['data_count']} dcode {dcode} dvalid {int(dcode != '..')} "
                     f"diff {st['diff_modcod']} {st['diff_pilots']} seen {st['seen']:08x} skipped 0", log)
    conf = [d for d in p.results if d["conf"]]
    data = [k for k, d in enumerate(p.results) if d["conf"] and d["code"] == true_code]
    if name == "pilots_untold":
        assert not data
        return
    assert all(d["code"] in (0, true_code) for d in conf), [d["code"] for d in conf]
    assert data and data[0] <= (lead if lead else 1), data
    if name == "lowsnr":
        assert any(d["carrier_hyp"] and not d["grid"] for d in p.results)
    if rx != (tx[0] + 1, tx[1]):
        assert st["diff_modcod"] + st["diff_pilots"] >= 1
