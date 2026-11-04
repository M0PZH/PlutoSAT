"""RX RTL blocks vs the fixed-point model (model/rxfixed.py), bit exact.

    cd sim/rx && ../../.venv/bin/python -m pytest -v test_rx.py
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


@pytest.mark.parametrize("stall,rms,ppm", [(0, 1500.0, 60.0), (30, 300.0, -100.0)])
def test_timing(timing_exe, stall, rms, ppm):
    os.makedirs(WORK, exist_ok=True)
    x, _ = signal(nframes=0, lead=6, rms=rms, ppm=ppm)
    coefs = fx.timing_coefs(0.35)
    kp, ki = fx.timing_gains()
    tr = fx.Timing(coefs, kp, ki)
    want = tr.run(x)
    cfile = os.path.join(WORK, "coef.hex")
    with open(cfile, "w") as f:
        for p in range(32):
            for j in range(32):
                f.write(f"{int(coefs[p, j]) & 0x3FFFF if j < fx.NT else 0:05x}\n")
    fin = os.path.join(WORK, f"t_in{stall}.hex")
    fout = os.path.join(WORK, f"t_out{stall}.hex")
    rxsim.write_iq(fin, x)
    log = rxsim.run(timing_exe, coef=cfile, kp=kp, ki=ki, stall=stall, **{"in": fin, "out": fout})
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


@pytest.mark.parametrize("stall,pilots,esn0,drop", [(0, True, 8.0, None), (25, False, 3.0, None),
                                                    (10, True, 8.0, (80000, 80100))])
def test_framesync(framesync_exe, stall, pilots, esn0, drop):
    """drop: remove symbols [a, b) -> shifted headers: flywheel, lost, search, re-lock."""
    os.makedirs(WORK, exist_ok=True)
    x, _ = signal(nframes=10 if drop else 3, lead=3, pilots=pilots, esn0=esn0)
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
    assert (want_tag == 1).sum() >= 2
    if drop:
        kinds = [e[0] for e in fs.events]
        assert "flywheel" in kinds and "lost" in kinds and kinds[-1] == "found"


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
    lead = kw.pop("lead", 3)
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


RX_SOURCES = ["gen/dvbs2_tables_pkg.sv", "dvbs2_pl_pkg.sv", "gen/dvbs2_rx_pkg.sv", "gen/dvbs2_ldpc_dec_pkg.sv",
              "dvbs2_async_fifo.sv", "dvbs2_rx_timing.sv", "dvbs2_rx_framesync.sv", "dvbs2_rx_rot.sv",
              "dvbs2_rx_cordic.sv", "dvbs2_udiv.sv", "dvbs2_rx_carrier.sv", "dvbs2_ldpc_dec.sv",
              "gen/dvbs2_bch_pkg.sv", "dvbs2_bch_dec.sv", "dvbs2_rx.sv"]


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
])
def test_rx_end_to_end(rx_exe, name, kw):
    """Samples -> RTL receiver: the LLRs equal the front-end model, the
    BBFRAMEs equal the LDPC model and the transmitted BBFRAMEs, and
    gr-dvbs2rx's LDPC/BCH decodes the RTL's LLRs to the same frames."""
    import ldpcdec as ld
    import rxmodel as rf
    import txmodel as m
    os.makedirs(WORK, exist_ok=True)
    kw = dict(kw)
    rate_idx, pilots = kw["rate_idx"], kw["pilots"]
    x, bb = signal(nframes=4, lead=8, **kw)
    r = fx.receive(x, rate_idx + 1, pilots)
    want = np.concatenate(r["frames"]).reshape(-1, 2)
    cfile = os.path.join(WORK, "coef.hex")
    write_coefs(cfile, fx.timing_coefs(0.35))
    fin = os.path.join(WORK, f"rx_in_{name}.hex")
    fout = os.path.join(WORK, f"rx_out_{name}.hex")
    fbytes = os.path.join(WORK, f"rx_bytes_{name}.hex")
    rxsim.write_iq(fin, x)
    kp, ki = fx.timing_gains()
    xa, xb = m.gold_init(0)
    log = rxsim.run(rx_exe, coef=cfile, period=22, stall=5, modcod=rate_idx + 1, pilots=int(pilots),
                    thresh=fx.fs_threshold(), xa=xa, xb=xb, tkp=kp, tki=ki,
                    pkp=r["carrier"].kp_p, pki=r["carrier"].ki_p, maxit=30, bytes=fbytes,
                    **{"in": fin, "out": fout})
    print(log)
    assert "overflows 0," in log
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
    got_b = rxsim.read_hex_cols(fbytes, (8, 1, 1, 8, 1, 8, 16))
    nb = kb // 8
    assert len(got_b) == nb * len(llr)
    for k in range(len(llr)):
        hard, it, conv = dec_m.decode(llr[k])
        out, ok, L = bchdec.decode(hard[:dec_m.code.K], rate_idx)
        g = got_b[k * nb:(k + 1) * nb]
        assert np.array_equal(g[:, 0], np.packbits(out))
        assert (g[-1, 2], g[-1, 3], g[-1, 4], g[-1, 5]) == (int(ok), it, int(conv), L)
        K = dec_m.code.K
        assert g[-1, 6] == np.sum((llr[k][:K] < 0) != hard[:K])            # pre-FEC errors
        assert ok and np.array_equal(np.unpackbits(g[:, 0].astype(np.uint8)), bb[k]), f"frame {k} not decoded"


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
    with open(fin, "w") as f:
        for llr in llrs:
            u = llr.astype(np.int64) & 0xFF
            for k in range(32400):
                f.write(f"{u[2 * k]:02x} {u[2 * k + 1]:02x} {int(k == 0)}\n")
    log = rxsim.run(ldpc_exe, rate=rate_idx, maxit=max_it, stall=stall, gap=gap, **{"in": fin, "out": fout})
    print(log, [(w[1], w[2], w[3]) for w in want])
    got = rxsim.read_hex_cols(fout, (8, 1, 1, 6, 8))
    nb = len(want[0][0])
    assert len(got) == nb * nframes, (len(got), nb * nframes)
    for k, (data, conv, it, ok, chan) in enumerate(want):
        g = got[k * nb:(k + 1) * nb]
        assert np.array_equal(g[:, 0], data), f"frame {k}: data differs"
        assert np.array_equal(g[:, 4], chan), f"frame {k}: channel decisions differ"
        assert g[-1, 1] == 1 and g[:-1, 1].sum() == 0
        assert g[-1, 2] == int(conv) and g[-1, 3] == it, (k, g[-1], conv, it)


# ---------------------------------------------------------------------------
# DDC
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def ddc_exe():
    return rxsim.build("tb_rx_ddc", ["dvbs2_rx_rot.sv", "dvbs2_rx_ddc.sv", os.path.join(HERE, "tb_rx_ddc.sv")])


@pytest.mark.parametrize("dec,finc,period", [(3, 0, 8), (3, 123456789, 6), (18, -98765432, 8), (1, 55555555, 8)])
def test_ddc(ddc_exe, dec, finc, period):
    os.makedirs(WORK, exist_ok=True)
    rng = np.random.default_rng(dec)
    x = np.clip(np.round(rng.standard_normal((6000, 2)) * 3000), -32768, 32767).astype(np.int64)
    x[100] = [32767, -32768]                      # saturation corner
    coefs = fx.ddc_coefs(dec) if dec > 1 else np.array([65536 - 1, 0])
    want = fx.ddc(x, coefs, dec, finc & 0xFFFFFFFF, 16)
    cfile = os.path.join(WORK, f"ddc_coef{dec}.hex")
    with open(cfile, "w") as f:
        for c in coefs:
            f.write(f"{int(c) & 0x3FFFF:05x}\n")
    fin = os.path.join(WORK, f"ddc_in{dec}.hex")
    fout = os.path.join(WORK, f"ddc_out{dec}.hex")
    rxsim.write_iq(fin, x)
    log = rxsim.run(ddc_exe, coef=cfile, dec=dec, ntaps=len(coefs), shift=16, finc=finc & 0xFFFFFFFF,
                    period=period, **{"in": fin, "out": fout})
    print(log)
    assert "dropped 0" in log
    got = rxsim.read_hex_cols(fout, (-16, -16))
    assert len(got) == len(want)
    bad = np.nonzero(np.any(got != want, axis=1))[0]
    assert len(bad) == 0, f"first mismatch {bad[0]}: {got[bad[0]]} vs {want[bad[0]]}"


# ---------------------------------------------------------------------------
# block-design wrapper: ADC samples -> DDC -> receiver -> DMA packets
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def rx_axi_exe():
    return rxsim.build("tb_rx_axi", RX_SOURCES + ["dvbs2_rx_ddc.sv", "dvbs2_rx_axi.sv",
                                                  os.path.join(HERE, "tb_rx_axi.sv")])


def test_rx_axi(rx_axi_exe):
    """ADC-rate samples (6 per symbol, carrier offset partly removed by the
    DDC NCO) through the register-configured wrapper: every DMA packet
    carries a transmitted BBFRAME, equal to the model chain."""
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
                    pkp=r["carrier"].kp_p, pki=r["carrier"].ki_p, maxit=30, **{"in": fin, "out": fout})
    print(log)
    lines = [l.split() for l in open(fregs)]
    regs = {int(a, 16): int(v, 16) for a, v in lines if not a.startswith("s")}
    snap_got = [int(v, 16) for a, v in lines if a.startswith("s")]
    print({hex(k): v for k, v in regs.items()})
    assert regs[0x50] == 0 and regs[0x54] == 0 and regs[0x8C] == 0     # nothing lost
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
        assert (h >> 32) & 0x1F == rate_idx + 1 and (h >> 37) & 1 == int(pilots)
        assert (h >> 40) & 0x3F == it and (h >> 48) & 1 == int(conv)
        assert (h >> 49) & 1 == int(ok) and (h >> 50) & 0x1F == L
        body = pkt[1:].view(np.uint8)[: kb // 8]                      # little endian: byte 0 first
        assert np.array_equal(body, wb), f"packet {k} body"
        assert np.array_equal(np.unpackbits(body), bb[k]), f"packet {k} is not the transmitted BBFRAME"
        nw = 1 + (kb // 8 + 7) // 8
        assert not pkt[nw:].any()


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
                f.write(f"{v:02x} {int(k == n // 8 - 1)} {i % 2} {7 + i} {c:02x}\n")
    log = rxsim.run(bch_exe, rate=rate_idx, stall=stall, **{"in": fin, "out": fout})
    print(log, [(w[1], w[2], w[5]) for w in want])
    got = rxsim.read_hex_cols(fout, (8, 1, 1, 1, 6, 5, 16))
    nb = kbch // 8
    assert len(got) == nb * len(want)
    for i, (data, ok, L, lok, it, right, raw) in enumerate(want):
        g = got[i * nb:(i + 1) * nb]
        assert np.array_equal(g[:, 0], data), f"frame {i}: data differs"
        assert g[-1, 1] == 1 and g[:-1, 1].sum() == 0
        assert (g[-1, 2], g[-1, 3], g[-1, 4], g[-1, 5]) == (int(ok), lok, it, L), (i, g[-1], ok, L)
        assert right == (nerrs[i] <= t)                    # the model corrects <= t
        assert g[-1, 6] == raw


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
    regs = {int(a, 16): int(v, 16) for a, v in lines if not a.startswith("s")}
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
