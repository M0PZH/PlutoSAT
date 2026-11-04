"""Two-slot RX engines (second DATV receive slot, docs/rx2_design.md) vs the
single-slot models, per slot, bit exact: each slot of a two-slot engine,
fed with its own signal while the other slot runs another one, equals the
model (model/rxfixed.py) of a single-slot receiver fed with that signal.

    cd sim/rx && ../../.venv/bin/python -m pytest -v test_rx2.py
"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "model"))
sys.path.insert(0, HERE)

import rxfixed as fx            # noqa: E402
import rxsim                     # noqa: E402
from test_rx import model_symbols, signal, write_coefs  # noqa: E402

WORK = os.path.join(rxsim.BUILD, "work2")


def read_segments(path, widths):
    """Output file of a two-slot testbench: the rows, split at "reset" lines."""
    segs, rows = [], []
    for line in open(path):
        if line.startswith("reset"):
            segs.append(rows)
            rows = []
        else:
            rows.append([int(v, 16) for v in line.split()])
    segs.append(rows)
    out = []
    for rows in segs:
        a = np.array(rows, np.int64).reshape(-1, len(widths))
        for c, w in enumerate(widths):
            if w < 0:
                a[:, c] = np.where(a[:, c] >= 1 << (-w - 1), a[:, c] - (1 << -w), a[:, c])
        out.append(a)
    return out


def assert_prefix(got, want, what, tail=40):
    """got equals the start of want and covers all of it but the last `tail` rows."""
    n = min(len(got), len(want))
    assert n > len(want) - tail, (what, len(got), len(want))
    bad = np.nonzero(np.any(got[:n] != want[:n], axis=1))[0]
    assert len(bad) == 0, f"{what}: first mismatch at row {bad[0]}: {got[bad[0]]} vs {want[bad[0]]}"


def test_generated():
    """rtl/dvbs2_rx_framesync2.sv and dvbs2_rx_carrier2.sv are up to date with the
    single-slot modules (tools/make_rx2.py)."""
    import subprocess
    subprocess.run([sys.executable, os.path.join(HERE, "..", "..", "tools", "make_rx2.py"), "--check"], check=True)


# ---------------------------------------------------------------------------
# timing
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def timing2_exe():
    return rxsim.build("tb_rx_timing2", ["dvbs2_rx_timing2.sv", os.path.join(HERE, "tb_rx_timing2.sv")])


T_A = dict(rate_idx=3, cfo=0.004, ppm=60.0, delay=0.37, seed=1, lead=6, rms=1500.0)
T_B = dict(rate_idx=1, pilots=False, cfo=-0.01, ppm=-100.0, delay=0.81, seed=7, lead=4, rms=300.0)
T_C = dict(rate_idx=0, cfo=0.002, ppm=-1000.0, delay=0.1, seed=3, lead=5, rms=800.0)

# name, signals, roll-offs, gain scale (kp, ki multipliers), integrator presets, stalls,
# per-slot reset after n samples (None: no reset)
TIMING2_CASES = [
    # both slots at full speed: the engine is the bottleneck (42 clocks per symbol shared)
    ("full", (T_A, T_B), (0.35, 0.20), ((1, 1), (1, 1)), (0, 0), (0, 0), (None, None)),
    # stalls, different gains, an integrator preset far off in slot 1
    ("stall", (T_A, T_C), (0.20, 0.35), ((1, 1), (2, 0.5)), (0, 33588), (30, 5), (None, None)),
    # slot 1 reset twice as fast as slot 0 runs: slot 0 must not notice
    ("reset1", (T_A, T_B), (0.35, 0.25), ((1, 1), (1, 1)), (0, -7000), (10, 0), (None, 23456)),
    ("reset0", (T_C, T_A), (0.35, 0.35), ((1, 1), (1, 1)), (0, 0), (0, 20), (5000, None)),
    # slot 1 idle (no input at all)
    ("idle1", (T_A, None), (0.35, 0.35), ((1, 1), (1, 1)), (0, 0), (0, 0), (None, None)),
]


@pytest.mark.parametrize("name,sig,rolloff,gsc,integ0,stall,rst_at", TIMING2_CASES,
                         ids=[c[0] for c in TIMING2_CASES])
def test_timing2(timing2_exe, name, sig, rolloff, gsc, integ0, stall, rst_at):
    os.makedirs(WORK, exist_ok=True)
    kp0, ki0 = fx.timing_gains()
    plus, want = {}, []
    for s in range(2):
        coefs = fx.timing_coefs(rolloff[s])
        kp, ki = int(kp0 * gsc[s][0]), int(ki0 * gsc[s][1])
        if sig[s] is None:
            x = np.zeros((0, 2), np.int64)
        else:
            kw = dict(sig[s])
            x, _ = signal(nframes=0, lead=kw.pop("lead"), **kw)
        want.append(fx.Timing(coefs, kp, ki, integ0=integ0[s]).run(x) if len(x) else np.zeros((0, 2)))
        cf = os.path.join(WORK, f"t2_{name}_coef{s}.hex")
        write_coefs(cf, coefs)
        fin = os.path.join(WORK, f"t2_{name}_in{s}.hex")
        rxsim.write_iq(fin, x)
        plus.update({f"in{s}": fin, f"out{s}": os.path.join(WORK, f"t2_{name}_out{s}.hex"), f"coef{s}": cf,
                     f"kp{s}": kp, f"ki{s}": ki, f"ia{s}": integ0[s], f"stall{s}": stall[s]})
        if rst_at[s] is not None:
            plus[f"rst{s}at"] = rst_at[s]
    log = rxsim.run(timing2_exe, **plus)
    print(log)
    if rst_at != (None, None):
        assert "cancelled 1" in log          # the reset came during that slot's symbol
    for s in range(2):
        segs = read_segments(plus[f"out{s}"], (-16, -16))
        if sig[s] is None:
            assert len(segs) == 1 and len(segs[0]) == 0
            continue
        assert len(segs) == (1 if rst_at[s] is None else 2)
        if rst_at[s] is not None:
            # before the reset: a part of the run (the symbols out so far)
            n = len(segs[0])
            assert n > 0 and np.array_equal(segs[0], want[s][:n]), f"slot {s} before the reset"
        assert_prefix(segs[-1], want[s], f"slot {s}")


# ---------------------------------------------------------------------------
# frame sync
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def framesync2_exe():
    return rxsim.build("tb_rx_framesync2", ["dvbs2_pl_pkg.sv", "dvbs2_rx_framesync2.sv",
                                            os.path.join(HERE, "tb_rx_framesync2.sv")], defines=["RX2_SCRAMBLE"])


def fs_case(kw):
    """symbols, model (tags, nout, events) of one slot. kw: signal() arguments plus
    nframes, lead, drop (symbols [a, b) removed), modcod (configured), pls (stand-in
    (busy clocks, code, conf))."""
    from test_rx import PlsStandIn
    kw = dict(kw)
    drop = kw.pop("drop", None)
    pls = kw.pop("pls", None)
    modcod = kw.pop("modcod", kw.get("rate_idx", 3) + 1)
    x, _ = signal(nframes=kw.pop("nframes", 3), lead=kw.pop("lead", 3), **kw)
    sym = model_symbols(x)
    if drop:
        sym = np.concatenate([sym[:drop[0]], sym[drop[1]:]])
    fs = fx.FrameSync(modcod, kw["pilots"])
    tags, nout = fs.run(sym, PlsStandIn(pls[1], pls[2]) if pls else None)
    return sym, modcod, fs, tags[:nout], nout


# name, the two slots, stall percent, per-slot reset after n symbols
FS2_CASES = [
    ("p_np", (dict(pilots=True, esn0=8.0), dict(pilots=False, esn0=3.0, rate_idx=6, seed=4)), (0, 0),
     (None, None)),
    # flywheel, lost, re-lock in slot 0; QPSK 1/4 at 0 dB in slot 1
    ("drop_stall", (dict(pilots=True, esn0=8.0, nframes=10, drop=(80000, 80100)),
                    dict(pilots=True, esn0=0.0, rate_idx=0, seed=2)), (25, 10), (None, None)),
    # PLS stand-ins slower than the frame sync needs them (both slots wait), slot 0 confirmed
    # (dummy frames: locked), slot 1 never confident (each CHECK dropped)
    ("pls_wait", (dict(pilots=True, esn0=8.0, nframes=0, lead=14, pls=(150000, 0, 1)),
                  dict(pilots=True, esn0=8.0, seed=5, pls=(110000, fx.pls_code(4, 0, 1), 0))), (5, 0),
     (None, None)),
    # QPSK 1/4 without pilots, QPSK 1/2 configured (the dummy-like search peak) in slot 0,
    # the other pilots decoded in slot 1
    ("np14_other", (dict(pilots=False, esn0=1.0, nframes=6, lead=0, rate_idx=0, modcod=4, seed=1),
                    dict(pilots=True, esn0=8.0, seed=6, pls=(20000, fx.pls_code(4, 0, 0), 1))), (15, 5),
     (None, None)),
    # slot 1 reset in the middle (while locked), slot 0 must not notice
    ("reset1", (dict(pilots=True, esn0=6.0, nframes=4, pls=(30000, fx.pls_code(4, 0, 1), 1)),
                dict(pilots=False, esn0=5.0, rate_idx=6, seed=3, pls=(5000, fx.pls_code(7, 0, 0), 1))), (10, 0),
     (None, 51234)),
    # slot 1 idle
    ("idle1", (dict(pilots=True, esn0=6.0, pls=(5000, fx.pls_code(4, 0, 1), 1)), None), (0, 0), (None, None)),
]


@pytest.mark.parametrize("name,kws,stall,rst_at", FS2_CASES, ids=[c[0] for c in FS2_CASES])
def test_framesync2(framesync2_exe, name, kws, stall, rst_at):
    os.makedirs(WORK, exist_ok=True)
    plus, want = {}, []
    for s in range(2):
        fin = os.path.join(WORK, f"f2_{name}_in{s}.hex")
        plus.update({f"in{s}": fin, f"out{s}": os.path.join(WORK, f"f2_{name}_out{s}.hex"), f"stall{s}": stall[s]})
        if kws[s] is None:
            rxsim.write_iq(fin, np.zeros((0, 2), np.int64))
            want.append(None)
            continue
        sym, modcod, fs, tags, nout = fs_case(kws[s])
        rxsim.write_iq(fin, sym)
        plus.update({f"modcod{s}": modcod, f"pilots{s}": int(kws[s]["pilots"]), f"thresh{s}": fs.T})
        if "pls" in kws[s]:
            busy, code, conf = kws[s]["pls"]
            plus.update({f"plsbusy{s}": busy, f"plscode{s}": code, f"plsconf{s}": conf})
        if rst_at[s] is not None:
            plus[f"rst{s}at"] = rst_at[s]
        want.append((sym[:nout], tags, fs.events))
    log = rxsim.run(framesync2_exe, **plus)
    print(log)
    for s in range(2):
        segs = read_segments(plus[f"out{s}"], (-16, -16, 1, 1))
        if want[s] is None:
            assert len(segs) == 1 and len(segs[0]) == 0
            continue
        sym, tags, ev = want[s]
        print(f"slot {s} events", ev)
        assert len(segs) == (1 if rst_at[s] is None else 2)
        for k, got in enumerate(segs):
            n = len(got) if k < len(segs) - 1 else len(sym)
            assert len(got) == n and n > 0, (s, k, len(got), len(sym))
            assert np.array_equal(got[:, :2], sym[:n]), f"slot {s}: symbols"
            got_tag = np.where(got[:, 2] == 1, 1 + got[:, 3], 0)
            bad = np.nonzero(got_tag != tags[:n])[0]
            assert len(bad) == 0, f"slot {s}: tag mismatch at {bad[:5]}: {got_tag[bad[:5]]} vs {tags[bad[:5]]}"
        kinds = [e[0] for e in ev]
        assert kinds.count("found") >= 2
    if name == "pls_wait":
        assert "hold 0" not in log
    if name == "drop_stall":
        kinds = [e[0] for e in want[0][2]]
        assert "flywheel" in kinds and "lost" in kinds and kinds[-1] == "found"


# ---------------------------------------------------------------------------
# carrier
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def carrier2_exe():
    return rxsim.build("tb_rx_carrier2", ["gen/dvbs2_tables_pkg.sv", "dvbs2_pl_pkg.sv", "gen/dvbs2_rx_pkg.sv",
                                          "dvbs2_rx_rot.sv", "dvbs2_rx_cordic.sv", "dvbs2_udiv.sv",
                                          "dvbs2_rx_carrier2.sv", os.path.join(HERE, "tb_rx_carrier2.sv")],
                       defines=["RX2_SCRAMBLE"])


def carrier_case(kw):
    """Tagged carrier input and the model's output of one slot (as test_rx.test_carrier;
    pn=(esn0, seed): pilotless_experiment's symbol-level channel with ideal tags)."""
    kw = dict(kw)
    drop = kw.pop("drop", None)
    if "pn" in kw:
        import pilotless_experiment as pe
        esn0, seed = kw.pop("pn")
        sym, tags, found, _, _ = pe.make_symbols(0, False, 3, esn0, 0.001, 0.002, seed, lead=8)
        modcod, pilots = 1, False
    else:
        pilots = kw["pilots"]
        modcod = kw.get("rate_idx", 3) + 1
        x, _ = signal(nframes=kw.pop("nframes", 2), lead=kw.pop("lead", 8), **kw)
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
    return sym, tags, found, modcod, pilots, cr, want


CR2_CASES = [
    ("p_np", (dict(pilots=True, esn0=6.0), dict(pilots=False, esn0=4.0, cfo=-0.003, rate_idx=6, seed=3)),
     (0, 20), 5, (None, None)),
    ("lowsnr_gap", (dict(pilots=True, esn0=0.0, rate_idx=0), dict(pilots=True, esn0=8.0, drop=(52000, 52002), seed=4)),
     (10, 0), 0, (None, None)),
    ("timeout_gapnp", (dict(pilots=True, esn0=8.0, drop="untag"),
                       dict(pilots=False, esn0=8.0, drop=(52000, 52001), seed=5)), (5, 5), 10, (None, None)),
    # the 1/1476 false-lock correction in slot 0, pilots off through all PLL gears in slot 1
    ("falselock_pn", (dict(pilots=True, esn0=-2.2, rate_idx=0, seed=16, lead=8, nframes=4), dict(pn=(0.0, 3))),
     (0, 10), 0, (None, None)),
    # slot 1 reset in the middle (while it sends a data frame), slot 0 must not notice
    ("reset1", (dict(pilots=True, esn0=6.0, nframes=3), dict(pilots=True, esn0=5.0, rate_idx=5, seed=8)),
     (0, 0), 5, (None, 63000)),
    ("reset0_np", (dict(pilots=False, esn0=4.0, rate_idx=2, nframes=3, seed=9), dict(pilots=True, esn0=5.0)),
     (5, 0), 0, (70000, None)),
    ("idle1", (dict(pilots=False, esn0=6.0, seed=2), None), (0, 0), 5, (None, None)),
]


@pytest.mark.parametrize("name,kws,stall,ostall,rst_at", CR2_CASES, ids=[c[0] for c in CR2_CASES])
def test_carrier2(carrier2_exe, name, kws, stall, ostall, rst_at):
    from test_rx import write_tagged
    import txmodel as m
    os.makedirs(WORK, exist_ok=True)
    plus, want = {"ostall": ostall}, []
    for s in range(2):
        fin = os.path.join(WORK, f"c2_{name}_in{s}.hex")
        plus.update({f"in{s}": fin, f"out{s}": os.path.join(WORK, f"c2_{name}_out{s}.hex"), f"stall{s}": stall[s]})
        if kws[s] is None:
            write_tagged(fin, [], [], [])
            want.append(None)
            continue
        sym, tags, found, modcod, pilots, cr, w = carrier_case(kws[s])
        write_tagged(fin, sym, tags, found)
        xa, xb = m.gold_init(0)
        plus.update({f"modcod{s}": modcod, f"pilots{s}": int(pilots), f"xa{s}": xa, f"xb{s}": xb,
                     f"kp{s}": cr.kp_p, f"ki{s}": cr.ki_p})
        if rst_at[s] is not None:
            plus[f"rst{s}at"] = rst_at[s]
        want.append((cr, w))
    log = rxsim.run(carrier2_exe, **plus)
    print(log)
    for s in range(2):
        segs = read_segments(plus[f"out{s}"], (-8, -8, 1, 1))
        if want[s] is None:
            assert len(segs) == 1 and len(segs[0]) == 0
            continue
        cr, w = want[s]
        assert len(w) >= 32400
        assert f"slot {s}: " in log and f"acq {cr.n_acq} fa {cr.n_fa} retype {cr.n_retype}" in \
            log.split(f"slot {s}: ")[1].split("\n")[0], (s, cr.n_acq, cr.n_fa, cr.n_retype)
        if name == "falselock_pn" and s == 0:
            assert cr.n_fa >= 1
        assert len(segs) == (1 if rst_at[s] is None else 2)
        if rst_at[s] is not None:
            g = segs[0]
            assert np.array_equal(g[:, :2], w[:len(g)]), f"slot {s} before the reset"
            assert len(g) > 0 and len(g) % 32400 != 0, "the reset should cut a frame"
        got = segs[-1]
        assert len(got) == len(w), (s, len(got), len(w))
        bad = np.nonzero(np.any(got[:, :2] != w, axis=1))[0]
        assert len(bad) == 0, f"slot {s}: {len(bad)} mismatches, first at word {bad[0]}: {got[bad[0], :2]} vs {w[bad[0]]}"
        assert np.array_equal(np.nonzero(got[:, 2])[0], np.arange(0, len(got), 32400))
        assert np.array_equal(np.nonzero(got[:, 3])[0], np.arange(32399, len(got), 32400))
    if rst_at != (None, None):
        assert "reset pending 0" not in log   # the reset came during a job of that slot
    if name == "reset0_np":
        assert "dropped 0" not in log         # words of the reset slot left in the output FIFO


# ---------------------------------------------------------------------------
# front end: input FIFOs, timing2, framesync2, carrier2, a PLS decoder per slot
# ---------------------------------------------------------------------------

FE2_SOURCES = ["gen/dvbs2_tables_pkg.sv", "dvbs2_pl_pkg.sv", "gen/dvbs2_rx_pkg.sv", "dvbs2_async_fifo.sv",
               "dvbs2_rx_timing2.sv", "dvbs2_rx_framesync2.sv", "dvbs2_rx_rot.sv", "dvbs2_rx_cordic.sv",
               "dvbs2_udiv.sv", "dvbs2_rx_carrier2.sv", "dvbs2_rx_pls.sv"]


@pytest.fixture(scope="module")
def fe2_exe():
    return rxsim.build("tb_rx_fe2", FE2_SOURCES + [os.path.join(HERE, "tb_rx_fe2.sv")], defines=["RX2_SCRAMBLE"])


def pls_line(d):
    return (d["code"], int(d["conf"]), int(d["carrier_hyp"]), int(d["grid"]), d["best"], d["second"],
            d["freq"] & 0xFFFF)


def decoded(llr_words, rate_idx, bb):
    """Which BBFRAMEs of bb the LLR frames decode to (LDPC and BCH models)."""
    import bchdec
    import ldpcdec as ld
    dec = ld.Decoder(rate_idx)
    out = []
    for f in np.asarray(llr_words).reshape(-1, 64800):
        hard, _, _ = dec.decode(f)
        bits, ok, _ = bchdec.decode(hard[:dec.code.K], rate_idx)
        if ok:
            out.append(np.asarray(bits))
    return [any(np.array_equal(d, b) for d in out) for b in bb]


def fe2_run(exe, name, slots, stall=5):
    """slots: per slot None (idle) or dict(x=samples, rate_idx=, pilots=, period=, rst_at=).
    Runs the testbench; each slot's LLR frames, PLS decodes and final status must equal
    fx.receive() of its samples (after a reset: the run from there; before it: a prefix).
    Returns per slot (model result, RTL LLR words) or None."""
    import txmodel as m
    os.makedirs(WORK, exist_ok=True)
    plus, res = {"stall": stall}, []
    kp, ki = fx.timing_gains()
    xa, xb = m.gold_init(0)
    for s in range(2):
        cf = os.path.join(WORK, f"fe2_{name}_coef{s}.hex")
        write_coefs(cf, fx.timing_coefs(0.35))
        fin, fout, fpls = (os.path.join(WORK, f"fe2_{name}_{k}{s}.hex") for k in ("in", "out", "pls"))
        plus.update({f"coef{s}": cf, f"in{s}": fin, f"out{s}": fout, f"pls{s}": fpls})
        sl = slots[s]
        if sl is None:
            rxsim.write_iq(fin, np.zeros((0, 2), np.int64))
            res.append(None)
            continue
        rxsim.write_iq(fin, sl["x"])
        r = fx.receive(sl["x"], sl["rate_idx"] + 1, sl["pilots"])
        plus.update({f"period{s}": sl["period"], f"modcod{s}": sl["rate_idx"] + 1, f"pilots{s}": int(sl["pilots"]),
                     f"thresh{s}": fx.fs_threshold(), f"xa{s}": xa, f"xb{s}": xb, f"tkp{s}": kp, f"tki{s}": ki,
                     f"pkp{s}": r["carrier"].kp_p, f"pki{s}": r["carrier"].ki_p})
        if sl.get("rst_at") is not None:
            plus[f"rst{s}at"] = sl["rst_at"]
        res.append(r)
    log = rxsim.run(exe, **plus)
    print(log)
    out = []
    for s in range(2):
        segs = read_segments(plus[f"out{s}"], (-8, -8, 1, 1))
        psegs = [[]]
        for ln in open(plus[f"pls{s}"]):
            if ln.startswith("reset"):
                psegs.append([])
            else:
                psegs[-1].append(tuple(int(v, 16) for v in ln.split()))
        r = res[s]
        line = log.split(f"slot {s}: ")[1].split("\n")[0]
        if r is None:
            assert len(segs) == 1 and len(segs[0]) == 0 and psegs == [[]]
            out.append(None)
            continue
        nseg = 1 if slots[s].get("rst_at") is None else 2
        assert len(segs) == nseg and len(psegs) == nseg
        want = np.concatenate(r["frames"]).reshape(-1, 2) if r["frames"] else np.zeros((0, 2), np.int64)
        want_p = [pls_line(d) for d in r["pls"].results]
        if nseg == 2:
            g = segs[0]
            assert len(g) > 0 and np.array_equal(g[:, :2], want[:len(g)]), f"slot {s} before the reset"
            assert psegs[0] == want_p[:len(psegs[0])]
        got = segs[-1]
        assert "overflows 0," in line and "skipped 0," in line, line
        cr = r["carrier"]
        assert f"amp {cr.A}, energy {cr.E}, gain {cr.G}," in line, (s, cr.A, cr.E, cr.G)
        assert psegs[-1] == want_p, f"slot {s}: PLS decodes"
        # (the RTL also sends the start of a frame the input ends in)
        assert len(want) <= len(got) < len(want) + 32400, (s, len(got), len(want))
        bad = np.nonzero(np.any(got[:len(want), :2] != want, axis=1))[0]
        assert len(bad) == 0, f"slot {s}: {len(bad)} mismatches, first at word {bad[0]}"
        if len(got):
            assert np.array_equal(np.nonzero(got[:, 2])[0], np.arange(0, len(got), 32400))
        out.append((r, got[:len(want), :2]))
    return out, log


def falselock_signal():
    """test_rx.test_rx_falselock's input: null-packet TS, QPSK 9/10 pilots, the RX
    started mid-frame with the AGC settling from above."""
    import falselock_experiment as fe
    tx, bb = fe.tx_signal(10, True, 10, "null")
    start = 28310
    return fe.rx_input(tx, start, 20.0, 0.002, 20.0, 3, rms=8000), bb, start


def check_falselock(r, llr, bb, start):
    import falselock_experiment as fe
    cl = fe.classify(r, fx.frame_len(True), start)
    assert not cl[0][2] and all(ok for _, _, ok in cl[1:]) and len(cl) >= 7, cl
    assert [e[0] for e in r["fs"].events].count("pls_drop") == 1
    got = decoded(llr, 10, bb)
    print("falselock slot decoded", got)
    assert got[-2]


def np14_signal():
    """test_rx.test_rx_start_np14's input: QPSK 1/4 without pilots at -1 dB after 8
    dummy frames."""
    return signal(nframes=4, lead=8, rate_idx=0, pilots=False, esn0=-1.0, seed=0)


# Sample periods: two slots together at most 2.38 MBd (42 clocks per symbol, 2 samples
# per symbol), e.g. 30 + 75 clocks per sample: 1.67 + 0.67 MBd.

def test_fe2_falselock_np14(fe2_exe):
    """The RX 1.4 regression cases, one in each slot at different rates: the null-packet
    false lock (QPSK 9/10 pilots) and the QPSK 1/4 no-pilots start-up."""
    xf, bbf, start = falselock_signal()
    xn, bbn = np14_signal()
    out, log = fe2_run(fe2_exe, "fl_np14", [dict(x=xf, rate_idx=10, pilots=True, period=30),
                                             dict(x=xn, rate_idx=0, pilots=False, period=75)])
    check_falselock(out[0][0], out[0][1], bbf, start)
    got = decoded(out[1][1], 0, bbn)
    print("np14 slot decoded", got, out[1][0]["fs"].events)
    assert all(got)


def test_fe2_np14_falselock(fe2_exe):
    """The same with the slots exchanged and the rates the other way round."""
    xf, bbf, start = falselock_signal()
    xn, bbn = np14_signal()
    out, log = fe2_run(fe2_exe, "np14_fl", [dict(x=xn, rate_idx=0, pilots=False, period=26),
                                             dict(x=xf, rate_idx=10, pilots=True, period=150)])
    assert all(decoded(out[0][1], 0, bbn))
    check_falselock(out[1][0], out[1][1], bbf, start)


def test_fe2_modcods(fe2_exe):
    """Different MODCODs, pilots, offsets and rates (test_rx_end_to_end's two signals)."""
    xa, bba = signal(nframes=4, lead=8, rate_idx=3, pilots=True, esn0=3.0, cfo=0.004, ppm=60.0)
    xb, bbb = signal(nframes=4, lead=8, rate_idx=6, pilots=False, esn0=6.0, cfo=-0.002, ppm=-80.0, rms=400.0)
    out, log = fe2_run(fe2_exe, "modcods", [dict(x=xa, rate_idx=3, pilots=True, period=44),
                                             dict(x=xb, rate_idx=6, pilots=False, period=44)])
    assert all(decoded(out[0][1], 3, bba)) and all(decoded(out[1][1], 6, bbb))


def test_fe2_reset1(fe2_exe):
    """Slot 1 reset (every engine, FIFO, PLS decoder) in the middle of a data frame
    while slot 0 runs: slot 0 equals the model as if nothing happened, slot 1 after
    the reset equals the model of its input from the start."""
    xa, bba = signal(nframes=3, lead=8, rate_idx=3, pilots=True, esn0=4.0, cfo=0.003)
    xb, bbb = signal(nframes=3, lead=8, rate_idx=5, pilots=False, esn0=7.0, cfo=-0.004, seed=3)
    out, log = fe2_run(fe2_exe, "reset1", [dict(x=xa, rate_idx=3, pilots=True, period=30),
                                            dict(x=xb, rate_idx=5, pilots=False, period=75, rst_at=95000)])
    assert all(decoded(out[0][1], 3, bba)) and all(decoded(out[1][1], 5, bbb))


def test_fe2_idle1(fe2_exe):
    """Slot 1 idle, slot 0 at the single-slot receiver's tested maximum (tb_rx: 44
    clocks per symbol, 2.27 MBd; at exactly 42 the input FIFO overflows in either)."""
    xa, bba = signal(nframes=3, lead=8, rate_idx=4, pilots=True, esn0=5.0, cfo=-0.005, ppm=30.0)
    out, log = fe2_run(fe2_exe, "idle1", [dict(x=xa, rate_idx=4, pilots=True, period=22), None])
    assert all(decoded(out[0][1], 4, bba))


# ---------------------------------------------------------------------------
# whole receiver, two slots: the core dvbs2_rx (tb_rx) and the wrapper
# dvbs2_rx_axi (tb_rx_axi, ADC samples holding both signals)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def rx_exe():
    from test_rx import RX_SOURCES
    return rxsim.build("tb_rx", RX_SOURCES + [os.path.join(HERE, "tb_rx.sv")])


def ldpc_bch_model(llr, rate_idx, max_it=30):
    """The decoders' model output for one LLR frame: (bytes, iterations, converged, BCH ok, nerr, raw)."""
    import bchdec
    import ldpcdec as ld
    dec = ld.Decoder(rate_idx)
    hard, it, conv = dec.decode(llr, max_it=max_it)
    K = dec.code.K
    out, ok, L = bchdec.decode(hard[:K], rate_idx)
    return np.packbits(out), it, conv, ok, L, int(np.sum((llr[:K] < 0) != hard[:K]))


def core_run(exe, name, slots, stall=5, **common):
    """slots: per slot None (idle) or dict(x=samples, rate_idx=, pilots=, period=, rst_at=, maxit=).
    Runs the whole two-slot core; per slot the carrier's LLR frames and PLS decodes
    must equal fx.receive() of its samples (after a reset: the run from there,
    before it a prefix), and the decoded frames (tagged with the slot) equal the
    LDPC/BCH models of the slot's complete LLR frames, in order, except the
    frames the LLR queue dropped (`drops` per slot: None = none allowed).
    Returns per slot dict(r=model, llr=RTL LLR frames, dec=decoded frames, line=status) or None."""
    import txmodel as m
    os.makedirs(WORK, exist_ok=True)
    plus, res = {"stall": stall, "axistall": 20, "axilat": 100}, []
    plus.update(common)
    kp, ki = fx.timing_gains()
    xa, xb = m.gold_init(0)
    for s in range(2):
        sl = slots[s]
        if sl is None:
            res.append(None)
            continue
        cf = os.path.join(WORK, f"core_{name}_coef{s}.hex")
        write_coefs(cf, fx.timing_coefs(0.35))
        fin, fout, fpls, fb = (os.path.join(WORK, f"core_{name}_{k}{s}.hex") for k in ("in", "out", "pls", "bytes"))
        rxsim.write_iq(fin, sl["x"])
        r = fx.receive(sl["x"], sl["rate_idx"] + 1, sl["pilots"])
        plus.update({f"coef{s}": cf, f"in{s}": fin, f"out{s}": fout, f"pls{s}": fpls, f"bytes{s}": fb,
                     f"period{s}": sl["period"], f"modcod{s}": sl["rate_idx"] + 1, f"pilots{s}": int(sl["pilots"]),
                     f"thresh{s}": fx.fs_threshold(), f"xa{s}": xa, f"xb{s}": xb, f"tkp{s}": kp, f"tki{s}": ki,
                     f"pkp{s}": r["carrier"].kp_p, f"pki{s}": r["carrier"].ki_p, f"maxit{s}": sl.get("maxit", 30)})
        if sl.get("rst_at") is not None:
            plus[f"rst{s}at"] = sl["rst_at"]
        res.append(r)
    log = rxsim.run(exe, **plus)
    print(log)
    out = []
    for s in range(2):
        line = log.split(f"slot {s}: ")[1].split("\n")[0]
        r, sl = res[s], slots[s]
        if r is None:
            assert line.startswith("0 samples,") and " 0 LLR words," in line and " 0 bytes," in line, line
            out.append(None)
            continue
        segs = read_segments(plus[f"out{s}"], (-8, -8, 1, 1))
        psegs = [[]]
        for ln in open(plus[f"pls{s}"]):
            if ln.startswith("reset"):
                psegs.append([])
            else:
                psegs[-1].append(tuple(int(v, 16) for v in ln.split()))
        nseg = 1 if sl.get("rst_at") is None else 2
        assert len(segs) == nseg and len(psegs) == nseg
        want = np.concatenate(r["frames"]).reshape(-1, 2) if r["frames"] else np.zeros((0, 2), np.int64)
        want_p = [pls_line(d) for d in r["pls"].results]
        if nseg == 2:
            g = segs[0]
            assert len(g) > 0 and np.array_equal(g[:, :2], want[:len(g)]), f"slot {s} before the reset"
            assert psegs[0] == want_p[:len(psegs[0])]
        got = segs[-1]
        assert "overflows 0," in line and "skipped 0," in line, line
        cr = r["carrier"]
        assert f"amp {cr.A}, energy {cr.E}, gain {cr.G}," in line, (s, cr.A, cr.E, cr.G)
        assert psegs[-1] == want_p, f"slot {s}: PLS decodes"
        assert len(want) <= len(got) < len(want) + 32400, (s, len(got), len(want))
        bad = np.nonzero(np.any(got[:len(want), :2] != want, axis=1))[0]
        assert len(bad) == 0, f"slot {s}: {len(bad)} mismatches, first at word {bad[0]}"
        # the frames sent to the decoder: the complete ones of every segment
        frames = []
        for g in segs:
            n = len(g) // 32400
            frames += [g[k * 32400:(k + 1) * 32400, :2].astype(np.int8).reshape(64800) for k in range(n)]
        # decoded frames of the slot
        rows = rxsim.read_hex_cols(plus[f"bytes{s}"], (8, 1, 1, 8, 1, 8, 16, 4, 8, 1, 1))
        ends = list(np.nonzero(rows[:, 1])[0]) if len(rows) else []
        got_f, p = [], 0
        for e in ends:
            got_f.append(rows[p:e + 1])
            p = e + 1
        assert p == len(rows)
        rate_idx = sl["rate_idx"]
        assert all(np.all(g[:, 7:] == (rate_idx, rate_idx + 1, int(sl["pilots"]), s)) for g in got_f)
        drops = int(line.split("llrq dropped ")[1].split(",")[0])
        if sl.get("drops") is None:
            assert drops == 0 and len(got_f) == len(frames), (s, drops, len(got_f), len(frames))
        else:
            assert len(got_f) + drops == len(frames), (s, drops, len(got_f), len(frames))
        # in order; with drops, a subsequence of the frames
        k = 0
        dec = []
        for g in got_f:
            it_got = int(g[-1, 3])
            while True:
                assert k < len(frames), f"slot {s}: decoded frame not among the LLR frames"
                wb, it, conv, ok, L, raw = ldpc_bch_model(frames[k], rate_idx, max_it=it_got if sl.get("cap") else
                                                           sl.get("maxit", 30))
                k += 1
                if len(wb) == len(g) and np.array_equal(g[:, 0], wb):
                    break
                assert sl.get("drops") is not None, f"slot {s}: frame {k - 1} differs"
            assert (g[-1, 2], g[-1, 3], g[-1, 4], g[-1, 5]) == (int(ok), it, int(conv), L), (s, k - 1)
            if ok:
                assert g[-1, 6] == raw
            dec.append(np.unpackbits(g[:, 0].astype(np.uint8)))
        n_ok = sum(1 for g in got_f if g[-1, 2])
        assert f"ldpc frames {len(got_f)} failed {sum(1 for g in got_f if not g[-1, 4])}," in line, line
        assert f"failed {len(got_f) - n_ok}, pls" in line, line
        out.append(dict(r=r, llr=frames, dec=dec, line=line,
                        iters=[(int(g[-1, 3]), int(g[-1, 4])) for g in got_f]))
    assert " axi err 0," in log
    return out, log


def all_decoded(dec, bb):
    return [any(np.array_equal(d, b) for d in dec) for b in bb]


def test_core2_modcods(rx_exe):
    """Whole core, two slots at once near the total rate limit (30 + 75 clocks per
    sample: 1.67 + 0.67 MBd): QPSK 1/2 pilots and QPSK 3/4 without pilots, different
    offsets; each slot equals the single-slot receiver, every BBFRAME decodes."""
    xa, bba = signal(nframes=4, lead=8, rate_idx=3, pilots=True, esn0=3.0, cfo=0.004, ppm=60.0)
    xb, bbb = signal(nframes=3, lead=8, rate_idx=6, pilots=False, esn0=6.0, cfo=-0.002, ppm=-80.0, rms=400.0)
    out, log = core_run(rx_exe, "modcods", [dict(x=xa, rate_idx=3, pilots=True, period=30),
                                             dict(x=xb, rate_idx=6, pilots=False, period=75)])
    assert all(all_decoded(out[0]["dec"], bba)) and all(all_decoded(out[1]["dec"], bbb))


def test_core2_falselock_np14(rx_exe):
    """The RX 1.4 regressions through the whole core, one in each slot: the null-packet
    false lock (QPSK 9/10 pilots) in slot 0, the QPSK 1/4 no-pilots start-up in slot 1."""
    xf, bbf, start = falselock_signal()
    xn, bbn = np14_signal()
    out, log = core_run(rx_exe, "fl_np14", [dict(x=xf, rate_idx=10, pilots=True, period=30),
                                             dict(x=xn, rate_idx=0, pilots=False, period=75)])
    check_falselock(out[0]["r"], np.concatenate(out[0]["llr"]).reshape(-1, 2), bbf, start)
    assert all(all_decoded(out[1]["dec"], bbn)), out[1]["r"]["fs"].events


def test_core2_np14_falselock(rx_exe):
    """The same with the slots exchanged."""
    xf, bbf, start = falselock_signal()
    xn, bbn = np14_signal()
    out, log = core_run(rx_exe, "np14_fl", [dict(x=xn, rate_idx=0, pilots=False, period=26),
                                             dict(x=xf, rate_idx=10, pilots=True, period=150)])
    assert all(all_decoded(out[0]["dec"], bbn))
    check_falselock(out[1]["r"], np.concatenate(out[1]["llr"]).reshape(-1, 2), bbf, start)


def test_core2_reset1(rx_exe):
    """Slot 1 reset in the middle of a data frame while slot 0 runs: slot 0 equals the
    model and decodes as if nothing happened; slot 1's cut frame is not decoded, its
    frames after the reset are (equal to the model of its input from the start)."""
    xa, bba = signal(nframes=3, lead=8, rate_idx=3, pilots=True, esn0=4.0, cfo=0.003)
    xb, bbb = signal(nframes=3, lead=8, rate_idx=5, pilots=False, esn0=7.0, cfo=-0.004, seed=3)
    out, log = core_run(rx_exe, "reset1", [dict(x=xa, rate_idx=3, pilots=True, period=30),
                                            dict(x=xb, rate_idx=5, pilots=False, period=75, rst_at=95000)])
    assert all(all_decoded(out[0]["dec"], bba)) and all(all_decoded(out[1]["dec"], bbb))


def test_core2_idle1(rx_exe):
    """Slot 1 idle, slot 0 at 2.27 MBd (44 clocks per symbol): the single-slot case."""
    xa, bba = signal(nframes=3, lead=8, rate_idx=4, pilots=True, esn0=5.0, cfo=-0.005, ppm=30.0)
    out, log = core_run(rx_exe, "idle1", [dict(x=xa, rate_idx=4, pilots=True, period=22), None])
    assert all(all_decoded(out[0]["dec"], bba))


def test_core2_drop(rx_exe):
    """LLR queue full with two slots (1.47 + 0.67 MBd): rings of one frame per slot
    (LLRQ_CFG frames per slot = 1) and the decoder held at 60 iterations of frames that
    do not converge (-1 dB QPSK 1/2): frames are dropped and counted per slot, the decoded ones are
    the slot's frames in order, each equal to the model."""
    xa, bba = signal(nframes=4, lead=8, rate_idx=3, pilots=True, esn0=-1.0, cfo=0.004, ppm=60.0, seed=11)
    xb, bbb = signal(nframes=4, lead=8, rate_idx=3, pilots=True, esn0=-1.0, cfo=-0.003, ppm=20.0, seed=12)
    out, log = core_run(rx_exe, "drop", [dict(x=xa, rate_idx=3, pilots=True, period=34, maxit=60, drops=1),
                                          dict(x=xb, rate_idx=3, pilots=True, period=75, maxit=60, drops=1)],
                        llrqnfr=1)
    drops = [int(o["line"].split("llrq dropped ")[1].split(",")[0]) for o in out]
    print("drops", drops)
    assert sum(drops) >= 1


def test_core2_itcap(rx_exe):
    """Shared iteration budget with two slots: QHI = 0, MAX_IT_LO = 3: while a frame
    waits in the queue or is being read from it, the one being decoded stops at 3
    iterations (or where it stands, if later). Every frame is decoded (nothing dropped), each equal to the
    model at its iteration count, and some were capped."""
    xa, bba = signal(nframes=4, lead=8, rate_idx=3, pilots=True, esn0=0.5, cfo=0.004, ppm=60.0, seed=13)
    xb, bbb = signal(nframes=4, lead=8, rate_idx=4, pilots=True, esn0=1.5, cfo=-0.003, ppm=20.0, seed=14)
    out, log = core_run(rx_exe, "itcap", [dict(x=xa, rate_idx=3, pilots=True, period=34, maxit=60, cap=True),
                                           dict(x=xb, rate_idx=4, pilots=True, period=75, maxit=60, cap=True)],
                        llrqqhi=0, llrqitlo=3)
    its = out[0]["iters"] + out[1]["iters"]
    print("iterations, converged", its)
    assert any(it < 60 and not conv for it, conv in its)                 # capped


@pytest.fixture(scope="module")
def rx_axi_exe():
    from test_rx import RX_SOURCES
    return rxsim.build("tb_rx_axi", RX_SOURCES + ["dvbs2_rx_ddc.sv", "dvbs2_rx_axi.sv",
                                                  os.path.join(HERE, "tb_rx_axi.sv")])


def two_signals(specs, fs=12e6, seed=5, rms=500.0):
    """ADC samples (12 MS/s, 12 bit) holding one DVB-S2 signal per spec dict(rate_idx,
    pilots, dec (symbol rate fs / (2 dec)), off (Hz from the LO), gold, nframes, lead,
    esn0, ppm), with one AWGN floor (each signal's power set for its Es/N0).
    Returns x and the BBFRAMEs per signal."""
    import grref
    import rxmodel as rf
    import txmodel as m
    rng = np.random.default_rng(seed)
    ys, bbs = [], []
    for k, sp in enumerate(specs):
        ri = sp["rate_idx"]
        kb = m.LDPC[m.RATES[ri]]["kbch"]
        ts = rng.integers(0, 256, ((sp["nframes"] + 1) * kb // 1504 + 3, 188), dtype=np.uint8)
        ts[:, 0] = 0x47
        bb = grref.bbheader(ts.ravel(), m.RATES[ri])[: sp["nframes"] * kb].reshape(sp["nframes"], kb)
        g = sp.get("gold", 0)
        frames = [m.dummy_plframe_signs(g)] * sp["lead"] + \
                 [m.plframe_signs(m.fec_frame(b, ri), ri + 1, sp["pilots"], g) for b in bb] + \
                 [m.dummy_plframe_signs(g)] * 2
        tx = m.rrc_interp(m.signs_to_iq(np.concatenate(frames)), m.rrc_coefs(8, 0.35, 16), 8, 16)
        sps = 2 * sp["dec"]
        y = rf.channel(tx, 8, sps, sp["off"] / (fs / sps), sp.get("ppm", 20.0), 0.3, 300.0, seed + k)
        # Es/N0 with unit noise power per sample: Es = P sps
        y *= np.sqrt(10 ** (sp["esn0"] / 10) / sps / np.mean(np.abs(y) ** 2))
        ys.append(y)
        bbs.append(bb)
    n = max(len(y) for y in ys)
    x = np.sqrt(0.5) * (rng.standard_normal(n) + 1j * rng.standard_normal(n))
    for y in ys:
        x[:len(y)] += y
    x *= rms / np.sqrt(np.mean(np.abs(x) ** 2) / 2)
    return np.clip(np.round(np.stack([x.real, x.imag], 1)), -2048, 2047).astype(np.int64), bbs


def test_axi2_two_signals(rx_axi_exe):
    """dvbs2_rx_axi with both slots on one ADC stream near the total rate limit: slot 0
    1.5 MBd QPSK 1/2 pilots 2.6 MHz below the LO, slot 1 667 kBd QPSK 3/4 without pilots
    (another Gold code) 2 MHz above it (2.17 MBd together), each slot configured through
    its register bank as dvbs2-rx does it (ddc_config). Every DMA packet of each slot
    (header bit 55) equals the model chain of that slot and carries a transmitted
    BBFRAME, each slot's counters and PLS registers equal its model, the frame
    counters run per slot, nothing is lost; the snapshot is slot 1's."""
    import txmodel as m
    from test_rx import check_pls_regs, read_regs, write_coefs as wc, SLOT_BANK, VERSION
    os.makedirs(WORK, exist_ok=True)
    fs = 12e6
    specs = [dict(rate_idx=3, pilots=True, dec=4, off=-2.6e6, gold=0, nframes=6, lead=8, esn0=4.0, ppm=30.0),
             dict(rate_idx=6, pilots=False, dec=9, off=2.0e6, gold=5, nframes=2, lead=8, esn0=8.0, ppm=-40.0)]
    x, bbs = two_signals(specs, fs)
    plus = dict(adcdiv=4, stall=20, axistall=30, axilat=200, en0=1, en1=1, snapslot=1)
    want, snaps = [], []
    kp, ki = fx.timing_gains()
    llr0 = fx.Carrier._llr
    for s, sp in enumerate(specs):
        cfg = fx.ddc_config(sp["dec"])
        finc = (-round(sp["off"] / fs * 2 ** 32)) & 0xFFFFFFFF
        x2 = fx.ddc(x, cfg["coefs"], cfg["D"], finc, cfg["shift"], cfg["R"], cfg["cic_shift"])
        snap = []

        def llr_rec(self, buf, y, snap=snap):
            if len(snap) < 1024:
                snap.append((int(y[0]), int(y[1])))
            llr0(self, buf, y)
        fx.Carrier._llr = llr_rec
        try:
            r = fx.receive(x2, sp["rate_idx"] + 1, sp["pilots"], gold=sp["gold"])
        finally:
            fx.Carrier._llr = llr0
        snaps.append(snap)
        w = [ldpc_bch_model(f, sp["rate_idx"]) for f in r["frames"]]
        print(f"slot {s}: {len(r['frames'])} frames, ok {[v[3] for v in w]}")
        want.append((r, w))
        fddc = os.path.join(WORK, f"axi2_ddc{s}.hex")
        with open(fddc, "w") as f:
            for v in cfg["coefs"]:
                f.write(f"{int(v) & 0x3FFFF:05x}\n")
        frrc = os.path.join(WORK, f"axi2_rrc{s}.hex")
        wc(frrc, fx.timing_coefs(0.35))
        xa, xb = m.gold_init(sp["gold"])
        plus.update({f"ddccoef{s}": fddc, f"rrccoef{s}": frrc, f"modcod{s}": sp["rate_idx"] + 1,
                     f"pilots{s}": int(sp["pilots"]), f"xa{s}": xa, f"xb{s}": xb, f"finc{s}": finc,
                     f"dec{s}": cfg["D"], f"ntaps{s}": len(cfg["coefs"]), f"shift{s}": cfg["shift"],
                     f"cicr{s}": cfg["R"], f"cicshift{s}": cfg["cic_shift"], f"tkp{s}": kp, f"tki{s}": ki,
                     f"thresh{s}": fx.fs_threshold(), f"pkp{s}": r["carrier"].kp_p, f"pki{s}": r["carrier"].ki_p,
                     f"maxit{s}": 30})
    fin, fout, fregs = (os.path.join(WORK, f"axi2_{k}.hex") for k in ("in", "out", "regs"))
    rxsim.write_iq(fin, x)
    log = rxsim.run(rx_axi_exe, regs=fregs, **plus, **{"in": fin, "out": fout})
    print(log)
    os.remove(fin)
    regs, lines = read_regs(fregs)
    print({hex(k): v for k, v in regs.items() if v})
    words = [l.split() for l in open(fout)]
    data = np.array([int(w, 16) for w, _ in words], dtype=np.uint64)
    assert len(data) % 1024 == 0
    pk = data.reshape(-1, 1024)
    assert regs[0x200] == VERSION and regs[0x2EC] == 0 and regs[0x2F0] == 0
    assert regs[0x2E8] >> 16 == sum(len(w) for _, w in want)
    for s, ((r, w), sp) in enumerate(zip(want, specs)):
        b = SLOT_BANK * s
        mine = [p for p in pk if (int(p[0]) >> 55) & 1 == s]
        kb = len(bbs[s][0])
        assert len(mine) == len(w) == regs[b + 0x90] == regs[b + 0x84], (s, len(mine), len(w))
        assert regs[b + 0x50] == 0 and regs[b + 0x54] == 0 and regs[b + 0x8C] == 0 and regs[b + 0xEC] == 0
        for k, (p, (wb, it, conv, ok, L, raw)) in enumerate(zip(mine, w)):
            h = int(p[0])
            assert h >> 56 == 0xB5 and h & 0xFFFF == kb // 8 and (h >> 16) & 0xFFFF == k
            assert (h >> 32) & 0x1F == sp["rate_idx"] + 1 and (h >> 37) & 1 == int(sp["pilots"])
            assert (h >> 40) & 0x3F == it and (h >> 48) & 1 == int(conv) and (h >> 49) & 1 == int(ok)
            assert (h >> 50) & 0x1F == L
            body = p[1:].view(np.uint8)[: kb // 8]
            assert np.array_equal(body, wb), f"slot {s} packet {k} body"
        dec = [np.unpackbits(p[1:].view(np.uint8)[: kb // 8]) for p in mine]
        got = all_decoded(dec, bbs[s])
        print(f"slot {s} decoded", got)
        assert all(got)
        check_pls_regs(regs, lines, r["pls"], sp["rate_idx"] + 1, sp["pilots"], s)
        cr = r["carrier"]
        assert (regs[b + 0x74], regs[b + 0x78], regs[b + 0x7C]) == (cr.A & 0xFFFFFFFF, (cr.E >> 4) & 0xFFFFFFFF, cr.G)
    snap_got = [int(v, 16) for a, v in lines if a.startswith("s")]
    sg = [((v & 0xFFFF) - ((v & 0x8000) << 1), (v >> 16) - ((v >> 31) << 16)) for v in snap_got]
    assert regs[0x2A0] & 0xB == 2 | 8 and sg == snaps[1], "snapshot (slot 1)"
