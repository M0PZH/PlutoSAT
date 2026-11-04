"""PROTOTYPE (rx2 study): two-slot carrier vs the model, per slot, bit exact.
    cd sim/rx && ../../.venv/bin/python test_rx2_carrier.py"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "model"))
sys.path.insert(0, HERE)
import rxfixed as fx            # noqa: E402
import rxsim                     # noqa: E402
import txmodel as m              # noqa: E402
from test_rx import signal, model_symbols, write_tagged  # noqa: E402

WORK = os.path.join(rxsim.BUILD, "work2")


def slot_input(tag, s, kw, drop=None):
    kw = dict(kw)
    pilots = kw["pilots"]
    modcod = kw.get("rate_idx", 3) + 1
    nfr = kw.pop("nframes", 2)
    lead = kw.pop("lead", 3)
    x, _ = signal(nframes=nfr, lead=lead, **kw)
    sym = model_symbols(x)
    if drop:
        sym = np.concatenate([sym[:drop[0]], sym[drop[1]:]])
    fs = fx.FrameSync(modcod, pilots)
    tags, nout = fs.run(sym)
    sym, tags = sym[:nout], tags[:nout].copy()
    found = fs.found[:nout]
    cr = fx.Carrier(modcod, pilots)
    want = cr.run(sym, tags, found)
    want = np.concatenate(want).reshape(-1, 2) if want else np.zeros((0, 2), np.int64)
    fin = os.path.join(WORK, f"c2_{tag}_in{s}.hex")
    write_tagged(fin, sym, tags, found)
    xa, xb = m.gold_init(0)
    plus = {f"in{s}": fin, f"out{s}": os.path.join(WORK, f"c2_{tag}_out{s}.hex"), f"modcod{s}": modcod,
            f"pilots{s}": int(pilots), f"xa{s}": xa, f"xb{s}": xb, f"kp{s}": cr.kp_p, f"ki{s}": cr.ki_p}
    return plus, want, cr


def run_case(exe, tag, kws, stall, drops=(None, None)):
    os.makedirs(WORK, exist_ok=True)
    plus, wants, crs = {}, [], []
    for s in range(2):
        p, w, cr = slot_input(tag, s, kws[s], drops[s])
        plus.update(p)
        wants.append(w)
        crs.append(cr)
    log = rxsim.run(exe, stall=stall, **plus)
    print(tag, log.strip())
    for s in range(2):
        cr = crs[s]
        assert f"slot {s}:" in log
        line = [ln for ln in log.splitlines() if ln.startswith(f"slot {s}:")][0]
        assert f"acq {cr.n_acq} fa {cr.n_fa} retype {cr.n_retype}" in line, (line, cr.n_acq, cr.n_fa)
        got = rxsim.read_hex_cols(plus[f"out{s}"], (-8, -8, 1, 1))
        want = wants[s]
        assert len(got) == len(want), (s, len(got), len(want))
        bad = np.nonzero(np.any(got[:, :2] != want, axis=1))[0]
        assert len(bad) == 0, f"{tag} slot {s}: {len(bad)} mismatches, first at {bad[0]}"
        assert np.array_equal(np.nonzero(got[:, 2])[0], np.arange(0, len(got), 32400))
        assert np.array_equal(np.nonzero(got[:, 3])[0], np.arange(32399, len(got), 32400))
        print(f"  slot {s}: {len(got)} LLR words ({len(got) // 32400} frames) equal the model")


def main():
    exe = rxsim.build("tb_rx_carrier2", ["gen/dvbs2_tables_pkg.sv", "dvbs2_pl_pkg.sv", "gen/dvbs2_rx_pkg.sv",
                                         "dvbs2_rx_rot.sv", "dvbs2_rx_cordic.sv", "dvbs2_udiv.sv",
                                         "dvbs2_rx_carrier2.sv", os.path.join(HERE, "tb_rx_carrier2.sv")])
    a = dict(pilots=True, esn0=6.0)
    b = dict(pilots=False, esn0=4.0, cfo=-0.003, rate_idx=6, seed=5)
    run_case(exe, "p_np", [a, b], 0)
    c = dict(pilots=True, esn0=0.0, rate_idx=0, seed=3, cfo=0.002)
    run_case(exe, "np_low_stall", [b, c], 20)
    run_case(exe, "gap", [dict(pilots=True, esn0=8.0), dict(pilots=True, esn0=6.0, rate_idx=4, seed=9)], 10,
             drops=((52000, 52002), None))


if __name__ == "__main__":
    main()
