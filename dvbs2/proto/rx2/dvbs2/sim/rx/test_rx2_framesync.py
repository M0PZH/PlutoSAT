"""PROTOTYPE (rx2 study): two-slot frame sync vs the model, per slot, bit exact.
    cd sim/rx && ../../.venv/bin/python test_rx2_framesync.py"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "model"))
sys.path.insert(0, HERE)
import rxfixed as fx            # noqa: E402
import rxsim                     # noqa: E402
from test_rx import signal, model_symbols  # noqa: E402

WORK = os.path.join(rxsim.BUILD, "work2")


def run_case(exe, tag, kws, stall, drops=(None, None)):
    os.makedirs(WORK, exist_ok=True)
    plus, want = {}, []
    for s in range(2):
        kw = dict(kws[s])
        modcod = kw.get("rate_idx", 3) + 1
        x, _ = signal(nframes=kw.pop("nframes", 3), lead=3, **kw)
        sym = model_symbols(x)
        if drops[s]:
            sym = np.concatenate([sym[:drops[s][0]], sym[drops[s][1]:]])
        fs = fx.FrameSync(modcod, kw["pilots"])
        tags, nout = fs.run(sym)
        fin = os.path.join(WORK, f"f2_{tag}_in{s}.hex")
        rxsim.write_iq(fin, sym)
        plus.update({f"in{s}": fin, f"out{s}": os.path.join(WORK, f"f2_{tag}_out{s}.hex"),
                     f"modcod{s}": modcod, f"pilots{s}": int(kw["pilots"]), f"thresh{s}": fs.T})
        want.append((sym[:nout], tags[:nout], fs.events))
    log = rxsim.run(exe, stall=stall, **plus)
    print(tag, log.strip())
    for s in range(2):
        sym, tags, ev = want[s]
        got = rxsim.read_hex_cols(plus[f"out{s}"], (-16, -16, 1, 1))
        assert len(got) == len(sym), (s, len(got), len(sym))
        assert np.array_equal(got[:, :2], sym)
        got_tag = np.where(got[:, 2] == 1, 1 + got[:, 3], 0)
        bad = np.nonzero(got_tag != tags)[0]
        assert len(bad) == 0, f"{tag} slot {s}: tag mismatch at {bad[:5]}"
        print(f"  slot {s}: {len(got)} symbols, {(tags != 0).sum()} headers equal the model; events {len(ev)}")


def main():
    exe = rxsim.build("tb_rx_framesync2", ["gen/dvbs2_tables_pkg.sv", "dvbs2_pl_pkg.sv", "dvbs2_rx_framesync2.sv",
                                           os.path.join(HERE, "tb_rx_framesync2.sv")])
    run_case(exe, "p_np", [dict(pilots=True, esn0=8.0), dict(pilots=False, esn0=3.0, rate_idx=6, seed=4)], 0)
    run_case(exe, "drop_stall", [dict(pilots=True, esn0=8.0, nframes=10),
                                 dict(pilots=True, esn0=0.0, rate_idx=0, seed=2)], 25,
             drops=((80000, 80100), None))


if __name__ == "__main__":
    main()
