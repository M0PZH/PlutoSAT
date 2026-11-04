"""PROTOTYPE (rx2 study): two-slot timing engine vs the model, per slot, bit exact.

    cd sim/rx && ../../.venv/bin/python test_rx2.py
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "model"))
sys.path.insert(0, HERE)

import rxfixed as fx            # noqa: E402
import rxfixed_experiment as ex  # noqa: E402
import rxsim                     # noqa: E402

WORK = os.path.join(rxsim.BUILD, "work2")


def coef_file(path, coefs):
    with open(path, "w") as f:
        for p in range(32):
            for j in range(32):
                f.write(f"{int(coefs[p, j]) & 0x3FFFF if j < fx.NT else 0:05x}\n")


def run_case(exe, tag, sig, stall, rolloff, gains):
    os.makedirs(WORK, exist_ok=True)
    plus = {}
    want = []
    for s in range(2):
        x, _ = ex.make_signal(sig[s]["rate_idx"], True, 0, sig[s]["cfo"], sig[s]["ppm"], sig[s]["delay"],
                              8.0, sig[s]["seed"], lead=sig[s]["lead"], rms=sig[s]["rms"])
        coefs = fx.timing_coefs(rolloff[s])
        kp, ki = gains[s]
        want.append(fx.Timing(coefs, kp, ki).run(x))
        cf = os.path.join(WORK, f"{tag}_coef{s}.hex")
        coef_file(cf, coefs)
        fin = os.path.join(WORK, f"{tag}_in{s}.hex")
        rxsim.write_iq(fin, x)
        plus.update({f"in{s}": fin, f"out{s}": os.path.join(WORK, f"{tag}_out{s}.hex"), f"coef{s}": cf,
                     f"kp{s}": kp, f"ki{s}": ki, f"stall{s}": stall[s]})
    log = rxsim.run(exe, **plus)
    print(tag, log.strip())
    for s in range(2):
        got = rxsim.read_hex_cols(plus[f"out{s}"], (-16, -16))
        n = min(len(got), len(want[s]))
        assert n > len(want[s]) - 40, (s, len(got), len(want[s]))
        bad = np.nonzero(np.any(got[:n] != want[s][:n], axis=1))[0]
        assert len(bad) == 0, f"{tag} slot {s}: first mismatch at symbol {bad[0]}"
        print(f"  slot {s}: {n} symbols equal the model")


def main():
    exe = rxsim.build("tb_rx_timing2", ["dvbs2_rx_timing2.sv", os.path.join(HERE, "tb_rx_timing2.sv")])
    kp, ki = fx.timing_gains()
    a = dict(rate_idx=3, cfo=0.004, ppm=60.0, delay=0.37, seed=1, lead=6, rms=1500.0)
    b = dict(rate_idx=1, cfo=-0.01, ppm=-100.0, delay=0.81, seed=7, lead=4, rms=300.0)
    # both slots at full speed: the engine is the bottleneck (shared 42 clocks/symbol)
    run_case(exe, "full", [a, b], (0, 0), (0.35, 0.20), [(kp, ki), (kp, ki)])
    # stalls, different gains
    run_case(exe, "stall", [a, b], (30, 5), (0.20, 0.35), [(kp, ki), (kp * 2, ki // 2)])


if __name__ == "__main__":
    main()
