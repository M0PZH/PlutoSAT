"""Carrier recovery at low Es/N0, pilots off vs on: frame error rate of the
fixed-point carrier model + the LDPC decoder model on symbol-level signals.

The carrier block is fed directly (ideal timing and frame sync: the symbols
at the AGC level, a tag on every header), so only the carrier recovery is
measured. Channel: carrier offset (cycles/symbol), Wiener phase noise
(sigma rad/symbol: phase(k+1) = phase(k) + N(0, sigma^2)) and AWGN.

  L(f) of a Wiener phase = sigma^2 Rs / (4 pi^2 f^2): sigma = 1e-3 at 1 MS/s
  is -76 dBc/Hz at 1 kHz, sigma = 3e-3 at 333 kS/s -71 dBc/Hz.

Baseline: the same LDPC decoder on ideal LLRs (ldpcdec.awgn_llr).

Each seed: 16 dummy frames (acquisition), --frames data frames, a dummy.

Run:  python pilotless_experiment.py [--rate 0] [--pn 0.001] [--cfo 0.001]
      [--esn0 -2,-1,0] [--pilots 0,1] [--seeds 16] [--frames 6] [--jobs 24]
      [--ideal N] [--cls module:Class --kw "{...}"]   (a Carrier variant)
"""
import argparse
import math
import multiprocessing as mp
import time

import numpy as np

import ldpcdec as ld
import rxfixed as fx
import txmodel as m


def make_symbols(rate_idx, pilots, nframes, esn0, cfo, pn, seed, lead=16, dummy_every=0, gold=0):
    """Symbols at the carrier input (int, per-axis RMS 4096 incl. noise), tags,
    found, the code words of the data frames and the true phase per symbol."""
    rng = np.random.default_rng(seed)
    kb = m.LDPC[m.RATES[rate_idx]]["kbch"]
    frames, tags, cws = [], [], []
    for _ in range(lead):
        frames.append(m.dummy_plframe_signs(gold))
        tags.append(2)
    for k in range(nframes):
        bb = rng.integers(0, 2, kb).astype(np.uint8)
        cw = m.fec_frame(bb, rate_idx)
        cws.append(cw)
        frames.append(m.plframe_signs(cw, rate_idx + 1, pilots, gold))
        tags.append(1)
        if dummy_every and (k + 1) % dummy_every == 0:
            frames.append(m.dummy_plframe_signs(gold))
            tags.append(2)
    frames.append(m.dummy_plframe_signs(gold))
    tags.append(2)
    t = np.zeros(sum(len(f) for f in frames), np.int64)
    pos = 0
    for f, ty in zip(frames, tags):
        t[pos] = ty
        pos += len(f)
    sg = np.concatenate(frames)
    s = ((1 - 2 * sg[:, 0].astype(float)) + 1j * (1 - 2 * sg[:, 1].astype(float))) / math.sqrt(2)
    n = len(s)
    ph = 2 * np.pi * cfo * np.arange(n) + rng.uniform(0, 2 * np.pi)
    if pn > 0:
        ph += np.cumsum(rng.standard_normal(n) * pn)
    nvar = 10 ** (-esn0 / 10)
    y = s * np.exp(1j * ph) + np.sqrt(nvar / 2) * (rng.standard_normal(n) + 1j * rng.standard_normal(n))
    y *= math.sqrt(fx.P_TARGET / (1 + nvar))
    x = np.stack([np.round(y.real), np.round(y.imag)], axis=1).astype(np.int64)
    return x, t, np.ones(n, np.int64), cws, ph


def run_one(args):
    (rate_idx, pilots, nframes, esn0, cfo, pn, seed, opts) = args
    x, t, found, cws, _ = make_symbols(rate_idx, pilots, nframes, esn0, cfo, pn, seed,
                                       dummy_every=opts.get("dummy_every", 0))
    cls = fx.Carrier
    if opts.get("cls"):
        import importlib
        mod, name = opts["cls"].split(":")
        cls = getattr(importlib.import_module(mod), name)
    cr = cls(rate_idx + 1, pilots, **opts.get("carrier", {}))
    frames = cr.run(x, t, found)
    dec = ld.Decoder(rate_idx)
    ok = []
    for k, cw in enumerate(cws):
        if k >= len(frames):
            ok.append(False)
            continue
        hard, it, conv = dec.decode(frames[k])
        ok.append(bool(np.array_equal(hard, cw)))
    return ok, dict(n_acq=cr.n_acq, fhat=cr.fhat)


def run_ideal(args):
    rate_idx, esn0, seed = args
    rng = np.random.default_rng(seed + 1000)
    kb = m.LDPC[m.RATES[rate_idx]]["kbch"]
    dec = ld.Decoder(rate_idx)
    bb = rng.integers(0, 2, kb).astype(np.uint8)
    cw = m.fec_frame(bb, rate_idx)
    hard, it, conv = dec.decode(ld.awgn_llr(cw, esn0, rng))
    return bool(np.array_equal(hard, cw))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=int, default=0)
    ap.add_argument("--pn", type=float, default=0.0)
    ap.add_argument("--cfo", type=float, default=0.0)
    ap.add_argument("--esn0", default="-2,-1,0")
    ap.add_argument("--pilots", default="0,1")
    ap.add_argument("--seeds", type=int, default=16)
    ap.add_argument("--frames", type=int, default=6)
    ap.add_argument("--jobs", type=int, default=24)
    ap.add_argument("--ideal", type=int, default=0, help="also N ideal-sync frames per point")
    ap.add_argument("--dummy-every", type=int, default=0)
    ap.add_argument("--cls", default="", help="module:Class of a Carrier variant")
    ap.add_argument("--kw", default="{}", help="Carrier keyword arguments (Python literal)")
    a = ap.parse_args()
    es = [float(v) for v in a.esn0.split(",")]
    pls = [int(v) for v in a.pilots.split(",")]
    import ast
    opts = dict(dummy_every=a.dummy_every, cls=a.cls, carrier=ast.literal_eval(a.kw))
    with mp.Pool(a.jobs) as pool:
        for e in es:
            line = f"{m.RATES[a.rate]} Es/N0 {e:+.1f} dB cfo {a.cfo} pn {a.pn}:"
            for p in pls:
                t0 = time.time()
                res = pool.map(run_one, [(a.rate, bool(p), a.frames, e, a.cfo, a.pn, s, opts)
                                         for s in range(a.seeds)])
                oks = [o for r in res for o in r[0]]
                nacq = sum(r[1]["n_acq"] for r in res)
                line += f"  pilots {p}: FER {oks.count(False)}/{len(oks)} (acq {nacq}, {time.time() - t0:.0f} s)"
            if a.ideal:
                oks = pool.map(run_ideal, [(a.rate, e, s) for s in range(a.ideal)])
                line += f"  ideal: FER {oks.count(False)}/{len(oks)}"
            print(line, flush=True)


if __name__ == "__main__":
    main()
