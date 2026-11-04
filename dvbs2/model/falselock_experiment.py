"""Frame sync false lock on repeating frames (null-packet TS).

A TS of null packets gives near-identical BBFRAMEs (the same BBHEADER, the
same payload bytes, only the packet headers moving with SYNCD): after BB
scrambling, BCH, LDPC and PL scrambling consecutive PLFRAMEs share most of
their symbols, so any data window correlates with itself exactly a frame
length later. The receiver starts at a random point of the stream (as after a
soft reset on the board) and we record where the frame sync locks.
"""
import argparse
import time

import numpy as np

import grref
import rxfixed as fx
import rxmodel as rf
import txmodel as m


def null_ts(npkt, fill=0xFF):
    ts = np.full((npkt, 188), fill, np.uint8)
    ts[:, 0], ts[:, 1], ts[:, 2], ts[:, 3] = 0x47, 0x1F, 0xFF, 0x10
    return ts


def prbs_ts(npkt, seed):
    ts = np.random.default_rng(seed).integers(0, 256, (npkt, 188), dtype=np.uint8)
    ts[:, 0] = 0x47
    return ts


def tx_signal(rate_idx, pilots, nframes, payload="null", seed=0, sps_tx=8):
    kb = m.LDPC[m.RATES[rate_idx]]["kbch"]
    npkt = (nframes + 1) * kb // 1504 + 3
    ts = null_ts(npkt) if payload == "null" else null_ts(npkt, 0) if payload == "zero" \
        else prbs_ts(npkt, seed)
    bb = grref.bbheader(ts.ravel(), m.RATES[rate_idx])[: nframes * kb].reshape(nframes, kb)
    frames = [m.plframe_signs(m.fec_frame(b, rate_idx), rate_idx + 1, pilots) for b in bb]
    c = m.rrc_coefs(sps_tx, 0.35, 16)
    return m.rrc_interp(m.signs_to_iq(np.concatenate(frames)), c, sps_tx, 16), bb


def rx_input(tx, start_sym, esn0, cfo, ppm, seed, sps_tx=8, rms=1500.0):
    y = rf.channel(tx[start_sym * sps_tx:], sps_tx, 2, cfo, ppm, 0.37, esn0, seed)
    y *= rms / np.sqrt(np.mean(np.abs(y) ** 2) / 2)
    x = np.stack([np.round(y.real), np.round(y.imag)], axis=1)
    return np.clip(x, -32768, 32767).astype(np.int64)


def classify(r, L, start_sym):
    """Header tags of the frame sync: right (a frame start, +-1 symbol) or wrong."""
    out = []
    for e in r["fs"].events:
        if e[0] in ("found", "flywheel"):
            ph = (e[1] + start_sym) % L
            d = min(ph, L - ph)
            out.append((e[0], e[1], d <= 1))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=int, default=10)
    ap.add_argument("--pilots", type=int, default=1)
    ap.add_argument("--frames", type=int, default=10)
    ap.add_argument("--payload", default="null")
    ap.add_argument("--esn0", type=float, default=20.0)
    ap.add_argument("--cfo", type=float, default=0.0)
    ap.add_argument("--ppm", type=float, default=0.0)
    ap.add_argument("--starts", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--rms", type=float, default=1500.0)
    a = ap.parse_args()
    L = fx.frame_len(a.pilots)
    tx, bb = tx_signal(a.rate, bool(a.pilots), a.frames, a.payload, a.seed)
    rng = np.random.default_rng(a.seed)
    nbad = 0
    for i in range(a.starts):
        start = int(rng.integers(0, L))
        x = rx_input(tx, start, a.esn0, a.cfo, a.ppm, a.seed + i, rms=a.rms)
        t0 = time.time()
        r = fx.receive(x, a.rate + 1, bool(a.pilots))
        cl = classify(r, L, start)
        first_right = next((k for k, (ty, n, ok) in enumerate(cl) if ty == "found" and ok), None)
        wrong = sum(1 for ty, n, ok in cl if not ok)
        st = r["pls"].status()
        d = st["data"]
        bad = first_right is None or wrong > 0
        nbad += bad
        print(f"start {start:6d}: tags {len(cl)}, wrong {wrong}, first right #{first_right}, "
              f"carrier frames {len(r['frames'])}, PLS conf data {st['data_count']}/{st['count']} "
              f"({d['modcod'] if d else '-'}), AGC g {r['timing'].g}, {time.time() - t0:.0f} s")
        print("   ", " ".join(f"{ty[0]}{n}{'' if ok else '!'}" for ty, n, ok in cl))
    print(f"{nbad}/{a.starts} starts with a wrong tag")


if __name__ == "__main__":
    main()
