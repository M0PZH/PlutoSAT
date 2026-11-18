"""MODCOD detection with a wrong hypothesis configured (8PSK, short frames).

MODCOD auto (station) and `dvbs2-rx --detect` configure the slot to a QPSK
normal-frame hypothesis and wait for the PLS decoder to name the code of the
headers the frame sync tags. This runs a stream of PLFRAMEs of one code
(random data, optionally dummy frames in between) through the channel and
the fixed-point receiver configured to another code, and records every PLS
decode with its ground truth: a real header (the tag within +-2 symbols of a
frame start) or not. Reported: the first confident decode of the true code
(symbols after the start, frames), the confident decodes naming another code
("false"), and the metric distributions.

    python plsdetect_experiment.py --modcod 13 --short 1 --pilots 1 --esn0 7 --starts 8
    python plsdetect_experiment.py --table          # the whole sweep (parallel)
"""
import argparse
import json
import os
import sys
import time

import numpy as np

import rxfixed as fx
import rxmodel as rf
import txmodel as m


def stream_phases(modcod, short, pilots, nframes, seed, dummy_every=0, lead_dummies=0):
    """Phase indices of nframes data PLFRAMEs with random FECFRAME bits (a
    dummy frame after every dummy_every data frames, lead_dummies first) and
    the frame start symbols."""
    rng = np.random.default_rng(seed)
    n = m.nldpc(short)
    frames, starts, codes = [], [], []
    pos = 0

    def add(ph, code):
        nonlocal pos
        frames.append(ph)
        starts.append(pos)
        codes.append(code)
        pos += len(ph)

    for _ in range(lead_dummies):
        add(m.dummy_plframe_phases(), 0)
    for k in range(nframes):
        bits = rng.integers(0, 2, n).astype(np.uint8)
        add(m.plframe_phases(bits, modcod, pilots, 0, short), fx.pls_code(modcod, short, pilots))
        if dummy_every and (k + 1) % dummy_every == 0:
            add(m.dummy_plframe_phases(), 0)
    return np.concatenate(frames), np.array(starts), np.array(codes)


def channel_awgn(iq, esn0, cfo, ppm, seed, delay=0.37, rms=1500.0):
    sps_tx = 8
    c = m.rrc_coefs(sps_tx, 0.35, 16)
    tx = m.rrc_interp(iq, c, sps_tx, 16)
    y = rf.channel(tx, sps_tx, 2, cfo, ppm, delay, esn0, seed)
    y *= rms / np.sqrt(np.mean(np.abs(y) ** 2) / 2)
    x = np.stack([np.round(y.real), np.round(y.imag)], axis=1)
    return np.clip(x, -32768, 32767).astype(np.int64)


def channel_adc(iq, esn0, cfo, seed, dec=6, rms=1500.0):
    """The board's front end: 12-bit ADC (AGC from far above), DDC."""
    import sr_offset_experiment as so
    cfg = fx.ddc_config(dec)
    delay = np.random.default_rng(50 + seed).random()
    return so.ddc_stream(so.adc_chunks(iq, dec, 0.0, cfo, esn0, seed, delay=delay, rms=rms,
                                       chunk=cfg["R"] * cfg["D"] * 8192), cfg)


class RecPls(fx.PlsDecoder):
    """PlsDecoder that records the symbol index of each decoded tag."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.k = 0
        self.tag_at = []

    def step(self, si, sq, t, fhat, acquired, probe=0):
        if (t or probe) and self.buf is None:
            self.tag_at.append(self.k)
        elif t or probe:
            self.tag_at[-1:] = [self.k]   # restarted (the model has no skip)
        super().step(si, sq, t, fhat, acquired, probe)
        self.k += 1


def header_positions(sym, codes_present):
    """Header starts in the receiver's symbol stream: peaks of the
    differential correlation with each code present (noise-free enough at
    the SNRs used: the ground truth is taken from a noise-free run)."""
    best = None
    for c in codes_present:
        met = fx.fs_metrics(sym, fx.diff_coefs(c >> 2, c & 1, (c >> 1) & 1))
        best = met if best is None else np.maximum(best, met)
    thr = best.max() / 4
    pk = np.nonzero((best > thr) & (best >= np.roll(best, 1)) & (best >= np.roll(best, -1)))[0]
    return pk - 89


def run(modcod, short, pilots, esn0, seed, cfg=(6, 1, 0), nframes=12, cfo=0.0, ppm=0.0,
        dummy_every=0, chain="awgn", start=None, verbose=False, probe_k=None, conf=None):
    """One start: returns a dict with the decodes and their ground truth."""
    ph, fstarts, _ = stream_phases(modcod, short, pilots, nframes + 1, seed, dummy_every)
    rng = np.random.default_rng(seed + 77)
    s0 = int(rng.integers(0, fstarts[1])) if start is None else start
    iq = m.phases_to_iq(ph[s0:])
    if chain == "adc":
        x = channel_adc(iq, esn0, cfo, seed)
    else:
        x = channel_awgn(iq, esn0, cfo, ppm, seed)
    cmod, cpil, csh = cfg
    kp, ki = fx.timing_gains(fx.TIMING_BW)
    tr = fx.Timing(fx.timing_coefs(0.35), kp, ki)
    sym = tr.run(x)
    fs = fx.FrameSync(cmod, cpil, short=csh)
    if probe_k is not None:
        fs.probe_k = probe_k
    cr = fx.Carrier(cmod, cpil, 0, short=csh)
    cr.pls = RecPls(cmod, cpil, short=csh)
    fs.run(sym, cr)
    # ground truth: the frame starts in sym (timing delay from the noise-free run)
    true_code = fx.pls_code(modcod, short, pilots)
    if not hasattr(run, "_cache"):
        run._cache = {}
    key = (modcod, short, pilots, seed, s0, nframes, dummy_every, chain, cfo)
    if key not in run._cache:
        if chain == "adc":
            xc = channel_adc(iq, 60.0, cfo, seed)
        else:
            xc = channel_awgn(iq, 60.0, cfo, ppm, seed)
        tr0 = fx.Timing(fx.timing_coefs(0.35), kp, ki)
        run._cache[key] = header_positions(tr0.run(xc), [true_code, 0])
    hpos = run._cache[key]
    flen = fx.frame_len(pilots, modcod, short)
    dec = []
    for k, d in zip(cr.pls.tag_at, cr.pls.results):
        real = bool(np.any(np.abs(hpos - k) <= 2))
        c = bool(d["conf"]) if conf is None else bool(conf(d, cr.pls))
        dec.append(dict(at=int(k), real=real, code=d["code"], conf=c, best=d["best"],
                        second=d["second"], freq=d["freq"], probe=d["probe"], **{kk: d[kk] for kk in d
                                                                                   if kk.startswith("x_")}))
    first = next((d["at"] for d in dec if d["conf"] and d["code"] == true_code), None)
    nfalse = sum(1 for d in dec if d["conf"] and d["code"] not in (true_code, 0))
    res = dict(modcod=modcod, short=short, pilots=pilots, esn0=esn0, seed=seed, cfg=cfg, start=s0,
               nsym=len(sym), flen=flen, first=first, first_frames=None if first is None else first / flen,
               nfalse=nfalse, ndec=len(dec), nreal=sum(d["real"] for d in dec), dec=dec,
               drops=fs.n_pls_drop, nprobe=int(fs.probe.sum()), acq=cr.n_acq if hasattr(cr, "n_acq") else None)
    if verbose:
        print(f"MODCOD {modcod} short {short} pilots {pilots} {esn0} dB seed {seed}: decodes {len(dec)} "
              f"(real {res['nreal']}), first right {res['first_frames']}, false confident {nfalse}")
        for d in dec:
            print(f"   {d['at']:7d} {'R' if d['real'] else '-'}{'p' if d['probe'] else ' '} code {d['code']:3d} "
                  f"({d['code'] >> 2}/{(d['code'] >> 1) & 1}/{d['code'] & 1}) {'C' if d['conf'] else ' '} "
                  f"best {d['best']:5d} second {d['second']:5d} f {d['freq']}")
    return res


def pls_windows(modcod, short, pilots, esn0, seed, nframes=6, chain="awgn", ndata=400, cfo=0.0, skip=20000):
    """PLS decodes (grid, no carrier frequency) of the real headers and of
    ndata random data windows of a stream, after `skip` symbols (the AGC
    settled): [(real, code, best, second, best other code at any frequency,
    E (strip units, 2 Er), best of the code at any frequency)]."""
    ph, _, _ = stream_phases(modcod, short, pilots, nframes, seed)
    iq = m.phases_to_iq(ph)
    kp, ki = fx.timing_gains(fx.TIMING_BW)
    sym, sym0 = [fx.Timing(fx.timing_coefs(0.35), kp, ki).run(
        channel_adc(iq, e, cfo, seed) if chain == "adc" else channel_awgn(iq, e, cfo, 0.0, seed))
        for e in (esn0, 60.0)]
    hp = header_positions(sym0, [fx.pls_code(modcod, short, pilots), 0])
    hp = hp[hp > skip]
    rng = np.random.default_rng(seed)
    grid = [(h * fx.PLS_STEP_DEFAULT) & 0xFFFF for h in range(-fx.PLS_H_DEFAULT, fx.PLS_H_DEFAULT + 1)]
    out = []

    def one(k, real):
        r = np.array([(fx.pls_quant(s[0]), fx.pls_quant(s[1])) for s in sym[k:k + 90]])
        mets = fx.pls_metrics(r, grid)
        best, bi, second = fx.pls_select(mets)
        c = bi % 128
        xi, xq = fx.pls_strip(r)
        out.append((real, int(fx._PLS_CODES[c]), best, second, int(np.delete(mets, c, axis=1).max()),
                    int((xi * xi + xq * xq).sum()), int(mets[:, c].max())))

    for k in hp:
        if k + 90 < len(sym):
            one(k, 1)
    for _ in range(ndata):
        k = int(rng.integers(skip, len(sym) - 100))
        if np.all(np.abs(hp - k) > 100):
            one(k, 0)
    return out


# codes near their thresholds (Es/N0, dB): (modcod, short, esn0)
CASES = [
    (1, 0, -2.0), (6, 0, 3.5), (11, 0, 7.0),
    (1, 1, -2.0), (6, 1, 3.5), (10, 1, 7.0),
    (12, 0, 6.0), (13, 0, 7.0), (17, 0, 11.0),
    (12, 1, 6.0), (13, 1, 7.0), (16, 1, 10.5),
]


def _job(a):
    modcod, short, pilots, esn0, seed, kw = a
    t0 = time.time()
    r = run(modcod, short, pilots, esn0, seed, **kw)
    r["secs"] = time.time() - t0
    return r


def table(args):
    from multiprocessing import Pool
    jobs = []
    for (mc, sh, e) in CASES:
        for p in (0, 1):
            for s in range(args.starts):
                kw = dict(cfg=(args.cfg_modcod, p if args.cfg_pilots < 0 else args.cfg_pilots, 0),
                          nframes=args.frames, cfo=args.cfo, chain=args.chain,
                          dummy_every=args.dummy_every, probe_k=args.probe)
                jobs.append((mc, sh, p, e + args.dsnr, 100 + s, kw))
    with Pool(args.jobs) as pool:
        res = pool.map(_job, jobs, chunksize=1)
    if args.out:
        with open(args.out, "w") as f:
            json.dump(res, f)
    summarize(res)


def summarize(res):
    groups = {}
    for r in res:
        groups.setdefault((r["modcod"], r["short"], r["pilots"], r["esn0"]), []).append(r)
    print("MODCOD short pilots Es/N0 | detected | frames to first (median/max) | false conf / decodes "
          "| real tags | false-conf best max")
    tf = tn = 0
    for k, rs in sorted(groups.items()):
        det = [r for r in rs if r["first"] is not None]
        fr = sorted(r["first_frames"] for r in det)
        nf = sum(r["nfalse"] for r in rs)
        nd = sum(r["ndec"] for r in rs)
        nreal = sum(r["nreal"] for r in rs)
        npr = sum(r["nprobe"] for r in rs)
        fb = [d["best"] for r in rs for d in r["dec"] if d["conf"] and not d["real"]]
        tf += nf
        tn += nd
        print(f"{k[0]:3d} {k[1]} {k[2]} {k[3]:5.1f} | {len(det)}/{len(rs)} | "
              f"{(fr[len(fr) // 2] if fr else float('nan')):5.1f} / {(fr[-1] if fr else float('nan')):5.1f} | "
              f"{nf} / {nd} | {nreal} (probes {npr}) | {max(fb) if fb else '-'}")
    print(f"total false confident {tf} of {tn} decodes")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--modcod", type=int, default=13)
    ap.add_argument("--short", type=int, default=0)
    ap.add_argument("--pilots", type=int, default=1)
    ap.add_argument("--esn0", type=float, default=7.0)
    ap.add_argument("--cfg-modcod", type=int, default=6)
    ap.add_argument("--cfg-pilots", type=int, default=-1, help="-1: the signal's pilots")
    ap.add_argument("--frames", type=int, default=12)
    ap.add_argument("--starts", type=int, default=4)
    ap.add_argument("--cfo", type=float, default=0.0)
    ap.add_argument("--chain", default="awgn")
    ap.add_argument("--dummy-every", type=int, default=0)
    ap.add_argument("--dsnr", type=float, default=0.0)
    ap.add_argument("--table", action="store_true")
    ap.add_argument("--probe", type=int, default=None)
    ap.add_argument("--jobs", type=int, default=os.cpu_count() // 2)
    ap.add_argument("--out")
    a = ap.parse_args()
    if a.table:
        return table(a)
    for s in range(a.starts):
        run(a.modcod, a.short, a.pilots, a.esn0, 100 + s,
            cfg=(a.cfg_modcod, a.pilots if a.cfg_pilots < 0 else a.cfg_pilots, 0), nframes=a.frames,
            cfo=a.cfo, chain=a.chain, dummy_every=a.dummy_every, verbose=True, probe_k=a.probe)


if __name__ == "__main__":
    sys.exit(main())
