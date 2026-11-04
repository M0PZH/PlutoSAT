"""Symbol-rate offset tolerance of the fixed-point receiver (rxfixed.py).

The AD9363 runs at 12 MS/s and the receiver's nominal symbol rate is
12e6 / (2 x decimation); QO-100 transmitters use integer kS/s rates
(333 kS/s -> decimation 18 = 333.333 kS/s, +1000 ppm; 66 kS/s -> 91 =
65.934 kS/s, -1000 ppm; 33 kS/s -> 182 = 32.967 kS/s, -1000 ppm). This
generates QPSK 1/2 PLFRAMEs with the TX model (8 samples/symbol), resamples
them to the ADC rate with a symbol-rate error e (the transmitter's symbol
rate is nominal x (1 + e)) by a 24-tap, 1024-phase windowed-sinc
interpolator, adds a residual carrier offset and AWGN over the whole ADC
band, quantises to 12 bits, runs the DDC (ddc_config(), as dvbs2-rx loads
it), then the receiver model, and decodes the LLRs with gr-dvbs2rx.

    ../.venv/bin/python sr_offset_experiment.py --dec 18 --ppm 1000 --frames 200 [--pilots]
    ../.venv/bin/python sr_offset_experiment.py --table            # the whole sweep (parallel)
    ../.venv/bin/python sr_offset_experiment.py --table --decs 3 --scale 0.06 --seeds 6 [--preset]
                                                                 # acquisition, 12 frames x 6 seeds

Reported per case: data frames decoded / sent, the first decoded frame
(acquisition), frames lost after it, frame-sync events (lost lock,
flywheel), header slips (consecutive detected data headers not exactly a
frame length apart), the timing integrator's steady state vs the expected
-2e / (1 + e) x 2^24 and the symbol at which its average settles.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

import grref
import rxfixed as fx
import rxmodel as rf
import txmodel as m

FS = 12e6
SPS_TX = 8
NPH_I = 1024            # interpolator phases
TAPS_I = np.arange(-11, 13)


def _interp_table():
    fc = 0.25                                   # cycles per TX sample (signal < 0.085)
    fr = np.arange(NPH_I + 1)[:, None] / NPH_I
    t = fr - TAPS_I[None, :]
    w = np.i0(9.0 * np.sqrt(np.clip(1 - (t / 12.5) ** 2, 0, 1))) / np.i0(9.0)
    h = 2 * fc * np.sinc(2 * fc * t) * w
    return h / h.sum(axis=1, keepdims=True)


H_I = _interp_table()


def tx_symbols(rate_idx, pilots, nframes, lead, seed):
    rate = m.RATES[rate_idx]
    kb = m.LDPC[rate]["kbch"]
    rng = np.random.default_rng(seed)
    ts = rng.integers(0, 256, ((nframes + 1) * kb // 1504 + 3, 188), dtype=np.uint8)
    ts[:, 0] = 0x47
    bb = grref.bbheader(ts.ravel(), rate)[: nframes * kb].reshape(nframes, kb)
    frames = [m.dummy_plframe_signs()] * lead + \
             [m.plframe_signs(m.fec_frame(b, rate_idx), rate_idx + 1, pilots) for b in bb] + \
             [m.dummy_plframe_signs()] * 2
    starts = np.cumsum([0] + [len(f) for f in frames])[:-1]
    return m.signs_to_iq(np.concatenate(frames)), bb, starts[lead:lead + nframes]


def adc_chunks(iq, dec, ppm, cfo, esn0, seed, delay=0.37, rms=400.0, chunk=1 << 20):
    """12-bit ADC samples (2 dec per nominal symbol) in chunks of `chunk`
    samples: the TX model output resampled with the symbol-rate error ppm,
    residual carrier offset cfo (cycles per nominal symbol), AWGN over the
    ADC band at Es/N0 = esn0 in the signal band."""
    coefs = m.rrc_coefs(SPS_TX, 0.35, 16)
    sps = 2 * dec
    nsym = len(iq)
    a = 1 + ppm * 1e-6
    n_out = int((nsym - 20) * sps / a)
    rng = np.random.default_rng(seed + 1000)
    # signal power per sample (float units of the TX output)
    seg = m.rrc_interp(iq[:4000], coefs, SPS_TX, 16)[200:]
    ps = np.mean(seg[:, 0].astype(float) ** 2 + seg[:, 1].astype(float) ** 2)
    var = ps * sps / 10 ** (esn0 / 10)
    scale = rms / math.sqrt((ps + var) / 2)
    for n0 in range(0, n_out, chunk):
        n = np.arange(n0, min(n_out, n0 + chunk))
        out = np.empty(len(n), np.complex128)
        for s in range(0, len(n), 1 << 17):
            nn = n[s:s + (1 << 17)]
            t = (nn / sps * a + delay) * SPS_TX
            i0 = np.floor(t).astype(np.int64)
            ph = np.rint((t - i0) * NPH_I).astype(np.int64)
            imin, imax = i0[0] + TAPS_I[0], i0[-1] + TAPS_I[-1]
            s0 = max(0, imin // SPS_TX - 16)
            s1 = min(nsym, imax // SPS_TX + 2)
            txs = m.rrc_interp(iq[s0:s1], coefs, SPS_TX, 16)
            txc = np.zeros(imax - imin + 1, np.complex128)
            lo, hi = max(imin, s0 * SPS_TX), min(imax + 1, s1 * SPS_TX)
            txc[lo - imin:hi - imin] = txs[lo - s0 * SPS_TX:hi - s0 * SPS_TX, 0] + \
                1j * txs[lo - s0 * SPS_TX:hi - s0 * SPS_TX, 1]
            idx = (i0 - imin)[:, None] + TAPS_I[None, :]
            out[s:s + len(nn)] = np.sum(txc[idx] * H_I[ph], axis=1)
        out *= np.exp(2j * np.pi * cfo / sps * n)
        out += math.sqrt(var / 2) * (rng.standard_normal(len(n)) + 1j * rng.standard_normal(len(n)))
        out *= scale
        x = np.stack([np.round(out.real), np.round(out.imag)], 1)
        yield np.clip(x, -2048, 2047).astype(np.int64)


def ddc_stream(chunks, cfg):
    """fx.ddc() over a chunked input (equal to one call on the whole input:
    each chunk is preceded by enough earlier samples for the CIC and FIR)."""
    R, D, h = cfg["R"], cfg["D"], cfg["coefs"]
    step = R * D
    ov = -(-(len(h) * R + 4 * R + step) // step) * step
    tail = np.zeros((0, 2), np.int64)
    first = True
    pending = np.zeros((0, 2), np.int64)
    outs = []
    for c in chunks:
        pending = np.vstack([pending, c])
        nuse = len(pending) // step * step
        if nuse == 0:
            continue
        seg = np.vstack([tail, pending[:nuse]])
        y = fx.ddc(seg, h, D, 0, cfg["shift"], R, cfg["cic_shift"])
        outs.append(y if first else y[len(tail) // step:])
        first = False
        tail = seg[-ov:] if len(seg) >= ov else seg
        if len(tail) % step:
            tail = tail[len(tail) % step:]
        pending = pending[nuse:]
    return np.vstack(outs)


class TimingRec(fx.Timing):
    """fx.Timing with the integrator recorded per symbol (int32 array)."""

    def run(self, x, trace=False):
        x = np.asarray(x, np.int64)
        out = []
        integ = []
        n = len(x)
        while self.k < n:
            k = self.k
            ph = self.mu >> (fx.MU_F - 5)
            on, mid = self._filter(x, k, ph)
            out.append(on)
            pi_, pq = self.prev
            e = (pi_ - on[0]) * mid[0] + (pq - on[1]) * mid[1]
            es = e >> fx.TED_SH
            self.prev = on
            self.integ += (es * self.ki) >> fx.KI_SH
            mu = self.mu + ((es * self.kp) >> fx.KP_SH) + self.integ
            st = mu >> fx.MU_F
            st = -1 if st < -1 else 1 if st > 1 else st
            self.mu = mu & ((1 << fx.MU_F) - 1)
            self.k = k + 2 + st
            p = on[0] * on[0] + on[1] * on[1]
            err = fx.sat((fx.P_TARGET - p) >> 9, 18)
            self.gacc += (self.g * err) >> fx.AGC_SH
            self.gacc = min(max(self.gacc, 1 << 15), (fx.G_MAX << 15) | 0x7FFF)
            self.g = self.gacc >> 15
            integ.append(self.integ)
        self.integ_trace = np.array(integ, np.int64)
        return np.array(out, np.int64).reshape(-1, 2)


def run_case(dec, ppm, pilots, nframes, esn0=2.0, cfo=0.004, seed=0, rate_idx=3, lead=0,
             preset=False):
    """preset: the timing integrator starts at the known offset
    (timing_integ_preset(), TIMING_INTEG write), else at 0."""
    t0 = time.time()
    iq, bb, starts = tx_symbols(rate_idx, pilots, nframes, lead, seed)
    cfg = fx.ddc_config(dec)
    x2 = ddc_stream(adc_chunks(iq, dec, ppm, cfo, esn0, seed, chunk=cfg["R"] * cfg["D"] * 8192), cfg)
    t1 = time.time()
    kp, ki = fx.timing_gains()
    integ0 = fx.timing_integ_preset(1 + ppm * 1e-6, 1.0) if preset else 0
    tr = TimingRec(fx.timing_coefs(0.35), kp, ki, integ0=integ0)
    sym = tr.run(x2)
    fs = fx.FrameSync(rate_idx + 1, pilots)
    cr = fx.Carrier(rate_idx + 1, pilots)
    fs.run(sym, cr)
    t2 = time.time()
    kb = m.LDPC[m.RATES[rate_idx]]["kbch"]
    dec_ok = np.zeros(nframes, bool)
    if cr.frames:
        out = rf.gr_ldpc_decode_llr(np.array(cr.frames), rate_idx)
        key = {bb[j][:256].tobytes(): j for j in range(nframes)}
        for k in range(len(out) // kb):
            o = out[k * kb:(k + 1) * kb]
            j = key.get(o[:256].tobytes())
            if j is not None and np.array_equal(o, bb[j]):
                dec_ok[j] = True
    first = int(np.argmax(dec_ok)) if dec_ok.any() else None
    lost_after = int(np.sum(~dec_ok[first:])) if first is not None else nframes
    ev = fs.events
    # header slips: the detected data headers' offset from the transmitted
    # header positions (detections within 50 symbols of one), changes of it
    found = [e[1] for e in ev if e[0] == "found" and e[2] == 0]
    offs = []
    for n_ in found:
        j = int(np.argmin(np.abs(starts - n_)))
        if abs(n_ - starts[j]) <= 50:
            offs.append(int(n_ - starts[j]))
    slips = int(sum(abs(b_ - a_) for a_, b_ in zip(offs, offs[1:])))
    integ = tr.integ_trace
    exp_integ = -2 * ppm * 1e-6 / (1 + ppm * 1e-6) * 2 ** 24
    half = integ[len(integ) // 2:]
    # settling: 1024-symbol average within 10 % of the expected value (or
    # 2 x 2^24 x 100e-6 = 3355, 100 ppm, whichever is larger) from then on
    tol = max(0.1 * abs(exp_integ), 3355)
    nb = len(integ) // 1024
    avg = integ[:nb * 1024].reshape(nb, 1024).mean(axis=1)
    bad = np.nonzero(np.abs(avg - exp_integ) > tol)[0]
    settle = int((bad[-1] + 1) * 1024) if len(bad) else 0
    res = dict(dec=dec, ppm=ppm, pilots=pilots, esn0=esn0, nframes=nframes, preset=preset, seed=seed,
               decoded=int(dec_ok.sum()), first=first, lost_after=lost_after,
               first_sym=int(starts[first]) if first is not None else None,
               lost_events=sum(1 for e in ev if e[0] == "lost"),
               flywheel=sum(1 for e in ev if e[0] == "flywheel"), slips=slips,
               offsets=sorted(set(offs)),
               integ_mean=float(half.mean()), integ_std=float(half.std()),
               integ_exp=exp_integ, integ_maxabs=int(np.abs(integ).max()),
               settle_sym=settle if settle < len(integ) else None, n_acq=cr.n_acq,
               pattern="".join("x."[int(d)] for d in dec_ok),
               t_gen=round(t1 - t0), t_rx=round(t2 - t1))
    return res


def fmt(r):
    return (f"dec {r['dec']:3d} {r['ppm']:+6d} ppm {'P' if r['pilots'] else 'N'} {r['esn0']} dB"
            f"{' preset' if r['preset'] else ''} seed {r['seed']}: "
            f"decoded {r['decoded']}/{r['nframes']}, first {r['first']}, lost after {r['lost_after']}, "
            f"lost-lock {r['lost_events']} flywheel {r['flywheel']} slips {r['slips']} acq {r['n_acq']}, "
            f"integ {r['integ_mean']:.0f} +- {r['integ_std']:.0f} (exp {r['integ_exp']:.0f}, "
            f"max {r['integ_maxabs']}), settled at {r['settle_sym']}, {r['t_gen']}+{r['t_rx']} s")


NFRAMES = {3: 200, 18: 200, 91: 100, 182: 100}
PPMS = [0, 200, -200, 500, -500, 1000, -1000, 2000, -2000, 5000, -5000]


def _job(kw):
    try:
        r = run_case(**kw)
    except Exception as exc:      # noqa: BLE001
        r = dict(kw, error=repr(exc))
    print(fmt(r) if "error" not in r else r, flush=True)
    return r


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dec", type=int, default=18)
    ap.add_argument("--ppm", type=int, default=1000)
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--pilots", action="store_true")
    ap.add_argument("--esn0", type=float, default=2.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--table", action="store_true")
    ap.add_argument("--decs", default="3,18,91,182")
    ap.add_argument("--ppms", default=",".join(map(str, PPMS)))
    ap.add_argument("--jobs", type=int, default=40)
    ap.add_argument("--out", default="sr_offset.json")
    ap.add_argument("--scale", type=float, default=1.0, help="frames per case x this (--table)")
    ap.add_argument("--seeds", type=int, default=1, help="seeds per case (--table)")
    ap.add_argument("--preset", action="store_true", help="timing integrator preset to the offset")
    a = ap.parse_args()
    if not a.table:
        print(fmt(run_case(a.dec, a.ppm, a.pilots, a.frames, a.esn0, seed=a.seed, preset=a.preset)))
        sys.exit(0)
    from multiprocessing import Pool
    jobs = [dict(dec=d, ppm=p, pilots=pl, nframes=max(4, int(NFRAMES[d] * a.scale)), esn0=a.esn0,
                 seed=a.seed + sd, preset=a.preset)
            for sd in range(a.seeds) for d in map(int, a.decs.split(",")) for pl in (True, False) for p in map(int, a.ppms.split(","))]
    jobs.sort(key=lambda j: -j["nframes"] * (j["dec"] + 20))
    with Pool(min(a.jobs, len(jobs))) as pool:
        res = pool.map(_job, jobs, chunksize=1)
    with open(a.out, "w") as f:
        json.dump(res, f, indent=1)
