"""Fixed-point receiver model on the exact TX model output through a channel.

TX: lead-in dummy PLFRAMEs (acquisition), data frames, trailing dummies.
Channel: rxmodel.channel (carrier offset, clock error, fractional delay,
AWGN), quantised to int16. Decoder: gr-dvbs2rx LDPC/BCH on our LLRs.
"""
import sys
import time

import numpy as np

import grref
import rxfixed as fx
import rxmodel as rf
import txmodel as m


def make_signal(rate_idx, pilots, nframes, cfo, ppm, delay, esn0, seed, lead=8, rms=1500.0):
    rate = m.RATES[rate_idx]
    kb = m.LDPC[rate]["kbch"]
    rng = np.random.default_rng(seed)
    ts = rng.integers(0, 256, ((nframes + 1) * kb // 1504 + 3, 188), dtype=np.uint8)
    ts[:, 0] = 0x47
    bb = grref.bbheader(ts.ravel(), rate)[: nframes * kb].reshape(nframes, kb)
    frames = [m.dummy_plframe_signs()] * lead + \
             [m.plframe_signs(m.fec_frame(b, rate_idx), rate_idx + 1, pilots) for b in bb] + \
             [m.dummy_plframe_signs()] * 2
    sps_tx = 8
    c = m.rrc_coefs(sps_tx, 0.35, 16)
    tx = m.rrc_interp(m.signs_to_iq(np.concatenate(frames)), c, sps_tx, 16)
    y = rf.channel(tx, sps_tx, 2, cfo, ppm, delay, esn0, seed)
    y *= rms / np.sqrt(np.mean(np.abs(y) ** 2) / 2)
    x = np.stack([np.round(y.real), np.round(y.imag)], axis=1)
    return np.clip(x, -32768, 32767).astype(np.int64), bb


def run(rate_idx=3, pilots=True, nframes=6, cfo=0.004, ppm=60.0, delay=0.37, esn0=6.0, seed=0, **kw):
    rate = m.RATES[rate_idx]
    kb = m.LDPC[rate]["kbch"]
    x, bb = make_signal(rate_idx, pilots, nframes, cfo, ppm, delay, esn0, seed, **kw)
    t0 = time.time()
    r = fx.receive(x, rate_idx + 1, pilots)
    t1 = time.time()
    frames = r["frames"]
    ev = r["fs"].events
    nfly = sum(1 for e in ev if e[0] == "flywheel")
    out = rf.gr_ldpc_decode_llr(np.array(frames), rate_idx) if frames else np.zeros(0, np.uint8)
    nout = len(out) // kb
    dec = [any(np.array_equal(out[k * kb:(k + 1) * kb], b) for k in range(nout)) for b in bb]
    cr = r["carrier"]
    f_est = cr.fhat / 2 ** 32
    print(f"{rate} pilots={pilots} cfo={cfo} ppm={ppm} Es/N0={esn0} dB: data frames out {len(frames)}, "
          f"decoded {sum(dec)}/{nframes} {''.join('x.'[d] for d in dec)}, flywheel {nfly}, "
          f"f {f_est:.5f}, AGC g {r['timing'].g}, A {cr.A} E {cr.E} G {cr.G}, {t1 - t0:.0f} s")
    return all(dec), r


CASES = [dict(), dict(esn0=2.0), dict(cfo=-0.01, ppm=-100), dict(rate_idx=0, esn0=-1.0),
         dict(rate_idx=10, esn0=8.0, pilots=False, cfo=0.0005),
         dict(rate_idx=7, esn0=6.0, pilots=False, cfo=-0.003, ppm=-80),
         dict(rate_idx=3, esn0=2.5, pilots=False)]

if __name__ == "__main__":
    sel = [int(a) for a in sys.argv[1:]] or range(len(CASES))
    good = True
    for i in sel:
        good &= run(**CASES[i])[0]
    sys.exit(0 if good else 1)
