"""Run the RX front-end model on the exact TX model output through a channel."""
import sys
import time

import numpy as np

import grref
import rxmodel as rx
import txmodel as m


def run(rate_idx=3, pilots=True, nframes=6, cfo=0.004, ppm=60.0, delay=0.37, esn0=6.0, seed=0, thresh=0.3, track=None):
    rate = m.RATES[rate_idx]
    kb = m.LDPC[rate]["kbch"]
    rng = np.random.default_rng(seed)
    ts = rng.integers(0, 256, ((nframes + 1) * kb // 1504 + 3, 188), dtype=np.uint8)
    ts[:, 0] = 0x47
    bb = grref.bbheader(ts.ravel(), rate)[: nframes * kb].reshape(nframes, kb)
    frames = [m.dummy_plframe_signs()] + \
             [m.plframe_signs(m.fec_frame(b, rate_idx), rate_idx + 1, pilots) for b in bb] + \
             [m.dummy_plframe_signs()]
    sps_tx = 8
    c = m.rrc_coefs(sps_tx, 0.35, 16)
    tx = m.rrc_interp(m.signs_to_iq(np.concatenate(frames)), c, sps_tx, 16)
    y = rx.channel(tx, sps_tx, 2, cfo, ppm, delay, esn0, seed)
    y /= np.sqrt(np.mean(np.abs(y) ** 2))                      # AGC
    t0 = time.time()
    sym = rx.timing_recovery(y, 0.35)
    starts = rx.find_frames(sym, rate_idx + 1, pilots, thresh=thresh)
    f_est = rx.header_cfo(sym, starts, pilots, rate_idx + 1)
    if track is None:
        # pilots: per-frame pilot interpolation; no pilots: continuous DD PLL
        track = not pilots
    if track:
        ref = rx.known_symbols(len(sym), starts, pilots, rate_idx + 1)
        sym = rx.carrier_track(sym, ref, f_est, dd=not pilots)
    else:
        sym = sym * np.exp(-2j * np.pi * f_est * np.arange(len(sym)))
    data, llrs = [], []
    for st, dummy in starts:
        if dummy:
            continue
        r = rx.recover_frame(sym, st, pilots, modcod=rate_idx + 1, tracked=track)
        if r is not None:
            data.append(r[0])
            llrs.append(rx.soft_demap(r[0], r[2]))
    t1 = time.time()
    found = sum(1 for _, d in starts if not d)
    out = rx.gr_ldpc_decode_llr(np.array(llrs), rate_idx) if llrs else np.zeros(0, np.uint8)
    nout = len(out) // kb
    ok = sum(np.array_equal(out[k * kb:(k + 1) * kb], bb[k]) for k in range(min(nout, nframes)))
    evm = 0.0
    if data:
        d = np.concatenate(data)
        ref = np.sign(d.real) + 1j * np.sign(d.imag)
        ref /= np.sqrt(2)
        evm = 10 * np.log10(np.mean(np.abs(d / np.mean(np.abs(d)) - ref) ** 2))
    print(f"{rate} pilots={pilots} cfo={cfo} cyc/sym ppm={ppm} Es/N0={esn0} dB: "
          f"frames found {found}/{nframes}, decoded {ok}/{nframes}, MER {-evm:.1f} dB, "
          f"CFO est {f_est:.5f} (true {cfo}), front-end {t1 - t0:.1f} s")
    return ok == nframes


if __name__ == "__main__":
    good = True
    for kw in [dict(), dict(esn0=2.0), dict(cfo=-0.01, ppm=-100), dict(rate_idx=0, esn0=-1.0),
               dict(rate_idx=10, esn0=8.0, pilots=False, cfo=0.0005),
               dict(rate_idx=7, esn0=6.0, pilots=False, cfo=-0.003, ppm=-80),
               dict(rate_idx=3, esn0=2.5, pilots=False)]:
        good &= run(**kw)
    sys.exit(0 if good else 1)
