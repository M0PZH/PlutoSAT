"""Check the bit-exact model (txmodel.py) against gr-dtv.

Run: dvbs2/.venv/bin/python -m pytest dvbs2/model/test_model.py
"""

import numpy as np
import pytest

import grref
import txmodel as m


def ts_packets(n, seed):
    rng = np.random.default_rng(seed)
    pk = rng.integers(0, 256, (n, 188), dtype=np.uint8)
    pk[:, 0] = 0x47
    return pk.ravel()


def gr_frames(rate_idx, pilots, gold, nframes=2, seed=0):
    rate = m.RATES[rate_idx]
    kbch = m.LDPC[rate]["kbch"]
    # enough TS for nframes BBFRAMEs (each holds < kbch/8 bytes of payload)
    npk = (nframes + 1) * kbch // (8 * 188) + 2
    ref = grref.tx_chain(ts_packets(npk, seed), rate, pilots=pilots, goldcode=gold)
    n = min(nframes, len(ref["bbframe"]) // kbch)
    assert n >= 1
    return ref, n


@pytest.mark.parametrize("rate_idx", range(11))
def test_fec(rate_idx):
    ref, n = gr_frames(rate_idx, True, 0)
    kbch = m.rate_params(rate_idx)[0]
    for f in range(n):
        bb = ref["bbframe"][f * kbch:(f + 1) * kbch]
        assert np.array_equal(m.bb_scramble(bb), ref["bbscrambled"][f * kbch:(f + 1) * kbch])
        assert np.array_equal(m.fec_frame(bb, rate_idx), ref["ldpc"][f * 64800:(f + 1) * 64800])


@pytest.mark.parametrize("rate_idx,pilots,gold", [(0, True, 0), (3, False, 0), (7, True, 0),
                                                  (10, False, 0), (9, True, 12345), (5, True, 262140)])
def test_plframe(rate_idx, pilots, gold):
    ref, n = gr_frames(rate_idx, pilots, gold, nframes=1)
    kbch = m.rate_params(rate_idx)[0]
    sig = m.plframe_signs(m.fec_frame(ref["bbframe"][:kbch], rate_idx), rate_idx + 1, pilots, gold)
    iq = m.signs_to_iq(sig)
    # gr-dtv 3.10 emits PLFRAMEs zero-stuffed to 2 samples/symbol
    gr = ref["plframe"][0::2][:len(sig)]
    gr_iq = np.stack([np.round(gr.real * 32767), np.round(gr.imag * 32767)], axis=1).astype(np.int64)
    bad = np.flatnonzero((gr_iq != iq).any(axis=1))
    assert len(gr) == len(sig)
    assert bad.size == 0, f"{bad.size} symbol mismatches, first at {bad[:8]}"


def test_rrc_vs_gnuradio():
    """Fixed-point polyphase RRC vs GR's float interpolating RRC: > 60 dB SNR."""
    sps, alpha, tpp = 24, 0.35, 16
    rng = np.random.default_rng(3)
    signs = rng.integers(0, 2, (400, 2)).astype(np.uint8)
    iq = m.signs_to_iq(signs)
    c = m.rrc_coefs(sps, alpha, tpp)
    y = m.rrc_interp(iq, c, sps, tpp)
    # float reference with identical (unquantised) taps through GR's filter
    import gnuradio.filter as grf
    from gnuradio import blocks, gr
    x = (iq[:, 0] + 1j * iq[:, 1]) / 32767.0
    taps = (c / c.max()).astype(float)
    tb = gr.top_block()
    src = blocks.vector_source_c(x.astype(np.complex64).tolist())
    f = grf.interp_fir_filter_ccf(sps, taps.tolist())
    snk = blocks.vector_sink_c()
    tb.connect(src, f, snk)
    tb.run()
    g = np.array(snk.data())
    yc = (y[:, 0] + 1j * y[:, 1]).astype(complex)
    gain = np.vdot(g, yc) / np.vdot(g, g)
    err = yc - gain * g
    snr = 10 * np.log10(np.mean(np.abs(yc) ** 2) / np.mean(np.abs(err) ** 2))
    assert snr > 60, snr
    assert np.abs(y).max() < 32767


# ---------------------------------------------------------------------------
# receiver model (rxfixed.py): frame sync / acquisition regressions
# ---------------------------------------------------------------------------

def test_rx_falselock_null_ts():
    """A TS of null packets (consecutive PLFRAMEs nearly identical), the RX
    started mid-frame with the AGC settling from above: up to RX 1.3 the frame
    sync locked a frame length after a stale search peak, on data, for good.
    Now a CHECK tag off the headers (that one, or random data passing a check
    by chance) is sent back to SEARCH by its PLS decode, and from the first
    header on every tag is a header with a confident decode."""
    import falselock_experiment as fe
    import rxfixed as fx
    L = fx.frame_len(True)
    tx, _ = fe.tx_signal(10, True, 6, "null")
    for start in (28310, 1363):
        x = fe.rx_input(tx, start, 20.0, 0.0, 0.0, 0, rms=8000)
        r = fx.receive(x, 11, True)
        cl = fe.classify(r, L, start)
        nwrong = next(k for k, c in enumerate(cl) if c[2])
        assert 1 <= nwrong <= 2 and all(ok for _, _, ok in cl[nwrong:]) and len(cl) - nwrong >= 3, cl
        assert [e[0] for e in r["fs"].events].count("pls_drop") == nwrong
        assert sum(1 for d in r["pls"].results if d["conf"] and d["modcod"] == 11) == len(cl) - nwrong


def test_rx_start_np14():
    """QPSK 1/4 without pilots at -1 dB after 8 dummy frames (the dummy
    header is a differential away from QPSK 1/4's): up to RX 1.3 a dummy frame
    taken for a data frame cost the whole start-up (seed 0: 0/6 frames)."""
    import rxfixed_experiment as ex
    assert ex.run(rate_idx=0, pilots=False, esn0=-1.0, seed=0, lead=8)[0]


def test_rx_esn0_restart():
    """Pilots off, the amplitude/noise average restarts after the frequency
    acquisition (up to RX 1.3 it began with a header measured at the wrong
    frequency: Es/N0 -20 dB, then ~0.6 dB per frame)."""
    import math
    import rxfixed as fx
    import rxfixed_experiment as ex
    x, _ = ex.make_signal(5, False, 3, 0.02, 60.0, 0.37, 20.0, 1, lead=8)
    cr = fx.receive(x, 6, False)["carrier"]
    est = [10 * math.log10(A * A / (E - A * A)) for (nhdr, f, fh, th, A, E, G) in cr.log if nhdr >= 3]
    assert len(est) >= 5 and all(18.5 < e < 22 for e in est), est


def test_rx_falselock_cycle():
    """Board finding (RX 1.4 as first built, QPSK 1/4 without pilots, null
    packets, digital loopback at 1 MS/s: half the starts acquired +374 kHz,
    retyped headers, failed frames, re-acquired). Reproduced through the ADC
    and DDC: after a PLS drop the search found the same repeating data again
    at the dummy length, a frame length after the last false tag, every
    frame; the carrier took those tags for confirmed headers and acquired
    0.374 cycles/symbol. Now the first search after a drop covers a data
    frame."""
    import falselock_experiment as fe
    import rxfixed as fx
    import rxmodel as rf
    x2, bb, start = fe.adc_signal(0, False, 12, 5)
    r = fx.receive(x2, 1, False)
    cr = r["carrier"]
    assert cr.n_acq == 1 and cr.n_retype == 0 and abs(cr.fhat) < (1 << 20)
    kb = m.LDPC["1/4"]["kbch"]
    out = rf.gr_ldpc_decode_llr(np.array(cr.frames), 0)
    dec = [any(np.array_equal(out[j * kb:(j + 1) * kb], b) for b in bb) for j in range(len(out) // kb)]
    assert len(dec) >= 3 and all(dec)
