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
