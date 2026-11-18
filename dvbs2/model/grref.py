"""Thin wrappers that run gr-dtv DVB-S2 TX blocks on numpy vectors.

These are the golden reference for the HDL. Every function takes/returns
numpy arrays; bit streams are unpacked (one bit per uint8, MSB-first order as
transmitted).
"""

import numpy as np
from gnuradio import blocks, dtv, filter as grfilter, gr
from gnuradio.filter import firdes

# Normal-frame QPSK code rates in MODCOD order (MODCOD 1..11).
QPSK_RATES = ["1/4", "1/3", "2/5", "1/2", "3/5", "2/3", "3/4", "4/5", "5/6", "8/9", "9/10"]

GR_RATE = {
    "1/4": dtv.C1_4, "1/3": dtv.C1_3, "2/5": dtv.C2_5, "1/2": dtv.C1_2,
    "3/5": dtv.C3_5, "2/3": dtv.C2_3, "3/4": dtv.C3_4, "4/5": dtv.C4_5,
    "5/6": dtv.C5_6, "8/9": dtv.C8_9, "9/10": dtv.C9_10,
}

# ETSI EN 302 307-1 Table 5a (normal FECFRAME): Kbch, Nbch(=Kldpc), t
NORMAL_BCH = {
    "1/4": (16008, 16200, 12), "1/3": (21408, 21600, 12), "2/5": (25728, 25920, 12),
    "1/2": (32208, 32400, 12), "3/5": (38688, 38880, 12), "2/3": (43040, 43200, 10),
    "3/4": (48408, 48600, 12), "4/5": (51648, 51840, 12), "5/6": (53840, 54000, 10),
    "8/9": (57472, 57600, 8), "9/10": (58192, 58320, 8),
}
NLDPC = 64800

# Table 5b (short FECFRAME, 16200 bits; no 9/10): Kbch, Nbch(=Kldpc), t
SHORT_RATES = QPSK_RATES[:10]
SHORT_BCH = {
    "1/4": (3072, 3240, 12), "1/3": (5232, 5400, 12), "2/5": (6312, 6480, 12),
    "1/2": (7032, 7200, 12), "3/5": (9552, 9720, 12), "2/3": (10632, 10800, 12),
    "3/4": (11712, 11880, 12), "4/5": (12432, 12600, 12), "5/6": (13152, 13320, 12),
    "8/9": (14232, 14400, 12),
}
NLDPC_SHORT = 16200

# 8PSK code rates (MODCOD 12..17)
PSK8_RATES = ["3/5", "2/3", "3/4", "5/6", "8/9", "9/10"]


def _fs(short):
    return dtv.FECFRAME_SHORT if short else dtv.FECFRAME_NORMAL


def _mod(psk8):
    return dtv.MOD_8PSK if psk8 else dtv.MOD_QPSK

ROLLOFF = {0.35: dtv.RO_0_35, 0.25: dtv.RO_0_25, 0.20: dtv.RO_0_20}


def _run(src_block_factory, blocks_chain, data, out_type):
    tb = gr.top_block()
    src = src_block_factory(data)
    sink = out_type()
    prev = src
    for b in blocks_chain:
        tb.connect(prev, b)
        prev = b
    tb.connect(prev, sink)
    tb.run()
    return np.array(sink.data())


def _src_b(data):
    return blocks.vector_source_b(np.asarray(data, dtype=np.uint8).tolist(), False)


def _src_c(data):
    return blocks.vector_source_c(np.asarray(data, dtype=np.complex64).tolist(), False)


def bbheader(ts_bytes, rate, rolloff=0.35, short=False):
    """TS packets (packed bytes) -> BBFRAME bits (unpacked), Kbch bits per frame."""
    b = dtv.dvb_bbheader_bb(dtv.STANDARD_DVBS2, _fs(short), GR_RATE[rate],
                            ROLLOFF[rolloff], dtv.INPUTMODE_NORMAL, dtv.INBAND_OFF, 168, 4000000)
    return _run(_src_b, [b], ts_bytes, blocks.vector_sink_b).astype(np.uint8)


def bbscrambler(bits, rate, short=False):
    b = dtv.dvb_bbscrambler_bb(dtv.STANDARD_DVBS2, _fs(short), GR_RATE[rate])
    return _run(_src_b, [b], bits, blocks.vector_sink_b).astype(np.uint8)


def bch(bits, rate, short=False):
    b = dtv.dvb_bch_bb(dtv.STANDARD_DVBS2, _fs(short), GR_RATE[rate])
    return _run(_src_b, [b], bits, blocks.vector_sink_b).astype(np.uint8)


def ldpc(bits, rate, short=False):
    b = dtv.dvb_ldpc_bb(dtv.STANDARD_DVBS2, _fs(short), GR_RATE[rate], dtv.MOD_QPSK)
    return _run(_src_b, [b], bits, blocks.vector_sink_b).astype(np.uint8)


def interleave_modulate(bits, rate, short=False, psk8=False):
    """FECFRAME bits -> QPSK or 8PSK symbols (complex64)."""
    il = dtv.dvbs2_interleaver_bb(_fs(short), GR_RATE[rate], _mod(psk8))
    mod = dtv.dvbs2_modulator_bc(_fs(short), GR_RATE[rate], _mod(psk8),
                                 dtv.INTERPOLATION_OFF)
    return _run(_src_b, [il, mod], bits, blocks.vector_sink_c)


def physical(symbols, rate, pilots=True, goldcode=0, short=False, psk8=False):
    """QPSK/8PSK symbols -> PLFRAMEs (header, pilots, PL scrambling)."""
    b = dtv.dvbs2_physical_cc(_fs(short), GR_RATE[rate], _mod(psk8),
                              dtv.PILOTS_ON if pilots else dtv.PILOTS_OFF, goldcode)
    return _run(_src_c, [b], symbols, blocks.vector_sink_c)


def rrc_taps(sps, rolloff, span_symbols):
    """GNU Radio RRC taps for an interpolate-by-sps shaping filter (gain = sps)."""
    return np.array(firdes.root_raised_cosine(sps, sps, 1.0, rolloff, span_symbols * sps + 1))


def shape(plframe_symbols, sps, rolloff, span_symbols):
    taps = rrc_taps(sps, rolloff, span_symbols)
    f = grfilter.interp_fir_filter_ccf(sps, taps.tolist())
    return _run(_src_c, [f], plframe_symbols, blocks.vector_sink_c), taps


def tx_chain(ts_bytes, rate, pilots=True, goldcode=0, rolloff=0.35, short=False, psk8=False):
    """Full GR TX chain; returns every intermediate stage as a dict."""
    out = {}
    out["bbframe"] = bbheader(ts_bytes, rate, rolloff, short)
    out["bbscrambled"] = bbscrambler(out["bbframe"], rate, short)
    out["bch"] = bch(out["bbscrambled"], rate, short)
    out["ldpc"] = ldpc(out["bch"], rate, short)
    out["symbols"] = interleave_modulate(out["ldpc"], rate, short, psk8)
    out["plframe"] = physical(out["symbols"], rate, pilots, goldcode, short, psk8)
    return out
