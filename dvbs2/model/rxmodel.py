"""DVB-S2 receiver front-end reference model (float), QPSK normal frames.

Algorithm plan for the PL implementation (design notes, section "receive chain"):

  2 samples/symbol from the DDC
  -> AGC
  -> polyphase RRC matched filter + Gardner timing recovery (no Farrow):
     NPH sub-filters, a PI loop picks the sub-filter (fractional delay)
  -> frame sync: differential correlation with the 25 SOF differentials,
     then PLSC decision by correlation against the expected codeword(s)
     ("known parameters" mode: MODCOD/pilots are known; dummy frames too)
  -> carrier: frequency from the header + pilot blocks (phase slope),
     phase per pilot block, linear interpolation across the data slots
  -> PL descrambling, pilot removal -> 32400 data symbols per frame
  -> soft demapping -> LDPC -> BCH

The model is checked end to end with txmodel (exact TX) + a channel, and
gr-dvbs2rx's FEC blocks as the decoder.
"""

import numpy as np

import txmodel as m

SOF_BITS = [(m.SOF >> (25 - i)) & 1 for i in range(26)]


def header_symbols(modcod, pilots):
    """Ideal pi/2-BPSK PLHEADER (90 complex symbols, unit-amplitude axes)."""
    s = m.plheader_signs(modcod, pilots)
    return np.array([(1 - 2 * si) + 1j * (1 - 2 * sq) for si, sq in s]) / np.sqrt(2)


def scramble_seq(n, gold=0):
    xa, xb = m.gold_init(gold)
    r = m.pl_scramble_seq(n, xa, xb)
    return np.exp(1j * np.pi / 2 * r)          # multiply by j^r


# --------------------------------------------------------------------------
# channel
# --------------------------------------------------------------------------

def channel(tx_iq_int, sps_tx, sps_rx, cfo, clock_ppm, delay, esn0_db, seed=0):
    """Resample the TX output (sps_tx) to sps_rx with a clock error and a
    fractional delay, add a carrier offset (cycles/symbol) and AWGN."""
    from scipy.signal import resample_poly
    x = (tx_iq_int[:, 0] + 1j * tx_iq_int[:, 1]) / 32768.0
    up = 8
    xf = resample_poly(x, up, 1)                  # fine grid: sps_tx*up per symbol
    fine = sps_tx * up
    nsym = len(x) / sps_tx
    n_out = int((nsym - 4) * sps_rx)
    t = (np.arange(n_out) / sps_rx * (1 + clock_ppm * 1e-6) + delay) * fine
    i0 = np.floor(t).astype(int)
    fr = t - i0
    i0 = np.clip(i0, 0, len(xf) - 2)
    y = xf[i0] * (1 - fr) + xf[i0 + 1] * fr
    y *= np.exp(2j * np.pi * cfo * np.arange(n_out) / sps_rx)
    es = np.mean(np.abs(y) ** 2) * sps_rx        # energy per symbol
    # white noise at fs = sps_rx / Ts: per-sample variance = N0 * fs = Es/(Es/N0) per symbol time
    var = es / 10 ** (esn0_db / 10)
    rng = np.random.default_rng(seed)
    y = y + np.sqrt(var / 2) * (rng.standard_normal(n_out) + 1j * rng.standard_normal(n_out))
    return y


# --------------------------------------------------------------------------
# timing recovery
# --------------------------------------------------------------------------

def rrc_bank(rolloff, nph=32, span=12):
    """RRC matched filter at 2 sps split into nph fractional-delay phases."""
    n = 2 * nph * span + 1
    t = (np.arange(n) - (n - 1) / 2) / (2 * nph)
    h = np.empty(n)
    a = rolloff
    for i, ti in enumerate(t):
        if abs(ti) < 1e-12:
            h[i] = 1 - a + 4 * a / np.pi
        elif abs(abs(4 * a * ti) - 1) < 1e-9:
            h[i] = a / np.sqrt(2) * ((1 + 2 / np.pi) * np.sin(np.pi / (4 * a)) + (1 - 2 / np.pi) * np.cos(np.pi / (4 * a)))
        else:
            h[i] = (np.sin(np.pi * ti * (1 - a)) + 4 * a * ti * np.cos(np.pi * ti * (1 + a))) / \
                (np.pi * ti * (1 - (4 * a * ti) ** 2))
    h /= np.sqrt(np.sum(h[::nph] ** 2) * 1.0)
    # phase k delays by k/nph sample
    taps = len(h) // nph
    bank = np.zeros((nph, taps + 1))
    for k in range(nph):
        sub = h[k::nph]
        bank[k, :len(sub)] = sub
    return bank


def timing_recovery(x, rolloff, bw=0.005, nph=32):
    """Gardner TED + PI loop on a polyphase RRC bank, 2 sps in, 1 sps out.

    Returns symbols (on-time strobes)."""
    bank = rrc_bank(rolloff, nph)
    ntap = bank.shape[1]
    # loop gains (normalised, damping 1/sqrt2)
    zeta = 1 / np.sqrt(2)
    theta = bw / (zeta + 0.25 / zeta)
    d = 1 + 2 * zeta * theta + theta ** 2
    kp = 4 * zeta * theta / d / 2.0
    ki = 4 * theta ** 2 / d / 2.0
    mu = 0.0              # fractional delay in samples (0..1)
    integ = 0.0
    out = []
    k = ntap
    prev_on = 0
    mid = 0
    while k + 2 < len(x):
        # fractional sample index of the on-time point
        ph = int(mu * nph) % nph
        seg = x[k - ntap + 1:k + 1][::-1]
        on = np.dot(bank[ph], seg)
        segm = x[k - ntap:k][::-1]
        mid = np.dot(bank[ph], segm)
        out.append(on)
        e = np.real((prev_on - on) * np.conj(mid))
        prev_on = on
        integ += ki * e
        mu += kp * e + integ
        step = 2
        while mu >= 1:
            mu -= 1
            step += 1
        while mu < 0:
            mu += 1
            step -= 1
        k += step
    return np.array(out)


# --------------------------------------------------------------------------
# frame sync, carrier recovery, descrambling
# --------------------------------------------------------------------------

def frame_lengths(pilots):
    return 90 + 32400 + (22 * 36 if pilots else 0)


def find_frames(sym, modcod, pilots, dummy=True, thresh=0.6):
    """Differential correlation with SOF + the known PLSC, at any carrier offset.

    Returns list of (start index, is_dummy)."""
    cands = [(header_symbols(modcod, pilots), False, frame_lengths(pilots))]
    if dummy:
        cands.append((header_symbols(0, False), True, 90 + 36 * 90))
    d = sym[1:] * np.conj(sym[:-1])
    d /= np.abs(d) + 1e-12
    out = []
    best = []
    for h, is_dummy, _ in cands:
        dh = h[1:] * np.conj(h[:-1])
        c = np.abs(np.correlate(d, dh, mode="valid")) / len(dh)
        best.append(c)
    n = min(len(b) for b in best)
    best = np.stack([b[:n] for b in best])
    k = 0
    while k < n:
        j = int(np.argmax(best[:, k]))
        if best[j, k] > thresh:
            # take the local maximum
            w = best[j, k:k + 5]
            k2 = k + int(np.argmax(w))
            out.append((k2, cands[j][1]))
            k = k2 + cands[j][2] - 10
        else:
            k += 1
    return out


def dd_pll(x, bw=0.002, zeta=0.707):
    """Second-order decision-directed PLL for (descrambled) QPSK symbols."""
    theta = bw / (zeta + 0.25 / zeta)
    d = 1 + 2 * zeta * theta + theta ** 2
    kp = 4 * zeta * theta / d
    ki = 4 * theta ** 2 / d
    out = np.empty_like(x)
    ph = 0.0
    integ = 0.0
    for k, v in enumerate(x):
        y = v * np.exp(-1j * ph)
        out[k] = y
        dec = (np.sign(y.real) + 1j * np.sign(y.imag)) / np.sqrt(2)
        e = np.imag(y * np.conj(dec)) / (abs(y) + 1e-9)
        integ += ki * e
        ph += kp * e + integ
    return out


def known_symbols(sym_len, starts, pilots, modcod, gold=0):
    """Reference value (or 0 = unknown) for every symbol of the stream."""
    ref = np.zeros(sym_len, complex)
    sc_d = scramble_seq(frame_lengths(pilots) - 90, gold)
    sc_u = scramble_seq(36 * 90, gold)
    for st, dummy in starts:
        h = header_symbols(0, False) if dummy else header_symbols(modcod, pilots)
        e = min(st + 90, sym_len)
        ref[st:e] = h[:e - st]
        if dummy:
            e = min(st + 90 + 3240, sym_len)
            ref[st + 90:e] = ((1 + 1j) / np.sqrt(2) * sc_u)[:e - st - 90]
        elif pilots:
            p = st + 90
            for slot in range(360):
                if slot > 0 and slot % 16 == 0:
                    e = min(p + 36, sym_len)
                    if p < sym_len:
                        ref[p:e] = ((1 + 1j) / np.sqrt(2) * sc_d[p - st - 90:p - st - 90 + 36])[:e - p]
                    p += 36
                p += 90
    return ref


def carrier_track(sym, ref, f0, bw=0.004, zeta=0.707, dd=True):
    """Continuous 2nd-order PLL over the whole stream: data-aided on known
    symbols (ref != 0), decision-directed (QPSK) elsewhere if dd. With pilots
    use dd=False: the loop then only updates on headers/pilots and free-runs
    (frequency integrator) across the data slots, which keeps working at very
    low SNR where QPSK decisions are unreliable. f0: initial frequency
    (cycles/symbol), e.g. from header_cfo()."""
    theta = bw / (zeta + 0.25 / zeta)
    d = 1 + 2 * zeta * theta + theta ** 2
    kp = 4 * zeta * theta / d
    ki = 4 * theta ** 2 / d
    out = np.empty_like(sym)
    ph = 0.0
    freq = 2 * np.pi * f0
    for k, v in enumerate(sym):
        y = v * np.exp(-1j * ph)
        out[k] = y
        r = ref[k]
        if r == 0:
            if not dd:
                ph += freq
                continue
            r = (np.sign(y.real) + 1j * np.sign(y.imag)) / np.sqrt(2)
        e = np.imag(y * np.conj(r)) / (abs(y) + 1e-9)
        freq += ki * e
        ph += freq + kp * e
    return out


def header_cfo(sym, starts, pilots, modcod):
    """Coarse carrier offset (cycles/symbol) from the known PLHEADERs.

    Kay estimate (lag 1, range +-0.5) then Luise & Reggiannini on the
    derotated header (lags 1..45), averaged over all detected frames."""
    acc1 = 0j
    zs = []
    for st, dummy in starts:
        h = header_symbols(0, False) if dummy else header_symbols(modcod, pilots)
        z = sym[st:st + 90] * np.conj(h)
        if len(z) < 90:
            continue
        zs.append(z)
        acc1 += np.sum(z[1:] * np.conj(z[:-1]))
    if not zs:
        return 0.0
    f1 = np.angle(acc1) / (2 * np.pi)
    M = 45
    acc = 0j
    for z in zs:
        z = z * np.exp(-2j * np.pi * f1 * np.arange(90))
        for mm in range(1, M + 1):
            acc += np.mean(z[mm:] * np.conj(z[:-mm]))
    return f1 + np.angle(acc) / (np.pi * (M + 1))


def recover_frame(sym, start, pilots, gold=0, modcod=4, tracked=False):
    """Carrier recovery + descrambling for one data PLFRAME starting at start.

    tracked=True: the stream is already carrier-corrected (carrier_track);
    only amplitude/noise estimation, descrambling and pilot removal remain."""
    nfr = frame_lengths(pilots)
    f = sym[start:start + nfr]
    if len(f) < nfr:
        return None
    hdr = header_symbols(modcod, pilots)
    body_ref = scramble_seq(nfr - 90, gold)
    # known-symbol positions: header + pilots (pilot = (1+j)/sqrt2 scrambled)
    known_idx = list(range(90))
    known_ref = list(hdr)
    if pilots:
        p = 90
        for slot in range(360):
            if slot > 0 and slot % 16 == 0:
                for i in range(36):
                    known_idx.append(p + i)
                    known_ref.append((1 + 1j) / np.sqrt(2) * body_ref[p - 90 + i])
                p += 36
            p += 90
    known_idx = np.array(known_idx)
    known_ref = np.array(known_ref)
    if tracked:
        z = f[known_idx] * np.conj(known_ref)
        amp = abs(np.mean(z))
        zk = z / max(amp, 1e-9)
        nvar = float(np.mean(np.abs(zk - np.mean(zk) / abs(np.mean(zk))) ** 2))
        body = f[90:] * np.conj(body_ref) / max(amp, 1e-9) * np.conj(np.mean(zk) / abs(np.mean(zk)))
        data = []
        p = 0
        for slot in range(360):
            if pilots and slot > 0 and slot % 16 == 0:
                p += 36
            data.append(body[p:p + 90])
            p += 90
        return np.concatenate(data), 0.0, nvar
    z = f[known_idx] * np.conj(known_ref)
    # frequency: weighted phase slope across known blocks
    blocks = [(0, 90)]
    if pilots:
        blocks += [(90 + 22 * 0 + k, 36) for k in range(0)]
    # block phasors
    groups = [z[:90]] + ([z[90 + 36 * b:90 + 36 * (b + 1)] for b in range(22)] if pilots else [])
    centres = [45.0] + ([known_idx[90 + 36 * b] + 18.0 for b in range(22)] if pilots else [])
    phasors = np.array([g.mean() for g in groups])
    if pilots:
        dph = np.angle(phasors[1:] * np.conj(phasors[:-1]))
        dt = np.diff(centres)
        w = np.abs(phasors[1:] * phasors[:-1])
        fo = np.sum(w * dph / dt) / np.sum(w)            # rad/symbol
    else:
        zz = z[1:] * np.conj(z[:-1])
        fo = np.angle(zz.mean())
    n = np.arange(nfr)
    f = f * np.exp(-1j * fo * n)
    z = f[known_idx] * np.conj(known_ref)
    groups = [z[:90]] + ([z[90 + 36 * b:90 + 36 * (b + 1)] for b in range(22)] if pilots else [])
    ph = np.unwrap(np.angle([g.mean() for g in groups]))
    if pilots:
        phase = np.interp(n, centres, ph)
        f = f * np.exp(-1j * phase)
    else:
        f = f * np.exp(-1j * ph[0])
        f[90:] = dd_pll(f[90:] * np.conj(body_ref), bw=0.002) * body_ref
    amp = np.abs(np.mean(z[:90] * np.exp(-1j * ph[0])))
    # noise variance (per complex symbol, at unit symbol amplitude) from the
    # known symbols after carrier correction
    zk = f[known_idx] * np.conj(known_ref) / max(amp, 1e-9)
    nvar = float(np.mean(np.abs(zk - 1) ** 2))
    body = f[90:] * np.conj(body_ref) / max(amp, 1e-9)
    # drop pilots
    data = []
    p = 0
    for slot in range(360):
        if pilots and slot > 0 and slot % 16 == 0:
            p += 36
        data.append(body[p:p + 90])
        p += 90
    return np.concatenate(data), fo, nvar


def gr_fec_decode(data_syms, rate_idx):
    """gr-dvbs2rx demapper + LDPC + BCH + BB descrambler on recovered symbols.

    data_syms: (nframes, 32400) complex at unit QPSK amplitude. Returns the
    BBFRAME bits (nframes * Kbch). gr-dvbs2rx decodes LDPC in SIMD batches of
    32 frames, so the input is padded with copies of the last frame."""
    from gnuradio import blocks, gr
    import gnuradio.dvbs2rx as rx
    rate = getattr(rx, "C" + m.RATES[rate_idx].replace("/", "_"))
    kbch = m.LDPC[m.RATES[rate_idx]]["kbch"]
    d = np.asarray(data_syms, np.complex64)
    n = len(d)
    pad = (-n) % 32
    d = np.concatenate([d, np.repeat(d[-1:], pad, axis=0)]) if pad else d
    tb = gr.top_block()
    src = blocks.vector_source_c(d.ravel())
    dem = rx.xfecframe_demapper_cb(rx.FECFRAME_NORMAL, rate, rx.MOD_QPSK)
    ldpc = rx.ldpc_decoder_bb(rx.STANDARD_DVBS2, rx.FECFRAME_NORMAL, rate, rx.MOD_QPSK,
                              rx.OM_MESSAGE, rx.INFO_OFF, 25, 0)
    bch = rx.bch_decoder_bb(rx.STANDARD_DVBS2, rx.FECFRAME_NORMAL, rate, rx.OM_MESSAGE, 0)
    desc = rx.bbdescrambler_bb(rx.STANDARD_DVBS2, rx.FECFRAME_NORMAL, rate)
    snk = blocks.vector_sink_b()
    tb.connect(src, dem, ldpc, bch, desc, snk)
    tb.run()
    out = np.array(snk.data(), dtype=np.uint8)
    if len(out) == len(d) * kbch // 8:      # packed bytes
        out = np.unpackbits(out)
    return out[: n * kbch]


LLR_MAX = 127


def soft_demap(data, nvar, scale=4.0):
    """QPSK max-log LLRs (log P(b=0)/P(b=1)) as int8, b0 from I, b1 from Q.

    data at unit symbol amplitude: points (+-1 +-j)/sqrt2. LLR = 2*sqrt2*y/nvar,
    times `scale` LSBs per unit of LLR, saturated to +-127."""
    k = 2 * np.sqrt(2) / max(nvar, 1e-6) * scale
    llr = np.empty(2 * len(data))
    llr[0::2] = k * data.real
    llr[1::2] = k * data.imag
    return np.clip(np.round(llr), -LLR_MAX, LLR_MAX).astype(np.int8)


def gr_ldpc_decode_llr(llrs, rate_idx):
    """gr-dvbs2rx LDPC + BCH + BB descrambler on int8 LLRs (nframes, 64800)."""
    from gnuradio import blocks, gr
    import gnuradio.dvbs2rx as rx
    rate = getattr(rx, "C" + m.RATES[rate_idx].replace("/", "_"))
    kbch = m.LDPC[m.RATES[rate_idx]]["kbch"]
    l = np.asarray(llrs, np.int8)
    n = len(l)
    pad = (-n) % 32
    l = np.concatenate([l, np.repeat(l[-1:], pad, axis=0)]) if pad else l
    tb = gr.top_block()
    src = blocks.vector_source_b(l.view(np.uint8).ravel())
    ldpc = rx.ldpc_decoder_bb(rx.STANDARD_DVBS2, rx.FECFRAME_NORMAL, rate, rx.MOD_QPSK,
                              rx.OM_MESSAGE, rx.INFO_OFF, 25, 0)
    bch = rx.bch_decoder_bb(rx.STANDARD_DVBS2, rx.FECFRAME_NORMAL, rate, rx.OM_MESSAGE, 0)
    desc = rx.bbdescrambler_bb(rx.STANDARD_DVBS2, rx.FECFRAME_NORMAL, rate)
    snk = blocks.vector_sink_b()
    tb.connect(src, ldpc, bch, desc, snk)
    tb.run()
    out = np.array(snk.data(), dtype=np.uint8)
    if len(out) == len(l) * kbch // 8:
        out = np.unpackbits(out)
    return out[: n * kbch]


# --------------------------------------------------------------------------
# PLS decoding (float reference of rxfixed.pls_decode)
# --------------------------------------------------------------------------

def pls_header_symbols(code):
    """PLHEADER of a 7-bit PLS code (MODCOD << 2 | short << 1 | pilots)."""
    hdr = (m.SOF << 64) | m.plsc_bits(code >> 2, (code >> 1) & 1, code & 1)
    bits = [(hdr >> (89 - i)) & 1 for i in range(90)]
    return np.array([((1 + 1j) if i % 2 == 0 else (-1 + 1j)) * (1 - 2 * y)
                     for i, y in enumerate(bits)]) / np.sqrt(2)


PLS_HDRS = np.array([pls_header_symbols(c) for c in range(128)])


def pls_decode_coherent(r, freqs=(0.0,)):
    """Maximum likelihood for an unknown phase: argmax over the frequency
    hypotheses (cycles/symbol) and the 128 codes of |sum conj(h_c) r e^(-j2pi f k)|^2.
    r: (..., 90) complex. Returns (code, frequency)."""
    r = np.asarray(r)
    k = np.arange(90)
    rot = np.exp(-2j * np.pi * np.asarray(freqs)[:, None] * k[None, :])
    met = np.abs((r[..., None, :] * rot) @ np.conj(PLS_HDRS).T) ** 2      # (..., nf, 128)
    met = met.reshape(*met.shape[:-2], -1)
    best = met.argmax(-1)
    return best % 128, np.asarray(freqs)[best // 128]


def pls_decode_differential(r):
    """Non-coherent alternative: argmax |sum_i conj(d_c,i) d_i|^2 over the 89
    header differentials d_i = r_i conj(r_(i-1)) (any carrier offset, like the
    frame sync). Much weaker: codes whose (64,7) codewords differ only in a
    few pair-to-pair transitions (the 16-pair halves of the MODCOD LSB row,
    the all-ones row of the frame size) are almost indistinguishable."""
    r = np.asarray(r)
    d = r[..., 1:] * np.conj(r[..., :-1])
    dh = PLS_HDRS[:, 1:] * np.conj(PLS_HDRS[:, :-1])
    return np.abs(d @ np.conj(dh).T).argmax(-1)
