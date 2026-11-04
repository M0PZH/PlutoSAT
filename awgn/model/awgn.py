"""Receive-side self-test noise source: bit-exact model of rtl/rx_awgn.sv.

Every ADC clock eight xorshift32 generators step once. The noise sample of
a clock is, per component, the sum of 16 bytes (four generators x four
bytes) minus 2040: Irwin-Hall(16) on [-2040, 2040], mean 0, standard
deviation SIGMA_G = sqrt(16 (256^2 - 1) / 12) = 295.6 (excess kurtosis
-0.075, |g| <= 6.9 sigma). On rx0:

    y = sat16((x * gain + g * nscale + 2^13) >> 14)     (noise enabled)
    y = x                                               (disabled)

gain: Q2.14 (16384 = 1), nscale: 0..131071, noise standard deviation per
component sigma = SIGMA_G * nscale / 2^14 = 0.01804 * nscale ADC LSB.
"""
import math

import numpy as np

M32 = 0xFFFFFFFF
SEEDS = [0x9E3779B9, 0x7F4A7C15, 0x85EBCA6B, 0xC2B2AE35,
         0x27D4EB2F, 0x165667B1, 0xD3A2646C, 0xFD7046C5]
OFFSET = 2040
SIGMA_G = math.sqrt(16 * (256 ** 2 - 1) / 12)
SCALE_SH = 14
NSCALE_MAX = (1 << 17) - 1


def xs32(x):
    x ^= (x << 13) & M32
    x ^= x >> 17
    x ^= (x << 5) & M32
    return x


def noise(nclk):
    """g_i, g_q for clocks 0 .. nclk-1 (clock 0: the state right after reset)."""
    st = np.array(SEEDS, np.uint64)
    gi = np.empty(nclk, np.int64)
    gq = np.empty(nclk, np.int64)
    m = np.uint64(M32)
    for t in range(nclk):
        b = sum(((st >> np.uint64(8 * k)) & np.uint64(255)) for k in range(4)).astype(np.int64)
        gi[t] = b[0:4].sum() - OFFSET
        gq[t] = b[4:8].sum() - OFFSET
        st = st ^ ((st << np.uint64(13)) & m)
        st = st ^ (st >> np.uint64(17))
        st = st ^ ((st << np.uint64(5)) & m)
    return gi, gq


def apply(x, g, gain, nscale):
    v = (np.asarray(x, np.int64) * gain + np.asarray(g, np.int64) * nscale + (1 << (SCALE_SH - 1))) >> SCALE_SH
    return np.clip(v, -32768, 32767)


def sigma(nscale):
    return SIGMA_G * nscale / (1 << SCALE_SH)


def nscale_for(sig):
    return int(round(sig / SIGMA_G * (1 << SCALE_SH)))


def esn0_db(psig, sig, fs, rs):
    """Es/N0 for a signal of power psig (sum over I and Q) at symbol rate rs,
    sample rate fs, and noise sigma per component."""
    return 10 * math.log10(psig * fs / (2 * sig ** 2 * rs))
