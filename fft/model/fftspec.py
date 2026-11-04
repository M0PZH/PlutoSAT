"""Spectrum analyser: bit-exact model of rtl/fft_*.sv around the Xilinx FFT.

Per frame (4096 consecutive samples x[n] = i[n] + j q[n] of one source):

  window   y[n] = (x[n] * w[n] + 2^16) >> 17          (each of I, Q; floor)
           w[n] = w[4095 - n] = WIN[n], n < 2048: 18-bit unsigned, 2^17 = 1.0
           (reset contents: symmetric 4-term Blackman-Harris, rtl/gen/fft_win.mem)
  FFT      X[k] = xfft(y) with the scale schedule SCALE (12 radix-2 stages,
           2 bits per stage, stage 0 in bits [1:0]; total shift S = sum):
           ~ sum_n y[n] exp(-2 pi j n k / 4096) / 2^S, 16-bit, convergent
           rounding in every stage. Bit-exact: the Xilinx C model (xfft_cmodel);
           float reference: numpy (fft_ref).
  power    p[k] = re(X[k])^2 + im(X[k])^2               (32-bit unsigned)
  average  A[k] = sum over NAVG frames of p[k]          (40-bit, no overflow
                                                         for NAVG <= 256)
  readout  R[j] = min(A[k] >> RSHIFT, 2^32 - 1),  j = k ^ 2048
           (FFT-shifted: j = 0 is -fs/2, j = 2048 is DC, j = 4095 is
           +fs/2 - fs/4096; frequency of word j: (j - 2048) fs / 4096)
"""
import ctypes
import math
import os
import sys

import numpy as np

N = 4096
LOG2N = 12
HALF = N // 2
WBITS = 17                     # window: 2^17 = 1.0
ACC_BITS = 40
NAVG_MAX = 256
RSHIFT_MAX = 8

HERE = os.path.dirname(os.path.abspath(__file__))
WIN_MEM = os.path.join(HERE, "..", "rtl", "gen", "fft_win.mem")

# 4-term Blackman-Harris (-92 dB sidelobes, ENBW 2.00 bins)
BH4 = (0.35875, 0.48829, 0.14128, 0.01168)


# ---------------------------------------------------------------- window
def window_half(kind="blackmanharris"):
    """The 2048 window table words (w[n] for n < 2048, w[4095 - n] = w[n])."""
    n = np.arange(HALF)
    if kind == "blackmanharris":
        a = BH4
        t = 2 * np.pi * n / (N - 1)
        w = a[0] - a[1] * np.cos(t) + a[2] * np.cos(2 * t) - a[3] * np.cos(3 * t)
    elif kind == "hann":
        w = 0.5 - 0.5 * np.cos(2 * np.pi * n / (N - 1))
    elif kind == "rect":
        w = np.ones(HALF)
    else:
        raise ValueError(kind)
    return np.clip(np.round(w * (1 << WBITS)), 0, 1 << WBITS).astype(np.int64)


def window_full(half):
    half = np.asarray(half, np.int64)
    return np.concatenate([half, half[::-1]])


def write_win_mem(path=WIN_MEM, kind="blackmanharris"):
    """rtl/gen/fft_win.mem: the reset contents of the window table (18-bit hex)."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write("".join(f"{v:05x}\n" for v in window_half(kind)))


def apply_window(i, q, half):
    """Integer windowing exactly as the RTL: (x * w + 2^16) >> 17."""
    w = window_full(half)
    i = np.asarray(i, np.int64)
    q = np.asarray(q, np.int64)
    r = 1 << (WBITS - 1)
    return (i * w + r) >> WBITS, (q * w + r) >> WBITS


# ---------------------------------------------------------------- scale schedule
def sched_list(word):
    """SCALE register -> per-stage shifts (stage 0 first)."""
    return [(word >> (2 * s)) & 3 for s in range(LOG2N)]


def sched_word(stages):
    return sum((v & 3) << (2 * s) for s, v in enumerate(stages))


def sched_shift(word):
    return sum(sched_list(word))


SCALE_RX = 0x555400     # 12-bit ADC data (reset value): stages 0..4 unscaled, total shift 7
SCALE_NBIQ = 0x555550   # nbiq at its default gain 8 (12-bit full scale = 16384): 10
SCALE_16 = 0x555555     # full-scale 16-bit data: shift 1 per stage, 12


# ---------------------------------------------------------------- Xilinx C model
CMODEL_DIR = "/opt/Xilinx/2026.1/lnx64/tools"
CMODEL_LIBPATH = os.path.join(CMODEL_DIR, "fpo_v7_1")


def ensure_cmodel_env():
    """The xfft C model needs the floating-point model's libraries on
    LD_LIBRARY_PATH, which only takes effect at process start: re-exec."""
    lp = os.environ.get("LD_LIBRARY_PATH", "")
    if CMODEL_LIBPATH in lp.split(":") or not os.path.isdir(CMODEL_LIBPATH):
        return
    env = dict(os.environ, LD_LIBRARY_PATH=CMODEL_LIBPATH + (":" + lp if lp else ""))
    os.execve(sys.executable, list(sys.orig_argv), env)


class _G(ctypes.Structure):
    _fields_ = [(n, ctypes.c_int) for n in (
        "C_NFFT_MAX", "C_ARCH", "C_HAS_NFFT", "C_USE_FLT_PT", "C_INPUT_WIDTH",
        "C_TWIDDLE_WIDTH", "C_HAS_SCALING", "C_HAS_BFP", "C_HAS_ROUNDING",
        "C_NSSR", "C_SYSTOLICFFT_INV")]


_PD = ctypes.POINTER(ctypes.c_double)


class _I(ctypes.Structure):
    _fields_ = [("nfft", ctypes.c_int), ("xn_re", _PD), ("xn_re_size", ctypes.c_int),
                ("xn_im", _PD), ("xn_im_size", ctypes.c_int),
                ("scaling_sch", ctypes.POINTER(ctypes.c_int)), ("scaling_sch_size", ctypes.c_int),
                ("direction", ctypes.c_int)]


class _O(ctypes.Structure):
    _fields_ = [("xk_re", _PD), ("xk_re_size", ctypes.c_int), ("xk_im", _PD),
                ("xk_im_size", ctypes.c_int), ("blk_exp", ctypes.c_int), ("overflow", ctypes.c_int)]


class XfftCModel:
    """Bit-accurate model of the fft_xfft instance (ip/fft_xfft.tcl)."""

    def __init__(self, rounding=True):
        lib = ctypes.CDLL(os.path.join(CMODEL_DIR, "fft_v9_1", "libIp_xfft_v9_1_bitacc_cmodel.so"))
        lib.xilinx_ip_xfft_v9_1_get_default_generics.restype = _G
        lib.xilinx_ip_xfft_v9_1_create_state.restype = ctypes.c_void_p
        lib.xilinx_ip_xfft_v9_1_create_state.argtypes = [_G]
        lib.xilinx_ip_xfft_v9_1_bitacc_simulate.argtypes = [ctypes.c_void_p, _I, ctypes.POINTER(_O)]
        g = lib.xilinx_ip_xfft_v9_1_get_default_generics()
        g.C_NFFT_MAX, g.C_ARCH, g.C_HAS_NFFT, g.C_USE_FLT_PT = LOG2N, 4, 0, 0
        g.C_INPUT_WIDTH, g.C_TWIDDLE_WIDTH = 16, 16
        g.C_HAS_SCALING, g.C_HAS_BFP, g.C_HAS_ROUNDING, g.C_NSSR = 1, 0, int(rounding), 1
        self.lib = lib
        self.state = lib.xilinx_ip_xfft_v9_1_create_state(g)

    def __call__(self, yi, yq, scale):
        """Integer input (16-bit) -> (integer complex output, overflow)."""
        re = np.ascontiguousarray(np.asarray(yi, np.float64) / 32768.0)
        im = np.ascontiguousarray(np.asarray(yq, np.float64) / 32768.0)
        sch = (ctypes.c_int * LOG2N)(*sched_list(scale))
        ore = np.zeros(N)
        oim = np.zeros(N)
        inp = _I(LOG2N, re.ctypes.data_as(_PD), N, im.ctypes.data_as(_PD), N, sch, LOG2N, 1)
        out = _O(ore.ctypes.data_as(_PD), N, oim.ctypes.data_as(_PD), N, 0, 0)
        if self.lib.xilinx_ip_xfft_v9_1_bitacc_simulate(self.state, inp, ctypes.byref(out)):
            raise RuntimeError("xfft C model failed")
        xr = np.round(ore * 32768).astype(np.int64)
        xi = np.round(oim * 32768).astype(np.int64)
        return xr, xi, bool(out.overflow)


_cm = None


def xfft_cmodel(yi, yq, scale):
    global _cm
    if _cm is None:
        _cm = XfftCModel()
    return _cm(yi, yq, scale)


# ---------------------------------------------------------------- float reference
def fft_ref(yi, yq, scale):
    """numpy: DFT of the windowed samples scaled by the schedule (complex float)."""
    return np.fft.fft(np.asarray(yi, float) + 1j * np.asarray(yq, float)) / 2.0 ** sched_shift(scale)


def fft_err_sigma(scale):
    """Standard deviation of the xfft rounding error of one output component,
    in output LSBs (derivation in README.md "Verification"):

    radix-2 stage s (s = 0..11) rounds its butterfly outputs to the 16-bit
    grid after shifting by b_s bits: each output gets an independent error of
    variance (1 - 4^-b_s) / 12 (convergent rounding of b_s dropped bits;
    0 when b_s = 0) plus the twiddle product rounding (1/12; stages 1..11,
    stage 0 has only trivial twiddles). The error made in stage s reaches an
    output through the 2^(11 - s) paths of the remaining stages, each scaled by
    2^-(b_{s+1} + ... + b_11): variance gain 2^(11 - s) / 4^(B_s)."""
    b = sched_list(scale)
    var = 0.0
    for s in range(LOG2N):
        v = (1 - 4.0 ** -b[s]) / 12 + (1 / 12 if s > 0 else 0)
        rest = sum(b[s + 1:])
        var += v * 2.0 ** (LOG2N - 1 - s) / 4.0 ** rest
    return math.sqrt(var)


# twiddle factors are 16-bit (|error| <= 2^-16 per component and stage): a
# relative magnitude error bound over 12 stages
TWIDDLE_REL = LOG2N * 2.0 ** -15


def power_tolerance(xref_frames, scale, k_sigma=7.0):
    """Per-bin bound on |sum_f |X_rtl|^2 - sum_f |X_ref|^2| (float reference):
    |X_rtl - X_ref| <= E = k sigma sqrt(2) + TWIDDLE_REL |X_ref|  (complex),
    so ||X_rtl|^2 - |X_ref|^2| <= 2 |X_ref| E + E^2 per frame."""
    sig = fft_err_sigma(scale)
    tol = 0.0
    for X in xref_frames:
        a = np.abs(X)
        e = k_sigma * sig * math.sqrt(2) + TWIDDLE_REL * a
        tol = tol + 2 * a * e + e * e
    return tol


# ---------------------------------------------------------------- spectrum
def power(xr, xi):
    xr = np.asarray(xr, np.int64)
    xi = np.asarray(xi, np.int64)
    return xr * xr + xi * xi


def shift_index():
    """Readout word j holds bin k = j ^ 2048."""
    return np.arange(N) ^ HALF


def readout(acc, rshift):
    """40-bit accumulator -> the 32-bit result words (FFT-shifted order) and
    whether any word saturated."""
    v = np.asarray(acc, np.int64) >> rshift
    sat = bool(np.any(v > 0xFFFFFFFF))
    v = np.minimum(v, 0xFFFFFFFF)
    return v[shift_index()], sat


def spectrum(frames, scale, rshift, win=None, cmodel=True):
    """frames: list of (i, q) sample arrays of 4096. Returns (result words,
    saturated, fft overflow, accumulator) using the bit-exact C model."""
    win = window_half() if win is None else win
    acc = np.zeros(N, np.int64)
    ovf = False
    for i, q in frames:
        yi, yq = apply_window(i, q, win)
        xr, xi, o = xfft_cmodel(yi, yq, scale)
        ovf |= o
        acc += power(xr, xi)
    assert np.all(acc < (1 << ACC_BITS))
    res, sat = readout(acc, rshift)
    return res, sat, ovf, acc


def to_db(words, rshift, navg, scale, full_scale=2048, win=None):
    """Result words -> dB relative to a full-scale complex tone of amplitude
    `full_scale` (2048 for the 12-bit ADC) centred on a bin."""
    win = window_half() if win is None else win
    cg = window_full(win).sum() / 2.0 ** WBITS          # coherent gain * N
    ref = (full_scale * cg / 2.0 ** sched_shift(scale)) ** 2
    p = np.asarray(words, float) * 2.0 ** rshift / navg
    return 10 * np.log10(np.maximum(p, 1e-3) / ref)


if __name__ == "__main__":
    write_win_mem()
    for name, s in (("rx (12-bit)", SCALE_RX), ("16-bit", SCALE_16)):
        print(f"{name:12s} SCALE=0x{s:06x} shift={sched_shift(s)} rounding sigma={fft_err_sigma(s):.3f} LSB")
