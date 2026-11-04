"""Build and run file-driven Verilator testbenches for the RX blocks."""
import os
import subprocess

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RTL = os.path.join(HERE, "..", "..", "rtl")
BUILD = os.path.join(HERE, "build")


def build(top, sources, params=(), defines=()):
    obj = os.path.join(BUILD, top)
    os.makedirs(obj, exist_ok=True)
    cmd = ["verilator", "--binary", "--timing", "-j", "16", "-O3", "-Wno-fatal",
           "-Wno-WIDTHTRUNC", "-Wno-WIDTHEXPAND", "--top-module", top, "-Mdir", obj,
           "-I" + RTL] + [f"-G{p}" for p in params] + [f"-D{d}" for d in defines] + \
          [os.path.join(RTL, s) if not os.path.isabs(s) else s for s in sources]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    return os.path.join(obj, "V" + top)


def run(exe, **plus):
    args = [exe] + [f"+{k}={v}" for k, v in plus.items()]
    r = subprocess.run(args, check=True, capture_output=True, text=True)
    return r.stdout


_HEX = np.frombuffer(b"0123456789abcdef", np.uint8)


def write_iq(path, x):
    """'iiii qqqq' lines (16-bit hex), vectorized."""
    x = np.asarray(x, np.int64).reshape(-1, 2) & 0xFFFF
    b = np.empty((len(x), 10), np.uint8)
    for c in range(2):
        for d in range(4):
            b[:, 5 * c + d] = _HEX[(x[:, c] >> (12 - 4 * d)) & 15]
    b[:, 4] = ord(" ")
    b[:, 9] = ord("\n")
    with open(path, "wb") as f:
        f.write(b.tobytes())


def read_hex_cols(path, widths):
    rows = []
    with open(path) as f:
        for line in f:
            rows.append([int(v, 16) for v in line.split()])
    a = np.array(rows, np.int64).reshape(-1, len(widths))
    for c, w in enumerate(widths):
        if w < 0:                       # signed
            w = -w
            a[:, c] = np.where(a[:, c] >= 1 << (w - 1), a[:, c] - (1 << w), a[:, c])
    return a
