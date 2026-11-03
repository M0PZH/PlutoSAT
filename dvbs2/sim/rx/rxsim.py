"""Build and run file-driven Verilator testbenches for the RX blocks."""
import os
import subprocess

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RTL = os.path.join(HERE, "..", "..", "rtl")
BUILD = os.path.join(HERE, "build")


def build(top, sources, params=()):
    obj = os.path.join(BUILD, top)
    os.makedirs(obj, exist_ok=True)
    cmd = ["verilator", "--binary", "--timing", "-j", "16", "-O3", "-Wno-fatal",
           "-Wno-WIDTHTRUNC", "-Wno-WIDTHEXPAND", "--top-module", top, "-Mdir", obj,
           "-I" + RTL] + [f"-G{p}" for p in params] + \
          [os.path.join(RTL, s) if not os.path.isabs(s) else s for s in sources]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    return os.path.join(obj, "V" + top)


def run(exe, **plus):
    args = [exe] + [f"+{k}={v}" for k, v in plus.items()]
    r = subprocess.run(args, check=True, capture_output=True, text=True)
    return r.stdout


def write_iq(path, x):
    x = np.asarray(x, np.int64) & 0xFFFF
    with open(path, "w") as f:
        for i, q in x:
            f.write(f"{i:04x} {q:04x}\n")


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
