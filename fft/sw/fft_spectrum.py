#!/usr/bin/env python3
"""Board test for the PL spectrum analyser (fft/): take averaged spectra and
print or save them in dB. Needs numpy; run as root on the board.

    fft_spectrum.py [--src rx0|rx1|nbiq] [--ch N] [--navg N] [--count K]
                    [--fs HZ] [--csv FILE] [--mem]

Finds the UIO device named "fft" (generic-uio, interrupt on DONE); --mem
uses /dev/mem at --base instead (polling). Not tested on hardware yet.
"""
import argparse
import glob
import math
import mmap
import os
import select
import struct
import sys
import time

import numpy as np

# register offsets: regs/fft.toml (regs/gen/fft_regs.h)
VERSION, ID, CTRL, CMD, NAVG, SCALE, RSHIFT, STATUS = 0x000, 0x004, 0x008, 0x00C, 0x010, 0x014, 0x018, 0x01C
DONE, FLAGS, FRAMES, AVGS, DROPS, RES_CFG, RES_SCALE, INFO = 0x020, 0x024, 0x028, 0x02C, 0x030, 0x034, 0x038, 0x03C
WIN, RESULT = 0x2000, 0x4000
ID_VALUE = 0x46465453
N = 4096
SRC = {"rx0": 0, "rx1": 1, "nbiq": 2}
SCALE_RX, SCALE_NBIQ = 0x555400, 0x555550


class Fft:
    def __init__(self, mem=False, base=0x7C540000):
        self.uio = None
        if not mem:
            for d in glob.glob("/sys/class/uio/uio*"):
                if open(os.path.join(d, "name")).read().strip() == "fft":
                    self.uio = os.open("/dev/" + os.path.basename(d), os.O_RDWR)
                    break
            if self.uio is None:
                sys.exit("no UIO device named fft (use --mem)")
            self.mm = mmap.mmap(self.uio, 0x10000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=0)
        else:
            fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
            self.mm = mmap.mmap(fd, 0x10000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=base)
        self.mv = memoryview(self.mm).cast("I")          # 32-bit accesses only
        if self.rd(ID) != ID_VALUE:
            sys.exit(f"bad ID 0x{self.rd(ID):08x}")

    def rd(self, a):
        return self.mv[a >> 2]

    def wr(self, a, v):
        self.mv[a >> 2] = v & 0xFFFFFFFF

    def wait_done(self, timeout=5.0):
        """The interrupt is level (DONE & IRQ_EN); uio_pdrv_genirq masks it
        in its handler, writing 1 to the UIO fd unmasks it."""
        t = time.time()
        while not self.rd(DONE) & 1:
            left = timeout - (time.time() - t)
            if left <= 0:
                raise TimeoutError("DONE not set")
            if self.uio is not None:
                os.write(self.uio, struct.pack("I", 1))
                if select.select([self.uio], [], [], left)[0]:
                    os.read(self.uio, 4)
            else:
                time.sleep(0.001)

    def result(self):
        base = RESULT >> 2
        return np.fromiter((self.mv[base + j] for j in range(N)), np.uint64, N)

    def configure(self, src, ch, navg, scale, rshift, run=False):
        self.wr(CTRL, 0)
        self.wr(CMD, 2)                                  # abort whatever runs
        self.wr(NAVG, navg)
        self.wr(SCALE, scale)
        self.wr(RSHIFT, rshift)
        self.wr(FLAGS, 0xF)
        self.wr(CTRL, int(run) | (self.uio is not None) << 1 | src << 4 | ch << 8)

    def spectra(self, count, run=True):
        """Yield (words, rshift, navg, scale, flags) per published result."""
        if not run:
            self.wr(CMD, 1)
        for k in range(count):
            self.wait_done()
            seq = self.rd(AVGS)
            cfg, scale, flags = self.rd(RES_CFG), self.rd(RES_SCALE), self.rd(FLAGS)
            w = self.result()
            if self.rd(AVGS) != seq:
                print("warning: result changed while reading", file=sys.stderr)
            self.wr(DONE, 1)                             # acknowledge
            if not run and k + 1 < count:
                self.wr(CMD, 1)
            yield w, (cfg >> 8) & 15, (cfg >> 16) & 0x1FF, scale, flags


def to_db(words, rshift, navg, scale, full_scale=2048.0):
    """dB relative to a full-scale complex tone (amplitude full_scale) on a bin
    centre, with the reset (Blackman-Harris) window: sum(w) / 2^17 = 1469.1."""
    shift = sum((scale >> (2 * s)) & 3 for s in range(12))
    ref = (full_scale * 1469.08 / 2.0 ** shift) ** 2
    p = words.astype(float) * 2.0 ** rshift / navg
    return 10 * np.log10(np.maximum(p, 1e-3) / ref)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", choices=SRC, default="rx0")
    ap.add_argument("--ch", type=int, default=0, help="nbiq channel 0..4")
    ap.add_argument("--navg", type=int, default=16)
    ap.add_argument("--scale", type=lambda s: int(s, 0), default=None)
    ap.add_argument("--count", type=int, default=1)
    ap.add_argument("--fs", type=float, default=12e6, help="sample rate of the source (Hz)")
    ap.add_argument("--csv", help="append spectra: one line per result, frequency order")
    ap.add_argument("--mem", action="store_true", help="use /dev/mem instead of UIO")
    ap.add_argument("--base", type=lambda s: int(s, 0), default=0x7C540000)
    a = ap.parse_args()

    f = Fft(a.mem, a.base)
    src = SRC[a.src]
    scale = a.scale if a.scale is not None else (SCALE_NBIQ if src == 2 else SCALE_RX)
    navg = max(1, min(a.navg, 256))
    rshift = min(8, int(math.floor(math.log2(navg))))   # result ~ mean power
    full = 16384.0 if src == 2 else 2048.0     # nbiq: default gain 8
    f.configure(src, a.ch, navg, scale, rshift, run=a.count > 1)
    freq = (np.arange(N) - N // 2) * a.fs / N
    t0 = time.time()
    out = open(a.csv, "a") if a.csv else None
    for k, (w, rsh, nav, sc, flags) in enumerate(f.spectra(a.count, run=a.count > 1)):
        db = to_db(w, rsh, nav, sc, full)
        j = int(np.argmax(db))
        floor = float(np.median(db))
        print(f"{k}: peak {db[j]:6.1f} dBFS at {freq[j] / 1e3:+9.3f} kHz, median {floor:6.1f} dBFS"
              f"{'  flags 0x%x' % flags if flags else ''}")
        if out:
            out.write(",".join(f"{v:.2f}" for v in db) + "\n")
    dt = time.time() - t0
    f.wr(CTRL, 0)
    print(f"{a.count} results in {dt:.2f} s ({a.count / dt:.1f}/s); frames {f.rd(FRAMES)}, drops {f.rd(DROPS)}")


if __name__ == "__main__":
    main()
