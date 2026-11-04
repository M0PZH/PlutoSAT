#!/usr/bin/env python3
"""FFT result DMA board test: fft (CTRL.STREAM) -> axi_fft_dma (S_AXI_HP3)
-> ring (UIO fft map2). Queues transfers of 16400 bytes (FLAGS TLAST),
checks every packet's MAGIC / LOG2N / SRC / NAVG / SEQ (gaps), then takes
4 single averages (STREAM without RUN) and compares each packet with RESULT
read over AXI-Lite; prints the peak bins. Run as root on the board; needs
numpy.

  fft_dmatest.py SRC CH LEN NAVG COUNT   (SRC 0 rx0, 1 rx1, 2 nbiq)

axi_dmac takes one transfer as active and holds one more in TRANSFER_SUBMIT:
two are queued at a time. SRC 2 needs the nbiq channel CH running (e.g.
`nbiq -c 1:100k:48k -o /dev/null -q &`): without it there are no samples.
"""
import glob
import mmap
import os
import sys
import time

import numpy as np

CTRL, CMD, NAVG, SCALE, RSHIFT, DONE, FLAGS, AVGS, STR_SENT, STR_DROPS = \
    0x008, 0x00C, 0x010, 0x014, 0x018, 0x020, 0x024, 0x02C, 0x040, 0x044
RESULT = 0x4000
LENS = {4096: 0, 2048: 1, 1024: 2, 512: 3}
# axi_dmac
D_IRQ_MASK, D_CONTROL, D_ID, D_SUBMIT, D_FLAGS, D_DEST, D_XLEN, D_DONE = \
    0x080, 0x400, 0x404, 0x408, 0x40C, 0x410, 0x418, 0x428
SLOT = 16400


def uio_maps(name):
    for d in glob.glob("/sys/class/uio/uio*"):
        if open(d + "/name").read().strip() == name:
            fd = os.open("/dev/" + os.path.basename(d), os.O_RDWR)
            maps = []
            for k in range(3):
                size = int(open(f"{d}/maps/map{k}/size").read(), 16)
                addr = int(open(f"{d}/maps/map{k}/addr").read(), 16)
                m = mmap.mmap(fd, size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE,
                              offset=k * mmap.PAGESIZE)
                maps.append((addr, memoryview(m).cast("I")))
            return maps
    sys.exit(f"no UIO {name}")


def main(src, ch, n, navg, count):
    (_, r), (_, d), (ring_pa, ring) = uio_maps("fft")
    scale = 0x555550 if src == 2 else 0x555400
    rd = lambda m, a: m[a >> 2]

    def wr(m, a, v):
        m[a >> 2] = v & 0xFFFFFFFF

    # fft off, DMA reset and enabled
    wr(r, CTRL, 0); wr(r, CMD, 2)
    wr(d, D_CONTROL, 0); wr(d, D_IRQ_MASK, 3); wr(d, D_CONTROL, 1)
    nslots = min(len(ring) * 4 // SLOT, 32)
    pend = []                                   # (id, slot)
    nxt = 0

    def submit():
        nonlocal nxt
        t = time.time()
        while rd(d, D_SUBMIT) & 1:               # one active + one waiting here
            if time.time() - t > 5:
                sys.exit(f"submit: queue stuck (STR_SENT {rd(r, STR_SENT)})")
        tid = rd(d, D_ID) & 3
        wr(d, D_DEST, ring_pa + nxt * SLOT)
        wr(d, D_XLEN, SLOT - 1)
        wr(d, D_FLAGS, 2)                       # TLAST ends the transfer
        wr(d, D_SUBMIT, 1)
        pend.append((tid, nxt))
        nxt = (nxt + 1) % nslots

    for _ in range(2):
        submit()
    s0, d0 = rd(r, STR_SENT), rd(r, STR_DROPS)
    wr(r, NAVG, navg); wr(r, SCALE, scale); wr(r, RSHIFT, 0); wr(r, FLAGS, 0x1F)
    wr(r, CTRL, 1 | 1 << 2 | src << 4 | ch << 8 | LENS[n] << 12)   # RUN | STREAM

    ok = bad = gaps = cmp_ok = cmp_bad = 0
    last_seq = None
    peaks = []
    t0 = time.time()
    for k in range(count):
        tid, slot = pend.pop(0)
        t = time.time()
        while not (rd(d, D_DONE) >> tid) & 1:
            if time.time() - t > 5:
                sys.exit(f"packet {k}: transfer {tid} not done (STR_SENT {rd(r, STR_SENT) - s0})" +
                         (": is nbiq channel %d running?" % ch if src == 2 and rd(r, STR_SENT) == s0 else ""))
        w = np.array(ring[slot * SLOT // 4: slot * SLOT // 4 + 4 + n], np.uint32)
        submit()
        magic, log2n, s_src, seq, navg_p = w[0] & 0xFF, (w[0] >> 8) & 15, (w[0] >> 12) & 3, int(w[1]), w[3] & 0x1FF
        good = magic == 0xF5 and (1 << log2n) == n and s_src == src and navg_p == navg
        if last_seq is not None and seq != (last_seq + 1) & 0xFFFFFFFF:
            gaps += 1
        last_seq = seq
        ok += good
        bad += not good
        data = w[4:4 + n]
        peaks.append(int(np.argmax(data)))
    dt = time.time() - t0
    wr(r, CTRL, 0); wr(r, CMD, 2)
    time.sleep(0.05)
    # single averages (STREAM without RUN): the published result stays in the
    # buffer, so the packet must equal RESULT read over AXI-Lite
    for _ in range(len(pend)):                  # drop what is still queued
        pend.pop(0)
    wr(d, D_CONTROL, 0); wr(d, D_CONTROL, 1)
    for _ in range(2):
        submit()
    wr(r, CTRL, 1 << 2 | src << 4 | ch << 8 | LENS[n] << 12)
    for k in range(4):
        wr(r, CMD, 1)
        tid, slot = pend.pop(0)
        t = time.time()
        while not (rd(d, D_DONE) >> tid) & 1:
            if time.time() - t > 5:
                sys.exit(f"single {k}: transfer {tid} not done")
        w = np.array(ring[slot * SLOT // 4: slot * SLOT // 4 + 4 + n], np.uint32)
        lite = np.array(r[RESULT >> 2:(RESULT >> 2) + n], np.uint32)
        if np.array_equal(lite, w[4:4 + n]) and int(w[1]) == rd(r, AVGS):
            cmp_ok += 1
        else:
            cmp_bad += 1
        submit()
        wr(r, DONE, 1)
    wr(r, CTRL, 0); wr(r, CMD, 2)
    time.sleep(0.05)
    wr(d, D_CONTROL, 0)
    print(f"src {src} ch {ch} N {n} navg {navg}: {count} packets in {dt:.2f} s, header ok {ok} bad {bad}, "
          f"SEQ gaps {gaps}, STR_SENT {rd(r, STR_SENT) - s0} STR_DROPS {rd(r, STR_DROPS) - d0}, "
          f"AXI-Lite compare equal {cmp_ok} differ {cmp_bad}, peak bins {sorted(set(peaks))[:5]}")
    return bad == 0 and cmp_bad == 0


if __name__ == "__main__":
    if len(sys.argv) != 6:
        sys.exit(__doc__)
    sys.exit(0 if main(*map(int, sys.argv[1:])) else 1)
