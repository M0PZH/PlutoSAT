"""DDR LLR queue (dvbs2_rx_llrq, RX2 step 3) -> LDPC decoder, the queue's
AXI master on a memory model with random stalls and latency (axi_mem.sv).
Every decoded frame equals model/ldpcdec.py (bits, iterations, convergence)
and carries its own slot, MODCOD, pilots and rate.

    cd sim/rx && ../../.venv/bin/python -m pytest -v test_llrq.py
"""
import os
import re
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "model"))
sys.path.insert(0, HERE)

import rxsim                     # noqa: E402

WORK = os.path.join(rxsim.BUILD, "work")
BASE = 0x3E800000
WIN = dict(memlo=f"{BASE:x}", memhi=f"{BASE + 0x200000:x}")     # the 2 MB region


@pytest.fixture(scope="module")
def llrq_exe():
    return rxsim.build("tb_llrq", ["gen/dvbs2_ldpc_dec_pkg.sv", "dvbs2_ldpc_dec.sv", "dvbs2_rx_llrq.sv",
                                   os.path.join(HERE, "axi_mem.sv"), os.path.join(HERE, "tb_llrq.sv")])


def make_frames(cases, seed):
    """[(rate_idx, esn0, pilots)] -> [dict(llr, rate, modcod, pilots, chan)]."""
    import ldpcdec as ld
    import txmodel as m
    rng = np.random.default_rng(seed)
    out = []
    for rate_idx, esn0, pilots in cases:
        code = ld.Code(rate_idx)
        bb = rng.integers(0, 2, code.kbch).astype(np.uint8)
        llr = ld.awgn_llr(m.fec_frame(bb, rate_idx), esn0, rng)
        out.append(dict(llr=llr, rate=rate_idx, modcod=rate_idx + 1, pilots=int(pilots),
                        chan=np.packbits((llr[:code.K] < 0).astype(np.uint8))))
    return out


def schedule(slots, burst=1440):
    """Interleave the slots' words in time. slots: [dict(frames, period,
    offset, cut={frame: words})]: each slot sends its words in bursts of
    `burst` (one per clock) every burst * period clocks, i.e. one word per
    `period` clocks on average; a frame in `cut` is abandoned after that many
    words. Returns the columns (gap, i, q, first, slot, modcod, pilots), the
    words ordered by time, one per clock at most."""
    cols = []
    for s, sl in enumerate(slots):
        parts = []
        for f, fr in enumerate(sl["frames"]):
            n = sl.get("cut", {}).get(f, 32400)
            u = fr["llr"].astype(np.int64).reshape(-1, 2)[:n] & 0xFF
            first = np.zeros(n, np.int64)
            first[0] = 1
            parts.append(np.column_stack([u, first, np.full(n, s), np.full(n, fr["modcod"]),
                                          np.full(n, fr["pilots"])]))
        w = np.concatenate(parts)
        k = np.arange(len(w))
        t = sl.get("offset", 0) + (k // burst) * burst * sl["period"] + (k % burst)
        cols.append(np.column_stack([t, w]))
    a = np.concatenate(cols)
    a = a[np.lexsort((a[:, 4], a[:, 0]))]
    t = a[:, 0] - np.arange(len(a))
    t = np.maximum.accumulate(t) + np.arange(len(a))                # at most one word per clock
    gap = np.diff(np.concatenate([[-1], t]))
    return np.column_stack([gap, a[:, 1:]])


def write_words(path, w):
    with open(path, "w") as f:
        f.write("".join(f"{g} {i:02x} {q:02x} {fi} {s} {mc} {p}\n" for g, i, q, fi, s, mc, p in w.tolist()))


def run_queue(exe, name, slots, burst=1440, **plus):
    os.makedirs(WORK, exist_ok=True)
    fin = os.path.join(WORK, f"q_in_{name}.hex")
    fout = os.path.join(WORK, f"q_out_{name}.hex")
    write_words(fin, schedule(slots, burst))
    args = dict(WIN)
    args.update(plus)
    log = rxsim.run(exe, **args, **{"in": fin, "out": fout})
    print(log)
    st = {k: int(v) for k, v in re.findall(r"(\w+)=(\d+)", log.split("tb_llrq:")[1])}
    got = rxsim.read_hex_cols(fout, (8, 1, 1, 6, 8, 4, 8))
    ends = np.nonzero(got[:, 1])[0]
    frames = np.split(got, ends[:-1] + 1) if len(ends) else []
    assert len(ends) == 0 or ends[-1] == len(got) - 1
    return st, frames


def check_frames(slots, frames, max_it, it_lo=None):
    """Each output frame: the next not yet seen input frame of its slot (by
    the channel decisions), equal to the model at the iteration count it
    reports. Returns, per slot, the input frames decoded and their iterations."""
    import ldpcdec as ld
    nxt = [0, 0]
    seen = [[], []]
    for k, g in enumerate(frames):
        tag = int(g[0, 6])
        s, pil, mc = (tag >> 6) & 1, (tag >> 5) & 1, tag & 31
        assert np.all(g[:, 6] == tag) and np.all(g[:, 5] == mc - 1), f"output frame {k}: tag/rate"
        fl = slots[s]["frames"]
        cut = slots[s].get("cut", {})
        j = nxt[s]
        while j < len(fl) and not (j not in cut and len(fl[j]["chan"]) == len(g)
                                   and np.array_equal(fl[j]["chan"], g[:, 4])):
            j += 1
        assert j < len(fl), f"output frame {k} (slot {s}) is no input frame of the slot, in order"
        fr = fl[j]
        assert (mc, pil) == (fr["modcod"], fr["pilots"])
        it, ok = int(g[-1, 3]), bool(g[-1, 2])
        hard, it_m, conv = ld.Decoder(fr["rate"]).decode(fr["llr"], max_it=it if not ok else max_it[s])
        K = ld.Code(fr["rate"]).K
        assert np.array_equal(g[:, 0], np.packbits(hard[:K])), f"output frame {k} (slot {s}, {j}): data"
        assert (it_m, conv) == (it, ok), (k, s, j, it_m, conv, it, ok)
        if not ok and it != max_it[s]:
            assert it_lo is not None and it_lo <= it < max_it[s], (k, it)    # stopped by the cap
        seen[s].append((j, it))
        nxt[s] = j + 1
    return seen


def test_llrq_two_slots(llrq_exe):
    """Both slots at the maximum total rate (2.38 MBd: one word per 42
    clocks), 1.5 MBd QPSK 3/4 pilots + 0.88 MBd QPSK 1/2 without pilots, in
    bursts of 1440 words at one per clock, the memory with 20 % stalls and
    up to 120 clocks latency: every frame decoded, in order per slot, none
    dropped, the front end stalled only briefly."""
    f0 = make_frames([(6, 5.0, 1)] * 4, seed=1)
    f1 = make_frames([(3, 2.2, 0)] * 3, seed=2)
    slots = [dict(frames=f0, period=67), dict(frames=f1, period=114, offset=5000)]
    st, frames = run_queue(llrq_exe, "two", slots, axistall=20, axilat=120, stall=5, maxit0=30, maxit1=25)
    seen = check_frames(slots, frames, (30, 25))
    print(st, seen)
    assert [j for j, _ in seen[0]] == list(range(4)) and [j for j, _ in seen[1]] == list(range(3))
    assert st["drop0"] == st["drop1"] == 0 and st["axi_err"] == 0 and st["qframes"] == 7
    assert st["stall_max"] < 300, st          # the input FIFO absorbs ~10k clocks


def test_llrq_backpressure(llrq_exe):
    """Heavy AXI backpressure (70 % stalls on every channel, latency up to
    400 clocks), the decoder output stalled 40 %, short bursts (the
    combined input one word per clock most of the time): the data still
    arrives intact, in order."""
    f0 = make_frames([(10, 7.5, 1), (0, -1.0, 1), (10, 7.5, 1)], seed=3)
    f1 = make_frames([(5, 4.0, 1), (5, 4.0, 1)], seed=4)
    slots = [dict(frames=f0, period=3), dict(frames=f1, period=3, offset=7)]
    st, frames = run_queue(llrq_exe, "bp", slots, burst=64, axistall=70, axilat=400, stall=40)
    seen = check_frames(slots, frames, (30, 30))
    print(st, seen)
    assert len(seen[0]) == 3 and len(seen[1]) == 2 and st["drop0"] == st["drop1"] == 0
    assert st["stall"] > 0                    # the queue did push back


def test_llrq_drop(llrq_exe):
    """Over budget: frames arrive far faster than the decoder (QPSK 1/4 at
    -3 dB, 20 iterations each, ~1.2M clocks, against one frame per ~130k
    clocks) with 2-frame rings: frames that find their ring full are dropped
    and counted; the rest come out intact, in order."""
    f0 = make_frames([(0, -3.0, 1)] * 6, seed=5)
    f1 = make_frames([(1, -2.5, 0)] * 5, seed=6)
    slots = [dict(frames=f0, period=4), dict(frames=f1, period=5, offset=11)]
    st, frames = run_queue(llrq_exe, "drop", slots, axistall=10, axilat=60, nfr=2, maxit0=20, maxit1=20)
    seen = check_frames(slots, frames, (20, 20))
    print(st, seen)
    assert st["drop0"] > 0 and st["drop1"] > 0
    assert len(seen[0]) + st["drop0"] == 6 and len(seen[1]) + st["drop1"] == 5
    assert st["qframes"] == len(frames) and st["stall_max"] < 300


def test_llrq_itcap(llrq_exe):
    """Shared iteration budget: while more than QHI = 1 frames wait, frames
    stop at MAX_IT_LO = 3 iterations (QPSK 1/4 at -3 dB never converges):
    the backlog drains, nothing is dropped, and each frame equals the model
    at the iteration count it got."""
    f0 = make_frames([(0, -3.0, 1)] * 7, seed=7)
    f1 = make_frames([(0, -3.0, 0)] * 4, seed=8)
    slots = [dict(frames=f0, period=8), dict(frames=f1, period=10, offset=3)]
    st, frames = run_queue(llrq_exe, "cap", slots, axistall=10, axilat=60, qhi=1, itlo=3, maxit0=25, maxit1=20)
    seen = check_frames(slots, frames, (25, 20), it_lo=3)
    print(st, seen)
    its = [it for s in seen for _, it in s]
    assert st["drop0"] == st["drop1"] == 0 and len(seen[0]) == 7 and len(seen[1]) == 4
    assert any(it in (25, 20) for it in its) and sum(3 <= it < 20 for it in its) >= 3, its


def test_llrq_abort(llrq_exe):
    """Abandoned frames (an s_first before the frame's last word: e.g. a
    per-slot reset of the front end): cut inside a burst, on a burst
    boundary, in the frame's last 4-beat burst and after one word. They do
    not come out; the next frames of the slot reuse the ring entry and
    decode."""
    f0 = make_frames([(3, 2.5, 1)] * 5, seed=9)
    f1 = make_frames([(7, 6.0, 1)] * 6, seed=10)
    slots = [dict(frames=f0, period=6, cut={1: 1000, 3: 32385}),
             dict(frames=f1, period=7, offset=2, cut={0: 4096, 2: 1, 3: 32399})]
    st, frames = run_queue(llrq_exe, "abort", slots, burst=500, axistall=30, axilat=100, nfr=3)
    seen = check_frames(slots, frames, (30, 30))
    print(st, seen)
    assert [j for j, _ in seen[0]] == [0, 2, 4] and [j for j, _ in seen[1]] == [1, 4, 5]
    assert st["drop0"] == st["drop1"] == 0 and st["qframes"] == 6


def test_llrq_reset(llrq_exe):
    """Soft reset with transfers in flight (write bursts, responses and
    read data owed): the queue drains the AXI side, then starts empty; the
    front end restarts at a frame start, and every frame after that decodes."""
    f0 = make_frames([(4, 3.5, 1)] * 5, seed=11)
    f1 = make_frames([(8, 6.5, 0)] * 5, seed=12)
    slots = [dict(frames=f0, period=5), dict(frames=f1, period=6, offset=1)]
    st, frames = run_queue(llrq_exe, "rst", slots, burst=200, axistall=40, axilat=300, rstat=300000)
    seen = check_frames(slots, frames, (30, 30))
    print(st, seen)
    # the reset at clock 300k: frames 0 / 0 were in the decoder or the queue,
    # frame 1 of each slot was being written; the following ones all decode
    assert [j for j, _ in seen[0]][-3:] == [2, 3, 4] and [j for j, _ in seen[1]][-3:] == [2, 3, 4]
