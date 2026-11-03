"""dvbs2_fec (BB scrambler + BCH + LDPC) vs gr-dtv, bit exact."""

import cocotb
import numpy as np
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

import grref
import txmodel as m
from axis import Sink, Source


def gr_bbframes(rate_idx, nframes, seed):
    rate = m.RATES[rate_idx]
    kbch = m.LDPC[rate]["kbch"]
    rng = np.random.default_rng(seed)
    npk = (nframes + 1) * kbch // (8 * 188) + 2
    ts = rng.integers(0, 256, (npk, 188), dtype=np.uint8)
    ts[:, 0] = 0x47
    bb = grref.bbheader(ts.ravel(), rate)[: nframes * kbch]
    ref = grref.ldpc(grref.bch(grref.bbscrambler(bb, rate), rate), rate)
    return bb.reshape(nframes, kbch), ref.reshape(nframes, 64800)


async def run_rates(dut, rates, pilots, idle, stall):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    src = Source(dut, idle_prob=idle, seed=11)
    snk = Sink(dut, ["data", "user", "last"], stall_prob=stall, seed=12)
    await ClockCycles(dut.clk, 4)
    dut.rst.value = 0
    cocotb.start_soon(snk.run())

    beats, refs, users = [], [], []
    for i, r in enumerate(rates):
        bb, ref = gr_bbframes(r, 1, seed=100 + i)
        user = (pilots << 4) | r
        by = np.packbits(bb[0])
        for k, b in enumerate(by):
            beats.append({"data": int(b), "user": user, "last": int(k == len(by) - 1)})
        refs.append(ref[0])
        users.append(user)
    await src.send(beats)

    total = 64800 * len(rates)
    for _ in range(4_000_000):
        if len(snk.beats) >= total:
            break
        await RisingEdge(dut.clk)
    assert len(snk.beats) == total, f"got {len(snk.beats)} of {total} bits"
    out = np.array(snk.beats)
    for f, (ref, user) in enumerate(zip(refs, users)):
        seg = out[f * 64800:(f + 1) * 64800]
        bad = np.flatnonzero(seg[:, 0] != ref)
        assert bad.size == 0, f"frame {f} rate {m.RATES[user & 15]}: {bad.size} bit errors, first {bad[:5]}"
        assert np.all(seg[:, 1] == user)
        assert seg[-1, 2] == 1 and seg[:-1, 2].sum() == 0
        dut._log.info(f"frame {f} rate {m.RATES[user & 15]}: 64800 bits match gr-dtv")


@cocotb.test()
async def fec_all_rates(dut):
    """All 11 QPSK code rates back to back, no stalls."""
    await run_rates(dut, list(range(11)), pilots=1, idle=0.0, stall=0.0)


@cocotb.test()
async def fec_backpressure(dut):
    """Random input gaps and output backpressure."""
    await run_rates(dut, [4, 0, 10], pilots=0, idle=0.3, stall=0.4)
