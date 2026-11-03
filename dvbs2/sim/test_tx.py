"""dvbs2_tx end to end.

* PLFRAME symbols (tap before shaping) vs gr-dtv dvbs2_physical_cc: exact.
* Dummy PLFRAMEs vs the model.
* Output samples (RRC interpolation + NCO) vs the bit-exact model: exact.
"""

import cocotb
import numpy as np
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

import grref
import txmodel as m
from axis import Sink, Source

TPP = 16


def gr_reference(rate_idx, pilots, gold, seed):
    """One BBFRAME (bits) and gr-dtv's PLFRAME symbol signs for it."""
    rate = m.RATES[rate_idx]
    kbch = m.LDPC[rate]["kbch"]
    rng = np.random.default_rng(seed)
    npk = 2 * kbch // (8 * 188) + 2
    ts = rng.integers(0, 256, (npk, 188), dtype=np.uint8)
    ts[:, 0] = 0x47
    ref = grref.tx_chain(ts.ravel(), rate, pilots=bool(pilots), goldcode=gold)
    nsym = 90 + 32400 + (22 * 36 if pilots else 0)
    pl = ref["plframe"][0::2][:nsym]   # gr-dtv zero-stuffs to 2 samples/symbol
    signs = np.stack([pl.real < 0, pl.imag < 0], axis=1).astype(np.uint8)
    return ref["bbframe"][:kbch], signs


async def setup(dut, sps, gold, finc, rolloff=0.35):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.enable.value = 0
    dut.sps.value = sps
    xa, xb = m.gold_init(gold)
    dut.gold_xa.value = xa
    dut.gold_xb.value = xb
    dut.nco_finc.value = finc
    dut.nco_bypass.value = 0
    dut.coef_we.value = 0
    await ClockCycles(dut.clk, 4)
    dut.rst.value = 0
    coefs = m.rrc_coefs(sps, rolloff, TPP)
    for p in range(sps):
        for j in range(TPP):
            dut.coef_addr.value = p * TPP + j
            dut.coef_data.value = int(coefs[p + sps * j]) & 0x3FFFF
            dut.coef_we.value = 1
            await RisingEdge(dut.clk)
    dut.coef_we.value = 0
    return coefs


async def symbol_monitor(dut, out):
    while True:
        await ReadOnly()
        if dut.sym_valid.value:
            out.append((int(dut.sym_si.value), int(dut.sym_sq.value),
                        int(dut.sym_sof.value), int(dut.sym_dummy.value)))
        await RisingEdge(dut.clk)


def split_frames(syms):
    syms = np.array(syms, dtype=np.uint8)
    starts = list(np.flatnonzero(syms[:, 2])) + [len(syms)]
    return [(bool(syms[a, 3]), syms[a:b, :2]) for a, b in zip(starts[:-1], starts[1:])]


def to_signed16(v):
    return v - 65536 if v >= 32768 else v


async def run(dut, frames, sps, gold, finc, stall, check_samples):
    coefs = await setup(dut, sps, gold, finc)
    syms = []
    cocotb.start_soon(symbol_monitor(dut, syms))
    snk = Sink(dut, ["i", "q"], stall_prob=stall, seed=5)
    cocotb.start_soon(snk.run())
    src = Source(dut)

    refs = [gr_reference(r, p, gold, seed=50 + k) for k, (r, p) in enumerate(frames)]
    beats = []
    for (r, p), (bb, _) in zip(frames, refs):
        by = np.packbits(bb)
        beats += [{"data": int(b), "user": (p << 4) | r, "last": int(k == len(by) - 1)}
                  for k, b in enumerate(by)]
    dut.enable.value = 1
    cocotb.start_soon(src.send(beats))

    # run until the frame after the last data frame has started
    def nframes():
        return int(dut.cnt_data_frames.value) + int(dut.cnt_dummy_frames.value)
    target = None
    for _ in range(20_000_000):
        await RisingEdge(dut.clk)
        if target is None and int(dut.cnt_data_frames.value) == len(frames):
            target = nframes() + 1
        if target is not None and nframes() >= target:
            break
    await ClockCycles(dut.clk, 50)
    dut.enable.value = 0
    assert int(dut.cnt_len_errors.value) == 0

    got = split_frames(syms)
    data = [f for d, f in got if not d]
    dummies = [f for d, f in got if d]
    dut._log.info(f"captured {len(got)} PLFRAMEs: {len(data)} data, {len(dummies)} dummy")
    assert len(data) >= len(frames)

    for k, ((r, p), (_, ref)) in enumerate(zip(frames, refs)):
        f = data[k]
        assert len(f) == len(ref), (len(f), len(ref))
        bad = np.flatnonzero((f != ref).any(axis=1))
        assert bad.size == 0, f"data frame {k}: {bad.size} symbol errors, first {bad[:5]}"
        dut._log.info(f"PLFRAME {k} (MODCOD {r + 1}, pilots {p}): {len(f)} symbols match gr-dtv")

    dref = m.dummy_plframe_signs(gold)
    for f in dummies[:-1]:            # the last one may be truncated
        assert np.array_equal(f, dref), "dummy PLFRAME mismatch"
    dut._log.info(f"{len(dummies) - 1} complete dummy PLFRAMEs match the model")

    if check_samples:
        allsyms = np.array(syms, dtype=np.uint8)[:, :2]
        model = m.nco_mix(m.rrc_interp(m.signs_to_iq(allsyms), coefs, sps, TPP), finc)
        rtl = np.array([(to_signed16(i), to_signed16(q)) for i, q in snk.beats], dtype=np.int64)
        n = min(len(rtl), len(model))
        assert n > 1000
        bad = np.flatnonzero((rtl[:n] != model[:n]).any(axis=1))
        assert bad.size == 0, f"{bad.size} sample mismatches, first {bad[:5]}: {rtl[bad[:3]]} vs {model[bad[:3]]}"
        dut._log.info(f"{n} output samples (RRC sps={sps} + NCO) match the model bit-exactly")


@cocotb.test()
async def tx_symbols_and_samples(dut):
    """Two data frames (1/2 pilots, 9/10 no pilots) among dummies, sps=4, NCO on."""
    await run(dut, [(3, 1), (10, 0)], sps=4, gold=0, finc=0x0A3D70A4, stall=0.3,
              check_samples=True)


@cocotb.test()
async def tx_gold_code(dut):
    """Gold code n = 12345, rates 1/4 and 8/9 with pilots."""
    await run(dut, [(0, 1), (9, 1)], sps=2, gold=12345, finc=0, stall=0.0,
              check_samples=False)
