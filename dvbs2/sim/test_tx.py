"""dvbs2_tx end to end.

* PLFRAME symbols (tap before shaping) vs gr-dtv dvbs2_physical_cc: exact.
* Dummy PLFRAMEs vs the model.
* Output samples (RRC interpolation + NCO) vs the bit-exact model: exact.
* Low symbol rates (RRC by P + CIC by R, P x R up to 360 samples per symbol)
  and FIR-only rates: dummy-frame symbols and every output sample vs the
  model, with output backpressure; the CIC at R = 32 with a full-scale input.
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


async def setup(dut, sps, gold, finc, rolloff=0.35, cfg=None):
    """Reset and configure; sps = samples per symbol of the RRC alone, or
    cfg = a txmodel.tx_config() dict (P, R, cic_shift, coefs). Returns the
    coefficients."""
    if cfg is None:
        cfg = dict(P=sps, R=1, cic_shift=0, coefs=m.rrc_coefs(sps, rolloff, TPP))
    sps = cfg["P"]
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.enable.value = 0
    dut.sps.value = sps
    dut.cic_r.value = cfg["R"] if cfg["R"] >= 2 else 0
    dut.cic_shift.value = cfg["cic_shift"]
    xa, xb = m.gold_init(gold)
    dut.gold_xa.value = xa
    dut.gold_xb.value = xb
    dut.nco_finc.value = finc
    dut.nco_bypass.value = 0
    dut.carrier_only.value = 0
    dut.coef_we.value = 0
    await ClockCycles(dut.clk, 4)
    dut.rst.value = 0
    coefs = cfg["coefs"]
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


@cocotb.test()
async def tx_carrier_only(dut):
    """Carrier-only test tone: constant symbols through the RRC and the NCO,
    bit exact against the model fed with all-zero sign pairs."""
    sps, finc = 4, 0x0C49BA5E
    coefs = await setup(dut, sps, 0, finc)
    snk = Sink(dut, ["i", "q"], stall_prob=0.2, seed=9)
    cocotb.start_soon(snk.run())
    dut.carrier_only.value = 1
    dut.enable.value = 1
    while len(snk.beats) < 6000:
        await RisingEdge(dut.clk)
    dut.enable.value = 0
    rtl = np.array([(to_signed16(i), to_signed16(q)) for i, q in snk.beats], dtype=np.int64)
    model = m.nco_mix(m.rrc_interp(m.signs_to_iq(np.zeros((len(rtl) // sps + 64, 2), np.uint8)),
                                   coefs, sps, TPP), finc)
    n = len(rtl)
    bad = np.flatnonzero((rtl[:n] != model[:n]).any(axis=1))
    assert bad.size == 0, f"{bad.size} sample mismatches, first {bad[:5]}"
    amp = np.abs(rtl[2000:, 0] + 1j * rtl[2000:, 1])
    assert amp.std() < 0.01 * amp.mean(), "not a constant-envelope tone"
    dut._log.info(f"{n} carrier-only samples match the model; amplitude {amp.mean():.0f}")


async def run_rate(dut, cfg, finc, nsym, stall=0.3, carrier=False):
    """Dummy PLFRAMEs (no data) through a tx_config() shaping setup: the
    symbol tap must equal the model's dummy frame, and every output sample
    nco_mix(tx_shape(symbols)) (whole frames would be too slow to simulate at
    360 samples per symbol)."""
    await setup(dut, 0, 0, finc, cfg=cfg)
    sps = cfg["P"] * cfg["R"]
    syms = []
    cocotb.start_soon(symbol_monitor(dut, syms))
    snk = Sink(dut, ["i", "q"], stall_prob=stall, seed=11)
    cocotb.start_soon(snk.run())
    dut.carrier_only.value = int(carrier)
    dut.enable.value = 1
    while len(snk.beats) < nsym * sps:
        await RisingEdge(dut.clk)
    dut.enable.value = 0
    s = np.array(syms, dtype=np.uint8)[:, :2]
    dref = m.dummy_plframe_signs(0)
    assert len(s) <= len(dref) and np.array_equal(s, dref[:len(s)]), "dummy PLFRAME symbols differ"
    if carrier:
        s = np.zeros_like(s)
    model = m.nco_mix(m.tx_shape(m.signs_to_iq(s), cfg, TPP), finc)
    rtl = np.array([(to_signed16(i), to_signed16(q)) for i, q in snk.beats], dtype=np.int64)
    n = len(rtl)
    assert len(model) >= n
    bad = np.flatnonzero((rtl != model[:n]).any(axis=1))
    assert bad.size == 0, f"{bad.size} sample mismatches, first {bad[:5]}: {rtl[bad[:3]]} vs {model[bad[:3]]}"
    rms = np.sqrt((rtl[n // 2:] ** 2).sum(axis=1).mean())
    dut._log.info(f"sps {sps} = P {cfg['P']} x R {cfg['R']} (shift {cfg['cic_shift']}): {len(s)} symbols, "
                  f"{n} samples match the model bit-exactly; rms {rms:.0f}, peak {np.abs(rtl).max()}")
    return rtl


def rate_cfg(sps, rolloff):
    """Shaping for 12 MS/s / sps (what dvbs2-tx programs)."""
    return m.tx_config(sps, rolloff, TPP)


@cocotb.test()
async def tx_rate_33k(dut):
    """33.33 kS/s: 360 samples per symbol = RRC x60, CIC x6, NCO on."""
    await run_rate(dut, rate_cfg(360, 0.35), 0x01234567, nsym=260)


@cocotb.test()
async def tx_rate_66k(dut):
    """66.67 kS/s: 180 = 60 x 3 (R not a power of 2), roll-off 0.2."""
    await run_rate(dut, rate_cfg(180, 0.20), 0xF0000000, nsym=300)


@cocotb.test()
async def tx_rate_125k(dut):
    """125 kS/s: 96 = 48 x 2, roll-off 0.25, no backpressure."""
    await run_rate(dut, rate_cfg(96, 0.25), 0x00C49BA6, nsym=400, stall=0.0)


@cocotb.test()
async def tx_rate_250k(dut):
    """250 kS/s: 48 samples per symbol, RRC alone (CIC off)."""
    cfg = rate_cfg(48, 0.35)
    assert cfg["R"] == 1
    await run_rate(dut, cfg, 0x0A3D70A4, nsym=500)


@cocotb.test()
async def tx_rate_333k(dut):
    """333.33 kS/s: 36 samples per symbol, RRC alone."""
    cfg = rate_cfg(36, 0.20)
    assert cfg["R"] == 1
    await run_rate(dut, cfg, 0, nsym=500)


@cocotb.test()
async def tx_cic_full_scale(dut):
    """CIC at R = 32 (shift 10) behind P = 2 with the largest taps: the RRC
    output saturates at +-full scale, so the integrators reach their
    largest values (exact in 26 bits) and the output saturation is used."""
    cfg = dict(P=2, R=32, cic_shift=m.cic_shift_for(32),
               coefs=np.full(2 * TPP, (1 << 17) - 1, dtype=np.int64))
    cfg["coefs"][1::4] = -(1 << 17)
    rtl = await run_rate(dut, cfg, 0, nsym=120, stall=0.2)
    assert np.abs(rtl).max() >= 32767


@cocotb.test()
async def tx_cic_carrier_only(dut):
    """Carrier-only tone through the CIC (360 = 60 x 6) and the NCO."""
    rtl = await run_rate(dut, rate_cfg(360, 0.35), 0x0C49BA5E, nsym=60, carrier=True)
    amp = np.abs(rtl[len(rtl) // 2:, 0] + 1j * rtl[len(rtl) // 2:, 1])
    assert amp.std() < 0.01 * amp.mean(), "not a constant-envelope tone"
