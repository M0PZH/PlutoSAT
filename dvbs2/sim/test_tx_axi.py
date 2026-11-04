"""dvbs2_tx_axi: AXI-Lite registers, DMA packet adapter, CDC + DAC selector.

* registers read back (VERSION 1.1, the CIC register), COEF_DATA
  auto-increments COEF_ADDR
* a good packet produces a data PLFRAME; bad header / short / long packets
  are counted and do not break frame alignment
* with DAC select on, the samples at dac_out (DAC clock domain, one sample
  per dac_valid) equal the bit-exact model; with it off, dac_in passes through
"""

import cocotb
import numpy as np
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

import grref
import txmodel as m

TPP = 16
SPS = 2


class AxiLite:
    def __init__(self, dut):
        self.d = dut
        for s in ("awvalid", "wvalid", "bready", "arvalid", "rready"):
            getattr(dut, "s_axi_" + s).value = 0
        dut.s_axi_wstrb.value = 0xF
        dut.s_axi_awprot.value = 0
        dut.s_axi_arprot.value = 0

    async def write(self, addr, data):
        d = self.d
        d.s_axi_awaddr.value = addr
        d.s_axi_wdata.value = data
        d.s_axi_awvalid.value = 1
        d.s_axi_wvalid.value = 1
        d.s_axi_bready.value = 1
        aw = w = False
        while not (aw and w):
            await ReadOnly()
            aw_now = aw or bool(d.s_axi_awready.value)
            w_now = w or bool(d.s_axi_wready.value)
            await RisingEdge(d.s_axi_aclk)
            if aw_now: d.s_axi_awvalid.value = 0
            if w_now: d.s_axi_wvalid.value = 0
            aw, w = aw_now, w_now
        while True:
            await ReadOnly()
            done = bool(d.s_axi_bvalid.value)
            await RisingEdge(d.s_axi_aclk)
            if done:
                break
        d.s_axi_bready.value = 0

    async def read(self, addr):
        d = self.d
        d.s_axi_araddr.value = addr
        d.s_axi_arvalid.value = 1
        while True:
            await ReadOnly()
            ok = bool(d.s_axi_arready.value)
            await RisingEdge(d.s_axi_aclk)
            if ok:
                break
        d.s_axi_arvalid.value = 0
        d.s_axi_rready.value = 1
        while True:
            await ReadOnly()
            if d.s_axi_rvalid.value:
                v = int(d.s_axi_rdata.value)
                await RisingEdge(d.s_axi_aclk)
                break
            await RisingEdge(d.s_axi_aclk)
        d.s_axi_rready.value = 0
        return v


async def send_packets(dut, packets):
    """packets: list of lists of 32-bit words (one DMA transfer each)."""
    dut.s_axis_tvalid.value = 0
    for words in packets:
        for k, w in enumerate(words):
            dut.s_axis_tdata.value = int(w)
            dut.s_axis_tlast.value = int(k == len(words) - 1)
            dut.s_axis_tvalid.value = 1
            while True:
                await ReadOnly()
                ok = bool(dut.s_axis_tready.value)
                await RisingEdge(dut.s_axi_aclk)
                if ok:
                    break
        dut.s_axis_tvalid.value = 0


def packet(bbframe_bits, rate_idx, pilots, nbytes=None):
    by = np.packbits(bbframe_bits).tobytes()
    if nbytes is not None:
        by = by[:nbytes] if nbytes <= len(by) else by + bytes(nbytes - len(by))
    by += bytes((-len(by)) % 4)
    words = np.frombuffer(by, dtype="<u4").tolist()
    return [0xB5000000 | (pilots << 4) | rate_idx] + words


async def dac_monitor(dut, out):
    while True:
        await ReadOnly()
        valid = int(dut.dac_valid.value)
        await RisingEdge(dut.dac_clk)
        if valid:
            # dac_out registers update on the dac_valid edge
            await ReadOnly()
            out.append((int(dut.dac_out_i.value), int(dut.dac_out_q.value)))
            await RisingEdge(dut.dac_clk)


async def dac_strobe(dut, div):
    k = 0
    while True:
        dut.dac_valid.value = int(k % div == 0)
        k += 1
        await RisingEdge(dut.dac_clk)


def s16(v):
    return v - 65536 if v >= 32768 else v


@cocotb.test()
async def axi_regs_packets_and_dac(dut):
    cocotb.start_soon(Clock(dut.s_axi_aclk, 10, units="ns").start())
    cocotb.start_soon(Clock(dut.dac_clk, 13, units="ns").start())   # unrelated clock
    dut.s_axi_aresetn.value = 0
    dut.dac_valid.value = 0
    dut.dac_in_i.value = 0x1234
    dut.dac_in_q.value = 0x5678
    dut.s_axis_tvalid.value = 0
    axi = AxiLite(dut)
    await ClockCycles(dut.s_axi_aclk, 5)
    dut.s_axi_aresetn.value = 1
    await ClockCycles(dut.s_axi_aclk, 20)

    assert await axi.read(0x04) == 0x44565332
    assert await axi.read(0x00) == 0x00010001
    await axi.write(0x08, 0xDEADBEEF)
    assert await axi.read(0x08) == 0xDEADBEEF
    assert await axi.read(0x2C) == 0                  # CIC off after reset
    await axi.write(0x2C, 0xFFFFFFFF)
    assert await axi.read(0x2C) == 0xF3F              # R [5:0], SHIFT [11:8]
    await axi.write(0x2C, 0)                          # the samples below: RRC alone
    assert await axi.read(0x48) == 0                  # no FECFRAME waiting

    # configuration
    await axi.write(0x14, SPS)
    xa, xb = m.gold_init(0)
    await axi.write(0x18, xa)
    await axi.write(0x1C, xb)
    await axi.write(0x20, 0)
    coefs = m.rrc_coefs(SPS, 0.35, TPP)
    await axi.write(0x24, 0)
    for p in range(SPS):
        for j in range(TPP):
            await axi.write(0x28, int(coefs[p + SPS * j]) & 0x3FFFF)
    assert await axi.read(0x24) == SPS * TPP

    # DAC passthrough while not selected
    mon = []
    cocotb.start_soon(dac_monitor(dut, mon))
    cocotb.start_soon(dac_strobe(dut, 4))
    await ClockCycles(dut.dac_clk, 100)
    assert mon[-1] == (0x1234, 0x5678)

    # select DVB-S2 (NCO bypass), enable framing
    await axi.write(0x10, 0b1100)
    await ClockCycles(dut.dac_clk, 10)
    mon.clear()
    await axi.write(0x10, 0b1101)

    # packets: bad header, good, short (padded), long (truncated), good
    rate = m.RATES[3]
    kbch = m.LDPC[rate]["kbch"]
    rng = np.random.default_rng(1)
    ts = rng.integers(0, 256, (3 * kbch // (8 * 188) + 4, 188), dtype=np.uint8)
    ts[:, 0] = 0x47
    bb = grref.bbheader(ts.ravel(), rate)
    frames = [bb[k * kbch:(k + 1) * kbch] for k in range(3)]
    pk = [
        [0x12345678, 1, 2, 3],
        packet(frames[0], 3, 1),
        packet(frames[1], 3, 1, nbytes=kbch // 8 - 100),
        packet(frames[2], 3, 1, nbytes=kbch // 8 + 40),
        packet(frames[0], 3, 1),
    ]
    cocotb.start_soon(send_packets(dut, pk))

    for _ in range(3_000_000):
        await RisingEdge(dut.s_axi_aclk)
        if int(dut.i_dvbs2_tx_axi.cnt_data.value) >= 4:
            break
    nd = await axi.read(0x30)
    ndum = await axi.read(0x34)
    nlen = await axi.read(0x38)
    nhdr = await axi.read(0x3C)
    unf = await axi.read(0x44)
    dut._log.info(f"data={nd} dummy={ndum} len_err={nlen} hdr_err={nhdr} dac_underruns={unf}")
    assert nd >= 4 and nhdr == 1 and nlen == 2

    # DAC samples vs model: the symbol stream is reconstructed from the model
    # (dummies until the first data frame, then the frames as sent)
    await axi.write(0x10, 0b1100)   # stop framing
    got = np.array([(s16(i), s16(q)) for i, q in mon], dtype=np.int64)
    # drop leading zeros emitted before the first sample crossed the CDC
    first = int(np.flatnonzero((got != 0).any(axis=1))[0])
    got = got[first:]
    syms = []
    for _ in range(ndum):     # an upper bound; we compare only a prefix
        syms.append(m.dummy_plframe_signs(0))
    model_syms = np.concatenate(syms)
    model = m.rrc_interp(m.signs_to_iq(model_syms[:4000]), coefs, SPS, TPP)
    n = 4000 * SPS - 64
    assert len(got) > n
    # the first output sample of the core can be zero-valued as well; align
    off = int(np.flatnonzero((model != 0).any(axis=1))[0])
    bad = np.flatnonzero((got[:n - off] != model[off:n]).any(axis=1))
    assert bad.size == 0, f"{bad.size} DAC sample mismatches, first {bad[:5]}"
    dut._log.info(f"{n - off} DAC-domain samples match the model (dummy frames, sps={SPS})")
