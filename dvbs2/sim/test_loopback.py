"""Closed loop: RTL TX samples -> gr-dvbs2rx (independent receiver) -> TS.

The RTL transmits QPSK 1/2 PLFRAMEs (pilots on). The samples of the data
frames are cut at PLFRAME boundaries, played in a seamless loop into
`dvbs2-rx`, and every TS packet the receiver outputs must equal the input
packet with the same sequence number; all packets carried by the frames
must be seen. (gr-dvbs2rx needs more frames to lock than the RTL sim can
produce in reasonable time, hence the loop.)
"""

import os
import signal
import subprocess
import time

import cocotb
import numpy as np
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

import grref
import txmodel as m
from axis import Sink, Source

TPP = 16
RATE_IDX, PILOTS, SPS, ROLLOFF = 3, 1, 2, 0.35
NFRAMES = 8
OUT = os.path.join(os.path.dirname(__file__), "sim_build", "loopback")


@cocotb.test()
async def loopback_gr_dvbs2rx(dut):
    os.makedirs(OUT, exist_ok=True)
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.enable.value = 0
    dut.sps.value = SPS
    xa, xb = m.gold_init(0)
    dut.gold_xa.value, dut.gold_xb.value = xa, xb
    dut.nco_finc.value = 0
    dut.nco_bypass.value = 1
    dut.carrier_only.value = 0
    dut.coef_we.value = 0
    await ClockCycles(dut.clk, 4)
    dut.rst.value = 0
    coefs = m.rrc_coefs(SPS, ROLLOFF, TPP)
    for p in range(SPS):
        for j in range(TPP):
            dut.coef_addr.value = p * TPP + j
            dut.coef_data.value = int(coefs[p + SPS * j]) & 0x3FFFF
            dut.coef_we.value = 1
            await RisingEdge(dut.clk)
    dut.coef_we.value = 0

    # TS packets -> BBFRAMEs with gr-dtv's BB header block
    rate = m.RATES[RATE_IDX]
    kbch = m.LDPC[rate]["kbch"]
    rng = np.random.default_rng(7)
    npk = (NFRAMES + 2) * kbch // (8 * 188) + 4
    ts = rng.integers(0, 256, (npk, 188), dtype=np.uint8)
    ts[:, 0] = 0x47
    for k in range(npk):  # sequence number in the payload, to locate packets
        ts[k, 4:8] = np.frombuffer(np.uint32(k).tobytes(), dtype=np.uint8)
    bb = grref.bbheader(ts.ravel(), rate, ROLLOFF)
    nfr = min(NFRAMES, len(bb) // kbch)
    beats = []
    for f in range(nfr):
        by = np.packbits(bb[f * kbch:(f + 1) * kbch])
        beats += [{"data": int(b), "user": (PILOTS << 4) | RATE_IDX, "last": int(k == len(by) - 1)}
                  for k, b in enumerate(by)]

    snk = Sink(dut, ["i", "q"])
    cocotb.start_soon(snk.run())
    dut.enable.value = 1
    cocotb.start_soon(Source(dut).send(beats))

    target = None
    lead_dummies = None
    for _ in range(50_000_000):
        await RisingEdge(dut.clk)
        nd = int(dut.cnt_data_frames.value)
        n = nd + int(dut.cnt_dummy_frames.value)
        if lead_dummies is None and nd == 1:
            lead_dummies = n - 1
        if target is None and nd == nfr:
            target = n + 1
        if target is not None and n >= target:
            break
    await ClockCycles(dut.clk, 5000)   # drain the shaping pipeline
    dut.enable.value = 0

    iq = np.array(snk.beats, dtype=np.int64)
    iq = np.where(iq >= 32768, iq - 65536, iq).astype(np.float32) / 32768.0
    samples = (iq[:, 0] + 1j * iq[:, 1]).astype(np.complex64)
    # whole data PLFRAMEs only: dummies are 3330 symbols, data frames 33282
    a = lead_dummies * 3330 * SPS
    b = a + nfr * 33282 * SPS
    assert len(samples) >= b
    iq_file = os.path.join(OUT, "tx_rtl.fc32")
    ts_file = os.path.join(OUT, "rx.ts")
    samples[a:b].tofile(iq_file)
    dut._log.info(f"{lead_dummies} leading dummies, {nfr} data frames -> {iq_file}")

    cmd = ["dvbs2-rx", "--source", "file", "--in-file", iq_file, "--in-repeat",
           "--in-iq-format", "fc32", "--sink", "file", "--out-file", ts_file,
           "--sps", str(SPS), "--sym-rate", "1000000", "--frame-size", "normal",
           "--modcod", "qpsk1/2", "--pilots", "on", "--rolloff", str(ROLLOFF),
           "--log-all", "--log-period", "5"]
    dut._log.info(" ".join(cmd))
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(30)
    proc.send_signal(signal.SIGTERM)
    log = proc.communicate(timeout=60)[0]
    stats = [ln for ln in log.splitlines() if '"lock"' in ln]
    dut._log.info(stats[-1] if stats else log[-1500:])

    rx = np.fromfile(ts_file, dtype=np.uint8)
    assert len(rx) % 188 == 0 and len(rx) > 0, f"receiver produced {len(rx)} bytes"
    rx = rx.reshape(-1, 188)
    seq = rx[:, 4:8].copy().view(np.uint32).ravel()
    for k, s in enumerate(seq):
        assert s < npk and np.array_equal(rx[k], ts[s]), f"rx packet {k} (seq {s}) differs"
    carried = nfr * (kbch - 80) // (188 * 8)
    distinct = len(set(seq.tolist()))
    dut._log.info(f"gr-dvbs2rx output {len(rx)} TS packets, all identical to the input; "
                  f"{distinct} distinct of {carried} carried by the frames")
    assert distinct >= carried - 1
