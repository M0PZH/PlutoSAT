"""Minimal AXI-Stream helpers for cocotb (no external cocotb-bus dependency)."""

import random

from cocotb.triggers import ReadOnly, RisingEdge


class Source:
    """Drives s_valid/s_data/... from a list of beats (dicts of signal -> value)."""

    def __init__(self, dut, prefix="s_", idle_prob=0.0, seed=0):
        self.dut, self.p, self.idle, self.rng = dut, prefix, idle_prob, random.Random(seed)
        getattr(dut, prefix + "valid").value = 0

    async def send(self, beats):
        valid = getattr(self.dut, self.p + "valid")
        ready = getattr(self.dut, self.p + "ready")
        for beat in beats:
            while self.idle and self.rng.random() < self.idle:
                valid.value = 0
                await RisingEdge(self.dut.clk)
            for k, v in beat.items():
                getattr(self.dut, self.p + k).value = v
            valid.value = 1
            while True:
                await ReadOnly()
                accepted = bool(ready.value)
                await RisingEdge(self.dut.clk)
                if accepted:
                    break
        valid.value = 0


class Sink:
    """Collects m_* beats; optional random backpressure."""

    def __init__(self, dut, fields, prefix="m_", stall_prob=0.0, seed=1):
        self.dut, self.p, self.fields = dut, prefix, fields
        self.stall, self.rng = stall_prob, random.Random(seed)
        self.beats = []
        getattr(dut, prefix + "ready").value = 0

    async def run(self):
        valid = getattr(self.dut, self.p + "valid")
        ready = getattr(self.dut, self.p + "ready")
        sigs = [getattr(self.dut, self.p + f) for f in self.fields]
        while True:
            r = 0 if (self.stall and self.rng.random() < self.stall) else 1
            ready.value = r
            await ReadOnly()
            if r and valid.value:
                self.beats.append(tuple(int(s.value) for s in sigs))
            await RisingEdge(self.dut.clk)
