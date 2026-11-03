# cocotb + Verilator testbenches for the DVB-S2 TX RTL.
#   make TOPLEVEL=dvbs2_fec MODULE=test_fec
#   make all-tests          (runs every testbench)
# Use dvbs2/.venv (cocotb, gnuradio via system site-packages).

DVBS2   := $(abspath ..)
VENV    ?= $(DVBS2)/.venv
export PATH := $(VENV)/bin:$(PATH)
export PYTHONPATH := $(DVBS2)/model:$(DVBS2)/sim:$(PYTHONPATH)

SIM ?= verilator
TOPLEVEL_LANG = verilog
TOPLEVEL ?= dvbs2_fec
MODULE ?= test_fec

GEN := $(DVBS2)/rtl/gen
VERILOG_SOURCES = $(wildcard $(GEN)/*.sv) $(wildcard $(DVBS2)/rtl/*.sv) $(DVBS2)/ip/dvbs2_tx_bd.v

EXTRA_ARGS += -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC --x-initial unique
EXTRA_ARGS += -GLDPC_ROM_FILE='"$(GEN)/ldpc_rom.mem"'
ifneq ($(filter $(TOPLEVEL),dvbs2_tx dvbs2_tx_bd),)
EXTRA_ARGS += -GNCO_LUT_FILE='"$(GEN)/nco_lut.mem"'
endif
SIM_BUILD = sim_build/$(TOPLEVEL)

include $(shell $(VENV)/bin/cocotb-config --makefiles)/Makefile.sim

.PHONY: all-tests
all-tests:
	$(MAKE) TOPLEVEL=dvbs2_fec MODULE=test_fec
	$(MAKE) TOPLEVEL=dvbs2_tx MODULE=test_tx
	$(MAKE) TOPLEVEL=dvbs2_tx_bd MODULE=test_tx_axi
	$(MAKE) TOPLEVEL=dvbs2_tx MODULE=test_loopback
