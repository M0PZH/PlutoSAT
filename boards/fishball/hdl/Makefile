####################################################################################
## fishball-sdr-7020 HDL project. Builds out of tree against the ADI hdl checkout.
####################################################################################

ifeq ($(ADI_HDL_DIR),)
ADI_HDL_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))../../../hdl)
endif
export ADI_HDL_DIR

PROJECT_NAME := fishball

# use the build host's cores: Vivado threads (max 32) and parallel IP synthesis
ADI_MAX_THREADS ?= 32
ADI_MAX_OOC_JOBS ?= 16
export ADI_MAX_THREADS ADI_MAX_OOC_JOBS

M_DEPS += $(ADI_HDL_DIR)/projects/common/xilinx/adi_fir_filter_constr.xdc
M_DEPS += $(ADI_HDL_DIR)/projects/common/xilinx/adi_fir_filter_bd.tcl
M_DEPS += $(ADI_HDL_DIR)/library/common/ad_iobuf.v
M_DEPS += $(ADI_HDL_DIR)/library/axi_ad9361/axi_ad9361_delay.tcl

LIB_DEPS += axi_ad9361

DVBS2_DIR := $(abspath $(ADI_HDL_DIR)/../dvbs2)
M_DEPS += $(wildcard $(DVBS2_DIR)/rtl/*.sv) $(wildcard $(DVBS2_DIR)/rtl/gen/*)
M_DEPS += $(DVBS2_DIR)/ip/dvbs2_tx_bd.v $(DVBS2_DIR)/ip/dvbs2_rx_bd.v
NBIQ_DIR := $(abspath $(ADI_HDL_DIR)/../nbiq)
M_DEPS += $(wildcard $(NBIQ_DIR)/rtl/*.sv) $(wildcard $(NBIQ_DIR)/rtl/gen/*) $(NBIQ_DIR)/ip/nbiq_bd.v
AWGN_DIR := $(abspath $(ADI_HDL_DIR)/../awgn)
M_DEPS += $(wildcard $(AWGN_DIR)/rtl/*.sv) $(wildcard $(AWGN_DIR)/ip/*.v)
FFT_DIR := $(abspath $(ADI_HDL_DIR)/../fft)
M_DEPS += $(wildcard $(FFT_DIR)/rtl/*.sv) $(wildcard $(FFT_DIR)/rtl/gen/*) $(wildcard $(FFT_DIR)/ip/*)
LIB_DEPS += axi_dmac
LIB_DEPS += util_pack/util_cpack2
LIB_DEPS += util_pack/util_upack2

include $(ADI_HDL_DIR)/projects/scripts/project-xilinx.mk
