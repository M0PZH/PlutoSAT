###############################################################################
## fishball-sdr-7020 block design.
## Derived from the ADI pluto project (Copyright (C) 2014-2025 Analog Devices,
## Inc., ADIBSD). Differences: xc7z020clg400, 32-bit/1 GB DDR3, AD9363 LVDS,
## GEM0 RGMII, SD0, no TDD/phaser glue, no PL FIR decimator/interpolator
## (fixed 12 MS/s; nbiq gives the narrow channels), no IIO streaming (no
## axi_ad9361 ADC/DAC DMAs, no util_cpack2/util_upack2): the AD9363's
## samples go to the PlutoSAT cores only.
##
## PS-PL AXI ports (boards/fishball/README.md, "AXI ports"):
##   M_AXI_GP0  all AXI-Lite registers        M_AXI_GP1  spare
##   S_AXI_HP0  reserved: future IQ transmit DMA (memory -> DAC)
##   S_AXI_HP1  reserved: second DATV receiver's LDPC LLR frame queue
##   S_AXI_HP2  nbiq ring writer (64-bit)
##   S_AXI_HP3  FFT result DMA (64-bit)
##   S_AXI_GP0  DVB-S2 RX DMA (32-bit)       S_AXI_GP1  DVB-S2 TX DMA (32-bit)
##   S_AXI_ACP  spare
## The HP ports stay in their 64-bit mode: a board that keeps its own FSBL
## (fishball Debian) leaves the AFI width registers at the 64-bit default.
###############################################################################

# create board design

# default ports

create_bd_intf_port -mode Master -vlnv xilinx.com:interface:ddrx_rtl:1.0 ddr
create_bd_intf_port -mode Master -vlnv xilinx.com:display_processing_system7:fixedio_rtl:1.0 fixed_io

create_bd_port -dir O spi0_csn_2_o
create_bd_port -dir O spi0_csn_1_o
create_bd_port -dir O spi0_csn_0_o
create_bd_port -dir I spi0_csn_i
create_bd_port -dir I spi0_clk_i
create_bd_port -dir O spi0_clk_o
create_bd_port -dir I spi0_sdo_i
create_bd_port -dir O spi0_sdo_o
create_bd_port -dir I spi0_sdi_i

create_bd_port -dir I -from 17 -to 0 gpio_i
create_bd_port -dir O -from 17 -to 0 gpio_o
create_bd_port -dir O -from 17 -to 0 gpio_t

create_bd_port -dir O spi_csn_o
create_bd_port -dir I spi_csn_i
create_bd_port -dir I spi_clk_i
create_bd_port -dir O spi_clk_o
create_bd_port -dir I spi_sdo_i
create_bd_port -dir O spi_sdo_o
create_bd_port -dir I spi_sdi_i

# instance: sys_ps7

ad_ip_instance processing_system7 sys_ps7

# ps7 settings

ad_ip_parameter sys_ps7 CONFIG.PCW_PRESET_BANK0_VOLTAGE {LVCMOS 3.3V}
ad_ip_parameter sys_ps7 CONFIG.PCW_PRESET_BANK1_VOLTAGE {LVCMOS 1.8V}
ad_ip_parameter sys_ps7 CONFIG.PCW_PACKAGE_NAME clg400
ad_ip_parameter sys_ps7 CONFIG.PCW_ENET0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_ENET0_ENET0_IO {MIO 16 .. 27}
ad_ip_parameter sys_ps7 CONFIG.PCW_ENET0_GRP_MDIO_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_ENET0_GRP_MDIO_IO {MIO 52 .. 53}
ad_ip_parameter sys_ps7 CONFIG.PCW_EN_CLK1_PORT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_EN_RST1_PORT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ 100.0
ad_ip_parameter sys_ps7 CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ 200.0
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_EMIO_GPIO_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_EMIO_GPIO_IO 18
ad_ip_parameter sys_ps7 CONFIG.PCW_SPI1_PERIPHERAL_ENABLE 0
ad_ip_parameter sys_ps7 CONFIG.PCW_I2C0_PERIPHERAL_ENABLE 0
ad_ip_parameter sys_ps7 CONFIG.PCW_UART1_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_UART1_UART1_IO {MIO 8 .. 9}
ad_ip_parameter sys_ps7 CONFIG.PCW_I2C1_PERIPHERAL_ENABLE 0
ad_ip_parameter sys_ps7 CONFIG.PCW_QSPI_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_QSPI_GRP_SINGLE_SS_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_SD0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_SD0_SD0_IO {MIO 40 .. 45}
ad_ip_parameter sys_ps7 CONFIG.PCW_SDIO_PERIPHERAL_FREQMHZ 50
ad_ip_parameter sys_ps7 CONFIG.PCW_SPI0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_SPI0_SPI0_IO EMIO
ad_ip_parameter sys_ps7 CONFIG.PCW_TTC0_PERIPHERAL_ENABLE 0
ad_ip_parameter sys_ps7 CONFIG.PCW_USE_FABRIC_INTERRUPT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_USB0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_MIO_GPIO_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_MIO_GPIO_IO MIO
ad_ip_parameter sys_ps7 CONFIG.PCW_USB0_RESET_IO {MIO 46}
ad_ip_parameter sys_ps7 CONFIG.PCW_USB0_RESET_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_IRQ_F2P_INTR 1
ad_ip_parameter sys_ps7 CONFIG.PCW_IRQ_F2P_MODE REVERSE
ad_ip_parameter sys_ps7 CONFIG.PCW_MIO_0_PULLUP {enabled}
ad_ip_parameter sys_ps7 CONFIG.PCW_MIO_9_PULLUP {enabled}
ad_ip_parameter sys_ps7 CONFIG.PCW_MIO_10_PULLUP {enabled}
ad_ip_parameter sys_ps7 CONFIG.PCW_MIO_11_PULLUP {enabled}
ad_ip_parameter sys_ps7 CONFIG.PCW_MIO_48_PULLUP {enabled}
ad_ip_parameter sys_ps7 CONFIG.PCW_MIO_49_PULLUP {disabled}
ad_ip_parameter sys_ps7 CONFIG.PCW_MIO_53_PULLUP {enabled}

# DDR 2x MT41K256M16 RE-125 (32 bit, 1 GB)

ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_PARTNO {MT41K256M16 RE-125}
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_BUS_WIDTH {32 Bit}
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_USE_INTERNAL_VREF 0
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_TRAIN_WRITE_LEVEL 1
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_TRAIN_READ_GATE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_TRAIN_DATA_EYE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_0 0.048
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_1 0.050
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY0 0.241
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY1 0.240

ad_ip_instance ilconcat sys_concat_intc
ad_ip_parameter sys_concat_intc CONFIG.NUM_PORTS 16

ad_ip_instance proc_sys_reset sys_rstgen
ad_ip_parameter sys_rstgen CONFIG.C_EXT_RST_WIDTH 1

# system reset/clock definitions

# add external spi

ad_ip_instance axi_quad_spi axi_spi
ad_ip_parameter axi_spi CONFIG.C_USE_STARTUP 0
ad_ip_parameter axi_spi CONFIG.C_NUM_SS_BITS 1
ad_ip_parameter axi_spi CONFIG.C_SCK_RATIO 8

ad_connect  sys_cpu_clk sys_ps7/FCLK_CLK0
ad_connect  sys_200m_clk sys_ps7/FCLK_CLK1
ad_connect  sys_cpu_reset sys_rstgen/peripheral_reset
ad_connect  sys_cpu_resetn sys_rstgen/peripheral_aresetn
ad_connect  sys_cpu_clk sys_rstgen/slowest_sync_clk
ad_connect  sys_rstgen/ext_reset_in sys_ps7/FCLK_RESET0_N

# interface connections

ad_connect  ddr sys_ps7/DDR
ad_connect  gpio_i sys_ps7/GPIO_I
ad_connect  gpio_o sys_ps7/GPIO_O
ad_connect  gpio_t sys_ps7/GPIO_T
ad_connect  fixed_io sys_ps7/FIXED_IO

# ps7 spi connections

ad_connect  spi0_csn_2_o sys_ps7/SPI0_SS2_O
ad_connect  spi0_csn_1_o sys_ps7/SPI0_SS1_O
ad_connect  spi0_csn_0_o sys_ps7/SPI0_SS_O
ad_connect  spi0_csn_i sys_ps7/SPI0_SS_I
ad_connect  spi0_clk_i sys_ps7/SPI0_SCLK_I
ad_connect  spi0_clk_o sys_ps7/SPI0_SCLK_O
ad_connect  spi0_sdo_i sys_ps7/SPI0_MOSI_I
ad_connect  spi0_sdo_o sys_ps7/SPI0_MOSI_O
ad_connect  spi0_sdi_i sys_ps7/SPI0_MISO_I

# axi spi connections

ad_connect  sys_cpu_clk  axi_spi/ext_spi_clk
ad_connect  spi_csn_i  axi_spi/ss_i
ad_connect  spi_csn_o  axi_spi/ss_o
ad_connect  spi_clk_i  axi_spi/sck_i
ad_connect  spi_clk_o  axi_spi/sck_o
ad_connect  spi_sdo_i  axi_spi/io0_i
ad_connect  spi_sdo_o  axi_spi/io0_o
ad_connect  spi_sdi_i  axi_spi/io1_i

# interrupts

ad_connect  sys_concat_intc/dout sys_ps7/IRQ_F2P
ad_connect  sys_concat_intc/In15 GND
ad_connect  sys_concat_intc/In14 GND
ad_connect  sys_concat_intc/In13 GND
ad_connect  sys_concat_intc/In12 GND
ad_connect  sys_concat_intc/In11 GND
ad_connect  sys_concat_intc/In10 GND
ad_connect  sys_concat_intc/In9 GND
ad_connect  sys_concat_intc/In8 GND
ad_connect  sys_concat_intc/In7 GND
ad_connect  sys_concat_intc/In6 GND
ad_connect  sys_concat_intc/In5 GND
ad_connect  sys_concat_intc/In4 GND
ad_connect  sys_concat_intc/In3 GND
ad_connect  sys_concat_intc/In2 GND
ad_connect  sys_concat_intc/In1 GND
ad_connect  sys_concat_intc/In0 GND

# iic

create_bd_intf_port -mode Master -vlnv xilinx.com:interface:iic_rtl:1.0 iic_main

ad_ip_instance axi_iic axi_iic_main

ad_connect  iic_main axi_iic_main/iic
ad_cpu_interconnect 0x41600000 axi_iic_main
ad_cpu_interrupt ps-15 mb-15 axi_iic_main/iic2intc_irpt

# ad9361

create_bd_port -dir I rx_clk_in_p
create_bd_port -dir I rx_clk_in_n
create_bd_port -dir I rx_frame_in_p
create_bd_port -dir I rx_frame_in_n
create_bd_port -dir I -from 5 -to 0 rx_data_in_p
create_bd_port -dir I -from 5 -to 0 rx_data_in_n

create_bd_port -dir O tx_clk_out_p
create_bd_port -dir O tx_clk_out_n
create_bd_port -dir O tx_frame_out_p
create_bd_port -dir O tx_frame_out_n
create_bd_port -dir O -from 5 -to 0 tx_data_out_p
create_bd_port -dir O -from 5 -to 0 tx_data_out_n

create_bd_port -dir O enable
create_bd_port -dir O txnrx
create_bd_port -dir I up_enable
create_bd_port -dir I up_txnrx

# ad9361 core(s)

ad_ip_instance axi_ad9361 axi_ad9361
ad_ip_parameter axi_ad9361 CONFIG.ID 0
ad_ip_parameter axi_ad9361 CONFIG.CMOS_OR_LVDS_N 0
ad_ip_parameter axi_ad9361 CONFIG.MODE_1R1T 0
ad_ip_parameter axi_ad9361 CONFIG.ADC_INIT_DELAY 30
# room for the PlutoSAT cores: no DDS test tones (dvbs2_tx's carrier-only
# mode gives a test tone; DMA transmit is unaffected) and no TDD (FDD only)
ad_ip_parameter axi_ad9361 CONFIG.DAC_DDS_DISABLE 1
ad_ip_parameter axi_ad9361 CONFIG.TDD_DISABLE 1

# no PL FIR interpolator/decimator (the Pluto's x8 filters for IIO rates
# below 2.08 MS/s): the AD9363 runs at a fixed 12 MS/s, nbiq provides the
# narrow channels; frees 44 DSPs and ~1.2k LUTs.
# No IIO streaming either (axi_ad9361_adc_dma + util_cpack2,
# axi_ad9361_dac_dma + util_upack2 are gone): the ad9361-phy, cf-ad9361-lpc
# and cf-ad9361-dds-core-lpc drivers probe without DMAs (no "dmas" in the
# device tree), so LO, gains, rates and the core's IQ/DC correction stay.

# Self-test noise source (awgn/): between the AD9363 receive data and all of
# its users (dvbs2_rx, nbiq, fft). Adds programmable
# AWGN to rx0; delays all four lanes by 6 ADC clocks.

set awgn_dir [file normalize [file join $ad_hdl_dir .. awgn]]
add_files -norecurse [concat \
  [glob [file join $awgn_dir rtl *.sv]] \
  [list [file join $awgn_dir ip rx_awgn_bd.v]]]
update_compile_order -fileset sources_1
create_bd_cell -type module -reference rx_awgn_bd rx_awgn

# connections

ad_connect  rx_clk_in_p axi_ad9361/rx_clk_in_p
ad_connect  rx_clk_in_n axi_ad9361/rx_clk_in_n
ad_connect  rx_frame_in_p axi_ad9361/rx_frame_in_p
ad_connect  rx_frame_in_n axi_ad9361/rx_frame_in_n
ad_connect  rx_data_in_p axi_ad9361/rx_data_in_p
ad_connect  rx_data_in_n axi_ad9361/rx_data_in_n
ad_connect  tx_clk_out_p axi_ad9361/tx_clk_out_p
ad_connect  tx_clk_out_n axi_ad9361/tx_clk_out_n
ad_connect  tx_frame_out_p axi_ad9361/tx_frame_out_p
ad_connect  tx_frame_out_n axi_ad9361/tx_frame_out_n
ad_connect  tx_data_out_p axi_ad9361/tx_data_out_p
ad_connect  tx_data_out_n axi_ad9361/tx_data_out_n
ad_connect  enable axi_ad9361/enable
ad_connect  txnrx axi_ad9361/txnrx
ad_connect  up_enable axi_ad9361/up_enable
ad_connect  up_txnrx axi_ad9361/up_txnrx

ad_connect  axi_ad9361/tdd_sync GND
ad_connect  sys_200m_clk axi_ad9361/delay_clk
ad_connect  axi_ad9361/l_clk axi_ad9361/clk

ad_connect axi_ad9361/l_clk rx_awgn/adc_clk
foreach l {i0 q0 i1 q1} {
  ad_connect axi_ad9361/adc_valid_$l rx_awgn/adc_valid_$l
  ad_connect axi_ad9361/adc_data_$l  rx_awgn/adc_data_$l
}

# no sample FIFOs behind the core: never an ADC overflow / DAC underflow;
# DAC channel 1 (TX2) gets zeros
ad_connect axi_ad9361/adc_dovf GND
ad_connect axi_ad9361/dac_dunf GND
ad_connect axi_ad9361/dac_data_i1 GND
ad_connect axi_ad9361/dac_data_q1 GND

# interconnects

ad_cpu_interconnect 0x79020000 axi_ad9361
ad_cpu_interconnect 0x7C430000 axi_spi

# interrupts (ps-13 and ps-12 were the IIO DMAs')

ad_cpu_interrupt ps-11 mb-11 axi_spi/ip2intc_irpt

# DVB-S2 transmitter (dvbs2/), fed by its own DMA (S_AXI_GP1), selectable
# onto DAC channel 0 by CTRL bit 2 ("DAC select": dvbs2_tx or dac_in_i/q).
#
# Future IQ transmit (memory -> DAC, up to 12 MS/s, narrow- or wideband):
# its DMA goes on S_AXI_HP0 (64-bit, reserved), its sample stream into
# dvbs2_tx/dac_in_i/q (dac_clk domain, one sample per dac_valid) instead
# of the zeros below, or into a separate mux after dvbs2_tx
# (dac_out_i/q -> axi_ad9361/dac_data_i0/q0).

set dvbs2_dir [file normalize [file join $ad_hdl_dir .. dvbs2]]
add_files -norecurse [concat \
  [glob [file join $dvbs2_dir rtl gen *.sv]] \
  [glob [file join $dvbs2_dir rtl *.sv]] \
  [list [file join $dvbs2_dir rtl gen ldpc_rom.mem] \
        [file join $dvbs2_dir rtl gen ldpc_dec_rom.mem] \
        [file join $dvbs2_dir rtl gen nco_lut.mem] \
        [file join $dvbs2_dir ip dvbs2_tx_bd.v] \
        [file join $dvbs2_dir ip dvbs2_rx_bd.v]]]
update_compile_order -fileset sources_1

create_bd_cell -type module -reference dvbs2_tx_bd dvbs2_tx

ad_connect axi_ad9361/l_clk          dvbs2_tx/dac_clk
ad_connect axi_ad9361/dac_valid_i0   dvbs2_tx/dac_valid
ad_connect GND                       dvbs2_tx/dac_in_i
ad_connect GND                       dvbs2_tx/dac_in_q
ad_connect dvbs2_tx/dac_out_i        axi_ad9361/dac_data_i0
ad_connect dvbs2_tx/dac_out_q        axi_ad9361/dac_data_q0

ad_ip_instance axi_dmac axi_dvbs2_dma
ad_ip_parameter axi_dvbs2_dma CONFIG.DMA_TYPE_SRC 0
ad_ip_parameter axi_dvbs2_dma CONFIG.DMA_TYPE_DEST 1
ad_ip_parameter axi_dvbs2_dma CONFIG.CYCLIC 0
ad_ip_parameter axi_dvbs2_dma CONFIG.SYNC_TRANSFER_START 0
ad_ip_parameter axi_dvbs2_dma CONFIG.AXI_SLICE_SRC 0
ad_ip_parameter axi_dvbs2_dma CONFIG.AXI_SLICE_DEST 0
ad_ip_parameter axi_dvbs2_dma CONFIG.DMA_2D_TRANSFER 0
ad_ip_parameter axi_dvbs2_dma CONFIG.DMA_DATA_WIDTH_SRC 32
ad_ip_parameter axi_dvbs2_dma CONFIG.DMA_DATA_WIDTH_DEST 32

ad_connect sys_cpu_clk axi_dvbs2_dma/m_axis_aclk
ad_connect axi_dvbs2_dma/m_axis dvbs2_tx/s_axis

ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_GP1 {1}
ad_connect sys_cpu_clk sys_ps7/S_AXI_GP1_ACLK
ad_connect axi_dvbs2_dma/m_src_axi sys_ps7/S_AXI_GP1
create_bd_addr_seg -range 0x40000000 -offset 0x00000000 \
                    [get_bd_addr_spaces axi_dvbs2_dma/m_src_axi] \
                    [get_bd_addr_segs sys_ps7/S_AXI_GP1/GP1_DDR_LOWOCM] \
                    SEG_sys_ps7_GP1_DDR_LOWOCM
ad_connect sys_cpu_clk axi_dvbs2_dma/m_src_axi_aclk
ad_connect sys_cpu_resetn axi_dvbs2_dma/m_src_axi_aresetn

ad_cpu_interconnect 0x7C480000 axi_dvbs2_dma
ad_cpu_interconnect 0x7C500000 dvbs2_tx
ad_cpu_interrupt ps-10 mb-10 axi_dvbs2_dma/irq

# DVB-S2 receiver (dvbs2/): AD9361 channel 0 ADC samples -> DDC -> demodulator
# -> LDPC -> one DMA packet per BBFRAME (axi_dvbs2_rx_dma, stream -> memory,
# S_AXI_GP0)

create_bd_cell -type module -reference dvbs2_rx_bd dvbs2_rx

ad_connect axi_ad9361/l_clk          dvbs2_rx/adc_clk
ad_connect rx_awgn/out_valid_i0     dvbs2_rx/adc_valid
ad_connect rx_awgn/out_data_i0      dvbs2_rx/adc_data_i
ad_connect rx_awgn/out_data_q0      dvbs2_rx/adc_data_q

ad_ip_instance axi_dmac axi_dvbs2_rx_dma
ad_ip_parameter axi_dvbs2_rx_dma CONFIG.DMA_TYPE_SRC 1
ad_ip_parameter axi_dvbs2_rx_dma CONFIG.DMA_TYPE_DEST 0
ad_ip_parameter axi_dvbs2_rx_dma CONFIG.CYCLIC 0
ad_ip_parameter axi_dvbs2_rx_dma CONFIG.SYNC_TRANSFER_START 0
ad_ip_parameter axi_dvbs2_rx_dma CONFIG.AXI_SLICE_SRC 0
ad_ip_parameter axi_dvbs2_rx_dma CONFIG.AXI_SLICE_DEST 0
ad_ip_parameter axi_dvbs2_rx_dma CONFIG.DMA_2D_TRANSFER 0
ad_ip_parameter axi_dvbs2_rx_dma CONFIG.DMA_DATA_WIDTH_SRC 64
ad_ip_parameter axi_dvbs2_rx_dma CONFIG.DMA_DATA_WIDTH_DEST 32

ad_connect sys_cpu_clk axi_dvbs2_rx_dma/s_axis_aclk
ad_connect dvbs2_rx/m_axis axi_dvbs2_rx_dma/s_axis

ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_GP0 {1}
ad_connect sys_cpu_clk sys_ps7/S_AXI_GP0_ACLK
ad_connect axi_dvbs2_rx_dma/m_dest_axi sys_ps7/S_AXI_GP0
create_bd_addr_seg -range 0x40000000 -offset 0x00000000 \
                    [get_bd_addr_spaces axi_dvbs2_rx_dma/m_dest_axi] \
                    [get_bd_addr_segs sys_ps7/S_AXI_GP0/GP0_DDR_LOWOCM] \
                    SEG_sys_ps7_GP0_DDR_LOWOCM
ad_connect sys_cpu_clk axi_dvbs2_rx_dma/m_dest_axi_aclk
ad_connect sys_cpu_resetn axi_dvbs2_rx_dma/m_dest_axi_aresetn

ad_cpu_interconnect 0x7C490000 axi_dvbs2_rx_dma
ad_cpu_interconnect 0x7C510000 dvbs2_rx
ad_cpu_interrupt ps-9 mb-9 axi_dvbs2_rx_dma/irq

# Narrowband IQ channels (nbiq/): 5 x (NCO, CIC, FIR) from either receive
# channel (4 user channels + 1 overview) -> one DDR ring per channel, written
# through S_AXI_HP2 (64-bit). tap_* (the FIR outputs) feed the spectrum FFT.

set nbiq_dir [file normalize [file join $ad_hdl_dir .. nbiq]]
add_files -norecurse [concat \
  [glob [file join $nbiq_dir rtl *.sv]] \
  [list [file join $nbiq_dir rtl gen nbiq_lut.mem] \
        [file join $nbiq_dir ip nbiq_bd.v]]]
update_compile_order -fileset sources_1

create_bd_cell -type module -reference nbiq_bd nbiq

ad_connect axi_ad9361/l_clk          nbiq/adc_clk
ad_connect rx_awgn/out_valid_i0     nbiq/adc_valid
ad_connect rx_awgn/out_data_i0      nbiq/adc_data_i0
ad_connect rx_awgn/out_data_q0      nbiq/adc_data_q0
ad_connect rx_awgn/out_data_i1      nbiq/adc_data_i1
ad_connect rx_awgn/out_data_q1      nbiq/adc_data_q1

ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_HP2 {1}
ad_ip_parameter sys_ps7 CONFIG.PCW_S_AXI_HP2_DATA_WIDTH {64}
ad_connect sys_cpu_clk sys_ps7/S_AXI_HP2_ACLK
ad_connect nbiq/m_axi sys_ps7/S_AXI_HP2
create_bd_addr_seg -range 0x40000000 -offset 0x00000000 \
                    [get_bd_addr_spaces nbiq/m_axi] \
                    [get_bd_addr_segs sys_ps7/S_AXI_HP2/HP2_DDR_LOWOCM] \
                    SEG_sys_ps7_HP2_DDR_LOWOCM

ad_cpu_interconnect 0x7C520000 nbiq
ad_cpu_interrupt ps-8 mb-8 nbiq/irq

# Spectrum analyser (fft/): averaged 512- to 4096-bin spectra of rx0, rx1 or an
# nbiq channel (the tap), read by the ARM over AXI-Lite (UIO "fft") or
# written to memory by axi_fft_dma through S_AXI_HP3

set fft_dir [file normalize [file join $ad_hdl_dir .. fft]]
source [file join $fft_dir ip fft_xfft.tcl]
fft_create_xfft
add_files -norecurse [list \
  [file join $fft_dir rtl fft_win.sv] [file join $fft_dir rtl fft_acc.sv] \
  [file join $fft_dir rtl fft_axi.sv] [file join $fft_dir rtl gen fft_win.mem] \
  [file join $fft_dir ip fft_bd.v]]
update_compile_order -fileset sources_1
create_bd_cell -type module -reference fft_bd fft

ad_connect axi_ad9361/l_clk       fft/adc_clk
ad_connect rx_awgn/out_valid_i0   fft/adc_valid
ad_connect rx_awgn/out_data_i0    fft/adc_data_i0
ad_connect rx_awgn/out_data_q0    fft/adc_data_q0
ad_connect rx_awgn/out_data_i1    fft/adc_data_i1
ad_connect rx_awgn/out_data_q1    fft/adc_data_q1
ad_connect nbiq/tap_valid         fft/tap_valid
ad_connect nbiq/tap_ch            fft/tap_ch
ad_connect nbiq/tap_i             fft/tap_i
ad_connect nbiq/tap_q             fft/tap_q

ad_cpu_interconnect 0x7C540000 fft
ad_cpu_interrupt ps-7 mb-7 fft/irq

# FFT result DMA (stream -> memory, S_AXI_HP3, 64-bit): one packet per
# result from fft/m_axis (CTRL.STREAM; s_axi_aclk = sys_cpu_clk domain), see
# fft/README.md "Result stream". Registers 0x7C4A0000, irq ps-13; buffer:
# fft-buffer in the device tree (1 MB @ 0x3EA00000), map1/map2 of the fft
# UIO node.

ad_ip_instance axi_dmac axi_fft_dma
ad_ip_parameter axi_fft_dma CONFIG.DMA_TYPE_SRC 1
ad_ip_parameter axi_fft_dma CONFIG.DMA_TYPE_DEST 0
ad_ip_parameter axi_fft_dma CONFIG.CYCLIC 0
ad_ip_parameter axi_fft_dma CONFIG.SYNC_TRANSFER_START 0
ad_ip_parameter axi_fft_dma CONFIG.AXI_SLICE_SRC 0
ad_ip_parameter axi_fft_dma CONFIG.AXI_SLICE_DEST 0
ad_ip_parameter axi_fft_dma CONFIG.DMA_2D_TRANSFER 0
ad_ip_parameter axi_fft_dma CONFIG.DMA_DATA_WIDTH_SRC 64
ad_ip_parameter axi_fft_dma CONFIG.DMA_DATA_WIDTH_DEST 64

ad_connect sys_cpu_clk axi_fft_dma/s_axis_aclk
ad_connect fft/m_axis axi_fft_dma/s_axis

ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_HP3 {1}
ad_ip_parameter sys_ps7 CONFIG.PCW_S_AXI_HP3_DATA_WIDTH {64}
ad_connect sys_cpu_clk sys_ps7/S_AXI_HP3_ACLK
ad_connect axi_fft_dma/m_dest_axi sys_ps7/S_AXI_HP3
create_bd_addr_seg -range 0x40000000 -offset 0x00000000 \
                    [get_bd_addr_spaces axi_fft_dma/m_dest_axi] \
                    [get_bd_addr_segs sys_ps7/S_AXI_HP3/HP3_DDR_LOWOCM] \
                    SEG_sys_ps7_HP3_DDR_LOWOCM
ad_connect sys_cpu_clk axi_fft_dma/m_dest_axi_aclk
ad_connect sys_cpu_resetn axi_fft_dma/m_dest_axi_aresetn

ad_cpu_interconnect 0x7C4A0000 axi_fft_dma
ad_cpu_interrupt ps-13 mb-13 axi_fft_dma/irq

ad_cpu_interconnect 0x7C530000 rx_awgn
