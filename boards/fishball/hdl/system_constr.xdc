###############################################################################
## fishball-sdr-7020 (xc7z020clg400-2 + AD9363, LVDS)
## Pin-out taken from the Fish-Wan plutosdr-fw-7020-SDR board files.
###############################################################################

# ad9361 data interface (LVDS, SWAP == 0x1)

set_property  -dict {PACKAGE_PIN  U18  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_clk_in_p]
set_property  -dict {PACKAGE_PIN  U19  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_clk_in_n]
set_property  -dict {PACKAGE_PIN  Y16  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_frame_in_p]
set_property  -dict {PACKAGE_PIN  Y17  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_frame_in_n]
set_property  -dict {PACKAGE_PIN  Y18  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_p[0]]
set_property  -dict {PACKAGE_PIN  Y19  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_n[0]]
set_property  -dict {PACKAGE_PIN  T16  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_p[1]]
set_property  -dict {PACKAGE_PIN  U17  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_n[1]]
set_property  -dict {PACKAGE_PIN  V20  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_p[2]]
set_property  -dict {PACKAGE_PIN  W20  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_n[2]]
set_property  -dict {PACKAGE_PIN  T17  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_p[3]]
set_property  -dict {PACKAGE_PIN  R18  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_n[3]]
set_property  -dict {PACKAGE_PIN  T20  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_p[4]]
set_property  -dict {PACKAGE_PIN  U20  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_n[4]]
set_property  -dict {PACKAGE_PIN  W18  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_p[5]]
set_property  -dict {PACKAGE_PIN  W19  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_n[5]]

set_property  -dict {PACKAGE_PIN  U14  IOSTANDARD LVDS_25} [get_ports tx_clk_out_p]
set_property  -dict {PACKAGE_PIN  U15  IOSTANDARD LVDS_25} [get_ports tx_clk_out_n]
set_property  -dict {PACKAGE_PIN  V16  IOSTANDARD LVDS_25} [get_ports tx_frame_out_p]
set_property  -dict {PACKAGE_PIN  W16  IOSTANDARD LVDS_25} [get_ports tx_frame_out_n]
set_property  -dict {PACKAGE_PIN  V15  IOSTANDARD LVDS_25} [get_ports tx_data_out_p[0]]
set_property  -dict {PACKAGE_PIN  W15  IOSTANDARD LVDS_25} [get_ports tx_data_out_n[0]]
set_property  -dict {PACKAGE_PIN  V12  IOSTANDARD LVDS_25} [get_ports tx_data_out_p[1]]
set_property  -dict {PACKAGE_PIN  W13  IOSTANDARD LVDS_25} [get_ports tx_data_out_n[1]]
set_property  -dict {PACKAGE_PIN  W14  IOSTANDARD LVDS_25} [get_ports tx_data_out_p[2]]
set_property  -dict {PACKAGE_PIN  Y14  IOSTANDARD LVDS_25} [get_ports tx_data_out_n[2]]
set_property  -dict {PACKAGE_PIN  T12  IOSTANDARD LVDS_25} [get_ports tx_data_out_p[3]]
set_property  -dict {PACKAGE_PIN  U12  IOSTANDARD LVDS_25} [get_ports tx_data_out_n[3]]
set_property  -dict {PACKAGE_PIN  T11  IOSTANDARD LVDS_25} [get_ports tx_data_out_p[4]]
set_property  -dict {PACKAGE_PIN  T10  IOSTANDARD LVDS_25} [get_ports tx_data_out_n[4]]
set_property  -dict {PACKAGE_PIN  U13  IOSTANDARD LVDS_25} [get_ports tx_data_out_p[5]]
set_property  -dict {PACKAGE_PIN  V13  IOSTANDARD LVDS_25} [get_ports tx_data_out_n[5]]

# ad9361 control / status

set_property  -dict {PACKAGE_PIN  L20  IOSTANDARD LVCMOS25} [get_ports gpio_status[0]]
set_property  -dict {PACKAGE_PIN  L19  IOSTANDARD LVCMOS25} [get_ports gpio_status[1]]
set_property  -dict {PACKAGE_PIN  K19  IOSTANDARD LVCMOS25} [get_ports gpio_status[2]]
set_property  -dict {PACKAGE_PIN  T14  IOSTANDARD LVCMOS25} [get_ports gpio_status[3]]
set_property  -dict {PACKAGE_PIN  P15  IOSTANDARD LVCMOS25} [get_ports gpio_status[4]]
set_property  -dict {PACKAGE_PIN  M20  IOSTANDARD LVCMOS25} [get_ports gpio_status[5]]
set_property  -dict {PACKAGE_PIN  M19  IOSTANDARD LVCMOS25} [get_ports gpio_status[6]]
set_property  -dict {PACKAGE_PIN  N20  IOSTANDARD LVCMOS25} [get_ports gpio_status[7]]

set_property  -dict {PACKAGE_PIN  J19  IOSTANDARD LVCMOS25} [get_ports gpio_ctl[0]]
set_property  -dict {PACKAGE_PIN  K14  IOSTANDARD LVCMOS25} [get_ports gpio_ctl[1]]
set_property  -dict {PACKAGE_PIN  L17  IOSTANDARD LVCMOS25} [get_ports gpio_ctl[2]]
set_property  -dict {PACKAGE_PIN  J20  IOSTANDARD LVCMOS25} [get_ports gpio_ctl[3]]
set_property  -dict {PACKAGE_PIN  P20  IOSTANDARD LVCMOS25} [get_ports gpio_en_agc]
set_property  -dict {PACKAGE_PIN  R19  IOSTANDARD LVCMOS25} [get_ports gpio_resetb]

set_property  -dict {PACKAGE_PIN  T15  IOSTANDARD LVCMOS25} [get_ports enable]
set_property  -dict {PACKAGE_PIN  P18  IOSTANDARD LVCMOS25} [get_ports txnrx]

# ad9361 spi (PS SPI0 via EMIO)

set_property  -dict {PACKAGE_PIN  R17  IOSTANDARD LVCMOS25 PULLTYPE PULLUP} [get_ports spi_csn]
set_property  -dict {PACKAGE_PIN  V18  IOSTANDARD LVCMOS25} [get_ports spi_clk]
set_property  -dict {PACKAGE_PIN  P16  IOSTANDARD LVCMOS25} [get_ports spi_mosi]
set_property  -dict {PACKAGE_PIN  V17  IOSTANDARD LVCMOS25} [get_ports spi_miso]

# i2c (axi_iic) and auxiliary PL spi (axi_quad_spi)

set_property  -dict {PACKAGE_PIN  M14  IOSTANDARD LVCMOS25 PULLTYPE PULLUP} [get_ports iic_scl]
set_property  -dict {PACKAGE_PIN  M15  IOSTANDARD LVCMOS25 PULLTYPE PULLUP} [get_ports iic_sda]

set_property  -dict {PACKAGE_PIN  L14  IOSTANDARD LVCMOS25} [get_ports pl_spi_clk_o]
set_property  -dict {PACKAGE_PIN  N15  IOSTANDARD LVCMOS25} [get_ports pl_spi_miso]
set_property  -dict {PACKAGE_PIN  N16  IOSTANDARD LVCMOS25} [get_ports pl_spi_mosi]

# clocks

create_clock -name rx_clk     -period  4.00 [get_ports rx_clk_in_p]
create_clock -name clk_fpga_0 -period 10    [get_pins "i_system_wrapper/system_i/sys_ps7/inst/PS7_i/FCLKCLK[0]"]
create_clock -name clk_fpga_1 -period  5    [get_pins "i_system_wrapper/system_i/sys_ps7/inst/PS7_i/FCLKCLK[1]"]
create_clock -name spi0_clk   -period 40    [get_pins -hier */EMIOSPI0SCLKO]

set_input_jitter clk_fpga_0 0.3
set_input_jitter clk_fpga_1 0.15

set_false_path -from [get_pins {i_system_wrapper/system_i/axi_ad9361/inst/i_rx/i_up_adc_common/up_adc_gpio_out_int_reg[0]/C}]
set_false_path -from [get_pins {i_system_wrapper/system_i/axi_ad9361/inst/i_tx/i_up_dac_common/up_dac_gpio_out_int_reg[0]/C}]

# The FIT image (bitstream + kernel + rootfs) must fit in the 14 MB QSPI
# partition; the 7020 bitstream is ~4 MB uncompressed.
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

# DVB-S2 TX: sys_cpu_clk <-> AD9361 l_clk crossings inside dvbs2_tx
# (Gray-coded FIFO pointers, LUTRAM read data, control-bit synchronisers)
set dvbs2_cdc [get_cells -hier -filter {NAME =~ */dvbs2_tx/*/i_cdc}]
set_max_delay -datapath_only 5.0 \
  -from [get_cells -hier -filter "NAME =~ ${dvbs2_cdc}/wgray_reg*"] \
  -to   [get_cells -hier -filter "NAME =~ ${dvbs2_cdc}/wgray_r1_reg*"]
set_max_delay -datapath_only 5.0 \
  -from [get_cells -hier -filter "NAME =~ ${dvbs2_cdc}/rgray_r_reg*"] \
  -to   [get_cells -hier -filter "NAME =~ ${dvbs2_cdc}/rgray_w1_reg*"]
set_max_delay -datapath_only 5.0 \
  -from [get_cells -hier -filter "NAME =~ ${dvbs2_cdc}/mem_reg*"] \
  -to   [get_cells -hier -filter {NAME =~ */dvbs2_tx/*/dac_out_*_reg*}]
set_false_path -to [get_cells -hier -filter {NAME =~ */dvbs2_tx/*/drst_sync_reg[0]}]
set_false_path -to [get_cells -hier -filter {NAME =~ */dvbs2_tx/*/dsel_sync_reg[0]}]
set_false_path -to [get_cells -hier -filter {NAME =~ */dvbs2_tx/*/unf_sync_reg[0]}]

# DVB-S2 RX: AD9361 l_clk -> sys_cpu_clk ADC sample FIFO inside dvbs2_rx
set dvbs2_rx_cdc [get_cells -hier -filter {NAME =~ */dvbs2_rx/*/u_adc_fifo}]
set_max_delay -datapath_only 5.0 \
  -from [get_cells -hier -filter "NAME =~ ${dvbs2_rx_cdc}/wgray_reg*"] \
  -to   [get_cells -hier -filter "NAME =~ ${dvbs2_rx_cdc}/wgray_r1_reg*"]
set_max_delay -datapath_only 5.0 \
  -from [get_cells -hier -filter "NAME =~ ${dvbs2_rx_cdc}/rgray_r_reg*"] \
  -to   [get_cells -hier -filter "NAME =~ ${dvbs2_rx_cdc}/rgray_w1_reg*"]
set_max_delay -datapath_only 10.0 \
  -from [get_cells -hier -filter "NAME =~ ${dvbs2_rx_cdc}/mem_reg*"]
set_false_path -to [get_cells -hier -filter {NAME =~ */dvbs2_rx/*/en_s_reg[0]}]
set_false_path -to [get_cells -hier -filter {NAME =~ */dvbs2_rx/*/rst_s_reg[0]}]

# Narrowband IQ channels: AD9361 l_clk -> sys_cpu_clk ADC sample FIFO inside nbiq
set nbiq_cdc [get_cells -hier -filter {NAME =~ */nbiq/*/u_adc_fifo}]
set_max_delay -datapath_only 5.0 \
  -from [get_cells -hier -filter "NAME =~ ${nbiq_cdc}/wgray_reg*"] \
  -to   [get_cells -hier -filter "NAME =~ ${nbiq_cdc}/wgray_r1_reg*"]
set_max_delay -datapath_only 5.0 \
  -from [get_cells -hier -filter "NAME =~ ${nbiq_cdc}/rgray_r_reg*"] \
  -to   [get_cells -hier -filter "NAME =~ ${nbiq_cdc}/rgray_w1_reg*"]
set_max_delay -datapath_only 10.0 \
  -from [get_cells -hier -filter "NAME =~ ${nbiq_cdc}/mem_reg*"]
set_false_path -to [get_cells -hier -filter {NAME =~ */nbiq/*/rst_s_reg[0]}]

# Self-test noise source: synchronisers, quasi-static configuration
# (sys_cpu_clk -> l_clk) and the power meter results (l_clk -> sys_cpu_clk)
set awgn_i [get_cells -hier -filter {NAME =~ */rx_awgn/*/i_rx_awgn_axi}]
set_false_path -to [get_cells -hier -filter "NAME =~ ${awgn_i}/arst_s_reg[0]"]
set_false_path -to [get_cells -hier -filter "NAME =~ ${awgn_i}/cfg_s_reg[0]"]
set_false_path -to [get_cells -hier -filter "NAME =~ ${awgn_i}/p_s_reg[0]"]
set_max_delay -datapath_only 10.0 \
  -from [get_cells -hier -filter "NAME =~ ${awgn_i}/en_reg || NAME =~ ${awgn_i}/gain_reg* || NAME =~ ${awgn_i}/nscale_reg*"] \
  -to   [get_cells -hier -filter "NAME =~ ${awgn_i}/a_en_reg || NAME =~ ${awgn_i}/a_gain_reg* || NAME =~ ${awgn_i}/a_nscale_reg*"]
set_max_delay -datapath_only 10.0 \
  -from [get_cells -hier -filter "NAME =~ ${awgn_i}/h_in_reg* || NAME =~ ${awgn_i}/h_out_reg*"] \
  -to   [get_cells -hier -filter "NAME =~ ${awgn_i}/r_in_reg* || NAME =~ ${awgn_i}/r_out_reg*"]
