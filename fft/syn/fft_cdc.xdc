# Clock-domain crossings inside fft_axi (AD9363 l_clk -> s_axi_aclk), for the
# out-of-context run (ooc.tcl). The same constraints for the full design, in
# the style of boards/fishball/hdl/system_constr.xdc, are in README.md.
#   u_adc_fifo: Gray-coded pointers both ways, LUTRAM read data
#   rst_s:      reset synchroniser into adc_clk
set fft_cdc [get_cells -hier -filter {NAME =~ *u_adc_fifo}]
set_max_delay -datapath_only 4.0 \
  -from [get_cells -hier -filter "NAME =~ ${fft_cdc}/wgray_reg*"] \
  -to   [get_cells -hier -filter "NAME =~ ${fft_cdc}/wgray_r1_reg*"]
set_max_delay -datapath_only 4.0 \
  -from [get_cells -hier -filter "NAME =~ ${fft_cdc}/rgray_r_reg*"] \
  -to   [get_cells -hier -filter "NAME =~ ${fft_cdc}/rgray_w1_reg*"]
set_max_delay -datapath_only 10.0 \
  -from [get_cells -hier -filter "NAME =~ ${fft_cdc}/mem_reg*"]
set_false_path -to [get_cells -hier -filter {NAME =~ *rst_s_reg[0]}]
