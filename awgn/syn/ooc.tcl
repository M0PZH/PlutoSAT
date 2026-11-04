# Out-of-context implementation of rx_awgn_axi (ADC clock 250 MHz, AXI 100 MHz):
#   vivado -mode batch -source ooc.tcl
set here [file dirname [file normalize [info script]]]
set rtl [file join $here .. rtl]
set out [file join $here out]
file mkdir $out
read_verilog -sv [list [file join $rtl rx_awgn.sv] [file join $rtl rx_awgn_axi.sv]]
synth_design -top rx_awgn_axi -part xc7z020clg400-2 -mode out_of_context
create_clock -period 10.0 [get_ports s_axi_aclk]
create_clock -period 4.0 [get_ports adc_clk]
set_clock_groups -asynchronous -group s_axi_aclk -group adc_clk
opt_design
place_design
route_design
report_utilization -file [file join $out util.rpt]
report_timing_summary -file [file join $out timing.rpt]
