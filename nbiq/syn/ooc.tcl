# Out-of-context synthesis + implementation of nbiq_axi for a resource and
# timing estimate:  vivado -mode batch -source ooc.tcl [-tclargs <MHz>]
set_param general.maxThreads 16
set mhz [expr {[llength $argv] > 0 ? [lindex $argv 0] : 100}]
set here [file dirname [file normalize [info script]]]
set rtl [file join $here .. rtl]
set out [file join $here out_nbiq]
file mkdir $out
read_verilog -sv [list [file join $rtl nbiq_chan.sv] [file join $rtl nbiq_comb.sv] [file join $rtl nbiq_fir.sv] [file join $rtl nbiq_sdpram.sv] \
  [file join $rtl nbiq_wr.sv] [file join $rtl nbiq_axi.sv] \
  [file join $here .. .. dvbs2 rtl dvbs2_async_fifo.sv]]
add_files [file join $rtl gen nbiq_lut.mem]
synth_design -top nbiq_axi -part xc7z020clg400-2 -mode out_of_context \
  -generic LUT_FILE=[file join $rtl gen nbiq_lut.mem]
create_clock -period [expr {1000.0 / $mhz}] [get_ports s_axi_aclk]
create_clock -period 16.0 [get_ports adc_clk]
set_clock_groups -asynchronous -group s_axi_aclk -group adc_clk
opt_design
if {[info exists ::env(SYNTH_ONLY)]} { report_utilization -hierarchical -hierarchical_depth 3 -file [file join $out util_hier.rpt]; report_ram_utilization -file [file join $out ram.rpt]; return }
place_design
route_design
report_utilization -file [file join $out util.rpt]
report_utilization -hierarchical -hierarchical_depth 3 -file [file join $out util_hier.rpt]
report_ram_utilization -file [file join $out ram.rpt]
report_timing_summary -file [file join $out timing.rpt]
