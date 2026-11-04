# Out-of-context synthesis + implementation of the spectrum analyser (fft_bd:
# fft_axi with the xfft IP) for a resource and timing estimate:
#   vivado -mode batch -source ooc.tcl [-tclargs <DOUBLE_BUF 0|1> [<MHz>]]
# s_axi_aclk at <MHz> (default 100), adc_clk at 250 MHz (4 ns, the l_clk
# constraint). Reports in out_db<DOUBLE_BUF>/. OOC_THREADS (environment)
# sets the thread count (default 16). The xfft IP is generated once into
# out_ip/ (delete it after changing ip/fft_xfft.tcl).
set_param general.maxThreads 16
if {[info exists ::env(OOC_THREADS)]} { set_param general.maxThreads $::env(OOC_THREADS) }
set db  [expr {[llength $argv] > 0 ? [lindex $argv 0] : 0}]
set mhz [expr {[llength $argv] > 1 ? [lindex $argv 1] : 100}]
set here [file dirname [file normalize [info script]]]
set fft [file join $here ..]
set out [file join $here out_db$db]
file mkdir $out
cd $out

create_project -in_memory -part xc7z020clg400-2
source [file join $fft ip fft_xfft.tcl]
set ipdir [file join $here out_ip]
if {[file exists [file join $ipdir fft_xfft fft_xfft.xci]]} {
  read_ip [file join $ipdir fft_xfft fft_xfft.xci]
} else {
  fft_create_xfft $ipdir
}
set_property generate_synth_checkpoint false [get_files fft_xfft.xci]
generate_target {synthesis} [get_ips fft_xfft]

read_verilog -sv [list [file join $fft rtl fft_win.sv] [file join $fft rtl fft_acc.sv] \
  [file join $fft rtl fft_axi.sv] [file join $fft .. dvbs2 rtl dvbs2_async_fifo.sv]]
read_verilog [file join $fft ip fft_bd.v]
add_files [file join $fft rtl gen fft_win.mem]
read_xdc -mode out_of_context [file join $here fft_cdc.xdc]

synth_design -top fft_bd -part xc7z020clg400-2 -mode out_of_context -generic DOUBLE_BUF=$db
create_clock -name s_axi_aclk -period [expr {1000.0 / $mhz}] [get_ports s_axi_aclk]
create_clock -name adc_clk -period 4.0 [get_ports adc_clk]
report_utilization -file [file join $out util_synth.rpt]
opt_design
place_design
phys_opt_design
route_design
report_utilization -file [file join $out util.rpt]
report_utilization -hierarchical -hierarchical_depth 4 -file [file join $out util_hier.rpt]
report_ram_utilization -file [file join $out ram.rpt]
report_timing_summary -max_paths 10 -file [file join $out timing.rpt]
report_cdc -details -file [file join $out cdc.rpt]
report_methodology -file [file join $out methodology.rpt]
foreach c {s_axi_aclk adc_clk} {
  set p [get_timing_paths -max_paths 1 -setup -to [get_clocks $c]]
  set h [get_timing_paths -max_paths 1 -hold -to [get_clocks $c]]
  puts "RESULT WNS $c [get_property SLACK $p] WHS [get_property SLACK $h]"
}
