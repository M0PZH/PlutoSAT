# Out-of-context implementation of the DVB-S2 cores for resource/timing numbers.
#   vivado -mode batch -source ooc.tcl [-tclargs <clock MHz> [dvbs2_tx | dvbs2_rx | dvbs2_ldpc_dec]]
set mhz [expr {[llength $argv] > 0 ? [lindex $argv 0] : 150}]
set top [expr {[llength $argv] > 1 ? [lindex $argv 1] : "dvbs2_tx"}]
set here [file dirname [file normalize [info script]]]
set rtl  [file join $here .. rtl]
set gen  [file join $rtl gen]
set out  [file join $here out_[string map {dvbs2_ ""} $top]_${mhz}mhz]
file mkdir $out

set pkgs [list [file join $gen dvbs2_tables_pkg.sv] [file join $gen dvbs2_rx_pkg.sv] \
            [file join $gen dvbs2_ldpc_dec_pkg.sv] [file join $rtl dvbs2_pl_pkg.sv]]
read_verilog -sv $pkgs
foreach f [glob [file join $rtl *.sv]] {
  if {[lsearch $pkgs $f] < 0} { read_verilog -sv $f }
}
if {$top eq "dvbs2_tx"} {
  synth_design -top dvbs2_tx -part xc7z020clg400-2 -mode out_of_context \
    -generic LDPC_ROM_FILE=\"[file join $gen ldpc_rom.mem]\" \
    -generic NCO_LUT_FILE=\"[file join $gen nco_lut.mem]\"
} elseif {$top eq "dvbs2_ldpc_dec"} {
  synth_design -top $top -part xc7z020clg400-2 -mode out_of_context \
    -generic ROM_FILE=\"[file join $gen ldpc_dec_rom.mem]\"
} else {
  synth_design -top $top -part xc7z020clg400-2 -mode out_of_context \
    -generic LUT_FILE=\"[file join $gen nco_lut.mem]\"
}
create_clock -name clk -period [expr {1000.0 / $mhz}] [get_ports clk]
if {[llength [get_ports -quiet in_clk]]} {
  create_clock -name in_clk -period [expr {1000.0 / $mhz}] [get_ports in_clk]
  set_clock_groups -asynchronous -group clk -group in_clk
}
opt_design
place_design
route_design
report_utilization -file [file join $out utilization.rpt]
report_utilization -hierarchical -hierarchical_depth 2 -file [file join $out utilization_hier.rpt]
report_timing_summary -file [file join $out timing.rpt]
report_timing -max_paths 200 -nworst 1 -unique_pins -slack_lesser_than 0 -file [file join $out failing_paths.rpt]
