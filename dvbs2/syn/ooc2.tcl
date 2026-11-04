# Out-of-context implementation of one RX engine, single-slot or two-slot
# (docs/rx2_design.md), for resource / timing numbers:
#   vivado -mode batch -source ooc2.tcl -tclargs <clock MHz> <top>
#   tops: dvbs2_rx_timing(2), dvbs2_rx_framesync(2), dvbs2_rx_carrier(2), dvbs2_rx_pls
# Reports go to out2_<top>_<MHz>mhz/ (utilization, per-hierarchy utilization,
# timing summary, RAM utilization).
if {[info exists ::env(OOC_THREADS)]} { set_param general.maxThreads $::env(OOC_THREADS) }
set mhz [lindex $argv 0]
set top [lindex $argv 1]
set here [file dirname [file normalize [info script]]]
set rtl  [file join $here .. rtl]
set gen  [file join $rtl gen]
set out  [file join $here out2_${top}_${mhz}mhz]
file mkdir $out
set pkgs [list [file join $gen dvbs2_tables_pkg.sv] [file join $gen dvbs2_rx_pkg.sv] \
            [file join $gen dvbs2_ldpc_dec_pkg.sv] [file join $gen dvbs2_bch_pkg.sv] \
            [file join $rtl dvbs2_pl_pkg.sv]]
read_verilog -sv $pkgs
foreach f [glob [file join $rtl *.sv]] { if {[lsearch $pkgs $f] < 0} { read_verilog -sv $f } }
set gargs {}
if {[string match "dvbs2_rx_carrier*" $top]} { lappend gargs -generic LUT_FILE=\"[file join $gen nco_lut.mem]\" }
synth_design -top $top -part xc7z020clg400-2 -mode out_of_context {*}$gargs
create_clock -name clk -period [expr {1000.0 / $mhz}] [get_ports clk]
opt_design
place_design
route_design
report_utilization -file [file join $out utilization.rpt]
report_utilization -hierarchical -hierarchical_depth 3 -file [file join $out utilization_hier.rpt]
report_timing_summary -file [file join $out timing.rpt]
report_ram_utilization -file [file join $out ram.rpt]
