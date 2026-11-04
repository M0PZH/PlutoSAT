set_param general.maxThreads 16
open_checkpoint /home/peter/plutosdr-fw/boards/fishball/hdl/fishball.runs/impl_1/system_top_routed.dcp
report_utilization -hierarchical -hierarchical_depth 8 -file /tmp/claude-1000/-home-peter-plutosdr-fw/88841fc5-81fc-4e77-97c4-6f4e979e8a85/scratchpad/rx2/board/hier8.rpt
report_utilization -slr -file /tmp/claude-1000/-home-peter-plutosdr-fw/88841fc5-81fc-4e77-97c4-6f4e979e8a85/scratchpad/rx2/board/dummy.rpt
set f [open /tmp/claude-1000/-home-peter-plutosdr-fw/88841fc5-81fc-4e77-97c4-6f4e979e8a85/scratchpad/rx2/board/brams.txt w]
foreach c [get_cells -hier -filter {REF_NAME =~ RAMB*}] { puts $f "[get_property REF_NAME $c] $c" }
close $f
set f [open /tmp/claude-1000/-home-peter-plutosdr-fw/88841fc5-81fc-4e77-97c4-6f4e979e8a85/scratchpad/rx2/board/dsps.txt w]
foreach c [get_cells -hier -filter {REF_NAME =~ DSP48*}] { puts $f "$c" }
close $f
report_design_analysis -congestion -file /tmp/claude-1000/-home-peter-plutosdr-fw/88841fc5-81fc-4e77-97c4-6f4e979e8a85/scratchpad/rx2/board/congestion.rpt
