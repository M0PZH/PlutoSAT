set_param general.maxThreads 16
open_checkpoint /home/peter/plutosdr-fw/boards/fishball/hdl/fishball.runs/impl_1/system_top_routed.dcp
report_utilization -hierarchical -hierarchical_depth 4 -file /tmp/claude-1000/-home-peter-plutosdr-fw/88841fc5-81fc-4e77-97c4-6f4e979e8a85/scratchpad/rx2/board/hier4.rpt
report_utilization -file /tmp/claude-1000/-home-peter-plutosdr-fw/88841fc5-81fc-4e77-97c4-6f4e979e8a85/scratchpad/rx2/board/util.rpt
set f [open /tmp/claude-1000/-home-peter-plutosdr-fw/88841fc5-81fc-4e77-97c4-6f4e979e8a85/scratchpad/rx2/board/brams.txt w]
foreach c [get_cells -hier -filter {PRIMITIVE_GROUP==BLOCKRAM}] { puts $f "[get_property REF_NAME $c] [get_property LOC $c] $c" }
close $f
set f [open /tmp/claude-1000/-home-peter-plutosdr-fw/88841fc5-81fc-4e77-97c4-6f4e979e8a85/scratchpad/rx2/board/dsps.txt w]
foreach c [get_cells -hier -filter {PRIMITIVE_GROUP==ARITHMETIC}] { puts $f "[get_property REF_NAME $c] $c" }
close $f
report_timing -max_paths 30 -nworst 1 -unique_pins -file /tmp/claude-1000/-home-peter-plutosdr-fw/88841fc5-81fc-4e77-97c4-6f4e979e8a85/scratchpad/rx2/board/worst.rpt
