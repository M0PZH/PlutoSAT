###############################################################################
## fishball-sdr-7020 Vivado project (out-of-tree ADI HDL project).
## ADI_HDL_DIR must point at the ADI hdl checkout (the Makefile exports it).
###############################################################################

source $::env(ADI_HDL_DIR)/scripts/adi_env.tcl
source $ad_hdl_dir/projects/scripts/adi_project_xilinx.tcl
source $ad_hdl_dir/projects/scripts/adi_board.tcl

adi_project_create fishball 0 {} "xc7z020clg400-2"

adi_project_files fishball [list \
  "system_top.v" \
  "system_constr.xdc" \
  "$ad_hdl_dir/library/common/ad_iobuf.v"]

set_property is_enabled false [get_files  *system_sys_ps7_0.xdc]

# the DVB-S2 RX core fills the device: timing-driven placement and routing,
# physical optimisation before and after routing
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE ExtraTimingOpt [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]

adi_project_run fishball
source $ad_hdl_dir/library/axi_ad9361/axi_ad9361_delay.tcl
