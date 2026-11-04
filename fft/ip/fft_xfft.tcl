# Creates the Xilinx FFT instance "fft_xfft" that fft/rtl/fft_axi.sv
# instantiates, in the current project (or in-memory project).
#
#   source fft/ip/fft_xfft.tcl
#   fft_create_xfft ?ip_dir? ?rounding?
#
# ip_dir:   where the .xci goes (default: the project's own IP directory)
# rounding: convergent_rounding (default, what the RTL and the model expect)
#           or truncation
#
# 4096 points, radix-2 Lite burst I/O, 16-bit input and phase factors, fixed
# scaling (run-time schedule), natural output order, non-realtime throttle
# (the input may pause between samples, the output may be held off),
# overflow flag. 100 MHz.

proc fft_create_xfft {{ip_dir ""} {rounding convergent_rounding}} {
  set args [list -vlnv xilinx.com:ip:xfft:9.1 -module_name fft_xfft]
  if {$ip_dir ne ""} {
    file mkdir $ip_dir
    lappend args -dir $ip_dir
  }
  create_ip {*}$args
  set ip [get_ips fft_xfft]
  set_property -dict [list \
    CONFIG.transform_length 4096 \
    CONFIG.implementation_options radix_2_lite_burst_io \
    CONFIG.target_clock_frequency 100 \
    CONFIG.input_width 16 \
    CONFIG.phase_factor_width 16 \
    CONFIG.scaling_options scaled \
    CONFIG.rounding_modes $rounding \
    CONFIG.output_ordering natural_order \
    CONFIG.throttle_scheme nonrealtime \
    CONFIG.aresetn true \
    CONFIG.ovflo true \
    CONFIG.xk_index false \
    CONFIG.run_time_configurable_transform_length false \
    CONFIG.cyclic_prefix_insertion false \
    CONFIG.memory_options_data block_ram \
    CONFIG.memory_options_phase_factors block_ram \
    CONFIG.memory_options_reorder block_ram \
    CONFIG.butterfly_type use_luts \
    CONFIG.complex_mult_type use_mults_resources \
  ] $ip
  return $ip
}
