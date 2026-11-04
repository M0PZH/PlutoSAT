# Generates the fft_xfft simulation model and its xsim compile scripts:
#   vivado -mode batch -source build_ip.tcl -tclargs <out_dir>
set_param general.maxThreads 16
set out [file normalize [lindex $argv 0]]
set here [file dirname [file normalize [info script]]]
create_project -in_memory -part xc7z020clg400-2
source [file join $here .. ip fft_xfft.tcl]
fft_create_xfft [file join $out ip]
generate_target {simulation} [get_ips fft_xfft]
export_ip_user_files -of_objects [get_ips fft_xfft] -no_script -force -quiet
export_simulation -of_objects [get_ips fft_xfft] -simulator xsim \
  -directory [file join $out export] -force
