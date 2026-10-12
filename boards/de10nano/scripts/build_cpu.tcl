package require ::quartus::project

set script_dir [file dirname [file normalize [info script]]]
source [file join $script_dir production_settings.tcl]
set board_dir [file normalize [file join $script_dir ..]]
set cpu_dir [file normalize [file join $board_dir ../..]]
set enable_x87 [expr {[llength $argv] > 0 && [lindex $argv 0] eq "1"}]
set cpu_mhz [expr {[llength $argv] > 1 ? [lindex $argv 1] : 85}]

if {![string is integer -strict $cpu_mhz] || $cpu_mhz < 1 || $cpu_mhz > 127} {
    error "CPU frequency must be an integer from 1 through 127 MHz"
}

set suffix [expr {$enable_x87 ? "-x87" : ""}]
set project_suffix [expr {$enable_x87 ? "_x87" : ""}]
set build_dir [file join $board_dir "build-cpu${suffix}"]
set project_name "z486_de10nano_cpu${project_suffix}"
set period_ns [expr {1000.0 / double($cpu_mhz)}]

file mkdir $build_dir
cd $build_dir
project_new $project_name -overwrite

set_global_assignment -name FAMILY "Cyclone V"
set_global_assignment -name DEVICE 5CSEBA6U23I7
set_global_assignment -name TOP_LEVEL_ENTITY z486
set_global_assignment -name PROJECT_OUTPUT_DIRECTORY output_files
set_global_assignment -name NUM_PARALLEL_PROCESSORS ALL
apply_z486_production_settings
set_global_assignment -name VERILOG_MACRO "Z486_ALTERA=1"
set_global_assignment -name SEARCH_PATH $cpu_dir
set_global_assignment -name SEARCH_PATH [file join $cpu_dir x87]
set_parameter -name ENABLE_X87 $enable_x87
set_parameter -name CLOCK_RATE_MHZ $cpu_mhz

set manifest [file join $cpu_dir boards common rtl_sources.f]
set manifest_file [open $manifest r]
while {[gets $manifest_file line] >= 0} {
    set line [string trim $line]
    if {$line eq "" || [string match "#*" $line]} {
        continue
    }
    if {[string match "boards/common/*" $line]} {
        continue
    }
    set_global_assignment -name SYSTEMVERILOG_FILE [file join $cpu_dir $line]
}
close $manifest_file

foreach image [list \
        [file join $cpu_dir pla_entry_rom.hex] \
        [file join $cpu_dir pla_group_entry.hex]] {
    file copy -force $image [file join $build_dir [file tail $image]]
    set_global_assignment -name HEX_FILE $image
}

foreach image [list \
        [file join $cpu_dir ucode.mif] \
        [file join $cpu_dir x87 x87_ucode.mif] \
        [file join $cpu_dir x87 x87_command_decode.mif] \
        [file join $cpu_dir x87 x87_logexp_tables.mif]] {
    file copy -force $image [file join $build_dir [file tail $image]]
    set_global_assignment -name MIF_FILE $image
}

set constraints_file [file join $build_dir cpu_ooc.sdc]
set constraints [open $constraints_file w]
puts $constraints [format {create_clock -name cpu_clk -period %.6f [get_ports clk]} \
    $period_ns]
puts $constraints {set_false_path -from [get_ports reset_n]}
puts $constraints {set data_inputs [remove_from_collection [all_inputs] [get_ports {clk reset_n}]]}
puts $constraints {set_input_delay -clock cpu_clk 0.0 $data_inputs}
puts $constraints {set_output_delay -clock cpu_clk 0.0 [all_outputs]}
puts $constraints {derive_clock_uncertainty}
close $constraints
set_global_assignment -name SDC_FILE $constraints_file

# Use the board's dedicated clock input so the clock network is realistic.
# Every other CPU port is virtual: it remains observable without consuming a
# package pin or adding package-I/O delay to the CPU measurement.
set_location_assignment PIN_V11 -to clk
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to clk
foreach port {
    reset_n addr[*] be[*] burstcount[*] din[*] dout[*] valid ready write io
    resp_valid intr nmi inta snoop_addr[*] snoop_valid a20_enable
    cpu_speed_sel[*] single_step dbg_CS[*] dbg_EIP[*] dbg_CS_base[*]
    dbg_pe dbg_vm dbg_x87_state[*] triple_fault_reset
} {
    set_instance_assignment -name VIRTUAL_PIN ON -to $port
}

export_assignments
project_close
