package require ::quartus::project

set script_dir [file dirname [file normalize [info script]]]
source [file join $script_dir production_settings.tcl]
set board_dir [file normalize [file join $script_dir ..]]
set cpu_dir [file normalize [file join $board_dir ../..]]
set cpu_mhz [expr {[llength $argv] > 0 ? [lindex $argv 0] : 50}]

if {$cpu_mhz != 50 && $cpu_mhz != 85} {
    error "DE10-Nano full build supports CPU_MHZ=50 or CPU_MHZ=85"
}

set fmax_build [expr {$cpu_mhz == 85}]
set build_dir [file join $board_dir [expr {$fmax_build ? "build-fmax" : "build"}]]
set project_name [expr {$fmax_build ? "z486_de10nano_fmax" : "z486_de10nano"}]

file mkdir $build_dir
cd $build_dir
project_new $project_name -overwrite

set_global_assignment -name FAMILY "Cyclone V"
set_global_assignment -name DEVICE 5CSEBA6U23I7
set_global_assignment -name TOP_LEVEL_ENTITY de10nano_top
set_global_assignment -name PROJECT_OUTPUT_DIRECTORY output_files
set_global_assignment -name NUM_PARALLEL_PROCESSORS ALL
set_global_assignment -name GENERATE_RBF_FILE ON
apply_z486_production_settings
if {!$fmax_build} {
    # The programming build must close its real 50 MHz clock. AUTO FIT may
    # skip timing-affecting passes once it judges further effort unnecessary.
    set_global_assignment -name FITTER_EFFORT "STANDARD FIT"
}
set_global_assignment -name VERILOG_MACRO "Z486_ALTERA=1"
set_global_assignment -name SEARCH_PATH $cpu_dir
set_global_assignment -name SEARCH_PATH [file join $cpu_dir x87]
set_parameter -name CPU_MHZ $cpu_mhz

set manifest [file join $cpu_dir boards common rtl_sources.f]
set manifest_file [open $manifest r]
while {[gets $manifest_file line] >= 0} {
    set line [string trim $line]
    if {$line eq "" || [string match "#*" $line]} {
        continue
    }
    set_global_assignment -name SYSTEMVERILOG_FILE [file join $cpu_dir $line]
}
close $manifest_file

set_global_assignment -name SYSTEMVERILOG_FILE \
    [file join $board_dir rtl de10nano_pll.sv]
set_global_assignment -name SYSTEMVERILOG_FILE \
    [file join $board_dir rtl de10nano_top.sv]
set_global_assignment -name SDC_FILE \
    [file join $board_dir constraints de10nano.sdc]

foreach image [list \
        [file join $cpu_dir pla_entry_rom.hex] \
        [file join $cpu_dir pla_group_entry.hex] \
        [file join $cpu_dir boards firmware blink.hex]] {
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

set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to FPGA_CLK1_50
set_location_assignment PIN_V11 -to FPGA_CLK1_50
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to KEY\[0\]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to KEY\[1\]
set_location_assignment PIN_AH17 -to KEY\[0\]
set_location_assignment PIN_AH16 -to KEY\[1\]

set led_pins {PIN_W15 PIN_AA24 PIN_V16 PIN_V15 PIN_AF26 PIN_AE26 PIN_Y16 PIN_AA23}
for {set i 0} {$i < 8} {incr i} {
    set port "LED\[$i\]"
    set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to $port
    set_location_assignment [lindex $led_pins $i] -to $port
}

export_assignments
project_close
