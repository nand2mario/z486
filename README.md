
# z486 - an 80486-class pipelined FPGA CPU in SystemVerilog

z486 is an x86 CPU core written in SystemVerilog and built around the original
Intel 80386 microcode. It combines the microcode engine with a pipelined
frontend, bounded hardwired implementations of common instructions, fixed-clock
speed control, and a microcoded x87 floating-point unit.

The separate [z386](https://github.com/nand2mario/z386) repository remains the
80386-faithful implementation. This repository contains the faster extended
core used by [z486_MiSTer](https://github.com/nand2mario/z486_MiSTer).
