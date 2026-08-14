
# z486 - an 80486-class pipelined FPGA CPU in SystemVerilog

z486 is an 80486-class pipelined CPU core written in SystemVerilog. A fast
frontend and hardwired implementations handle common instructions, while a
microcoded control engine handles complex x86 operations. The core also includes
experimental, incomplete x87 support sufficient to run TurboQuake.

In the z486_MiSTer system, the core runs the Doom timedemo at 29.1 FPS at
maximum detail, compared with 21.0 FPS on ao486 using the same MiSTer setup.

The separate [z386](https://github.com/nand2mario/z386) repository remains the
80386-faithful implementation. This repository contains the faster extended
core used by [z486_MiSTer](https://github.com/nand2mario/z486_MiSTer).

## License

Copyright 2026 nand2mario. The SystemVerilog, Python, and Markdown files
(`*.sv`, `*.svh`, `*.py`, and `*.md`) are licensed under the
[Apache License 2.0](LICENSE). See
[License Scope](LICENSE-SCOPE.md) for details.
