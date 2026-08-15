
# z486 - an 80486-class pipelined FPGA CPU in SystemVerilog

z486 is an 80486-class pipelined CPU core written in SystemVerilog. A fast
frontend and hardwired implementations handle common instructions, while a
microcoded control engine handles complex x86 operations. The core also includes
experimental, incomplete x87 support sufficient to run TurboQuake.

The separate [z386](https://github.com/nand2mario/z386) repository remains the
80386-faithful implementation. This repository contains the faster extended
core used by [z486_MiSTer](https://github.com/nand2mario/z486_MiSTer).

## Performance

### Dhrystone 2.1

z486 delivers 70% more Dhrystone performance per MHz than ao486 and 47% more
than z386, as measured by DMIPS/MHz.

| Core | DMIPS/MHz | CPI | Cyclone V ALMs |
| --- | ---: | ---: | ---: |
| z386 | 0.225 | 4.101 | 15,545 |
| ao486 | 0.194 | 4.556 | 15,190 |
| **z486** | **0.330** | **2.800** | **21,906** |

All cores execute the same i386 binary. z386 and z486 use matched 8 KB
instruction and 8 KB data caches; ao486 uses its native cache. ALMs are
standalone seed-1 fits on the same DE10-Nano Cyclone V with identical Quartus
settings. z486 includes x87; without it, z486 uses 16,329 ALMs, 5.0% more than
z386. ao486 counts retirement at a different boundary, so its CPI is less
directly comparable.

### DOOM

![Board-measured Doom and 3DBench performance](docs/dos_performance.svg)

These are board measurements using each core's native release configuration:
85 MHz for z386 and z486, and 90 MHz for ao486. The z386 v0.4 result uses its
release 16 KB instruction and 16 KB data caches, whereas the Dhrystone
comparison above uses matched 8 KB + 8 KB configurations for z386 and z486.

The complete methodology and analysis are in the
[z486 technical report](https://nand2mario.github.io/posts/2026/z486/).

## License

Copyright 2026 nand2mario. The SystemVerilog, Python, and Markdown files
(`*.sv`, `*.svh`, `*.py`, and `*.md`) are licensed under the
[Apache License 2.0](LICENSE). See
[License Scope](LICENSE-SCOPE.md) for details.
