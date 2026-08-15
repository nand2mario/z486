# z486 - an 80486-class pipelined FPGA CPU in SystemVerilog

z486 is an 80486-class pipelined CPU core written in SystemVerilog. It combines
a fast frontend and hardwired implementations of common instructions with a
microcoded control engine for complex x86 behavior. The core also integrates
experimental, incomplete x87 floating-point support sufficient to run DOS
Quake.

The design targets useful 486-class performance on mid-sized FPGAs while
retaining protected mode, paging, segmentation, and the complex architectural
behavior inherited from z386.

This repository contains the CPU RTL and its verification environment. To run
z486 as a complete PC on MiSTer, use
[z486_MiSTer](https://github.com/nand2mario/z486_MiSTer).

## Design

The frontend decodes instruction structure and literals through D1 and D2
stages. From D2, instructions enter one of two control paths:

* Hardwired control executes common data movement, arithmetic, stack, branch,
  and memory operations as short sequences of hardware steps.
* The microsequencer executes complex instructions and architectural corner
  cases through the existing microcoded datapath.

Both paths share the address, integer data, memory, protection, interrupt, and
floating-point units. Separate configurable instruction and data caches connect
the CPU to the external memory interface. The microcode-driven x87 unit shares
arithmetic hardware across operations to keep its FPGA area manageable.

The top-level module is `z486`. Its main configuration parameters include
`DCACHE_SET_BITS`, `ICACHE_SET_BITS`, and `ENABLE_X87`.

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
instruction and 8 KB data caches; ao486 uses its native cache. Area figures are
standalone seed-1 fits on the same DE10-Nano Cyclone V with identical Quartus
settings. z486 includes x87; without it, z486 uses 16,329 ALMs, 5.0% more than
z386. ao486 counts retirement at a different pipeline boundary, so its CPI is
less directly comparable.

### DOOM

At 85 MHz in z486_MiSTer, z486 runs the maximum-detail Doom timedemo at 29.1
FPS: 39% faster than ao486 at 90 MHz and 27% faster than z386 v0.4 at 85 MHz.

![Board-measured Doom and 3DBench performance](docs/dos_performance.svg)

These are board measurements using each core's native release configuration:
85 MHz for z386 and z486, and 90 MHz for ao486. The z386 v0.4 result uses its
release 16 KB instruction and 16 KB data caches, whereas the Dhrystone
comparison above uses matched 8 KB + 8 KB configurations for z386 and z486.

The complete methodology and analysis are in the
[z486 technical report](https://nand2mario.github.io/posts/2026/z486/).

## Build and test

The regression tests use Verilator and Python:

```bash
cd tests
make test-simple
make test-protected
make test-l1-cache
make test-l1-icache
make test-x87-core
make test-x87-integration
```

Additional targets cover Dhrystone, Berkeley TestFloat, the broader
`test386.py` suite, and external real- and protected-mode single-step reference
datasets. Run `make help` in `tests` for the current target list.

## Related projects

The [z386](https://github.com/nand2mario/z386) repository contains the
80386-faithful predecessor.

## Credits

z486 was written by nand2mario and developed from z386. The underlying 80386
work builds on Intel microcode disassembly and silicon reverse-engineering by
[reenigne](https://www.reenigne.org/blog/),
[gloriouscow](https://github.com/dbalsom),
[smartest blob](https://github.com/a-mcego), and
[Ken Shirriff](https://www.righto.com/).

## License

Copyright 2026 nand2mario. The SystemVerilog, Python, and Markdown files
(`*.sv`, `*.svh`, `*.py`, and `*.md`) are licensed under the
[Apache License 2.0](LICENSE). See [License Scope](LICENSE-SCOPE.md) for
details. The recovered microcode image is not covered by this license.
