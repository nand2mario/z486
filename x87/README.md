# x87

This directory contains the microcode-driven x87 implementation used by z386x.
The older operation-specific implementation has been retired; selected blocks
remain under `tests/x87_reference` only as differential-test references.

`build_x87_microcode.py` is the control-store source of truth. It emits the
MIF used by Quartus, the SVH used by simulation, symbolic entry addresses, and
a review listing. Do not hand-edit those generated files.
