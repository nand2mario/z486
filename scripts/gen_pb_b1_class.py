#!/usr/bin/env python3
"""Regenerate pb_b1_class(), the successor class for port B's dead slot.

hardwired_control decides at a predecessor's issue whether the instruction
leaving D1 may take the predecessor's dead slot (B1).  The decision needs four
bits of that instruction's recipe: B1-eligible type, reads flags, uses an EA,
and Jcc.  Taken from the D1 entry point, they sit behind the entry ROMs, the
entry select and the recipe table, which set the clk_sys critical path.

With no REP or LOCK prefix the four bits depend only on the 0F prefix, the
opcode, ModR/M.reg and whether ModR/M names memory; never on the operand size
or PE.  This script proves that by evaluating every case with the decoder's
own lookups and the recipe functions (a Verilator testbench that includes
pla_control.svh, pla_entry.svh, z486_pkg.sv and both ROM hex files), then
writes the table as a casez function in ../pb_b1_class.svh.  A simulation
check in hardwired_control compares it with the recipe table at every load.

Rerun after changing the entry PLA, the group ROM or ucode_recipes.svh:

    ./gen_pb_b1_class.py
"""

from __future__ import annotations

import collections
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
CORE = HERE.parent
OUT = CORE / "pb_b1_class.svh"

TB = r"""
module tb_pb_b1_class;
import z486_pkg::*;
`include "pla_control.svh"
`include "pla_entry.svh"

// As hardwired_control.sv.
function automatic logic pb_b1_type(input dec_entry_t e, input recipe_meta_t r);
    logic [2:0] k;
    begin
        k = recipe_early_kind(e.entry_point);
        pb_b1_type = r.hardwired &&
                     ((k == RECIPE_EARLY_BRANCH) ||
                      (e.rel_branch_kind == REL_BRANCH_CALL) ||
                      (!r.jcc && !r.br_rel && (e.rel_branch_kind == REL_BRANCH_NONE) &&
                       ((k == RECIPE_EARLY_NONE) || (k == RECIPE_EARLY_EA) ||
                        (k == RECIPE_EARLY_LOAD) || (k == RECIPE_EARLY_STORE) ||
                        (k == RECIPE_EARLY_RMW) || (k == RECIPE_EARLY_STACK))));
    end
endfunction

reg [63:0] entry_rom [0:1023];
reg [15:0] group_entry_rom [0:1023];
integer fo;
dec_entry_t e;
recipe_meta_t r;
logic [11:0] ctl;
logic [6:0] grp;
logic [15:0] first, final_e;
logic has_modrm;
initial begin
    $readmemh("pla_entry_base.hex", entry_rom);
    $readmemh("pla_group_base.hex", group_entry_rom);
    fo = $fopen("cls.txt", "w");
    for (int f = 0; f < 2; f++)
    for (int op = 0; op < 256; op++)
    for (int d = 0; d < 2; d++)
    for (int pe = 0; pe < 2; pe++)
    for (int m = 0; m < 256; m++) begin
        // As decoder.sv build_struct_work, without prefixes.
        ctl = pla_control_opcode_lookup(f[0], op[7:0]);
        has_modrm = ctl[2] & ~ctl[0];
        first = entry_rom[{op[7:0], 1'b0, f[0]}][{d[0], pe[0]}*16 +: 16];
        grp = pla_group_lookup({d[0], op[7:0], pe[0], f[0]});
        final_e = (grp[6] && has_modrm)
            ? group_entry_rom[{grp[5:0], m[5:3], (m[7:6] != 2'b11)}] : first;
        e = '0;
        e.has_0f = f[0]; e.opcode = op[7:0]; e.modrm = m[7:0];
        e.has_modrm = has_modrm; e.data32 = d[0];
        e.rep_lock = PREFIX_NOREPLOCK;
        if ((!f && op[7:4] == 4'h7) || (f && op[7:4] == 4'h8))
            e.rel_branch_kind = REL_BRANCH_JCC;
        else if (!f && ((op == 8'hEB) || (op == 8'hE9)))
            e.rel_branch_kind = REL_BRANCH_JMP;
        else if (!f && (op == 8'hE8))
            e.rel_branch_kind = REL_BRANCH_CALL;
        e.entry_point = final_e[11:0];
        if (f && (op[7:3] == 5'b11001)) e.entry_point = UADDR_BSWAP;
        else if (f && (op[7:1] == 7'b1100000))
            e.entry_point = (m[7:6] == 2'b11) ? UADDR_XADD_R : UADDR_XADD_M;
        else if (f && (op[7:1] == 7'b1011000))
            e.entry_point = (m[7:6] == 2'b11) ? UADDR_CMPXCHG_R : UADDR_CMPXCHG_M;
        else if (f && (op[7:1] == 7'b0000100)) e.entry_point = UADDR_INVD;
        e.entry_point = recipe_effective_entry(e.entry_point, e.opcode, e.modrm);
        r = recipe_metadata(e);
        $fwrite(fo, "%0d %0d %0d %0d %0d %0d%0d%0d%0d\n", f, op, d, pe, m,
                pb_b1_type(e, r), r.reads_flags, r.uses_ea, r.jcc);
    end
    $fclose(fo);
    $finish;
end
endmodule
"""


def evaluate(work: Path) -> list[str]:
    (work / "tb.sv").write_text(TB)
    for name in ("pla_entry_base.hex", "pla_group_base.hex"):
        shutil.copy(CORE / name, work / name)
    subprocess.run(
        ["verilator", "--binary", "-Wno-fatal", "-Wno-lint", "-Wno-style",
         f"-I{CORE}", str(CORE / "z486_pkg.sv"), "tb.sv",
         "--top-module", "tb_pb_b1_class", "-o", "tb"],
        cwd=work, check=True, stdout=subprocess.DEVNULL)
    subprocess.run(["./obj_dir/tb"], cwd=work, check=True,
                   stdout=subprocess.DEVNULL)
    return (work / "cls.txt").read_text().split("\n")


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        lines = evaluate(Path(tmp))

    # table[(f, op)][(reg, mem)] = class; prove mode and r/m independence.
    table: dict = collections.defaultdict(dict)
    for line in lines:
        if not line:
            continue
        f, op, d, pe, m, cls = line.split()
        f, op, m = int(f), int(op), int(m)
        key = ((m >> 3) & 7, int((m >> 6) != 3))
        cls = int(cls, 2)
        # Only the type bit gates the others; drop don't-cares.
        if not cls >> 3:
            cls = 0
        old = table[(f, op)].setdefault(key, cls)
        if old != cls:
            sys.exit(f"class depends on operand size, PE or ModR/M.rm: "
                     f"0f={f} op={op:02x} modrm={m:02x}")

    rows = []
    for (f, op), cases in sorted(table.items()):
        vals = set(cases.values())
        if vals == {0}:
            continue
        head = f"{f}_{op:08b}"
        if len(vals) == 1:
            rows.append((f"{head}_???_?", vals.pop()))
            continue
        by_mem = [{cases[(reg, mem)] for reg in range(8)} for mem in (0, 1)]
        if all(len(s) == 1 for s in by_mem):
            for mem in (0, 1):
                if by_mem[mem] != {0}:
                    rows.append((f"{head}_???_{mem}", by_mem[mem].pop()))
            continue
        for reg in range(8):
            for mem in (0, 1):
                if cases[(reg, mem)]:
                    rows.append((f"{head}_{reg:03b}_{mem}", cases[(reg, mem)]))

    out = [
        "// Generated by scripts/gen_pb_b1_class.py; do not edit.",
        "// Port B dead-slot class of an instruction without REP/LOCK, from",
        "// {0F, opcode, ModR/M.reg, ModR/M is memory}:",
        "// {B1 type, reads flags, uses EA, Jcc}.",
        "function automatic logic [3:0] pb_b1_class(input logic has_0f,",
        "                                           input logic [7:0] opcode,",
        "                                           input logic [7:0] modrm);",
        "    casez ({has_0f, opcode, modrm[5:3], modrm[7:6] != 2'b11})",
    ]
    for pat, val in rows:
        out.append(f"        13'b{pat}: pb_b1_class = 4'b{val:04b};")
    out += [
        "        default: pb_b1_class = 4'b0000;",
        "    endcase",
        "endfunction",
        "",
    ]
    OUT.write_text("\n".join(out))
    print(f"Wrote {OUT.name}: {len(rows)} rows")
    return 0


if __name__ == "__main__":
    sys.exit(main())
