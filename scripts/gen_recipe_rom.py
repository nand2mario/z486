#!/usr/bin/env python3
"""Generate the decode ROMs' effective entries and recipe IDs.

The D1 decode ROMs (pla_entry_rom.hex, first level, and pla_group_entry.hex,
{16-bit protected-mode group row, ModR/M.reg, memory}) are M10K images read one
cycle ahead of D1, with one lane per {data32, PE}.  Beside each 16-bit 80386
PLA entry, a lane carries

  * the effective microcode entry: optimizer overlays (recipe_effective_entry)
    are resolved here rather than by D1 logic; and
  * a recipe ID, which recipe_id_decode() (recipe_rom.svh) expands into the
    instruction's hardwired-recipe metadata, early kind and entry action.

ucode_recipes.svh remains the specification.  This script evaluates it for
every decode input with the decoder's own lookups (a Verilator testbench
including z486_pkg.sv, pla_control.svh, pla_entry.svh and the base ROM
images), then proves that each ROM address determines one entry and one ID.
What a ROM address cannot know is applied in D1:

  * a REP or LOCK prefix clears the recipe metadata (as recipe_metadata);
  * an invalid LOCK clears the early kind and the entry action (entry 82B);
  * ADC/SBB's carry input (reads_carry and the decoded ALU operation);
  * the byte-operand bit, which comes from the opcode.

A simulation check in decoder.sv compares the result with the specification
for every instruction leaving D2.  Rerun after changing the entry PLA, the
group ROM (gen_pla_group.py) or the recipe inventory (ucode_optimize.py):

    ./gen_recipe_rom.py            # writes ../pla_entry_rom.hex,
                                   # ../pla_group_entry.hex, ../recipe_rom.svh
    ./gen_recipe_rom.py --check    # fail if they are stale
"""

from __future__ import annotations

import argparse
import collections
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
CORE = HERE.parent
ENTRY_BASE = CORE / "pla_entry_base.hex"
GROUP_BASE = CORE / "pla_group_base.hex"
ENTRY_ROM = CORE / "pla_entry_rom.hex"
GROUP_ROM = CORE / "pla_group_entry.hex"
SVH = CORE / "recipe_rom.svh"

ID_BITS = 6
SLOT_BITS = 16 + ID_BITS          # {recipe ID, 80386 PLA bits 15:12, entry}

# recipe_meta_t, most significant field first (z486_pkg.sv).
META_FIELDS = [
    ("hardwired", 1), ("multi_ustep", 1), ("commit_sel", 3),
    ("slot_has_work", 1), ("reads_flags", 1), ("uses_ea", 1),
    ("reads_dst", 1), ("reads_src", 1), ("reads_ecx", 1),
    ("writes_srcreg", 1), ("op_byte", 1), ("jcc", 1), ("writes_flags", 1),
    ("br_rel", 1),
]
META_BITS = sum(width for _, width in META_FIELDS)


def meta_shift(name: str) -> int:
    shift = META_BITS
    for field, width in META_FIELDS:
        shift -= width
        if field == name:
            return shift
    raise KeyError(name)


OP_BYTE = 1 << meta_shift("op_byte")
READS_FLAGS = 1 << meta_shift("reads_flags")

TB = r"""
module tb_recipe_rom;
import z486_pkg::*;
`include "pla_control.svh"
`include "pla_entry.svh"

reg [63:0] entry_rom [0:1023];
reg [15:0] group_rom [0:1023];
integer fo;
dec_entry_t e;
recipe_meta_t r;
logic [11:0] ctl, arch, eff;
logic [6:0] grp, grp0;
logic [15:0] first, final_e;
logic has_modrm, is_group, ovr486, carry;
logic [9:0] addr, addr0;
logic [2:0] alu;
initial begin
    $readmemh("pla_entry_base.hex", entry_rom);
    $readmemh("pla_group_base.hex", group_rom);
    fo = $fopen("cases.txt", "w");
    for (int f = 0; f < 2; f++)
    for (int op = 0; op < 256; op++)
    for (int rep = 0; rep < 2; rep++)
    for (int d = 0; d < 2; d++)
    for (int pe = 0; pe < 2; pe++)
    for (int m = 0; m < 256; m++) begin
        // As decoder.sv build_struct_work.
        // D1 consumes prefixes and the 0F escape; their slots are unused.
        if (!f && ((op == 8'h0F) || (op == 8'h26) || (op == 8'h2E) ||
                   (op == 8'h36) || (op == 8'h3E) || (op[7:2] == 6'b011001) ||
                   (op == 8'hF0) || (op == 8'hF2) || (op == 8'hF3))) continue;
        ctl = pla_control_opcode_lookup(f[0], op[7:0]);
        has_modrm = ctl[2] & ~ctl[0];
        if (!has_modrm && m != 0) continue;
        first = entry_rom[{op[7:0], rep[0], f[0]}][{d[0], pe[0]}*16 +: 16];
        grp = pla_group_lookup({d[0], op[7:0], pe[0], f[0]});
        // The group ROM is preread with the 16-bit protected-mode row (as decoder.sv).
        grp0 = pla_group_lookup({1'b0, op[7:0], 1'b1, f[0]});
        is_group = grp[6] && has_modrm;
        addr0 = {grp0[5:0], m[5:3], (m[7:6] != 2'b11)};
        addr = is_group ? {grp[5:0], m[5:3], (m[7:6] != 2'b11)}
                        : {op[7:0], rep[0], f[0]};
        final_e = is_group ? group_rom[addr] : first;
        arch = final_e[11:0];
        ovr486 = 1'b1;
        if (f && (op[7:3] == 5'b11001)) arch = UADDR_BSWAP;
        else if (f && (op[7:1] == 7'b1100000))
            arch = (m[7:6] == 2'b11) ? UADDR_XADD_R : UADDR_XADD_M;
        else if (f && (op[7:1] == 7'b1011000))
            arch = (m[7:6] == 2'b11) ? UADDR_CMPXCHG_R : UADDR_CMPXCHG_M;
        else if (f && (op[7:1] == 7'b0000100)) arch = UADDR_INVD;
        else ovr486 = 1'b0;
        e = '0;
        e.has_0f = f[0]; e.opcode = op[7:0]; e.data32 = d[0];
        e.has_modrm = has_modrm; e.modrm = has_modrm ? m[7:0] : 8'h00;
        // Prefix-independent ROM contents; D1 clears the metadata for REP.
        e.rep_lock = PREFIX_NOREPLOCK;
        eff = recipe_effective_entry(arch, e.opcode, e.modrm);
        e.entry_point = eff;
        r = recipe_metadata(e);
        // As decode_instruction_alu_op for the ALU opcodes: ADC or SBB.
        alu = (op[7] && has_modrm) ? m[5:3] : op[5:3];
        carry = (alu == 3'd2) || (alu == 3'd3);
        if (is_group && !grp0[6])
            $fatal(1, "group opcode %02x (0f=%0d) is not a group in the preread mode", op, f);
        $fwrite(fo, "%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d\n",
                is_group, is_group ? addr0 : addr, d, pe, f, op, m, rep, final_e,
                eff, ovr486, r, recipe_early_kind(eff), recipe_action(eff), carry);
    end
    $fclose(fo);
    $finish;
end
endmodule
"""


def evaluate(work: Path) -> list[str]:
    (work / "tb.sv").write_text(TB)
    # The package includes this script's own output; evaluate without it.
    (work / SVH.name).write_text(
        "function automatic recipe_rom_t recipe_id_decode(input logic [5:0] id);\n"
        "    recipe_id_decode = '0;\nendfunction\n")
    for path in (ENTRY_BASE, GROUP_BASE):
        shutil.copy(path, work / path.name)
    subprocess.run(
        ["verilator", "--binary", "-Wno-fatal", "-Wno-lint", "-Wno-style",
         f"-I{work}", f"-I{CORE}", str(CORE / "z486_pkg.sv"), "tb.sv",
         "--top-module", "tb_recipe_rom", "-o", "tb"],
        cwd=work, check=True, stdout=subprocess.DEVNULL)
    subprocess.run(["./obj_dir/tb"], cwd=work, check=True,
                   stdout=subprocess.DEVNULL)
    return (work / "cases.txt").read_text().splitlines()


def build(lines: list[str]):
    cases = []
    for line in lines:
        (is_group, addr, d, pe, f, op, m, rep, final_e, eff, ovr486, meta,
         early, action, carry) = map(int, line.split())
        cases.append(dict(is_group=is_group, addr=addr, d=d, pe=pe, f=f,
                          op=op, m=m, rep=rep, final=final_e, eff=eff,
                          ovr486=ovr486, meta=meta, early=early,
                          action=action, carry=carry))

    # ADC/SBB entries: reads_flags varies with the operation and equals the
    # carry input; D1 rebuilds it from reads_carry and the decoded ALU op.
    # Instructions without a recipe (all-zero metadata) never read flags.
    def has_recipe(c):
        return bool(c["meta"] & ~OP_BYTE)

    flags_by_entry = collections.defaultdict(set)
    for c in cases:
        if has_recipe(c):
            flags_by_entry[c["eff"]].add(bool(c["meta"] & READS_FLAGS))
    carry_entries = {entry for entry, values in flags_by_entry.items()
                     if len(values) > 1}
    for c in cases:
        if c["eff"] in carry_entries and has_recipe(c):
            if bool(c["meta"] & READS_FLAGS) != bool(c["carry"]):
                sys.exit(f"entry {c['eff']:03x}: reads_flags is not the ADC/SBB "
                         f"carry input (0f={c['f']} op={c['op']:02x} modrm={c['m']:02x})")

    def recipe_of(c):
        meta = c["meta"] & ~OP_BYTE
        reads_carry = int(c["eff"] in carry_entries and has_recipe(c))
        if reads_carry:
            meta &= ~READS_FLAGS
        return (meta, reads_carry, c["early"], c["action"])

    zero = (0, 0, 0, 0)
    slots: dict[tuple, set] = collections.defaultdict(set)
    base_hi: dict[tuple, int] = {}
    for c in cases:
        key = ("g" if c["is_group"] else "f", c["addr"], c["d"], c["pe"])
        recipe = recipe_of(c)
        if c["ovr486"]:
            # D1 substitutes the 486 entry; the ROM keeps the PLA entry.
            if recipe != zero:
                sys.exit(f"486 extension 0f {c['op']:02x} has recipe metadata")
            value = (c["final"] & 0xFFF, recipe)
        else:
            value = (c["eff"], recipe)
        slots[key].add(value)
        base_hi.setdefault(key, c["final"] >> 12)

    conflicts = {key: values for key, values in slots.items() if len(values) > 1}
    if conflicts:
        for key, values in sorted(conflicts.items())[:20]:
            print(f"conflict at {key}: {sorted(values)}", file=sys.stderr)
        sys.exit(f"{len(conflicts)} decode-ROM addresses need more than one "
                 "entry or recipe; split their group rows (gen_pla_group.py)")

    recipes = sorted({value[1] for values in slots.values() for value in values}
                     - {zero})
    ids = {zero: 0}
    for recipe in recipes:
        ids[recipe] = len(ids)
    if len(ids) > (1 << ID_BITS):
        sys.exit(f"{len(ids)} recipes exceed the {ID_BITS}-bit recipe ID")
    users = collections.defaultdict(set)
    for values in slots.values():
        (entry, recipe), = values
        users[ids[recipe]].add(entry)
    return slots, base_hi, ids, users


def render_entry_rom(slots, base_hi, ids) -> str:
    base = [int(line, 16) for line in ENTRY_BASE.read_text().split()]
    words = []
    for addr, word in enumerate(base):
        out = 0
        for d in (0, 1):
            for pe in (0, 1):
                lane = (d << 1) | pe
                old = (word >> (16 * lane)) & 0xFFFF
                key = ("f", addr, d, pe)
                if key in slots:
                    (entry, recipe), = slots[key]
                    value = (ids[recipe] << 16) | (base_hi[key] << 12) | entry
                else:
                    value = old
                out |= value << (SLOT_BITS * lane)
        words.append(out)
    digits = (4 * SLOT_BITS + 3) // 4
    return "".join(f"{word:0{digits}x}\n" for word in words)


def render_group_rom(slots, base_hi, ids) -> str:
    """{mode-0 group row, ModR/M.reg, memory} -> four {data32, PE} lanes."""
    words = []
    for addr in range(1024):
        out = 0
        for d in (0, 1):
            for pe in (0, 1):
                key = ("g", addr, d, pe)
                if key in slots:
                    (entry, recipe), = slots[key]
                    value = (ids[recipe] << 16) | (base_hi[key] << 12) | entry
                    out |= value << (SLOT_BITS * ((d << 1) | pe))
        words.append(out)
    digits = (4 * SLOT_BITS + 3) // 4
    return "".join(f"{word:0{digits}x}\n" for word in words)


def render_svh(ids, users) -> str:
    out = [
        "// Generated by scripts/gen_recipe_rom.py; do not edit.",
        "// Recipe ID (decode ROM bits 21:16) -> hardwired-recipe metadata, early",
        "// kind and entry action. The decoder applies the prefix, invalid-LOCK,",
        "// ADC/SBB carry and byte-operand qualifiers (see gen_recipe_rom.py).",
        f"localparam int RECIPE_ID_BITS = {ID_BITS};",
        "",
        "function automatic recipe_rom_t recipe_id_decode(",
        f"    input logic [{ID_BITS - 1}:0] id",
        ");",
        "    recipe_rom_t r;",
        "    r = '0;",
        "    unique case (id)",
    ]
    for recipe, index in sorted(ids.items(), key=lambda item: item[1]):
        if index == 0:
            continue
        meta, reads_carry, early, action = recipe
        sets = []
        for name, width in META_FIELDS:
            value = (meta >> meta_shift(name)) & ((1 << width) - 1)
            if value:
                sets.append(f"r.meta.{name} = {width}'d{value};")
        if reads_carry:
            sets.append("r.reads_carry = 1'b1;")
        if early:
            sets.append(f"r.early = 3'd{early};")
        if action:
            sets.append(f"r.action = 2'd{action};")
        entries = " ".join(f"{entry:03X}" for entry in sorted(users[index]))
        out.append(f"        {ID_BITS}'d{index}: begin  // {entries}")
        out += [f"            {statement}" for statement in sets]
        out.append("        end")
    out += [
        "        default: ;",
        "    endcase",
        "    recipe_id_decode = r;",
        "endfunction",
        "",
    ]
    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true",
                        help="fail if the generated files are stale")
    args = parser.parse_args()

    with tempfile.TemporaryDirectory() as tmp:
        lines = evaluate(Path(tmp))
    slots, base_hi, ids, users = build(lines)
    outputs = {
        ENTRY_ROM: render_entry_rom(slots, base_hi, ids),
        GROUP_ROM: render_group_rom(slots, base_hi, ids),
        SVH: render_svh(ids, users),
    }
    if args.check:
        stale = [path.name for path, text in outputs.items()
                 if not path.exists() or path.read_text() != text]
        if stale:
            raise SystemExit(f"stale: {', '.join(stale)}; rerun {Path(__file__).name}")
        print("decode ROMs and recipe IDs are up to date")
        return 0
    for path, text in outputs.items():
        path.write_text(text)
    print(f"{len(slots)} decode-ROM addresses, {len(ids) - 1} recipe IDs "
          f"(of {(1 << ID_BITS) - 1}); wrote {ENTRY_ROM.name}, {GROUP_ROM.name}, "
          f"{SVH.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
