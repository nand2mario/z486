#!/usr/bin/env python3
"""Assemble the x87 control store and review artifacts."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


FLOW = {
    "NEXT": 0,
    "JUMP": 1,
    "BRANCH": 2,
    "LOOP": 3,
    "WAIT": 4,
    "FINISH": 5,
}

COND = {
    "FALSE": 0,
    "TRUE": 1,
    "SPECIAL": 2,
    "INTEGRAL": 3,
    "SUBUNIT": 4,
    "COUNT_MORE": 5,
    "TRANSFER_READY": 6,
    "ZERO": 7,
    "SHIFT_LEFT": 8,
    "COUNT_ZERO": 9,
    "INVALID": 10,
    "DIRECT_READY": 11,
    "NORMALIZE_MORE": 12,
    "SHIFT_RIGHT": 13,
    "SHIFT_RIGHT_MORE": 14,
    "ADDSUB_NORMALIZE_MORE": 15,
    "CORDIC_LIMB_MORE": 16,
    "CORDIC_LOAD_MORE": 17,
    "CORDIC_ALIGN_MORE": 18,
    "ARITH_DIRECT": 19,
    # Operation-local aliases preserve the existing condition encoding. These values
    # are never tested by the same microprogram as their generic names.
    "TRANS_NEEDS_AUX": 3,
    "TRANS_ATAN2": 8,
}

SRC = {
    "ZERO": 0,
    "OPERAND_A": 1,
    "OPERAND_B": 2,
    "WORK": 3,
    "RESULT": 4,
    "TRANSFER": 5,
    "ONE": 6,
    "FORMAT": 7,
}

STATE = {
    "HOLD": 0,
    "RESTORE_COUNT": 1,
    "FORMAT_EXP_DEC": 2,
    "FORMAT_EXP_INC": 3,
    "TRANS_SELECT": 4,
    "TRANS_NORMALIZE": 5,
    "TRANS_LOAD_AUX": 6,
}

PREPARE = {
    "HOLD": 0,
    "LOAD_ROUNDINT": 1,
    "LOAD_FIST": 2,
    "FST": 3,
    "FLD": 4,
    "FILD": 5,
    "ADDSUB": 6,
    "MUL": 7,
    "DIV": 8,
    "SQRT": 9,
}

CLASSIFY = {
    "HOLD": 0,
    "ROUNDINT": 1,
    "FIST": 2,
    "FIST_RANGE": 3,
    "ADDSUB": 4,
    "MUL": 5,
    "DIVSQRT": 6,
    "TRANS": 7,
}

ENGINE = {
    "HOLD": 0,
    "MUL_ISSUE": 1,
    "MUL_ACCUMULATE": 2,
    "DIV_ITERATE": 3,
    "SQRT_ITERATE": 4,
    "TRANS_RANGE_ITERATE": 5,
    "TRANS_RANGE_FINALIZE": 6,
    "TRANS_CORDIC_PREP": 8,
    "CORDIC_ALIGN_PREP": 9,
    "ADDSUB_ALIGN": 10,
    "CORDIC_X_PREP": 11,
    "CORDIC_Y_PREP": 12,
    "CORDIC_Z_PREP": 13,
    "CORDIC_NEXT": 14,
    "CORDIC_OUTPUT_PREP": 15,
    "CORDIC_AUX_PREP": 16,
    "CORDIC_BEGIN": 17,
    "CORDIC_OUTPUT_CAPTURE": 18,
}

PACK = {
    "HOLD": 0,
    "PACK_INTERNAL": 1,
    "COPY_A": 2,
    "ROUNDINT_SPECIAL": 3,
    "ROUNDINT_SUBUNIT": 4,
    "PACK_FIST": 5,
    "FIST_INVALID": 6,
    "ROUND_PACK_FST": 7,
    "PACK_FST_SPECIAL": 8,
    "PACK_FLD_DENORMAL": 9,
    "PACK_FILD": 10,
    "PACK_ADDSUB": 11,
    "PACK_MUL": 12,
    "PACK_DIVSQRT": 13,
    "PACK_TRANS": 14,
}

ALU_ROUTE = {
    "HOLD": 0,
    "ROUND": 1,
    "CALCULATE_ADDSUB": 2,
    "NORMALIZE_ADDSUB": 3,
    "PREP_ROUND_ADDSUB": 4,
    "PREP_ROUND_MUL": 5,
    "PREP_ROUND_DIVSQRT": 6,
    "PREP_ROUND_TRANS": 7,
}

SHIFT_ROUTE = {
    "HOLD": 0,
    "LEFT": 1,
    "RIGHT": 2,
    "FORMAT": 3,
}
COUNT = {"HOLD": 0, "LOAD": 1, "INC": 2, "DEC": 3}
GRS = {"HOLD": 0, "LOAD": 1, "SHIFT": 2, "CLEAR": 3}
COMMIT = {
    "NONE": 0,
    "REPLACE_ST0": 1,
    "PUSH": 2,
    "POP": 3,
    "TRANSFER": 4,
}
SIDE = {"NONE": 0, "SET_STATUS": 1, "QUIET_NAN": 2, "SET_INEXACT": 3}

SCRATCH_READ = {
    "HOLD": 0,
    "ALIGN": 1,
    "X_LOW": 2,
    "X_HIGH": 3,
    "Y_LOW": 4,
    "Y_HIGH": 5,
    "Z": 6,
    "OUTPUT": 7,
}

SCRATCH_WRITE = {
    "HOLD": 0,
    "LOAD": 1,
    "ALIGN": 2,
    "X": 3,
    "Y": 4,
    "Z": 5,
    "PRIMARY": 6,
    "AUX": 7,
}


@dataclass(frozen=True)
class UOp:
    flow: str = "NEXT"
    target: str | int = 0
    condition: str = "FALSE"
    src_a: str = "ZERO"
    src_b: str = "ZERO"
    state: str = "HOLD"
    prepare: str = "HOLD"
    classify: str = "HOLD"
    engine: str = "HOLD"
    pack: str = "HOLD"
    alu_route: str = "HOLD"
    destination: str = "NONE"  # Transitional generator annotation only.
    shift_route: str = "HOLD"
    count: str = "HOLD"
    grs: str = "HOLD"
    commit: str = "NONE"
    flags: int = 0
    scratch_read: str = "HOLD"
    scratch_write: str = "HOLD"
    side_effect: str = "NONE"  # Transitional generator annotation only.


ProgramItem = tuple[str | None, UOp]


PROGRAM: list[ProgramItem] = [
    ("entry_frndint", UOp(classify="ROUNDINT")),
    (None, UOp(flow="BRANCH", target="frnd_special", condition="SPECIAL")),
    (None, UOp(flow="BRANCH", target="frnd_copy", condition="INTEGRAL")),
    (None, UOp(flow="BRANCH", target="frnd_subunit", condition="SUBUNIT")),
    (None, UOp(prepare="LOAD_ROUNDINT", destination="WORK", count="LOAD", grs="LOAD")),
    ("frnd_shift", UOp(flow="LOOP", target="frnd_shift", condition="COUNT_MORE",
                       src_a="WORK", destination="WORK",
                       shift_route="RIGHT", count="DEC", grs="SHIFT")),
    (None, UOp(src_a="WORK", alu_route="ROUND", destination="WORK")),
    (None, UOp(state="RESTORE_COUNT", count="LOAD")),
    ("frnd_restore", UOp(flow="LOOP", target="frnd_restore", condition="COUNT_MORE",
                         src_a="WORK", destination="WORK",
                         shift_route="LEFT", count="DEC")),
    (None, UOp(src_a="WORK", pack="PACK_INTERNAL", destination="RESULT")),
    (None, UOp(flow="JUMP", target="frnd_commit")),
    ("frnd_copy", UOp(src_a="OPERAND_A", pack="COPY_A", destination="RESULT")),
    (None, UOp(flow="JUMP", target="frnd_commit")),
    ("frnd_special", UOp(src_a="OPERAND_A", pack="ROUNDINT_SPECIAL",
                         destination="RESULT", side_effect="SET_STATUS")),
    (None, UOp(flow="JUMP", target="frnd_commit")),
    ("frnd_subunit", UOp(src_a="OPERAND_A", pack="ROUNDINT_SUBUNIT",
                         destination="RESULT", side_effect="SET_INEXACT")),
    ("frnd_commit", UOp(flow="FINISH", src_a="RESULT", commit="REPLACE_ST0")),

    ("entry_fld_m32", UOp(prepare="FLD", destination="WORK")),
    (None, UOp(flow="BRANCH", target="fld_commit", condition="DIRECT_READY")),
    ("fld32_normalize", UOp(flow="LOOP", target="fld32_normalize",
                             condition="NORMALIZE_MORE", src_a="WORK",
                             state="FORMAT_EXP_DEC", destination="WORK",
                             shift_route="LEFT")),
    (None, UOp(flow="JUMP", target="fld_pack_denormal")),
    ("entry_fld_m64", UOp(prepare="FLD", destination="WORK")),
    (None, UOp(flow="BRANCH", target="fld_commit", condition="DIRECT_READY")),
    ("fld64_normalize", UOp(flow="LOOP", target="fld64_normalize",
                             condition="NORMALIZE_MORE", src_a="WORK",
                             state="FORMAT_EXP_DEC", destination="WORK",
                             shift_route="LEFT")),
    ("fld_pack_denormal", UOp(src_a="WORK", pack="PACK_FLD_DENORMAL",
                               destination="RESULT")),
    ("fld_commit", UOp(flow="FINISH", src_a="RESULT", commit="PUSH")),
    ("entry_fild", UOp(prepare="FILD", destination="WORK")),
    (None, UOp(flow="BRANCH", target="fld_commit", condition="DIRECT_READY")),
    (None, UOp(flow="BRANCH", target="fild_shift_right", condition="SHIFT_RIGHT")),
    (None, UOp(flow="BRANCH", target="fild_shift_left", condition="SHIFT_LEFT")),
    (None, UOp(flow="JUMP", target="fild_pack")),
    ("fild_shift_right", UOp(flow="LOOP", target="fild_shift_right",
                              condition="SHIFT_RIGHT_MORE", src_a="WORK",
                              state="FORMAT_EXP_INC", destination="WORK",
                              shift_route="RIGHT", grs="SHIFT")),
    (None, UOp(flow="JUMP", target="fild_pack")),
    ("fild_shift_left", UOp(flow="LOOP", target="fild_shift_left",
                             condition="NORMALIZE_MORE", src_a="WORK",
                             state="FORMAT_EXP_DEC", destination="WORK",
                             shift_route="LEFT")),
    ("fild_pack", UOp(src_a="WORK", pack="PACK_FILD",
                       destination="RESULT")),
    (None, UOp(flow="JUMP", target="fld_commit")),
    ("entry_fst_m32", UOp(src_a="OPERAND_A", prepare="FST",
                           destination="WORK", count="LOAD", grs="LOAD")),
    (None, UOp(flow="BRANCH", target="fst_special", condition="SPECIAL")),
    (None, UOp(flow="BRANCH", target="fst_round_pack", condition="COUNT_ZERO")),
    ("fst32_shift", UOp(flow="LOOP", target="fst32_shift", condition="COUNT_MORE",
                         src_a="WORK", destination="WORK",
                         shift_route="RIGHT", count="DEC", grs="SHIFT")),
    (None, UOp(flow="JUMP", target="fst_round_pack")),
    ("entry_fst_m64", UOp(src_a="OPERAND_A", prepare="FST",
                           destination="WORK", count="LOAD", grs="LOAD")),
    (None, UOp(flow="BRANCH", target="fst_special", condition="SPECIAL")),
    (None, UOp(flow="BRANCH", target="fst_round_pack", condition="COUNT_ZERO")),
    ("fst64_shift", UOp(flow="LOOP", target="fst64_shift", condition="COUNT_MORE",
                         src_a="WORK", destination="WORK",
                         shift_route="RIGHT", count="DEC", grs="SHIFT")),
    ("fst_round_pack", UOp(src_a="WORK", pack="ROUND_PACK_FST",
                            destination="TRANSFER")),
    ("fst_commit", UOp(flow="FINISH", src_a="TRANSFER", commit="TRANSFER")),
    ("fst_special", UOp(src_a="OPERAND_A", pack="PACK_FST_SPECIAL",
                         destination="TRANSFER")),
    (None, UOp(flow="JUMP", target="fst_commit")),
    ("entry_fist", UOp(src_a="OPERAND_A", classify="FIST")),
    (None, UOp(flow="BRANCH", target="fist_invalid", condition="INVALID")),
    (None, UOp(flow="BRANCH", target="fist_check", condition="ZERO")),
    (None, UOp(src_a="OPERAND_A", prepare="LOAD_FIST", destination="WORK",
               count="LOAD", grs="LOAD")),
    (None, UOp(flow="BRANCH", target="fist_round", condition="COUNT_ZERO")),
    (None, UOp(flow="BRANCH", target="fist_shift_left", condition="SHIFT_LEFT")),
    ("fist_shift_right", UOp(flow="LOOP", target="fist_shift_right",
                              condition="COUNT_MORE", src_a="WORK",
                              destination="WORK",
                              shift_route="RIGHT", count="DEC", grs="SHIFT")),
    (None, UOp(flow="JUMP", target="fist_round")),
    ("fist_shift_left", UOp(flow="LOOP", target="fist_shift_left",
                             condition="COUNT_MORE", src_a="WORK",
                             destination="WORK",
                             shift_route="LEFT", count="DEC")),
    ("fist_round", UOp(src_a="WORK", alu_route="ROUND", destination="WORK")),
    ("fist_check", UOp(src_a="WORK", classify="FIST_RANGE")),
    (None, UOp(flow="BRANCH", target="fist_invalid", condition="INVALID")),
    (None, UOp(src_a="WORK", pack="PACK_FIST", destination="TRANSFER")),
    ("fist_commit", UOp(flow="FINISH", src_a="TRANSFER", commit="TRANSFER")),
    ("fist_invalid", UOp(pack="FIST_INVALID", destination="TRANSFER",
                          side_effect="SET_STATUS")),
    (None, UOp(flow="JUMP", target="fist_commit")),

    ("entry_addsub", UOp(flow="BRANCH", target="addsub_commit",
                           condition="ARITH_DIRECT",
                           src_a="OPERAND_A", src_b="OPERAND_B",
                           classify="ADDSUB", prepare="ADDSUB",
                           destination="WORK", count="LOAD")),
    ("addsub_align", UOp(flow="LOOP", target="addsub_align",
                          condition="COUNT_MORE", src_a="OPERAND_B",
                          engine="ADDSUB_ALIGN", count="DEC", grs="SHIFT")),
    ("addsub_calculate", UOp(src_a="WORK", src_b="OPERAND_B",
                              alu_route="CALCULATE_ADDSUB",
                              destination="WORK")),
    ("addsub_normalize", UOp(flow="LOOP", target="addsub_normalize",
                              condition="ADDSUB_NORMALIZE_MORE", src_a="WORK",
                              alu_route="NORMALIZE_ADDSUB",
                              destination="WORK")),
    (None, UOp(flow="BRANCH", target="addsub_commit",
               condition="DIRECT_READY")),
    (None, UOp(src_a="WORK", alu_route="PREP_ROUND_ADDSUB",
               destination="WORK", grs="LOAD")),
    (None, UOp(src_a="WORK", pack="PACK_ADDSUB", destination="RESULT")),
    ("addsub_commit", UOp(flow="FINISH", src_a="RESULT",
                           commit="REPLACE_ST0")),

    ("entry_compare", UOp(src_a="OPERAND_A", src_b="OPERAND_B",
                           classify="ADDSUB",
                           side_effect="SET_STATUS")),
    (None, UOp(flow="FINISH", side_effect="SET_STATUS")),

    ("entry_mul", UOp(flow="BRANCH", target="mul_commit",
                       condition="ARITH_DIRECT",
                       src_a="OPERAND_A", src_b="OPERAND_B",
                       classify="MUL", prepare="MUL")),
    (None, UOp(engine="MUL_ACCUMULATE")),
    (None, UOp(alu_route="PREP_ROUND_MUL", destination="WORK",
               grs="LOAD")),
    (None, UOp(src_a="WORK", pack="PACK_MUL",
               destination="RESULT")),
    ("mul_commit", UOp(flow="FINISH", src_a="RESULT",
                        commit="REPLACE_ST0")),

    ("entry_div", UOp(src_a="OPERAND_A", src_b="OPERAND_B",
                       classify="DIVSQRT")),
    (None, UOp(flow="BRANCH", target="divsqrt_commit",
               condition="DIRECT_READY")),
    (None, UOp(src_a="OPERAND_A", src_b="OPERAND_B",
               prepare="DIV", count="LOAD")),
    ("div_iterate", UOp(flow="LOOP", target="div_iterate",
                         condition="COUNT_MORE", engine="DIV_ITERATE",
                         count="DEC")),
    (None, UOp(alu_route="PREP_ROUND_DIVSQRT", destination="WORK",
               grs="LOAD")),
    (None, UOp(src_a="WORK", alu_route="ROUND",
               destination="WORK")),
    (None, UOp(src_a="WORK", pack="PACK_DIVSQRT",
               destination="RESULT")),
    ("divsqrt_commit", UOp(flow="FINISH", src_a="RESULT",
                            commit="REPLACE_ST0")),

    ("entry_sqrt", UOp(src_a="OPERAND_A",
                        classify="DIVSQRT")),
    (None, UOp(flow="BRANCH", target="divsqrt_commit",
               condition="DIRECT_READY")),
    (None, UOp(src_a="OPERAND_A", prepare="SQRT", count="LOAD")),
    ("sqrt_iterate", UOp(flow="LOOP", target="sqrt_iterate",
                          condition="COUNT_MORE", engine="SQRT_ITERATE",
                          count="DEC")),
    (None, UOp(alu_route="PREP_ROUND_DIVSQRT", destination="WORK",
               grs="LOAD")),
    (None, UOp(src_a="WORK", alu_route="ROUND",
               destination="WORK")),
    (None, UOp(src_a="WORK", pack="PACK_DIVSQRT",
               destination="RESULT")),
    (None, UOp(flow="JUMP", target="divsqrt_commit")),

    ("entry_trans", UOp(src_a="OPERAND_A", src_b="OPERAND_B",
                         classify="TRANS")),
    (None, UOp(flow="BRANCH", target="trans_commit",
               condition="DIRECT_READY")),
    (None, UOp(flow="BRANCH", target="trans_normalize_enter",
               condition="SPECIAL")),
    (None, UOp(flow="BRANCH", target="trans_atan_start",
               condition="TRANS_ATAN2")),
    ("trans_range", UOp(flow="LOOP", target="trans_range",
                         condition="COUNT_MORE",
                         engine="TRANS_RANGE_ITERATE", count="DEC")),
    (None, UOp(engine="TRANS_RANGE_FINALIZE")),
    (None, UOp(flow="JUMP", target="trans_cordic_prepare")),
    ("trans_atan_start", UOp(flow="JUMP", target="trans_cordic_prepare")),
    ("trans_cordic_prepare", UOp(engine="TRANS_CORDIC_PREP")),
    ("trans_cordic_load", UOp(
        flow="LOOP", target="trans_cordic_load",
        condition="CORDIC_LOAD_MORE", scratch_write="LOAD")),
    (None, UOp(flow="BRANCH", target="trans_atan_align_check",
               condition="TRANS_ATAN2")),
    (None, UOp(flow="JUMP", target="trans_cordic_begin")),
    ("trans_atan_align_check", UOp(
        flow="BRANCH", target="trans_cordic_begin",
        condition="COUNT_ZERO")),
    ("trans_atan_align_prepare", UOp(engine="CORDIC_ALIGN_PREP")),
    ("trans_atan_align_read", UOp(scratch_read="ALIGN")),
    ("trans_atan_align_write", UOp(
        flow="LOOP", target="trans_atan_align_read",
        condition="CORDIC_ALIGN_MORE", scratch_write="ALIGN")),
    (None, UOp(flow="LOOP", target="trans_atan_align_prepare",
               condition="COUNT_MORE", count="DEC")),
    ("trans_cordic_begin", UOp(engine="CORDIC_BEGIN")),
    ("trans_cordic_x_prepare", UOp(engine="CORDIC_X_PREP")),
    ("trans_cordic_x_low", UOp(scratch_read="X_LOW")),
    (None, UOp(scratch_read="X_HIGH")),
    ("trans_cordic_x_write", UOp(
        flow="LOOP", target="trans_cordic_x_low",
        condition="CORDIC_LIMB_MORE", scratch_write="X")),
    (None, UOp(engine="CORDIC_Y_PREP")),
    ("trans_cordic_y_low", UOp(scratch_read="Y_LOW")),
    (None, UOp(scratch_read="Y_HIGH")),
    ("trans_cordic_y_write", UOp(
        flow="LOOP", target="trans_cordic_y_low",
        condition="CORDIC_LIMB_MORE", scratch_write="Y")),
    (None, UOp(engine="CORDIC_Z_PREP")),
    ("trans_cordic_z_read", UOp(scratch_read="Z")),
    ("trans_cordic_z_write", UOp(
        flow="LOOP", target="trans_cordic_z_read",
        condition="CORDIC_LIMB_MORE", scratch_write="Z")),
    (None, UOp(flow="LOOP", target="trans_cordic_x_prepare",
               condition="COUNT_MORE", engine="CORDIC_NEXT",
               count="DEC")),
    (None, UOp(engine="CORDIC_OUTPUT_PREP")),
    ("trans_cordic_output_read", UOp(scratch_read="OUTPUT")),
    (None, UOp(engine="CORDIC_OUTPUT_CAPTURE")),
    ("trans_cordic_output_write", UOp(
        flow="LOOP", target="trans_cordic_output_read",
        condition="CORDIC_LIMB_MORE", scratch_write="PRIMARY")),
    (None, UOp(engine="CORDIC_AUX_PREP")),
    ("trans_cordic_aux_read", UOp(scratch_read="OUTPUT")),
    (None, UOp(engine="CORDIC_OUTPUT_CAPTURE")),
    ("trans_cordic_aux_write", UOp(
        flow="LOOP", target="trans_cordic_aux_read",
        condition="CORDIC_LIMB_MORE", scratch_write="AUX")),
    (None, UOp(state="TRANS_SELECT")),
    ("trans_normalize_enter", UOp(flow="BRANCH", target="trans_commit",
                                   condition="ZERO")),
    ("trans_normalize", UOp(flow="LOOP", target="trans_normalize",
                             condition="NORMALIZE_MORE",
                             state="TRANS_NORMALIZE",
                             destination="WORK", shift_route="FORMAT")),
    ("trans_round", UOp(alu_route="PREP_ROUND_TRANS",
                         destination="WORK", grs="LOAD")),
    (None, UOp(src_a="WORK", alu_route="ROUND",
               destination="WORK")),
    (None, UOp(flow="BRANCH", target="trans_aux",
               condition="TRANS_NEEDS_AUX")),
    (None, UOp(src_a="WORK", pack="PACK_TRANS",
               destination="RESULT")),
    (None, UOp(flow="JUMP", target="trans_commit")),
    ("trans_aux", UOp(src_a="WORK", pack="PACK_TRANS",
                       destination="RESULT")),
    (None, UOp(state="TRANS_LOAD_AUX", destination="WORK")),
    (None, UOp(flow="BRANCH", target="trans_commit", condition="ZERO")),
    (None, UOp(flow="JUMP", target="trans_normalize")),
    ("trans_commit", UOp(flow="FINISH", src_a="RESULT",
                          commit="REPLACE_ST0")),

    # Sequencer-only regression routines.
    ("entry_test_branch", UOp(flow="BRANCH", target="test_branch_taken", condition="TRUE")),
    (None, UOp(flow="FINISH", side_effect="QUIET_NAN")),
    ("test_branch_taken", UOp(flow="FINISH", side_effect="SET_STATUS")),
    ("entry_test_loop", UOp(flow="LOOP", target="entry_test_loop", condition="COUNT_MORE",
                            count="DEC")),
    (None, UOp(flow="FINISH")),
    ("entry_test_wait", UOp(flow="WAIT", condition="TRANSFER_READY")),
    (None, UOp(flow="FINISH")),
]


def enum_value(table: dict[str, int], name: str, field: str) -> int:
    if name not in table:
        raise ValueError(f"unknown {field} value {name!r}")
    return table[name]


def validate_horizontal_controls(address: int, uop: UOp) -> None:
    """Reject horizontal combinations that need the same datapath."""
    major_lanes = (
        uop.prepare != "HOLD",
        uop.engine != "HOLD",
        uop.pack != "HOLD",
        uop.alu_route != "HOLD",
        uop.classify != "HOLD",
        uop.state != "HOLD",
    )
    classify_prepare_pair = (
        (uop.classify == "ADDSUB" and uop.prepare == "ADDSUB") or
        (uop.classify == "MUL" and uop.prepare == "MUL")
    )
    if sum(major_lanes) > 1 and not (
        classify_prepare_pair and sum(major_lanes) == 2
    ):
        raise ValueError(f"multiple major resource lanes at {address:#x}")

    if (uop.shift_route in {"LEFT", "RIGHT", "FORMAT"} and
        uop.destination != "WORK"):
        raise ValueError(
            f"shift without WORK destination at {address:#x}"
        )

    if uop.shift_route in {"LEFT", "RIGHT"} and uop.state not in {
        "HOLD", "FORMAT_EXP_DEC", "FORMAT_EXP_INC"
    }:
        raise ValueError(
            f"state operation {uop.state} conflicts with work shift at "
            f"{address:#x}"
        )
    if uop.shift_route in {"LEFT", "RIGHT"} and uop.alu_route != "HOLD":
        raise ValueError(f"work operation conflicts with shift at {address:#x}")

    count_dec_engines = {
        "HOLD", "MUL_ACCUMULATE", "DIV_ITERATE", "SQRT_ITERATE",
        "TRANS_RANGE_ITERATE", "ADDSUB_ALIGN",
        "CORDIC_NEXT",
    }
    if (uop.count == "DEC" and
        (uop.state != "HOLD" or
         uop.engine not in count_dec_engines)):
        raise ValueError(
            f"state operation {uop.state} conflicts with count decrement at "
            f"{address:#x}"
        )

    if uop.grs == "SHIFT" and not (
        (uop.shift_route == "RIGHT" and
         uop.state in {"HOLD", "FORMAT_EXP_INC"}) or
        uop.engine == "ADDSUB_ALIGN"
    ):
        raise ValueError(f"unsupported horizontal GRS shift at {address:#x}")


def assemble() -> tuple[list[int], dict[str, int], list[str]]:
    labels: dict[str, int] = {}
    for address, (label, _) in enumerate(PROGRAM):
        if label is not None:
            if label in labels:
                raise ValueError(f"duplicate label {label}")
            labels[label] = address

    words: list[int] = []
    listing: list[str] = []
    for address, (label, uop) in enumerate(PROGRAM):
        validate_horizontal_controls(address, uop)
        target = labels[uop.target] if isinstance(uop.target, str) else uop.target
        fields = [
            (target, 8, "target"),
            (enum_value(FLOW, uop.flow, "flow"), 3, "flow"),
            (enum_value(COND, uop.condition, "condition"), 5, "condition"),
            (enum_value(ALU_ROUTE, uop.alu_route, "alu_route"), 4,
             "alu_route"),
            (enum_value(SHIFT_ROUTE, uop.shift_route, "shift_route"), 4,
             "shift_route"),
            (enum_value(PREPARE, uop.prepare, "prepare"), 4, "prepare"),
            (enum_value(CLASSIFY, uop.classify, "classify"), 4, "classify"),
            (enum_value(PACK, uop.pack, "pack"), 4, "pack"),
            (enum_value(ENGINE, uop.engine, "engine"), 5, "engine"),
            (enum_value(STATE, uop.state, "state"), 4, "state"),
            (enum_value(COUNT, uop.count, "count"), 2, "count"),
            (enum_value(GRS, uop.grs, "grs"), 2, "grs"),
            (enum_value(COMMIT, uop.commit, "commit"), 3, "commit"),
            (uop.flags, 2, "flags"),
            (enum_value(SCRATCH_READ, uop.scratch_read,
                        "scratch_read"), 3, "scratch_read"),
            (enum_value(SCRATCH_WRITE, uop.scratch_write,
                        "scratch_write"), 3, "scratch_write"),
            (0, 4, "reserved"),
        ]
        word = 0
        for value, width, field in fields:
            if not 0 <= value < (1 << width):
                raise ValueError(f"{field}={value} exceeds {width} bits at {address:#x}")
            word = (word << width) | value
        if word.bit_length() > 64:
            raise ValueError(f"word at {address:#x} exceeds 64 bits")
        words.append(word)
        label_text = f"{label}:" if label else ""
        listing.append(
            f"{address:02x} {word:016x} {label_text:<22} "
            f"{uop.flow:<7} {uop.prepare:<16} {uop.classify:<12} "
            f"{uop.pack:<20} {uop.engine:<22} {uop.state:<20} "
            f"{uop.alu_route:<22} {uop.shift_route:<8} "
            f"{uop.scratch_read:<8} {uop.scratch_write:<8} {uop.commit}"
        )

    if len(words) > 256:
        raise ValueError(f"control store contains {len(words)} words; maximum is 256")
    words.extend([0] * (256 - len(words)))
    return words, labels, listing


def write_mif(path: Path, words: list[int]) -> None:
    lines = [
        "WIDTH=64;",
        "DEPTH=256;",
        "ADDRESS_RADIX=HEX;",
        "DATA_RADIX=HEX;",
        "CONTENT BEGIN",
    ]
    lines.extend(f"    {address:02X} : {word:016X};" for address, word in enumerate(words))
    lines.append("END;")
    path.write_text("\n".join(lines) + "\n")


def write_svh(path: Path, words: list[int]) -> None:
    lines = [
        "// Generated by build_x87_microcode.py. Do not edit.",
        "function automatic logic [63:0] x87_ucode_word(input logic [7:0] rom_address);",
        "    case (rom_address)",
    ]
    lines.extend(
        f"        8'h{address:02x}: x87_ucode_word = 64'h{word:016x};"
        for address, word in enumerate(words) if word
    )
    lines.extend([
        "        default: x87_ucode_word = 64'h0;",
        "    endcase",
        "endfunction",
    ])
    path.write_text("\n".join(lines) + "\n")


def write_entries(path: Path, labels: dict[str, int]) -> None:
    lines = ["// Generated by build_x87_microcode.py. Do not edit."]
    for name, value in COND.items():
        lines.append(f"localparam logic [4:0] X87_COND_{name} = 5'd{value};")
    for name, value in STATE.items():
        lines.append(
            f"localparam logic [3:0] X87_STATE_{name} = 4'd{value};"
        )
    for name, value in PREPARE.items():
        lines.append(
            f"localparam logic [3:0] X87_PREPARE_{name} = 4'd{value};"
        )
    for name, value in CLASSIFY.items():
        lines.append(
            f"localparam logic [3:0] X87_CLASSIFY_{name} = 4'd{value};"
        )
    for name, value in ENGINE.items():
        lines.append(
            f"localparam logic [4:0] X87_ENGINE_{name} = 5'd{value};"
        )
    for name, value in PACK.items():
        lines.append(
            f"localparam logic [3:0] X87_PACK_{name} = 4'd{value};"
        )
    for name, value in ALU_ROUTE.items():
        lines.append(f"localparam logic [3:0] X87_ALU_{name} = 4'd{value};")
    for name, value in SHIFT_ROUTE.items():
        lines.append(f"localparam logic [3:0] X87_SHIFT_{name} = 4'd{value};")
    for name, value in COUNT.items():
        lines.append(f"localparam logic [1:0] X87_COUNT_{name} = 2'd{value};")
    for name, value in GRS.items():
        lines.append(f"localparam logic [1:0] X87_GRS_{name} = 2'd{value};")
    for name, value in COMMIT.items():
        lines.append(f"localparam logic [2:0] X87_COMMIT_{name} = 3'd{value};")
    for name, value in SCRATCH_READ.items():
        lines.append(
            f"localparam logic [2:0] X87_SCRATCH_READ_{name} = 3'd{value};"
        )
    for name, value in SCRATCH_WRITE.items():
        lines.append(
            f"localparam logic [2:0] X87_SCRATCH_WRITE_{name} = 3'd{value};"
        )
    for label, address in labels.items():
        if label.startswith("entry_"):
            name = "X87_" + label.upper()
            lines.append(f"localparam logic [7:0] {name} = 8'h{address:02x};")
    path.write_text("\n".join(lines) + "\n")


def main() -> None:
    out_dir = Path(__file__).resolve().parent
    words, labels, listing = assemble()
    write_mif(out_dir / "x87_ucode.mif", words)
    write_svh(out_dir / "x87_ucode.svh", words)
    write_entries(out_dir / "x87_entries.svh", labels)
    (out_dir / "x87_ucode.lst").write_text("\n".join(listing) + "\n")
    print(f"wrote {len(PROGRAM)} x87 microinstructions")


if __name__ == "__main__":
    main()
