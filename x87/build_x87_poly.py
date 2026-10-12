#!/usr/bin/env python3
"""Generate the x87 transcendental programs (x87_poly_rom.svh).

FSIN, FCOS, FSINCOS, FPTAN and FPATAN run as short programs of the
executor's own operations (multiply, add, subtract, divide, round to
integer, compare), sequenced by x87_control's polynomial engine. Each
operation rounds to the executor's 53-bit significand, round to nearest;
the last one rounds per the control word. The engine holds temporaries in a
16-entry scratch and reads constants from a 64-entry table.

  sin/cos: Cody-Waite reduction by pi/2 in four parts (n * part exact for
           |n| < 2^20), then the fdlibm kernels __kernel_sin/__kernel_cos;
           the quadrant (n mod 4) selects and signs the result.
  atan2:   t = min(|y|,|x|) / max(|y|,|x|), fdlibm's atan reduction
           (thresholds 7/16 and 11/16, c = 0, 1/2, 1), its polynomial, then
           pi/2 - r when |y| > |x|, pi - r when x < 0, and y's sign.

The script also runs both programs on double-precision inputs (the executor
rounds as IEEE double within double's range) against mpmath and fails if the
error exceeds the limit.
"""

from __future__ import annotations

import argparse
import math
import random
import struct
from fractions import Fraction
from pathlib import Path

import mpmath

mpmath.mp.prec = 200

# --------------------------------------------------------------------------
# Constants (all exactly representable doubles)
# --------------------------------------------------------------------------


def rn(x) -> float:
    """mpmath number rounded to the nearest double."""
    return float(mpmath.mpf(x))


def trunc_bits(x, bits: int) -> float:
    """x truncated to `bits` significant bits."""
    m, e = mpmath.frexp(mpmath.mpf(x))
    scaled = mpmath.floor(m * 2**bits)
    return float(mpmath.ldexp(scaled, e - bits))


PI = mpmath.pi
PIO2 = PI / 2

# pi/2 in four parts: three 33-bit heads (products with |n| < 2^20 exact)
# and a double tail.
P1 = trunc_bits(PIO2, 33)
P2 = trunc_bits(PIO2 - P1, 33)
P3 = trunc_bits(PIO2 - P1 - P2, 33)
P4 = rn(PIO2 - P1 - P2 - P3)


def hilo(x) -> tuple[float, float]:
    hi = rn(x)
    return hi, rn(mpmath.mpf(x) - hi)


def h2d(h: int) -> float:
    return struct.unpack(">d", h.to_bytes(8, "big"))[0]


# fdlibm k_sin.c / k_cos.c / s_atan.c coefficients.
S = [h2d(v) for v in (0xBFC5555555555549, 0x3F8111111110F8A6, 0xBF2A01A019C161D5,
                      0x3EC71DE357B1FE7D, 0xBE5AE5E68A2B9CEB, 0x3DE5D93A5ACFD57C)]
C = [h2d(v) for v in (0x3FA555555555554C, 0xBF56C16C16C15177, 0x3EFA01A019CB1590,
                      0xBE927E4F809C52AD, 0x3E21EE9EBDB4B1C4, 0xBDA8FAE9BE8838D4)]
AT = [h2d(v) for v in (0x3FD555555555550D, 0xBFC999999998EBC4, 0x3FC24924920083FF,
                       0xBFBC71C6FE231671, 0x3FB745CDC54C206E, 0xBFB3B0F2AF749A6D,
                       0x3FB10D66A0D03D51, 0xBFADDE2D52DEFD9A, 0x3FA97B4B24760DEB,
                       0xBFA2B4442C6A6C2F, 0x3F90AD3AE322DA11)]
# atan2's result is base + atan(c) or base - atan(c), plus or minus the
# kernel's atan(t'), with the quadrant folded into the hi/lo constants:
#   A: |y| <= |x|, x >= 0:  atan(c)        + atan(t')
#   B: |y| >  |x|, x >= 0:  pi/2 - atan(c) - atan(t')
#   C: |y| <= |x|, x <  0:  pi - atan(c)   - atan(t')
#   D: |y| >  |x|, x <  0:  pi/2 + atan(c) + atan(t')
ATAN_C = [mpmath.mpf(0), mpmath.atan(mpmath.mpf(1) / 2), PI / 4]
ATAN_TABLES = {
    "A": [hilo(a) for a in ATAN_C],
    "B": [hilo(PIO2 - a) for a in ATAN_C],
    "C": [hilo(PI - a) for a in ATAN_C],
    "D": [hilo(PIO2 + a) for a in ATAN_C],
}

CONSTS: list[tuple[str, float]] = [
    ("TWO_OVER_PI", rn(2 / PI)),
    ("P1", P1), ("P2", P2), ("P3", P3), ("P4", P4),
    *[(f"S{i + 1}", v) for i, v in enumerate(S)],
    *[(f"C{i + 1}", v) for i, v in enumerate(C)],
    ("HALF", 0.5), ("ONE", 1.0),
    ("K7_16", 7 / 16), ("K11_16", 11 / 16),
    # Indexed by the atan reduction id (0, 1, 2): keep each group adjacent.
    ("CT0", 0.0), ("CT1", 0.5), ("CT2", 1.0),
    *[(f"HI{q}{i}", hl[0]) for q, tab in ATAN_TABLES.items() for i, hl in enumerate(tab)],
    *[(f"LO{q}{i}", hl[1]) for q, tab in ATAN_TABLES.items() for i, hl in enumerate(tab)],
    *[(f"AT{i}", v) for i, v in enumerate(AT)],
]
CONST_INDEX = {name: i for i, (name, _) in enumerate(CONSTS)}
assert len(CONSTS) <= 64

# --------------------------------------------------------------------------
# Program
# --------------------------------------------------------------------------

OPS = {"MUL": 0, "ADD": 1, "SUB": 2, "DIV": 3, "RND": 4, "CMP": 5, "AUX": 6}
PREDS = ["ALWAYS", "Q0", "Q1", "Q2", "Q3", "Q0S", "Q1S", "Q2S", "Q3S",
         "Q0C", "Q1C", "Q2C", "Q3C", "SWAP", "NOSWAP", "YNEG", "QA", "QB", "QC", "QD"]
PRED = {name: i for i, name in enumerate(PREDS)}
SETP = {"NONE": 0, "SWAP": 1, "ID1": 2, "ID2": 3}   # CMP: SWAP = a < b, IDn = a >= b
SLOTS = ["IN_A", "IN_B", "N", "R", "Z", "T1", "P", "Q", "S", "C", "H",
         "T", "U", "V", "W", "A2"]
SLOT = {name: i for i, name in enumerate(SLOTS)}
assert not set(SLOTS) & {name for name, _ in CONSTS}, "slot and constant names overlap"


class Op:
    def __init__(self, op, a=None, b=None, dest="T1", pred="ALWAYS", final=False,
                 setp="NONE", a_neg=False, a_abs=False, b_abs=False, a_id=False, b_id=False):
        self.op, self.a, self.b, self.dest = op, a, b, dest
        self.pred, self.final, self.setp = pred, final, setp
        self.a_neg, self.a_abs, self.b_abs, self.a_id, self.b_id = a_neg, a_abs, b_abs, a_id, b_id


def horner(acc: str, var: str, coeffs: list[str]) -> list[Op]:
    """acc = coeffs[0] + var*(coeffs[1] + ... + var*coeffs[-1])."""
    ops = [Op("MUL", var, coeffs[-1], acc)]
    for k in reversed(coeffs[:-1]):
        ops.append(Op("ADD", acc, k, acc))
        if k != coeffs[0]:
            ops.append(Op("MUL", acc, var, acc))
    return ops


SINCOS = [
    Op("MUL", "IN_A", "TWO_OVER_PI", "T1"),
    Op("RND", "T1", None, "N"),
    # A = x - n*P1 is exact; the tail n*(P2 + P3 + P4) is subtracted once,
    # and RLO = (A - R) - tail keeps what that rounding dropped (r = R + RLO).
    Op("MUL", "N", "P1", "T1"), Op("SUB", "IN_A", "T1", "U"),
    Op("MUL", "N", "P2", "T1"), Op("MUL", "N", "P3", "W"), Op("ADD", "T1", "W", "T1"),
    Op("MUL", "N", "P4", "W"), Op("ADD", "T1", "W", "T1"),
    Op("SUB", "U", "T1", "R"), Op("SUB", "U", "R", "W"), Op("SUB", "W", "T1", "V"),
    Op("MUL", "R", "R", "Z"),
    # sin(r) = R + (RLO + R*z*(S1 + z*(S2 + ... + z*S6)))
    *horner("P", "Z", ["S1", "S2", "S3", "S4", "S5", "S6"]),
    Op("MUL", "R", "Z", "T1"), Op("MUL", "T1", "P", "T1"), Op("ADD", "T1", "V", "T1"),
    Op("ADD", "R", "T1", "S"),
    # cos(r) = 1 - (z/2 - (z*z*(C1 + z*(C2 + ... + z*C6)) - R*RLO))
    *horner("Q", "Z", ["C1", "C2", "C3", "C4", "C5", "C6"]),
    Op("MUL", "Z", "Z", "T1"), Op("MUL", "T1", "Q", "T1"),
    Op("MUL", "R", "V", "W"), Op("SUB", "T1", "W", "T1"),
    Op("MUL", "Z", "HALF", "H"), Op("SUB", "H", "T1", "T1"), Op("SUB", "ONE", "T1", "C"),
    # cos(x) for FPTAN/FSINCOS, then sin(x) or cos(x) by quadrant.
    Op("AUX", "C", pred="Q0"), Op("AUX", "S", pred="Q1", a_neg=True),
    Op("AUX", "C", pred="Q2", a_neg=True), Op("AUX", "S", pred="Q3"),
    Op("MUL", "S", "ONE", pred="Q0S", final=True),
    Op("MUL", "C", "ONE", pred="Q1S", final=True),
    Op("MUL", "S", "ONE", pred="Q2S", final=True, a_neg=True),
    Op("MUL", "C", "ONE", pred="Q3S", final=True, a_neg=True),
    Op("MUL", "C", "ONE", pred="Q0C", final=True),
    Op("MUL", "S", "ONE", pred="Q1C", final=True, a_neg=True),
    Op("MUL", "C", "ONE", pred="Q2C", final=True, a_neg=True),
    Op("MUL", "S", "ONE", pred="Q3C", final=True),
]

# FPATAN: IN_A = Y (ST1), IN_B = X (ST0).
ATAN = [
    Op("CMP", "IN_B", "IN_A", None, setp="SWAP", a_abs=True, b_abs=True),
    Op("DIV", "IN_A", "IN_B", "T", pred="NOSWAP", a_abs=True, b_abs=True),
    Op("DIV", "IN_B", "IN_A", "T", pred="SWAP", a_abs=True, b_abs=True),
    Op("CMP", "T", "K7_16", None, setp="ID1"),
    Op("CMP", "T", "K11_16", None, setp="ID2"),
    Op("SUB", "T", "CT0", "U", b_id=True),
    Op("MUL", "T", "CT0", "V", b_id=True),
    Op("ADD", "V", "ONE", "V"),
    Op("DIV", "U", "V", "T"),
    Op("MUL", "T", "T", "Z"), Op("MUL", "Z", "Z", "W"),
    *horner("P", "W", ["AT0", "AT2", "AT4", "AT6", "AT8", "AT10"]),
    Op("MUL", "P", "Z", "P"),
    *horner("A2", "W", ["AT1", "AT3", "AT5", "AT7", "AT9"]),
    Op("MUL", "A2", "W", "A2"),
    Op("ADD", "P", "A2", "S"), Op("MUL", "T", "S", "S"),
    # Kernel added (A, D): R = HI - ((t'*s - LO) - t'); subtracted (B, C):
    # R = HI - ((t' - t'*s) - LO).
    *[o for q in "AD" for o in (
        Op("SUB", "S", f"LO{q}0", "S", pred=f"Q{q}", b_id=True),
        Op("SUB", "S", "T", "S", pred=f"Q{q}"),
        Op("SUB", f"HI{q}0", "S", "R", pred=f"Q{q}", a_id=True))],
    *[o for q in "BC" for o in (
        Op("SUB", "T", "S", "S", pred=f"Q{q}"),
        Op("SUB", "S", f"LO{q}0", "S", pred=f"Q{q}", b_id=True),
        Op("SUB", f"HI{q}0", "S", "R", pred=f"Q{q}", a_id=True))],
    Op("MUL", "R", "ONE", pred="YNEG", final=True, a_neg=True),
    Op("MUL", "R", "ONE", final=True),
]

PROGRAM = SINCOS + ATAN
ATAN_ENTRY = len(SINCOS)


def operand(name: str | None) -> tuple[int, int]:
    """(is_const, index)."""
    if name is None:
        return 0, 0
    if name in SLOT:
        return 0, SLOT[name]
    return 1, CONST_INDEX[name]


def encode(o: Op) -> int:
    a_c, a_i = operand(o.a)
    b_c, b_i = operand(o.b)
    dest = SLOT[o.dest] if o.dest else 0
    word = 0
    for value, width in ((OPS[o.op], 3), (int(o.final), 1), (a_c, 1), (a_i, 6), (int(o.a_id), 1),
                         (int(o.a_neg), 1), (int(o.a_abs), 1), (b_c, 1), (b_i, 6), (int(o.b_id), 1),
                         (int(o.b_abs), 1), (dest, 4), (PRED[o.pred], 5), (SETP[o.setp], 2)):
        assert 0 <= value < (1 << width)
        word = (word << width) | value
    return word


WORD_BITS = 3 + 1 + 1 + 6 + 1 + 1 + 1 + 1 + 6 + 1 + 1 + 4 + 5 + 2

# --------------------------------------------------------------------------
# Emulation (IEEE double, round to nearest; the executor matches within
# double's exponent range)
# --------------------------------------------------------------------------


def run(prog: list[Op], entry: int, a: float, b: float, cosine: bool):
    slots = {"IN_A": a, "IN_B": b}
    const = dict(CONSTS)
    swap = idn = 0
    quad = 0
    xneg, yneg = b < 0 or (b == 0 and math.copysign(1, b) < 0), a < 0
    aux = None
    pc = entry

    def fetch(name, neg=False, absv=False, use_id=False):
        if name in SLOT:
            v = slots[name]
        else:
            if use_id:
                base = name[:-1]
                name = f"{base}{int(name[-1]) + idn}"
            v = const[name]
        if absv:
            v = abs(v)
        if neg:
            v = -v
        return v

    while True:
        o = prog[pc]
        pc += 1
        p = o.pred
        q = quad
        ok = {"ALWAYS": True, "SWAP": swap == 1, "NOSWAP": swap == 0, "YNEG": yneg,
              "QA": not swap and not xneg, "QB": swap and not xneg,
              "QC": not swap and xneg, "QD": swap and xneg}
        for k in range(4):
            ok[f"Q{k}"] = q == k
            ok[f"Q{k}S"] = q == k and not cosine
            ok[f"Q{k}C"] = q == k and cosine
        if not ok[p]:
            continue
        x = fetch(o.a, o.a_neg, o.a_abs, o.a_id)
        y = fetch(o.b, False, o.b_abs, o.b_id) if o.b else None
        if o.op == "AUX":
            aux = x
            continue
        if o.op == "CMP":
            less = x < y
            if o.setp == "SWAP":
                swap = int(less)
            else:
                idn += int(not less)
            continue
        if o.op == "MUL":
            r = x * y
        elif o.op == "ADD":
            r = x + y
        elif o.op == "SUB":
            r = x - y
        elif o.op == "DIV":
            r = x / y
        elif o.op == "RND":
            r = float(round(x))
            n = int(r)
            quad = n & 3
        if o.final:
            return r, aux
        slots[o.dest] = r


def ulp_error(got: float, exact) -> float:
    exact = mpmath.mpf(exact)
    if exact == 0:
        return 0.0 if got == 0 else float("inf")
    e = mpmath.floor(mpmath.log(abs(exact), 2))
    return float(abs(mpmath.mpf(got) - exact) / mpmath.ldexp(1, int(e) - 52))


def check(samples: int, limit: float) -> None:
    rng = random.Random(486)
    worst = {"sin": 0.0, "cos": 0.0, "atan2": 0.0}
    where = {}
    for i in range(samples):
        e = rng.uniform(-26, 19)                       # |x| from 2^-26 to 2^19
        x = math.copysign(2.0 ** e * rng.uniform(1, 2), rng.choice((-1, 1)))
        for name, cosine, fn in (("sin", False, mpmath.sin), ("cos", True, mpmath.cos)):
            got, aux = run(PROGRAM, 0, x, 0.0, cosine)
            err = ulp_error(got, fn(mpmath.mpf(x)))
            if err > worst[name]:
                worst[name], where[name] = err, x
            if name == "sin":
                aerr = ulp_error(aux, mpmath.cos(mpmath.mpf(x)))
                if aerr > worst["cos"]:
                    worst["cos"], where["cos"] = aerr, x
        y = math.copysign(2.0 ** rng.uniform(-30, 30) * rng.uniform(1, 2), rng.choice((-1, 1)))
        xx = math.copysign(2.0 ** rng.uniform(-30, 30) * rng.uniform(1, 2), rng.choice((-1, 1)))
        got, _ = run(PROGRAM, ATAN_ENTRY, y, xx, False)
        err = ulp_error(got, mpmath.atan2(mpmath.mpf(y), mpmath.mpf(xx)))
        if err > worst["atan2"]:
            worst["atan2"], where["atan2"] = err, (y, xx)
    for name, err in worst.items():
        print(f"  {name:6s} max error {err:.3f} ulp at {where.get(name)}")
        if err > limit:
            raise SystemExit(f"{name}: {err:.3f} ulp exceeds {limit}")


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------


X87_ZERO, X87_NORMAL = 1, 2     # x87_pkg::x87_class_t


def to_reg(v: float) -> int:
    """Double to x87_pkg::x87_reg_t {sign, exp[14:0], sig[52:0], G, R, S, class[2:0]}."""
    bits = struct.unpack(">Q", struct.pack(">d", v))[0]
    sign = bits >> 63
    if v == 0.0:
        return (sign << 74) | X87_ZERO
    exp = (bits >> 52) & 0x7FF
    frac = bits & ((1 << 52) - 1)
    assert 0 < exp < 0x7FF
    sig = (1 << 52) | frac
    return (sign << 74) | ((exp - 1023 + 16383) << 59) | (sig << 6) | X87_NORMAL


TEST_OPS = {"FSIN": 0, "FCOS": 1, "FSINCOS": 2, "FPTAN": 3, "FPATAN": 4}


def write_test(path: Path, count: int) -> None:
    """A real-mode test that runs each operation on random arguments and
    compares the stored doubles with this script's emulation bit for bit."""
    rng = random.Random(4860)
    rows = []

    def d2h(v: float) -> int:
        return struct.unpack(">Q", struct.pack(">d", v))[0]

    for i in range(count):
        op = list(TEST_OPS)[i % len(TEST_OPS)]
        x = math.copysign(2.0 ** rng.uniform(-20, 19) * rng.uniform(1, 2), rng.choice((-1, 1)))
        y = math.copysign(2.0 ** rng.uniform(-20, 20) * rng.uniform(1, 2), rng.choice((-1, 1)))
        if op == "FPATAN":
            r1, _ = run(PROGRAM, ATAN_ENTRY, y, x, False)    # ST1 = y, ST0 = x
            r2 = 0.0
        else:
            sin_x, cos_x = run(PROGRAM, 0, x, 0.0, False)
            if op == "FSIN":
                r1, r2 = sin_x, 0.0
            elif op == "FCOS":
                r1, _ = run(PROGRAM, 0, x, 0.0, True)
                r2 = 0.0
            elif op == "FSINCOS":
                r1, r2 = cos_x, sin_x                     # ST0 = cos, ST1 = sin
            else:
                r1, r2 = 1.0, sin_x / cos_x               # ST0 = 1.0, ST1 = tan
        rows.append((TEST_OPS[op], x, y, r1, r2))

    asm = [
        "; Generated by x87/build_x87_poly.py --test: FSIN, FCOS, FSINCOS, FPTAN",
        "; and FPATAN on random arguments against the program's emulation, bit",
        "; for bit (which build_x87_poly.py checks against mpmath).",
        "; Fail code (port E4): case index + 1.",
        "", "BITS 16", "org 0", "",
        "STATUS_PORT equ 0xE0", "DATA_PORT   equ 0xE4", "",
        "start:", "    cli", "    mov ax, cs", "    mov ds, ax", "    mov ss, ax", "    mov sp, 0x700",
        "    fninit", "    mov si, cases", "    xor bp, bp",
        "next:", f"    cmp bp, {len(rows)}", "    je pass",
        "    mov al, [si]",
        "    cmp al, 4", "    je do_atan",
        "    fld qword [si + 8]",
        "    cmp al, 0", "    jne .n0", "    fsin", "    jmp one",
        ".n0:", "    cmp al, 1", "    jne .n1", "    fcos", "    jmp one",
        ".n1:", "    cmp al, 2", "    jne .n2", "    fsincos", "    jmp two",
        ".n2:", "    fptan", "    jmp two",
        "do_atan:", "    fld qword [si + 16]", "    fld qword [si + 8]", "    fpatan",
        "one:", "    fstp qword [result]", "    mov eax, [result]", "    cmp eax, [si + 24]", "    jne fail",
        "    mov eax, [result + 4]", "    cmp eax, [si + 28]", "    jne fail", "    jmp advance",
        "two:", "    fstp qword [result]", "    mov eax, [result]", "    cmp eax, [si + 24]", "    jne fail",
        "    mov eax, [result + 4]", "    cmp eax, [si + 28]", "    jne fail",
        "    fstp qword [result]", "    mov eax, [result]", "    cmp eax, [si + 32]", "    jne fail",
        "    mov eax, [result + 4]", "    cmp eax, [si + 36]", "    jne fail",
        "advance:", "    add si, 40", "    inc bp", "    jmp next",
        "pass:", "    mov al, 1", "    out STATUS_PORT, al", "    hlt",
        "fail:", "    mov ax, bp", "    inc ax", "    out DATA_PORT, ax", "    mov al, 0xff",
        "    out STATUS_PORT, al", "    hlt", "",
        "align 8", "result: dq 0", "align 8", "cases:",
    ]
    for op, x, y, r1, r2 in rows:
        asm.append(f"    dd {op}, 0")
        asm.append(f"    dq 0x{d2h(x):016x}, 0x{d2h(y):016x}, 0x{d2h(r1):016x}, 0x{d2h(r2):016x}")
    path.write_text("\n".join(asm) + "\n")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--samples", type=int, default=4000)
    ap.add_argument("--limit", type=float, default=1.1)
    ap.add_argument("--test", type=int, metavar="N", default=0,
                    help="also write tests/programs/x87_trans_poly.asm with N cases")
    args = ap.parse_args()

    print(f"program: {len(SINCOS)} sin/cos words, {len(ATAN)} atan2 words; {len(CONSTS)} constants")
    check(args.samples, args.limit)

    root = Path(__file__).resolve().parent
    pred_terms = {
        "ALWAYS": "1'b1", "SWAP": "swap", "NOSWAP": "!swap", "YNEG": "yneg",
        "QA": "!swap && !xneg", "QB": "swap && !xneg", "QC": "!swap && xneg", "QD": "swap && xneg",
    }
    for k in range(4):
        pred_terms[f"Q{k}"] = f"quadrant == 2'd{k}"
        pred_terms[f"Q{k}S"] = f"quadrant == 2'd{k} && !cosine"
        pred_terms[f"Q{k}C"] = f"quadrant == 2'd{k} && cosine"
    lines = [
        "// Generated by build_x87_poly.py: x87 transcendental programs. Do not edit.",
        "typedef struct packed {",
        "    logic [2:0] op;        // X87_POLY_OP_*",
        "    logic       final_op;  // last operation: rounds per the control word",
        "    logic       a_const;   // A from the constant table (else scratch)",
        "    logic [5:0] a_index;",
        "    logic       a_id;      // add the atan reduction id to A's constant index",
        "    logic       a_neg;",
        "    logic       a_abs;",
        "    logic       b_const;",
        "    logic [5:0] b_index;",
        "    logic       b_id;",
        "    logic       b_abs;",
        "    logic [3:0] dest;      // scratch slot written with the result",
        "    logic [4:0] pred;      // run only when the predicate holds",
        "    logic [1:0] setp;      // CMP: 1 swap = A < B, 2/3 id += A >= B",
        "} x87_poly_word_t;",
        *[f"localparam logic [2:0] X87_POLY_OP_{name} = 3'd{v};" for name, v in OPS.items()],
        *[f"localparam logic [1:0] X87_POLY_SETP_{name} = 2'd{v};" for name, v in SETP.items()],
        f"localparam int X87_POLY_WORD_BITS = {WORD_BITS};",
        f"localparam logic [6:0] X87_POLY_ENTRY_SINCOS = 7'd0;",
        f"localparam logic [6:0] X87_POLY_ENTRY_ATAN = 7'd{ATAN_ENTRY};",
        "",
        f"function automatic logic [{WORD_BITS - 1}:0] x87_poly_word(input logic [6:0] address);",
        "    case (address)",
    ]
    for i, o in enumerate(PROGRAM):
        desc = f"{o.op} {o.a or ''}{',' + o.b if o.b else ''} -> {o.dest or '-'} [{o.pred}]" + (" final" if o.final else "")
        lines.append(f"        7'd{i}: return {WORD_BITS}'h{encode(o):0{(WORD_BITS + 3) // 4}x}; // {desc}")
    lines += ["        default: return '0;", "    endcase", "endfunction", "",
              "function automatic logic [74:0] x87_poly_const(input logic [5:0] address);",
              "    case (address)"]
    for i, (name, v) in enumerate(CONSTS):
        lines.append(f"        6'd{i}: return 75'h{to_reg(v):019x}; // {name} = {v!r}")
    lines += ["        default: return '0;", "    endcase", "endfunction", "",
              "function automatic logic x87_poly_pred(input logic [4:0] pred, input logic [1:0] quadrant,",
              "                                       input logic cosine, input logic swap,",
              "                                       input logic xneg, input logic yneg);",
              "    case (pred)"]
    for name in PREDS:
        lines.append(f"        5'd{PRED[name]}: return {pred_terms[name]}; // {name}")
    lines += ["        default: return 1'b0;", "    endcase", "endfunction", ""]
    (root / "x87_poly_rom.svh").write_text("\n".join(lines))
    print("wrote x87_poly_rom.svh")
    if args.test:
        write_test(root.parent / "tests" / "programs" / "x87_trans_poly.asm", args.test)
        print(f"wrote tests/programs/x87_trans_poly.asm ({args.test} cases)")


if __name__ == "__main__":
    main()
