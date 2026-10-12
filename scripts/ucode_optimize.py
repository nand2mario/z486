#!/usr/bin/env python3
"""Apply z486 microcode optimizations to the extracted 80386 CROM.

`ucode_base.hex` is the original 37-bit extracted microcode and is NEVER edited.
This script applies the documented PATCHES below and writes the optimized
40-bit `ucode.hex` (+ `ucode.mif`): the native word remains in bits 36:0 and
the D2 early kind occupies bits 39:37. This script also owns the hardwired
common-instruction recipe inventory and
generates its SystemVerilog lookup and human-readable manifest, so microcode
words and the recipes that consume them cannot silently drift apart.

37-bit word field layout:
    bus[5:0]  sub[7:6]  op[10:8]  aluop[17:11]  src[23:18]  dst[30:24]  alusrc[36:31]
  RNI = op field 0 (default 7);  DLY = sub field 0 (default 3).
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path

BASE_DEPTH = 2560          # Extracted 80386 CROM words (000-9FF).
ROM_DEPTH = 2560          # ROM words; grow in 512-word steps (UCODE_ROM_DEPTH too)
                          # when optimizer-owned words go past 9FF.
UCODE_BITS = 37
ROM_BITS = 40
SRC_TMPC = 0x0C            # Canonical CROM source encoding.
DEST_SRCREG = 0x3E         # Canonical CROM destination encoding.
DEST_USTEP_RPTI_EIP = 0x6D # Optimizer-owned: restart EIP write.
DEST_USTEP_TASK_CS = 0x6E  # Optimizer-owned: task load establishes CS RPL.
DEST_USTEP_FAULT_DONE = 0x6F # Optimizer-owned: fault delivery completion.
DEST_USTEP_INVLPG = 0x70   # Optimizer-owned: invalidate one TLB page.
DEST_USTEP_X87_STORE = 0x71 # Optimizer-owned: x87 store command and result read.
DEST_USTEP_ALU = 0x7E       # Optimizer-owned: commit this word's ALU result to DSTREG.
DEST_USTEP_BSWAP = 0x7C     # Optimizer-owned: byte-swap SRCREG into itself.
ALUJMP_JDESCA = 0x20        # Optimizer-owned: jump if descriptor A bit is set.
ALUJMP_USTEP_AAD_SHIFT = 0x21 # Optimizer-owned: AAD barrel result and CF clear.
ALUJMP_USTEP_IMUL_FLAGS = 0x1E # Optimizer-owned: IMUL CF/OF from the DSP product.
ALUJMP_USTEP_FAULT_DONE = 0x22 # Optimizer-owned: fault delivery completion.
ALUJMP_PASS = 0x14
SRC_USTEP_SEG_INDEX = 0x3A  # Optimizer-owned: decoded segment as legacy IRF selector.
SRC_FOP = 0x3B              # Architectural x87 ESC/ModR/M command.
SRC_MDTMP = 0x1D            # Multiplier low product / quotient.

# field name -> (shift, width)
FIELDS = {
    'bus': (0, 6), 'sub': (6, 2), 'op': (8, 3), 'aluop': (11, 7),
    'src': (18, 6), 'dst': (24, 7), 'alusrc': (31, 6),
}


def set_fields(word: int, **kw: int) -> int:
    for name, val in kw.items():
        shift, width = FIELDS[name]
        mask = ((1 << width) - 1) << shift
        word = (word & ~mask) | ((val << shift) & mask)
    return word


@dataclass
class Patch:
    addr: int
    comment: str
    fields: dict | None = None    # override these fields on the base word
    copy_from: int | None = None  # set this address's word from another base word
                                  # (fields, if also given, are applied on top)
    word: int | None = None       # absolute 37-bit word


class EarlyKind(IntEnum):
    """D2 operation class encoded by the three ROM metadata bits."""

    SEQ = 0
    NONE = 1
    EA = 2
    LOAD = 3
    STORE = 4
    RMW = 5
    BRANCH = 6
    STACK = 7


class OverlayQualifier(IntEnum):
    """Structured decoder predicates for optimizer-owned entry overlays."""

    X87_M32_FLOAT = 1
    RMW_MEMORY = 2
    RMW_UNARY = 3
    X87_REG = 4
    X87_ST_M32 = 5


class RecipeAction(IntEnum):
    """Registered D2 actions selected by optimizer-owned entry addresses."""

    NONE = 0
    X87_OVERLAY = 1
    INVLPG = 2
    RMW_FAST = 3


@dataclass(frozen=True)
class Recipe:
    name: str
    entry: int
    early: EarlyKind
    legacy: tuple[tuple[int, ...], ...]
    targets: tuple[tuple[int, ...], ...]
    commit: str
    slot: str
    hazards: tuple[str, ...] = ()
    overlay_retire: bool = False


@dataclass(frozen=True)
class OverlayRecipe:
    """Qualified architectural entry redirected to optimizer-owned usteps."""

    name: str
    source_entry: int
    entry: int
    early: EarlyKind
    targets: tuple[int, ...]
    qualifier: OverlayQualifier
    action: RecipeAction
    commit: str
    hazards: tuple[str, ...] = ()
    retire_in_delay: bool = False


PATCHES = [
    # ---- 80486 instruction extensions -----------------------------------
    # The extracted 80386 PUSHFD routine first truncates EFLAGS to 16 bits at
    # 7F3, before 7F4 applies the architectural 0x37fd7 mask: on a 386 that
    # stores VM and RF as 0. A 486 must keep AC, so 7F3 masks with 0xFFFCFFFF
    # instead (ALUSRC 0x11, otherwise only a jump-target field): AC kept, VM
    # and RF stored as 0 (i486 PRM: PUSHFD reads RF as clear). Keeping this in
    # the generated z486 ROM avoids changing the shared MASK16 literal used by
    # interrupt and fault delivery.
    Patch(0x7F3, "486 PUSHFD: keep AC, store VM and RF as 0, before architectural mask",
          fields=dict(alusrc=0x11)),

    # 0F C8-CF has no 80386 PLA entry. The decoder redirects it to this
    # otherwise unused word and selects the register through opcode[2:0].
    Patch(0x9C4, "BSWAP r32 extension: SRCREG -> byte-swapped SRCREG + RNI",
          copy_from=0x003, fields=dict(dst=DEST_USTEP_BSWAP)),

    # XADD (0F C0/C1) and CMPXCHG (0F B0/B1) have no 80386 PLA entry; the
    # decoder selects these words by ModR/M form. Operands follow ALU r/m,r:
    # DSTREG is r/m, SRCREG is reg; the decoder supplies ADD for XADD, CMP for
    # CMPXCHG's CMPTST and condition Z for its JNcond.
    # XADD r,r: SRC gets the old DEST before DEST gets the sum, so XADD r,r
    # with one register leaves the sum.
    Patch(0x9D9, "XADD r,r: DSTREG -> TMPB, ALU DSTREG + SRCREG (flags)",
          copy_from=0x030, fields=dict(src=0x3D, dst=0x0B, alusrc=0x3E, aluop=0x00)),
    Patch(0x9DA, "XADD r,r: TMPB -> SRCREG + RNI",
          copy_from=0x030, fields=dict(src=0x0B, dst=0x3E, op=0)),
    Patch(0x9DB, "XADD r,r: SIGMA -> DSTREG in the RNI delay slot",
          copy_from=0x030, fields=dict(src=0x1E, dst=0x3D)),
    # XADD m,r: read for write (as ALU m,r), write the sum, and update SRC from
    # OPR_R only after the write has completed, as XCHG m,r does (an RNI delay
    # slot GPR write takes SIGMA, OPR_R, COUNTR or -1).
    Patch(0x9DC, "XADD m,r: flags backup + RD for write",
          copy_from=0x04A),
    Patch(0x9DD, "XADD m,r: DLY for the read",
          copy_from=0x030, fields=dict(sub=0)),
    Patch(0x9DE, "XADD m,r: ALU OPR_R + SRCREG (flags)",
          copy_from=0x030, fields=dict(src=0x2D, alusrc=0x3E, aluop=0x00)),
    Patch(0x9DF, "XADD m,r: SIGMA -> OPR_W + WR",
          copy_from=0x030, fields=dict(src=0x1E, dst=0x2D, bus=0x12)),
    Patch(0x9E0, "XADD m,r: RNI + DLY for the write",
          copy_from=0x030, fields=dict(op=0, sub=0)),
    Patch(0x9E1, "XADD m,r: OPR_R -> SRCREG + UNL in the RNI delay slot",
          copy_from=0x030, fields=dict(src=0x2D, dst=0x3E, sub=1)),
    # CMPXCHG r,r: compare the accumulator with DEST; equal stores SRC in
    # DEST, otherwise the accumulator takes DEST.
    Patch(0x9E2, "CMPXCHG r,r: CMPTST eAX_AL - DSTREG (flags)",
          copy_from=0x030, fields=dict(src=0x28, alusrc=0x3D, aluop=0x03)),
    Patch(0x9E3, "CMPXCHG r,r: JNcond (not equal) -> 9E7",
          copy_from=0x030, fields=dict(aluop=0x41, alusrc=0x03)),
    Patch(0x9E4, "CMPXCHG r,r: blank jump delay slot",
          copy_from=0x030),
    Patch(0x9E5, "CMPXCHG r,r equal: SRCREG -> DSTREG + RNI",
          copy_from=0x030, fields=dict(src=0x3E, dst=0x3D, op=0)),
    Patch(0x9E6, "CMPXCHG r,r equal: blank RNI delay slot",
          copy_from=0x030),
    Patch(0x9E7, "CMPXCHG r,r not equal: DSTREG -> eAX_AL + RNI",
          copy_from=0x030, fields=dict(src=0x3D, dst=0x28, op=0)),
    Patch(0x9E8, "CMPXCHG r,r not equal: blank RNI delay slot",
          copy_from=0x030),
    # CMPXCHG m,r always writes the destination: SRC when equal, the old
    # value otherwise (as the 486 does); the accumulator is updated only
    # after that write has completed.
    Patch(0x9E9, "CMPXCHG m,r: flags backup + RD for write",
          copy_from=0x04A),
    Patch(0x9EA, "CMPXCHG m,r: DLY for the read",
          copy_from=0x030, fields=dict(sub=0)),
    Patch(0x9EB, "CMPXCHG m,r: CMPTST eAX_AL - OPR_R (flags) + UNL (as ALU m,r 04C)",
          copy_from=0x030, fields=dict(src=0x28, alusrc=0x0F, aluop=0x03, sub=1)),
    Patch(0x9EC, "CMPXCHG m,r: SRCREG -> OPR_W, JNcond (not equal) -> 9F0",
          copy_from=0x030, fields=dict(src=0x3E, dst=0x2D, aluop=0x41, alusrc=0x03)),
    Patch(0x9ED, "CMPXCHG m,r: blank jump delay slot",
          copy_from=0x030),
    Patch(0x9EE, "CMPXCHG m,r equal: WR + RNI",
          copy_from=0x030, fields=dict(bus=0x12, op=0)),
    Patch(0x9EF, "CMPXCHG m,r equal: DLY for the write in the RNI delay slot",
          copy_from=0x030, fields=dict(sub=0)),
    Patch(0x9F0, "CMPXCHG m,r not equal: OPR_R -> OPR_W + WR",
          copy_from=0x030, fields=dict(src=0x2D, dst=0x2D, bus=0x12)),
    Patch(0x9F1, "CMPXCHG m,r not equal: RNI + DLY for the write",
          copy_from=0x030, fields=dict(op=0, sub=0)),
    Patch(0x9F2, "CMPXCHG m,r not equal: OPR_R -> eAX_AL in the RNI delay slot",
          copy_from=0x030, fields=dict(src=0x2D, dst=0x28)),
    # INVD/WBINVD (0F 08/09): CPL 0 only (as CLTS); the L1 caches are
    # write-through and coherent with every bus master, so there is nothing
    # to write back and discarding lines is not architecturally visible.
    Patch(0x9F3, "INVD/WBINVD: LJMPNP NO_PRIVILEGE (as CLTS)",
          copy_from=0x0DC),
    Patch(0x9F4, "INVD/WBINVD: RNI in the jump delay slot",
          copy_from=0x030, fields=dict(op=0)),
    Patch(0x9F5, "INVD/WBINVD: blank RNI delay slot",
          copy_from=0x030),

    # 0F 01 /7 uses an address operand but performs no data transfer. The CPU
    # sidecar serializes this RNI word with the paging unit and invalidates the
    # addressed TLB entry; the following blank word is its architectural delay
    # slot.
    Patch(0x9C7, "INVLPG m extension: paging-owned single-page invalidate + RNI",
          fields=dict(dst=DEST_USTEP_INVLPG, op=0)),
    Patch(0x9C8, "INVLPG m extension: blank RNI delay slot",
          copy_from=0x030),

    # Opcode 90 is architecturally XCHG EAX,EAX, but does not need the shared
    # three-word XCHG r,EAX routine.  D1 redirects only that opcode to a blank
    # hardwired RNI word; 91-97 retain the original exchange path.
    Patch(0x9C9, "NOP extension: blank hardwired RNI word",
          copy_from=0x030, fields=dict(op=0)),
    Patch(0x9CA, "NOP extension: blank RNI delay slot",
          copy_from=0x030),

    # D8 m32 arithmetic and D9 /0 FLD use a paging-owned demand read and post
    # the completed operand directly to the integrated x87. Dynamic CR0/x87
    # eligibility falls back to the original 4D7 routine.
    Patch(0x9C5, "x87 m32 direct-load overlay: wait for fault-checked operand",
          copy_from=0x20E),
    Patch(0x9C6, "x87 m32 direct-load overlay: retire after x87 queue accepts operand",
          copy_from=0x20F),

    # Register-form FP instructions (4C1) post their FOP straight to the
    # integrated x87; hardware records FIP/FCS/FOP (the 4CE/4CF/4D0 moves) when
    # the direct path is taken, so neither word writes state and the fallback
    # to the original 4C1 routine is clean.
    Patch(0x9D1, "x87 register direct overlay: command posted at issue",
          copy_from=0x20E),
    Patch(0x9D2, "x87 register direct overlay: retire",
          copy_from=0x20F),

    # FST/FSTP/FIST/FISTP m32 (53C) in 32-bit code: check the destination for
    # writing first (FSTP pops, so the store must not fault after the
    # command), then hardware posts the FOP and reads the result into OPR_R,
    # and the word after writes it back. The first two words write nothing,
    # so the fallback to 53C (whose protection test raises #NM first) is clean.
    Patch(0x9D3, "x87 m32 store overlay: fallback point",
          copy_from=0x20E),
    Patch(0x9D4, "x87 m32 store overlay: fallback delay slot",
          copy_from=0x20E),
    Patch(0x9D5, "x87 m32 store overlay: check the destination for writing",
          copy_from=0x030, fields=dict(bus=0x0A)),
    Patch(0x9D6, "x87 m32 store overlay: after the check (DLY), command and result read into OPR_R",
          copy_from=0x030, fields=dict(dst=DEST_USTEP_X87_STORE, sub=0)),
    Patch(0x9D7, "x87 m32 store overlay: write the result back and retire",
          copy_from=0x0AE, fields=dict(op=0)),
    Patch(0x9D8, "x87 m32 store overlay: RNI delay slot waits for the write",
          copy_from=0x047),

    # Cached RMW alternate entries. RD_FAST/WR_FAST are semantic actions owned
    # by the entry, while the ordinary fields keep ALU, flags, OPR_R/OPR_W and
    # RNI behavior in the microcode/data-unit path. Rejected RD_FAST probes
    # redirect to the untouched 04A/04E routines.
    Patch(0x9CB, "ALU m,r RD_FAST read_write entry",
          copy_from=0x04A, fields=dict(bus=0x3F)),
    Patch(0x9CC, "ALU m,r fast ALU from OPR_R plus RNI",
          copy_from=0x04D, fields=dict(src=0x2D, op=0)),
    Patch(0x9CD, "ALU m,r WR_FAST in RNI delay slot",
          copy_from=0x046, fields=dict(bus=0x3F, op=7)),
    Patch(0x9CE, "unary memory RD_FAST read_write entry",
          copy_from=0x04E, fields=dict(bus=0x3F, alusrc=0x3E)),
    Patch(0x9CF, "unary memory fast ALU from OPR_R plus RNI",
          copy_from=0x050, fields=dict(op=0)),
    Patch(0x9D0, "unary memory WR_FAST in RNI delay slot",
          copy_from=0x046, fields=dict(bus=0x3F, op=7)),

    # ---- Original 386 microcode repairs ---------------------------------
    # BSR's loop leaves the final bit index in TMPC. The extracted routine's
    # 182 CLZF/RNI word does not encode the architectural destination write,
    # which previously required an opcode-specific RTL writeback. Make the
    # write part of the microcode word so normal destination handling owns it.
    Patch(0x182, "BSR completion: TMPC -> SRCREG with CLZF/RNI",
          fields=dict(src=SRC_TMPC, dst=DEST_SRCREG)),

    # RPTI's EIP restore restarts an interrupted repeat instruction. Give the
    # write its own semantic destination instead of recognizing routine
    # addresses in the CPU control path.
    Patch(0x20D, "RPTI restart: TMPeIP -> restart EIP and prefetch",
          fields=dict(dst=DEST_USTEP_RPTI_EIP)),

    # A task switch establishes CPL from the incoming CS selector. Ordinary
    # protected-mode CS writes retain the current RPL, so distinguish this
    # architectural task-load write in the generated microcode.
    Patch(0x76F, "LOAD_TASK: incoming selector establishes full CS including RPL",
          fields=dict(dst=DEST_USTEP_TASK_CS)),

    # Descriptor loading writes the Accessed bit back only when it was clear.
    # Express the branch condition in the micro-op rather than recognizing the
    # original routine address in the sequencer path.
    Patch(0x5D3, "LD_DESCRIPTOR: jump when descriptor Accessed bit is already set",
          fields=dict(aluop=ALUJMP_JDESCA)),

    # Fault delivery has three completion paths with different useful fields.
    # Mark the two NOPMOVE words through ALU/JMP and the task-gate OR word
    # through its otherwise-empty destination, producing one semantic event.
    Patch(0x639, "cross-privilege interrupt delivery completion",
          fields=dict(aluop=ALUJMP_USTEP_FAULT_DONE)),
    Patch(0x7E0, "task-gate interrupt delivery completion",
          fields=dict(dst=DEST_USTEP_FAULT_DONE)),
    Patch(0x8E3, "ordinary interrupt/fault delivery completion",
          fields=dict(aluop=ALUJMP_USTEP_FAULT_DONE)),

    # LSS/LFS/LGS originally borrow IMM to carry the second opcode byte, then
    # XOR it with 0x10 to form the segment IRF selector. Use the decoder's
    # segment selection directly, leaving the architectural immediate field
    # exclusively for instruction operands.
    Patch(0x0C9, "LSS: decoded segment target replaces opcode-in-IMM XOR",
          fields=dict(src=SRC_USTEP_SEG_INDEX, alusrc=0, aluop=ALUJMP_PASS)),
    Patch(0x0D0, "LFS/LGS: decoded segment target replaces opcode-in-IMM XOR",
          fields=dict(src=SRC_USTEP_SEG_INDEX, alusrc=0, aluop=ALUJMP_PASS)),
    Patch(0x5C5, "LSS/LFS/LGS descriptor load: retain decoded segment target",
          fields=dict(src=SRC_USTEP_SEG_INDEX, alusrc=0, aluop=ALUJMP_PASS)),

    # x87 command transport uses the architectural OPCODE/FOP source rather
    # than borrowing the instruction immediate field. Every IMM read in the
    # original x87 region observed that borrowed FOP value.
    Patch(0x3D0, "x87 flag command: decoded FOP -> OPR_W",
          fields=dict(src=SRC_FOP)),
    Patch(0x3E2, "x87 misc command setup: decoded FOP source",
          fields=dict(src=SRC_FOP)),
    Patch(0x3E3, "x87 misc command: decoded FOP -> OPR_W",
          fields=dict(src=SRC_FOP)),
    Patch(0x404, "x87 save command: decoded FOP -> OPR_W",
          fields=dict(src=SRC_FOP)),
    Patch(0x478, "x87 restore command: decoded FOP -> OPR_W",
          fields=dict(src=SRC_FOP)),
    Patch(0x4CC, "x87 register command setup: decoded FOP source",
          fields=dict(src=SRC_FOP)),
    Patch(0x4CD, "x87 register command: decoded FOP -> OPR_W",
          fields=dict(src=SRC_FOP)),
    Patch(0x500, "x87 load command: decoded FOP -> OPR_W",
          fields=dict(src=SRC_FOP)),
    Patch(0x507, "x87 load loop: retain decoded FOP",
          fields=dict(src=SRC_FOP)),
    Patch(0x51E, "x87 load command: decoded FOP -> OPR_W",
          fields=dict(src=SRC_FOP)),
    Patch(0x531, "x87 short load: decoded FOP -> TMPF",
          fields=dict(src=SRC_FOP)),
    Patch(0x566, "x87 store command: decoded FOP -> OPR_W",
          fields=dict(src=SRC_FOP)),
    Patch(0x56A, "x87 store abort: decoded FOP -> TMPF",
          fields=dict(src=SRC_FOP)),

    # AAD's final ADC must behave as ADD regardless of the incoming carry.
    # Mark its preceding barrel step explicitly instead of recognizing opcode
    # D5 inside the data unit's shared SHIFT implementation.
    Patch(0x1A0, "AAD shift: barrel result plus explicit carry clear",
          fields=dict(aluop=ALUJMP_USTEP_AAD_SHIFT)),

    # REP STOS has already decremented COUNTR at 267 and records the precise
    # architectural count beside the EDI update at 269. Loop directly to the
    # address step while count remains nonzero and no interrupt is pending.
    Patch(0x268, "REP STOS: skip redundant per-element count check",
          fields=dict(alusrc=0x3D, aluop=0x47)),

    # ---- IMUL r,r/m(,imm) 17 -> 12 cycles -----------------------------
    # The 80386 shift-and-add multiplier left the product split across
    # SIGMA/MDTMP and realigned it in six words (1B6-1BB: LDBSRM, two SHIFTs,
    # IMCS). The DSP multiplier delivers the low product (MDTMP source) and
    # the high product (SIGMA) already aligned when the 1B5 IMUL4 repeat
    # exits, so retire straight away: SRCREG takes the low product and the
    # new micro-op sets CF = OF = (high product != sign extension of the low
    # one), as IMCS did. SF/ZF/AF/PF stay unchanged as before.
    #   patched:  1B5 IMUL4 RPT / 1B6 MDTMP->SRCREG IMUL_FLAGS RNI / 1B7 slot
    #   was:      1B5 IMUL4 RPT / 1B6-1BB realign + IMCS + RNI / 1BC slot
    Patch(0x1B6, "IMUL2_COMMON: low product -> SRCREG, CF/OF from the product, RNI",
          copy_from=0x030, fields=dict(src=SRC_MDTMP, dst=DEST_SRCREG,
                                       aluop=ALUJMP_USTEP_IMUL_FLAGS, op=0)),
    Patch(0x1B7, "IMUL2_COMMON: blank RNI delay slot",
          copy_from=0x030),

    # ---- direct ALU usteps -------------------------------------------
    # Every hardwired ALU retire word owns its architectural write through one
    # destination encoding. This replaces the parallel RECIPE_COMMIT_ALU write
    # site while leaving SEQ/hardwired_off to execute the original slot writeback.
    Patch(0x003, "MOV r,r ustep: commit entry ALU result to DSTREG",
          fields=dict(dst=DEST_USTEP_ALU)),
    Patch(0x005, "MOV r,imm ustep: commit entry ALU result to DSTREG",
          fields=dict(dst=DEST_USTEP_ALU)),
    Patch(0x01D, "ALU r,r ustep: commit entry ALU result to DSTREG",
          fields=dict(dst=DEST_USTEP_ALU)),
    Patch(0x021, "INC/DEC/NOT/NEG r ustep: commit entry ALU result to DSTREG",
          fields=dict(dst=DEST_USTEP_ALU)),
    Patch(0x023, "ALU r,imm ustep: commit entry ALU result to DSTREG",
          fields=dict(dst=DEST_USTEP_ALU)),

    # ---- Load (MOV r,m) 4 -> 3 cycles --------------------------------------
    # The PIPT dcache returns the read data by 01A: with the modrm linear
    # registered at i_issue, the 019 RD is issued/accepted at i_first and
    # resp_valid lands the next cycle (01A).  The original routine then spends
    # two more cycles -- 01B (bare RNI) and 01C (the result write).  Fold RNI
    # into the 01A DLY (exactly POP's 0A0 "RNI DLY") and move the result write
    # up into 01B, the RNI delay slot.  01C becomes unreached.
    #   patched:  019 RD / 01A RNI DLY / 01B OPR_R->DSTREG
    #   was:      019 RD / 01A DLY / 01B RNI / 01C OPR_R->DSTREG
    Patch(0x01A, "MOV r,m load 4->3: 01A DLY -> RNI DLY (data is back by 01A)",
          fields=dict(op=0)),
    Patch(0x01B, "MOV r,m load 4->3: 01B RNI -> OPR_R->DSTREG (write in RNI delay slot)",
          copy_from=0x01C),

    # ---- ALU r,m 4 -> 3 cycles (M5 F-ALUM) --------------------------------
    # The base routine stages the memory operand through TMPB (029 OPR_R->TMPB)
    # only because the alusrc field has no OPR_R encoding.  The m,r CMPTST
    # forms (033/037) already read OPR_R directly via the source field; the
    # z486 hardware adds ALUSRC_OPR_R (0x0F, unused in the base CROM as a
    # consumed ALU source) to complete the symmetry for r,m.
    #   patched:  027 RD / 028 DLY / 029 DSTREG(op)OPR_R +-&|^ RNI / 02A slot SIGMA->DSTREG
    #   was:      027 RD / 028 DLY / 029 OPR_R->TMPB / 02A DSTREG(op)TMPB RNI / 02B slot
    # hardwired shape is unchanged (multi_ustep, commit_sel=ALU): the RNI word moves
    # one earlier, chainN keys on uc_next content, the ALU-commit sideband
    # keys on the RNI word's aluop.  SEQ/hardwired_off path speeds up identically.
    Patch(0x029, "ALU r,m 4->3: 029 = ALU DSTREG,OPR_R +-&|^ RNI (was OPR_R->TMPB)",
          copy_from=0x02A, fields=dict(alusrc=0x0F, dst=DEST_USTEP_ALU)),
    Patch(0x02A, "ALU r,m 4->3: 02A = SIGMA->DSTREG (slot writeback; was the ALU word)",
          copy_from=0x02B),

    # ---- CMP r,m 4 -> 3 cycles (M5 F-ALUM, CMPTST flavor) ------------------
    #   patched:  02C RD / 02D DLY / 02E DSTREG,OPR_R CMPTST RNI / 02F blank slot
    #   was:      02C RD / 02D DLY / 02E OPR_R->TMPB / 02F DSTREG,TMPB CMPTST RNI
    # Only 3A/3B (CMP r,m, group 0x05) enter 02C; CMP m,r (38/39, group 0x04)
    # has its own already-folded routine at 035.  02F must become a blank slot
    # word: the old CMPTST there would redo flags from stale TMPB in SEQ mode.
    Patch(0x02E, "CMP r,m 4->3: 02E = CMPTST DSTREG,OPR_R RNI (was OPR_R->TMPB)",
          copy_from=0x02F, fields=dict(alusrc=0x0F)),
    Patch(0x02F, "CMP r,m 4->3: 02F = blank slot (was the CMPTST word)",
          copy_from=0x030),

    # ---- ALU m,r / m,imm RMW 6 -> 4 cycles (M5 F-RMW) ----------------------
    # Retime into the INC/DEC-m shape (04E/04F/050), which the base CROM
    # already uses: move the JMP WRITE_RESULT up into the DLY word (fires at
    # DLY release), and do the ALU in the jump delay slot reading OPR_R
    # directly via the source field (dst port = the memory operand, correct
    # m,r operand order).  The shared 046 SIGMA->OPR_W+WR+RNI / 047 DLY tail
    # and the 04A/039 FLGSBA restart backup are untouched, so write-fault
    # restart semantics are identical.  Stays SEQ (not hardwired-classified).
    #   patched:  04A FLGSBA+RD / 04B DLY+JMP(046) / 04C OPR_R,SRCREG +-&|^ / 046 WR+RNI / 047
    #   was:      04A FLGSBA+RD / 04B DLY / 04C OPR_R->TMPB+JMP / 04D TMPB,SRCREG / 046 / 047
    # Jump offset: reljump target = uaddr + sext6(alusrc) with uaddr already
    # at word+1, so 04B: 0x46-0x4C = -6 = 0x3A; 03A: 0x46-0x3B = +11 = 0x0B.
    Patch(0x04B, "ALU m,r 6->4: 04B = DLY + JMP WRITE_RESULT (was pure DLY)",
          fields=dict(aluop=0x5A, alusrc=0x3A)),
    Patch(0x04C, "ALU m,r 6->4: 04C = OPR_R,SRCREG +-&|^ in jump delay slot (04D unreached)",
          copy_from=0x04D, fields=dict(src=0x2D)),
    Patch(0x03A, "ALU m,i 6->4: 03A = DLY + JMP WRITE_RESULT (was pure DLY)",
          fields=dict(aluop=0x5A, alusrc=0x0B)),
    Patch(0x03B, "ALU m,i 6->4: 03B = OPR_R,IMM +-&|^ in jump delay slot (03C unreached)",
          copy_from=0x03C, fields=dict(src=0x2D)),
]


# Current hardwired routines and their bounded target recipes. `legacy` is
# descriptive: it records the current general-sequencer control paths.
# `targets` are explicit recipes and must contain 1..3 logical
# usteps.  A memory ustep may hold for completion, absorbing legacy DLY/JMP
# plumbing without increasing the target step count.
HARDWIRED_RECIPES = [
    Recipe("nop", 0x9C9, EarlyKind.NONE,
           ((0x9C9,),), ((0x9C9,),), "none", "reclaim"),
    Recipe("mov-r-r", 0x003, EarlyKind.NONE,
           ((0x003, 0x004),), ((0x003,),), "alu-dst", "reclaim", ("src",)),
    Recipe("mov-r-imm", 0x005, EarlyKind.NONE,
           ((0x005, 0x006),), ((0x005,),), "alu-dst", "reclaim"),
    Recipe("alu-r-r", 0x01D, EarlyKind.NONE,
           ((0x01D, 0x01E),), ((0x01D,),), "alu-dst/flags", "reclaim",
           ("dst", "src", "flags-adc-sbb")),
    Recipe("cmp-test-r-r", 0x01F, EarlyKind.NONE,
           ((0x01F, 0x020),), ((0x01F,),), "flags", "reclaim", ("dst", "src")),
    Recipe("inc-dec-not-neg-r", 0x021, EarlyKind.NONE,
           ((0x021, 0x022),), ((0x021,),), "alu-dst/flags", "reclaim", ("dst",)),
    Recipe("alu-r-imm", 0x023, EarlyKind.NONE,
           ((0x023, 0x024),), ((0x023,),), "alu-dst/flags", "reclaim",
           ("dst", "flags-adc-sbb")),
    Recipe("cmp-test-r-imm", 0x025, EarlyKind.NONE,
           ((0x025, 0x026),), ((0x025,),), "flags", "reclaim", ("dst",)),
    Recipe("lea", 0x0B9, EarlyKind.EA,
           ((0x0B9, 0x0BA),), ((0x0B9,),), "src-reg", "reclaim", ("ea",)),

    Recipe("shift-r-imm", 0x0F9, EarlyKind.NONE,
           ((0x0F9, 0x0FA, 0x0FB),), ((0x0F9, 0x0FA),), "shift-dst/flags",
           "reclaim", ("dst",)),
    Recipe("shift-r-cl", 0x0FF, EarlyKind.NONE,
           ((0x0FF, 0x100, 0x101),), ((0x0FF, 0x100),), "shift-dst/flags",
           "reclaim", ("dst", "ecx")),
    Recipe("shxd-r-imm", 0x0FC, EarlyKind.NONE,
           ((0x0FC, 0x0FD, 0x0FE),), ((0x0FC, 0x0FD),), "shift-dst/flags",
           "reclaim", ("dst", "src")),
    Recipe("shxd-r-cl", 0x102, EarlyKind.NONE,
           ((0x102, 0x103, 0x104),), ((0x102, 0x103),), "shift-dst/flags",
           "reclaim", ("dst", "src", "ecx")),
    Recipe("shift-r-one", 0x105, EarlyKind.NONE,
           ((0x105, 0x106, 0x107),), ((0x105, 0x106),), "shift-dst/flags",
           "reclaim", ("dst",)),
    Recipe("szext-r-16", 0x1E8, EarlyKind.NONE,
           ((0x1E8, 0x1E9, 0x1EA),), ((0x1E8, 0x1E9),), "sigma-src", "reclaim",
           ("dst",)),
    Recipe("szext-r-32", 0x1F0, EarlyKind.NONE,
           ((0x1F0, 0x1F1, 0x1F2),), ((0x1F0, 0x1F1),), "sigma-src", "reclaim",
           ("dst",)),

    Recipe("store-r", 0x013, EarlyKind.STORE,
           ((0x013, 0x014),), ((0x013,),), "store", "retain", ("ea", "src")),
    Recipe("store-imm", 0x015, EarlyKind.STORE,
           ((0x015, 0x016),), ((0x015,),), "store", "retain", ("ea",)),
    Recipe("load-r", 0x019, EarlyKind.LOAD,
           ((0x019, 0x01A, 0x01B),), ((0x019, 0x01A),), "mem-dst", "retain",
           ("ea",)),
    Recipe("alu-r-m", 0x027, EarlyKind.LOAD,
           ((0x027, 0x028, 0x029, 0x02A),), ((0x027, 0x029),), "alu-dst/flags",
           "reclaim", ("ea", "dst", "flags-adc-sbb")),
    Recipe("cmp-r-m", 0x02C, EarlyKind.LOAD,
           ((0x02C, 0x02D, 0x02E, 0x02F),), ((0x02C, 0x02E),), "flags",
           "reclaim", ("ea",)),
    Recipe("cmp-test-m-imm", 0x031, EarlyKind.LOAD,
           ((0x031, 0x032, 0x033, 0x034),), ((0x031, 0x033),), "flags",
           "reclaim", ("ea",)),
    Recipe("cmp-test-m-r", 0x035, EarlyKind.LOAD,
           ((0x035, 0x036, 0x037, 0x038),), ((0x035, 0x037),), "flags",
           "reclaim", ("ea", "src")),
    Recipe("rmw-m-imm", 0x039, EarlyKind.RMW,
           ((0x039, 0x03A, 0x03B, 0x046, 0x047),), ((0x039, 0x03B, 0x046),),
           "store/flags", "retain", ("ea",)),
    Recipe("rmw-m-r", 0x04A, EarlyKind.RMW,
           ((0x04A, 0x04B, 0x04C, 0x046, 0x047),), ((0x04A, 0x04C, 0x046),),
           "store/flags", "retain", ("ea", "src")),
    Recipe("szext-m-16", 0x1EB, EarlyKind.LOAD,
           ((0x1EB, 0x1EC, 0x1ED, 0x1EE, 0x1EF),), ((0x1EB, 0x1ED, 0x1EE),),
           "sigma-src", "reclaim", ("ea",)),
    Recipe("szext-m-32", 0x1F3, EarlyKind.LOAD,
           ((0x1F3, 0x1F4, 0x1F5, 0x1F6, 0x1F7),), ((0x1F3, 0x1F5, 0x1F6),),
           "sigma-src", "reclaim", ("ea",)),

    Recipe("jcc-rel", 0x065, EarlyKind.BRANCH,
           ((0x065, 0x066), (0x065, 0x067, 0x068)),
           ((0x065,), (0x065, 0x068)), "eip", "conditional", ("flags",), True),
    Recipe("jmp-rel", 0x06A, EarlyKind.BRANCH,
           ((0x06A, 0x06B, 0x06C, 0x067, 0x068),), ((0x06A, 0x068),),
           "eip", "reclaim"),
    Recipe("call-rel", 0x075, EarlyKind.STACK,
           ((0x075, 0x076, 0x077, 0x078, 0x067, 0x068),),
           ((0x075, 0x077, 0x068),), "store/esp/eip", "reclaim", ("stack",)),
    Recipe("ret-near", 0x072, EarlyKind.STACK,
           ((0x072, 0x073, 0x074, 0x06B, 0x06C, 0x067, 0x068),),
           ((0x072, 0x074, 0x068),), "esp/eip", "reclaim", ("stack",)),
    Recipe("push-r", 0x086, EarlyKind.STACK,
           ((0x086, 0x087),), ((0x086,),), "store/esp", "retain", ("stack", "dst")),
    Recipe("push-seg", 0x09B, EarlyKind.STACK,
           ((0x09B, 0x09C),), ((0x09B,),), "store/esp", "retain", ("stack",)),
    Recipe("push-imm", 0x09D, EarlyKind.STACK,
           ((0x09D, 0x09E),), ((0x09D,),), "store/esp", "retain", ("stack",)),
    Recipe("pop-r", 0x09F, EarlyKind.STACK,
           ((0x09F, 0x0A0, 0x0A1),), ((0x09F, 0x0A0),), "mem-dst/esp", "retain",
           ("stack",)),
]


OVERLAY_RECIPES = [
    OverlayRecipe("x87-m32-load", 0x4D7, 0x9C5, EarlyKind.LOAD,
                  (0x9C5, 0x9C6), OverlayQualifier.X87_M32_FLOAT,
                  RecipeAction.X87_OVERLAY, "x87-direct-m32",
                  ("ea", "paging", "x87-order")),
    OverlayRecipe("x87-reg", 0x4C1, 0x9D1, EarlyKind.NONE,
                  (0x9D1, 0x9D2), OverlayQualifier.X87_REG,
                  RecipeAction.X87_OVERLAY, "x87-direct-reg",
                  ("x87-order",)),
    OverlayRecipe("x87-st-m32", 0x53C, 0x9D3, EarlyKind.SEQ,
                  (0x9D3, 0x9D4, 0x9D5, 0x9D6, 0x9D7, 0x9D8), OverlayQualifier.X87_ST_M32,
                  RecipeAction.X87_OVERLAY, "x87-direct-store",
                  ("ea", "paging", "x87-order"), True),
    OverlayRecipe("rmw-m-r-fast", 0x04A, 0x9CB, EarlyKind.RMW,
                  (0x9CB, 0x9CC, 0x9CD), OverlayQualifier.RMW_MEMORY,
                  RecipeAction.RMW_FAST, "store/flags",
                  ("ea", "paging", "store-order"), True),
    OverlayRecipe("rmw-unary-fast", 0x04E, 0x9CE, EarlyKind.RMW,
                  (0x9CE, 0x9CF, 0x9D0), OverlayQualifier.RMW_UNARY,
                  RecipeAction.RMW_FAST, "store/flags",
                  ("ea", "paging", "store-order"), True),
]

# Semantic execution actions for ordinary generated microcode entries.  Keep
# literal entry addresses in this generated decode table; execution registers
# and consumes only the action.
ENTRY_ACTIONS = {
    0x9C7: RecipeAction.INVLPG,
}


def read_words(path: Path) -> list[int]:
    words: list[int] = []
    for lineno, raw in enumerate(path.read_text().splitlines(), start=1):
        line = raw.split('//', 1)[0].split('#', 1)[0].strip()
        if not line:
            continue
        word = int(line, 16)
        if not 0 <= word < (1 << UCODE_BITS):
            raise ValueError(f"{path}:{lineno}: word out of {UCODE_BITS}-bit range: 0x{word:x}")
        words.append(word)
    if len(words) != BASE_DEPTH:
        raise ValueError(f"{path}: expected {BASE_DEPTH} words, found {len(words)}")
    # Words beyond the extracted CROM start as the CROM's unused-word value.
    return words + [(1 << UCODE_BITS) - 1] * (ROM_DEPTH - BASE_DEPTH)


def render_hex(words: list[int]) -> str:
    return ''.join(f"{w:010X}\n" for w in words)


def render_mif(words: list[int]) -> str:
    lines = [f"WIDTH={ROM_BITS};", f"DEPTH={ROM_DEPTH};", "",
             "ADDRESS_RADIX=HEX;", "DATA_RADIX=HEX;", "", "CONTENT BEGIN"]
    lines += [f"    {a:03X} : {w:010X};" for a, w in enumerate(words)]
    lines.append("END;")
    return "\n".join(lines) + "\n"


def apply_patches(base: list[int]) -> list[int]:
    words = base[:]
    print(f"Applying {len(PATCHES)} microcode patch(es):")
    for p in PATCHES:
        old = words[p.addr]
        if p.word is not None:
            new = p.word
        elif p.copy_from is not None:
            new = base[p.copy_from]
            if p.fields is not None:
                new = set_fields(new, **p.fields)
        elif p.fields is not None:
            new = set_fields(old, **p.fields)
        else:
            raise ValueError(f"patch at 0x{p.addr:03X} has no action")
        words[p.addr] = new
        print(f"  0x{p.addr:03X}: {old:010X} -> {new:010X}  {p.comment}")
    return words


def annotate_recipes(words: list[int]) -> list[int]:
    """Attach the D2 early kind to each hardwired entry word."""
    annotated = words[:]
    for recipe in HARDWIRED_RECIPES:
        annotated[recipe.entry] |= int(recipe.early) << UCODE_BITS
    for recipe in OVERLAY_RECIPES:
        annotated[recipe.entry] |= int(recipe.early) << UCODE_BITS
    if any(word >= (1 << ROM_BITS) for word in annotated):
        raise ValueError(f"annotated word exceeds {ROM_BITS}-bit ROM width")
    return annotated


def get_field(word: int, name: str) -> int:
    shift, width = FIELDS[name]
    return (word >> shift) & ((1 << width) - 1)


def fmt_path(path: tuple[int, ...]) -> str:
    return " ".join(f"{addr:03X}" for addr in path)


def validate_recipes(words: list[int]) -> None:
    default_word = (1 << UCODE_BITS) - 1
    entries: dict[int, str] = {}

    for recipe in HARDWIRED_RECIPES:
        if recipe.entry in entries:
            raise ValueError(
                f"recipe {recipe.name}: entry 0x{recipe.entry:03X} already used by "
                f"{entries[recipe.entry]}"
            )
        entries[recipe.entry] = recipe.name

        if recipe.early == EarlyKind.SEQ:
            raise ValueError(f"recipe {recipe.name}: hardwired recipe cannot use SEQ early kind")
        if not recipe.legacy or not recipe.targets:
            raise ValueError(f"recipe {recipe.name}: legacy and target paths are required")

        for label, paths in (("legacy", recipe.legacy), ("target", recipe.targets)):
            for path in paths:
                if not path or path[0] != recipe.entry:
                    raise ValueError(
                        f"recipe {recipe.name}: {label} path must start at entry "
                        f"0x{recipe.entry:03X}: {fmt_path(path)}"
                    )
                if label == "target" and not 1 <= len(path) <= 3:
                    raise ValueError(
                        f"recipe {recipe.name}: target has {len(path)} usteps, expected 1..3"
                    )
                for addr in path:
                    if not 0 <= addr < ROM_DEPTH:
                        raise ValueError(f"recipe {recipe.name}: address 0x{addr:X} out of ROM")
                    # A legacy path may intentionally execute a blank delay
                    # slot.  Target usteps must all perform explicit work.
                    if label == "target" and words[addr] == default_word:
                        raise ValueError(
                            f"recipe {recipe.name}: address 0x{addr:03X} is an unused ROM word"
                        )

        if not recipe.overlay_retire:
            for path in recipe.targets:
                if get_field(words[path[-1]], "op") != 0:
                    raise ValueError(
                        f"recipe {recipe.name}: final target word 0x{path[-1]:03X} is not RNI"
                    )

    overlay_entries: set[int] = set()
    for recipe in OVERLAY_RECIPES:
        if recipe.entry in entries or recipe.entry in overlay_entries:
            raise ValueError(f"overlay {recipe.name}: duplicate entry 0x{recipe.entry:03X}")
        if recipe.source_entry == recipe.entry:
            raise ValueError(f"overlay {recipe.name}: source and overlay entries match")
        if recipe.action == RecipeAction.NONE:
            raise ValueError(f"overlay {recipe.name}: invalid action {recipe.action}")
        if not 1 <= len(recipe.targets) <= 6 or recipe.targets[0] != recipe.entry:
            raise ValueError(f"overlay {recipe.name}: target must be 1..6 words from its entry")
        for addr in recipe.targets:
            if not 0 <= addr < ROM_DEPTH or words[addr] == default_word:
                raise ValueError(f"overlay {recipe.name}: invalid target word 0x{addr:03X}")
        rni_addr = recipe.targets[-2] if recipe.retire_in_delay else recipe.targets[-1]
        if get_field(words[rni_addr], "op") != 0:
            raise ValueError(f"overlay {recipe.name}: retirement word is not RNI")
        overlay_entries.add(recipe.entry)

    for entry, action in ENTRY_ACTIONS.items():
        if not 0 <= entry < ROM_DEPTH or words[entry] == default_word:
            raise ValueError(f"entry action {action.name}: invalid word 0x{entry:03X}")
        if action == RecipeAction.NONE:
            raise ValueError(f"entry action 0x{entry:03X}: invalid action {action}")
        if (action == RecipeAction.INVLPG and
                get_field(words[entry], "dst") != DEST_USTEP_INVLPG):
            raise ValueError(
                f"entry action 0x{entry:03X}: INVLPG marker is missing from microcode"
            )


def render_recipe_manifest(words: list[int]) -> str:
    validate_recipes(words)
    lines = [
        "# z486 hardwired-instruction recipe manifest",
        "",
        "Generated by `scripts/ucode_optimize.py`; do not edit manually.",
        "Legacy paths describe the general sequencer; target paths are bounded",
        "hardwired recipes. A memory uStep may",
        "hold while its request completes.",
        "",
        "| recipe | entry | early | target usteps | legacy paths | commit | slot | hazards |",
        "| --- | ---: | --- | --- | --- | --- | --- | --- |",
    ]
    for recipe in HARDWIRED_RECIPES:
        targets = " / ".join(fmt_path(path) for path in recipe.targets)
        legacy = " / ".join(fmt_path(path) for path in recipe.legacy)
        hazards = ", ".join(recipe.hazards) if recipe.hazards else "-"
        lines.append(
            f"| `{recipe.name}` | `{recipe.entry:03X}` | `{recipe.early.name}` | "
            f"`{targets}` | `{legacy}` | `{recipe.commit}` | `{recipe.slot}` | "
            f"{hazards} |"
        )
    lines += [
        "",
        f"Recipes: {len(HARDWIRED_RECIPES)}. Native microcode remains {UCODE_BITS}-bit.",
        "The generated 40-bit ROM image stores the D2 early kind in bits 39:37.",
        "",
        "## Qualified overlays",
        "",
        "| overlay | architectural entry | effective entry | action | target usteps | hazards |",
        "| --- | ---: | ---: | --- | --- | --- |",
    ]
    for recipe in OVERLAY_RECIPES:
        hazards = ", ".join(recipe.hazards) if recipe.hazards else "-"
        lines.append(
            f"| `{recipe.name}` | `{recipe.source_entry:03X}` | `{recipe.entry:03X}` | "
            f"`{recipe.action.name}` | `{' '.join(f'{a:03X}' for a in recipe.targets)}` | "
            f"{hazards} |"
        )
    lines += ["", f"Qualified overlays: {len(OVERLAY_RECIPES)}.", ""]
    return "\n".join(lines)


def render_recipe_svh(words: list[int]) -> str:
    validate_recipes(words)
    recipes = {recipe.name: recipe for recipe in HARDWIRED_RECIPES}
    emitted: set[str] = set()

    def recipe_entries(*names: str) -> str:
        emitted.update(names)
        return ", ".join(f"12'h{recipes[name].entry:03X}" for name in names)

    def overlay_qualifier_expr(recipe: OverlayRecipe) -> str:
        if recipe.qualifier == OverlayQualifier.X87_M32_FLOAT:
            return "(opcode == 8'hD8) || ((opcode == 8'hD9) && (modrm[5:3] == 3'd0))"
        if recipe.qualifier == OverlayQualifier.X87_REG:
            return "modrm[7:6] == 2'b11"
        if recipe.qualifier == OverlayQualifier.X87_ST_M32:
            return "(modrm[7:6] != 2'b11) && ((opcode == 8'hD9) || (opcode == 8'hDB)) && " \
                   "((modrm[5:3] == 3'd2) || (modrm[5:3] == 3'd3))"
        if recipe.qualifier == OverlayQualifier.RMW_UNARY:
            return "(modrm[7:6] != 2'b11) && ((((opcode == 8'hF6) || (opcode == 8'hF7)) && " \
                   "((modrm[5:3] == 3'd2) || (modrm[5:3] == 3'd3))) || " \
                   "(((opcode == 8'hFE) || (opcode == 8'hFF)) && " \
                   "((modrm[5:3] == 3'd0) || (modrm[5:3] == 3'd1))))"
        if recipe.qualifier == OverlayQualifier.RMW_MEMORY:
            return "modrm[7:6] != 2'b11"
        raise ValueError(f"overlay {recipe.name}: unhandled qualifier {recipe.qualifier}")

    lines = [
        "// Generated by scripts/ucode_optimize.py; do not edit.",
        "localparam logic [2:0] RECIPE_EARLY_SEQ    = 3'd0;",
        "localparam logic [2:0] RECIPE_EARLY_NONE   = 3'd1;",
        "localparam logic [2:0] RECIPE_EARLY_EA     = 3'd2;",
        "localparam logic [2:0] RECIPE_EARLY_LOAD   = 3'd3;",
        "localparam logic [2:0] RECIPE_EARLY_STORE  = 3'd4;",
        "localparam logic [2:0] RECIPE_EARLY_RMW    = 3'd5;",
        "localparam logic [2:0] RECIPE_EARLY_BRANCH = 3'd6;",
        "localparam logic [2:0] RECIPE_EARLY_STACK  = 3'd7;",
        "",
        f"localparam logic [1:0] RECIPE_ACTION_NONE = 2'd{int(RecipeAction.NONE)};",
    ]
    for action in RecipeAction:
        if action != RecipeAction.NONE:
            lines.append(
                f"localparam logic [1:0] RECIPE_ACTION_{action.name} = 2'd{int(action)};"
            )
    lines += [
        "",
        "// Resolve opcode-qualified overlays during D1 structural decode.",
        "function automatic logic [11:0] recipe_effective_entry(",
        "    input logic [11:0] entry,",
        "    input logic [7:0] opcode,",
        "    input logic [7:0] modrm",
        ");",
        "    recipe_effective_entry = entry;",
        "    unique case (entry)",
    ]
    for recipe in OVERLAY_RECIPES:
        lines += [
            f"        12'h{recipe.source_entry:03X}: begin",
            f"            if ({overlay_qualifier_expr(recipe)})",
            f"                recipe_effective_entry = 12'h{recipe.entry:03X};",
            "        end",
        ]
    lines += [
        "        12'h0B6: begin",
        "            if (opcode == 8'h90)",
        "                recipe_effective_entry = 12'h9C9;",
        "        end",
    ]
    lines += [
        "        default: ;",
        "    endcase",
        "endfunction",
        "",
        "function automatic logic [11:0] recipe_fallback_entry(input logic [11:0] entry);",
        "    unique case (entry)",
    ]
    for recipe in OVERLAY_RECIPES:
        lines.append(
            f"        12'h{recipe.entry:03X}: recipe_fallback_entry = 12'h{recipe.source_entry:03X};"
        )
    lines += [
        "        default: recipe_fallback_entry = entry;",
        "    endcase",
        "endfunction",
        "",
        "function automatic logic [1:0] recipe_action(input logic [11:0] entry);",
        "    unique case (entry)",
    ]
    for recipe in OVERLAY_RECIPES:
        lines.append(
            f"        12'h{recipe.entry:03X}: recipe_action = RECIPE_ACTION_{recipe.action.name};"
        )
    for entry, action in sorted(ENTRY_ACTIONS.items()):
        lines.append(
            f"        12'h{entry:03X}: recipe_action = RECIPE_ACTION_{action.name};"
        )
    lines += [
        "        default: recipe_action = RECIPE_ACTION_NONE;",
        "    endcase",
        "endfunction",
        "",
        "function automatic logic [2:0] recipe_early_kind(input logic [11:0] entry);",
        "    unique case (entry)",
    ]
    by_kind: dict[EarlyKind, list[int]] = {}
    for recipe in HARDWIRED_RECIPES:
        by_kind.setdefault(recipe.early, []).append(recipe.entry)
    for recipe in OVERLAY_RECIPES:
        by_kind.setdefault(recipe.early, []).append(recipe.entry)
    for kind in EarlyKind:
        if kind == EarlyKind.SEQ or kind not in by_kind:
            continue
        entry_list = ", ".join(f"12'h{addr:03X}" for addr in sorted(by_kind[kind]))
        lines.append(f"        {entry_list}: recipe_early_kind = RECIPE_EARLY_{kind.name};")
    lines += [
        "        default: recipe_early_kind = RECIPE_EARLY_SEQ;",
        "    endcase",
        "endfunction",
        "",
        "// Architectural GPR inputs needed by a hardwired recipe. The full recipe",
        "// classifier separately qualifies legality; this compact D1 sidecar only",
        "// removes deferred-write hazard decoding from the issue critical path.",
        "function automatic logic [7:0] recipe_gpr_read_mask(input dec_entry_t e);",
        "    logic [7:0] mask;",
        "    logic op_byte;",
        "    logic [2:0] dst_idx, src_idx;",
        "    mask = e.stack_op ? 8'h10 : 8'h00;",
        "    if (e.opcode[7:4] == 4'h4 || e.opcode[7:4] == 4'h5)",
        "        op_byte = 1'b0;",
        "    else if (e.opcode[7:4] == 4'hB && !e.has_0f)",
        "        op_byte = !e.opcode[3];",
        "    else",
        "        op_byte = !e.opcode[0];",
        "    dst_idx = op_byte ? {1'b0, e.dst_reg_sel[1:0]} : e.dst_reg_sel;",
        "    src_idx = op_byte ? {1'b0, e.src_reg_sel[1:0]} : e.src_reg_sel;",
        "    unique case (e.entry_point)",
        f"        {recipe_entries('mov-r-r', 'store-r')}: mask[src_idx] = 1'b1;",
        f"        {recipe_entries('alu-r-r', 'cmp-test-r-r', 'shxd-r-imm')}: begin",
        "            mask[dst_idx] = 1'b1; mask[src_idx] = 1'b1;",
        "        end",
        f"        {recipe_entries('inc-dec-not-neg-r', 'alu-r-imm', 'cmp-test-r-imm', 'shift-r-imm', 'shift-r-one', 'szext-r-16', 'szext-r-32', 'push-r', 'alu-r-m')}: mask[dst_idx] = 1'b1;",
        f"        {recipe_entries('shift-r-cl')}: begin",
        "            mask[dst_idx] = 1'b1; mask[3'd1] = 1'b1;",
        "        end",
        f"        {recipe_entries('shxd-r-cl')}: begin",
        "            mask[dst_idx] = 1'b1; mask[src_idx] = 1'b1; mask[3'd1] = 1'b1;",
        "        end",
        "        default: ;",
        "    endcase",
        "    recipe_gpr_read_mask = mask;",
        "endfunction",
        "",
        "// Entry-point-derived hardwired control generated from the recipe inventory.",
        "function automatic recipe_meta_t recipe_metadata(input dec_entry_t e);",
        "    recipe_meta_t r;",
        "    logic [2:0] grp;",
        "    r = '0;",
        "    if (e.opcode[7:4] == 4'h4 || e.opcode[7:4] == 4'h5)",
        "        r.op_byte = 1'b0;",
        "    else if (e.opcode[7:4] == 4'hB && !e.has_0f)",
        "        r.op_byte = !e.opcode[3];",
        "    else",
        "        r.op_byte = !e.opcode[0];",
        "    grp = ((e.opcode == 8'h80) || (e.opcode == 8'h81) ||",
        "           (e.opcode == 8'h83)) ? e.modrm[5:3] : e.opcode[5:3];",
        "    if (e.rep_lock == PREFIX_NOREPLOCK) begin",
        "        unique case (e.entry_point)",
        f"            {recipe_entries('nop')}: begin",
        "                r.hardwired = 1'b1;",
        "            end",
        f"            {recipe_entries('mov-r-r')}: begin",
        "                r.hardwired = 1'b1; r.commit_sel = RECIPE_COMMIT_ALU;",
        "                r.reads_src = 1'b1;",
        "            end",
        f"            {recipe_entries('mov-r-imm')}: begin",
        "                if ((e.opcode[7:4] == 4'hB) ||",
        "                    ((e.opcode[7:1] == 7'b1100011) &&",
        "                     (e.modrm[7:6] == 2'b11) && (e.modrm[5:3] == 3'b000))) begin",
        "                    r.hardwired = 1'b1; r.commit_sel = RECIPE_COMMIT_ALU;",
        "                end",
        "            end",
        f"            {recipe_entries('alu-r-r')}: begin",
        "                r.hardwired = 1'b1; r.commit_sel = RECIPE_COMMIT_ALU;",
        "                r.reads_flags = (grp == 3'b010) || (grp == 3'b011);",
        "                r.reads_dst = 1'b1; r.reads_src = 1'b1;",
        "                r.writes_flags = 1'b1;",
        "            end",
        f"            {recipe_entries('cmp-test-r-r')}: begin",
        "                r.hardwired = 1'b1; r.reads_dst = 1'b1; r.reads_src = 1'b1;",
        "                r.writes_flags = 1'b1;",
        "            end",
        f"            {recipe_entries('inc-dec-not-neg-r')}: begin",
        "                r.hardwired = 1'b1; r.commit_sel = RECIPE_COMMIT_ALU;",
        "                r.reads_dst = 1'b1; r.writes_flags = 1'b1;",
        "            end",
        f"            {recipe_entries('alu-r-imm')}: begin",
        "                if (!e.has_0f && (((e.opcode[7:6] == 2'b00) &&",
        "                    (e.opcode[2:1] == 2'b10)) ||",
        "                    (((e.opcode == 8'h80) || (e.opcode == 8'h81) ||",
        "                      (e.opcode == 8'h83)) && (e.modrm[7:6] == 2'b11) &&",
        "                     (grp != 3'b111)))) begin",
        "                    r.hardwired = 1'b1; r.commit_sel = RECIPE_COMMIT_ALU;",
        "                    r.reads_flags = (grp == 3'b010) || (grp == 3'b011);",
        "                    r.reads_dst = 1'b1; r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('cmp-test-r-imm')}: begin",
        "                if (!e.has_0f && (((e.opcode[7:6] == 2'b00) &&",
        "                    (e.opcode[2:1] == 2'b10) && (grp == 3'b111)) ||",
        "                    (((e.opcode == 8'h80) || (e.opcode == 8'h81) ||",
        "                      (e.opcode == 8'h83)) && (e.modrm[7:6] == 2'b11) &&",
        "                     (grp == 3'b111)) || (e.opcode[7:1] == 7'b1010100) ||",
        "                    ((e.opcode[7:1] == 7'b1111011) &&",
        "                     (e.modrm[7:6] == 2'b11) && (e.modrm[5:3] == 3'b000)))) begin",
        "                    r.hardwired = 1'b1; r.reads_dst = 1'b1;",
        "                    r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('lea')}: begin",
        "                if (e.has_modrm && (e.modrm[7:6] != 2'b11)) begin",
        "                    r.hardwired = 1'b1; r.uses_ea = 1'b1;",
        "                    r.writes_srcreg = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('shift-r-imm', 'shift-r-one')}: begin",
        "                if (!e.has_0f && (e.modrm[7:6] == 2'b11) &&",
        "                    (e.modrm[5:3] != 3'b010) && (e.modrm[5:3] != 3'b011) &&",
        "                    (e.modrm[5:3] != 3'b110)) begin",
        "                    r.hardwired = 1'b1; r.multi_ustep = 1'b1;",
        "                    r.commit_sel = RECIPE_COMMIT_SHIFT; r.reads_dst = 1'b1;",
        "                    r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('shift-r-cl')}: begin",
        "                if (!e.has_0f && (e.modrm[7:6] == 2'b11) &&",
        "                    (e.modrm[5:3] != 3'b010) && (e.modrm[5:3] != 3'b011) &&",
        "                    (e.modrm[5:3] != 3'b110)) begin",
        "                    r.hardwired = 1'b1; r.multi_ustep = 1'b1;",
        "                    r.commit_sel = RECIPE_COMMIT_SHIFT; r.reads_dst = 1'b1;",
        "                    r.reads_ecx = 1'b1; r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('shxd-r-imm')}: begin",
        "                if (e.has_0f && (e.modrm[7:6] == 2'b11) &&",
        "                    ((e.opcode == 8'hA4) || (e.opcode == 8'hAC))) begin",
        "                    r.hardwired = 1'b1; r.multi_ustep = 1'b1;",
        "                    r.commit_sel = RECIPE_COMMIT_SHIFT;",
        "                    r.reads_dst = 1'b1; r.reads_src = 1'b1;",
        "                    r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('shxd-r-cl')}: begin",
        "                if (e.has_0f && (e.modrm[7:6] == 2'b11) &&",
        "                    ((e.opcode == 8'hA5) || (e.opcode == 8'hAD))) begin",
        "                    r.hardwired = 1'b1; r.multi_ustep = 1'b1;",
        "                    r.commit_sel = RECIPE_COMMIT_SHIFT;",
        "                    r.reads_dst = 1'b1; r.reads_src = 1'b1;",
        "                    r.reads_ecx = 1'b1; r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('szext-r-16', 'szext-r-32')}: begin",
        "                r.hardwired = 1'b1; r.multi_ustep = 1'b1;",
        "                r.commit_sel = RECIPE_COMMIT_SIGSRC;",
        "                r.writes_srcreg = 1'b1; r.reads_dst = 1'b1;",
        "            end",
        f"            {recipe_entries('store-r')}: begin",
        "                r.hardwired = 1'b1; r.slot_has_work = 1'b1; r.uses_ea = 1'b1;",
        "                r.reads_src = 1'b1;",
        "            end",
        f"            {recipe_entries('store-imm')}: begin",
        "                if ((e.opcode[7:1] == 7'b1100011) &&",
        "                    (e.modrm[7:6] != 2'b11) && (e.modrm[5:3] == 3'b000)) begin",
        "                    r.hardwired = 1'b1; r.slot_has_work = 1'b1; r.uses_ea = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('load-r')}: begin",
        "                if (!e.has_0f && (((e.opcode[7:2] == 6'b100010) &&",
        "                    e.opcode[1] && (e.modrm[7:6] != 2'b11)) ||",
        "                    ((e.opcode[7:2] == 6'b101000) && !e.opcode[1]))) begin",
        "                    r.hardwired = 1'b1; r.multi_ustep = 1'b1;",
        "                    r.commit_sel = RECIPE_COMMIT_MEM; r.slot_has_work = 1'b1;",
        "                    r.uses_ea = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('alu-r-m')}: begin",
        "                if (!e.has_0f && (e.opcode[7:6] == 2'b00) &&",
        "                    !e.opcode[2] && e.opcode[1] && e.has_modrm &&",
        "                    (e.modrm[7:6] != 2'b11) && (grp != 3'b111)) begin",
        "                    r.hardwired = 1'b1; r.multi_ustep = 1'b1;",
        "                    r.commit_sel = RECIPE_COMMIT_ALU; r.uses_ea = 1'b1;",
        "                    r.reads_flags = (grp == 3'b010) || (grp == 3'b011);",
        "                    r.reads_dst = 1'b1;",
        "                    r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('cmp-r-m', 'cmp-test-m-r')}: begin",
        "                if (!e.has_0f) begin",
        "                    r.hardwired = 1'b1; r.multi_ustep = 1'b1;",
        "                    r.uses_ea = 1'b1; r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('cmp-test-m-imm')}: begin",
        "                if (!e.has_0f && (((e.opcode == 8'h80) ||",
        "                    (e.opcode == 8'h81) || (e.opcode == 8'h83)) &&",
        "                    (e.modrm[7:6] != 2'b11) && (grp == 3'b111) ||",
        "                    ((e.opcode[7:1] == 7'b1111011) &&",
        "                     (e.modrm[7:6] != 2'b11) &&",
        "                     (e.modrm[5:3] == 3'b000)))) begin",
        "                    r.hardwired = 1'b1; r.multi_ustep = 1'b1;",
        "                    r.uses_ea = 1'b1; r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('rmw-m-imm')}: begin",
        "                if (!e.has_0f && ((e.opcode == 8'h80) ||",
        "                    (e.opcode == 8'h81) || (e.opcode == 8'h83)) &&",
        "                    (e.modrm[7:6] != 2'b11) && (grp != 3'b111)) begin",
        "                    r.hardwired = 1'b1; r.multi_ustep = 1'b1;",
        "                    r.slot_has_work = 1'b1; r.uses_ea = 1'b1;",
        "                    r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('rmw-m-r')}: begin",
        "                if (!e.has_0f) begin",
        "                    r.hardwired = 1'b1; r.multi_ustep = 1'b1;",
        "                    r.slot_has_work = 1'b1; r.uses_ea = 1'b1;",
        "                    r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('szext-m-16', 'szext-m-32')}: begin",
        "                r.hardwired = 1'b1; r.multi_ustep = 1'b1;",
        "                r.commit_sel = RECIPE_COMMIT_SIGSRC;",
        "                r.uses_ea = 1'b1; r.writes_srcreg = 1'b1;",
        "            end",
        f"            {recipe_entries('jcc-rel')}: begin",
        "                r.hardwired = 1'b1; r.multi_ustep = 1'b1; r.jcc = 1'b1;",
        "                r.reads_flags = 1'b1; r.br_rel = 1'b1;",
        "            end",
        f"            {recipe_entries('jmp-rel')}: begin",
        "                r.hardwired = 1'b1; r.multi_ustep = 1'b1; r.br_rel = 1'b1;",
        "            end",
        f"            {recipe_entries('call-rel')}: begin",
        "                r.hardwired = 1'b1; r.multi_ustep = 1'b1;",
        "                if (e.data32) r.commit_sel = RECIPE_COMMIT_ESP;",
        "                r.uses_ea = 1'b1; r.br_rel = 1'b1;",
        "            end",
        f"            {recipe_entries('ret-near')}: begin",
        "                r.hardwired = 1'b1; r.multi_ustep = 1'b1; r.uses_ea = 1'b1;",
        "            end",
        f"            {recipe_entries('push-r')}: begin",
        "                if (e.opcode[7:3] == 5'b01010) begin",
        "                    r.hardwired = 1'b1; r.commit_sel = RECIPE_COMMIT_ESP;",
        "                    r.slot_has_work = 1'b1; r.uses_ea = 1'b1;",
        "                    r.reads_dst = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('push-seg')}: begin",
        "                if ((e.opcode[7:6] == 2'b00) && (e.opcode[2:0] == 3'b110)) begin",
        "                    r.hardwired = 1'b1; r.commit_sel = RECIPE_COMMIT_ESP;",
        "                    r.slot_has_work = 1'b1; r.uses_ea = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('push-imm')}: begin",
        "                if ((e.opcode == 8'h68) || (e.opcode == 8'h6A)) begin",
        "                    r.hardwired = 1'b1; r.commit_sel = RECIPE_COMMIT_ESP;",
        "                    r.slot_has_work = 1'b1; r.uses_ea = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('pop-r')}: begin",
        "                if (e.opcode[7:3] == 5'b01011) begin",
        "                    r.hardwired = 1'b1; r.multi_ustep = 1'b1;",
        "                    r.commit_sel = RECIPE_COMMIT_MEM; r.slot_has_work = 1'b1;",
        "                    r.uses_ea = 1'b1;",
        "                end",
        "            end",
    ]
    for recipe in OVERLAY_RECIPES:
        if recipe.action == RecipeAction.X87_OVERLAY:
            uses_ea = "1'b0" if recipe.qualifier == OverlayQualifier.X87_REG else "1'b1"
            lines += [
                f"            12'h{recipe.entry:03X}: begin",
                "                // Direct x87 transport; normal sequencer retirement.",
                f"                r.commit_sel = RECIPE_ACTION_X87_DIRECT; r.uses_ea = {uses_ea};",
                "            end",
            ]
        elif recipe.action == RecipeAction.RMW_FAST:
            lines += [
                f"            12'h{recipe.entry:03X}: begin",
                "                // Sequenced BRAM overlay; RD_FAST/WR_FAST own memory only.",
                "                r.uses_ea = 1'b1;",
                "            end",
            ]
        else:
            raise ValueError(
                f"overlay {recipe.name}: no hardwired class for action {recipe.action}"
            )
    lines += [
        "            default: ;",
        "        endcase",
        "    end",
        "    recipe_metadata = r;",
        "endfunction",
        "",
    ]
    missing = set(recipes) - emitted
    if missing:
        raise ValueError(f"hardwired recipes missing from recipe_metadata: {sorted(missing)}")
    return "\n".join(lines)


def main() -> int:
    here = Path(__file__).resolve().parent.parent   # Core directory.
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", type=Path, default=here / "ucode_base.hex")
    ap.add_argument("--hex", type=Path, default=here / "ucode.hex")
    ap.add_argument("--mif", type=Path, default=here / "ucode.mif")
    ap.add_argument("--recipe-svh", type=Path, default=here / "ucode_recipes.svh")
    ap.add_argument("--recipe-manifest", type=Path, default=here / "ucode_recipes.md")
    ap.add_argument("--check", action="store_true",
                    help="fail if generated microcode or recipe files are stale")
    args = ap.parse_args()

    base = read_words(args.base)
    words = apply_patches(base)
    validate_recipes(words)
    rom_words = annotate_recipes(words)
    hex_txt, mif_txt = render_hex(rom_words), render_mif(rom_words)
    recipe_svh_txt = render_recipe_svh(words)
    recipe_manifest_txt = render_recipe_manifest(words)

    if args.check:
        stale = (not args.hex.exists() or args.hex.read_text() != hex_txt or
                 not args.mif.exists() or args.mif.read_text() != mif_txt or
                 not args.recipe_svh.exists() or
                 args.recipe_svh.read_text() != recipe_svh_txt or
                 not args.recipe_manifest.exists() or
                 args.recipe_manifest.read_text() != recipe_manifest_txt)
        if stale:
            raise SystemExit("generated microcode/recipe files are stale; rerun ucode_optimize.py")
        print("generated microcode and recipe files are up to date")
        return 0

    args.hex.write_text(hex_txt)
    args.mif.write_text(mif_txt)
    args.recipe_svh.write_text(recipe_svh_txt)
    args.recipe_manifest.write_text(recipe_manifest_txt)
    print(f"wrote {args.hex}, {args.mif}, {args.recipe_svh}, and {args.recipe_manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
