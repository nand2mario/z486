; FXTRACT, FPREM1, FSINCOS and FYL2XP1: the four D9 operations the x87
; command ROM used to treat as FNOP. Wolfenstein 3D's Borland FPU probe runs
; FSINCOS and then divides by the cosine it expects on the stack.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

start:
    cli
    cld
    mov ax, cs
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, 0x700
    push es
    xor ax, ax
    mov es, ax
    mov word [es:16 * 4], fpu_fault
    mov ax, cs
    mov word [es:16 * 4 + 2], ax
    pop es
    fninit

    ; FXTRACT 8.0: ST0 = significand 1.0, ST1 = exponent 3.0.
    mov dx, 1
    fld dword [real_8]
    fxtract
    fstp dword [result32]
    cmp dword [result32], 0x3f800000
    jne fail
    fstp dword [result32]
    cmp dword [result32], 0x40400000
    jne fail

    ; FXTRACT 0.375 = 1.5 * 2^-2.
    mov dx, 2
    fld dword [real_0_375]
    fxtract
    fstp dword [result32]
    cmp dword [result32], 0x3fc00000
    jne fail
    fstp dword [result32]
    cmp dword [result32], 0xc0000000
    jne fail

    ; FXTRACT pushes: two values from one, TOP moves down.
    mov dx, 3
    fld dword [real_8]
    fxtract
    fnstsw ax
    and ax, 0x3800
    cmp ax, 0x3000                              ; TOP 6
    jne fail
    fstp st0
    fstp st0

    ; FXTRACT of an extended denormal, 2^-16383: significand 1.0,
    ; exponent -16383.0 (C67FFC00 as a single).
    mov dx, 11
    fld tword [denormal80]
    fxtract
    fstp dword [result32]
    cmp dword [result32], 0x3f800000
    jne fail
    fstp dword [result32]
    cmp dword [result32], 0xc67ffc00
    jne fail
    fnclex

    ; FPREM1 7 by 2: quotient rounds to 4 (nearest even), remainder -1.
    mov dx, 4
    fld dword [real_2]
    fld dword [real_7]
    fprem1
    fnstsw ax
    and ax, 0x4700
    cmp ax, 0x0100                              ; Q2:Q1:Q0 = 100 -> C0
    jne fail
    fstp dword [result32]
    cmp dword [result32], 0xbf800000
    jne fail
    fstp st0

    ; FPREM 7 by 2 still truncates: quotient 3, remainder 1.
    mov dx, 5
    fld dword [real_2]
    fld dword [real_7]
    fprem
    fstp dword [result32]
    cmp dword [result32], 0x3f800000
    jne fail
    fstp st0

    ; FSINCOS 0.5: ST0 = cos, ST1 = sin.
    mov dx, 6
    fld dword [real_0_5]
    fsincos
    fnstsw ax
    and ax, 0x3800
    cmp ax, 0x3000                              ; FLD to TOP 7, push to 6
    jne fail
    fstp dword [result32]
    cmp dword [result32], 0x3f60a940
    jne fail
    fstp dword [result32]
    cmp dword [result32], 0x3ef57744
    jne fail

    ; FSINCOS then divide, as the Borland probe does: sin/cos = tan.
    mov dx, 7
    fld dword [real_0_5]
    fsincos
    fdivp st1, st0
    fnstsw ax
    test ax, 0x0041                             ; no stack fault / IE
    jnz fail
    fstp st0

    ; FYL2XP1: 3 * log2(1 + 0.25), popped.
    mov dx, 8
    fld dword [real_3]
    fld dword [real_0_25]
    fyl2xp1
    fstp dword [result32]
    cmp dword [result32], 0x3f773da4
    jne fail
    fnstsw ax
    and ax, 0x3800
    jnz fail                                    ; stack empty again

    ; FLDCW keeps the reserved bits: 15:13 and 7 read 0, bit 6 reads 1
    ; (Dracula's Secret stores back 1330h as 1370h).
    mov dx, 9
    fldcw word [cw_1330]
    fnstcw word [result32]
    cmp word [result32], 0x1370
    jne fail
    fldcw word [cw_ffff]
    fnstcw word [result32]
    cmp word [result32], 0x1f7f
    jne fail
    fninit

    ; CR0.NE=0 with an unmasked error pending: a 486 freezes at the next
    ; waiting FP instruction until IGNNE#, but the no-wait control forms run
    ; (this testbench has no IRQ13: a freeze here would time out).
    mov dx, 10
    fldcw word [cw_unmask_ze]
    fldz
    fld1
    fdiv st0, st1                               ; ZE, ES pending, no #MF
    fnstsw ax
    test ax, 0x0080
    jz fail
    fnstcw word [result32]
    fnclex                                      ; clears ES: waiting is free again
    fwait
    fninit

    mov al, STATUS_PASS
    out STATUS_PORT, al
    hlt

fail:
    shl eax, 16                                 ; AX (a status word) in 31:16
    mov ax, dx                                  ; the check in 15:0
    out DATA_PORT, eax
    mov al, STATUS_FAIL
    out STATUS_PORT, al
    hlt

fpu_fault:
    mov dx, 0x10
    jmp fail

align 4
real_8:      dd 0x41000000
real_0_375:  dd 0x3ec00000
real_7:      dd 0x40e00000
real_2:      dd 0x40000000
real_3:      dd 0x40400000
real_0_5:    dd 0x3f000000
real_0_25:   dd 0x3e800000
result32:    dd 0
cw_1330:     dw 0x1330
cw_ffff:     dw 0xffff
cw_unmask_ze: dw 0x037b
denormal80:  dq 0x4000000000000000
             dw 0x0000
