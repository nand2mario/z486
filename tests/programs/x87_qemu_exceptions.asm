; SPDX-License-Identifier: GPL-2.0-or-later
;
; Applicable 80387 cases ported from QEMU's
; tests/tcg/i386/test-i386-fp-exceptions.c.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF
EXC_MASK    equ 0x003d              ; IE | ZE | OE | UE | PE
IE          equ 0x0001
ZE          equ 0x0004
OE_PE       equ 0x0028
UE_PE       equ 0x0030
PE          equ 0x0020

%macro BEGIN_TEST 1
    mov byte [test_id], %1
    fninit
%endmacro

%macro CHECK_STATUS 1
    fnstsw ax
    and ax, EXC_MASK
    cmp ax, %1
    jne fail
%endmacro

start:
    cli
    cld
    mov ax, cs
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, 0x700

    ; Loading a signaling NaN widens and quiets it while setting IE.
    BEGIN_TEST 0x10
    fld dword [snan32]
    CHECK_STATUS IE
    BEGIN_TEST 0x11
    fld qword [snan64]
    CHECK_STATUS IE

    ; Narrow stores report range and precision exceptions.
    BEGIN_TEST 0x20
    fld tword [ext_min]
    fstp dword [result32]
    CHECK_STATUS UE_PE
    BEGIN_TEST 0x21
    fld tword [ext_min]
    fstp qword [result64]
    CHECK_STATUS UE_PE
    BEGIN_TEST 0x22
    fld tword [ext_max]
    fstp dword [result32]
    CHECK_STATUS OE_PE
    BEGIN_TEST 0x23
    fld tword [ext_max]
    fstp qword [result64]
    CHECK_STATUS OE_PE
    BEGIN_TEST 0x24
    fld tword [ext_third]
    fstp dword [result32]
    CHECK_STATUS PE
    BEGIN_TEST 0x25
    fld tword [ext_third]
    fstp qword [result64]
    CHECK_STATUS PE
    BEGIN_TEST 0x26
    fld tword [snan80]
    fstp dword [result32]
    CHECK_STATUS IE
    BEGIN_TEST 0x27
    fld tword [snan80]
    fstp qword [result64]
    CHECK_STATUS IE

    ; FRNDINT reports discarded fractional data and invalid operands.
    BEGIN_TEST 0x30
    fld tword [ext_min]
    frndint
    CHECK_STATUS PE
    BEGIN_TEST 0x31
    fld tword [snan80]
    frndint
    CHECK_STATUS IE

    ; FCOM rejects all NaNs; FUCOM accepts quiet NaNs but rejects SNaNs.
    BEGIN_TEST 0x40
    fldz
    fld tword [qnan80]
    fcom st1
    CHECK_STATUS IE
    BEGIN_TEST 0x41
    fldz
    fld tword [qnan80]
    fucom st1
    CHECK_STATUS 0
    BEGIN_TEST 0x42
    fldz
    fld tword [snan80]
    fucom st1
    CHECK_STATUS IE

    ; Arithmetic special cases and range/precision flags.
    BEGIN_TEST 0x50
    fld tword [neg_inf]
    fld tword [pos_inf]
    faddp st1, st0
    CHECK_STATUS IE
    BEGIN_TEST 0x51
    fldz
    fld tword [pos_inf]
    fmulp st1, st0
    CHECK_STATUS IE
    BEGIN_TEST 0x52
    fldz
    fld1
    fdivp st1, st0                     ; 0 / 1: exact
    CHECK_STATUS 0
    BEGIN_TEST 0x53
    fldz
    fld1
    fdiv st0, st1                      ; 1 / 0
    CHECK_STATUS ZE
    BEGIN_TEST 0x54
    fld tword [pos_inf]
    fld tword [pos_inf]
    fdivp st1, st0                     ; inf / inf
    CHECK_STATUS IE
    BEGIN_TEST 0x55
    fldz
    fldz
    fdivp st1, st0                     ; 0 / 0
    CHECK_STATUS IE

    ; Square root preserves -0 and rejects negative finite/infinite values.
    BEGIN_TEST 0x60
    fld tword [neg_max]
    fsqrt
    CHECK_STATUS IE
    BEGIN_TEST 0x61
    fld tword [neg_inf]
    fsqrt
    CHECK_STATUS IE
    BEGIN_TEST 0x62
    fld tword [neg_zero]
    fsqrt
    CHECK_STATUS 0
    fstp qword [result64]
    cmp dword [result64], 0
    jne fail
    cmp dword [result64 + 4], 0x80000000
    jne fail

    ; Integer stores distinguish inexact rounding from invalid overflow.
    BEGIN_TEST 0x70
    fld tword [one_point_five]
    fistp word [result16]
    CHECK_STATUS PE
    BEGIN_TEST 0x71
    fld tword [pos_32767_5]
    fistp word [result16]
    CHECK_STATUS IE
    BEGIN_TEST 0x72
    fld tword [pos_2p63]
    fistp qword [result64]
    CHECK_STATUS IE
    BEGIN_TEST 0x73
    fld tword [qnan80]
    fistp dword [result32]
    CHECK_STATUS IE

pass:
    mov al, STATUS_PASS
    jmp report

fail:
    movzx eax, byte [test_id]
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL

report:
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

align 8
test_id:       db 0
result16:      dw 0
result32:      dd 0
result64:      dq 0

snan32:        dd 0x7f800001
snan64:        dq 0x7ff0000000000001
snan80:        dq 0x8000000000000001
                dw 0x7fff
qnan80:        dq 0xc000000000000001
                dw 0x7fff
pos_inf:       dq 0x8000000000000000
                dw 0x7fff
neg_inf:       dq 0x8000000000000000
                dw 0xffff
neg_zero:      dq 0
                dw 0x8000
ext_min:       dq 0x8000000000000000
                dw 0x0001
ext_max:       dq 0xffffffffffffffff
                dw 0x7ffe
neg_max:       dq 0xffffffffffffffff
                dw 0xfffe
ext_third:     dq 0xaaaaaaaaaaaaaaab
                dw 0x3ffd
one_point_five:dq 0xc000000000000000
                dw 0x3fff
pos_32767_5:   dq 0xffff000000000000
                dw 0x400d
pos_2p63:      dq 0x8000000000000000
                dw 0x403e
