; SPDX-License-Identifier: GPL-2.0-or-later
;
; Directed 80387 cases ported from QEMU's tests/tcg/i386 x87 tests:
;   test-i386-fxam.c and test-i386-fldcst.c
;
; Keep this as a separate compatibility test. These checks use raw m80 values
; so a binary64 approximation cannot accidentally satisfy an x87 result.

BITS 16
org 0

STATUS_PORT equ 0xE0
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF
FXAM_MASK   equ 0x4700

%macro CHECK_FXAM 2
    fninit
    fld tword [%1]
    fxam
    fnstsw ax
    and ax, FXAM_MASK
    cmp ax, %2
    jne fail
    fstp st0
%endmacro

%macro CHECK_CONSTANT 3
    mov word [control_word], %1
    fldcw word [control_word]
    %2
    fstp tword [result80]
    mov si, %3
    mov di, result80
    mov cx, 5
    repe cmpsw
    jne fail
%endmacro

%macro CHECK_FPATAN64 3
    fninit
    fld tword [%2]                         ; Y -> ST(1)
    fld tword [%1]                         ; X -> ST(0)
    fpatan                                 ; atan2(Y, X), pop X
    fstp qword [result64]
    cmp dword [result64], ((%3) & 0xffffffff)
    jne fail
    cmp dword [result64 + 4], ((%3) >> 32)
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

    ; QEMU test-i386-fxam.c classification and sign checks.
    CHECK_FXAM pos_zero,       0x4000
    CHECK_FXAM neg_zero,       0x4200
    CHECK_FXAM pos_normal,     0x0400
    CHECK_FXAM neg_normal,     0x0600
    CHECK_FXAM pos_infinity,   0x0500
    CHECK_FXAM neg_infinity,   0x0700
    CHECK_FXAM pos_qnan,       0x0100
    CHECK_FXAM neg_qnan,       0x0300
    CHECK_FXAM pos_snan,       0x0100
    CHECK_FXAM neg_snan,       0x0300
    CHECK_FXAM pos_denormal,   0x4400
    CHECK_FXAM neg_denormal,   0x4600
    CHECK_FXAM pos_pseudo,     0x4400
    CHECK_FXAM neg_pseudo,     0x4600

    ; QEMU test-i386-fldcst.c verifies that constant loading observes RC while
    ; retaining all 64 explicit significand bits. PC does not truncate these.
    CHECK_CONSTANT 0x037f, fldl2t, l2t_down
    CHECK_CONSTANT 0x077f, fldl2t, l2t_down
    CHECK_CONSTANT 0x0f7f, fldl2t, l2t_down
    CHECK_CONSTANT 0x0b7f, fldl2t, l2t_up

    CHECK_CONSTANT 0x037f, fldl2e, l2e_up
    CHECK_CONSTANT 0x077f, fldl2e, l2e_down
    CHECK_CONSTANT 0x0f7f, fldl2e, l2e_down
    CHECK_CONSTANT 0x0b7f, fldl2e, l2e_up

    CHECK_CONSTANT 0x037f, fldpi, pi_up
    CHECK_CONSTANT 0x077f, fldpi, pi_down
    CHECK_CONSTANT 0x0f7f, fldpi, pi_down
    CHECK_CONSTANT 0x0b7f, fldpi, pi_up

    CHECK_CONSTANT 0x037f, fldlg2, lg2_up
    CHECK_CONSTANT 0x077f, fldlg2, lg2_down
    CHECK_CONSTANT 0x0f7f, fldlg2, lg2_down
    CHECK_CONSTANT 0x0b7f, fldlg2, lg2_up

    CHECK_CONSTANT 0x037f, fldln2, ln2_up
    CHECK_CONSTANT 0x077f, fldln2, ln2_down
    CHECK_CONSTANT 0x0f7f, fldln2, ln2_down
    CHECK_CONSTANT 0x0b7f, fldln2, ln2_up

    ; QEMU test-i386-fpatan.c special-value quadrant matrix. The executor is
    ; 53-bit, so compare the architecturally rounded binary64 result here.
    CHECK_FPATAN64 neg_infinity, neg_infinity, 0xc002d97c7f3321d2
    CHECK_FPATAN64 neg_infinity, neg_normal,   0xc00921fb54442d18
    CHECK_FPATAN64 neg_infinity, neg_zero,     0xc00921fb54442d18
    CHECK_FPATAN64 neg_infinity, pos_zero,     0x400921fb54442d18
    CHECK_FPATAN64 neg_infinity, pos_normal,   0x400921fb54442d18
    CHECK_FPATAN64 neg_infinity, pos_infinity, 0x4002d97c7f3321d2

    CHECK_FPATAN64 neg_normal,   neg_infinity, 0xbff921fb54442d18
    CHECK_FPATAN64 neg_normal,   neg_zero,     0xc00921fb54442d18
    CHECK_FPATAN64 neg_normal,   pos_zero,     0x400921fb54442d18
    CHECK_FPATAN64 neg_normal,   pos_infinity, 0x3ff921fb54442d18

    CHECK_FPATAN64 neg_zero,     neg_infinity, 0xbff921fb54442d18
    CHECK_FPATAN64 neg_zero,     neg_normal,   0xbff921fb54442d18
    CHECK_FPATAN64 neg_zero,     neg_zero,     0xc00921fb54442d18
    CHECK_FPATAN64 neg_zero,     pos_zero,     0x400921fb54442d18
    CHECK_FPATAN64 neg_zero,     pos_normal,   0x3ff921fb54442d18
    CHECK_FPATAN64 neg_zero,     pos_infinity, 0x3ff921fb54442d18

    CHECK_FPATAN64 pos_zero,     neg_infinity, 0xbff921fb54442d18
    CHECK_FPATAN64 pos_zero,     neg_normal,   0xbff921fb54442d18
    CHECK_FPATAN64 pos_zero,     neg_zero,     0x8000000000000000
    CHECK_FPATAN64 pos_zero,     pos_zero,     0x0000000000000000
    CHECK_FPATAN64 pos_zero,     pos_normal,   0x3ff921fb54442d18
    CHECK_FPATAN64 pos_zero,     pos_infinity, 0x3ff921fb54442d18

    CHECK_FPATAN64 pos_normal,   neg_infinity, 0xbff921fb54442d18
    CHECK_FPATAN64 pos_normal,   neg_zero,     0x8000000000000000
    CHECK_FPATAN64 pos_normal,   pos_zero,     0x0000000000000000
    CHECK_FPATAN64 pos_normal,   pos_infinity, 0x3ff921fb54442d18

    CHECK_FPATAN64 pos_infinity, neg_infinity, 0xbfe921fb54442d18
    CHECK_FPATAN64 pos_infinity, neg_normal,   0x8000000000000000
    CHECK_FPATAN64 pos_infinity, neg_zero,     0x8000000000000000
    CHECK_FPATAN64 pos_infinity, pos_zero,     0x0000000000000000
    CHECK_FPATAN64 pos_infinity, pos_normal,   0x0000000000000000
    CHECK_FPATAN64 pos_infinity, pos_infinity, 0x3fe921fb54442d18

pass:
    mov al, STATUS_PASS
    jmp report

fail:
    mov al, STATUS_FAIL

report:
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

align 8
control_word: dw 0x037f
result80:     times 10 db 0
result64:     dq 0

pos_zero:     dq 0x0000000000000000
              dw 0x0000
neg_zero:     dq 0x0000000000000000
              dw 0x8000
pos_normal:   dq 0x8000000000000000
              dw 0x3fff
neg_normal:   dq 0x8000000000000000
              dw 0xbfff
pos_infinity: dq 0x8000000000000000
              dw 0x7fff
neg_infinity: dq 0x8000000000000000
              dw 0xffff
pos_qnan:     dq 0xc000000000000001
              dw 0x7fff
neg_qnan:     dq 0xc000000000000001
              dw 0xffff
pos_snan:     dq 0x8000000000000001
              dw 0x7fff
neg_snan:     dq 0x8000000000000001
              dw 0xffff
pos_denormal: dq 0x0000000000000001
              dw 0x0000
neg_denormal: dq 0x0000000000000001
              dw 0x8000
pos_pseudo:   dq 0x8000000000000000
              dw 0x0000
neg_pseudo:   dq 0x8000000000000000
              dw 0x8000

l2t_down: dq 0xd49a784bcd1b8afe
          dw 0x4000
l2t_up:   dq 0xd49a784bcd1b8aff
          dw 0x4000
l2e_down: dq 0xb8aa3b295c17f0bb
          dw 0x3fff
l2e_up:   dq 0xb8aa3b295c17f0bc
          dw 0x3fff
pi_down:  dq 0xc90fdaa22168c234
          dw 0x4000
pi_up:    dq 0xc90fdaa22168c235
          dw 0x4000
lg2_down: dq 0x9a209a84fbcff798
          dw 0x3ffd
lg2_up:   dq 0x9a209a84fbcff799
          dw 0x3ffd
ln2_down: dq 0xb17217f7d1cf79ab
          dw 0x3ffe
ln2_up:   dq 0xb17217f7d1cf79ac
          dw 0x3ffe
