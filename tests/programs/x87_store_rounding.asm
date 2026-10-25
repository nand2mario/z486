; FIST/FST must commit C1 from the conversion, not leave start_store's zero.
; Intel486 PRM (1992), Table 15-1. Discworld II's PCM decoder exercises FISTP.
BITS 16
org 0

%macro CHECK_FLAGS 1
    fnstsw ax
    and ax, 0x0261                  ; C1, PE, SF, IE
    cmp ax, %1
    jne fail
%endmacro

start:
    cli
    mov ax, cs
    mov ds, ax
    mov ss, ax
    mov sp, 0x700
    fninit

    mov dx, 1
    fld qword [up]
    fistp dword [result]
    CHECK_FLAGS 0x0220
    cmp dword [result], 15
    jne fail

    mov dx, 2
    fnclex
    fld qword [down]
    fist word [result]
    CHECK_FLAGS 0x0020
    cmp word [result], 14
    jne fail

    mov dx, 3
    fnclex
    fchs
    fldcw [round_down]
    fistp qword [result]
    CHECK_FLAGS 0x0220
    cmp dword [result], -15
    jne fail
    cmp dword [result+4], -1
    jne fail

    mov dx, 4
    fninit
    fld qword [exact]
    fistp dword [result]
    CHECK_FLAGS 0
    cmp dword [result], 15
    jne fail

    mov dx, 5
    fld qword [single_up]
    fstp dword [result]
    CHECK_FLAGS 0x0220
    cmp dword [result], 0x3f800001
    jne fail

    mov dx, 6
    fnclex
    fld tword [double_up]
    fstp qword [result]
    CHECK_FLAGS 0x0220
    cmp dword [result], 1
    jne fail
    cmp dword [result+4], 0x3ff00000
    jne fail

    mov dx, 7
    fnclex
    fld qword [up]
    fbstp tword [result]
    CHECK_FLAGS 0x0220
    cmp word [result], 0x0015
    jne fail

    mov dx, 8
    fnclex
    fld qword [too_large]
    fistp word [result]
    CHECK_FLAGS 0x0001
    cmp word [result], 0x8000
    jne fail

    mov dx, 9
    fninit
    fistp dword [result]             ; Underflow must retain C1=0, IE|SF.
    CHECK_FLAGS 0x0041
    cmp dword [result], 0x80000000
    jne fail

    mov al, 1
    out 0xe0, al
    hlt
fail:
    mov ax, dx
    out 0xe4, ax
    mov al, 0xff
    out 0xe0, al
    hlt
align 8
up: dq 0x402d800000000000           ; 14.75 -> 15, magnitude rounded up
down: dq 0x402c800000000000         ; 14.25 -> 14 (or -15, round down)
exact: dq 0x402e000000000000        ; 15
single_up: dq 0x3ff0000018000000    ; 1 + 3/4 binary32 ulp
double_up: dq 0x8000000000000600    ; 1 + 3/4 binary64 ulp, extended
           dw 0x3fff
too_large: dq 0x40f0000c00000000    ; 65536.75, invalid 16-bit conversion
round_down: dw 0x077f
result: times 10 db 0
