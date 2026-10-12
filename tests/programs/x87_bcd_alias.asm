; FST ST(i), FFREEP, FBLD and the undocumented register-form aliases a
; 387/486 executes (FSTP1/8/9, FXCH4/7, FCOM2, FCOMP3/5). Before they were
; decoded, FST ST(i), FFREEP and the aliases did nothing and FBLD hung the
; CPU waiting for the x87 to take its operand.
; Fail code (port E4): AX = status word in 31:16, check number in 15:0.

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
    fninit

    ; FST ST(1): ST1 = ST0, no pop.
    mov dx, 1
    fldz
    fld dword [real_2]                      ; ST0 = 2, ST1 = 0
    fst st1
    call top_field
    cmp ax, 0x3000                          ; TOP 6: still two values
    jne fail
    fstp st0
    fstp dword [result32]
    cmp dword [result32], 0x40000000
    jne fail

    ; FFREEP ST0: frees ST0 and pops.
    mov dx, 2
    fninit
    fld1
    db 0xdf, 0xc0                           ; ffreep st0
    call top_field
    jnz fail
    fnstenv [env]
    cmp word [env + 4], 0xffff              ; all tags empty
    jne fail

    ; FFREEP ST(1): frees the old ST1, pops ST0; ST0 is the freed register.
    mov dx, 3
    fninit
    fld1
    fld1
    db 0xdf, 0xc1                           ; ffreep st1
    call top_field
    cmp ax, 0x3800                          ; TOP 7
    jne fail
    fxam
    fnstsw ax
    and ax, 0x4500
    cmp ax, 0x4100                          ; C3:C0 = 1x1: empty
    jne fail

    ; FBLD +1234.
    mov dx, 4
    fninit
    fbld [bcd_1234]
    fistp dword [result32]
    cmp dword [result32], 1234
    jne fail

    ; FBLD -987654321098765 (exact in z486's 53-bit significand).
    mov dx, 5
    fbld [bcd_neg]
    fistp qword [result64]
    cmp dword [result64], 0xcf07aff3
    jne fail
    cmp dword [result64 + 4], 0xfffc7dbb
    jne fail

    ; FBLD 576460752303423488 (2^59, all 18 digits) round-trips through FBSTP.
    mov dx, 6
    fbld [bcd_2p59]
    fbstp [result80]
    mov si, result80
    mov di, bcd_2p59
    mov cx, 10
    repe cmpsb
    jne fail

    ; FBLD -0 loads -0.
    mov dx, 7
    fbld [bcd_neg_zero]
    fxam
    fnstsw ax
    and ax, 0x4700
    cmp ax, 0x4200                          ; C3 zero, C1 negative
    jne fail
    fstp st0

    ; FSTP1 (D9 D8+i) = FSTP ST(i).
    mov dx, 8
    fninit
    fldz
    fld dword [real_2]
    db 0xd9, 0xd9                           ; fstp1 st1
    call top_field
    cmp ax, 0x3800
    jne fail
    fstp dword [result32]
    cmp dword [result32], 0x40000000
    jne fail

    ; FSTP8 (DF D0+i) and FSTP9 (DF D8+i).
    mov dx, 9
    fninit
    fldz
    fld dword [real_2]
    db 0xdf, 0xd1                           ; fstp8 st1
    fldz
    db 0xdf, 0xd9                           ; fstp9 st1: ST1 = 0
    fstp dword [result32]
    cmp dword [result32], 0
    jne fail
    call top_field
    jnz fail

    ; FXCH4 (DD C8+i) and FXCH7 (DF C8+i).
    mov dx, 10
    fninit
    fldz
    fld dword [real_2]                      ; ST0 = 2, ST1 = 0
    db 0xdd, 0xc9                           ; fxch4 st1: ST0 = 0
    fst dword [result32]
    cmp dword [result32], 0
    jne fail
    db 0xdf, 0xc9                           ; fxch7 st1: ST0 = 2
    fstp dword [result32]
    cmp dword [result32], 0x40000000
    jne fail
    fstp st0

    ; FCOM2 (DC D0+i): 2 < 3 sets C0, no pop.
    mov dx, 11
    fninit
    fld dword [real_3]
    fld dword [real_2]                      ; ST0 = 2, ST1 = 3
    db 0xdc, 0xd1                           ; fcom2 st1
    fnstsw ax
    and ax, 0x4500
    cmp ax, 0x0100
    jne fail
    call top_field
    cmp ax, 0x3000
    jne fail

    ; FCOMP3 (DC D8+i) and FCOMP5 (DE D0+i) compare and pop.
    mov dx, 12
    db 0xdc, 0xd9                           ; fcomp3 st1: 2 < 3, pop
    fnstsw ax
    and ax, 0x4500
    cmp ax, 0x0100
    jne fail
    call top_field
    cmp ax, 0x3800
    jne fail
    fld dword [real_3]
    db 0xde, 0xd1                           ; fcomp5 st1: 3 = 3, pop
    fnstsw ax
    and ax, 0x4500
    cmp ax, 0x4000
    jne fail
    call top_field
    cmp ax, 0x3800
    jne fail

    mov al, STATUS_PASS
    out STATUS_PORT, al
    hlt

; AX = status word TOP field (bits 13:11), ZF set when TOP is 0.
top_field:
    fnstsw ax
    and ax, 0x3800
    ret

fail:
    fnstsw ax
    shl eax, 16
    mov ax, dx
    out DATA_PORT, eax
    mov al, STATUS_FAIL
    out STATUS_PORT, al
    hlt

align 4
real_2:       dd 0x40000000
real_3:       dd 0x40400000
result32:     dd 0
result64:     dq 0
result80:     times 10 db 0
align 2
bcd_1234:     db 0x34, 0x12, 0, 0, 0, 0, 0, 0, 0, 0x00
bcd_neg:      db 0x65, 0x87, 0x09, 0x21, 0x43, 0x65, 0x87, 0x09, 0x00, 0x80
bcd_2p59:     db 0x88, 0x34, 0x42, 0x03, 0x23, 0x75, 0x60, 0x64, 0x57, 0x00
bcd_neg_zero: db 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x80
env:          times 14 db 0
