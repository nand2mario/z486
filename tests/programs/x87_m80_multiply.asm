; Fistfight's BASIC VAL("5") scales 5e18 by the extended constants 0.01
; and 1e-16. Dropping their operand GRS bits in FMUL produces the previous
; binary64 number below 5; its exact stage-number comparison rejects the INI.
BITS 16
org 0
start:
    cli
    mov ax, cs
    mov ds, ax
    mov ss, ax
    mov sp, 0x700
    fninit

    mov dx, 1
    fld tword [five_e18]
    fld tword [one_e_minus2]
    fmulp st1, st0
    fst qword [result]
    cmp dword [result], 0x85d8a000
    jne fail
    cmp dword [result+4], 0x43663457
    jne fail

    mov dx, 2
    fld tword [one_e_minus16]
    fmulp st1, st0
    fstp qword [result]
    cmp dword [result], 0
    jne fail
    cmp dword [result+4], 0x40140000
    jne fail

    ; A raw extended operand halfway between binary64 1 and its successor:
    ; round-to-nearest is ties-to-even; round-up must retain its guard bit.
    mov dx, 3
    fld1
    fld tword [half_ulp_above_one]
    fmulp st1, st0
    fstp qword [result]
    cmp dword [result], 0
    jne fail
    cmp dword [result+4], 0x3ff00000
    jne fail

    mov dx, 4
    fldcw [round_up]
    fld1
    fld tword [half_ulp_above_one]
    fmulp st1, st0
    fstp qword [result]
    cmp dword [result], 1
    jne fail
    cmp dword [result+4], 0x3ff00000
    jne fail

    ; Normalization must also see a carry from the extended operand tail.
    mov dx, 5
    fninit
    fld1
    fld tword [below_two]
    fmulp st1, st0
    fstp qword [result]
    cmp dword [result], 0
    jne fail
    cmp dword [result+4], 0x40000000
    jne fail

    ; Both operands may carry a raw extended tail.
    mov dx, 6
    fld tword [one_e_minus2]
    fld tword [one_e_minus16]
    fmulp st1, st0
    fstp qword [result]
    cmp dword [result], 0xd1d243ac
    jne fail
    cmp dword [result+4], 0x3c32725d
    jne fail

    mov dx, 7
    fldcw [round_down]
    fld1
    fld tword [half_ulp_above_one]
    fchs
    fmulp st1, st0
    fstp qword [result]
    cmp dword [result], 1
    jne fail
    cmp dword [result+4], 0xbff00000
    jne fail

    ; A tiny raw tail is still significant for directed single rounding.
    mov dx, 8
    fldcw [single_up]
    fld1
    fld tword [half_ulp_above_one]
    fmulp st1, st0
    fstp dword [result]
    cmp dword [result], 0x3f800001
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
five_e18: dq 0x8ac7230489e80000
          dw 0x403d
one_e_minus2: dq 0xa3d70a3d70a3d70a
              dw 0x3ff8
one_e_minus16: dq 0xe69594bec44de15b
               dw 0x3fc9
half_ulp_above_one: dq 0x8000000000000400
                    dw 0x3fff
below_two: dq 0xffffffffffffffff
           dw 0x3fff
round_up: dw 0x0b7f
round_down: dw 0x077f
single_up: dw 0x087f
result: dq 0
