; Exercise x87 memory arithmetic through the original 80386 ESC microcode.

BITS 16
org 0

STATUS_PORT equ 0xE0
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

    fld dword [real_2_5]
    fadd dword [real_1_5]
    fstp dword [result32]
    cmp dword [result32], 0x40800000       ; 4.0
    jne fail

    fld dword [real_2_5]
    fmul dword [real_1_5]
    fstp dword [result32]
    cmp dword [result32], 0x40700000       ; 3.75
    jne fail

    fld dword [real_2_5]
    fsub dword [real_1_5]
    fstp dword [result32]
    cmp dword [result32], 0x3f800000       ; 1.0
    jne fail

    fld dword [real_2_5]
    fsubr dword [real_1_5]
    fstp dword [result32]
    cmp dword [result32], 0xbf800000       ; -1.0
    jne fail

    fld qword [real64_1_25]
    fadd qword [real64_2_5]
    fstp qword [result64]
    cmp dword [result64], 0x00000000
    jne fail
    cmp dword [result64 + 4], 0x400e0000   ; 3.75
    jne fail

    fild dword [int_5]
    fiadd dword [int_minus_2]
    fistp dword [result32]
    cmp dword [result32], 3
    jne fail

    fild word [int_3]
    fimul word [int16_minus_2]
    fistp word [result16]
    cmp word [result16], -6
    jne fail

    fild dword [int_5]
    fisubr dword [int_2]
    fistp dword [result32]
    cmp dword [result32], -3
    jne fail

    fld dword [real_2_5]
    frndint
    fistp dword [result32]
    cmp dword [result32], 2                  ; nearest-even
    jne fail

    fldcw word [round_down]
    fld dword [real_minus_1_5]
    frndint
    fistp dword [result32]
    cmp dword [result32], -2
    jne fail
    fldcw word [round_nearest]

    fld dword [real_2_5]
    fcom dword [real_1_5]
    fnstsw ax
    and ax, 0x4500
    jnz fail                                  ; greater
    fcomp dword [real_2_5]
    fnstsw ax
    and ax, 0x4500
    cmp ax, 0x4000                            ; equal, then pop
    jne fail

    fild dword [int_5]
    ficom dword [int_2]
    fnstsw ax
    and ax, 0x4500
    jnz fail                                  ; greater
    ficomp word [int_5]
    fnstsw ax
    and ax, 0x4500
    cmp ax, 0x4000                            ; equal, then pop
    jne fail

    fld dword [real_7_5]
    fdiv dword [real_2_5]
    fstp dword [result32]
    cmp dword [result32], 0x40400000          ; 3.0
    jne fail

    fld dword [real_7_5]
    fdivr dword [real_2_5]
    fstp dword [result32]
    cmp dword [result32], 0x3eaaaaab          ; 1/3
    jne fail

    fild dword [int_21]
    fidiv dword [int_7]
    fistp dword [result32]
    cmp dword [result32], 3
    jne fail

    fild word [int_3]
    fidivr word [int_12]
    fistp word [result16]
    cmp word [result16], 4
    jne fail

    fld dword [real_9]
    fsqrt
    fstp dword [result32]
    cmp dword [result32], 0x40400000          ; 3.0
    jne fail

    fninit
    fldz
    fld1
    fdiv st0, st1                             ; +1 / +0
    fnstsw ax
    test ax, 0x0004                           ; ZE
    jz fail
    fstp dword [result32]
    cmp dword [result32], 0x7f800000
    jne fail

    ; Cold transcendental instructions use the same command/stack protocol.
    fninit
    fld dword [real_0_5]
    fsin
    fstp dword [result32]
    cmp dword [result32], 0x3ef57744
    jne fail

    fld dword [real_0_5]
    fcos
    fstp dword [result32]
    cmp dword [result32], 0x3f60a940
    jne fail

    fld dword [real_0_5]
    fptan
    fstp dword [scratch32]
    cmp dword [scratch32], 0x3f800000
    jne fail
    fstp dword [result32]
    cmp dword [result32], 0x3f0bda7b
    jne fail

    fld dword [real_1_0]
    fld dword [real_minus_1_0]
    fpatan
    fstp dword [result32]
    cmp dword [result32], 0x4016cbe4
    jne fail

    ; Unmasked arithmetic does not commit its result. ERROR# remains active
    ; until FWAIT enters vector 16 and the handler clears the exception.
    fninit
    fldcw word [unmask_divide_by_zero]
    fldz
    fld1
    fdiv st0, st1
    fnstsw ax
    and ax, 0x8084                           ; B, ES, ZE
    cmp ax, 0x8084
    jne fail
    cmp byte [fault_seen], 0
    jne fail
    fwait
    cmp byte [fault_seen], 1
    jne fail
    fstp dword [result32]
    cmp dword [result32], 0x3f800000         ; ST(0) was not overwritten
    jne fail

    fninit

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

fpu_fault:
    inc byte [cs:fault_seen]
    fnclex
    iret

align 8
real_0_5:      dd 0x3f000000
real_1_0:      dd 0x3f800000
real_minus_1_0: dd 0xbf800000
real_1_5:      dd 0x3fc00000
real_2_5:      dd 0x40200000
real_7_5:      dd 0x40f00000
real_9:        dd 0x41100000
real_minus_1_5:dd 0xbfc00000
real64_1_25:   dq 0x3ff4000000000000
real64_2_5:    dq 0x4004000000000000
int_2:         dd 2
int_3:         dw 3
int_5:         dd 5
int_7:         dd 7
int_12:        dw 12
int_21:        dd 21
int_minus_2:   dd -2
int16_minus_2: dw -2
result16:      dw 0
result32:      dd 0
scratch32:     dd 0
result64:      dq 0
round_nearest: dw 0x037f
round_down:    dw 0x077f
unmask_divide_by_zero: dw 0x037b
fault_seen:    db 0
