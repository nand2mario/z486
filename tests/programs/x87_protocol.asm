; x87_protocol.asm - Exercise the original 80386/80387 command and data stream
;
; This is a protocol trace corpus, not an arithmetic-result test. With the
; no-FPU model, reserved x87 reads return all ones. Run with:
;   SIM_PLUSARGS=+trace_x87 ./test_protected_mode.py x87_protocol

BITS 16
org 0

STATUS_PORT equ 0xE0
STATUS_PASS equ 0x01

start:
    cli
    cld
    mov ax, cs
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, 0x700

    ; Control and status commands.
    fninit
    fnclex
    fnstsw ax
    fnstsw word [status_word]
    fnstcw word [control_word_out]
    fldcw word [control_word]

    ; Register-only stack and arithmetic commands.
    fld1
    fldz
    fxch st1
    fadd st0, st1
    fmulp st1, st0
    fstp st0

    ; Real memory transfers.
    fld dword [real32]
    fstp dword [real32_out]
    fld qword [real64]
    fstp qword [real64_out]
    fld tword [real80]
    fstp tword [real80_out]

    ; Integer memory transfers.
    fild word [int16]
    fistp word [int16_out]
    fild dword [int32]
    fistp dword [int32_out]
    fild qword [int64]
    fistp qword [int64_out]

    ; Environment and complete-state streams.
    fnstenv [env_image]
    fldenv [env_image]
    fnsave [save_image]
    frstor [save_image]

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

align 4
control_word:       dw 0x037f
control_word_out:   dw 0
status_word:        dw 0
int16:              dw -1234
int16_out:          dw 0
int32:              dd -12345678
int32_out:          dd 0
int64:              dq -1234567890123
int64_out:          dq 0
real32:             dd 0x3fc00000               ; 1.5
real32_out:         dd 0
real64:             dq 0x4004000000000000       ; 2.5
real64_out:         dq 0
real80:             dq 0xc000000000000000       ; 3.0 significand
                    dw 0x4000
real80_out:         times 10 db 0
env_image:          times 32 db 0
save_image:         times 128 db 0
