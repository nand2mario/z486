; x87_env32.asm - Exercise the 32-bit protected-mode environment stream.
;
; TurboQuake saves the environment, changes the saved exception masks, then
; reloads it. The 32-bit form transfers seven dwords (28 bytes).

BITS 32
org 0

STATUS_PORT equ 0xE0
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

start:
    cli
    cld
    fninit
    fldcw [control_unmasked]

    fnstenv [env_image]
    cmp word [env_image], 0x0340
    jne .fail_01
    fnstcw [control_out]
    cmp word [control_out], 0x037f
    jne .fail_02

    or dword [env_image], 0x3f
    fldenv [env_image]
    fnstcw [control_out]
    cmp word [control_out], 0x037f
    jne .fail_03

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

.fail_01:
    mov al, 1
    jmp .fail
.fail_02:
    mov al, 2
    jmp .fail
.fail_03:
    mov al, 3
.fail:
    mov dx, 0xE4
    out dx, al
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

align 4
control_unmasked: dw 0x0340
control_out:      dw 0
env_image:       times 28 db 0
