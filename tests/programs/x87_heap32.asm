; x87_heap32.asm - Reproduce TurboQuake's heap-size conversion.
;
; Quake converts an eight-megabyte integer byte count to a double for its
; "%4.1f megabyte heap" message. The expected result is exactly 8.0.

BITS 32
org 0

STATUS_PORT equ 0xE0
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

start:
    cli
    cld
    fninit

    fld dword [megabyte_scale]
    fimul dword [heap_bytes]
    fstp qword [result]

    cmp dword [result], 0x00000000
    jne .fail_low
    cmp dword [result + 4], 0x40200000
    jne .fail_high

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

.fail_low:
    mov al, 1
    jmp .fail
.fail_high:
    mov al, 2
.fail:
    mov dx, 0xE4
    out dx, al
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

align 4
megabyte_scale: dd 0x35800000       ; 1.0 / (1024.0 * 1024.0)
heap_bytes:     dd 8388608
result:         dq 0
