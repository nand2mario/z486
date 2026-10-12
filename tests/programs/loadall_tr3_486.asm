; 486 decode changes to the 386 entry PLA (gen_pla_group.py ENTRY_PATCHES):
; 0F 07 (the 386's LOADALL) raises #UD, MOV to and from TR3 (a 486 cache test
; register) runs like TR4-7, and TR0-2 still raise #UD.
; Fail code (port E4): the check number.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

start:
    cli
    xor ax, ax
    mov ds, ax
    mov word [6*4], ud_handler
    mov word [6*4+2], cs
    mov ax, cs
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, 0x700

    ; 0F 07 raises #UD (the 386 ran LOADALL and loaded registers from ES:EDI).
    mov dx, 1
    mov byte [ud_seen], 0
    mov word [ud_resume], after_loadall
    db 0x0f, 0x07
after_loadall:
    cmp byte [ud_seen], 1
    jne fail

    ; MOV TR3,EAX and MOV EAX,TR3 run without #UD.
    mov dx, 2
    mov byte [ud_seen], 0
    mov word [ud_resume], fail
    xor eax, eax
    db 0x0f, 0x26, 0xd8                     ; mov tr3, eax
    db 0x0f, 0x24, 0xd8                     ; mov eax, tr3
    cmp byte [ud_seen], 0
    jne fail

    ; MOV EAX,TR2 still raises #UD.
    mov dx, 3
    mov word [ud_resume], after_tr2
    db 0x0f, 0x24, 0xd0                     ; mov eax, tr2
after_tr2:
    cmp byte [ud_seen], 1
    jne fail

    mov al, 1
    out STATUS_PORT, al
    hlt

; #UD: count it and continue at ud_resume.
ud_handler:
    mov byte [ud_seen], 1
    add sp, 2                               ; drop the faulting IP
    push word [ud_resume]
    iret

fail:
    mov ax, dx
    out DATA_PORT, ax
    mov al, 0xff
    out STATUS_PORT, al
    hlt

align 2
ud_resume: dw 0
ud_seen:   db 0
