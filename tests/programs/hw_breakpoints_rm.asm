; hw_breakpoints_rm.asm - a data-write hardware breakpoint in real mode
;
; Turbo Debugger 3.1's TDH386.SYS sets a write breakpoint the same way: DR0 =
; the linear address (0040:0050, the BIOS cursor position = 450h), DR7 L0
; with R/W0 = 01 (writes) and LEN0 = 01 (2 bytes); the #DB arrives through
; IVT vector 1 after the writing instruction.
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 fail code.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

start:
    cli
    xor ax, ax
    mov ds, ax
    mov word [1*4], db_handler
    mov word [1*4+2], cs
    mov ax, cs
    mov ds, ax
    mov ss, ax
    mov sp, 0x700
    mov ax, 0x40
    mov es, ax

    mov eax, 0x450
    mov dr0, eax
    mov eax, 0x00050001             ; LEN0 01 (2 bytes), R/W0 01 (write), L0
    mov dr7, eax

    ; 1. Word write to 0040:0050 traps after the instruction.
    mov word [es:0x50], 0x1234
after_write1:
    nop
    mov ax, 1
    cmp word [log_count], 1
    jne fail
    cmp word [log_ip], after_write1
    jne fail
    test word [log_dr6], 1          ; B0
    jz fail

    ; 2. Byte write to 0040:0051 (inside the 2-byte range) traps.
    mov byte [es:0x51], 0x56
    nop
    mov ax, 2
    cmp word [log_count], 2
    jne fail

    ; 3. Write to 0040:0052 (outside the range) does not trap.
    mov word [es:0x52], 0x789a
    nop
    mov ax, 3
    cmp word [log_count], 2
    jne fail

    ; 4. A read of 0040:0050 does not trap (write-only breakpoint).
    mov bx, [es:0x50]
    nop
    mov ax, 4
    cmp word [log_count], 2
    jne fail

    xor eax, eax
    mov dr7, eax
    mov al, 1
    out STATUS_PORT, al
    hlt

db_handler:
    push bp
    mov bp, sp
    push eax
    mov eax, dr6
    mov [log_dr6], ax
    mov ax, [bp + 2]                ; pushed IP
    mov [log_ip], ax
    inc word [log_count]
    xor eax, eax
    mov dr6, eax
    pop eax
    pop bp
    iret

fail:
    out DATA_PORT, ax
    mov al, 0xff
    out STATUS_PORT, al
    hlt

align 2
log_count: dw 0
log_ip:    dw 0
log_dr6:   dw 0
