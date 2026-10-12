; tss_t_bit.asm - the TSS debug-trap (T) bit
;
; A task switch to a TSS with T set (offset 64h, bit 0) raises #DB after the
; switch, before the new task's first instruction: the handler sees that
; instruction's EIP and DR6.BT (bit 15). The switch also clears DR7's local
; enables (L0-L3) and keeps the global ones (486 PRM 11.2.2, 11.2.3).
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 fail code.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

SEL_CODE0   equ 0x08
SEL_DATA0   equ 0x10
SEL_TSS_OLD equ 0x18
SEL_TSS_NEW equ 0x20

STACK_OLD   equ 0x3000
STACK_NEW   equ 0x4000

start:
    cli
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]

    mov eax, cr0
    or eax, 1
    mov cr0, eax
    jmp dword SEL_CODE0:pm_entry

BITS 32
pm_entry:
    mov ax, SEL_DATA0
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov esp, STACK_OLD

    mov ax, SEL_TSS_OLD
    ltr ax

    ; G0 and L1 enabled (DR0/DR1 at linear 0, never executed): the switch
    ; clears L1 and keeps G0.
    xor eax, eax
    mov dr0, eax
    mov dr1, eax
    mov dr6, eax
    mov eax, 0x00000006
    mov dr7, eax

    jmp SEL_TSS_NEW:0

    mov eax, 0x10
    jmp fail

task_entry:
    mov eax, 0x21
    cmp dword [log_count], 1            ; one #DB, before this instruction
    jne fail
    mov eax, 0x22
    cmp dword [log_eip], task_entry
    jne fail
    mov eax, 0x23
    test dword [log_dr6], 0x8000        ; BT
    jz fail
    mov eax, 0x24
    cmp ebx, 0x55667788                 ; the incoming task's state
    jne fail
    mov eax, dr7
    and eax, 0xFF
    cmp eax, 0x02                       ; L1 cleared, G0 kept
    mov eax, 0x25
    jne fail

    xor eax, eax
    mov dr7, eax
    mov al, STATUS_PASS
    out STATUS_PORT, al
    hlt

; #DB in the new task: log the pushed EIP and DR6, clear DR6.
db_handler:
    push eax
    mov eax, dr6
    mov [log_dr6], eax
    mov eax, [esp + 4]
    mov [log_eip], eax
    inc dword [log_count]
    xor eax, eax
    mov dr6, eax
    pop eax
    iretd

fail:
    out DATA_PORT, eax
    mov al, STATUS_FAIL
    out STATUS_PORT, al
    hlt

align 4
log_count: dd 0
log_dr6:   dd 0
log_eip:   dd 0

BITS 16
align 8
gdt:
    dq 0
    dq 0x00CF9B010000FFFF       ; 08: 32-bit code, base 10000h
    dq 0x00CF93010000FFFF       ; 10: 32-bit data, base 10000h

    ; 18: available 386 TSS, base 10000h + tss_old, limit 67h
    dw 0x0067
    dw tss_old
    db 0x01
    db 10001001b
    db 0
    db 0

    ; 20: available 386 TSS, base 10000h + tss_new, limit 67h
    dw 0x0067
    dw tss_new
    db 0x01
    db 10001001b
    db 0
    db 0
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x10000

align 8
idt:
    dq 0
    dw db_handler, SEL_CODE0    ; vector 1: #DB
    db 0, 10001110b             ; present, DPL 0, 32-bit interrupt gate
    dw 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x10000

align 4
tss_old:
    times 26 dd 0

align 4
tss_new:
    dd 0                        ; 00 backlink
    dd 0                        ; 04 ESP0
    dd 0                        ; 08 SS0
    dd 0                        ; 0C ESP1
    dd 0                        ; 10 SS1
    dd 0                        ; 14 ESP2
    dd 0                        ; 18 SS2
    dd 0                        ; 1C CR3 (paging disabled)
    dd task_entry               ; 20 EIP
    dd 0x00000002               ; 24 EFLAGS
    dd 0                        ; 28 EAX
    dd 0                        ; 2C ECX
    dd 0                        ; 30 EDX
    dd 0x55667788               ; 34 EBX
    dd STACK_NEW                ; 38 ESP
    dd 0                        ; 3C EBP
    dd 0                        ; 40 ESI
    dd 0                        ; 44 EDI
    dd SEL_DATA0                ; 48 ES
    dd SEL_CODE0                ; 4C CS
    dd SEL_DATA0                ; 50 SS
    dd SEL_DATA0                ; 54 DS
    dd 0                        ; 58 FS
    dd 0                        ; 5C GS
    dd 0                        ; 60 LDTR
    dw 1                        ; 64 debug trap: T set
    dw 0x0068                   ; 66 I/O bitmap beyond TSS limit
