; Direct m32 x87 reads must preserve precise page-fault ordering. The operand
; crosses from the sole mapped data page into an absent page; FLD must fault
; without pushing a value onto the x87 stack.

BITS 32
ORG 0

STATUS_PORT equ 0xE0
DATA_BASE   equ 0x20000000
DATA_LAST   equ 0x00000ffe

align 8
gdt:
    dq 0
    dq 0x00cf9b010000ffff     ; 0x08: code, base 0x00010000
    dq 0x20cf93000000ffff     ; 0x10: data, base 0x20000000
    dq 0x30cf93000000ffff     ; 0x18: stack, base 0x30000000
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd 0x00010000 + gdt

pf_handler:
    mov eax, cr2
    cmp eax, DATA_BASE + 0x1000
    jne fail_handler
    cmp dword [ss:esp], 0                   ; read from absent supervisor page
    jne fail_handler
    cmp dword [ss:esp + 4], fault_fld
    jne fail_handler
    fnstsw ax
    and eax, 0x3800
    jnz fail_handler                        ; failed FLD must not push
    mov dword [ss:esp + 4], after_fld_fault
    add esp, 4                              ; discard #PF error code
    iretd

fail_handler:
    mov al, 0xff
    out STATUS_PORT, al
    hlt

align 8
idt:
    times 14 dq 0
    dw pf_handler
    dw 0x0008
    db 0
    db 0x8e
    dw 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd 0x00010000 + idt

times 0x200 - ($ - $$) db 0x90
start:
    cli
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp, 0x8000
    fninit

fault_fld:
    fld dword [DATA_LAST]
after_fld_fault:
    fnstsw ax
    and eax, 0x3800
    jnz fail

    mov al, 1
    out STATUS_PORT, al
    hlt

fail:
    mov al, 0xff
    out STATUS_PORT, al
    hlt

times 0x400 - ($ - $$) db 0
