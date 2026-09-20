; Reduce BFF96FEB..BFF97000 with the guest's branch displacement and
; absolute data addresses. CPL0 and synthetic frame remain deliberate limits.
BITS 32
ORG 0
CODE_LINEAR equ 0xbff96000
POINTER equ 0xbffc99e0
TARGET equ 0x81b1200c
STATUS_PORT equ 0xe0
DATA_PORT equ 0xe4

align 8
gdt:
    dq 0
    dq 0xbfcf9bf96000ffff
    dq 0x00cf93000000ffff
gdt_end:
gdt_desc:
    dw gdt_end - gdt - 1
    dd CODE_LINEAR + gdt

pf_handler:
    mov eax, cr2
    and eax, 0xfffff000
    cmp eax, 0xbff97000
    jne fail
    cmp dword [ss:esp], 0
    jne fail
    cmp dword [ss:esp + 4], fault_site
    jne fail
    cmp esp, 0x1b310
    jne fail
    test dword [ss:esp + 12], 0x200
    jz fail
    cmp edx, TARGET
    jne fail
    cmp edi, 1
    jne fail
    mov eax, 0x534f0000
    out DATA_PORT, eax
    mov al, 1
    out STATUS_PORT, al
    hlt
fail:
    mov eax, 0x534f0001
    out DATA_PORT, eax
    mov al, 0xff
    out STATUS_PORT, al
    hlt

align 8
idt:
    times 14 dq 0
    dw pf_handler
    dw 8
    db 0
    db 0x8e
    dw 0
idt_end:
idt_desc:
    dw idt_end - idt - 1
    dd CODE_LINEAR + idt

times 0x200 - ($ - $$) db 0x90
start:
    ; The ordinary testbench seeds prefetch with the physical code address.
    ; An identity bootstrap alias supplies this JMP; the redirect then uses
    ; architectural CS.base for the actual high-address reduction.
    jmp setup
    nop
setup:
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp, 0x1b320
    mov dword [POINTER], TARGET
    mov dword [TARGET], 0x7562647a
    mov dword [TARGET + 4], 0x00323369
    xor eax, eax
    sti

times 0xfeb - ($ - $$) db 0x90
boundary_chain:
    test eax, eax
    jnz near absent_target
    mov edx, [POINTER]
    mov edi, 1
fault_site:
    db 0x80, 0x3a
next_page:
    db 0
    jmp fail
times 0x1083 - ($ - $$) db 0x90
absent_target:
    jmp fail
