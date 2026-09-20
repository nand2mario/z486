; A complete MOV EBP,ESP ends at a page boundary. A fetch fault for the
; following page must not discard that older instruction's register write.
BITS 32
ORG 0

STATUS_PORT equ 0xe0
DATA_PORT equ 0xe4
CODE_LINEAR equ 0x10000

align 8
gdt:
    dq 0
    dq 0x00cf9b010000ffff
    dq 0x00cf93010000ffff
gdt_end:
gdt_desc:
    dw gdt_end - gdt - 1
    dd CODE_LINEAR + gdt

pf_handler:
    mov eax, cr2
    and eax, 0xfffff000
    cmp eax, CODE_LINEAR + 0x1000
    jne fail_cr2
    cmp dword [ss:esp], 0
    jne fail_code
    cmp dword [ss:esp + 4], next_page
    jne fail_eip
    cmp esp, 0xfeec
    jne fail_stack
    ; PUSH at FFD has decremented ESP from FF00 to FEFC. The complete
    ; 8B EC at FFE must have committed before #PF is delivered for 1000.
    cmp ebp, 0xfefc
    jne fail_ebp
    cmp dword [ss:0xfefc], 0x12345678
    jne fail_saved
    mov eax, 0x534b0000
    out DATA_PORT, eax
    mov al, 1
    out STATUS_PORT, al
    hlt

fail_ebp:
    mov eax, ebp
    out DATA_PORT, eax
    mov eax, 0x534b0001
    jmp fail
fail_cr2:
    mov eax, 0x534b0002
    jmp fail
fail_code:
    mov eax, 0x534b0003
    jmp fail
fail_eip:
    mov eax, [ss:esp + 4]
    out DATA_PORT, eax
    mov eax, 0x534b0004
    jmp fail
fail_stack:
    mov eax, esp
    out DATA_PORT, eax
    mov eax, 0x534b0005
    jmp fail
fail_saved:
    mov eax, 0x534b0006
fail:
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
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov ebp, 0x12345678
    mov esp, 0xff00
%ifdef WARM_STACK
    mov dword [ss:0xfefc], 0x87654321
    mov eax, [ss:0xfefc]
%endif
%ifdef ENTER_CALL
    mov esp, 0xff04
    call wrapper
%endif
%ifdef ENTER_JUMP
    jmp wrapper
%endif
    ; Sequential execution lets the prefetcher discover the absent page
    ; ahead of the last complete instruction, as in the hardware trace.
times 0xffd - ($ - $$) db 0x90
wrapper:
    push ebp
    db 0x8b, 0xec
next_page:
    nop
