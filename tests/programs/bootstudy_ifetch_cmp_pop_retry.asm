; Cold POP followed by the same split-CMP map-and-retry boundary.
; POP increments ESP to 1B324; the #PF frame starts at 1B314.
; Late split-instruction #PF must preserve predecessor writes/flags, map the
; missing code page, return to the same CMP, and complete it exactly once.
BITS 32
ORG 0
CODE_LINEAR equ 0xbff96000
POINTER equ 0xbffc99e0
TARGET equ 0x81b1200c
COUNT equ 0x1b100
NEXT_PTE equ 0x2e5c
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
    cmp eax, 0xbff97000
    jne fail
    cmp dword [ss:esp], 0
    jne fail
    cmp dword [ss:esp + 4], fault_site
    jne fail
    cmp esp, 0x1b314
    jne fail
    cmp edx, TARGET
    jne fail
    cmp edi, 1
    jne fail
    ; TEST EAX,EAX set ZF/PF and cleared CF/SF/OF before the incomplete CMP.
    mov eax, [ss:esp + 12]
    and eax, 0x8c5
    cmp eax, 0x44
    jne fail
    test dword [ss:esp + 12], 0x200
    jz fail
    cmp dword [COUNT], 0
    jne fail
    cmp dword [NEXT_PTE], 0
    jne fail
    mov dword [COUNT], 1
    ; Mapping order fixes the high-code PT at physical 2000. The 2000
    ; identity mapping lets the handler update its absent-page PTE directly.
    mov dword [NEXT_PTE], 0x11003
    mov eax, cr3
    mov cr3, eax
    add esp, 4
    iretd

pass:
    mov eax, 0x53500000
    out DATA_PORT, eax
    mov al, 1
    out STATUS_PORT, al
    hlt
fail:
    mov eax, 0x53500001
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
    jmp setup
    nop
setup:
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp, 0x1b320
    mov dword [ss:0x1b320], 1
    xor edi, edi
    mov dword [COUNT], 0
    mov dword [POINTER], TARGET
    mov dword [TARGET], 0x7562647a
    mov dword [TARGET + 4], 0x00323369
    xor eax, eax
    sti

times 0xfeb - ($ - $$) db 0x90
    test eax, eax
    jnz near absent_target
    mov edx, [POINTER]
    times 4 nop
    pop edi
fault_site:
    db 0x80, 0x3a
next_page:
    db 0
    ; 7a - 0 has no CF/PF/AF/ZF/SF/OF flags set.
    pushfd
    pop eax
    and eax, 0x8d5
    jnz fail
    cmp dword [COUNT], 1
    jne fail
    cmp edx, TARGET
    jne fail
    cmp edi, 1
    jne fail
    cmp esp, 0x1b324
    jne fail
    jmp pass
times 0x1083 - ($ - $$) db 0x90
absent_target:
    jmp fail
