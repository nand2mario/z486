
; a REP string instruction immediately followed by an instruction whose ifetch faults (absent code
; page).  The ifetch #PF of the YOUNGER instruction must not squash the register write-back (ECX/EDI/ESI update)
; of the OLDER, already-retiring REP MOVSD: after the handler maps the page and IRETs, ECX must be 0, EDI advanced.
BITS 32
ORG 0
STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
CODE_LINEAR equ 0x00010000
align 8
gdt:
    dq 0
    dq 0x00cf9b010000ffff
    dq 0x00cf93010000ffff
gdt_end:
gdt_desc:
    dw gdt_end - gdt - 1
    dd CODE_LINEAR + gdt
pfn: dd 0
pf_handler:
    pushad
    mov eax, cr2
    inc dword [ds:pfn]
    shr eax, 12
    shl eax, 2
    add eax, 0x1000 - CODE_LINEAR
    mov ebx, cr2
    and ebx, 0xfffff000
    or ebx, 3
    mov [ds:eax], ebx
    mov eax, cr3
    mov cr3, eax
    popad
    add esp, 4
    iretd
fail_out:
    out DATA_PORT, eax
    mov eax, ecx
    out DATA_PORT, eax
    mov eax, edi
    out DATA_PORT, eax
    mov eax, [ds:pfn]
    out DATA_PORT, eax
fail:
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
    dd CODE_LINEAR + idt
times 0x0200 - ($ - $$) db 0x90
start:
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp, 0x0e00
    mov dword [ds:0x800], 0xdeadbeef
    mov dword [ds:0x900], 0
    mov dword [ds:0x904], 0x55555555
    jmp tail_start
times 8175 - ($ - $$) db 0x90
tail_start:
    mov esi, 0x800
    mov edi, 0x900
    mov ecx, 1
    rep movsd
    mov esi, 0x8000
check:
    cmp ecx, 0
    jne bad_ecx
    cmp edi, 2308
    jne bad_edi
    cmp dword [ds:0x900], 0xdeadbeef
    jne bad_data
    cmp dword [ds:pfn], 1
    jne bad_pfn
    mov al, 1
    out STATUS_PORT, al
    hlt
bad_ecx:
    mov eax, 1
    jmp fail_out
bad_edi:
    mov eax, 2
    jmp fail_out
bad_data:
    mov eax, 3
    jmp fail_out
bad_pfn:
    mov eax, 4
    jmp fail_out
