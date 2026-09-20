
; a data write whose page is absent, immediately followed (within the same fetch group) by an
; instruction on an absent code page.  Architecturally the data #PF (older instruction) must be delivered first
; (EIP = store start, CR2 = data page), then after restart the ifetch #PF (CR2 = code page).  The store must land.
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
    mov ecx, [esp+0x24]             ; frame EIP (after pushad)
    mov edx, [ds:pfn]
    cmp edx, 0
    jne second
    ; first fault must be the DATA fault at the store
    cmp eax, 0x13150
    jne fail_first
    cmp ecx, 0x1ff9
    jne fail_first
    jmp mapit
second:
    cmp eax, 0x12000
    jne fail_second
    cmp ecx, 0x2000
    jne fail_second
mapit:
    inc dword [ds:pfn]
    shr eax, 12
    shl eax, 2
    add eax, 0x1000 - CODE_LINEAR
    mov dword [ds:eax], 0
    mov ebx, cr2
    and ebx, 0xfffff000
    or ebx, 3
    mov [ds:eax], ebx
    mov eax, cr3
    mov cr3, eax
    popad
    add esp, 4
    iretd
fail_first:
    mov edx, 1
    jmp fail_out
fail_second:
    mov edx, 2
fail_out:
    mov [ds:0x100], edx
    mov edx, eax
    mov eax, [ds:0x100]
    out DATA_PORT, eax
    mov eax, edx
    out DATA_PORT, eax
    mov eax, ecx
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
    mov ebx, 0x11111111

    jmp tail_start
times 8183 - ($ - $$) db 0x90
tail_start:
    nop
    nop
store_ins:
    mov [ds:0x3150], ebx

times 0x2000 - ($ - $$) db 0x90
check:
    ; ifetch fault page: reached after both faults restarted
    cmp dword [ds:pfn], 2
    jne fail
    cmp dword [ds:0x3150], 0x11111111
    jne fail_lost
    mov al, 1
    out STATUS_PORT, al
    hlt
fail_lost:
    mov eax, 0x77
    out DATA_PORT, eax
    mov eax, [ds:0x3150]
    out DATA_PORT, eax
    jmp fail
