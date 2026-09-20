
; ring-3 PUSH to an absent stack page (write #PF discovered late) immediately followed by CALL rel32
; (early redirect of the prefetcher to the callee).  The #PF handler must run from its own entry, the pushed frame
; EIP must be the PUSH, and the callee must not execute before the handler.  Repeated with the stack page evicted
; by a ring-0 service each time.
BITS 32
ORG 0
STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
IDT_OFF equ 0xE000
TSS_OFF equ 0xE800
CODE_LINEAR equ 0x00010000
align 8
gdt:
    dq 0
    dq 0x00cf9b010000ffff
    dq 0x00cf93010000ffff
    dq 0x00cffb010000ffff          ; 0x18 ring3 code
    dq 0x00cff3010000ffff          ; 0x20 ring3 data
    dw 0x0067                      ; 0x28 TSS
    dw (0x10000 + TSS_OFF) & 0xffff
    db ((0x10000 + TSS_OFF) >> 16) & 0xff
    db 0x89
    db 0
    db ((0x10000 + TSS_OFF) >> 24) & 0xff
gdt_end:
gdt_desc:
    dw gdt_end - gdt - 1
    dd 0x10000 + gdt
idt_desc:
    dw 0x10f
    dd 0x10000 + idt
pfn: dd 0
calls: dd 0
iters: dd 0
expect_eip: dd 0
pf_handler:
    pushad
    mov eax, cr2
    mov ecx, [esp+0x24]
    cmp eax, 0x1BFFC
    jne fail_first_a
    mov edx, [esp+0x30]            ; saved user ESP
    cmp edx, 0xC000
    jne fail_first_esp
    mov edx, [ds:expect_eip]
    cmp ecx, edx
    jne fail_first_b
    inc dword [ds:pfn]
    mov ebx, 0x1B000 | 0x27
    mov [ds:0x1000 + 0x1B*4 - 0x10000], ebx
    mov eax, cr3
    mov cr3, eax
    popad
    add esp, 4
    iretd
fail_first_a:
    mov edx, 1
    jmp fail_out
fail_first_esp:
    mov edx, 2
    jmp fail_out
fail_first_b:
    mov edx, 3
fail_out:
    mov eax, edx
    out DATA_PORT, eax
    mov eax, cr2
    out DATA_PORT, eax
    mov eax, ecx
    out DATA_PORT, eax
    mov eax, [esp+0x30]
    out DATA_PORT, eax
fail:
    mov al, 0xff
    out STATUS_PORT, al
    hlt
gp_handler:
    pushad
    mov eax, 0x13
    out DATA_PORT, eax
    mov eax, [esp+0x24]
    out DATA_PORT, eax
    mov eax, [esp+0x20]
    out DATA_PORT, eax
    jmp fail
early_sub:
    mov eax, 0x55
    out DATA_PORT, eax
    jmp fail
evict_svc:                          ; int 0x21: ring0 service, evict the user stack page
    pushad
    mov dword [ds:0x1000 + 0x1B*4 - 0x10000], 0
    mov eax, cr3
    mov cr3, eax
    popad
    iretd
align 8
idt:
    times 13 dq 0
    dw gp_handler
    dw 0x0008
    db 0
    db 0x8e
    dw 0
    dw pf_handler
    dw 0x0008
    db 0
    db 0x8e
    dw 0
    times 18 dq 0
    dw evict_svc                    ; 33
    dw 0x0008
    db 0
    db 0xee
    dw 0
idt_end:
times 0x0400 - ($ - $$) db 0x90
main:
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp, 0xF000
    mov ax, 0x28
    ltr ax
    mov ax, 0x23
    mov ds, ax
    mov es, ax
    push dword 0x23
    push dword 0xC000
    push dword 0x3002
    push dword 0x1B
    push dword ring3
    iretd
ring3:
    mov ecx, 6
loop_top:
    mov esi, 0x2000
    int 0x21
    nop
    nop
    mov dword [ds:expect_eip], push_ins
push_ins:
    push eax

    call sub1
    inc dword [ds:iters]
    dec ecx
    jnz loop_top
    cmp dword [ds:pfn], 6
    jne fail
    cmp dword [ds:calls], 6
    jne fail
    cmp esp, 0xC000
    jne fail
    mov al, 1
    out STATUS_PORT, al
    hlt
sub1:
    cmp dword [ds:pfn], 0
    je early_sub
    inc dword [ds:calls]
    ret 4
times TSS_OFF - ($ - $$) db 0
tss:
    dd 0
    dd 0xF000
    dd 0x10
    times 22 dd 0
    dw 0
    dw 0x68
times 0x10000 - ($ - $$) db 0x90
