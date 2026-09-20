; LH32 hardware trace: MOV[EDI+7C],ESI immediately followed by MOV[EDI+6C],EBX.
; A fault on the first store must name/retry that store, not its successor.
BITS 32
ORG 0
OBJECT equ 0x81b23000
BASE equ 0x81b13000
HEAP equ 0x81b33000
ALIAS equ 0x40000
COUNT equ 0x50000
PTE equ 0x2000 + ((OBJECT >> 12) & 0x3ff) * 4
DATA_PORT equ 0xe4
STATUS_PORT equ 0xe0

align 8
gdt:
    dq 0
    dq 0x00cf9b010000ffff
    dq 0x00cf93000000ffff
gdt_end:
gdt_desc:
    dw gdt_end-gdt-1
    dd 0x10000+gdt

pf_handler:
    mov edx,cr2
    cmp edx,OBJECT+0x7c
    jne bad_cr2
    cmp dword [ss:esp],2
    jne bad_error
    cmp dword [ss:esp+4],first_store
    jne bad_eip
    cmp esi,HEAP
    jne bad_register
    cmp edi,OBJECT
    jne bad_register
    cmp ebx,BASE
    jne bad_register
    cmp eax,HEAP+0x78
    jne bad_register
    cmp dword [ALIAS+0x7c],0xa5a5a5a5
    jne bad_sentinel
    cmp dword [ALIAS+0x6c],0x5a5a5a5a
    jne bad_sentinel
    inc dword [COUNT]
    cmp dword [COUNT],1
    jne bad_count
    mov dword [PTE],0x30063
    invlpg [edi]
    add esp,4
    iretd

bad_cr2:
    mov eax,edx
    out DATA_PORT,eax
    mov eax,1
    jmp fail
bad_error:
    mov eax,2
    jmp fail
bad_eip:
    mov eax,[ss:esp+4]
    out DATA_PORT,eax
    mov eax,3
    jmp fail
bad_register:
    mov eax,4
    jmp fail
bad_sentinel:
    mov eax,5
    jmp fail
bad_count:
    mov eax,6
    jmp fail

align 8
idt:
    times 14 dq 0
    dw pf_handler,8
    db 0,0x8e
    dw 0
idt_end:
idt_desc:
    dw idt_end-idt-1
    dd 0x10000+idt

times 0x400-($-$$) db 0x90
start:
    cli
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp,0x1b320
    mov dword [ALIAS+0x7c],0xa5a5a5a5
    mov dword [ALIAS+0x6c],0x5a5a5a5a
    mov dword [COUNT],0
%ifndef PRESENT_PAGE
    mov dword [PTE],0
%endif
    mov edi,OBJECT
    invlpg [edi]
    mov esi,HEAP
    mov ebx,BASE
    mov eax,HEAP+0x78
    jmp chain

times 0x47f-($-$$) db 0x90
chain:
    or eax,eax
    jz short unexpected_zero
first_store:
    mov [edi+0x7c],esi
%ifdef SERIAL_GAP
    times 16 nop
%endif
second_store:
    mov [edi+0x6c],ebx
    sub eax,ebx
    cmp eax,0x20078
    jne bad_sub
    cmp dword [OBJECT+0x7c],HEAP
    jne bad_handle
    cmp dword [OBJECT+0x6c],BASE
    jne bad_base
%ifdef PRESENT_PAGE
    cmp dword [COUNT],0
%else
    cmp dword [COUNT],1
%endif
    jne bad_count
    mov eax,0x4c530000
    out DATA_PORT,eax
    mov al,1
    out STATUS_PORT,al
    hlt
unexpected_zero:
    mov eax,10
    jmp fail
bad_handle:
    mov eax,7
    jmp fail
bad_base:
    mov eax,8
    jmp fail
bad_sub:
    mov eax,9
fail:
    out DATA_PORT,eax
    mov al,0xff
    out STATUS_PORT,al
    hlt
