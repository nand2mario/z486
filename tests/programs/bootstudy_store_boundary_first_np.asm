; Two writes on distinct pages: preserve older writes and reject younger ones.
BITS 32
ORG 0
FIRST equ 0x81b23000
SECOND equ 0x81b24000
ALIAS_FIRST equ 0x40000
ALIAS_SECOND equ 0x41000
FIRST_PTE equ 0x2000+((FIRST>>12)&0x3ff)*4
SECOND_PTE equ 0x2000+((SECOND>>12)&0x3ff)*4
VALUE_FIRST equ 0x81b33000
VALUE_SECOND equ 0x81b13000
COUNT equ 0x50000
DATA_PORT equ 0xe4
STATUS_PORT equ 0xe0
%ifdef SECOND_FAULT
FAULT_ADDR equ SECOND+0x6c
FAULT_PTE equ SECOND_PTE
REPAIRED_PTE equ 0x60063
%else
FAULT_ADDR equ FIRST+0x7c
FAULT_PTE equ FIRST_PTE
REPAIRED_PTE equ 0x30063
%endif
%ifdef READ_ONLY
ERROR_CODE equ 3
%else
ERROR_CODE equ 2
%endif

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
    cmp edx,FAULT_ADDR
    jne bad_cr2
    cmp dword [ss:esp],ERROR_CODE
    jne bad_error
%ifdef SECOND_FAULT
    cmp dword [ss:esp+4],second_store
%else
    cmp dword [ss:esp+4],first_store
%endif
    jne bad_eip
    cmp esi,VALUE_FIRST
    jne bad_register
    cmp edi,FIRST
    jne bad_register
    cmp ebx,VALUE_SECOND
    jne bad_register
    cmp eax,VALUE_FIRST+0x78
    jne bad_register
%ifdef SECOND_FAULT
    cmp dword [ALIAS_FIRST+0x7c],VALUE_FIRST
%else
    cmp dword [ALIAS_FIRST+0x7c],0xa5a5a5a5
%endif
    jne bad_first
    cmp dword [ALIAS_SECOND+0x6c],0x5a5a5a5a
    jne bad_second
    inc dword [COUNT]
    cmp dword [COUNT],1
    jne bad_count
    mov dword [FAULT_PTE],REPAIRED_PTE
    invlpg [FAULT_ADDR]
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
bad_first:
    mov eax,5
    jmp fail
bad_second:
    mov eax,6
    jmp fail
bad_count:
    mov eax,7
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
    mov dword [ALIAS_FIRST+0x7c],0xa5a5a5a5
    mov dword [ALIAS_SECOND+0x6c],0x5a5a5a5a
    mov dword [COUNT],0
%ifdef READ_ONLY
    mov dword [FAULT_PTE],REPAIRED_PTE & ~2
%else
    mov dword [FAULT_PTE],0
%endif
    mov edi,FIRST
    invlpg [edi]
    invlpg [SECOND]
    mov esi,VALUE_FIRST
    mov ebx,VALUE_SECOND
    mov eax,VALUE_FIRST+0x78
    jmp chain

times 0x47f-($-$$) db 0x90
chain:
    or eax,eax
    jz short unexpected_zero
first_store:
    mov [edi+0x7c],esi
second_store:
    mov [edi+0x106c],ebx
    sub eax,ebx
    cmp eax,0x20078
    jne bad_sub
    cmp dword [FIRST+0x7c],VALUE_FIRST
    jne bad_first
    cmp dword [SECOND+0x6c],VALUE_SECOND
    jne bad_second
    cmp dword [COUNT],1
    jne bad_count
    mov eax,0x53500000
    out DATA_PORT,eax
    mov al,1
    out STATUS_PORT,al
    hlt
unexpected_zero:
    mov eax,8
    jmp fail
bad_sub:
    mov eax,9
fail:
    out DATA_PORT,eax
    mov al,0xff
    out STATUS_PORT,al
    hlt
