; Captured Windows chain, now with the destination page deliberately absent.
BITS 32
ORG 0
%ifndef FLAG_SET
%define FLAG_SET 0
%endif
STATUS_PORT equ 0xe0
DATA_PORT equ 0xe4
TARGET equ 0x8164bffc
FRAME equ 0x1b330
COUNT equ 0x30010

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
    cmp edx,TARGET
    jne fail
    cmp dword [ss:esp],2
    jne fail
    cmp dword [ss:esp+4],fault_store
    jne fail
    cmp eax,TARGET
    jne fail
    cmp ebp,FRAME
    jne fail
    test dword [ss:esp+12],0x40
%if FLAG_SET
    jnz fail
%else
    jz fail
%endif
    inc dword [COUNT]
    mov dword [ss:esp+4],after_fault
    add esp,4
    iretd

fail:
    mov eax,0x534d0001
    out DATA_PORT,eax
    mov al,0xff
    out STATUS_PORT,al
    hlt

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

times 0x200-($-$$) db 0x90
start:
    cli
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp,0x1b320
    mov ebp,FRAME
    mov dword [ebp-4],TARGET
    mov byte [ebp+0x15],FLAG_SET*4
    mov dword [COUNT],0
%ifdef NO_CHAIN_TEST
    test byte [ebp+0x15],4
%endif
    sti
    jmp chain

after_fault:
    cmp dword [COUNT],1
    jne fail
    mov eax,0x534d0000
    out DATA_PORT,eax
    mov al,1
    out STATUS_PORT,al
    hlt

times 0xe7f-($-$$) db 0x90
chain:
    db 0x8b,0x45,0xfc
%ifdef NO_CHAIN_TEST
    times 4 nop
%else
    db 0xf6,0x45,0x15,4
%endif
fault_store:
    db 0xc7,0,0,0,0,0xa0
%ifdef NOP_SUCCESSOR
    nop
%elifdef MOV_SUCCESSOR
    mov ecx,eax
%else
    jne short branch_taken
%endif
    jmp fail
branch_taken:
    jmp fail
