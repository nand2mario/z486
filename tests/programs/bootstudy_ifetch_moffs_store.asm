; Observed BFF37FC4..BFF37FFF ends with an incomplete 66 A3 moffs store.
; Synthetic CPL0 frame/data; final address byte BF is a reconstruction.
BITS 32
ORG 0
CODE_LINEAR equ 0xbff37000
DATA equ 0xbff40000
ALIAS equ 0x40000
COUNT equ 0x1b100
NEXT_PTE equ 0x2ce0
STATUS_PORT equ 0xe0
DATA_PORT equ 0xe4

align 8
gdt:
    dq 0
    dq 0xbfcf9bf37000ffff
    dq 0x00cf93000000ffff
gdt_end:
gdt_desc:
    dw gdt_end-gdt-1
    dd CODE_LINEAR+gdt

pf_handler:
    mov ebx,cr2
    cmp ebx,CODE_LINEAR+0x1000
    jne fail
    cmp dword [ss:esp],0
    jne fail
    cmp dword [ss:esp+4],fault_site
    jne fail
    cmp esp,0x1b310
    jne fail
    cmp eax,0x16f
    jne fail
    cmp ecx,0x5a37
    jne fail
    cmp dword [DATA+0x284],0x12345678
    jne fail
    cmp dword [DATA+0x288],0x12345678
    jne fail
%ifdef NOP_PREDECESSOR
    cmp word [DATA+0x296],0xa5a5
%else
    cmp word [DATA+0x296],cx
%endif
    jne fail
    cmp word [DATA+0x2c2],0xa55a
    jne fail
    cmp dword [COUNT],0
    jne fail
    test dword [ss:esp+12],0x200
    jz fail
    mov dword [COUNT],1
    mov dword [NEXT_PTE],0x11003
    mov ebx,cr3
    mov cr3,ebx
    add esp,4
    iretd

fail:
    mov eax,0x4d530001
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
    dd CODE_LINEAR+idt

times 0x200-($-$$) db 0x90
start:
    jmp setup
    nop
setup:
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp,0x1b320
    mov dword [COUNT],0
    mov word [ALIAS+0x296],0xa5a5
    mov word [ALIAS+0x2c2],0xa55a
    mov eax,0x16f
    mov ecx,0x5a37
    mov edx,0x12345678
    sti

times 0xfc4-($-$$) db 0x90
boundary_chain:
    mov [DATA+0x284],edx
    mov [DATA+0x294],ax
    mov [DATA+0x288],edx
    mov dword [DATA+0x2c4],0xbff37c08
    mov [DATA+0x292],cx
    mov [DATA+0x2ba],ecx
    mov [DATA+0x28e],cx
%ifdef NOP_PREDECESSOR
    times 7 nop
%else
    mov [DATA+0x296],cx
%endif
fault_site:
    db 0x66,0xa3,0xc2,0x02,0xf4
next_page:
    db 0xbf
    cmp word [DATA+0x2c2],ax
    jne fail
    cmp eax,0x16f
    jne fail
    cmp ecx,0x5a37
    jne fail
    cmp esp,0x1b320
    jne fail
%ifdef PRESENT_CODE
    cmp dword [COUNT],0
%else
    cmp dword [COUNT],1
%endif
    jne fail
    mov eax,0x4d530000
    out DATA_PORT,eax
    mov al,1
    out STATUS_PORT,al
    hlt
