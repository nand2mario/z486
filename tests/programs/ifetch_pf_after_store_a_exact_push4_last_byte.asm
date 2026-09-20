; ifetch_pf_after_store_a_exact_push4_last_byte
;
; Four PUSHes end exactly on the last byte of code page N (5+1+5+5 bytes); the next instruction, MOV 
; [ebp-4],edi, starts at byte 0 of page N+1, which is NOT present. The instruction-fetch #PF is raised 
; while the last PUSH store is still in flight.
;
; Ring 3, paging on. A ring-0 kernel repeats the scenario 30 times, re-arming the
; not-present PTE each time, and checks the exact ordered list of #PFs (CR2, EIP,
; ESP, error code), the pushed stack words and the final state.
; Regression for a hang: an instruction-fetch #PF that pulses while the microcode
; ROM is held by an in-flight store used to leave the pre-fault micro-op to run
; after fault delivery started, which dropped uc_active in the middle of the #PF
; microcode and left the CPU stalled with interrupts enabled.
BITS 32
org 0
C0 equ 8
D0 equ 16
C3 equ 43
D3 equ 51
TSS_SEL equ 24
KSTACK equ 0x58000
IMG equ 0x10000
V_ITER equ 0x30100
V_PFC equ 0x30104
V_INTR equ 0x30108
V_NEXP equ 0x3010c
V_ISOK equ 0x30110
ESP0 equ 0x56fae4
EBP0 equ 0x56fb00
FTAB_LIN equ ftab+0x10000
FTABW_LIN equ ftab_warm+0x10000
times 0x400-($-$$) db 0x90
start:
 cli
 mov esp,KSTACK
 lgdt [cs:gdt_desc]
 lidt [cs:idt_desc]
 mov ax,D0
 mov ds,ax
 mov es,ax
 mov ss,ax
 mov fs,ax
 mov gs,ax
 mov ax,TSS_SEL
 ltr ax
 or dword [0x4],4
 or dword [0x708],4
 mov eax,cr3
 mov cr3,eax
 mov dword [V_ITER],0
 mov dword [V_INTR],0
 jmp next_iter
next_iter:
 cli
 mov esp,KSTACK
 mov ax,D0
 mov ds,ax
 mov es,ax
 mov fs,ax
 mov gs,ax
 mov dword [V_PFC],0
 mov dword [0x44ae0],0xDEADBE00+1
 mov dword [0x44adc],0xDEADBE00+2
 mov dword [0x44ad8],0xDEADBE00+3
 mov dword [0x44ad4],0xDEADBE00+4
 mov dword [0x44ad0],0xDEADBE00+5
 mov dword [0x44acc],0xDEADBE00+6
 mov dword [0x44ac8],0xDEADBE00+7
 mov dword [0x44afc],0xA55AA55A
 mov dword [0x3f8c],0x0
 mov dword [V_NEXP],1
 mov dword [V_ISOK],0
 mov dword [0x25b8],0x40027
 invlpg [0x70be3000]
 invlpg [0x56e000]
 invlpg [0x56f000]
 mov ecx,[V_ITER]
 and ecx,15
 add ecx,3
.pad: dec ecx
 jnz .pad
 mov edi,0
 mov ebp,EBP0
 mov ebx,0x2468ace0
 mov ax,D3
 mov ds,ax
 mov es,ax
 mov fs,ax
 mov gs,ax
 push dword D3
 push dword ESP0
 push dword 0x202
 push dword C3
 push dword 0x70be2ff0
 iretd
%macro SAVE 0
 pushad
 push ds
 push es
 mov ax,D0
 mov ds,ax
 mov es,ax
%endmacro
%macro RESTORE 0
 pop es
 pop ds
 popad
%endmacro
pf_handler:
 SAVE
 mov ebx,[V_PFC]
 cmp ebx,[V_NEXP]
 jae fail_pf_extra
 mov esi,ebx
 shl esi,4
 cmp dword [V_ISOK],0
 jne .usew
 add esi,FTAB_LIN
 jmp .cmp
.usew: add esi,FTABW_LIN
.cmp:
 mov eax,cr2
 cmp eax,[esi]
 jne fail_pf_cr2
 mov ecx,[esp+44]
 cmp ecx,[esi+4]
 jne fail_pf_eip
 mov ecx,[esp+56]
 cmp ecx,[esi+8]
 jne fail_pf_esp
 mov ecx,[esp+40]
 and ecx,3            ; compare P/W only: the U/S bit depends on the CPL used for the
 mov edx,[esi+12]      ; fetch, which this fixture does not test
 and edx,3
 cmp ecx,edx
 jne fail_pf_err
 cmp dword [esp+48],C3
 jne fail_pf_seg
 cmp dword [esp+60],D3
 jne fail_pf_seg
 and eax,0xfffff000
 cmp eax,0x70be3000
 je .mapn1
 cmp eax,0x56e000
 je .maps1
 jmp fail_pf_page
.mapn1:
 mov dword [0x3f8c],0x18027
 invlpg [0x70be3000]
 jmp .done
.maps1: mov dword [0x25b8],0x40027
 invlpg [0x56e000]
.done: inc dword [V_PFC]
 RESTORE
 add esp,4
 iretd
report_handler:
 SAVE
 cmp dword [esp+36],0x53510000
 jne fail_magic
 mov eax,[V_PFC]
 cmp eax,[V_NEXP]
 jne fail_pfc
 cmp dword [esp+52],ESP0
 jne fail_esp
 cmp dword [0x44ae0],0x2000000
 jne fail_stack1
 cmp dword [0x44adc],0x0
 jne fail_stack2
 cmp dword [0x44ad8],0x70be303c
 jne fail_stack3
 cmp dword [0x44ad4],0x80000002
 jne fail_stack4
 cmp dword [0x44afc],0
 jne fail_store
 inc dword [V_ITER]
 cmp dword [V_ITER],30
 jb next_iter
 mov eax,0x53490000
 out 0xe4,eax
 mov al,1
 out 0xe0,al
 hlt
isr:
 SAVE
 inc dword [V_INTR]
 RESTORE
 iretd
fail_pf_extra: mov eax,0x4C330100+0
 jmp fail
fail_pf_cr2: mov eax,0x4C330100+1
 jmp fail
fail_pf_eip: mov eax,0x4C330100+2
 jmp fail
fail_pf_esp: mov eax,0x4C330100+3
 jmp fail
fail_pf_err: mov eax,0x4C330100+4
 jmp fail
fail_pf_seg: mov eax,0x4C330100+5
 jmp fail
fail_pf_page: mov eax,0x4C330100+6
 jmp fail
fail_magic: mov eax,0x4C330100+7
 jmp fail
fail_pfc: mov eax,0x4C330100+8
 jmp fail
fail_esp: mov eax,0x4C330100+9
 jmp fail
fail_stack1: mov eax,0x4C330100+10
 jmp fail
fail_stack2: mov eax,0x4C330100+11
 jmp fail
fail_stack3: mov eax,0x4C330100+12
 jmp fail
fail_stack4: mov eax,0x4C330100+13
 jmp fail
fail_store: mov eax,0x4C330100+14
 jmp fail
fail_exc: mov eax,0x4C330100+15
 jmp fail
fail: mov dx,0xe4
 out dx,eax
 mov dx,0xe0
 mov al,0xff
 out dx,al
 hlt
kern_end:
align 16
ftab:
 dd 0x70be3000,0x70be3000,0x56fad4,0x4
 dd 0,0,0,0
ftab_warm:
 dd 0,0,0,0
end_of_kernel_code:
times 0x2000-($-$$) db 0
times 0x2000-($-$$) db 0
 times 4096 db 144
times 0x3000-($-$$) db 0
 times 4064 db 144
 db 144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,104,0,0,0,2,87,104,60,48,190,112,104,2,0,0,128
times 0x5000-($-$$) db 0
align 8
gdt: dq 0
 dq 0x00cf9b010000ffff
 dq 0x00cf93000000ffff
 dw 0x67,tss
 db 1,0x89,0,0
 dq 0
 dq 0x00cffb000000ffff
 dq 0x00cff3000000ffff
gdt_end:
gdt_desc: dw gdt_end-gdt-1
 dd gdt+0x10000
times 0x5100-($-$$) db 0
tss: dd 0,KSTACK,D0
 times 90 db 0
 dw 104
times 0x5200-($-$$) db 0
align 8
idt:
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw pf_handler,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw isr,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw fail_exc,C0
 db 0,0x8e
 dw 0
 dw report_handler,C0
 db 0,0xee
 dw 0
idt_end:
idt_desc: dw idt_end-idt-1
 dd idt+0x10000
times 0x6000-($-$$) db 0
 times 1728 db 0
 db 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,189,112,0,0,0,0,0,0,0,0
 times 2336 db 0
times 0x7000-($-$$) db 0
 db 184,1,0,0,0,194,16,0,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144
 times 4064 db 144
times 0x8000-($-$$) db 0
 db 137,125,252,255,21,212,22,189,112,133,192,15,132,239,7,0,0,57,125,252,15,133,230,7,0,0,129,252,228,250,86,0
 db 15,133,218,7,0,0,184,0,0,81,83,205,128,235,254,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144
 times 1984 db 144
 db 184,81,0,51,76,205,128,235,254,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144
 times 2016 db 144
times 0x9000-($-$$) db 0
 times 4096 db 144
