; Two-pass CPL3/LDT lower-stack #PF fixture.  Pass 0 warms the wrapper with
; the lower page present; pass 1 clears only that PTE and retries the same code.
BITS 32
org 0
%ifndef WARM_STACK_FAULT_INDEX
%define WARM_STACK_FAULT_INDEX 4
%endif
%ifndef WARM_STACK_FAULT_RO
%define WARM_STACK_FAULT_RO 0
%endif
%if WARM_STACK_FAULT_INDEX < 0 || WARM_STACK_FAULT_INDEX > 4
%error WARM_STACK_FAULT_INDEX must select wrapper push 0 through 4
%endif
%if WARM_STACK_FAULT_RO < 0 || WARM_STACK_FAULT_RO > 1
%error WARM_STACK_FAULT_RO must be 0 (nonpresent) or 1 (read-only)
%endif
C0 equ 8
D0 equ 0x10
TSS_SEL equ 0x18
LDT_SEL equ 0x20
C3 equ 0x167
D3 equ 0x16f
F3 equ 0x1977
; Each index moves the initial caller ESP so its selected wrapper PUSH stores
; at the same lower-page address 005feffc.
USTACK equ 0x005ff008 + (4 * WARM_STACK_FAULT_INDEX)
KSTACK equ 0x00058000
STRING equ 0x30020
PF_COUNT equ 0x30100
RUN equ 0x30104
STACK_ALIAS equ 0x40000
SENTINEL equ 0xa55aa55a
; 005fe000 is PDE 1; the generated second page table starts at physical 2000.
STACK_GROW_PTE equ 0x2000 + (((0x005fe000 >> 12) & 0x3ff) * 4)
STACK_GROW_PTE_VALUE equ 0x00070067
STACK_GROW_PTE_RO_VALUE equ 0x00070065 ; P|U|A|D, physical 70000
%if WARM_STACK_FAULT_RO = 1
WARM_EXPECTED_PF_ERROR equ 7 ; present, write, CPL3
%else
WARM_EXPECTED_PF_ERROR equ 6 ; nonpresent, write, CPL3
%endif

times 0x170-($-$$) db 0x90
inner:
 push ebp
 mov ebp,esp
 push ecx
 push edi
 mov edi,[ebp+8]
 or ecx,-1
 xor eax,eax
 cld
 repne scasb
 or eax,-2
 sub eax,ecx
 pop edi
 pop ecx
 leave
 ret 4

; The lower-page push remains 68 f1 00 00 00 at offset 3ad.
times 0x3a9-($-$$) db 0x90
wrapper:
 push ebx
 push esi
 push edi
 push ebp
stack_growth_push:
 push dword 0xf1
 push dword seh_handler+0x10000
 push dword [fs:0]
 mov [fs:0],esp
 mov eax,esp
 push dword [eax+0x20]
 call inner
 pop dword [fs:0]
 add esp,8
 pop ebp
 pop edi
 pop esi
 pop ebx
 ret 4
%if WARM_STACK_FAULT_INDEX = 0
WARM_EXPECTED_PF_EIP equ wrapper
%elif WARM_STACK_FAULT_INDEX = 1
WARM_EXPECTED_PF_EIP equ wrapper+1
%elif WARM_STACK_FAULT_INDEX = 2
WARM_EXPECTED_PF_EIP equ wrapper+2
%elif WARM_STACK_FAULT_INDEX = 3
WARM_EXPECTED_PF_EIP equ wrapper+3
%else
WARM_EXPECTED_PF_EIP equ stack_growth_push
%endif

times 0x400-($-$$) db 0x90
start:
 cli
 cld
 mov esp,KSTACK
 lgdt [cs:gdt_desc]
 lidt [cs:idt_desc]
 mov ax,D0
 mov ds,ax
 mov es,ax
 mov ss,ax
 mov fs,ax
 mov gs,ax
 mov ax,LDT_SEL
 lldt ax
 mov ax,TSS_SEL
 ltr ax
 ; The JSON builder creates supervisor PDEs; both used PDEs must allow CPL3.
 or dword [0],4
 or dword [4],4
 mov eax,cr3
 mov cr3,eax
 mov esi,path+0x10000
 mov edi,STRING
 mov ecx,24
 rep movsb
 mov dword [PF_COUNT],0
 mov dword [RUN],0
 mov dword [STACK_ALIAS+0xffc],SENTINEL
 ; JSON gives this PTE present for warm pass 0.
 mov dword [STACK_GROW_PTE],STACK_GROW_PTE_VALUE
 invlpg [0x005feffc]
 jmp enter_user

; Ring 0 deliberately recreates the complete CPL3 entry state for each pass.
; It never reloads CR3 or touches the wrapper's code mapping between passes.
enter_user:
 cli
 mov esp,KSTACK
 mov ax,D0
 mov ds,ax
 mov es,ax
 mov fs,ax
 mov gs,ax
 mov dword [0x60000],0x11223344
 mov ax,D3
 mov ds,ax
 mov es,ax
 mov ax,F3
 mov fs,ax
 mov eax,0x0badc0de
 mov ebx,0x13579bdf
 mov esi,0x2468ace0
 mov edi,0x005ff050
 mov ebp,0x005ff020
 mov ecx,0x55aa7777
 mov edx,0x76543210
 std
 push dword D3
 push dword USTACK
 pushfd
 push dword C3
 push dword user_entry
 iretd

user_entry:
 push dword STRING
 call wrapper
 cmp eax,23
 jne user_fail_length
 cmp ebx,0x13579bdf
 jne user_fail_ebx
 cmp esi,0x2468ace0
 jne user_fail_esi
 cmp edi,0x005ff050
 jne user_fail_edi
 cmp ebp,0x005ff020
 jne user_fail_ebp
 cmp ecx,0x55aa7777
 jne user_fail_ecx
 cmp edx,0x76543210
 jne user_fail_edx
 cmp esp,USTACK
 jne user_fail_esp
 mov eax,[RUN]
 cmp dword [PF_COUNT],eax
 jne user_fail_pf_count
 cmp dword [fs:0],0x11223344
 jne user_fail_fs
 pushfd
 pop eax
 test eax,0x3400
 jnz user_fail_flags
 mov eax,23
 int 0x80
 jmp $

%macro SAVE_USER 0
 pushad
 push ds
 push es
 push fs
 push gs
 mov ax,D0
 mov ds,ax
 mov es,ax
%endmacro
%macro RESTORE_USER 0
 pop gs
 pop fs
 pop es
 pop ds
 popad
%endmacro

pf_handler:
 SAVE_USER
 cmp dword [RUN],1
 jne fail_pf_run
 cmp dword [esp+48],WARM_EXPECTED_PF_ERROR
 jne fail_pf_error
 cmp dword [esp+52],WARM_EXPECTED_PF_EIP
 jne fail_pf_eip
 cmp dword [esp+56],C3
 jne fail_pf_cs
 cmp dword [esp+64],0x005ff000
 jne fail_pf_esp
 cmp dword [esp+68],D3
 jne fail_pf_ss
 mov eax,cr2
 cmp eax,0x005feffc
 jne fail_pf_cr2
 cmp dword [esp+16],0x005ff050
 jne fail_pf_gpr
 cmp dword [esp+20],0x2468ace0
 jne fail_pf_gpr
 cmp dword [esp+24],0x005ff020
 jne fail_pf_gpr
 cmp dword [esp+32],0x13579bdf
 jne fail_pf_gpr
 cmp dword [esp+36],0x76543210
 jne fail_pf_gpr
 cmp dword [esp+40],0x55aa7777
 jne fail_pf_gpr
 cmp dword [esp+44],0x0badc0de
 jne fail_pf_gpr
 cmp dword [STACK_ALIAS+0xffc],SENTINEL
 jne fail_pf_sentinel
 inc dword [PF_COUNT]
 cmp dword [PF_COUNT],1
 jne fail_pf_count
 mov dword [STACK_GROW_PTE],STACK_GROW_PTE_VALUE
 invlpg [0x005feffc]
 RESTORE_USER
 add esp,4
 iretd

report_handler:
 SAVE_USER
 cmp dword [esp+52],C3
 jne fail_report_frame
 cmp dword [esp+64],D3
 jne fail_report_frame
 test dword [esp+56],0x3000
 jnz fail_report_frame
 cmp dword [esp+44],23
 jne kernel_fail
 mov eax,[RUN]
 cmp dword [PF_COUNT],eax
 jne fail_pf_count
 test eax,eax
 jnz report_pass
 mov dword [RUN],1
 mov dword [STACK_ALIAS+0xffc],SENTINEL
 ; Keep all code translations/cache state warm; invalidate only the data page.
%if WARM_STACK_FAULT_RO = 1
 mov dword [STACK_GROW_PTE],STACK_GROW_PTE_RO_VALUE
%else
 mov dword [STACK_GROW_PTE],0
%endif
 invlpg [0x005feffc]
 jmp enter_user
report_pass:
 mov eax,0x53570000
 mov dx,0xe4
 out dx,eax
 mov dx,0xe0
 mov al,1
 out dx,al
 hlt
 jmp $

%macro UFAIL 2
%1:
 mov eax,0x4c330000+%2
 int 0x80
 jmp $
%endmacro
UFAIL user_fail_length,1
UFAIL user_fail_ebx,2
UFAIL user_fail_esi,3
UFAIL user_fail_edi,4
UFAIL user_fail_ebp,5
UFAIL user_fail_ecx,6
UFAIL user_fail_edx,7
UFAIL user_fail_esp,8
UFAIL user_fail_fs,9
UFAIL user_fail_flags,10
UFAIL user_fail_pf_count,11
UFAIL seh_handler,12
%macro KFAIL 2
%1:
 mov eax,0x4c330000+%2
 jmp kernel_fail
%endmacro
KFAIL fail_pf_run,0x20
KFAIL fail_pf_error,0x21
KFAIL fail_pf_eip,0x22
KFAIL fail_pf_cs,0x23
KFAIL fail_pf_esp,0x24
KFAIL fail_pf_ss,0x25
KFAIL fail_pf_cr2,0x26
KFAIL fail_pf_gpr,0x27
KFAIL fail_pf_sentinel,0x28
KFAIL fail_pf_count,0x29
KFAIL fail_report_frame,0x2a
KFAIL ud_handler,0x100
KFAIL gp_handler,0x101
kernel_fail:
 mov dx,0xe4
 out dx,eax
 mov dx,0xe0
 mov al,0xff
 out dx,al
 hlt
 jmp $

path: db 'C:\WINDOWS\EXPLORER.EXE',0
times 0x2000-($-$$) db 0
align 8
gdt:
 dq 0
 dq 0x00cf9b010000ffff
 dq 0x00cf93000000ffff
 dw 0x67,tss
 db 1,0x89,0,0
 dw ldt_end-ldt-1,ldt
 db 1,0x82,0,0
gdt_end:
gdt_desc:
 dw gdt_end-gdt-1
 dd gdt+0x10000
align 8
idt:
 times 6 dq 0
 dw ud_handler,C0
 db 0,0x8e
 dw 0
 times 6 dq 0
 dw gp_handler,C0
 db 0,0x8e
 dw 0
 dw pf_handler,C0
 db 0,0x8e
 dw 0
 times 113 dq 0
 dw report_handler,C0
 db 0,0xee
 dw 0
idt_end:
idt_desc:
 dw idt_end-idt-1
 dd idt+0x10000
align 4
tss:
 dd 0,KSTACK,D0
 times 90 db 0
 dw 104
align 8
ldt:
 times 44 dq 0
 dq 0x00cffb010000ffff
 dq 0x00cff3000000ffff
 times (814-46) dq 0
 dq 0x00cff3060000ffff
ldt_end:
