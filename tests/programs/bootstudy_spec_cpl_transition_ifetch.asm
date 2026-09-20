; CPL0 speculative target retained across IRETD to CPL3.
; Default deliberately exercises the retained spec_valid path.  The control
; reloads CR3 before IRETD, killing the speculative buffer while keeping the
; identical user target and #PF oracle.
;
; The target is a full 16-byte fetch line.  A broken frontend can reuse that
; line after the CPL0 -> CPL3 transition, execute TARGET_MARKER, and then
; fault at TARGET_NEXT_EIP when the demand fetch crosses into the next line.
; A correct frontend faults before executing any target byte, at TARGET_EIP.
; Both outcomes validate the first #PF frame; only the former reports the
; supervisor-line-reuse failure.
BITS 32
org 0
%ifndef KILL_SPEC
%define KILL_SPEC 0
%endif
%ifndef IRET_DIRECT_TARGET
%define IRET_DIRECT_TARGET 0
%endif
C0 equ 8
D0 equ 0x10
C3 equ 0x167
D3 equ 0x16f
TSS_SEL equ 0x18
LDT_SEL equ 0x20
KSTACK equ 0x58000
USTACK equ 0x000300f0
CODE_BASE equ 0x10000
TARGET_EIP equ 0x4000
TARGET_LINEAR equ (CODE_BASE + TARGET_EIP)
TARGET_NEXT_EIP equ (TARGET_EIP + 0x10)
TARGET_NEXT_LINEAR equ (CODE_BASE + TARGET_NEXT_EIP)
TARGET_PTE equ (0x1000 + ((TARGET_LINEAR >> 12) * 4))
TARGET_MARKER equ 0x4c530063
PF_COUNT equ 0x30000
%if IRET_DIRECT_TARGET
EXPECTED_UNEXECUTED_EAX equ 1
%else
EXPECTED_UNEXECUTED_EAX equ 0
%endif

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
 mov ax,LDT_SEL
 lldt ax
 mov ax,TSS_SEL
 ltr ax
 ; PDE0 and the user entry page must permit CPL3; target remains supervisor.
 or dword [0],4
 mov dword [TARGET_PTE],0x00014063
 mov dword [PF_COUNT],0
 mov eax,cr3
 mov cr3,eax
 ; A not-taken 32-bit Jcc launches a CPL0 speculative fetch of TARGET.
 mov eax,1
 test eax,eax
 jz spec_target
 ; Leave ample straight-line time for the speculative response to become valid.
 times 160 db 0x90
%if KILL_SPEC
 ; Control: CR3 write is prefetch spec_global_kill, so target must page-check.
 mov eax,cr3
 mov cr3,eax
%endif
 ; IRETD changes CPL and flushes the normal queue without a CR3 change by default.
 ; The direct variant makes this very q_flush target supervisor-only, exposing
 ; a same-edge old-CPL fetch if privilege is sampled before IRETD commits.
 push dword D3
 push dword USTACK
 push dword 0x202
 push dword C3
%if IRET_DIRECT_TARGET
 push dword spec_target
%else
 push dword user_entry
%endif
 iretd

%macro SAVE 0
 pushad
 push ds
 push es
 mov ax,D0
 mov ds,ax
 mov es,ax
%endmacro
pf_handler:
 SAVE
 ; All frame offsets include PUSHAD plus the two 32-bit segment pushes in
 ; SAVE.  Saved EAX is +36; #PF error, EIP, CS, EFLAGS, old ESP and old SS
 ; begin at +40.  A second #PF cannot satisfy this one-shot oracle.
 cmp dword [PF_COUNT],0
 jne fail_repeat_pf
 inc dword [PF_COUNT]
 cmp dword [esp+36],TARGET_MARKER
 je .target_reuse
 ; The normal path explicitly clears EAX in user_entry.  The direct IRETD
 ; variant retains the kernel's EAX=1.  In either case, no target marker means
 ; the first CPL3 fetch must fault at the supervisor target itself.
 cmp dword [esp+36],EXPECTED_UNEXECUTED_EAX
 jne fail
 cmp dword [esp+40],5
 jne fail
 cmp dword [esp+44],TARGET_EIP
 jne fail
 cmp dword [esp+48],C3
 jne fail
 cmp dword [esp+56],USTACK
 jne fail
 cmp dword [esp+60],D3
 jne fail
 mov eax,cr2
 cmp eax,TARGET_LINEAR
 jne fail
 mov eax,0x53500000
 jmp report
.target_reuse:
 ; CPL3 has executed bytes from the cached CPL0 supervisor line.  The marker
 ; must be followed by the demand #PF at the next fetch line, not a random
 ; exception or a fault at a different address.
 cmp dword [esp+40],5
 jne fail
 cmp dword [esp+44],TARGET_NEXT_EIP
 jne fail
 cmp dword [esp+48],C3
 jne fail
 cmp dword [esp+56],USTACK
 jne fail
 cmp dword [esp+60],D3
 jne fail
 mov eax,cr2
 cmp eax,TARGET_NEXT_LINEAR
 jne fail
 mov eax,TARGET_MARKER
 jmp report

report_handler:
 SAVE
 ; Target reuse is reported only after its next-line #PF frame is validated.
 ; This DPL3 gate remains solely as a distinct diagnostic for an unexpected
 ; fallthrough of the user branch.
 cmp dword [esp+36],0x4c530063
 je .target_reuse
 cmp dword [esp+36],0x4c530062
 je .fallthrough
 mov eax,0x4c530060
 jmp report
.target_reuse:
 mov eax,0x4c530063
 jmp report
.fallthrough:
 mov eax,0x4c530062
 jmp report
fail_repeat_pf:
 mov eax,0x4c530064
 jmp report
fail:
 mov eax,0x4c530061
report:
 mov dx,0xe4
 out dx,eax
 mov dx,0xe0
 cmp eax,0x53500000
 jne .failure_status
 mov al,1
 out dx,al
 hlt
.failure_status:
 mov al,0xff
 out dx,al
 hlt

; CPL3 starts on user-mapped page 0x11000. Its taken Jcc is the exact target
; line fetched speculatively under CPL0. Correct behaviour faults #PF(5).
times 0x1800-($-$$) db 0x90
user_entry:
 xor eax,eax
 test eax,eax
 jz spec_target
 mov eax,0x4c530062
 int 0x80
 jmp $

; This line is present supervisor-only. Reaching it in CPL3 proves a
; spec_match_now/spec_flush_hit reuse bypassed the paging U/S check.
times TARGET_EIP-($-$$) db 0x90
spec_target:
 mov eax,TARGET_MARKER
 jmp short spec_target_next
 ; Keep the following target exactly one fetch line away.  A retained-line
 ; hit may execute only the marker/jump; the next line must take demand #PF.
 times TARGET_NEXT_EIP-($-$$) db 0x90
spec_target_next:
 nop

times 0x5000-($-$$) db 0
align 8
gdt: dq 0
 dq 0x00cf9b010000ffff
 dq 0x00cf93000000ffff
 dw 0x67,tss
 db 1,0x89,0,0
 dw ldt_end-ldt-1,ldt
 db 1,0x82,0,0
gdt_end:
gdt_desc: dw gdt_end-gdt-1
 dd gdt+CODE_BASE
align 8
idt:
 times 14 dq 0
 dw pf_handler,C0
 db 0,0x8e
 dw 0
 times 113 dq 0
 dw report_handler,C0
 db 0,0xee
 dw 0
idt_end:
idt_desc: dw idt_end-idt-1
 dd idt+CODE_BASE
align 4
tss: dd 0,KSTACK,D0
 times 90 db 0
 dw 104
align 8
ldt:
 times 44 dq 0
 dq 0x00cffb010000ffff ; C3: base CODE_BASE, DPL3
 dq 0x00cff3000000ffff
ldt_end:



