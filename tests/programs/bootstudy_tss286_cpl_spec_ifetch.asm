; 286 hardware-task-switch CPL context for a retained speculative target.
;
; CPL0 warms supervisor-only TARGET through a not-taken relative Jcc.  A far
; JMP to an available 286 TSS has no CR3 slot, so it changes CPL to 3 without
; the CR3/global-kill path.  The incoming user task reaches USER_ENTRY and
; takes another relative Jcc to the same TARGET.  Correct hardware must take
; #PF(5) before TARGET_MARKER; reuse of the CPL0 line executes the marker and
; faults only at the following fetch line.
BITS 32
org 0

C0            equ 0x08
D0            equ 0x10
TSS_OLD       equ 0x18
TSS_NEW       equ 0x20
C3            equ 0x2b
D3            equ 0x33
CODE_BASE     equ 0x10000
KSTACK        equ 0x8000
USTACK        equ 0x30f0
PF_COUNT      equ 0x3000
TARGET_EIP    equ 0x4000
TARGET_LINEAR equ (CODE_BASE + TARGET_EIP)
TARGET_NEXT_EIP equ (TARGET_EIP + 0x10)
TARGET_NEXT_LINEAR equ (CODE_BASE + TARGET_NEXT_EIP)
TARGET_PTE    equ (0x1000 + ((TARGET_LINEAR >> 12) * 4))
TARGET_MARKER equ 0x54323663

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
    mov ax,TSS_OLD
    ltr ax

    ; PDE0 and the user entry/stack pages are user-accessible.  TARGET is
    ; deliberately changed from the generated RWU image to supervisor-only.
    or dword [0],4
    mov dword [TARGET_PTE],0x00014063
    mov dword [PF_COUNT],0

    ; Warm the full TARGET line as CPL0, then leave enough runout for its
    ; response to become spec_valid before the 286 task switch.
    mov eax,1
    test eax,eax
    jz spec_target
    times 160 db 0x90

    ; A 286 TSS contains no CR3.  Its saved CS:IP below changes the incoming
    ; task to CPL3 at USER_ENTRY while preserving the same address space.
    jmp TSS_NEW:0

    mov eax,0x54323607
    jmp fail

; The incoming 286 task begins in this mapped CPL3 page.  These checks prove
; that the task switch, rather than an ordinary far transfer, supplied CPL3.
times 0x1800-($-$$) db 0x90
user_entry:
    mov ax,cs
    cmp ax,C3
    jne user_fail_cs
    test al,3
    jnz user_cpl_ok
user_fail_cs:
    mov eax,0x54323602
    jmp user_fail
user_cpl_ok:
    mov ax,ss
    cmp ax,D3
    jne user_fail_ss
    xor eax,eax
    test eax,eax
    jz spec_target
    mov eax,0x54323603
    jmp user_fail
user_fail_ss:
    mov eax,0x54323604
user_fail:
    mov dx,0xe4
    out dx,eax
    mov al,0xff
    mov dx,0xe0
    out dx,al
    hlt

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
    ; One and only one #PF is expected.  The saved user EAX is +36; #PF error,
    ; EIP, CS, EFLAGS, old ESP and old SS start at +40 after SAVE.
    cmp dword [PF_COUNT],0
    jne fail_repeat
    inc dword [PF_COUNT]
    cmp dword [esp+36],TARGET_MARKER
    je .target_reuse
    cmp dword [esp+36],0
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
    mov eax,0x54323600
    jmp report
.target_reuse:
    ; The first fault is still checked precisely: seeing this later line only
    ; after TARGET_MARKER isolates stale CPL0 supervisor-line execution.
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

; This supervisor-only line was fetched under CPL0.  The marker/jump are in
; its first 16 bytes; the next line must require a separate CPL3 page walk.
times TARGET_EIP-($-$$) db 0x90
spec_target:
    mov eax,TARGET_MARKER
    jmp short spec_target_next
    times TARGET_NEXT_EIP-($-$$) db 0x90
spec_target_next:
    nop

fail_repeat:
    mov eax,0x54323606
    jmp report
fail:
    mov eax,0x54323601
report:
    mov dx,0xe4
    out dx,eax
    mov dx,0xe0
    cmp eax,0x54323600
    jne .failure
    mov al,1
    out dx,al
    hlt
.failure:
    mov al,0xff
    out dx,al
    hlt

; GDT/TSS data stays in the mapped source image.  Both task descriptors use
; 286 available-TSS type 1 and the standard 44-byte (limit 2Bh) layout.
times 0x5000-($-$$) db 0
align 8
gdt:
    dq 0
    dq 0x00cf9b010000ffff           ; 08: ring-0 code, base CODE_BASE
    dq 0x00cf93000000ffff           ; 10: ring-0 data, base 0
    dw 0x002b,tss_old
    db 0x01,0x81,0,0                ; 18: available 286 TSS
    dw 0x002b,tss_new
    db 0x01,0x81,0,0                ; 20: available 286 TSS
    dq 0x00cffb010000ffff           ; 28: ring-3 code, base CODE_BASE
    dq 0x00cff3000000ffff           ; 30: ring-3 data, base 0
gdt_end:
gdt_desc: dw gdt_end-gdt-1
    dd gdt+CODE_BASE

align 8
idt:
    times 14 dq 0
    dw pf_handler,C0
    db 0,0x8e
    dw 0
idt_end:
idt_desc: dw idt_end-idt-1
    dd idt+CODE_BASE

align 4
tss_old:
    ; The outgoing task is populated by the hardware switch; ring-0 stack
    ; fields are nevertheless valid if an unexpected fault arrives first.
    dw 0,KSTACK,D0,0,0,0,0
    times 15 dw 0

align 4
tss_new:
    dw 0                            ; +00 backlink
    dw KSTACK                       ; +02 SP0
    dw D0                           ; +04 SS0
    dw 0                            ; +06 SP1
    dw 0                            ; +08 SS1
    dw 0                            ; +0A SP2
    dw 0                            ; +0C SS2
    dw user_entry                   ; +0E IP
    dw 0x3002                       ; +10 FLAGS, IOPL=3 for user diagnostics
    dw 0                            ; +12 AX
    dw 0                            ; +14 CX
    dw 0                            ; +16 DX
    dw 0                            ; +18 BX
    dw USTACK                       ; +1A SP
    dw 0                            ; +1C BP
    dw 0                            ; +1E SI
    dw 0                            ; +20 DI
    dw D3                           ; +22 ES
    dw C3                           ; +24 CS
    dw D3                           ; +26 SS
    dw D3                           ; +28 DS
    dw 0                            ; +2A LDT
