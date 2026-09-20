; bootstudy_ifetch_admission_deadlock_no_branch.asm
;
; Negative control for bootstudy_ifetch_admission_deadlock.asm: identical
; setup, but the not-taken JNZ immediately before the boundary MOV/MOV/CMP
; is replaced with NOPs. An earlier study found this variant does NOT
; trigger the admission-deadlock race (the conditional-branch scheduling is
; part of what causes the retirement-window pulse to be missed) -- this
; should PASS identically before and after the fix, proving the
; fix does not depend on or change ordinary (non-racing) fault delivery.
BITS 32
ORG 0
CODE_LINEAR equ 0xbff96000
POINTER     equ 0xbffc99e0
TARGET      equ 0x81b1200c
STATUS_PORT equ 0xe0
DATA_PORT   equ 0xe4

; See bootstudy_ifetch_admission_deadlock.asm: entry trampoline forces a
; real CS.base-relative refetch before anything else runs.
entry:
    jmp near start

align 8
gdt:
    dq 0
    dq 0xbfcf9bf96000ffff
    dq 0x00cf93000000ffff
gdt_end:
gdt_desc:
    dw gdt_end - gdt - 1
    dd CODE_LINEAR + gdt

pf_handler:
    mov eax, cr2
    and eax, 0xfffff000
    cmp eax, 0xbff97000
    jne fail
    cmp dword [ss:esp], 0
    jne fail
    cmp dword [ss:esp + 4], fault_site
    jne fail
    cmp esp, 0x1b310
    jne fail
    test dword [ss:esp + 12], 0x200
    jz fail
    cmp edx, TARGET
    jne fail
    cmp edi, 1
    jne fail
    mov eax, 0x534f0000
    out DATA_PORT, eax
    mov al, 1
    out STATUS_PORT, al
    hlt
fail:
    mov eax, 0x534f0001
    out DATA_PORT, eax
    mov al, 0xff
    out STATUS_PORT, al
    hlt

align 8
idt:
    times 14 dq 0
    dw pf_handler
    dw 8
    db 0
    db 0x8e
    dw 0
idt_end:
idt_desc:
    dw idt_end - idt - 1
    dd CODE_LINEAR + idt

times 0x200 - ($ - $$) db 0x90
start:
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp, 0x1b320
    mov dword [POINTER], TARGET
    mov dword [TARGET], 0x7562647a
    mov dword [TARGET + 4], 0x00323369
    xor eax, eax
    sti

times 0xfeb - ($ - $$) db 0x90
boundary_chain:
    test eax, eax               ; preserved (2 bytes) so offsets below match
    nop                          ; the branch fixture's 6-byte JNZ near/rel32
    nop
    nop
    nop
    nop
    nop
    mov edx, [POINTER]
    mov edi, 1
fault_site:
    db 0x80, 0x3a
next_page:
    db 0x00
    jmp fail
times 0x1083 - ($ - $$) db 0x90
absent_target:
    jmp fail
