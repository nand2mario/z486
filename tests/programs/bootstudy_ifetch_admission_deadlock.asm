; bootstudy_ifetch_admission_deadlock.asm
;
; Regression fixture for the "late fetch-fault admission deadlock" seen while
; booting Windows 98 (stall at CS:EIP=0167:C0001408, reached via the
; straight-line sequence 0xBFFC7B0B..0xBFFC7B1E). It is derived from
; bootstudy_ifetch_cmp_guest.asm: high linear code/data addresses
; (matching the real capture's address class) plus a not-taken JNZ
; immediately before a MOV/MOV/CMP sequence that straddles the code page
; boundary, with the following page deliberately absent. Without the fix,
; this hard-hangs (TIMEOUT) instead of delivering #PF; with the fix, the
; page fault is reported and the handler runs to completion.
BITS 32
ORG 0
CODE_LINEAR equ 0xbff96000
POINTER     equ 0xbffc99e0
TARGET      equ 0x81b1200c
STATUS_PORT equ 0xe0
DATA_PORT   equ 0xe4

; Entry trampoline: the testbench seeds the very first fetch from the
; PHYSICAL code address (code_phys_base), not CS.base -- an identity page
; table entry for that physical page makes the trampoline bytes themselves
; readable either way, but everything is only guaranteed to use proper
; CS.base-relative (segmented+paged) linear addresses AFTER an actual
; architectural control transfer forces a fresh CS.base+EIP derivation.
; This single unconditional near jump is that transfer.
entry:
    jmp near start

align 8
gdt:
    dq 0
    dq 0xbfcf9bf96000ffff     ; flat-ish 32-bit code, base 0xbff96000
    dq 0x00cf93000000ffff     ; flat 32-bit data, base 0
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
    db 0x8e                    ; present DPL0 386 interrupt gate
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
    test eax, eax
    jnz near absent_target     ; not taken, but the branch itself matters
    mov edx, [POINTER]
    mov edi, 1
fault_site:
    db 0x80, 0x3a               ; CMP byte [EDX], imm8 -- opcode+modrm here,
next_page:
    db 0x00                     ; immediate byte lies on the absent next page
    jmp fail
times 0x1083 - ($ - $$) db 0x90
absent_target:
    jmp fail
