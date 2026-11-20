; Linux 0.11 fault0: a timer IRQ preempts #PF before its first instruction.
; The testbench asserts INTR on the real #PF request and verifies that the
; IRQ is accepted before pf_handler is issued. #PF uses a trap gate (IF=1);
; IRQ uses an interrupt gate. Its nested ring-0 frame has THREE DWORDs,
; while the original ring-3 #PF frame has six, including error code 4.
; No force or fake CPU frame is used. After IRQ, repair #PF and retry fetch.
BITS 32
ORG 0

STATUS_PORT equ 0xe0
DATA_PORT equ 0xe4
CODE_BASE equ 0x10000
KSTACK_TOP equ 0x38000
USER_ESP equ 0x31000
SEL_CODE0 equ 8
SEL_DATA0 equ 0x10
SEL_CODE3 equ 0x1b
SEL_DATA3 equ 0x23
SEL_TSS equ 0x28
IRQ_VECTOR equ 0x20
FAULT_LINEAR equ CODE_BASE + 0x2000
FAULT_PTE equ 0x1000 + (FAULT_LINEAR >> 12) * 4

gdt:
    dq 0
    dq 0x00cf9b010000ffff
    dq 0x00cf93000000ffff
    dq 0x00cffb010000ffff
    dq 0x00cff3000000ffff
    dw 0x67, tss
    db 1, 0x89, 0, 0
gdt_end:
gdt_desc:
    dw gdt_end - gdt - 1
    dd CODE_BASE + gdt
align 8
idt:
    times 13 dq 0
    dw gp_handler, SEL_CODE0
    db 0, 0x8e
    dw 0
    dw pf_handler, SEL_CODE0
    db 0, 0x8f                       ; #PF trap gate preserves IF
    dw 0
    times IRQ_VECTOR - 15 dq 0
    dw irq_handler, SEL_CODE0
    db 0, 0x8e                       ; IRQ interrupt gate clears IF
    dw 0
idt_end:
idt_desc:
    dw idt_end - idt - 1
    dd CODE_BASE + idt
align 4
tss:
    dd 0, KSTACK_TOP, SEL_DATA0
    times 23 dd 0
irq_count: dd 0
pf_count: dd 0

times 0x280 - ($ - $$) db 0x90
gp_handler:
    mov eax, 0x0d000001
    jmp fail

; Coupled to +pf_irq_entry=768 in the JSON fixture configuration.
times 0x300 - ($ - $$) db 0x90
pf_handler:
    pushad
    lea ebp, [esp + 32]
    mov eax, 0x0e000001
    cmp ebp, KSTACK_TOP - 24
    jne fail
    cmp dword [CODE_BASE + irq_count], 1
    jne fail
    cmp dword [ebp], 4               ; non-present user read/fetch
    jne fail
    cmp dword [ebp + 4], fault_target
    jne fail
    cmp dword [ebp + 8], SEL_CODE3
    jne fail
    mov eax, cr2
    cmp eax, FAULT_LINEAR
    jne fail_pf_addr
    mov eax, [ebp + 12]
    and eax, 0x3200
    cmp eax, 0x3200                  ; original IF/IOPL preserved
    jne fail_pf_addr
    cmp dword [ebp + 16], USER_ESP
    jne fail_pf_addr
    cmp dword [ebp + 20], SEL_DATA3
    jne fail_pf_addr
    inc dword [CODE_BASE + pf_count]
    or dword [FAULT_PTE], 1
    invlpg [FAULT_LINEAR]
    popad
    add esp, 4                      ; discard ONLY the #PF error word
    iretd
fail_pf_addr:
    mov eax, 0x0e000002
    jmp fail

times 0x400 - ($ - $$) db 0x90
irq_handler:
    ; Check before any stack modification, so an extra error word is explicit.
    cmp esp, KSTACK_TOP - 36
    jne fail_irq_stack
    pushad
    lea ebp, [esp + 32]
    mov eax, 0x20000002
    cmp dword [ebp], pf_handler
    jne fail
    cmp dword [ebp + 4], SEL_CODE0
    jne fail
    test dword [ebp + 8], 0x200      ; #PF did not clear IF
    jz fail
    pushfd
    pop eax
    test eax, 0x200                  ; IRQ DID clear IF
    jnz fail_irq_flags
    mov eax, 0x20000003
    cmp dword [ebp + 12], 4          ; original #PF frame directly follows
    jne fail
    cmp dword [ebp + 16], fault_target
    jne fail
    cmp dword [ebp + 20], SEL_CODE3
    jne fail
    cmp dword [ebp + 28], USER_ESP
    jne fail
    cmp dword [ebp + 32], SEL_DATA3
    jne fail
    cmp dword [CODE_BASE + pf_count], 0
    jne fail
    inc dword [CODE_BASE + irq_count]
    cmp dword [CODE_BASE + irq_count], 1
    jne fail
    popad
    iretd
fail_irq_flags:
    mov eax, 0x20000004
    jmp fail
fail_irq_stack:
    ; The fixture logs actual ESP and four top DWORDs before the failure.
    mov eax, esp
    out DATA_PORT, eax
    mov eax, [esp]
    out DATA_PORT, eax
    mov eax, [esp + 4]
    out DATA_PORT, eax
    mov eax, [esp + 8]
    out DATA_PORT, eax
    mov eax, [esp + 12]
    out DATA_PORT, eax
    mov eax, 0x20000001
    jmp fail

times 0x800 - ($ - $$) db 0x90
start:
    cli
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov ax, SEL_DATA0
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov esp, KSTACK_TOP
    mov ax, SEL_TSS
    ltr ax
    and dword [FAULT_PTE], 0xfffffffe
    mov eax, cr3
    mov cr3, eax
    push dword SEL_DATA3
    push dword USER_ESP
    push dword 0x3202                ; IF=1; IOPL=3 permits result-port output
    push dword SEL_CODE3
    push dword user
    iretd

times 0x1000 - ($ - $$) db 0x90
user:
    mov ax, SEL_DATA3
    mov ds, ax
    mov es, ax
    mov eax, 0xabcdef01
    mov ecx, 0x11223344
    mov ebx, 0x55667788
    jmp fault_target

fail:
    out DATA_PORT, eax
    mov al, 0xff
    out STATUS_PORT, al
    jmp $

times 0x2000 - ($ - $$) db 0x90
fault_target:
    cmp eax, 0xabcdef01
    jne fail_user
    cmp ecx, 0x11223344
    jne fail_user
    cmp ebx, 0x55667788
    jne fail_user
    cmp esp, USER_ESP
    jne fail_user
    cmp dword [CODE_BASE + irq_count], 1
    jne fail_user
    cmp dword [CODE_BASE + pf_count], 1
    jne fail_user
    mov al, 1
    out STATUS_PORT, al
    jmp $
fail_user:
    mov eax, 0x03000001
    jmp fail
