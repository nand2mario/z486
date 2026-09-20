; trapgate_pf_intr_errcode - regression fixture.
; The #PF handler is a TRAP gate (IF stays 1) and an INTR is already pending, so
; the interrupt is recognised at handler entry, before the handler's first
; instruction issues.  The sequencer predicates (error_code_flag, ...) were
; cleared only at i_issue, so the interrupt frame got a stale error code pushed
; and the ISR's iretd derailed.  Fixed by also clearing them at interrupt_entry
; (z486.sv).  Handler = exact Win98 #PF entry stub bytes
; (pushad; cmp dword cs:[2D9],0; jne rel32) at the Win98 alignment.

BITS 32
ORG 0
STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
SIGNAL_PORT equ 0xE8
SIGNAL_CYCLES_PORT equ 0xEC
SIGNAL_VECTOR_PORT equ 0xF4
absent_data equ 0x40000          ; DS-relative -> linear 0x50000, page not mapped
STUBADDR equ 0x408 + 0
TSS_OFF equ 0xF00

align 8
gdt:
    dq 0
    dq 0x00cf9b010000ffff
    dq 0x00cf93010000ffff
    dq 0x00cffb010000ffff          ; 0x18 ring3 code
    dq 0x00cff3010000ffff          ; 0x20 ring3 data
    dw 0x0067                      ; 0x28 TSS
    dw (0x10000 + TSS_OFF) & 0xffff
    db ((0x10000 + TSS_OFF) >> 16) & 0xff
    db 0x89
    db 0
    db ((0x10000 + TSS_OFF) >> 24) & 0xff
gdt_end:
gdt_desc:
    dw gdt_end - gdt - 1
    dd 0x10000 + gdt
scratch: dd 0
pf_count: dd 0
intr_count: dd 0
isr:
    inc dword [ds:intr_count]
    iretd
ud_handler:
    mov eax, 6
    out DATA_PORT, eax
    jmp fail
gp_handler:
    mov eax, 13
    out DATA_PORT, eax
    jmp fail
fail_cnt:
    mov eax, 9
    out DATA_PORT, eax
fail:
    mov al, 0xff
    out STATUS_PORT, al
    hlt
align 8
idt:
    times 6 dq 0
    dw ud_handler
    dw 0x0008
    db 0
    db 0x8e
    dw 0
    times 6 dq 0
    dw gp_handler
    dw 0x0008
    db 0
    db 0x8e
    dw 0
    dw stub
    dw 0x0008
    db 0
    db 143
    dw 0
    times 17 dq 0
    dw isr
    dw 0x0008
    db 0
    db 0x8e
    dw 0
idt_end:
idt_desc:
    dw idt_end - idt - 1
    dd 0x10000 + idt

; fast path lives at (flag address + 4), exactly like Win98 (2D9 / 2DD)
times 0x2D9 - ($ - $$) db 0x90
flag: dd 1
fast:
    inc dword [ds:pf_count]
    add dword [esp+36], 5
    popad
    add esp, 4
    iretd

times STUBADDR - ($ - $$) db 0x90
stub:
    pushad
    cmp dword [cs:flag], 0
    jne near fast
    mov esi, 0x38
    cld
    jmp near fast

times 0x800 - ($ - $$) db 0x90
start:
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp, 0xF000
    mov al, 0x20
    out SIGNAL_VECTOR_PORT, al
    mov ax, 30
    out SIGNAL_CYCLES_PORT, ax


    mov esi, absent_data
    mov edi, 1
    sti
    mov al, 1
    out SIGNAL_PORT, al
mainloop:


    mov eax, [absent_data]
after_fault:

    dec edi
    jnz mainloop
    cmp dword [ds:pf_count], 1
    jne fail_cnt

    mov al, 1
    out STATUS_PORT, al
    hlt
resume_call_abs:
    add esp, 4
    jmp near after_fault
resume_jmp_abs:
    jmp near after_fault
times TSS_OFF - ($ - $$) db 0
tss:
    dd 0
    dd 0xF000        ; esp0
    dd 0x10          ; ss0
    times 22 dd 0
    dw 0
    dw 0x68          ; iomap base beyond limit

times 0x10000 - ($ - $$) db 0x90
absent_code:
    mov al, 0xff
    out STATUS_PORT, al
    hlt
