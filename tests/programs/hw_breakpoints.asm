; hw_breakpoints.asm - debug-register breakpoints (486 PRM chapter 11)
;
; Instruction breakpoints are faults: #DB before the instruction, DR6 Bn,
; the pushed EFLAGS has RF so IRETD resumes it without a second fault. Data
; breakpoints are traps after the accessing instruction (exact: the 486 slows
; down while any Ln/Gn is set); a REP string stops after the iteration that hit
; and resumes. DR6 reports every breakpoint whose condition holds, enabled or
; not, and BS with a data hit under TF.
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 fail code.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF
SEL_CODE0   equ 0x08
SEL_DATA0   equ 0x10
BASE        equ 0x10000             ; code and data segment base
RF_FLAG     equ 0x10000
TF_FLAG     equ 0x100

start:
    cli
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov eax, cr0
    or eax, 1
    mov cr0, eax
    db 0x66, 0xEA
    dd pm32_entry
    dw SEL_CODE0

BITS 32
pm32_entry:
    mov ax, SEL_DATA0
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov esp, 0x9000

    ; 1. Instruction breakpoint on bp_exec (DR0, R/W 00, LEN 00, L0).
    mov eax, BASE + bp_exec
    mov dr0, eax
    mov eax, 0x00000001
    mov dr7, eax
    xor ebx, ebx
    nop
bp_exec:
    inc ebx                         ; faults once, then runs with RF
    mov eax, 1
    cmp dword [log_count], 1
    jne fail
    cmp dword [log_eip], bp_exec    ; a fault: EIP of the breakpointed instruction
    jne fail
    test dword [log_dr6], 1         ; B0
    jz fail
    test dword [log_flags], RF_FLAG ; pushed EFLAGS has RF
    jz fail
    cmp ebx, 1                      ; the instruction ran once
    jne fail

    ; 2. Dword write breakpoint on wvar (DR1, R/W 01, LEN 11, L1); DR3 on the
    ; same dword for reads or writes but disabled: DR6 reports it too.
    call clear_log
    mov eax, BASE + wvar
    mov dr1, eax
    mov dr3, eax
    mov eax, 0xF0D00004             ; LEN3 11 R/W3 11, LEN1 11 R/W1 01, L1
    mov dr7, eax
    mov eax, [wvar]                 ; a read: no trap
    mov dword [wvar], 5
after_write:
    nop
    mov eax, 2
    cmp dword [log_count], 1
    jne fail
    cmp dword [log_eip], after_write ; a trap: EIP after the write
    jne fail
    mov ecx, [log_dr6]
    and ecx, 0xF
    cmp ecx, 0xA                    ; B1 and the disabled B3
    jne fail
    test dword [log_flags], RF_FLAG ; traps do not set RF
    jnz fail

    ; 3. Byte read/write breakpoint on bvar+1 (DR2, R/W 11, LEN 00, L2):
    ; a word access covering it traps, byte accesses beside it do not.
    call clear_log
    mov eax, BASE + bvar + 1
    mov dr2, eax
    mov eax, 0x03000010
    mov dr7, eax
    mov al, [bvar]
    mov al, [bvar + 2]
    mov eax, 3
    cmp dword [log_count], 0
    jne fail
    mov ax, [bvar]
    nop
    mov eax, 0x13
    cmp dword [log_count], 1
    jne fail
    test dword [log_dr6], 4         ; B2
    jz fail

    ; 4. REP STOSB through a write breakpoint on rbuf+8 (DR1, LEN 00): the
    ; trap comes after that iteration with EIP at the REP instruction; the
    ; handler disarms and IRETD resumes the string.
    call clear_log
    mov eax, BASE + rbuf + 8
    mov dr1, eax
    mov eax, 0x00100004
    mov dr7, eax
    mov byte [disarm], 1
    mov edi, rbuf
    mov ecx, 16
    mov al, 0x55
    cld
rep_insn:
    rep stosb
    mov eax, 4
    cmp dword [log_count], 1
    jne fail
    cmp dword [log_eip], rep_insn
    jne fail
    cmp dword [log_ecx], 7          ; bytes 0-8 stored
    jne fail
    cmp ecx, 0                      ; resumed to the end
    jne fail
    cmp byte [rbuf + 15], 0x55
    jne fail
    mov byte [disarm], 0

    ; 5. TF and a data breakpoint in one instruction: one #DB with BS and B1.
    call clear_log
    mov eax, BASE + wvar
    mov dr1, eax
    mov eax, 0x00D00004
    mov dr7, eax
    pushfd
    or dword [esp], TF_FLAG
    popfd
    mov dword [wvar], 6             ; single-step trap and data trap together
    mov eax, 5
    cmp dword [log_count], 1
    jne fail
    mov ecx, [log_dr6]
    and ecx, 0x400F
    cmp ecx, 0x4002                 ; BS and B1
    jne fail

    xor eax, eax
    mov dr7, eax
    mov al, STATUS_PASS
    out STATUS_PORT, al
    hlt

clear_log:
    mov dword [log_count], 0
    mov dword [log_dr6], 0
    ret

; #DB: log DR6, the pushed EIP/EFLAGS and ECX; clear DR6 and TF; optionally
; disarm. An instruction-breakpoint return relies on the pushed RF.
db_handler:
    push eax
    mov eax, dr6
    mov [log_dr6], eax
    mov eax, [esp + 4]
    mov [log_eip], eax
    mov eax, [esp + 12]
    mov [log_flags], eax
    mov [log_ecx], ecx
    inc dword [log_count]
    xor eax, eax
    mov dr6, eax
    cmp byte [disarm], 0
    je .keep
    mov dr7, eax
.keep:
    and dword [esp + 12], ~TF_FLAG
    pop eax
    iretd

fail:
    out DATA_PORT, eax
    mov al, STATUS_FAIL
    out STATUS_PORT, al
    hlt

align 4
log_count: dd 0
log_dr6:   dd 0
log_eip:   dd 0
log_flags: dd 0
log_ecx:   dd 0
wvar:      dd 0
bvar:      dd 0x44332211
disarm:    db 0
align 4
rbuf:      times 16 db 0

BITS 16
align 8
gdt:
    dq 0
    dq 0x00CF9B010000FFFF           ; 08: ring-0 code, base 10000h
    dq 0x00CF93010000FFFF           ; 10: ring-0 data, base 10000h
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + BASE

align 8
idt:
    dq 0
    dw db_handler, SEL_CODE0        ; vector 1: #DB
    db 0, 10001110b                 ; present, DPL 0, 32-bit interrupt gate
    dw 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + BASE
