; ac_int_keep.asm - interrupt delivery keeps the 486 AC flag
;
; The 386 microcode clears the whole high word of EFLAGS on interrupt entry
; (8D5 trap/interrupt gate, 632 more privilege), where a 386 has only VM and
; RF. The i486 PRM's INT pseudocode clears VM, TF, NT and RF, not AC: the
; handler and the pushed image keep it. A DOS extender's 386/486 test
; (POPFD toggling AC) then saw a 386 after the next interrupt (Fistfight).
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 fail code.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF
AC_FLAG     equ 0x40000
SEL_CODE0   equ 0x08
SEL_DATA0   equ 0x10

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

    ; Set AC (CR0.AM is clear, so no alignment checks) and read it back.
    pushfd
    pop eax
    or eax, AC_FLAG
    push eax
    popfd
    pushfd
    pop eax
    test eax, AC_FLAG
    jz fail_1

    int 0x30                    ; 32-bit interrupt gate, same privilege

    ; IRETD restored the pushed image: AC must still be set.
    pushfd
    pop eax
    test eax, AC_FLAG
    jz fail_4

    mov al, STATUS_PASS
    out STATUS_PORT, al
    hlt

int30_handler:
    pushfd                      ; handler EFLAGS keep AC
    pop eax
    test eax, AC_FLAG
    jz fail_2
    test dword [esp + 8], AC_FLAG   ; pushed EFLAGS image keeps AC
    jz fail_3
    iretd

fail_1:
    mov eax, 1
    jmp fail
fail_2:
    mov eax, 2
    jmp fail
fail_3:
    mov eax, 3
    jmp fail
fail_4:
    mov eax, 4
fail:
    out DATA_PORT, eax
    mov al, STATUS_FAIL
    out STATUS_PORT, al
    hlt

BITS 16
align 8
gdt:
    dq 0
    dq 0x00CF9B010000FFFF       ; 08: ring-0 code, base 10000h
    dq 0x00CF93010000FFFF       ; 10: ring-0 data, base 10000h
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x10000

align 8
idt:
    times 0x30 dq 0
    dw int30_handler, SEL_CODE0
    db 0, 10001110b             ; present, DPL 0, 32-bit interrupt gate
    dw 0                        ; offset 31:16
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x10000
