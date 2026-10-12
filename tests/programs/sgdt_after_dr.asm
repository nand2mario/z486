; sgdt_after_dr.asm - SGDT/SIDT store the table bases whatever COUNTR holds
;
; MOV DRn selects the debug register with COUNTR = 0x70+n (SELECT_DR_TR) and
; reads it with "-1 -> IRF" + LBAS. SGDT (340) and SIDT (33D) also read with
; LBAS, but name DESGDT/DESIDT in the destination and leave COUNTR as an
; earlier instruction set it (JECXZ loads it with ECX). LBAS took the DR path
; on COUNTR alone, so with COUNTR bits 6:3 = 1110 the stored base was DRn or 0
; (Windows 3.1 VMM: jecxz with ECX = F000E6F5, then SGDT stored base 0 and
; Windows hung at its logo).
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 fail code.

BITS 32
ORG 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

section .text
start:
    mov esp, 0x0F00

    ; Non-zero table bases (the testbench's are 0, which hides a zero LBAS).
    ; No segment load or interrupt follows, so the tables are never read.
    mov word [0x240], 0x0027
    mov dword [0x242], 0x00123450
    lgdt [0x240]
    mov word [0x250], 0x07FF
    mov dword [0x252], 0x00654320
    lidt [0x250]
    sgdt [0x200]
    sidt [0x210]

    ; A DR value that differs from both bases. JECXZ (not taken) leaves
    ; COUNTR = ECX; bits 6:3 = 1110 look like DR/TR index 0x70-0x77: 0xF5 (the
    ; Windows VMM's ECX = F000E6F5) selects TR6-7 (0), 0x70 selects DR0.
    mov eax, 0xDEADBEEF
    mov dr0, eax
    mov ecx, 0xF000E6F5
    jecxz .ecx_zero
    sgdt [0x220]
    mov ecx, 0x70
    jecxz .ecx_zero
    sidt [0x230]
    jmp .check
.ecx_zero:
    jmp .fail_6
.check:

    cmp dword [0x202], 0x00123450
    jne .fail_6
    cmp dword [0x212], 0x00654320
    jne .fail_6
    mov eax, [0x202]
    cmp eax, [0x222]
    jne .fail_1
    mov ax, [0x200]
    cmp ax, [0x220]
    jne .fail_2
    mov eax, [0x212]
    cmp eax, [0x232]
    jne .fail_3
    mov ax, [0x210]
    cmp ax, [0x230]
    jne .fail_4

    ; MOV r,DRn itself still reads the DR.
    mov ecx, dr0
    cmp ecx, 0xDEADBEEF
    jne .fail_5

    mov al, STATUS_PASS
    out STATUS_PORT, al
    hlt

.fail_1:
    mov eax, 1
    jmp .fail
.fail_2:
    mov eax, 2
    jmp .fail
.fail_3:
    mov eax, 3
    jmp .fail
.fail_4:
    mov eax, 4
    jmp .fail
.fail_5:
    mov eax, 5
    jmp .fail
.fail_6:
    mov eax, 6
.fail:
    out DATA_PORT, eax
    mov al, STATUS_FAIL
    out STATUS_PORT, al
    hlt
