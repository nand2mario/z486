; Reproduces the classic Cyrix-vs-Intel/AMD 486 CPU-detection probe used by
; pre-CPUID software (CHKCPU, Memtest86+, and believed to be Windows 95/NT4
; Setup's SUNWIN hardware-detection module). Real Intel/AMD 486 silicon
; perturbs EFLAGS as an undocumented side effect of unsigned DIV; real Cyrix
; 486 leaves them unchanged, which these routines use as their discriminator.
; See nand2mario/z486#82. PASS means our core no longer looks like Cyrix.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

start:
    cli
    xor ax, ax
    mov ss, ax
    mov sp, 0x8000

    ; Exact is486Cyrix discriminator from the issue #82 report.
    xor ax, ax
    sahf                         ; Load flags from AH=0 (bit 1 forced to 1).
    mov ax, 5
    mov bx, 2
    div bl                       ; AL=quotient=2, AH=remainder=1 (then overwritten below).
    lahf                         ; AH = current flags byte.
    cmp ah, 2                    ; ==2 means flags looked unchanged (Cyrix-misdetection outcome).
    je .fail                     ; If unchanged, our fix did NOT work -- fail the test.

    mov al, 0x01
    out STATUS_PORT, al
    hlt

.fail:
    mov al, 1
    out DATA_PORT, al
    mov al, 0xff
    out STATUS_PORT, al
    hlt
