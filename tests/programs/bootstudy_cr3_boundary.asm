; Full-CPU CR3/prefetch boundary stress. No forced internal DUT signals.
BITS 32
ORG 0

TARGET equ 0x40000
NEW_TARGET equ 0x50000
OLD_TABLE equ 0x1000
NEW_DIR equ 0x2000
NEW_TABLE equ 0x3000
TARGET_PTE equ OLD_TABLE + (TARGET >> 12) * 4
NEW_TARGET_PTE equ NEW_TABLE + (TARGET >> 12) * 4
FLAGS equ 0x63

start:
    cli
    cld
    mov esp, 0x7f000
    mov dword [TARGET], 0x11111111
    mov dword [NEW_TARGET], 0x33333333

    mov edi, NEW_DIR
    xor eax, eax
    mov ecx, 1024
    rep stosd
    mov esi, OLD_TABLE
    mov edi, NEW_TABLE
    mov ecx, 1024
    rep movsd
    mov dword [NEW_DIR], NEW_TABLE | FLAGS
    mov dword [NEW_TARGET_PTE], NEW_TARGET | FLAGS

    ; A same-value CR3 reload must also discard the cached old translation.
    mov eax, [TARGET]
    cmp eax, 0x11111111
    jne fail
    mov dword [TARGET_PTE], NEW_TARGET | FLAGS
    xor eax, eax
    mov cr3, eax
    mov ebx, [TARGET]
    cmp ebx, 0x33333333
    jne fail
    mov dword [TARGET_PTE], TARGET | FLAGS
    xor eax, eax
    mov cr3, eax
    mov ebx, [TARGET]
    cmp ebx, 0x11111111
    jne fail

    mov ebp, 32
    jmp site0

fail:
    mov eax, ebx
    out 0xe4, eax
    mov al, 0xff
    out 0xe0, al
    hlt
    jmp $

; The MOV CR3 begins the specified number of bytes before a 4-KiB boundary.
; Every switch flushes instruction translations; the next page is cold again.
%macro SWITCH 5
    align 4096, db 0x90
    times 4096 - %2 - 5 db 0x90
%1:
    mov eax, %3
    mov cr3, eax
    mov ebx, [TARGET]
    cmp ebx, %4
    jne fail
    jmp %5
%endmacro

SWITCH site0, 3,  NEW_DIR, 0x33333333, site1
SWITCH site1, 4,  0,       0x11111111, site2
SWITCH site2, 6,  NEW_DIR, 0x33333333, site3
SWITCH site3, 8,  0,       0x11111111, site4
SWITCH site4, 12, NEW_DIR, 0x33333333, site5
SWITCH site5, 16, 0,       0x11111111, site6
SWITCH site6, 24, NEW_DIR, 0x33333333, site7
SWITCH site7, 32, 0,       0x11111111, round_done

round_done:
    dec ebp
    jnz site0
    mov al, 1
    out 0xe0, al
    hlt
