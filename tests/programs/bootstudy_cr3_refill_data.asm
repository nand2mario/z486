; Both instruction pages contain identical code after MOV CR3. An ordinary
; data read through that page must use the new mapping, independently of
; any i486 instruction-prefetch serialization assumptions.
BITS 32
ORG 0

NEW_DIR equ 0x2000
NEW_TABLE equ 0x3000
REPLACEMENT equ 0x60000
CODE_BASE equ 0x10000
NEXT_LINEAR equ CODE_BASE + 0x2000
MARKER_OFFSET equ 0x100
FLAGS equ 0x63

%macro POST_CR3 0
    mov ebx, [NEXT_LINEAR + MARKER_OFFSET]
    cmp ebx, 0x33333333
    jne %%fail
    mov eax, 0xc3000000
    out 0xe4, eax
    mov al, 1
    out 0xe0, al
    hlt
    jmp $
%%fail:
    mov eax, ebx
    out 0xe4, eax
    mov al, 0xff
    out 0xe0, al
    hlt
    jmp $
%endmacro

start:
    cli
    cld
    mov esp, 0x7f000
    mov edi, NEW_DIR
    xor eax, eax
    mov ecx, 1024
    rep stosd
    mov esi, 0x1000
    mov edi, NEW_TABLE
    mov ecx, 1024
    rep movsd
    mov dword [NEW_DIR], NEW_TABLE | FLAGS
    mov dword [NEW_TABLE + (NEXT_LINEAR >> 12) * 4], REPLACEMENT | FLAGS
    mov esi, CODE_BASE + replacement_code
    mov edi, REPLACEMENT
    mov ecx, replacement_end - replacement_code
    rep movsb
    mov dword [REPLACEMENT + MARKER_OFFSET], 0x33333333
    jmp switch_cr3

replacement_code:
    POST_CR3
replacement_end:

    times 0x1ff8 - ($-$$) db 0x90
switch_cr3:
    mov eax, NEW_DIR
    mov cr3, eax
old_context_code:
    POST_CR3
    times 0x2000 + MARKER_OFFSET - ($-$$) db 0x90
    dd 0x11111111
