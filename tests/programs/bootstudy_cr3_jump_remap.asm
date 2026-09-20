; The first instruction after MOV CR3 is the same explicit near jump in both
; mappings. Its target must be fetched from the new context after redirection.
BITS 32
ORG 0

NEW_DIR equ 0x2000
NEW_TABLE equ 0x3000
REPLACEMENT equ 0x60000
CODE_BASE equ 0x10000
NEXT_LINEAR equ CODE_BASE + 0x2000
FLAGS equ 0x63

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
    jmp switch_cr3

replacement_code:
    jmp near replacement_target
    times 0x100 - ($-replacement_code) db 0x90
replacement_target:
    mov eax, 0xc3000000
    out 0xe4, eax
    mov al, 1
    out 0xe0, al
    hlt
    jmp $
replacement_end:

    times 0x1ff8 - ($-$$) db 0x90
switch_cr3:
    mov eax, NEW_DIR
    mov cr3, eax
old_context_code:
    jmp near old_context_target
    times 0x2100 - ($-$$) db 0x90
old_context_target:
    mov eax, 0xc3000001
    out 0xe4, eax
    mov al, 0xff
    out 0xe0, al
    hlt
    jmp $
