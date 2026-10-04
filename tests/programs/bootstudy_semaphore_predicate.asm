; Exact captured VMM semaphore predicate, including its flag-preserving RET.
; Exercise partial-register load, full EAX decrement, high-byte SHR and aliases.
BITS 32
ORG 0
COUNT equ 0x50000
ALIAS equ 0x60000

%macro CHECK 3
    mov word [ebx], %1
    mov eax, %2
    call predicate
%if %3
    jnz fail_ready
%else
    jz fail_blocked
%endif
%endmacro

start:
    cli
    cld
    mov esp, 0x7f000
    mov esi, COUNT
    mov ebx, ALIAS
    mov dword [esi], 0x45530000
    cmp dword [ebx], 0x45530000
    jne fail_alias
    mov edi, 32
again:
    CHECK 0,      0xc1413cb0, 0
    inc word [ebx]
    mov eax, 0xc1413cb0
    call predicate
    jnz fail_increment
    CHECK 2,      0xc1413cb0, 1
    CHECK 0x0101, 0xffff0000, 1
    CHECK 0x7fff, 0x80000000, 1
    CHECK 0x8000, 0x12345678, 1
    CHECK 0x8001, 0xc1413cb0, 0
    CHECK 0xffff, 0x00000000, 0
    CHECK 1,      0x0000ffff, 1
    dec word [ebx]
    mov eax, 0xc1413cb0
    call predicate
    jz fail_decrement
    cmp dword [esi], 0x45530000
    jne fail_alias
    dec edi
    jnz again
    mov eax, 0x53450000
    out 0xe4, eax
    mov al, 1
    out 0xe0, al
    hlt
    jmp $

predicate:
    mov ax, [esi]
    dec eax
    shr ah, 7
    ret

fail_ready:     mov edx, 0x53450001
                jmp fail
fail_blocked:   mov edx, 0x53450002
                jmp fail
fail_alias:     mov edx, 0x53450003
                jmp fail
fail_increment: mov edx, 0x53450004
                jmp fail
fail_decrement: mov edx, 0x53450005
fail:
    out 0xe4, eax
    mov eax, edx
    out 0xe4, eax
    mov al, 0xff
    out 0xe0, al
    hlt
    jmp $
