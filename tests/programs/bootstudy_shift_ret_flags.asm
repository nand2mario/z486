bits 32

; Every operation is immediately followed by RET. Test both zero and nonzero
; stack low bytes, all operand widths, high bytes, rotates, and zero counts.
; AF is checked only for count zero; OF only for count zero or one.
%macro CHECK 9
    mov esp, ebp
    mov eax, %4
    mov ecx, 3
    mov edx, %9
    push dword %6
    popfd
    call %%operation
    pushfd
    pop ebx
    and ebx, %7
    cmp ebx, %8
    jne fail
    cmp eax, %5
    jne fail
    jmp %%done
%%operation:
    %1 %2, %3
    ret
%%done:
%endmacro

    mov ebp, 0x7F000
    mov edi, 16
again:
    CHECK shr, ah, 7, 0xC141FF55, 0xC1410155, 2, 0xC5, 0x01, 1
    CHECK shr, ah, 7, 0xC1410155, 0xC1410055, 2, 0xC5, 0x44, 2
    CHECK shl, al, 1, 0xAABBCC81, 0xAABBCC02, 2, 0x8C5, 0x801, 3
    CHECK shr, al, 1, 0xAABBCC81, 0xAABBCC40, 2, 0x8C5, 0x801, 4
    CHECK sar, al, 1, 0xAABBCC81, 0xAABBCCC0, 2, 0x8C5, 0x085, 5
    CHECK shl, ax, 1, 0xAABB8001, 0xAABB0002, 2, 0x8C5, 0x801, 6
    CHECK shr, ax, 1, 0xAABB8001, 0xAABB4000, 2, 0x8C5, 0x805, 7
    CHECK sar, ax, 1, 0xAABB8001, 0xAABBC000, 2, 0x8C5, 0x085, 8
    CHECK shl, eax, 1, 0x80000001, 0x00000002, 2, 0x8C5, 0x801, 9
    CHECK shr, eax, 1, 0x80000001, 0x40000000, 2, 0x8C5, 0x805, 10
    CHECK sar, eax, 1, 0x80000001, 0xC0000000, 2, 0x8C5, 0x085, 11
    CHECK shl, eax, cl, 0x10000001, 0x80000008, 2, 0xC5, 0x80, 12
    CHECK shr, eax, cl, 0x80000001, 0x10000000, 2, 0xC5, 0x04, 13
    CHECK sar, eax, cl, 0x80000001, 0xF0000000, 2, 0xC5, 0x84, 14
    CHECK rol, al, 1, 0xAABBCC81, 0xAABBCC03, 0xC6, 0x8C5, 0x8C5, 15
    CHECK ror, al, 1, 0xAABBCC81, 0xAABBCCC0, 0xC6, 0x8C5, 0x0C5, 16
    CHECK rcl, al, 1, 0xAABBCC80, 0xAABBCC01, 0xC7, 0x8C5, 0x8C5, 17
    CHECK rcr, al, 1, 0xAABBCC01, 0xAABBCC80, 0xC7, 0x8C5, 0x8C5, 18
    CHECK rol, ax, 1, 0xAABB8001, 0xAABB0003, 0xC6, 0x8C5, 0x8C5, 19
    CHECK ror, eax, 1, 0x80000001, 0xC0000000, 0xC6, 0x8C5, 0x0C5, 20
    CHECK rcl, eax, 1, 0x80000000, 0x00000001, 0xC7, 0x8C5, 0x8C5, 21
    CHECK rcr, ax, 1, 0xAABB0001, 0xAABB8000, 0xC7, 0x8C5, 0x8C5, 22
    CHECK shl, ah, 0, 0xC1418055, 0xC1418055, 0x8D7, 0x8D5, 0x8D5, 23
    CHECK shr, ax, 0, 0xAABB8001, 0xAABB8001, 0x8D7, 0x8D5, 0x8D5, 24
    CHECK sar, eax, 0, 0x80000001, 0x80000001, 0x8D7, 0x8D5, 0x8D5, 25
    CHECK rol, al, 0, 0xAABBCC81, 0xAABBCC81, 0x8D7, 0x8D5, 0x8D5, 26
    CHECK rcl, eax, 0, 0x80000000, 0x80000000, 0x8D7, 0x8D5, 0x8D5, 27
    sub ebp, 0x44
    dec edi
    jnz again
    mov eax, 0x53460000
    out 0xE4, eax
    mov al, 1
    out 0xE0, al
    hlt
fail:
    mov eax, edx
    or eax, 0x53460000
    out 0xE4, eax
    mov al, 0xFF
    out 0xE0, al
    hlt
