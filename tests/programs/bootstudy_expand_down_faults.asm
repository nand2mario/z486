bits 32
%define COUNT 0x50080
%define CASE  0x50084
%assign case_id 0

%macro DESC 3
    mov dword [0x2000+%1], %2
    mov dword [0x2004+%1], %3
%endmacro

%macro BAD_ACCESS 5
    %assign case_id case_id+1
    mov dword [CASE], case_id
    mov ebp, %1
    mov edi, %%resume
    mov esi, %%fault
    mov eax, 0x13579BDF
%%fault:
    %ifidni %2, load
        mov %5, [%3:%4]
    %elifidni %2, rmw
        inc dword [%3:%4]
    %else
        mov dword [%3:%4], 0
    %endif
    jmp fail
%%resume:
    cmp dword [COUNT], case_id
    jne fail
    cmp eax, 0x13579BDF
    jne fail
%endmacro

    mov esp, 0x7F000
    DESC 0x00, 0, 0
    DESC 0x08, 0x0000FFFF, 0x00CF9A01 ; CS base 10000, 32-bit
    DESC 0x10, 0x0000FFFF, 0x00CF9200 ; flat data/stack
    DESC 0x18, 0x00001000, 0x00009600 ; expand-down, B=0
    DESC 0x20, 0x00001000, 0x00409600 ; expand-down, B=1
    DESC 0x28, 0x00001000, 0x00409E00 ; readable conforming code
    DESC 0x30, 0x00001000, 0x00009600 ; expand-down SS, B=0
    DESC 0x38, 0x00001000, 0x00409400 ; read-only expand-down
    mov word [0x2040], 0x003F
    mov dword [0x2042], 0x2000
    lgdt [0x2040]
    mov dword [0x3068], (0x08 << 16) | (gp_handler-$$)
    mov dword [0x306C], 0x00008E00
    mov dword [0x3060], (0x08 << 16) | (ss_handler-$$)
    mov dword [0x3064], 0x00008E00
    mov word [0x2048], 0x07FF
    mov dword [0x204A], 0x3000
    lidt [0x2048]
    mov dword [COUNT], 0
    mov dword [CASE], 0
    mov dword [0x1000], 0x13579BDF
    mov dword [0xFFFC], 0xAABBCCDD
    mov dword [0x50000], 0x11223344
    mov ecx, [0x1000] ; warm data cache before the invalid FS access
    mov ax, 0x18
    mov fs, ax
    mov eax, [fs:0xFFFC]
    cmp eax, 0xAABBCCDD
    jne fail
    mov eax, [fs:0xFFFC]
    cmp eax, 0xAABBCCDD
    jne fail
    BAD_ACCESS 13, load, fs, 0x1000, eax
    BAD_ACCESS 13, load, fs, 0x0000, ax
    BAD_ACCESS 13, rmw, fs, 0x1000, eax
    BAD_ACCESS 13, load, fs, 0xFFFF, ax
    BAD_ACCESS 13, load, fs, 0xFFFD, eax
    BAD_ACCESS 13, load, fs, 0x50000, eax
    BAD_ACCESS 13, store, fs, 0x1000, eax
    mov ax, 0x20
    mov fs, ax
    mov eax, [fs:0x50000]
    cmp eax, 0x11223344
    jne fail
    inc dword [fs:0x50000]
    inc dword [fs:0x50000]
    cmp dword [0x50000], 0x11223346
    jne fail
    BAD_ACCESS 13, load, fs, 0x1000, eax
    BAD_ACCESS 13, rmw, fs, 0x1000, eax
    mov ax, 0x28
    mov fs, ax
    mov al, [fs:0x1000] ; conforming is not expand-down
    cmp al, 0xDF
    jne fail
    BAD_ACCESS 13, load, fs, 0x1001, eax
    mov esp, 0xF000
    mov ax, 0x30
    mov ss, ax
    BAD_ACCESS 12, load, ss, 0x1000, eax
    BAD_ACCESS 12, rmw, ss, 0x1000, eax
    mov ax, 0x10
    mov ss, ax
    mov esp, 0x7F000
    mov ax, 0x38
    mov fs, ax
    mov eax, [fs:0x50000]
    cmp eax, 0x11223346
    jne fail
    BAD_ACCESS 13, store, fs, 0x50000, eax
    cmp dword [COUNT], 13
    jne fail
    cmp dword [0x1000], 0x13579BDF
    jne fail
    cmp dword [0x50000], 0x11223346
    jne fail
    mov eax, 0x53480000
    out 0xE4, eax
    mov al, 1
    out 0xE0, al
    hlt

gp_handler:
    cmp ebp, 13
    jne wrong_fault
    jmp fault_common
ss_handler:
    cmp ebp, 12
    jne wrong_fault
fault_common:
    cmp dword [esp], 0
    jne wrong_fault
    cmp dword [esp+4], esi
    jne wrong_fault
    inc dword [COUNT]
    mov [esp+4], edi
    add esp, 4
    iretd
wrong_fault:
    mov eax, 0x5348FFFF
    jmp report_fail
fail:
    mov eax, [CASE]
    or eax, 0x53480000
report_fail:
    out 0xE4, eax
    mov al, 0xFF
    out 0xE0, al
    hlt
