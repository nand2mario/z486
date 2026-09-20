bits 32
%ifndef THUNK_ITERATIONS
%define THUNK_ITERATIONS 32
%endif
%define PHASE 0x50080
%define COUNT 0x50084
%macro DESC 3
    mov dword [0x2000+%1],%2
    mov dword [0x2004+%1],%3
%endmacro
    mov esp,0x7F000
    DESC 0x00,0,0
    DESC 0x08,0x0000FFFF,0x00409A01 ; 32-bit code, base 10000
    DESC 0x10,0x000097A0,0x00409600 ; flat expand-down stack/data
    DESC 0x18,0x95101FFF,0x00009201 ; 16-bit stack, base 19510
    DESC 0x20,0x0000FFFF,0x00009A01 ; 16-bit code, base 10000
    DESC 0x28,0x00000038,0x00009205 ; TIB, base 50000
    DESC 0x30,0x2000003F,0x00009200 ; GDT alias, base 2000
    mov word [0x2040],0x003F
    mov dword [0x2042],0x2000
    lgdt [0x2040]
    mov dword [0x3060],(8<<16)|(unexpected-$$)
    mov dword [0x3064],0x00008E00
    mov dword [0x3068],(8<<16)|(unexpected-$$)
    mov dword [0x306C],0x00008E00
    mov word [0x2048],0x07FF
    mov dword [0x204A],0x3000
    lidt [0x2048]
    mov word [0x5000E],0x18
    mov word [0x5001C],0
    mov dword [0x50090],0x500D8
    ; CS has base10000 in this fixture, unlike the guest's flat CS167.
    mov dword [0x60100],(0x20<<16)|(thunk16-$$)
    mov ax,0x28
    mov fs,ax
    mov ax,0x10
    mov ds,ax
    mov es,ax
    mov ss,ax
    mov dword [gs:COUNT],THUNK_ITERATIONS
again:
    mov esp,0x1B35C
    mov ebp,0x1B39C
    mov ebx,0x01476EC3
    mov esi,0x17CD8
    mov edi,0x17CE2
    mov eax,0x11223344
    mov word [0x17CE2],0xBEEF
    mov dword [gs:PHASE],1
    push dword 0x17CE2
    push dword 1
    call wrapper
    mov dword [gs:PHASE],6
    cmp ebp,0x1B39C
    jne fail32
    cmp esp,0x1B35C
    jne fail32
    cmp eax,0x1234
    jne fail32
    cmp word [0x17CE2],0x1234
    jne fail32
    cmp ebx,0x01476EC3
    jne fail32
    cmp esi,0x17CD8
    jne fail32
    cmp edi,0x17CE2
    jne fail32
    dec dword [gs:COUNT]
    jnz again
    mov eax,0x534A0000
    out 0xE4,eax
    mov al,1
    out 0xE0,al
    hlt
unexpected:
    mov eax,[gs:PHASE]
    or eax,0x534A8000
    jmp report32
fail32:
    mov eax,[gs:PHASE]
    or eax,0x534A0000
report32:
    out 0xE4,eax
    mov al,0xFF
    out 0xE0,al
    hlt

times 0x1FFD-($-$$) db 0x90
wrapper:
    push ebp
    mov ebp,esp
    push ebx
    push esi
    push edi
    mov ecx,[ebp+8]
    push cs
    push dword thunk_return
    push eax
    mov eax,[0x50090]
    mov eax,[cs:eax+0x28]
    xchg [esp],eax
    o16 retf
thunk_return:
    mov dword [gs:PHASE],5
    cmp ebp,0x1B34C
    jne fail32
    cmp esp,0x1B340
    jne fail32
    mov ecx,[ebp+0xC]
    cmp ecx,0x17CE2
    jne fail32
    mov [ecx],ax
    movzx eax,ax
    pop edi
    pop esi
    pop ebx
    leave
    ret 8

bits 16
thunk16:
    push ds
    push ebx
    push fs
    push ebp
    mov ebp,esp
    push ss
    push ebp
    push di
    push esi
    mov esi,esp
    a32 mov dword [gs:PHASE],2
    cmp ebp,0x1B32C
    jne fail16
    cmp dword [gs:ebp],0x1B34C
    jne fail16
    test word [fs:0x1C],1
    jnz fail16
    push eax
    push cx
    mov ds,[cs:gdt_alias]
    mov bx,[fs:0x0E]
    and bl,0xF8
    mov ah,[bx+7]
    mov al,[bx+4]
    shl eax,16
    mov ax,[bx+2]
    pop cx
    mov ds,[ebp+0xA]
    mov bx,[ebp+6]
    mov ss,[fs:0x0E]
    sub esp,eax
    nop
    pop eax
    a32 mov dword [gs:PHASE],3
    cmp esp,0x1E10
    jne fail16
    pop esi
    pop di
    nop
    push cs
    call api_a
    push cx
    nop
    push cs
    call api_b
    push ax
    nop
    push cs
    call api_c
    pop ax
    movzx eax,ax
    a32 mov dword [gs:PHASE],4
    mov bp,sp
    lss esp,[bp]
    nop
    pop ebp
    pop fs
    pop ebx
    pop cx
    mov ds,cx
    mov es,cx
    o32 retf
gdt_alias: dw 0x30
api_a:
    push bp
    mov bp,sp
    push bx
    pop bx
    pop bp
    retf
api_b:
    push bp
    mov bp,sp
    cmp word [ss:bp+6],1
    jne fail16
    mov ax,0x1234
    pop bp
    retf 2
api_c:
    push bp
    mov bp,sp
    pop bp
    retf
fail16:
    a32 mov eax,[gs:PHASE]
    or eax,0x534A0000
    out 0xE4,eax
    mov al,0xFF
    out 0xE0,al
    hlt
