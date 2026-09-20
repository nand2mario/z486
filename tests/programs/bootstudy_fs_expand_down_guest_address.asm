; Exact captured EAX and unaligned operand, with high linear pages remapped.
bits 32

; Enter 16-bit code through an architectural far jump, rather than relying
; on testbench cache forcing to change the decoder's initial default size.
    mov esp, 0x7F000
    mov dword [0x3000], 0
    mov dword [0x3004], 0
    mov dword [0x3008], 0x0000FFFF
    mov dword [0x300C], 0x00CF9A01 ; CS 08: base 10000, 32-bit
    mov dword [0x3010], 0x0000FFFF
    mov dword [0x3014], 0x00CF9200 ; DS/SS 10: flat 32-bit
    mov dword [0x3018], 0x0000FFFF
    mov dword [0x301C], 0x00009A01 ; CS 18: base 10000, 16-bit
    mov dword [0x3020], 0x000097A0
    mov dword [0x3024], 0x00409600 ; FS 20: expand-down, B=1
    mov word [0x3040], 0x0027
    mov dword [0x3042], 0x3000
    lgdt [0x3040]
    mov dword [0x4068], (0x08 << 16) | (gp_handler - $$)
    mov dword [0x406C], 0x00008E00
    mov word [0x3048], 0x07FF
    mov dword [0x304A], 0x4000
    lidt [0x3048]
    mov ax, 0x20
    mov fs, ax
    jmp 0x18:entry16

bits 16
entry16:

; Model the new Windows 0147:7F21 fault: 16-bit code, 32-bit address and
; operand overrides, FS writable expand-down with base 0 and limit 97A0.
; Offset 00C90FD6 is legal because it is ABOVE the expand-down lower bound.
    mov dword [dword 0x00C90FD6], 0
    mov eax, 0x00C90F9E
    db 0x67, 0x66, 0x64, 0xFF, 0x40, 0x38 ; inc dword ptr fs:[eax+38h]
    cmp dword [dword 0x00C90FD6], 1
    jne fail
    mov eax, 0x53470000
    out 0xE4, eax
    mov al, 1
    out 0xE0, al
    hlt
fail:
    mov eax, 0x53470001
    out 0xE4, eax
    mov al, 0xFF
    out 0xE0, al
    hlt

bits 32
gp_handler:
    mov eax, 0x5347000D
    out 0xE4, eax
    mov al, 0xFF
    out 0xE0, al
    hlt
