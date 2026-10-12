; mov_dr03_gpr.asm - DR0-3 keep their values and MOV DRn never writes a GPR
;
; The MOV DR/TR microcode selects DRn with COUNTR = 0x70+n (SELECT_DR_TR) and
; stores through the internal register file: MOV DRn,rd writes "SRCREG -> IRF"
; with SBAS (38E), MOV rd,DRn writes "-1 -> IRF" with LBAS (3B2). Index 0x70+n
; is the DR's descriptor-cache base, not a GPR; z486 wrote GPR n instead, so
; "mov ecx,dr0" set EAX = -1 (Windows 3.1 VMM: #GP on the next [eax] store).
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

    ; Distinct bystanders in every GPR index a DR number could alias.
    mov eax, 0xA0A0A0A0
    mov ecx, 0xC1C1C1C1
    mov edx, 0xD2D2D2D2
    mov ebx, 0xB3B3B3B3
    mov ebp, 0x5555AAAA
    mov esi, 0x51515151
    mov edi, 0xD1D1D1D1

    ; MOV DRn,rd from a register other than GPR n.
    mov dr0, esi
    mov dr1, edi
    mov dr2, ebp
    mov dr3, edx

    cmp eax, 0xA0A0A0A0
    jne .fail_1
    cmp ecx, 0xC1C1C1C1
    jne .fail_1
    cmp edx, 0xD2D2D2D2
    jne .fail_1
    cmp ebx, 0xB3B3B3B3
    jne .fail_1

    ; MOV rd,DRn into a register other than GPR n, then use GPR n as an
    ; address right away (the Windows VMM pattern: mov ecx,dr0 / mov [eax],ecx).
    mov eax, 0x100
    mov ecx, dr0
    mov [eax], ecx
    cmp ecx, 0x51515151
    jne .fail_2
    cmp eax, 0x100
    jne .fail_2

    mov ecx, 0x104
    mov eax, dr1
    mov [ecx], eax
    cmp eax, 0xD1D1D1D1
    jne .fail_3
    cmp ecx, 0x104
    jne .fail_3

    mov edx, 0x108
    mov eax, dr2
    mov [edx], eax
    cmp eax, 0x5555AAAA
    jne .fail_4
    cmp edx, 0x108
    jne .fail_4

    mov ebx, 0x10C
    mov eax, dr3
    mov [ebx], eax
    cmp eax, 0xD2D2D2D2
    jne .fail_5
    cmp ebx, 0x10C
    jne .fail_5

    ; The stores went where the untouched bases pointed.
    cmp dword [0x100], 0x51515151
    jne .fail_6
    cmp dword [0x10C], 0xD2D2D2D2
    jne .fail_6

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
