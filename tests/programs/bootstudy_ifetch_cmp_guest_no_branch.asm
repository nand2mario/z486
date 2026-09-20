; Preserve all instruction offsets while removing only the nearby JNZ.
%macro jnz 1
    times 6 db 0x90
%endmacro
%include "programs/bootstudy_ifetch_cmp_guest.asm"
