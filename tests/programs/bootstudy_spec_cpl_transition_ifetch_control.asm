; Control: CR3 kills CPL0 speculative line before the same CPL3 branch.
%define KILL_SPEC 1
%include "programs/bootstudy_spec_cpl_transition_ifetch.asm"
