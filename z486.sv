// z486 - An x86 core with the original 386 microcode and 486-style pipelining
//
// nand2mario, July 2026
//
// Functional units
//   1. Prefetch and Bus Interface
//   2. Decode1 (structural decode)
//   3. Decode2 (literals capture and early-address)
//   4. Segmentation Unit
//   5. Paging Unit (including TLB)
//   6. Protection Test Unit
//   7. Execution (control unit for microcoded and hardwired instructions)
//   8. Write-back
//   9. Address and Data Units (EA, ALU, register file, shifter, flags)
//  10. x87 Coprocessor

module z486
    import z486_pkg::*;
#(
    parameter PROTECT_UMA_ROM = 0,
    parameter DCACHE_SET_BITS = 8,   // dcache size: 8 = 16KB, 7 = 8KB
    parameter ICACHE_SET_BITS = 8,   // icache size: 8 = 16KB, 7 = 8KB
    parameter ENABLE_X87 = 0,
    parameter [6:0] CLOCK_RATE_MHZ = 7'd85
)
(
    input              clk,
    input              reset_n,

    // 32-bit bus interface (ready/valid handshake)
    output     [31:2]  addr,        // Physical address [31:2]
    output      [3:0]  be,          // Byte enables
    output      [7:0]  burstcount,  // Burst length in DWORDs
    input      [31:0]  din,         // Data input
    output     [31:0]  dout,        // Data output
    output             valid,       // Request valid (held until ready)
    input              ready,       // Handshake: transfer on valid && ready
    output             write,       // 1=write, 0=read (stable while valid)
    output             io,          // I/O vs memory (1=I/O, 0=memory)
    input              resp_valid,  // Read data valid (1-cycle pulse)

    // Interrupts
    input              intr,        // Maskable interrupt request
    input              nmi,         // Non-maskable interrupt
    output             inta,        // Interrupt acknowledge

    // External memory writers can invalidate matching L1 lines.
    input      [31:0]  snoop_addr,
    input              snoop_valid,

    input              a20_enable,  // A20 gate input

    // Architectural execution rate: 0=full, 1=15, 2=30, 3=56 MHz.
    input       [1:0]  cpu_speed_sel,

    // Debug/test control
    input              single_step, // Halt after each instruction (for single-step tests)

    output     [15:0]  dbg_CS,
    output     [31:0]  dbg_EIP,
    output     [31:0]  dbg_CS_base,
    output             dbg_pe,
    output             dbg_vm,
    output     [31:0]  dbg_x87_state,

    // A fault while delivering #DF shuts down the 386 and requests reset.
    output reg          triple_fault_reset
);

//=============================================================================
// CPU state and cross-unit interconnect
//=============================================================================

// Architectural state and externally visible debug state.
reg dbg_first_done;                 // Debug: first instruction finished execution
reg halted;                         // Tracks when the core is halted
reg [31:0] debug_ip;                // Debug: IP at instruction completion

reg [31:0] CR0, CR2, CR3;
reg [31:0] DR6, DR7;
wire [31:0] EAX, ECX, EDX, EBX, ESP, EBP, ESI, EDI;
reg [31:0] EIP = 32'h0000FFF0;      // Architectural IP (next instruction) - reset vector

// Bits: 31..22 21 20 19 18 17 16 15..14 13..12 11 10 9  8  7  6  5  4  3  2  1  0
//       Rsvd   ID VIP VIF AC VM RF Rsvd  IOPL   OF DF IF TF SF ZF 0  AF 0  PF 1  CF
wire [31:0] EFLAGS;
wire [31:0] uc_flags;               // Internal ALU flags for microcode conditionals
wire [31:0] eflags_fwd;             // Includes pending ALU/shifter retirement
wire [31:0] eflags_ahead;           // Includes current-cycle ALU flags

// Shared internal registers. Their owning units drive these interconnects.
wire [31:0] TMPC, TMPG;
wire [31:0] PROTUN;                  // Protection register, owned by protection_unit
wire [31:0] SIGMA;                  // ALU result
wire [31:0] FLAGSB;                 // FLAGS backup for INT

wire [31:0] OPR_R;                  // Read operand register, owned by the paging unit
wire [31:0] OPR_W;                  // Bus operation data registers
wire [31:0] IND;                    // Internal address register
wire [31:0] IND_DELTA;              // Signed microcode address stride
wire [31:0] ind_linear;             // Relocated linear address of IND
wire        ind_linear_valid;
wire [31:0] ea_reg;                 // Current instruction EA, owned by address_unit

// Instruction-wide size state, consumed by control, address, and data units.
reg [1:0]  op_size;                 // Runtime operand size: 0=byte, 1=word, 2=dword (modifiable by BITS8/16/32)
reg [1:0]  op_size_decode;          // Decoded operand size (saved at i_issue, restored by BITSDE)
reg [1:0]  srcreg_size;             // Same as op_size most of the time, different for MOVZX/MOVSX and etc
reg [1:0]  srcreg_size_decode;      // Decoded srcreg_size (saved at i_issue, restored by BITSDE)
(* preserve *) reg [1:0] op_size_src;            // Local copy for generic source mux fanout
(* preserve *) reg [1:0] op_size_src_decode;
(* preserve *) reg [1:0] srcreg_size_src;
(* preserve *) reg [1:0] srcreg_size_src_decode;
wire       is_dword = (op_size == 2'd2); // Runtime dword flag

// Integer Data Unit interconnect.
wire [31:0] alu_src;                // ALU source input this cycle
wire [4:0] alu_op5;                 // ALU operation this cycle
wire [31:0] alu_src_r;              // Registered alu_src for jumps (32-bit)
wire [31:0] alu_result;

// Data Unit results consumed by control/address logic
wire [31:0] shift_result;

wire [31:0] muldiv_result;

// ALU source data for operand access
wire [31:0] source_value_live;
wire [31:0] alu_src_data;
wire [31:0] protun_write_value;
wire [15:0] cs_source_value;

// Segmentation and protection state shared across the address pipeline.
reg [15:0] ES = 16'h0000;
reg [15:0] CS = 16'hF000;           // Reset value
reg [15:0] SS = 16'h0000;
reg [15:0] DS = 16'h0000;
reg [15:0] FS = 16'h0000;
reg [15:0] GS = 16'h0000;
reg [15:0] LDTR, TR;                // Task Register
reg [31:0] SLCTR;                   // Selector temp used by protected-mode descriptor microcode (32-bit: LAR/LSL store full descriptor hi DWORD)
// Forward SLCTR from dest_value when being written in the same cycle
wire        slctr_fwd_en = uc_exec && (uc_dest == DEST_SLCTR || uc_dest == DEST_TMP_TR) && !prot_is_ptovrr;
wire [31:0] slctr_fwd = slctr_fwd_en ? dest_value : SLCTR;

wire [31:0] desc_raw_hi;            // Raw descriptor high DWORD, owned by protection_unit
wire        seg_cmd_valid;          // Segmentation command executes at i_issue/uc_exec

seg_desc_t desc_cache [0:7];        // ES..GS, TR, and LDTR hidden descriptors
wire D = desc_cache[SEG_CS].D_B;    // Default operand size
wire [31:0] idt_base;
wire [19:0] idt_limit;
wire [31:0] gdt_base;
wire [19:0] gdt_limit;
wire [31:0] CS_base = desc_cache[SEG_CS].base;

wire       pe = CR0[0];             // Protected mode enable
wire       vm = EFLAGS[17];         // Virtual 8086 mode

assign dbg_CS  = CS;
assign dbg_EIP = EIP;
assign dbg_CS_base = CS_base;
assign dbg_pe  = pe;
assign dbg_vm  = vm;
wire [1:0] cpl = vm ? 2'd3 : !pe ? 2'd0 : CS[1:0];

reg [2:0]  latched_pf_code;         // Latched page fault error code (for LPCR microcode access)
reg [31:0] latched_pf_addr;         // Latched faulting linear address (for LPCR microcode access)

// Frontend interconnect.
wire [63:0] win_d1;                 // registered raw window at the prefetcher's D1 cursor
wire [5:0]  d1_avail;               // bytes fetched beyond the D1 cursor
wire [3:0]  d1_adv;                 // D1 cursor advance this cycle (a prefix, or
                                    //   the instruction rest at handoff, 0-11)
wire [31:0] win_lit;                // D2 literal window at pop_cursor + lit_off
wire [5:0]  lit_avail;              // bytes fetched beyond that point
wire [4:0]  dec_lit_off;            // literal offset from the pop cursor
wire        dec_pop_now;            // one instruction completed D2: pop its bytes
wire [4:0]  dec_pop_len;            //   (registered length from the skeleton)
wire       pf_full;
wire       q_flush;                 // Flush queue (branch/jump) - combinational for i.immediate gating
wire       pe_mode_toggle_now;      // CR0.PE changed this cycle: re-decode next bytes in new mode
wire       uc_ctl_pref;             // Previous-cycle predecode: current uop is BUSOP_PREF

// The live compare read dest_value[0] -- doc/z486/old/core_notes_v51.md #1
wire cr0_wr_bit0 = (uc_source == SRC_MDTMP) ? muldiv_result[0] :
                   (uc_source == SRC_SIGMA) ? SIGMA[0]  : 1'b0;
assign pe_mode_toggle_now = uc_exec && (uc_dest == DEST_CR0) && (cr0_wr_bit0 != CR0[0]);
assign q_flush = (uc_exec && uc_ctl_pref && !uc_pref_suppress_prev && !early_redirected)
               || early_redirect || pe_mode_toggle_now;
// synthesis translate_off
always @(posedge clk)
    if (reset_n && uc_exec && (uc_dest == DEST_CR0) && (dest_value[0] !== cr0_wr_bit0))
        $fatal(1, "CR0-WRITE SOURCE INVARIANT BROKEN: uc_addr=%03x src=%02x dv0=%b fast0=%b",
               uc_addr, uc_source, dest_value[0], cr0_wr_bit0);
// synthesis translate_on

wire        page_fault;             // Page fault (declared fully at paging unit instantiation)
wire [1:0]  prot_cpl;               // CPL for protection unit (declared fully near protection logic)

// Paging and memory interconnect.
wire        mem_servicing;          // memory request in flight
wire        mem_dly_grace;          // optimistic read: DLY may execute this (lookup) cycle
wire        mem_write_dly_grace;    // posted write in PG_MEM_TLB: next non-bus uop may execute now
wire        mem_write_wait;         // unposted demand write still fault-capable: stall ALL uops
                                    // (its instruction may already have chained away)
wire        mem_opt_wait;           // optimistic read missed: stall all uops until fill done
wire        mem_accepted;           // memory request accepted (ready pulse)
wire        mem_complete_now;       // combinational, request completing THIS cycle
wire        mem_read_complete;      // paging-owned demand read data is valid

// Prefetch ↔ paging unit toggle signals
wire        pf_req_toggle;
wire [31:0] pf_linear_addr;
wire        pf_redirect_queued;
wire        pf_ack_toggle;
wire [127:0] pf_rdata;
wire        pf_fault;
wire [2:0]  pf_fault_code;
wire [31:0] pf_fault_addr;
wire        ifetch_page_fault;
wire [2:0]  ifetch_fault_code;
wire [31:0] ifetch_fault_addr;
wire        decoder_fetch_blocked;

// Physical cache channels connect paging, memory arbitration, and x87.
wire        dcache_req_valid;
wire [31:0] dcache_req_phys_addr_raw;
wire        dcache_req_write;
wire [3:0]  dcache_req_be;
wire [31:0] dcache_req_wdata;
wire        dcache_req_is_io;
wire        dcache_req_is_inta;
wire        dcache_req_is_x87;
wire        dcache_req_is_vga_mem;
wire        dcache_req_accepted;
wire        dcache_req_complete;
wire        dcache_read_complete;
wire [31:0] dcache_rdata;
wire        icache_req_valid;
wire [31:0] icache_req_phys_addr_raw;
wire        icache_req_accepted;
wire        icache_req_complete;
wire [127:0] icache_rdata;

wire        x87_req_selected;
wire        x87_req_accepted;
wire        x87_req_complete;
wire        x87_read_complete;
wire [31:0] x87_rdata;
wire        x87_busy_n;
wire        x87_pereq;
wire        x87_error_n;
wire        x87_direct_active;
wire        x87_direct_mem_req;

// Microcode and macro-instruction lifecycle interconnect.
// After a jump, the next micro-op still executes (delay slot) before jump takes effect.
wire [11:0] uaddr_next;             // Next address, launched early to the ucode ROM
wire [11:0] uaddr;                  // Address being fetched in the current ucode pipeline
wire [11:0] uc_addr;                // Address of current uc (for debug)
wire [11:0] uc_addr_mem_r;          // Address aligned with the ROM q_mem stage
wire [50:0] uc;                     // Current microcode word + pre-computed bits (50:37)
wire [50:0] uc_next;
wire [5:0]  uc_buscode;             // Bus operation code from microcode
wire [6:0]  uc_dest;                // Destination field from microcode
wire [5:0]  uc_source;              // Source field from microcode
wire [31:0] dest_value;             // Data Unit-selected destination value
wire [6:0]  uc_aluop;               // ALU operation / microcode jump condition
wire [2:0]  uc_opcode;              // RNI/RPT operation field
wire        uc_is_rni;
wire        alu_update_flags;
wire        uc_bus_or_dly;
wire        uc_is_mem_busop;
wire        uc_is_write;
wire        uc_is_check_write;
wire        uc_is_word_op;
wire        uc_is_dword_op;
wire        uc_jpereq_fwd;
wire        uc_p_io_rd;
wire        uc_p_io_wr;
wire        uc_p_iack;
wire        uc_p_pure_dly;
wire        uc_p_rpt;
wire        uc_p_wio;
wire [11:0] prot_jump_addr;         // Protection-unit microcode redirect
wire        gp_fault_trigger;       // Segmentation/protection #GP request
wire        div_overflow;           // Data Unit divide exception request

wire seq_advance;
seq_redirect_t seq_fault_redirect;
seq_redirect_t seq_boundary_redirect;
seq_condition_t seq_conditions;

// Instruction lifecycle: D2 start -> issue into EX -> first EX -> RNI -> delay slot.
wire       i_entry;                 // Normal (non-chained) D2 start
wire       i_issue;                 // D2 transfers one instruction into EX
reg        i_first;                 // First ucode execution cycle after issue
wire       i_rni;                   // RNI detected in this cycle (combinational from uc bits)
wire       i_rni_delay;             // RNI delay slot - RNI has been executed. this is last instruction cycle

// Interrupt-controller outputs used by D2 admission and execution boundaries.
wire       intr_pending;
wire       nmi_request_active;
wire       interrupt_pending;
wire       inhibit_interrupts;
reg        tf_active_r;
reg        tf_trap_suppress_r;

// Fault requests cross the address, data, D2, and sequencer boundaries.
wire       any_fault_issue;
wire       any_fault;
reg        any_fault_r;

wire       d2_start;                // Launch a macro entry address into the ROM
wire [11:0] d2_start_entry;         // Entry address selected on d2_start
wire       d2_valid;                // A launched macro entry is resident in D2
wire       d2_ready;                // D2 may transfer its instruction to EX
wire       d2_push;                 // Decoder completed the resident D2 payload
reg        d2_valid_r;
reg        d2_waited_r;             // D2 held after predecessor delay slot
reg        d2_stale_slot_r;         // D2 also waited during predecessor delay slot
reg        throttle_parked_r;       // predecessor retired; successor D2 waits for rate debt
wire [11:0] d2_entry_r;             // Effective entry resident in D2
wire        d2_rom_mem_resident;    // D2 entry tag aligned with ROM q_mem
wire       init_cycle = d2_valid_r; // Temporary waveform alias; not control logic
reg        uc_active;               // Tracks when instruction execution has begun
reg        fault_suppress_delay_slot;   // Fault handling: suppress delay slot after fault triggers
reg        interrupt_entry;         // Interrupt handler is being entered
reg        stack_init_pending;      // Cycle after i_issue for a stack operation
wire       prot_test_inflight;      // Protection test is waiting for a result
wire [31:0] COUNTR;                 // Data Unit counter register

// Hardwired common-instruction control. See doc/z486/hardwired_instructions.md.
// A recipe contains one to three native uSteps; chaining may reclaim an obsolete
// RNI slot. Results remain owned by the Data and Address Units.
recipe_state_t recipe_state;               // Current hardwired recipe state
recipe_pending_write_t recipe_mem_write;   // Deferred load commit
recipe_pending_write_t recipe_shift_write; // Deferred shift commit
wire [31:0] recipe_shift_data;              // Deferred shift result
wire       recipe_slot_stale;               // Reclaimed RNI slot is stale
wire       hardwired_off;                    // Simulation-only disable
wire       recipe_rni;                       // Current uStep contains RNI
wire       branch_ustep_redirect;     // bounded branch uStep redirects frontend
wire       branch_ustep_exec;         // execute bounded branch uStep this cycle
recipe_meta_t d2_recipe;             // D2 instruction's generated recipe class
wire       x87_direct_candidate;              // D2 entry is the direct x87 overlay
wire       d2_hardwired;             // D2 entry is a bounded hardwired recipe
wire       chain_start;                // launch a chained hardwired successor
wire       chain_from_next;               // successor comes from registered skid entry
wire [11:0] chain_entry;          // chained successor's ROM entry
wire       jcc_fold_active;            // not-taken Jcc occupies reclaimed slot

// Empty D2 may launch directly from D1. Otherwise the resident D2 skeleton
// supplies a registered entry address when the current instruction ends.
wire       d1_issue_direct;
wire [11:0] d1_issue_entry_point;
dec_entry_t d1_issue_entry;
wire       i_entry_raw = (i_rni || i_rni_delay || ~uc_active) && ~halted && !stall &&
                         (d1_issue_direct || !decq_empty) && !q_flush && !d2_valid &&
                         !fault_suppress_delay_slot && !interrupt_entry;
assign     i_entry = i_entry_raw && !any_fault_issue;

// D2 -> EX readiness. Keep this factored from the valid bit so v53 can move
// D2 ownership without changing the meaning of the transfer edge.
wire       tf_trap_pending = tf_active_r && !tf_trap_suppress_r;
wire       interrupt_deliverable = tf_trap_pending || nmi_request_active ||
                                   (intr_pending && EFLAGS[9] && !inhibit_interrupts);
wire       interrupt_at_boundary = i_rni_delay && interrupt_deliverable && !single_step;

// Fixed-clock CPU throttle. Hardwired memory/stack pairs remain atomic while
// the controller repays execution-rate debt between instructions.
wire        throttle_hold;
wire        throttle_release_ready;
wire        throttle_full;
// Loads/POPs defer their GPR commit, while PUSH recipes retain an architectural
// stack-update delay slot after WR. Splitting either pair can replay the entry
// word; for PUSH SP that changes the posted data from old SP to post-push SP.
wire        throttle_atomic_chain = recipe_state.hardwired && uc_active && i_rni &&
    ((recipe_state.commit_sel == RECIPE_COMMIT_MEM) ||
     ((recipe_state.commit_sel == RECIPE_COMMIT_ESP) && recipe_state.slot_has_work));
// D2 launch is normally hidden under the predecessor's last cycle. When the
// predecessor has already retired, overlap it with the final repayment cycle.
wire        throttle_release_cycle = throttle_parked_r && throttle_release_ready;

assign     d2_valid = d2_valid_r;
assign     d2_ready = d2_push && !stall &&
                      (!throttle_hold || throttle_atomic_chain ||
                       throttle_release_cycle) && !any_fault_issue &&
                      !(i_rni && tf_trap_pending && !single_step) &&
                      !interrupt_at_boundary && !q_flush;
assign     i_issue = d2_valid && d2_ready;

wire       core_live = !halted && uc_active && !fault_suppress_delay_slot && !interrupt_entry;
wire       dly_grace_now = mem_dly_grace && uc_p_pure_dly;
wire       posted_write_release = mem_write_dly_grace && !uc_busreq;    // release non-busop writes after one cycle
wire       mem_block_busy = (uc_bus_or_dly && !dly_grace_now && !posted_write_release) ||
                            mem_opt_wait || mem_write_wait; // demand op in flight
wire       mem_block_idle = (uc_busreq && !mem_accepted);  // uop wants the bus, paging not ready
wire       stall_mem = mem_servicing ? mem_block_busy : (mem_req_current && !mem_accepted);
wire       stall_wio = (uc_is_wio && !interrupt_pending && !single_step);
wire       stall_x87_direct;
// An entry may reach the ROM before its D2 literals arrive. Let the ending
// instruction execute its architectural RNI delay slot, then hold the ROM word
// until D2 can transfer it to EX.
wire       stall_d2 = d2_valid && !d2_push && !i_rni_delay;
wire       stall = stall_mem || stall_wio || stall_d2 || stall_x87_direct;

// Repeat
wire       prot_result_now;
wire       repeat_active = uc_is_rpt && (COUNTR[4:0] != 0 || prot_test_inflight) && !prot_result_now
                           && !(uc_is_wio && interrupt_pending);

// uc_exec: master enable for microcode execution
wire       d2_release_hold = d2_valid && (d2_waited_r || d2_stale_slot_r);
wire       uc_exec = core_live && !(mem_servicing ? mem_block_busy : mem_block_idle) &&
                     !stall_wio && !stall_d2 && !stall_x87_direct &&
                     !d2_release_hold && !throttle_parked_r && !recipe_slot_stale;
wire       uc_exec_writeback = uc_exec;  // local copies for reducing fanout
wire       uc_exec_shift = uc_exec;

assign     seg_cmd_valid = i_issue || uc_exec;

dec_entry_t i_bus;            // Instruction resident in unified D2
wire       decq_has_jmp_call; // D1/D2 holds a JMP/CALL rel (halt speculative prefetch)
wire       decq_empty;        // Legacy name: unified D2 has no skeleton
dec_entry_t i_bus2;           // Registered D1 skid successor
dec_entry_t i;                // Current instruction (latched at i_issue; written far below)
wire       decq_has2;         // i_bus2 is valid

dec_entry_t d2_entry;         // entry completing D2 this cycle (AGU/i_entry source)
ea_dec_t    d2_agu_dec;       // EA decode for d2_entry

// A normal boundary uses i_entry. Chaining launches the successor on the
// predecessor's transfer/execute edge and may overlap issue.
assign d2_start = i_entry || chain_start;
wire [11:0] d2_start_entry_arch = chain_start
                      ? chain_entry
                      : (decq_empty ? d1_issue_entry_point : i_bus.entry_point);
// D1 resolves qualified overlays. Keep live CR0/x87 state out of the D2 ROM
// address; unsafe cases branch to the generated fallback after i_issue.
assign d2_start_entry = d2_start_entry_arch;

// q is EX-owned. A completed ROM lookup may wait in q_mem, but it advances
// into q only on the D2->EX transfer edge.
wire        d2_rom_cancel = interrupt_at_boundary || any_fault || any_fault_issue;
wire        microcode_rom_base_ce = !stall_mem && !stall_wio && !repeat_active;
(* noprune *) reg [2:0] early_kind_probe_r;
wire [5:0]  uc_source_shift;
wire [3:0]  uc_shift_source_class;
wire [1:0]  uc_shift2_source;
wire        uc_is_shift2;
wire [5:0]  uc_alu_src_shift;
wire [6:0]  uc_aluop_shift;
wire [2:0]  uc_dly_source;
wire [8:0]  uc_mem_ctrl;
wire        uc_fpu_f8;
wire        microcode_rom_ce;
wire [2:0]  d2_kind;


//=============================================================================
// Unit 1: Prefetch queue and Bus Interface
//=============================================================================
wire [31:0] pf_flush_addr;          // Prefetch flush address

memory #(
    .PROTECT_UMA_ROM(PROTECT_UMA_ROM),
    .DCACHE_SET_BITS(DCACHE_SET_BITS),
    .ICACHE_SET_BITS(ICACHE_SET_BITS),
    .ENABLE_X87(ENABLE_X87)
) memory_inst (
    .clk(clk),
    .reset_n(reset_n),
    .a20_enable(a20_enable),

    .dcache_req_valid(dcache_req_valid),
    .dcache_req_phys_addr_raw(dcache_req_phys_addr_raw),
    .dcache_req_write(dcache_req_write),
    .dcache_req_be(dcache_req_be),
    .dcache_req_wdata(dcache_req_wdata),
    .dcache_req_is_io(dcache_req_is_io),
    .dcache_req_is_inta(dcache_req_is_inta),
    .dcache_req_is_x87(dcache_req_is_x87),
    .dcache_req_is_vga_mem(dcache_req_is_vga_mem),
    .dcache_req_accepted(dcache_req_accepted),
    .dcache_req_complete(dcache_req_complete),
    .dcache_read_complete(dcache_read_complete),
    .dcache_rdata(dcache_rdata),

    .x87_req_selected(x87_req_selected),
    .x87_req_accepted(x87_req_accepted),
    .x87_req_complete(x87_req_complete),
    .x87_read_complete(x87_read_complete),
    .x87_rdata(x87_rdata),

    .icache_req_valid(icache_req_valid),
    .icache_req_phys_addr_raw(icache_req_phys_addr_raw),
    .icache_req_accepted(icache_req_accepted),
    .icache_req_complete(icache_req_complete),
    .icache_rdata(icache_rdata),

    .snoop_addr(snoop_addr),
    .snoop_valid(snoop_valid),

    .addr(addr),
    .be(be),
    .burstcount(burstcount),
    .din(din),
    .dout(dout),
    .valid(valid),
    .ready(ready),
    .write(write),
    .io(io),
    .resp_valid(resp_valid),
    .inta(inta)
);

// Prefetch Unit: 16-byte circular buffer
prefetch prefetch_inst (
    .clk(clk),
    .reset_n(reset_n),
    // Queue output to decoder
    .win_d1(win_d1),
    .d1_avail(d1_avail),
    .d1_adv(d1_adv),
    .win_lit(win_lit),
    .lit_avail(lit_avail),
    .lit_off(dec_lit_off),
    .q_full(pf_full),
    .pop_now(dec_pop_now),
    .pop_len(dec_pop_len),
    // Flush
    .q_flush(q_flush),
    .pf_flush_addr(pf_flush_addr),
    // Toggle interface to paging unit
    .pf_req_toggle(pf_req_toggle),
    .pf_linear_addr(pf_linear_addr),
    .pf_redirect_queued(pf_redirect_queued),
    .pf_ack_toggle(pf_ack_toggle),
    .pf_rdata(pf_rdata),
    .pf_fault(pf_fault),
    .pf_fault_code(pf_fault_code),
    .pf_fault_addr(pf_fault_addr),
    // Decode may run ahead while an older instruction is still active.  A
    // retained fetch fault becomes precise once EX is empty or the older
    // instruction reaches its non-stalled retirement boundary.
    .fetch_blocked(decoder_fetch_blocked &&
                   (!uc_active || (i_rni_delay && !stall))),
    .ifetch_fault(ifetch_page_fault),
    .ifetch_fault_code(ifetch_fault_code),
    .ifetch_fault_addr(ifetch_fault_addr),
    // Control
    .pf_suspend(page_fault),
    .halt_speculative(decq_has_jmp_call),

    // z486 speculative branch-target line
    .spec_req(pf_spec_req),
    .spec_linear(spec_target_lin),
    .spec_owner(pf_spec_owner_r),
    .spec_store_valid(pf_spec_store),
    .spec_store_linear(pf_spec_store_linear),
    .spec_global_kill(pf_spec_global_kill)
);

// z486 speculative branch-target fetch
wire        spec_br_rel8   = !i_bus.has_0f && (i_bus.opcode[7:4] == 4'h7 || i_bus.opcode == 8'hEB);
wire [31:0] spec_disp      = spec_br_rel8 ? {{24{i_bus.displacement[7]}}, i_bus.displacement[7:0]}
                                          : i_bus.displacement;
// Stale-EIP pop guard: a pop CHAINED into a control transfer's
wire        spec_eip_stale = uc_exec && recipe_rni && (uc_dest == DEST_eIP);
wire [31:0] spec_target_eip = EIP + ({27'd0, i_bus.length} + spec_disp);
wire [31:0] spec_target_lin = CS_base + spec_target_eip;
wire        pf_spec_req    = i_issue && d2_recipe.br_rel && i_bus.data32 && !hardwired_off &&
                             !spec_eip_stale;
// Ownership: set when an instruction's i_issue requests a spec fetch, cleared
// by any later pop, flush, or interrupt entry - so it is only up while the
// requesting branch itself is the current instruction, which is exactly when
// its taken-flush address provably equals the spec target.
reg pf_spec_owner_r;
always_ff @(posedge clk) begin
    if (!reset_n)
        pf_spec_owner_r <= 1'b0;
    else begin
        if (i_issue)
            pf_spec_owner_r <= pf_spec_req;
        if (q_flush || interrupt_entry || any_fault)
            pf_spec_owner_r <= 1'b0;
    end
end

// Store invalidation is line-selective inside prefetch. External coherence or
// an address-space change invalidates the speculative line conservatively.
wire        pf_spec_store;
wire [31:0] pf_spec_store_linear;
reg         pf_snoop_kill_r;
always_ff @(posedge clk) begin
    if (!reset_n)
        pf_snoop_kill_r <= 1'b0;
    else
        pf_snoop_kill_r <= snoop_valid;
end
wire        pf_spec_global_kill = pf_snoop_kill_r || cr3_write ||
                                  (uc_exec && (uc_dest == DEST_CR0));

//=============================================================================
// Unit 2: Decode1 (structural decode)
//=============================================================================
decoder decoder_inst (
    .clk        (clk),
    .reset_n    (reset_n),

    // Prefetch queue interface (two-cursor protocol)
    .win_d1     (win_d1),
    .d1_avail   (d1_avail),
    .d1_adv     (d1_adv),
    .win_lit    (win_lit),
    .lit_avail  (lit_avail),
    .lit_off    (dec_lit_off),
    .pop_now    (dec_pop_now),
    .pop_len    (dec_pop_len),

    // Mode signals
    .D          (D),
    .pe_enable  (pe & ~vm),  // Native protected mode: PE=1 and VM=0 (V86 uses real-mode entry points)

    // Control signals
    .q_flush    (q_flush),
    .i_issue      (i_issue),

    // Decoded instruction output
    .i_bus      (i_bus),
    .decq_empty (decq_empty),
    .i_bus2     (i_bus2),
    .decq_has2  (decq_has2),
    .decq_has_jmp_call(decq_has_jmp_call),

    // Unified D2 payload and registered D1 skid
    .d2_entry   (d2_entry),
    .d2_push    (d2_push),
    .d1_issue_direct(d1_issue_direct),
    .d1_issue_entry_point(d1_issue_entry_point),
    .d1_issue_entry(d1_issue_entry),
    .fetch_blocked(decoder_fetch_blocked)
);

//=============================================================================
// Unit 3: Decode2 - literals capture and early-address (EA decode, relocate,
//                    and required forwarding to start memory operations at i_issue)
//=============================================================================

// Decode an entry's precomputed EA selectors for the head and chain targets.
function automatic ea_dec_t ea_decode_of(input dec_entry_t e);
    ea_dec_t r;
    // Defaults (also the "no modrm / has moffs" case)
    r = '0;
    // base/index onehot selectors are precomputed in D1 (decq-registered)
    r.base_sel  = e.ea_base_onehot;
    r.index_sel = e.ea_index_onehot;
    if (e.has_modrm && !e.has_moffs) begin
        if (e.addr32) begin
            // 32-bit addressing mode
            r.scale     = e.has_sib ? e.sib[7:6] : 2'b00;
            r.s2b       = e.has_sib && (e.sib[5:3] == 3'b100);  // No index, scale to base
        end else begin
            // 16-bit addressing mode
            r.is16      = 1'b1;
        end

        // D2 literal capture has already normalized disp8 and leaves this zero
        // for addressing forms without a displacement.
        r.disp = e.displacement;

        // POP r/m (8F) with ESP base: Intel 386 says EA uses post-increment ESP.
        if (e.opcode == 8'h8F && e.addr32 && e.has_sib && e.sib[2:0] == 3'b100)
            r.disp = e.displacement + (e.data32 ? 32'd4 : 32'd2);
    end
    ea_decode_of = r;
endfunction

// One-hot GPR mux used only by the speculative D2 AGU observer.
function automatic [31:0] onehot_gpr_mux(input [7:0] sel);
    case (sel)
        8'h01: onehot_gpr_mux = EAX;
        8'h02: onehot_gpr_mux = ECX;
        8'h04: onehot_gpr_mux = EDX;
        8'h08: onehot_gpr_mux = EBX;
        8'h10: onehot_gpr_mux = ESP;
        8'h20: onehot_gpr_mux = EBP;
        8'h40: onehot_gpr_mux = ESI;
        8'h80: onehot_gpr_mux = EDI;
        default: onehot_gpr_mux = 32'h0;
    endcase
endfunction

ea_dec_t ea_dec_cur;    // queue head (i_entry latch source, chain2 target)
assign ea_dec_cur = ea_decode_of(i_bus);

// z486 chain-INTO memory/LEA (EA reg-match): the target's i_p
ea_dec_t chain_next_ea;  // chain1 target = the entry behind the head
assign chain_next_ea = ea_decode_of(i_bus2);

// Hardwired recipe policy: classify the D2 instruction, prove successor
// overlap safe, and replace only recipe slots known to be redundant.
hardwired_control hardwired_control_inst (
    .clk(clk),
    .reset_n(reset_n),
    .issue_instr(i_bus),
    .next_instr(i_bus2),
    .exec_instr(i),
    .issue_ea(ea_dec_cur),
    .next_ea(chain_next_ea),
    .decq_has2(decq_has2),
    .decq_empty(decq_empty),
    .d2_push(d2_push),
    .d2_kind(d2_kind),
    .d2_valid(d2_valid),
    .d2_waited(d2_waited_r),
    .i_issue(i_issue),
    .i_first(i_first),
    .uc_active(uc_active),
    .uc_exec(uc_exec),
    .i_rni(i_rni),
    .i_rni_delay(i_rni_delay),
    .uc_next_rni(uc_next[10:8] == 3'b000),
    .uc_aluop(uc_aluop),
    .alu_write_flags(uc_exec && alu_update_flags),
    .flags_live(eflags_fwd),
    .flags_ahead(eflags_ahead),
    .op_size(op_size),
    .mem_commit(recipe_mem_write),
    .shift_commit(recipe_shift_write),
    .q_flush(q_flush),
    .interrupt_entry(interrupt_entry),
    .interrupt_pending(interrupt_pending),
    .trap_active(tf_active_r),
    .single_step(single_step),
    .any_fault(any_fault),
    .any_fault_r(any_fault_r),
    .any_fault_issue(any_fault_issue),
    .throttle_hold(throttle_hold),
    .stall(stall),
    .issue_recipe(d2_recipe),
    .issue_hardwired(d2_hardwired),
    .x87_direct_candidate(x87_direct_candidate),
    .disabled(hardwired_off),
    .chain_start(chain_start),
    .chain_from_next(chain_from_next),
    .chain_entry(chain_entry),
    .recipe_rni(recipe_rni),
    .recipe_state(recipe_state),
    .slot_stale(recipe_slot_stale),
    .fold_active(jcc_fold_active),
    .branch_ustep_exec(branch_ustep_exec),
    .branch_redirect(branch_ustep_redirect)
);

// D2 residency and throttle state. Microcode ROM/address flow is owned by
// microsequencer; this block controls only the macro instruction presented to it.
always_ff @(posedge clk) begin
    if (!reset_n) begin
        d2_valid_r <= 1'b0;
        d2_waited_r <= 1'b0;
        d2_stale_slot_r <= 1'b0;
        throttle_parked_r <= 1'b0;
        stack_init_pending <= 1'b0;
    end else begin
        if (d2_start || i_issue || q_flush || any_fault) begin
            d2_waited_r <= 1'b0;
            d2_stale_slot_r <= 1'b0;
        end else if (stall_d2) begin
            d2_waited_r <= 1'b1;
        end else if (d2_valid && !d2_push && i_rni_delay &&
                     (uc_exec || recipe_slot_stale)) begin
            // q remains on the predecessor's slot while D2 waits. Suppress
            // that stale word when D2 becomes ready on the following cycle.
            d2_stale_slot_r <= 1'b1;
        end

        if (!d2_valid || i_issue || q_flush || any_fault ||
            interrupt_at_boundary || throttle_full)
            throttle_parked_r <= 1'b0;
        else if (d2_valid && i_rni_delay && throttle_hold &&
                 (uc_exec || recipe_slot_stale))
            // Retire the predecessor's architectural delay slot on this edge,
            // then keep the prefetched successor out of EX until release.
            throttle_parked_r <= 1'b1;

        if (any_fault)
            d2_valid_r <= 1'b0;
        if (i_entry)
            d2_valid_r <= 1'b1;

        if (i_issue) begin
            d2_valid_r <= 1'b0;
            i_first <= 1'b1;
            stack_init_pending <= i_bus.stack_op;
        end

        if (chain_start)
            d2_valid_r <= 1'b1;

        if (!stall) begin
            if (stack_init_pending && !i_issue)
                stack_init_pending <= 1'b0;
            if (i_first && !i_issue)
                i_first <= 1'b0;
        end

        if (q_flush || page_fault)
            d2_valid_r <= 1'b0;

        // Boundary recognition is last and overrides speculative D2 state.
        if (i_rni_delay && !stall && !page_fault &&
            ((tf_trap_pending && !single_step) ||
             (nmi_request_active && !single_step) ||
             (intr_pending && EFLAGS[9] && !single_step && !inhibit_interrupts)))
            d2_valid_r <= 1'b0;
    end
end

always_ff @(posedge clk) begin
    if (i_issue)
        early_kind_probe_r <= d2_kind;
end

// A chained PUSH's RNI word holds post-push ESP in SIGMA. A following stack
// recipe consumes this focused bypass during its own D2 address calculation.
wire        recipe_esp_fwd = recipe_rni && (recipe_state.commit_sel == RECIPE_COMMIT_ESP);

// RNI-slot architectural GPR writes use only these four sources in the Intel
// ROM. Keep this mux narrow: it feeds the next instruction's D2 EA bypass.
function automatic [31:0] dly_fwd_mux(input [2:0] s);
    case (s)
        3'd1:    dly_fwd_mux = SIGMA;
        3'd2:    dly_fwd_mux = OPR_R;
        3'd3:    dly_fwd_mux = COUNTR;
        3'd4:    dly_fwd_mux = 32'hFFFF_FFFF;
        default:    dly_fwd_mux = 32'h0;
    endcase
endfunction
wire [31:0] dly_fwd_value = dly_fwd_mux(uc_dly_source);

// synthesis translate_off
// Re-prove the narrow mux against the full source read on every delay-slot
// GPR write (the ROM can change; a new source field must be added here).
always @(posedge clk)
    if (reset_n && dly_gpr_we && uc_exec &&
        (dly_fwd_value !== ((uc_source == SRC_IRF2) ? IND : dest_value)))
        $fatal(1, "DLY-FWD MUX MISMATCH: uc_addr=%03x src=%02x narrow=%08x full=%08x",
               uc_addr, uc_source, dly_fwd_value, dest_value);
// synthesis translate_on

// Delay-slot GPR write descriptor for early-EA forwarding (which GPR the
// delay-slot uop writes, and how)
localparam [1:0] FWD_BLO = 2'd0, FWD_BHI = 2'd1, FWD_W = 2'd2, FWD_D = 2'd3;

// {we, sel[2:0], mode[1:0]} for a delay-slot write to microcode dest `dest`
function automatic [5:0] decode_dly_gpr(input [6:0] dest);
    reg       we; reg [2:0] sel; reg [1:0] mode; reg [2:0] rs;
    begin
        we = 1'b0; sel = 3'd0; mode = FWD_D;
        case (dest)
            DEST_DSTREG, DEST_SRCREG: begin
                rs = (dest == DEST_DSTREG) ? i.dst_reg_sel : i.src_reg_sel;
                we = 1'b1;
                if (op_size == 2'd0) begin               // byte: rs[2]=high-byte, rs[1:0]=GPR
                    sel  = {1'b0, rs[1:0]};
                    mode = rs[2] ? FWD_BHI : FWD_BLO;
                end else begin
                    sel  = rs;
                    mode = (op_size == 2'd1) ? FWD_W : FWD_D;
                end
            end
            DEST_EAX, DEST_ECX, DEST_EDX, DEST_EBX,
            DEST_ESP, DEST_EBP, DEST_ESI, DEST_EDI:
                begin we = 1'b1; sel = dest[2:0]; mode = FWD_D; end
            DEST_eSP:
                begin we = 1'b1; sel = 3'd4;
                      mode = (pe && desc_cache[SEG_SS].D_B) ? FWD_D : FWD_W; end
            DEST_AX, DEST_CX, DEST_DX, DEST_BX, DEST_SP, DEST_BP, DEST_SI, DEST_DI:
                begin we = 1'b1; sel = dest[2:0]; mode = FWD_W; end
            DEST_AL, DEST_CL, DEST_DL, DEST_BL:
                begin we = 1'b1; sel = {1'b0, dest[1:0]}; mode = FWD_BLO; end
            DEST_AH, DEST_CH, DEST_DH, DEST_BH:
                begin we = 1'b1; sel = {1'b0, dest[1:0]}; mode = FWD_BHI; end
            DEST_eAX_AL:
                begin we = 1'b1; sel = 3'd0;
                      mode = (op_size == 2'd0) ? FWD_BLO : (op_size == 2'd1) ? FWD_W : FWD_D; end
            DEST_eDX_AH: begin
                we = 1'b1;
                if (op_size == 2'd0) begin sel = 3'd0; mode = FWD_BHI; end  // AH
                else begin sel = 3'd2; mode = (op_size == 2'd1) ? FWD_W : FWD_D; end
            end
            DEST_eCX: begin we = 1'b1; sel = 3'd1; mode = i.addr32 ? FWD_D : FWD_W; end
            DEST_eSI: begin we = 1'b1; sel = 3'd6; mode = i.addr32 ? FWD_D : FWD_W; end
            DEST_eDI: begin we = 1'b1; sel = 3'd7; mode = i.addr32 ? FWD_D : FWD_W; end
            DEST_IRF: if (COUNTR[5:3] != 3'b100)
                begin we = 1'b1; sel = COUNTR[2:0]; mode = is_dword ? FWD_D : FWD_W; end
            default: ;
        endcase
        decode_dly_gpr = {we, sel, mode};
    end
endfunction

// Predecode from uc_next (the microword that becomes uc next cycle); register on
// the same enable as uc so dly_gpr_*_pre_r tracks decode_dly_gpr(uc_dest).
wire [5:0] dly_gpr_pre   = decode_dly_gpr(uc_next[30:24]);
wire       dly_is_irf_pre = (uc_next[30:24] == DEST_IRF);
reg        dly_gpr_we_pre_r;
reg [2:0]  dly_gpr_sel_pre_r;
reg [1:0]  dly_gpr_mode_pre_r;
reg        dly_is_irf_pre_r;
always_ff @(posedge clk) begin
    if (!reset_n) begin
        dly_gpr_we_pre_r <= 1'b0; dly_gpr_sel_pre_r <= 3'd0;
        dly_gpr_mode_pre_r <= FWD_D; dly_is_irf_pre_r <= 1'b0;
    end else if (microcode_rom_ce) begin
        dly_gpr_we_pre_r   <= dly_gpr_pre[5];
        dly_gpr_sel_pre_r  <= dly_gpr_pre[4:2];
        dly_gpr_mode_pre_r <= dly_gpr_pre[1:0];
        dly_is_irf_pre_r   <= dly_is_irf_pre;
    end
end

// Functional descriptor: registered predecode, IRF index taken live from COUNTR.
// !recipe_slot_stale: a hardwired instruction's slot word is stale and writes nothing;
// its result committed at the entry-word cycle, so the register file is current.
wire       dly_gpr_we   = i_rni_delay && !recipe_slot_stale &&
                          (dly_is_irf_pre_r ? (COUNTR[5:3] != 3'b100) : dly_gpr_we_pre_r);
wire [2:0] dly_gpr_sel  = dly_is_irf_pre_r ? COUNTR[2:0]              : dly_gpr_sel_pre_r;
wire [1:0] dly_gpr_mode = dly_is_irf_pre_r ? (is_dword ? FWD_D : FWD_W) : dly_gpr_mode_pre_r;
gpr_forward_t dly_gpr_forward;
assign dly_gpr_forward.valid = dly_gpr_we;
assign dly_gpr_forward.dst = dly_gpr_sel;
assign dly_gpr_forward.mode = dly_gpr_mode;
assign dly_gpr_forward.data = dly_fwd_value;

wire       dly_esp_fwd = dly_gpr_we && (dly_gpr_sel == 3'd4);
wire       shc_esp_fwd = recipe_shift_write.valid && (recipe_shift_write.dst == 3'd4) &&
                         (recipe_shift_write.size != 2'd0);
wire [31:0] forwarded_esp = recipe_esp_fwd ? SIGMA :
                            dly_esp_fwd  ? dly_fwd_value :
                            shc_esp_fwd  ? (recipe_shift_write.size == 2'd1
                                            ? {ESP[31:16], recipe_shift_data[15:0]}
                                            : recipe_shift_data) : ESP;

// D1 supplies registered selectors to the Address Unit. Literal displacement
// remains a D2 value and is consumed when the instruction fires.
ea_dec_t d2_start_ea_dec;
assign d2_start_ea_dec = (chain_start && chain_from_next) ? chain_next_ea :
                         (i_entry && decq_empty)
                             ? ea_decode_of(d1_issue_entry) : ea_dec_cur;
gpr_ref_t ea_base_ref;
gpr_ref_t ea_index_ref;
wire [31:0] ea_base_value;
wire [31:0] ea_index_value;
wire [31:0] ea_early;
wire [31:0] issue_ind_linear;

// D2-AGU observer
assign d2_agu_dec = ea_decode_of(d2_entry);
wire [31:0] d2_agu_base  = onehot_gpr_mux(d2_agu_dec.base_sel);
wire [31:0] d2_agu_index = onehot_gpr_mux(d2_agu_dec.index_sel);
wire [63:0] d2_agu_prep  = ea_scale_operands(
    d2_agu_base, d2_agu_index, d2_agu_dec.scale, d2_agu_dec.s2b);
wire [31:0] d2_agu_a = d2_agu_prep[63:32];
wire [31:0] d2_agu_b = d2_agu_prep[31:0];
wire [31:0] d2_agu_c = d2_agu_dec.disp;
wire [2:0] d2_agu_seg = d2_entry.mem_seg[2:0];
wire [31:0] d2_agu_segbase = desc_cache[d2_agu_seg].base;
wire [31:0] d2_agu_lin = (d2_agu_a ^ d2_agu_b ^ d2_agu_c)
                       + (((d2_agu_a & d2_agu_b) | (d2_agu_a & d2_agu_c) |
                           (d2_agu_b & d2_agu_c)) << 1)
                       + d2_agu_segbase;

// GPR-write snoop: one-hot of architectural GPRs written THIS cycle
function automatic [7:0] gpr_wr_expand(input [2:0] sel);
    gpr_wr_expand = (8'h1 << sel) | (8'h1 << {1'b0, sel[1:0]});
endfunction
// Architectural GPRs written by the current microword. Non-GPR destinations
// default to zero; segment/address-mode changes invalidate the sidecar through
// the independent controls below. This keeps their broad destination decode
// out of the D2 AGU conflict path.
function automatic [7:0] gpr_dest_mask(input [6:0] dst);
    gpr_dest_mask = 8'h00;
    case (dst)
        DEST_EAX, DEST_AX, DEST_AL, DEST_AH, DEST_eAX_AL:
            gpr_dest_mask = 8'h01;
        DEST_ECX, DEST_CX, DEST_CL, DEST_CH, DEST_eCX:
            gpr_dest_mask = 8'h02;
        DEST_EDX, DEST_DX:
            gpr_dest_mask = 8'h04;
        DEST_eDX_AH:
            gpr_dest_mask = op_size == 2'd0 ? 8'h01 : 8'h04;
        DEST_DL, DEST_DH:
            gpr_dest_mask = 8'h04;
        DEST_EBX, DEST_BX, DEST_BL, DEST_BH:
            gpr_dest_mask = 8'h08;
        DEST_ESP, DEST_SP, DEST_eSP:
            gpr_dest_mask = 8'h10;
        DEST_EBP, DEST_BP:
            gpr_dest_mask = 8'h20;
        DEST_ESI, DEST_SI, DEST_eSI:
            gpr_dest_mask = 8'h40;
        DEST_EDI, DEST_DI, DEST_eDI:
            gpr_dest_mask = 8'h80;
        DEST_DSTREG:
            gpr_dest_mask = gpr_wr_expand(i.dst_reg_sel);
        DEST_SRCREG, DEST_USTEP_BSWAP:
            gpr_dest_mask = gpr_wr_expand(i.src_reg_sel);
        DEST_USTEP_ALU:
            gpr_dest_mask = gpr_wr_expand(i.dst_reg_sel);
        DEST_IRF:
            if (COUNTR[5:3] != 3'b100)
                gpr_dest_mask = 8'h01 << COUNTR[2:0];
        default: ;
    endcase
endfunction
wire [7:0] d2_agu_ucmask = uc_exec ? gpr_dest_mask(uc_dest) : 8'h00;
wire [7:0] ea_inval_gpr =
    d2_agu_ucmask |
    (recipe_shift_write.valid ? gpr_wr_expand(recipe_shift_write.dst) : 8'h0) |
    ((uc_exec && recipe_mem_write.valid) ? gpr_wr_expand(recipe_mem_write.dst) : 8'h0) |
    ((uc_exec && recipe_rni && !any_fault && recipe_state.commit_sel == RECIPE_COMMIT_ALU)
        ? gpr_wr_expand(i.dst_reg_sel) : 8'h0) |
    ((uc_exec && recipe_rni && !any_fault && recipe_state.commit_sel == RECIPE_COMMIT_SIGSRC)
        ? gpr_wr_expand(i.src_reg_sel) : 8'h0) |
    ((uc_exec && recipe_rni && !any_fault && recipe_state.commit_sel == RECIPE_COMMIT_ESP)
        ? 8'h10 : 8'h0) |
    ((uc_exec && uc_aluop == ALUJMP_CLZF && i.has_0f && i.opcode == 8'hBD)
        ? gpr_wr_expand(i.src_reg_sel) : 8'h0);
// Clear-all events: segment state may change under any committed seg
// command or descriptor load; the effective-mask mode must be stable.
reg d2_agu_effmask_r;
always_ff @(posedge clk) d2_agu_effmask_r <= eff_mask_pending;
// Only cache-MUTATING segment commands clear the sidecars; INIT_SEG /
// UPDATE_SEG select which base to read (every memory pop issues one) and
// mutate nothing.
wire seg_cmd_mutates = (seg_cmd != SEG_CMD_NONE) &&
                       (seg_cmd != SEG_CMD_INIT_SEG) &&
                       (seg_cmd != SEG_CMD_UPDATE_SEG) &&
                       (seg_cmd != SEG_CMD_SPCR);
wire ea_inval_all = (seg_cmd_valid && seg_cmd_mutates) ||
                    (d2_agu_effmask_r != eff_mask_pending);

// Eligibility (MVP): plain 32-bit MEMORY modrm EA (mod!=11), no
// moffs/stack, 32-bit mask active, and no conflicting write in the
// compute cycle itself.
wire d2_agu_valid = d2_push &&
                    d2_entry.has_modrm && (d2_entry.modrm[7:6] != 2'b11) &&
                    !d2_entry.has_moffs &&
                    !d2_entry.stack_op && d2_entry.addr32 &&
                    eff_mask_pending &&
                    (((d2_agu_dec.base_sel | d2_agu_dec.index_sel) & ea_inval_gpr) == 8'h00);
wire [31:0] head_ea_lin = d2_agu_lin;
wire        head_ea_v   = d2_agu_valid;

// Same-cycle conflict mask: a write committing at the CONSUMING edge
wire head_ea_usable = head_ea_v &&
    (((i_bus.ea_base_onehot | i_bus.ea_index_onehot) & ea_inval_gpr) == 8'h00) &&
    !ea_inval_all;


//=============================================================================
// Unit 4: Segmentation Unit
//=============================================================================
wire [3:0]  mem_seg_sel;
wire        mem_seg_is_io;
wire        descsw_mode;
wire        mem_is_dtable;
wire        tss_access_flag;
wire [31:0] seg_base_pending;  // next seg_base_r from seg unit; for unified linear_address relocate
wire        eff_mask_pending;  // next (addr_size||is_dtable) from seg unit; 0 => mask offset to 16b
wire [31:0] seg_lar_result, seg_llim_result, seg_lbas_result;

// Segmentation unit command encoder
reg  [3:0]  seg_cmd;
reg  [3:0]  seg_cmd_target;
reg  [31:0] seg_cmd_data;
// Decoded instruction register (all fields from decoder, latched at i_issue)
// dec_entry_t i; -- declaration moved up beside i_bus2 (Quartus cannot
// forward-reference struct members from the fast-chain gates)
wire [3:0] modrm_resolved_seg = apply_seg_override_type(
    calc_default_seg_type(i.modrm, i.sib, i.has_sib, i.addr32), i.seg);

// Pre-computed default segment for new instruction (combinational, used by INIT_SEG)
wire [3:0] init_default_seg = i_bus.stack_op ? SEG_SS :
                              i_bus.has_moffs ? SEG_DS :
                              i_bus.has_modrm ? calc_default_seg_type(i_bus.modrm, i_bus.sib, i_bus.has_sib, i_bus.addr32) :
                              SEG_DS;
wire [3:0] init_final_seg = i_bus.stack_op ? init_default_seg :
                            apply_seg_override_type(init_default_seg, i_bus.seg);

// Pre-computed access size for limit check (replaces op_size + is_dword in seg unit)
// Limit-check the actual access width: RD W/WR W = word (seg/limit reads; o32
// stride only bumps ESP); else srcreg_size (byte for MOVSX/MOVZX, not dest op_size).
wire [1:0] gp_access_adj = uc_is_word_op ? 2'd1 :
                           (srcreg_size == 2'd0) ? 2'd0 : (srcreg_size == 2'd2) ? 2'd3 : 2'd1;

wire        mem_op_eligible, gp_fault_mem_op, gp_fault_wr_op, ss_segment_fault;
prot_transition_t prot_transition;

segmentation_unit seg_unit (
    .clk              (clk),
    .reset_n          (reset_n),
    // Command interface — descriptor cache manipulation
    .seg_cmd_valid    (seg_cmd_valid),
    .stssaf_pulse     (uc_exec && uc_aluop == ALUJMP_STSSAF),
    .ctssaf_pulse     (uc_exec && uc_aluop == ALUJMP_CTSSAF),
    .seg_cmd          (seg_cmd),
    .seg_target       (seg_cmd_target),
    .init_addr32      (i_bus.addr32),
    .init_stack_op    (i_bus.stack_op),
    .clear_descsw     (uc_dest == DEST_DESSTK),
    .seg_data         (seg_cmd_data),
    .desc_lo          (TMPC),
    .desc_hi          (desc_raw_hi),
    .slctr            (SLCTR[15:0]),
    .transition       (prot_transition),
    .desc_cache       (desc_cache),
    .idt_base         (idt_base),
    .idt_limit        (idt_limit),
    .gdt_base         (gdt_base),
    .gdt_limit        (gdt_limit),
    .lar_result       (seg_lar_result),
    .llim_result      (seg_llim_result),
    .lbas_result      (seg_lbas_result),
    // Segment state
    .seg_sel          (mem_seg_sel),
    .seg_is_io        (mem_seg_is_io),
    .is_dtable        (mem_is_dtable),
    .descsw_mode      (descsw_mode),
    .tss_access_flag  (tss_access_flag),
    // Address translation
    .pe               (pe),
    .vm               (vm),
    .cpl              (cpl),
    .offset           (IND),
    .access_size      (gp_access_adj),
    .check_en         (mem_op_eligible),
    .is_mem_op        (gp_fault_mem_op),
    .is_write         (gp_fault_wr_op),
    .seg_base_pending (seg_base_pending),
    .eff_mask_pending (eff_mask_pending),
    .seg_fault        (gp_fault_trigger),
    .is_stack_fault   (ss_segment_fault)
);

always_comb begin
    seg_cmd = SEG_CMD_NONE;
    seg_cmd_target = SEG_NONE;
    seg_cmd_data = dest_value;

    if (i_issue) begin
        seg_cmd_target = init_final_seg;
    end else if ((uc_buscode == BUSOP_IND_PLUS_ALU || uc_buscode == BUSOP_IND_ALU2 ||
                  uc_buscode == BUSOP_IND_SRC)
                 && (uc_dest == DEST_DES_OS || uc_dest == DEST_DES_SR)) begin
        seg_cmd_target = modrm_resolved_seg;
    end else begin
        seg_cmd_target = resolve_seg_target(uc_dest, seg_reg_sel, COUNTR[5:0]);
    end

    if (i_issue) begin
        seg_cmd = SEG_CMD_INIT_SEG;
    end else if (uc_dest == DEST_DESCSW) begin
        seg_cmd = SEG_CMD_DESCSW;
    end else begin
        case (uc_buscode)
            BUSOP_IND_PLUS_ALU,
            BUSOP_IND_ALU2,
            BUSOP_IND_SRC: begin
                seg_cmd = SEG_CMD_UPDATE_SEG;
            end
            BUSOP_SBRM: begin
                if (!pe || vm)
                    seg_cmd = SEG_CMD_SBRM;
            end
            BUSOP_SAR: begin
                seg_cmd = SEG_CMD_SAR;
            end
            BUSOP_SLIM: begin
                seg_cmd = (uc_dest == DEST_DESPTR) ? SEG_CMD_SLIM_TABLE : SEG_CMD_SLIM;
            end
            BUSOP_SBAS: begin
                if (uc_dest == DEST_DESPTR)
                    seg_cmd = SEG_CMD_SBAS;
            end
            BUSOP_SDEH: begin
                if (pe && !gate_detect_cond)   // use cond, not _now (uc_exec already in valid)
                    seg_cmd = SEG_CMD_SDEH;
            end
            BUSOP_SDES: begin
                if (pe && !gate_detect_cond) begin
                    seg_cmd = SEG_CMD_SDES;
                    seg_cmd_data = alu_src_data;
                end
            end
            BUSOP_SDEL: begin
                if (pe && !gate_detect_cond) begin
                    seg_cmd = SEG_CMD_SDEL;
                    // SDEL's descriptor-low operand is encoded in the ALU source
                    // field. Most sites use TMPC, but cross-privilege CALL uses TMPD.
                    seg_cmd_data = alu_src_data;
                end
            end
            BUSOP_SPCR: begin
                seg_cmd = SEG_CMD_SPCR;
            end
            default: ;
        endcase
    end
end  // always_comb


//=============================================================================
// Unit 5: Paging Unit (including TLB)
//=============================================================================

// WR W / RD W access width = |IND_DELTA| (the stack/TSS slot stride).
// Ordinary accesses use source width: MOVZX/MOVSX read byte/word operands
// even though their architectural destination and op_size are dword.
wire ind_delta_dword = (IND_DELTA == 32'd4) || (IND_DELTA == -32'd4);
wire [1:0] mem_eff_size = uc_is_word_op ? (ind_delta_dword ? 2'd2 : 2'd1) :
                          uc_is_dword_op ? 2'd2 : srcreg_size;

wire [31:0] mem_wdata = (uc_buscode == BUSOP_WR_OPR ||
                         uc_buscode == BUSOP_WR_OPR_WORD) ? OPR_R :
    uc_is_word_op ? source_value_live :
    (uc_dest == DEST_OPR_W) ? (stack_init_pending ? source_value_live : dest_value) :
    OPR_W;

// div_overflow fires only at the first DIV7/PREDIV word
assign any_fault_issue = gp_fault_trigger || page_fault;
assign any_fault = any_fault_issue || div_overflow;
// Registered any_fault is used for deferred SIGMA/TMPeSP writes.
always_ff @(posedge clk) any_fault_r <= any_fault;
wire        data_page_fault;
wire [2:0]  data_fault_code;
wire [31:0] data_cr2_out;
// The executing instruction is older than a blocked frontend fetch. If both
// faults arrive together, preserve the demand-side exception and CR2 value.
wire [2:0]  pg_fault_code = data_page_fault ? data_fault_code : ifetch_fault_code;
wire [31:0] pg_cr2_out = data_page_fault ? data_cr2_out : ifetch_fault_addr;
assign page_fault = data_page_fault || ifetch_page_fault;

// CR3 write detection for TLB flush
wire cr3_write = uc_exec && uc_buscode == BUSOP_SPCR && uc_dest == DEST_PDBR;

// IO request detection
wire mem_is_io = mem_seg_is_io;     // registered in segmentation_unit alongside seg_sel
wire io_busop_rd = uc_p_io_rd && mem_is_io;
wire io_busop_wr = uc_p_io_wr && mem_is_io;

wire iack_busop = uc_p_iack;        // IACK bus operation (interrupt acknowledge)

// A delayed D2 entry may already occupy the shared ROM q while it is still
// waiting for literals. It is not an EX uop yet and must not issue its bus op.
assign mem_op_eligible = core_live && !mem_servicing &&
                         !stall_d2 && !d2_release_hold &&
                         !throttle_parked_r;
// A failed protection test redirects after its third architectural delay uop.
// That uop may finish internal setup, but its protected bus operation must not
// escape before the fault handler takes control (notably denied VM86 I/O).
wire uc_data_busreq = !prot_redirect_prev &&
                      ((uc_is_mem_busop && !mem_is_io) ||
                       io_busop_rd || io_busop_wr);
wire uc_busreq = uc_data_busreq || iack_busop;
wire mem_req_current = mem_op_eligible && uc_busreq;    // drives paging unit
// Delay prefetch on upcoming demand memory
wire mem_req_upcoming = uc_next[39] && !halted && (uc_active || d2_valid);

// Implicit supervisor access: descriptor table and TSS reads, cross-privilege
// stack writes use CPL=0 for paging regardless of current CPL.
wire implicit_supervisor = mem_is_dtable || (mem_seg_sel == SEG_TR) ||
                           descsw_mode || (vm && CS[1:0] == 2'b00);
wire [1:0] pg_cpl = implicit_supervisor ? 2'b00 : cpl;

// Registered fault redirect state.
reg         gp_fault_r;
reg         ss_fault_r;

wire        mem_req_to_paging = mem_op_eligible &&
                                (uc_data_busreq || x87_direct_mem_req) &&
                                !gp_fault_trigger;
wire        iack_req_to_paging = mem_op_eligible && iack_busop && !gp_fault_trigger;
wire        mem_write_now = x87_direct_mem_req ? 1'b0 :
                            (uc_is_write || (io_busop_wr && mem_is_io));
wire [1:0]  paging_mem_eff_size = x87_direct_mem_req ? 2'd2 :
                                                               mem_eff_size;
wire [3:0]  mem_be_now = iack_busop ? 4'b1111 :
                          calc_be(paging_mem_eff_size, ind_linear[1:0]);
wire [31:0] paging_linear_addr = ind_linear;
assign pf_spec_store = mem_req_to_paging && mem_write_now && mem_accepted;
assign pf_spec_store_linear = paging_linear_addr;
wire        paging_live_valid  = ind_linear_valid;
wire        paging_mem_rd_ind = !x87_direct_mem_req &&
                                (uc_buscode == BUSOP_RD_IND);
wire        paging_is_write_access = !x87_direct_mem_req &&
                                      (uc_is_write || uc_is_check_write);
wire        mem_ea_read = x87_direct_mem_req ||
                          (i_first && instr_ind_is_ea);

// Paging unit instantiation
paging_unit paging_inst (
    .clk                (clk),
    .reset_n            (reset_n),
    .cr0                (CR0),
    .cr3                (CR3),
    .cr3_write          (cr3_write),

    // Memory/IO request: current RD/WR/IACK uop is held by stall until accepted.
    .mem_req            (mem_req_to_paging),
    .mem_inta_req       (iack_req_to_paging),
    .mem_inta_addr      (IND),
    .mem_ea_read        (mem_ea_read),      // modrm/stack/moffs reads SET-read (linear relocated at i_issue); microcode IND reads excluded
    .mem_req_precheck   (mem_op_eligible &&
                         (uc_data_busreq || x87_direct_mem_req)),
    .mem_req_upcoming   (mem_req_upcoming), // suppresses prefetch start to minimize contention
    .mem_accepted       (mem_accepted),     // ready: request accepted this cycle
    .mem_servicing      (mem_servicing),
    .mem_complete_now   (mem_complete_now), // combinational: bus op completing this cycle
    .mem_read_complete  (mem_read_complete),
    .mem_dly_grace      (mem_dly_grace),
    .mem_write_dly_grace(mem_write_dly_grace),
    .mem_opt_wait       (mem_opt_wait),
    .mem_write_wait     (mem_write_wait),
    .linear_addr        (paging_linear_addr),
    .live_valid         (paging_live_valid),
    .mem_op_size        (paging_mem_eff_size),
    .mem_write          (mem_write_now),
    .mem_wdata          (mem_wdata),
    .mem_rd_ind         (paging_mem_rd_ind),
    .is_write_access    (paging_is_write_access),
    .mem_check_only     (uc_is_check_write),
    .cpl                (pg_cpl),
    .mem_is_io          (mem_is_io),
    .mem_be             (mem_be_now),

    // Prefetch (toggle protocol)
    .pf_req_toggle      (pf_req_toggle),
    .pf_ack_toggle      (pf_ack_toggle),
    .pf_redirect_queued (pf_redirect_queued),
    .pf_linear_addr     (pf_linear_addr),
    .pf_rdata           (pf_rdata),
    .pf_fault           (pf_fault),
    .pf_fault_code      (pf_fault_code),
    .pf_fault_addr      (pf_fault_addr),

    // Demand-side physical request interface
    .dcache_req_valid   (dcache_req_valid),
    .dcache_req_phys_addr(dcache_req_phys_addr_raw),
    .dcache_req_write   (dcache_req_write),
    .dcache_req_be      (dcache_req_be),
    .dcache_req_wdata   (dcache_req_wdata),
    .dcache_req_is_io   (dcache_req_is_io),
    .dcache_req_is_inta (dcache_req_is_inta),
    .dcache_req_is_x87  (dcache_req_is_x87),
    .dcache_req_is_vga_mem(dcache_req_is_vga_mem),
    .dcache_req_accepted(dcache_req_accepted),
    .dcache_req_complete(dcache_req_complete),
    .dcache_read_complete(dcache_read_complete),
    .dcache_rdata       (dcache_rdata),

    // Instruction-prefetch physical request interface
    .icache_req_valid   (icache_req_valid),
    .icache_req_phys_addr(icache_req_phys_addr_raw),
    .icache_req_accepted(icache_req_accepted),
    .icache_req_complete(icache_req_complete),
    .icache_rdata       (icache_rdata),

    // OPR_R
    .OPR_R              (OPR_R),

    // Status
    .page_fault         (data_page_fault),
    .fault_code         (data_fault_code),
    .cr2_out            (data_cr2_out)
);

always_ff @(posedge clk) begin
    if (!reset_n) begin
        gp_fault_r <= 1'b0;
        ss_fault_r <= 1'b0;
    end else begin
        gp_fault_r <= gp_fault_trigger;
        ss_fault_r <= ss_segment_fault;
    end
end

// CR3 register update
always_ff @(posedge clk) begin
    if (!reset_n)
        CR3 <= 32'h0;
    else if (cr3_write) begin
        CR3 <= IND;
    end
end


//=============================================================================
// Unit 6: Protection Test Unit (PLA4)
//=============================================================================
wire prot_pipe_en = !stall;
wire selector_null_wire = (slctr_fwd[15:3] == 13'b0) && !slctr_fwd[2];
wire prot_jump_valid;               // Status outputs retained for debug visibility
wire prot_validation_ok;
wire prot_result_valid;
wire [15:0] selector_desc_end = {slctr_fwd[15:3], 3'b111};
wire selector_oob_wire = slctr_fwd[2]
    ? ({12'h0, desc_cache[7].limit} < {4'h0, selector_desc_end})
    : (gdt_limit[15:0] < selector_desc_end);

wire [1:0]  prot_desc_dpl;
wire        prot_is_ptovrr;

protection_unit protection_unit_inst (
    .clk(clk),
    .reset_n(reset_n),
    .pipe_en(prot_pipe_en),

    .uc_exec(uc_exec),
    .uc_exec_writeback(uc_exec_writeback),
    .uc_aluop(uc_aluop),
    .uc_alu_src(uc_alu_src),
    .uc_dest(uc_dest),
    .uc_source_value(protun_write_value),
    .opr_r(OPR_R),

    .selector_rpl(slctr_fwd[1:0]),
    .selector_ti(slctr_fwd[2]),
    .selector_null(selector_null_wire),
    .selector_oob(selector_oob_wire),

    .cpl(cpl),
    .transition_rpl(SLCTR[1:0]),
    .pe_mode(pe),
    .cr0_et(CR0[4]),
    .cr0_ts(CR0[3]),
    .cr0_em(CR0[2]),
    .cr0_mp(CR0[1]),
    .cs_descriptor_dpl(desc_cache[SEG_CS].DPL),
    .cs_descriptor_exec(desc_cache[SEG_CS].seg_type[3]),
    .cs_descriptor_conforming(desc_cache[SEG_CS].seg_type[2]),
    .cs_selector_rpl(CS[1:0]),

    .test_mode(1'b0),
    .test_state_vector(10'h000),

    .jump_addr(prot_jump_addr),
    .jump_valid(prot_jump_valid),
    .validation_ok(prot_validation_ok),
    .result_valid(prot_result_valid),
    .protun_value(PROTUN),
    .desc_raw_hi(desc_raw_hi),
    .descriptor_dpl_live(prot_desc_dpl),
    .test_inflight(prot_test_inflight),
    .result_now(prot_result_now),
    .redirect_taken(prot_redirect_taken),
    .redirect_prev(prot_redirect_prev),
    .is_ptovrr(prot_is_ptovrr),
    .effective_cpl(prot_cpl),
    .transition(prot_transition)
);


//=============================================================================
// Unit 7: Execution - microcoded and hardwired instruction control
// "chaining" means hardwired issue into a reclaimed microcode slot.
//=============================================================================

// Fault delivery is sequencer control state. Address and data units contribute
// requests through the cross-unit fault signals declared at the front.
localparam logic [1:0] FAULT_IDLE       = 2'd0;
localparam logic [1:0] FAULT_DELIVERING = 2'd1;
localparam logic [1:0] FAULT_DOUBLE     = 2'd2;
reg  [1:0] fault_delivery_state;
reg        fault_seen_r;
reg        fault_combine_active;
reg        gp_fault_double_r;
wire       fault_start = any_fault && !fault_seen_r;
wire       double_fault_start = (fault_delivery_state == FAULT_DELIVERING) &&
                                fault_combine_active;

wire [5:0] uc_alu_src   = uc[36:31];  // ABCDEF: ALU source / jump offset
assign uc_dest          = uc[30:24];  // GHIJKLM: destination
assign uc_source        = uc[23:18];  // NOPQRS: source
assign uc_aluop         = uc[17:11];  // TUVWXYZ: ALU operation / jump condition
assign uc_opcode        = uc[10:8];   // 012: opcode (RNI, RPT, etc.)
assign uc_is_rni        = (uc_opcode == 3'b000); // testbench/waveform compatibility
// subcode field uc[7:6] (DLY/UNL/WIO) is consumed via ROM predecode bits only
assign uc_buscode       = uc[5:0];    // 56789&: bus operation code
assign alu_update_flags = uc[37];     // ALU result retires architectural flags
assign uc_bus_or_dly     = uc[38];
assign uc_is_mem_busop   = uc_mem_ctrl[0];
assign uc_is_write       = uc_mem_ctrl[1];
assign uc_is_check_write = uc_mem_ctrl[2];
assign uc_is_word_op     = uc_mem_ctrl[3];
assign uc_is_dword_op    = uc_mem_ctrl[4];
assign uc_jpereq_fwd     = uc_mem_ctrl[5];
assign uc_p_io_rd        = uc_mem_ctrl[6];
assign uc_p_io_wr        = uc_mem_ctrl[7];
assign uc_p_iack         = uc_mem_ctrl[8];
assign uc_p_pure_dly     = uc[48];
assign uc_p_rpt          = uc[49];
assign uc_p_wio          = uc[50];
wire       uc_jump_taken_prev;          // Jump taken last cycle (for RNi: terminate only in delay slot)
wire       uc_pref_suppress_prev;       // Taken LOOP/Jcc micro-jump cancels its speculative PREF delay slot

reg        instr_is_cmp;
reg        instr_ind_is_ea;
reg        instr_is_port_io;        // IN/OUT/INS/OUTS: VM86 must always check the TSS bitmap
reg  [4:0] alu_grp_op;              // Pre-decoded ALU op for ALUJMP_ALU/INCDEC (from i_bus at i_issue)
reg        instr_is_loop;           // E0/E1: LOOPNE/LOOPE (eliminates 7-bit compare from jump path)
reg  [1:0] instr_bt_sel;            // BT operation selector (eliminates 8-bit compare from ALU path)
reg  [4:0] instr_szext_op;          // Pre-decoded MOVZX/MOVSX/CBW ALU op
wire       prot_redirect_prev;      // Previous protection redirect suppresses its delay slot

reg [2:0]  seg_reg_sel;             // Segment register index (0=ES,1=CS,2=SS,3=DS,4=FS,5=GS)

wire [31:0] countr_masked = i.addr32 ? COUNTR : {16'h0, COUNTR[15:0]};
reg [31:0] TMPeIP;                  // Saved EIP for RPTI (repeat instruction)
reg [31:0] wr_restart_eip;          // TMPeIP captured at every demand-write issue: a write
                                    // fault (perm/walk/crossing) may surface after the issuing
                                    // instruction chained away and TMPeIP moved on
reg [31:0] TMPeSP;                  // Saved ESP for fault handling
wire       flags_backup_active;     // Set at i_issue/FLGSBA, cleared on interrupt_entry - guards FLAGSB writes
reg        misc1_flag;              // Set by SMISC1 {-33-}, tested by JMISC1 {-53-}
reg        misc2_flag;              // Set by SMISC2 {-35-}, tested by JMISC2 {-55-}
reg        error_code_flag;         // Set by SERRCF {-36-}, tested by JNERRC {-56-}
reg        interrupt_hw;            // Set for hardware interrupts, tested by JINTSW {-52-}
reg        task_saved_flag;         // STSKS/CTSKS latch: outgoing TSS has been saved during this switch
reg        no_fault_flag;           // SNOFLT/JNOFLT: descriptor probes fail by clearing ZF, not raising #GP
reg        rep_fault_flag;          // SREPF/CREPF/JREP: interrupted REP MOVS needs index/count correction
reg        jcc_active;              // Currently executing a Jcc instruction (for alu_src_r in BUSOP_IND_PLUS_ALU)
reg        instr_eip_written;       // EIP was written during instruction (RPTI restart)
reg        gate_in_progress;        // Prevent second LDTST (at 5C3) from re-triggering gate detection

// Hardwired relative-branch target and microcode PREF restart selection.
// Target formation remains beside EIP/redirect ownership, while chain_start
// owns branch eligibility, folding, and synthetic-RNI control.
// Microcode PREF restarts from IND.
wire [31:0] pf_flush_ip = IND;
assign pf_flush_addr = branch_ustep_redirect ? (CS_base + ea_reg) :
                       early_redirect        ? (CS_base + br_target) :
                       pe_mode_toggle_now    ? (CS_base + EIP) :
                                               (CS_base + pf_flush_ip);

wire        br_is_jcc      = (i.opcode[7:4] == 4'b0111 && !i.has_0f) ||
                             (i.opcode[7:4] == 4'b1000 && i.has_0f);
wire        br_is_jmp_rel  = !i.has_0f && (i.opcode == 8'hEB || i.opcode == 8'hE9);
wire        br_is_call_rel = !i.has_0f && (i.opcode == 8'hE8);
wire        br_is_rel8     = !i.has_0f && (i.opcode[7:4] == 4'b0111 || i.opcode == 8'hEB);
wire [31:0] br_disp        = br_is_rel8 ? {{24{i.displacement[7]}}, i.displacement[7:0]}
                                        : i.displacement;
wire [31:0] br_target      = EIP + br_disp;
`ifdef Z486_DEBUG_BRANCH_TARGET
// synthesis translate_off
always @(posedge clk) begin
    // Validate the microcode-PREF flush path
    if (reset_n && q_flush && !early_redirect && is_dword && (br_is_jcc || br_is_jmp_rel || br_is_call_rel) &&
        (pf_flush_ip !== (CS_base + br_target)))   // compare LINEAR vs LINEAR (pf_flush_ip is IND = CS_base+EIP+disp)
        $display("%0t: BR TARGET MISMATCH computed=%08x actual=%08x op=%02x CS:EIP=%0x:%0x",
                 $time, CS_base + br_target, pf_flush_ip, i.opcode, CS, EIP);
end
// synthesis translate_on
`endif

// i_first PRECISE early branch redirect (NOT a prediction).
wire br_jcc_taken = br_is_jcc && condition_true(i.opcode[3:0], eflags_fwd);
wire early_redirect = branch_ustep_redirect ||
                      (i_first && is_dword && br_is_call_rel);
reg  early_redirected;
// Cleared at the NEXT instruction's arrival
always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n)                            early_redirected <= 1'b0;
    else if (early_redirect)                 early_redirected <= 1'b1;
    else if (i_entry || i_issue || interrupt_entry) early_redirected <= 1'b0;
end

wire uc_is_wio = uc_p_wio;  // WIO: wait for interrupt/IO (HLT, only with RPT)
wire uc_is_rpt = uc_p_rpt;

// LOOP/REP Condition Logic
wire loop_zf_sense = instr_is_loop ? i.opcode[0] : i.rep_lock[0];  // ZF sense for branch
wire countr_will_be_nonzero = instr_is_loop ? (countr_masked != 32'h1) : (countr_masked != 32'h0);
wire zf_check = instr_is_loop ? (loop_zf_sense == EFLAGS[6]) : (loop_zf_sense != EFLAGS[6]);
wire loopne_condition = instr_is_loop ? (countr_will_be_nonzero && zf_check)
                                      : (!countr_will_be_nonzero || zf_check);

// GP Fault Detection — handled by segmentation_unit
assign gp_fault_mem_op = x87_direct_mem_req ||
                         (uc_is_mem_busop && (uc_buscode != BUSOP_RD_D));
assign gp_fault_wr_op = uc_is_write || uc_is_check_write;

always_comb begin
    seq_conditions = '0;
    seq_conditions.jncond = !condition_true(i.opcode[3:0], eflags_fwd);
    seq_conditions.count_zero = (countr_masked == 32'h0);
    seq_conditions.count_nonzero = (countr_masked != 32'h0);
    seq_conditions.count_low_not_one = (countr_masked[3:0] != 4'h1);
    seq_conditions.count_not_one = (countr_masked != 32'h1);
    seq_conditions.count_one = (countr_masked == 32'h1);
    seq_conditions.loopne = instr_is_loop ? !loopne_condition : loopne_condition;
    seq_conditions.greater = !uc_flags[6] && (uc_flags[7] == uc_flags[11]);
    seq_conditions.no_carry = !uc_flags[0];
    seq_conditions.no_overflow = !uc_flags[11];
    // PEREQ branches while the request signal is inactive.
    seq_conditions.pereq_inactive = ENABLE_X87 ? !x87_pereq : uc_jpereq_fwd;
    seq_conditions.flags_backup_inactive = !flags_backup_active;
    seq_conditions.tss_access = tss_access_flag;
    seq_conditions.interrupt_hw = interrupt_hw;
    seq_conditions.misc1 = misc1_flag;
    seq_conditions.task_unsaved = !task_saved_flag;
    seq_conditions.misc2 = misc2_flag;
    seq_conditions.no_error_code = !error_code_flag;
    seq_conditions.no_fault = no_fault_flag;
    seq_conditions.rep_fault = rep_fault_flag;
    seq_conditions.nested_task = EFLAGS[14];
    seq_conditions.io_ok = !pe ||
        (cpl <= EFLAGS[13:12] && (!vm || !instr_is_port_io));
    seq_conditions.no_interrupt = !interrupt_pending;
    seq_conditions.x87_not_busy = ENABLE_X87 ? x87_busy_n : 1'b1;
    seq_conditions.x87_error = ENABLE_X87 ? !x87_error_n : 1'b0;
    seq_conditions.task_16bit = !desc_cache[6].seg_type[3];
end

always_ff @(posedge clk) begin
    if (!reset_n) begin
        task_saved_flag <= 1'b0;
    end else if (uc_exec) begin
        if (uc_aluop == ALUJMP_STSKS)
            task_saved_flag <= 1'b1;
        else if (uc_aluop == ALUJMP_CTSKS)
            task_saved_flag <= 1'b0;
    end
end

always_ff @(posedge clk) begin
    if (!reset_n) begin
        no_fault_flag  <= 1'b0;
        rep_fault_flag <= 1'b0;
    end else begin
        // Fault/interrupt entry does not pulse i_issue, so these remain visible
        // to the corresponding fault-handler microcode.
        if (i_issue) begin
            no_fault_flag  <= 1'b0;
            rep_fault_flag <= 1'b0;
        end
        if (uc_exec) begin
            if (uc_aluop == ALUJMP_SNOFLT)
                no_fault_flag <= 1'b1;
            if (uc_aluop == ALUJMP_SREPF)
                rep_fault_flag <= 1'b1;
            else if (uc_aluop == ALUJMP_CREPF)
                rep_fault_flag <= 1'b0;
        end
    end
end

// TODO: fix special casing
// Suppress JMP in LD_DESCRIPTOR at 5D3 when Accessed bit needs GDT write-back.
// When A=0 in the descriptor, fall through to 5D5-5D7 which writes A=1 back to GDT.
// When A=1, take JMP to skip write-back (A already set).
wire desc_accessed_writeback = pe && (uc_aluop == ALUJMP_JMP) &&
                               (uc_addr == 12'h5D3) && !desc_raw_hi[8];

wire prot_redirect_taken;
// Qualified overlays launch without live architectural state on the ROM
// address. Their first ustep redirects unsafe cases to original microcode;
// the following overlay word is the architectural jump delay slot.
wire recipe_fallback_taken = uc_exec &&
    (uc_addr == i.entry_point) &&
    (recipe_action(i.entry_point) == RECIPE_ACTION_X87_M32_LOAD) &&
    !x87_direct_active;
wire gate_detect_cond = pe && (uc_buscode == BUSOP_SDEL) &&
                        !gate_in_progress && !desc_raw_hi[12] && (desc_raw_hi[11:8] == 4'hC);
wire gate_detect_now = uc_exec && gate_detect_cond;

assign seq_advance = ((((i_issue && !d2_waited_r) | uc_exec) |
                       (fault_suppress_delay_slot & !stall)) &
                      !halted && !repeat_active);

// Fault redirects override macro and chained entries. A page fault has priority over a
// simultaneous segment/general-protection fault, matching the original tree.
always_comb begin
    seq_fault_redirect = '0;
    if (gp_fault_r) begin
        seq_fault_redirect.valid = 1'b1;
        seq_fault_redirect.target = gp_fault_double_r ? UADDR_DOUBLE_FAULT :
                                    (ss_fault_r ? UADDR_STACK_FAULT :
                                                  UADDR_GENERAL_FAULT1);
    end
    if (page_fault) begin
        seq_fault_redirect.valid = 1'b1;
        seq_fault_redirect.target = double_fault_start
                                  ? UADDR_DOUBLE_FAULT : UADDR_PAGE_FAULT;
    end
end

// Interrupt dispatch is a macro-instruction boundary redirect. The explicit
// page-fault gate preserves fault priority without feeding this command back
// into demand-memory control.
always_comb begin
    seq_boundary_redirect = '0;
    if (i_rni_delay && !stall && !page_fault) begin
        if (tf_trap_pending && !single_step) begin
            seq_boundary_redirect.valid = 1'b1;
            seq_boundary_redirect.target = UADDR_SINGLE_STEP;
        end else if (nmi_request_active && !single_step) begin
            seq_boundary_redirect.valid = 1'b1;
            seq_boundary_redirect.target = UADDR_NMI;
        end else if (intr_pending && EFLAGS[9] && !single_step &&
                     !inhibit_interrupts) begin
            seq_boundary_redirect.valid = 1'b1;
            seq_boundary_redirect.target = UADDR_HARDWARE_IRQ;
        end
    end
end

// The sequencer consumes the execution, fault, and instruction-boundary
// commands above and owns the microcode ROM pipeline and address arbitration.
microsequencer microsequencer_inst (
    .clk(clk),
    .reset_n(reset_n),
    .rom_base_ce(microcode_rom_base_ce),
    .q_flush(q_flush),
    .d2_cancel(d2_rom_cancel),
    .d2_start(d2_start),
    .d2_start_entry(d2_start_entry),
    .d2_valid(d2_valid),
    .seq_advance(seq_advance),
    .macro_entry_valid(i_entry_raw),
    .chain_entry_valid(chain_start),
    .uc_exec(uc_exec),
    .repeat_active(repeat_active),
    .prot_redirect_prev(prot_redirect_prev),
    .jcc_fold_active(jcc_fold_active),
    .branch_ustep_exec(branch_ustep_exec),
    .macro_active(uc_active),
    .instr_eip_written(instr_eip_written),
    .any_fault(any_fault),
    .i_issue(i_issue),
    .stall(stall),
    .page_fault(page_fault),
    .pe(pe),
    .vm(vm),
    .cpl_nonzero(cpl != 2'b00),
    .desc_accessed_writeback(desc_accessed_writeback),
    .conditions(seq_conditions),
    .prot_redirect_valid(prot_redirect_taken),
    .prot_redirect_target(prot_jump_addr),
    .recipe_redirect_valid(recipe_fallback_taken),
    .recipe_redirect_target(recipe_fallback_entry(i.entry_point)),
    .set_rpl_redirect(prot_transition.set_rpl_redirect),
    .div_redirect_valid(div_overflow),
    .div_redirect_target(double_fault_start ? UADDR_DOUBLE_FAULT : UADDR_DIVIDE_ERROR),
    .gate_redirect(gate_detect_now),
    .fault_redirect(seq_fault_redirect),
    .boundary_redirect(seq_boundary_redirect),
    .uaddr(uaddr),
    .uaddr_next(uaddr_next),
    .uc_addr(uc_addr),
    .uc_addr_mem(uc_addr_mem_r),
    .i_rni_delay(i_rni_delay),
    .jump_taken_prev(uc_jump_taken_prev),
    .pref_suppress_prev(uc_pref_suppress_prev),
    .i_rni(i_rni),
    .d2_entry(d2_entry_r),
    .d2_kind(d2_kind),
    .d2_rom_mem_resident(d2_rom_mem_resident),
    .rom_q_ce(microcode_rom_ce),
    .uc(uc),
    .uc_next(uc_next),
    .uc_source_shift(uc_source_shift),
    .uc_shift_source_class(uc_shift_source_class),
    .uc_shift2_source(uc_shift2_source),
    .uc_is_shift2(uc_is_shift2),
    .uc_alu_src_shift(uc_alu_src_shift),
    .uc_aluop_shift(uc_aluop_shift),
    .uc_dly_source(uc_dly_source),
    .uc_mem_ctrl(uc_mem_ctrl),
    .uc_fpu_f8(uc_fpu_f8),
    .uc_ctl_pref(uc_ctl_pref)
);

// Interrupt paths clear delivery state only after committing handler CS/SS.
// A fault while #DF is being delivered requests processor reset.
always_ff @(posedge clk) begin
    if (!reset_n) begin
        fault_delivery_state <= FAULT_IDLE;
        fault_seen_r <= 1'b0;
        fault_combine_active <= 1'b0;
        gp_fault_double_r <= 1'b0;
        triple_fault_reset <= 1'b0;
    end else begin
        fault_seen_r <= any_fault;
        triple_fault_reset <= 1'b0;

        if (gp_fault_trigger)
            gp_fault_double_r <= double_fault_start;

        if (uc_exec && uc_aluop == ALUJMP_SCNTFF)
            fault_combine_active <= 1'b1;

        if (fault_start) begin
            case (fault_delivery_state)
                FAULT_IDLE: begin
                    fault_delivery_state <= FAULT_DELIVERING;
                    fault_combine_active <= 1'b0;
                end
                FAULT_DELIVERING: begin
                    if (fault_combine_active) begin
                        fault_delivery_state <= FAULT_DOUBLE;
                        fault_combine_active <= 1'b0;
                    end
                end
                default:          triple_fault_reset <= 1'b1;
            endcase
        end

        if (uc_exec &&
            (uc_addr == UADDR_TRAP_INT_DONE || uc_addr == UADDR_PRIV_INT_DONE) &&
            !any_fault) begin
            fault_delivery_state <= FAULT_IDLE;
            fault_combine_active <= 1'b0;
        end
    end
end

// synthesis translate_off
reg trace_fault_state_en = 1'b0;
initial trace_fault_state_en = $test$plusargs("trace_fault_state");

always @(posedge clk) begin
    if (reset_n && trace_fault_state_en) begin
        if (fault_start)
            $display("%0t FAULT-START state=%0d combine=%b gp=%b ss=%b pf=%b div=%b uaddr=%03x CS:EIP=%04x:%08x addr=%08x",
                     $time, fault_delivery_state, fault_combine_active,
                     gp_fault_trigger, ss_segment_fault, page_fault, div_overflow, uc_addr,
                     CS, EIP, page_fault ? pg_cr2_out : IND);
        if (uc_exec && uc_aluop == ALUJMP_SCNTFF)
            $display("%0t FAULT-COMBINE state=%0d uaddr=%03x", $time,
                     fault_delivery_state, uc_addr);
        if (uc_exec &&
            (uc_addr == UADDR_TRAP_INT_DONE || uc_addr == UADDR_PRIV_INT_DONE))
            $display("%0t FAULT-DONE state=%0d", $time, fault_delivery_state);
        if (triple_fault_reset)
            $display("%0t TRIPLE-FAULT RESET", $time);
    end
end
// synthesis translate_on

// Macro-instruction execution lifecycle and architectural boundary handling.
// This state consumes sequencer events but does not select microcode addresses.
always_ff @(posedge clk) begin
    if (!reset_n) begin
        uc_active <= 1'b0;
        instr_eip_written <= 1'b0;
        dbg_first_done <= 1'b0;
        debug_ip <= 32'h0;
        gate_in_progress <= 1'b0;
        interrupt_entry <= 1'b0;
        tf_active_r <= 1'b0;
        tf_trap_suppress_r <= 1'b0;
    end else begin
        if (!stall)
            interrupt_entry <= 1'b0;

        if (i_rni_delay && !stall && !page_fault) begin
            dbg_first_done <= 1'b1;
            if (single_step)
                halted <= 1'b1;
            if (!i_issue && !d2_valid)
                uc_active <= 1'b0;
        end

        if (uc_exec) begin
            if ((uc_aluop == ALUJMP_PTSELE) && gate_in_progress)
                gate_in_progress <= 1'b0;

            if (i_rni && uc_active && !instr_eip_written && !any_fault) begin
                if (branch_ustep_redirect)
                    debug_ip <= ea_reg;
                else if (uc_dest == DEST_EIP || uc_dest == DEST_eIP)
                    debug_ip <= is_dword ? alu_result : {EIP[31:16], alu_result[15:0]};
                else
                    debug_ip <= EIP;
            end

            if ((uc_dest == DEST_EIP || uc_dest == DEST_eIP) && in_rpti_routine)
                instr_eip_written <= 1'b1;

            if (i_rni && uc_active && instr_eip_written && !stall)
                uc_active <= 1'b0;  // RPTI restart

            if (gate_detect_now)
                gate_in_progress <= 1'b1;
        end

        fault_suppress_delay_slot <= any_fault || any_fault_r ||
                                     (fault_suppress_delay_slot && stall);

        if (i_issue) begin
            uc_active <= 1'b1;
            tf_active_r <= EFLAGS[8];
            tf_trap_suppress_r <= instr_mov_or_pop_ss(i_bus) ||
                (i_bus.opcode == 8'hCC) || (i_bus.opcode == 8'hCD) ||
                ((i_bus.opcode == 8'hCE) && EFLAGS[11]);
            instr_eip_written <= 1'b0;
            gate_in_progress <= 1'b0;
        end

        if (q_flush && pe_mode_toggle_now)
            uc_active <= 1'b0;

        // Fetch faults may arrive while no uop is active.
        if (page_fault) begin
            uc_active <= 1'b1;
            latched_pf_code <= pg_fault_code;
            latched_pf_addr <= pg_cr2_out;
        end

        // Interrupt recognition is last so it overrides speculative successor state.
        if (i_rni_delay && !stall && !page_fault) begin
            if (tf_trap_pending && !single_step) begin
                uc_active <= 1'b1;
                interrupt_entry <= 1'b1;
                tf_active_r <= 1'b0;
                tf_trap_suppress_r <= 1'b0;
            end else if (nmi_request_active && !single_step) begin
                uc_active <= 1'b1;
                interrupt_entry <= 1'b1;
            end else if (intr_pending && EFLAGS[9] && !single_step && !inhibit_interrupts) begin
                uc_active <= 1'b1;
                interrupt_entry <= 1'b1;
            end
        end
    end
end

// synthesis translate_off
always @(posedge clk)
    if (reset_n && throttle_parked_r && !d2_valid)
        $fatal(1, "throttle parked without a resident D2 successor");
// synthesis translate_on

// Instruction Signals (latched at i_issue)
always_ff @(posedge clk) begin
    if (!reset_n) begin
        i <= '0;
        instr_is_cmp <= 1'b0;
        instr_is_port_io <= 1'b0;
        instr_ind_is_ea <= 1'b0;
        jcc_active <= 1'b0;
    end else if (i_issue) begin
        i <= i_bus;
        i.entry_point <= d2_entry_r;
        // LSS/LFS/LGS (0F B2/B4/B5): put opcode in i.immediate for microcode XOR trick
        if (i_bus.has_0f && (i_bus.opcode == 8'hB2 || i_bus.opcode == 8'hB4 || i_bus.opcode == 8'hB5))
            i.immediate <= {24'h0, i_bus.opcode};
        // ESC commands write the architectural 11-bit FOP value to the x87:
        // primary opcode low three bits followed by the complete ModR/M byte.
        if (!i_bus.has_0f && (i_bus.opcode[7:3] == 5'b11011))
            i.immediate <= {21'h0, i_bus.opcode[2:0], i_bus.modrm};
        instr_is_cmp <= i_bus.opcode[7:2] == 6'b100000 || i_bus.opcode[7:3] == 5'b00111;
        instr_is_port_io <= !i_bus.has_0f &&
            ((i_bus.opcode[7:2] == 6'b011011) ||  // 6C-6F: INS/OUTS
             (i_bus.opcode[7:2] == 6'b111001) ||  // E4-E7: IN/OUT imm8
             (i_bus.opcode[7:2] == 6'b111011));   // EC-EF: IN/OUT DX
        instr_ind_is_ea <= i_bus.has_modrm || i_bus.stack_op || i_bus.has_moffs;
        // Pre-decode ALU group op: eliminates i.opcode/i.modrm muxes from ALU critical path
        alu_grp_op <= i_bus.opcode[7] ? i_bus.modrm[5:3] : i_bus.opcode[5:3];
        // Track Jcc instruction for alu_src_r substitution in BUSOP_IND_PLUS_ALU
        jcc_active <= (i_bus.opcode[7:4] == 4'b0111) ||
                      (i_bus.has_0f && i_bus.opcode[7:4] == 4'b1000);
        // Pre-decode: eliminates opcode comparisons from execution critical paths
        instr_is_loop <= (i_bus.opcode[7:1] == 7'b1110000);  // E0/E1
        // BT operation selector: immediate form (BA) uses modrm[4:3], register forms use opcode[4:3]
        instr_bt_sel <= (i_bus.opcode == 8'hBA) ? i_bus.modrm[4:3] : i_bus.opcode[4:3];
        // MOVZX/MOVSX/CBW pre-decode: opcode bits select sign/zero and byte/word source
        if (i_bus.opcode[0] || ~i_bus.opcode[5])              // 98, B7, BF
            instr_szext_op <= i_bus.opcode[3] ? ALU_SEXT : ALU_ZEXT;
        else                                                    // B6, BE
            instr_szext_op <= i_bus.opcode[3] ? ALU_SEXT_B : ALU_ZEXT_B;
    end
    if (interrupt_entry)
        jcc_active <= 1'b0;  // Clear on interrupt — prevent is_jcc from using stale displacement
end



// Sequencer predicates are control state, not architectural flags.
always_ff @(posedge clk) begin
    if (!reset_n) begin
        misc1_flag <= 1'b0;
        misc2_flag <= 1'b0;
        error_code_flag <= 1'b0;
        interrupt_hw <= 1'b0;
    end else begin
        if (i_issue && !halted) begin
            misc1_flag <= 1'b0;
            misc2_flag <= 1'b0;
            error_code_flag <= 1'b0;
            interrupt_hw <= 1'b0;
        end
        if (uc_exec) begin
            case (uc_aluop)
                ALUJMP_SMISC1: misc1_flag <= 1'b1;
                ALUJMP_SMISC2: misc2_flag <= 1'b1;
                ALUJMP_CMISC2: misc2_flag <= 1'b0;
                ALUJMP_SERRCF: error_code_flag <= 1'b1;
                ALUJMP_SINTHW: interrupt_hw <= 1'b1;
                default: ;
            endcase
        end
    end
end

//=============================================================================
// Unit 8: Same-cycle architectural commit
//=============================================================================

// SEGREG (Segment Register Operand)
always_ff @(posedge clk) begin
    if (!reset_n) begin
        seg_reg_sel <= 3'b0;
    end else if (i_issue && !halted) begin
        // Instruction start: load SEGREG and seg_reg_sel for segment instructions
        // PUSH ES (06), PUSH CS (0E), PUSH SS (16), PUSH DS (1E), POP ES (07), POP SS (17), POP DS (1F)
        if (i_bus.opcode[7:5] == 3'b000 && i_bus.opcode[2:1] == 2'b11) begin
            case (i_bus.opcode[4:3])
                2'b00: begin seg_reg_sel <= 3'd0; end  // ES
                2'b01: begin seg_reg_sel <= 3'd1; end  // CS
                2'b10: begin seg_reg_sel <= 3'd2; end  // SS
                2'b11: begin seg_reg_sel <= 3'd3; end  // DS
            endcase
        end
        // 0F A0/A1/A8/A9: PUSH/POP FS/GS
        else if (i_bus.has_0f && (i_bus.opcode == 8'hA0 || i_bus.opcode == 8'hA1 ||
                                  i_bus.opcode == 8'hA8 || i_bus.opcode == 8'hA9)) begin
            seg_reg_sel <= i_bus.opcode[3] ? 3'd5 : 3'd4;  // GS=5, FS=4
        end
        // MOV r/m,Sreg (8C) and MOV Sreg,r/m (8E)
        else if (i_bus.opcode == 8'h8C || i_bus.opcode == 8'h8E) begin
            seg_reg_sel <= i_bus.modrm[5:3];  // i.modrm reg field is segment index
        end
        // LES (C4), LDS (C5)
        else if (i_bus.opcode == 8'hC4) seg_reg_sel <= 3'd0;  // ES
        else if (i_bus.opcode == 8'hC5) seg_reg_sel <= 3'd3;  // DS
        // LSS (0F B2), LFS (0F B4), LGS (0F B5)
        else if (i_bus.has_0f && i_bus.opcode == 8'hB2) seg_reg_sel <= 3'd2;  // SS
        else if (i_bus.has_0f && i_bus.opcode == 8'hB4) seg_reg_sel <= 3'd4;  // FS
        else if (i_bus.has_0f && i_bus.opcode == 8'hB5) seg_reg_sel <= 3'd5;  // GS
    end
end

// EIP destinations use only these four sources in the canonical ROM. Keep the
// full microcode source mux off this architectural write path.
function automatic [31:0] eip_source_mux(input [5:0] source);
    case (source)
        SRC_SIGMA:  eip_source_mux = SIGMA;
        SRC_TMPG:   eip_source_mux = TMPG;
        SRC_TMPeIP: eip_source_mux = TMPeIP;
        SRC_OPR_R:  eip_source_mux = OPR_R;
        default:    eip_source_mux = 32'h0;
    endcase
endfunction
wire [31:0] eip_source_value = eip_source_mux(uc_source_shift);

// synthesis translate_off
always @(posedge clk)
    if (reset_n && uc_exec &&
        (uc_dest == DEST_EIP || uc_dest == DEST_eIP || uc_dest == DEST_IP) &&
        (eip_source_value !== alu_result))
        $fatal(1, "EIP SOURCE MUX MISMATCH: uc_addr=%03x src=%02x narrow=%08x alu=%08x",
               uc_addr, uc_source, eip_source_value, alu_result);
// synthesis translate_on

// EIP (Instruction Pointer)
always_ff @(posedge clk) begin
    if (!reset_n) begin
        EIP <= 32'h0000FFF0;  // 386 reset vector offset
    end else if (branch_ustep_redirect) begin
        EIP <= ea_reg;
    end else if (i_issue && !halted /*&& (~uc_active || i_rni_delay)*/) begin
        // z486: a chained i_issue can land on a control transfer's fina -- doc/z486/old/core_notes_v51.md #25
        if (uc_exec && recipe_rni && (uc_dest == DEST_eIP)) begin
            automatic logic [31:0] tgt = is_dword
                                       ? eip_source_value
                                       : {16'h0, eip_source_value[15:0]};
            if (D)
                EIP <= tgt + {27'b0, i_bus.length};
            else
                EIP <= {16'h0, tgt[15:0] + {11'b0, i_bus.length}};
        end else if (D)
            EIP <= EIP + {27'b0, i_bus.length};
        else
            EIP <= {16'h0, EIP[15:0] + {11'b0, i_bus.length}};
    end else if (uc_exec && (uc_dest == DEST_EIP || uc_dest == DEST_eIP || uc_dest == DEST_IP)) begin
        // Microcode destination write to EIP -- doc/z486/old/core_notes_v51.md #26
        if (uc_dest == DEST_EIP) begin
            if (D)
                EIP <= eip_source_value;
            else
                EIP <= {16'h0, eip_source_value[15:0]};
        end else if (uc_dest == DEST_eIP) begin
            if (is_dword)
                EIP <= eip_source_value;
            else
                EIP <= {16'h0, eip_source_value[15:0]};
        end else begin
            // DEST_IP: always 16-bit
            EIP <= {16'h0, eip_source_value[15:0]};
        end
    end
end

// op_size (Operand Size) and srcreg_size
// srcreg_size differs from op_size for MOVZX/MOVSX (source smaller than dest)
always_ff @(posedge clk) begin
    if (!reset_n) begin
        op_size <= 2'd1;  // Default to word size (16-bit real mode)
        srcreg_size <= 2'd1;
        op_size_src <= 2'd1;
        srcreg_size_src <= 2'd1;
    end else if (i_issue && !halted) begin
        // Instruction start: set op_size from decoded instruction
        automatic logic init_is_setcc = i_bus.has_0f && (i_bus.opcode[7:4] == 4'b1001);  // 0F 90-9F
        automatic logic init_is_movzx_movsx = i_bus.has_0f && (i_bus.opcode[7:4] == 4'b1011) && (i_bus.opcode[2:1] == 2'b11);  // 0F B6/B7/BE/BF
        automatic logic init_is_movzx_word = init_is_movzx_movsx && i_bus.opcode[0];  // B7/BF: word source, always dword dest
        automatic logic init_is_xlat = !i_bus.has_0f && (i_bus.opcode == 8'hD7);
        automatic logic init_byte = init_is_setcc ? 1'b1 :
                                    init_is_movzx_movsx ? 1'b0 :
                                    init_is_xlat ? 1'b1 :
                                    (i_bus.has_embedded_register && i_bus.has_w_bit) ? ~i_bus.opcode[3] :
                                    i_bus.has_w_bit ? ~i_bus.opcode[0] : 1'b0;
        // op_size: destination size. B7/BF always dword (ignore 66 prefix)
        automatic logic [1:0] init_op_size = init_byte ? 2'd0 :
                                             init_is_movzx_word ? 2'd2 :
                                             (i_bus.data32 ? 2'd2 : 2'd1);
        automatic logic [1:0] init_srcreg_size = init_is_movzx_movsx ? (i_bus.opcode[0] ? 2'd1 : 2'd0) : init_op_size;
        op_size <= init_op_size;
        op_size_decode <= init_op_size;
        op_size_src <= init_op_size;
        op_size_src_decode <= init_op_size;
        // srcreg_size: for MOVZX/MOVSX, source is byte (B6/BE) or word (B7/BF)
        srcreg_size <= init_srcreg_size;
        srcreg_size_decode <= init_srcreg_size;
        srcreg_size_src <= init_srcreg_size;
        srcreg_size_src_decode <= init_srcreg_size;
    end else if (uc_exec) begin
        // Microcode BITS operations
        case (uc_aluop)
            ALUJMP_BITS8:  begin op_size <= 2'd0; srcreg_size <= 2'd0; op_size_src <= 2'd0; srcreg_size_src <= 2'd0; end
            ALUJMP_BITS16: begin op_size <= 2'd1; srcreg_size <= 2'd1; op_size_src <= 2'd1; srcreg_size_src <= 2'd1; end
            ALUJMP_BITS32: begin op_size <= 2'd2; srcreg_size <= 2'd2; op_size_src <= 2'd2; srcreg_size_src <= 2'd2; end
            ALUJMP_BITSDE: begin
                op_size <= op_size_decode;
                srcreg_size <= srcreg_size_decode;
                op_size_src <= op_size_src_decode;
                srcreg_size_src <= srcreg_size_src_decode;
            end
            default: ;
        endcase
    end
end

// GPR and internal registers
always_ff @(posedge clk) begin
    automatic logic [31:0] external_dest_value;
    automatic logic [15:0] cs_value;
    external_dest_value = dest_value;
    cs_value = cs_source_value;
    if (!reset_n) begin
        CS <= 16'hF000;
        DS <= 16'h0000;
        ES <= 16'h0000;
        SS <= 16'h0000;
        FS <= 16'h0000;
        GS <= 16'h0000;
        LDTR <= 16'h0000;
        TR <= 16'h0000;
        SLCTR <= 32'h0;

        // BOOTUP 9BA-9BB leaves PE/MP/EM/TS/PG clear and sets ET for 80387.
        CR0 <= 32'h0000_0010;
        CR2 <= 32'h0;
        DR6 <= 32'h0;
        DR7 <= 32'h0;

    end else begin
        if (uc_exec) begin
        if (uc_source == SRC_IRF2)
            external_dest_value = IND;  // use combinational IRF2

        // Dead legs removed : the microcode never emits direct named-GPR dest codes
        // other than EAX/EDX/ESP/EBP/eSP/AX/BP/AL/AH (writes go via
        // DSTREG/SRCREG/IRF).  Restore a leg if the ROM ever changes.
        case (uc_dest)
            DEST_TMP_TR: begin
                SLCTR <= external_dest_value; // encoding 0x13 = SLCTR2, same register as SLCTR
            end
            DEST_TMPeIP: TMPeIP <= external_dest_value;
            DEST_TMPeSP: TMPeSP <= external_dest_value;
            DEST_MDTMP,
            DEST_MDTMP4: ;  // Private multiply/divide registers

            DEST_CR0: begin
                CR0 <= external_dest_value;
                // Entering protected mode makes CPL 0 until a later control
                // transfer establishes a different visible CS RPL.
                if (external_dest_value[0] && !CR0[0])
                    CS[1:0] <= 2'b00;
            end
            DEST_CR2: begin
                CR2 <= external_dest_value;
            end

            DEST_DR6: DR6 <= external_dest_value;
            DEST_DR7: DR7 <= external_dest_value;

            // Paging-related destinations (NOP for now)
            DEST_PAGER5: ; // Page cache register - paging-related, NOP

            // Direct segment register destinations (LDS/LES/LFS/LGS/LSS microcode)
            DEST_CS: begin
                // Workaround for call gates: FARJUMP2 (2F0-2F3) skips PASS at 2ED, so SIGMA isn't set from COUNTR
                // At 2F3, if SIGMA=0 but COUNTR!=0, use COUNTR (set by gate_detect for call gates)
                if (cs_value == 16'h0000 && COUNTR[15:0] != 16'h0000 && uc_addr == 12'h2F3)
                    cs_value = COUNTR[15:0];
                // LOAD_TASK reads the incoming CS at 76F.  That selector
                // establishes CPL directly; TASK_RETURN has already cleared
                // task_saved_flag before reaching this common load sequence.
                if (pe && !vm && uc_addr != 12'h76F)
                    CS[15:2] <= cs_value[15:2];
                else
                    CS <= cs_value;
            end
            DEST_ES: ES <= external_dest_value[15:0];
            DEST_SS: SS <= external_dest_value[15:0];
            DEST_DS: DS <= external_dest_value[15:0];
            DEST_FS: FS <= external_dest_value[15:0];
            DEST_GS: GS <= external_dest_value[15:0];
            DEST_LDTR: LDTR <= external_dest_value[15:0];
            DEST_TR: TR <= external_dest_value[15:0];
            DEST_SLCTR: begin
                SLCTR <= external_dest_value;
            end

            DEST_IRF: begin
                if (COUNTR[5:3] == 3'b100)
                case (COUNTR[5:0])
                    6'h20: if (uc_buscode != BUSOP_SAR && uc_buscode != BUSOP_SLIM) ES <= external_dest_value[15:0];
                    6'h22: if (uc_buscode != BUSOP_SAR && uc_buscode != BUSOP_SLIM) SS <= external_dest_value[15:0];
                    6'h23: if (uc_buscode != BUSOP_SAR && uc_buscode != BUSOP_SLIM) DS <= external_dest_value[15:0];
                    6'h24: if (uc_buscode != BUSOP_SAR && uc_buscode != BUSOP_SLIM) FS <= external_dest_value[15:0];
                    6'h25: if (uc_buscode != BUSOP_SAR && uc_buscode != BUSOP_SLIM) GS <= external_dest_value[15:0];
                    default: ;
                endcase
            end

            DEST_SEGREG: begin
                // Write to actual segment register using pre-decoded seg_reg_sel
                case (seg_reg_sel)
                    3'd0: ES <= external_dest_value[15:0];
                    3'd1: ; // CS - not writable
                    3'd2: SS <= external_dest_value[15:0];
                    3'd3: DS <= external_dest_value[15:0];
                    3'd4: FS <= external_dest_value[15:0];
                    3'd5: GS <= external_dest_value[15:0];
                    default: ;
                endcase
            end

            default: ; // No write
        endcase

        // COPY_STACK_DPL: commit the transition DPL to CS[1:0].
        if (prot_transition.copy_stack_dpl && prot_transition.active)
            CS[1:0] <= prot_transition.copy_dpl;

        // WRITE_RPL: write new CPL into SLCTR[1:0] from loaded CS descriptor's DPL
        if (prot_transition.write_rpl)
            SLCTR[1:0] <= desc_raw_hi[14:13];

        end
    end

    // TMPeIP/TMPeSP: save EIP/ESP at instruction start and fault entry
    // Must be outside uc_exec gate because i_issue fires before uc_active is set
    if (i_issue) begin
        TMPeIP <= EIP;
        // TMPeSP <= forwarded_esp;
    end
    if (i_first)
        TMPeSP <= ESP;  // instruction-start ESP; fault frame (SRC_TMPeSP) is restartable,
                        // so it must use the START ESP even if the instruction already
                        // committed a stack push before faulting (e.g. ENTER's PUSH EBP).

    // Chained-store fault attribution: capture the restart IP at every demand WRITE issue
    if (mem_req_to_paging && mem_write_now && mem_accepted)
        wr_restart_eip <= TMPeIP;
    if (page_fault && pg_fault_code[1])
        TMPeIP <= wr_restart_eip;
    else if (ifetch_page_fault) begin
        // A cross-page instruction can fault before i_issue captures its restart
        // state. The architectural registers still describe that boundary.
        TMPeIP <= EIP;
        TMPeSP <= ESP;
    end
end

//=============================================================================
// Unit 9: Address and integer datapath
//=============================================================================

address_unit address_unit_inst (
    .clk(clk),
    .reset_n(reset_n),
    .instr_issue(i_issue),
    .instr(i_bus),
    .d2_start(d2_start),
    .d2_ea(d2_start_ea_dec),
    .displacement(d2_agu_dec.disp),
    .ea_base(ea_base_ref),
    .ea_index(ea_index_ref),
    .ea_base_value(ea_base_value),
    .ea_index_value(ea_index_value),
    .branch_relative(d2_recipe.br_rel),
    .branch_target_eip(spec_target_eip),
    .forwarded_esp(forwarded_esp),
    .ss_stack32(desc_cache[SEG_SS].D_B),
    .exec(uc_exec),
    .exec_addr32(i.addr32),
    .busop(uc_buscode),
    .source(uc_source),
    .alu_source(uc_alu_src),
    .destination(uc_dest),
    .aluop(uc_aluop),
    .source_value(dest_value),
    .alu_value(alu_src),
    .alu_value_hold(alu_src_r),
    .jcc_active(jcc_active),
    .pe(pe),
    .is_dword(is_dword),
    .descsw_mode(descsw_mode),
    .cs_stack32(desc_cache[SEG_CS].D_B),
    .seg_cmd(seg_cmd),
    .seg_sel(mem_seg_sel),
    .seg_base_pending(seg_base_pending),
    .eff_mask_pending(eff_mask_pending),
    .lar_result(seg_lar_result),
    .llim_result(seg_llim_result),
    .lbas_result(seg_lbas_result),
    .fault_code(latched_pf_code),
    .fault_addr(latched_pf_addr),
    .cr3(CR3),
    .ind(IND),
    .ind_delta(IND_DELTA),
    .ind_linear(ind_linear),
    .ind_linear_valid(ind_linear_valid),
    .ea(ea_reg),
    .issue_ea(ea_early),
    .issue_linear(issue_ind_linear)
);

// Derive control signals from ALU opcode
// INC=11000, DEC=11001, INC2=11100, DEC2=11101: all have op[4:3]==11 && op[1]==0
wire alu_update_carry = !(alu_op5[4:3] == 2'b11 && !alu_op5[1]);
wire in_rpti_routine = (uaddr >= 12'h208) && (uaddr <= 12'h20e);   // TODO: Remove this special case

assign alu_op5 = map_alu_op(uc_aluop_shift);

// IMUL: F6.5, F7.5, 0FAF, 69, 6B; MUL: F6.4 and F7.4.
wire is_signed_mul = i.opcode[7:6] != 2'b11 || i.modrm[3];
wire clear_rf = (i_rni_delay && i.opcode != 8'hCF && i.opcode != 8'h9D) ||
                (recipe_rni && uc_exec);

function automatic [4:0] map_alu_op(input [6:0] uc_op);
begin
    casez (uc_op)
        ALUJMP_ALU:    map_alu_op = alu_grp_op;
        ALUJMP_INCDEC: map_alu_op = i.opcode[7] ? {3'b110, alu_grp_op[1:0]}
                                                 : {4'b1100, alu_grp_op[0]};
        ALUJMP_SHIFT1: map_alu_op = ALU_PASS;
        ALUJMP_CMPTST: map_alu_op = instr_is_cmp ? ALU_CMP : ALU_AND;
        ALUJMP_SZ_EXT: map_alu_op = instr_szext_op;
        ALUJMP_AND:    map_alu_op = ALU_AND;
        ALUJMP_OR:     map_alu_op = ALU_OR;
        ALUJMP_XOR:    map_alu_op = ALU_XOR;
        ALUJMP_SIGN:   map_alu_op = ALU_SIGN;
        ALUJMP_ADD:    map_alu_op = ALU_ADD;
        ALUJMP_ADC:    map_alu_op = ALU_ADC;
        ALUJMP_SUB:    map_alu_op = ALU_SUBT;
        ALUJMP_CMP:    map_alu_op = ALU_CMP;
        ALUJMP_SHIFT,
        ALUJMP_SHIFT2: map_alu_op = ALU_PASS;
        ALUJMP_PASS2:  map_alu_op = ALU_PASS2;
        ALUJMP_AAAAAS: map_alu_op = i.opcode[3] ? ALU_AAS : ALU_AAA;
        ALUJMP_BITS16: map_alu_op = ALU_PASS;
        ALUJMP_DAADAS: map_alu_op = i.opcode[3] ? ALU_DAS : ALU_DAA;
        ALUJMP_PASS,
        ALUJMP_JMP,
        ALUJMP_NOPMOVE: map_alu_op = ALU_PASS;
        ALUJMP_SERECO: begin
            case (instr_bt_sel)
                2'b00: map_alu_op = ALU_PASS;
                2'b01: map_alu_op = ALU_OR;
                2'b10: map_alu_op = ALU_ANDN;
                2'b11: map_alu_op = ALU_XOR;
            endcase
        end
        default: map_alu_op = ALU_PASS;
    endcase
end
endfunction

data_unit data_unit_inst (
    .clk(clk),
    .reset_n(reset_n),
    .exec(uc_exec),
    .shift_exec(uc_exec_shift),
    .instr_start(i_issue),
    .halted(halted),
    .ifetch_page_fault(ifetch_page_fault),
    .interrupt_entry(interrupt_entry),
    .repeat_active(repeat_active),
    .clear_rf(clear_rf),
    .pipeline_advance(!stall),
    .stack_op(i_bus.stack_op),
    .stack_dir(i_bus.stack_dir),
    .stack_data32(i_bus.data32),
    .stack32(desc_cache[SEG_SS].D_B),
    .gate_detect(gate_detect_now),
    .any_fault(any_fault_r),
    .uc_active(uc_active),
    .recipe_rni(recipe_rni),
    .recipe_state(recipe_state),
    .hardwired_off(hardwired_off),
    .recipe_commit_cancel(any_fault),
    .aluop(uc_aluop),
    .alu_operation(alu_op5),
    .shift_aluop(uc_aluop_shift),
    .dest(uc_dest),
    .source_field(uc_source_shift),
    .source_live(uc_source),
    .alu_source(uc_alu_src_shift),
    .alu_source_live(uc_alu_src),
    .fpu_f8(uc_fpu_f8),
    .shift_source_class(uc_shift_source_class),
    .shift2_source(uc_shift2_source),
    .op_size(op_size),
    .srcreg_size(srcreg_size),
    .op_size_src(op_size_src),
    .srcreg_size_src(srcreg_size_src),
    .update_arch_flags(alu_update_flags),
    .update_carry(alu_update_carry),
    .instr(i),
    .next_instr(i_bus),
    .pe(pe),
    .cpl(cpl),
    .is_dword(is_dword),
    .is_signed_mul(is_signed_mul),
    .eip(EIP),
    .cr0(CR0),
    .cr2(CR2),
    .tmpeip(TMPeIP),
    .tmpesp(TMPeSP),
    .dr6(DR6),
    .dr7(DR7),
    .slctr(SLCTR),
    .protun(PROTUN),
    .ind(IND),
    .ea(ea_reg),
    .es(ES),
    .cs(CS),
    .ss(SS),
    .ds(DS),
    .fs(FS),
    .gs(GS),
    .ldtr(LDTR),
    .tr(TR),
    .seg_reg_sel(seg_reg_sel),
    .forwarded_esp(forwarded_esp),
    .desc_raw_hi(desc_raw_hi),
    .opr_r(OPR_R),
    .ea_base(ea_base_ref),
    .ea_index(ea_index_ref),
    .dly_gpr_forward(dly_gpr_forward),
    .sigma(SIGMA),
    .countr(COUNTR),
    .alu_src_hold(alu_src_r),
    .source_value_live(source_value_live),
    .alu_source_value_live(alu_src_data),
    .dest_value(dest_value),
    .alu_src(alu_src),
    .eax(EAX),
    .ecx(ECX),
    .edx(EDX),
    .ebx(EBX),
    .esp(ESP),
    .ebp(EBP),
    .esi(ESI),
    .edi(EDI),
    .tmpc(TMPC),
    .tmpg(TMPG),
    .opr_w(OPR_W),
    .protection_source_value(protun_write_value),
    .cs_source_value(cs_source_value),
    .ea_base_value(ea_base_value),
    .ea_index_value(ea_index_value),
    .eflags(EFLAGS),
    .uc_flags(uc_flags),
    .flags_backup(FLAGSB),
    .flags_backup_active(flags_backup_active),
    .eflags_fwd(eflags_fwd),
    .eflags_ahead(eflags_ahead),
    .recipe_shift_write(recipe_shift_write),
    .recipe_shift_data(recipe_shift_data),
    .recipe_memory_write(recipe_mem_write),
    .alu_result(alu_result),
    .shift_result(shift_result),
    .muldiv_result(muldiv_result),
    .div_overflow(div_overflow)
);

// Debug tap (read by tb_z486 hierarchically; not used in the core).
wire use_shifter_result = (uc_aluop == ALUJMP_SHIFT2) || (uc_aluop == ALUJMP_SHIFT);


//=============================================================================
// Unit 10: x87 Coprocessor
//=============================================================================

x87_unit #(.ENABLE_X87(ENABLE_X87)) x87 (
    .clk(clk),
    .reset_n(reset_n),
    .req_valid(x87_req_selected),
    .req_data_port(dcache_req_phys_addr_raw[2]),
    .req_write(dcache_req_write),
    .req_be(dcache_req_be),
    .req_wdata(dcache_req_wdata),
    .req_accepted(x87_req_accepted),
    .req_complete(x87_req_complete),
    .req_read_complete(x87_read_complete),
    .req_rdata(x87_rdata),
    .direct_launch(i_issue),
    .direct_candidate(x87_direct_candidate),
    .direct_allowed(!CR0[3] && !CR0[2]),
    .direct_fop(i.immediate[10:0]),
    .direct_active(x87_direct_active),
    .direct_mem_req(x87_direct_mem_req),
    .direct_stall(stall_x87_direct),
    .mem_accepted(mem_accepted),
    .mem_addr_low(ind_linear[1:0]),
    .mem_read_complete(mem_read_complete),
    .mem_servicing(mem_servicing),
    .mem_rdata(dcache_rdata),
    .split_rdata(OPR_R),
    .cancel(gp_fault_trigger || page_fault || interrupt_entry || q_flush),
    .busy_n(x87_busy_n),
    .pereq(x87_pereq),
    .error_n(x87_error_n),
    .debug_state(dbg_x87_state)
);


//=============================================================================
// Miscellaneous control modules
//=============================================================================

wire nmi_accept_boundary = i_rni_delay && !stall && !page_fault &&
                           nmi_request_active && !single_step;

function automatic logic instr_mov_or_pop_ss(input dec_entry_t entry);
    instr_mov_or_pop_ss = (entry.opcode == 8'h17) ||
        ((entry.opcode == 8'h8E) && (entry.modrm[5:3] == 3'd2));
endfunction

interrupt_controller interrupts (
    .clk(clk),
    .reset_n(reset_n),
    .intr(intr),
    .nmi(nmi),
    .iflag(EFLAGS[9]),
    .i_rni(i_rni),
    .shadow_start(i_rni && ((i.opcode == 8'hFB) || instr_mov_or_pop_ss(i))),
    .uc_exec(uc_exec),
    .uc_aluop(uc_aluop),
    .nmi_accept_boundary(nmi_accept_boundary),
    .intr_pending(intr_pending),
    .nmi_request_active(nmi_request_active),
    .interrupt_pending(interrupt_pending),
    .inhibit_interrupts(inhibit_interrupts)
);

wire throttle_active_cycle = (i_issue && !throttle_parked_r) ||
                             (uc_active && !stall && !d2_release_hold &&
                              !throttle_parked_r);

cpu_throttle #(.CLOCK_RATE_MHZ(CLOCK_RATE_MHZ)) throttle (
    .clk(clk),
    .reset_n(reset_n),
    .speed_sel(cpu_speed_sel),
    .active_cycle(throttle_active_cycle),
    .hold(throttle_hold),
    .release_cycle(throttle_release_ready),
    .full_speed(throttle_full)
);


endmodule
