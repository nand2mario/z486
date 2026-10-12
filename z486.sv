//
// z486
// An x86 CPU core with the original 80386 microcode and 486-style pipelining
//

`include "z486_platform.svh"
module z486
    import z486_pkg::*;
#(
    parameter PROTECT_UMA_ROM = 0,
    parameter DCACHE_SET_BITS = 7,   // dcache size: 7 = 8KB, 8 = 16KB
    // 1: D-cache fills ask for the missing dword first; memory returns the
    // line's dwords in wrapping order from the requested one.
    parameter DCACHE_LINE_WRAP = 0,
    parameter ICACHE_SET_BITS = 7,   // icache size: 7 = 8KB, 8 = 16KB
    parameter ENABLE_X87 = 0,
    parameter ENABLE_DEVICE_MMIO = 0,
    parameter [31:0] DEVICE_MMIO_MASK = 32'hff00_0000,
    parameter [6:0] CLOCK_RATE_MHZ = 7'd85
)
(
    input              clk,
    input              reset_n,
    input              device_mmio_enable,
    input      [31:0]  device_mmio_base,

    // 32-bit bus interface (ready/valid handshake)
    output     [31:2]  addr,        // Physical address [31:2]
    output      [3:0]  be,          // Byte enables
    output      [7:0]  burstcount,  // Burst length in DWORDs
    output             line_read,   // Request is one complete cache line
    input      [31:0]  din,         // Data input
    input      [127:0] line_din,    // Complete aligned cache-line response
    output     [31:0]  dout,        // Data output
    output             valid,       // Request valid (held until ready)
    input              ready,       // Handshake: transfer on valid && ready
    output             write,       // 1=write, 0=read (stable while valid)
    output             io,          // I/O vs memory (1=I/O, 0=memory)
    input              resp_valid,  // Read data valid (1-cycle pulse)
    input              line_resp_valid, // line_din valid (1-cycle pulse)

    // Interrupts
    input              intr,        // Maskable interrupt request
    input              nmi,         // Non-maskable interrupt
    output             inta,        // Interrupt acknowledge
    output             x87_ferr,    // FERR#: an unmasked x87 error is pending (AT: IRQ13)
    input              x87_ignne,   // IGNNE#: proceed past it with CR0.NE=0 (AT: port F0h)

    // External memory writers can invalidate matching L1 lines.
    input      [31:0]  snoop_addr,
    input              snoop_valid,

    input              a20_enable,  // A20 gate input

    // Architectural execution rate: 0=full, 1=15, 2=30, 3=56 MHz.
    input       [1:0]  cpu_speed_sel,
    // Dev menu: run every instruction through the 386 microcode.
    input              fast_off_req,
    // Dev menu: L1 caches off (every access goes to memory).
    input              cache_off_req,
    // Dev menu: no coprocessor, as a build without the x87 (taken at reset).
    input              x87_off_req,

    // Debug/test control
    input              single_step, // Halt after each instruction (for single-step tests)

    output     [15:0]  dbg_CS,
    output     [31:0]  dbg_EIP,
    output     [31:0]  dbg_CS_base,
    output             dbg_pe,
    output             dbg_vm,
    output     [31:0]  dbg_x87_state,

    // A fault while delivering #DF shuts down the 386 and requests reset.
    output triple_fault_reset
);

//=============================================================================
// CPU state and cross-unit interconnect
//=============================================================================

// Architectural state and externally visible debug state.
wire dbg_first_done;  // Debug: first instruction finished execution
wire halted;  // Tracks when the core is halted
wire [31:0] debug_ip;  // Debug: IP at instruction completion

reg [31:0] CR0, CR2, CR3;
reg [31:0] DR6, DR7;
// DR0-3 live in the 386's descriptor cache as base fields: SELECT_DR_TR puts
// 0x70+n in COUNTR, MOV DRn,r stores through IRF with SBAS (38E) and MOV r,DRn
// reads them back with LBAS (3B2). TR6/TR7 (0x76/0x77) are not kept.
reg [31:0] DR03 [0:3];
wire [31:0] EAX, ECX, EDX, EBX, ESP, EBP, ESI, EDI;
reg [31:0] EIP = 32'h0000FFF0;      // Architectural IP (next instruction) - reset vector

// Bits: 31..22 21 20 19 18 17 16 15..14 13..12 11 10 9  8  7  6  5  4  3  2  1  0
//       Rsvd   ID VIP VIF AC VM RF Rsvd  IOPL   OF DF IF TF SF ZF 0  AF 0  PF 1  CF
wire [31:0] EFLAGS;
wire [31:0] uc_flags;               // Internal ALU flags for microcode conditionals
wire [31:0] eflags_fwd;             // Includes pending ALU/shifter retirement
wire        branch_condition_true;

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
`Z486_KEEP reg [1:0] op_size_src;            // Local copy for generic source mux fanout
// Further identical copies split op_size_src's fanout (several hundred): the
// data unit's operand formatting and the dword decode. Preserved registers are
// not duplicated by the fitter, so the copies are explicit.
`Z486_KEEP reg [1:0] op_size_du;             // data unit operand size
`Z486_KEEP reg [1:0] op_size_dw;             // is_dword
`Z486_KEEP reg [1:0] op_size_src_decode;
`Z486_KEEP reg [1:0] srcreg_size_src;
`Z486_KEEP reg [1:0] srcreg_size_src_decode;
// Width consumers read the timing-local replica of op_size.
wire       is_dword = (op_size_dw == 2'd2);

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
wire [31:0] memory_write_source_value;
wire [31:0] alu_src_data;
wire [31:0] protun_write_value;
wire [15:0] cs_source_value;
wire        uc_exec;
wire        prot_is_ptovrr;
wire [11:0] uc_addr;
wire [6:0]  uc_dest;
wire [5:0]  uc_source;
wire [31:0] dest_value;
wire        uc_pref_suppress_prev;
wire        stall;
wire        decq_empty;
wire        uc_busreq;
wire        mem_req_current;
wire        uc_is_wio;
wire        uc_is_rpt;
wire        cr3_write;
wire [1:0]  pg_cpl;
wire        seg_gp_fault;
wire [31:0] issue_ind_linear;
wire [1:0]  issue_ind_linear_low;
wire [31:0] issue_mem_linear;          // issuing instruction's memory address (stack, moffs, ModR/M)
wire [31:0] forwarded_esp;             // ESP including a pending stack update
wire [7:0]  pend_write_mask;           // GPRs a late producer writes on this edge
// Data-access pipeline (data_access.sv) connections
// L1 data cache: probe/resolve port and direct store port
wire         dcache_wr_ready;
wire         fast_store_accepted;
wire [3:0]   fast_store_be;
wire         fast_store_valid;
wire [31:0]  fast_store_wdata;
wire [31:0]  rmw_fast_phys_r;
wire [31:0]  st_phys;
wire         st_route;
wire         st_take;
// Paging unit and sidecar TLB
wire         paging_demand_idle;
wire         sidecar_bg_pre;
wire         st_tlb_pre;
wire [1:0]   ucrd_cpl_r;
wire         ucrd_hit;
wire [31:0]  ucrd_linear_r;
wire         ucrd_phys_ok_r;
wire [31:0]  ucrd_phys_r;
wire         ucrd_route_pre;
wire [1:0]   ucrd_size_r;
wire         ucrd_slow_req_r;
wire         ucrd_slow_submit;
wire         ucrd_slow_wait_r;
wire         ucrd_take;
wire         ucrd_x87_r;
hardwired_load_payload_t vipt_load_slow_r;
wire [31:0]  vipt_probe_linear;
wire         vipt_slow_addr_owned;
wire         vipt_slow_phys_ok_r;
wire [31:0]  vipt_slow_phys_r;
wire         vipt_slow_submit;
// Data unit: load writeback and operands
wire         direct_wb_retire;
wire         fast_opr_commit;
wire [31:0]  fast_opr_data;
wire         vipt_load_alu_dst_capture;
wire [31:0]  vipt_load_alu_dst_capture_data;
wire [2:0]   vipt_load_alu_dst_capture_dst;
wire [1:0]   vipt_load_alu_dst_capture_size;
wire [4:0]   vipt_load_wb_alu_op_r;
wire [31:0]  vipt_load_wb_data;
wire [7:0]   vipt_load_wb_dst_onehot_r;
wire [7:0]   vipt_load_wb_fwd_mask_r;
wire [2:0]   vipt_load_wb_dst_r;
wire         vipt_load_wb_is_alu_r;
wire [1:0]   vipt_load_wb_size_r;
wire [31:0]  vipt_load_wb_target_r;
wire         vipt_load_wb_valid_r;
// D2 instruction and issue
wire         d2_vipt_ea_hazard;
wire         d2_plain_load_overlap_ready;
wire         d2_vipt_candidate;
wire         d2_vipt_load;
wire         d2_vipt_pipe_ready;
wire         d2_vipt_pop;
wire         d2_vipt_ret;
wire         d2_vipt_rmw;
wire         d2_vipt_rmw_candidate;
wire         vipt_issue_load;
// Microsequencer and core control
wire         pop_direct_r;
wire         rd_fast_finish;
wire         rd_fast_valid_r;
wire         ret_redirect;
wire         rmw_fallback_delay_r;
wire         rmw_fast_active_r;
wire         stall_fast_store;
wire         stall_rmw_probe;
wire         stall_ucrd;
wire         vipt_load_ex_probed_r;
hardwired_load_token_t vipt_load_ex_r;
wire         vipt_load_exec_block;
hardwired_load_token_t vipt_load_replay_r;
wire         vipt_load_slow_busy;
wire         vipt_load_slow_wait_r;
wire        dly_gpr_we;
wire        eff_mask_pending;
reg  [3:0]  seg_cmd;
wire gate_detect_cond;
wire [31:0] br_target;
reg         early_redirected;

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
// Flat segments (page-granular, 4 GB limit, not expand-down): no contained
// access can violate their limit, so direct loads skip the limit check.
wire ds_flat   = desc_cache[SEG_DS].G && (&desc_cache[SEG_DS].limit) &&
                 (desc_cache[SEG_DS].seg_type[3] || !desc_cache[SEG_DS].seg_type[2]);
wire ss_flat32 = desc_cache[SEG_SS].G && desc_cache[SEG_SS].D_B && (&desc_cache[SEG_SS].limit) &&
                 (desc_cache[SEG_SS].seg_type[3] || !desc_cache[SEG_SS].seg_type[2]);
// Segments a direct load may read without the microcode's segment check:
// data, or readable code (a null selector loads type 0 with S=0, which an
// access must #GP). Indexed as desc_cache: ES CS SS DS FS GS.
function automatic logic desc_readable(input seg_desc_t d);
    desc_readable = d.S && (!d.seg_type[3] || d.seg_type[1]);
endfunction
wire [5:0] seg_readable = {desc_readable(desc_cache[SEG_GS]), desc_readable(desc_cache[SEG_FS]),
                           desc_readable(desc_cache[SEG_DS]), desc_readable(desc_cache[SEG_SS]),
                           desc_readable(desc_cache[SEG_CS]), desc_readable(desc_cache[SEG_ES])};

wire       pe = CR0[0];             // Protected mode enable
wire       vm = EFLAGS[17];         // Virtual 8086 mode
logic      decoder_default32_r;     // D1-local mode replicas
logic      decoder_native_pe_r;

always_ff @(posedge clk) begin
    if (!reset_n) begin
        decoder_default32_r <= 1'b0;
        decoder_native_pe_r <= 1'b0;
    end else begin
        decoder_default32_r <= D;
        decoder_native_pe_r <= pe && !vm;
    end
end

assign dbg_CS  = CS;
assign dbg_EIP = EIP;
assign dbg_CS_base = CS_base;
assign dbg_pe  = pe;
assign dbg_vm  = vm;
wire [1:0] cpl = vm ? 2'd3 : !pe ? 2'd0 : CS[1:0];

wire [2:0] latched_pf_code;  // Latched page fault error code (for LPCR microcode access)
wire [31:0] latched_pf_addr;  // Latched faulting linear address (for LPCR microcode access)

// Frontend interconnect.
wire [63:0] k1q;                 // registered raw window at the prefetcher's D1 cursor
wire [63:0] k1q_early;           // speculative next D1 window for entry ROM
wire [5:0]  k1q_avail;               // bytes fetched beyond the D1 cursor
wire [3:0]  k1p_adv;                 // D1 cursor advance this cycle (a prefix, or
                                    //   the instruction rest at handoff, 0-11)
wire [3:0]  k1p_preread_adv;         // structural advance without D2 backpressure
wire [31:0] k2q;                // D2 literal window at pop_cursor + k2p_off
wire [5:0]  k2q_avail;              // bytes fetched beyond that point
wire [4:0]  dec_lit_off;            // literal offset from the pop cursor
wire        dec_pop_now;            // one instruction completed D2: pop its bytes
wire [4:0]  dec_pop_len;            //   (registered length from the skeleton)
wire       pf_full;
wire       q_flush;                 // Flush queue (branch/jump) - combinational for i.immediate gating
wire       pe_mode_toggle_now;      // CR0.PE changed this cycle: re-decode next bytes in new mode
wire       uc_ctl_pref;             // Previous-cycle predecode: current uop is BUSOP_PREF
wire       early_redirect;

// CR0.PE bit of the value being written.
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
wire        data_page_fault;
wire [1:0]  prot_cpl;               // CPL for protection unit (declared fully near protection logic)

// Paging and memory interconnect.
wire        mem_servicing;          // memory request in flight
wire        mem_dly_grace;          // optimistic read: DLY may execute this (lookup) cycle
wire        mem_write_dly_grace;    // posted write in PG_MEM_TLB: next non-bus uop may execute now
wire        mem_write_wait;         // unposted demand write still fault-capable: stall all uops
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
wire [11:0] dcache_req_preread_offset;
wire        dcache_req_preread_priority;
wire        dcache_req_write;
wire [3:0]  dcache_req_be;
wire [31:0] dcache_req_wdata;
wire [31:0] dcache_direct_wdata;
wire [31:0] x87_req_wdata;
wire        dcache_req_is_io;
wire        dcache_req_is_inta;
wire        dcache_req_is_x87;
wire        dcache_req_is_vga_mem;
wire        dcache_req_accepted;
wire        dcache_req_complete;
wire        dcache_read_complete;
wire [31:0] dcache_rdata;
wire        dcache_vipt_probe_valid;
wire [11:0] dcache_vipt_probe_offset;
wire        dcache_vipt_probe_ready;
wire        dcache_vipt_probe_accepted;
wire        dcache_vipt_probe_direct_accepted;
wire        dcache_vipt_resolve_valid;
wire [31:0] dcache_vipt_resolve_phys_addr;
wire        dcache_vipt_resolve_hit;
wire [31:0] dcache_vipt_resolve_data;
wire        dcache_vipt_resolve_fill;   // a direct-load miss may fill at once
wire        dcache_vipt_fill_start;
wire        dcache_vipt_fill_resp_valid;
wire [31:0] dcache_vipt_fill_resp_data;
wire        vipt_tlb_hit;
wire [31:0] vipt_tlb_phys_addr;
wire        vipt_tlb_writable;
wire        vipt_tlb_user;
wire        vipt_tlb_dirty;
wire        vipt_tlb_is_vga_mem;
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
wire        x87_direct_taken;          // the issued x87 overlay went direct
wire        x87_direct_reg_taken;      // ...as a register form
wire        x87_store_opr_commit;      // the x87 m32 store result arrived for OPR_R
wire [31:0] x87_store_opr_data;
wire        x87_store_hold;            // the store word waits for the x87 result
wire        x87_direct_mem_req;

// Microcode and macro-instruction lifecycle interconnect.
// After a jump, the next micro-op still executes (delay slot) before jump takes effect.
wire [11:0] uaddr_next;             // Next address, launched early to the ucode ROM
wire [11:0] uaddr;                  // Address being fetched in the current ucode pipeline
wire [11:0] uc_addr_mem_r;          // Address aligned with the ROM q_mem stage
wire [50:0] uc;                     // Current microcode word + pre-computed bits (50:37)
wire [50:0] uc_next;
wire [5:0]  uc_buscode;             // Bus operation code from microcode
wire [5:0]  uc_alu_src;             // ALU source / micro-jump offset field
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
wire       i_issue;                 // D2 transfers one instruction into EX
reg        i_first;                 // First ucode execution cycle after issue
wire       i_rni_raw;               // Raw RNI decode from the resident ROM word
wire       i_rni = i_rni_raw && !rmw_fallback_delay_r;
wire       i_rni_delay;             // RNI delay slot - RNI has been executed. this is last instruction cycle
wire       i_rni_delay_ea;          // Physically local copy for EA forwarding

// Interrupt-controller outputs used by D2 admission and execution boundaries.
wire       intr_pending;
wire       nmi_request_active;
wire       interrupt_pending;
wire       inhibit_interrupts;
wire tf_active_r;
wire tf_trap_suppress_r;

// Fault requests cross the address, data, D2, and sequencer boundaries.
wire       any_fault_issue;
wire       any_fault;
reg        any_fault_r;

wire       d2_ready;                // D2 may transfer its instruction to EX
wire       d2_push;                 // Decoder completed the resident D2 payload
reg        d2_ea_split_done_r;      // D2a captured base + scaled index
wire       d2_ea_split_wait;        // Three-term EA needs its D2a cycle
wire       d2_ea_split_conflict;    // Its base/index is written on this edge
reg        throttle_parked_r;       // predecessor retired; successor D2 waits for rate debt
wire [11:0] issue_entry;            // Entry of the instruction issuing now
wire        pb_load;                // Skeleton loads: ROM port B reads its entry
wire [11:0] pb_load_entry;
wire        pb_valid;               // Port B holds the skeleton's first word
wire        pb_issue;               // Skeleton may issue into a dead RNI slot
wire        pb_slot;                // Port B supplies the next executing word
wire uc_active;  // Tracks when instruction execution has begun
wire fault_suppress_delay_slot;  // Fault handling: suppress delay slot after fault triggers
wire interrupt_entry;  // Interrupt handler is being entered
reg        stack_init_pending;      // Cycle after i_issue for a stack operation
wire       prot_test_inflight;      // Protection test is waiting for a result
wire [31:0] COUNTR;                 // Data Unit counter register
// IRF index 0x70-0x77 (SELECT_DR_TR): DR0-3/TR6-7, never a GPR.
wire        irf_is_dr = COUNTR[6:3] == 4'b1110;

// Hardwired common-instruction control: recipes of one to three uSteps.
recipe_state_t recipe_state;               // Current hardwired recipe state
recipe_pending_write_t recipe_mem_write;   // Deferred load commit
recipe_pending_write_t recipe_shift_write; // Deferred shift commit
wire [31:0] recipe_shift_data;              // Deferred shift result
wire       recipe_slot_stale;               // Reclaimed RNI slot is stale
// Fast-path switch (Dev menu). Off runs every instruction through the 386
// microcode: each fast path checks hardwired_off at one eligibility point. A
// new request takes effect only on a pipeline flush, which discards D2's
// decisions, so no instruction sees both settings.
logic      fast_off_sim = 1'b0;           // simulation plusargs below
// Caches off takes effect per request: a disabled L1 still patches store hits,
// so it stays coherent and can come back on at any time.
logic      cache_off_sim = 1'b0;
// The x87 switch is taken at reset only: removing the FPU under a running
// program would only lose its state. A build without the FPU behaves as the
// switch does (a 486SX: ESC instructions do nothing, FNSTSW stores nothing).
logic      x87_off_sim = 1'b0;
reg        x87_off;
assign x87_ferr = !x87_off && !x87_error_n;
always_ff @(posedge clk)
    if (!reset_n)
        x87_off <= x87_off_req || x87_off_sim || !ENABLE_X87;
reg        hardwired_off;
always_ff @(posedge clk)
    if (!reset_n || q_flush)
        hardwired_off <= fast_off_req || fast_off_sim;

// synthesis translate_off
// +z486_fast_off starts with the fast paths off; +z486_fast_toggle=N flips the
// request at random, about once every N cycles. +z486_cache_off and
// +z486_cache_toggle=N do the same for the L1 caches.
integer fast_toggle_period = 0;
integer cache_toggle_period = 0;
initial begin
    if ($test$plusargs("z486_hardwired_off") || $test$plusargs("z486_fast_off"))
        fast_off_sim = 1'b1;
    if ($test$plusargs("z486_cache_off"))
        cache_off_sim = 1'b1;
    if ($test$plusargs("z486_x87_off"))
        x87_off_sim = 1'b1;
    void'($value$plusargs("z486_fast_toggle=%d", fast_toggle_period));
    void'($value$plusargs("z486_cache_toggle=%d", cache_toggle_period));
end
always @(posedge clk) begin
    if (fast_toggle_period > 0 && ($urandom % fast_toggle_period) == 0)
        fast_off_sim <= !fast_off_sim;
    if (cache_toggle_period > 0 && ($urandom % cache_toggle_period) == 0)
        cache_off_sim <= !cache_off_sim;
end
// synthesis translate_on
wire       recipe_rni;                       // Current uStep contains RNI
wire       branch_ustep_redirect;     // bounded branch uStep redirects frontend
wire       branch_ustep_rni;          // synthetic RNI, independent of execution stall
wire       branch_ustep_exec;         // execute bounded branch uStep this cycle
recipe_meta_t d2_recipe;             // D2 instruction's generated recipe class
wire       x87_direct_candidate;              // D2 entry is the direct x87 overlay
wire       flags_backup_active;     // Set at i_issue/FLGSBA, cleared on interrupt_entry - guards FLAGSB writes
wire       d2_hardwired;             // D2 entry is a bounded hardwired recipe
wire       jcc_fold_active;            // not-taken Jcc occupies reclaimed slot
// These nets feed module ports before their generating logic appears below.
// Declare them here so XSim cannot create disconnected implicit one-bit nets.
wire [31:0] spec_target_lin;
wire        pf_spec_req;
reg         pf_spec_owner_r;
wire        pf_spec_store;
wire [31:0] pf_spec_store_linear;
wire        pf_spec_global_kill;
wire        prot_redirect_prev;
wire        prot_redirect_taken;

// Empty D2 may launch directly from D1. Otherwise the resident D2 skeleton
// supplies a registered entry address when the current instruction ends.
wire       d1_issue_direct;
dec_entry_t d1_issue_entry;
// Every D2 instruction issues its first word from ROM port B.
wire       pb_issue_b23 = pb_valid && (i_rni_delay || ~uc_active) &&
                          ~halted && !q_flush && !fault_suppress_delay_slot &&
                          !interrupt_entry && !any_fault_issue;

// D2 -> EX readiness.
wire       tf_trap_pending = tf_active_r && !tf_trap_suppress_r;
wire       interrupt_deliverable = tf_trap_pending || nmi_request_active ||
                                   (intr_pending && EFLAGS[9] && !inhibit_interrupts);
wire       interrupt_at_boundary = i_rni_delay && interrupt_deliverable && !single_step;

// Fixed-clock CPU throttle.
wire        throttle_hold;
wire        throttle_release_ready;
wire        throttle_full;
// The throttle never splits a load/POP commit or a PUSH from its delay slot.
wire        throttle_atomic_chain = recipe_state.hardwired && uc_active && i_rni &&
    ((recipe_state.commit_sel == RECIPE_COMMIT_MEM) ||
     ((recipe_state.commit_sel == RECIPE_COMMIT_ESP) && recipe_state.slot_has_work));
// D2 launch overlaps the predecessor's last cycle or the final repayment cycle.
wire        throttle_release_cycle = throttle_parked_r && throttle_release_ready;

// An instruction occupies D2 exactly when ROM port B holds its first word.
wire       d2_resident = pb_valid;
// A trap, NMI or interrupt taken at this boundary cancels the D2 issue.
wire       boundary_take = i_rni_delay && !stall && !page_fault &&
    ((tf_trap_pending && !single_step) ||
     (nmi_request_active && !single_step) ||
     (intr_pending && EFLAGS[9] && !single_step && !inhibit_interrupts));
wire       d2_payload_ready = d2_push && !d2_ea_split_wait;
wire       d2_ready_base = d2_payload_ready && !stall &&
                      (!throttle_hold || throttle_atomic_chain ||
                       throttle_release_cycle) && !any_fault_issue &&
                      !(i_rni && tf_trap_pending && !single_step) &&
                      !interrupt_at_boundary && !q_flush &&
                      !d2_vipt_ea_hazard;
assign     d2_ready = d2_ready_base &&
                      !vipt_load_replay_r.valid && !vipt_load_slow_busy &&
                      (!d2_vipt_candidate || d2_vipt_load) &&
                      (!d2_vipt_rmw_candidate || d2_vipt_rmw) &&
                      (!vipt_load_ex_r.valid || d2_vipt_pipe_ready ||
                       d2_plain_load_overlap_ready);
// New VIPT loads: occupancy check on registered older-token state only.
wire       d2_vipt_issue_ready = d2_ready_base &&
                                  (!vipt_load_ex_r.valid ||
                                   vipt_load_ex_probed_r);
wire       i_issue_reference = (pb_issue_b23 || pb_issue) && d2_ready;
assign     i_issue = (pb_issue_b23 || pb_issue) &&
                     (d2_vipt_load ? d2_vipt_issue_ready : d2_ready);

// synthesis translate_off
always_ff @(posedge clk) begin
    if (reset_n && (i_issue !== i_issue_reference))
        $fatal(1, "VIPT ISSUE READY MISMATCH: direct=%b generic=%b",
               i_issue, i_issue_reference);
end
// synthesis translate_on

// A data page fault on an older posted write redirects this cycle: the word
// waiting behind that write (a successor's) must not commit anything.
wire       core_live = !halted && uc_active && !fault_suppress_delay_slot && !interrupt_entry &&
                       !data_page_fault;
wire       dly_grace_now = mem_dly_grace && uc_p_pure_dly;
wire       posted_write_release = mem_write_dly_grace && !uc_busreq;    // release non-busop writes after one cycle
// A word whose bus operation the direct load suppresses (its ROM word while
// the token owns the access) does not wait for paging.
wire       uc_bus_or_dly_live = uc_bus_or_dly && !vipt_load_exec_block;
wire       mem_block_busy = (uc_bus_or_dly_live && !dly_grace_now && !posted_write_release) ||
                            mem_opt_wait || mem_write_wait; // demand op in flight
wire       mem_block_idle = (uc_busreq && !vipt_load_exec_block && !mem_accepted);  // uop wants the bus, paging not ready
wire       stall_mem = (mem_servicing ? mem_block_busy : (mem_req_current && !mem_accepted)) ||
                       stall_ucrd;
wire       stall_wio = uc_active && uc_is_wio &&
                       !interrupt_pending && !single_step;
wire       stall_x87_direct;
wire       stall_invlpg;
assign stall = stall_mem || stall_wio || stall_x87_direct ||
               stall_invlpg || stall_fast_store || stall_rmw_probe;

// Repeat
wire       prot_result_now;
wire       repeat_active = uc_is_rpt && (COUNTR[4:0] != 0 || prot_test_inflight) && !prot_result_now
                           && !(uc_is_wio && interrupt_pending);

// uc_exec: master enable for microcode execution
// uc_slot_live: every hold except a direct load's own ROM words.
wire       uc_slot_live = core_live && !(mem_servicing ? mem_block_busy : mem_block_idle) &&
                 !stall_ucrd &&
                 !stall_wio && !stall_x87_direct && !stall_invlpg &&
                 !stall_fast_store && !stall_rmw_probe &&
                 !throttle_parked_r && !recipe_slot_stale && !rmw_fallback_delay_r;
// A direct POP's first ROM word executes for its ESP write while the data
// pipeline completes the load (the word's read is suppressed).
assign uc_exec = uc_slot_live && (!vipt_load_exec_block || (pop_direct_r && i_first));
wire       uc_exec_writeback = uc_exec;  // local copies for reducing fanout
wire       uc_exec_shift = uc_exec;

assign     seg_cmd_valid = i_issue || uc_exec;

dec_entry_t i_bus;            // Instruction resident in unified D2
// A direct load's address and byte lane: the ModR/M lane comes from the fast
// low-bit adders; a moffs lane from the relocated immediate.
wire [31:0] issue_load_linear = issue_mem_linear;
wire [1:0]  issue_load_low = (i_bus.has_moffs || i_bus.stack_op) ? issue_mem_linear[1:0]
                                                                : issue_ind_linear_low;
wire       decq_has_jmp_call; // D1/D2 holds a JMP/CALL rel (halt speculative prefetch)
dec_entry_t pb_next_instr;    // Instruction entering the D2 skeleton from D1
ea_dec_t    skel_load_ea_dec; // Its EA decode, latched by the Address Unit
dec_entry_t i;                // Current instruction (latched at i_issue; written far below)

// Hold a successor whose base/index (or ESP) is a pending direct-load destination.
wire [7:0] d2_ea_read_mask = i_bus.ea_base_onehot |
                             i_bus.ea_index_onehot |
                             (i_bus.stack_op ? 8'h10 : 8'h00);
// Plain-load WB data forwards into the D2 EA reader; ALU-load results do not.
wire [7:0] vipt_load_wb_alu_dst_mask =
    (vipt_load_wb_valid_r && vipt_load_wb_is_alu_r)
        ? vipt_load_wb_dst_onehot_r : 8'h00;
wire [7:0] vipt_pending_dst_mask =
    (vipt_load_ex_r.valid     ? vipt_load_ex_r.dst_onehot     : 8'h00) |
    (vipt_load_replay_r.valid ? vipt_load_replay_r.dst_onehot : 8'h00) |
    (vipt_load_slow_busy      ? vipt_load_slow_r.dst_onehot   : 8'h00) |
    vipt_load_wb_alu_dst_mask;
assign d2_vipt_ea_hazard = |(d2_ea_read_mask & vipt_pending_dst_mask);

// A dead slot takes port B's word only when the instruction can issue now.
wire       pb_load_ready = !x87_direct_candidate && !d2_vipt_ea_hazard &&
    (!vipt_load_ex_r.valid || d2_vipt_candidate ||
     (vipt_load_ex_probed_r && !vipt_load_ex_r.is_alu)) &&
    (!d2_vipt_rmw_candidate ||
     (dcache_vipt_probe_ready && !vipt_load_replay_r.valid && !vipt_load_slow_busy &&
      !rmw_fast_active_r && !mem_servicing)) &&
    ((i_bus.recipe_early != RECIPE_EARLY_LOAD &&
      !d2_vipt_candidate) ||
     ((d2_vipt_candidate ? (d2_vipt_load &&
                            (!vipt_load_ex_r.valid || vipt_load_ex_probed_r))
                         : !vipt_load_ex_r.valid) &&
      !vipt_load_replay_r.valid && !vipt_load_slow_busy));

dec_entry_t d2_entry;         // entry completing D2 this cycle (AGU/i_entry source)
ea_dec_t    d2_agu_dec;       // EA decode for d2_entry


// A fault or an event taken at the boundary holds the ROM output register.
wire        rom_q_hold = interrupt_at_boundary || any_fault || any_fault_issue;
// A replayed RMW preread, or an RMW store in its delay slot, holds the entry word.
wire        microcode_rom_base_ce = !stall_mem && !stall_wio && !repeat_active &&
                                    !stall_rmw_probe && !x87_store_hold &&
                                    !stall_fast_store;
`Z486_NO_PRUNE reg [2:0] early_kind_probe_r;
wire [5:0]  uc_source_shift;
wire [3:0]  uc_shift_source_class;
wire [1:0]  uc_shift2_source;
wire        uc_is_shift2;
wire        uc_shift_uc_carry;
wire [5:0]  uc_alu_src_shift;
wire [6:0]  uc_aluop_shift;
wire [1:0]  uc_shift_sigma_sel;
wire [6:0]  uc_alu_op_sel;          // pre-decoded ALU op: {from IR, CMP/TEST, op}
wire [2:0]  uc_dly_source;
wire [8:0]  uc_mem_ctrl;
wire [8:0]  uc_ind_ctrl;
wire        uc_fpu_f8;
wire        uc_force_word;
wire        microcode_rom_ce;
wire [2:0]  d2_kind;

// Decode the shifter source one cycle ahead (q_mem leads the executing uop).
function automatic [1:0] shift2_source_next_decode(input [5:0] source);
    case (source)
        SRC_TMPC:   shift2_source_next_decode = 2'd0;
        SRC_TMPE:   shift2_source_next_decode = 2'd1;
        SRC_SIGMA:  shift2_source_next_decode = 2'd2;
        SRC_SRCREG: shift2_source_next_decode = 2'd3;
        default:    shift2_source_next_decode = 2'd0;
    endcase
endfunction
wire       uc_next_is_shift2 = uc_next[17:11] == ALUJMP_SHIFT2;
wire       uc_next_is_src_shift =
    (uc_next[17:11] == ALUJMP_SHIFT) &&
    (uc_next[23:18] == SRC_SRCREG);
wire       uc_next_captures_shift_source = uc_next_is_shift2 ||
                                           uc_next_is_src_shift;
wire [1:0] uc_next_shift2_source = shift2_source_next_decode(uc_next[23:18]);

// synthesis translate_off
// The immutable Intel ROM has only two plain SHIFT words whose source is
// SRCREG. They are the signed bit-index scaling steps for BT and BTS/BTR/BTC.
always @(posedge clk)
    if (reset_n && uc_exec && (uc_aluop == ALUJMP_SHIFT) &&
        (uc_source == SRC_SRCREG) &&
        (uc_addr != 12'h136) && (uc_addr != 12'h14F))
        $fatal(1, "UNCAPTURED SRCREG SHIFT: uc_addr=%03x", uc_addr);
// synthesis translate_on


//=============================================================================
// Unit 1: Prefetch queue and Bus Interface
//=============================================================================
wire [31:0] pf_flush_addr;          // Prefetch flush address

memory #(
    .PROTECT_UMA_ROM(PROTECT_UMA_ROM),
    .DCACHE_SET_BITS(DCACHE_SET_BITS),
    .DCACHE_LINE_WRAP(DCACHE_LINE_WRAP),
    .ICACHE_SET_BITS(ICACHE_SET_BITS),
    .ENABLE_X87(ENABLE_X87),
    .ENABLE_DEVICE_MMIO(ENABLE_DEVICE_MMIO),
    .DEVICE_MMIO_MASK(DEVICE_MMIO_MASK)
) memory_inst (
    .clk(clk),
    .reset_n(reset_n),
    .a20_enable(a20_enable),
    .cache_enable(!cache_off_req && !cache_off_sim),
    .x87_off(x87_off),
    .device_mmio_enable(device_mmio_enable),
    .device_mmio_base(device_mmio_base),

    .dcache_req_valid(dcache_req_valid),
    .dcache_req_phys_addr_raw(dcache_req_phys_addr_raw),
    .dcache_req_preread_offset(dcache_req_preread_offset),
    .dcache_req_preread_priority(dcache_req_preread_priority),
    .dcache_req_write(dcache_req_write),
    .dcache_req_be(dcache_req_be),
    .dcache_req_wdata(dcache_req_wdata),
    .dcache_direct_wdata(dcache_direct_wdata),
    .dcache_req_is_io(dcache_req_is_io),
    .dcache_req_is_inta(dcache_req_is_inta),
    .dcache_req_is_x87(dcache_req_is_x87),
    .dcache_req_is_vga_mem(dcache_req_is_vga_mem),
    .dcache_req_accepted(dcache_req_accepted),
    .dcache_req_complete(dcache_req_complete),
    .dcache_read_complete(dcache_read_complete),
    .dcache_rdata(dcache_rdata),
    .fast_store_valid(fast_store_valid),
    .fast_store_phys_addr_raw(st_take ? st_phys : rmw_fast_phys_r),
    .fast_store_be(fast_store_be),
    .fast_store_wdata(fast_store_wdata),
    .fast_store_accepted(fast_store_accepted),
    .dcache_wr_ready(dcache_wr_ready),
    .dcache_vipt_probe_valid(dcache_vipt_probe_valid),
    .dcache_vipt_probe_offset(dcache_vipt_probe_offset),
    .dcache_vipt_probe_ready(dcache_vipt_probe_ready),
    .dcache_vipt_probe_accepted(dcache_vipt_probe_accepted),
    .dcache_vipt_probe_direct_accepted(dcache_vipt_probe_direct_accepted),
    .dcache_vipt_resolve_valid(dcache_vipt_resolve_valid),
    .dcache_vipt_resolve_phys_addr_raw(dcache_vipt_resolve_phys_addr),
    .dcache_vipt_resolve_hit(dcache_vipt_resolve_hit),
    .dcache_vipt_resolve_data(dcache_vipt_resolve_data),
    .dcache_vipt_resolve_fill(dcache_vipt_resolve_fill),
    .dcache_vipt_fill_start(dcache_vipt_fill_start),
    .dcache_vipt_fill_resp_valid(dcache_vipt_fill_resp_valid),
    .dcache_vipt_fill_resp_data(dcache_vipt_fill_resp_data),

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
    .line_read(line_read),
    .din(din),
    .line_din(line_din),
    .dout(dout),
    .valid(valid),
    .ready(ready),
    .write(write),
    .io(io),
    .resp_valid(resp_valid),
    .line_resp_valid(line_resp_valid),
    .inta(inta)
);

// Prefetch Unit: 16-byte circular buffer
prefetch prefetch_inst (
    .clk(clk),
    .reset_n(reset_n),
    // Queue output to decoder
    .k1q(k1q),
    .k1q_early(k1q_early),
    .k1q_avail(k1q_avail),
    .k1p_adv(k1p_adv),
    .k1p_preread_adv(k1p_preread_adv),
    .k2q(k2q),
    .k2q_avail(k2q_avail),
    .k2p_off(dec_lit_off),
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
    // A retained fetch fault becomes precise once older instructions retire.
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
wire [31:0] spec_disp      = i_bus.branch_rel8 ? {{24{i_bus.displacement[7]}}, i_bus.displacement[7:0]}
                                          : i_bus.displacement;
// A 16-bit operand size truncates the new EIP to 16 bits.
wire [31:0] spec_target_sum = EIP + ({27'd0, i_bus.length} + spec_disp);
wire [31:0] spec_target_eip = i_bus.data32 ? spec_target_sum : {16'h0, spec_target_sum[15:0]};

// Return-address stack: CALL pushes at issue; a direct-load RET predicts its
// target line and keeps it only if the prediction matches.
localparam integer RSB_DEPTH = 4;
reg  [31:0] rsb [0:RSB_DEPTH-1];
reg  [RSB_DEPTH-1:0] rsb_valid;
reg  [1:0]  rsb_top_r;
reg  [31:0] ret_pred_r;                // the issued RET's predicted target
wire        d2_is_call = !i_bus.has_0f &&
                         ((i_bus.rel_branch_kind == REL_BRANCH_CALL) ||
                          ((i_bus.opcode == 8'hFF) && (i_bus.modrm[5:3] == 3'd2)));
wire [31:0] call_return_sum = EIP + {27'd0, i_bus.length};
wire [31:0] call_return = i_bus.data32 ? call_return_sum : {16'h0, call_return_sum[15:0]};
wire        d2_ret_pred = d2_vipt_ret && rsb_valid[rsb_top_r];
wire        ret_issue = vipt_issue_load && d2_vipt_ret;
always_ff @(posedge clk) begin
    if (!reset_n) begin
        rsb_valid <= '0;
        rsb_top_r <= 2'd0;
    end else if (i_issue && d2_is_call) begin
        rsb_top_r <= rsb_top_r + 2'd1;
        rsb[rsb_top_r + 2'd1] <= call_return;
        rsb_valid[rsb_top_r + 2'd1] <= 1'b1;
    end else if (ret_issue) begin
        rsb_top_r <= rsb_top_r - 2'd1;
        rsb_valid[rsb_top_r] <= 1'b0;
    end
    if (ret_issue)
        ret_pred_r <= rsb[rsb_top_r];
end
assign spec_target_lin = CS_base + (d2_ret_pred ? rsb[rsb_top_r] : spec_target_eip);
// A relative branch in D2 requests its target line; an EIP write cancels the request.
wire        eip_write_now = branch_ustep_redirect || ret_redirect ||
                            (uc_exec && ((uc_dest == DEST_EIP) || (uc_dest == DEST_eIP) ||
                                         (uc_dest == DEST_IP) ||
                                         (uc_dest == DEST_USTEP_RPTI_EIP)));
reg         d2_spec_sent_r;            // the D2 branch's target line is requested
// A hardwired branch uStep that executes not taken no longer needs its line.
wire        spec_owner_release = branch_ustep_exec && !branch_ustep_redirect;
assign pf_spec_req = d2_resident && d2_push &&
                     ((i_bus.rel_branch_kind != REL_BRANCH_NONE) || d2_ret_pred) &&
                     !hardwired_off && !d2_spec_sent_r && !eip_write_now &&
                     !(pf_spec_owner_r && !spec_owner_release);
// A RET checks its prediction when its probe resolves in EX.
wire        ret_pred_wrong = vipt_load_ex_r.valid && vipt_load_ex_probed_r &&
                             vipt_load_ex_r.is_ret &&
                             (dcache_vipt_resolve_data != ret_pred_r);
always_ff @(posedge clk) begin
    if (!reset_n || q_flush || i_issue || interrupt_entry || any_fault || eip_write_now)
        d2_spec_sent_r <= 1'b0;
    else if (pf_spec_req)
        d2_spec_sent_r <= 1'b1;
end
// synthesis translate_off
logic [31:0] d2_spec_sent_lin_r;
always_ff @(posedge clk) begin
    if (pf_spec_req) d2_spec_sent_lin_r <= spec_target_lin;
    if (reset_n && i_issue && d2_spec_sent_r && !eip_write_now &&
        (d2_spec_sent_lin_r !== spec_target_lin))
        $fatal(1, "D2 SPEC TARGET STALE: sent %08x issue %08x EIP %08x op %02x uc %03x",
               d2_spec_sent_lin_r, spec_target_lin, EIP, i_bus.opcode, uc_addr);
end
// synthesis translate_on
// Spec-line ownership: held only while the requesting branch is current.
always_ff @(posedge clk) begin
    if (!reset_n)
        pf_spec_owner_r <= 1'b0;
    else begin
        if (spec_owner_release)
            pf_spec_owner_r <= 1'b0;
        // A RET owns its predicted line only as a direct load (a ROM-path RET
        // redirects from microcode, unchecked against the prediction).
        if (i_issue)
            pf_spec_owner_r <= ((d2_spec_sent_r && !eip_write_now) || pf_spec_req) &&
                               (!d2_vipt_ret || d2_vipt_load);
        // A RET whose prediction was wrong, or whose probe missed (its
        // redirect waits for paging), gives up the line.
        if (q_flush || interrupt_entry || any_fault || vipt_load_slow_busy || ret_pred_wrong)
            pf_spec_owner_r <= 1'b0;
    end
end

// Store invalidation is line-selective inside prefetch. External coherence or
// an address-space change invalidates the speculative line conservatively.
reg         pf_snoop_kill_r;
always_ff @(posedge clk) begin
    if (!reset_n)
        pf_snoop_kill_r <= 1'b0;
    else
        pf_snoop_kill_r <= snoop_valid;
end
assign pf_spec_global_kill = pf_snoop_kill_r || cr3_write ||
                             (uc_exec && (uc_dest == DEST_CR0));

//=============================================================================
// Unit 2: Decode1 (structural decode)
//=============================================================================
decoder decoder_inst (
    .clk        (clk),
    .reset_n    (reset_n),

    // Prefetch queue interface (two-cursor protocol)
    .k1q     (k1q),
    .k1q_early(k1q_early),
    .k1q_avail   (k1q_avail),
    .k1p_adv     (k1p_adv),
    .k1p_preread_adv(k1p_preread_adv),
    .k2q    (k2q),
    .k2q_avail  (k2q_avail),
    .k2p_off    (dec_lit_off),
    .pop_now    (dec_pop_now),
    .pop_len    (dec_pop_len),

    // Mode signals
    .D          (decoder_default32_r),
    .pe_enable  (decoder_native_pe_r), // V86 uses real-mode entry points

    // Control signals
    .q_flush    (q_flush),
    .i_issue      (i_issue),

    // Decoded instruction output
    .i_bus      (i_bus),
    .decq_empty (decq_empty),
    .decq_has_jmp_call(decq_has_jmp_call),
    .pb_load(pb_load),
    .pb_entry(pb_load_entry),

    // Unified D2 payload
    .d2_entry   (d2_entry),
    .d2_push    (d2_push),
    .d1_issue_direct(d1_issue_direct),
    .d1_issue_entry(d1_issue_entry),
    .fetch_blocked(decoder_fetch_blocked)
);

// The local mode replicas lag architectural state by one cycle. A far
// transfer loads the new CS descriptor before its frontend flush, so D1 may
// hand off the sequential successor in that cycle, decoded in the old mode;
// the flush must discard it before anything issues.
// synthesis translate_off
logic d1_stale_mode_handoff;
always_ff @(posedge clk) begin
    if (!reset_n || q_flush)
        d1_stale_mode_handoff <= 1'b0;
    else if (d1_issue_direct &&
             ({decoder_default32_r, decoder_native_pe_r} !== {D, pe && !vm}))
        d1_stale_mode_handoff <= 1'b1;
    if (reset_n && d1_stale_mode_handoff && i_issue && !q_flush)
        $fatal(1, "D1 MODE REPLICA MISMATCH");
end
// synthesis translate_on

//=============================================================================
// Unit 3: Decode2 - literal capture and early address
//=============================================================================

// Decode an entry's precomputed EA selectors.
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

ea_dec_t ea_dec_cur;    // D2 head
assign ea_dec_cur = ea_decode_of(i_bus);


// Hardwired recipe policy: classify the D2 instruction, prove successor
// overlap safe, and replace only recipe slots known to be redundant.
hardwired_control hardwired_control_inst (
    .clk(clk),
    .reset_n(reset_n),
    .issue_instr(i_bus),
    .exec_instr(i),
    .issue_ea(ea_dec_cur),
    .decq_empty(decq_empty),
    .d2_push(d2_payload_ready),
    .d2_kind(d2_kind),

    .i_issue(i_issue),
    .i_first(i_first),
    .uc_active(uc_active),
    .uc_exec(uc_exec),
    .uc_slot_live(uc_slot_live),
    .i_rni(i_rni),
    .i_rni_delay(i_rni_delay),
    .uc_next_rni(uc_next[10:8] == 3'b000),
    .uc_aluop(uc_aluop),
    .alu_write_flags((uc_exec && alu_update_flags) ||
                     (vipt_load_wb_valid_r && vipt_load_wb_is_alu_r &&
                      !any_fault)),
    .flags_live(eflags_fwd),
    .exec_condition_true(branch_condition_true),
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
    .load_pipe_issue(vipt_issue_load),
    .load_pipe_pop(vipt_issue_load && d2_vipt_pop),
    .load_pipe_ret(vipt_issue_load && d2_vipt_ret),
    .load_wb_retire(direct_wb_retire),
    .load_probe_wait(d2_resident && d2_vipt_candidate && !d2_vipt_load),
    .pb_valid(pb_valid),
    .tf_issue(EFLAGS[8]),
    .pb_load_ready(pb_load_ready),
    .pb_load(pb_load),
    .pb_next_instr(pb_next_instr),
    .pb_next_ea(skel_load_ea_dec),
    .issue_recipe(d2_recipe),
    .issue_hardwired(d2_hardwired),
    .x87_direct_candidate(x87_direct_candidate),
    .disabled(hardwired_off),
    .pb_issue(pb_issue),
    .pb_slot(pb_slot),
    .recipe_rni(recipe_rni),
    .recipe_state(recipe_state),
    .slot_stale(recipe_slot_stale),
    .fold_active(jcc_fold_active),
    .branch_ustep_rni(branch_ustep_rni),
    .branch_ustep_exec(branch_ustep_exec),
    .branch_redirect(branch_ustep_redirect)
);

// D2 residency and throttle state.
always_ff @(posedge clk) begin
    if (!reset_n) begin
        d2_ea_split_done_r <= 1'b0;
        throttle_parked_r <= 1'b0;
        stack_init_pending <= 1'b0;
    end else begin
        if (q_flush || any_fault)
            d2_ea_split_done_r <= 1'b0;
        else if (d2_ea_split_wait || d2_ea_split_conflict)
            // Recapture a partial sum whose base or index is written on the capture edge.
            d2_ea_split_done_r <= !d2_ea_split_conflict;
        else if (i_issue)
            d2_ea_split_done_r <= 1'b0;

        if (!d2_resident || i_issue || q_flush || any_fault ||
            interrupt_at_boundary || throttle_full)
            throttle_parked_r <= 1'b0;
        else if (d2_resident && i_rni_delay && throttle_hold &&
                 (uc_exec || recipe_slot_stale))
            // Retire the predecessor's architectural delay slot on this edge,
            // then keep the prefetched successor out of EX until release.
            throttle_parked_r <= 1'b1;

        if (i_issue) begin
            i_first <= 1'b1;
            stack_init_pending <= i_bus.stack_op;
        end


        if (!stall) begin
            if (stack_init_pending && !i_issue)
                stack_init_pending <= 1'b0;
            if (i_first && !i_issue)
                i_first <= 1'b0;
        end
    end
end


// synthesis translate_off
always_ff @(posedge clk) begin
    // Paging may present a cached demand while WR_FAST waits for the L1 (a
    // prefetch walk's PTE read); bus_unit holds it off until the store is
    // accepted. Both owning the L1 CPU port in one cycle would be the bug.
    if (reset_n && fast_store_accepted && dcache_req_accepted &&
        memory_inst.bus_unit_inst.normal_cache_req)
        $fatal(1, "WR_FAST collided with paging demand");
    if (reset_n && uc_exec &&
        (i.ucode_action == RECIPE_ACTION_RMW_FAST) &&
        (uc_addr == (i.entry_point + 12'd1)) && !rd_fast_finish &&
        !rmw_fast_active_r)
        $fatal(1, "RMW fast ALU executed without a qualified RD_FAST token");
end
// synthesis translate_on

always_ff @(posedge clk) begin
    if (i_issue)
        early_kind_probe_r <= d2_kind;
end

// A PUSH's RNI word holds post-push ESP in SIGMA. A following stack
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
            DEST_IRF: if (COUNTR[5:3] != 3'b100 && !irf_is_dr &&
                          !(COUNTR[2:0] == 3'd4 && !i.has_0f && i.opcode == 8'h61))
                begin we = 1'b1; sel = COUNTR[2:0]; mode = is_dword ? FWD_D : FWD_W; end
            default: ;
        endcase
        decode_dly_gpr = {we, sel, mode};
    end
endfunction

// Predecode from uc_next (the microword that becomes uc next cycle); register on
// the same enable as uc so dly_gpr_*_pre_r tracks decode_dly_gpr(uc_dest).
wire [5:0] dly_gpr_pre   = decode_dly_gpr(uc_next[30:24]);
reg        dly_gpr_we_pre_r;
reg [2:0]  dly_gpr_sel_pre_r;
reg [1:0]  dly_gpr_mode_pre_r;
always_ff @(posedge clk) begin
    if (!reset_n) begin
        dly_gpr_we_pre_r <= 1'b0; dly_gpr_sel_pre_r <= 3'd0;
        dly_gpr_mode_pre_r <= FWD_D;
    end else if (microcode_rom_ce) begin
        dly_gpr_we_pre_r   <= dly_gpr_pre[5];
        dly_gpr_sel_pre_r  <= dly_gpr_pre[4:2];
        dly_gpr_mode_pre_r <= dly_gpr_pre[1:0];
    end
end

// Delay-slot GPR write descriptor (a stale recipe slot word writes nothing).
assign dly_gpr_we = i_rni_delay_ea && !recipe_slot_stale &&
                    dly_gpr_we_pre_r;
wire [2:0] dly_gpr_sel  = dly_gpr_sel_pre_r;
wire [1:0] dly_gpr_mode = dly_gpr_mode_pre_r;
gpr_forward_t dly_gpr_forward;
assign dly_gpr_forward.valid = dly_gpr_we;
assign dly_gpr_forward.dst = dly_gpr_sel;
assign dly_gpr_forward.mode = dly_gpr_mode;
assign dly_gpr_forward.data = dly_fwd_value;


wire       dly_esp_fwd = dly_gpr_we && (dly_gpr_sel == 3'd4);
wire       shc_esp_fwd = recipe_shift_write.valid && (recipe_shift_write.dst == 3'd4) &&
                         (recipe_shift_write.size != 2'd0);
wire       vipt_esp_fwd = vipt_load_wb_valid_r &&
                          vipt_load_wb_dst_onehot_r[4];
wire [31:0] vipt_esp_value = (vipt_load_wb_size_r == 2'd1)
                           ? {ESP[31:16], vipt_load_wb_data[15:0]}
                           : vipt_load_wb_data;
assign forwarded_esp = (recipe_esp_fwd || (pop_direct_r && i_first)) ? SIGMA :
                            dly_esp_fwd  ? (dly_gpr_mode == FWD_W
                                            ? {ESP[31:16], dly_fwd_value[15:0]}
                                            : dly_fwd_value) :
                            shc_esp_fwd  ? (recipe_shift_write.size == 2'd1
                                            ? {ESP[31:16], recipe_shift_data[15:0]}
                                            : recipe_shift_data) :
                            vipt_esp_fwd ? vipt_esp_value : ESP;

// Base/index selectors latched as the instruction enters D2.
// The instruction entering the skeleton, for the port-B dead-slot decision.
assign pb_next_instr = d1_issue_entry;
assign skel_load_ea_dec = ea_decode_of(d1_issue_entry);
gpr_ref_t ea_base_ref;
gpr_ref_t ea_index_ref;
wire [31:0] ea_base_value;
wire [31:0] ea_index_value;
wire [31:0] ea_early;

// D2-AGU observer
assign d2_agu_dec = ea_decode_of(d2_entry);
// Use the split cycle only once the incoming entry is resident.
wire d2_ea_three_term = d2_resident && d2_push &&
                        (d2_entry.ea_complex || d2_entry.ea_uses_post_pop_esp);
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
// GPRs written by the current microword (zero for other destinations).
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
            if (COUNTR[5:3] != 3'b100 && !irf_is_dr &&
                !(COUNTR[2:0] == 3'd4 && !i.has_0f && i.opcode == 8'h61))
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
        ? 8'h10 : 8'h0);
// Hold D2 a cycle if a deferred producer writes a base/index on the issue edge.
wire [7:0] d2_split_commit_mask = pend_write_mask;
wire d2_ea_split_refresh = d2_ea_split_done_r &&
    (((d2_agu_dec.base_sel | d2_agu_dec.index_sel) & d2_split_commit_mask) != 8'h00);
// A capture whose base/index is written on the same edge is retried next cycle.
assign d2_ea_split_conflict = ((d2_agu_dec.base_sel | d2_agu_dec.index_sel) &
                               (ea_inval_gpr | d2_split_commit_mask)) != 8'h00;
assign d2_ea_split_wait = d2_ea_three_term &&
                          (!d2_ea_split_done_r || d2_ea_split_refresh);
// Clear-all events: segment state may change under any committed seg
// command or descriptor load; the effective-mask mode must be stable.
reg d2_agu_effmask_r;
always_ff @(posedge clk) d2_agu_effmask_r <= eff_mask_pending;
// Only cache-mutating segment commands clear the sidecars.
wire seg_cmd_mutates = (seg_cmd != SEG_CMD_NONE) &&
                       (seg_cmd != SEG_CMD_INIT_SEG) &&
                       (seg_cmd != SEG_CMD_UPDATE_SEG) &&
                       (seg_cmd != SEG_CMD_SPCR);
wire ea_inval_all = (seg_cmd_valid && seg_cmd_mutates) ||
                    (d2_agu_effmask_r != eff_mask_pending);

// Eligibility: 32-bit memory ModR/M EA, no moffs/stack, no same-cycle conflict.
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
wire [31:0] seg_base_exec;     // microcode relocation view, excluding issue INIT_SEG
wire        eff_mask_exec;
wire [31:0] seg_lar_result, seg_llim_result, seg_lbas_result;

// Segmentation unit command encoder
reg  [3:0]  seg_cmd_target;
reg  [31:0] seg_cmd_data;
reg  [3:0]  uc_seg_cmd;
reg  [3:0]  uc_seg_target;
reg  [31:0] uc_seg_data;
// Decoded instruction register (all fields from decoder, latched at i_issue)
wire [3:0] modrm_resolved_seg = apply_seg_override_type(
    calc_default_seg_type(i.modrm, i.sib, i.has_sib, i.addr32), i.seg);

// Pre-computed default segment for new instruction (combinational, used by INIT_SEG)
wire [3:0] init_default_seg = i_bus.stack_op ? SEG_SS :
                              i_bus.has_moffs ? SEG_DS :
                              i_bus.has_modrm ? calc_default_seg_type(i_bus.modrm, i_bus.sib, i_bus.has_sib, i_bus.addr32) :
                              SEG_DS;
wire [3:0] init_final_seg = i_bus.stack_op ? init_default_seg :
                            apply_seg_override_type(init_default_seg, i_bus.seg);

// Issue-time segment base and mask come straight from the decoder.
wire [31:0] issue_seg_base = desc_cache[i_bus.mem_seg[2:0]].base;
wire        issue_eff_mask = (i_bus.stack_op && pe)
                           ? desc_cache[SEG_SS].D_B : i_bus.addr32;

// Access width for the limit check (source width for MOVZX/MOVSX).
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
    .instr_done       (i_rni_delay || (recipe_rni && uc_exec)),
    .seg_cmd          (seg_cmd),
    .seg_target       (seg_cmd_target),
    .exec_seg_cmd     (uc_seg_cmd),
    .exec_seg_target  (uc_seg_target),
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
    // MOV r,DRn reads "-1 -> IRF" with LBAS; SGDT/SIDT's LBAS names its
    // descriptor in the destination and leaves COUNTR stale.
    .lbas_dr_sel      (irf_is_dr && uc_dest == DEST_IRF),
    .lbas_dr_value    (COUNTR[2] ? 32'h0 : DR03[COUNTR[1:0]]),
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
    .seg_base_exec    (seg_base_exec),
    .eff_mask_exec    (eff_mask_exec),
    .seg_fault        (seg_gp_fault),
    .is_stack_fault   (ss_segment_fault),
    // Decoder D2: issuing instruction, EA recipe, displacement and D2 segment base (ISLA/IESSEG, K2Q)
    .au_instr_issue(i_issue),
    .au_instr(i_bus),
    .au_d2_start(pb_load),
    .au_d2_ea(skel_load_ea_dec),
    .au_split_ea_prepare(d2_ea_three_term && !i_issue),
    .au_split_ea_use(d2_ea_three_term && d2_ea_split_done_r),
    .au_split_ea_adjust(d2_entry.ea_uses_post_pop_esp ? (d2_entry.data32 ? 3'd4 : 3'd2) : 3'd0),
    .au_displacement(d2_agu_dec.disp),
    .au_branch_relative(i_bus.rel_branch_kind != REL_BRANCH_NONE),
    .au_branch_target_eip(spec_target_eip),
    .au_issue_seg_base(issue_seg_base),
    .au_issue_eff_mask(issue_eff_mask),
    // Datapath: I-bus base/index reads and E-stage operands
    .au_ea_base(ea_base_ref),
    .au_ea_index(ea_index_ref),
    .au_ea_base_value(ea_base_value),
    .au_ea_index_value(ea_index_value),
    .au_forwarded_esp(forwarded_esp),
    .au_source_value(dest_value),
    .au_alu_value(alu_src),
    .au_alu_value_hold(alu_src_r),
    .au_is_dword(is_dword),
    // Microsequencer: E-stage IND control
    .au_exec(uc_exec),
    .au_exec_addr32(i.addr32),
    .au_alu_source(uc_alu_src),
    .au_ind_ctrl(uc_ind_ctrl),
    // Keyed on the microword, not the instruction: a late write fault of the
    // predecessor runs its delivery microcode while a Jcc is the current
    // instruction, and its IN=+ words must add their own ALU constant.
    .au_jcc_word(uc_aluop == ALUJMP_JNcond),
    // Paging and control registers: fault readback
    .au_fault_code(latched_pf_code),
    .au_fault_addr(latched_pf_addr),
    .au_cr3(CR3),
    // LA bus to paging and cache
    .au_issue_linear(issue_ind_linear),
    .au_issue_linear_low(issue_ind_linear_low),
    .au_issue_mem_linear(issue_mem_linear),
    .au_ind_linear(ind_linear),
    .au_ind_linear_valid(ind_linear_valid),
    // EA bus and IND to control and datapath
    .au_issue_ea(ea_early),
    .au_ea(ea_reg),
    .au_ind(IND),
    .au_ind_delta(IND_DELTA)
);

// Execution-relocation view of the microcode segment command.
always_comb begin
    uc_seg_cmd = SEG_CMD_NONE;
    uc_seg_data = dest_value;
    if ((uc_buscode == BUSOP_IND_PLUS_ALU || uc_buscode == BUSOP_IND_ALU2 ||
         uc_buscode == BUSOP_IND_SRC) &&
        (uc_dest == DEST_DES_OS || uc_dest == DEST_DES_SR))
        uc_seg_target = modrm_resolved_seg;
    else
        uc_seg_target = resolve_seg_target(uc_dest, i.seg_reg_sel, COUNTR[5:0]);

    if (uc_dest == DEST_DESCSW) begin
        uc_seg_cmd = SEG_CMD_DESCSW;
    end else begin
        case (uc_buscode)
            BUSOP_IND_PLUS_ALU,
            BUSOP_IND_ALU2,
            BUSOP_IND_SRC: begin
                uc_seg_cmd = SEG_CMD_UPDATE_SEG;
            end
            BUSOP_SBRM: begin
                if (!pe || vm)
                    uc_seg_cmd = SEG_CMD_SBRM;
            end
            BUSOP_SAR: begin
                uc_seg_cmd = SEG_CMD_SAR;
            end
            BUSOP_SLIM: begin
                uc_seg_cmd = (uc_dest == DEST_DESPTR)
                           ? SEG_CMD_SLIM_TABLE : SEG_CMD_SLIM;
            end
            BUSOP_SBAS: begin
                if (uc_dest == DEST_DESPTR)
                    uc_seg_cmd = SEG_CMD_SBAS;
            end
            BUSOP_SDEH: begin
                if (pe && !gate_detect_cond)   // use cond, not _now (uc_exec already in valid)
                    uc_seg_cmd = SEG_CMD_SDEH;
            end
            BUSOP_SDES: begin
                if (pe && !gate_detect_cond) begin
                    uc_seg_cmd = SEG_CMD_SDES;
                    uc_seg_data = alu_src_data;
                end
            end
            BUSOP_SDEL: begin
                if (pe && !gate_detect_cond) begin
                    uc_seg_cmd = SEG_CMD_SDEL;
                    // SDEL's descriptor-low operand is encoded in the ALU source
                    // field. Most sites use TMPC, but cross-privilege CALL uses TMPD.
                    uc_seg_data = alu_src_data;
                end
            end
            BUSOP_SPCR: begin
                uc_seg_cmd = SEG_CMD_SPCR;
            end
            default: ;
        endcase
    end
end

always_comb begin
    if (i_issue) begin
        seg_cmd = SEG_CMD_INIT_SEG;
        seg_cmd_target = init_final_seg;
        seg_cmd_data = dest_value;
    end else begin
        seg_cmd = uc_seg_cmd;
        seg_cmd_target = uc_seg_target;
        seg_cmd_data = uc_seg_data;
    end
end


//=============================================================================
// Unit 5: Paging Unit (including TLB)
//=============================================================================

// Access width: |IND_DELTA| for RD W/WR W, else the source width.
wire ind_delta_dword = (IND_DELTA == 32'd4) || (IND_DELTA == -32'd4);
wire [1:0] mem_eff_size = uc_is_word_op
                          ? ((ind_delta_dword && !uc_force_word) ? 2'd2 : 2'd1) :
                          uc_is_dword_op ? 2'd2 : srcreg_size;

wire [31:0] mem_wdata = (uc_buscode == BUSOP_WR_OPR ||
                         uc_buscode == BUSOP_WR_OPR_WORD) ? OPR_R :
    uc_is_word_op ? memory_write_source_value :
    (uc_dest == DEST_OPR_W) ? (stack_init_pending ? source_value_live : dest_value) :
    OPR_W;

// synthesis translate_off
always @(posedge clk)
    if (reset_n && uc_exec && (uc_buscode == BUSOP_WR_WORD) &&
        (memory_write_source_value !== source_value_live))
        $fatal(1, "WR-W SOURCE MUX MISMATCH: uc_addr=%03x src=%02x narrow=%08x full=%08x",
               uc_addr, uc_source, memory_write_source_value, source_value_live);
// synthesis translate_on

// INVLPG: decoder-registered action; its address is already in IND.
wire invlpg_active = uc_active && i_first &&
    (i.ucode_action == RECIPE_ACTION_INVLPG);
wire invlpg_priv_fault = invlpg_active && pe && (cpl != 2'b00);
wire invlpg_request = invlpg_active && !invlpg_priv_fault && !seg_gp_fault;
wire invlpg_ack;
// Waiting for an older page walk does not depend on seg_fault.
assign stall_invlpg = invlpg_active && !invlpg_priv_fault && !invlpg_ack;
// RD_FAST uses the authoritative segment checker only as a qualifier. A
// rejection re-enters the original routine, which owns precise fault delivery.
assign gp_fault_trigger = (seg_gp_fault && !rd_fast_valid_r) ||
                          invlpg_priv_fault;

// Deferred GPR commits cancel on any_fault_issue only: a divide overflow fires
// deep inside DIV's microcode, never while a load or hardwired recipe commits.
// synthesis translate_off
always @(posedge clk)
    if (reset_n && div_overflow &&
        (vipt_load_wb_valid_r || recipe_mem_write.valid || recipe_rni))
        $fatal(1, "divide overflow coincides with a deferred GPR commit");
// synthesis translate_on
// div_overflow fires only at the first DIV7/PREDIV word
assign any_fault_issue = gp_fault_trigger || page_fault;
assign any_fault = any_fault_issue || div_overflow;
// Registered any_fault is used for deferred SIGMA/TMPeSP writes.
always_ff @(posedge clk) any_fault_r <= any_fault;
wire [2:0]  data_fault_code;
wire [31:0] data_cr2_out;
// The executing instruction is older than a blocked frontend fetch. If both
// faults arrive together, preserve the demand-side exception and CR2 value.
// A code fetch's walk can run before the instruction's privilege is
// established (an IRET to ring 3 starts prefetching before it loads CS), so
// a fetch fault takes U/S from the CPL it is raised at; a fetch is a read.
wire [2:0]  pg_fault_code = data_page_fault ? data_fault_code
                                            : {cpl == 2'd3, 1'b0, ifetch_fault_code[0]};
wire [31:0] pg_cr2_out = data_page_fault ? data_cr2_out : ifetch_fault_addr;
assign page_fault = data_page_fault || ifetch_page_fault;

// CR3 write detection for TLB flush
assign cr3_write = uc_exec && uc_buscode == BUSOP_SPCR && uc_dest == DEST_PDBR;

// IO request detection
wire mem_is_io = mem_seg_is_io;     // registered in segmentation_unit alongside seg_sel
wire io_busop_rd = uc_p_io_rd && mem_is_io;
wire io_busop_wr = uc_p_io_wr && mem_is_io;

wire iack_busop = uc_p_iack;        // IACK bus operation (interrupt acknowledge)

// A stale slot word (a dead slot that issued nothing) starts no bus cycle.
assign mem_op_eligible = core_live && !mem_servicing && !recipe_slot_stale && !stall_ucrd &&
                         !throttle_parked_r && !vipt_load_exec_block &&
                         !(i_rni_delay && d2_vipt_candidate);
// A failed protection test blocks bus operations in its delay slots.
wire uc_data_busreq = !prot_redirect_prev &&
                      ((uc_is_mem_busop && !mem_is_io) ||
                       io_busop_rd || io_busop_wr);
assign uc_busreq = uc_data_busreq || iack_busop;
assign mem_req_current = mem_op_eligible && uc_busreq;  // drives paging unit
// Delay prefetch on upcoming demand memory
wire mem_req_upcoming = uc_next[39] && !halted && (uc_active || d2_resident);

// VM as the current instruction issued. An IRET to V86 sets VM before it
// reads the rest of its frame from the ring-0 stack; those reads stay
// supervisor, while everything a V86 instruction does is user (CPL 3).
reg vm_at_issue;
always_ff @(posedge clk)
    if (!reset_n)
        vm_at_issue <= 1'b0;
    else if (i_issue && !stall)
        vm_at_issue <= vm;

// Implicit supervisor access: descriptor table and TSS reads, cross-privilege
// stack writes, and an IRET's frame after it has entered V86 use CPL=0 for
// paging regardless of current CPL.
wire implicit_supervisor = mem_is_dtable || (mem_seg_sel == SEG_TR) ||
                           descsw_mode || (vm && !vm_at_issue);
assign pg_cpl = implicit_supervisor ? 2'b00 : cpl;

// Registered fault redirect state.
reg         gp_fault_r;
reg         ss_fault_r;

wire        mem_req_to_paging = (mem_op_eligible &&
                                 (uc_data_busreq || x87_direct_mem_req) &&
                                 !gp_fault_trigger && !ucrd_route_pre && !st_route) ||
                                vipt_slow_submit || ucrd_slow_submit;

wire        iack_req_to_paging = mem_op_eligible && iack_busop && !gp_fault_trigger;
wire        mem_write_now = (x87_direct_mem_req || vipt_slow_submit || ucrd_slow_submit) ? 1'b0 :
                            (uc_is_write || (io_busop_wr && mem_is_io));
wire [1:0]  paging_mem_eff_size = vipt_slow_submit
                                ? vipt_load_slow_r.mem_size
                                : ucrd_slow_submit ? ucrd_size_r
                                : x87_direct_mem_req ? 2'd2 : mem_eff_size;
wire [31:0] paging_linear_addr = vipt_slow_addr_owned
                               ? vipt_load_slow_r.linear_addr
                               : ucrd_slow_req_r ? ucrd_linear_r : ind_linear;
wire [3:0]  mem_be_now = iack_busop ? 4'b1111 :
                          calc_be(paging_mem_eff_size,
                                  paging_linear_addr[1:0]);
assign pf_spec_store = (mem_req_to_paging && mem_write_now && mem_accepted) || st_take;
assign pf_spec_store_linear = paging_linear_addr;
wire        paging_owned_submit = vipt_slow_submit || ucrd_slow_submit;
wire        paging_live_valid  = paging_owned_submit ? 1'b1 : ind_linear_valid;
wire        paging_mem_rd_ind = !x87_direct_mem_req && !paging_owned_submit &&
                                (uc_buscode == BUSOP_RD_IND);
wire        paging_is_write_access = !x87_direct_mem_req && !paging_owned_submit &&
                                      (uc_is_write || uc_is_check_write);


data_access data_access_inst (
    // Clock and reset
    .clk(clk),
    .reset_n(reset_n),
    // L1 data cache: probe/resolve port and direct store port
    .dcache_vipt_probe_accepted(dcache_vipt_probe_accepted),
    .dcache_vipt_probe_direct_accepted(dcache_vipt_probe_direct_accepted),
    .dcache_vipt_probe_ready(dcache_vipt_probe_ready),
    .dcache_vipt_resolve_data(dcache_vipt_resolve_data),
    .dcache_vipt_resolve_hit(dcache_vipt_resolve_hit),
    .dcache_vipt_fill_start(dcache_vipt_fill_start),
    .dcache_vipt_fill_resp_valid(dcache_vipt_fill_resp_valid),
    .dcache_vipt_fill_resp_data(dcache_vipt_fill_resp_data),
    .dcache_wr_ready(dcache_wr_ready),
    .fast_store_accepted(fast_store_accepted),
    .dcache_vipt_probe_offset(dcache_vipt_probe_offset),
    .dcache_vipt_probe_valid(dcache_vipt_probe_valid),
    .dcache_vipt_resolve_phys_addr(dcache_vipt_resolve_phys_addr),
    .dcache_vipt_resolve_valid(dcache_vipt_resolve_valid),
    .dcache_vipt_resolve_fill(dcache_vipt_resolve_fill),
    .fast_store_be(fast_store_be),
    .fast_store_valid(fast_store_valid),
    .fast_store_wdata(fast_store_wdata),
    .rmw_fast_phys_r(rmw_fast_phys_r),
    .st_phys(st_phys),
    .st_route(st_route),
    .st_take(st_take),
    // Paging unit and sidecar TLB
    .CR0(CR0),
    .mem_accepted(mem_accepted),
    .mem_servicing(mem_servicing),
    .paging_demand_idle(paging_demand_idle),
    .pg_cpl(pg_cpl),
    .vipt_tlb_dirty(vipt_tlb_dirty),
    .vipt_tlb_hit(vipt_tlb_hit),
    .vipt_tlb_is_vga_mem(vipt_tlb_is_vga_mem),
    .vipt_tlb_phys_addr(vipt_tlb_phys_addr),
    .vipt_tlb_user(vipt_tlb_user),
    .vipt_tlb_writable(vipt_tlb_writable),
    .sidecar_bg_pre(sidecar_bg_pre),
    .st_tlb_pre(st_tlb_pre),
    .ucrd_cpl_r(ucrd_cpl_r),
    .ucrd_hit(ucrd_hit),
    .ucrd_linear_r(ucrd_linear_r),
    .ucrd_phys_ok_r(ucrd_phys_ok_r),
    .ucrd_phys_r(ucrd_phys_r),
    .ucrd_route_pre(ucrd_route_pre),
    .ucrd_size_r(ucrd_size_r),
    .ucrd_slow_req_r(ucrd_slow_req_r),
    .ucrd_slow_submit(ucrd_slow_submit),
    .ucrd_slow_wait_r(ucrd_slow_wait_r),
    .ucrd_take(ucrd_take),
    .ucrd_x87_r(ucrd_x87_r),
    .vipt_load_slow_r(vipt_load_slow_r),
    .vipt_probe_linear(vipt_probe_linear),
    .vipt_slow_addr_owned(vipt_slow_addr_owned),
    .vipt_slow_phys_ok_r(vipt_slow_phys_ok_r),
    .vipt_slow_phys_r(vipt_slow_phys_r),
    .vipt_slow_submit(vipt_slow_submit),
    // Address and segmentation units
    .ds_flat(ds_flat),
    .ind_linear(ind_linear),
    .ind_linear_valid(ind_linear_valid),
    .issue_ind_linear(issue_ind_linear),
    .issue_ind_linear_low(issue_ind_linear_low),
    .issue_load_linear(issue_load_linear),
    .issue_load_low(issue_load_low),
    .issue_mem_linear(issue_mem_linear),
    .pe(pe),
    .vm(vm),
    .seg_gp_fault(seg_gp_fault),
    .ss_flat32(ss_flat32),
    .seg_readable(seg_readable),
    // Data unit: load writeback and operands
    .forwarded_esp(forwarded_esp),
    .mem_wdata(mem_wdata),
    .OPR_R(OPR_R),
    .SIGMA(SIGMA),
    .direct_wb_retire(direct_wb_retire),
    .fast_opr_commit(fast_opr_commit),
    .fast_opr_data(fast_opr_data),
    .vipt_load_alu_dst_capture(vipt_load_alu_dst_capture),
    .vipt_load_alu_dst_capture_data(vipt_load_alu_dst_capture_data),
    .vipt_load_alu_dst_capture_dst(vipt_load_alu_dst_capture_dst),
    .vipt_load_alu_dst_capture_size(vipt_load_alu_dst_capture_size),
    .vipt_load_wb_alu_op_r(vipt_load_wb_alu_op_r),
    .vipt_load_wb_data(vipt_load_wb_data),
    .vipt_load_wb_dst_onehot_r(vipt_load_wb_dst_onehot_r),
    .vipt_load_wb_fwd_mask_r(vipt_load_wb_fwd_mask_r),
    .vipt_load_wb_dst_r(vipt_load_wb_dst_r),
    .vipt_load_wb_is_alu_r(vipt_load_wb_is_alu_r),
    .vipt_load_wb_size_r(vipt_load_wb_size_r),
    .vipt_load_wb_target_r(vipt_load_wb_target_r),
    .vipt_load_wb_valid_r(vipt_load_wb_valid_r),
    // D2 instruction and issue
    .d2_vipt_ea_hazard(d2_vipt_ea_hazard),
    .EIP(EIP),
    .hardwired_off(hardwired_off),
    .i_bus(i_bus),
    .i_issue(i_issue),
    .single_step(single_step),
    .d2_plain_load_overlap_ready(d2_plain_load_overlap_ready),
    .d2_vipt_candidate(d2_vipt_candidate),
    .d2_vipt_load(d2_vipt_load),
    .d2_vipt_pipe_ready(d2_vipt_pipe_ready),
    .d2_vipt_pop(d2_vipt_pop),
    .d2_vipt_ret(d2_vipt_ret),
    .d2_vipt_rmw(d2_vipt_rmw),
    .d2_vipt_rmw_candidate(d2_vipt_rmw_candidate),
    .vipt_issue_load(vipt_issue_load),
    // Microsequencer and core control
    .any_fault(any_fault),
    .gp_fault_trigger(gp_fault_trigger),
    .i_ex(i),
    .i_first(i_first),
    .i_rni_delay(i_rni_delay),
    .interrupt_entry(interrupt_entry),
    .mem_eff_size(mem_eff_size),
    .mem_is_io(mem_is_io),
    .mem_op_eligible(mem_op_eligible),
    .q_flush(q_flush),
    .stall(stall),
    .stall_invlpg(stall_invlpg),
    .stall_wio(stall_wio),
    .stall_x87_direct(stall_x87_direct),
    .uc_active(uc_active),
    .uc_addr(uc_addr),
    .uc_addr_mem_r(uc_addr_mem_r),
    .uc_buscode(uc_buscode),
    .uc_busreq(uc_busreq),
    .uc_bus_or_dly(uc_bus_or_dly),
    .uc_data_busreq(uc_data_busreq),
    .uc_exec(uc_exec),
    .uc_is_check_write(uc_is_check_write),
    .uc_is_mem_busop(uc_is_mem_busop),
    .uc_is_write(uc_is_write),
    .uc_p_pure_dly(uc_p_pure_dly),
    .x87_direct_mem_req(x87_direct_mem_req),
    .pop_direct_r(pop_direct_r),
    .rd_fast_finish(rd_fast_finish),
    .rd_fast_valid_r(rd_fast_valid_r),
    .ret_redirect(ret_redirect),
    .rmw_fallback_delay_r(rmw_fallback_delay_r),
    .rmw_fast_active_r(rmw_fast_active_r),
    .stall_fast_store(stall_fast_store),
    .stall_rmw_probe(stall_rmw_probe),
    .stall_ucrd(stall_ucrd),
    .vipt_load_ex_probed_r(vipt_load_ex_probed_r),
    .vipt_load_ex_r(vipt_load_ex_r),
    .vipt_load_exec_block(vipt_load_exec_block),
    .vipt_load_replay_r(vipt_load_replay_r),
    .vipt_load_slow_busy(vipt_load_slow_busy),
    .vipt_load_slow_wait_r(vipt_load_slow_wait_r)
);

// Paging unit instantiation
paging_unit paging_inst (
    .clk                (clk),
    .reset_n            (reset_n),
    .cr0                (CR0),
    .cr3                (CR3),
    .cr3_write          (cr3_write),
    .invlpg_req         (invlpg_request),
    .invlpg_linear      (ind_linear),
    .invlpg_ack         (invlpg_ack),

    // Memory/IO request: current RD/WR/IACK uop is held by stall until accepted.
    .mem_req            (mem_req_to_paging),
    .mem_inta_req       (iack_req_to_paging),
    .mem_inta_addr      (IND),
    .mem_req_precheck   ((mem_op_eligible &&
                          (uc_data_busreq || x87_direct_mem_req) && !ucrd_route_pre &&
                          !st_route) ||
                         vipt_slow_submit || ucrd_slow_submit),
    .mem_req_upcoming   (mem_req_upcoming), // suppresses prefetch start to minimize contention
    .mem_accepted       (mem_accepted),     // ready: request accepted this cycle
    .mem_servicing      (mem_servicing),
    .demand_idle        (paging_demand_idle),
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
    .mem_check_only     (paging_owned_submit ? 1'b0 : uc_is_check_write),
    .pretrans_valid     ((ucrd_slow_submit && ucrd_phys_ok_r) ||
                         (vipt_slow_submit && vipt_slow_phys_ok_r)),
    .pretrans_phys      (ucrd_slow_submit ? ucrd_phys_r : vipt_slow_phys_r),
    .cpl                (ucrd_slow_submit ? ucrd_cpl_r : pg_cpl),
    .mem_is_io          (paging_owned_submit ? 1'b0 : mem_is_io),
    .mem_be             (mem_be_now),
    .fast_off           (hardwired_off),
    .vipt_preread       (dcache_vipt_probe_valid || st_tlb_pre || sidecar_bg_pre),
    .vipt_linear_addr   (st_tlb_pre ? issue_mem_linear :
                         sidecar_bg_pre ? ind_linear : vipt_probe_linear),
    .vipt_fallback      (paging_owned_submit),
    .vipt_tlb_hit       (vipt_tlb_hit),
    .vipt_tlb_phys_addr (vipt_tlb_phys_addr),
    .vipt_tlb_writable  (vipt_tlb_writable),
    .vipt_tlb_user      (vipt_tlb_user),
    .vipt_tlb_dirty     (vipt_tlb_dirty),
    .vipt_tlb_is_vga_mem(vipt_tlb_is_vga_mem),
    .fast_opr_commit    (fast_opr_commit || x87_store_opr_commit),
    .fast_opr_data      (x87_store_opr_commit ? x87_store_opr_data : fast_opr_data),

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
    .dcache_req_preread_offset(dcache_req_preread_offset),
    .dcache_req_preread_priority(dcache_req_preread_priority),
    .dcache_req_write   (dcache_req_write),
    .dcache_req_be      (dcache_req_be),
    .dcache_req_wdata   (dcache_req_wdata),
    .dcache_direct_wdata(dcache_direct_wdata),
    .x87_req_wdata      (x87_req_wdata),
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
// The LDT limit is page-granular when its descriptor's G is set.
wire selector_oob_wire = slctr_fwd[2]
    ? (seg_effective_limit(desc_cache[7]) < {16'h0, selector_desc_end})
    : (gdt_limit[15:0] < selector_desc_end);

wire [1:0]  prot_desc_dpl;
wire        protun_write_low16_nonzero;

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
    .uc_source_low16_nonzero(protun_write_low16_nonzero),
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
    .x87_off(x87_off),
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
//=============================================================================

wire double_fault_start;
wire gate_detect_now;
wire instr_eip_written;
wire misc2_flag;
wire recipe_fallback_taken;
event_control #(.ENABLE_X87(ENABLE_X87)) event_control_inst (
    // Clock and reset
    .clk(clk),
    .reset_n(reset_n),
    .x87_off(x87_off),
    // Microsequencer and E-stage lifecycle (current microword and its enables)
    .uc_addr(uc_addr),
    .uc_aluop(uc_aluop),
    .uc_buscode(uc_buscode),
    .uc_dest(uc_dest),
    .uc_exec(uc_exec),
    .uc_flags(uc_flags),
    .uc_jpereq_fwd(uc_jpereq_fwd),
    .stall(stall),
    .repeat_active(repeat_active),
    .q_flush(q_flush),
    .d2_resident(d2_resident),
    .i_issue(i_issue),
    .i_first(i_first),
    .i_rni(i_rni),
    .i_rni_delay(i_rni_delay),
    .direct_wb_retire(direct_wb_retire),
    .rmw_fallback_delay_r(rmw_fallback_delay_r),
    .throttle_parked_r(throttle_parked_r),
    .x87_direct_taken(x87_direct_taken),
    // Decoder: D2 entry and the EX instruction register
    .i_bus(i_bus),
    .i(i),
    // Datapath and architectural state
    .COUNTR(COUNTR),
    .CS(CS),
    .EIP(EIP),
    .EFLAGS(EFLAGS),
    .eflags_fwd(eflags_fwd),
    .IND(IND),
    .alu_result(alu_result),
    .ea_reg(ea_reg),
    .is_dword(is_dword),
    .pe(pe),
    .vm(vm),
    .cpl(cpl),
    .pe_mode_toggle_now(pe_mode_toggle_now),
    .branch_ustep_redirect(branch_ustep_redirect),
    .flags_backup_active(flags_backup_active),
    // Segmentation and protection test unit
    .desc_cache(desc_cache),
    .desc_raw_hi(desc_raw_hi),
    .tss_access_flag(tss_access_flag),
    // Fault requests (segmentation, paging, datapath)
    .any_fault(any_fault),
    .any_fault_r(any_fault_r),
    .gp_fault_trigger(gp_fault_trigger),
    .gp_fault_r(gp_fault_r),
    .ss_segment_fault(ss_segment_fault),
    .ss_fault_r(ss_fault_r),
    .page_fault(page_fault),
    .data_page_fault(data_page_fault),
    .pg_fault_code(pg_fault_code),
    .pg_cr2_out(pg_cr2_out),
    .div_overflow(div_overflow),
    // Interrupt controller and debug traps
    .intr_pending(intr_pending),
    .nmi_request_active(nmi_request_active),
    .interrupt_pending(interrupt_pending),
    .inhibit_interrupts(inhibit_interrupts),
    .tf_trap_pending(tf_trap_pending),
    .single_step(single_step),
    // FPU handshake
    .x87_pereq(x87_pereq),
    .x87_busy_n(x87_busy_n),
    .x87_error_n(x87_error_n),
    .x87_ne(CR0[5]),
    .x87_ignne(x87_ignne),
    // To the microsequencer: conditions and redirect commands
    .seq_advance(seq_advance),
    .seq_conditions(seq_conditions),
    .seq_fault_redirect(seq_fault_redirect),
    .seq_boundary_redirect(seq_boundary_redirect),
    .recipe_fallback_taken(recipe_fallback_taken),
    .gate_detect_now(gate_detect_now),
    .gate_detect_cond(gate_detect_cond),
    .double_fault_start(double_fault_start),
    // Macro-instruction lifecycle and delivery state
    .uc_active(uc_active),
    .halted(halted),
    .instr_eip_written(instr_eip_written),
    .interrupt_entry(interrupt_entry),
    .fault_suppress_delay_slot(fault_suppress_delay_slot),
    .tf_active_r(tf_active_r),
    .tf_trap_suppress_r(tf_trap_suppress_r),
    .latched_pf_code(latched_pf_code),
    .latched_pf_addr(latched_pf_addr),
    .misc2_flag(misc2_flag),
    // Debug and board
    .dbg_first_done(dbg_first_done),
    .debug_ip(debug_ip),
    .triple_fault_reset(triple_fault_reset)
);

assign uc_alu_src       = uc[36:31];  // ABCDEF: ALU source / jump offset
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

reg [31:0] TMPeIP;                  // Saved EIP for RPTI (repeat instruction)
reg [31:0] wr_restart_eip;          // TMPeIP at each demand-write issue, for late write faults
reg [31:0] ucrd_restart_eip;        // TMPeIP/TMPeSP when a microcode read probes: its miss can
reg [31:0] ucrd_restart_esp;        // fault after younger instructions have issued
reg [31:0] wr_restart_esp;          // and the writer's instruction-start ESP
reg [31:0] TMPeSP;                  // Saved ESP for fault handling

// Hardwired relative-branch target and microcode PREF restart (from IND).
wire [31:0] pf_flush_ip = IND;

wire        br_is_jcc      = i.rel_branch_kind == REL_BRANCH_JCC;
wire        br_is_jmp_rel  = i.rel_branch_kind == REL_BRANCH_JMP;
wire        br_is_call_rel = i.rel_branch_kind == REL_BRANCH_CALL;
wire [31:0] br_disp        = i.branch_rel8 ? {{24{i.displacement[7]}}, i.displacement[7:0]}
                                        : i.displacement;
assign br_target = EIP + br_disp;

// A taken Jcc's redirect (branch_ustep_redirect) is decided by forwarded flags
// late in the cycle, so its address is added on its own and selected after
// the adders; the other sources are selected before theirs. The precise CALL
// redirect is named on its own here so the flags do not reach that select.
wire        early_call_redirect = i_first && is_dword && br_is_call_rel && !early_redirected;
wire [31:0] pf_branch_addr = CS_base + ea_reg;
wire [31:0] pf_other_addr  = ret_redirect        ? (CS_base + vipt_load_wb_target_r) :
                             early_call_redirect ? (CS_base + br_target) :
                             pe_mode_toggle_now  ? (CS_base + EIP) :
                                                   (CS_base + pf_flush_ip);
assign pf_flush_addr = branch_ustep_redirect ? pf_branch_addr : pf_other_addr;
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

// Precise early CALL redirect at i_first; its branch uStep must not flush again.
assign early_redirect = (branch_ustep_redirect && !early_redirected) || ret_redirect ||
                        early_call_redirect;
// A fault or interrupt abandons the instruction that owned an early redirect.
// Clear that ownership before its handler's microcode PREF reaches q_flush.
always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n)                                  early_redirected <= 1'b0;
    else if (any_fault || interrupt_entry)         early_redirected <= 1'b0;
    else if (early_redirect)                       early_redirected <= 1'b1;
    else if (i_issue)                              early_redirected <= 1'b0;
end

assign uc_is_wio = uc_p_wio;  // WIO: wait for interrupt/IO (HLT, only with RPT)
assign uc_is_rpt = uc_p_rpt;

// GP Fault Detection — handled by segmentation_unit
assign gp_fault_mem_op = invlpg_active || x87_direct_mem_req ||
                         rd_fast_valid_r ||
                         (uc_is_mem_busop && (uc_buscode != BUSOP_RD_D));
assign gp_fault_wr_op = rd_fast_valid_r || uc_is_write ||
                        uc_is_check_write;

// The sequencer consumes the execution, fault, and instruction-boundary
// commands above and owns the microcode ROM pipeline and address arbitration.
microsequencer microsequencer_inst (
    // Clock and reset
    .clk(clk),
    .reset_n(reset_n),
    // ROM port B (D2): the D2 skeleton's first word, read when the skeleton loads (US5293592 latches 35)
    .pb_load(pb_load),
    .pb_load_entry(pb_load_entry),
    .pb_kill(boundary_take || interrupt_entry),
    .pb_valid(pb_valid),
    // D2 -> EX issue: the issuing instruction's first word comes from port B (pb_slot)
    .i_issue(i_issue),
    .pb_slot(pb_slot),
    .issue_entry(issue_entry),
    .d2_kind(d2_kind),
    // ROM port A (EX): the sequencer's address and ROM pipeline
    .rom_base_ce(microcode_rom_base_ce),
    .rom_q_ce(microcode_rom_ce),
    .q_hold(rom_q_hold),
    .seq_advance(seq_advance),
    .uaddr(uaddr),
    .uaddr_next(uaddr_next),
    .uc_addr(uc_addr),
    .uc_addr_mem(uc_addr_mem_r),
    // Execution control (event control and the execution core)
    .q_flush(q_flush),
    .stall(stall),
    .uc_exec(uc_exec),
    .repeat_active(repeat_active),
    .macro_active(uc_active),
    .instr_eip_written(instr_eip_written),
    .any_fault(any_fault),
    .page_fault(page_fault),
    // Micro-branch conditions and redirect sources (protection, recipes, divide, faults, boundaries)
    .conditions(seq_conditions),
    .pe(pe),
    .vm(vm),
    .cpl_nonzero(cpl != 2'b00),
    .prot_redirect_prev(prot_redirect_prev),
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
    // End of instruction: RNI, synthetic RNIs and the delay slot
    .jcc_fold_active(jcc_fold_active),
    .branch_ustep_rni(branch_ustep_rni),
    .load_wb_retire(direct_wb_retire),
    .i_rni(i_rni_raw),
    .i_rni_delay(i_rni_delay),
    .i_rni_delay_ea(i_rni_delay_ea),
    .jump_taken_prev(uc_jump_taken_prev),
    .pref_suppress_prev(uc_pref_suppress_prev),
    // Microinstruction to the units (port A's ROM output, predecoded)
    .uc(uc),
    .uc_next(uc_next),
    .uc_source_shift(uc_source_shift),
    .uc_shift_source_class(uc_shift_source_class),
    .uc_shift2_source(uc_shift2_source),
    .uc_is_shift2(uc_is_shift2),
    .uc_shift_uc_carry(uc_shift_uc_carry),
    .uc_alu_src_shift(uc_alu_src_shift),
    .uc_aluop_shift(uc_aluop_shift),
    .uc_shift_sigma_sel(uc_shift_sigma_sel),
    .uc_alu_op_sel(uc_alu_op_sel),
    .uc_dly_source(uc_dly_source),
    .uc_mem_ctrl(uc_mem_ctrl),
    .uc_ind_ctrl(uc_ind_ctrl),
    .uc_fpu_f8(uc_fpu_f8),
    .uc_force_word(uc_force_word),
    .uc_ctl_pref(uc_ctl_pref)
);

// synthesis translate_off
always @(posedge clk)
    if (reset_n && pb_slot && i_issue && (issue_entry != i_bus.entry_point))
        $fatal(1, "port-B issue entry %03x differs from the D2 entry %03x",
               issue_entry, i_bus.entry_point);
// synthesis translate_on

// Instruction Signals (latched at i_issue)
always_ff @(posedge clk) begin
    if (!reset_n) begin
        i <= '0;
    end else if (i_issue) begin
        i <= i_bus;
        i.entry_point <= issue_entry;
    end
    if (interrupt_entry)
        i.rel_branch_kind <= REL_BRANCH_NONE;
end



//=============================================================================
// Unit 8: Same-cycle architectural commit
//=============================================================================

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
        (uc_dest == DEST_EIP || uc_dest == DEST_eIP || uc_dest == DEST_IP ||
         uc_dest == DEST_USTEP_RPTI_EIP) &&
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
    end else if (ret_redirect) begin
        EIP <= vipt_load_wb_target_r;
    end else if (i_issue && !halted /*&& (~uc_active || i_rni_delay)*/) begin
        // An issue can land on a control transfer's final word.
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
    end else if (uc_exec && (uc_dest == DEST_EIP || uc_dest == DEST_eIP ||
                            uc_dest == DEST_IP || uc_dest == DEST_USTEP_RPTI_EIP)) begin
        // Microcode destination write to EIP.
        if (uc_dest == DEST_EIP || uc_dest == DEST_USTEP_RPTI_EIP) begin
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
        op_size_du <= 2'd1;
        op_size_dw <= 2'd1;
        srcreg_size_src <= 2'd1;
    end else if (i_issue && !halted) begin
        // Instruction start: widths have already been resolved in D1.
        op_size <= i_bus.operand_size;
        op_size_decode <= i_bus.operand_size;
        op_size_src <= i_bus.operand_size;
        op_size_du <= i_bus.operand_size;
        op_size_dw <= i_bus.operand_size;
        op_size_src_decode <= i_bus.operand_size;
        srcreg_size <= i_bus.source_size;
        srcreg_size_decode <= i_bus.source_size;
        srcreg_size_src <= i_bus.source_size;
        srcreg_size_src_decode <= i_bus.source_size;
    end else if (uc_exec) begin
        // Microcode BITS operations
        case (uc_aluop)
            ALUJMP_BITS8:  begin op_size <= 2'd0; srcreg_size <= 2'd0; op_size_src <= 2'd0; op_size_du <= 2'd0; op_size_dw <= 2'd0; srcreg_size_src <= 2'd0; end
            ALUJMP_BITS16: begin op_size <= 2'd1; srcreg_size <= 2'd1; op_size_src <= 2'd1; op_size_du <= 2'd1; op_size_dw <= 2'd1; srcreg_size_src <= 2'd1; end
            ALUJMP_BITS32: begin op_size <= 2'd2; srcreg_size <= 2'd2; op_size_src <= 2'd2; op_size_du <= 2'd2; op_size_dw <= 2'd2; srcreg_size_src <= 2'd2; end
            ALUJMP_BITSDE: begin
                op_size <= op_size_decode;
                srcreg_size <= srcreg_size_decode;
                op_size_src <= op_size_src_decode;
                op_size_du <= op_size_src_decode;
                op_size_dw <= op_size_src_decode;
                srcreg_size_src <= srcreg_size_src_decode;
            end
            default: ;
        endcase
    end
end

// GPR and internal registers
always_ff @(posedge clk) begin
    automatic logic [31:0] external_dest_value;
    external_dest_value = dest_value;
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
        for (int n = 0; n < 4; n++) DR03[n] <= 32'h0;

    end else begin
        if (uc_exec) begin
        if (uc_source == SRC_IRF2)
            external_dest_value = IND;  // use combinational IRF2

        // Only the named-GPR destinations the microcode uses are decoded.
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
                // Ordinary protected-mode control transfers retain CPL. Task
                // loading uses DEST_USTEP_TASK_CS to establish a new RPL.
                if (pe && !vm)
                    CS[15:2] <= cs_source_value[15:2];
                else
                    CS <= cs_source_value;
            end
            DEST_USTEP_TASK_CS: CS <= cs_source_value;
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
                if (irf_is_dr && !COUNTR[2] && uc_buscode == BUSOP_SBAS)
                    DR03[COUNTR[1:0]] <= external_dest_value;
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
                case (i.seg_reg_sel)
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
    if (i_issue) begin
        TMPeIP <= EIP;
    end
    if (i_first)
        TMPeSP <= ESP;  // instruction-start ESP for a restartable fault frame

    // Chained-store fault attribution: capture the restart IP at every demand WRITE issue
    // and ESP: a PUSH whose write faults restarts with the ESP it started with,
    // not that of a successor issued meanwhile. In the writer's first cycle
    // TMPeSP is still loading, so take ESP directly. A check-write (CW, e.g.
    // ENTER's probe of the new stack) faults with a write code too.
    if (mem_req_to_paging && (mem_write_now || paging_is_write_access) && mem_accepted) begin
        wr_restart_eip <= TMPeIP;
        wr_restart_esp <= i_first ? ESP : TMPeSP;
    end
    if (ucrd_take) begin
        ucrd_restart_eip <= TMPeIP;
        ucrd_restart_esp <= i_first ? ESP : TMPeSP;
    end
    if (page_fault && pg_fault_code[1]) begin
        TMPeIP <= wr_restart_eip;
        TMPeSP <= wr_restart_esp;
    end else if (data_page_fault && ucrd_slow_wait_r) begin
        TMPeIP <= ucrd_restart_eip;
        TMPeSP <= ucrd_restart_esp;
    end else if (data_page_fault && vipt_load_slow_wait_r) begin
        TMPeIP <= vipt_load_slow_r.restart_eip;
        // A direct POP has already written ESP; its fault restarts from the
        // ESP it started with, even if a younger instruction since issued.
        if (vipt_load_slow_r.restore_esp)
            TMPeSP <= vipt_load_slow_r.esp_restore;
    end
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



// Derive control signals from ALU opcode
// INC=11000, DEC=11001, INC2=11100, DEC2=11101: all have op[4:3]==11 && op[1]==0
wire alu_update_carry = !(alu_op5[4:3] == 2'b11 && !alu_op5[1]);
assign alu_op5 = uc_alu_op_sel[6] ? i.decoded_alu_op :
                 uc_alu_op_sel[5] ? (i.cmptest_is_cmp ? ALU_CMP : ALU_AND) :
                                    uc_alu_op_sel[4:0];

// IMUL: F6.5, F7.5, 0FAF, 69, 6B; MUL: F6.4 and F7.4.
wire is_signed_mul = i.mul_signed;
wire clear_rf = (i_rni_delay &&
                 i.boundary_action != BOUNDARY_ACTION_PRESERVE_RF) ||
                (recipe_rni && uc_exec);

// synthesis translate_off
// Reference: the ALU op from the raw ALU/jump field.
function automatic [4:0] map_alu_op(input [6:0] uc_op);
begin
    casez (uc_op)
        ALUJMP_ALU,
        ALUJMP_INCDEC: map_alu_op = i.decoded_alu_op;
        ALUJMP_SHIFT1: map_alu_op = ALU_PASS;
        ALUJMP_CMPTST: map_alu_op = i.cmptest_is_cmp ? ALU_CMP : ALU_AND;
        ALUJMP_SZ_EXT: map_alu_op = i.decoded_alu_op;
        ALUJMP_AND:    map_alu_op = ALU_AND;
        ALUJMP_OR:     map_alu_op = ALU_OR;
        ALUJMP_XOR:    map_alu_op = ALU_XOR;
        ALUJMP_SIGN:   map_alu_op = ALU_SIGN;
        ALUJMP_ADD:    map_alu_op = ALU_ADD;
        ALUJMP_ADC:    map_alu_op = ALU_ADC;
        ALUJMP_SUB:    map_alu_op = ALU_SUBT;
        ALUJMP_CMP:    map_alu_op = ALU_CMP;
        ALUJMP_SHIFT,
        ALUJMP_USTEP_AAD_SHIFT,
        ALUJMP_SHIFT2: map_alu_op = ALU_PASS;
        ALUJMP_PASS2:  map_alu_op = ALU_PASS2;
        ALUJMP_AAAAAS: map_alu_op = i.decoded_alu_op;
        ALUJMP_BITS16: map_alu_op = ALU_PASS;
        ALUJMP_DAADAS: map_alu_op = i.decoded_alu_op;
        ALUJMP_PASS,
        ALUJMP_JMP,
        ALUJMP_NOPMOVE: map_alu_op = ALU_PASS;
        ALUJMP_SERECO: map_alu_op = i.decoded_alu_op;
        default: map_alu_op = ALU_PASS;
    endcase
end
endfunction
always_ff @(posedge clk)
    if (reset_n && (^uc_aluop_shift !== 1'bx) && (alu_op5 !== map_alu_op(uc_aluop_shift)))
        $fatal(1, "ALU OP SELECT MISMATCH: %h vs %h", alu_op5, map_alu_op(uc_aluop_shift));
// synthesis translate_on

data_unit data_unit_inst (
    // Clock and reset
    .clk(clk),
    .reset_n(reset_n),
    // Microsequencer: current microword fields and E-stage enables
    .exec(uc_exec),
    .shift_exec(uc_exec_shift),
    .pipeline_advance(!stall),
    .repeat_active(repeat_active),
    .aluop(uc_aluop),
    .alu_operation(alu_op5),
    .update_arch_flags(alu_update_flags),
    .update_carry(alu_update_carry),
    .dest(uc_dest),
    .source_field(uc_source_shift),
    .source_live(uc_source),
    .alu_source(uc_alu_src_shift),
    .alu_source_live(uc_alu_src),
    .fpu_f8(uc_fpu_f8),
    .shift_aluop(uc_aluop_shift),
    .shift_sigma_sel(uc_shift_sigma_sel),
    .shift_source_class(uc_shift_source_class),
    .shift2_source(uc_shift2_source),
    .shift_is_shift2(uc_is_shift2),
    .shift2_capture_ce(microcode_rom_ce),
    .shift2_next_valid(uc_next_captures_shift_source),
    .shift2_next_source(uc_next_shift2_source),
    .shift_uc_carry(uc_shift_uc_carry),
    // Event control: instruction lifecycle, faults and interrupt delivery
    .instr_start(i_issue),
    .uc_active(uc_active),
    .halted(halted),
    .ifetch_page_fault(ifetch_page_fault),
    .interrupt_entry(interrupt_entry),
    .any_fault(any_fault_r),
    .clear_rf(clear_rf),
    .gate_detect(gate_detect_now),
    .flags_backup_active(flags_backup_active),
    // Decoder: EX and D2 instruction, operand sizes, stack-operation class (ispval)
    .instr(i),
    .next_instr(i_bus),
    .op_size(op_size_du),
    .srcreg_size(srcreg_size_src),
    .op_size_src(op_size_src),
    .srcreg_size_src(srcreg_size_src),
    .is_dword(is_dword),
    .is_signed_mul(is_signed_mul),
    .stack_op(i_bus.stack_op),
    .stack_dir(i_bus.stack_dir),
    .stack_data32(i_bus.data32),
    .stack32(desc_cache[SEG_SS].D_B),
    // Hardwired control: recipe state and deferred recipe commits
    .recipe_rni(recipe_rni),
    .recipe_state(recipe_state),
    .recipe_commit_cancel(any_fault_issue),
    .recipe_shift_write(recipe_shift_write),
    .recipe_shift_data(recipe_shift_data),
    .recipe_memory_write(recipe_mem_write),
    // Load pipeline: registered VIPT load write-back into the register file
    .load_wb_valid(vipt_load_wb_valid_r),
    .load_issue(vipt_issue_load),
    .load_pipe_flush(q_flush || any_fault || interrupt_entry),
    .load_wb_dst(vipt_load_wb_dst_r),
    .load_wb_size(vipt_load_wb_size_r),
    .load_wb_data(vipt_load_wb_data),
    .load_wb_is_alu(vipt_load_wb_is_alu_r),
    .load_wb_fwd_mask(vipt_load_wb_fwd_mask_r),
    .load_wb_alu_op(vipt_load_wb_alu_op_r),
    .load_alu_dst_capture(vipt_load_alu_dst_capture),
    .load_alu_dst_capture_dst(vipt_load_alu_dst_capture_dst),
    .load_alu_dst_capture_size(vipt_load_alu_dst_capture_size),
    .load_alu_dst_capture_data(vipt_load_alu_dst_capture_data),
    // Shorters (US5142635): bypasses around the register file
    .dly_gpr_forward(dly_gpr_forward),
    .eflags_fwd(eflags_fwd),
    .branch_condition_true(branch_condition_true),
    // Segmentation: I-bus base/index reads and address registers
    .ea_base(ea_base_ref),
    .ea_index(ea_index_ref),
    .ea_base_value(ea_base_value),
    .ea_index_value(ea_index_value),
    .forwarded_esp(forwarded_esp),
    .pend_write_mask(pend_write_mask),
    .x87_reg_commit(uc_exec && i_first && x87_direct_reg_taken &&
                    (i.ucode_action == RECIPE_ACTION_X87_OVERLAY)),
    .x87_store_commit(x87_store_opr_commit),
    .ind(IND),
    .ea(ea_reg),
    // Segmentation and protection: selectors, descriptor and protection sources
    .es(ES),
    .cs(CS),
    .ss(SS),
    .ds(DS),
    .fs(FS),
    .gs(GS),
    .ldtr(LDTR),
    .tr(TR),
    .seg_reg_sel(i.seg_reg_sel),
    .desc_raw_hi(desc_raw_hi),
    .slctr(SLCTR),
    .protun(PROTUN),
    .pe(pe),
    .cpl(cpl),
    .protection_source_value(protun_write_value),
    .protection_source_low16_nonzero(protun_write_low16_nonzero),
    .cs_source_value(cs_source_value),
    // Cache and bus unit: memory operand in (R bus) and write data out
    .opr_r(OPR_R),
    .opr_w(OPR_W),
    .memory_write_source_value(memory_write_source_value),
    // Control registers and restart state read as microcode sources
    .eip(EIP),
    .cr0(CR0),
    .cr2(CR2),
    .dr6(DR6),
    .dr7(DR7),
    .tmpeip(TMPeIP),
    .tmpesp(TMPeSP),
    // Register file, internal registers and flags (datapath state)
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
    .countr(COUNTR),
    .eflags(EFLAGS),
    .uc_flags(uc_flags),
    .flags_backup(FLAGSB),
    // E-stage results
    .sigma(SIGMA),
    .alu_result(alu_result),
    .shift_result(shift_result),
    .muldiv_result(muldiv_result),
    .div_overflow(div_overflow),
    .alu_src(alu_src),
    .alu_src_hold(alu_src_r),
    .source_value_live(source_value_live),
    .alu_source_value_live(alu_src_data),
    .dest_value(dest_value)
);

// Debug tap (read by tb_z486 hierarchically; not used in the core).
wire use_shifter_result = (uc_aluop == ALUJMP_SHIFT2) ||
                          (uc_aluop == ALUJMP_SHIFT) ||
                          (uc_aluop == ALUJMP_USTEP_AAD_SHIFT);


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
    .req_wdata(x87_req_wdata),
    .req_accepted(x87_req_accepted),
    .req_complete(x87_req_complete),
    .req_read_complete(x87_read_complete),
    .req_rdata(x87_rdata),
    .direct_launch(i_issue),
    .direct_candidate(x87_direct_candidate),
    .direct_allowed(!CR0[3] && !CR0[2] && !x87_off),
    .direct_fop(i.fop),
    .direct_reg(i_bus.modrm[7:6] == 2'b11),
    .direct_store((i_bus.modrm[7:6] != 2'b11) && ((i_bus.opcode == 8'hD9) || (i_bus.opcode == 8'hDB)) &&
                  ((i_bus.modrm[5:3] == 3'd2) || (i_bus.modrm[5:3] == 3'd3))),
    .direct_data32(i_bus.data32),
    .store_word(uc_active && (uc_dest == DEST_USTEP_X87_STORE)),
    .store_go(uc_active && (uc_dest == DEST_USTEP_X87_STORE) && !stall_mem),
    .store_opr_commit(x87_store_opr_commit),
    .store_opr_data(x87_store_opr_data),
    .store_hold(x87_store_hold),
    .direct_taken(x87_direct_taken),
    .direct_reg_taken(x87_direct_reg_taken),
    .direct_active(x87_direct_active),
    .direct_mem_req(x87_direct_mem_req),
    .direct_stall(stall_x87_direct),
    .mem_accepted(mem_accepted),
    .mem_addr_low(ind_linear[1:0]),
    .mem_read_complete(mem_read_complete || (ucrd_hit && ucrd_x87_r)),
    .mem_servicing(mem_servicing),
    .mem_rdata((ucrd_hit && ucrd_x87_r) ? dcache_vipt_resolve_data : dcache_rdata),
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

interrupt_controller interrupts (
    .clk(clk),
    .reset_n(reset_n),
    .intr(intr),
    .nmi(nmi),
    .iflag(EFLAGS[9]),
    .i_rni(i_rni),
    .shadow_start(i_rni &&
                  ((i.boundary_action == BOUNDARY_ACTION_STI) ||
                   (i.boundary_action == BOUNDARY_ACTION_LOAD_SS))),
    .uc_exec(uc_exec),
    .uc_aluop(uc_aluop),
    .nmi_accept_boundary(nmi_accept_boundary),
    .intr_pending(intr_pending),
    .nmi_request_active(nmi_request_active),
    .interrupt_pending(interrupt_pending),
    .inhibit_interrupts(inhibit_interrupts)
);

wire throttle_active_cycle = (i_issue && !throttle_parked_r) ||
                             (uc_active && !stall &&
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

// synthesis translate_off
// Memory write watch: +watch_lo=<hex> +watch_hi=<hex> logs every external
// memory write in [lo, hi) with the CS:EIP executing when it reaches the bus.
// +watch2_lo/+watch2_hi add a second range.
reg [31:0] watch_lo, watch_hi, watch2_lo, watch2_hi;
reg        watch_en = 1'b0, watch2_en = 1'b0;
initial begin
    if ($value$plusargs("watch_lo=%h", watch_lo) &&
        $value$plusargs("watch_hi=%h", watch_hi)) watch_en = 1'b1;
    if ($value$plusargs("watch2_lo=%h", watch2_lo) &&
        $value$plusargs("watch2_hi=%h", watch2_hi)) watch2_en = 1'b1;
end
always @(posedge clk)
    if (valid && ready && write && !io &&
        ((watch_en && ({addr, 2'b00} >= watch_lo) && ({addr, 2'b00} < watch_hi)) ||
         (watch2_en && ({addr, 2'b00} >= watch2_lo) && ({addr, 2'b00} < watch2_hi))))
        $display("%0t: WATCH wr %08h be=%b data=%08h CS:EIP=%04h:%08h vm=%b",
                 $time, {addr, 2'b00}, be, dout, CS, EIP, EFLAGS[17]);
// synthesis translate_on

endmodule
