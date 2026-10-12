//
// Event Control
// Fault, trap and interrupt redirects of the microsequencer, and the instruction lifecycle
//
`include "z486_platform.svh"
module event_control
    import z486_pkg::*;
#(
    parameter ENABLE_X87 = 0
)
(
    // Clock and reset
    // Clock and reset
    input  logic clk,
    input  logic reset_n,
    input  logic x87_off,               // Dev menu: no coprocessor

    // Microsequencer and E-stage lifecycle (current microword and its enables)
    input  logic [11:0] uc_addr,
    input  logic [6:0] uc_aluop,
    input  logic [5:0] uc_buscode,
    input  logic [6:0] uc_dest,
    input  logic uc_exec,
    input  logic [31:0] uc_flags,
    input  logic uc_jpereq_fwd,
    input  logic stall,
    input  logic repeat_active,
    input  logic q_flush,
    input  logic d2_resident,      // an instruction occupies D2 (port A or port B)
    input  logic i_issue,
    input  logic i_first,
    input  logic i_rni,
    input  logic i_rni_delay,
    input  logic direct_wb_retire,
    input  logic rmw_fallback_delay_r,
    input  logic throttle_parked_r,
    input  logic x87_direct_taken,   // the x87 overlay went direct at issue

    // Decoder: D2 entry and the EX instruction register
    input  dec_entry_t i_bus,
    input  dec_entry_t i,

    // Datapath and architectural state
    input  logic [31:0] COUNTR,
    input  logic [15:0] CS,
    input  logic [31:0] EIP,
    input  logic [31:0] EFLAGS,
    input  logic [31:0] eflags_fwd,
    input  logic [31:0] IND,
    input  logic [31:0] alu_result,
    input  logic [31:0] ea_reg,
    input  logic is_dword,
    input  logic pe,
    input  logic vm,
    input  logic [1:0] cpl,
    input  logic pe_mode_toggle_now,
    input  logic branch_ustep_redirect,
    input  logic flags_backup_active,

    // Segmentation and protection test unit
    input  seg_desc_t desc_cache [0:7],
    input  logic [31:0] desc_raw_hi,
    input  logic tss_access_flag,

    // Fault requests (segmentation, paging, datapath)
    input  logic any_fault,
    input  logic any_fault_r,
    input  logic gp_fault_trigger,
    input  logic gp_fault_r,
    input  logic ss_segment_fault,
    input  logic ss_fault_r,
    input  logic page_fault,
    input  logic data_page_fault,     // the executing instruction's access (not a fetch)
    input  logic [2:0] pg_fault_code,
    input  logic [31:0] pg_cr2_out,
    input  logic div_overflow,

    // Interrupt controller and debug traps
    input  logic intr_pending,
    input  logic nmi_request_active,
    input  logic interrupt_pending,
    input  logic inhibit_interrupts,
    input  logic tf_trap_pending,
    input  logic single_step,
    input  logic boundary_point,     // an instruction boundary decides traps and interrupts now
    input  logic dbg_data_trap,      // an enabled data breakpoint matched in the last instruction
    input  logic dbg_data_trap_now,  // the same, counting this cycle's compare (JNOINT)
    input  logic dbg_exec_fault,     // an enabled instruction breakpoint at the next one, RF clear

    // FPU handshake
    input  logic x87_pereq,
    input  logic x87_busy_n,
    input  logic x87_error_n,
    input  logic x87_ne,             // CR0.NE: an x87 error raises #MF (vector 16)
    input  logic x87_ignne,          // IGNNE#: with NE=0, proceed past a pending error

    // To the microsequencer: conditions and redirect commands
    output logic seq_advance,
    output seq_condition_t seq_conditions,
    output seq_redirect_t seq_fault_redirect,
    output seq_redirect_t seq_boundary_redirect,
    output logic recipe_fallback_taken,
    output logic gate_detect_now,
    output logic gate_detect_cond,
    output logic double_fault_start,

    // Macro-instruction lifecycle and delivery state
    output logic uc_active,
    output logic halted,
    output logic instr_eip_written,
    output logic interrupt_entry,
    output logic fault_suppress_delay_slot,
    output logic tf_active_r,
    output logic tf_trap_suppress_r,
    output logic [2:0] latched_pf_code,
    output logic [31:0] latched_pf_addr,
    output logic misc2_flag,

    // Debug and board
    output logic dbg_first_done,
    output logic [31:0] debug_ip,
    output logic triple_fault_reset
);

wire x87_on = ENABLE_X87 && !x87_off;
// A 486 with CR0.NE=0 freezes only at a waiting FP instruction: WAIT and
// every ESC but the no-wait control forms (FNSTENV, FNSTCW, FNSAVE, FNSTSW,
// FNENI, FNDISI, FNCLEX, FNINIT, FNSETPM), which read the pending error.
wire [2:0] esc_group = i.opcode[2:0];
wire x87_no_wait =
    !i.has_0f && (i.opcode[7:3] == 5'b11011) &&
    ((((esc_group == 3'd1) || (esc_group == 3'd5)) && (i.modrm[7:6] != 2'b11) &&
      i.modrm[5]) ||                                   // /6, /7 memory forms
     ((esc_group == 3'd7) && (i.modrm == 8'hE0)) ||    // FNSTSW AX
     ((esc_group == 3'd3) && (i.modrm[7:3] == 5'b11100) &&
      (i.modrm[2:0] <= 3'd4)));                        // FNENI..FNSETPM
wire x87_frozen = !x87_error_n && !x87_ne && !x87_ignne && !x87_no_wait;


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
assign double_fault_start = (fault_delivery_state == FAULT_DELIVERING) &&
                                fault_combine_active;

wire [31:0] countr_masked = i.addr32 ? COUNTR : {16'h0, COUNTR[15:0]};

reg        misc1_flag;              // Set by SMISC1 {-33-}, tested by JMISC1 {-53-}
// misc2_flag: port
reg        error_code_flag;         // Set by SERRCF {-36-}, tested by JNERRC {-56-}
reg        interrupt_hw;            // Set for hardware interrupts, tested by JINTSW {-52-}
reg        task_saved_flag;         // STSKS/CTSKS latch: outgoing TSS has been saved during this switch
reg        no_fault_flag;           // SNOFLT/JNOFLT: descriptor probes fail by clearing ZF, not raising #GP
reg        rep_fault_flag;          // SREPF/CREPF/JREP: interrupted REP MOVS needs index/count correction
// instr_eip_written: port
reg        gate_in_progress;        // Prevent second LDTST (at 5C3) from re-triggering gate detection

// LOOP/REP Condition Logic
wire instr_is_loop = i.repeat_kind != REPEAT_KIND_REP;
wire loop_zf_sense = instr_is_loop ? (i.repeat_kind == REPEAT_KIND_LOOPE)
                                   : i.rep_lock[0];
wire countr_will_be_nonzero = instr_is_loop ? (countr_masked != 32'h1) : (countr_masked != 32'h0);
wire zf_check = instr_is_loop ? (loop_zf_sense == EFLAGS[6]) : (loop_zf_sense != EFLAGS[6]);
wire loopne_condition = instr_is_loop ? (countr_will_be_nonzero && zf_check)
                                      : (!countr_will_be_nonzero || zf_check);


always_comb begin
    seq_conditions = '0;
    seq_conditions.jncond = !condition_true(i.branch_condition, eflags_fwd);
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
    seq_conditions.pereq_inactive = x87_on ? !x87_pereq : uc_jpereq_fwd;
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
        (cpl <= EFLAGS[13:12] && (!vm || !i.port_io));
    // A data breakpoint stops a REP string after its iteration, as an
    // interrupt does (RPTI), and is taken at the following boundary.
    seq_conditions.no_interrupt = !interrupt_pending && !dbg_data_trap_now;
    // The 386 microcode takes ERROR# as #MF (FPU_ERRORn). A 486 does that only
    // with CR0.NE; with NE=0 (DOS-compatible) the error goes out as FERR# to
    // IRQ13 and a waiting FP instruction freezes, taking interrupts, until
    // IGNNE# (the IRQ13 handler's write to port F0h) lets it proceed.
    seq_conditions.x87_not_busy = x87_on ? x87_busy_n && !x87_frozen : 1'b1;
    seq_conditions.x87_error = x87_on && !x87_error_n && x87_ne;
    seq_conditions.task_16bit = !desc_cache[6].seg_type[3];
    seq_conditions.desc_accessed = desc_raw_hi[8];
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

// Qualified overlays launch without live architectural state on the ROM
// address. Their first ustep redirects unsafe cases to original microcode;
// the following overlay word is the architectural jump delay slot.
assign recipe_fallback_taken =
    (uc_exec && i_first &&
     (i.ucode_action == RECIPE_ACTION_X87_OVERLAY) &&
     !x87_direct_taken) || rmw_fallback_delay_r;
assign gate_detect_cond = pe && (uc_buscode == BUSOP_SDEL) &&
                          !gate_in_progress && !desc_raw_hi[12] &&
                          (desc_raw_hi[11:8] == 4'hC);
assign gate_detect_now = uc_exec && gate_detect_cond;

assign seq_advance = (((i_issue | uc_exec |
                        direct_wb_retire) |
                       (fault_suppress_delay_slot & !stall)) &
                      !halted && !repeat_active);

// Fault redirects override the port-B continuation. A page fault has priority over a
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
    if (boundary_point) begin
        // Debug traps of the last instruction, then the next instruction's
        // breakpoint fault, then NMI, then INTR (486 PRM 9.8).
        if ((tf_trap_pending || dbg_data_trap || dbg_exec_fault) && !single_step) begin
            seq_boundary_redirect.valid = 1'b1;
            seq_boundary_redirect.target = tf_trap_pending ? UADDR_SINGLE_STEP
                                                           : UADDR_BREAKPOINT;
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


wire fault_delivery_done = uc_exec &&
    ((uc_aluop == ALUJMP_USTEP_FAULT_DONE) ||
     (uc_dest == DEST_USTEP_FAULT_DONE));

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

        if (fault_delivery_done && !any_fault) begin
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
        if (fault_delivery_done)
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
        halted <= 1'b0;
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

        // Interrupt dispatch owns this registered cleanup cycle before the
        // first handler uStep can execute.  Clear the RPTI ownership marker
        // from that local pulse rather than extending its input mux with the
        // live interrupt-recognition cone.
        if (interrupt_entry)
            instr_eip_written <= 1'b0;

        if (i_rni_delay && !stall && !page_fault) begin
            dbg_first_done <= 1'b1;
            if (single_step)
                halted <= 1'b1;
            if (!i_issue)
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

            if (uc_dest == DEST_USTEP_RPTI_EIP)
                instr_eip_written <= 1'b1;

            if (i_rni && uc_active && instr_eip_written && !stall)
                uc_active <= 1'b0;  // RPTI restart

            if (gate_detect_now)
                gate_in_progress <= 1'b1;
        end

        if (direct_wb_retire && uc_active && !instr_eip_written && !any_fault)
            debug_ip <= EIP;

        fault_suppress_delay_slot <= any_fault || any_fault_r ||
                                     (fault_suppress_delay_slot && stall);

        if (i_issue) begin
            uc_active <= 1'b1;
            tf_active_r <= EFLAGS[8];
            tf_trap_suppress_r <=
                (i_bus.boundary_action == BOUNDARY_ACTION_LOAD_SS) ||
                (i_bus.boundary_action == BOUNDARY_ACTION_SOFT_INT) ||
                ((i_bus.boundary_action == BOUNDARY_ACTION_INTO) && EFLAGS[11]);
            instr_eip_written <= 1'b0;
            gate_in_progress <= 1'b0;
        end

        // A faulting instruction does not complete, so its single-step trap
        // is not taken: TF traps after the instruction re-executes. A fetch
        // fault belongs to an instruction that never issued and keeps the
        // previous one's trap.
        if (gp_fault_trigger || data_page_fault || div_overflow) begin
            tf_active_r <= 1'b0;
            tf_trap_suppress_r <= 1'b0;
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
        if (boundary_point) begin
            if ((tf_trap_pending || dbg_data_trap || dbg_exec_fault) && !single_step) begin
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
    if (reset_n && throttle_parked_r && !d2_resident)
        $fatal(1, "throttle parked without a resident D2 successor");

// RPTI marks its restarted instruction by writing EIP before presenting an
// interrupt boundary. That ownership must not leak into interrupt delivery,
// where it suppresses the delivery routine's normal completion boundary.
reg interrupt_entry_check_r;
always @(posedge clk)
    interrupt_entry_check_r <= interrupt_entry;
always @(posedge clk)
    if (reset_n && interrupt_entry_check_r && instr_eip_written)
        $fatal(1, "restart EIP ownership leaked into interrupt delivery");
// synthesis translate_on


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


endmodule
