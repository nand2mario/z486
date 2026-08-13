// Microcode ROM pipeline and control-flow state. Producers retain their
// timing-critical target generation; this unit owns final address arbitration.
module microsequencer
    import z486_pkg::*;
(
    input  logic        clk,
    input  logic        reset_n,

    input  logic        rom_base_ce,          // Base ROM pipeline clock-enable
    input  logic        q_flush,
    input  logic        d2_cancel,
    input  logic        d2_start,             // D2 presents a new entry point
    input  logic [11:0] d2_start_entry,       // PLA/recipe entry selected in D1/D2
    input  logic        d2_valid,
    input  logic        i_issue,              // D2 transfers its instruction to EX

    input  logic        seq_advance,          // Advance sequencer and ROM pipeline
    input  logic        macro_entry_valid,    // Launch decoded macro instruction
    input  logic        chain_entry_valid,     // Start a chained successor entry
    input  logic        uc_exec,              // Current micro-op may take effect
    input  logic        repeat_active,
    input  logic        prot_redirect_prev,   // Protection redirect delay-slot state
    input  logic        jcc_fold_active,      // Folded Jcc supplies synthetic RNI
    input  logic        branch_ustep_exec,    // Hardwired branch supplies synthetic RNI
    input  logic        macro_active,
    input  logic        instr_eip_written,
    input  logic        any_fault,
    input  logic        stall,
    input  logic        page_fault,
    input  logic        pe,
    input  logic        vm,
    input  logic        cpl_nonzero,
    input  logic        desc_accessed_writeback,
    input  seq_condition_t conditions,       // Precomputed micro-branch conditions
    input  logic        prot_redirect_valid,
    input  logic [11:0] prot_redirect_target,
    input  logic        recipe_redirect_valid,
    input  logic [11:0] recipe_redirect_target,
    input  logic        set_rpl_redirect,
    input  logic        div_redirect_valid,
    input  logic [11:0] div_redirect_target,
    input  logic        gate_redirect,
    input  seq_redirect_t fault_redirect,     // Highest-priority fault target
    input  seq_redirect_t boundary_redirect,  // Interrupt/reset boundary target

    output logic [11:0] uaddr,                // Registered ROM request address
    output logic [11:0] uaddr_next,           // Arbitrated next ROM address
    output logic [11:0] uc_addr,              // Address of executing micro-op
    output logic [11:0] uc_addr_mem,          // Address in ROM memory stage
    output logic        i_rni_delay,           // RNI delay slot is executing
    output logic        jump_taken_prev,      // Micro-jump delay-slot state
    output logic        pref_suppress_prev,   // Taken conditional PREF suppression
    output logic        i_rni,

    output logic [11:0] d2_entry,             // Entry associated with resident D2 word
    output logic [2:0]  d2_kind,              // Resident recipe/microcode kind
    output logic        d2_rom_mem_resident,  // D2 entry reached ROM memory stage
    output logic        rom_q_ce,             // ROM output-register clock-enable

    output logic [50:0] uc,
    output logic [50:0] uc_next,              // ROM-early word, one cycle ahead of uc
    output logic [5:0]  uc_source_shift,
    output logic [3:0]  uc_shift_source_class,
    output logic [1:0]  uc_shift2_source,
    output logic        uc_is_shift2,
    output logic [5:0]  uc_alu_src_shift,
    output logic [6:0]  uc_aluop_shift,
    output logic [2:0]  uc_dly_source,
    output logic [8:0]  uc_mem_ctrl,
    output logic        uc_fpu_f8,
    output logic        uc_ctl_pref
);

logic        d2_rom_mem_r;
logic        d2_rom_q_r;
logic        d2_launch_id_r;
logic        d2_id_r;
logic        d2_rom_mem_id_r;
logic        d2_rom_q_id_r;
logic [2:0]  d2_rom_q_kind_r;
logic        d2_slot_prefetched_r;
logic [11:0] return_stack [0:3];
logic [1:0]  return_sp;

wire [5:0] uc_alu_src = uc[36:31];
wire [5:0] uc_source = uc[23:18];
wire [6:0] uc_aluop = uc[17:11];
wire [2:0] uc_opcode = uc[10:8];
wire [11:0] uc_ljump_target = {uc_source, uc_alu_src};
wire [11:0] return_target = return_stack[return_sp - 2'd1];
wire reljump_taken = uc_exec && !repeat_active &&
                     reljump_condition(uc_aluop, conditions) &&
                     !desc_accessed_writeback && !prot_redirect_prev &&
                     !jcc_fold_active && !branch_ustep_exec;
wire pref_suppress_taken = reljump_taken &&
    (uc_aluop == ALUJMP_JNcond || uc_aluop == ALUJMP_JCNTNZ ||
     uc_aluop == ALUJMP_JCNT1 || uc_aluop == ALUJMP_LOOPnE);
wire normal_jump_taken = reljump_taken || prot_redirect_valid ||
    recipe_redirect_valid || (uc_exec && (
    (uc_aluop == ALUJMP_LJMPP && pe && !vm) ||
    (uc_aluop == ALUJMP_LJMPNP && pe && cpl_nonzero) ||
    (uc_aluop == ALUJMP_LJMP86 && vm) ||
    (uc_aluop == ALUJMP_LCALL) ||
    ((uc_aluop == ALUJMP_LJUMP) && !prot_redirect_prev)));
wire flow_call = uc_exec && (uc_aluop == ALUJMP_LCALL);
wire flow_return = uc_exec && (uc_aluop == ALUJMP_RETURN);
wire rni_base = (((uc_opcode == 3'b000) || (uc_opcode == 3'b010)) &&
                 !jump_taken_prev) ||
                ((uc_opcode == 3'b001) && jump_taken_prev);
assign i_rni = rni_base || jcc_fold_active || branch_ustep_exec;

seq_redirect_t exec_redirect;

wire d2_rom_hold = d2_valid && d2_rom_mem_resident && !i_issue && !d2_cancel;
wire d2_delay_preload = d2_valid && d2_rom_mem_resident && i_issue &&
                        !d2_start && !d2_slot_prefetched_r;
wire rom_addr_ce = rom_base_ce && !d2_rom_hold;
assign rom_q_ce = rom_base_ce && !d2_rom_hold && !d2_cancel;

always_comb begin
    exec_redirect = '0;

    if (uc_exec) begin
        if (reljump_taken) begin
            exec_redirect.valid = 1'b1;
            exec_redirect.target = uc_addr + 12'd1 +
                                   {{6{uc_alu_src[5]}}, uc_alu_src};
        end

        case (uc_aluop)
            ALUJMP_LJMP86: if (vm) begin
                exec_redirect.valid = 1'b1;
                exec_redirect.target = uc_ljump_target;
            end
            ALUJMP_LJMPP: if (pe && !vm) begin
                exec_redirect.valid = 1'b1;
                exec_redirect.target = uc_ljump_target;
            end
            ALUJMP_LJMPNP: if (pe && cpl_nonzero) begin
                exec_redirect.valid = 1'b1;
                exec_redirect.target = uc_ljump_target;
            end
            ALUJMP_LCALL: begin
                exec_redirect.valid = 1'b1;
                exec_redirect.target = uc_ljump_target;
            end
            ALUJMP_LJUMP: if (!prot_redirect_prev) begin
                exec_redirect.valid = 1'b1;
                exec_redirect.target = uc_ljump_target;
            end
            ALUJMP_RETURN: begin
                exec_redirect.valid = 1'b1;
                exec_redirect.target = return_target;
            end
            default: ;
        endcase

        if (prot_redirect_valid) begin
            exec_redirect.valid = 1'b1;
            exec_redirect.target = prot_redirect_target;
        end
        if (recipe_redirect_valid) begin
            exec_redirect.valid = 1'b1;
            exec_redirect.target = recipe_redirect_target;
        end
        if (set_rpl_redirect) begin
            exec_redirect.valid = 1'b1;
            exec_redirect.target = 12'h5fb;
        end
        if (div_redirect_valid) begin
            exec_redirect.valid = 1'b1;
            exec_redirect.target = div_redirect_target;
        end
        if (gate_redirect) begin
            exec_redirect.valid = 1'b1;
            exec_redirect.target = 12'h5be;
        end
    end
end

always_comb begin
    uaddr_next = uaddr;
    if (seq_advance)
        uaddr_next = uaddr + 12'd1;
    if (d2_delay_preload)
        uaddr_next = d2_entry + 12'd1;
    if (macro_entry_valid)
        uaddr_next = d2_start_entry;
    if (exec_redirect.valid)
        uaddr_next = exec_redirect.target;
    if (chain_entry_valid)
        uaddr_next = d2_start_entry;
    if (fault_redirect.valid)
        uaddr_next = fault_redirect.target;
    if (boundary_redirect.valid)
        uaddr_next = boundary_redirect.target;
    if (!reset_n)
        uaddr_next = 12'h000;
end

wire [11:0] rom_addr = d2_delay_preload ? (d2_entry + 12'd1) : uaddr_next;
wire [50:0] rom_q;
wire [50:0] rom_q_early;
wire [2:0]  rom_kind_early;

ucode_rom microcode_rom_inst (
    .clk(clk),
    .addr_ce(rom_addr_ce),
    .q_ce(rom_q_ce),
    .addr(rom_addr),
    .q_early(rom_q_early),
    .q(rom_q),
    .q_kind_early(rom_kind_early),
    .q_shift_source(uc_source_shift),
    .q_shift_source_class(uc_shift_source_class),
    .q_shift2_source(uc_shift2_source),
    .q_is_shift2(uc_is_shift2),
    .q_shift_alu_src(uc_alu_src_shift),
    .q_shift_aluop(uc_aluop_shift),
    .q_dly_source(uc_dly_source),
    .q_mem_ctrl(uc_mem_ctrl),
    .q_fpu_f8(uc_fpu_f8)
);

assign uc = rom_q;
assign uc_next = rom_q_early;
assign d2_rom_mem_resident = d2_rom_mem_r && (d2_rom_mem_id_r == d2_id_r);
wire d2_rom_resident = d2_rom_q_r && (d2_rom_q_id_r == d2_id_r);
assign d2_kind = d2_rom_resident ? d2_rom_q_kind_r : rom_kind_early;
// Keep each macro entry tagged across q_mem and q so a replaced D2 entry
// cannot execute the predecessor's still-resident ROM word.
always_ff @(posedge clk) begin
    if (!reset_n || q_flush) begin
        d2_rom_mem_r <= 1'b0;
        d2_rom_q_r <= 1'b0;
        d2_entry <= 12'h000;
        d2_launch_id_r <= 1'b0;
        d2_id_r <= 1'b0;
        d2_rom_mem_id_r <= 1'b0;
        d2_rom_q_id_r <= 1'b0;
        d2_rom_q_kind_r <= 3'b000;
        d2_slot_prefetched_r <= 1'b0;
    end else begin
        if (rom_addr_ce) begin
            d2_rom_mem_r <= d2_start;
            if (d2_start) begin
                d2_entry <= d2_start_entry;
                d2_launch_id_r <= !d2_launch_id_r;
                d2_id_r <= !d2_launch_id_r;
                d2_rom_mem_id_r <= !d2_launch_id_r;
            end
        end
        if (rom_q_ce) begin
            d2_rom_q_r <= d2_rom_mem_r;
            d2_rom_q_id_r <= d2_rom_mem_id_r;
            d2_rom_q_kind_r <= rom_kind_early;
        end
        if (d2_start)
            d2_slot_prefetched_r <= 1'b0;
        else if (d2_delay_preload && rom_addr_ce)
            d2_slot_prefetched_r <= 1'b1;
        else if (i_issue)
            d2_slot_prefetched_r <= 1'b0;
    end
end

// Sequencer state updates retain the original in-block priority. RNI delay
// cancellation is last so a page fault cannot expose a stale delay slot.
always_ff @(posedge clk) begin
    if (!reset_n) begin
        uaddr <= 12'h000;
        uc_addr_mem <= 12'h000;
        uc_addr <= 12'h000;
        return_sp <= 2'd0;
        i_rni_delay <= 1'b0;
        jump_taken_prev <= 1'b0;
        pref_suppress_prev <= 1'b0;
        uc_ctl_pref <= 1'b0;
    end else begin
        uaddr <= uaddr_next;

        if (rom_addr_ce)
            uc_addr_mem <= rom_addr;
        if (rom_q_ce) begin
            uc_addr <= uc_addr_mem;
            uc_ctl_pref <= (rom_q_early[5:0] == BUSOP_PREF);
        end

        if (i_rni_delay && !stall && !page_fault)
            i_rni_delay <= 1'b0;
        if (uc_exec && i_rni && macro_active && !instr_eip_written &&
            !any_fault && !i_issue)
            i_rni_delay <= 1'b1;
        if (page_fault)
            i_rni_delay <= 1'b0;

        if (uc_exec) begin
            jump_taken_prev <= normal_jump_taken;
            pref_suppress_prev <= pref_suppress_taken;
        end

        if (flow_call) begin
            return_stack[return_sp] <= uaddr + 12'd1;
            return_sp <= return_sp + 2'd1;
        end else if (flow_return) begin
            return_sp <= return_sp - 2'd1;
        end
        if (gate_redirect) begin
            return_stack[return_sp] <= uc_addr;
            return_sp <= return_sp + 2'd1;
        end
    end
end

function automatic logic reljump_condition(
    input logic [6:0] aluop,
    input seq_condition_t c
);
    case (aluop)
        ALUJMP_JNcond: reljump_condition = c.jncond;
        ALUJMP_JCNTZ: reljump_condition = c.count_zero;
        ALUJMP_JCNTNZ, ALUJMP_JCNZNI: reljump_condition = c.count_nonzero;
        ALUJMP_JCT4N1: reljump_condition = c.count_low_not_one;
        ALUJMP_JCNTN1: reljump_condition = c.count_not_one;
        ALUJMP_JCNT1: reljump_condition = c.count_one;
        ALUJMP_LOOPnE: reljump_condition = c.loopne;
        ALUJMP_JG: reljump_condition = c.greater;
        ALUJMP_JNC: reljump_condition = c.no_carry;
        ALUJMP_JNO: reljump_condition = c.no_overflow;
        ALUJMP_JPEREQ: reljump_condition = c.pereq_inactive;
        ALUJMP_JNFLGB: reljump_condition = c.flags_backup_inactive;
        ALUJMP_JTSSAF: reljump_condition = c.tss_access;
        ALUJMP_JINTSW: reljump_condition = !c.interrupt_hw;
        ALUJMP_JMISC1: reljump_condition = c.misc1;
        ALUJMP_JEXTFT: reljump_condition = c.interrupt_hw;
        ALUJMP_JSTSKL: reljump_condition = !c.misc1 && !c.interrupt_hw;
        ALUJMP_JNTSKS: reljump_condition = c.task_unsaved;
        ALUJMP_JMISC2: reljump_condition = c.misc2;
        ALUJMP_JNERRC: reljump_condition = c.no_error_code;
        ALUJMP_JNOFLT: reljump_condition = c.no_fault;
        ALUJMP_JREP: reljump_condition = c.rep_fault;
        ALUJMP_JNT: reljump_condition = c.nested_task;
        ALUJMP_JIO_OK: reljump_condition = c.io_ok;
        ALUJMP_JMP: reljump_condition = 1'b1;
        ALUJMP_JNOINT: reljump_condition = c.no_interrupt;
        ALUJMP_JNBUSY: reljump_condition = c.x87_not_busy;
        ALUJMP_JBUSY: reljump_condition = c.x87_error;
        ALUJMP_JICEWT: reljump_condition = 1'b0;
        ALUJMP_J16BIT: reljump_condition = c.task_16bit;
        default: reljump_condition = 1'b0;
    endcase
endfunction

endmodule
