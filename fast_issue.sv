// FAST instruction classification, chaining, hazard checks, and bounded
// branch-uStep control. Chain decisions remain combinational into the ROM.
module fast_issue
    import z386_pkg::*;
(
    input  logic        clk,
    input  logic        reset_n,
    input  dec_entry_t  pop_instr,       // Instruction entering execution
    input  dec_entry_t  next_instr,      // Following instruction for chain decision
    input  dec_entry_t  exec_instr,      // Instruction currently in execution
    input  ea_dec_t     pop_ea,          // Pop instruction EA dependencies
    input  ea_dec_t     next_ea,         // Following instruction EA dependencies
    input  logic        decq_has2,
    input  logic        decq_empty,
    input  logic        d2_push,
    input  logic [2:0]  d2_kind,
    input  logic        d2_valid,
    input  logic        d2_fire,
    input  logic        d2_waited,       // D2 already consumed a wait cycle
    input  logic        i_pop,
    input  logic        i_first,
    input  logic        uc_active,
    input  logic        uc_exec,
    input  logic        i_rni,
    input  logic        i_rni_delay,
    input  logic        uc_next_rni,
    input  logic [6:0]  uc_aluop,
    input  logic        alu_write_flags,
    input  logic [31:0] flags_live,      // Current-cycle condition flags
    input  logic [31:0] flags_ahead,     // Flags after pending ALU commit
    input  logic [1:0]  op_size,
    input  fast_pending_write_t mem_commit,   // Deferred load GPR write
    input  fast_pending_write_t shift_commit, // Deferred shift GPR write
    input  logic        q_flush,
    input  logic        interrupt_entry,
    input  logic        interrupt_pending,
    input  logic        trap_active,
    input  logic        single_step,
    input  logic        any_fault,
    input  logic        any_fault_r,
    input  logic        any_fault_pop,
    input  logic        throttle_hold,
    input  logic        stall,

    output fast_class_t pop_class,
    output logic        pop_fast,
    output logic        pop_x87,
    output logic        disabled,
    output logic        issue_valid,      // Launch a FAST instruction this cycle
    output logic        issue_from_next,  // Skip current queue head via chaining
    output logic [11:0] issue_entry,      // ROM entry for selected FAST recipe
    output logic        last,             // Current uStep retires the instruction
    output fast_exec_state_t state,       // Latched execution recipe
    output logic        dead_slot,        // Squashed FAST slot after redirect/fault
    output logic        fold_active,      // Jcc folded into predecessor delay slot
    output logic        branch_ustep_exec, // Execute hardwired branch uStep
    output logic        branch_redirect   // FAST branch redirects the frontend
);

fast_class_t next_class;
logic branch_ustep_r;
logic branch_ustep_jcc_r;
logic jcc_fold_r;
logic jcc_pop_taken_r;
logic jcc_pop_valid_r;
logic fast_off = 1'b0;

// synthesis translate_off
initial if ($test$plusargs("z386x_fast_off")) fast_off = 1'b1;
// synthesis translate_on

assign pop_class = recipe_fast_class(pop_instr);
assign disabled = fast_off;
assign next_class = recipe_fast_class(next_instr);
assign pop_fast = pop_class.fast && !fast_off;
assign pop_x87 = pop_class.commit_sel == FAST_COMMIT_X87;
wire active = i_first && state.fast;
assign last = state.fast && uc_active && i_rni;

function automatic logic [2:0] wide_widx(input logic [2:0] sel,
                                         input logic is_byte);
    wide_widx = is_byte ? {1'b0, sel[1:0]} : sel;
endfunction

function automatic logic gpr_overlap(input logic [2:0] a,
                                     input logic a_byte,
                                     input logic [2:0] b,
                                     input logic b_byte);
    if (!a_byte && !b_byte)      gpr_overlap = (a == b);
    else if (a_byte && b_byte)   gpr_overlap = (a[1:0] == b[1:0]);
    else if (a_byte)             gpr_overlap = !b[2] && (a[1:0] == b[1:0]);
    else                         gpr_overlap = !a[2] && (b[1:0] == a[1:0]);
endfunction

function automatic logic ea_conflict(input logic we,
                                     input logic [2:0] widx,
                                     input ea_dec_t ea,
                                     input dec_entry_t entry,
                                     input logic ignore_esp);
    ea_conflict = we &&
        (ea.base_sel[widx] || ea.index_sel[widx] ||
         (entry.stack_op && (widx == 3'd4) && !ignore_esp));
endfunction

function automatic logic hazard_uses(input ea_dec_t ea,
                                     input dec_entry_t entry,
                                     input fast_class_t fc,
                                     input logic hazard,
                                     input logic [2:0] widx,
                                     input logic [2:0] hreg,
                                     input logic [1:0] hsize);
    hazard_uses = hazard &&
        (ea.base_sel[widx] || ea.index_sel[widx] ||
         (fc.reads_dst && gpr_overlap(hreg, hsize == 2'd0,
                                      entry.dst_reg_sel, fc.op_byte)) ||
         (fc.reads_src && gpr_overlap(hreg, hsize == 2'd0,
                                      entry.src_reg_sel, fc.op_byte)) ||
         (fc.reads_ecx && gpr_overlap(hreg, hsize == 2'd0, 3'd1, 1'b0)) ||
         (entry.stack_op && (widx == 3'd4)));
endfunction

wire pred1_we = (pop_class.commit_sel == FAST_COMMIT_ALU) ||
                (pop_class.commit_sel == FAST_COMMIT_ESP) ||
                pop_class.writes_srcreg;
wire [2:0] pred1_widx = (pop_class.commit_sel == FAST_COMMIT_ESP) ? 3'd4 :
    wide_widx(pop_class.writes_srcreg ? pop_instr.src_reg_sel
                                      : pop_instr.dst_reg_sel,
               !pop_class.writes_srcreg && pop_class.op_byte);
wire ea1_conflict = ea_conflict(pred1_we, pred1_widx, next_ea, next_instr,
                                pop_class.commit_sel == FAST_COMMIT_ESP);

wire pred2_we = (state.commit_sel != FAST_COMMIT_NONE) &&
                (state.commit_sel != FAST_COMMIT_X87);
wire [2:0] pred2_widx = (state.commit_sel == FAST_COMMIT_SIGSRC)
    ? exec_instr.src_reg_sel
    : wide_widx(exec_instr.dst_reg_sel, op_size == 2'd0);
wire ea2_conflict = ea_conflict(pred2_we, pred2_widx, pop_ea, pop_instr, 1'b0);

wire mem_set = last && uc_exec && (state.commit_sel == FAST_COMMIT_MEM);
wire mem_hazard = mem_set || mem_commit.valid;
wire [2:0] mem_hreg = mem_set ? exec_instr.dst_reg_sel : mem_commit.dst;
wire [1:0] mem_hsize = mem_set ? op_size : mem_commit.size;
wire [2:0] mem_widx = wide_widx(mem_hreg, mem_hsize == 2'd0);
wire mem_conf1 = hazard_uses(next_ea, next_instr, next_class, mem_hazard,
                             mem_widx, mem_hreg, mem_hsize);
wire mem_confN = hazard_uses(pop_ea, pop_instr, pop_class, mem_hazard,
                             mem_widx, mem_hreg, mem_hsize);

wire shift_set = last && uc_exec && (state.commit_sel == FAST_COMMIT_SHIFT);
wire shift_hazard = shift_set || shift_commit.valid;
wire [2:0] shift_hreg = shift_set ? exec_instr.dst_reg_sel : shift_commit.dst;
wire [1:0] shift_hsize = shift_set ? op_size : shift_commit.size;
wire [2:0] shift_widx = wide_widx(shift_hreg, shift_hsize == 2'd0);
wire shift_conf1 = hazard_uses(next_ea, next_instr, next_class, shift_hazard,
                               shift_widx, shift_hreg, shift_hsize);
wire shift_confN = hazard_uses(pop_ea, pop_instr, pop_class, shift_hazard,
                               shift_widx, shift_hreg, shift_hsize);

wire next_ok = decq_has2 && next_class.fast &&
    (!next_class.reads_flags || !pop_class.writes_flags || next_class.jcc) &&
    (!next_class.uses_ea || !ea1_conflict) && !mem_conf1 && !shift_conf1;

wire loaduse_conflict =
    (pop_class.reads_dst && gpr_overlap(exec_instr.dst_reg_sel,
        op_size == 2'd0, pop_instr.dst_reg_sel, pop_class.op_byte)) ||
    (pop_class.reads_src && gpr_overlap(exec_instr.dst_reg_sel,
        op_size == 2'd0, pop_instr.src_reg_sel, pop_class.op_byte)) ||
    (pop_class.reads_ecx && gpr_overlap(exec_instr.dst_reg_sel,
        op_size == 2'd0, 3'd1, 1'b0));
wire head_ok = !decq_empty && d2_push && pop_class.fast &&
    (!pop_class.reads_flags || !state.writes_flags || pop_class.jcc) &&
    (!pop_class.uses_ea || !ea2_conflict) &&
    !((state.commit_sel == FAST_COMMIT_MEM ||
       state.commit_sel == FAST_COMMIT_SHIFT) &&
      loaduse_conflict) && !mem_confN && !shift_confN;

wire jcc_unsafe = uc_exec && ((uc_aluop == ALUJMP_SHIFT2) ||
                              (uc_aluop == ALUJMP_SEZF));
wire fold_now = i_pop && pop_fast && pop_class.jcc &&
                !(alu_write_flags || jcc_unsafe) &&
                !condition_true(pop_instr.opcode[3:0], flags_live);
assign fold_active = jcc_fold_r && i_first;

wire issue1 = i_pop && pop_fast &&
    ((!pop_class.multi_word && !pop_class.jcc) || fold_now) && next_ok;
wire issueN = state.fast && state.multi_word && uc_exec && uc_next_rni &&
    !i_pop && !i_rni_delay && !q_flush && head_ok;
wire issueJ = active && state.jcc && uc_exec && jcc_pop_valid_r &&
    !jcc_pop_taken_r && !q_flush && head_ok && !jcc_fold_r;
assign issue_valid = (issue1 || issueN || issueJ) &&
    (!d2_valid || d2_fire) && !d2_waited && !throttle_hold &&
    !interrupt_pending && !trap_active && !single_step && !any_fault_pop;
assign issue_from_next = issue1;
assign issue_entry = issue1 ? next_instr.entry_point : pop_instr.entry_point;

assign branch_ustep_exec = i_first && branch_ustep_r && uc_exec;
assign branch_redirect = branch_ustep_exec &&
                         (!branch_ustep_jcc_r || jcc_pop_taken_r);

always_ff @(posedge clk) begin
    if (!reset_n) begin
        state <= '0;
        branch_ustep_r <= 1'b0;
        branch_ustep_jcc_r <= 1'b0;
        jcc_fold_r <= 1'b0;
        jcc_pop_taken_r <= 1'b0;
        jcc_pop_valid_r <= 1'b0;
    end else begin
        if (q_flush || interrupt_entry || any_fault) begin
            jcc_fold_r <= 1'b0;
            jcc_pop_valid_r <= 1'b0;
        end else if (i_pop) begin
            jcc_fold_r <= fold_now;
            jcc_pop_valid_r <= pop_class.jcc && !jcc_unsafe;
            jcc_pop_taken_r <= condition_true(pop_instr.opcode[3:0], flags_ahead);
        end else if (!stall) begin
            jcc_fold_r <= 1'b0;
        end

        if (i_pop) begin
            state.fast <= pop_fast;
            state.multi_word <= pop_fast && pop_class.multi_word;
            state.jcc <= pop_fast && pop_class.jcc;
            state.writes_flags <= pop_class.writes_flags;
            state.commit_sel <= pop_class.commit_sel;
            state.keep_slot <= pop_class.keep_slot;
            branch_ustep_r <= pop_fast &&
                ((d2_kind == RECIPE_EARLY_BRANCH) || pop_instr.opcode == 8'hE8) &&
                pop_instr.data32 && (!pop_instr.stack_op || pop_instr.opcode == 8'hE8) &&
                (!pop_class.jcc || !jcc_unsafe);
            branch_ustep_jcc_r <= pop_class.jcc;
        end
        if (any_fault || any_fault_r || interrupt_entry) begin
            state.fast <= 1'b0;
            state.multi_word <= 1'b0;
            state.jcc <= 1'b0;
            state.commit_sel <= FAST_COMMIT_NONE;
            branch_ustep_r <= 1'b0;
        end
    end
end

always_ff @(posedge clk) begin
    if (!reset_n)
        dead_slot <= 1'b0;
    else if (!stall)
        dead_slot <= last && uc_exec && !i_pop && !state.keep_slot;
end

// synthesis translate_off
always @(posedge clk)
    if (reset_n && fold_active && uc_exec &&
        condition_true(exec_instr.opcode[3:0], flags_live))
        $display("%0t JCC-FOLD MISMATCH: opcode=%02x flags=%08x",
                 $time, exec_instr.opcode, flags_live);

int unsigned ds_total, ds_empty, ds_seq, ds_flags, ds_ea, ds_memc;
int unsigned ds_intr, ds_other, ds_other_1w, ds_keepslot;
always @(posedge clk) begin
    if (reset_n && !stall && last && uc_exec && !i_pop && state.keep_slot)
        ds_keepslot++;
    if (reset_n && !stall && last && uc_exec && !i_pop && !state.keep_slot) begin
        ds_total++;
        if (decq_empty)                                      ds_empty++;
        else if (!pop_class.fast || fast_off)                ds_seq++;
        else if (interrupt_pending || single_step)           ds_intr++;
        else if (pop_class.reads_flags && state.writes_flags &&
                 !pop_class.jcc)                             ds_flags++;
        else if (pop_class.uses_ea && ea2_conflict)          ds_ea++;
        else if (mem_confN)                                  ds_memc++;
        else if (!state.multi_word)                          ds_other_1w++;
        else                                                 ds_other++;
    end
end
final if (ds_total > 0)
    $display("z386x dead-slot breakdown: total=%0d empty=%0d seq=%0d flags=%0d ea=%0d memc=%0d intr=%0d other1w=%0d other=%0d keepslot=%0d",
             ds_total, ds_empty, ds_seq, ds_flags, ds_ea, ds_memc, ds_intr,
             ds_other_1w, ds_other, ds_keepslot);
// synthesis translate_on

endmodule
