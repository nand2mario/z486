// Hardwired common-instruction control. Terminology:
//   hardwired instruction - architectural instruction selected for this control;
//   recipe                - its generated sequence of one to three uSteps;
//   issue                 - i_issue transfers one instruction from D2 into EX;
//   chaining              - starts its successor early in a reclaimed RNI slot.
// See doc/z486/hardwired_instructions.md. Chain decisions remain combinational
// into the ROM so slot reclamation does not add an issue cycle.
module hardwired_control
    import z486_pkg::*;
(
    input  logic        clk,
    input  logic        reset_n,
    input  dec_entry_t  issue_instr,     // Instruction entering execution
    input  dec_entry_t  next_instr,      // Following instruction for chain decision
    input  dec_entry_t  exec_instr,      // Instruction currently in execution
    input  ea_dec_t     issue_ea,        // Issuing instruction EA dependencies
    input  ea_dec_t     next_ea,         // Following instruction EA dependencies
    input  logic        decq_has2,
    input  logic        decq_empty,
    input  logic        d2_push,
    input  logic [2:0]  d2_kind,
    input  logic        d2_valid,
    input  logic        d2_waited,       // D2 already consumed a wait cycle
    input  logic        i_issue,
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
    input  recipe_pending_write_t mem_commit,   // Deferred load GPR write
    input  recipe_pending_write_t shift_commit, // Deferred shift GPR write
    input  logic        q_flush,
    input  logic        interrupt_entry,
    input  logic        interrupt_pending,
    input  logic        trap_active,
    input  logic        single_step,
    input  logic        any_fault,
    input  logic        any_fault_r,
    input  logic        any_fault_issue,
    input  logic        throttle_hold,
    input  logic        stall,

    output recipe_meta_t issue_recipe,
    output logic        issue_hardwired,
    output logic        x87_direct_candidate,
    output logic        disabled,
    output logic        chain_start,      // Start a successor in a reclaimed slot
    output logic        chain_from_next,  // Successor is behind the queue head
    output logic [11:0] chain_entry,      // Successor's ROM entry
    output logic        recipe_rni,       // Current recipe uStep contains RNI
    output recipe_state_t recipe_state,  // Latched execution recipe
    output logic        slot_stale,       // Suppress the reclaimed RNI slot
    output logic        fold_active,      // Jcc folded into predecessor delay slot
    output logic        branch_ustep_exec, // Execute hardwired branch uStep
    output logic        branch_redirect   // Hardwired branch redirects frontend
);

recipe_meta_t next_recipe;
logic branch_ustep_r;
logic branch_ustep_jcc_r;
logic jcc_fold_r;
logic jcc_issue_taken_r;
logic jcc_issue_valid_r;
logic hardwired_off = 1'b0;

// synthesis translate_off
initial if ($test$plusargs("z486_hardwired_off") ||
            $test$plusargs("z486_fast_off")) hardwired_off = 1'b1;
// synthesis translate_on

//=============================================================================
// Recipe classification and active execution state
//=============================================================================

assign issue_recipe = recipe_metadata(issue_instr);
assign disabled = hardwired_off;
assign next_recipe = recipe_metadata(next_instr);
assign issue_hardwired = issue_recipe.hardwired && !hardwired_off;
assign x87_direct_candidate =
    issue_recipe.commit_sel == RECIPE_ACTION_X87_DIRECT;
wire recipe_active = i_first && recipe_state.hardwired;
assign recipe_rni = recipe_state.hardwired && uc_active && i_rni;

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
                                     input recipe_meta_t fc,
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

//=============================================================================
// Predecessor/successor dependency checks
//=============================================================================

wire pred1_we = (issue_recipe.commit_sel == RECIPE_COMMIT_ALU) ||
                (issue_recipe.commit_sel == RECIPE_COMMIT_ESP) ||
                issue_recipe.writes_srcreg;
wire [2:0] pred1_widx = (issue_recipe.commit_sel == RECIPE_COMMIT_ESP) ? 3'd4 :
    wide_widx(issue_recipe.writes_srcreg ? issue_instr.src_reg_sel
                                      : issue_instr.dst_reg_sel,
               !issue_recipe.writes_srcreg && issue_recipe.op_byte);
wire ea1_conflict = ea_conflict(pred1_we, pred1_widx, next_ea, next_instr,
                                issue_recipe.commit_sel == RECIPE_COMMIT_ESP);

wire pred2_we = recipe_state.commit_sel != RECIPE_COMMIT_NONE;
wire [2:0] pred2_widx = (recipe_state.commit_sel == RECIPE_COMMIT_SIGSRC)
    ? exec_instr.src_reg_sel
    : wide_widx(exec_instr.dst_reg_sel, op_size == 2'd0);
wire ea2_conflict = ea_conflict(pred2_we, pred2_widx, issue_ea, issue_instr, 1'b0);

wire mem_set = recipe_rni && uc_exec && (recipe_state.commit_sel == RECIPE_COMMIT_MEM);
wire mem_hazard = mem_set || mem_commit.valid;
wire [2:0] mem_hreg = mem_set ? exec_instr.dst_reg_sel : mem_commit.dst;
wire [1:0] mem_hsize = mem_set ? op_size : mem_commit.size;
wire [2:0] mem_widx = wide_widx(mem_hreg, mem_hsize == 2'd0);
wire mem_conf1 = hazard_uses(next_ea, next_instr, next_recipe, mem_hazard,
                             mem_widx, mem_hreg, mem_hsize);
wire mem_confN = hazard_uses(issue_ea, issue_instr, issue_recipe, mem_hazard,
                             mem_widx, mem_hreg, mem_hsize);

wire shift_set = recipe_rni && uc_exec && (recipe_state.commit_sel == RECIPE_COMMIT_SHIFT);
wire shift_hazard = shift_set || shift_commit.valid;
wire [2:0] shift_hreg = shift_set ? exec_instr.dst_reg_sel : shift_commit.dst;
wire [1:0] shift_hsize = shift_set ? op_size : shift_commit.size;
wire [2:0] shift_widx = wide_widx(shift_hreg, shift_hsize == 2'd0);
wire shift_conf1 = hazard_uses(next_ea, next_instr, next_recipe, shift_hazard,
                               shift_widx, shift_hreg, shift_hsize);
wire shift_confN = hazard_uses(issue_ea, issue_instr, issue_recipe, shift_hazard,
                               shift_widx, shift_hreg, shift_hsize);

wire next_chain_safe = decq_has2 && next_recipe.hardwired &&
    (!next_recipe.reads_flags || !issue_recipe.writes_flags || next_recipe.jcc) &&
    (!next_recipe.uses_ea || !ea1_conflict) && !mem_conf1 && !shift_conf1;

wire loaduse_conflict =
    (issue_recipe.reads_dst && gpr_overlap(exec_instr.dst_reg_sel,
        op_size == 2'd0, issue_instr.dst_reg_sel, issue_recipe.op_byte)) ||
    (issue_recipe.reads_src && gpr_overlap(exec_instr.dst_reg_sel,
        op_size == 2'd0, issue_instr.src_reg_sel, issue_recipe.op_byte)) ||
    (issue_recipe.reads_ecx && gpr_overlap(exec_instr.dst_reg_sel,
        op_size == 2'd0, 3'd1, 1'b0));
wire head_chain_safe = !decq_empty && d2_push && issue_recipe.hardwired &&
    (!issue_recipe.reads_flags || !recipe_state.writes_flags || issue_recipe.jcc) &&
    (!issue_recipe.uses_ea || !ea2_conflict) &&
    !((recipe_state.commit_sel == RECIPE_COMMIT_MEM ||
       recipe_state.commit_sel == RECIPE_COMMIT_SHIFT) &&
      loaduse_conflict) && !mem_confN && !shift_confN;

//=============================================================================
// Jcc folding and hardwired successor issue
//=============================================================================

wire jcc_unsafe = uc_exec && ((uc_aluop == ALUJMP_SHIFT2) ||
                              (uc_aluop == ALUJMP_SEZF));
wire fold_now = i_issue && issue_hardwired && issue_recipe.jcc &&
                !(alu_write_flags || jcc_unsafe) &&
                !condition_true(issue_instr.opcode[3:0], flags_live);
assign fold_active = jcc_fold_r && i_first;

wire chain_after_single = i_issue && issue_hardwired &&
    ((!issue_recipe.multi_ustep && !issue_recipe.jcc) || fold_now) && next_chain_safe;
wire chain_after_multi = recipe_state.hardwired && recipe_state.multi_ustep && uc_exec && uc_next_rni &&
    !i_issue && !i_rni_delay && !q_flush && head_chain_safe;
wire chain_after_jcc = recipe_active && recipe_state.jcc && uc_exec && jcc_issue_valid_r &&
    !jcc_issue_taken_r && !q_flush && head_chain_safe && !jcc_fold_r;
assign chain_start = (chain_after_single || chain_after_multi || chain_after_jcc) &&
    (!d2_valid || i_issue) && !d2_waited && !throttle_hold &&
    !interrupt_pending && !trap_active && !single_step && !any_fault_issue;
assign chain_from_next = chain_after_single;
assign chain_entry = chain_after_single ? next_instr.entry_point : issue_instr.entry_point;

//=============================================================================
// Bounded branch uStep and recipe state
//=============================================================================

assign branch_ustep_exec = i_first && branch_ustep_r && uc_exec;
assign branch_redirect = branch_ustep_exec &&
                         (!branch_ustep_jcc_r || jcc_issue_taken_r);

always_ff @(posedge clk) begin
    if (!reset_n) begin
        recipe_state <= '0;
        branch_ustep_r <= 1'b0;
        branch_ustep_jcc_r <= 1'b0;
        jcc_fold_r <= 1'b0;
        jcc_issue_taken_r <= 1'b0;
        jcc_issue_valid_r <= 1'b0;
    end else begin
        if (q_flush || interrupt_entry || any_fault) begin
            jcc_fold_r <= 1'b0;
            jcc_issue_valid_r <= 1'b0;
        end else if (i_issue) begin
            jcc_fold_r <= fold_now;
            jcc_issue_valid_r <= issue_recipe.jcc && !jcc_unsafe;
            jcc_issue_taken_r <= condition_true(issue_instr.opcode[3:0], flags_ahead);
        end else if (!stall) begin
            jcc_fold_r <= 1'b0;
        end

        if (i_issue) begin
            recipe_state.hardwired <= issue_hardwired;
            recipe_state.multi_ustep <= issue_hardwired && issue_recipe.multi_ustep;
            recipe_state.jcc <= issue_hardwired && issue_recipe.jcc;
            recipe_state.writes_flags <= issue_recipe.writes_flags;
            recipe_state.commit_sel <= issue_recipe.commit_sel;
            recipe_state.slot_has_work <= issue_recipe.slot_has_work;
            branch_ustep_r <= issue_hardwired &&
                ((d2_kind == RECIPE_EARLY_BRANCH) || issue_instr.opcode == 8'hE8) &&
                issue_instr.data32 && (!issue_instr.stack_op || issue_instr.opcode == 8'hE8) &&
                (!issue_recipe.jcc || !jcc_unsafe);
            branch_ustep_jcc_r <= issue_recipe.jcc;
        end
        if (any_fault || any_fault_r || interrupt_entry) begin
            recipe_state.hardwired <= 1'b0;
            recipe_state.multi_ustep <= 1'b0;
            recipe_state.jcc <= 1'b0;
            recipe_state.commit_sel <= RECIPE_COMMIT_NONE;
            branch_ustep_r <= 1'b0;
        end
    end
end

// The original RNI slot is dead only for recipes that moved all architectural
// work into their bounded uSteps. Memory/store recipes use slot_has_work instead.
always_ff @(posedge clk) begin
    if (!reset_n)
        slot_stale <= 1'b0;
    else if (!stall)
        slot_stale <= recipe_rni && uc_exec && !i_issue && !recipe_state.slot_has_work;
end

//=============================================================================
// Simulation checks and chaining diagnostics
//=============================================================================

// synthesis translate_off
always @(posedge clk)
    if (reset_n && fold_active && uc_exec &&
        condition_true(exec_instr.opcode[3:0], flags_live))
        $display("%0t JCC-FOLD MISMATCH: opcode=%02x flags=%08x",
                 $time, exec_instr.opcode, flags_live);

int unsigned ds_total, ds_empty, ds_seq, ds_flags, ds_ea, ds_memc;
int unsigned ds_intr, ds_other, ds_other_1w, ds_keepslot;
always @(posedge clk) begin
    if (reset_n && !stall && recipe_rni && uc_exec && !i_issue && recipe_state.slot_has_work)
        ds_keepslot++;
    if (reset_n && !stall && recipe_rni && uc_exec && !i_issue && !recipe_state.slot_has_work) begin
        ds_total++;
        if (decq_empty)                                      ds_empty++;
        else if (!issue_recipe.hardwired || hardwired_off)                ds_seq++;
        else if (interrupt_pending || single_step)           ds_intr++;
        else if (issue_recipe.reads_flags && recipe_state.writes_flags &&
                 !issue_recipe.jcc)                             ds_flags++;
        else if (issue_recipe.uses_ea && ea2_conflict)          ds_ea++;
        else if (mem_confN)                                  ds_memc++;
        else if (!recipe_state.multi_ustep)                          ds_other_1w++;
        else                                                 ds_other++;
    end
end
final if (ds_total > 0)
    $display("z486 dead-slot breakdown: total=%0d empty=%0d seq=%0d flags=%0d ea=%0d memc=%0d intr=%0d other1w=%0d other=%0d keepslot=%0d",
             ds_total, ds_empty, ds_seq, ds_flags, ds_ea, ds_memc, ds_intr,
             ds_other_1w, ds_other, ds_keepslot);
// synthesis translate_on

endmodule
