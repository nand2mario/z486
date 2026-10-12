//
// Hardwired Control
// Generated one- to three-step recipes that execute common instructions from D2
//
module hardwired_control
    import z486_pkg::*;
(
    input  logic        clk,
    input  logic        reset_n,
    input  dec_entry_t  issue_instr,     // Instruction entering execution
    input  dec_entry_t  exec_instr,      // Instruction currently in execution
    input  ea_dec_t     issue_ea,        // Issuing instruction EA dependencies
    input  logic        decq_empty,
    input  logic        d2_push,
    input  logic [2:0]  d2_kind,
    input  logic        i_issue,
    input  logic        i_first,
    input  logic        uc_active,
    input  logic        uc_exec,
    input  logic        uc_slot_live,        // uc_exec without a direct load's own hold
    input  logic        i_rni,
    input  logic        i_rni_delay,
    input  logic        uc_next_rni,
    input  logic [6:0]  uc_aluop,
    input  logic        alu_write_flags,
    input  logic [31:0] flags_live,      // Current-cycle condition flags
    input  logic        exec_condition_true,
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
    input  logic        load_pipe_issue,     // Direct load bypasses its ROM recipe
    input  logic        load_pipe_pop,       //   and it is a POP (writes ESP in its first cycle)
    input  logic        load_pipe_ret,       //   and it is a RET (no fall-through successor)
    input  logic        load_wb_retire,      // Registered VIPT hit retires in WB
    input  logic        load_probe_wait,     // D2 direct load is waiting to issue
    input  logic        pb_valid,            // ROM port B holds the skeleton's first word
    input  logic        tf_issue,            // TF of the instruction issuing now (live EFLAGS)
    input  logic        pb_load_ready,       // a load skeleton could issue now (registered terms)
    input  logic        pb_load,             // the skeleton takes a new instruction this edge
    input  dec_entry_t  pb_next_instr,       //   that instruction (D1 handoff)
    input  ea_dec_t     pb_next_ea,          //   and its EA decode

    output recipe_meta_t issue_recipe,
    output logic        issue_hardwired,
    output logic        x87_direct_candidate,
    input  logic        disabled,           // fast paths off (Dev menu)
    output logic        pb_issue,         // Issue the skeleton into a dead RNI slot (B1)
    output logic        pb_slot,          // ROM port B supplies the next executing word
    output logic        recipe_rni,       // Current recipe uStep contains RNI
    output recipe_state_t recipe_state,  // Latched execution recipe
    output logic        slot_stale,       // Suppress the reclaimed RNI slot
    output logic        fold_active,      // Jcc folded into predecessor delay slot
    output logic        branch_ustep_rni,  // Hardwired branch supplies synthetic RNI
    output logic        branch_ustep_exec, // Execute hardwired branch uStep
    output logic        branch_redirect   // Hardwired branch redirects frontend
);

logic branch_ustep_r;
logic branch_ustep_jcc_r;
logic jcc_fold_r;
logic jcc_issue_valid_r;

//=============================================================================
// Recipe classification and active execution state
//=============================================================================

// Registered at the D1 handoff from the decode ROMs' recipe ID.
assign issue_recipe = issue_instr.recipe;
assign issue_hardwired = issue_recipe.hardwired && !disabled;
assign x87_direct_candidate =
    issue_recipe.commit_sel == RECIPE_ACTION_X87_DIRECT && !disabled;
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
                                     input logic hazard,
                                     input logic [2:0] widx);
    hazard_uses = hazard &&
        (ea.base_sel[widx] || ea.index_sel[widx] ||
         entry.recipe_gpr_read_mask[widx]);
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

wire pred2_we = recipe_state.commit_sel != RECIPE_COMMIT_NONE;
wire [2:0] pred2_widx = (recipe_state.commit_sel == RECIPE_COMMIT_SIGSRC)
    ? exec_instr.src_reg_sel
    : wide_widx(exec_instr.dst_reg_sel, op_size == 2'd0);
wire ea2_conflict = ea_conflict(pred2_we, pred2_widx, issue_ea, issue_instr, 1'b0);

wire mem_set = recipe_rni && uc_exec &&
               (recipe_state.commit_sel == RECIPE_COMMIT_MEM);
wire mem_hazard = mem_set || mem_commit.valid;
wire [2:0] mem_hreg = mem_set ? exec_instr.dst_reg_sel : mem_commit.dst;
wire [1:0] mem_hsize = mem_set ? op_size : mem_commit.size;
wire [2:0] mem_widx = wide_widx(mem_hreg, mem_hsize == 2'd0);
wire mem_confN = hazard_uses(issue_ea, issue_instr, mem_hazard, mem_widx);

// MOVZX/MOVSX use SRCREG as their architectural destination in the original
// microcode, while ordinary MOV uses DSTREG.  Normalize byte aliases as well
// so a direct AH load blocks a successor that consumes EAX as an EA input.
wire issue_load_is_movx = issue_instr.has_0f &&
    ((issue_instr.opcode == 8'hB6) || (issue_instr.opcode == 8'hB7) ||
     (issue_instr.opcode == 8'hBE) || (issue_instr.opcode == 8'hBF));
wire [2:0] issue_load_dst = issue_load_is_movx
                          ? issue_instr.src_reg_sel
                          : issue_instr.dst_reg_sel;
wire [2:0] issue_load_widx = wide_widx(issue_load_dst,
                                      !issue_load_is_movx &&
                                      (issue_instr.operand_size == 2'd0));

wire shift_set = recipe_rni && uc_exec && (recipe_state.commit_sel == RECIPE_COMMIT_SHIFT);
wire shift_hazard = shift_set || shift_commit.valid;
wire [2:0] shift_hreg = shift_set ? exec_instr.dst_reg_sel : shift_commit.dst;
wire [1:0] shift_hsize = shift_set ? op_size : shift_commit.size;
wire [2:0] shift_widx = wide_widx(shift_hreg, shift_hsize == 2'd0);
// A shift captured on this edge is not architectural until the following WB
// edge, so its dependent D2 head must wait.  An already-pending shift commits
// on the current issue edge; the new instruction reads its operands one cycle
// later and therefore needs no additional register dependency bubble.  D2 EA
// reads already have the pending-shift bypass in the Data Unit.
wire shift_confN = hazard_uses(
    issue_ea, issue_instr, shift_set,
    wide_widx(exec_instr.dst_reg_sel, op_size == 2'd0));

wire loaduse_conflict =
    (issue_recipe.reads_dst && gpr_overlap(exec_instr.dst_reg_sel,
        op_size == 2'd0, issue_instr.dst_reg_sel, issue_recipe.op_byte)) ||
    (issue_recipe.reads_src && gpr_overlap(exec_instr.dst_reg_sel,
        op_size == 2'd0, issue_instr.src_reg_sel, issue_recipe.op_byte)) ||
    (issue_recipe.reads_ecx && gpr_overlap(exec_instr.dst_reg_sel,
        op_size == 2'd0, 3'd1, 1'b0));
wire head_issue_safe = !decq_empty && d2_push && issue_recipe.hardwired &&
    (!issue_recipe.reads_flags || !recipe_state.writes_flags || issue_recipe.jcc) &&
    (!issue_recipe.uses_ea || !ea2_conflict) &&
    // Loads forward their deferred OPR_R value to all GPR data readers.  EA
    // base/index dependencies remain covered by ea2_conflict and therefore
    // retain the i486 one-cycle pointer-load interlock.
    !((recipe_state.commit_sel == RECIPE_COMMIT_SHIFT) &&
      loaduse_conflict) && !mem_confN && !shift_confN;

//=============================================================================
// Jcc folding and hardwired successor issue
//=============================================================================

wire jcc_unsafe = uc_exec && ((uc_aluop == ALUJMP_SHIFT2) ||
                              (uc_aluop == ALUJMP_SEZF));
wire fold_now = i_issue && issue_hardwired && issue_recipe.jcc &&
                !(alu_write_flags || jcc_unsafe) &&
                !condition_true(issue_instr.branch_condition, flags_live);
assign fold_active = jcc_fold_r && i_first;
// A non-folded Jcc already owns a branch uStep. Evaluate it there, after the
// predecessor's flags have reached the registered flag-forwarding boundary,
// rather than carrying the predecessor's live ALU result into an issue FF.
wire jcc_exec_taken = exec_condition_true;
// synthesis translate_off
always_ff @(posedge clk)
    if (reset_n && (exec_condition_true !==
                    condition_true(exec_instr.branch_condition, flags_live)))
        $fatal(1, "FORWARDED JCC CONDITION MISMATCH");
// synthesis translate_on

//-----------------------------------------------------------------------------
// Dead-slot issue from ROM port B (B1): US5293592 latches 35 hold the first
// microinstruction, so a successor needs no ROM launch to take a dead RNI
// slot. Whether it may take the next one is decided a cycle early and
// registered:
//   - at a single-uStep predecessor's issue, for the instruction entering the
//     skeleton from D1, against that predecessor;
//   - one word before a multi-uStep predecessor's RNI, for the resident
//     skeleton.
// Every hardwired recipe qualifies. Microcode instructions (x87 included)
// issue from port B at the RNI delay slot or into an idle sequencer.
//-----------------------------------------------------------------------------
function automatic logic pb_b1_type(input dec_entry_t e, input recipe_meta_t r);
    logic [2:0] k;
    begin
        k = e.recipe_early;
        pb_b1_type = r.hardwired &&
                     ((k == RECIPE_EARLY_BRANCH) ||
                      (e.rel_branch_kind == REL_BRANCH_CALL) ||
                      (!r.jcc && !r.br_rel && (e.rel_branch_kind == REL_BRANCH_NONE) &&
                       ((k == RECIPE_EARLY_NONE) || (k == RECIPE_EARLY_EA) ||
                        (k == RECIPE_EARLY_LOAD) || (k == RECIPE_EARLY_STORE) ||
                        (k == RECIPE_EARLY_RMW) || (k == RECIPE_EARLY_STACK))));
    end
endfunction

// The successor's GPR reads for the dead-slot decision, from its opcode and
// register fields alone, so it does not wait for the D1 entry point (the
// group entry table would otherwise set the cycle). A superset of
// recipe_gpr_read_mask for every hardwired recipe (checked in simulation):
//   source field      two-operand ALU/CMP/TEST (not r,m), MOV r,r and stores, SHLD/SHRD
//   destination field all but the MOV family (88-8B, A0-A3, B0-BF, C6/C7),
//                     LEA, POP, RET and relative branches
//   ECX               shifts and SHLD/SHRD by CL; ESP for a stack operation.
function automatic logic [7:0] pb_read_mask(input dec_entry_t e);
    logic [7:0] mask;
    logic op_byte, src_read, dst_read, ecx_read;
    logic [7:0] o;
    begin
        o = e.opcode;
        if (o[7:4] == 4'h4 || o[7:4] == 4'h5)
            op_byte = 1'b0;
        else if (o[7:4] == 4'hB && !e.has_0f)
            op_byte = !o[3];
        else
            op_byte = !o[0];
        src_read = e.has_0f
            ? ((o == 8'hA4) || (o == 8'hA5) || (o == 8'hAC) || (o == 8'hAD))
            : (((o[7:6] == 2'b00) && !o[2] && !(o[1] && (e.modrm[7:6] != 2'b11))) ||
               (o == 8'h84) || (o == 8'h85) ||
               (o == 8'h88) || (o == 8'h89) || (o == 8'hA2) || (o == 8'hA3) ||
               (((o == 8'h8A) || (o == 8'h8B)) && (e.modrm[7:6] == 2'b11)));
        dst_read = (e.rel_branch_kind == REL_BRANCH_NONE) &&
            (e.has_0f ||
             !(((o >= 8'h88) && (o <= 8'h8B)) || (o == 8'h8D) || (o[7:4] == 4'hB) ||
               ((o >= 8'hA0) && (o <= 8'hA3)) || (o == 8'hC2) || (o == 8'hC3) ||
               (o == 8'hC6) || (o == 8'hC7) || (o[7:3] == 5'b01011)));
        ecx_read = e.has_0f ? ((o == 8'hA5) || (o == 8'hAD))
                            : ((o == 8'hD2) || (o == 8'hD3));
        mask = e.stack_op ? 8'h10 : 8'h00;
        if (dst_read) mask[op_byte ? {1'b0, e.dst_reg_sel[1:0]} : e.dst_reg_sel] = 1'b1;
        if (src_read) mask[op_byte ? {1'b0, e.src_reg_sel[1:0]} : e.src_reg_sel] = 1'b1;
        if (ecx_read) mask[3'd1] = 1'b1;
        pb_read_mask = mask;
    end
endfunction
wire [7:0] pbn_read_mask = pb_read_mask(pb_next_instr);

// A byte or word load into a register (MOV, MOVZX/MOVSX, POP) merges into its
// destination's prior value, captured from the forwarded register view. An
// ALU load's M3 result is not in that view.
function automatic logic [7:0] pb_merge_mask(input dec_entry_t e);
    logic [7:0] o;
    logic merge;
    begin
        o = e.opcode;
        merge = (e.operand_size != 2'd2) &&
            (e.has_0f ? ((o == 8'hB6) || (o == 8'hB7) || (o == 8'hBE) || (o == 8'hBF))
                      : ((o == 8'h8A) || (o == 8'h8B) || (o == 8'hA0) || (o == 8'hA1) ||
                         (o[7:3] == 5'b01011)));
        pb_merge_mask = 8'h00;
        // MOVZX/MOVSX name their destination in the ModRM reg field.
        if (merge)
            pb_merge_mask[e.has_0f ? e.src_reg_sel :
                          (e.operand_size == 2'd0) ? {1'b0, e.dst_reg_sel[1:0]} :
                                                     e.dst_reg_sel] = 1'b1;
    end
endfunction
wire [7:0] pbn_merge_mask = pb_merge_mask(pb_next_instr);

// The slot is the predecessor's RNI word, a clock after this decision. A shift
// older than the predecessor has committed by then (D2 EA reads forward it),
// so only a shifting predecessor's own result conflicts.
wire [2:0] pred_shift_widx = wide_widx(issue_instr.dst_reg_sel, issue_instr.operand_size == 2'd0);
wire pbn_shift_conf = (issue_recipe.commit_sel == RECIPE_COMMIT_SHIFT) &&
    (pb_next_ea.base_sel[pred_shift_widx] || pb_next_ea.index_sel[pred_shift_widx] ||
     pbn_read_mask[pred_shift_widx]);
// The successor's recipe class, from its opcode and ModR/M fields alone
// (pb_b1_class.svh, generated from the recipe table and checked against it in
// simulation). Taken from the D1 entry point, the entry ROMs and the recipe
// table would set the cycle.
`include "pb_b1_class.svh"
wire [3:0] pbn_class = pb_b1_class(pb_next_instr.has_0f, pb_next_instr.opcode,
                                   pb_next_instr.modrm);
wire pbn_type = pbn_class[3] && (pb_next_instr.rep_lock == PREFIX_NOREPLOCK);
wire pbn_reads_flags = pbn_class[2];
wire pbn_uses_ea = pbn_class[1];
wire pbn_jcc = pbn_class[0];
wire pbn_safe = (!pbn_reads_flags || !issue_recipe.writes_flags || pbn_jcc) &&
    (!pbn_uses_ea ||
     !ea_conflict(pred1_we, pred1_widx, pb_next_ea, pb_next_instr,
                  issue_recipe.commit_sel == RECIPE_COMMIT_ESP)) &&
    !(mem_hazard && (pb_next_ea.base_sel[mem_widx] || pb_next_ea.index_sel[mem_widx] ||
                     pbn_read_mask[mem_widx])) &&
    !pbn_shift_conf;
wire skel_type = pb_b1_type(issue_instr, issue_recipe);
// A decision taken at an instruction's issue must use that instruction's TF:
// trap_active (tf_active_r) still describes its predecessor on this edge.
// A Jcc's slot (folded, or its branch uStep) is dead when the branch falls
// through; a taken branch blocks the issue and its flush drops the word.
wire pb_b1_from_issue = i_issue && issue_hardwired &&
    ((!issue_recipe.multi_ustep && !issue_recipe.jcc) || (issue_recipe.jcc && !jcc_unsafe)) &&
    !tf_issue &&
    pb_load && pbn_type && pbn_safe;
wire pb_b1_from_multi = recipe_state.hardwired && recipe_state.multi_ustep &&
    uc_exec && uc_next_rni && !i_issue && !i_rni_delay && !q_flush &&
    pb_valid && skel_type && head_issue_safe;
// A direct load completes in the data pipeline, so its first word is already
// a dead slot. Its successor may not read the load's M3 ALU result (no WB
// forwarding) or, after a flag-writing load, the flags.
wire pbn_load_alu_conf = issue_recipe.writes_flags &&
    (pbn_read_mask[issue_load_widx] || pbn_merge_mask[issue_load_widx]);
// A direct POP writes ESP in its first cycle; a stack successor takes the
// forwarded value, but an ESP base or index waits for the register.
wire pb_b1_from_load = load_pipe_issue && !tf_issue &&
    pb_load && pbn_type && pbn_safe && !pbn_load_alu_conf &&
    !(load_pipe_pop && (pb_next_ea.base_sel[4] || pb_next_ea.index_sel[4])) &&
    !load_pipe_ret;
logic pb_b1_ok_r;
logic pb_load_slot_r;   // the executing word is a direct load's first word
always_ff @(posedge clk) begin
    if (!reset_n || q_flush || any_fault) begin
        pb_b1_ok_r <= 1'b0;
        pb_load_slot_r <= 1'b0;
    end else if (!stall) begin
        // A successor that waits out its EA interlock on the load's first
        // word stays eligible for the load's retire word.
        pb_b1_ok_r <= pb_b1_from_issue || pb_b1_from_multi || pb_b1_from_load ||
                      (pb_b1_ok_r && pb_load_slot_r && !pb_issue);
        pb_load_slot_r <= load_pipe_issue;
    end
end

// A dead slot: the instruction's work is done, so its successor may take the
// slot from port B (B1). A hardwired recipe's RNI word qualifies (a store or
// push slot's work is done by the hardwired commit), as does a Jcc that falls
// through (folded, or in its branch uStep) and a direct load's first word.
// Legality (predecessor shape, hazards) is the registered pb_b1_ok_r. With
// the RNI delay slot (B2) and an idle sequencer (B3) these decide, from
// registered terms only, when the ROM output and port A switch to port B.
wire pb_dead_slot = (recipe_rni && !recipe_state.jcc && !branch_ustep_rni &&
                     !jcc_fold_r) ||
                    (recipe_state.jcc && jcc_issue_valid_r && i_first &&
                     (jcc_fold_r || branch_ustep_rni)) ||
                    pb_load_slot_r;
// The dead slot takes port B's word only when the issue is already known: the
// registered eligibility, a complete payload, and no pending interrupt, trap
// or throttle. A stall freezes the ROM output; a fault suppresses the slot.
// So a word that is loaded but not issued never runs its bus operation.
wire pb_b1_ready = pb_b1_ok_r && d2_push && pb_load_ready && !throttle_hold &&
                   !interrupt_pending && !trap_active && !single_step && !disabled;
assign pb_slot = pb_valid &&
                 ((pb_dead_slot && pb_b1_ready) || i_rni_delay || !uc_active);

assign pb_issue = pb_dead_slot && pb_b1_ready && (pb_load_slot_r || load_wb_retire ? uc_slot_live : uc_exec) && pb_valid &&
                  !branch_redirect && !any_fault_issue && !q_flush && !stall;


//=============================================================================
// Bounded branch uStep and recipe state
//=============================================================================

// RNI describes the resident uStep and, like a ROM RNI bit, remains visible
// while execution is stalled.  Keeping uc_exec out of this qualifier breaks
// the D2-stall -> uc_exec -> synthetic-RNI -> D2-stall feedback cone.  The
// redirect itself remains execution-qualified below.
assign branch_ustep_rni = i_first && branch_ustep_r;
assign branch_ustep_exec = branch_ustep_rni && uc_exec;
assign branch_redirect = branch_ustep_exec &&
                         (!branch_ustep_jcc_r || jcc_exec_taken);

always_ff @(posedge clk) begin
    if (!reset_n) begin
        recipe_state <= '0;
        branch_ustep_r <= 1'b0;
        branch_ustep_jcc_r <= 1'b0;
        jcc_fold_r <= 1'b0;
        jcc_issue_valid_r <= 1'b0;
    end else begin
        if (q_flush || interrupt_entry || any_fault) begin
            jcc_fold_r <= 1'b0;
            jcc_issue_valid_r <= 1'b0;
        end else if (i_issue) begin
            jcc_fold_r <= fold_now;
            jcc_issue_valid_r <= issue_recipe.jcc && !jcc_unsafe;
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
                ((d2_kind == RECIPE_EARLY_BRANCH) ||
                 (issue_instr.rel_branch_kind == REL_BRANCH_CALL)) &&
                // A 16-bit CALL pushes a 16-bit return address in microcode.
                (issue_instr.data32 ||
                 (issue_instr.rel_branch_kind != REL_BRANCH_CALL)) &&
                (!issue_instr.stack_op ||
                 (issue_instr.rel_branch_kind == REL_BRANCH_CALL)) &&
                (!issue_recipe.jcc || !jcc_unsafe);
            branch_ustep_jcc_r <= issue_instr.rel_branch_kind == REL_BRANCH_JCC;
        end
        if (load_wb_retire && !i_issue) begin
            recipe_state.commit_sel <= RECIPE_COMMIT_NONE;
            recipe_state.slot_has_work <= 1'b0;
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
    else if (!stall) begin
        // A VIPT probe wait can outlive the stale-slot pulse. Keep ownership
        // only while the same hardwired recipe remains at its RNI word;
        // otherwise a stale state from an older recipe could suppress useful
        // work after the sequencer has moved on.
        if (slot_stale && recipe_rni && load_probe_wait && !i_issue)
            slot_stale <= 1'b1;
        else
            slot_stale <= recipe_rni && (uc_exec || load_wb_retire) && !i_issue &&
                          (!recipe_state.slot_has_work || load_wb_retire);
    end
end

//=============================================================================
// Simulation checks
//=============================================================================

// synthesis translate_off
recipe_meta_t pbn_recipe;
assign pbn_recipe = recipe_metadata(pb_next_instr);
wire pbn_type_exact = pb_b1_type(pb_next_instr, pbn_recipe);
always @(posedge clk)
    if (reset_n && pb_load &&
        ((pbn_type != pbn_type_exact) ||
         (pbn_type && ({pbn_reads_flags, pbn_uses_ea, pbn_jcc} !=
                       {pbn_recipe.reads_flags, pbn_recipe.uses_ea, pbn_recipe.jcc}))))
        $fatal(1, "pb_b1_class disagrees with the recipe table: entry %03x opcode %02x modrm %02x class %b type %b",
               pb_next_instr.entry_point, pb_next_instr.opcode, pb_next_instr.modrm,
               pbn_class, pbn_type_exact);
always @(posedge clk)
    if (reset_n && pb_load && pbn_type &&
        |(recipe_gpr_read_mask(pb_next_instr) & ~pbn_read_mask))
        $fatal(1, "pb_read_mask misses a GPR read: entry %03x opcode %02x mask %02x exact %02x",
               pb_next_instr.entry_point, pb_next_instr.opcode, pbn_read_mask,
               recipe_gpr_read_mask(pb_next_instr));

// A dead slot that took port B's word without an issue must not execute it:
// the word belongs to an instruction still in D2.
logic pb_unissued_r;
always_ff @(posedge clk)
    if (!reset_n || q_flush || any_fault) pb_unissued_r <= 1'b0;
    else if (!stall) pb_unissued_r <= pb_slot && pb_dead_slot && !i_issue;
longint pb_unissued_n = 0, pb_unissued_exec_n = 0;
always @(posedge clk)
    if (reset_n && pb_unissued_r && !stall) begin
        pb_unissued_n <= pb_unissued_n + 1;
        if (uc_exec && uc_active && !i_first) begin
            pb_unissued_exec_n <= pb_unissued_exec_n + 1;
            $error("port-B word executes without issue");
        end
    end
final $display("PB_UNISSUED dead-slot selections %0d, executed %0d",
               pb_unissued_n, pb_unissued_exec_n);

always @(posedge clk)
    if (reset_n && fold_active && uc_exec &&
        condition_true(exec_instr.branch_condition, flags_live))
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
        else if (!issue_recipe.hardwired || disabled)                     ds_seq++;
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
