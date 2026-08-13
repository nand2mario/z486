// Owns the architectural address registers and their relocated linear form.
// D2 supplies an effective address; segmentation supplies the active base/mask.
module address_unit
    import z486_pkg::*;
(
    input  logic        clk,
    input  logic        reset_n,
    input  logic        instr_pop,           // Latch the next architectural address
    input  dec_entry_t  instr,
    input  logic        d2_start,            // Latch D2 base/index selection
    input  ea_dec_t     d2_ea,               // Predecoded D2 address recipe
    input  logic [31:0] displacement,
    output gpr_ref_t    ea_base,             // GPR read request to data unit
    output gpr_ref_t    ea_index,            // Second GPR read request to data unit
    input  logic [31:0] ea_base_value,
    input  logic [31:0] ea_index_value,
    input  logic        branch_relative,     // EA is a relative branch target
    input  logic [31:0] branch_target_eip,   // D2-computed relative target
    input  logic [31:0] forwarded_esp,       // ESP including prior delay-slot write
    input  logic        ss_stack32,

    input  logic        exec,                // Execute a microcode IND operation
    input  logic        exec_addr32,
    input  logic [5:0]  busop,
    input  logic [5:0]  source,
    input  logic [5:0]  alu_source,
    input  logic [6:0]  destination,
    input  logic [6:0]  aluop,
    input  logic [31:0] source_value,
    input  logic [31:0] alu_value,
    input  logic [31:0] alu_value_hold,
    input  logic        jcc_active,
    input  logic        pe,
    input  logic        is_dword,
    input  logic        descsw_mode,
    input  logic        cs_stack32,

    input  logic [3:0]  seg_cmd,             // Segment command paired with this cycle
    input  logic [3:0]  seg_sel,             // Segment selected by segmentation unit
    input  logic [31:0] seg_base_pending,    // Base for current relocation
    input  logic        eff_mask_pending,    // 32-bit effective-offset enable
    input  logic [31:0] lar_result,
    input  logic [31:0] llim_result,
    input  logic [31:0] lbas_result,
    input  logic [2:0]  fault_code,
    input  logic [31:0] fault_addr,
    input  logic [31:0] cr3,

    output logic [31:0] ind,
    output logic [31:0] ind_delta,
    output logic [31:0] ind_linear,          // Registered relocated IND
    output logic        ind_linear_valid,    // Relocation matches current IND
    output logic [31:0] ea,
    output logic [31:0] pop_ea,              // Combinational D2 effective address
    output logic [31:0] pop_linear           // Combinational D2 relocated address
);

logic [1:0] ea_scale_r;
logic       ea_is_16bit_r;
logic       ea_scale_to_base_r;

//=============================================================================
// D2 effective-address formation
//=============================================================================

function automatic logic [2:0] onehot_idx(input logic [7:0] onehot);
    onehot_idx = onehot[1] ? 3'd1 : onehot[2] ? 3'd2 :
                 onehot[3] ? 3'd3 : onehot[4] ? 3'd4 :
                 onehot[5] ? 3'd5 : onehot[6] ? 3'd6 :
                 onehot[7] ? 3'd7 : 3'd0;
endfunction

wire [63:0] ea_terms = ea_scale_operands(
    ea_base_value, ea_index_value, ea_scale_r, ea_scale_to_base_r);
wire [31:0] ea_term_a = ea_terms[63:32];
wire [31:0] ea_term_b = ea_terms[31:0];
wire [31:0] ea_offset_full = ea_term_a + ea_term_b + displacement;
wire [31:0] effective_addr = ea_is_16bit_r
                           ? {16'd0, ea_offset_full[15:0]} : ea_offset_full;
wire [31:0] ea_csa_sum = ea_term_a ^ ea_term_b ^ displacement;
wire [31:0] ea_csa_carry = ((ea_term_a & ea_term_b) |
                            (ea_term_a & displacement) |
                            (ea_term_b & displacement)) << 1;
wire [31:0] linear32 = ea_csa_sum + ea_csa_carry + seg_base_pending;
wire [31:0] linear16 = {16'd0, ea_offset_full[15:0]} + seg_base_pending;
wire [31:0] effective_linear = (ea_is_16bit_r || !eff_mask_pending)
                             ? linear16 : linear32;
assign pop_ea = effective_addr;
assign pop_linear = effective_linear;

always_ff @(posedge clk) begin
    if (d2_start) begin
        ea_base.valid <= |d2_ea.base_sel;
        ea_base.index <= onehot_idx(d2_ea.base_sel);
        ea_index.valid <= |d2_ea.index_sel;
        ea_index.index <= onehot_idx(d2_ea.index_sel);
        ea_scale_r <= d2_ea.scale;
        ea_is_16bit_r <= d2_ea.is16;
        ea_scale_to_base_r <= d2_ea.s2b;
    end
end

// synthesis translate_off
// Prove the fused linear adder against the architectural relocate operation.
always @(posedge clk)
    if (reset_n && instr_pop && instr.has_modrm && !instr.stack_op &&
        !instr.has_moffs &&
        (effective_linear !==
         ((eff_mask_pending ? effective_addr :
                               {16'd0, effective_addr[15:0]}) +
          seg_base_pending)))
        $fatal(1,
               "POP-LIN FUSE MISMATCH: fused=%08x ref=%08x ea=%08x seg=%08x",
               effective_linear,
               (eff_mask_pending ? effective_addr :
                                    {16'd0, effective_addr[15:0]}) +
                   seg_base_pending,
               effective_addr, seg_base_pending);
// synthesis translate_on

//=============================================================================
// Relocation helpers and microcode IND updates
//=============================================================================

function automatic logic [31:0] relocate(input logic [31:0] offset);
    relocate = (eff_mask_pending ? offset : {16'd0, offset[15:0]}) +
               seg_base_pending;
endfunction

// Preserve the dedicated microcode relocation cone used before extraction.
(* keep *) wire [31:0] seg_base_pending_exec = seg_base_pending;

function automatic logic [31:0] relocate_exec(input logic [31:0] offset);
    relocate_exec = (eff_mask_pending ? offset : {16'd0, offset[15:0]}) +
                    seg_base_pending_exec;
endfunction

// Keep the IND update and segment relocation in the FPGA's fused adder.
function automatic logic [31:0] relocate_add2(
    input logic [31:0] a,
    input logic [31:0] b,
    input logic        mask16
);
    relocate_add2 = mask16
                  ? ({16'd0, a[15:0] + b[15:0]} + seg_base_pending_exec)
                  : (a + b + seg_base_pending_exec);
endfunction

always_ff @(posedge clk) begin
    if (instr_pop)
        ea <= branch_relative ? branch_target_eip : effective_addr;
end

always_ff @(posedge clk) begin
    if (!reset_n) begin
        ind <= 32'd0;
        ind_delta <= 32'd4;
        ind_linear <= 32'd0;
        ind_linear_valid <= 1'b0;
    end else if (instr_pop) begin
        ind_linear_valid <= 1'b0;
        ind_delta <= !instr.stack_op ? 32'd2 :
                     !instr.stack_dir ? (instr.data32 ? -32'd4 : -32'd2) :
                                        (instr.data32 ? 32'd4 : 32'd2);
        if (instr.stack_op && instr.stack_dir) begin
            automatic logic [31:0] stack_offset =
                ss_stack32 ? forwarded_esp : {16'd0, forwarded_esp[15:0]};
            ind <= stack_offset;
            ind_linear <= relocate(stack_offset);
            ind_linear_valid <= 1'b1;
        end else if (instr.stack_op && !instr.stack_dir) begin
            automatic logic [31:0] stack_offset = ss_stack32
                ? forwarded_esp - (instr.data32 ? 32'd4 : 32'd2)
                : {16'd0, forwarded_esp[15:0] -
                          (instr.data32 ? 16'd4 : 16'd2)};
            ind <= stack_offset;
            ind_linear <= relocate(stack_offset);
            ind_linear_valid <= 1'b1;
        end else if (instr.has_moffs) begin
            ind <= instr.addr32 ? instr.immediate
                                : {16'd0, instr.immediate[15:0]};
            ind_linear <= relocate(instr.immediate);
            ind_linear_valid <= 1'b1;
        end else if (instr.has_modrm) begin
            ind <= effective_addr;
            ind_linear <= effective_linear;
            ind_linear_valid <= 1'b1;
        end
    end else if (exec) begin
        automatic logic [31:0] reloc_source = ind;
        automatic logic        use_three_term = 1'b0;
        automatic logic        write_linear = 1'b0;
        automatic logic [31:0] linear_a = ind;
        automatic logic [31:0] linear_b = 32'd0;
        automatic logic        mask16 = !eff_mask_pending;

        if (seg_cmd == SEG_CMD_DESCSW ||
            (aluop == ALUJMP_STSSAF && seg_sel == SEG_SS))
            write_linear = 1'b1;

        case (busop)
            BUSOP_IND_PLUS_ALU: begin
                automatic logic [31:0] next_ind;
                automatic logic [31:0] operand1;
                automatic logic [31:0] operand2;
                operand1 = source == SRC_IRF2 ? ind : source_value;
                operand2 = jcc_active ? alu_value_hold : alu_value;
                if (alu_source != ALUSRC_ZERO)
                    ind_delta <= operand2;
                if (destination == DEST_DESSTK)
                    mask16 = !pe || !ss_stack32;
                else if (destination == DEST_DESCOD)
                    mask16 = !is_dword;
                else if (destination == DEST_DES_ES ||
                         destination == DEST_DES_OS ||
                         destination == DEST_DES_SR)
                    mask16 = !exec_addr32;
                next_ind = operand1 + operand2;
                if (mask16 && (destination == DEST_DESSTK ||
                               destination == DEST_DESCOD ||
                               destination == DEST_DES_ES ||
                               destination == DEST_DES_OS ||
                               destination == DEST_DES_SR))
                    next_ind = {16'd0, next_ind[15:0]};
                use_three_term = 1'b1;
                linear_a = operand1;
                linear_b = operand2;
                write_linear = 1'b1;
                ind <= next_ind;
                reloc_source = next_ind;
                ind_linear_valid <= 1'b1;
            end
            BUSOP_IND_ALU2: begin
                write_linear = 1'b1;
                ind <= alu_value;
                reloc_source = alu_value;
                ind_linear_valid <= 1'b1;
            end
            BUSOP_IND_SRC: begin
                automatic logic [31:0] next_ind = source_value;
                write_linear = 1'b1;
                if (destination == DEST_DESSTK && (!pe || !ss_stack32))
                    next_ind = {16'd0, next_ind[15:0]};
                else if (destination == DEST_DESCOD && !is_dword)
                    next_ind = {16'd0, next_ind[15:0]};
                ind <= next_ind;
                reloc_source = next_ind;
                ind_linear_valid <= 1'b1;
            end
            BUSOP_IND_PLUS: begin
                automatic logic [31:0] next_ind = ind + alu_value;
                write_linear = 1'b1;
                if (!pe && !exec_addr32)
                    next_ind = {16'd0, next_ind[15:0]};
                ind <= next_ind;
                reloc_source = next_ind;
                use_three_term = 1'b1;
                linear_a = ind;
                linear_b = alu_value;
                ind_linear_valid <= 1'b1;
                if (alu_source != ALUSRC_ZERO)
                    ind_delta <= alu_value;
            end
            BUSOP_IN_PLUS_D: begin
                automatic logic [31:0] next_ind = ind + ind_delta;
                write_linear = 1'b1;
                if (!pe ? !exec_addr32
                        : !(descsw_mode ? cs_stack32 : ss_stack32))
                    next_ind = {16'd0, next_ind[15:0]};
                ind <= next_ind;
                reloc_source = next_ind;
                use_three_term = 1'b1;
                linear_a = ind;
                linear_b = ind_delta;
                ind_linear_valid <= 1'b1;
            end
            BUSOP_LAR: begin
                ind <= lar_result;
                ind_linear_valid <= 1'b0;
            end
            BUSOP_LLIM: begin
                ind <= llim_result;
                ind_linear_valid <= 1'b0;
            end
            BUSOP_LBAS: begin
                ind <= lbas_result;
                ind_linear_valid <= 1'b0;
            end
            BUSOP_LPCR: begin
                case (destination)
                    DEST_PFERRC: ind <= {29'd0, fault_code};
                    DEST_LATTTF: ind <= fault_addr;
                    DEST_PDBR:   ind <= cr3;
                    default: ;
                endcase
                ind_linear_valid <= 1'b0;
            end
            default: ;
        endcase

        if (write_linear) begin
            if (use_three_term)
                ind_linear <= relocate_add2(linear_a, linear_b, mask16);
            else
                ind_linear <= relocate_exec(reloc_source);
        end
    end
end

endmodule
