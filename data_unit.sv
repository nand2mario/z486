// Integer Data Unit. Owns integer register state, source/destination selection,
// arithmetic engines, and architectural/microcode flags.
`include "z486_platform.svh"
module data_unit
    import z486_pkg::*;
(
    input  logic        clk,
    input  logic        reset_n,

    input  logic        exec,                   // Execute the current micro-op
    input  logic        shift_exec,             // Execute registered shift control
    input  logic        instr_start,            // First cycle of a new instruction
    input  logic        halted,
    input  logic        ifetch_page_fault,
    input  logic        interrupt_entry,
    input  logic        repeat_active,
    input  logic        clear_rf,
    input  logic        pipeline_advance,        // Advance deferred flag state
    input  logic        stack_op,
    input  logic        stack_dir,
    input  logic        stack_data32,
    input  logic        stack32,
    input  logic        gate_detect,
    input  logic        any_fault,
    input  logic        uc_active,
    input  logic        recipe_rni,               // Current recipe uStep contains RNI
    input  recipe_state_t recipe_state,           // Latched hardwired recipe
    input  logic        hardwired_off,
    input  logic        recipe_commit_cancel,     // Cancel deferred recipe commit
    input  logic        load_wb_valid,             // Registered VIPT load WB
    input  logic [2:0]  load_wb_dst,
    input  logic [1:0]  load_wb_size,
    input  logic [31:0] load_wb_data,
    input  logic        load_wb_is_alu,          // Registered memory operand feeds shared ALU
    input  logic [4:0]  load_wb_alu_op,
    input  logic        load_alu_dst_capture,    // Capture destination GPR at EX/WB boundary
    input  logic [2:0]  load_alu_dst_capture_dst,
    input  logic [1:0]  load_alu_dst_capture_size,
    input  logic [31:0] load_alu_dst_capture_data,

    input  logic [6:0]  aluop,
    input  logic [4:0]  alu_operation,
    input  logic [6:0]  shift_aluop,             // ROM-early ALU/jump field for shifter
    input  logic [1:0]  shift_sigma_sel,         // Registered barrel SIGMA selector
    input  logic [6:0]  dest,
    input  logic [5:0]  source_field,
    input  logic [5:0]  source_live,             // Timing-selected live source field
    input  logic [5:0]  alu_source,
    input  logic [5:0]  alu_source_live,          // Timing-selected live ALU source
    input  logic        fpu_f8,
    input  logic [3:0]  shift_source_class,      // Predecoded shifter source class
    input  logic [1:0]  shift2_source,           // Predecoded SHIFT2 source
    input  logic        shift_is_shift2,         // Registered ROM SHIFT2 decode
    input  logic        shift2_capture_ce,       // Advance q_mem -> q operand capture
    input  logic        shift2_next_valid,       // q_mem word is SHIFT2
    input  logic [1:0]  shift2_next_source,      // q_mem SHIFT2 source class
    input  logic        shift_uc_carry,          // BSR loop needs carry immediately
    input  logic [1:0]  op_size,
    input  logic [1:0]  srcreg_size,
    input  logic [1:0]  op_size_src,             // Source-mux operand size
    input  logic [1:0]  srcreg_size_src,          // Source-mux register size
    input  logic        update_arch_flags,
    input  logic        update_carry,

    input  dec_entry_t  instr,
    input  dec_entry_t  next_instr,              // D2 instruction for setup lookahead
    input  logic        pe,
    input  logic [1:0]  cpl,
    input  logic        is_dword,
    input  logic        is_signed_mul,
    input  logic [31:0] eip,
    input  logic [31:0] cr0,
    input  logic [31:0] cr2,
    input  logic [31:0] tmpeip,
    input  logic [31:0] tmpesp,
    input  logic [31:0] dr6,
    input  logic [31:0] dr7,
    input  logic [31:0] slctr,
    input  logic [31:0] protun,
    input  logic [31:0] ind,
    input  logic [31:0] ea,
    input  logic [15:0] es,
    input  logic [15:0] cs,
    input  logic [15:0] ss,
    input  logic [15:0] ds,
    input  logic [15:0] fs,
    input  logic [15:0] gs,
    input  logic [15:0] ldtr,
    input  logic [15:0] tr,
    input  logic [2:0]  seg_reg_sel,
    input  logic [31:0] forwarded_esp,           // ESP including pending stack update
    input  logic [31:0] desc_raw_hi,
    input  logic [31:0] opr_r,
    input  gpr_ref_t    ea_base,                 // Address-unit base GPR reference
    input  gpr_ref_t    ea_index,                // Address-unit index GPR reference
    input  gpr_forward_t dly_gpr_forward,        // RNI-delay GPR bypass
    output logic [31:0] sigma,
    output logic [31:0] countr,
    output logic [31:0] alu_src_hold,             // Registered ALU source operand
    output logic [31:0] source_value_live,        // Selected microcode source value
    output logic [31:0] memory_write_source_value,// Narrow source mux for WR W
    output logic [31:0] alu_source_value_live,    // Selected ALU-source value
    output logic [31:0] dest_value,
    output logic [31:0] alu_src,
    output logic [31:0] eax,
    output logic [31:0] ecx,
    output logic [31:0] edx,
    output logic [31:0] ebx,
    output logic [31:0] esp,
    output logic [31:0] ebp,
    output logic [31:0] esi,
    output logic [31:0] edi,
    output logic [31:0] tmpc,
    output logic [31:0] tmpg,
    output logic [31:0] opr_w,
    output logic [31:0] protection_source_value, // Source value for protection unit
    output logic        protection_source_low16_nonzero,
    output logic [15:0] cs_source_value,          // Source value for CS updates
    output logic [31:0] ea_base_value,            // Base GPR value for address unit
    output logic [31:0] ea_index_value,           // Index GPR value for address unit
    output logic [31:0] eflags,
    output logic [31:0] uc_flags,
    output logic [31:0] flags_backup,
    output logic        flags_backup_active,
    output logic [31:0] eflags_fwd,               // Current-cycle flag forwarding
    output logic        branch_condition_true,    // Selected forwarded Jcc condition

    output recipe_pending_write_t recipe_shift_write, // Deferred shift GPR commit
    output logic [31:0] recipe_shift_data,          // Deferred shift result
    output recipe_pending_write_t recipe_memory_write, // Deferred load GPR commit

    output logic [31:0] muldiv_result,
    output logic        div_overflow,
    output logic [31:0] alu_result,
    output logic [31:0] shift_result
);

logic [31:0] alu_dst;
logic [31:0] alu_flags;
logic        alu_zsp_update;
logic [31:0] load_wb_commit_data;
logic        load_wb_alu_exec;
logic        load_wb_alu_commit;
logic [31:0] load_wb_alu_result;
logic [31:0] load_wb_alu_flags;
logic [31:0] load_wb_dst_base_r;
logic [31:0] load_wb_forward_data_r;

logic [31:0] tmpb, tmpd, tmpe, tmpf, tmph;
logic [31:0] csopcd, fsveip, oproff;
logic [2:0] src_reg_sel_r;           // EX-local copies of issued GPR selectors
logic [2:0] dst_reg_sel_r;
logic [2:0] recipe_shift_widx;       // Byte-normalized deferred-shift GPR
logic [7:0] recipe_memory_dst_onehot;// Byte-normalized deferred-load GPR
logic [1:0] recipe_memory_mode;      // Byte-low/high, word, or dword merge

always_ff @(posedge clk) begin
    if (!reset_n) begin
        src_reg_sel_r <= 3'd0;
        dst_reg_sel_r <= 3'd0;
    end else if (instr_start) begin
        src_reg_sel_r <= next_instr.src_reg_sel;
        dst_reg_sel_r <= next_instr.dst_reg_sel;
    end
end

logic [31:0] shift_setup_result;
logic [1:0]  shift_data_size;
logic        shift_count_nonzero;
logic        shift_bit_test_cf;

logic        muldiv_sigma_write;
logic [31:0] muldiv_sigma_value;
logic        muldiv_tmpb_write;
logic [31:0] muldiv_tmpb_value;
logic        muldiv_counter_early_exit;
logic        muldiv_quotient_zero;
logic        muldiv_quotient_sign;
logic        muldiv_quotient_parity;
logic        muldiv_flag_overflow;

logic sh_flags_commit;
logic sh_flags_we_zsp;
logic sh_flags_we_of;
logic sh_flags_cf;
logic sh_flags_of;
logic sh_flags_zf;
logic sh_flags_sf;
logic sh_flags_pf;

logic [31:0] shift2_capture_value;

logic        flag2_eflags_p;
logic        flag2_ucflags_p;
logic [31:0] flag2_result_r;
logic        flag2_cf_r;
logic        flag2_af_r;
logic        flag2_of_r;
logic        flag2_zsp_r;
logic [1:0]  flag2_size_r;
logic        clear_if_pending;

//=============================================================================
// Operand selection and EA forwarding
//=============================================================================

function automatic logic [31:0] read_gpr_value(
    input logic [2:0] reg_sel,
    input logic [1:0] size
);
    case (size)
        2'd0: begin
            case (reg_sel)
                3'd0: read_gpr_value = {24'd0, eax[7:0]};
                3'd1: read_gpr_value = {24'd0, ecx[7:0]};
                3'd2: read_gpr_value = {24'd0, edx[7:0]};
                3'd3: read_gpr_value = {24'd0, ebx[7:0]};
                3'd4: read_gpr_value = {24'd0, eax[15:8]};
                3'd5: read_gpr_value = {24'd0, ecx[15:8]};
                3'd6: read_gpr_value = {24'd0, edx[15:8]};
                3'd7: read_gpr_value = {24'd0, ebx[15:8]};
            endcase
        end
        2'd1: begin
            case (reg_sel)
                3'd0: read_gpr_value = {16'd0, eax[15:0]};
                3'd1: read_gpr_value = {16'd0, ecx[15:0]};
                3'd2: read_gpr_value = {16'd0, edx[15:0]};
                3'd3: read_gpr_value = {16'd0, ebx[15:0]};
                3'd4: read_gpr_value = {16'd0, esp[15:0]};
                3'd5: read_gpr_value = {16'd0, ebp[15:0]};
                3'd6: read_gpr_value = {16'd0, esi[15:0]};
                3'd7: read_gpr_value = {16'd0, edi[15:0]};
            endcase
        end
        default: begin
            case (reg_sel)
                3'd0: read_gpr_value = eax;
                3'd1: read_gpr_value = ecx;
                3'd2: read_gpr_value = edx;
                3'd3: read_gpr_value = ebx;
                3'd4: read_gpr_value = esp;
                3'd5: read_gpr_value = ebp;
                3'd6: read_gpr_value = esi;
                3'd7: read_gpr_value = edi;
            endcase
        end
    endcase
endfunction

localparam logic [1:0] EA_FWD_BLO = 2'd0;
localparam logic [1:0] EA_FWD_BHI = 2'd1;
localparam logic [1:0] EA_FWD_W   = 2'd2;
localparam logic [1:0] EA_FWD_D   = 2'd3;

// Form the plain-load architectural value once at WB.  The destination's
// prior full-width value was captured from the EX token one cycle earlier, so
// byte/word merging does not select the GPR bank from the live WB destination
// on the timing-critical successor-EA path.
function automatic logic [31:0] merge_load_forward(
    input logic [31:0] base,
    input logic [31:0] data,
    input logic [2:0]  dst,
    input logic [1:0]  size
);
    if (size == 2'd0)
        merge_load_forward = dst[2]
            ? {base[31:16], data[7:0], base[7:0]}
            : {base[31:8], data[7:0]};
    else if (size == 2'd1)
        merge_load_forward = {base[31:16], data[15:0]};
    else
        merge_load_forward = data;
endfunction

wire [31:0] load_wb_forward_data = load_wb_forward_data_r;
wire [2:0] load_wb_widx = (load_wb_size == 2'd0)
                         ? {1'b0, load_wb_dst[1:0]} : load_wb_dst;

// WB-to-EX bypass for a hardwired load's immediate successor.  Cache data is
// already registered in load_wb_data; this mux therefore starts at the WB
// boundary and does not extend the cache finalize path into the ALU.
function automatic logic [31:0] read_gpr_load_forwarded(
    input logic [2:0] reg_sel,
    input logic [1:0] size
);
    logic [2:0] read_widx;
    logic [31:0] merged;
    begin
        read_widx = (size == 2'd0) ? {1'b0, reg_sel[1:0]} : reg_sel;
        merged = read_gpr_value(read_widx, 2'd2);

        // Plain loads may feed a successor directly from registered cache
        // data.  M3's value is the output of another combinational ALU, so it
        // is interlocked until architectural commit instead of creating a
        // private-ALU -> successor-ALU/EA path in one cycle.
        if (load_wb_valid && !load_wb_is_alu &&
            (load_wb_widx == read_widx))
            merged = load_wb_forward_data;

        // The legacy ROM load path reaches the same registered-WB boundary
        // through recipe_memory_write.  Forward it as well so address-size 16
        // and other non-VIPT loads obey the same no-bubble data-use rule.
        if (recipe_memory_write.valid &&
            recipe_memory_dst_onehot[read_widx]) begin
            case (recipe_memory_mode)
                EA_FWD_BLO: merged = {merged[31:8], opr_r[7:0]};
                EA_FWD_BHI: merged = {merged[31:16], opr_r[7:0],
                                      merged[7:0]};
                EA_FWD_W:   merged = {merged[31:16], opr_r[15:0]};
                default:    merged = opr_r;
            endcase
        end

        if (size == 2'd0)
            read_gpr_load_forwarded = reg_sel[2]
                                    ? {24'd0, merged[15:8]}
                                    : {24'd0, merged[7:0]};
        else if (size == 2'd1)
            read_gpr_load_forwarded = {16'd0, merged[15:0]};
        else
            read_gpr_load_forwarded = merged;
    end
endfunction

// D2 reads the architectural GPR bank with delay-slot and deferred-shift
// forwarding. Delay-slot data wins if both producers name the same register.
function automatic logic [31:0] read_ea_gpr(
    input logic       valid,
    input logic [2:0] idx
);
    logic [31:0] current_value, dly_value, shift_value, load_value;
    logic        dly_hit, shift_hit, load_hit;
    begin
        current_value = valid ? read_gpr_value(idx, 2'd2) : 32'd0;
        dly_hit = dly_gpr_forward.valid && valid &&
                  (dly_gpr_forward.dst == idx);
        shift_hit = recipe_shift_write.valid && valid &&
                    (recipe_shift_widx == idx);
        load_hit = load_wb_valid && !load_wb_is_alu && valid &&
                   (load_wb_widx == idx);

        // Format each producer before the priority mux. This keeps delay-slot
        // data out of the shift/load mode selection on the D2 EA path.
        case (dly_gpr_forward.mode)
            EA_FWD_BLO: dly_value =
                {current_value[31:8], dly_gpr_forward.data[7:0]};
            EA_FWD_BHI: dly_value =
                {current_value[31:16], dly_gpr_forward.data[7:0],
                 current_value[7:0]};
            EA_FWD_W: dly_value =
                {current_value[31:16], dly_gpr_forward.data[15:0]};
            default: dly_value = dly_gpr_forward.data;
        endcase
        if (recipe_shift_write.size == 2'd0)
            shift_value = recipe_shift_write.dst[2]
                ? {current_value[31:16], recipe_shift_data[7:0],
                   current_value[7:0]}
                : {current_value[31:8], recipe_shift_data[7:0]};
        else if (recipe_shift_write.size == 2'd1)
            shift_value = {current_value[31:16], recipe_shift_data[15:0]};
        else
            shift_value = recipe_shift_data;

        load_value = load_wb_forward_data;

        read_ea_gpr = dly_hit   ? dly_value :
                      shift_hit ? shift_value :
                      load_hit  ? load_value : current_value;
    end
endfunction

// Every direct load reads its destination after older architectural writes
// have settled but before cache data enters WB. Plain-load WB forwarding keeps
// a chained older load visible at this capture edge. M3 uses the same value as
// its private ALU destination.
wire [2:0] load_capture_widx = (load_alu_dst_capture_size == 2'd0)
                             ? {1'b0, load_alu_dst_capture_dst[1:0]}
                             : load_alu_dst_capture_dst;
wire [31:0] load_capture_base = read_gpr_load_forwarded(load_capture_widx,
                                                        2'd2);

always_ff @(posedge clk) begin
    if (!reset_n) begin
        load_wb_dst_base_r <= 32'd0;
        load_wb_forward_data_r <= 32'd0;
    end else if (load_alu_dst_capture) begin
        load_wb_dst_base_r <= load_capture_base;
        load_wb_forward_data_r <= merge_load_forward(
            load_capture_base, load_alu_dst_capture_data,
            load_alu_dst_capture_dst, load_alu_dst_capture_size);
    end
end

function automatic logic [31:0] read_alu_source(input logic [5:0] field);
    case (field)
        ALUSRC_EAX: read_alu_source = read_gpr_load_forwarded(3'd0, 2'd2);
        ALUSRC_ECX: read_alu_source = read_gpr_load_forwarded(3'd1, 2'd2);
        ALUSRC_EDX: read_alu_source = read_gpr_load_forwarded(3'd2, 2'd2);
        ALUSRC_EBX: read_alu_source = read_gpr_load_forwarded(3'd3, 2'd2);
        ALUSRC_ESP: read_alu_source = read_gpr_load_forwarded(3'd4, 2'd2);
        ALUSRC_EBP: read_alu_source = read_gpr_load_forwarded(3'd5, 2'd2);
        ALUSRC_ESI: read_alu_source = read_gpr_load_forwarded(3'd6, 2'd2);
        ALUSRC_EDI: read_alu_source = read_gpr_load_forwarded(3'd7, 2'd2);
        ALUSRC_IMM8: read_alu_source = instr.has_modrm ? instr.immediate : instr.displacement;
        ALUSRC_IMM: read_alu_source = instr.immediate;
        ALUSRC_TMPB: read_alu_source = tmpb;
        ALUSRC_TMPC: read_alu_source = tmpc;
        ALUSRC_TMPD: read_alu_source = tmpd;
        ALUSRC_OPR_R: read_alu_source = opr_r;
        ALUSRC_TMPG: read_alu_source = tmpg;
        ALUSRC_TMPH: read_alu_source = slctr;
        ALUSRC_PROTUN: read_alu_source = protun;
        ALUSRC_ALLONES: read_alu_source = 32'hffff_ffff;
        ALUSRC_FLAGS_MASK: read_alu_source = 32'h0007_7fd7;
        ALUSRC_CONST_4000: read_alu_source = 32'h4000;
        ALUSRC_CONST_N200: read_alu_source = 32'hffff_fdff;
        ALUSRC_CONST_8: read_alu_source = 32'd8;
        ALUSRC_CONST_40: read_alu_source = 32'h40;
        ALUSRC_CONST_F0000: read_alu_source = 32'h000f_0000;
        ALUSRC_CONST_0D: read_alu_source = 32'h0d;
        ALUSRC_CONST_5D: read_alu_source = 32'h5d;
        ALUSRC_SIGMA: read_alu_source = sigma;
        ALUSRC_CONST_FC: read_alu_source = 32'h8000_00fc;
        ALUSRC_CONST_1: read_alu_source = 32'd1;
        ALUSRC_CONST_2: read_alu_source = 32'd2;
        ALUSRC_CONST_16: read_alu_source = 32'd16;
        ALUSRC_CONST_3: read_alu_source = 32'd3;
        ALUSRC_CONST_4: read_alu_source = 32'd4;
        ALUSRC_CONST_6: read_alu_source = 32'd6;
        ALUSRC_CONST_7: read_alu_source = 32'd7;
        ALUSRC_CONST_0F: read_alu_source = 32'h0f;
        ALUSRC_CONST_65: read_alu_source = 32'h65;
        ALUSRC_CONST_1F: read_alu_source = 32'h1f;
        ALUSRC_CONST_FFFF0000: read_alu_source = 32'hffff_0000;
        ALUSRC_CONST_60: read_alu_source = 32'h60;
        ALUSRC_CONST_7FF: read_alu_source = 32'h7ff;
        ALUSRC_CONST_9: read_alu_source = 32'd9;
        ALUSRC_CONST_29: read_alu_source = 32'h29;
        ALUSRC_CONST_70: read_alu_source = 32'h70;
        ALUSRC_CONST_73: read_alu_source = 32'h73;
        ALUSRC_CONST_1FF: read_alu_source = 32'h1ff;
        ALUSRC_CONST_8200: read_alu_source = 32'h8200;
        ALUSRC_CONST_71: read_alu_source = 32'h47;
        ALUSRC_CONST_NEG1: read_alu_source = 32'hffff_ffff;
        ALUSRC_CONST_NEG2: read_alu_source = 32'hffff_fffe;
        ALUSRC_CONST_NEG4: read_alu_source = 32'hffff_fffc;
        ALUSRC_MASK16: read_alu_source = 32'h0000_ffff;
        ALUSRC_CONST_0: read_alu_source = 32'd0;
        ALUSRC_WORDSZ: read_alu_source = is_dword ? 32'd4 :
                                         op_size == 2'd0 ? 32'd1 : 32'd2;
        ALUSRC_NEGWSZ: read_alu_source = is_dword ? 32'hffff_fffc :
                                         op_size == 2'd0 ? 32'hffff_ffff :
                                                           32'hffff_fffe;
        ALUSRC_INCREM: read_alu_source = eflags[10] ?
            (op_size == 2'd0 ? 32'hffff_ffff :
             op_size == 2'd1 ? 32'hffff_fffe : 32'hffff_fffc) :
            (op_size == 2'd0 ? 32'd1 : op_size == 2'd1 ? 32'd2 : 32'd4);
        ALUSRC_BITS_V: read_alu_source = op_size == 2'd0 ? 32'd7 :
                                          op_size == 2'd2 ? 32'd31 : 32'd15;
        ALUSRC_DSTREG: read_alu_source = read_gpr_load_forwarded(dst_reg_sel_r, op_size);
        ALUSRC_SRCREG: read_alu_source = read_gpr_load_forwarded(src_reg_sel_r, op_size);
        ALUSRC_ZERO: read_alu_source = 32'd0;
        default: read_alu_source = 32'd0;
    endcase
endfunction

function automatic logic [31:0] read_source(input logic [5:0] field);
    case (field)
        SRC_EAX: read_source = read_gpr_load_forwarded(3'd0, 2'd2);
        SRC_ECX: read_source = read_gpr_load_forwarded(3'd1, 2'd2);
        SRC_EDX: read_source = read_gpr_load_forwarded(3'd2, 2'd2);
        SRC_ESP: read_source = read_gpr_load_forwarded(3'd4, 2'd2);
        SRC_EBP: read_source = read_gpr_load_forwarded(3'd5, 2'd2);
        SRC_ESI: read_source = read_gpr_load_forwarded(3'd6, 2'd2);
        SRC_EDI: read_source = read_gpr_load_forwarded(3'd7, 2'd2);
        SRC_EIP: read_source = eip;
        SRC_EFLAGS: read_source = eflags;
        SRC_CR0: read_source = cr0;
        SRC_CR2: read_source = cr2;
        SRC_TMPB: read_source = tmpb;
        SRC_TMPC: read_source = tmpc;
        SRC_TMPD: read_source = tmpd;
        SRC_TMPE: read_source = tmpe;
        SRC_TMPF: read_source = tmpf;
        SRC_FLAGSB: read_source = flags_backup;
        SRC_TMPG: read_source = tmpg;
        SRC_TMPH: read_source = tmph;
        SRC_TMP_TR: read_source = slctr;
        SRC_COUNTR: read_source = countr;
        SRC_PROTUN: read_source = protun;
        SRC_TMPeIP: read_source = tmpeip;
        SRC_TMPeSP: read_source = op_size_src == 2'd2 ? tmpesp :
                                                           {16'd0, tmpesp[15:0]};
        SRC_DR6: read_source = dr6;
        SRC_DR7: read_source = dr7;
        SRC_CSOPCD: read_source = csopcd;
        SRC_OPROFF: read_source = oproff;
        // MDTMP is factored beside this generic mux below. Keeping its
        // registered mul/div result out of this large source tree shortens
        // the common quotient/product-to-GPR writeback path.
        SRC_MDTMP: read_source = 32'd0;
        SRC_SIGMA: read_source = sigma;
        SRC_IMM: read_source = instr.immediate;
        SRC_ES: read_source = {16'd0, es};
        SRC_CS: read_source = {16'd0, cs};
        SRC_SS: read_source = {16'd0, ss};
        SRC_DS: read_source = {16'd0, ds};
        SRC_FS: read_source = {16'd0, fs};
        SRC_GS: read_source = {16'd0, gs};
        SRC_LDTR: read_source = {16'd0, ldtr};
        SRC_TR: read_source = {16'd0, tr};
        SRC_SLCTR: read_source = {16'd0, slctr[15:3], 3'b000};
        // Width-sensitive GPR sources are factored beside this generic mux.
        SRC_eAX_AL: read_source = 32'd0;
        SRC_eDX_AH: read_source = 32'd0;
        SRC_OPR_R: read_source = opr_r;
        SRC_IRF2: read_source = ind;
        SRC_EA: read_source = ea;
        SRC_eCX: read_source = read_gpr_load_forwarded(3'd1, 2'd2);
        SRC_IRF: read_source = 32'd0;
        SRC_USTEP_SEG_INDEX: read_source = {24'd0, 5'b10100, seg_reg_sel};
        SRC_FOP: read_source = {21'd0, instr.fop};
        SRC_SEGREG: begin
            case (seg_reg_sel)
                3'd0: read_source = {16'd0, es};
                3'd1: read_source = {16'd0, cs};
                3'd2: read_source = {16'd0, ss};
                3'd3: read_source = {16'd0, ds};
                3'd4: read_source = {16'd0, fs};
                3'd5: read_source = {16'd0, gs};
                default: read_source = 32'd0;
            endcase
        end
        SRC_DSTREG: read_source = 32'd0;
        SRC_SRCREG: read_source = 32'd0;
        SRC_NEG1: read_source = 32'hffff_ffff;
        default: read_source = 32'd0;
    endcase
endfunction

function automatic logic source_is_factored_gpr(input logic [5:0] field);
    case (field)
        SRC_eAX_AL, SRC_eDX_AH, SRC_IRF, SRC_DSTREG, SRC_SRCREG:
            source_is_factored_gpr = 1'b1;
        default: source_is_factored_gpr = 1'b0;
    endcase
endfunction

// These five sources share the byte/word/dword register formatter. Keeping
// them beside the generic microcode source mux prevents op_size from crossing
// the full source tree before descriptor and architectural writeback.
function automatic logic [31:0] read_factored_gpr_source(
    input logic [5:0] field
);
    case (field)
        SRC_eAX_AL: read_factored_gpr_source = read_gpr_load_forwarded(
            3'd0, op_size_src);
        SRC_eDX_AH: read_factored_gpr_source = read_gpr_load_forwarded(
            op_size_src == 2'd0 ? 3'd4 : 3'd2, op_size_src);
        SRC_IRF: read_factored_gpr_source = read_gpr_load_forwarded(
            countr[2:0], op_size_src == 2'd2 ? 2'd2 : 2'd1);
        SRC_DSTREG: read_factored_gpr_source = read_gpr_load_forwarded(
            dst_reg_sel_r, srcreg_size_src);
        SRC_SRCREG: read_factored_gpr_source = read_gpr_load_forwarded(
            src_reg_sel_r, op_size_src);
        default: read_factored_gpr_source = 32'd0;
    endcase
endfunction

// Dedicated protection and CS readers keep the full generic source mux off
// their timing-sensitive consumers while reusing the local GPR read ports.
function automatic logic [31:0] read_protection_source(
    input logic [5:0] field,
    input logic [31:0] generic_value
);
    case (field)
        SRC_ZERO:    read_protection_source = 32'd0;
        SRC_NEG1:    read_protection_source = 32'hffff_ffff;
        SRC_CR0:     read_protection_source = cr0;
        SRC_TMPH:    read_protection_source = tmph;
        SRC_TMP_TR:  read_protection_source = slctr;
        SRC_COUNTR,
        SRC_PROTUN:  read_protection_source = protun;
        SRC_SIGMA:   read_protection_source = sigma;
        SRC_CS:      read_protection_source = {16'd0, cs};
        SRC_OPR_R:   read_protection_source = opr_r;
        SRC_IRF2:    read_protection_source = ind;
        SRC_TMPE:    read_protection_source = tmpe;
        SRC_DSTREG:  read_protection_source = read_gpr_value(dst_reg_sel_r,
                                                              srcreg_size);
        SRC_SRCREG:  read_protection_source = read_gpr_value(src_reg_sel_r,
                                                              op_size);
        default:     read_protection_source = generic_value;
    endcase
endfunction

function automatic logic [15:0] read_cs_source(
    input logic [5:0] field,
    input logic [31:0] generic_value
);
    case (field)
        SRC_SIGMA:  read_cs_source = sigma[15:0];
        SRC_TMPH:   read_cs_source = tmph[15:0];
        SRC_OPR_R:  read_cs_source = opr_r[15:0];
        SRC_PROTUN: read_cs_source = protun[15:0];
        default:    read_cs_source = generic_value[15:0];
    endcase
endfunction

// WR W uses a small, fixed subset of the microcode source field. Keep the
// generic source mux off the write-data path into paging and the cache.
function automatic logic [31:0] read_memory_write_source(input logic [5:0] field);
    case (field)
        SRC_TMPB:   read_memory_write_source = tmpb;
        SRC_CR0:    read_memory_write_source = cr0;
        SRC_IMM:    read_memory_write_source = instr.immediate;
        SRC_FOP:    read_memory_write_source = {21'd0, instr.fop};
        SRC_PROTUN: read_memory_write_source = protun;
        SRC_SIGMA:  read_memory_write_source = sigma;
        SRC_ES:     read_memory_write_source = {16'd0, es};
        SRC_CS:     read_memory_write_source = {16'd0, cs};
        SRC_SS:     read_memory_write_source = {16'd0, ss};
        SRC_DS:     read_memory_write_source = {16'd0, ds};
        SRC_FS:     read_memory_write_source = {16'd0, fs};
        SRC_GS:     read_memory_write_source = {16'd0, gs};
        SRC_LDTR:   read_memory_write_source = {16'd0, ldtr};
        SRC_TR:     read_memory_write_source = {16'd0, tr};
        SRC_SEGREG: begin
            case (seg_reg_sel)
                3'd0: read_memory_write_source = {16'd0, es};
                3'd1: read_memory_write_source = {16'd0, cs};
                3'd2: read_memory_write_source = {16'd0, ss};
                3'd3: read_memory_write_source = {16'd0, ds};
                3'd4: read_memory_write_source = {16'd0, fs};
                3'd5: read_memory_write_source = {16'd0, gs};
                default: read_memory_write_source = 32'd0;
            endcase
        end
        SRC_IRF: read_memory_write_source = read_gpr_load_forwarded(
            countr[2:0], op_size_src == 2'd2 ? 2'd2 : 2'd1);
        default: read_memory_write_source = 32'd0;
    endcase
endfunction

always_comb begin
    source_value_live = source_live == SRC_MDTMP ? muldiv_result :
                        source_is_factored_gpr(source_live)
                      ? read_factored_gpr_source(source_live)
                      : read_source(source_live);
    memory_write_source_value = read_memory_write_source(source_live);
    alu_source_value_live = read_alu_source(alu_source_live);
    alu_dst = source_field == SRC_MDTMP ? muldiv_result :
              source_is_factored_gpr(source_field)
            ? read_factored_gpr_source(source_field)
            : read_source(source_field);
    dest_value = alu_dst;
    alu_src = fpu_f8 ? 32'h8000_00f8 : read_alu_source(alu_source);
    protection_source_value = read_protection_source(source_live,
                                                     source_value_live);
    protection_source_low16_nonzero = |protection_source_value[15:0];
    cs_source_value = read_cs_source(source_live, source_value_live);
    ea_base_value = read_ea_gpr(ea_base.valid, ea_base.index);
    ea_index_value = read_ea_gpr(ea_index.valid, ea_index.index);
end

//=============================================================================
// Internal registers and architectural GPR writeback
//=============================================================================

always_ff @(posedge clk) begin
    if (!reset_n) begin
        tmpb   <= 32'd0;
        tmpc   <= 32'd0;
        tmpd   <= 32'd0;
        tmpe   <= 32'd0;
        tmpf   <= 32'd0;
        csopcd <= 32'd0;
        fsveip <= 32'd0;
        oproff <= 32'd0;
        opr_w  <= 32'd0;
    end else if (exec) begin
        case (dest)
            DEST_TMPB:   tmpb   <= dest_value;
            DEST_TMPC:   tmpc   <= dest_value;
            DEST_TMPD:   tmpd   <= dest_value;
            DEST_TMPE:   tmpe   <= dest_value;
            DEST_TMPF:   tmpf   <= dest_value;
            DEST_TMPG:   tmpg   <= dest_value;
            DEST_TMPH:   tmph   <= dest_value;
            DEST_CSOPCD: csopcd <= dest_value;
            DEST_FSVeIP: fsveip <= dest_value;
            DEST_OPROFF: oproff <= dest_value;
            DEST_OPR_W:  opr_w  <= dest_value;
            default: ;
        endcase

        if (aluop == ALUJMP_PTSELE && !gate_detect)
            tmph <= alu_dst;

        if (gate_detect) begin
            tmpb <= desc_raw_hi;
            tmph <= {16'd0, tmpc[31:16]};
            tmpg <= {desc_raw_hi[31:16], tmpc[15:0]};
        end

        if (muldiv_tmpb_write)
            tmpb <= muldiv_tmpb_value;
    end
end

task automatic write_gpr(
    input logic [2:0] reg_sel,
    input logic [31:0] value,
    input logic [1:0] size
);
    case (size)
        2'd0: begin
            case (reg_sel)
                3'd0: eax[7:0]    <= value[7:0];
                3'd1: ecx[7:0]    <= value[7:0];
                3'd2: edx[7:0]    <= value[7:0];
                3'd3: ebx[7:0]    <= value[7:0];
                3'd4: eax[15:8]   <= value[7:0];
                3'd5: ecx[15:8]   <= value[7:0];
                3'd6: edx[15:8]   <= value[7:0];
                3'd7: ebx[15:8]   <= value[7:0];
            endcase
        end
        2'd1: begin
            case (reg_sel)
                3'd0: eax[15:0]     <= value[15:0];
                3'd1: ecx[15:0]     <= value[15:0];
                3'd2: edx[15:0]     <= value[15:0];
                3'd3: ebx[15:0]     <= value[15:0];
                3'd4: esp[15:0]     <= value[15:0];
                3'd5: ebp[15:0]     <= value[15:0];
                3'd6: esi[15:0]     <= value[15:0];
                3'd7: edi[15:0]     <= value[15:0];
            endcase
        end
        2'd2: begin
            case (reg_sel)
                3'd0: eax     <= value;
                3'd1: ecx <= value;
                3'd2: edx     <= value;
                3'd3: ebx     <= value;
                3'd4: esp     <= value;
                3'd5: ebp     <= value;
                3'd6: esi     <= value;
                3'd7: edi     <= value;
            endcase
        end
        default: ;
    endcase
endtask

// Deferred recipe commits are Data Unit writeback state. Chain control observes
// the compact pending descriptors for dependency checks and D2 forwarding.
always_ff @(posedge clk) begin
    if (!reset_n) begin
        recipe_shift_write <= '0;
        recipe_shift_widx <= 3'd0;
        recipe_memory_write <= '0;
        recipe_memory_dst_onehot <= 8'd0;
        recipe_memory_mode <= EA_FWD_D;
    end else if (pipeline_advance) begin
        recipe_memory_write.valid <= recipe_rni && exec && instr_start &&
                                   !recipe_commit_cancel &&
                                   (recipe_state.commit_sel == RECIPE_COMMIT_MEM);
        if (recipe_rni && exec && instr_start &&
            (recipe_state.commit_sel == RECIPE_COMMIT_MEM)) begin
            recipe_memory_write.dst <= dst_reg_sel_r;
            recipe_memory_write.size <= op_size;
            recipe_memory_dst_onehot <= 8'b1 << ((op_size == 2'd0)
                                               ? {1'b0, dst_reg_sel_r[1:0]}
                                               : dst_reg_sel_r);
            recipe_memory_mode <= (op_size == 2'd0)
                                ? (dst_reg_sel_r[2] ? EA_FWD_BHI : EA_FWD_BLO)
                                : (op_size == 2'd1 ? EA_FWD_W : EA_FWD_D);
        end

        recipe_shift_write.valid <= recipe_rni && exec && !recipe_commit_cancel &&
                                  (recipe_state.commit_sel == RECIPE_COMMIT_SHIFT);
        if (recipe_rni && exec &&
            (recipe_state.commit_sel == RECIPE_COMMIT_SHIFT)) begin
            recipe_shift_write.dst <= dst_reg_sel_r;
            recipe_shift_write.size <= op_size;
            recipe_shift_widx <= (op_size == 2'd0)
                               ? {1'b0, dst_reg_sel_r[1:0]}
                               : dst_reg_sel_r;
            recipe_shift_data <= shift_result;
        end
    end
end

// synthesis translate_off
always_ff @(posedge clk) begin
    if (reset_n && recipe_shift_write.valid &&
        (recipe_shift_widx !== ((recipe_shift_write.size == 2'd0)
                              ? {1'b0, recipe_shift_write.dst[1:0]}
                              : recipe_shift_write.dst)))
        $fatal(1, "Deferred shift normalized destination mismatch");
end
// synthesis translate_on

// All GPR producers retain their original ordering. Later assignments are
// younger and therefore win when two producers target the same register.
always_ff @(posedge clk) begin
    if (!reset_n) begin
        eax     <= 32'd0;
        ecx     <= 32'd0;
        edx     <= 32'h0000_0303;
        ebx     <= 32'd0;
        esp     <= 32'd0;
        ebp     <= 32'd0;
        esi     <= 32'd0;
        edi     <= 32'd0;
    end else begin
        if (recipe_shift_write.valid)
            write_gpr(recipe_shift_write.dst, recipe_shift_data,
                      recipe_shift_write.size);

        if (load_wb_valid && !recipe_commit_cancel)
            write_gpr(load_wb_dst, load_wb_commit_data, load_wb_size);

        // A chained successor may own EX while an older hardwired load's
        // pending token retires. The token is already fully qualified.
        if (recipe_memory_write.valid && !recipe_commit_cancel)
            write_gpr(recipe_memory_write.dst, opr_r,
                      recipe_memory_write.size);

        // A younger VIPT candidate can shadow the ROM writeback slot of a
        // hardwired load.  If an interrupt redirects that boundary, retire
        // the completed OPR_R value before the handler starts using OPR_R.
        if (interrupt_entry && recipe_rni && recipe_state.hardwired &&
            (recipe_state.commit_sel == RECIPE_COMMIT_MEM) &&
            !recipe_commit_cancel)
            write_gpr(dst_reg_sel_r, opr_r, op_size);

        if (exec) begin
            case (dest)
                DEST_EAX: eax <= dest_value;
                DEST_EDX: edx <= dest_value;
                DEST_ESP: esp <= dest_value;
                DEST_eSP:
                    if (pe && stack32)
                        esp <= dest_value;
                    else
                        esp[15:0] <= dest_value[15:0];
                DEST_EBP: ebp <= dest_value;

                DEST_DSTREG: write_gpr(dst_reg_sel_r, dest_value, op_size);
                DEST_SRCREG: write_gpr(src_reg_sel_r, dest_value, op_size);
                DEST_AX:     write_gpr(3'd0, dest_value, 2'd1);
                DEST_BP:     write_gpr(3'd5, dest_value, 2'd1);
                DEST_eAX_AL: write_gpr(3'd0, dest_value, op_size);
                DEST_eDX_AH: write_gpr(op_size == 2'd0 ? 3'd4 : 3'd2,
                                       dest_value, op_size);
                DEST_eCX: write_gpr(3'd1, dest_value, instr.addr32 ? 2'd2 : 2'd1);
                DEST_eSI: write_gpr(3'd6, dest_value, instr.addr32 ? 2'd2 : 2'd1);
                DEST_eDI: write_gpr(3'd7, dest_value, instr.addr32 ? 2'd2 : 2'd1);
                DEST_AL:  write_gpr(3'd0, dest_value, 2'd0);
                DEST_AH:  write_gpr(3'd4, dest_value, 2'd0);

                DEST_USTEP_BSWAP:
                    write_gpr(src_reg_sel_r,
                              {dest_value[7:0], dest_value[15:8],
                               dest_value[23:16], dest_value[31:24]}, 2'd2);

                DEST_USTEP_ALU:
                    if (recipe_state.hardwired && !hardwired_off &&
                        !recipe_commit_cancel)
                        write_gpr(dst_reg_sel_r, alu_result, op_size);

                DEST_IRF:
                    if (countr[5:3] != 3'b100)
                        write_gpr(countr[2:0], dest_value,
                                  is_dword ? 2'd2 : 2'd1);
                default: ;
            endcase

            // REP STOS records its restartable count only after the element
            // store has cleared DLY. The loop can then bypass its redundant
            // COUNTR->eCX word without exposing a decremented count on a
            // faulting store.
            if (!instr.has_0f &&
                ((instr.opcode == 8'hAA) || (instr.opcode == 8'hAB)) &&
                (instr.rep_lock == PREFIX_REP) &&
                (dest == DEST_eDI) && (source_field == SRC_SIGMA))
                write_gpr(3'd1, countr, instr.addr32 ? 2'd2 : 2'd1);

            if (recipe_rni && !recipe_commit_cancel &&
                recipe_state.commit_sel == RECIPE_COMMIT_SIGSRC)
                write_gpr(src_reg_sel_r, sigma,
                          aluop == ALUJMP_BITS32 ? 2'd2 : 2'd1);

            if (recipe_rni && !recipe_commit_cancel &&
                recipe_state.commit_sel == RECIPE_COMMIT_ESP)
                esp <= sigma;

        end
    end
end

//=============================================================================
// Arithmetic state and flags
//=============================================================================

// Arithmetic state is retired beside its producer so result selection does
// not cross the top-level module boundary.
wire flag2_class_uc = (aluop == ALUJMP_ALU)    || (aluop == ALUJMP_INCDEC) ||
                      (aluop == ALUJMP_CMPTST) || (aluop == ALUJMP_AND)    ||
                      (aluop == ALUJMP_OR)     || (aluop == ALUJMP_XOR)    ||
                      (aluop == ALUJMP_ADD)    || (aluop == ALUJMP_ADC)    ||
                      (aluop == ALUJMP_SUB)    || (aluop == ALUJMP_CMP)    ||
                      (aluop == ALUJMP_AAAAAS) || (aluop == ALUJMP_DAADAS);

wire flag2_zf = flag2_size_r == 2'd0 ? flag2_result_r[7:0] == 8'd0 :
                flag2_size_r == 2'd1 ? flag2_result_r[15:0] == 16'd0 :
                                            flag2_result_r == 32'd0;
wire flag2_sf = flag2_size_r == 2'd0 ? flag2_result_r[7] :
                flag2_size_r == 2'd1 ? flag2_result_r[15] :
                                            flag2_result_r[31];
wire flag2_pf = ~^flag2_result_r[7:0];

always_ff @(posedge clk) begin
    if (!reset_n) begin
        sigma <= 32'd0;
    end else begin
        if (instr_start && stack_op) begin
            logic [31:0] stack_delta;
            logic [15:0] new_sp;

            stack_delta = stack_data32 ? 32'd4 : 32'd2;
            if (stack32) begin
                sigma <= stack_dir ? forwarded_esp + stack_delta
                                   : forwarded_esp - stack_delta;
            end else begin
                new_sp = stack_dir ? forwarded_esp[15:0] + stack_delta[15:0]
                                   : forwarded_esp[15:0] - stack_delta[15:0];
                sigma <= {forwarded_esp[31:16], new_sp};
            end
        end else if (gate_detect) begin
            sigma <= {16'd0, tmpc[31:16]};
        end else if (exec) begin
            if (shift_sigma_sel == 2'd1)
                sigma <= shift_setup_result;
            else if (shift_sigma_sel == 2'd2)
                sigma <= shift_result;
            else case (aluop)
                ALUJMP_ALU,
                ALUJMP_INCDEC,
                ALUJMP_IMCS,
                ALUJMP_SZ_EXT,
                ALUJMP_AND,
                ALUJMP_OR,
                ALUJMP_XOR,
                ALUJMP_SIGN,
                ALUJMP_ADD,
                ALUJMP_ADC,
                ALUJMP_SUB,
                ALUJMP_CMP,
                ALUJMP_PASS,
                ALUJMP_PASS2,
                ALUJMP_AAAAAS,
                ALUJMP_DAADAS,
                ALUJMP_SERECO: sigma <= alu_result;

                ALUJMP_IMUL3,
                ALUJMP_IMUL4,
                ALUJMP_SZ_EX2,
                ALUJMP_DIV5,
                ALUJMP_PREDIV,
                ALUJMP_IDIV1,
                ALUJMP_IDIV2,
                ALUJMP_DIV7:
                    if (muldiv_sigma_write)
                        sigma <= muldiv_sigma_value;
                default: ;
            endcase
        end

        // Fault handling has final priority over a current arithmetic result.
        if (any_fault)
            sigma <= 32'd0;
    end
end

// Branch control consumes only the selected condition, not the full forwarded
// flags bus. Evaluate it beside the forwarding mux so a just-retired shift or
// ALU result crosses the module boundary as one bit on the redirect path.
assign branch_condition_true = condition_true(instr.branch_condition,
                                              eflags_fwd);

always_ff @(posedge clk) begin
    if (!reset_n) begin
        countr <= 32'd0;
    end else if (interrupt_entry) begin
        countr[4:0] <= 5'd0;
    end else if (gate_detect) begin
        countr <= {16'd0, tmpc[31:16]};
    end else if (muldiv_counter_early_exit) begin
        countr[4:0] <= 5'd0;
    end else if (exec) begin
        if (aluop == ALUJMP_LDCNTR)
            countr <= alu_source[5] ? {26'd0, alu_source_value_live[5:0]}
                                    : alu_source_value_live;
        else if (aluop == ALUJMP_DECNTR)
            countr <= countr - 32'd1;
        else if (dest == DEST_COUNT5)
            countr <= {27'd0, dest_value[4:0]};
        else if (dest == DEST_COUNTR)
            countr <= dest_value;
        else if (repeat_active &&
                 (aluop == ALUJMP_DIV7 || aluop == ALUJMP_IMUL3 ||
                  aluop == ALUJMP_IMUL4 || aluop == ALUJMP_PREDIV))
            countr[4:0] <= countr[4:0] - 5'd1;
    end
end

always_ff @(posedge clk) begin
    if (instr_start && !halted &&
        (next_instr.rel_branch_kind == REL_BRANCH_JCC)) begin
        if (next_instr.branch_rel8)
            alu_src_hold <= {{24{next_instr.displacement[7]}},
                             next_instr.displacement[7:0]};
        else
            alu_src_hold <= next_instr.displacement;
    end else if (!((instr.rel_branch_kind == REL_BRANCH_JCC) && uc_active)) begin
        alu_src_hold <= alu_src;
    end
end

always_ff @(posedge clk) begin
    if (!reset_n) begin
        flag2_eflags_p  <= 1'b0;
        flag2_ucflags_p <= 1'b0;
    end else begin
        flag2_eflags_p  <= (exec && update_arch_flags) || load_wb_alu_commit;
        flag2_ucflags_p <= (exec && flag2_class_uc) || load_wb_alu_commit;
        if ((exec && flag2_class_uc) || load_wb_alu_commit) begin
            flag2_result_r <= load_wb_alu_commit ? load_wb_alu_result
                                                 : alu_result;
            flag2_cf_r     <= load_wb_alu_commit ? load_wb_alu_flags[0]
                                                 : alu_flags[0];
            flag2_af_r     <= load_wb_alu_commit ? load_wb_alu_flags[4]
                                                 : alu_flags[4];
            flag2_of_r     <= load_wb_alu_commit ? load_wb_alu_flags[11]
                                                 : alu_flags[11];
            flag2_zsp_r    <= load_wb_alu_commit ? 1'b1 : alu_zsp_update;
            flag2_size_r   <= load_wb_alu_commit ? load_wb_size : op_size;
        end
    end
end

always_comb begin
    if (sh_flags_commit) begin
        eflags_fwd = {eflags[31:12],
                      sh_flags_we_of ? sh_flags_of : eflags[11],
                      eflags[10:8],
                      sh_flags_we_zsp ? sh_flags_sf : eflags[7],
                      sh_flags_we_zsp ? sh_flags_zf : eflags[6],
                      eflags[5:3],
                      sh_flags_we_zsp ? sh_flags_pf : eflags[2],
                      eflags[1], sh_flags_cf};
    end else if (flag2_eflags_p) begin
        eflags_fwd = {eflags[31:12], flag2_of_r, eflags[10:8],
                      flag2_zsp_r ? flag2_sf : eflags[7],
                      flag2_zsp_r ? flag2_zf : eflags[6],
                      eflags[5], flag2_af_r, eflags[3],
                      flag2_zsp_r ? flag2_pf : eflags[2],
                      eflags[1], flag2_cf_r};
    end else begin
        eflags_fwd = eflags;
    end

end

always_ff @(posedge clk) begin
    if (!reset_n) begin
        uc_flags <= 32'h0000_0002;
    end else begin
        if (instr_start)
            uc_flags <= eflags;
        if (flag2_ucflags_p) begin
            uc_flags[0]  <= flag2_cf_r;
            uc_flags[4]  <= flag2_af_r;
            uc_flags[11] <= flag2_of_r;
            if (flag2_zsp_r) begin
                uc_flags[2] <= flag2_pf;
                uc_flags[6] <= flag2_zf;
                uc_flags[7] <= flag2_sf;
            end
        end
        if (sh_flags_commit) begin
            uc_flags[0] <= sh_flags_cf;
            if (sh_flags_we_zsp) begin
                uc_flags[2] <= sh_flags_pf;
                uc_flags[6] <= sh_flags_zf;
                uc_flags[7] <= sh_flags_sf;
            end
            if (sh_flags_we_of)
                uc_flags[11] <= sh_flags_of;
        end
        if (!instr_start && exec) begin
            case (aluop)
                ALUJMP_BITTST: uc_flags[0] <= shift_bit_test_cf;
                default: ;
            endcase
        end
        if (!instr_start && exec && shift_uc_carry)
            uc_flags[0] <= op_size == 2'd0 ? tmpb[7] :
                           op_size == 2'd1 ? tmpb[15] : tmpb[31];
    end
end

always_ff @(posedge clk) begin
    if (!reset_n) begin
        eflags <= 32'h0000_0002;
        clear_if_pending <= 1'b0;
    end else begin
        if (instr_start && !halted)
            clear_if_pending <= 1'b0;

        if (flag2_eflags_p) begin
            eflags[0]  <= flag2_cf_r;
            eflags[1]  <= 1'b1;
            eflags[4]  <= flag2_af_r;
            eflags[11] <= flag2_of_r;
            if (flag2_zsp_r) begin
                eflags[2] <= flag2_pf;
                eflags[6] <= flag2_zf;
                eflags[7] <= flag2_sf;
            end
        end

        if (sh_flags_commit) begin
            eflags[0] <= sh_flags_cf;
            if (sh_flags_we_zsp) begin
                eflags[2] <= sh_flags_pf;
                eflags[6] <= sh_flags_zf;
                eflags[7] <= sh_flags_sf;
            end
            if (sh_flags_we_of)
                eflags[11] <= sh_flags_of;
        end

        if (exec) begin
            case (aluop)
                ALUJMP_FLGOPS: begin
                    case (instr.flag_op)
                        FLAG_OP_CMC: eflags[0]  <= ~eflags[0];
                        FLAG_OP_CLC: eflags[0]  <= 1'b0;
                        FLAG_OP_STC: eflags[0]  <= 1'b1;
                        FLAG_OP_CLI: eflags[9]  <= 1'b0;
                        FLAG_OP_STI: eflags[9]  <= 1'b1;
                        FLAG_OP_CLD: eflags[10] <= 1'b0;
                        FLAG_OP_STD: eflags[10] <= 1'b1;
                        default: ;
                    endcase
                end
                ALUJMP_BITTST: eflags[0] <= shift_bit_test_cf;
                ALUJMP_DIV5: begin
                    eflags[0] <= 1'b0;
                    if (instr.div_quotient_zf) begin
                        // Real 486/AMD silicon perturbs SF/ZF/PF/AF as an
                        // undocumented side effect of unsigned DIV; Cyrix
                        // 486 leaves them unchanged. Pre-CPUID software
                        // (CHKCPU, Memtest86+, Win9x/NT Setup) probes this
                        // to distinguish "real" 486/AMD from Cyrix -- see
                        // nand2mario/z486#82. AF's forced-1 value is an
                        // unverified best guess (no real-silicon reference
                        // found); SF/ZF/PF follow the quotient.
                        eflags[6] <= muldiv_quotient_zero;
                        eflags[7] <= muldiv_quotient_sign;
                        eflags[2] <= muldiv_quotient_parity;
                        eflags[4] <= 1'b1;
                    end
                end
                ALUJMP_CLZF: eflags[6] <= 1'b0;
                ALUJMP_SEZF: eflags[6] <= 1'b1;
                ALUJMP_CLI: clear_if_pending <= 1'b1;
                ALUJMP_CLT: begin
                    eflags[8] <= 1'b0;
                    if (clear_if_pending)
                        eflags[9] <= 1'b0;
                    clear_if_pending <= 1'b0;
                end
                ALUJMP_SHIFT2: ;
                ALUJMP_SHIFT: ;
                ALUJMP_USTEP_AAD_SHIFT: eflags[0] <= 1'b0;
                ALUJMP_SZ_EX2,
                ALUJMP_IMCS: begin
                    eflags[0]  <= muldiv_flag_overflow;
                    eflags[11] <= muldiv_flag_overflow;
                end
                default: ;
            endcase

            if (dest == DEST_FLAGSL)
                eflags[7:0] <= (dest_value[7:0] & 8'hD5) | 8'h02;

            if (dest == DEST_FLAGS) begin
                // POPFD's microcode forces BITS16 before FLAGS writeback;
                // retain the decoded width for the 486-only AC bit.
                if (instr.data32)
                    eflags[18] <= dest_value[18];

                if (pe) begin
                    eflags[7:0]   <= (dest_value[7:0] & 8'hD5) | 8'h02;
                    eflags[8]     <= dest_value[8];
                    eflags[9]     <= cpl <= eflags[13:12] ?
                                     dest_value[9] : eflags[9];
                    eflags[11:10] <= dest_value[11:10];
                    eflags[13:12] <= cpl == 2'b00 ?
                                     dest_value[13:12] : eflags[13:12];
                    eflags[14]    <= dest_value[14];
                    if (is_dword && cpl == 2'b00) begin
                        eflags[16] <= dest_value[16];
                        eflags[17] <= dest_value[17];
                    end
                end else begin
                    eflags[15:0] <= (dest_value[15:0] & 16'h7FD5) | 16'h0002;
                end
            end

            if (dest == DEST_EFLAGS)
                eflags <= (dest_value & 32'h0007_7fd5) | 32'h0000_0002;
        end

        if (clear_rf)
            eflags[16] <= 1'b0;
    end
end

always_ff @(posedge clk) begin
    if (!reset_n) begin
        flags_backup_active <= 1'b0;
        flags_backup <= 32'd0;
    end else if (ifetch_page_fault) begin
        flags_backup_active <= 1'b1;
        flags_backup <= eflags;
    end else if (interrupt_entry) begin
        flags_backup_active <= 1'b0;
    end else if (instr_start && !halted) begin
        flags_backup_active <= 1'b1;
        flags_backup <= eflags_fwd;
    end else if (exec && aluop == ALUJMP_FLGSBA) begin
        if (!flags_backup_active) begin
            flags_backup_active <= 1'b1;
            flags_backup <= eflags;
        end
    end else if (exec && dest == DEST_FLAGSB) begin
        if (!flags_backup_active)
            flags_backup <= dest_value;
    end
end

//=============================================================================
// Arithmetic engines
//=============================================================================

// Direct register-memory ALU operations execute after the registered VIPT
// result boundary.  Keep their seven-operation add/logic datapath separate
// from the microcode ALU: sharing it creates a combinational loop through the
// WB-to-successor GPR bypass, and qualifying computation with fault state drags
// the divide-overflow cone onto every load result.  Faults gate commit below;
// they need not gate this side-effect-free calculation.
assign load_wb_alu_exec = load_wb_valid && load_wb_is_alu;
assign load_wb_alu_commit = load_wb_alu_exec && !recipe_commit_cancel;

wire [31:0] load_wb_alu_dst = (load_wb_size == 2'd0)
    ? (load_wb_dst[2] ? {24'd0, load_wb_dst_base_r[15:8]}
                      : {24'd0, load_wb_dst_base_r[7:0]})
    : (load_wb_size == 2'd1) ? {16'd0, load_wb_dst_base_r[15:0]}
                             : load_wb_dst_base_r;
logic [31:0] load_wb_add_a;
logic [31:0] load_wb_add_b;
logic        load_wb_add_cin;
logic        load_wb_arith;
logic        load_wb_sub;
logic [31:0] load_wb_logic_result;

always_comb begin
    load_wb_add_a = load_wb_alu_dst;
    load_wb_add_b = load_wb_data;
    load_wb_add_cin = 1'b0;
    load_wb_arith = 1'b0;
    load_wb_sub = 1'b0;
    load_wb_logic_result = load_wb_alu_dst;

    case (load_wb_alu_op)
        ALU_ADD: load_wb_arith = 1'b1;
        ALU_ADC: begin
            load_wb_arith = 1'b1;
            load_wb_add_cin = eflags_fwd[0];
        end
        ALU_SUBT: begin
            load_wb_arith = 1'b1;
            load_wb_sub = 1'b1;
            load_wb_add_b = ~load_wb_data;
            load_wb_add_cin = 1'b1;
        end
        ALU_SBB: begin
            load_wb_arith = 1'b1;
            load_wb_sub = 1'b1;
            load_wb_add_b = ~load_wb_data;
            load_wb_add_cin = ~eflags_fwd[0];
        end
        ALU_AND: load_wb_logic_result = load_wb_alu_dst & load_wb_data;
        ALU_OR:  load_wb_logic_result = load_wb_alu_dst | load_wb_data;
        ALU_XOR: load_wb_logic_result = load_wb_alu_dst ^ load_wb_data;
        default: ;
    endcase
end

wire [32:0] load_wb_sum33 = {1'b0, load_wb_add_a} +
                            {1'b0, load_wb_add_b} +
                            {32'd0, load_wb_add_cin};
wire [8:0] load_wb_sum8 = {1'b0, load_wb_add_a[7:0]} +
                          {1'b0, load_wb_add_b[7:0]} +
                          {8'd0, load_wb_add_cin};
wire [16:0] load_wb_sum16 = {1'b0, load_wb_add_a[15:0]} +
                            {1'b0, load_wb_add_b[15:0]} +
                            {16'd0, load_wb_add_cin};
assign load_wb_alu_result = load_wb_arith ? load_wb_sum33[31:0]
                                          : load_wb_logic_result;

wire load_wb_result_sign = load_wb_size == 2'd0
                         ? load_wb_alu_result[7]
                         : load_wb_size == 2'd1
                         ? load_wb_alu_result[15] : load_wb_alu_result[31];
wire load_wb_dst_sign = load_wb_size == 2'd0
                      ? load_wb_alu_dst[7]
                      : load_wb_size == 2'd1
                      ? load_wb_alu_dst[15] : load_wb_alu_dst[31];
wire load_wb_src_sign = load_wb_size == 2'd0
                      ? load_wb_data[7]
                      : load_wb_size == 2'd1
                      ? load_wb_data[15] : load_wb_data[31];
wire load_wb_carry = load_wb_size == 2'd0 ? load_wb_sum8[8] :
                     load_wb_size == 2'd1 ? load_wb_sum16[16] :
                                            load_wb_sum33[32];
wire load_wb_overflow = load_wb_arith &&
    (load_wb_sub
        ? ((load_wb_dst_sign ^ load_wb_src_sign) &
           (load_wb_dst_sign ^ load_wb_result_sign))
        : (~(load_wb_dst_sign ^ load_wb_src_sign) &
           (load_wb_dst_sign ^ load_wb_result_sign)));
wire load_wb_aux_carry = load_wb_arith &&
    (load_wb_alu_dst[4] ^ load_wb_data[4] ^ load_wb_alu_result[4]);
wire load_wb_zero = load_wb_size == 2'd0
                  ? load_wb_alu_result[7:0] == 8'd0
                  : load_wb_size == 2'd1
                  ? load_wb_alu_result[15:0] == 16'd0
                  : load_wb_alu_result == 32'd0;
wire load_wb_parity = ~^load_wb_alu_result[7:0];

always_comb begin
    load_wb_alu_flags = eflags_fwd;
    load_wb_alu_flags[11] = load_wb_overflow;
    load_wb_alu_flags[7] = load_wb_result_sign;
    load_wb_alu_flags[6] = load_wb_zero;
    load_wb_alu_flags[4] = load_wb_aux_carry;
    load_wb_alu_flags[2] = load_wb_parity;
    load_wb_alu_flags[0] = load_wb_arith
                         ? (load_wb_sub ? ~load_wb_carry : load_wb_carry)
                         : 1'b0;
end

assign load_wb_commit_data = load_wb_is_alu ? load_wb_alu_result
                                            : load_wb_data;

// Capture the operand for a q_mem SHIFT2 word or the two SRCREG SHIFT words.
// A predecessor may update the selected temporary or SIGMA on this same edge,
// so forward that exact value. SRCREG uses the existing load-WB bypass and the
// architectural operand width; its preceding LDBSRU records that width.
always_comb begin
    case (shift2_next_source)
        2'd0: shift2_capture_value = (exec && dest == DEST_TMPC)
                                    ? dest_value : tmpc;
        2'd1: shift2_capture_value = (exec && dest == DEST_TMPE)
                                    ? dest_value : tmpe;
        2'd2: shift2_capture_value = (exec && aluop == ALUJMP_SHIFT1)
                                    ? shift_setup_result : sigma;
        2'd3: shift2_capture_value = read_gpr_load_forwarded(src_reg_sel_r,
                                                              op_size);
        default: shift2_capture_value = 32'd0;
    endcase
end

`ifdef Z486_USE_ALTERA_ALU
alu_alt alu_inst (
`else
alu alu_inst (
`endif
    .op(alu_operation),
    .src(alu_src),
    .dst(alu_dst),
    .op_size(op_size),
    .flags(eflags_fwd),
    .update_carry(update_carry),
    .result(alu_result),
    .flags_out(alu_flags),
    .zsp_update(alu_zsp_update)
);

shifter shifter_inst (
    .clk(clk),
    .reset_n(reset_n),
    .exec(shift_exec),
    .aluop(aluop),
    .shift_aluop(shift_aluop),
    .source_field(source_field),
    .source_class(shift_source_class),
    .shift2_source(shift2_source),
    .is_shift2(shift_is_shift2),
    .capture_ce(shift2_capture_ce),
    .capture_valid(shift2_next_valid),
    .capture_value(shift2_capture_value),
    .alu_source(alu_source),
    .instr_start(instr_start),
    .instr_is_shxd_next(next_instr.shift_is_double),
    .carry_in(eflags_fwd[0]),
    .shift_right(instr.shift_right),
    .shift_operation(instr.shift_operation),
    .op_size(op_size),
    .alu_dst(alu_dst),
    .alu_src(alu_src),
    .gpr_src_op_size(read_gpr_load_forwarded(src_reg_sel_r, op_size)),
    .gpr_dst_shift_size(read_gpr_load_forwarded(dst_reg_sel_r, shift_data_size)),
    .gpr_src_shift_size(read_gpr_load_forwarded(src_reg_sel_r, shift_data_size)),
    .immediate(instr.immediate),
    .ecx(read_gpr_load_forwarded(3'd1, 2'd2)),
    .sigma(sigma),
    .tmpb(tmpb),
    .tmpc(tmpc),
    .tmpd(tmpd),
    .tmpe(tmpe),
    .opr_r(opr_r),
    .countr(countr),
    .data_size(shift_data_size),
    .result(shift_result),
    .setup_result(shift_setup_result),
    .bit_test_cf(shift_bit_test_cf),
    .count_nonzero(shift_count_nonzero),
    .flags_commit(sh_flags_commit),
    .flags_we_zsp(sh_flags_we_zsp),
    .flags_we_of(sh_flags_we_of),
    .flags_cf(sh_flags_cf),
    .flags_of(sh_flags_of),
    .flags_zf(sh_flags_zf),
    .flags_sf(sh_flags_sf),
    .flags_pf(sh_flags_pf)
);

mul_div mul_div_inst (
    .clk(clk),
    .reset_n(reset_n),
    .exec(exec),
    .instr_start(instr_start),
    .repeat_active(repeat_active),
    .aluop(aluop),
    .dest(dest),
    .op_size(op_size),
    .is_signed_mul(is_signed_mul),
    .sigma(sigma),
    .tmpb(tmpb),
    .tmpd(tmpd),
    .dest_value(dest_value),
    .result(muldiv_result),
    .sigma_write(muldiv_sigma_write),
    .sigma_value(muldiv_sigma_value),
    .tmpb_write(muldiv_tmpb_write),
    .tmpb_value(muldiv_tmpb_value),
    .counter_early_exit(muldiv_counter_early_exit),
    .div_overflow(div_overflow),
    .div_quotient_zero(muldiv_quotient_zero),
    .div_quotient_sign(muldiv_quotient_sign),
    .div_quotient_parity(muldiv_quotient_parity),
    .mul_flag_overflow(muldiv_flag_overflow)
);

endmodule
