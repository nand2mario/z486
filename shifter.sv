//
// Barrel Shifter
// Microcode-driven shifts and rotates, from SHIFT1 setup to SHIFT2 flags
//
module shifter
    import z486_pkg::*;
(
    input  logic        clk,
    input  logic        reset_n,
    input  logic        exec,               // Execute current shift micro-op
    input  logic [6:0]  aluop,
    input  logic [6:0]  shift_aluop,        // ROM-early ALU/jump field
    input  logic [5:0]  source_field,
    input  logic [3:0]  source_class,       // Predecoded SHIFT1 operand source
    input  logic [1:0]  shift2_source,      // Predecoded SHIFT2 operand source
    input  logic        is_shift2,          // ROM-predecoded SHIFT2 control
    input  logic        capture_ce,         // q_mem -> q advance
    input  logic        capture_valid,      // Upcoming q word prereads its operand
    input  logic [31:0] capture_value,      // Forwarded upcoming shift operand
    input  logic [5:0]  alu_source,

    input  logic        instr_start,        // Capture per-instruction shift state
    input  logic        instr_is_shxd_next, // D2 lookahead for SHLD/SHRD
    input  logic        carry_in,           // Carry captured at instruction start
    input  logic        shift_right,
    input  logic [2:0]  shift_operation,
    input  logic [1:0]  op_size,

    input  logic [31:0] alu_dst,
    input  logic [31:0] alu_src,
    input  logic [31:0] gpr_src_op_size,
    input  logic [31:0] gpr_dst_shift_size,
    input  logic [31:0] gpr_src_shift_size,
    input  logic [31:0] immediate,
    input  logic [31:0] ecx,
    input  logic [31:0] sigma,
    input  logic [31:0] tmpb,
    input  logic [31:0] tmpc,
    input  logic [31:0] tmpd,
    input  logic [31:0] tmpe,
    input  logic [31:0] opr_r,
    input  logic [31:0] countr,

    output logic [1:0]  data_size,
    output logic [31:0] result,
    output logic [31:0] setup_result,       // SHIFT1 result consumed by data unit
    output logic        bit_test_cf,        // Direct BITTST operand tap (bypasses barrel)
    output logic        count_nonzero,      // Captured count enables SHIFT2 commit
    output logic        flags_commit,       // SHIFT2 deferred flags are ready
    output logic        flags_we_zsp,
    output logic        flags_we_of,
    output logic        flags_cf,
    output logic        flags_of,
    output logic        flags_zf,
    output logic        flags_sf,
    output logic        flags_pf
);

logic        instr_is_shxd;
logic        instr_cf;
logic        swap;
logic [5:0]  count;       // Needs to represent 32 for LDBSLU with count zero.
logic [4:0]  count_raw_r;
logic        overflow;
logic        eq_width;
logic        eq_width_cf;
logic        set_zsp;
logic [1:0]  shift1_size;
logic [2:0]  operation;
logic [31:0] source_value;
logic [31:0] alu_value;
logic [31:0] shift2_operand_r;

wire use_captured_source = is_shift2 ||
    ((aluop == ALUJMP_SHIFT) && (source_class == 4'd3));
wire [5:0] width = op_size == 2'd0 ? 6'd8 :
                   op_size == 2'd1 ? 6'd16 : 6'd32;
wire [31:0] width_mask = op_size == 2'd0 ? 32'h0000_00ff :
                         op_size == 2'd1 ? 32'h0000_ffff : 32'hffff_ffff;

assign data_size = use_captured_source ? shift1_size : op_size;
assign count_nonzero = count_raw_r != 5'd0;

always_ff @(posedge clk) begin
    if (!reset_n) begin
        instr_is_shxd <= 1'b0;
        instr_cf      <= 1'b0;
    end else if (instr_start) begin
        instr_is_shxd <= instr_is_shxd_next;
        instr_cf      <= carry_in;
    end
end

always_ff @(posedge clk) begin
    if (!reset_n)
        shift2_operand_r <= 32'd0;
    else if (capture_ce && capture_valid)
        shift2_operand_r <= capture_value;
end

always_comb begin
    if (use_captured_source) begin
        source_value = shift2_operand_r;
    end else begin
        case (source_class)
            4'd1:    source_value = sigma;
            // The only DSTREG source sites are BITTST, whose operand width is
            // the same width already selected for the shifter ALU operand.
            4'd2:    source_value = gpr_dst_shift_size;
            4'd4:    source_value = immediate;
            4'd5:    source_value = tmpb;
            4'd6:    source_value = tmpc;
            4'd7:    source_value = tmpd;
            4'd8:    source_value = tmpe;
            4'd9:    source_value = opr_r;
            4'd10:   source_value = countr;
            4'd11:   source_value = 32'hffff_ffff;
            default: source_value = 32'd0;
        endcase
    end
end

// synthesis translate_off
// Prove that moving the only plain SRCREG SHIFT operands and widths to the
// existing preread registers preserves the former live values.
always @(posedge clk)
    if (reset_n && exec && (aluop == ALUJMP_SHIFT) &&
        (source_class == 4'd3)) begin
        if (shift2_operand_r !== gpr_src_op_size)
            $fatal(1, "SRCREG SHIFT PREREAD MISMATCH: captured=%08x live=%08x",
                   shift2_operand_r, gpr_src_op_size);
        if (shift1_size !== op_size)
            $fatal(1, "SRCREG SHIFT SIZE MISMATCH: captured=%x live=%x",
                   shift1_size, op_size);
    end
always @(posedge clk)
    if (reset_n && exec && (aluop == ALUJMP_BITTST) &&
        (source_class == 4'd2) && (data_size != op_size))
        $fatal(1, "BITTST DSTREG source/ALU width mismatch");
// synthesis translate_on

always_comb begin
    case (alu_source)
        ALUSRC_CONST_0:      alu_value = 32'd0;
        ALUSRC_TMPC:         alu_value = tmpc;
        ALUSRC_TMPD:         alu_value = tmpd;
        ALUSRC_TMPB:         alu_value = tmpb;
        ALUSRC_DSTREG:       alu_value = gpr_dst_shift_size;
        ALUSRC_SRCREG:       alu_value = gpr_src_shift_size;
        ALUSRC_ECX:          alu_value = ecx;
        ALUSRC_IMM:          alu_value = immediate;
        ALUSRC_BITS_V:       alu_value = data_size == 2'd0 ? 32'd7 :
                                         data_size == 2'd2 ? 32'd31 : 32'd15;
        ALUSRC_CONST_1:      alu_value = 32'd1;
        ALUSRC_CONST_3:      alu_value = 32'd3;
        ALUSRC_CONST_7:      alu_value = 32'd7;
        ALUSRC_CONST_1FF:    alu_value = 32'h0000_01ff;
        ALUSRC_CONST_4000:   alu_value = 32'h0000_4000;
        ALUSRC_CONST_F0000:  alu_value = 32'h000f_0000;
        ALUSRC_MASK16:       alu_value = 32'h0000_ffff;
        ALUSRC_CONST_FFFF0000: alu_value = 32'hffff_0000;
        default:             alu_value = 32'd0;
    endcase
end

wire [31:0] high_word = swap ? alu_value : source_value;
wire [31:0] low_word  = swap ? source_value : alu_value;
wire [63:0] shift_input = data_size == 2'd0 ? {high_word, low_word[7:0]} :
                          data_size == 2'd1 ? {high_word, low_word[15:0]} :
                                                {high_word, low_word};
wire [63:0] shifted = shift_input >> count;
wire        is_sar = (shift_operation == SAR) && !instr_is_shxd;
wire        low_sign = data_size == 2'd0 ? low_word[7] :
                       data_size == 2'd1 ? low_word[15] : low_word[31];
wire [31:0] sar_overflow_result = low_sign ? 32'hffff_ffff : 32'd0;
wire        last_out_lsb = shift_input[count-1];
// SHIFT2 width has only three architectural values. Spell out these taps so
// flag retirement does not infer full variable-index muxes after the barrel.
wire        last_out_msb = data_size == 2'd0 ? shifted[8] :
                           data_size == 2'd1 ? shifted[16] : shifted[32];
wire        shifted_sign = data_size == 2'd0 ? shifted[7] :
                           data_size == 2'd1 ? shifted[15] : shifted[31];
wire        shifted_next_sign = data_size == 2'd0 ? shifted[6] :
                                data_size == 2'd1 ? shifted[14] : shifted[30];
// Overflow is architecturally defined only for a count of one.  Right
// operations use a barrel count of one, while left operations are represented
// as a right shift by width-1.  Select fixed taps for those two cases instead
// of routing the 64-bit barrel result back into the OF register.
wire count1_barrel_right = count == 6'd1;
wire count1_shifted_sign_right = data_size == 2'd0 ? shift_input[8] :
                                     data_size == 2'd1 ? shift_input[16] :
                                                         shift_input[32];
wire count1_shifted_sign_left = data_size == 2'd0 ? shift_input[14] :
                                    data_size == 2'd1 ? shift_input[30] :
                                                        shift_input[62];
wire count1_shifted_next_sign_right = data_size == 2'd0 ? shift_input[7] :
                                          data_size == 2'd1 ? shift_input[15] :
                                                              shift_input[31];
wire count1_shifted_next_sign_left = data_size == 2'd0 ? shift_input[13] :
                                         data_size == 2'd1 ? shift_input[29] :
                                                             shift_input[61];
wire count1_last_out_msb_right = data_size == 2'd0 ? shift_input[9] :
                                    data_size == 2'd1 ? shift_input[17] :
                                                        shift_input[33];
wire count1_last_out_msb_left = data_size == 2'd0 ? shift_input[15] :
                                   data_size == 2'd1 ? shift_input[31] :
                                                       shift_input[63];
wire count1_shifted_sign = count1_barrel_right ? count1_shifted_sign_right :
                                                count1_shifted_sign_left;
wire count1_shifted_next_sign = count1_barrel_right ?
    count1_shifted_next_sign_right : count1_shifted_next_sign_left;
wire count1_last_out_msb = count1_barrel_right ? count1_last_out_msb_right :
                                                count1_last_out_msb_left;

// synthesis translate_off
always @(posedge clk)
    if (reset_n && exec && is_shift2 && (count_raw_r == 5'd1) &&
        ((count1_shifted_sign !== shifted_sign) ||
         (count1_shifted_next_sign !== shifted_next_sign) ||
         (count1_last_out_msb !== last_out_msb)))
        $fatal(1, "count-one fixed shift taps disagree with barrel result");
// synthesis translate_on

assign result = overflow ? (is_sar ? sar_overflow_result : 32'd0) : shifted[31:0];
// Every BITTST site uses a right-count setup with swap clear.  Its carry is
// therefore the selected low-word bit; do not route the full 64-bit barrel
// result back into the architectural flag write path.
assign bit_test_cf = alu_value[count[4:0]];
// Preserve the barrel result for the one-cycle deferred flag retirement.
// The next instruction's stack setup can claim SIGMA on the SHIFT2 edge,
// so shared SIGMA is not a reliable source for the preceding shift's flags.
// Dedicated result/size registers keep the barrel out of the flag-write path.
logic [31:0] flags_result_r;
logic [1:0] flags_size_r;
assign flags_pf = ~^flags_result_r[7:0];
assign flags_zf = flags_size_r == 2'd0 ? flags_result_r[7:0] == 8'd0 :
                  flags_size_r == 2'd1 ? flags_result_r[15:0] == 16'd0 :
                                             flags_result_r == 32'd0;
assign flags_sf = flags_size_r == 2'd0 ? flags_result_r[7] :
                  flags_size_r == 2'd1 ? flags_result_r[15] : flags_result_r[31];

always_comb begin
    setup_result = alu_dst;
    if (!instr_is_shxd) begin
        case (shift_operation)
            RCL:     setup_result = (instr_cf << (width-1)) |
                                    ((alu_dst & width_mask) >> 1);
            RCR:     setup_result = {alu_dst, instr_cf};
            SHL,
            SHR,
            SAL:     setup_result = 32'd0;
            SAR:     setup_result = op_size == 2'd0 ? {32{alu_dst[7]}} :
                                    op_size == 2'd1 ? {32{alu_dst[15]}} :
                                                        {32{alu_dst[31]}};
            default: setup_result = alu_dst;
        endcase
    end
end

always_ff @(posedge clk) begin
    if (exec) begin
        case (shift_aluop)
            ALUJMP_SHIFT1: begin
                automatic logic [5:0] reduced_count;
                automatic logic [5:0] raw_count;
                raw_count = alu_src[4:0];
                case (op_size)
                    2'd0:    reduced_count = {3'd0, raw_count[2:0]};
                    2'd1:    reduced_count = {2'd0, raw_count[3:0]};
                    default: reduced_count = raw_count;
                endcase
                count_raw_r <= raw_count;
                shift1_size <= op_size;

                if (instr_is_shxd) begin
                    swap <= !shift_right;
                    count <= shift_right ? raw_count : width - raw_count;
                    operation <= shift_right ? ROR : ROL;
                    set_zsp <= 1'b1;
                end else begin
                    swap <= !shift_operation[0];
                    overflow <= raw_count >= width &&
                                (shift_operation == SHL || shift_operation == SAL ||
                                 shift_operation == SHR || shift_operation == SAR);
                    eq_width <= raw_count == width &&
                                (shift_operation == SHL || shift_operation == SAL ||
                                 shift_operation == SHR || shift_operation == SAR);
                    eq_width_cf <= shift_operation[0] ? alu_dst[width-1] : alu_dst[0];
                    case (shift_operation)
                        ROL:     count <= width - reduced_count;
                        ROR:     count <= reduced_count[4:0];
                        RCL:     count <= width - raw_count[4:0];
                        RCR:     count <= raw_count[4:0];
                        SHL,
                        SAL:     count <= raw_count >= width ? 5'd31 : width - raw_count;
                        SHR:     count <= raw_count >= width ? 5'd31 : raw_count;
                        default: count <= raw_count >= width ? 5'd31 : raw_count;
                    endcase
                    operation <= shift_operation;
                    set_zsp <= shift_operation == SHL || shift_operation == SHR ||
                               shift_operation == SAR || shift_operation == SAL;
                end
            end

            ALUJMP_LDBSRM: begin
                swap <= 1'b0;
                count <= alu_src[4:0] & (width - 1'b1);
            end
            ALUJMP_LDBSRU: begin
                swap <= 1'b0;
                count <= alu_src[4:0];
                shift1_size <= op_size;
                overflow <= 1'b0;
            end
            ALUJMP_LDBSLM: begin
                swap <= 1'b1;
                count <= width - (alu_src[4:0] & (width - 1'b1));
                overflow <= 1'b0;
            end
            ALUJMP_LDBSLU: begin
                swap <= 1'b1;
                count <= width - alu_src[4:0];
                shift1_size <= op_size;
                count_raw_r <= alu_src[4:0];
                set_zsp <= 1'b0;
                operation <= SHL;
                overflow <= 1'b0;
            end
            default: ;
        endcase
    end
end

// SHIFT2 flags retire one cycle after the barrel operation. This keeps the
// barrel output out of the architectural flag-write path.
always_ff @(posedge clk) begin
    if (!reset_n) begin
        flags_commit <= 1'b0;
        flags_result_r <= 32'd0;
        flags_size_r <= 2'd0;
    end else begin
        flags_commit <= exec && is_shift2 && count_nonzero;
        if (exec && is_shift2 && count_nonzero) begin
            flags_result_r <= result;
            flags_size_r <= shift1_size;
            flags_we_zsp <= set_zsp;
            flags_we_of <= 1'b0;
            if (instr_is_shxd) begin
                flags_cf <= shift_right ? last_out_lsb : last_out_msb;
                if (count_raw_r == 5'd1) begin
                    flags_we_of <= 1'b1;
                    flags_of <= shift_right ?
                        (count1_shifted_sign ^ count1_shifted_next_sign) :
                        (count1_shifted_sign ^ count1_last_out_msb);
                end
            end else begin
                case (operation)
                    SHL,
                    SAL: flags_cf <= overflow ? (eq_width ? eq_width_cf : 1'b0) : last_out_msb;
                    RCL: flags_cf <= last_out_msb;
                    SHR: flags_cf <= overflow ? (eq_width ? eq_width_cf : 1'b0) : last_out_lsb;
                    SAR: flags_cf <= overflow ? low_sign : last_out_lsb;
                    RCR: flags_cf <= last_out_lsb;
                    ROL: flags_cf <= shifted[0];
                    ROR: flags_cf <= shifted_sign;
                    default: flags_cf <= 1'b0;
                endcase
                if (count_raw_r == 5'd1) begin
                    case (operation)
                        SHL: begin
                            flags_we_of <= 1'b1;
                            flags_of <= count1_shifted_sign ^
                                        count1_last_out_msb;
                        end
                        SHR: begin
                            flags_we_of <= 1'b1;
                            flags_of <= low_sign;
                        end
                        SAR: begin
                            flags_we_of <= 1'b1;
                            flags_of <= 1'b0;
                        end
                        ROR,
                        RCR: begin
                            flags_we_of <= 1'b1;
                            flags_of <= count1_shifted_sign ^
                                        count1_shifted_next_sign;
                        end
                        default: ;
                    endcase
                end
                if (operation == ROL) begin
                    flags_we_of <= 1'b1;
                    flags_of <= shifted_sign ^ shifted[0];
                end
                if (operation == RCL) begin
                    flags_we_of <= 1'b1;
                    flags_of <= shifted_sign ^ last_out_msb;
                end
            end
        end
    end
end

// synthesis translate_off
logic [31:0] shift2_live_reference;
always_comb begin
    case (shift2_source)
        2'd0:    shift2_live_reference = tmpc;
        2'd1:    shift2_live_reference = tmpe;
        2'd2:    shift2_live_reference = sigma;
        2'd3:    shift2_live_reference = gpr_src_shift_size;
        default: shift2_live_reference = 32'd0;
    endcase
end
always @(posedge clk)
    if (reset_n && exec && is_shift2 &&
        (shift2_operand_r !== shift2_live_reference))
        $fatal(1, "SHIFT2 OPERAND CAPTURE MISMATCH: captured=%08x live=%08x source=%0d",
               shift2_operand_r, shift2_live_reference, shift2_source);
always @(posedge clk)
    if (reset_n && exec && (shift_aluop == ALUJMP_SHIFT2) &&
        source_field != SRC_TMPC && source_field != SRC_TMPE &&
        source_field != SRC_SIGMA && source_field != SRC_SRCREG)
        $fatal(1, "SHIFT2 source outside ROM predecode inventory: %02x", source_field);
always @(posedge clk)
    if (reset_n && exec && (aluop == ALUJMP_BITTST) &&
        (bit_test_cf !== result[0]))
        $fatal(1, "BITTST DIRECT CARRY MISMATCH: direct=%b barrel=%b",
               bit_test_cf, result[0]);
// synthesis translate_on

endmodule
