// Microsequenced conversion and arithmetic datapath. The v1 numeric type is
// kept at this migration boundary so both implementations compare exactly.
module x87_executor
    import x87_pkg::*, x87_ucode_pkg::*;
(
    input  logic                 clk,
    input  logic                 reset,
    input  logic                 start,
    input  x87_exec_op_t      exec_op,
    input  logic           [1:0] integer_size,
    input  logic           [1:0] precision_control,
    input  logic           [1:0] rounding_mode,
    input  logic                 quiet_compare,
    input  logic                 trans_cosine,
    input  logic                 trans_tangent_pair,
    input  logic                 trans_atan2,
    input  x87_reg_t             operand,
    input  x87_reg_t             operand_b,
    input  logic          [63:0] transfer_in,
    output logic                 busy,
    output logic                 done,
    output logic           [2:0] commit_action,
    output x87_reg_t             result,
    output x87_reg_t             auxiliary_result,
    output logic          [63:0] transfer_out,
    output logic                 invalid,
    output logic                 inexact,
    output logic                 divide_by_zero,
    output logic                 overflow,
    output logic                 underflow,
    output logic                 denormal_operand,
    output logic                 range_incomplete,
    output logic                 rounded_up,
    output logic                 compare_unordered,
    output logic                 compare_less,
    output logic                 compare_equal
);

logic [7:0] entry;
logic [31:0] conditions;
logic seq_active;
logic seq_exec_valid;
logic seq_done;
logic [7:0] uaddr;
x87_uop_t uop;

x87_reg_t result_r;
logic [63:0] transfer_r;
logic [67:0] work_r;
logic [55:0] add_small_r;
logic [5:0] count_r;
logic [5:0] restore_count_r;
logic guard_r;
logic round_r;
logic sticky_r;
logic input_sign_r;
logic special_r;
logic integral_r;
logic subunit_r;
logic zero_r;
logic shift_left_r;
logic invalid_r;
logic direct_ready_r;
logic shift_right_r;
logic signed [16:0] unbiased_r;
logic signed [16:0] format_exp_r;
logic [14:0] add_exp_r;
logic add_result_sign_r;
logic add_small_sign_r;
logic format_m32;
logic arithmetic_op;
logic effective_b_sign;
logic round_sign;
logic addsub_normalize_more;
logic round_discarded;
logic round_increment;
logic [67:0] rounded_work;
logic [67:0] arithmetic_lhs;
logic [67:0] arithmetic_rhs;
logic arithmetic_subtract;
logic [67:0] arithmetic_result;
logic [1:0] operand_magnitude;
logic [105:0] mul_accum_r;
logic [105:0] mul_addend;
logic [105:0] mul_accum_next;
logic [53:0] mul_product_r;
logic [26:0] mul_a_r;
logic [26:0] mul_b_r;
logic [1:0] mul_partial_r;
logic [14:0] mul_exp_r;
logic mul_result_sign_r;
logic [52:0] divsqrt_divisor_r;
logic [57:0] divsqrt_remainder_r;
logic [55:0] divsqrt_result_bits_r;
logic [111:0] divsqrt_radicand_r;
logic signed [16:0] divsqrt_exp_r;
logic divsqrt_result_sign_r;
logic [57:0] divsqrt_shifted;
logic [57:0] divsqrt_trial;
logic [58:0] divsqrt_subtract_ext;
logic [57:0] divsqrt_after_subtract;
logic divsqrt_next_bit;
logic divsqrt_remainder_nonzero;
logic [52:0] trans_range_sig_r;
logic [7:0] trans_count_r;
logic [120:0] trans_range_remainder_r;
logic [1:0] trans_quadrant_r;
logic signed [82:0] trans_x_r;
logic signed [82:0] trans_y_r;
logic signed [82:0] trans_z_r;
logic signed [82:0] trans_x_step_r;
logic signed [82:0] trans_y_step_r;
logic signed [82:0] trans_angle_step_r;
logic trans_cordic_sub_r;
logic [6:0] trans_shift_count_r;
logic [6:0] trans_atan_address_r;
logic signed [82:0] trans_atan_value;
logic trans_shift_x_r;
logic trans_x_sign_r;
logic trans_y_sign_r;
logic trans_result_sign_r;
logic [82:0] trans_magnitude_r;
logic signed [16:0] trans_exp_r;
logic trans_aux_sign_r;
logic [82:0] trans_aux_magnitude_r;
logic trans_rounding_aux_r;
x87_reg_t trans_auxiliary_result_r;
logic trans_range_bit;
logic trans_range_subtract;
logic [121:0] trans_range_shifted;
logic [122:0] trans_range_subtract_ext;
logic [120:0] trans_range_next;
logic [1:0] trans_quadrant_next;
logic signed [121:0] trans_reduced_q120;
logic signed [82:0] trans_reduced_q80;
logic trans_cordic_sub;
logic signed [82:0] cordic_lhs;
logic signed [82:0] cordic_rhs;
logic cordic_subtract;
logic signed [82:0] cordic_result;
logic trans_operation;
logic trans_normalize_more;
logic trans_needs_aux;

localparam logic [120:0] TRANS_PIO2_Q120 =
    121'h1921fb54442d18469898cc51701b83a;
localparam logic [120:0] TRANS_PIO4_Q120 =
    121'h0c90fdaa22168c234c4c6628b80dc1d;
localparam logic signed [82:0] TRANS_CORDIC_K_Q80 =
    83'h009b74eda8435e5a67f5f9;
localparam logic signed [82:0] TRANS_PI_Q80 =
    83'sh03243f6a8885a308d3131a;
localparam logic signed [82:0] TRANS_PIO2_Q80 =
    83'sh01921fb54442d18469898d;
localparam logic signed [82:0] TRANS_PIO4_Q80 =
    83'sh00c90fdaa22168c234c4c6;

assign busy = seq_active;
assign done = seq_done;
assign result = result_r;
assign auxiliary_result = trans_auxiliary_result_r;
assign transfer_out = transfer_r;
assign trans_operation = exec_op == X87_ARITH_TRANS;
assign arithmetic_op = (exec_op == X87_ARITH_ADD) ||
                       (exec_op == X87_ARITH_SUB);
assign effective_b_sign = operand_b.sign ^
                          (exec_op == X87_ARITH_SUB);
assign round_sign = trans_operation ? trans_result_sign_r
                  : ((exec_op == X87_ARITH_DIV) ||
                     (exec_op == X87_ARITH_SQRT))
                  ? divsqrt_result_sign_r
                  : (exec_op == X87_ARITH_MUL) ? mul_result_sign_r
                  : arithmetic_op ? add_result_sign_r : operand.sign;
assign addsub_normalize_more = (work_r[56:0] != 57'h0) &&
                               (work_r[56] || !work_r[55]);
assign format_m32 = (exec_op == X87_CONVERT_FLD_M32) ||
                    (exec_op == X87_CONVERT_FST_M32);

assign round_discarded = guard_r || sticky_r;
always_comb begin
    case (rounding_mode)
        2'b00: round_increment = guard_r && (sticky_r || work_r[0]);
        2'b01: round_increment = round_sign && round_discarded;
        2'b10: round_increment = !round_sign && round_discarded;
        default: round_increment = 1'b0;
    endcase
end
always_comb begin
    arithmetic_lhs = work_r;
    arithmetic_rhs = {{67{1'b0}}, round_increment};
    arithmetic_subtract = 1'b0;
    if (uop.alu_route == X87_ALU_CALCULATE_ADDSUB) begin
        arithmetic_lhs = {11'h0, 1'b0, work_r[55:0]};
        arithmetic_rhs = {11'h0, 1'b0, add_small_r};
        arithmetic_subtract = add_result_sign_r != add_small_sign_r;
    end
end
assign arithmetic_result = arithmetic_subtract
                         ? arithmetic_lhs - arithmetic_rhs
                         : arithmetic_lhs + arithmetic_rhs;
assign rounded_work = arithmetic_result;

always_comb begin
    case (mul_partial_r)
        2'd0: mul_addend = {{52{1'b0}}, mul_product_r};
        2'd1,
        2'd2: mul_addend = {{25{1'b0}}, mul_product_r, {27{1'b0}}};
        default: mul_addend = {mul_product_r[51:0], {54{1'b0}}};
    endcase
end
assign mul_accum_next = mul_accum_r + mul_addend;

always_comb begin
    if (uop.engine == X87_ENGINE_DIV_ITERATE) begin
        divsqrt_shifted = {4'h0, (divsqrt_remainder_r[53:0] << 1)};
        divsqrt_trial = {4'h0, 1'b0, divsqrt_divisor_r};
    end else begin
        divsqrt_shifted = (divsqrt_remainder_r << 2) |
                          {{56{1'b0}}, divsqrt_radicand_r[111:110]};
        divsqrt_trial = ({2'b0, divsqrt_result_bits_r} << 2) | 58'd1;
    end
end
assign divsqrt_subtract_ext = {1'b0, divsqrt_shifted} -
                              {1'b0, divsqrt_trial};
assign divsqrt_next_bit = !divsqrt_subtract_ext[58];
assign divsqrt_after_subtract = divsqrt_subtract_ext[57:0];
assign divsqrt_remainder_nonzero =
    (exec_op == X87_ARITH_SQRT)
        ? (divsqrt_remainder_r != 58'h0)
        : (divsqrt_remainder_r[53:0] != 54'h0);

assign trans_range_bit = trans_range_sig_r[52];
assign trans_range_shifted = {trans_range_remainder_r, trans_range_bit};
assign trans_range_subtract_ext =
    {1'b0, trans_range_shifted} - {2'b0, TRANS_PIO2_Q120};
assign trans_range_subtract = !trans_range_subtract_ext[122];
assign trans_range_next = trans_range_subtract
                        ? trans_range_subtract_ext[120:0]
                        : trans_range_shifted[120:0];
assign trans_quadrant_next =
    {trans_quadrant_r[0], trans_range_subtract};
always_comb begin
    if (trans_range_remainder_r > TRANS_PIO4_Q120)
        trans_reduced_q120 = $signed({1'b0, trans_range_remainder_r}) -
                             $signed({1'b0, TRANS_PIO2_Q120});
    else
        trans_reduced_q120 = $signed({1'b0, trans_range_remainder_r});
end
assign trans_reduced_q80 =
    {trans_reduced_q120[121], trans_reduced_q120[121:40]};
assign trans_cordic_sub = trans_atan2 ? trans_y_r[82] : !trans_z_r[82];
always_comb begin
    cordic_lhs = 83'sh0;
    cordic_rhs = 83'sh0;
    cordic_subtract = 1'b0;
    case (uop.alu_route)
        X87_ALU_CORDIC_X: begin
            cordic_lhs = trans_x_r;
            cordic_rhs = trans_y_step_r;
            cordic_subtract = trans_cordic_sub_r;
        end
        X87_ALU_CORDIC_Y: begin
            cordic_lhs = trans_y_r;
            cordic_rhs = trans_x_step_r;
            cordic_subtract = !trans_cordic_sub_r;
        end
        X87_ALU_CORDIC_Z: begin
            cordic_lhs = trans_z_r;
            cordic_rhs = trans_angle_step_r;
            cordic_subtract = trans_cordic_sub_r;
        end
        default: ;
    endcase
end
assign cordic_result = cordic_subtract
                     ? cordic_lhs - cordic_rhs
                     : cordic_lhs + cordic_rhs;
assign trans_normalize_more = (trans_magnitude_r != 83'h0) &&
                              ((trans_magnitude_r[82:81] != 2'b00) ||
                               !trans_magnitude_r[80]);
assign trans_needs_aux = trans_tangent_pair && !trans_atan2 &&
                         !trans_rounding_aux_r;

x87_cordic_rom trans_atan_rom (
    .clk(clk),
    .address(trans_atan_address_r),
    .value(trans_atan_value)
);

always_comb begin
    case (exec_op)
        X87_CONVERT_FLD_M32: entry = X87_ENTRY_FLD_M32;
        X87_CONVERT_FLD_M64: entry = X87_ENTRY_FLD_M64;
        X87_CONVERT_FILD:    entry = X87_ENTRY_FILD;
        X87_CONVERT_FST_M32: entry = X87_ENTRY_FST_M32;
        X87_CONVERT_FST_M64: entry = X87_ENTRY_FST_M64;
        X87_CONVERT_FIST:    entry = X87_ENTRY_FIST;
        X87_ARITH_ADD,
        X87_ARITH_SUB:       entry = X87_ENTRY_ADDSUB;
        X87_ARITH_COMPARE:   entry = X87_ENTRY_COMPARE;
        X87_ARITH_MUL:       entry = X87_ENTRY_MUL;
        X87_ARITH_DIV:       entry = X87_ENTRY_DIV;
        X87_ARITH_SQRT:      entry = X87_ENTRY_SQRT;
        X87_ARITH_TRANS:     entry = X87_ENTRY_TRANS;
        default:                entry = X87_ENTRY_FRNDINT;
    endcase

    conditions = '0;
    conditions[X87_COND_TRUE] = 1'b1;
    conditions[X87_COND_SPECIAL] = special_r;
    conditions[X87_COND_INTEGRAL] = trans_operation
                                      ? trans_needs_aux : integral_r;
    conditions[X87_COND_SUBUNIT] = subunit_r;
    conditions[X87_COND_COUNT_MORE] = trans_operation
                                        ? (trans_count_r > 8'd1)
                                        : (count_r > 6'd1);
    conditions[X87_COND_TRANSFER_READY] = 1'b1;
    conditions[X87_COND_ZERO] = trans_operation
                                  ? (trans_magnitude_r == 83'h0) : zero_r;
    conditions[X87_COND_SHIFT_LEFT] = trans_operation
                                        ? trans_atan2 : shift_left_r;
    conditions[X87_COND_COUNT_ZERO] = trans_operation
                                        ? (trans_count_r == 8'd0)
                                        : (count_r == 6'd0);
    conditions[X87_COND_INVALID] = invalid_r;
    conditions[X87_COND_DIRECT_READY] = direct_ready_r;
    conditions[X87_COND_NORMALIZE_MORE] = trans_operation
                                            ? trans_normalize_more
                                            : !work_r[51];
    conditions[X87_COND_SHIFT_RIGHT] = shift_right_r;
    conditions[X87_COND_SHIFT_RIGHT_MORE] = |work_r[63:54];
    conditions[X87_COND_ADDSUB_NORMALIZE_MORE] =
        addsub_normalize_more;
    conditions[X87_COND_CORDIC_SHIFT_MORE] =
        trans_shift_count_r > 7'd4;
end

x87_sequencer sequencer (
    .clk(clk),
    .reset(reset),
    .start(start),
    .entry(entry),
    .conditions(conditions),
    .active(seq_active),
    .exec_valid(seq_exec_valid),
    .done(seq_done),
    .uaddr(uaddr),
    .uop(uop)
);

function automatic logic is_signaling_nan(input x87_reg_t value);
    return (value.class_id == X87_NAN) && !value.sig[51];
endfunction

function automatic x87_reg_t quiet_nan(input x87_reg_t value);
    x87_reg_t quiet;
    begin
        quiet = value;
        quiet.exp = 15'h7fff;
        quiet.sig[52:51] = 2'b11;
        quiet.class_id = X87_NAN;
        return quiet;
    end
endfunction

function automatic logic is_nan(input x87_reg_t value);
    return value.class_id == X87_NAN;
endfunction

function automatic logic is_infinity(input x87_reg_t value);
    return value.class_id == X87_INFINITY;
endfunction

function automatic logic is_zero(input x87_reg_t value);
    return value.class_id == X87_ZERO;
endfunction

function automatic logic is_finite(input x87_reg_t value);
    return (value.class_id == X87_NORMAL) ||
           (value.class_id == X87_DENORMAL);
endfunction

function automatic logic overflow_to_infinity(
    input logic sign,
    input logic [1:0] mode
);
    case (mode)
        2'b00: return 1'b1;
        2'b01: return sign;
        2'b10: return !sign;
        default: return 1'b0;
    endcase
endfunction

// Return 0 for equal magnitudes, 1 for |a| < |b|, and 2 for |a| > |b|.
function automatic logic [1:0] magnitude_relation(
    input x87_reg_t a,
    input x87_reg_t b
);
    logic [55:0] a_ext;
    logic [55:0] b_ext;
    begin
        a_ext = {a.sig, a.guard_bit, a.round_bit, a.sticky_bit};
        b_ext = {b.sig, b.guard_bit, b.round_bit, b.sticky_bit};
        if (is_infinity(a) && !is_infinity(b))
            return 2'd2;
        if (!is_infinity(a) && is_infinity(b))
            return 2'd1;
        if (is_zero(a) && is_zero(b))
            return 2'd0;
        if (a.exp < b.exp)
            return 2'd1;
        if (a.exp > b.exp)
            return 2'd2;
        if (a_ext < b_ext)
            return 2'd1;
        if (a_ext > b_ext)
            return 2'd2;
        return 2'd0;
    end
endfunction

// {unordered, less, equal}; greater is represented by zero.
function automatic logic [2:0] compare_relation(
    input x87_reg_t a,
    input x87_reg_t b,
    input logic [1:0] magnitude
);
    begin
        if (is_nan(a) || is_nan(b) ||
            (a.class_id == X87_EMPTY) || (b.class_id == X87_EMPTY))
            return 3'b100;
        if (is_zero(a) && is_zero(b))
            return 3'b001;
        if (a.sign != b.sign)
            return a.sign ? 3'b010 : 3'b000;
        if (magnitude == 2'd0)
            return 3'b001;
        if (!a.sign)
            return (magnitude == 2'd1) ? 3'b010 : 3'b000;
        return (magnitude == 2'd2) ? 3'b010 : 3'b000;
    end
endfunction

assign operand_magnitude = magnitude_relation(operand, operand_b);

function automatic logic directed_increment(
    input logic sign,
    input logic [1:0] mode,
    input logic discarded
);
    case (mode)
        2'b01: return sign && discarded;
        2'b10: return !sign && discarded;
        default: return 1'b0;
    endcase
endfunction

function automatic logic signed [63:0] sized_integer(
    input logic [63:0] raw,
    input logic [1:0] size
);
    case (size)
        2'd0: return {{48{raw[15]}}, raw[15:0]};
        2'd1: return {{32{raw[31]}}, raw[31:0]};
        default: return raw;
    endcase
endfunction

function automatic logic [63:0] integer_indefinite(input logic [1:0] size);
    case (size)
        2'd0: return 64'h0000_0000_0000_8000;
        2'd1: return 64'h0000_0000_8000_0000;
        default: return 64'h8000_0000_0000_0000;
    endcase
endfunction

always_ff @(posedge clk) begin
    if (reset) begin
        result_r <= x87_empty();
        transfer_r <= 64'h0;
        work_r <= 68'h0;
        add_small_r <= 56'h0;
        count_r <= 6'd0;
        restore_count_r <= 6'd0;
        guard_r <= 1'b0;
        round_r <= 1'b0;
        sticky_r <= 1'b0;
        input_sign_r <= 1'b0;
        special_r <= 1'b0;
        integral_r <= 1'b0;
        subunit_r <= 1'b0;
        zero_r <= 1'b0;
        shift_left_r <= 1'b0;
        invalid_r <= 1'b0;
        direct_ready_r <= 1'b0;
        shift_right_r <= 1'b0;
        unbiased_r <= 17'sd0;
        format_exp_r <= 17'sd0;
        add_exp_r <= 15'h0;
        add_result_sign_r <= 1'b0;
        add_small_sign_r <= 1'b0;
        mul_accum_r <= 106'h0;
        mul_product_r <= 54'h0;
        mul_a_r <= 27'h0;
        mul_b_r <= 27'h0;
        mul_partial_r <= 2'd0;
        mul_exp_r <= 15'h0;
        mul_result_sign_r <= 1'b0;
        divsqrt_divisor_r <= 53'h0;
        divsqrt_remainder_r <= 58'h0;
        divsqrt_result_bits_r <= 56'h0;
        divsqrt_radicand_r <= 112'h0;
        divsqrt_exp_r <= 17'sd0;
        divsqrt_result_sign_r <= 1'b0;
        trans_range_sig_r <= 53'h0;
        trans_count_r <= 8'h0;
        trans_range_remainder_r <= 121'h0;
        trans_quadrant_r <= 2'b00;
        trans_x_r <= 83'sh0;
        trans_y_r <= 83'sh0;
        trans_z_r <= 83'sh0;
        trans_x_step_r <= 83'sh0;
        trans_y_step_r <= 83'sh0;
        trans_angle_step_r <= 83'sh0;
        trans_cordic_sub_r <= 1'b0;
        trans_shift_count_r <= 7'h0;
        trans_atan_address_r <= 7'h0;
        trans_shift_x_r <= 1'b0;
        trans_x_sign_r <= 1'b0;
        trans_y_sign_r <= 1'b0;
        trans_result_sign_r <= 1'b0;
        trans_magnitude_r <= 83'h0;
        trans_exp_r <= 17'sd0;
        trans_aux_sign_r <= 1'b0;
        trans_aux_magnitude_r <= 83'h0;
        trans_rounding_aux_r <= 1'b0;
        trans_auxiliary_result_r <= x87_empty();
        invalid <= 1'b0;
        inexact <= 1'b0;
        divide_by_zero <= 1'b0;
        overflow <= 1'b0;
        underflow <= 1'b0;
        denormal_operand <= 1'b0;
        range_incomplete <= 1'b0;
        rounded_up <= 1'b0;
        compare_unordered <= 1'b0;
        compare_less <= 1'b0;
        compare_equal <= 1'b0;
        commit_action <= X87_COMMIT_NONE;
    end else begin
        if (start) begin
            invalid <= 1'b0;
            inexact <= 1'b0;
            divide_by_zero <= 1'b0;
            overflow <= 1'b0;
            underflow <= 1'b0;
            denormal_operand <=
                ((exec_op == X87_ARITH_DIV) ||
                 (exec_op == X87_ARITH_SQRT) ||
                 (exec_op == X87_ARITH_TRANS)) &&
                ((operand.class_id == X87_DENORMAL) ||
                 (((exec_op == X87_ARITH_DIV) ||
                   ((exec_op == X87_ARITH_TRANS) && trans_atan2)) &&
                  (operand_b.class_id == X87_DENORMAL)));
            range_incomplete <= 1'b0;
            rounded_up <= 1'b0;
            round_r <= 1'b0;
            special_r <= 1'b0;
            integral_r <= 1'b0;
            subunit_r <= 1'b0;
            zero_r <= 1'b0;
            shift_left_r <= 1'b0;
            invalid_r <= 1'b0;
            direct_ready_r <= 1'b0;
            shift_right_r <= 1'b0;
            compare_unordered <= 1'b0;
            compare_less <= 1'b0;
            compare_equal <= 1'b0;
            trans_rounding_aux_r <= 1'b0;
            trans_auxiliary_result_r <= x87_empty();
            commit_action <= X87_COMMIT_NONE;
        end

        if (seq_exec_valid) begin
            if (uop.flow == X87_FLOW_FINISH)
                commit_action <= uop.commit;

            // Horizontal resource controls update independently of the
            // remaining legacy operation lane. The generator guarantees that
            // no operation selected in the same word also writes this state.
            case (uop.shift_route)
                X87_SHIFT_LEFT:  work_r <= work_r << 1;
                X87_SHIFT_RIGHT: work_r <= work_r >> 1;
                X87_SHIFT_CORDIC_LOAD: begin
                    trans_x_step_r <= trans_x_r;
                    trans_y_step_r <= trans_y_r;
                    trans_angle_step_r <= trans_atan_value;
                    trans_cordic_sub_r <= trans_cordic_sub;
                    trans_shift_count_r <=
                        7'd80 - trans_count_r[6:0];
                end
                X87_SHIFT_CORDIC_STEP: begin
                    if (trans_shift_count_r >= 7'd4) begin
                        trans_x_step_r <= trans_x_step_r >>> 4;
                        trans_y_step_r <= trans_y_step_r >>> 4;
                        trans_shift_count_r <= trans_shift_count_r - 7'd4;
                    end else begin
                        case (trans_shift_count_r[1:0])
                            2'd1: begin
                                trans_x_step_r <= trans_x_step_r >>> 1;
                                trans_y_step_r <= trans_y_step_r >>> 1;
                            end
                            2'd2: begin
                                trans_x_step_r <= trans_x_step_r >>> 2;
                                trans_y_step_r <= trans_y_step_r >>> 2;
                            end
                            2'd3: begin
                                trans_x_step_r <= trans_x_step_r >>> 3;
                                trans_y_step_r <= trans_y_step_r >>> 3;
                            end
                            default: ;
                        endcase
                        trans_shift_count_r <= 7'd0;
                    end
                end
                default: ;
            endcase

            if (uop.count == X87_COUNT_DEC) begin
                if (trans_operation)
                    trans_count_r <= trans_count_r - 8'd1;
                else
                    count_r <= count_r - 6'd1;
            end

            if ((uop.grs == X87_GRS_SHIFT) &&
                (uop.shift_route == X87_SHIFT_RIGHT)) begin
                if (uop.count == X87_COUNT_DEC) begin
                    if (count_r == 6'd1)
                        guard_r <= work_r[0];
                    else
                        sticky_r <= sticky_r || work_r[0];
                end else begin
                    guard_r <= work_r[0];
                    round_r <= guard_r;
                    sticky_r <= sticky_r || round_r;
                end
            end

            case (uop.classify)
                X87_CLASSIFY_ROUNDINT: begin
                    logic signed [16:0] unbiased;
                    unbiased = $signed({2'b00, operand.exp}) - 17'sd16383;
                    unbiased_r <= unbiased;
                    special_r <= (operand.class_id != X87_NORMAL) &&
                                 (operand.class_id != X87_DENORMAL);
                    integral_r <= unbiased >= 17'sd52;
                    subunit_r <= unbiased < 0;
                end

                X87_CLASSIFY_FIST: begin
                    logic signed [16:0] unbiased;
                    unbiased = $signed({2'b00, operand.exp}) - 17'sd16383;
                    unbiased_r <= unbiased;
                    zero_r <= operand.class_id == X87_ZERO;
                    invalid_r <= ((operand.class_id != X87_ZERO) &&
                                  (operand.class_id != X87_NORMAL) &&
                                  (operand.class_id != X87_DENORMAL)) ||
                                 (((operand.class_id == X87_NORMAL) ||
                                   (operand.class_id == X87_DENORMAL)) &&
                                  (unbiased > 17'sd63));
                    shift_left_r <= unbiased > 17'sd52;
                    if (operand.class_id == X87_ZERO)
                        work_r <= 68'h0;
                end

                X87_CLASSIFY_FIST_RANGE: begin
                    logic upper_nonzero;
                    logic range_overflow;
                    upper_nonzero = |work_r[67:64];
                    case (integer_size)
                        2'd0:
                            range_overflow = |work_r[67:16] ||
                                (operand.sign ? work_r[15]
                                                  ? |work_r[14:0]
                                                  : 1'b0
                                                  : work_r[15]);
                        2'd1:
                            range_overflow = |work_r[67:32] ||
                                (operand.sign ? work_r[31]
                                                  ? |work_r[30:0]
                                                  : 1'b0
                                                  : work_r[31]);
                        default:
                            range_overflow = upper_nonzero ||
                                (operand.sign ? work_r[63]
                                                  ? |work_r[62:0]
                                                  : 1'b0
                                                  : work_r[63]);
                    endcase
                    invalid_r <= range_overflow;
                end

                X87_CLASSIFY_ADDSUB: begin
                    logic [2:0] relation;
                    x87_reg_t effective_b;

                    effective_b = operand_b;
                    effective_b.sign = effective_b_sign;
                    direct_ready_r <= 1'b1;
                    if (exec_op == X87_ARITH_COMPARE) begin
                        relation = compare_relation(operand, operand_b,
                                                    operand_magnitude);
                        compare_unordered <= relation[2];
                        compare_less <= relation[1];
                        compare_equal <= relation[0];
                        invalid <= relation[2] &&
                                   (!quiet_compare ||
                                    is_signaling_nan(operand) ||
                                    is_signaling_nan(operand_b));
                    end else if (is_nan(operand) || is_nan(operand_b)) begin
                        result_r <= is_nan(operand) ? quiet_nan(operand)
                                                   : quiet_nan(operand_b);
                        invalid <= is_signaling_nan(operand) ||
                                   is_signaling_nan(operand_b);
                    end else if (is_infinity(operand) &&
                                 is_infinity(operand_b) &&
                                 (operand.sign != effective_b_sign)) begin
                        result_r <= x87_indefinite();
                        invalid <= 1'b1;
                    end else if (is_infinity(operand)) begin
                        result_r <= operand;
                    end else if (is_infinity(operand_b)) begin
                        result_r <= effective_b;
                    end else if (is_zero(operand) && is_zero(operand_b)) begin
                        result_r <= x87_zero(
                            (operand.sign == effective_b_sign)
                                ? operand.sign : (rounding_mode == 2'b01));
                    end else if (is_zero(operand)) begin
                        result_r <= effective_b;
                    end else if (is_zero(operand_b)) begin
                        result_r <= operand;
                    end else begin
                        direct_ready_r <= 1'b0;
                    end
                end

                X87_CLASSIFY_MUL: begin
                    mul_result_sign_r <= operand.sign ^ operand_b.sign;
                    direct_ready_r <= 1'b1;
                    if (is_nan(operand) || is_nan(operand_b)) begin
                        result_r <= is_nan(operand) ? quiet_nan(operand)
                                                   : quiet_nan(operand_b);
                        invalid <= is_signaling_nan(operand) ||
                                   is_signaling_nan(operand_b);
                    end else if ((is_infinity(operand) && is_zero(operand_b)) ||
                                 (is_zero(operand) && is_infinity(operand_b))) begin
                        result_r <= x87_indefinite();
                        invalid <= 1'b1;
                    end else if (is_infinity(operand) ||
                                 is_infinity(operand_b)) begin
                        result_r <= '0;
                        result_r.sign <= operand.sign ^ operand_b.sign;
                        result_r.exp <= 15'h7fff;
                        result_r.sig <= {1'b1, 52'h0};
                        result_r.class_id <= X87_INFINITY;
                    end else if (is_zero(operand) || is_zero(operand_b)) begin
                        result_r <= x87_zero(operand.sign ^ operand_b.sign);
                    end else begin
                        direct_ready_r <= 1'b0;
                    end
                end

                X87_CLASSIFY_DIVSQRT: begin
                    logic square_root;

                    square_root = exec_op == X87_ARITH_SQRT;
                    divsqrt_result_sign_r <= square_root
                                           ? operand.sign
                                           : operand.sign ^ operand_b.sign;
                    direct_ready_r <= 1'b1;
                    if (is_nan(operand) ||
                        (!square_root && is_nan(operand_b))) begin
                        result_r <= is_nan(operand) ? quiet_nan(operand)
                                                   : quiet_nan(operand_b);
                        invalid <= is_signaling_nan(operand) ||
                                   (!square_root &&
                                    is_signaling_nan(operand_b));
                    end else if (square_root) begin
                        if (operand.sign && !is_zero(operand)) begin
                            result_r <= x87_indefinite();
                            invalid <= 1'b1;
                        end else if (is_infinity(operand) ||
                                     is_zero(operand)) begin
                            result_r <= operand;
                        end else begin
                            direct_ready_r <= 1'b0;
                        end
                    end else if ((is_zero(operand) && is_zero(operand_b)) ||
                                 (is_infinity(operand) &&
                                  is_infinity(operand_b))) begin
                        result_r <= x87_indefinite();
                        invalid <= 1'b1;
                    end else if (is_zero(operand_b)) begin
                        result_r <= '0;
                        result_r.sign <= operand.sign ^ operand_b.sign;
                        result_r.exp <= 15'h7fff;
                        result_r.sig <= {1'b1, 52'h0};
                        result_r.class_id <= X87_INFINITY;
                        divide_by_zero <= is_finite(operand);
                    end else if (is_infinity(operand)) begin
                        result_r <= '0;
                        result_r.sign <= operand.sign ^ operand_b.sign;
                        result_r.exp <= 15'h7fff;
                        result_r.sig <= {1'b1, 52'h0};
                        result_r.class_id <= X87_INFINITY;
                    end else if (is_zero(operand) ||
                                 is_infinity(operand_b)) begin
                        result_r <= x87_zero(operand.sign ^ operand_b.sign);
                    end else begin
                        direct_ready_r <= 1'b0;
                    end
                end

                X87_CLASSIFY_TRANS: begin
                    logic signed [16:0] unbiased_exp;
                    logic [14:0] exponent_delta;

                    unbiased_exp = $signed({2'b00, operand.exp}) -
                                   17'sd16383;
                    exponent_delta = 15'h0;
                    direct_ready_r <= 1'b1;
                    trans_atan_address_r <= 7'd0;

                    if (trans_atan2 &&
                        (is_nan(operand) || is_nan(operand_b))) begin
                        result_r <= is_nan(operand) ? quiet_nan(operand)
                                                   : quiet_nan(operand_b);
                        invalid <= is_signaling_nan(operand) ||
                                   is_signaling_nan(operand_b);
                    end else if (trans_atan2 &&
                                 ((operand.class_id == X87_EMPTY) ||
                                  (operand_b.class_id == X87_EMPTY))) begin
                        result_r <= x87_indefinite();
                        invalid <= 1'b1;
                    end else if (trans_atan2 && is_zero(operand)) begin
                        if (operand_b.sign) begin
                            trans_result_sign_r <= operand.sign;
                            trans_magnitude_r <= TRANS_PI_Q80;
                            trans_exp_r <= 17'sd16383;
                            inexact <= 1'b1;
                            special_r <= 1'b1;
                            direct_ready_r <= 1'b0;
                        end else begin
                            result_r <= x87_zero(operand.sign);
                        end
                    end else if (trans_atan2 && is_zero(operand_b)) begin
                        trans_result_sign_r <= operand.sign;
                        trans_magnitude_r <= TRANS_PIO2_Q80;
                        trans_exp_r <= 17'sd16383;
                        inexact <= 1'b1;
                        special_r <= 1'b1;
                        direct_ready_r <= 1'b0;
                    end else if (trans_atan2 && is_infinity(operand) &&
                                 is_infinity(operand_b)) begin
                        trans_result_sign_r <= operand.sign;
                        trans_magnitude_r <= operand_b.sign
                            ? TRANS_PI_Q80 - TRANS_PIO4_Q80
                            : TRANS_PIO4_Q80;
                        trans_exp_r <= 17'sd16383;
                        inexact <= 1'b1;
                        special_r <= 1'b1;
                        direct_ready_r <= 1'b0;
                    end else if (trans_atan2 && is_infinity(operand)) begin
                        trans_result_sign_r <= operand.sign;
                        trans_magnitude_r <= TRANS_PIO2_Q80;
                        trans_exp_r <= 17'sd16383;
                        inexact <= 1'b1;
                        special_r <= 1'b1;
                        direct_ready_r <= 1'b0;
                    end else if (trans_atan2 && is_infinity(operand_b)) begin
                        if (operand_b.sign) begin
                            trans_result_sign_r <= operand.sign;
                            trans_magnitude_r <= TRANS_PI_Q80;
                            trans_exp_r <= 17'sd16383;
                            inexact <= 1'b1;
                            special_r <= 1'b1;
                            direct_ready_r <= 1'b0;
                        end else begin
                            result_r <= x87_zero(operand.sign);
                        end
                    end else if (trans_atan2) begin
                        trans_x_r <=
                            $signed({2'b00, operand_b.sig, 28'h0});
                        trans_y_r <=
                            $signed({2'b00, operand.sig, 28'h0});
                        trans_z_r <= 83'sh0;
                        trans_x_sign_r <= operand_b.sign;
                        trans_y_sign_r <= operand.sign;
                        direct_ready_r <= 1'b0;
                        if (operand_b.exp < operand.exp) begin
                            exponent_delta = operand.exp - operand_b.exp;
                            trans_count_r <= exponent_delta > 15'd82
                                           ? 8'd82
                                           : {1'b0, exponent_delta[6:0]};
                            trans_shift_x_r <= 1'b1;
                        end else if (operand.exp < operand_b.exp) begin
                            exponent_delta = operand_b.exp - operand.exp;
                            trans_count_r <= exponent_delta > 15'd82
                                           ? 8'd82
                                           : {1'b0, exponent_delta[6:0]};
                            trans_shift_x_r <= 1'b0;
                        end else begin
                            trans_count_r <= 8'd0;
                            trans_shift_x_r <= 1'b0;
                        end
                    end else if (is_nan(operand)) begin
                        result_r <= quiet_nan(operand);
                        invalid <= is_signaling_nan(operand);
                    end else if (is_infinity(operand) ||
                                 (operand.class_id == X87_EMPTY)) begin
                        result_r <= x87_indefinite();
                        invalid <= 1'b1;
                    end else if (is_zero(operand)) begin
                        result_r <= trans_cosine ? x87_one() : operand;
                        if (trans_tangent_pair)
                            trans_auxiliary_result_r <= x87_one();
                    end else if (unbiased_exp >= 17'sd63) begin
                        result_r <= operand;
                        range_incomplete <= 1'b1;
                    end else if (unbiased_exp <= -17'sd27) begin
                        result_r <= trans_cosine ? x87_one() : operand;
                        if (trans_tangent_pair)
                            trans_auxiliary_result_r <= x87_one();
                        inexact <= 1'b1;
                    end else begin
                        trans_range_sig_r <= operand.sig;
                        trans_count_r <= unbiased_exp[7:0] + 8'd121;
                        trans_range_remainder_r <= 121'h0;
                        trans_quadrant_r <= 2'b00;
                        direct_ready_r <= 1'b0;
                    end
                end

                default: ;
            endcase

            case (uop.engine)
                X87_ENGINE_ADDSUB_ALIGN:
                    add_small_r <= {1'b0, add_small_r[55:2],
                                    add_small_r[1] | add_small_r[0]};

                X87_ENGINE_MUL_ISSUE:
                    mul_product_r <= mul_a_r * mul_b_r;

                X87_ENGINE_MUL_ACCUMULATE: begin
                    mul_accum_r <= mul_accum_next;
                    if (mul_partial_r != 2'd3) begin
                        mul_partial_r <= mul_partial_r + 2'd1;
                        case (mul_partial_r)
                            2'd0: begin
                                mul_a_r <= {1'b0, operand.sig[52:27]};
                                mul_b_r <= operand_b.sig[26:0];
                            end
                            2'd1: begin
                                mul_a_r <= operand.sig[26:0];
                                mul_b_r <= {1'b0, operand_b.sig[52:27]};
                            end
                            default: begin
                                mul_a_r <= {1'b0, operand.sig[52:27]};
                                mul_b_r <= {1'b0, operand_b.sig[52:27]};
                            end
                        endcase
                    end
                end

                X87_ENGINE_DIV_ITERATE: begin
                    divsqrt_result_bits_r <=
                        {divsqrt_result_bits_r[54:0], divsqrt_next_bit};
                    divsqrt_remainder_r <= divsqrt_next_bit
                        ? divsqrt_after_subtract
                        : divsqrt_shifted;
                end

                X87_ENGINE_SQRT_ITERATE: begin
                    divsqrt_radicand_r <= divsqrt_radicand_r << 2;
                    divsqrt_result_bits_r <=
                        {divsqrt_result_bits_r[54:0], divsqrt_next_bit};
                    divsqrt_remainder_r <= divsqrt_next_bit
                        ? divsqrt_after_subtract
                        : divsqrt_shifted;
                end

                X87_ENGINE_TRANS_RANGE_ITERATE: begin
                    trans_range_sig_r <= trans_range_sig_r << 1;
                    trans_range_remainder_r <= trans_range_next;
                    trans_quadrant_r <= trans_quadrant_next;
                end

                X87_ENGINE_TRANS_RANGE_FINALIZE: begin
                    trans_z_r <= trans_reduced_q80;
                    trans_quadrant_r <=
                        (trans_range_remainder_r > TRANS_PIO4_Q120)
                            ? trans_quadrant_r + 2'd1
                            : trans_quadrant_r;
                    trans_atan_address_r <= 7'd0;
                end

                X87_ENGINE_TRANS_ALIGN: begin
                    if (trans_shift_x_r)
                        trans_x_r <= trans_x_r >>> 1;
                    else
                        trans_y_r <= trans_y_r >>> 1;
                end

                X87_ENGINE_TRANS_CORDIC_PREP: begin
                    if (!trans_atan2) begin
                        trans_x_r <= TRANS_CORDIC_K_Q80;
                        trans_y_r <= 83'sh0;
                    end
                    trans_count_r <= 8'd80;
                    trans_atan_address_r <= 7'd1;
                end

                default: ;
            endcase

            case (uop.alu_route)
                X87_ALU_ROUND: begin
                    work_r <= rounded_work;
                    inexact <= round_discarded;
                    rounded_up <= round_increment;
                end

                X87_ALU_CALCULATE_ADDSUB:
                    work_r <= arithmetic_result;

                X87_ALU_NORMALIZE_ADDSUB: begin
                    if (work_r[56:0] == 57'h0) begin
                        result_r <= x87_zero(rounding_mode == 2'b01);
                        direct_ready_r <= 1'b1;
                    end else if (work_r[56]) begin
                        work_r <= {11'h0, 1'b0, work_r[56:2],
                                   work_r[1] | work_r[0]};
                        add_exp_r <= add_exp_r + 15'd1;
                    end else if (!work_r[55]) begin
                        work_r <= work_r << 1;
                        add_exp_r <= add_exp_r - 15'd1;
                    end
                end

                X87_ALU_PREP_ROUND_ADDSUB: begin
                    if (precision_control == 2'b00) begin
                        work_r <= {44'h0, work_r[55:32]};
                        guard_r <= work_r[31];
                        sticky_r <= |work_r[30:0];
                    end else begin
                        work_r <= {15'h0, work_r[55:3]};
                        guard_r <= work_r[2];
                        sticky_r <= |work_r[1:0];
                    end
                end

                X87_ALU_PREP_ROUND_MUL: begin
                    logic [52:0] normalized_sig;
                    logic normalized_guard;
                    logic normalized_round;
                    logic normalized_sticky;

                    if (mul_accum_r[105]) begin
                        normalized_sig = mul_accum_r[105:53];
                        normalized_guard = mul_accum_r[52];
                        normalized_round = mul_accum_r[51];
                        normalized_sticky = |mul_accum_r[50:0];
                        mul_exp_r <= mul_exp_r + 15'd1;
                    end else begin
                        normalized_sig = mul_accum_r[104:52];
                        normalized_guard = mul_accum_r[51];
                        normalized_round = mul_accum_r[50];
                        normalized_sticky = |mul_accum_r[49:0];
                    end

                    round_r <= 1'b0;
                    if (precision_control == 2'b00) begin
                        work_r <= {44'h0, normalized_sig[52:29]};
                        guard_r <= normalized_sig[28];
                        sticky_r <= |normalized_sig[27:0] ||
                                    normalized_guard || normalized_round ||
                                    normalized_sticky;
                    end else begin
                        work_r <= {15'h0, normalized_sig};
                        guard_r <= normalized_guard;
                        sticky_r <= normalized_round || normalized_sticky;
                    end
                end

                X87_ALU_PREP_ROUND_DIVSQRT: begin
                    round_r <= 1'b0;
                    if (precision_control == 2'b00) begin
                        work_r <= {44'h0, divsqrt_result_bits_r[55:32]};
                        guard_r <= divsqrt_result_bits_r[31];
                        sticky_r <= |divsqrt_result_bits_r[30:0] ||
                                    divsqrt_remainder_nonzero;
                    end else begin
                        work_r <= {15'h0, divsqrt_result_bits_r[55:3]};
                        guard_r <= divsqrt_result_bits_r[2];
                        sticky_r <= |divsqrt_result_bits_r[1:0] ||
                                    divsqrt_remainder_nonzero;
                    end
                end

                X87_ALU_PREP_ROUND_TRANS: begin
                    round_r <= 1'b0;
                    if (precision_control == 2'b00) begin
                        work_r <= {44'h0, trans_magnitude_r[80:57]};
                        guard_r <= trans_magnitude_r[56];
                        sticky_r <= |trans_magnitude_r[55:0];
                    end else begin
                        work_r <= {15'h0, trans_magnitude_r[80:28]};
                        guard_r <= trans_magnitude_r[27];
                        sticky_r <= |trans_magnitude_r[26:0];
                    end
                end

                X87_ALU_CORDIC_X: begin
                    trans_x_r <= cordic_result;
                end

                X87_ALU_CORDIC_Y:
                    trans_y_r <= cordic_result;

                X87_ALU_CORDIC_Z: begin
                    logic [6:0] iteration;

                    iteration = 7'd80 - trans_count_r[6:0];
                    trans_z_r <= cordic_result;
                    trans_atan_address_r <= iteration + 7'd2;
                end

                default: ;
            endcase

            case (uop.pack)
                X87_PACK_PACK_INTERNAL: begin
                    result_r <= '0;
                    result_r.sign <= operand.sign;
                    // WORK may contain a carry after the restore shift.
                    if (work_r[53]) begin
                        result_r.exp <= operand.exp + 15'd1;
                        result_r.sig <= {1'b1, 52'h0};
                    end else begin
                        result_r.exp <= operand.exp;
                        result_r.sig <= work_r[52:0];
                    end
                    result_r.class_id <= X87_NORMAL;
                end

                X87_PACK_COPY_A:
                    result_r <= operand;

                X87_PACK_ROUNDINT_SPECIAL: begin
                    if (operand.class_id == X87_NAN) begin
                        result_r <= quiet_nan(operand);
                        invalid <= is_signaling_nan(operand);
                    end else if ((operand.class_id == X87_INFINITY) ||
                                 (operand.class_id == X87_ZERO)) begin
                        result_r <= operand;
                    end else begin
                        result_r <= x87_indefinite();
                        invalid <= 1'b1;
                    end
                end

                X87_PACK_ROUNDINT_SUBUNIT: begin
                    logic above_half;
                    logic increment_to_one;
                    above_half = |operand.sig[51:0] || operand.guard_bit ||
                                 operand.round_bit || operand.sticky_bit;
                    increment_to_one = directed_increment(
                        operand.sign, rounding_mode, 1'b1);
                    if (rounding_mode == 2'b00)
                        increment_to_one =
                            (unbiased_r == -17'sd1) && above_half;
                    result_r <= increment_to_one ? x87_one()
                                                 : x87_zero(operand.sign);
                    if (increment_to_one)
                        result_r.sign <= operand.sign;
                    inexact <= 1'b1;
                    rounded_up <= increment_to_one;
                end

                X87_PACK_PACK_FIST: begin
                    logic [63:0] magnitude;
                    logic [63:0] signed_result;
                    magnitude = work_r[63:0];
                    signed_result = operand.sign ? (~magnitude + 64'd1)
                                                 : magnitude;
                    case (integer_size)
                        2'd0: transfer_r <= {48'h0, signed_result[15:0]};
                        2'd1: transfer_r <= {32'h0, signed_result[31:0]};
                        default: transfer_r <= signed_result;
                    endcase
                end

                X87_PACK_FIST_INVALID: begin
                    transfer_r <= integer_indefinite(integer_size);
                    invalid <= 1'b1;
                    // Invalid conversion takes precedence over precision.
                    inexact <= 1'b0;
                    rounded_up <= 1'b0;
                end

                X87_PACK_ROUND_PACK_FST: begin
                    logic signed [16:0] packed_exp;
                    inexact <= round_discarded;
                    if (format_m32) begin
                        if (format_exp_r <= 0) begin
                            underflow <= round_discarded;
                            transfer_r <= rounded_work[23]
                                ? {32'h0, operand.sign, 8'h01, 23'h0}
                                : {32'h0, operand.sign, 8'h00,
                                   rounded_work[22:0]};
                        end else begin
                            packed_exp = format_exp_r +
                                         (rounded_work[24] ? 17'sd1 : 17'sd0);
                            if (packed_exp >= 17'sd255) begin
                                overflow <= 1'b1;
                                inexact <= 1'b1;
                                transfer_r <= overflow_to_infinity(
                                    operand.sign, rounding_mode)
                                    ? {32'h0, operand.sign, 8'hff, 23'h0}
                                    : {32'h0, operand.sign, 8'hfe, 23'h7fffff};
                            end else if (rounded_work[24])
                                transfer_r <= {32'h0, operand.sign,
                                               packed_exp[7:0],
                                               rounded_work[23:1]};
                            else
                                transfer_r <= {32'h0, operand.sign,
                                               packed_exp[7:0],
                                               rounded_work[22:0]};
                        end
                    end else begin
                        if (format_exp_r <= 0) begin
                            underflow <= round_discarded;
                            transfer_r <= rounded_work[52]
                                ? {operand.sign, 11'h001, 52'h0}
                                : {operand.sign, 11'h000,
                                   rounded_work[51:0]};
                        end else begin
                            packed_exp = format_exp_r +
                                         (rounded_work[53] ? 17'sd1 : 17'sd0);
                            if (packed_exp >= 17'sd2047) begin
                                overflow <= 1'b1;
                                inexact <= 1'b1;
                                transfer_r <= overflow_to_infinity(
                                    operand.sign, rounding_mode)
                                    ? {operand.sign, 11'h7ff, 52'h0}
                                    : {operand.sign, 11'h7fe,
                                       52'hf_ffff_ffff_ffff};
                            end else if (rounded_work[53])
                                transfer_r <= {operand.sign,
                                               packed_exp[10:0],
                                               rounded_work[52:1]};
                            else
                                transfer_r <= {operand.sign,
                                               packed_exp[10:0],
                                               rounded_work[51:0]};
                        end
                    end
                end

                X87_PACK_PACK_FST_SPECIAL: begin
                    if (format_m32) begin
                        case (operand.class_id)
                            X87_ZERO:
                                transfer_r <= {32'h0, operand.sign, 31'h0};
                            X87_INFINITY:
                                transfer_r <= {32'h0, operand.sign,
                                               8'hff, 23'h0};
                            X87_NAN:
                                begin
                                    transfer_r <= {32'h0, operand.sign,
                                                   8'hff, 1'b1,
                                                   operand.sig[50:29]};
                                    invalid <= is_signaling_nan(operand);
                                end
                            default:
                                transfer_r <= 64'h0000_0000_ffc0_0000;
                        endcase
                    end else begin
                        case (operand.class_id)
                            X87_ZERO:
                                transfer_r <= {operand.sign, 63'h0};
                            X87_INFINITY:
                                transfer_r <= {operand.sign, 11'h7ff, 52'h0};
                            X87_NAN:
                                begin
                                    transfer_r <= {operand.sign, 11'h7ff,
                                                   1'b1, operand.sig[50:0]};
                                    invalid <= is_signaling_nan(operand);
                                end
                            default:
                                transfer_r <= 64'hfff8_0000_0000_0000;
                        endcase
                    end
                end

                X87_PACK_PACK_FLD_DENORMAL: begin
                    result_r <= '0;
                    result_r.sign <= format_m32 ? transfer_in[31]
                                                : transfer_in[63];
                    result_r.exp <= format_exp_r[14:0];
                    result_r.sig <= work_r[52:0];
                    result_r.class_id <= X87_DENORMAL;
                end

                X87_PACK_PACK_FILD: begin
                    result_r <= '0;
                    result_r.sign <= input_sign_r;
                    result_r.exp <= format_exp_r[14:0];
                    result_r.sig <= work_r[52:0];
                    result_r.guard_bit <= guard_r;
                    result_r.round_bit <= round_r;
                    result_r.sticky_bit <= sticky_r;
                    result_r.class_id <= X87_NORMAL;
                end

                X87_PACK_PACK_ADDSUB: begin
                    result_r <= '0;
                    result_r.sign <= add_result_sign_r;
                    result_r.class_id <= X87_NORMAL;
                    if (precision_control == 2'b00) begin
                        result_r.exp <= add_exp_r +
                                        {{14{1'b0}}, work_r[24]};
                        result_r.sig <= work_r[24]
                                      ? {1'b1, 52'h0}
                                      : {work_r[23:0], 29'h0};
                    end else begin
                        result_r.exp <= add_exp_r +
                                        {{14{1'b0}}, work_r[53]};
                        result_r.sig <= work_r[53]
                                      ? {1'b1, 52'h0} : work_r[52:0];
                    end
                end

                X87_PACK_PACK_MUL: begin
                    result_r <= '0;
                    result_r.sign <= mul_result_sign_r;
                    result_r.class_id <= X87_NORMAL;
                    if (precision_control == 2'b00) begin
                        result_r.exp <= mul_exp_r +
                                        {{14{1'b0}}, work_r[24]};
                        result_r.sig <= work_r[24]
                                      ? {1'b1, 52'h0}
                                      : {work_r[23:0], 29'h0};
                    end else begin
                        result_r.exp <= mul_exp_r +
                                        {{14{1'b0}}, work_r[53]};
                        result_r.sig <= work_r[53]
                                      ? {1'b1, 52'h0} : work_r[52:0];
                    end
                end

                X87_PACK_PACK_DIVSQRT: begin
                    logic rounded_carry;
                    logic signed [17:0] rounded_exp;

                    rounded_carry = (precision_control == 2'b00)
                                  ? work_r[24] : work_r[53];
                    rounded_exp =
                        $signed({divsqrt_exp_r[16], divsqrt_exp_r}) +
                        $signed({17'h0, rounded_carry});
                    result_r <= '0;
                    result_r.sign <= divsqrt_result_sign_r;
                    inexact <= round_discarded;
                    if (rounded_exp >= 18'sd32767) begin
                        overflow <= 1'b1;
                        inexact <= 1'b1;
                        if (overflow_to_infinity(divsqrt_result_sign_r,
                                                 rounding_mode)) begin
                            result_r.exp <= 15'h7fff;
                            result_r.sig <= {1'b1, 52'h0};
                            result_r.class_id <= X87_INFINITY;
                        end else begin
                            result_r.exp <= 15'h7ffe;
                            result_r.sig <= {53{1'b1}};
                            result_r.class_id <= X87_NORMAL;
                        end
                    end else if (rounded_exp <= 0) begin
                        result_r <= x87_zero(divsqrt_result_sign_r);
                        underflow <= 1'b1;
                        inexact <= 1'b1;
                    end else begin
                        result_r.exp <= rounded_exp[14:0];
                        result_r.sig <= (precision_control == 2'b00)
                                      ? (work_r[24]
                                         ? {1'b1, 52'h0}
                                         : {work_r[23:0], 29'h0})
                                      : (work_r[53]
                                         ? {1'b1, 52'h0}
                                         : work_r[52:0]);
                        result_r.class_id <= X87_NORMAL;
                    end
                end

                X87_PACK_PACK_TRANS: begin
                    x87_reg_t packed_value;
                    logic rounded_carry;

                    rounded_carry = (precision_control == 2'b00)
                                  ? work_r[24] : work_r[53];
                    packed_value = '0;
                    packed_value.sign = trans_result_sign_r;
                    packed_value.exp = trans_exp_r[14:0] +
                                       {{14{1'b0}}, rounded_carry};
                    packed_value.sig = (precision_control == 2'b00)
                               ? (work_r[24]
                                  ? {1'b1, 52'h0}
                                  : {work_r[23:0], 29'h0})
                               : (work_r[53]
                                  ? {1'b1, 52'h0} : work_r[52:0]);
                    packed_value.class_id = X87_NORMAL;
                    if (trans_rounding_aux_r)
                        trans_auxiliary_result_r <= packed_value;
                    else
                        result_r <= packed_value;
                end

                default: ;
            endcase

            case (uop.prepare)
                X87_PREPARE_LOAD_ROUNDINT: begin
                    count_r <= 6'(17'sd52 - unbiased_r);
                    restore_count_r <= 6'(17'sd52 - unbiased_r);
                    work_r <= {15'h0, operand.sig};
                    guard_r <= 1'b0;
                    sticky_r <= operand.guard_bit || operand.round_bit ||
                                operand.sticky_bit;
                end

                X87_PREPARE_LOAD_FIST: begin
                    logic signed [16:0] shift_count;
                    work_r <= {15'h0, operand.sig};
                    if (unbiased_r > 17'sd52) begin
                        shift_count = unbiased_r - 17'sd52;
                        count_r <= (shift_count > 17'sd63) ? 6'd63
                                                          : 6'(shift_count);
                        guard_r <= operand.guard_bit;
                        sticky_r <= operand.round_bit ||
                                    operand.sticky_bit;
                    end else begin
                        shift_count = 17'sd52 - unbiased_r;
                        count_r <= (shift_count > 17'sd54) ? 6'd54
                                                          : 6'(shift_count);
                        guard_r <= 1'b0;
                        sticky_r <= operand.guard_bit ||
                                    operand.round_bit ||
                                    operand.sticky_bit;
                    end
                end

                X87_PREPARE_FST: begin
                    logic signed [16:0] format_exp;
                    logic signed [16:0] shift_count;
                    format_exp = $signed({2'b00, operand.exp}) -
                                 (format_m32 ? 17'sd16256 : 17'sd15360);
                    format_exp_r <= format_exp;
                    special_r <= (operand.class_id != X87_NORMAL) &&
                                 (operand.class_id != X87_DENORMAL);
                    if (format_m32 && (format_exp > 0)) begin
                        work_r <= {44'h0, operand.sig[52:29]};
                        count_r <= 6'd0;
                        guard_r <= operand.sig[28];
                        round_r <= 1'b0;
                        sticky_r <= |operand.sig[27:0] ||
                                    operand.guard_bit ||
                                    operand.round_bit ||
                                    operand.sticky_bit;
                    end else if (!format_m32 && (format_exp > 0)) begin
                        work_r <= {15'h0, operand.sig};
                        count_r <= 6'd0;
                        guard_r <= operand.guard_bit;
                        round_r <= 1'b0;
                        sticky_r <= operand.round_bit ||
                                    operand.sticky_bit;
                    end else begin
                        work_r <= {15'h0, operand.sig};
                        shift_count = (format_m32 ? 17'sd30 : 17'sd1) -
                                      format_exp;
                        count_r <= (shift_count > 17'sd54) ? 6'd54
                                                          : 6'(shift_count);
                        guard_r <= 1'b0;
                        round_r <= 1'b0;
                        sticky_r <= operand.guard_bit ||
                                    operand.round_bit ||
                                    operand.sticky_bit;
                    end
                end

                X87_PREPARE_FLD: begin
                    result_r <= '0;
                    direct_ready_r <= 1'b1;
                    if (format_m32) begin
                        result_r.sign <= transfer_in[31];
                        if (transfer_in[30:23] == 8'h00) begin
                            if (transfer_in[22:0] == 23'h0) begin
                                result_r.class_id <= X87_ZERO;
                            end else begin
                                work_r <= {45'h0, transfer_in[22:0]};
                                format_exp_r <= 17'sd16286;
                                direct_ready_r <= 1'b0;
                            end
                        end else if (transfer_in[30:23] == 8'hff) begin
                            result_r.exp <= 15'h7fff;
                            result_r.sig <= {
                                1'b1,
                                transfer_in[22] || |transfer_in[21:0],
                                transfer_in[21:0], 29'h0};
                            result_r.class_id <= (transfer_in[22:0] == 23'h0)
                                               ? X87_INFINITY : X87_NAN;
                            invalid <= (transfer_in[22:0] != 23'h0) &&
                                       !transfer_in[22];
                        end else begin
                            result_r.exp <= {7'h0, transfer_in[30:23]} +
                                            15'd16256;
                            result_r.sig <= {1'b1, transfer_in[22:0], 29'h0};
                            result_r.class_id <= X87_NORMAL;
                        end
                    end else begin
                        result_r.sign <= transfer_in[63];
                        if (transfer_in[62:52] == 11'h000) begin
                            if (transfer_in[51:0] == 52'h0) begin
                                result_r.class_id <= X87_ZERO;
                            end else begin
                                work_r <= {16'h0, transfer_in[51:0]};
                                format_exp_r <= 17'sd15361;
                                direct_ready_r <= 1'b0;
                            end
                        end else if (transfer_in[62:52] == 11'h7ff) begin
                            result_r.exp <= 15'h7fff;
                            result_r.sig <= {
                                1'b1,
                                transfer_in[51] || |transfer_in[50:0],
                                transfer_in[50:0]};
                            result_r.class_id <= (transfer_in[51:0] == 52'h0)
                                               ? X87_INFINITY : X87_NAN;
                            invalid <= (transfer_in[51:0] != 52'h0) &&
                                       !transfer_in[51];
                        end else begin
                            result_r.exp <= {4'h0, transfer_in[62:52]} +
                                            15'd15360;
                            result_r.sig <= {1'b1, transfer_in[51:0]};
                            result_r.class_id <= X87_NORMAL;
                        end
                    end
                end

                X87_PREPARE_FILD: begin
                    logic signed [63:0] signed_input;
                    logic [63:0] magnitude;
                    signed_input = sized_integer(transfer_in, integer_size);
                    magnitude = signed_input[63] ? (~signed_input + 64'd1)
                                                 : signed_input;
                    result_r <= x87_zero(1'b0);
                    direct_ready_r <= signed_input == 0;
                    shift_right_r <= |magnitude[63:53];
                    shift_left_r <= (magnitude != 0) &&
                                    !(|magnitude[63:52]);
                    work_r <= {4'h0, magnitude};
                    format_exp_r <= 17'sd16435;
                    guard_r <= 1'b0;
                    round_r <= 1'b0;
                    sticky_r <= 1'b0;
                    zero_r <= signed_input == 0;
                    input_sign_r <= signed_input[63];
                end

                default: begin end
            endcase

            case (uop.state)
                X87_STATE_RESTORE_COUNT:
                    count_r <= restore_count_r;

                X87_STATE_FORMAT_EXP_DEC: begin
                    format_exp_r <= format_exp_r - 17'sd1;
                end

                X87_STATE_FORMAT_EXP_INC: begin
                    format_exp_r <= format_exp_r + 17'sd1;
                end

                default: begin end
            endcase

            case (uop.prepare)
                X87_PREPARE_ADDSUB: begin
                    logic [55:0] a_extended;
                    logic [55:0] b_extended;
                    logic [14:0] exponent_delta;

                    a_extended = {operand.sig, operand.guard_bit,
                                  operand.round_bit, operand.sticky_bit};
                    b_extended = {operand_b.sig, operand_b.guard_bit,
                                  operand_b.round_bit, operand_b.sticky_bit};
                    if (operand_magnitude != 2'd1) begin
                        work_r <= {12'h0, a_extended};
                        add_small_r <= b_extended;
                        add_result_sign_r <= operand.sign;
                        add_small_sign_r <= effective_b_sign;
                        add_exp_r <= operand.exp;
                        exponent_delta = operand.exp - operand_b.exp;
                    end else begin
                        work_r <= {12'h0, b_extended};
                        add_small_r <= a_extended;
                        add_result_sign_r <= effective_b_sign;
                        add_small_sign_r <= operand.sign;
                        add_exp_r <= operand_b.exp;
                        exponent_delta = operand_b.exp - operand.exp;
                    end
                    if (exponent_delta >= 15'd56) begin
                        add_small_r <= 56'h1;
                        count_r <= 6'd0;
                    end else begin
                        count_r <= exponent_delta[5:0];
                    end
                end

                X87_PREPARE_MUL: begin
                    mul_exp_r <= operand.exp + operand_b.exp - 15'h3fff;
                    mul_accum_r <= 106'h0;
                    mul_partial_r <= 2'd0;
                    count_r <= 6'd4;
                    mul_a_r <= operand.sig[26:0];
                    mul_b_r <= operand_b.sig[26:0];
                end

                X87_PREPARE_DIV: begin
                    divsqrt_divisor_r <= operand_b.sig;
                    divsqrt_result_bits_r <= 56'h1;
                    count_r <= 6'd55;
                    divsqrt_exp_r <=
                        $signed({2'b00, operand.exp}) -
                        $signed({2'b00, operand_b.exp}) + 17'sd16383;
                    if (operand.sig >= operand_b.sig) begin
                        divsqrt_remainder_r <=
                            {4'h0, {1'b0, operand.sig} -
                                   {1'b0, operand_b.sig}};
                    end else begin
                        divsqrt_remainder_r <=
                            {4'h0, ({1'b0, operand.sig} << 1) -
                                   {1'b0, operand_b.sig}};
                        divsqrt_exp_r <=
                            $signed({2'b00, operand.exp}) -
                            $signed({2'b00, operand_b.exp}) + 17'sd16382;
                    end
                end

                X87_PREPARE_SQRT: begin
                    logic [14:0] exponent_distance;

                    divsqrt_result_sign_r <= operand.sign;
                    if (operand.exp >= 15'h3fff) begin
                        exponent_distance = operand.exp - 15'h3fff;
                        divsqrt_exp_r <= 17'sd16383 +
                            $signed({2'b00, exponent_distance >> 1});
                    end else begin
                        exponent_distance = 15'h3fff - operand.exp;
                        divsqrt_exp_r <= 17'sd16383 -
                            $signed({2'b00,
                                     (exponent_distance + 15'd1) >> 1});
                    end
                    divsqrt_radicand_r <= operand.exp[0]
                        ? {1'b0, operand.sig, 58'h0}
                        : {operand.sig, 59'h0};
                    divsqrt_remainder_r <= 58'h0;
                    divsqrt_result_bits_r <= 56'h0;
                    count_r <= 6'd56;
                end

                default: begin end
            endcase

            case (uop.state)
                X87_STATE_TRANS_SELECT: begin
                    logic signed [82:0] sine_value;
                    logic signed [82:0] cosine_value;
                    logic signed [82:0] selected_value;

                    sine_value = 83'sh0;
                    cosine_value = 83'sh0;
                    selected_value = 83'sh0;
                    if (trans_atan2) begin
                        if (trans_x_sign_r)
                            selected_value = trans_y_sign_r
                                ? trans_z_r - TRANS_PI_Q80
                                : TRANS_PI_Q80 - trans_z_r;
                        else
                            selected_value = trans_y_sign_r
                                ? -trans_z_r : trans_z_r;
                    end else begin
                        case (trans_quadrant_r)
                            2'd0: begin
                                sine_value = trans_y_r;
                                cosine_value = trans_x_r;
                            end
                            2'd1: begin
                                sine_value = trans_x_r;
                                cosine_value = -trans_y_r;
                            end
                            2'd2: begin
                                sine_value = -trans_y_r;
                                cosine_value = -trans_x_r;
                            end
                            default: begin
                                sine_value = -trans_x_r;
                                cosine_value = trans_y_r;
                            end
                        endcase
                        if (operand.sign)
                            sine_value = -sine_value;
                        selected_value = trans_cosine
                                       ? cosine_value : sine_value;
                        if (trans_tangent_pair) begin
                            trans_aux_sign_r <= cosine_value[82];
                            trans_aux_magnitude_r <= cosine_value[82]
                                                  ? -cosine_value
                                                  : cosine_value;
                        end
                    end
                    trans_result_sign_r <= selected_value[82];
                    trans_magnitude_r <= selected_value[82]
                                           ? -selected_value
                                           : selected_value;
                    trans_exp_r <= 17'sd16383;
                    inexact <= 1'b1;
                end

                X87_STATE_TRANS_NORMALIZE: begin
                    if (trans_magnitude_r[82:81] != 2'b00) begin
                        trans_magnitude_r <= trans_magnitude_r >> 1;
                        trans_exp_r <= trans_exp_r + 17'sd1;
                    end else if (!trans_magnitude_r[80]) begin
                        trans_magnitude_r <= trans_magnitude_r << 1;
                        trans_exp_r <= trans_exp_r - 17'sd1;
                    end
                end

                X87_STATE_TRANS_LOAD_AUX: begin
                    trans_result_sign_r <= trans_aux_sign_r;
                    trans_magnitude_r <= trans_aux_magnitude_r;
                    trans_exp_r <= 17'sd16383;
                    trans_rounding_aux_r <= 1'b1;
                end
                default: begin end
            endcase
        end
    end
end

endmodule
