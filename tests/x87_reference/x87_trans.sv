// Serialized trigonometric engine. A high-precision binary remainder stage
// reduces all architecturally valid arguments before an 80-step Q80 CORDIC.
module x87_trans
    import x87_pkg::*;
(
    input  logic       clk,
    input  logic       reset,
    input  logic       start,
    input  logic       cosine,
    input  logic       tangent_pair,
    input  logic       atan2_mode,
    input  logic [1:0] precision_control,
    input  logic [1:0] rounding_mode,
    input  x87_reg_t   operand,
    input  x87_reg_t   operand_b,

    output logic       busy,
    output logic       done,
    output x87_reg_t   result,
    output x87_reg_t   auxiliary_result,
    output logic       invalid,
    output logic       inexact,
    output logic       denormal_operand,
    output logic       range_incomplete
);

typedef enum logic [3:0] {
    TR_IDLE,
    TR_CLASSIFY,
    TR_RANGE,
    TR_ATAN_ALIGN,
    TR_CORDIC_PREPARE,
    TR_CORDIC,
    TR_SELECT,
    TR_NORMALIZE,
    TR_ROUND,
    TR_FINISH
} trans_state_t;

// Q120 gives enough pi/2 precision to reduce a 53-bit argument near 2^63
// without losing the final 53-bit trigonometric result.
localparam logic [120:0] PIO2_Q120 =
    121'h1921fb54442d18469898cc51701b83a;
localparam logic [120:0] PIO4_Q120 =
    121'h0c90fdaa22168c234c4c6628b80dc1d;
localparam logic signed [82:0] CORDIC_K_Q80 =
    83'h009b74eda8435e5a67f5f9;

trans_state_t state;
x87_reg_t operand_r;
x87_reg_t operand_b_r;
logic cosine_r;
logic tangent_pair_r;
logic atan2_r;
logic [1:0] precision_r;
logic [1:0] rounding_r;

logic [52:0] range_sig;
logic [7:0] range_count;
logic [120:0] range_remainder;
logic [1:0] quadrant;

logic signed [82:0] cordic_x;
logic signed [82:0] cordic_y;
logic signed [82:0] cordic_z;
logic [6:0] cordic_iteration;
logic [6:0] atan_address;
logic signed [82:0] atan_value;
logic [6:0] atan_align_count;
logic atan_shift_x;
logic atan_x_sign;
logic atan_y_sign;

logic result_sign;
logic [82:0] result_magnitude;
logic signed [16:0] result_exp;
logic auxiliary_sign;
logic [82:0] auxiliary_magnitude;
logic rounding_auxiliary;

logic range_bit;
logic range_subtract;
logic [121:0] range_shifted;
logic [121:0] range_after_subtract;
logic [120:0] range_next;
logic [1:0] quadrant_next;
logic signed [121:0] reduced_q120;
logic signed [82:0] reduced_q80;
logic signed [82:0] cordic_x_shift;
logic signed [82:0] cordic_y_shift;

logic [52:0] rounded_sig;
logic rounded_carry;
logic round_discarded;
logic round_increment;
x87_reg_t rounded_result;

localparam logic signed [82:0] PI_Q80 =
    83'sh03243f6a8885a308d3131a;
localparam logic signed [82:0] PIO2_Q80 =
    83'sh01921fb54442d18469898d;
localparam logic signed [82:0] PIO4_Q80 =
    83'sh00c90fdaa22168c234c4c6;

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

x87_cordic_rom atan_rom (
    .clk(clk),
    .address(atan_address),
    .value(atan_value)
);

always_comb begin
    logic guard_bit;
    logic sticky_bit;
    logic [24:0] rounded24;
    logic [53:0] rounded53;

    range_bit = range_sig[52];
    range_shifted = {range_remainder[120:0], range_bit};
    range_subtract = range_shifted >= {1'b0, PIO2_Q120};
    range_after_subtract = range_shifted - {1'b0, PIO2_Q120};
    range_next = range_subtract ? range_after_subtract[120:0]
                                : range_shifted[120:0];
    quadrant_next = {quadrant[0], range_subtract};
    if (range_next > PIO4_Q120)
        reduced_q120 = $signed({1'b0, range_next}) -
                       $signed({1'b0, PIO2_Q120});
    else
        reduced_q120 = $signed({1'b0, range_next});
    // The arithmetic shift by 40 leaves the sign plus Q80 bits below.
    reduced_q80 = {reduced_q120[121], reduced_q120[121:40]};

    cordic_x_shift = cordic_x >>> cordic_iteration;
    cordic_y_shift = cordic_y >>> cordic_iteration;

    rounded_sig = result_magnitude[80:28];
    rounded_carry = 1'b0;
    round_discarded = 1'b0;
    round_increment = 1'b0;
    guard_bit = 1'b0;
    sticky_bit = 1'b0;
    rounded24 = 25'h0;
    rounded53 = 54'h0;
    rounded_result = '0;

    if (precision_r == 2'b00) begin
        guard_bit = result_magnitude[56];
        sticky_bit = |result_magnitude[55:0];
        round_discarded = guard_bit || sticky_bit;
        case (rounding_r)
            2'b00: round_increment = guard_bit &&
                                      (sticky_bit || result_magnitude[57]);
            2'b01: round_increment = result_sign && round_discarded;
            2'b10: round_increment = !result_sign && round_discarded;
            default: round_increment = 1'b0;
        endcase
        rounded24 = {1'b0, result_magnitude[80:57]} +
                    {24'h0, round_increment};
        rounded_carry = rounded24[24];
        rounded_sig = rounded24[24]
                    ? {1'b1, 52'h0} : {rounded24[23:0], 29'h0};
    end else begin
        guard_bit = result_magnitude[27];
        sticky_bit = |result_magnitude[26:0];
        round_discarded = guard_bit || sticky_bit;
        case (rounding_r)
            2'b00: round_increment = guard_bit &&
                                      (sticky_bit || result_magnitude[28]);
            2'b01: round_increment = result_sign && round_discarded;
            2'b10: round_increment = !result_sign && round_discarded;
            default: round_increment = 1'b0;
        endcase
        rounded53 = {1'b0, result_magnitude[80:28]} +
                    {53'h0, round_increment};
        rounded_carry = rounded53[53];
        rounded_sig = rounded53[53]
                    ? {1'b1, 52'h0} : rounded53[52:0];
    end

    rounded_result.sign = result_sign;
    rounded_result.exp = result_exp[14:0] + {14'h0, rounded_carry};
    rounded_result.sig = rounded_sig;
    rounded_result.class_id = X87_NORMAL;
end

always_ff @(posedge clk) begin
    if (reset) begin
        state <= TR_IDLE;
        busy <= 1'b0;
        done <= 1'b0;
        result <= x87_empty();
        auxiliary_result <= x87_empty();
        invalid <= 1'b0;
        inexact <= 1'b0;
        denormal_operand <= 1'b0;
        range_incomplete <= 1'b0;
        operand_r <= x87_empty();
        operand_b_r <= x87_empty();
        cosine_r <= 1'b0;
        tangent_pair_r <= 1'b0;
        atan2_r <= 1'b0;
        precision_r <= 2'b00;
        rounding_r <= 2'b00;
        range_sig <= 53'h0;
        range_count <= 8'h0;
        range_remainder <= 121'h0;
        quadrant <= 2'b00;
        cordic_x <= 83'sh0;
        cordic_y <= 83'sh0;
        cordic_z <= 83'sh0;
        cordic_iteration <= 7'h0;
        atan_address <= 7'h0;
        atan_align_count <= 7'h0;
        atan_shift_x <= 1'b0;
        atan_x_sign <= 1'b0;
        atan_y_sign <= 1'b0;
        result_sign <= 1'b0;
        result_magnitude <= 83'h0;
        result_exp <= 17'sd0;
        auxiliary_sign <= 1'b0;
        auxiliary_magnitude <= 83'h0;
        rounding_auxiliary <= 1'b0;
    end else begin
        done <= 1'b0;

        case (state)
            TR_IDLE: begin
                if (start) begin
                    operand_r <= operand;
                    operand_b_r <= operand_b;
                    cosine_r <= cosine;
                    tangent_pair_r <= tangent_pair;
                    atan2_r <= atan2_mode;
                    precision_r <= precision_control;
                    rounding_r <= rounding_mode;
                    invalid <= 1'b0;
                    inexact <= 1'b0;
                    rounding_auxiliary <= 1'b0;
                    denormal_operand <= (operand.class_id == X87_DENORMAL) ||
                                        (atan2_mode &&
                                         (operand_b.class_id == X87_DENORMAL));
                    range_incomplete <= 1'b0;
                    busy <= 1'b1;
                    state <= TR_CLASSIFY;
                end
            end

            TR_CLASSIFY: begin
                logic signed [16:0] unbiased_exp;
                logic [14:0] exponent_delta;

                unbiased_exp = $signed({2'b00, operand_r.exp}) - 17'sd16383;
                exponent_delta = 15'h0;
                if (atan2_r && ((operand_r.class_id == X87_NAN) ||
                                (operand_b_r.class_id == X87_NAN))) begin
                    result <= (operand_r.class_id == X87_NAN)
                            ? quiet_nan(operand_r) : quiet_nan(operand_b_r);
                    invalid <= is_signaling_nan(operand_r) ||
                               is_signaling_nan(operand_b_r);
                    state <= TR_FINISH;
                end else if (atan2_r &&
                             (operand_r.class_id == X87_EMPTY ||
                              operand_b_r.class_id == X87_EMPTY)) begin
                    result <= x87_indefinite();
                    invalid <= 1'b1;
                    state <= TR_FINISH;
                end else if (atan2_r &&
                             (operand_r.class_id == X87_ZERO)) begin
                    if (operand_b_r.sign) begin
                        result_sign <= operand_r.sign;
                        result_magnitude <= PI_Q80;
                        result_exp <= 17'sd16383;
                        inexact <= 1'b1;
                        state <= TR_NORMALIZE;
                    end else begin
                        result <= x87_zero(operand_r.sign);
                        state <= TR_FINISH;
                    end
                end else if (atan2_r &&
                             (operand_b_r.class_id == X87_ZERO)) begin
                    result_sign <= operand_r.sign;
                    result_magnitude <= PIO2_Q80;
                    result_exp <= 17'sd16383;
                    inexact <= 1'b1;
                    state <= TR_NORMALIZE;
                end else if (atan2_r &&
                             (operand_r.class_id == X87_INFINITY) &&
                             (operand_b_r.class_id == X87_INFINITY)) begin
                    result_sign <= operand_r.sign;
                    result_magnitude <= operand_b_r.sign
                                      ? (PI_Q80 - PIO4_Q80) : PIO4_Q80;
                    result_exp <= 17'sd16383;
                    inexact <= 1'b1;
                    state <= TR_NORMALIZE;
                end else if (atan2_r &&
                             (operand_r.class_id == X87_INFINITY)) begin
                    result_sign <= operand_r.sign;
                    result_magnitude <= PIO2_Q80;
                    result_exp <= 17'sd16383;
                    inexact <= 1'b1;
                    state <= TR_NORMALIZE;
                end else if (atan2_r &&
                             (operand_b_r.class_id == X87_INFINITY)) begin
                    if (operand_b_r.sign) begin
                        result_sign <= operand_r.sign;
                        result_magnitude <= PI_Q80;
                        result_exp <= 17'sd16383;
                        inexact <= 1'b1;
                        state <= TR_NORMALIZE;
                    end else begin
                        result <= x87_zero(operand_r.sign);
                        state <= TR_FINISH;
                    end
                end else if (atan2_r) begin
                    cordic_x <= $signed({2'b00, operand_b_r.sig, 28'h0});
                    cordic_y <= $signed({2'b00, operand_r.sig, 28'h0});
                    cordic_z <= 83'sh0;
                    atan_x_sign <= operand_b_r.sign;
                    atan_y_sign <= operand_r.sign;
                    atan_address <= 7'd0;
                    if (operand_b_r.exp < operand_r.exp) begin
                        exponent_delta = operand_r.exp - operand_b_r.exp;
                        atan_align_count <= exponent_delta > 15'd82
                                          ? 7'd82 : exponent_delta[6:0];
                        atan_shift_x <= 1'b1;
                        state <= TR_ATAN_ALIGN;
                    end else if (operand_r.exp < operand_b_r.exp) begin
                        exponent_delta = operand_b_r.exp - operand_r.exp;
                        atan_align_count <= exponent_delta > 15'd82
                                          ? 7'd82 : exponent_delta[6:0];
                        atan_shift_x <= 1'b0;
                        state <= TR_ATAN_ALIGN;
                    end else begin
                        state <= TR_CORDIC_PREPARE;
                    end
                end else if (operand_r.class_id == X87_NAN) begin
                    result <= quiet_nan(operand_r);
                    invalid <= is_signaling_nan(operand_r);
                    state <= TR_FINISH;
                end else if ((operand_r.class_id == X87_INFINITY) ||
                             (operand_r.class_id == X87_EMPTY)) begin
                    result <= x87_indefinite();
                    invalid <= 1'b1;
                    state <= TR_FINISH;
                end else if (operand_r.class_id == X87_ZERO) begin
                    result <= cosine_r ? x87_one() : operand_r;
                    if (tangent_pair_r)
                        auxiliary_result <= x87_one();
                    state <= TR_FINISH;
                end else if (unbiased_exp >= 17'sd63) begin
                    // 80387 leaves ST(0) unchanged and sets C2 when reduction
                    // cannot complete for an argument at or above 2^63.
                    result <= operand_r;
                    range_incomplete <= 1'b1;
                    state <= TR_FINISH;
                end else if (unbiased_exp <= -17'sd27) begin
                    // At 53-bit precision sin(x) rounds to x and cos(x) to 1
                    // throughout this interval.
                    result <= cosine_r ? x87_one() : operand_r;
                    if (tangent_pair_r)
                        auxiliary_result <= x87_one();
                    inexact <= 1'b1;
                    state <= TR_FINISH;
                end else begin
                    range_sig <= operand_r.sig;
                    // Classification bounds this sum to 95..183.
                    range_count <= unbiased_exp[7:0] + 8'd121;
                    range_remainder <= 121'h0;
                    quadrant <= 2'b00;
                    state <= TR_RANGE;
                end
            end

            TR_RANGE: begin
                range_sig <= range_sig << 1;
                range_remainder <= range_next;
                quadrant <= quadrant_next;
                range_count <= range_count - 8'd1;
                if (range_count == 8'd1) begin
                    cordic_z <= reduced_q80;
                    quadrant <= (range_next > PIO4_Q120)
                              ? quadrant_next + 2'd1 : quadrant_next;
                    atan_address <= 7'd0;
                    state <= TR_CORDIC_PREPARE;
                end
            end

            TR_ATAN_ALIGN: begin
                if (atan_shift_x)
                    cordic_x <= cordic_x >>> 1;
                else
                    cordic_y <= cordic_y >>> 1;
                atan_align_count <= atan_align_count - 7'd1;
                if (atan_align_count == 7'd1)
                    state <= TR_CORDIC_PREPARE;
            end

            TR_CORDIC_PREPARE: begin
                if (!atan2_r) begin
                    cordic_x <= CORDIC_K_Q80;
                    cordic_y <= 83'sh0;
                end
                cordic_iteration <= 7'd0;
                // atan(0) is already at the registered ROM output. Present
                // atan(1)'s address now so it is ready for the next rotation.
                atan_address <= 7'd1;
                state <= TR_CORDIC;
            end

            TR_CORDIC: begin
                if (atan2_r) begin
                    if (!cordic_y[82]) begin
                        cordic_x <= cordic_x + cordic_y_shift;
                        cordic_y <= cordic_y - cordic_x_shift;
                        cordic_z <= cordic_z + atan_value;
                    end else begin
                        cordic_x <= cordic_x - cordic_y_shift;
                        cordic_y <= cordic_y + cordic_x_shift;
                        cordic_z <= cordic_z - atan_value;
                    end
                end else if (!cordic_z[82]) begin
                    cordic_x <= cordic_x - cordic_y_shift;
                    cordic_y <= cordic_y + cordic_x_shift;
                    cordic_z <= cordic_z - atan_value;
                end else begin
                    cordic_x <= cordic_x + cordic_y_shift;
                    cordic_y <= cordic_y - cordic_x_shift;
                    cordic_z <= cordic_z + atan_value;
                end
                cordic_iteration <= cordic_iteration + 7'd1;
                atan_address <= cordic_iteration + 7'd2;
                if (cordic_iteration == 7'd79)
                    state <= TR_SELECT;
            end

            TR_SELECT: begin
                logic signed [82:0] sine_value;
                logic signed [82:0] cosine_value;
                logic signed [82:0] selected_value;

                sine_value = 83'sh0;
                cosine_value = 83'sh0;
                if (atan2_r) begin
                    if (atan_x_sign)
                        selected_value = atan_y_sign
                                       ? cordic_z - PI_Q80
                                       : PI_Q80 - cordic_z;
                    else
                        selected_value = atan_y_sign ? -cordic_z : cordic_z;
                end else begin
                    case (quadrant)
                        2'd0: begin sine_value = cordic_y;  cosine_value = cordic_x;  end
                        2'd1: begin sine_value = cordic_x;  cosine_value = -cordic_y; end
                        2'd2: begin sine_value = -cordic_y; cosine_value = -cordic_x; end
                        default: begin sine_value = -cordic_x; cosine_value = cordic_y; end
                    endcase
                    if (operand_r.sign)
                        sine_value = -sine_value;
                    selected_value = cosine_r ? cosine_value : sine_value;
                    if (tangent_pair_r) begin
                        auxiliary_sign <= cosine_value[82];
                        auxiliary_magnitude <= cosine_value[82]
                                             ? -cosine_value : cosine_value;
                    end
                end
                result_sign <= selected_value[82];
                result_magnitude <= selected_value[82]
                                  ? -selected_value : selected_value;
                result_exp <= 17'sd16383;
                rounding_auxiliary <= 1'b0;
                inexact <= 1'b1;
                state <= TR_NORMALIZE;
            end

            TR_NORMALIZE: begin
                if (result_magnitude == 83'h0) begin
                    result <= x87_zero(result_sign);
                    state <= TR_FINISH;
                end else if (result_magnitude[82:81] != 0) begin
                    result_magnitude <= result_magnitude >> 1;
                    result_exp <= result_exp + 17'sd1;
                end else if (!result_magnitude[80]) begin
                    result_magnitude <= result_magnitude << 1;
                    result_exp <= result_exp - 17'sd1;
                end else begin
                    state <= TR_ROUND;
                end
            end

            TR_ROUND: begin
                inexact <= round_discarded;
                if (tangent_pair_r && !atan2_r && !rounding_auxiliary) begin
                    result <= rounded_result;
                    result_sign <= auxiliary_sign;
                    result_magnitude <= auxiliary_magnitude;
                    result_exp <= 17'sd16383;
                    rounding_auxiliary <= 1'b1;
                    state <= TR_NORMALIZE;
                end else begin
                    if (rounding_auxiliary)
                        auxiliary_result <= rounded_result;
                    else
                        result <= rounded_result;
                    state <= TR_FINISH;
                end
            end

            TR_FINISH: begin
                busy <= 1'b0;
                done <= 1'b1;
                state <= TR_IDLE;
            end

            default: state <= TR_IDLE;
        endcase
    end
end

endmodule
