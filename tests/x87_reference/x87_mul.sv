// Serialized 53-bit significand multiplier. Four 27-by-27 partial products
// share one DSP-friendly multiply and a separately registered accumulator.
module x87_mul
    import x87_pkg::*;
(
    input  logic        clk,
    input  logic        reset,
    input  logic        start,
    input  logic [1:0]  precision_control,
    input  logic [1:0]  rounding_mode,
    input  x87_reg_t    operand_a,
    input  x87_reg_t    operand_b,
    output logic        busy,
    output logic        done,
    output x87_reg_t    result,
    output logic        invalid,
    output logic        inexact
);

typedef enum logic [3:0] {
    MUL_IDLE,
    MUL_CLASSIFY,
    MUL_ISSUE,
    MUL_ACCUMULATE,
    MUL_NORMALIZE,
    MUL_ROUND,
    MUL_FINISH
} mul_state_t;

mul_state_t state;
x87_reg_t a_r;
x87_reg_t b_r;
logic [1:0] precision_r;
logic [1:0] rounding_r;
logic result_sign;
logic [14:0] result_exp;
logic [1:0] partial_index;
logic [26:0] mul_a;
logic [26:0] mul_b;
logic [53:0] mul_product;
logic [105:0] product_accum;
logic [52:0] normalized_sig;
logic normalized_guard;
logic normalized_round;
logic normalized_sticky;
logic [52:0] rounded_sig;
logic rounded_carry;
logic round_discarded;
logic round_increment;

function automatic logic is_nan(input x87_reg_t value);
    return value.class_id == X87_NAN;
endfunction

function automatic logic is_signaling_nan(input x87_reg_t value);
    return is_nan(value) && !value.sig[51];
endfunction

function automatic logic is_infinity(input x87_reg_t value);
    return value.class_id == X87_INFINITY;
endfunction

function automatic logic is_zero(input x87_reg_t value);
    return value.class_id == X87_ZERO;
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

always_comb begin
    logic guard_bit;
    logic sticky_bit;
    logic [24:0] rounded24;
    logic [53:0] rounded53;

    if (product_accum[105]) begin
        normalized_sig = product_accum[105:53];
        normalized_guard = product_accum[52];
        normalized_round = product_accum[51];
        normalized_sticky = |product_accum[50:0];
    end else begin
        normalized_sig = product_accum[104:52];
        normalized_guard = product_accum[51];
        normalized_round = product_accum[50];
        normalized_sticky = |product_accum[49:0];
    end

    rounded_sig = normalized_sig;
    rounded_carry = 1'b0;
    round_discarded = 1'b0;
    round_increment = 1'b0;
    guard_bit = 1'b0;
    sticky_bit = 1'b0;
    rounded24 = 25'h0;
    rounded53 = 54'h0;

    if (precision_r == 2'b00) begin
        guard_bit = normalized_sig[28];
        sticky_bit = |normalized_sig[27:0] || normalized_guard ||
                     normalized_round || normalized_sticky;
        round_discarded = guard_bit || sticky_bit;
        case (rounding_r)
            2'b00: round_increment = guard_bit &&
                                      (sticky_bit || normalized_sig[29]);
            2'b01: round_increment = result_sign && round_discarded;
            2'b10: round_increment = !result_sign && round_discarded;
            default: round_increment = 1'b0;
        endcase
        rounded24 = {1'b0, normalized_sig[52:29]} +
                    {{24{1'b0}}, round_increment};
        rounded_carry = rounded24[24];
        rounded_sig = rounded24[24]
                    ? {1'b1, 52'h0}
                    : {rounded24[23:0], 29'h0};
    end else begin
        guard_bit = normalized_guard;
        sticky_bit = normalized_round || normalized_sticky;
        round_discarded = guard_bit || sticky_bit;
        case (rounding_r)
            2'b00: round_increment = guard_bit &&
                                      (sticky_bit || normalized_sig[0]);
            2'b01: round_increment = result_sign && round_discarded;
            2'b10: round_increment = !result_sign && round_discarded;
            default: round_increment = 1'b0;
        endcase
        rounded53 = {1'b0, normalized_sig} +
                    {{53{1'b0}}, round_increment};
        rounded_carry = rounded53[53];
        rounded_sig = rounded53[53]
                    ? {1'b1, 52'h0} : rounded53[52:0];
    end
end

always_ff @(posedge clk) begin
    if (reset) begin
        state <= MUL_IDLE;
        busy <= 1'b0;
        done <= 1'b0;
        result <= x87_empty();
        invalid <= 1'b0;
        inexact <= 1'b0;
        a_r <= x87_empty();
        b_r <= x87_empty();
        precision_r <= 2'b00;
        rounding_r <= 2'b00;
        result_sign <= 1'b0;
        result_exp <= 15'h0;
        partial_index <= 2'd0;
        mul_a <= 27'h0;
        mul_b <= 27'h0;
        mul_product <= 54'h0;
        product_accum <= 106'h0;
    end else begin
        done <= 1'b0;

        case (state)
            MUL_IDLE: begin
                if (start) begin
                    a_r <= operand_a;
                    b_r <= operand_b;
                    precision_r <= precision_control;
                    rounding_r <= rounding_mode;
                    invalid <= 1'b0;
                    inexact <= 1'b0;
                    busy <= 1'b1;
                    state <= MUL_CLASSIFY;
                end
            end

            MUL_CLASSIFY: begin
                result_sign <= a_r.sign ^ b_r.sign;
                if (is_nan(a_r) || is_nan(b_r)) begin
                    result <= is_nan(a_r) ? quiet_nan(a_r) : quiet_nan(b_r);
                    invalid <= is_signaling_nan(a_r) || is_signaling_nan(b_r);
                    state <= MUL_FINISH;
                end else if ((is_infinity(a_r) && is_zero(b_r)) ||
                             (is_zero(a_r) && is_infinity(b_r))) begin
                    result <= x87_indefinite();
                    invalid <= 1'b1;
                    state <= MUL_FINISH;
                end else if (is_infinity(a_r) || is_infinity(b_r)) begin
                    result <= '0;
                    result.sign <= a_r.sign ^ b_r.sign;
                    result.exp <= 15'h7fff;
                    result.sig <= {1'b1, 52'h0};
                    result.class_id <= X87_INFINITY;
                    state <= MUL_FINISH;
                end else if (is_zero(a_r) || is_zero(b_r)) begin
                    result <= x87_zero(a_r.sign ^ b_r.sign);
                    state <= MUL_FINISH;
                end else begin
                    result_exp <= a_r.exp + b_r.exp - 15'h3fff;
                    product_accum <= 106'h0;
                    partial_index <= 2'd0;
                    mul_a <= a_r.sig[26:0];
                    mul_b <= b_r.sig[26:0];
                    state <= MUL_ISSUE;
                end
            end

            MUL_ISSUE: begin
                mul_product <= mul_a * mul_b;
                state <= MUL_ACCUMULATE;
            end

            MUL_ACCUMULATE: begin
                case (partial_index)
                    2'd0: product_accum <= product_accum +
                                             {{52{1'b0}}, mul_product};
                    2'd1, 2'd2: product_accum <= product_accum +
                                             {{25{1'b0}}, mul_product,
                                               {27{1'b0}}};
                    default: product_accum <= product_accum +
                                             {mul_product[51:0], {54{1'b0}}};
                endcase

                if (partial_index == 2'd3) begin
                    state <= MUL_NORMALIZE;
                end else begin
                    partial_index <= partial_index + 2'd1;
                    case (partial_index)
                        2'd0: begin
                            mul_a <= {1'b0, a_r.sig[52:27]};
                            mul_b <= b_r.sig[26:0];
                        end
                        2'd1: begin
                            mul_a <= a_r.sig[26:0];
                            mul_b <= {1'b0, b_r.sig[52:27]};
                        end
                        default: begin
                            mul_a <= {1'b0, a_r.sig[52:27]};
                            mul_b <= {1'b0, b_r.sig[52:27]};
                        end
                    endcase
                    state <= MUL_ISSUE;
                end
            end

            MUL_NORMALIZE: begin
                if (product_accum[105])
                    result_exp <= result_exp + 15'd1;
                state <= MUL_ROUND;
            end

            MUL_ROUND: begin
                result <= '0;
                result.sign <= result_sign;
                result.exp <= result_exp + {{14{1'b0}}, rounded_carry};
                result.sig <= rounded_sig;
                result.class_id <= X87_NORMAL;
                inexact <= round_discarded;
                state <= MUL_FINISH;
            end

            MUL_FINISH: begin
                busy <= 1'b0;
                done <= 1'b1;
                state <= MUL_IDLE;
            end

            default: state <= MUL_IDLE;
        endcase
    end
end

endmodule
