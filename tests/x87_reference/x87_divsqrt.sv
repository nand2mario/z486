// Shared serialized divide/square-root engine. Both paths produce a 53-bit
// significand plus three rounding bits before entering the common round stage.
module x87_divsqrt
    import x87_pkg::*;
(
    input  logic       clk,
    input  logic       reset,
    input  logic       start,
    input  logic       square_root,
    input  logic [1:0] precision_control,
    input  logic [1:0] rounding_mode,
    input  x87_reg_t   operand_a,
    input  x87_reg_t   operand_b,

    output logic       busy,
    output logic       done,
    output x87_reg_t   result,
    output logic       invalid,
    output logic       divide_by_zero,
    output logic       overflow,
    output logic       underflow,
    output logic       inexact,
    output logic       denormal_operand
);

typedef enum logic [3:0] {
    DS_IDLE,
    DS_CLASSIFY,
    DS_DIV_PREPARE,
    DS_DIV_ITERATE,
    DS_SQRT_PREPARE,
    DS_SQRT_ITERATE,
    DS_ROUND,
    DS_FINISH
} divsqrt_state_t;

divsqrt_state_t state;
x87_reg_t a_r;
x87_reg_t b_r;
logic sqrt_r;
logic [1:0] precision_r;
logic [1:0] rounding_r;
logic result_sign;
logic signed [16:0] result_exp;

logic [52:0] divisor;
logic [53:0] div_remainder;
logic [55:0] result_bits;
logic [5:0] iteration_count;

logic [111:0] sqrt_radicand;
logic [57:0] sqrt_remainder;
logic [55:0] sqrt_root;

logic [53:0] div_remainder_shifted;
logic div_next_bit;
logic [57:0] sqrt_remainder_shifted;
logic [57:0] sqrt_trial;
logic sqrt_next_bit;
logic remainder_nonzero;
logic [52:0] rounded_sig;
logic rounded_carry;
logic round_discarded;
logic round_increment;
logic signed [17:0] rounded_exp;

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

function automatic logic is_finite(input x87_reg_t value);
    return (value.class_id == X87_NORMAL) ||
           (value.class_id == X87_DENORMAL);
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

always_comb begin
    logic guard_bit;
    logic sticky_bit;
    logic [24:0] rounded24;
    logic [53:0] rounded53;

    div_remainder_shifted = div_remainder << 1;
    div_next_bit = div_remainder_shifted >= {1'b0, divisor};

    sqrt_remainder_shifted = (sqrt_remainder << 2) |
                             {{56{1'b0}}, sqrt_radicand[111:110]};
    sqrt_trial = ({2'b0, sqrt_root} << 2) | 58'd1;
    sqrt_next_bit = sqrt_remainder_shifted >= sqrt_trial;

    remainder_nonzero = sqrt_r ? (sqrt_remainder != 0)
                               : (div_remainder != 0);
    rounded_sig = result_bits[55:3];
    rounded_carry = 1'b0;
    round_discarded = 1'b0;
    round_increment = 1'b0;
    guard_bit = 1'b0;
    sticky_bit = 1'b0;
    rounded24 = 25'h0;
    rounded53 = 54'h0;

    if (precision_r == 2'b00) begin
        guard_bit = result_bits[31];
        sticky_bit = |result_bits[30:0] || remainder_nonzero;
        round_discarded = guard_bit || sticky_bit;
        case (rounding_r)
            2'b00: round_increment = guard_bit &&
                                      (sticky_bit || result_bits[32]);
            2'b01: round_increment = result_sign && round_discarded;
            2'b10: round_increment = !result_sign && round_discarded;
            default: round_increment = 1'b0;
        endcase
        rounded24 = {1'b0, result_bits[55:32]} +
                    {{24{1'b0}}, round_increment};
        rounded_carry = rounded24[24];
        rounded_sig = rounded24[24]
                    ? {1'b1, 52'h0}
                    : {rounded24[23:0], 29'h0};
    end else begin
        guard_bit = result_bits[2];
        sticky_bit = result_bits[1] || result_bits[0] || remainder_nonzero;
        round_discarded = guard_bit || sticky_bit;
        case (rounding_r)
            2'b00: round_increment = guard_bit &&
                                      (sticky_bit || result_bits[3]);
            2'b01: round_increment = result_sign && round_discarded;
            2'b10: round_increment = !result_sign && round_discarded;
            default: round_increment = 1'b0;
        endcase
        rounded53 = {1'b0, result_bits[55:3]} +
                    {{53{1'b0}}, round_increment};
        rounded_carry = rounded53[53];
        rounded_sig = rounded53[53]
                    ? {1'b1, 52'h0} : rounded53[52:0];
    end

    rounded_exp = result_exp + $signed({17'h0, rounded_carry});
end

always_ff @(posedge clk) begin
    if (reset) begin
        state <= DS_IDLE;
        busy <= 1'b0;
        done <= 1'b0;
        result <= x87_empty();
        invalid <= 1'b0;
        divide_by_zero <= 1'b0;
        overflow <= 1'b0;
        underflow <= 1'b0;
        inexact <= 1'b0;
        denormal_operand <= 1'b0;
        a_r <= x87_empty();
        b_r <= x87_empty();
        sqrt_r <= 1'b0;
        precision_r <= 2'b00;
        rounding_r <= 2'b00;
        result_sign <= 1'b0;
        result_exp <= '0;
        divisor <= '0;
        div_remainder <= '0;
        result_bits <= '0;
        iteration_count <= '0;
        sqrt_radicand <= '0;
        sqrt_remainder <= '0;
        sqrt_root <= '0;
    end else begin
        done <= 1'b0;

        case (state)
            DS_IDLE: begin
                if (start) begin
                    a_r <= operand_a;
                    b_r <= operand_b;
                    sqrt_r <= square_root;
                    precision_r <= precision_control;
                    rounding_r <= rounding_mode;
                    invalid <= 1'b0;
                    divide_by_zero <= 1'b0;
                    overflow <= 1'b0;
                    underflow <= 1'b0;
                    inexact <= 1'b0;
                    denormal_operand <=
                        (operand_a.class_id == X87_DENORMAL) ||
                        (!square_root &&
                         (operand_b.class_id == X87_DENORMAL));
                    busy <= 1'b1;
                    state <= DS_CLASSIFY;
                end
            end

            DS_CLASSIFY: begin
                result_sign <= sqrt_r ? a_r.sign : (a_r.sign ^ b_r.sign);
                if (is_nan(a_r) || (!sqrt_r && is_nan(b_r))) begin
                    result <= is_nan(a_r) ? quiet_nan(a_r) : quiet_nan(b_r);
                    invalid <= is_signaling_nan(a_r) ||
                               (!sqrt_r && is_signaling_nan(b_r));
                    state <= DS_FINISH;
                end else if (sqrt_r) begin
                    if (a_r.sign && !is_zero(a_r)) begin
                        result <= x87_indefinite();
                        invalid <= 1'b1;
                        state <= DS_FINISH;
                    end else if (is_infinity(a_r) || is_zero(a_r)) begin
                        result <= a_r;
                        state <= DS_FINISH;
                    end else begin
                        state <= DS_SQRT_PREPARE;
                    end
                end else if ((is_zero(a_r) && is_zero(b_r)) ||
                             (is_infinity(a_r) && is_infinity(b_r))) begin
                    result <= x87_indefinite();
                    invalid <= 1'b1;
                    state <= DS_FINISH;
                end else if (is_zero(b_r)) begin
                    result <= '0;
                    result.sign <= a_r.sign ^ b_r.sign;
                    result.exp <= 15'h7fff;
                    result.sig <= {1'b1, 52'h0};
                    result.class_id <= X87_INFINITY;
                    divide_by_zero <= is_finite(a_r);
                    state <= DS_FINISH;
                end else if (is_infinity(a_r)) begin
                    result <= '0;
                    result.sign <= a_r.sign ^ b_r.sign;
                    result.exp <= 15'h7fff;
                    result.sig <= {1'b1, 52'h0};
                    result.class_id <= X87_INFINITY;
                    state <= DS_FINISH;
                end else if (is_zero(a_r) || is_infinity(b_r)) begin
                    result <= x87_zero(a_r.sign ^ b_r.sign);
                    state <= DS_FINISH;
                end else begin
                    state <= DS_DIV_PREPARE;
                end
            end

            DS_DIV_PREPARE: begin
                divisor <= b_r.sig;
                result_bits <= 56'h1;
                iteration_count <= 6'd55;
                result_exp <= $signed({2'b00, a_r.exp}) -
                              $signed({2'b00, b_r.exp}) + 17'sd16383;
                if (a_r.sig >= b_r.sig) begin
                    div_remainder <= {1'b0, a_r.sig} -
                                     {1'b0, b_r.sig};
                end else begin
                    div_remainder <= ({1'b0, a_r.sig} << 1) -
                                     {1'b0, b_r.sig};
                    result_exp <= $signed({2'b00, a_r.exp}) -
                                  $signed({2'b00, b_r.exp}) + 17'sd16382;
                end
                state <= DS_DIV_ITERATE;
            end

            DS_DIV_ITERATE: begin
                result_bits <= {result_bits[54:0], div_next_bit};
                div_remainder <= div_next_bit
                               ? div_remainder_shifted - {1'b0, divisor}
                               : div_remainder_shifted;
                iteration_count <= iteration_count - 6'd1;
                if (iteration_count == 6'd1)
                    state <= DS_ROUND;
            end

            DS_SQRT_PREPARE: begin
                logic [14:0] exponent_distance;

                result_sign <= a_r.sign;
                if (a_r.exp >= 15'h3fff) begin
                    exponent_distance = a_r.exp - 15'h3fff;
                    result_exp <= 17'sd16383 +
                                  $signed({2'b00, exponent_distance >> 1});
                end else begin
                    exponent_distance = 15'h3fff - a_r.exp;
                    result_exp <= 17'sd16383 -
                                  $signed({2'b00,
                                           (exponent_distance + 15'd1) >> 1});
                end
                sqrt_radicand <= a_r.exp[0]
                               ? {1'b0, a_r.sig, 58'h0}
                               : {a_r.sig, 59'h0};
                sqrt_remainder <= 58'h0;
                sqrt_root <= 56'h0;
                iteration_count <= 6'd56;
                state <= DS_SQRT_ITERATE;
            end

            DS_SQRT_ITERATE: begin
                sqrt_radicand <= sqrt_radicand << 2;
                sqrt_root <= {sqrt_root[54:0], sqrt_next_bit};
                sqrt_remainder <= sqrt_next_bit
                                ? sqrt_remainder_shifted - sqrt_trial
                                : sqrt_remainder_shifted;
                iteration_count <= iteration_count - 6'd1;
                if (iteration_count == 6'd1) begin
                    result_bits <= {sqrt_root[54:0], sqrt_next_bit};
                    state <= DS_ROUND;
                end
            end

            DS_ROUND: begin
                result <= '0;
                result.sign <= result_sign;
                inexact <= round_discarded;
                if (rounded_exp >= 18'sd32767) begin
                    overflow <= 1'b1;
                    inexact <= 1'b1;
                    if (overflow_to_infinity(result_sign, rounding_r)) begin
                        result.exp <= 15'h7fff;
                        result.sig <= {1'b1, 52'h0};
                        result.class_id <= X87_INFINITY;
                    end else begin
                        result.exp <= 15'h7ffe;
                        result.sig <= {53{1'b1}};
                        result.class_id <= X87_NORMAL;
                    end
                end else if (rounded_exp <= 0) begin
                    result <= x87_zero(result_sign);
                    underflow <= 1'b1;
                    inexact <= 1'b1;
                end else begin
                    result.exp <= rounded_exp[14:0];
                    result.sig <= rounded_sig;
                    result.class_id <= X87_NORMAL;
                end
                state <= DS_FINISH;
            end

            DS_FINISH: begin
                busy <= 1'b0;
                done <= 1'b1;
                state <= DS_IDLE;
            end

            default: state <= DS_IDLE;
        endcase
    end
end

endmodule
