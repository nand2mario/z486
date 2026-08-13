// Serialized FRNDINT engine. Fraction bits are shifted out and restored around
// a narrow increment step, avoiding a variable 53-bit mask and barrel shifter.
module x87_roundint
    import x87_pkg::*;
(
    input  logic       clk,
    input  logic       reset,
    input  logic       start,
    input  logic [1:0] rounding_mode,
    input  x87_reg_t   operand,
    output logic       busy,
    output logic       done,
    output x87_reg_t   result,
    output logic       invalid,
    output logic       inexact,
    output logic       rounded_up
);

typedef enum logic [2:0] {
    RI_IDLE,
    RI_CLASSIFY,
    RI_SHIFT_RIGHT,
    RI_ROUND,
    RI_SHIFT_LEFT,
    RI_FINISH
} roundint_state_t;

roundint_state_t state;
x87_reg_t operand_r;
logic [1:0] rounding_r;
logic [53:0] work_sig;
logic [5:0] shift_count;
logic [5:0] restore_count;
logic guard_bit;
logic sticky_bit;

function automatic logic is_nan(input x87_reg_t value);
    return value.class_id == X87_NAN;
endfunction

function automatic logic is_signaling_nan(input x87_reg_t value);
    return is_nan(value) && !value.sig[51];
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

always_ff @(posedge clk) begin
    if (reset) begin
        state <= RI_IDLE;
        operand_r <= x87_empty();
        rounding_r <= 2'b00;
        work_sig <= 54'h0;
        shift_count <= 6'd0;
        restore_count <= 6'd0;
        guard_bit <= 1'b0;
        sticky_bit <= 1'b0;
        busy <= 1'b0;
        done <= 1'b0;
        result <= x87_empty();
        invalid <= 1'b0;
        inexact <= 1'b0;
        rounded_up <= 1'b0;
    end else begin
        done <= 1'b0;

        case (state)
            RI_IDLE: begin
                if (start) begin
                    operand_r <= operand;
                    rounding_r <= rounding_mode;
                    invalid <= 1'b0;
                    inexact <= 1'b0;
                    rounded_up <= 1'b0;
                    busy <= 1'b1;
                    state <= RI_CLASSIFY;
                end
            end

            RI_CLASSIFY: begin
                logic signed [16:0] unbiased_exp;
                logic above_half;
                logic increment_to_one;

                unbiased_exp = $signed({2'b00, operand_r.exp}) - 17'sd16383;
                above_half = |operand_r.sig[51:0] || operand_r.guard_bit ||
                             operand_r.round_bit || operand_r.sticky_bit;
                increment_to_one = directed_increment(
                    operand_r.sign, rounding_r, 1'b1);
                if (rounding_r == 2'b00)
                    increment_to_one = (unbiased_exp == -17'sd1) && above_half;

                if (is_nan(operand_r)) begin
                    result <= quiet_nan(operand_r);
                    invalid <= is_signaling_nan(operand_r);
                    state <= RI_FINISH;
                end else if ((operand_r.class_id == X87_INFINITY) ||
                             (operand_r.class_id == X87_ZERO)) begin
                    result <= operand_r;
                    state <= RI_FINISH;
                end else if (unbiased_exp >= 17'sd52) begin
                    result <= operand_r;
                    state <= RI_FINISH;
                end else if (unbiased_exp < 0) begin
                    result <= increment_to_one ? x87_one()
                                               : x87_zero(operand_r.sign);
                    if (increment_to_one)
                        result.sign <= operand_r.sign;
                    inexact <= 1'b1;
                    rounded_up <= increment_to_one;
                    state <= RI_FINISH;
                end else begin
                    shift_count <= 6'(17'sd52 - unbiased_exp);
                    restore_count <= 6'(17'sd52 - unbiased_exp);
                    work_sig <= {1'b0, operand_r.sig};
                    sticky_bit <= operand_r.guard_bit || operand_r.round_bit ||
                                  operand_r.sticky_bit;
                    guard_bit <= 1'b0;
                    state <= RI_SHIFT_RIGHT;
                end
            end

            RI_SHIFT_RIGHT: begin
                work_sig <= work_sig >> 1;
                shift_count <= shift_count - 6'd1;
                if (shift_count == 6'd1) begin
                    guard_bit <= work_sig[0];
                    state <= RI_ROUND;
                end else begin
                    sticky_bit <= sticky_bit || work_sig[0];
                end
            end

            RI_ROUND: begin
                logic increment;
                logic discarded;

                discarded = guard_bit || sticky_bit;
                case (rounding_r)
                    2'b00: increment = guard_bit &&
                                            (sticky_bit || work_sig[0]);
                    2'b01: increment = operand_r.sign && discarded;
                    2'b10: increment = !operand_r.sign && discarded;
                    default: increment = 1'b0;
                endcase
                if (increment)
                    work_sig <= work_sig + 54'd1;
                inexact <= discarded;
                rounded_up <= increment;
                state <= RI_SHIFT_LEFT;
            end

            RI_SHIFT_LEFT: begin
                work_sig <= work_sig << 1;
                restore_count <= restore_count - 6'd1;
                if (restore_count == 6'd1) begin
                    result <= '0;
                    result.sign <= operand_r.sign;
                    if (work_sig[52]) begin
                        result.exp <= operand_r.exp + 15'd1;
                        result.sig <= {1'b1, 52'h0};
                    end else begin
                        result.exp <= operand_r.exp;
                        result.sig <= work_sig[51:0] << 1;
                    end
                    result.class_id <= X87_NORMAL;
                    state <= RI_FINISH;
                end
            end

            RI_FINISH: begin
                busy <= 1'b0;
                done <= 1'b1;
                state <= RI_IDLE;
            end

            default: state <= RI_IDLE;
        endcase
    end
end

endmodule
