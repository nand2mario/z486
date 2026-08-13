// Serialized binary add/subtract and compare engine for the x87 sidecar.
// Alignment and cancellation normalization advance one bit per clock.
module x87_addsub
    import x87_pkg::*;
(
    input  logic       clk,
    input  logic       reset,
    input  logic       start,
    input  logic       compare_only,
    input  logic       quiet_compare,
    input  logic       subtract,
    input  logic [1:0] precision_control,
    input  logic [1:0] rounding_mode,
    input  x87_reg_t   operand_a,
    input  x87_reg_t   operand_b,

    output logic       busy,
    output logic       done,
    output x87_reg_t   result,
    output logic       invalid,
    output logic       inexact,
    output logic       compare_unordered,
    output logic       compare_less,
    output logic       compare_equal
);

typedef enum logic [2:0] {
    AS_IDLE,
    AS_CLASSIFY,
    AS_PREPARE,
    AS_ALIGN,
    AS_CALCULATE,
    AS_NORMALIZE,
    AS_ROUND,
    AS_FINISH
} addsub_state_t;

addsub_state_t state;
x87_reg_t a_r;
x87_reg_t b_r;
logic compare_r;
logic quiet_compare_r;
logic subtract_r;
logic [1:0] precision_r;
logic [1:0] rounding_r;

logic [55:0] large_ext;
logic [55:0] small_ext;
logic        large_sign;
logic        small_sign;
logic [14:0] result_exp;
logic [14:0] small_exp;
logic [14:0] align_count;
logic [56:0] work_ext;

logic [52:0] rounded_sig;
logic        rounded_carry;
logic        round_discarded;
logic        round_increment;

wire b_effective_sign = b_r.sign ^ subtract_r;
wire [55:0] a_extended = {a_r.sig, a_r.guard_bit,
                          a_r.round_bit, a_r.sticky_bit};
wire [55:0] b_extended = {b_r.sig, b_r.guard_bit,
                          b_r.round_bit, b_r.sticky_bit};

function automatic logic is_nan(input x87_reg_t value);
    return value.class_id == X87_NAN;
endfunction

function automatic logic is_infinity(input x87_reg_t value);
    return value.class_id == X87_INFINITY;
endfunction

function automatic logic is_zero(input x87_reg_t value);
    return value.class_id == X87_ZERO;
endfunction

function automatic logic is_signaling_nan(input x87_reg_t value);
    return is_nan(value) && !value.sig[51];
endfunction

function automatic x87_reg_t quiet_nan(input x87_reg_t value);
    x87_reg_t quieted;
    begin
        quieted = value;
        quieted.class_id = X87_NAN;
        quieted.exp = 15'h7fff;
        quieted.sig[52:51] = 2'b11;
        return quieted;
    end
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
            magnitude_relation = 2'd2;
        else if (!is_infinity(a) && is_infinity(b))
            magnitude_relation = 2'd1;
        else if (is_zero(a) && is_zero(b))
            magnitude_relation = 2'd0;
        else if (a.exp < b.exp)
            magnitude_relation = 2'd1;
        else if (a.exp > b.exp)
            magnitude_relation = 2'd2;
        else if (a_ext < b_ext)
            magnitude_relation = 2'd1;
        else if (a_ext > b_ext)
            magnitude_relation = 2'd2;
        else
            magnitude_relation = 2'd0;
    end
endfunction

// {unordered, less, equal}; greater is represented by zero.
function automatic logic [2:0] compare_relation(
    input x87_reg_t a,
    input x87_reg_t b
);
    logic [1:0] magnitude;
    begin
        if (is_nan(a) || is_nan(b) ||
            (a.class_id == X87_EMPTY) || (b.class_id == X87_EMPTY)) begin
            compare_relation = 3'b100;
        end else if (is_zero(a) && is_zero(b)) begin
            compare_relation = 3'b001;
        end else if (a.sign != b.sign) begin
            compare_relation = a.sign ? 3'b010 : 3'b000;
        end else begin
            magnitude = magnitude_relation(a, b);
            if (magnitude == 2'd0)
                compare_relation = 3'b001;
            else if (!a.sign)
                compare_relation = (magnitude == 2'd1) ? 3'b010 : 3'b000;
            else
                compare_relation = (magnitude == 2'd2) ? 3'b010 : 3'b000;
        end
    end
endfunction

// Arithmetic precision control is applied after normalization. The initial
// implementation maps reserved and 64-bit precision settings to 53 bits.
always_comb begin
    logic guard_bit;
    logic sticky_bit;
    logic [24:0] rounded24;
    logic [53:0] rounded53;

    rounded_sig = work_ext[55:3];
    rounded_carry = 1'b0;
    round_discarded = 1'b0;
    round_increment = 1'b0;
    guard_bit = 1'b0;
    sticky_bit = 1'b0;
    rounded24 = 25'h0;
    rounded53 = 54'h0;

    if (precision_r == 2'b00) begin
        guard_bit = work_ext[31];
        sticky_bit = |work_ext[30:0];
        round_discarded = guard_bit || sticky_bit;
        case (rounding_r)
            2'b00: round_increment = guard_bit &&
                                          (sticky_bit || work_ext[32]);
            2'b01: round_increment = large_sign && round_discarded;
            2'b10: round_increment = !large_sign && round_discarded;
            default: round_increment = 1'b0;
        endcase
        rounded24 = {1'b0, work_ext[55:32]} +
                    {{24{1'b0}}, round_increment};
        rounded_carry = rounded24[24];
        rounded_sig = rounded24[24]
                    ? {1'b1, 52'h0}
                    : {rounded24[23:0], 29'h0};
    end else begin
        guard_bit = work_ext[2];
        sticky_bit = |work_ext[1:0];
        round_discarded = guard_bit || sticky_bit;
        case (rounding_r)
            2'b00: round_increment = guard_bit &&
                                          (sticky_bit || work_ext[3]);
            2'b01: round_increment = large_sign && round_discarded;
            2'b10: round_increment = !large_sign && round_discarded;
            default: round_increment = 1'b0;
        endcase
        rounded53 = {1'b0, work_ext[55:3]} +
                    {{53{1'b0}}, round_increment};
        rounded_carry = rounded53[53];
        rounded_sig = rounded53[53]
                    ? {1'b1, 52'h0} : rounded53[52:0];
    end
end

always_ff @(posedge clk) begin
    if (reset) begin
        state <= AS_IDLE;
        busy <= 1'b0;
        done <= 1'b0;
        result <= x87_empty();
        invalid <= 1'b0;
        inexact <= 1'b0;
        compare_unordered <= 1'b0;
        compare_less <= 1'b0;
        compare_equal <= 1'b0;
        a_r <= x87_empty();
        b_r <= x87_empty();
        compare_r <= 1'b0;
        quiet_compare_r <= 1'b0;
        subtract_r <= 1'b0;
        precision_r <= 2'b10;
        rounding_r <= 2'b00;
        large_ext <= 56'h0;
        small_ext <= 56'h0;
        large_sign <= 1'b0;
        small_sign <= 1'b0;
        result_exp <= 15'h0;
        small_exp <= 15'h0;
        align_count <= 15'h0;
        work_ext <= 57'h0;
    end else begin
        done <= 1'b0;

        case (state)
            AS_IDLE: begin
                if (start) begin
                    a_r <= operand_a;
                    b_r <= operand_b;
                    compare_r <= compare_only;
                    quiet_compare_r <= quiet_compare;
                    subtract_r <= subtract;
                    precision_r <= precision_control;
                    rounding_r <= rounding_mode;
                    invalid <= 1'b0;
                    inexact <= 1'b0;
                    compare_unordered <= 1'b0;
                    compare_less <= 1'b0;
                    compare_equal <= 1'b0;
                    busy <= 1'b1;
                    state <= AS_CLASSIFY;
                end
            end

            AS_CLASSIFY: begin
                logic [2:0] relation;

                if (compare_r) begin
                    relation = compare_relation(a_r, b_r);
                    compare_unordered <= relation[2];
                    compare_less <= relation[1];
                    compare_equal <= relation[0];
                    invalid <= relation[2] &&
                               (!quiet_compare_r || is_signaling_nan(a_r) ||
                                is_signaling_nan(b_r));
                    state <= AS_FINISH;
                end else if (is_nan(a_r) || is_nan(b_r)) begin
                    result <= is_nan(a_r) ? quiet_nan(a_r) : quiet_nan(b_r);
                    invalid <= is_signaling_nan(a_r) || is_signaling_nan(b_r);
                    state <= AS_FINISH;
                end else if (is_infinity(a_r) && is_infinity(b_r) &&
                             (a_r.sign != b_effective_sign)) begin
                    result <= x87_indefinite();
                    invalid <= 1'b1;
                    state <= AS_FINISH;
                end else if (is_infinity(a_r)) begin
                    result <= a_r;
                    state <= AS_FINISH;
                end else if (is_infinity(b_r)) begin
                    result <= b_r;
                    result.sign <= b_effective_sign;
                    state <= AS_FINISH;
                end else if (is_zero(a_r) && is_zero(b_r)) begin
                    result <= x87_zero((a_r.sign == b_effective_sign)
                                     ? a_r.sign : (rounding_r == 2'b01));
                    state <= AS_FINISH;
                end else if (is_zero(a_r)) begin
                    result <= b_r;
                    result.sign <= b_effective_sign;
                    state <= AS_FINISH;
                end else if (is_zero(b_r)) begin
                    result <= a_r;
                    state <= AS_FINISH;
                end else begin
                    if ((a_r.exp > b_r.exp) ||
                        ((a_r.exp == b_r.exp) &&
                         (a_extended >= b_extended))) begin
                        large_ext <= a_extended;
                        small_ext <= b_extended;
                        large_sign <= a_r.sign;
                        small_sign <= b_effective_sign;
                        result_exp <= a_r.exp;
                        small_exp <= b_r.exp;
                    end else begin
                        large_ext <= b_extended;
                        small_ext <= a_extended;
                        large_sign <= b_effective_sign;
                        small_sign <= a_r.sign;
                        result_exp <= b_r.exp;
                        small_exp <= a_r.exp;
                    end
                    state <= AS_PREPARE;
                end
            end

            AS_PREPARE: begin
                if ((result_exp - small_exp) >= 15'd56) begin
                    small_ext <= (|small_ext) ? 56'h1 : 56'h0;
                    align_count <= 15'h0;
                end else begin
                    align_count <= result_exp - small_exp;
                end
                state <= AS_ALIGN;
            end

            AS_ALIGN: begin
                if (align_count != 0) begin
                    small_ext <= {1'b0, small_ext[55:2],
                                  small_ext[1] | small_ext[0]};
                    align_count <= align_count - 15'd1;
                    if (align_count == 15'd1)
                        state <= AS_CALCULATE;
                end else begin
                    state <= AS_CALCULATE;
                end
            end

            AS_CALCULATE: begin
                if (large_sign == small_sign)
                    work_ext <= {1'b0, large_ext} + {1'b0, small_ext};
                else
                    work_ext <= {1'b0, large_ext} - {1'b0, small_ext};
                state <= AS_NORMALIZE;
            end

            AS_NORMALIZE: begin
                if (work_ext == 0) begin
                    result <= x87_zero(rounding_r == 2'b01);
                    state <= AS_FINISH;
                end else if (work_ext[56]) begin
                    work_ext <= {1'b0, work_ext[56:2],
                                 work_ext[1] | work_ext[0]};
                    result_exp <= result_exp + 15'd1;
                end else if (!work_ext[55]) begin
                    work_ext <= work_ext << 1;
                    result_exp <= result_exp - 15'd1;
                end else begin
                    state <= AS_ROUND;
                end
            end

            AS_ROUND: begin
                result <= '0;
                result.sign <= large_sign;
                result.exp <= result_exp +
                              {{14{1'b0}}, rounded_carry};
                result.sig <= rounded_sig;
                result.class_id <= X87_NORMAL;
                inexact <= round_discarded;
                state <= AS_FINISH;
            end

            AS_FINISH: begin
                busy <= 1'b0;
                done <= 1'b1;
                state <= AS_IDLE;
            end

            default: state <= AS_IDLE;
        endcase
    end
end

endmodule
