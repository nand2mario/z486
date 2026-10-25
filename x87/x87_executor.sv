//
// x87 Executor
// Add, multiply and divide/square root datapaths with shared rounding; transcendental
// operands are classified here and computed by x87_control's programs
//
module x87_executor
    import x87_pkg::*, x87_ucode_pkg::*;
(
    input  logic                 clk,                // Core/system clock.
    input  logic                 reset,              // Synchronous active-high reset.
    input  logic                 start,              // Launch the selected operation.
    input  x87_exec_op_t         exec_op,            // Conversion/arithmetic operation.
    input  logic           [1:0] integer_size,       // Integer transfer width selector.
    input  logic           [1:0] precision_control,  // x87 PC rounding precision.
    input  logic           [1:0] rounding_mode,      // x87 RC rounding direction.
    input  logic                 quiet_compare,      // Suppress QNaN invalid exception.
    input  logic                 trans_cosine,       // Select cosine instead of sine.
    input  logic                 trans_atan2,        // Select two-operand arctangent.
    input  x87_reg_t             operand,            // Primary numeric operand.
    input  x87_reg_t             operand_b,          // Secondary numeric operand.
    input  logic          [63:0] transfer_in,        // Raw load/conversion input bits.
    output logic                 busy,               // Microsequence is active.
    output logic                 done,               // Operation completed this cycle.
    output logic           [2:0] commit_action,      // Stack/transfer retirement action.
    output x87_reg_t             result,             // Primary numeric result.
    output logic          [63:0] transfer_out,       // Raw store/conversion output bits.
    output logic                 invalid,            // Invalid-operation exception result.
    output logic                 inexact,            // Precision exception result.
    output logic                 divide_by_zero,     // Divide-by-zero exception result.
    output logic                 overflow,           // Overflow exception result.
    output logic                 underflow,          // Underflow exception result.
    output logic                 denormal_operand,   // Denormal-operand exception result.
    output logic                 range_incomplete,   // Transcendental range reduction failed.
    output logic                 rounded_up,         // C1-style rounding-direction result.
    output logic                 compare_unordered,  // Compare result is unordered.
    output logic                 compare_less,       // Primary operand is less.
    output logic                 compare_equal,      // Operands compare equal.
    output logic           [7:0] debug_uaddr         // Current x87 microcode address.
);

logic [7:0] entry;          // Numeric microprogram entry selected by exec_op.
logic [31:0] conditions;    // Registered-state predicates visible to the sequencer.
logic seq_active;           // Numeric microprogram is executing.
logic seq_exec_valid;       // Current control-store word may update executor state.
logic seq_done;             // FINISH pulse, aligned with commit_action.
logic [7:0] uaddr;          // Current numeric microcode address.
x87_uop_t uop;              // Current 64-bit horizontal control word.
x87_uop_t uop_rom;

x87_reg_t result_r;         // Provisional numeric result; never writes stack directly.
logic [63:0] transfer_r;    // Provisional integer/IEEE store payload.
logic [67:0] work_r;        // Shared significand, normalization, and rounding lane.
logic [55:0] add_small_r;   // Exponent-aligned smaller add/sub operand with sticky bit.
logic [5:0] count_r;        // Shared conversion/alignment/iteration counter.
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
logic fild_shift_four;
logic fist_shift_four;
logic addsub_shift_four;
logic work_shift_four;
// Keep the raw m80 operand's G/R/S tail until the product is rounded. Dropping
// it first biases BASIC's decimal scaling constants (VAL("5") becomes < 5).
// Binary64 operands have a zero tail and retain their previous result/latency.
logic [55:0] mul_sig_a, mul_sig_b;
logic [111:0] mul_product;  // Assembled 56-by-56 product, including operand GRS.
logic [53:0] mul_p00;
logic [55:0] mul_p01;
logic [55:0] mul_p10;
logic [57:0] mul_p11;
logic [52:0] mul_p01_core, mul_p10_core;
logic [51:0] mul_p11_core;
logic [29:0] mul_p01_tail, mul_p10_tail, mul_p11_cross;
logic [5:0] mul_p11_top;
logic [57:0] mul_p11_r;
logic [28:0] mul_limb1_r;
logic [30:0] mul_limb2_r;
logic [28:0] mul_limb1_sum;
logic [30:0] mul_limb2_sum;
logic [31:0] mul_top_sum;
logic [14:0] mul_exp_r;
logic mul_result_sign_r;
// Divide and square root are non-restoring loops on the shared adder: WORK
// holds the partial remainder P, pre-shifted for the next step (2P for divide,
// 4P + the next radicand pair for square root). After a non-negative P the
// adder subtracts the divisor or the trial root {Q, 01}, after a negative one
// it adds the divisor or {Q, 11}; the result bit (P >= 0) is the restoring
// bit and shifts into the add/subtract alignment register (idle during these
// operations). The remainder is never restored: a flag tracks whether the
// restoring remainder is zero.
logic signed [16:0] divsqrt_exp_r;
logic divsqrt_result_sign_r;
logic divsqrt_lead_r;               // Divide: the dividend's significand >= the divisor's.
logic divsqrt_active_r;             // Between prepare and round: the adder subtracts
                                    // the divisor or trial root (a register, not a
                                    // control-store decode, selects it).
logic divsqrt_neg_r;                // The last partial remainder was negative.
logic divsqrt_zero_r;               // The restoring remainder is zero.
logic divsqrt_pair_zero_r;          // The radicand pair in WORK is zero (always, for divide).
logic divsqrt_iterate;
logic divsqrt_next_bit;
logic [1:0] sqrt_next_pair;         // Radicand bits for the next remainder.
logic divsqrt_remainder_nonzero;

assign busy = seq_active;
assign done = seq_done;
assign result = result_r;
assign transfer_out = transfer_r;
assign arithmetic_op = (exec_op == X87_ARITH_ADD) ||
                       (exec_op == X87_ARITH_SUB);
assign effective_b_sign = operand_b.sign ^
                          (exec_op == X87_ARITH_SUB);
assign round_sign = ((exec_op == X87_ARITH_DIV) ||
                     (exec_op == X87_ARITH_SQRT))
                  ? divsqrt_result_sign_r
                  : (exec_op == X87_ARITH_MUL) ? mul_result_sign_r
                  : arithmetic_op ? add_result_sign_r : operand.sign;
assign addsub_normalize_more = (work_r[56:0] != 57'h0) &&
                               (work_r[56] || !work_r[55]);
assign fild_shift_four = (exec_op == X87_CONVERT_FILD) &&
                         (uop.shift_route == X87_SHIFT_LEFT) &&
                         (work_r[51:48] == 4'b0000);
assign fist_shift_four = (exec_op == X87_CONVERT_FIST) &&
                         (count_r >= 6'd4);
assign addsub_shift_four =
    (uop.engine == X87_ENGINE_ADDSUB_ALIGN) && (count_r >= 6'd4);
assign work_shift_four = fild_shift_four || fist_shift_four;
assign format_m32 = (exec_op == X87_CONVERT_FLD_M32) ||
                    (exec_op == X87_CONVERT_FST_M32);

// pi/4 (0), pi/2 (1), pi (2), 3pi/4 (3), nearest at 53 bits, for FPATAN's
// special operands.
function automatic x87_reg_t trans_pi_multiple(input logic sign, input logic [1:0] which);
    x87_reg_t value;
    value = '0;
    value.sign = sign;
    value.class_id = X87_NORMAL;
    case (which)
        2'd0: begin value.exp = 15'h3ffe; value.sig = 53'h1921fb54442d18; end
        2'd1: begin value.exp = 15'h3fff; value.sig = 53'h1921fb54442d18; end
        2'd2: begin value.exp = 15'h4000; value.sig = 53'h1921fb54442d18; end
        default: begin value.exp = 15'h4000; value.sig = 53'h12d97c7f3321d2; end
    endcase
    return value;
endfunction
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
    if (divsqrt_active_r) begin
        arithmetic_rhs = (exec_op == X87_ARITH_SQRT)
                       ? {10'h0, add_small_r, divsqrt_neg_r, 1'b1}
                       : {15'h0, operand_b.sig};
        arithmetic_subtract = !divsqrt_neg_r;
    end
end
assign arithmetic_result = arithmetic_subtract
                         ? arithmetic_lhs - arithmetic_rhs
                         : arithmetic_lhs + arithmetic_rhs;
assign rounded_work = arithmetic_result;

assign mul_sig_a = {operand.sig, operand.guard_bit, operand.round_bit, operand.sticky_bit};
assign mul_sig_b = {operand_b.sig, operand_b.guard_bit, operand_b.round_bit, operand_b.sticky_bit};
// The original 53-bit multiplier used 27x27/27x26/26x27/26x26 DSP products.
// Keep those shapes: a 29-bit upper limb otherwise spills into extra DSPs.
// Its top three bits are cheap shift/add corrections, not extra multipliers.
function automatic logic [29:0] mul_27x3(input logic [26:0] a,
                                        input logic [2:0] b);
    return ({30{b[0]}} & {3'd0, a}) +
           ({30{b[1]}} & {2'd0, a, 1'b0}) +
           ({30{b[2]}} & {1'd0, a, 2'd0});
endfunction
assign mul_p00 = mul_sig_a[26:0] * mul_sig_b[26:0];
assign mul_p01_core = mul_sig_a[26:0] * mul_sig_b[52:27];
assign mul_p10_core = mul_sig_a[52:27] * mul_sig_b[26:0];
assign mul_p11_core = mul_sig_a[52:27] * mul_sig_b[52:27];
assign mul_p01_tail = mul_27x3(mul_sig_a[26:0], mul_sig_b[55:53]);
assign mul_p10_tail = mul_27x3(mul_sig_b[26:0], mul_sig_a[55:53]);
assign mul_p11_cross = mul_27x3({1'b0, mul_sig_a[52:27]}, mul_sig_b[55:53]) +
                        mul_27x3({1'b0, mul_sig_b[52:27]}, mul_sig_a[55:53]);
assign mul_p11_top = 6'(mul_27x3({24'd0, mul_sig_a[55:53]}, mul_sig_b[55:53]));
assign mul_p01 = {3'd0, mul_p01_core} + {mul_p01_tail, 26'd0};
assign mul_p10 = {3'd0, mul_p10_core} + {mul_p10_tail, 26'd0};
assign mul_p11 = {6'd0, mul_p11_core} + {2'd0, mul_p11_cross, 26'd0} +
                  {mul_p11_top, 52'd0};
assign mul_limb1_sum = {2'b0, mul_p00[53:27]} +
                       {2'b0, mul_p01[26:0]} +
                       {2'b0, mul_p10[26:0]};
assign mul_limb2_sum = {2'b0, mul_p01[55:27]} +
                       {2'b0, mul_p10[55:27]} +
                       {4'b0, mul_p11_r[26:0]} +
                       {{29{1'b0}}, mul_limb1_r[28:27]};
assign mul_top_sum = {1'b0, mul_p11_r[57:27]} +
                     {{28{1'b0}}, mul_limb2_r[30:27]};
assign mul_product = {mul_top_sum[30:0],
                      mul_limb2_r[26:0],
                      mul_limb1_r[26:0],
                      mul_p00[26:0]};

assign divsqrt_iterate = (uop.engine == X87_ENGINE_DIV_ITERATE) ||
                         (uop.engine == X87_ENGINE_SQRT_ITERATE);
assign divsqrt_next_bit = !arithmetic_result[67];
// Square root: iteration j (count_r = 56 - j) brings in radicand pair j + 1.
// The radicand is {0, sig} for an odd biased exponent, else {sig, 0}.
always_comb begin
    logic [53:0] radicand;
    logic [5:0] pair;
    radicand = operand.exp[0] ? {1'b0, operand.sig} : {operand.sig, 1'b0};
    pair = 6'd57 - count_r;
    sqrt_next_pair = (pair < 6'd27) ? radicand[6'd53 - {pair[4:0], 1'b0} -: 2] : 2'b00;
end
assign divsqrt_remainder_nonzero = !divsqrt_zero_r;

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
    conditions[X87_COND_INTEGRAL] = integral_r;
    conditions[X87_COND_SUBUNIT] = subunit_r;
    // A divide whose first quotient bit is set has its 56 bits one step early.
    conditions[X87_COND_COUNT_MORE] =
        ((uop.engine == X87_ENGINE_DIV_ITERATE) && divsqrt_lead_r)
        ? (count_r > 6'd2) :
        (((exec_op == X87_CONVERT_FIST) && fist_shift_four) ||
           addsub_shift_four)
        ? (count_r > 6'd4) : (count_r > 6'd1);
    conditions[X87_COND_TRANSFER_READY] = 1'b1;
    conditions[X87_COND_ZERO] = zero_r;
    conditions[X87_COND_SHIFT_LEFT] = shift_left_r;
    conditions[X87_COND_COUNT_ZERO] = count_r == 6'd0;
    conditions[X87_COND_INVALID] = invalid_r;
    conditions[X87_COND_DIRECT_READY] = direct_ready_r;
    conditions[X87_COND_NORMALIZE_MORE] = !work_r[51];
    conditions[X87_COND_SHIFT_RIGHT] = shift_right_r;
    conditions[X87_COND_SHIFT_RIGHT_MORE] = |work_r[63:54];
    conditions[X87_COND_ADDSUB_NORMALIZE_MORE] =
        addsub_normalize_more;
    conditions[X87_COND_ARITH_DIRECT] =
        is_nan(operand) || is_nan(operand_b) ||
        is_infinity(operand) || is_infinity(operand_b) ||
        is_zero(operand) || is_zero(operand_b);
end

`ifdef X87_ABLATE
// Area measurement only (see x87_control): microcode fields owned by an
// ablated feature decode as HOLD, so synthesis prunes their datapath.
localparam int X87_ABLATE_MASK = `X87_ABLATE;
always_comb begin
    uop = uop_rom;
    if (X87_ABLATE_MASK[4]) begin   // Transcendentals: classification (programs run in x87_control).
        if (uop_rom.classify == X87_CLASSIFY_TRANS) uop.classify = X87_CLASSIFY_HOLD;
    end
    if (X87_ABLATE_MASK[6]) begin   // Divide and square root.
        if (((uop_rom.engine == X87_ENGINE_DIV_ITERATE) || (uop_rom.engine == X87_ENGINE_SQRT_ITERATE)))
            uop.engine = X87_ENGINE_HOLD;
        if (((uop_rom.prepare == X87_PREPARE_DIV) || (uop_rom.prepare == X87_PREPARE_SQRT))) uop.prepare = X87_PREPARE_HOLD;
        if (uop_rom.classify == X87_CLASSIFY_DIVSQRT) uop.classify = X87_CLASSIFY_HOLD;
        if (uop_rom.pack == X87_PACK_PACK_DIVSQRT) uop.pack = X87_PACK_HOLD;
        if (uop_rom.alu_route == X87_ALU_PREP_ROUND_DIVSQRT) uop.alu_route = X87_ALU_HOLD;
    end
    if (X87_ABLATE_MASK[7]) begin   // Multiply.
        if (((uop_rom.engine == X87_ENGINE_MUL_ISSUE) || (uop_rom.engine == X87_ENGINE_MUL_ACCUMULATE)))
            uop.engine = X87_ENGINE_HOLD;
        if (uop_rom.prepare == X87_PREPARE_MUL) uop.prepare = X87_PREPARE_HOLD;
        if (uop_rom.classify == X87_CLASSIFY_MUL) uop.classify = X87_CLASSIFY_HOLD;
        if (uop_rom.pack == X87_PACK_PACK_MUL) uop.pack = X87_PACK_HOLD;
        if (uop_rom.alu_route == X87_ALU_PREP_ROUND_MUL) uop.alu_route = X87_ALU_HOLD;
    end
    if (X87_ABLATE_MASK[8]) begin   // Integer load/store (FILD/FIST).
        if (((uop_rom.prepare == X87_PREPARE_LOAD_FIST) || (uop_rom.prepare == X87_PREPARE_FILD))) uop.prepare = X87_PREPARE_HOLD;
        if (((uop_rom.classify == X87_CLASSIFY_FIST) || (uop_rom.classify == X87_CLASSIFY_FIST_RANGE))) uop.classify = X87_CLASSIFY_HOLD;
        if (((uop_rom.pack == X87_PACK_PACK_FIST) || (uop_rom.pack == X87_PACK_FIST_INVALID) || (uop_rom.pack == X87_PACK_PACK_FILD)))
            uop.pack = X87_PACK_HOLD;
    end
    if (X87_ABLATE_MASK[9]) begin   // m32/m64 real load/store.
        if (((uop_rom.prepare == X87_PREPARE_FST) || (uop_rom.prepare == X87_PREPARE_FLD))) uop.prepare = X87_PREPARE_HOLD;
        if (((uop_rom.pack == X87_PACK_ROUND_PACK_FST) || (uop_rom.pack == X87_PACK_PACK_FST_SPECIAL) || (uop_rom.pack == X87_PACK_PACK_FLD_DENORMAL)))
            uop.pack = X87_PACK_HOLD;
    end
    if (X87_ABLATE_MASK[10]) begin  // FRNDINT.
        if (uop_rom.prepare == X87_PREPARE_LOAD_ROUNDINT) uop.prepare = X87_PREPARE_HOLD;
        if (uop_rom.classify == X87_CLASSIFY_ROUNDINT) uop.classify = X87_CLASSIFY_HOLD;
        if (((uop_rom.pack == X87_PACK_ROUNDINT_SPECIAL) || (uop_rom.pack == X87_PACK_ROUNDINT_SUBUNIT)))
            uop.pack = X87_PACK_HOLD;
    end
    if (X87_ABLATE_MASK[11]) begin  // Add/subtract.
        if (uop_rom.engine == X87_ENGINE_ADDSUB_ALIGN) uop.engine = X87_ENGINE_HOLD;
        if (uop_rom.prepare == X87_PREPARE_ADDSUB) uop.prepare = X87_PREPARE_HOLD;
        if (uop_rom.classify == X87_CLASSIFY_ADDSUB) uop.classify = X87_CLASSIFY_HOLD;
        if (uop_rom.pack == X87_PACK_PACK_ADDSUB) uop.pack = X87_PACK_HOLD;
        if (((uop_rom.alu_route == X87_ALU_CALCULATE_ADDSUB) || (uop_rom.alu_route == X87_ALU_NORMALIZE_ADDSUB) || (uop_rom.alu_route == X87_ALU_PREP_ROUND_ADDSUB)))
            uop.alu_route = X87_ALU_HOLD;
    end
end
`else
assign uop = uop_rom;
`endif

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
    .uop(uop_rom)
);

assign debug_uaddr = uaddr;

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

// Shared mantissa, format, and retirement bank. This owns the 68-bit work
// register and GRS bits, conversion/normalization state, final result and
// transfer registers, exception outputs, comparisons, and commit action.
always_ff @(posedge clk) begin
    if (reset) begin
        result_r <= x87_empty();
        transfer_r <= 64'h0;
        work_r <= 68'h0;
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
                commit_action <= X87_COMMIT_NONE;
        end

        if (seq_exec_valid) begin
            if (uop.flow == X87_FLOW_FINISH)
                commit_action <= uop.commit;

            // Horizontal resource controls update independently of the
            // remaining legacy operation lane. The generator guarantees that
            // no operation selected in the same word also writes this state.
            case (uop.shift_route)
                X87_SHIFT_LEFT:
                    work_r <= work_shift_four ? work_r << 4 : work_r << 1;
                X87_SHIFT_RIGHT:
                    work_r <= work_shift_four ? work_r >> 4 : work_r >> 1;
                default: ;
            endcase

            if ((uop.count == X87_COUNT_DEC) &&
                (count_r != 6'd0))
                count_r <= count_r -
                    ((fist_shift_four || addsub_shift_four) ? 6'd4 : 6'd1);

            if ((uop.grs == X87_GRS_SHIFT) &&
                (uop.shift_route == X87_SHIFT_RIGHT)) begin
                if (uop.count == X87_COUNT_DEC) begin
                    if (fist_shift_four && (count_r == 6'd4)) begin
                        guard_r <= work_r[3];
                        sticky_r <= sticky_r || |work_r[2:0];
                    end else if (fist_shift_four) begin
                        sticky_r <= sticky_r || |work_r[3:0];
                    end else if (count_r == 6'd1)
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

                    // Special and out-of-range operands commit directly; a
                    // finite operand finishes with no commit and x87_control
                    // runs its program (x87_poly_rom.svh).
                    unbiased_exp = $signed({2'b00, operand.exp}) -
                                   17'sd16383;
                    direct_ready_r <= 1'b1;

                    // FPATAN's special results (multiples of pi/4) are the
                    // nearest 53-bit values; y's sign.
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
                            inexact <= 1'b1;
                            result_r <= trans_pi_multiple(operand.sign, 2'd2);
                        end else begin
                            result_r <= x87_zero(operand.sign);
                        end
                    end else if (trans_atan2 && is_zero(operand_b)) begin
                        inexact <= 1'b1;
                        result_r <= trans_pi_multiple(operand.sign, 2'd1);
                    end else if (trans_atan2 && is_infinity(operand) &&
                                 is_infinity(operand_b)) begin
                        inexact <= 1'b1;
                        result_r <= trans_pi_multiple(operand.sign,
                                                      operand_b.sign ? 2'd3 : 2'd0);
                    end else if (trans_atan2 && is_infinity(operand)) begin
                        inexact <= 1'b1;
                        result_r <= trans_pi_multiple(operand.sign, 2'd1);
                    end else if (trans_atan2 && is_infinity(operand_b)) begin
                        if (operand_b.sign) begin
                            inexact <= 1'b1;
                            result_r <= trans_pi_multiple(operand.sign, 2'd2);
                        end else begin
                            result_r <= x87_zero(operand.sign);
                        end
                    end else if (trans_atan2) begin
                        direct_ready_r <= 1'b0;
                    end else if (is_nan(operand)) begin
                        result_r <= quiet_nan(operand);
                        invalid <= is_signaling_nan(operand);
                    end else if (is_infinity(operand) ||
                                 (operand.class_id == X87_EMPTY)) begin
                        result_r <= x87_indefinite();
                        invalid <= 1'b1;
                    end else if (is_zero(operand)) begin
                        result_r <= trans_cosine ? x87_one() : operand;
                    end else if (unbiased_exp >= 17'sd63) begin
                        result_r <= operand;
                        range_incomplete <= 1'b1;
                    end else if (unbiased_exp <= -17'sd27) begin
                        result_r <= trans_cosine ? x87_one() : operand;
                        inexact <= 1'b1;
                    end else begin
                        direct_ready_r <= 1'b0;
                    end
                end

                default: ;
            endcase

            case (uop.engine)
                X87_ENGINE_DIV_ITERATE:
                    work_r <= arithmetic_result << 1;
                X87_ENGINE_SQRT_ITERATE:
                    work_r <= {arithmetic_result[65:0], sqrt_next_pair};
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
                    end else if (!work_r[55]) begin
                        work_r <= (work_r[55:52] == 4'h0)
                                ? work_r << 4 : work_r << 1;
                    end
                end

                X87_ALU_PREP_ROUND_ADDSUB: begin
                    logic [67:0] prepared_work;
                    logic prepared_guard;
                    logic prepared_sticky;
                    logic prepared_increment;

                    if (precision_control == 2'b00) begin
                        prepared_work = {44'h0, work_r[55:32]};
                        prepared_guard = work_r[31];
                        prepared_sticky = |work_r[30:0];
                    end else begin
                        prepared_work = {15'h0, work_r[55:3]};
                        prepared_guard = work_r[2];
                        prepared_sticky = |work_r[1:0];
                    end
                    prepared_increment =
                        (rounding_mode == 2'b00)
                            ? prepared_guard &&
                              (prepared_sticky || prepared_work[0])
                            : directed_increment(
                                add_result_sign_r, rounding_mode,
                                prepared_guard || prepared_sticky);
                    work_r <= prepared_work +
                              {{67{1'b0}}, prepared_increment};
                    guard_r <= prepared_guard;
                    sticky_r <= prepared_sticky;
                    inexact <= prepared_guard || prepared_sticky;
                    rounded_up <= prepared_increment;
                end

                X87_ALU_PREP_ROUND_MUL: begin
                    logic [52:0] normalized_sig;
                    logic normalized_guard;
                    logic normalized_round;
                    logic normalized_sticky;
                    logic [67:0] prepared_work;
                    logic prepared_guard;
                    logic prepared_sticky;
                    logic prepared_increment;

                    if (mul_product[111]) begin
                        normalized_sig = mul_product[111:59];
                        normalized_guard = mul_product[58];
                        normalized_round = mul_product[57];
                        normalized_sticky = |mul_product[56:0];
                    end else begin
                        normalized_sig = mul_product[110:58];
                        normalized_guard = mul_product[57];
                        normalized_round = mul_product[56];
                        normalized_sticky = |mul_product[55:0];
                    end

                    round_r <= 1'b0;
                    if (precision_control == 2'b00) begin
                        prepared_work = {44'h0, normalized_sig[52:29]};
                        prepared_guard = normalized_sig[28];
                        prepared_sticky = |normalized_sig[27:0] ||
                                          normalized_guard ||
                                          normalized_round ||
                                          normalized_sticky;
                    end else begin
                        prepared_work = {15'h0, normalized_sig};
                        prepared_guard = normalized_guard;
                        prepared_sticky = normalized_round ||
                                          normalized_sticky;
                    end
                    prepared_increment =
                        (rounding_mode == 2'b00)
                            ? prepared_guard &&
                              (prepared_sticky || prepared_work[0])
                            : directed_increment(
                                mul_result_sign_r, rounding_mode,
                                prepared_guard || prepared_sticky);
                    work_r <= prepared_work +
                              {{67{1'b0}}, prepared_increment};
                    guard_r <= prepared_guard;
                    sticky_r <= prepared_sticky;
                    inexact <= prepared_guard || prepared_sticky;
                    rounded_up <= prepared_increment;
                end

                X87_ALU_PREP_ROUND_DIVSQRT: begin
                    round_r <= 1'b0;
                    if (precision_control == 2'b00) begin
                        work_r <= {44'h0, add_small_r[55:32]};
                        guard_r <= add_small_r[31];
                        sticky_r <= |add_small_r[30:0] ||
                                    divsqrt_remainder_nonzero;
                    end else begin
                        work_r <= {15'h0, add_small_r[55:3]};
                        guard_r <= add_small_r[2];
                        sticky_r <= |add_small_r[1:0] ||
                                    divsqrt_remainder_nonzero;
                    end
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
                    rounded_up <= round_increment;
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
                        // Align to the integer point in one step, as the
                        // 4/1-bit loop would: guard takes the last bit shifted
                        // out, sticky every bit below it. The count is then
                        // zero and the right-shift loop is skipped.
                        logic [5:0]  sc;
                        logic [67:0] aligned_in;
                        logic        lower_sticky;
                        shift_count = 17'sd52 - unbiased_r;
                        sc = (shift_count > 17'sd54) ? 6'd54 : 6'(shift_count);
                        aligned_in = {15'h0, operand.sig};
                        lower_sticky = 1'b0;
                        for (int b = 0; b < 54; b++)
                            if ((b + 1) < int'(sc) && aligned_in[b]) lower_sticky = 1'b1;
                        work_r <= aligned_in >> sc;
                        count_r <= 6'd0;
                        guard_r <= (sc != 6'd0) ? aligned_in[sc - 6'd1] : 1'b0;
                        sticky_r <= operand.guard_bit ||
                                    operand.round_bit ||
                                    operand.sticky_bit || lower_sticky;
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
                    logic [5:0]  lead;      // left shift that puts the MSB at bit 52
                    signed_input = sized_integer(transfer_in, integer_size);
                    magnitude = signed_input[63] ? (~signed_input + 64'd1)
                                                 : signed_input;
                    // An integer below 2^52 normalizes exactly by a left shift:
                    // do it here, in one step, instead of the 4/1-bit loop
                    // (the loop then has nothing to do and is skipped).
                    lead = 6'd0;
                    for (int b = 0; b <= 52; b++)
                        if (magnitude[b]) lead = 6'(52 - b);
                    result_r <= x87_zero(1'b0);
                    direct_ready_r <= signed_input == 0;
                    shift_right_r <= |magnitude[63:53];
                    shift_left_r <= 1'b0;
                    if ((magnitude != 0) && !(|magnitude[63:52])) begin
                        work_r <= {4'h0, magnitude << lead};
                        format_exp_r <= 17'sd16435 - 17'(lead);
                    end else begin
                        work_r <= {4'h0, magnitude};
                        format_exp_r <= 17'sd16435;
                    end
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
                    format_exp_r <= format_exp_r -
                                    (fild_shift_four ? 17'sd4 : 17'sd1);
                end

                X87_STATE_FORMAT_EXP_INC: begin
                    format_exp_r <= format_exp_r + 17'sd1;
                end

                default: begin end
            endcase

            case (uop.prepare)
                X87_PREPARE_ADDSUB: begin
                    logic [14:0] exponent_delta;

                    if (operand_magnitude != 2'd1) begin
                        work_r <= {12'h0, operand.sig,
                                   operand.guard_bit, operand.round_bit,
                                   operand.sticky_bit};
                        exponent_delta = operand.exp - operand_b.exp;
                    end else begin
                        work_r <= {12'h0, operand_b.sig,
                                   operand_b.guard_bit, operand_b.round_bit,
                                   operand_b.sticky_bit};
                        exponent_delta = operand_b.exp - operand.exp;
                    end
                    if (exponent_delta >= 15'd56) begin
                        count_r <= 6'd0;
                    end else begin
                        count_r <= exponent_delta[5:0];
                    end
                end

                X87_PREPARE_MUL: begin
                    count_r <= 6'd2;
                end

                X87_PREPARE_DIV: begin
                    count_r <= 6'd57;
                    work_r <= {15'h0, operand.sig};
                end

                X87_PREPARE_SQRT: begin
                    count_r <= 6'd56;
                    work_r <= {66'h0, operand.exp[0] ? {1'b0, operand.sig[52]}
                                                     : operand.sig[52:51]};
                end

                default: begin end
            endcase
        end
    end
end

// Add/subtract alignment bank. It tracks the smaller significand and the
// exponent/sign associated with the shared 68-bit mantissa work register.
always_ff @(posedge clk) begin
    if (reset) begin
        add_small_r <= 56'h0;
        add_exp_r <= 15'h0;
        add_result_sign_r <= 1'b0;
        add_small_sign_r <= 1'b0;
    end else if (seq_exec_valid) begin
        if ((uop.engine == X87_ENGINE_ADDSUB_ALIGN) &&
            (count_r != 6'd0)) begin
            if (addsub_shift_four)
                add_small_r <= {4'b0, add_small_r[55:5],
                                |add_small_r[4:0]};
            else
                add_small_r <= {1'b0, add_small_r[55:2],
                                add_small_r[1] | add_small_r[0]};
        end

        if (uop.alu_route == X87_ALU_NORMALIZE_ADDSUB) begin
            if ((work_r[56:0] != 57'h0) && work_r[56])
                add_exp_r <= add_exp_r + 15'd1;
            else if ((work_r[56:0] != 57'h0) && !work_r[55])
                add_exp_r <= add_exp_r -
                    ((work_r[55:52] == 4'h0) ? 15'd4 : 15'd1);
        end

        if (divsqrt_iterate)
            add_small_r <= {add_small_r[54:0], divsqrt_next_bit};
        if ((uop.prepare == X87_PREPARE_DIV) || (uop.prepare == X87_PREPARE_SQRT))
            add_small_r <= 56'h0;

        if (uop.prepare == X87_PREPARE_ADDSUB) begin
            logic [55:0] a_extended;
            logic [55:0] b_extended;
            logic [14:0] exponent_delta;

            a_extended = {operand.sig, operand.guard_bit,
                          operand.round_bit, operand.sticky_bit};
            b_extended = {operand_b.sig, operand_b.guard_bit,
                          operand_b.round_bit, operand_b.sticky_bit};
            if (operand_magnitude != 2'd1) begin
                add_small_r <= b_extended;
                add_result_sign_r <= operand.sign;
                add_small_sign_r <= effective_b_sign;
                add_exp_r <= operand.exp;
                exponent_delta = operand.exp - operand_b.exp;
            end else begin
                add_small_r <= a_extended;
                add_result_sign_r <= effective_b_sign;
                add_small_sign_r <= operand.sign;
                add_exp_r <= operand_b.exp;
                exponent_delta = operand_b.exp - operand.exp;
            end
            if (exponent_delta >= 15'd56)
                add_small_r <= 56'h1;
        end
    end
end

// Divide/square-root exponent and sign. The divide exponent assumes the
// quotient's leading one in its second bit (57 steps); when the first step
// finds the dividend's significand >= the divisor's, the loop ends a step
// early and the round step raises the exponent by one.
always_ff @(posedge clk) begin
    if (reset) begin
        divsqrt_exp_r <= 17'sd0;
        divsqrt_result_sign_r <= 1'b0;
        divsqrt_lead_r <= 1'b0;
        divsqrt_active_r <= 1'b0;
        divsqrt_neg_r <= 1'b0;
        divsqrt_zero_r <= 1'b0;
        divsqrt_pair_zero_r <= 1'b0;
    end else begin
        if (start)
            divsqrt_active_r <= 1'b0;
        if (seq_exec_valid) begin
            if ((uop.prepare == X87_PREPARE_DIV) ||
                (uop.prepare == X87_PREPARE_SQRT)) begin
                divsqrt_active_r <= 1'b1;
                divsqrt_neg_r <= 1'b0;
            end
            if (divsqrt_iterate) begin
                divsqrt_neg_r <= arithmetic_result[67];
                divsqrt_zero_r <= (arithmetic_result == 68'h0) ||
                                  (divsqrt_zero_r && divsqrt_pair_zero_r);
                divsqrt_pair_zero_r <= (exec_op != X87_ARITH_SQRT) ||
                                       (sqrt_next_pair == 2'b00);
            end
            if (uop.alu_route == X87_ALU_PREP_ROUND_DIVSQRT) begin
                divsqrt_active_r <= 1'b0;
                if (divsqrt_lead_r)
                    divsqrt_exp_r <= divsqrt_exp_r + 17'sd1;
            end

            if (uop.classify == X87_CLASSIFY_DIVSQRT)
                divsqrt_result_sign_r <= (exec_op == X87_ARITH_SQRT)
                                       ? operand.sign
                                       : operand.sign ^ operand_b.sign;

            if ((uop.engine == X87_ENGINE_DIV_ITERATE) && (count_r == 6'd57))
                divsqrt_lead_r <= divsqrt_next_bit;

            case (uop.prepare)
                X87_PREPARE_DIV: begin
                    divsqrt_lead_r <= 1'b0;
                    divsqrt_zero_r <= 1'b0;
                    divsqrt_pair_zero_r <= 1'b1;
                    divsqrt_exp_r <=
                        $signed({2'b00, operand.exp}) -
                        $signed({2'b00, operand_b.exp}) + 17'sd16382;
                end

                X87_PREPARE_SQRT: begin
                    logic [14:0] exponent_distance;

                    divsqrt_lead_r <= 1'b0;
                    divsqrt_zero_r <= 1'b1;
                    divsqrt_pair_zero_r <= operand.exp[0]
                        ? !operand.sig[52] : (operand.sig[52:51] == 2'b00);
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
                end
                default: ;
            endcase
        end
    end
end

// Capture the corrected high product in the existing prepare step: it is
// first consumed by the following accumulate step, so no cycle is added.
always_ff @(posedge clk) begin
    if (reset) begin
        mul_p11_r <= 58'd0;
        mul_limb1_r <= 29'h0;
        mul_limb2_r <= 31'h0;
        mul_exp_r <= 15'h0;
        mul_result_sign_r <= 1'b0;
    end else if (seq_exec_valid) begin
        if (uop.classify == X87_CLASSIFY_MUL)
            mul_result_sign_r <= operand.sign ^ operand_b.sign;

        case (uop.engine)
            X87_ENGINE_MUL_ISSUE: ;

            X87_ENGINE_MUL_ACCUMULATE: begin
                mul_limb2_r <= mul_limb2_sum;
            end
            default: ;
        endcase

        if ((uop.alu_route == X87_ALU_PREP_ROUND_MUL) && mul_product[111])
            mul_exp_r <= mul_exp_r + 15'd1;

        if (uop.prepare == X87_PREPARE_MUL) begin
            mul_p11_r <= mul_p11;
            mul_exp_r <= operand.exp + operand_b.exp - 15'h3fff;
            mul_limb1_r <= mul_limb1_sum;
        end
    end
end

// synthesis translate_off
always @(posedge clk)
    if (!reset && seq_exec_valid && (uop.alu_route == X87_ALU_PREP_ROUND_MUL)) begin
        if (mul_product !== (112'(mul_sig_a) * 112'(mul_sig_b)))
            $fatal(1, "x87 partial-product reconstruction mismatch");
    end
// synthesis translate_on

endmodule
