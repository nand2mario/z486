`timescale 1ns/1ns

module tb_x87_executor;

import x87_pkg::*;
import x87_ucode_pkg::*;

logic clk = 1'b0;
logic reset = 1'b1;
logic start;
x87_exec_op_t exec_op;
logic [1:0] integer_size;
logic [1:0] precision_control;
logic [1:0] rounding_mode;
logic quiet_compare;
logic trans_cosine;
logic trans_atan2;
x87_reg_t operand;
x87_reg_t operand_b;
logic [63:0] transfer_in;
logic v2_busy;
logic v2_done;
logic [2:0] v2_commit;
x87_reg_t v2_result;
logic [63:0] v2_transfer;
logic v2_invalid;
logic v2_inexact;
logic v2_divide_by_zero;
logic v2_overflow;
logic v2_underflow;
logic v2_denormal_operand;
logic v2_range_incomplete;
logic v2_rounded_up;
logic v2_compare_unordered;
logic v2_compare_less;
logic v2_compare_equal;
logic [2:0] expected_v2_commit;
longint unsigned cycle_count;
longint unsigned operation_start_cycle;
longint unsigned latency_total [0:13];
integer latency_count [0:13];
integer latency_min [0:13];
integer latency_max [0:13];
x87_exec_op_t active_operation;

function automatic string operation_name(input x87_exec_op_t operation);
    case (operation)
        X87_CONVERT_FRNDINT: operation_name = "FRNDINT";
        X87_CONVERT_FLD_M32: operation_name = "FLD_M32";
        X87_CONVERT_FLD_M64: operation_name = "FLD_M64";
        X87_CONVERT_FILD: operation_name = "FILD";
        X87_CONVERT_FST_M32: operation_name = "FST_M32";
        X87_CONVERT_FST_M64: operation_name = "FST_M64";
        X87_CONVERT_FIST: operation_name = "FIST";
        X87_ARITH_ADD: operation_name = "ADD";
        X87_ARITH_SUB: operation_name = "SUB";
        X87_ARITH_COMPARE: operation_name = "COMPARE";
        X87_ARITH_MUL: operation_name = "MUL";
        X87_ARITH_DIV: operation_name = "DIV";
        X87_ARITH_SQRT: operation_name = "SQRT";
        X87_ARITH_TRANS: operation_name = "TRANS";
        default: operation_name = "UNKNOWN";
    endcase
endfunction

task automatic display_latency_summary;
    integer operation;
    begin
        $display("x87 executor latency (start-to-done cycles)");
        $display("operation count min average max");
        for (operation = 0; operation < 14; operation = operation + 1) begin
            if (latency_count[operation] != 0)
                $display("%-9s %5d %3d %7.2f %3d",
                         operation_name(x87_exec_op_t'(operation)),
                         latency_count[operation],
                         latency_min[operation],
                         $itor(latency_total[operation]) /
                             $itor(latency_count[operation]),
                         latency_max[operation]);
        end
    end
endtask

always_ff @(posedge clk) begin
    if (reset) begin
        expected_v2_commit <= X87_COMMIT_NONE;
        cycle_count <= 0;
        operation_start_cycle <= 0;
        active_operation <= X87_CONVERT_FRNDINT;
    end else if (start) begin
        cycle_count <= cycle_count + 1;
        operation_start_cycle <= cycle_count;
        active_operation <= exec_op;
        case (exec_op)
            X87_CONVERT_FLD_M32,
            X87_CONVERT_FLD_M64,
            X87_CONVERT_FILD:
                expected_v2_commit <= X87_COMMIT_PUSH;
            X87_CONVERT_FST_M32,
            X87_CONVERT_FST_M64,
            X87_CONVERT_FIST:
                expected_v2_commit <= X87_COMMIT_TRANSFER;
            X87_ARITH_COMPARE:
                expected_v2_commit <= X87_COMMIT_NONE;
            default:
                expected_v2_commit <= X87_COMMIT_REPLACE_ST0;
        endcase
    end else begin
        cycle_count <= cycle_count + 1;
    end
    if (v2_done) begin
        integer latency;
        latency = cycle_count - operation_start_cycle;
        latency_count[active_operation] <= latency_count[active_operation] + 1;
        latency_total[active_operation] <= latency_total[active_operation] + latency;
        if (latency < latency_min[active_operation])
            latency_min[active_operation] <= latency;
        if (latency > latency_max[active_operation])
            latency_max[active_operation] <= latency;
        // TRANS's commit (none for a program) is checked by test_trans_classify.
        assert ((v2_commit == expected_v2_commit) || (exec_op == X87_ARITH_TRANS))
            else $fatal(1, "commit action mismatch: got %0d expected %0d",
                        v2_commit, expected_v2_commit);
    end
end

logic v1_start;
logic v1_busy;
logic v1_done;
x87_reg_t v1_result;
logic v1_invalid;
logic v1_inexact;
logic v1_rounded_up;

logic v1_add_start;
logic v1_add_busy;
logic v1_add_done;
x87_reg_t v1_add_result;
logic v1_add_invalid;
logic v1_add_inexact;
logic v1_compare_unordered;
logic v1_compare_less;
logic v1_compare_equal;
logic v1_mul_start;
logic v1_mul_busy;
logic v1_mul_done;
x87_reg_t v1_mul_result;
logic v1_mul_invalid;
logic v1_mul_inexact;
logic v1_divsqrt_start;
logic v1_divsqrt_busy;
logic v1_divsqrt_done;
x87_reg_t v1_divsqrt_result;
logic v1_divsqrt_invalid;
logic v1_divsqrt_divide_by_zero;
logic v1_divsqrt_overflow;
logic v1_divsqrt_underflow;
logic v1_divsqrt_inexact;
logic v1_divsqrt_denormal_operand;

always #5 clk = ~clk;

x87_roundint v1_roundint (
    .clk(clk),
    .reset(reset),
    .start(v1_start),
    .rounding_mode(rounding_mode),
    .operand(operand),
    .busy(v1_busy),
    .done(v1_done),
    .result(v1_result),
    .invalid(v1_invalid),
    .inexact(v1_inexact),
    .rounded_up(v1_rounded_up)
);

x87_executor dut (
    .clk(clk),
    .reset(reset),
    .start(start),
    .exec_op(exec_op),
    .integer_size(integer_size),
    .precision_control(precision_control),
    .rounding_mode(rounding_mode),
    .quiet_compare(quiet_compare),
    .trans_cosine(trans_cosine),
    .trans_atan2(trans_atan2),
    .operand(operand),
    .operand_b(operand_b),
    .transfer_in(transfer_in),
    .busy(v2_busy),
    .done(v2_done),
    .commit_action(v2_commit),
    .result(v2_result),
    .transfer_out(v2_transfer),
    .invalid(v2_invalid),
    .inexact(v2_inexact),
    .divide_by_zero(v2_divide_by_zero),
    .overflow(v2_overflow),
    .underflow(v2_underflow),
    .denormal_operand(v2_denormal_operand),
    .range_incomplete(v2_range_incomplete),
    .rounded_up(v2_rounded_up),
    .compare_unordered(v2_compare_unordered),
    .compare_less(v2_compare_less),
    .compare_equal(v2_compare_equal)
);

x87_addsub v1_addsub (
    .clk(clk),
    .reset(reset),
    .start(v1_add_start),
    .compare_only(exec_op == X87_ARITH_COMPARE),
    .quiet_compare(quiet_compare),
    .subtract(exec_op == X87_ARITH_SUB),
    .precision_control(precision_control),
    .rounding_mode(rounding_mode),
    .operand_a(operand),
    .operand_b(operand_b),
    .busy(v1_add_busy),
    .done(v1_add_done),
    .result(v1_add_result),
    .invalid(v1_add_invalid),
    .inexact(v1_add_inexact),
    .compare_unordered(v1_compare_unordered),
    .compare_less(v1_compare_less),
    .compare_equal(v1_compare_equal)
);

x87_mul v1_multiplier (
    .clk(clk),
    .reset(reset),
    .start(v1_mul_start),
    .precision_control(precision_control),
    .rounding_mode(rounding_mode),
    .operand_a(operand),
    .operand_b(operand_b),
    .busy(v1_mul_busy),
    .done(v1_mul_done),
    .result(v1_mul_result),
    .invalid(v1_mul_invalid),
    .inexact(v1_mul_inexact)
);

x87_divsqrt v1_divide_square_root (
    .clk(clk),
    .reset(reset),
    .start(v1_divsqrt_start),
    .square_root(exec_op == X87_ARITH_SQRT),
    .precision_control(precision_control),
    .rounding_mode(rounding_mode),
    .operand_a(operand),
    .operand_b(operand_b),
    .busy(v1_divsqrt_busy),
    .done(v1_divsqrt_done),
    .result(v1_divsqrt_result),
    .invalid(v1_divsqrt_invalid),
    .divide_by_zero(v1_divsqrt_divide_by_zero),
    .overflow(v1_divsqrt_overflow),
    .underflow(v1_divsqrt_underflow),
    .inexact(v1_divsqrt_inexact),
    .denormal_operand(v1_divsqrt_denormal_operand)
);

task automatic pulse_start(input logic also_v1);
    begin
        @(negedge clk);
        start = 1'b1;
        v1_start = also_v1;
        @(negedge clk);
        start = 1'b0;
        v1_start = 1'b0;
    end
endtask

// TRANS only classifies: special and out-of-range operands commit a result,
// finite ones finish with no commit (x87_control runs the program).
task automatic test_trans_classify(
    input x87_reg_t a,
    input x87_reg_t b,
    input logic cosine,
    input logic atan2_mode,
    input logic [2:0] expected_commit,
    input x87_reg_t expected_result,
    input logic expected_invalid,
    input logic expected_inexact,
    input logic expected_range_incomplete
);
    begin
        operand = a;
        operand_b = b;
        exec_op = X87_ARITH_TRANS;
        trans_cosine = cosine;
        trans_atan2 = atan2_mode;
        precision_control = 2'd3;
        rounding_mode = 2'd0;
        pulse_start(1'b0);
        wait (v2_done);
        if ((v2_commit !== expected_commit) ||
            ((expected_commit != X87_COMMIT_NONE) &&
             ((v2_result !== expected_result) ||
              (v2_invalid !== expected_invalid) ||
              (v2_inexact !== expected_inexact) ||
              (v2_range_incomplete !== expected_range_incomplete)))) begin
            $display("trans cos/atan=%b%b a=%h b=%h", cosine, atan2_mode, a, b);
            $display("expected commit=%0d result=%h flags=%b%b%b", expected_commit,
                     expected_result, expected_invalid, expected_inexact,
                     expected_range_incomplete);
            $display("got      commit=%0d result=%h flags=%b%b%b", v2_commit,
                     v2_result, v2_invalid, v2_inexact, v2_range_incomplete);
            $fatal(1, "x87 transcendental classification mismatch");
        end
    end
endtask

task automatic test_divsqrt(
    input x87_reg_t a,
    input x87_reg_t b,
    input x87_exec_op_t operation,
    input logic [1:0] precision,
    input logic [1:0] mode
);
    x87_reg_t expected_result;
    logic expected_invalid;
    logic expected_divide_by_zero;
    logic expected_overflow;
    logic expected_underflow;
    logic expected_inexact;
    logic expected_denormal_operand;
    begin
        operand = a;
        operand_b = b;
        exec_op = operation;
        precision_control = precision;
        rounding_mode = mode;
        @(negedge clk);
        start = 1'b1;
        v1_divsqrt_start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        v1_divsqrt_start = 1'b0;
        fork
            begin
                wait (v1_divsqrt_done);
                expected_result = v1_divsqrt_result;
                expected_invalid = v1_divsqrt_invalid;
                expected_divide_by_zero = v1_divsqrt_divide_by_zero;
                expected_overflow = v1_divsqrt_overflow;
                expected_underflow = v1_divsqrt_underflow;
                expected_inexact = v1_divsqrt_inexact;
                expected_denormal_operand = v1_divsqrt_denormal_operand;
            end
            begin
                wait (v2_done);
            end
        join
        if ((v2_result !== expected_result) ||
            (v2_invalid !== expected_invalid) ||
            (v2_divide_by_zero !== expected_divide_by_zero) ||
            (v2_overflow !== expected_overflow) ||
            (v2_underflow !== expected_underflow) ||
            (v2_inexact !== expected_inexact) ||
            (v2_denormal_operand !== expected_denormal_operand)) begin
            $display("divsqrt op=%0d precision=%0d mode=%0d",
                     operation, precision, mode);
            $display("a=%h b=%h", a, b);
            $display("v1 result=%h flags=%b%b%b%b%b%b", expected_result,
                     expected_invalid, expected_divide_by_zero,
                     expected_overflow, expected_underflow,
                     expected_inexact, expected_denormal_operand);
            $display("v2 result=%h flags=%b%b%b%b%b%b", v2_result,
                     v2_invalid, v2_divide_by_zero, v2_overflow,
                     v2_underflow, v2_inexact, v2_denormal_operand);
            $fatal(1, "x87 divide/square-root mismatch");
        end
    end
endtask

task automatic test_multiply(
    input x87_reg_t a,
    input x87_reg_t b,
    input logic [1:0] precision,
    input logic [1:0] mode
);
    x87_reg_t expected_result;
    logic expected_invalid;
    logic expected_inexact;
    begin
        operand = a;
        operand_b = b;
        exec_op = X87_ARITH_MUL;
        precision_control = precision;
        rounding_mode = mode;
        @(negedge clk);
        start = 1'b1;
        v1_mul_start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        v1_mul_start = 1'b0;
        fork
            begin
                wait (v1_mul_done);
                expected_result = v1_mul_result;
                expected_invalid = v1_mul_invalid;
                expected_inexact = v1_mul_inexact;
            end
            begin
                wait (v2_done);
            end
        join
        if ((v2_result !== expected_result) ||
            (v2_invalid !== expected_invalid) ||
            (v2_inexact !== expected_inexact)) begin
            $display("multiply precision=%0d mode=%0d", precision, mode);
            $display("a=%h b=%h", a, b);
            $display("v1 result=%h flags=%b%b", expected_result,
                     expected_invalid, expected_inexact);
            $display("v2 result=%h flags=%b%b", v2_result,
                     v2_invalid, v2_inexact);
            $fatal(1, "x87 multiply mismatch");
        end
    end
endtask

task automatic test_arithmetic(
    input x87_reg_t a,
    input x87_reg_t b,
    input x87_exec_op_t operation,
    input logic [1:0] precision,
    input logic [1:0] mode,
    input logic quiet
);
    x87_reg_t expected_result;
    logic expected_invalid;
    logic expected_inexact;
    logic expected_unordered;
    logic expected_less;
    logic expected_equal;
    begin
        operand = a;
        operand_b = b;
        exec_op = operation;
        precision_control = precision;
        rounding_mode = mode;
        quiet_compare = quiet;
        @(negedge clk);
        start = 1'b1;
        v1_add_start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        v1_add_start = 1'b0;
        fork
            begin
                wait (v1_add_done);
                expected_result = v1_add_result;
                expected_invalid = v1_add_invalid;
                expected_inexact = v1_add_inexact;
                expected_unordered = v1_compare_unordered;
                expected_less = v1_compare_less;
                expected_equal = v1_compare_equal;
            end
            begin
                wait (v2_done);
            end
        join
        if (((operation != X87_ARITH_COMPARE) &&
             (v2_result !== expected_result)) ||
            (v2_invalid !== expected_invalid) ||
            (v2_inexact !== expected_inexact) ||
            (v2_compare_unordered !== expected_unordered) ||
            (v2_compare_less !== expected_less) ||
            (v2_compare_equal !== expected_equal)) begin
            $display("arithmetic op=%0d precision=%0d mode=%0d", operation,
                     precision, mode);
            $display("a=%h b=%h", a, b);
            $display("v1 result=%h flags=%b%b/%b%b%b", expected_result,
                     expected_invalid, expected_inexact, expected_unordered,
                     expected_less, expected_equal);
            $display("v2 result=%h flags=%b%b/%b%b%b", v2_result,
                     v2_invalid, v2_inexact, v2_compare_unordered,
                     v2_compare_less, v2_compare_equal);
            $fatal(1, "x87 add/subtract/compare mismatch");
        end
    end
endtask

task automatic test_round(input logic [63:0] raw, input logic [1:0] mode);
    x87_reg_t expected_result;
    logic expected_invalid;
    logic expected_inexact;
    logic expected_rounded_up;
    begin
        operand = x87_from_m64(raw);
        rounding_mode = mode;
        exec_op = X87_CONVERT_FRNDINT;
        pulse_start(1'b1);
        fork
            begin
                wait (v1_done);
                expected_result = v1_result;
                expected_invalid = v1_invalid;
                expected_inexact = v1_inexact;
                expected_rounded_up = v1_rounded_up;
            end
            begin
                wait (v2_done);
            end
        join
        if ((v2_result !== expected_result) ||
            (v2_invalid !== expected_invalid) ||
            (v2_inexact !== expected_inexact) ||
            (v2_rounded_up !== expected_rounded_up)) begin
            $display("FRND raw=%016x mode=%0d", raw, mode);
            $display("v1 result=%h flags=%b%b%b", expected_result,
                     expected_invalid, expected_inexact, expected_rounded_up);
            $display("v2 result=%h flags=%b%b%b", v2_result,
                     v2_invalid, v2_inexact, v2_rounded_up);
            $fatal(1, "x87 FRNDINT mismatch");
        end
    end
endtask

task automatic test_load(
    input x87_exec_op_t operation,
    input logic [63:0] raw,
    input logic [1:0] size,
    input x87_reg_t expected
);
    begin
        exec_op = operation;
        transfer_in = raw;
        integer_size = size;
        pulse_start(1'b0);
        wait (v2_done);
        if (v2_result !== expected)
            $fatal(1, "load conversion op=%0d raw=%016x got=%h expected=%h",
                   operation, raw, v2_result, expected);
    end
endtask

task automatic test_store(
    input x87_exec_op_t operation,
    input x87_reg_t value,
    input logic [1:0] size,
    input logic [63:0] expected
);
    begin
        exec_op = operation;
        operand = value;
        integer_size = size;
        pulse_start(1'b0);
        wait (v2_done);
        if (v2_transfer !== expected)
            $fatal(1, "store conversion op=%0d got=%016x expected=%016x",
                   operation, v2_transfer, expected);
    end
endtask

task automatic test_integer_store(
    input x87_reg_t value,
    input logic [1:0] size,
    input logic [63:0] expected,
    input logic expected_invalid
);
    begin
        exec_op = X87_CONVERT_FIST;
        operand = value;
        integer_size = size;
        pulse_start(1'b0);
        wait (v2_done);
        if ((v2_transfer !== expected) ||
            (v2_invalid !== expected_invalid) || v2_inexact)
            $fatal(1,
                   "integer store size=%0d got=%016x flags=%b%b expected=%016x/%b0",
                   size, v2_transfer, v2_invalid, v2_inexact,
                   expected, expected_invalid);
    end
endtask

task automatic test_integer_round(
    input logic [63:0] raw,
    input logic [1:0] mode,
    input logic [63:0] expected
);
    begin
        rounding_mode = mode;
        test_store(X87_CONVERT_FIST, x87_from_m64(raw), 2'd2, expected);
        if (v2_invalid || !v2_inexact)
            $fatal(1, "integer round raw=%016x mode=%0d flags=%b%b",
                   raw, mode, v2_invalid, v2_inexact);
    end
endtask

initial begin
    logic [63:0] round_cases [0:11];
    logic [31:0] m32_cases [0:7];
    logic [63:0] m64_cases [0:7];
    logic signed [63:0] integer_cases [0:7];
    logic [63:0] arith_a [0:9];
    logic [63:0] arith_b [0:9];
    integer case_index;
    integer mode;
    integer operation;
    logic signed [63:0] expected_integer;
    x87_reg_t value;

    start = 1'b0;
    v1_start = 1'b0;
    v1_add_start = 1'b0;
    v1_mul_start = 1'b0;
    v1_divsqrt_start = 1'b0;
    exec_op = X87_CONVERT_FRNDINT;
    integer_size = 2'd2;
    precision_control = 2'd2;
    rounding_mode = 2'd0;
    quiet_compare = 1'b0;
    trans_cosine = 1'b0;
    trans_atan2 = 1'b0;
    operand = x87_empty();
    operand_b = x87_empty();
    transfer_in = 64'h0;
    cycle_count = 0;
    operation_start_cycle = 0;
    active_operation = X87_CONVERT_FRNDINT;
    for (operation = 0; operation < 14; operation = operation + 1) begin
        latency_total[operation] = 0;
        latency_count[operation] = 0;
        latency_min[operation] = 32'h7fff_ffff;
        latency_max[operation] = 0;
    end

    round_cases[0] = 64'h0000_0000_0000_0000; // +0
    round_cases[1] = 64'h8000_0000_0000_0000; // -0
    round_cases[2] = 64'h3fd0_0000_0000_0000; // 0.25
    round_cases[3] = 64'h3fe0_0000_0000_0000; // 0.5
    round_cases[4] = 64'h3fe8_0000_0000_0000; // 0.75
    round_cases[5] = 64'h3ff8_0000_0000_0000; // 1.5
    round_cases[6] = 64'h4004_0000_0000_0000; // 2.5
    round_cases[7] = 64'h400c_0000_0000_0000; // 3.5
    round_cases[8] = 64'hbff8_0000_0000_0000; // -1.5
    round_cases[9] = 64'h4330_0000_0000_0000; // 2^52
    round_cases[10] = 64'h7ff0_0000_0000_0000; // infinity
    round_cases[11] = 64'h7ff8_1234_5678_9abc; // quiet NaN

    m32_cases[0] = 32'h0000_0000; // +0
    m32_cases[1] = 32'h8000_0000; // -0
    m32_cases[2] = 32'h0000_0001; // minimum denormal
    m32_cases[3] = 32'h007f_ffff; // maximum denormal
    m32_cases[4] = 32'h0080_0000; // minimum normal
    m32_cases[5] = 32'h7f80_0000; // +infinity
    m32_cases[6] = 32'hff80_0000; // -infinity
    m32_cases[7] = 32'h7fc1_2345; // quiet NaN

    m64_cases[0] = 64'h0000_0000_0000_0000; // +0
    m64_cases[1] = 64'h8000_0000_0000_0000; // -0
    m64_cases[2] = 64'h0000_0000_0000_0001; // minimum denormal
    m64_cases[3] = 64'h000f_ffff_ffff_ffff; // maximum denormal
    m64_cases[4] = 64'h0010_0000_0000_0000; // minimum normal
    m64_cases[5] = 64'h7ff0_0000_0000_0000; // +infinity
    m64_cases[6] = 64'hfff0_0000_0000_0000; // -infinity
    m64_cases[7] = 64'h7ff8_1234_5678_9abc; // quiet NaN

    integer_cases[0] = 64'sd0;
    integer_cases[1] = 64'sd1;
    integer_cases[2] = -64'sd1;
    integer_cases[3] = 64'sh0010_0000_0000_0001; // 2^52 + 1
    integer_cases[4] = -64'sh0010_0000_0000_0001;
    integer_cases[5] = 64'sh7fff_ffff_ffff_ffff;
    integer_cases[6] = 64'sh8000_0000_0000_0000;
    integer_cases[7] = 64'sh4000_0000_0000_0001;

    arith_a[0] = 64'h3ff0_0000_0000_0000; // 1.0
    arith_b[0] = 64'h4000_0000_0000_0000; // 2.0
    arith_a[1] = 64'h4008_0000_0000_0000; // 3.0
    arith_b[1] = 64'h4008_0000_0000_0000; // cancellation
    arith_a[2] = 64'h3ff0_0000_0000_0000;
    arith_b[2] = 64'h3ca0_0000_0000_0000; // large exponent delta
    arith_a[3] = 64'hbff8_0000_0000_0000; // -1.5
    arith_b[3] = 64'h3fe0_0000_0000_0000; // +0.5
    arith_a[4] = 64'h0000_0000_0000_0000; // +0
    arith_b[4] = 64'h8000_0000_0000_0000; // -0
    arith_a[5] = 64'h7ff0_0000_0000_0000; // +infinity
    arith_b[5] = 64'hfff0_0000_0000_0000; // -infinity
    arith_a[6] = 64'h7ff8_1234_5678_9abc; // quiet NaN
    arith_b[6] = 64'h3ff0_0000_0000_0000;
    arith_a[7] = 64'h7ff0_0000_0000_0001; // signaling NaN
    arith_b[7] = 64'h3ff0_0000_0000_0000;
    arith_a[8] = 64'h4330_0000_0000_0001; // rounding boundary
    arith_b[8] = 64'h3ff0_0000_0000_0000;
    arith_a[9] = 64'h0010_0000_0000_0000; // minimum normal
    arith_b[9] = 64'h000f_ffff_ffff_ffff; // maximum denormal

    repeat (3) @(posedge clk);
    reset = 1'b0;

    for (mode = 0; mode < 4; mode = mode + 1)
        for (case_index = 0; case_index < 12; case_index = case_index + 1)
            test_round(round_cases[case_index], 2'(mode));

    for (case_index = 0; case_index < 8; case_index = case_index + 1) begin
        test_load(X87_CONVERT_FLD_M32, {32'h0, m32_cases[case_index]}, 2'd0,
                  x87_from_m32(m32_cases[case_index]));
        test_load(X87_CONVERT_FLD_M64, m64_cases[case_index], 2'd0,
                  x87_from_m64(m64_cases[case_index]));
        test_load(X87_CONVERT_FILD, integer_cases[case_index], 2'd2,
                  x87_from_i64(integer_cases[case_index]));
    end

    for (mode = 0; mode < 4; mode = mode + 1) begin
        for (case_index = 0; case_index < 8; case_index = case_index + 1) begin
            value = x87_from_m32(m32_cases[case_index]);
            test_store(X87_CONVERT_FST_M32, value, 2'd0,
                       {32'h0, x87_to_m32(value, 2'(mode))});
            value = x87_from_m64(m64_cases[case_index]);
            test_store(X87_CONVERT_FST_M64, value, 2'd0,
                       x87_to_m64(value, 2'(mode)));
        end
    end

    test_load(X87_CONVERT_FLD_M32, 64'h0000_0000_3fc0_0000, 2'd0,
              x87_from_m32(32'h3fc0_0000));
    test_load(X87_CONVERT_FLD_M32, 64'h0000_0000_0000_0001, 2'd0,
              x87_from_m32(32'h0000_0001));
    test_load(X87_CONVERT_FLD_M64, 64'hc004_0000_0000_0000, 2'd0,
              x87_from_m64(64'hc004_0000_0000_0000));
    test_load(X87_CONVERT_FILD, 64'h0000_0000_ffff_ff80, 2'd0,
              x87_from_i64(-64'sd128));
    test_load(X87_CONVERT_FILD, 64'h0000_0000_ffff_ff80, 2'd1,
              x87_from_i64(-64'sd128));
    test_load(X87_CONVERT_FILD, 64'hffff_ffff_ffff_ff80, 2'd2,
              x87_from_i64(-64'sd128));
    test_load(X87_CONVERT_FILD, 64'h0000_0000_0000_7fff, 2'd0,
              x87_from_i64(64'sd32767));
    test_load(X87_CONVERT_FILD, 64'h0000_0000_0000_8000, 2'd0,
              x87_from_i64(-64'sd32768));
    test_load(X87_CONVERT_FILD, 64'h0000_0000_7abc_def1, 2'd1,
              x87_from_i64(64'sh7abc_def1));
    test_load(X87_CONVERT_FILD, 64'h0000_0000_8000_0000, 2'd1,
              x87_from_i64(-64'sd2147483648));

    rounding_mode = 2'd0;
    value = x87_from_m64(64'h4009_21fb_5444_2d18);
    test_store(X87_CONVERT_FST_M32, value, 2'd0,
               {32'h0, x87_to_m32(value, 2'd0)});
    test_store(X87_CONVERT_FST_M64, value, 2'd0,
               x87_to_m64(value, 2'd0));
    value = x87_from_m64(64'h7fefffff_ffffffff); // m32 overflow
    for (mode = 0; mode < 4; mode = mode + 1) begin
        rounding_mode = 2'(mode);
        test_store(X87_CONVERT_FST_M32, value, 2'd0,
                   {32'h0, x87_to_m32(value, 2'(mode))});
    end
    value = x87_from_m64(64'h36a0_0000_0000_0001); // m32 underflow
    for (mode = 0; mode < 4; mode = mode + 1) begin
        rounding_mode = 2'(mode);
        test_store(X87_CONVERT_FST_M32, value, 2'd0,
                   {32'h0, x87_to_m32(value, 2'(mode))});
    end
    rounding_mode = 2'd0;
    value = x87_from_m64(64'hc029_0000_0000_0000); // -12.5
    expected_integer = x87_to_i64(value, 2'd0);
    test_store(X87_CONVERT_FIST, value, 2'd0,
               {48'h0, expected_integer[15:0]});
    test_store(X87_CONVERT_FIST, value, 2'd1,
               {32'h0, expected_integer[31:0]});
    test_store(X87_CONVERT_FIST, value, 2'd2,
               expected_integer);

    // Signed integer boundaries use the architectural indefinite value on
    // positive overflow, while the corresponding negative minima are valid.
    test_integer_store(x87_from_m64(64'h40e0_0000_0000_0000), 2'd0,
                       64'h0000_0000_0000_8000, 1'b1); // +2^15
    test_integer_store(x87_from_i64(-64'sd32768), 2'd0,
                       64'h0000_0000_0000_8000, 1'b0);
    test_integer_store(x87_from_m64(64'h41e0_0000_0000_0000), 2'd1,
                       64'h0000_0000_8000_0000, 1'b1); // +2^31
    test_integer_store(x87_from_m64(64'h41e0_0003_fffb_ffff), 2'd1,
                       64'h0000_0000_8000_0000, 1'b1); // fractional overflow
    test_integer_store(x87_from_i64(-64'sd2147483648), 2'd1,
                       64'h0000_0000_8000_0000, 1'b0);
    test_integer_store(x87_from_m64(64'h43e0_0000_0000_0000), 2'd2,
                       64'h8000_0000_0000_0000, 1'b1); // +2^63
    test_integer_store(x87_from_i64(64'sh8000_0000_0000_0000), 2'd2,
                       64'h8000_0000_0000_0000, 1'b0);

    // Fractional magnitudes exercise nearest-even and all three directed
    // modes after the significand has passed through the shared shift/GRS lane.
    test_integer_round(64'h3fe0_0000_0000_0000, 2'd0, 64'd0); // +0.5
    test_integer_round(64'h3fe8_0000_0000_0000, 2'd0, 64'd1); // +0.75
    test_integer_round(64'hbfe8_0000_0000_0000, 2'd0, -64'sd1); // -0.75
    test_integer_round(64'h3fd0_0000_0000_0000, 2'd1, 64'd0); // +0.25 down
    test_integer_round(64'hbfd0_0000_0000_0000, 2'd1, -64'sd1); // -0.25 down
    test_integer_round(64'h3fd0_0000_0000_0000, 2'd2, 64'd1); // +0.25 up
    test_integer_round(64'hbfd0_0000_0000_0000, 2'd2, 64'd0); // -0.25 up
    test_integer_round(64'h3fe8_0000_0000_0000, 2'd3, 64'd0); // +0.75 truncate
    test_integer_round(64'hbfe8_0000_0000_0000, 2'd3, 64'd0); // -0.75 truncate

    for (mode = 0; mode < 4; mode = mode + 1) begin
        for (case_index = 0; case_index < 10; case_index = case_index + 1) begin
            test_arithmetic(x87_from_m64(arith_a[case_index]),
                            x87_from_m64(arith_b[case_index]),
                            X87_ARITH_ADD, 2'd0, 2'(mode), 1'b0);
            test_arithmetic(x87_from_m64(arith_a[case_index]),
                            x87_from_m64(arith_b[case_index]),
                            X87_ARITH_SUB, 2'd2, 2'(mode), 1'b0);
            test_multiply(x87_from_m64(arith_a[case_index]),
                          x87_from_m64(arith_b[case_index]),
                          2'd0, 2'(mode));
            test_multiply(x87_from_m64(arith_a[case_index]),
                          x87_from_m64(arith_b[case_index]),
                          2'd2, 2'(mode));
            test_divsqrt(x87_from_m64(arith_a[case_index]),
                         x87_from_m64(arith_b[case_index]),
                         X87_ARITH_DIV, 2'd0, 2'(mode));
            test_divsqrt(x87_from_m64(arith_a[case_index]), x87_empty(),
                         X87_ARITH_SQRT, 2'd2, 2'(mode));
        end
    end
    test_divsqrt(x87_one(), x87_zero(1'b0), X87_ARITH_DIV,
                 2'd2, 2'd0);
    begin
        x87_reg_t pio2;
        x87_reg_t pi3o4;
        pio2 = x87_from_m64(64'h3ff9_21fb_5444_2d18);
        pi3o4 = x87_from_m64(64'h4002_d97c_7f33_21d2);
        test_trans_classify(x87_from_m64(64'h3fe0_0000_0000_0000), x87_empty(),
                            1'b0, 1'b0, X87_COMMIT_NONE, x87_empty(),
                            1'b0, 1'b0, 1'b0);                    // sin(0.5)
        test_trans_classify(x87_from_m64(64'h3fe0_0000_0000_0000),
                            x87_from_m64(64'hc000_0000_0000_0000),
                            1'b0, 1'b1, X87_COMMIT_NONE, x87_empty(),
                            1'b0, 1'b0, 1'b0);                    // atan2(0.5, -2)
        test_trans_classify(x87_zero(1'b1), x87_empty(), 1'b0, 1'b0,
                            X87_COMMIT_REPLACE_ST0, x87_zero(1'b1),
                            1'b0, 1'b0, 1'b0);                    // sin(-0)
        test_trans_classify(x87_zero(1'b0), x87_empty(), 1'b1, 1'b0,
                            X87_COMMIT_REPLACE_ST0, x87_one(),
                            1'b0, 1'b0, 1'b0);                    // cos(0)
        test_trans_classify(x87_from_m64(64'h3e10_0000_0000_0000), x87_empty(),
                            1'b0, 1'b0, X87_COMMIT_REPLACE_ST0,
                            x87_from_m64(64'h3e10_0000_0000_0000),
                            1'b0, 1'b1, 1'b0);                    // sin(2^-30)
        test_trans_classify(x87_from_m64(64'h43e0_0000_0000_0000), x87_empty(),
                            1'b0, 1'b0, X87_COMMIT_REPLACE_ST0,
                            x87_from_m64(64'h43e0_0000_0000_0000),
                            1'b0, 1'b0, 1'b1);                    // sin(2^63)
        test_trans_classify(x87_from_m64(64'h7ff8_1234_5678_9abc), x87_empty(),
                            1'b0, 1'b0, X87_COMMIT_REPLACE_ST0,
                            x87_from_m64(64'h7ff8_1234_5678_9abc),
                            1'b0, 1'b0, 1'b0);                    // sin(QNaN)
        test_trans_classify(x87_zero(1'b1), x87_one(), 1'b0, 1'b1,
                            X87_COMMIT_REPLACE_ST0, x87_zero(1'b1),
                            1'b0, 1'b0, 1'b0);                    // atan2(-0, 1)
        test_trans_classify(x87_one(), x87_zero(1'b0), 1'b0, 1'b1,
                            X87_COMMIT_REPLACE_ST0, pio2,
                            1'b0, 1'b1, 1'b0);                    // atan2(1, +0)
        test_trans_classify(x87_from_m64(64'h7ff0_0000_0000_0000),
                            x87_from_m64(64'hfff0_0000_0000_0000), 1'b0, 1'b1,
                            X87_COMMIT_REPLACE_ST0, pi3o4,
                            1'b0, 1'b1, 1'b0);                    // atan2(inf, -inf)
    end
    for (case_index = 0; case_index < 10; case_index = case_index + 1) begin
        test_arithmetic(x87_from_m64(arith_a[case_index]),
                        x87_from_m64(arith_b[case_index]),
                        X87_ARITH_COMPARE, 2'd2, 2'd0, 1'b0);
        test_arithmetic(x87_from_m64(arith_a[case_index]),
                        x87_from_m64(arith_b[case_index]),
                        X87_ARITH_COMPARE, 2'd2, 2'd0, 1'b1);
    end

    @(negedge clk);
    display_latency_summary();
    $display("x87 datapath PASS: conversion and arithmetic differential");
    $finish;
end

endmodule
