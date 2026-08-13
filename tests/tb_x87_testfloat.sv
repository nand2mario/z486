`timescale 1ns/1ns

// Independent arithmetic check using binary64 vectors from Berkeley
// TestFloat. Binary64 maps directly to the implemented 53-bit x87 precision.
module tb_x87_testfloat;

import x87_pkg::*;
import x87_ucode_pkg::*;

logic clk = 1'b0;
logic reset = 1'b1;
logic start = 1'b0;
x87_exec_op_t exec_op;
logic [1:0] integer_size;
logic [1:0] rounding_mode;
x87_reg_t operand;
x87_reg_t operand_b;
logic busy;
logic done;
x87_reg_t result;
logic invalid;
logic inexact;
logic divide_by_zero;
logic overflow;
logic underflow;
logic [63:0] transfer_out;

always #5 clk = ~clk;

x87_executor dut (
    .clk(clk),
    .reset(reset),
    .start(start),
    .exec_op(exec_op),
    .integer_size(integer_size),
    .precision_control(2'b10),
    .rounding_mode(rounding_mode),
    .quiet_compare(1'b0),
    .trans_cosine(1'b0),
    .trans_tangent_pair(1'b0),
    .trans_atan2(1'b0),
    .operand(operand),
    .operand_b(operand_b),
    .transfer_in(64'h0),
    .busy(busy),
    .done(done),
    .commit_action(),
    .result(result),
    .auxiliary_result(),
    .transfer_out(transfer_out),
    .invalid(invalid),
    .inexact(inexact),
    .divide_by_zero(divide_by_zero),
    .overflow(overflow),
    .underflow(underflow),
    .denormal_operand(),
    .range_incomplete(),
    .rounded_up(),
    .compare_unordered(),
    .compare_less(),
    .compare_equal()
);

function automatic logic is_nan64(input logic [63:0] value);
    return (value[62:52] == 11'h7ff) && (value[51:0] != 0);
endfunction

function automatic logic is_subnormal64(input logic [63:0] value);
    return (value[62:52] == 0) && (value[51:0] != 0);
endfunction

task automatic run_vector(
    input logic [63:0] a,
    input logic [63:0] b,
    input logic [63:0] expected,
    input logic [7:0] expected_flags,
    input integer vector_number
);
    integer cycles;
    logic [63:0] actual;
    x87_reg_t expected_internal;
    begin
        operand = x87_from_m64(a);
        operand_b = x87_from_m64(b);
        @(negedge clk);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        cycles = 0;
        while (!done && (cycles < 512)) begin
            @(posedge clk);
            cycles = cycles + 1;
        end
        if (!done)
            $fatal(1, "TestFloat vector %0d timed out", vector_number);
        @(negedge clk);

        actual = x87_to_m64(result, rounding_mode);
        expected_internal = x87_from_m64(expected);
        if (is_nan64(expected)) begin
            if (result.class_id != X87_NAN)
                $fatal(1,
                       "TestFloat vector %0d expected NaN, got %016x",
                       vector_number, actual);
        end else if (is_subnormal64(expected)) begin
            // A binary64 subnormal is normalized in the x87 extended
            // exponent range, so compare its encoded value, not class_id.
            if (actual !== expected)
                $fatal(1,
                       "TestFloat vector %0d subnormal mismatch: a=%016x b=%016x got=%016x expected=%016x",
                       vector_number, a, b, actual, expected);
        end else if (result !== expected_internal) begin
            $fatal(1,
                   "TestFloat vector %0d result mismatch: a=%016x b=%016x got=%h/%016x expected=%h/%016x",
                   vector_number, a, b, result, actual, expected_internal,
                   expected);
        end

        if ((invalid !== expected_flags[4]) ||
            (divide_by_zero !== expected_flags[3]) ||
            overflow || underflow ||
            (inexact !== expected_flags[0])) begin
            $fatal(1,
                   "TestFloat vector %0d flag mismatch: got I/Z/O/U/P=%b%b%b%b%b expected=%02x",
                   vector_number, invalid, divide_by_zero, overflow,
                   underflow, inexact, expected_flags);
        end
    end
endtask

task automatic run_integer_vector(
    input logic [63:0] a,
    input logic [63:0] expected,
    input logic [7:0] expected_flags,
    input integer vector_number
);
    integer cycles;
    logic [63:0] expected_masked;
    begin
        operand = x87_from_m64(a);
        @(negedge clk);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        cycles = 0;
        while (!done && (cycles < 512)) begin
            @(posedge clk);
            cycles = cycles + 1;
        end
        if (!done)
            $fatal(1, "TestFloat integer vector %0d timed out",
                   vector_number);
        @(negedge clk);

        expected_masked = (integer_size == 2'd1)
                        ? {32'h0, expected[31:0]} : expected;
        if ((transfer_out !== expected_masked) ||
            (invalid !== expected_flags[4]) ||
            (inexact !== expected_flags[0])) begin
            $fatal(1,
                   "TestFloat integer vector %0d mismatch: a=%016x got=%016x/%b%b expected=%016x/%02x",
                   vector_number, a, transfer_out, invalid, inexact,
                   expected_masked, expected_flags);
        end
    end
endtask

initial begin
    string vector_path;
    integer operation;
    integer round_mode;
    integer fd;
    integer fields;
    integer vectors;
    integer skipped;
    logic [63:0] a;
    logic [63:0] b;
    logic [63:0] expected;
    logic [7:0] expected_flags;

    if (!$value$plusargs("VECTORS=%s", vector_path))
        $fatal(1, "missing +VECTORS=<TestFloat output>");
    if (!$value$plusargs("OP=%d", operation))
        $fatal(1,
               "missing +OP=<0:add,1:sub,2:mul,3:div,4:sqrt,5:round,6:i32,7:i64>");
    if (!$value$plusargs("ROUND=%d", round_mode))
        $fatal(1, "missing +ROUND=<0:nearest,1:down,2:up,3:zero>");

    integer_size = 2'd2;
    case (operation)
        0: exec_op = X87_ARITH_ADD;
        1: exec_op = X87_ARITH_SUB;
        2: exec_op = X87_ARITH_MUL;
        3: exec_op = X87_ARITH_DIV;
        4: exec_op = X87_ARITH_SQRT;
        5: exec_op = X87_CONVERT_FRNDINT;
        6: begin
            exec_op = X87_CONVERT_FIST;
            integer_size = 2'd1;
        end
        7: exec_op = X87_CONVERT_FIST;
        default: $fatal(1, "invalid TestFloat operation %0d", operation);
    endcase
    rounding_mode = round_mode[1:0];
    operand = x87_zero(1'b0);
    operand_b = x87_zero(1'b0);

    repeat (4) @(posedge clk);
    reset = 1'b0;
    repeat (2) @(posedge clk);

    fd = $fopen(vector_path, "r");
    if (fd == 0)
        $fatal(1, "cannot open TestFloat vectors: %s", vector_path);

    vectors = 0;
    skipped = 0;
    while (!$feof(fd)) begin
        a = '0;
        b = '0;
        expected = '0;
        expected_flags = '0;
        if (operation >= 4)
            fields = $fscanf(fd, "%h %h %h\n", a, expected,
                             expected_flags);
        else
            fields = $fscanf(fd, "%h %h %h %h\n", a, b, expected,
                             expected_flags);

        if (fields == ((operation >= 4) ? 3 : 4)) begin
            vectors = vectors + 1;
            // Binary64 overflow and underflow flags are not directly
            // comparable: x87 arithmetic retains the extended exponent range.
            if (expected_flags[2] || expected_flags[1])
                skipped = skipped + 1;
            else if (operation >= 6)
                run_integer_vector(a, expected, expected_flags, vectors);
            else
                run_vector(a, b, expected, expected_flags, vectors);
        end
    end
    $fclose(fd);

    if (vectors == skipped)
        $fatal(1, "no comparable TestFloat vectors were executed");
    $display("PASS: TestFloat op=%0d round=%0d vectors=%0d skipped=%0d",
             operation, round_mode, vectors, skipped);
    $finish;
end

endmodule
