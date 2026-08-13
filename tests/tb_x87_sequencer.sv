`timescale 1ns/1ns

module tb_x87_sequencer;

import x87_ucode_pkg::*;

logic clk = 1'b0;
logic reset = 1'b1;
logic start;
logic [7:0] entry;
logic [31:0] conditions;
logic active;
logic exec_valid;
logic done;
logic [7:0] uaddr;
x87_uop_t uop;

always #5 clk = ~clk;

x87_sequencer dut (
    .clk(clk),
    .reset(reset),
    .start(start),
    .entry(entry),
    .conditions(conditions),
    .active(active),
    .exec_valid(exec_valid),
    .done(done),
    .uaddr(uaddr),
    .uop(uop)
);

task automatic launch(input logic [7:0] start_addr);
    begin
        @(negedge clk);
        entry = start_addr;
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        if (!active || !exec_valid || (uaddr != start_addr))
            $fatal(1, "entry launch failed: active=%0b uaddr=%02x", active, uaddr);
    end
endtask

initial begin
    integer loop_visits;
    integer wait_visits;

    start = 1'b0;
    entry = 8'h00;
    conditions = '0;
    conditions[X87_COND_TRUE] = 1'b1;

    repeat (3) @(posedge clk);
    reset = 1'b0;

    launch(X87_ENTRY_TEST_BRANCH);
    if (uop.flow != X87_FLOW_BRANCH)
        $fatal(1, "branch entry did not decode as BRANCH");
    @(negedge clk);
    if (uaddr != (X87_ENTRY_TEST_BRANCH + 8'd2))
        $fatal(1, "taken branch reached %02x", uaddr);
    @(negedge clk);
    if (!done || active)
        $fatal(1, "branch routine did not finish");

    conditions[X87_COND_COUNT_MORE] = 1'b1;
    launch(X87_ENTRY_TEST_LOOP);
    loop_visits = 1;
    while (uaddr == X87_ENTRY_TEST_LOOP) begin
        if (loop_visits == 3)
            conditions[X87_COND_COUNT_MORE] = 1'b0;
        @(negedge clk);
        if (uaddr == X87_ENTRY_TEST_LOOP)
            loop_visits = loop_visits + 1;
    end
    if (loop_visits != 3 ||
        uaddr != (X87_ENTRY_TEST_LOOP + 8'd1))
        $fatal(1, "loop visits=%0d exit=%02x", loop_visits, uaddr);
    @(negedge clk);
    if (!done)
        $fatal(1, "loop routine did not finish");

    conditions[X87_COND_TRANSFER_READY] = 1'b0;
    launch(X87_ENTRY_TEST_WAIT);
    wait_visits = 1;
    repeat (2) begin
        @(negedge clk);
        if (uaddr != X87_ENTRY_TEST_WAIT)
            $fatal(1, "WAIT advanced without ready");
        wait_visits = wait_visits + 1;
    end
    conditions[X87_COND_TRANSFER_READY] = 1'b1;
    @(negedge clk);
    if (uaddr != (X87_ENTRY_TEST_WAIT + 8'd1))
        $fatal(1, "WAIT did not advance when ready");
    @(negedge clk);
    if (!done)
        $fatal(1, "wait routine did not finish");

    $display("x87 sequencer PASS: branch, loop, wait");
    $finish;
end

endmodule
