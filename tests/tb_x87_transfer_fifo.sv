`timescale 1ns/1ns

module tb_x87_transfer_fifo;

logic clk = 1'b0;
logic reset = 1'b1;
logic clear = 1'b0;
logic push_valid = 1'b0;
logic [35:0] push_data = '0;
logic push_ready;
logic pop_valid;
logic [35:0] pop_data;
logic pop_ready = 1'b0;
logic [1:0] count;

always #5 clk = ~clk;

x87_transfer_fifo dut (.*);

task automatic push(input logic [35:0] value);
    begin
        @(negedge clk);
        push_data = value;
        push_valid = 1'b1;
        do @(posedge clk); while (!push_ready);
        @(negedge clk);
        push_valid = 1'b0;
    end
endtask

task automatic pop(input logic [35:0] expected);
    begin
        @(negedge clk);
        pop_ready = 1'b1;
        do @(posedge clk); while (!pop_valid);
        if (pop_data !== expected)
            $fatal(1, "x87 FIFO mismatch: got=%09x expected=%09x",
                   pop_data, expected);
        @(negedge clk);
        pop_ready = 1'b0;
    end
endtask

initial begin
    repeat (3) @(posedge clk);
    reset = 1'b0;

    push(36'h1_11111111);
    push(36'h2_22222222);
    push(36'h3_33333333);
    if (count != 2'd3 || push_ready)
        $fatal(1, "x87 FIFO did not report full");
    pop(36'h1_11111111);
    pop(36'h2_22222222);

    // Replace the remaining head while dequeuing to exercise independent
    // read/write pointers and stable occupancy.
    @(negedge clk);
    push_data = 36'h4_44444444;
    push_valid = 1'b1;
    pop_ready = 1'b1;
    @(posedge clk);
    if (!push_ready || !pop_valid || pop_data !== 36'h3_33333333)
        $fatal(1, "x87 FIFO simultaneous transfer failed");
    @(negedge clk);
    push_valid = 1'b0;
    pop_ready = 1'b0;
    pop(36'h4_44444444);

    push(36'ha_aaaaaaaa);
    @(negedge clk);
    clear = 1'b1;
    @(posedge clk);
    @(negedge clk);
    clear = 1'b0;
    if (count != 0 || pop_valid)
        $fatal(1, "x87 FIFO clear failed");

    $display("x87 transfer FIFO PASS");
    $finish;
end

endmodule
