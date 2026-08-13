`timescale 1ns/1ns

module tb_x87_bridge;
    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic reset = 1'b1;
    logic req_valid = 1'b0;
    logic [31:0] req_addr = 32'h0;
    logic req_write = 1'b0;
    logic [3:0] req_be = 4'h0;
    logic [31:0] req_wdata = 32'h0;
    logic req_accepted;
    logic req_complete;
    logic req_read_complete;
    logic [31:0] req_rdata;

    logic cmd_valid;
    logic [10:0] cmd_fop;
    logic cmd_ready = 1'b0;
    logic word_in_valid;
    logic [3:0] word_in_be;
    logic [31:0] word_in_data;
    logic word_in_ready = 1'b0;
    logic read_req_valid;
    logic read_req_data_port;
    logic [3:0] read_req_be;
    logic read_req_ready = 1'b0;
    logic read_resp_valid = 1'b0;
    logic [31:0] read_resp_data = 32'h0;

    x87_bridge dut (.*);

    task automatic begin_req(
        input logic [31:0] addr,
        input logic write_req,
        input logic [3:0] be,
        input logic [31:0] data
    );
    begin
        @(negedge clk);
        req_addr = addr;
        req_write = write_req;
        req_be = be;
        req_wdata = data;
        req_valid = 1'b1;
        #1;
        if (!req_accepted)
            $fatal(1, "x87 bridge did not accept idle request");
        @(negedge clk);
        req_valid = 1'b0;
    end
    endtask

    task automatic wait_complete(input logic read_req, input logic [31:0] expected);
    begin
        while (!req_complete) @(negedge clk);
        if (req_read_complete != read_req)
            $fatal(1, "x87 bridge read-complete mismatch");
        if (read_req && req_rdata !== expected)
            $fatal(1, "x87 bridge read mismatch got=%08x expected=%08x", req_rdata, expected);
    end
    endtask

    initial begin
        repeat (4) @(posedge clk);
        reset <= 1'b0;

        // Command writes are held until the sidecar accepts them.
        begin_req(32'h8000_00f8, 1'b1, 4'h3, 32'h0000_03e3);
        repeat (2) @(negedge clk);
        if (!cmd_valid || cmd_fop != 11'h3e3)
            $fatal(1, "x87 command stream mismatch");
        cmd_ready = 1'b1;
        @(negedge clk);
        cmd_ready = 1'b0;
        wait_complete(1'b0, 32'h0);

        // Operand words preserve their width and data.
        begin_req(32'h8000_00fc, 1'b1, 4'hf, 32'h4004_0000);
        repeat (2) @(negedge clk);
        if (!word_in_valid || word_in_be != 4'hf || word_in_data != 32'h4004_0000)
            $fatal(1, "x87 operand stream mismatch");
        word_in_ready = 1'b1;
        @(negedge clk);
        word_in_ready = 1'b0;
        wait_complete(1'b0, 32'h0);

        // Read request and response are independently backpressured.
        begin_req(32'h8000_00fc, 1'b0, 4'h3, 32'h0);
        repeat (2) @(negedge clk);
        if (!read_req_valid || !read_req_data_port || read_req_be != 4'h3)
            $fatal(1, "x87 read request mismatch");
        read_req_ready = 1'b1;
        @(negedge clk);
        read_req_ready = 1'b0;
        repeat (3) @(negedge clk);
        read_resp_data = 32'h1234_abcd;
        read_resp_valid = 1'b1;
        @(negedge clk);
        read_resp_valid = 1'b0;
        wait_complete(1'b1, 32'h1234_abcd);

        $display("x87 bridge unit test PASS");
        $finish;
    end

    initial begin
        repeat (200) @(posedge clk);
        $fatal(1, "x87 bridge unit test timeout");
    end
endmodule
