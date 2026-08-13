`timescale 1ns/1ns

module tb_x87_core;

logic clk = 1'b0;
logic reset = 1'b1;
logic cmd_valid = 1'b0;
logic [10:0] cmd_fop = '0;
logic cmd_ready;
logic word_in_valid = 1'b0;
logic [3:0] word_in_be = '0;
logic [31:0] word_in_data = '0;
logic word_in_ready;
logic read_req_valid = 1'b0;
logic read_req_data_port = 1'b0;
logic [3:0] read_req_be = '0;
logic read_req_ready;
logic read_resp_valid;
logic [31:0] read_resp_data;
logic busy_n;
logic pereq;
logic error_n;
logic [31:0] saved_state [0:26];

always #5 clk = ~clk;

x87_core dut (
    .clk(clk), .reset(reset),
    .cmd_valid(cmd_valid), .cmd_fop(cmd_fop), .cmd_ready(cmd_ready),
    .word_in_valid(word_in_valid), .word_in_be(word_in_be),
    .word_in_data(word_in_data), .word_in_ready(word_in_ready),
    .read_req_valid(read_req_valid), .read_req_data_port(read_req_data_port),
    .read_req_be(read_req_be), .read_req_ready(read_req_ready),
    .read_resp_valid(read_resp_valid), .read_resp_data(read_resp_data),
    .busy_n(busy_n), .pereq(pereq), .error_n(error_n)
);

task automatic send_command(input logic [10:0] fop);
    begin
        @(negedge clk);
        cmd_fop = fop;
        cmd_valid = 1'b1;
        do @(posedge clk); while (!cmd_ready);
        @(negedge clk);
        cmd_valid = 1'b0;
    end
endtask

task automatic wait_command_complete;
    begin
        // Advance past command acceptance before observing ready again.
        do @(posedge clk); while (!cmd_ready);
        @(negedge clk);
    end
endtask

task automatic send_word(input logic [31:0] data, input logic [3:0] be);
    begin
        @(negedge clk);
        word_in_data = data;
        word_in_be = be;
        word_in_valid = 1'b1;
        do @(posedge clk); while (!word_in_ready);
        @(negedge clk);
        word_in_valid = 1'b0;
    end
endtask

task automatic read_word(output logic [31:0] data);
    begin
        @(negedge clk);
        read_req_data_port = 1'b1;
        read_req_be = 4'hf;
        read_req_valid = 1'b1;
        do @(posedge clk); while (!read_req_ready);
        @(negedge clk);
        read_req_valid = 1'b0;
        do @(posedge clk); while (!read_resp_valid);
        data = read_resp_data;
    end
endtask

task automatic expect_f8_word(input logic [31:0] expected, input logic [3:0] be);
    logic [31:0] mask;
    begin
        mask = {{8{be[3]}}, {8{be[2]}}, {8{be[1]}}, {8{be[0]}}};
        @(negedge clk);
        read_req_data_port = 1'b0;
        read_req_be = be;
        read_req_valid = 1'b1;
        do @(posedge clk); while (!read_req_ready);
        @(negedge clk);
        read_req_valid = 1'b0;
        do @(posedge clk); while (!read_resp_valid);
        if ((read_resp_data & mask) !== (expected & mask))
            $fatal(1, "x87 f8 mismatch: got=%08x expected=%08x be=%x",
                   read_resp_data, expected, be);
        @(negedge clk);
        while (pereq)
            @(negedge clk);
    end
endtask

task automatic expect_word(input logic [31:0] expected, input logic [3:0] be);
    logic [31:0] actual;
    logic [31:0] mask;
    begin
        read_word(actual);
        mask = {{8{be[3]}}, {8{be[2]}}, {8{be[1]}}, {8{be[0]}}};
        if ((actual & mask) !== (expected & mask))
            $fatal(1, "x87 read mismatch: got=%08x expected=%08x be=%x",
                   actual, expected, be);
    end
endtask

task automatic load_m80(input logic [79:0] value);
    begin
        send_command(11'h328);
        send_word(value[31:0], 4'hf);
        send_word(value[63:32], 4'hf);
        send_word({16'h0, value[79:64]}, 4'h3);
    end
endtask

task automatic load_m80_burst(input logic [79:0] value);
    logic [31:0] words [0:2];
    logic [3:0] byte_enables [0:2];
    integer index;
    begin
        words[0] = value[31:0];
        words[1] = value[63:32];
        words[2] = {16'h0, value[79:64]};
        byte_enables[0] = 4'hf;
        byte_enables[1] = 4'hf;
        byte_enables[2] = 4'h3;
        send_command(11'h328);
        while (!word_in_ready)
            @(negedge clk);
        if (!pereq)
            $fatal(1, "x87 input transfer ready without PEREQ");
        @(negedge clk);
        word_in_valid = 1'b1;
        for (index = 0; index < 3; index = index + 1) begin
            word_in_data = words[index];
            word_in_be = byte_enables[index];
            do @(posedge clk); while (!word_in_ready);
            @(negedge clk);
        end
        word_in_valid = 1'b0;
    end
endtask

task automatic expect_store_m80(input logic [79:0] value);
    begin
        send_command(11'h338);
        while (!dut.transfer_pop_valid)
            @(negedge clk);
        if (!pereq)
            $fatal(1, "x87 output FIFO data available without PEREQ");
        expect_word(value[31:0], 4'hf);
        expect_word(value[63:32], 4'hf);
        expect_word({16'h0, value[79:64]}, 4'h3);
    end
endtask

initial begin
    logic [79:0] value_a;
    logic [79:0] value_b;
    logic [79:0] value_c;
    integer word_index;

    value_a = 80'h3fff_8000_0000_0000_0123;
    value_b = 80'hc000_a5a5_5a5a_0123_4567;
    value_c = 80'h0000_8000_0000_0000_03ff;

    repeat (4) @(posedge clk);
    reset = 1'b0;

    // Direct m80 transfers retain all 64 explicit significand bits.
    send_command(11'h3e3);
    load_m80_burst(value_a);
    send_command(11'h338);
    // A three-word result can be produced independently of CPU reads.
    while (dut.transfer_count != 2'd3)
        @(posedge clk);
    if (!dut.tx_generation_done)
        $fatal(1, "x87 output FIFO filled before generation completed");
    expect_word(value_a[31:0], 4'hf);
    expect_word(value_a[63:32], 4'hf);
    expect_word({16'h0, value_a[79:64]}, 4'h3);
    load_m80(value_c);
    expect_store_m80(value_c);

    // Sign-only operations must not pass through the 53-bit arithmetic form.
    load_m80(value_a);
    send_command(11'h1e0); // FCHS
    expect_store_m80({1'b1, value_a[78:0]});
    load_m80(value_b);
    send_command(11'h1e1); // FABS
    expect_store_m80({1'b0, value_b[78:0]});

    // Register copies and exchanges retain raw architectural payloads.
    load_m80(value_a);
    load_m80(value_b);
    send_command(11'h1c1); // FLD ST(1)
    expect_store_m80(value_a);
    send_command(11'h1c9); // FXCH ST(1)
    expect_store_m80(value_a);
    expect_store_m80(value_b);

    // FSAVE and FRSTOR preserve the same raw values across the 640-bit image.
    load_m80(value_a);
    load_m80(value_b);
    send_command(11'h536);
    for (word_index = 0; word_index < 27; word_index = word_index + 1)
        read_word(saved_state[word_index]);
    send_command(11'h526);
    for (word_index = 0; word_index < 27; word_index = word_index + 1)
        send_word(saved_state[word_index], word_index < 7 ? 4'h3 : 4'hf);
    expect_store_m80(value_b);
    expect_store_m80(value_a);

    // The DOS coprocessor probe uses the memory form of FNSTSW.
    send_command(11'h3e3);               // FNINIT
    send_command(11'h53e);               // FNSTSW m16
    expect_f8_word(32'h0000_0000, 4'h3);
    send_command(11'h138);               // FNSTCW m16
    expect_f8_word(32'h0000_037f, 4'h3);

    // An interrupt can make the 80386 replay a memory instruction after its
    // x87 command was accepted but before any operand bytes were transferred.
    send_command(11'h344);               // FILD m32int
    while (dut.rx_kind != dut.RX_I32)
        @(negedge clk);
    cmd_fop = 11'h3e3;                   // Unrelated FNINIT stays blocked
    cmd_valid = 1'b1;
    @(posedge clk);
    if (cmd_ready)
        $fatal(1, "unrelated command replaced pending FILD operand");
    @(negedge clk);
    cmd_valid = 1'b0;
    send_command(11'h344);               // RPTI replay before first data word
    send_word(32'd42, 4'hf);
    wait_command_complete();
    expect_store_m80(80'h4004_a800_0000_0000_0000);

    // Memory arithmetic must release BUSY# before the CPU supplies its
    // operand. The 80386 waits for this boundary before writing port 0xFC.
    load_m80(80'h3fff_8000_0000_0000_0000); // 1.0
    send_command(11'h445);                   // FADD m64real
    while (!dut.memory_math_pending)
        @(negedge clk);
    if (!busy_n || !word_in_ready)
        $fatal(1, "FADD m64 blocked operand transfer: busy_n=%b ready=%b",
               busy_n, word_in_ready);
    send_word(32'h0000_0000, 4'hf);          // 1.0, low dword
    send_word(32'h3ff0_0000, 4'hf);          // 1.0, high dword
    wait_command_complete();
    expect_store_m80(80'h4000_8000_0000_0000_0000); // 2.0

    // RPTI may also replay memory arithmetic while its operand is pending.
    // TurboQuake exercises this exact FCOMP m32 command sequence.
    load_m80(80'h3fff_8000_0000_0000_0000); // 1.0
    send_command(11'h05a);                   // FCOMP dword [edx+0xc]
    while (!dut.memory_math_pending || (dut.rx_kind != dut.RX_M32))
        @(negedge clk);
    send_command(11'h05a);                   // RPTI replay before operand
    send_word(32'h3f80_0000, 4'hf);          // 1.0f
    wait_command_complete();
    if ((dut.top !== 3'd0) || (dut.tag_word !== 16'hffff) ||
        (dut.status_flags[14] !== 1'b1) ||
        (dut.status_flags[10] !== 1'b0) ||
        (dut.status_flags[8] !== 1'b0))
        $fatal(1, "replayed FCOMP m32 failed: top=%x tags=%04x status=%04x",
               dut.top, dut.tag_word, dut.status_flags);

    // Both unordered and ordered compare-and-pop-twice encodings must empty
    // their two operands. TurboQuake's formatter relies on FUCOMPP.
    load_m80(value_a);
    load_m80(value_b);
    send_command(11'h2e9);               // FUCOMPP
    wait_command_complete();
    if ((dut.top !== 3'd0) || (dut.tag_word !== 16'hffff))
        $fatal(1, "FUCOMPP did not pop twice: top=%x tags=%04x",
               dut.top, dut.tag_word);
    load_m80(value_a);
    load_m80(value_b);
    send_command(11'h6d9);               // FCOMPP
    wait_command_complete();
    if ((dut.top !== 3'd0) || (dut.tag_word !== 16'hffff))
        $fatal(1, "FCOMPP did not pop twice: top=%x tags=%04x",
               dut.top, dut.tag_word);
    send_command(11'h3e3);               // Isolate following environment test

    // TurboQuake changes exception masks through an FNSTENV/FLDENV pair.
    // Environment stores stream seven dwords and mask exceptions only after
    // the CPU has consumed the final word.
    send_command(11'h128);               // FLDCW: leave exception masks clear
    send_word(32'h0000_0340, 4'h3);
    load_m80(value_a);                    // TOP=7, physical R7 valid
    send_command(11'h136);               // FNSTENV
    expect_word(32'h0000_0340, 4'h3);
    expect_word(32'h0000_3800, 4'h3);
    expect_word(32'h0000_3fff, 4'h3);
    repeat (4) expect_word(32'h0000_0000, 4'h3);
    send_command(11'h138);               // FNSTCW reflects FSTENV masking
    expect_f8_word(32'h0000_037f, 4'h3);

    // FLDENV restores the x87-owned words and consumes the four CPU pointer
    // slots supplied by the integer processor.
    send_command(11'h126);
    send_word(32'h0000_0b40, 4'h3);
    send_word(32'h0000_1000, 4'h3);       // TOP=2
    send_word(32'h0000_ffff, 4'h3);       // All physical registers empty
    repeat (4) send_word(32'ha5a5_5a5a, 4'hf);
    send_command(11'h136);
    expect_word(32'h0000_0b40, 4'h3);
    expect_word(32'h0000_1000, 4'h3);
    expect_word(32'h0000_ffff, 4'h3);
    repeat (4) expect_word(32'h0000_0000, 4'h3);

    $display("x87 core PASS: exact stack and environment transfers");
    $finish;
end

endmodule
