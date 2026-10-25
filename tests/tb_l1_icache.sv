`timescale 1ns/1ns

module tb_l1_icache #(parameter ENABLE_SHARED_MAILBOX = 0);
    reg clk = 0;
    always #5 clk = ~clk;

    reg reset = 1;
    reg [31:0] cpu_addr = 32'h0;
    wire [127:0] cpu_line;
    reg cpu_valid = 1'b0;
    wire cpu_ready;
    wire cpu_resp_valid;

    wire [31:0] mem_addr;
    reg [31:0] mem_dout = 32'h0;
    reg [127:0] mem_line_dout = 128'h0;
    wire [3:0] mem_be;
    wire [7:0] mem_burstcount;
    reg mem_ready = 1'b0;
    wire mem_valid;
    reg mem_resp_valid = 1'b0;
    reg mem_line_resp_valid = 1'b0;
    reg wide_mode = 1'b0;
    reg wide_pending = 1'b0;
    reg [31:0] wide_addr = 32'h0;
    integer line_response_count = 0;

    reg [31:0] patch_addr = 32'h0;
    reg [31:0] patch_data = 32'h0;
    reg [3:0] patch_be = 4'h0;
    reg patch_valid = 1'b0;
    reg [31:0] invalidate_addr = 32'h0;
    reg invalidate_valid = 1'b0;

    l1_icache #(.SET_BITS(3), .ENABLE_SHARED_MAILBOX(ENABLE_SHARED_MAILBOX)) dut (
        .clk(clk),
        .reset(reset),
        .cpu_addr(cpu_addr),
        .cpu_line(cpu_line),
        .cpu_valid(cpu_valid),
        .cpu_ready(cpu_ready),
        .cpu_resp_valid(cpu_resp_valid),
        .mem_addr(mem_addr),
        .mem_dout(mem_dout),
        .mem_line_dout(mem_line_dout),
        .mem_be(mem_be),
        .mem_burstcount(mem_burstcount),
        .mem_busy(1'b0),
        .mem_valid(mem_valid),
        .mem_ready(mem_ready),
        .mem_resp_valid(mem_resp_valid),
        .mem_line_resp_valid(mem_line_resp_valid),
        .patch_addr(patch_addr),
        .patch_data(patch_data),
        .patch_be(patch_be),
        .patch_valid(patch_valid),
        .invalidate_addr(invalidate_addr),
        .invalidate_valid(invalidate_valid),
        .cache_enable(1'b1)
    );

    reg [7:0] mem [0:4095];
    reg [31:0] rd_addr = 32'h0;
    reg [7:0] rd_left = 8'd0;
    integer mem_request_count = 0;
    integer mem_request_before;

    task automatic mem_put32(input [31:0] addr, input [31:0] data);
    begin
        mem[addr + 0] = data[7:0];
        mem[addr + 1] = data[15:8];
        mem[addr + 2] = data[23:16];
        mem[addr + 3] = data[31:24];
    end
    endtask

    task automatic mem_put_line(
        input [31:0] addr,
        input [31:0] word0,
        input [31:0] word1,
        input [31:0] word2,
        input [31:0] word3
    );
    begin
        mem_put32(addr + 32'd0, word0);
        mem_put32(addr + 32'd4, word1);
        mem_put32(addr + 32'd8, word2);
        mem_put32(addr + 32'd12, word3);
    end
    endtask

    function automatic [31:0] mem_get32(input [31:0] addr);
        if (addr[31:25] == 7'b0000001) begin
            case (addr[3:2])
                2'd0: mem_get32 = 32'h1357_9BDF;
                2'd1: mem_get32 = 32'h2468_ACE0;
                2'd2: mem_get32 = 32'h55AA_00FF;
                default: mem_get32 = 32'hAA55_FF00;
            endcase
        end else begin
            mem_get32 = {mem[addr[11:0] + 3], mem[addr[11:0] + 2], mem[addr[11:0] + 1], mem[addr[11:0] + 0]};
        end
    endfunction

    always_ff @(posedge clk) begin
        mem_ready <= 1'b0;
        mem_resp_valid <= 1'b0;
        mem_line_resp_valid <= 1'b0;

        if (wide_pending) begin
            mem_line_dout <= {mem_get32(wide_addr + 32'd12),
                              mem_get32(wide_addr + 32'd8),
                              mem_get32(wide_addr + 32'd4),
                              mem_get32(wide_addr)};
            mem_line_resp_valid <= 1'b1;
            wide_pending <= 1'b0;
            line_response_count <= line_response_count + 1;
        end

        if (rd_left != 8'd0) begin
            mem_resp_valid <= 1'b1;
            mem_dout <= mem_get32(rd_addr);
            rd_addr <= rd_addr + 32'd4;
            rd_left <= rd_left - 8'd1;
        end

        if (mem_valid && !mem_ready && rd_left == 8'd0 && !wide_pending) begin
            mem_ready <= 1'b1;
            if (wide_mode && mem_burstcount == 8'd4) begin
                wide_addr <= mem_addr;
                wide_pending <= 1'b1;
            end else begin
                rd_addr <= mem_addr;
                rd_left <= mem_burstcount == 8'd0 ? 8'd1 : mem_burstcount;
            end
            mem_request_count <= mem_request_count + 1;
        end
    end

    task automatic cache_read(input [31:0] addr, input [127:0] expected);
    begin
        do @(negedge clk); while (!cpu_ready);
        cpu_addr = addr;
        cpu_valid = 1'b1;
        @(negedge clk);
        cpu_valid = 1'b0;
        if (!cpu_resp_valid)
            do @(negedge clk); while (!cpu_resp_valid);
        if (cpu_line !== expected) begin
            $display("L1 ICACHE READ FAIL addr=%08x got=%032x expected=%032x",
                     addr, cpu_line, expected);
            $fatal(1);
        end
    end
    endtask

    initial begin
        fork
            begin
                repeat (2000) @(posedge clk);
                $display("L1 ICACHE TIMEOUT state=%0d ready=%0b resp=%0b",
                         dut.state, cpu_ready, cpu_resp_valid);
                $fatal(1);
            end
        join_none

        for (integer n = 0; n < 4096; n = n + 1)
            mem[n] = 8'h0;
        mem_put32(32'h40, 32'h4433_2211);
        mem_put32(32'h44, 32'h8877_6655);
        mem_put32(32'h48, 32'hCCBB_AA99);
        mem_put32(32'h4C, 32'h00FF_EEDD);

        repeat (5) @(posedge clk);
        reset <= 1'b0;
        repeat (20) @(posedge clk);

        cache_read(32'h40, 128'h00FF_EEDD_CCBB_AA99_8877_6655_4433_2211);
        cache_read(32'h40, 128'h00FF_EEDD_CCBB_AA99_8877_6655_4433_2211);
        // Complete physical tags distinguish lines separated by 32MB.
        cache_read(32'h0200_0040, 128'hAA55_FF00_55AA_00FF_2468_ACE0_1357_9BDF);
        cache_read(32'h40, 128'h00FF_EEDD_CCBB_AA99_8877_6655_4433_2211);

        // Accept a hit in the same cycle that a store snoops the cached line.
        // The old line must not escape before the registered invalidation.
        do @(negedge clk); while (!cpu_ready);
        mem_put32(32'h40, 32'hDEAD_BEEF);
        cpu_addr = 32'h40;
        cpu_valid = 1'b1;
        patch_addr = 32'h40;
        patch_data = 32'hDEAD_BEEF;
        patch_be = 4'hF;
        patch_valid = 1'b1;
        @(negedge clk);
        cpu_valid = 1'b0;
        patch_valid = 1'b0;
        if (cpu_resp_valid) begin
            $display("L1 ICACHE SNOOP RACE exposed stale hit %032x", cpu_line);
            $fatal(1);
        end
        do @(negedge clk); while (!cpu_resp_valid);
        if (cpu_line !== 128'h00FF_EEDD_CCBB_AA99_8877_6655_DEAD_BEEF) begin
            $display("L1 ICACHE SNOOP RACE FAIL got=%032x", cpu_line);
            $fatal(1);
        end

        // External DMA snoops are address-only and invalidate through the
        // registered snoop stage. A later fetch must refill the modified line.
        mem_put32(32'h44, 32'hCAFE_BABE);
        @(negedge clk);
        invalidate_addr = 32'h44;
        invalidate_valid = 1'b1;
        @(negedge clk);
        invalidate_valid = 1'b0;
        repeat (2) @(negedge clk);
        cache_read(32'h40, 128'h00FF_EEDD_CCBB_AA99_CAFE_BABE_DEAD_BEEF);

        // An address-only invalidation that coincides with lookup must also
        // suppress the stale hit and force a refill through the registered stage.
        do @(negedge clk); while (!cpu_ready);
        mem_put32(32'h48, 32'h1234_5678);
        cpu_addr = 32'h40;
        cpu_valid = 1'b1;
        invalidate_addr = 32'h48;
        invalidate_valid = 1'b1;
        @(negedge clk);
        cpu_valid = 1'b0;
        invalidate_valid = 1'b0;
        if (cpu_resp_valid) begin
            $display("L1 ICACHE INVALIDATE RACE exposed stale hit %032x", cpu_line);
            $fatal(1);
        end
        do @(negedge clk); while (!cpu_resp_valid);
        if (cpu_line !== 128'h00FF_EEDD_1234_5678_CAFE_BABE_DEAD_BEEF) begin
            $display("L1 ICACHE INVALIDATE RACE FAIL got=%032x", cpu_line);
            $fatal(1);
        end

        // A live invalidation on the final fill beat is captured one cycle
        // before its tag match can complete.  The just-filled line must stay
        // invalid rather than briefly becoming a stale hit.
        mem_put32(32'h100, 32'h1111_0000);
        mem_put32(32'h104, 32'h3333_2222);
        mem_put32(32'h108, 32'h5555_4444);
        mem_put32(32'h10C, 32'h7777_6666);
        do @(negedge clk); while (!cpu_ready);
        cpu_addr = 32'h100;
        cpu_valid = 1'b1;
        @(negedge clk);
        cpu_valid = 1'b0;
        do @(negedge clk); while (!(dut.state == 3'd3 &&
                                    dut.fill_count == 2'd3 &&
                                    mem_resp_valid));
        mem_put32(32'h100, 32'hDEAD_C0DE);
        invalidate_addr = 32'h100;
        invalidate_valid = 1'b1;
        @(negedge clk);
        invalidate_valid = 1'b0;
        if (!cpu_resp_valid)
            do @(negedge clk); while (!cpu_resp_valid);
        if (cpu_line !== 128'h7777_6666_5555_4444_3333_2222_1111_0000) begin
            $display("L1 ICACHE FINAL FILL RESPONSE FAIL got=%032x", cpu_line);
            $fatal(1);
        end
        repeat (2) @(negedge clk);
        cache_read(32'h100, 128'h7777_6666_5555_4444_3333_2222_DEAD_C0DE);

        // A registered snoop miss does not consume a tag write port.  When it
        // overlaps the last beat of an unrelated fill, that fill must remain
        // cached instead of causing a later avoidable miss.
        mem_put32(32'h140, 32'h0123_4567);
        mem_put32(32'h144, 32'h89AB_CDEF);
        mem_put32(32'h148, 32'h7654_3210);
        mem_put32(32'h14C, 32'hFEDC_BA98);
        do @(negedge clk); while (!cpu_ready);
        cpu_addr = 32'h140;
        cpu_valid = 1'b1;
        @(negedge clk);
        cpu_valid = 1'b0;
        do @(negedge clk); while (!(dut.state == 3'd3 &&
                                    dut.fill_count == 2'd2 &&
                                    mem_resp_valid));
        invalidate_addr = 32'h300;
        invalidate_valid = 1'b1;
        @(negedge clk);
        invalidate_valid = 1'b0;
        if (!cpu_resp_valid)
            do @(negedge clk); while (!cpu_resp_valid);
        repeat (2) @(negedge clk);
        mem_request_before = mem_request_count;
        cache_read(32'h140, 128'hFEDC_BA98_7654_3210_89AB_CDEF_0123_4567);
        if (mem_request_count != mem_request_before) begin
            $display("L1 ICACHE UNRELATED SNOOP suppressed fill");
            $fatal(1);
        end

        // Recreate the packed-tag collision from FastDoom.  Four lines in set
        // zero occupy all four ways; with the initial PLRU state, 0x300 is in
        // way 1.  A fresh set-one fill chooses way 0.  A matching invalidate
        // for way 1 on the final beat must clear that way while the fill tag
        // is installed in way 0.  The old global tag-write arbitration wrote
        // the fill data but suppressed its tag, leaving a stale valid tag
        // paired with the wrong line.
        reset = 1'b1;
        repeat (5) @(posedge clk);
        reset = 1'b0;
        repeat (20) @(posedge clk);
        mem_put_line(32'h200, 32'h2000_0000, 32'h2000_0001,
                     32'h2000_0002, 32'h2000_0003);
        mem_put_line(32'h280, 32'h2800_0000, 32'h2800_0001,
                     32'h2800_0002, 32'h2800_0003);
        mem_put_line(32'h300, 32'h3000_0000, 32'h3000_0001,
                     32'h3000_0002, 32'h3000_0003);
        mem_put_line(32'h380, 32'h3800_0000, 32'h3800_0001,
                     32'h3800_0002, 32'h3800_0003);
        mem_put_line(32'h210, 32'h2100_0000, 32'h2100_0001,
                     32'h2100_0002, 32'h2100_0003);
        mem_put_line(32'h220, 32'h2200_0000, 32'h2200_0001,
                     32'h2200_0002, 32'h2200_0003);
        cache_read(32'h200, 128'h2000_0003_2000_0002_2000_0001_2000_0000);
        cache_read(32'h280, 128'h2800_0003_2800_0002_2800_0001_2800_0000);
        cache_read(32'h300, 128'h3000_0003_3000_0002_3000_0001_3000_0000);
        cache_read(32'h380, 128'h3800_0003_3800_0002_3800_0001_3800_0000);
        if (!dut.tag_way1[0][20] || dut.tag_way1[0][19:0] != 20'h00006) begin
            $display("L1 ICACHE COLLISION SETUP expected 0x300 in way 1");
            $fatal(1);
        end

        do @(negedge clk); while (!cpu_ready);
        cpu_addr = 32'h210;
        cpu_valid = 1'b1;
        @(negedge clk);
        cpu_valid = 1'b0;
        do @(negedge clk); while (!(dut.state == 3'd3 &&
                                    dut.fill_count == 2'd2 &&
                                    mem_resp_valid));
        invalidate_addr = 32'h300;
        invalidate_valid = 1'b1;
        @(negedge clk);
        invalidate_valid = 1'b0;
        if (!cpu_resp_valid)
            do @(negedge clk); while (!cpu_resp_valid);
        if (cpu_line !== 128'h2100_0003_2100_0002_2100_0001_2100_0000) begin
            $display("L1 ICACHE DIFFERENT-WAY COLLISION RESPONSE FAIL got=%032x", cpu_line);
            $fatal(1);
        end
        repeat (2) @(negedge clk);
        mem_request_before = mem_request_count;
        cache_read(32'h210, 128'h2100_0003_2100_0002_2100_0001_2100_0000);
        if (mem_request_count != mem_request_before) begin
            $display("L1 ICACHE DIFFERENT-WAY COLLISION lost fill tag");
            $fatal(1);
        end

        // If the snoop and a different-line fill need the same way RAM, the
        // fill tag may win: replacing the old tag also invalidates the snooped
        // line, so both operations are satisfied by the single RAM write.
        do @(negedge clk); while (!cpu_ready);
        cpu_addr = 32'h220;
        cpu_valid = 1'b1;
        @(negedge clk);
        cpu_valid = 1'b0;
        do @(negedge clk); while (!(dut.state == 3'd3 &&
                                    dut.fill_count == 2'd2 &&
                                    mem_resp_valid));
        invalidate_addr = 32'h200;
        invalidate_valid = 1'b1;
        @(negedge clk);
        invalidate_valid = 1'b0;
        if (!cpu_resp_valid)
            do @(negedge clk); while (!cpu_resp_valid);
        if (cpu_line !== 128'h2200_0003_2200_0002_2200_0001_2200_0000) begin
            $display("L1 ICACHE SAME-WAY COLLISION RESPONSE FAIL got=%032x", cpu_line);
            $fatal(1);
        end
        repeat (2) @(negedge clk);
        mem_request_before = mem_request_count;
        cache_read(32'h220, 128'h2200_0003_2200_0002_2200_0001_2200_0000);
        if (mem_request_count != mem_request_before) begin
            $display("L1 ICACHE SAME-WAY COLLISION lost fill tag");
            $fatal(1);
        end

        // A native KV260 DDR line arrives as one 128-bit response pulse.
        reset = 1'b1;
        repeat (5) @(posedge clk);
        reset = 1'b0;
        repeat (20) @(posedge clk);
        mem_put_line(32'h400, 32'h4000_0000, 32'h4000_0001,
                     32'h4000_0002, 32'h4000_0003);
        wide_mode = 1'b1;
        mem_request_before = mem_request_count;
        cache_read(32'h400,
                   128'h4000_0003_4000_0002_4000_0001_4000_0000);
        if (line_response_count != 1) begin
            $display("L1 ICACHE WIDE FILL expected one line response, got %0d",
                     line_response_count);
            $fatal(1);
        end
        cache_read(32'h400,
                   128'h4000_0003_4000_0002_4000_0001_4000_0000);
        if (mem_request_count != mem_request_before + 1) begin
            $display("L1 ICACHE WIDE FILL request count before=%0d after=%0d",
                     mem_request_before, mem_request_count);
            $fatal(1);
        end

        if (ENABLE_SHARED_MAILBOX) begin
            mem_put_line(32'h500, 1, 2, 3, 4);
            mem_request_before = mem_request_count;
            cache_read(32'h000ce500, 128'h00000004_00000003_00000002_00000001);
            mem_put_line(32'h500, 5, 6, 7, 8);
            cache_read(32'h000ce500, 128'h00000008_00000007_00000006_00000005);
            if (mem_request_count != mem_request_before + 2)
                $fatal(1, "shared instruction line was installed");
            $display("Shared mailbox I-cache PASS: uncached line bursts");
        end
        $display("L1 PIPT instruction cache unit test PASS");
        $finish;
    end
endmodule
