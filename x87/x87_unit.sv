// CPU-facing x87 adapter. It owns the 386-compatible port bridge and the FAST
// m32 demand-read transport; paging/cache arbitration remains in z486.
module x87_unit #(
    parameter ENABLE_X87 = 0
)(
    input  logic        clk,
    input  logic        reset_n,

    input  logic        req_valid,
    input  logic        req_data_port,
    input  logic        req_write,
    input  logic [3:0]  req_be,
    input  logic [31:0] req_wdata,
    output logic        req_accepted,
    output logic        req_complete,
    output logic        req_read_complete,
    output logic [31:0] req_rdata,

    input  logic        fast_launch,
    input  logic        fast_candidate,
    input  logic        fast_allowed,
    input  logic [10:0] fast_fop,
    output logic        fast_active,
    output logic        fast_mem_req,
    output logic        fast_stall,

    input  logic        mem_accepted,
    input  logic [1:0]  mem_addr_low,
    input  logic        mem_read_complete,
    input  logic        mem_servicing,
    input  logic [31:0] mem_rdata,
    input  logic [31:0] split_rdata,
    input  logic        cancel,

    output logic        busy_n,
    output logic        pereq,
    output logic        error_n,
    output logic [31:0] debug_state
);

logic        queue_safe;
logic        fast_ready;
logic        fast_issued_r;
logic        fast_crossing_r;
logic        fast_data_valid_r;
logic [31:0] fast_data_r;
wire         fast_valid = fast_active && fast_data_valid_r;
wire         fast_release = fast_valid && fast_ready;

assign fast_mem_req = fast_active && !fast_issued_r && !fast_data_valid_r;
assign fast_stall = fast_active && !fast_release;

generate
if (ENABLE_X87) begin : gen_x87
    wire        cmd_valid;
    wire [10:0] cmd_fop;
    wire        cmd_ready;
    wire        word_in_valid;
    wire  [3:0] word_in_be;
    wire [31:0] word_in_data;
    wire        word_in_ready;
    wire        read_req_valid;
    wire        read_req_data_port;
    wire  [3:0] read_req_be;
    wire        read_req_ready;
    wire        read_resp_valid;
    wire [31:0] read_resp_data;

    x87_bridge bridge (
        .clk(clk), .reset(!reset_n),
        .req_valid(req_valid), .req_data_port(req_data_port),
        .req_write(req_write), .req_be(req_be), .req_wdata(req_wdata),
        .req_accepted(req_accepted), .req_complete(req_complete),
        .req_read_complete(req_read_complete), .req_rdata(req_rdata),
        .cmd_valid(cmd_valid), .cmd_fop(cmd_fop), .cmd_ready(cmd_ready),
        .word_in_valid(word_in_valid), .word_in_be(word_in_be),
        .word_in_data(word_in_data), .word_in_ready(word_in_ready),
        .read_req_valid(read_req_valid),
        .read_req_data_port(read_req_data_port), .read_req_be(read_req_be),
        .read_req_ready(read_req_ready), .read_resp_valid(read_resp_valid),
        .read_resp_data(read_resp_data)
    );

    x87_control control (
        .clk(clk), .reset(!reset_n),
        .cmd_valid(cmd_valid), .cmd_fop(cmd_fop), .cmd_ready(cmd_ready),
        .fast_m32_valid(fast_valid), .fast_m32_fop(fast_fop),
        .fast_m32_data(fast_data_r), .fast_m32_ready(fast_ready),
        .word_in_valid(word_in_valid), .word_in_be(word_in_be),
        .word_in_data(word_in_data), .word_in_ready(word_in_ready),
        .read_req_valid(read_req_valid),
        .read_req_data_port(read_req_data_port), .read_req_be(read_req_be),
        .read_req_ready(read_req_ready), .read_resp_valid(read_resp_valid),
        .read_resp_data(read_resp_data),
        .busy_n(busy_n), .pereq(pereq), .error_n(error_n),
        .queue_safe(queue_safe), .debug_state(debug_state)
    );
end else begin : gen_no_x87
    assign req_accepted = 1'b0;
    assign req_complete = 1'b0;
    assign req_read_complete = 1'b0;
    assign req_rdata = 32'h0;
    assign busy_n = 1'b1;
    assign pereq = 1'b0;
    assign error_n = 1'b1;
    assign queue_safe = 1'b0;
    assign fast_ready = 1'b0;
    assign debug_state = 32'h8000_0000;
end
endgenerate

always_ff @(posedge clk) begin
    if (!reset_n) begin
        fast_active       <= 1'b0;
        fast_issued_r     <= 1'b0;
        fast_crossing_r   <= 1'b0;
        fast_data_valid_r <= 1'b0;
        fast_data_r       <= 32'h0;
    end else begin
        if (fast_release) begin
            fast_active       <= 1'b0;
            fast_issued_r     <= 1'b0;
            fast_data_valid_r <= 1'b0;
        end

        if (fast_launch) begin
            fast_active       <= ENABLE_X87 && fast_candidate &&
                                 fast_allowed && queue_safe;
            fast_issued_r     <= 1'b0;
            fast_crossing_r   <= 1'b0;
            fast_data_valid_r <= 1'b0;
        end

        if (fast_mem_req && mem_accepted) begin
            fast_issued_r   <= 1'b1;
            fast_crossing_r <= mem_addr_low != 2'b00;
        end

        if (fast_issued_r && !fast_crossing_r && mem_read_complete) begin
            fast_data_r       <= mem_rdata;
            fast_data_valid_r <= 1'b1;
        end else if (fast_issued_r && fast_crossing_r && !mem_servicing) begin
            fast_data_r       <= split_rdata;
            fast_data_valid_r <= 1'b1;
        end

        if (cancel) begin
            fast_active       <= 1'b0;
            fast_issued_r     <= 1'b0;
            fast_data_valid_r <= 1'b0;
        end
    end
end

endmodule
