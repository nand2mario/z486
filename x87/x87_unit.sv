// CPU-facing x87 adapter. It owns the 386-compatible port bridge and the direct
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

    input  logic        direct_launch,
    input  logic        direct_candidate,
    input  logic        direct_allowed,
    input  logic [10:0] direct_fop,
    output logic        direct_active,
    output logic        direct_mem_req,
    output logic        direct_stall,

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
logic        direct_ready;
logic        direct_issued_r;
logic        direct_crossing_r;
logic        direct_data_valid_r;
logic [31:0] direct_data_r;
wire         direct_valid = direct_active && direct_data_valid_r;
wire         direct_release = direct_valid && direct_ready;

assign direct_mem_req = direct_active && !direct_issued_r && !direct_data_valid_r;
assign direct_stall = direct_active && !direct_release;

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
        .direct_m32_valid(direct_valid), .direct_m32_fop(direct_fop),
        .direct_m32_data(direct_data_r), .direct_m32_ready(direct_ready),
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
    assign direct_ready = 1'b0;
    assign debug_state = 32'h8000_0000;
end
endgenerate

always_ff @(posedge clk) begin
    if (!reset_n) begin
        direct_active       <= 1'b0;
        direct_issued_r     <= 1'b0;
        direct_crossing_r   <= 1'b0;
        direct_data_valid_r <= 1'b0;
        direct_data_r       <= 32'h0;
    end else begin
        if (direct_release) begin
            direct_active       <= 1'b0;
            direct_issued_r     <= 1'b0;
            direct_data_valid_r <= 1'b0;
        end

        if (direct_launch) begin
            direct_active       <= ENABLE_X87 && direct_candidate &&
                                 direct_allowed && queue_safe;
            direct_issued_r     <= 1'b0;
            direct_crossing_r   <= 1'b0;
            direct_data_valid_r <= 1'b0;
        end

        if (direct_mem_req && mem_accepted) begin
            direct_issued_r   <= 1'b1;
            direct_crossing_r <= mem_addr_low != 2'b00;
        end

        if (direct_issued_r && !direct_crossing_r && mem_read_complete) begin
            direct_data_r       <= mem_rdata;
            direct_data_valid_r <= 1'b1;
        end else if (direct_issued_r && direct_crossing_r && !mem_servicing) begin
            direct_data_r       <= split_rdata;
            direct_data_valid_r <= 1'b1;
        end

        if (cancel) begin
            direct_active       <= 1'b0;
            direct_issued_r     <= 1'b0;
            direct_data_valid_r <= 1'b0;
        end
    end
end

endmodule
