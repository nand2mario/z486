// Registered adapter between paging's preclassified x87 requests and streams.
module x87_bridge (
    input  logic        clk,
    input  logic        reset,

    input  logic        req_valid,          // Paging presents an x87 I/O cycle.
    input  logic        req_data_port,      // 0=f8 command/status; 1=fc operand data.
    input  logic        req_write,          // CPU-to-x87 when set.
    input  logic  [3:0] req_be,
    input  logic [31:0] req_wdata,
    output logic        req_accepted,
    output logic        req_complete,
    output logic        req_read_complete,
    output logic [31:0] req_rdata,

    output logic        cmd_valid,          // Registered command-stream request.
    output logic [10:0] cmd_fop,            // ESC opcode plus complete ModR/M FOP.
    input  logic        cmd_ready,

    output logic        word_in_valid,      // CPU supplies one operand/state fragment.
    output logic  [3:0] word_in_be,
    output logic [31:0] word_in_data,
    input  logic        word_in_ready,

    output logic        read_req_valid,     // CPU requests status or output fragment.
    output logic        read_req_data_port,
    output logic  [3:0] read_req_be,
    input  logic        read_req_ready,
    input  logic        read_resp_valid,
    input  logic [31:0] read_resp_data
);

typedef enum logic [2:0] {
    BR_IDLE,
    BR_DISPATCH,
    BR_READ_WAIT,
    BR_COMPLETE
} bridge_state_t;

bridge_state_t state;       // One registered CPU request from capture to completion.
logic          write_r;     // Captured request direction.
logic          data_port_r; // Captured f8/fc port selector.
logic [3:0]    be_r;        // Captured byte lanes for partial transfers.
logic [31:0]   wdata_r;     // Captured CPU write payload.
logic [31:0]   rdata_r;     // Registered CPU read response.

wire dispatch_ready = write_r ? (data_port_r ? word_in_ready : cmd_ready)
                              : read_req_ready;

always_comb begin
    req_accepted = (state == BR_IDLE) && req_valid;
    req_complete = (state == BR_COMPLETE);
    req_read_complete = req_complete && !write_r;
    req_rdata = rdata_r;

    cmd_valid = (state == BR_DISPATCH) && write_r && !data_port_r;
    cmd_fop = wdata_r[10:0];

    word_in_valid = (state == BR_DISPATCH) && write_r && data_port_r;
    word_in_be = be_r;
    word_in_data = wdata_r;

    read_req_valid = (state == BR_DISPATCH) && !write_r;
    read_req_data_port = data_port_r;
    read_req_be = be_r;
end

always_ff @(posedge clk) begin
    if (reset) begin
        state <= BR_IDLE;
        write_r <= 1'b0;
        data_port_r <= 1'b0;
        be_r <= 4'h0;
        wdata_r <= 32'h0;
        rdata_r <= 32'h0;
    end else begin
        case (state)
            BR_IDLE: begin
                if (req_valid) begin
                    write_r <= req_write;
                    data_port_r <= req_data_port;
                    be_r <= req_be;
                    wdata_r <= req_wdata;
                    state <= BR_DISPATCH;
                end
            end

            BR_DISPATCH: begin
                if (dispatch_ready)
                    state <= write_r ? BR_COMPLETE : BR_READ_WAIT;
            end

            BR_READ_WAIT: begin
                if (read_resp_valid) begin
                    rdata_r <= read_resp_data;
                    state <= BR_COMPLETE;
                end
            end

            BR_COMPLETE: state <= BR_IDLE;
            default: state <= BR_IDLE;
        endcase
    end
end

endmodule
