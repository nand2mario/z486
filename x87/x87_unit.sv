//
// Floating-Point Unit
// CPU-facing x87 top, using the 387 I/O port protocol (0xF8/0xFC)
//
module x87_unit #(
    parameter ENABLE_X87 = 0
)(
    input  logic        clk,
    input  logic        reset_n,

    input  logic        req_valid,          // Paging presents an x87 I/O cycle.
    input  logic        req_data_port,      // 0=f8 command/status; 1=fc operand data.
    input  logic        req_write,          // Direction of the CPU-side cycle.
    input  logic [3:0]  req_be,
    input  logic [31:0] req_wdata,
    output logic        req_accepted,       // Bridge captured the request.
    output logic        req_complete,       // Bridge completed the protocol action.
    output logic        req_read_complete,
    output logic [31:0] req_rdata,

    input  logic        direct_launch,      // Instruction issue checks the m32 overlay.
    input  logic        direct_candidate,   // Recipe is an eligible m32 x87 form.
    input  logic        direct_allowed,     // CR0 and runtime policy permit x87 use.
    input  logic [10:0] direct_fop,         // FOP of the executing instruction.
    input  logic        direct_reg,         // The issuing x87 form is a register form.
    input  logic        direct_store,       // ...an m32 store (FST/FSTP/FIST/FISTP m32).
    input  logic        direct_data32,      // ...in 32-bit operand size.
    input  logic        store_word,         // The store overlay's command/read word is current.
    input  logic        store_go,           // ...and its destination check completed (no memory stall).
    output logic        store_opr_commit,   // The store result arrived: write OPR_R.
    output logic        store_hold,         // Hold the ROM at the store word until then.
    output logic [31:0] store_opr_data,
    output logic        direct_taken,       // The issued x87 overlay went direct.
    output logic        direct_reg_taken,   // ...as a register form (no operand).
    output logic        direct_active,      // One direct transport owns instruction issue.
    output logic        direct_mem_req,     // Launch fault-checked demand read through paging.
    output logic        direct_stall,       // Hold retirement until control accepts the operand.

    input  logic        mem_accepted,
    input  logic [1:0]  mem_addr_low,
    input  logic        mem_read_complete,
    input  logic        mem_servicing,
    input  logic [31:0] mem_rdata,
    input  logic [31:0] split_rdata,
    input  logic        cancel,             // Fault, interrupt, or frontend squash.

    output logic        busy_n,
    output logic        pereq,
    output logic        error_n,
    output logic [31:0] debug_state
);

logic        queue_safe;           // Masked-exception mode permits one queued command.
logic        direct_ready;         // x87_control can capture the direct FOP/data pair.
logic        direct_issued_r;      // Demand read has crossed the paging handshake.
logic        direct_crossing_r;    // Dword spans two naturally aligned memory words.
logic        direct_data_valid_r;  // Complete fault-checked operand is buffered.
logic [31:0] direct_data_r;        // Buffered direct m32 operand.
wire         direct_valid = direct_active && direct_data_valid_r;
// Register forms: the FOP is posted to the bridge as a command write, in the
// executing instruction's first cycle. Taken only when the 386 routine would
// post it at once: CR0 allows x87 use, no unmasked exception is pending, the
// x87 is idle or may queue one command, and the bridge is free (no posted or
// direct command still waiting), so the bridge accepts it immediately.
logic        direct_reg_r;         // The issued instruction took the register path.
logic        dcmd_pending_r;       // ...and its command awaits bridge acceptance.
logic        posted_pending;       // The bridge holds a posted write.
logic        br_accepted, br_complete;
logic        br_read_complete;
logic [31:0] br_rdata;
// m32 stores: in the store word, post the FOP, then read the result from the
// data port; the sequencer holds in that word until it arrives.
typedef enum logic [1:0] {ST_IDLE, ST_CMD, ST_READ, ST_DONE} st_state_t;
st_state_t   st_state;
logic        direct_st_r;          // The issued instruction took the store path.
wire         st_cmd = direct_st_r && store_go && !cancel &&
                      (st_state == ST_IDLE || st_state == ST_CMD);
wire         st_read_req = (st_state == ST_READ);
logic        st_read_issued_r;     // The bridge accepted the result read.
wire         dcmd_sel = dcmd_pending_r || st_cmd || st_read_req;
wire         own_cmd = dcmd_pending_r || st_cmd;     // a command write (F8)
wire         own_read = st_read_req && !st_read_issued_r;
assign direct_taken = direct_active || direct_reg_r || direct_st_r;
assign direct_reg_taken = direct_reg_r;
wire         direct_release = direct_valid && direct_ready;

assign direct_mem_req = direct_active && !direct_issued_r && !direct_data_valid_r;
assign store_hold = direct_st_r && store_word && (st_state != ST_DONE);
assign direct_stall = (direct_active && !direct_release) || store_hold;

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
    wire        ctl_busy_n, ctl_pereq;
    // A posted port write is the x87's as soon as the bridge accepts it: until
    // it dispatches, report BUSY# and PEREQ as control does for an accepted
    // command, so the coprocessor-wait microcode samples a settled protocol.
    assign busy_n = ctl_busy_n && !posted_pending;
    assign pereq  = ctl_pereq || posted_pending;

    // The direct command has the bridge to itself: no ESC routine (the only
    // source of paging's x87 cycles) runs while it waits. Paging sees neither
    // its acceptance nor its completion.
    assign req_accepted = br_accepted && !dcmd_sel;
    assign req_complete = br_complete && !dcmd_sel;
    assign req_read_complete = br_read_complete && !dcmd_sel;
    assign req_rdata = br_rdata;
    assign store_opr_commit = st_read_req && br_read_complete;
    assign store_opr_data = br_rdata;
    x87_bridge bridge (
        .clk(clk), .reset(!reset_n),
        .req_valid(own_cmd || own_read || (!dcmd_sel && req_valid)),
        .req_data_port(own_cmd ? 1'b0 : own_read ? 1'b1 : req_data_port),
        .req_write(own_cmd || (!dcmd_sel && req_write)),
        .req_be(own_cmd ? 4'h3 : own_read ? 4'hF : req_be),
        .req_wdata(own_cmd ? {21'd0, direct_fop} : req_wdata),
        .req_accepted(br_accepted), .req_complete(br_complete),
        .req_read_complete(br_read_complete), .req_rdata(br_rdata),
        .posted_pending(posted_pending),
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
        .busy_n(ctl_busy_n), .pereq(ctl_pereq), .error_n(error_n),
        .queue_safe(queue_safe), .debug_state(debug_state)
    );
end else begin : gen_no_x87
    assign req_accepted = 1'b0;
    assign req_complete = 1'b0;
    assign store_opr_commit = 1'b0;
    assign store_opr_data = 32'h0;
    assign br_accepted = 1'b0;
    assign br_read_complete = 1'b0;
    assign br_rdata = 32'h0;
    assign br_complete = 1'b0;
    assign posted_pending = 1'b0;
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
        direct_st_r         <= 1'b0;
        st_state            <= ST_IDLE;
        st_read_issued_r    <= 1'b0;
        direct_reg_r        <= 1'b0;
        dcmd_pending_r      <= 1'b0;
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

        if (dcmd_pending_r && br_accepted)
            dcmd_pending_r <= 1'b0;

        case (st_state)
            ST_IDLE: if (st_cmd) st_state <= br_accepted ? ST_READ : ST_CMD;
            ST_CMD:  if (br_accepted) st_state <= ST_READ;
            ST_READ: begin
                if (own_read && br_accepted) st_read_issued_r <= 1'b1;
                if (br_read_complete) begin
                    st_state <= ST_DONE;
                    st_read_issued_r <= 1'b0;
                end
            end
            ST_DONE: if (!store_word) st_state <= ST_IDLE;
        endcase

        if (direct_launch) begin
            direct_active       <= ENABLE_X87 && direct_candidate && !direct_reg &&
                                 !direct_store && direct_allowed && queue_safe;
            direct_st_r         <= ENABLE_X87 && direct_candidate && direct_store && direct_data32 &&
                                 direct_allowed && error_n && (busy_n || queue_safe) &&
                                 !posted_pending && !dcmd_pending_r && (st_state == ST_IDLE);
            direct_reg_r        <= ENABLE_X87 && direct_candidate && direct_reg &&
                                 direct_allowed && error_n && (busy_n || queue_safe) &&
                                 !posted_pending && !dcmd_pending_r;
            dcmd_pending_r      <= ENABLE_X87 && direct_candidate && direct_reg &&
                                 direct_allowed && error_n && (busy_n || queue_safe) &&
                                 !posted_pending && !dcmd_pending_r;
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
            direct_st_r         <= 1'b0;
            st_state            <= ST_IDLE;
            st_read_issued_r    <= 1'b0;
            direct_reg_r        <= 1'b0;
            dcmd_pending_r      <= 1'b0;
            direct_active       <= 1'b0;
            direct_issued_r     <= 1'b0;
            direct_data_valid_r <= 1'b0;
        end
    end
end

`ifdef Z486_COSIM
// Co-simulation: the x87 has no command, transfer or result in flight, so its
// architectural state covers every ESC instruction the CPU has issued.
export "DPI-C" function cosim_x87_idle;
function int cosim_x87_idle();
    return int'(busy_n && !pereq && !dcmd_pending_r && (st_state == ST_IDLE));
endfunction
`endif

endmodule
