module x87_sequencer
    import x87_ucode_pkg::*;
(
    input  logic          clk,
    input  logic          reset,
    input  logic          start,
    input  logic    [7:0] entry,
    input  logic   [31:0] conditions,
    output logic          active,
    output logic          exec_valid,
    output logic          done,
    output logic    [7:0] uaddr,
    output x87_uop_t   uop
);

logic [7:0] fetch_addr;
logic [7:0] flow_addr;
logic       condition_true;

x87_ucode_rom control_store (
    .clk(clk),
    .address(fetch_addr),
    .uop(uop)
);

assign exec_valid = active;
assign condition_true = conditions[uop.condition];

always_comb begin
    flow_addr = uaddr + 8'd1;
    case (uop.flow)
        X87_FLOW_JUMP:
            flow_addr = uop.target;
        X87_FLOW_BRANCH,
        X87_FLOW_LOOP:
            flow_addr = condition_true ? uop.target : uaddr + 8'd1;
        X87_FLOW_WAIT:
            flow_addr = condition_true ? uaddr + 8'd1 : uaddr;
        default:
            flow_addr = uaddr + 8'd1;
    endcase

    fetch_addr = start ? entry : flow_addr;
end

always_ff @(posedge clk) begin
    if (reset) begin
        active <= 1'b0;
        done <= 1'b0;
        uaddr <= 8'h00;
    end else begin
        done <= 1'b0;
        if (start) begin
            active <= 1'b1;
            uaddr <= entry;
        end else if (active) begin
            if (uop.flow == X87_FLOW_FINISH) begin
                active <= 1'b0;
                done <= 1'b1;
            end else begin
                uaddr <= flow_addr;
            end
        end
    end
end

endmodule
