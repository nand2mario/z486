module x87_sequencer
    import x87_ucode_pkg::*;
(
    input  logic          clk,
    input  logic          reset,
    input  logic          start,       // Launch entry; accepted only while executor is idle.
    input  logic    [7:0] entry,       // Operation-specific control-store entry.
    input  logic   [31:0] conditions,  // Predicates derived from registered executor state.
    output logic          active,      // Numeric microprogram owns the executor.
    output logic          exec_valid,  // Current uop may update its owned state.
    output logic          done,        // FINISH pulse; commit is carried by current uop.
    output logic    [7:0] uaddr,       // Current executing microcode address.
    output x87_uop_t      uop          // Current horizontal control word.
);

logic [7:0] fetch_addr;      // Address captured by synchronous control-store ROM.
logic [7:0] flow_addr;       // Next address selected from current registered state.
logic       condition_true;  // Predicate selected by the current uop.

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
