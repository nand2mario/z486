// Shared three-word coprocessor transfer queue. Direction is owned by the
// command controller; the queue itself is a neutral ready/valid boundary.
module x87_transfer_fifo (
    input  logic        clk,
    input  logic        reset,
    input  logic        clear,       // Abort the current directional transfer.

    input  logic        push_valid,
    input  logic [35:0] push_data,   // {byte enables, 32-bit transfer word}.
    output logic        push_ready,

    output logic        pop_valid,
    output logic [35:0] pop_data,    // Oldest byte-qualified transfer word.
    input  logic        pop_ready,

    output logic [1:0]  count
);

logic [35:0] words [0:2]; // One maximum-width 80-bit operand plus byte enables.
logic [1:0] read_ptr;     // Wraps modulo three.
logic [1:0] write_ptr;    // Wraps modulo three.
logic       push_fire;
logic       pop_fire;

function automatic logic [1:0] next_ptr(input logic [1:0] pointer);
    return pointer == 2'd2 ? 2'd0 : pointer + 2'd1;
endfunction

assign pop_valid = count != 2'd0;
assign pop_data = words[read_ptr];
assign pop_fire = pop_valid && pop_ready;
// A dequeue frees its slot at the active edge, including when the queue was
// full, so a replacement word may be accepted in the same cycle.
assign push_ready = (count != 2'd3) || pop_fire;
assign push_fire = push_valid && push_ready;

always_ff @(posedge clk) begin
    if (reset || clear) begin
        read_ptr <= 2'd0;
        write_ptr <= 2'd0;
        count <= 2'd0;
    end else begin
        if (push_fire) begin
            words[write_ptr] <= push_data;
            write_ptr <= next_ptr(write_ptr);
        end
        if (pop_fire)
            read_ptr <= next_ptr(read_ptr);

        case ({push_fire, pop_fire})
            2'b10: count <= count + 2'd1;
            2'b01: count <= count - 2'd1;
            default: count <= count;
        endcase
    end
end

endmodule
