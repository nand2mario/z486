// Two mirrored memories implement a two-read, one-write scratch file. The
// synchronous read boundary keeps scratch data out of sequencer address logic.
module x87_cordic_scratch (
    input  logic        clk,
    input  logic  [3:0] read_addr_a,
    output logic [27:0] read_data_a,
    input  logic  [3:0] read_addr_b,
    output logic [27:0] read_data_b,
    input  logic        write_enable,
    input  logic  [3:0] write_addr,
    input  logic [27:0] write_data
);

(* ramstyle = "M10K, no_rw_check" *) logic [27:0] words_a [0:15];
(* ramstyle = "M10K, no_rw_check" *) logic [27:0] words_b [0:15];

always_ff @(posedge clk) begin
    if (write_enable) begin
        words_a[write_addr] <= write_data;
        words_b[write_addr] <= write_data;
    end
    read_data_a <= words_a[read_addr_a];
    read_data_b <= words_b[read_addr_b];
end

endmodule
