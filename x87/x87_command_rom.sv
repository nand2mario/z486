// Synchronous x87 command decoder. The generated image replaces repeated
// ESC/FOP comparisons in the control unit with a BRAM lookup.
module x87_command_rom
    import x87_ucode_pkg::*;
(
    input  logic                    clk,
    input  logic             [10:0] address, // Complete or canonicalized architectural FOP.
    output x87_command_decode_t     decode   // Registered command action and operands.
);

logic [22:0] raw_decode;
assign decode = x87_command_decode_t'(raw_decode);

`ifdef ALTERA_RESERVED_QIS

altsyncram #(
    .operation_mode("ROM"),
    .width_a(23),
    .widthad_a(11),
    .numwords_a(2048),
    .outdata_reg_a("UNREGISTERED"),
    .address_aclr_a("NONE"),
    .outdata_aclr_a("NONE"),
    .init_file("x87_command_decode.mif"),
    .ram_block_type("M10K"),
    .intended_device_family("Cyclone V"),
    .lpm_type("altsyncram")
) command_decode_rom (
    .address_a(address),
    .clock0(clk),
    .clocken0(1'b1),
    .q_a(raw_decode),
    .aclr0(1'b0),
    .addressstall_a(1'b0),
    .clocken1(1'b1),
    .clocken2(1'b1),
    .clocken3(1'b1),
    .rden_a(1'b1),
    .eccstatus()
);

`else

`include "x87_command_decode.svh"

always_ff @(posedge clk)
    raw_decode <= x87_command_decode_word(address);

`endif

endmodule
