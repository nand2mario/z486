// Registered arctangent table for the serialized Q80 CORDIC. Quartus maps
// the constants to M10Ks; simulation uses the generated case table below.
module x87_cordic_rom (
    input  logic               clk,
    input  logic         [6:0] address,
    output logic signed [82:0] value
);

`ifdef ALTERA_RESERVED_QIS

logic [82:0] value_raw;
assign value = value_raw;

altsyncram #(
    .operation_mode("ROM"),
    .width_a(83),
    .widthad_a(7),
    .numwords_a(128),
    .outdata_reg_a("CLOCK0"),
    .address_aclr_a("NONE"),
    .outdata_aclr_a("NONE"),
    .init_file("x87_cordic_atan.mif"),
    .ram_block_type("M10K"),
    .intended_device_family("Cyclone V"),
    .lpm_type("altsyncram")
) atan_rom (
    .address_a(address),
    .clock0(clk),
    .clocken0(1'b1),
    .q_a(value_raw),
    .aclr0(1'b0),
    .addressstall_a(1'b0),
    .clocken1(1'b1),
    .clocken2(1'b1),
    .clocken3(1'b1),
    .rden_a(1'b1),
    .eccstatus()
);

`else

`include "x87_cordic_atan.svh"

always_ff @(posedge clk)
    value <= x87_cordic_atan_constant(address);

`endif

endmodule
