// Synchronous x87 control store. The generated image is the source
// of truth; simulation and Quartus consume equivalent generated forms.
module x87_ucode_rom
    import x87_ucode_pkg::*;
(
    input  logic          clk,
    input  logic    [7:0] address,
    output x87_uop_t   uop
);

logic [63:0] raw_uop;
assign uop = x87_uop_t'(raw_uop);

`ifdef ALTERA_RESERVED_QIS

altsyncram #(
    .operation_mode("ROM"),
    .width_a(64),
    .widthad_a(8),
    .numwords_a(256),
    .outdata_reg_a("CLOCK0"),
    .address_aclr_a("NONE"),
    .outdata_aclr_a("NONE"),
    .init_file("x87_ucode.mif"),
    .ram_block_type("M10K"),
    .intended_device_family("Cyclone V"),
    .lpm_type("altsyncram")
) control_store (
    .address_a(address),
    .clock0(clk),
    .clocken0(1'b1),
    .q_a(raw_uop),
    .aclr0(1'b0),
    .addressstall_a(1'b0),
    .clocken1(1'b1),
    .clocken2(1'b1),
    .clocken3(1'b1),
    .rden_a(1'b1),
    .eccstatus()
);

`else

`include "x87_ucode.svh"

always_ff @(posedge clk)
    raw_uop <= x87_ucode_word(address);

`endif

endmodule
