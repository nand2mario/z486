// Two synchronous ports store the eight architectural 80-bit x87 registers.
// Decode happens at the consumer so untouched m80 values round-trip exactly.
module x87_stack_mem (
    input  logic        clk,

    input  logic [2:0]  addr_a,
    input  logic        write_a,
    input  logic [79:0] write_data_a,
    output logic [79:0] read_data_a,

    input  logic [2:0]  addr_b,
    input  logic        write_b,
    input  logic [79:0] write_data_b,
    output logic [79:0] read_data_b
);

`ifdef ALTERA_RESERVED_QIS

altsyncram #(
    .address_reg_b("CLOCK0"),
    .clock_enable_input_a("BYPASS"),
    .clock_enable_input_b("BYPASS"),
    .clock_enable_output_a("BYPASS"),
    .clock_enable_output_b("BYPASS"),
    .indata_reg_b("CLOCK0"),
    .intended_device_family("Cyclone V"),
    .lpm_type("altsyncram"),
    .numwords_a(8),
    .numwords_b(8),
    .operation_mode("BIDIR_DUAL_PORT"),
    .outdata_aclr_a("NONE"),
    .outdata_aclr_b("NONE"),
    .outdata_reg_a("UNREGISTERED"),
    .outdata_reg_b("UNREGISTERED"),
    .power_up_uninitialized("TRUE"),
    .ram_block_type("M10K"),
    .read_during_write_mode_port_a("NEW_DATA_NO_NBE_READ"),
    .read_during_write_mode_port_b("NEW_DATA_NO_NBE_READ"),
    .read_during_write_mode_mixed_ports("DONT_CARE"),
    .width_a(80),
    .width_b(80),
    .widthad_a(3),
    .widthad_b(3),
    .width_byteena_a(1),
    .width_byteena_b(1),
    .wrcontrol_wraddress_reg_b("CLOCK0")
) ram (
    .clock0(clk),
    .address_a(addr_a),
    .address_b(addr_b),
    .data_a(write_data_a),
    .data_b(write_data_b),
    .wren_a(write_a),
    .wren_b(write_b),
    .q_a(read_data_a),
    .q_b(read_data_b),
    .aclr0(1'b0),
    .aclr1(1'b0),
    .addressstall_a(1'b0),
    .addressstall_b(1'b0),
    .byteena_a(1'b1),
    .byteena_b(1'b1),
    .clocken0(1'b1),
    .clocken1(1'b1),
    .clocken2(1'b1),
    .clocken3(1'b1),
    .eccstatus(),
    .rden_a(1'b1),
    .rden_b(1'b1)
);

`else

logic [79:0] mem [0:7];

always_ff @(posedge clk) begin
    if (write_a) begin
        mem[addr_a] <= write_data_a;
        read_data_a <= write_data_a;
    end else begin
        read_data_a <= mem[addr_a];
    end
    if (write_b) begin
        mem[addr_b] <= write_data_b;
        read_data_b <= write_data_b;
    end else begin
        read_data_b <= mem[addr_b];
    end
end

`endif

endmodule
