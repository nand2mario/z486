module z486_led_demo_soc #(
    parameter [6:0] CLOCK_RATE_MHZ = 7'd50,
    parameter ENABLE_X87 = 1'b0,
    parameter FIRMWARE_HEX = "boards/firmware/blink.hex",
    parameter [15:0] LED_IO_PORT = 16'h0080
) (
    input  logic        clk,
    input  logic        reset_n,
    output logic [7:0]  led,
    output logic        firmware_seen,
    output logic        led_write_pulse,
    output logic [7:0]  led_write_data,
    output logic        triple_fault,
    output logic [15:0] dbg_cs,
    output logic [31:0] dbg_eip
);

logic [31:2] cpu_addr;
logic  [3:0] cpu_be;
logic  [7:0] cpu_burstcount;
logic [31:0] cpu_din;
logic [31:0] cpu_dout;
logic        cpu_valid;
logic        cpu_ready;
logic        cpu_write;
logic        cpu_io;
logic        cpu_resp_valid;
logic        cpu_inta;

logic        read_active;
logic [7:0]  read_beats_left;
logic [13:0] read_word;
logic        read_is_rom;
logic [31:0] rom_q;
logic [7:0]  led_latch;

wire [31:0] cpu_byte_addr = {cpu_addr, 2'b00};
wire request_is_rom = !cpu_io &&
                      ((cpu_byte_addr[31:16] == 16'hffff) ||
                       (cpu_byte_addr[31:16] == 16'h000f));
wire request_is_led = cpu_io &&
                      (cpu_byte_addr[15:2] == LED_IO_PORT[15:2]);

assign cpu_ready = !read_active;
assign cpu_din = read_is_rom ? rom_q : 32'hffff_ffff;
assign led = triple_fault ? 8'he7 : (firmware_seen ? led_latch : 8'h81);

function automatic logic [7:0] selected_byte(
    input logic [31:0] data,
    input logic  [3:0] byte_enable
);
    casez (byte_enable)
        4'b???1: selected_byte = data[7:0];
        4'b??10: selected_byte = data[15:8];
        4'b?100: selected_byte = data[23:16];
        4'b1000: selected_byte = data[31:24];
        default: selected_byte = data[7:0];
    endcase
endfunction

z486_demo_rom #(
    .INIT_HEX(FIRMWARE_HEX)
) firmware_rom (
    .clk(clk),
    .address(read_word),
    .q(rom_q)
);

always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        cpu_resp_valid <= 1'b0;
        read_active <= 1'b0;
        read_beats_left <= 8'd0;
        read_word <= 14'd0;
        read_is_rom <= 1'b0;
        led_latch <= 8'h81;
        firmware_seen <= 1'b0;
        led_write_pulse <= 1'b0;
        led_write_data <= 8'd0;
    end else begin
        cpu_resp_valid <= 1'b0;
        led_write_pulse <= 1'b0;

        if (cpu_valid && cpu_ready) begin
            if (cpu_write) begin
                if (request_is_led) begin
                    led_latch <= selected_byte(cpu_dout, cpu_be);
                    led_write_data <= selected_byte(cpu_dout, cpu_be);
                    led_write_pulse <= 1'b1;
                    firmware_seen <= 1'b1;
                end
            end else begin
                read_active <= 1'b1;
                read_beats_left <= (cpu_burstcount == 0) ? 8'd1
                                                         : cpu_burstcount;
                read_word <= cpu_addr[15:2];
                read_is_rom <= request_is_rom;
            end
        end

        if (read_active) begin
            cpu_resp_valid <= 1'b1;
            read_word <= read_word + 14'd1;
            if (read_beats_left == 8'd1) begin
                read_active <= 1'b0;
                read_beats_left <= 8'd0;
            end else begin
                read_beats_left <= read_beats_left - 8'd1;
            end
        end
    end
end

z486 #(
    .ENABLE_X87(ENABLE_X87),
    .CLOCK_RATE_MHZ(CLOCK_RATE_MHZ)
) cpu (
    .clk(clk),
    .reset_n(reset_n),
    .addr(cpu_addr),
    .be(cpu_be),
    .burstcount(cpu_burstcount),
    .line_read(),
    .din(cpu_din),
    .line_din(128'd0),
    .dout(cpu_dout),
    .valid(cpu_valid),
    .ready(cpu_ready),
    .write(cpu_write),
    .io(cpu_io),
    .resp_valid(cpu_resp_valid),
    .line_resp_valid(1'b0),
    .intr(1'b0),
    .nmi(1'b0),
    .inta(cpu_inta),
    .x87_ferr(),
    .x87_ignne(1'b0),
    .snoop_addr(32'd0),
    .snoop_valid(1'b0),
    .a20_enable(1'b1),
    .cpu_speed_sel(2'd0),
    .fast_off_req(1'b0),
    .cache_off_req(1'b0),
    .x87_off_req(1'b0),
    .single_step(1'b0),
    .dbg_CS(dbg_cs),
    .dbg_EIP(dbg_eip),
    .dbg_CS_base(),
    .dbg_pe(),
    .dbg_vm(),
    .dbg_x87_state(),
    .triple_fault_reset(triple_fault)
);

endmodule
