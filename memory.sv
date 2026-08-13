// z386x cache and external-memory fabric.
// Owns both L1 caches, A20 masking, refill arbitration, and response tracking.
module memory #(
    parameter PROTECT_UMA_ROM = 0,
    parameter DCACHE_SET_BITS = 8,
    parameter ICACHE_SET_BITS = 8,
    parameter ENABLE_X87 = 0
) (
    input              clk,
    input              reset_n,
    input              a20_enable,

    // Paging-unit demand request
    input              dcache_req_valid,
    input      [31:0]  dcache_req_phys_addr_raw,
    input              dcache_req_write,
    input       [3:0]  dcache_req_be,
    input      [31:0]  dcache_req_wdata,
    input              dcache_req_is_io,
    input              dcache_req_is_inta,
    input              dcache_req_is_x87,
    input              dcache_req_is_vga_mem,
    output             dcache_req_accepted, // Request ownership transferred
    output             dcache_req_complete, // Read or write operation completed
    output             dcache_read_complete, // Read data is valid this cycle
    output     [31:0]  dcache_rdata,

    // x87 data-port response
    output             x87_req_selected,    // Route demand request to x87 port space
    input              x87_req_accepted,
    input              x87_req_complete,
    input              x87_read_complete,
    input      [31:0]  x87_rdata,

    // Paging-unit instruction request
    input              icache_req_valid,
    input      [31:0]  icache_req_phys_addr_raw,
    output             icache_req_accepted, // Prefetch request ownership transferred
    output             icache_req_complete, // Full instruction line is valid
    output     [127:0] icache_rdata,

    // External memory writers invalidate both caches.
    input      [31:0]  snoop_addr,
    input              snoop_valid,          // External writer invalidates this line

    // External memory bus
    output     [31:2]  addr,
    output      [3:0]  be,
    output      [7:0]  burstcount,
    input      [31:0]  din,
    output     [31:0]  dout,
    output             valid,                // External request is present
    input              ready,                // External request was accepted
    output             write,
    output             io,
    input              resp_valid,           // External read beat returned
    output             inta
);

wire [31:0] dcache_req_phys_addr = (!a20_enable && !dcache_req_is_io)
                                      ? (dcache_req_phys_addr_raw & ~32'h0010_0000)
                                      : dcache_req_phys_addr_raw;
wire [31:0] icache_req_phys_addr = !a20_enable
                                      ? (icache_req_phys_addr_raw & ~32'h0010_0000)
                                      : icache_req_phys_addr_raw;

wire [31:0] dcache_cpu_dout;
wire        dcache_cpu_ready;
wire        dcache_cpu_resp_valid;
wire [31:0] dcache_mem_addr;
wire [31:0] dcache_mem_din;
wire  [3:0] dcache_mem_be;
wire  [7:0] dcache_mem_burstcount;
wire        dcache_mem_valid;
wire        dcache_mem_write;
wire        dcache_mem_ready;
wire        dcache_mem_resp_valid;

wire [127:0] icache_cpu_line;
wire         icache_cpu_ready;
wire         icache_cpu_resp_valid;
wire [31:0]  icache_mem_addr;
wire  [3:0]  icache_mem_be;
wire  [7:0]  icache_mem_burstcount;
wire         icache_mem_valid;
wire         icache_mem_ready;
wire         icache_mem_resp_valid;

logic [7:0] dcache_rd_pending;
logic [7:0] icache_rd_pending;
logic       dcache_cpu_rd_pending;
logic       icache_cpu_rd_pending;
logic       direct_rd_pending;

assign x87_req_selected = ENABLE_X87 && dcache_req_valid && dcache_req_is_x87;

// VGA aperture accesses are device transactions. Bypass the posted L1 store
// queue so an ET4000 bank-register write cannot overtake framebuffer writes.
wire dcache_cpu_req = dcache_req_valid && !dcache_req_is_io &&
                      !dcache_req_is_inta && !dcache_req_is_vga_mem &&
                      !x87_req_selected;
wire dcache_direct_req = dcache_req_valid && !x87_req_selected &&
                         (dcache_req_is_io || dcache_req_is_inta ||
                          dcache_req_is_vga_mem);
wire dcache_read_pending = (dcache_rd_pending != 8'd0);
wire icache_read_pending = (icache_rd_pending != 8'd0);
wire dcache_read_accept = dcache_cpu_req && !dcache_req_write && dcache_cpu_ready;
wire icache_read_accept = icache_req_valid && icache_cpu_ready;
wire dcache_read_done = dcache_cpu_resp_valid &&
                        (dcache_cpu_rd_pending || dcache_read_accept);
wire icache_read_done = icache_cpu_resp_valid &&
                        (icache_cpu_rd_pending || icache_read_accept);
wire ext_direct_req = dcache_direct_req && !direct_rd_pending &&
                      !dcache_read_pending && !icache_read_pending;
wire ext_dcache_req = dcache_mem_valid && !ext_direct_req &&
                      !direct_rd_pending && !icache_read_pending;
wire ext_icache_req = icache_mem_valid && !ext_direct_req && !ext_dcache_req &&
                      !direct_rd_pending && !dcache_read_pending;

localparam [1:0] EXT_SRC_NONE   = 2'd0;
localparam [1:0] EXT_SRC_DIRECT = 2'd1;
localparam [1:0] EXT_SRC_DCACHE = 2'd2;
localparam [1:0] EXT_SRC_ICACHE = 2'd3;

logic        ext_valid_r;
logic [1:0]  ext_src_r;
logic [31:2] ext_addr_r;
logic [3:0]  ext_be_r;
logic [7:0]  ext_burstcount_r;
logic [31:0] ext_dout_r;
logic        ext_write_r;
logic        ext_io_r;
logic        ext_inta_r;

wire ext_direct_accept = ext_valid_r && ready && (ext_src_r == EXT_SRC_DIRECT);
wire ext_dcache_accept = ext_valid_r && ready && (ext_src_r == EXT_SRC_DCACHE);
wire ext_icache_accept = ext_valid_r && ready && (ext_src_r == EXT_SRC_ICACHE);
wire direct_rd_resp_now = resp_valid &&
                          (direct_rd_pending || (ext_direct_accept && !ext_write_r));
wire icache_write_snoop = dcache_cpu_req && dcache_req_write && dcache_cpu_ready;
logic       icache_write_snoop_pending;
logic [31:0] icache_write_snoop_addr_r;
logic [31:0] icache_write_snoop_data_r;
logic  [3:0] icache_write_snoop_be_r;

wire normal_req_accepted = dcache_cpu_req ? dcache_cpu_ready : ext_direct_accept;
wire normal_req_complete = dcache_cpu_resp_valid ||
                           (dcache_cpu_req && dcache_req_write && dcache_cpu_ready) ||
                           direct_rd_resp_now ||
                           (ext_direct_accept && ext_write_r);
wire normal_read_complete = dcache_cpu_resp_valid || direct_rd_resp_now;
wire [31:0] normal_rdata = dcache_cpu_resp_valid ? dcache_cpu_dout : din;

assign dcache_req_accepted = x87_req_selected ? x87_req_accepted : normal_req_accepted;
assign dcache_req_complete = normal_req_complete || x87_req_complete;
assign dcache_read_complete = normal_read_complete || x87_read_complete;
assign dcache_rdata = x87_read_complete ? x87_rdata : normal_rdata;
assign icache_req_accepted = icache_cpu_ready;
assign icache_req_complete = icache_cpu_resp_valid;
assign icache_rdata = icache_cpu_line;

assign addr       = ext_addr_r;
assign be         = ext_be_r;
assign burstcount = ext_burstcount_r;
assign dout       = ext_dout_r;
assign valid      = ext_valid_r;
assign write      = ext_valid_r && ext_write_r;
assign io         = ext_valid_r && ext_io_r;
assign inta       = ext_valid_r && ext_inta_r;

assign dcache_mem_ready = ext_dcache_accept;
assign icache_mem_ready = ext_icache_accept;
assign dcache_mem_resp_valid = dcache_read_pending && resp_valid;
assign icache_mem_resp_valid = icache_read_pending && resp_valid;

always_ff @(posedge clk) begin
    if (!reset_n) begin
        ext_valid_r <= 1'b0;
        ext_src_r <= EXT_SRC_NONE;
        ext_addr_r <= 30'h0;
        ext_be_r <= 4'h0;
        ext_burstcount_r <= 8'h0;
        ext_dout_r <= 32'h0;
        ext_write_r <= 1'b0;
        ext_io_r <= 1'b0;
        ext_inta_r <= 1'b0;
        dcache_rd_pending <= 8'd0;
        icache_rd_pending <= 8'd0;
        dcache_cpu_rd_pending <= 1'b0;
        icache_cpu_rd_pending <= 1'b0;
        direct_rd_pending <= 1'b0;
        icache_write_snoop_pending <= 1'b0;
        icache_write_snoop_addr_r <= 32'h0;
        icache_write_snoop_data_r <= 32'h0;
        icache_write_snoop_be_r <= 4'h0;
    end else begin
        if (icache_write_snoop) begin
            // Keep dcache write finalization off the icache RAM write path.
            icache_write_snoop_pending <= 1'b1;
            icache_write_snoop_addr_r <= dcache_req_phys_addr;
            icache_write_snoop_data_r <= dcache_req_wdata;
            icache_write_snoop_be_r <= dcache_req_be;
        end else if (icache_write_snoop_pending && !snoop_valid) begin
            icache_write_snoop_pending <= 1'b0;
        end

        if (ext_valid_r) begin
            if (ready) begin
                ext_valid_r <= 1'b0;
                ext_src_r <= EXT_SRC_NONE;
            end
        end else if (ext_direct_req) begin
            ext_valid_r <= 1'b1;
            ext_src_r <= EXT_SRC_DIRECT;
            ext_addr_r <= dcache_req_phys_addr[31:2];
            ext_be_r <= dcache_req_be;
            ext_burstcount_r <= 8'd1;
            if (dcache_req_write)
                ext_dout_r <= dcache_req_wdata;
            ext_write_r <= dcache_req_write;
            ext_io_r <= dcache_req_is_io;
            ext_inta_r <= dcache_req_is_inta;
        end else if (ext_dcache_req) begin
            ext_valid_r <= 1'b1;
            ext_src_r <= EXT_SRC_DCACHE;
            ext_addr_r <= dcache_mem_addr[31:2];
            ext_be_r <= dcache_mem_be;
            ext_burstcount_r <= dcache_mem_burstcount;
            if (dcache_mem_write)
                ext_dout_r <= dcache_mem_din;
            ext_write_r <= dcache_mem_write;
            ext_io_r <= 1'b0;
            ext_inta_r <= 1'b0;
        end else if (ext_icache_req) begin
            ext_valid_r <= 1'b1;
            ext_src_r <= EXT_SRC_ICACHE;
            ext_addr_r <= icache_mem_addr[31:2];
            ext_be_r <= icache_mem_be;
            ext_burstcount_r <= icache_mem_burstcount;
            ext_dout_r <= 32'h0;
            ext_write_r <= 1'b0;
            ext_io_r <= 1'b0;
            ext_inta_r <= 1'b0;
        end

        if (ext_dcache_accept && !dcache_mem_write)
            dcache_rd_pending <= dcache_mem_burstcount;
        else if (dcache_read_pending && resp_valid)
            dcache_rd_pending <= dcache_rd_pending - 8'd1;

        if (ext_icache_accept)
            icache_rd_pending <= icache_mem_burstcount;
        else if (icache_read_pending && resp_valid)
            icache_rd_pending <= icache_rd_pending - 8'd1;

        if (dcache_read_accept && !dcache_read_done)
            dcache_cpu_rd_pending <= 1'b1;
        else if (dcache_read_done)
            dcache_cpu_rd_pending <= 1'b0;

        if (icache_read_accept && !icache_read_done)
            icache_cpu_rd_pending <= 1'b1;
        else if (icache_read_done)
            icache_cpu_rd_pending <= 1'b0;

        if (ext_direct_accept && !ext_write_r && !resp_valid)
            direct_rd_pending <= 1'b1;
        else if (direct_rd_pending && resp_valid)
            direct_rd_pending <= 1'b0;
    end
end

l1_cache #(
    .PROTECT_UMA_ROM(PROTECT_UMA_ROM),
    .SET_BITS(DCACHE_SET_BITS)
) dcache_inst (
    .clk(clk),
    .reset(!reset_n),
    .cpu_addr(dcache_req_phys_addr),
    .cpu_din(dcache_req_wdata),
    .cpu_dout(dcache_cpu_dout),
    .cpu_be(dcache_req_be),
    .cpu_valid(dcache_cpu_req),
    .cpu_write(dcache_req_write),
    .cpu_ready(dcache_cpu_ready),
    .cpu_resp_valid(dcache_cpu_resp_valid),
    .mem_addr(dcache_mem_addr),
    .mem_din(dcache_mem_din),
    .mem_dout(din),
    .mem_be(dcache_mem_be),
    .mem_burstcount(dcache_mem_burstcount),
    .mem_busy(ext_valid_r || direct_rd_pending || icache_read_pending),
    .mem_valid(dcache_mem_valid),
    .mem_write(dcache_mem_write),
    .mem_ready(dcache_mem_ready),
    .mem_resp_valid(dcache_mem_resp_valid),
    .snoop_addr(snoop_addr),
    .snoop_valid(snoop_valid),
    .cache_enable(1'b1)
);

l1_icache #(
    .SET_BITS(ICACHE_SET_BITS)
) icache_inst (
    .clk(clk),
    .reset(!reset_n),
    .cpu_addr(icache_req_phys_addr),
    .cpu_line(icache_cpu_line),
    .cpu_valid(icache_req_valid),
    .cpu_ready(icache_cpu_ready),
    .cpu_resp_valid(icache_cpu_resp_valid),
    .mem_addr(icache_mem_addr),
    .mem_dout(din),
    .mem_be(icache_mem_be),
    .mem_burstcount(icache_mem_burstcount),
    .mem_busy(ext_valid_r || direct_rd_pending || dcache_read_pending ||
              dcache_mem_valid),
    .mem_valid(icache_mem_valid),
    .mem_ready(icache_mem_ready),
    .mem_resp_valid(icache_mem_resp_valid),
    .patch_addr(icache_write_snoop_addr_r),
    .patch_data(icache_write_snoop_data_r),
    .patch_be(icache_write_snoop_be_r),
    .patch_valid(icache_write_snoop_pending),
    .invalidate_addr(snoop_addr),
    .invalidate_valid(snoop_valid),
    .cache_enable(1'b1)
);

endmodule
