//
// Cache Unit
// A20 masking, the instruction and data caches, the VIPT load port and request steering
//
module cache_unit
    import z486_pkg::*;
#(
    parameter PROTECT_UMA_ROM = 0,
    parameter ENABLE_SHARED_MAILBOX = 0,
    parameter DCACHE_SET_BITS = 7,
    parameter DCACHE_LINE_WRAP = 0,
    parameter ICACHE_SET_BITS = 7,
    parameter ENABLE_DEVICE_MMIO = 0,
    parameter [31:0] DEVICE_MMIO_MASK = 32'hff00_0000
)
(
    // Clock, reset and board configuration
    input  logic clk,
    input  logic reset_n,
    input  logic a20_enable,
    input  logic cache_enable,          // Dev menu: L1 caches on
    input  logic device_mmio_enable,
    input  logic [31:0] device_mmio_base,

    // Paging unit: demand data request (physical address, before A20 masking)
    input  logic dcache_req_valid,
    input  logic [31:0] dcache_req_phys_addr_raw,
    input  logic [11:0] dcache_req_preread_offset,
    input  logic dcache_req_preread_priority,
    input  logic dcache_req_write,
    input  logic [3:0] dcache_req_be,
    input  logic [31:0] dcache_req_wdata,
    input  logic dcache_req_is_io,
    input  logic dcache_req_is_inta,
    input  logic dcache_req_is_vga_mem,

    // Execution core: WR_FAST store and VIPT load/RMW preread
    input  logic fast_store_valid,
    input  logic [31:0] fast_store_phys_addr_raw,
    input  logic [3:0] fast_store_be,
    input  logic [31:0] fast_store_wdata,
    output logic fast_store_accepted,
    input  logic dcache_vipt_probe_valid,
    input  logic [11:0] dcache_vipt_probe_offset,
    output logic dcache_vipt_probe_ready,
    output logic dcache_vipt_probe_accepted,
    output logic dcache_vipt_probe_direct_accepted,
    input  logic dcache_vipt_resolve_valid,
    input  logic [31:0] dcache_vipt_resolve_phys_addr_raw,
    output logic dcache_vipt_resolve_hit,
    output logic [31:0] dcache_vipt_resolve_data,
    input  logic dcache_vipt_resolve_fill,      // a miss may fill at once
    output logic dcache_vipt_fill_start,        // the miss's fill started
    output logic dcache_vipt_fill_resp_valid,   // its missing dword
    output logic [31:0] dcache_vipt_fill_resp_data,

    // Paging unit: instruction line request (prefetch)
    input  logic icache_req_valid,
    input  logic [31:0] icache_req_phys_addr_raw,
    output logic icache_req_accepted,
    output logic icache_req_complete,
    output logic [127:0] icache_rdata,

    // Bus interface unit: request routing and D-cache CPU response
    input  logic x87_req_selected,
    output logic normal_cache_req,
    output logic dcache_req_is_device_mmio,
    output logic [31:0] dcache_req_phys_addr,
    output logic dcache_cpu_ready,
    output logic dcache_cpu_wr_ready,
    output logic dcache_cpu_resp_valid,
    output logic [31:0] dcache_cpu_dout,
    output logic dcache_stores_drained,

    // Bus interface unit: line fills and write-buffer drain (KBA/KBWR/KBRD)
    output logic dcache_mem_valid,
    output logic [31:0] dcache_mem_addr,
    output logic dcache_mem_write,
    output logic [3:0] dcache_mem_be,
    output logic [7:0] dcache_mem_burstcount,
    output logic [31:0] dcache_mem_din,
    input  logic dcache_mem_ready,
    input  logic dcache_mem_resp_valid,
    input  logic dcache_mem_line_resp_valid,
    output logic icache_mem_valid,
    output logic [31:0] icache_mem_addr,
    output logic [3:0] icache_mem_be,
    output logic [7:0] icache_mem_burstcount,
    input  logic icache_mem_ready,
    input  logic icache_mem_resp_valid,
    input  logic icache_mem_line_resp_valid,
    input  logic ext_valid_r,
    input  logic direct_rd_pending,
    input  logic dcache_read_pending,
    input  logic icache_read_pending,
    input  logic [31:0] din,
    input  logic [127:0] line_din,

    // External coherence (snoop invalidation)
    input  logic [31:0] snoop_addr,
    input  logic snoop_valid
);


assign dcache_req_phys_addr = (!a20_enable && !dcache_req_is_io)
                                      ? (dcache_req_phys_addr_raw & ~32'h0010_0000)
                                      : dcache_req_phys_addr_raw;
wire [31:0] dcache_vipt_resolve_phys_addr = !a20_enable
                                      ? (dcache_vipt_resolve_phys_addr_raw & ~32'h0010_0000)
                                      : dcache_vipt_resolve_phys_addr_raw;
wire [31:0] fast_store_phys_addr = !a20_enable
                                      ? (fast_store_phys_addr_raw & ~32'h0010_0000)
                                      : fast_store_phys_addr_raw;
wire [31:0] icache_req_phys_addr = !a20_enable
                                      ? (icache_req_phys_addr_raw & ~32'h0010_0000)
                                      : icache_req_phys_addr_raw;
assign dcache_req_is_device_mmio = ENABLE_DEVICE_MMIO &&
                                  device_mmio_enable &&
                                  ((dcache_req_phys_addr & DEVICE_MMIO_MASK) ==
                                   (device_mmio_base & DEVICE_MMIO_MASK));
wire dcache_vipt_is_device_mmio = ENABLE_DEVICE_MMIO &&
                                   device_mmio_enable &&
                                   ((dcache_vipt_resolve_phys_addr &
                                     DEVICE_MMIO_MASK) ==
                                    (device_mmio_base & DEVICE_MMIO_MASK));

// dcache_cpu_dout: port
wire        dcache_vipt_resolve_hit_cache;
// dcache_cpu_ready: port
// dcache_cpu_resp_valid: port
// dcache_stores_drained: port
wire [31:0] dcache_store_patch_addr;
wire [31:0] dcache_store_patch_data;
wire  [3:0] dcache_store_patch_be;
wire        dcache_store_patch_valid;
// dcache_mem_addr: port
// dcache_mem_din: port
// dcache_mem_be: port
// dcache_mem_burstcount: port
// dcache_mem_valid: port
// dcache_mem_write: port

wire [127:0] icache_cpu_line;
wire         icache_cpu_ready;
wire         icache_cpu_resp_valid;
// icache_mem_addr: port
// icache_mem_be: port
// icache_mem_burstcount: port
// icache_mem_valid: port

assign dcache_vipt_resolve_hit = dcache_vipt_resolve_hit_cache &&
                                  !dcache_vipt_is_device_mmio;

logic       dcache_cpu_rd_pending;
logic       icache_cpu_rd_pending;

// VGA aperture accesses are device transactions. Bypass the posted L1 store
// queue so an ET4000 bank-register write cannot overtake framebuffer writes.
assign normal_cache_req = dcache_req_valid && !dcache_req_is_io &&
                        !dcache_req_is_inta && !dcache_req_is_vga_mem &&
                        !dcache_req_is_device_mmio &&
                        !x87_req_selected;
wire dcache_cpu_req = fast_store_valid || normal_cache_req;
wire [31:0] dcache_cpu_addr = fast_store_valid ? fast_store_phys_addr
                                              : dcache_req_phys_addr;
wire [31:0] dcache_cpu_wdata = fast_store_valid ? fast_store_wdata
                                               : dcache_req_wdata;
wire [3:0] dcache_cpu_be = fast_store_valid ? fast_store_be : dcache_req_be;
wire dcache_cpu_write = fast_store_valid || dcache_req_write;
// WR_FAST already owns a translated physical address.  Its request must also
// own the synchronous RAM preread; leaving the normal paging preread here
// makes the following S_LOOKUP compare against an unrelated set.
wire [11:0] dcache_cpu_preread_offset = fast_store_valid
                                      ? fast_store_phys_addr[11:0]
                                      : dcache_req_preread_offset;
wire dcache_cpu_preread_priority = fast_store_valid ||
                                   dcache_req_preread_priority;

wire dcache_read_accept = normal_cache_req && !dcache_req_write && dcache_cpu_ready;
wire icache_read_accept = icache_req_valid && icache_cpu_ready;
wire dcache_read_done = dcache_cpu_resp_valid &&
                        (dcache_cpu_rd_pending || dcache_read_accept);
wire icache_read_done = icache_cpu_resp_valid &&
                        (icache_cpu_rd_pending || icache_read_accept);

logic       icache_write_snoop_pending;
logic [31:0] icache_write_snoop_addr_r;
logic [31:0] icache_write_snoop_data_r;
logic  [3:0] icache_write_snoop_be_r;
// Normally the I-cache consumes the D-cache's registered S_LOOKUP store
// directly.  The one-entry pending slot only resolves an external invalidate
// collision (or holds the following store while an older pending patch is
// consumed), preserving invalidate priority without returning to live paging
// request signals.
wire icache_write_patch_valid = !snoop_valid &&
                                (icache_write_snoop_pending ||
                                 dcache_store_patch_valid);
wire [31:0] icache_write_patch_addr = icache_write_snoop_pending
                                    ? icache_write_snoop_addr_r
                                    : dcache_store_patch_addr;
wire [31:0] icache_write_patch_data = icache_write_snoop_pending
                                    ? icache_write_snoop_data_r
                                    : dcache_store_patch_data;
wire [3:0] icache_write_patch_be = icache_write_snoop_pending
                                 ? icache_write_snoop_be_r
                                 : dcache_store_patch_be;

assign fast_store_accepted = fast_store_valid && dcache_cpu_wr_ready;

assign icache_req_accepted = icache_cpu_ready;
assign icache_req_complete = icache_cpu_resp_valid;
assign icache_rdata = icache_cpu_line;

// Cache-side request tracking and the I-cache write-patch slot.
always_ff @(posedge clk) begin
    if (!reset_n) begin
        dcache_cpu_rd_pending <= 1'b0;
        icache_cpu_rd_pending <= 1'b0;
        icache_write_snoop_pending <= 1'b0;
        icache_write_snoop_addr_r <= 32'h0;
        icache_write_snoop_data_r <= 32'h0;
        icache_write_snoop_be_r <= 4'h0;
    end else begin
        if (dcache_store_patch_valid &&
            (snoop_valid || icache_write_snoop_pending)) begin
            // The pending patch, when present, is consumed on this edge. Keep
            // the newly accepted store for the following cycle.
            icache_write_snoop_pending <= 1'b1;
            icache_write_snoop_addr_r <= dcache_store_patch_addr;
            icache_write_snoop_data_r <= dcache_store_patch_data;
            icache_write_snoop_be_r <= dcache_store_patch_be;
        end else if (icache_write_snoop_pending && !snoop_valid) begin
            icache_write_snoop_pending <= 1'b0;
        end

        if (dcache_read_accept && !dcache_read_done)
            dcache_cpu_rd_pending <= 1'b1;
        else if (dcache_read_done)
            dcache_cpu_rd_pending <= 1'b0;

        if (icache_read_accept && !icache_read_done)
            icache_cpu_rd_pending <= 1'b1;
        else if (icache_read_done)
            icache_cpu_rd_pending <= 1'b0;
    end
end

l1_cache #(
    .PROTECT_UMA_ROM(PROTECT_UMA_ROM),
    .ENABLE_SHARED_MAILBOX(ENABLE_SHARED_MAILBOX),
    .SET_BITS(DCACHE_SET_BITS),
    .LINE_WRAP(DCACHE_LINE_WRAP)
) dcache_inst (
    .clk(clk),
    .reset(!reset_n),
    .cpu_addr(dcache_cpu_addr),
    .cpu_preread_offset(dcache_cpu_preread_offset),
    .cpu_preread_priority(dcache_cpu_preread_priority),
    .cpu_din(dcache_cpu_wdata),
    .cpu_dout(dcache_cpu_dout),
    .cpu_be(dcache_cpu_be),
    .cpu_valid(dcache_cpu_req),
    .cpu_write(dcache_cpu_write),
    // I/O, INTA, x87, and VGA/device transactions are routed around this
    // cache above, so accepted D-cache requests need no physical-address
    // aperture decode on their register inputs.
    .cpu_uncacheable(1'b0),
    .cpu_ready(dcache_cpu_ready),
    .cpu_wr_ready(dcache_cpu_wr_ready),
    .store_patch_busy(snoop_valid || icache_write_snoop_pending),
    .cpu_resp_valid(dcache_cpu_resp_valid),
    .stores_drained(dcache_stores_drained),
    .store_patch_addr(dcache_store_patch_addr),
    .store_patch_data(dcache_store_patch_data),
    .store_patch_be(dcache_store_patch_be),
    .store_patch_valid(dcache_store_patch_valid),
    .vipt_probe_offset(dcache_vipt_probe_offset),
    .vipt_probe_valid(dcache_vipt_probe_valid),
    .vipt_probe_ready(dcache_vipt_probe_ready),
    .vipt_probe_accepted(dcache_vipt_probe_accepted),
    .vipt_probe_direct_accepted(dcache_vipt_probe_direct_accepted),
    .vipt_resolve_phys_addr(dcache_vipt_resolve_phys_addr),
    .vipt_resolve_valid(dcache_vipt_resolve_valid),
    .vipt_resolve_data(dcache_vipt_resolve_data),
    .vipt_resolve_hit(dcache_vipt_resolve_hit_cache),
    // Device windows are not cacheable: their misses take the CPU port.
    .vipt_resolve_fill(dcache_vipt_resolve_fill && !dcache_vipt_is_device_mmio),
    .vipt_fill_start(dcache_vipt_fill_start),
    .vipt_fill_resp_valid(dcache_vipt_fill_resp_valid),
    .vipt_fill_resp_data(dcache_vipt_fill_resp_data),
    .mem_addr(dcache_mem_addr),
    .mem_din(dcache_mem_din),
    .mem_dout(din),
    .mem_line_dout(line_din),
    .mem_be(dcache_mem_be),
    .mem_burstcount(dcache_mem_burstcount),
    .mem_busy(ext_valid_r || direct_rd_pending || icache_read_pending),
    .mem_valid(dcache_mem_valid),
    .mem_write(dcache_mem_write),
    .mem_ready(dcache_mem_ready),
    .mem_resp_valid(dcache_mem_resp_valid),
    .mem_line_resp_valid(dcache_mem_line_resp_valid),
    .snoop_addr(snoop_addr),
    .snoop_valid(snoop_valid),
    .cache_enable(cache_enable)
);

l1_icache #(
    .ENABLE_SHARED_MAILBOX(ENABLE_SHARED_MAILBOX),
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
    .mem_line_dout(line_din),
    .mem_be(icache_mem_be),
    .mem_burstcount(icache_mem_burstcount),
    .mem_busy(ext_valid_r || direct_rd_pending || dcache_read_pending ||
              dcache_mem_valid),
    .mem_valid(icache_mem_valid),
    .mem_ready(icache_mem_ready),
    .mem_resp_valid(icache_mem_resp_valid),
    .mem_line_resp_valid(icache_mem_line_resp_valid),
    .patch_addr(icache_write_patch_addr),
    .patch_data(icache_write_patch_data),
    .patch_be(icache_write_patch_be),
    .patch_valid(icache_write_patch_valid),
    .invalidate_addr(snoop_addr),
    .invalidate_valid(snoop_valid),
    .cache_enable(cache_enable)
);

endmodule
