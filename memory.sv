//
// Memory Unit
// Connects the bus interface unit and the cache unit
//
module memory #(
    parameter PROTECT_UMA_ROM = 0,
    parameter DCACHE_SET_BITS = 7,
    parameter DCACHE_LINE_WRAP = 0,
    parameter ICACHE_SET_BITS = 7,
    parameter ENABLE_X87 = 0,
    parameter ENABLE_DEVICE_MMIO = 0,
    parameter [31:0] DEVICE_MMIO_MASK = 32'hff00_0000
) (
    input              clk,
    input              reset_n,
    input              a20_enable,
    input              cache_enable,  // Dev menu: L1 caches on
    input              x87_off,       // Dev menu: no coprocessor
    input              device_mmio_enable,
    input      [31:0]  device_mmio_base,

    // Paging-unit demand request
    input              dcache_req_valid,
    input      [31:0]  dcache_req_phys_addr_raw,
    input      [11:0]  dcache_req_preread_offset,
    input              dcache_req_preread_priority,
    input              dcache_req_write,
    input       [3:0]  dcache_req_be,
    input      [31:0]  dcache_req_wdata,
    input      [31:0]  dcache_direct_wdata,
    input              dcache_req_is_io,
    input              dcache_req_is_inta,
    input              dcache_req_is_x87,
    input              dcache_req_is_vga_mem,
    output             dcache_req_accepted, // Request ownership transferred
    output             dcache_req_complete, // Read or write operation completed
    output             dcache_read_complete, // Read data is valid this cycle
    output     [31:0]  dcache_rdata,

    // Retained, already-qualified physical store from a microcode fast path.
    // Acceptance transfers irrevocable ownership to the ordinary cache/store
    // queue; the cache may perform its internal enqueue on the following edge.
    input              fast_store_valid,
    input      [31:0]  fast_store_phys_addr_raw,
    input       [3:0]  fast_store_be,
    input      [31:0]  fast_store_wdata,
    output             fast_store_accepted,
    output             dcache_wr_ready,      // the L1 takes a direct store this cycle (registered)

    // Non-owning hardwired-load preread. A miss is retried through the demand
    // request interface above; this port never starts a fill by itself.
    input              dcache_vipt_probe_valid,
    input      [11:0]  dcache_vipt_probe_offset,
    output             dcache_vipt_probe_ready,
    output             dcache_vipt_probe_accepted,
    output             dcache_vipt_probe_direct_accepted,
    input              dcache_vipt_resolve_valid,
    input      [31:0]  dcache_vipt_resolve_phys_addr_raw,
    output             dcache_vipt_resolve_hit,
    output     [31:0]  dcache_vipt_resolve_data,
    input              dcache_vipt_resolve_fill,
    output             dcache_vipt_fill_start,
    output             dcache_vipt_fill_resp_valid,
    output     [31:0]  dcache_vipt_fill_resp_data,

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
    output             line_read,             // Request is one complete cache line
    input      [31:0]  din,
    input      [127:0] line_din,
    output     [31:0]  dout,
    output             valid,                // External request is present
    input              ready,                // External request was accepted
    output             write,
    output             io,
    input              resp_valid,           // External read beat returned
    input              line_resp_valid,      // Complete aligned line returned
    output             inta
);

wire [31:0] dcache_cpu_dout;
wire dcache_cpu_ready;
wire dcache_cpu_wr_ready;
assign dcache_wr_ready = dcache_cpu_wr_ready;
wire dcache_cpu_resp_valid;
wire [31:0] dcache_mem_addr;
wire [3:0] dcache_mem_be;
wire [7:0] dcache_mem_burstcount;
wire [31:0] dcache_mem_din;
wire dcache_mem_line_resp_valid;
wire dcache_mem_ready;
wire dcache_mem_resp_valid;
wire dcache_mem_valid;
wire dcache_mem_write;
wire dcache_req_is_device_mmio;
wire [31:0] dcache_req_phys_addr;
wire dcache_stores_drained;
wire [31:0] icache_mem_addr;
wire [3:0] icache_mem_be;
wire [7:0] icache_mem_burstcount;
wire icache_mem_line_resp_valid;
wire icache_mem_ready;
wire icache_mem_resp_valid;
wire icache_mem_valid;
wire normal_cache_req;
wire dcache_read_pending;
wire direct_rd_pending;
wire ext_valid_r;
wire icache_read_pending;
cache_unit #(.PROTECT_UMA_ROM(PROTECT_UMA_ROM), .DCACHE_SET_BITS(DCACHE_SET_BITS), .DCACHE_LINE_WRAP(DCACHE_LINE_WRAP), .ICACHE_SET_BITS(ICACHE_SET_BITS), .ENABLE_DEVICE_MMIO(ENABLE_DEVICE_MMIO), .DEVICE_MMIO_MASK(DEVICE_MMIO_MASK)) cache_unit_inst (
    // Clock, reset and board configuration
    .clk(clk),
    .reset_n(reset_n),
    .a20_enable(a20_enable),
    .cache_enable(cache_enable),
    .device_mmio_enable(device_mmio_enable),
    .device_mmio_base(device_mmio_base),
    // Paging unit: demand data request (physical address, before A20 masking)
    .dcache_req_valid(dcache_req_valid),
    .dcache_req_phys_addr_raw(dcache_req_phys_addr_raw),
    .dcache_req_preread_offset(dcache_req_preread_offset),
    .dcache_req_preread_priority(dcache_req_preread_priority),
    .dcache_req_write(dcache_req_write),
    .dcache_req_be(dcache_req_be),
    .dcache_req_wdata(dcache_req_wdata),
    .dcache_req_is_io(dcache_req_is_io),
    .dcache_req_is_inta(dcache_req_is_inta),
    .dcache_req_is_vga_mem(dcache_req_is_vga_mem),
    // Execution core: WR_FAST store and VIPT load/RMW preread
    .fast_store_valid(fast_store_valid),
    .fast_store_phys_addr_raw(fast_store_phys_addr_raw),
    .fast_store_be(fast_store_be),
    .fast_store_wdata(fast_store_wdata),
    .fast_store_accepted(fast_store_accepted),
    .dcache_vipt_probe_valid(dcache_vipt_probe_valid),
    .dcache_vipt_probe_offset(dcache_vipt_probe_offset),
    .dcache_vipt_probe_ready(dcache_vipt_probe_ready),
    .dcache_vipt_probe_accepted(dcache_vipt_probe_accepted),
    .dcache_vipt_probe_direct_accepted(dcache_vipt_probe_direct_accepted),
    .dcache_vipt_resolve_valid(dcache_vipt_resolve_valid),
    .dcache_vipt_resolve_phys_addr_raw(dcache_vipt_resolve_phys_addr_raw),
    .dcache_vipt_resolve_hit(dcache_vipt_resolve_hit),
    .dcache_vipt_resolve_data(dcache_vipt_resolve_data),
    .dcache_vipt_resolve_fill(dcache_vipt_resolve_fill),
    .dcache_vipt_fill_start(dcache_vipt_fill_start),
    .dcache_vipt_fill_resp_valid(dcache_vipt_fill_resp_valid),
    .dcache_vipt_fill_resp_data(dcache_vipt_fill_resp_data),
    // Paging unit: instruction line request (prefetch)
    .icache_req_valid(icache_req_valid),
    .icache_req_phys_addr_raw(icache_req_phys_addr_raw),
    .icache_req_accepted(icache_req_accepted),
    .icache_req_complete(icache_req_complete),
    .icache_rdata(icache_rdata),
    // Bus interface unit: request routing and D-cache CPU response
    .x87_req_selected(x87_req_selected),
    .normal_cache_req(normal_cache_req),
    .dcache_req_is_device_mmio(dcache_req_is_device_mmio),
    .dcache_req_phys_addr(dcache_req_phys_addr),
    .dcache_cpu_ready(dcache_cpu_ready),
    .dcache_cpu_wr_ready(dcache_cpu_wr_ready),
    .dcache_cpu_resp_valid(dcache_cpu_resp_valid),
    .dcache_cpu_dout(dcache_cpu_dout),
    .dcache_stores_drained(dcache_stores_drained),
    // Bus interface unit: line fills and write-buffer drain (KBA/KBWR/KBRD)
    .dcache_mem_valid(dcache_mem_valid),
    .dcache_mem_addr(dcache_mem_addr),
    .dcache_mem_write(dcache_mem_write),
    .dcache_mem_be(dcache_mem_be),
    .dcache_mem_burstcount(dcache_mem_burstcount),
    .dcache_mem_din(dcache_mem_din),
    .dcache_mem_ready(dcache_mem_ready),
    .dcache_mem_resp_valid(dcache_mem_resp_valid),
    .dcache_mem_line_resp_valid(dcache_mem_line_resp_valid),
    .icache_mem_valid(icache_mem_valid),
    .icache_mem_addr(icache_mem_addr),
    .icache_mem_be(icache_mem_be),
    .icache_mem_burstcount(icache_mem_burstcount),
    .icache_mem_ready(icache_mem_ready),
    .icache_mem_resp_valid(icache_mem_resp_valid),
    .icache_mem_line_resp_valid(icache_mem_line_resp_valid),
    .ext_valid_r(ext_valid_r),
    .direct_rd_pending(direct_rd_pending),
    .dcache_read_pending(dcache_read_pending),
    .icache_read_pending(icache_read_pending),
    .din(din),
    .line_din(line_din),
    // External coherence (snoop invalidation)
    .snoop_addr(snoop_addr),
    .snoop_valid(snoop_valid)
);

bus_unit #(.ENABLE_X87(ENABLE_X87)) bus_unit_inst (
    // Clock and reset
    .clk(clk),
    .reset_n(reset_n),
    .x87_off(x87_off),
    // Paging unit: demand request, cycle type and completion
    .dcache_req_valid(dcache_req_valid),
    .dcache_req_write(dcache_req_write),
    .dcache_req_be(dcache_req_be),
    .dcache_direct_wdata(dcache_direct_wdata),
    .dcache_req_is_io(dcache_req_is_io),
    .dcache_req_is_inta(dcache_req_is_inta),
    .dcache_req_is_vga_mem(dcache_req_is_vga_mem),
    .dcache_req_is_x87(dcache_req_is_x87),
    .dcache_req_accepted(dcache_req_accepted),
    .dcache_req_complete(dcache_req_complete),
    .dcache_read_complete(dcache_read_complete),
    .dcache_rdata(dcache_rdata),
    // Execution core: WR_FAST store in flight
    .fast_store_valid(fast_store_valid),
    // FPU: coprocessor data-port cycles
    .x87_req_selected(x87_req_selected),
    .x87_req_accepted(x87_req_accepted),
    .x87_req_complete(x87_req_complete),
    .x87_read_complete(x87_read_complete),
    .x87_rdata(x87_rdata),
    // Cache unit: request routing and D-cache CPU response
    .normal_cache_req(normal_cache_req),
    .dcache_req_is_device_mmio(dcache_req_is_device_mmio),
    .dcache_req_phys_addr(dcache_req_phys_addr),
    .dcache_cpu_ready(dcache_cpu_ready),
    .dcache_cpu_wr_ready(dcache_cpu_wr_ready),
    .dcache_cpu_resp_valid(dcache_cpu_resp_valid),
    .dcache_cpu_dout(dcache_cpu_dout),
    .dcache_stores_drained(dcache_stores_drained),
    // Cache unit: line fills and write-buffer drain
    .dcache_mem_valid(dcache_mem_valid),
    .dcache_mem_addr(dcache_mem_addr),
    .dcache_mem_write(dcache_mem_write),
    .dcache_mem_be(dcache_mem_be),
    .dcache_mem_burstcount(dcache_mem_burstcount),
    .dcache_mem_din(dcache_mem_din),
    .dcache_mem_ready(dcache_mem_ready),
    .dcache_mem_resp_valid(dcache_mem_resp_valid),
    .dcache_mem_line_resp_valid(dcache_mem_line_resp_valid),
    .icache_mem_valid(icache_mem_valid),
    .icache_mem_addr(icache_mem_addr),
    .icache_mem_be(icache_mem_be),
    .icache_mem_burstcount(icache_mem_burstcount),
    .icache_mem_ready(icache_mem_ready),
    .icache_mem_resp_valid(icache_mem_resp_valid),
    .icache_mem_line_resp_valid(icache_mem_line_resp_valid),
    .ext_valid_r(ext_valid_r),
    .direct_rd_pending(direct_rd_pending),
    .dcache_read_pending(dcache_read_pending),
    .icache_read_pending(icache_read_pending),
    // External bus (XA/XD)
    .valid(valid),
    .ready(ready),
    .addr(addr),
    .be(be),
    .burstcount(burstcount),
    .line_read(line_read),
    .write(write),
    .io(io),
    .inta(inta),
    .dout(dout),
    .din(din),
    .resp_valid(resp_valid),
    .line_resp_valid(line_resp_valid)
);
endmodule
