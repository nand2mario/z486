//
// L1 Data Cache
// Physically indexed, physically tagged data cache with a store queue
//
`include "z486_platform.svh"
module l1_cache #(
    // Four ways, 16 bytes per line. SET_BITS=7 gives the default 8KB data
    // cache (128 sets x 4 ways x 16 B); use 8 for 16KB.
    parameter integer SET_BITS = 7,
    parameter PROTECT_UMA_ROM = 0,
    // 1: a line fill requests the missing dword first and memory returns the
    // line's dwords in wrapping order from it (critical word first).
    parameter LINE_WRAP = 0
) (
    input         clk,
    input         reset,

    // CPU side — physical address request/response.
    input  [31:0] cpu_addr,    // physical byte address; the cache indexes off the
                               // page-offset bits [11:2] (translation-invariant,
                               // so available without the TLB result) and tags off
                               // [31:12], exactly like l1_icache
    input  [11:0] cpu_preread_offset, // untranslated low address for RAM preread
    input         cpu_preread_priority,// demand intent owns preread over VIPT probe
    input  [31:0] cpu_din,
    output [31:0] cpu_dout,
    input   [3:0] cpu_be,
    input         cpu_valid,
    input         cpu_write,
    input         cpu_uncacheable,
    output        cpu_ready,      // a read may be presented (registered)
    output        cpu_wr_ready,   // a write may be presented (registered)
    output        cpu_resp_valid,
    output        stores_drained,

    // Accepted CPU stores are registered before S_LOOKUP.  Export that
    // registered payload for the instruction-cache coherence patch.
    output [31:0] store_patch_addr,
    output [31:0] store_patch_data,
    output  [3:0] store_patch_be,
    // The coherence patch consumer cannot take a store this cycle; holds off
    // a store accepted behind another store's S_LOOKUP.
    input         store_patch_busy,
    output        store_patch_valid,

    // Side-effect-free hardwired-load path. D2 selects the RAM word; EX
    // supplies the physical tag exactly one cycle later. The cache does not
    // retain request ownership and a miss retries through the CPU interface.
    input  [11:0] vipt_probe_offset,
    input         vipt_probe_valid,
    output        vipt_probe_ready,
    output        vipt_probe_accepted,
    output        vipt_probe_direct_accepted,
    input  [31:0] vipt_resolve_phys_addr,
    input         vipt_resolve_valid,
    output [31:0] vipt_resolve_data,
    output        vipt_resolve_hit,
    // A missing resolve that may fill at once (a cacheable direct load): the
    // cache starts the line fill in the resolve cycle (vipt_fill_start) and
    // returns the missing dword on vipt_fill_resp_* instead of the CPU port.
    input         vipt_resolve_fill,
    output        vipt_fill_start,
    output        vipt_fill_resp_valid,
    output [31:0] vipt_fill_resp_data,

    // Memory side.
    output [31:0] mem_addr,
    output [31:0] mem_din,
    input  [31:0] mem_dout,
    input [127:0] mem_line_dout,
    output  [3:0] mem_be,
    output  [7:0] mem_burstcount,
    input         mem_busy,
    output        mem_valid,
    output        mem_write,
    input         mem_ready,
    input         mem_resp_valid,
    input         mem_line_resp_valid,

    // Physical-address snoop.  The first implementation invalidates a whole
    // set; this is conservative and keeps snoop matching off the read hit path.
    input  [31:0] snoop_addr,
    input         snoop_valid,

    input         cache_enable
);

localparam integer WORD_OFFSET_BITS = 2;
localparam integer BYTE_OFFSET_BITS = 2;
localparam integer LINE_OFFSET_BITS = WORD_OFFSET_BITS + BYTE_OFFSET_BITS;
localparam integer NUM_SETS = 1 << SET_BITS;
localparam integer BRAM_ADDR_BITS = SET_BITS + WORD_OFFSET_BITS;
localparam integer PHYS_ADDR_BITS = 27; // maximum supported RAM is 128MB
localparam integer TAG_BITS = PHYS_ADDR_BITS - LINE_OFFSET_BITS - SET_BITS;
localparam integer SET_LSB = LINE_OFFSET_BITS;
localparam integer SET_MSB = SET_LSB + SET_BITS - 1;
localparam integer TAG_LSB = SET_MSB + 1;
localparam integer TAG_MSB = PHYS_ADDR_BITS - 1;
localparam integer TAG_RAM_BITS = (TAG_BITS < 16) ? 16 : (TAG_BITS + 1);
localparam integer TAG_VALID_BIT = TAG_BITS;
localparam integer STOREQ_DEPTH = `ifdef STOREQ_DEPTH_OVERRIDE `STOREQ_DEPTH_OVERRIDE `else 4 `endif;
localparam integer STOREQ_IDX_BITS = $clog2(STOREQ_DEPTH);
localparam integer STOREQ_CNT_BITS = $clog2(STOREQ_DEPTH + 1);
localparam [STOREQ_CNT_BITS-1:0] STOREQ_DEPTH_VALUE = STOREQ_CNT_BITS'(STOREQ_DEPTH);
localparam [STOREQ_IDX_BITS-1:0] STOREQ_LAST_IDX = STOREQ_IDX_BITS'(STOREQ_DEPTH - 1);
localparam [SET_BITS-1:0] LAST_SET = SET_BITS'(NUM_SETS - 1);

// Address decomposition. Include the complete physical tag so larger SDRAM
// configurations cannot alias cache lines at 32MB boundaries.
wire [TAG_BITS-1:0] cpu_tag = cpu_addr[TAG_MSB:TAG_LSB];
// Set/word array index from the physical address page-offset bits
// (cpu_addr[11:2], translation-invariant -- available without the TLB result).
wire [SET_BITS-1:0] cpu_set = cpu_addr[SET_MSB:SET_LSB];
wire [WORD_OFFSET_BITS-1:0] cpu_word = cpu_addr[LINE_OFFSET_BITS-1:BYTE_OFFSET_BITS];
wire [BRAM_ADDR_BITS-1:0] cpu_bram_addr = {cpu_set, cpu_word};
wire [SET_BITS-1:0] cpu_preread_set =
    cpu_preread_offset[SET_MSB:SET_LSB];
wire [WORD_OFFSET_BITS-1:0] cpu_preread_word =
    cpu_preread_offset[LINE_OFFSET_BITS-1:BYTE_OFFSET_BITS];
wire [SET_BITS-1:0] vipt_probe_set = vipt_probe_offset[SET_MSB:SET_LSB];
wire [WORD_OFFSET_BITS-1:0] vipt_probe_word =
    vipt_probe_offset[LINE_OFFSET_BITS-1:BYTE_OFFSET_BITS];
wire [SET_BITS-1:0] snoop_set = snoop_addr[SET_MSB:SET_LSB];
wire request_uncacheable = !cache_enable || cpu_uncacheable;
`ifdef VERILATOR
// Simulation harness switch: a loaded snapshot whose BIOS keeps state in its
// shadow (SeaBIOS runs interrupt handlers on a stack at 0xE0000-0xEFFFF)
// needs the UMA ROM window writable.
reg sim_uma_rom_writable /* verilator public_flat_rw */ = 1'b0;
`else
wire sim_uma_rom_writable = 1'b0;
`endif
// Decode the full address: a partial compare also protects the 32MB aliases
// (0x20C0000-0x20FFFFF, ...) and silently drops writes to that RAM.
wire cpu_protect_write = PROTECT_UMA_ROM && !sim_uma_rom_writable && cpu_write &&
                         (cpu_addr[31:18] == 14'h0003);

// Tag/data storage.
// Keep validity in the otherwise under-filled tag RAM word. This removes four
// asynchronously indexed 256-bit register arrays from the preread address
// path without changing the synchronous lookup boundary.
`Z486_BLOCK_RAM reg [TAG_RAM_BITS-1:0] tag_way0 [0:NUM_SETS-1];
`Z486_BLOCK_RAM reg [TAG_RAM_BITS-1:0] tag_way1 [0:NUM_SETS-1];
`Z486_BLOCK_RAM reg [TAG_RAM_BITS-1:0] tag_way2 [0:NUM_SETS-1];
`Z486_BLOCK_RAM reg [TAG_RAM_BITS-1:0] tag_way3 [0:NUM_SETS-1];
reg [2:0] plru_set [0:NUM_SETS-1];

`Z486_BLOCK_RAM reg [31:0] data_way0 [0:(NUM_SETS << WORD_OFFSET_BITS)-1];
`Z486_BLOCK_RAM reg [31:0] data_way1 [0:(NUM_SETS << WORD_OFFSET_BITS)-1];
`Z486_BLOCK_RAM reg [31:0] data_way2 [0:(NUM_SETS << WORD_OFFSET_BITS)-1];
`Z486_BLOCK_RAM reg [31:0] data_way3 [0:(NUM_SETS << WORD_OFFSET_BITS)-1];

// Synchronous cache read result for the request accepted in the previous cycle.
// Register each complete tag word as one RAM read; Quartus 17 otherwise treats
// separate tag and valid slices as independent read ports and implements the
// arrays in logic.
reg [TAG_RAM_BITS-1:0] rd_tag_entry0_r, rd_tag_entry1_r;
reg [TAG_RAM_BITS-1:0] rd_tag_entry2_r, rd_tag_entry3_r;
wire [TAG_BITS-1:0] rd_tag0_r = rd_tag_entry0_r[TAG_BITS-1:0];
wire [TAG_BITS-1:0] rd_tag1_r = rd_tag_entry1_r[TAG_BITS-1:0];
wire [TAG_BITS-1:0] rd_tag2_r = rd_tag_entry2_r[TAG_BITS-1:0];
wire [TAG_BITS-1:0] rd_tag3_r = rd_tag_entry3_r[TAG_BITS-1:0];
wire rd_valid0_r = rd_tag_entry0_r[TAG_VALID_BIT];
wire rd_valid1_r = rd_tag_entry1_r[TAG_VALID_BIT];
wire rd_valid2_r = rd_tag_entry2_r[TAG_VALID_BIT];
wire rd_valid3_r = rd_tag_entry3_r[TAG_VALID_BIT];
reg [31:0] rd_data0_r, rd_data1_r, rd_data2_r, rd_data3_r;
reg [2:0] rd_plru_r;

// Accepted request register.
reg        req_valid_r;
reg [31:0] req_addr_r;
reg [31:0] req_din_r;
reg  [3:0] req_be_r;
reg        req_write_r;
reg        req_uncacheable_r;
reg        req_protect_write_r;
reg [TAG_BITS-1:0] req_tag_r;
reg [SET_BITS-1:0] req_set_r;
reg [WORD_OFFSET_BITS-1:0] req_word_r;

// Write-through store queue.
reg [29:0] storeq_addr [0:STOREQ_DEPTH-1];
reg [31:0] storeq_data [0:STOREQ_DEPTH-1];
reg  [3:0] storeq_be   [0:STOREQ_DEPTH-1];
reg        storeq_valid[0:STOREQ_DEPTH-1];
reg [STOREQ_IDX_BITS-1:0] storeq_head;
reg [STOREQ_IDX_BITS-1:0] storeq_tail;
reg [STOREQ_CNT_BITS-1:0] storeq_count;
reg        storeq_draining;

wire storeq_full = (storeq_count == STOREQ_DEPTH_VALUE);
wire storeq_empty = (storeq_count == {STOREQ_CNT_BITS{1'b0}});
wire storeq_can_accept = !storeq_full || (storeq_draining && mem_ready);
// Device transactions are serializing.  The memory fabric uses this status to
// keep I/O and other direct accesses behind every older posted store.
assign stores_drained = storeq_empty && !storeq_draining &&
                        !(req_valid_r && req_write_r && !req_protect_write_r);

// Memory-side registers.
reg        mem_valid_r;
reg        mem_write_r;
reg [31:0] mem_addr_r;
reg [31:0] mem_din_r;
reg  [3:0] mem_be_r;
reg  [7:0] mem_burstcount_r;

assign mem_valid = mem_valid_r;
assign mem_write = mem_write_r;
assign mem_addr = mem_addr_r;
assign mem_din = mem_din_r;
assign mem_be = mem_be_r;
assign mem_burstcount = mem_burstcount_r;

// Cache FSM.
localparam [2:0] S_RESET_INIT  = 3'd0;
localparam [2:0] S_IDLE        = 3'd1;
localparam [2:0] S_LOOKUP      = 3'd2;
localparam [2:0] S_FILL        = 3'd3;
localparam [2:0] S_BYPASS_WAIT = 3'd4;

reg [2:0] state;
reg [SET_BITS-1:0] init_set;
reg [SET_BITS-1:0] snoop_set_r;
reg snoop_valid_r;
reg [WORD_OFFSET_BITS-1:0] fill_count;
reg [WORD_OFFSET_BITS-1:0] fill_target_word;
reg [SET_BITS-1:0] fill_set;
reg [TAG_BITS-1:0] fill_tag;
reg [1:0] fill_way;
reg [2:0] fill_plru_r;
reg fill_requested;
reg fill_target_returned;
reg [127:0] wide_fill_line;
reg wide_fill_install;
reg fill_direct_r;          // the fill answers vipt_fill_resp, not the CPU port
reg fill_resp_valid_r;
reg probe_in_fill_r;        // the resolving probe preread during a fill

reg [31:0] dout_r;
reg resp_valid_r;
reg ready_r;

assign cpu_ready = ready_r;
assign store_patch_addr = req_addr_r;
assign store_patch_data = req_din_r;
assign store_patch_be = req_be_r;
assign store_patch_valid = (state == S_LOOKUP) && req_valid_r &&
                           req_write_r && !req_protect_write_r;

function automatic [31:0] be_mask(input [3:0] be);
begin
    be_mask = {{8{be[3]}}, {8{be[2]}}, {8{be[1]}}, {8{be[0]}}};
end
endfunction

function automatic [31:0] merge32(input [31:0] old_data, input [31:0] new_data, input [3:0] be);
    automatic reg [31:0] mask;
begin
    mask = be_mask(be);
    merge32 = (old_data & ~mask) | (new_data & mask);
end
endfunction

function automatic [127:0] forward_storeq_line_slot(
    input [127:0] value,
    input         slot_live,
    input  [29:0] slot_addr,
    input  [31:0] slot_data,
    input   [3:0] slot_be,
    input  [27:0] line_addr
);
    automatic reg [127:0] result;
begin
    result = value;
    if (slot_live && slot_addr[29:2] == line_addr)
        result[{slot_addr[1:0], 5'b0} +: 32] =
            merge32(result[{slot_addr[1:0], 5'b0} +: 32], slot_data, slot_be);
    forward_storeq_line_slot = result;
end
endfunction

function automatic [31:0] forward_storeq_slot(
    input [31:0] value,
    input        slot_live,
    input [29:0] slot_addr,
    input [31:0] slot_data,
    input  [3:0] slot_be,
    input [29:0] addr_dw
);
begin
    forward_storeq_slot = (slot_live && slot_addr == addr_dw) ?
                          merge32(value, slot_data, slot_be) : value;
end
endfunction

function automatic [STOREQ_IDX_BITS-1:0] storeq_next_idx(input [STOREQ_IDX_BITS-1:0] idx);
begin
    storeq_next_idx = (idx == STOREQ_LAST_IDX) ? {STOREQ_IDX_BITS{1'b0}} : (idx + 1'b1);
end
endfunction

// Slot holding the k-th oldest entry.
function automatic [STOREQ_IDX_BITS-1:0] storeq_age_idx(
    input [STOREQ_IDX_BITS-1:0] tail, input integer k);
    integer i;
begin
    i = tail + k;
    storeq_age_idx = STOREQ_IDX_BITS'((i >= STOREQ_DEPTH) ? i - STOREQ_DEPTH : i);
end
endfunction

function automatic [STOREQ_IDX_BITS-1:0] storeq_prev_idx(input [STOREQ_IDX_BITS-1:0] idx);
begin
    storeq_prev_idx = (idx == {STOREQ_IDX_BITS{1'b0}}) ? STOREQ_LAST_IDX : (idx - 1'b1);
end
endfunction

function automatic [1:0] way_encode(input [3:0] hit_vec);
begin
    way_encode = hit_vec[0] ? 2'd0 :
                 hit_vec[1] ? 2'd1 :
                 hit_vec[2] ? 2'd2 : 2'd3;
end
endfunction

function automatic [31:0] way_data_mux(
    input [1:0] way,
    input [31:0] data0,
    input [31:0] data1,
    input [31:0] data2,
    input [31:0] data3
);
begin
    case (way)
        2'd0: way_data_mux = data0;
        2'd1: way_data_mux = data1;
        2'd2: way_data_mux = data2;
        default: way_data_mux = data3;
    endcase
end
endfunction

function automatic [2:0] plru_update(input [2:0] plru, input [1:0] way);
begin
    case (way)
        2'd0: plru_update = {plru[2], 1'b1, 1'b1};
        2'd1: plru_update = {plru[2], 1'b0, 1'b1};
        2'd2: plru_update = {1'b1, plru[1], 1'b0};
        default: plru_update = {1'b0, plru[1], 1'b0};
    endcase
end
endfunction

function automatic [1:0] plru_victim(input [2:0] plru);
begin
    if (!plru[0])
        plru_victim = plru[1] ? 2'd1 : 2'd0;
    else
        plru_victim = plru[2] ? 2'd3 : 2'd2;
end
endfunction

wire [3:0] lookup_hit_vec = {
    rd_valid3_r && (rd_tag3_r == req_tag_r),
    rd_valid2_r && (rd_tag2_r == req_tag_r),
    rd_valid1_r && (rd_tag1_r == req_tag_r),
    rd_valid0_r && (rd_tag0_r == req_tag_r)
};
wire lookup_hit = |lookup_hit_vec;
wire [1:0] lookup_way = way_encode(lookup_hit_vec);
wire [31:0] lookup_way_ram_data = way_data_mux(lookup_way, rd_data0_r, rd_data1_r, rd_data2_r, rd_data3_r);
// A read may preread its dword in the cycle a store patches the data RAM (a
// pipelined store, or a VIPT probe below). When both name the same dword and
// way, take the registered patch value rather than the RAM read that raced it;
// a read that came after the patch merges the same bytes harmlessly.
reg         patch_fwd_valid_r;
reg  [1:0]  patch_fwd_way_r;
reg  [31:0] patch_fwd_data_r;
reg  [SET_BITS+WORD_OFFSET_BITS-1:0] patch_fwd_addr_r;
wire patch_fwd_hit = patch_fwd_valid_r && (patch_fwd_way_r == lookup_way) &&
                     (patch_fwd_addr_r == {req_set_r, req_word_r});
wire [31:0] lookup_way_data = patch_fwd_hit ? patch_fwd_data_r : lookup_way_ram_data;
wire [TAG_BITS-1:0] vipt_resolve_tag =
    vipt_resolve_phys_addr[TAG_MSB:TAG_LSB];
// A probe that preread during a fill must not hit the way being refilled in
// that set: its old tag stays valid until the last dword while the data RAM
// already holds part of the new line (fill_probe_mask_r, set at the preread).
reg  [3:0] fill_probe_mask_r;
wire [3:0] vipt_hit_vec = ~fill_probe_mask_r & {
    rd_valid3_r && (rd_tag3_r == vipt_resolve_tag),
    rd_valid2_r && (rd_tag2_r == vipt_resolve_tag),
    rd_valid1_r && (rd_tag1_r == vipt_resolve_tag),
    rd_valid0_r && (rd_tag0_r == vipt_resolve_tag)
};
wire [1:0] vipt_hit_way = way_encode(vipt_hit_vec);
wire [31:0] vipt_way_ram_data = way_data_mux(
    vipt_hit_way, rd_data0_r, rd_data1_r, rd_data2_r, rd_data3_r);
// A probe may preread in the cycle a store patches the same dword (the probe
// shares the read port with a store's S_LOOKUP). Take the registered patch
// value over the RAM read that raced it; set/word bits are page-offset bits.
wire vipt_patch_fwd_hit = patch_fwd_valid_r && (patch_fwd_way_r == vipt_hit_way) &&
    (patch_fwd_addr_r == vipt_resolve_phys_addr[SET_MSB:BYTE_OFFSET_BITS]);
wire [31:0] vipt_way_data = vipt_patch_fwd_hit ? patch_fwd_data_r : vipt_way_ram_data;
// A younger VIPT lookup may share the preread used to capture an older store
// to the same word. The store reaches S_LOOKUP while that younger request
// finalizes, so forward its registered bytes over the RAM's old data.
wire vipt_lookup_store_match = (state == S_LOOKUP) && req_valid_r &&
    req_write_r && !req_protect_write_r &&
    (req_addr_r[31:2] == vipt_resolve_phys_addr[31:2]);
assign vipt_resolve_data = vipt_lookup_store_match
                         ? merge32(vipt_way_data, req_din_r, req_be_r)
                         : vipt_way_data;
assign vipt_resolve_hit = vipt_resolve_valid && cache_enable &&
                          (vipt_resolve_phys_addr[31:17] != 15'h5) &&
                          (|vipt_hit_vec);
// Fill straight from a direct-load probe miss: the probe's preread holds the
// set's tags and PLRU and its translation is final, so the line is requested
// without a CPU-port request and its second lookup. A probe that preread
// during a fill may miss a line that fill has just installed, so it takes
// the CPU port, whose lookup sees the installed tag.
wire vipt_fill_start_now = vipt_resolve_valid && vipt_resolve_fill && cache_enable &&
                           (vipt_resolve_phys_addr[31:17] != 15'h5) &&
                           !(|vipt_hit_vec) && !probe_in_fill_r &&
                           (state == S_IDLE) && ready_r && !cpu_valid;
assign vipt_fill_start = vipt_fill_start_now;
assign vipt_fill_resp_valid = fill_resp_valid_r;
assign vipt_fill_resp_data = dout_r;
wire [BRAM_ADDR_BITS-1:0] req_bram_addr = {req_set_r, req_word_r};
wire can_accept_cpu = (state == S_IDLE) && !reset && (!cpu_write || cpu_protect_write || storeq_can_accept);
wire ready_when_idle = !reset && storeq_can_accept;
// Store pipelining, like the i486 write buffer taking one store per clock:
// while a store enqueues in S_LOOKUP, accept the next store and preread its
// set on the free RAM read port. lookup_wr_room_r holds the queue capacity for
// that store, computed when the current store was accepted. The opening is
// registered state only: requesters see it through cpu_wr_ready without any
// address, TLB or request-type term.
reg  lookup_wr_room_r;
wire lookup_store_busy = (state == S_LOOKUP) && req_valid_r &&
                         req_write_r && !req_protect_write_r;
wire lookup_wr_open = lookup_store_busy && lookup_wr_room_r && !store_patch_busy;
assign cpu_wr_ready = ready_r || lookup_wr_open;
wire lookup_wr_accept = cpu_valid && cpu_write && lookup_wr_open;
wire accept_cpu = (cpu_valid && ready_r && can_accept_cpu) || lookup_wr_accept;
wire [29:0] req_addr_dw = req_addr_r[31:2];
wire [29:0] fill_addr_dw = {req_addr_r[31:4], fill_count};
// The fill's last dword: the one before its first (word 0 unless wrapping).
wire fill_last_word = LINE_WRAP ? ((fill_count + 1'b1) == fill_target_word)
                                : (fill_count == {WORD_OFFSET_BITS{1'b1}});
logic [31:0] lookup_forward_data;
logic [31:0] fill_word_data;
logic [31:0] bypass_forward_data;
logic [127:0] wide_line_data;
wire lookup_read_hit_now = (state == S_LOOKUP) && req_valid_r &&
                           !req_write_r && !req_uncacheable_r && lookup_hit;

assign cpu_dout = lookup_read_hit_now ? lookup_forward_data : dout_r;
assign cpu_resp_valid = lookup_read_hit_now || resp_valid_r;

// Uncacheable reads cannot bypass posted stores.  Besides preserving normal
// memory ordering, VGA reads depend on all earlier planar writes being visible.
// Keep draining while such a read waits, then reserve the memory port once the
// queue is empty.
// A probe resolving this cycle may start a line fill (vipt_fill_start): keep
// the memory port free for it rather than posting a store ahead of the read,
// unless the store queue is full.
reg  probe_resolving_r;
wire drain_block_state = (state == S_RESET_INIT) || (state == S_FILL) ||
                         (probe_resolving_r && vipt_resolve_fill && !storeq_full) ||
                         (state == S_BYPASS_WAIT) ||
                         ((state == S_LOOKUP) && !req_protect_write_r && !req_write_r &&
                          (req_uncacheable_r ? storeq_empty : !lookup_hit));
wire drain_issue_now = !storeq_empty && !storeq_draining && !mem_valid_r &&
                       !mem_busy && !drain_block_state;

wire [STOREQ_IDX_BITS-1:0] storeq_prev = storeq_prev_idx(storeq_head);
wire storeq_merge_lookup = !storeq_empty && storeq_valid[storeq_prev] &&
                           (storeq_addr[storeq_prev] == req_addr_r[31:2]) &&
                           !req_uncacheable_r &&
                           !(storeq_prev == storeq_tail && storeq_draining);
// The S_LOOKUP write merging into the very entry the drain is launching
// this cycle: fold the incoming bytes into the launch latch too.
wire storeq_merge_wr_now = (state == S_LOOKUP) && req_write_r &&
                           !req_protect_write_r && storeq_merge_lookup;
wire drain_merge_now = storeq_merge_wr_now && (storeq_prev == storeq_tail);
// Store-queue count after this cycle's enqueue, including a simultaneously
// completing drain.  Drives the post-write ready_r so a full queue is seen
// immediately despite the one-cycle-late enqueue.
wire storeq_dequeuing = storeq_draining && mem_ready;
wire [STOREQ_CNT_BITS-1:0] storeq_count_wr_next =
     storeq_merge_lookup ? (storeq_dequeuing ? storeq_count - 1'b1 : storeq_count)
                         : (storeq_dequeuing ? storeq_count : storeq_count + 1'b1);

always_comb begin
    lookup_forward_data = lookup_way_data;
    fill_word_data = mem_dout;
    bypass_forward_data = mem_dout;
    wide_line_data = mem_line_dout;

    // Oldest to youngest, so the youngest matching store's bytes win.
    for (int k = 0; k < STOREQ_DEPTH; k++) begin
        if (k < storeq_count) begin
            lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_valid[storeq_age_idx(storeq_tail, k)], storeq_addr[storeq_age_idx(storeq_tail, k)], storeq_data[storeq_age_idx(storeq_tail, k)], storeq_be[storeq_age_idx(storeq_tail, k)], req_addr_dw);
            fill_word_data = forward_storeq_slot(fill_word_data, storeq_valid[storeq_age_idx(storeq_tail, k)], storeq_addr[storeq_age_idx(storeq_tail, k)], storeq_data[storeq_age_idx(storeq_tail, k)], storeq_be[storeq_age_idx(storeq_tail, k)], fill_addr_dw);
            bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_valid[storeq_age_idx(storeq_tail, k)], storeq_addr[storeq_age_idx(storeq_tail, k)], storeq_data[storeq_age_idx(storeq_tail, k)], storeq_be[storeq_age_idx(storeq_tail, k)], req_addr_dw);
            wide_line_data = forward_storeq_line_slot(wide_line_data, storeq_valid[storeq_age_idx(storeq_tail, k)], storeq_addr[storeq_age_idx(storeq_tail, k)], storeq_data[storeq_age_idx(storeq_tail, k)], storeq_be[storeq_age_idx(storeq_tail, k)], req_addr_r[31:4]);
        end
    end
end

// Preread runs on every ready idle cycle, with no cpu_valid/TLB gating: when
// no request is accepted the preread results are garbage that S_LOOKUP never
// sees (it is only entered on accept_cpu).  This keeps the TLB-hit cone off
// the wide rd_*_r register enables.
wire idle_preread = (state == S_IDLE) && ready_r;
// The store-pipelining preread, likewise ungated by the late accept.
wire lookup_wr_preread = lookup_wr_open;
// A posted store patches at most one data way during S_LOOKUP.  The inferred
// cache RAMs have an independent read port, so a VIPT lookup can preread the
// next load in that cycle, in the store's set too: a read that races the patch
// takes the registered patch value (vipt_patch_fwd_hit). Store hits change no
// tag, and a probe that misses falls back to the paging path, which chooses
// any victim from current PLRU state.
wire store_lookup_preread = (state == S_LOOKUP) && req_valid_r &&
                            req_write_r && !req_protect_write_r;
// Store hits patch the cache before it can accept another probe; store misses
// have no matching line, and later fills are patched before the tag is valid.
// Capacity is a registered-state fact. Demand arbitration must not feed back
// through D2 issue; a denied speculative probe is replayed by the CPU.
// Hit-under-fill: once a fill has returned the missing dword to its
// requester, VIPT probes may read the other lines while the rest of the line
// streams in on the write port. A probe miss retries through the CPU
// interface, which waits for the fill; demand requests and stores still do.
wire fill_probe_open = (state == S_FILL) && fill_target_returned;
assign vipt_probe_ready = idle_preread || store_lookup_preread || fill_probe_open;
wire vipt_probe_fire = vipt_probe_valid &&
                       (idle_preread || store_lookup_preread || fill_probe_open) &&
                       !cpu_preread_priority;
// When an accepted demand store owns the preread RAM at the same page offset,
// its RAM address is also the exact VIPT lookup address. Share that read
// instead of rejecting and replaying the younger instruction. Demand reads
// retain exclusive priority and the existing replay contract.
wire vipt_probe_share = vipt_probe_valid && cpu_preread_priority &&
                        idle_preread && cpu_write &&
                        (vipt_probe_set == cpu_preread_set) &&
                        (vipt_probe_word == cpu_preread_word);
assign vipt_probe_accepted = vipt_probe_fire || vipt_probe_share;
// Replay is mutually exclusive with a demand request and therefore cannot
// use the address-qualified sharing arm.  Expose the direct acceptance fact
// so its EX capture does not inherit the paging preread address cone.
assign vipt_probe_direct_accepted = vipt_probe_fire;
wire [SET_BITS-1:0] preread_set =
    vipt_probe_fire ? vipt_probe_set : cpu_preread_set;
wire [WORD_OFFSET_BITS-1:0] preread_word =
    vipt_probe_fire ? vipt_probe_word : cpu_preread_word;
wire [BRAM_ADDR_BITS-1:0] preread_bram_addr = {preread_set, preread_word};
// A store that hits patches the line even while the cache is disabled, so the
// cache stays coherent and can be re-enabled without a flush.
wire data_store_write = (state == S_LOOKUP) && req_valid_r && req_write_r &&
                        !req_protect_write_r && lookup_hit;
wire data_fill_write = (state == S_FILL) &&
                       (mem_resp_valid || wide_fill_install);
wire [1:0] data_write_way = data_store_write ? lookup_way : fill_way;

always_ff @(posedge clk) begin
    if (reset) begin
        fill_probe_mask_r <= 4'h0;
        probe_in_fill_r <= 1'b0;
    end else if (vipt_probe_fire) begin
        fill_probe_mask_r <= (fill_probe_open && (vipt_probe_set == fill_set))
                           ? (4'h1 << fill_way) : 4'h0;
        probe_in_fill_r <= fill_probe_open;
    end
end

always_ff @(posedge clk)
    probe_resolving_r <= !reset && vipt_probe_fire;

always_ff @(posedge clk) begin
    if (reset)
        patch_fwd_valid_r <= 1'b0;
    else
        patch_fwd_valid_r <= data_store_write;
    patch_fwd_way_r <= lookup_way;
    patch_fwd_addr_r <= {req_set_r, req_word_r};
    patch_fwd_data_r <= merge32(lookup_way_data, req_din_r, req_be_r);
end

// synthesis translate_off
// A valid demand request always owns the preread, so a pipelined store never
// shares the RAM read port with a VIPT probe.
always @(posedge clk)
    if (!reset && lookup_wr_accept && !cpu_preread_priority)
        $fatal(1, "L1 pipelined store accepted without preread priority");
// synthesis translate_on
wire [BRAM_ADDR_BITS-1:0] data_write_addr = data_store_write ?
                                            req_bram_addr :
                                            {fill_set, fill_count};
wire [31:0] data_write_value = data_store_write ?
                               merge32(lookup_way_data, req_din_r, req_be_r) :
                               wide_fill_install ?
                               wide_fill_line[{fill_count, 5'b0} +: 32] :
                               fill_word_data;

// Keep each tag array in one conventional synchronous-read/synchronous-write
// process. Quartus 17 will not infer a block RAM when the packed valid bit is
// written from the snoop, reset-init, and fill branches of the cache FSM.
wire tag_fill_write = (state == S_FILL) &&
                      (mem_resp_valid || wide_fill_install) &&
                      fill_last_word;
wire tag_clear_all = (state == S_RESET_INIT) || snoop_valid_r;
wire [SET_BITS-1:0] tag_clear_set = (state == S_RESET_INIT) ?
                                    init_set : snoop_set_r;
wire [TAG_RAM_BITS-1:0] tag_fill_entry =
    {{(TAG_RAM_BITS-TAG_BITS-1){1'b0}}, 1'b1, fill_tag};
wire tag_fill_way0 = tag_fill_write && (fill_way == 2'd0);
wire tag_fill_way1 = tag_fill_write && (fill_way == 2'd1);
wire tag_fill_way2 = tag_fill_write && (fill_way == 2'd2);
wire tag_fill_way3 = tag_fill_write && (fill_way == 2'd3);

always_ff @(posedge clk) begin
    if (idle_preread || lookup_wr_preread || vipt_probe_fire) begin
        rd_tag_entry0_r <= tag_way0[preread_set];
        rd_tag_entry1_r <= tag_way1[preread_set];
        rd_tag_entry2_r <= tag_way2[preread_set];
        rd_tag_entry3_r <= tag_way3[preread_set];
        rd_data0_r <= data_way0[preread_bram_addr];
        rd_data1_r <= data_way1[preread_bram_addr];
        rd_data2_r <= data_way2[preread_bram_addr];
        rd_data3_r <= data_way3[preread_bram_addr];
        rd_plru_r <= plru_set[preread_set];
    end

    // A single process for both ports is recognized as simple dual-port RAM
    // by Vivado and Quartus. The former split-process task form mapped the
    // cache storage to registers in Vivado.
    if (data_store_write || data_fill_write) begin
        case (data_write_way)
            2'd0: data_way0[data_write_addr] <= data_write_value;
            2'd1: data_way1[data_write_addr] <= data_write_value;
            2'd2: data_way2[data_write_addr] <= data_write_value;
            default: data_way3[data_write_addr] <= data_write_value;
        endcase
    end

    if (tag_clear_all || tag_fill_way0)
        tag_way0[tag_fill_way0 ? fill_set : tag_clear_set] <=
            tag_fill_way0 ? tag_fill_entry : '0;
    if (tag_clear_all || tag_fill_way1)
        tag_way1[tag_fill_way1 ? fill_set : tag_clear_set] <=
            tag_fill_way1 ? tag_fill_entry : '0;
    if (tag_clear_all || tag_fill_way2)
        tag_way2[tag_fill_way2 ? fill_set : tag_clear_set] <=
            tag_fill_way2 ? tag_fill_entry : '0;
    if (tag_clear_all || tag_fill_way3)
        tag_way3[tag_fill_way3 ? fill_set : tag_clear_set] <=
            tag_fill_way3 ? tag_fill_entry : '0;
end

always_ff @(posedge clk) begin
    if (reset) begin
        state <= S_RESET_INIT;
        init_set <= {SET_BITS{1'b0}};
        req_valid_r <= 1'b0;
        ready_r <= 1'b0;
        resp_valid_r <= 1'b0;
        dout_r <= 32'h0;
        mem_valid_r <= 1'b0;
        mem_write_r <= 1'b0;
        mem_addr_r <= 32'h0;
        mem_din_r <= 32'h0;
        mem_be_r <= 4'h0;
        mem_burstcount_r <= 8'h0;
        storeq_head <= {STOREQ_IDX_BITS{1'b0}};
        storeq_tail <= {STOREQ_IDX_BITS{1'b0}};
        storeq_count <= {STOREQ_CNT_BITS{1'b0}};
        storeq_draining <= 1'b0;
        lookup_wr_room_r <= 1'b0;
        fill_requested <= 1'b0;
        fill_target_returned <= 1'b0;
        wide_fill_line <= 128'd0;
        wide_fill_install <= 1'b0;
        fill_direct_r <= 1'b0;
        fill_resp_valid_r <= 1'b0;
        snoop_set_r <= {SET_BITS{1'b0}};
        snoop_valid_r <= 1'b0;
        for (integer i = 0; i < STOREQ_DEPTH; i = i + 1)
            storeq_valid[i] <= 1'b0;
    end else begin
        ready_r <= (state == S_IDLE) && ready_when_idle;
        resp_valid_r <= 1'b0;
        fill_resp_valid_r <= 1'b0;
        snoop_valid_r <= snoop_valid;
        if (snoop_valid)
            snoop_set_r <= snoop_set;

        if (mem_valid_r && mem_ready)
            mem_valid_r <= 1'b0;

        if (storeq_draining && mem_ready) begin
            storeq_valid[storeq_tail] <= 1'b0;
            storeq_tail <= storeq_next_idx(storeq_tail);
            storeq_count <= storeq_count - 1'b1;
            storeq_draining <= 1'b0;
        end

        // FSM-independent store-queue drain launch (see drain_issue_now).
        if (drain_issue_now) begin
            mem_valid_r <= 1'b1;
            mem_write_r <= 1'b1;
            mem_addr_r <= {storeq_addr[storeq_tail], 2'b00};
            // A same-cycle merge into this very entry (drain_merge_now) must
            // reach memory: latch the merged data/be, matching what the
            // storeq entry itself is updated to this cycle.
            mem_din_r <= drain_merge_now
                       ? merge32(storeq_data[storeq_tail], req_din_r, req_be_r)
                       : storeq_data[storeq_tail];
            mem_be_r <= drain_merge_now ? (storeq_be[storeq_tail] | req_be_r)
                                        : storeq_be[storeq_tail];
            mem_burstcount_r <= 8'd1;
            storeq_draining <= 1'b1;
        end

        case (state)
            S_RESET_INIT: begin
                plru_set[init_set] <= 3'b000;
                if (init_set == LAST_SET) begin
                    state <= S_IDLE;
                    ready_r <= ready_when_idle;
                end else begin
                    init_set <= init_set + 1'b1;
                end
            end

            S_IDLE: begin
                // Wide request captures run on every ready cycle, with no
                // cpu_valid/TLB gating: garbage is captured when nothing is
                // accepted, but S_LOOKUP (the only consumer) is entered on
                // accept_cpu alone.  Keeps the TLB cone off these enables.
                if (ready_r) begin
                    req_addr_r <= cpu_addr;
                    req_din_r <= cpu_din;
                    req_be_r <= cpu_be;
                    req_write_r <= cpu_write;
                    req_uncacheable_r <= request_uncacheable;
                    req_protect_write_r <= cpu_protect_write;
                    req_tag_r <= cpu_tag;
                    req_set_r <= cpu_set;
                    req_word_r <= cpu_word;
                end
                if (accept_cpu) begin
                    ready_r <= 1'b0;
                    req_valid_r <= 1'b1;
                    state <= S_LOOKUP;
                end else if (vipt_fill_start_now) begin
                    ready_r <= 1'b0;
                    req_addr_r <= vipt_resolve_phys_addr;
                    fill_set <= vipt_resolve_phys_addr[SET_MSB:SET_LSB];
                    fill_tag <= vipt_resolve_tag;
                    fill_way <= plru_victim(rd_plru_r);
                    fill_plru_r <= rd_plru_r;
                    fill_count <= LINE_WRAP
                        ? vipt_resolve_phys_addr[LINE_OFFSET_BITS-1:BYTE_OFFSET_BITS]
                        : {WORD_OFFSET_BITS{1'b0}};
                    fill_target_word <=
                        vipt_resolve_phys_addr[LINE_OFFSET_BITS-1:BYTE_OFFSET_BITS];
                    fill_requested <= 1'b0;
                    fill_target_returned <= 1'b0;
                    wide_fill_install <= 1'b0;
                    fill_direct_r <= 1'b1;
                    state <= S_FILL;
                end
                lookup_wr_room_r <= (storeq_count <= STOREQ_CNT_BITS'(STOREQ_DEPTH - 2));
            end

            S_LOOKUP: begin
                req_valid_r <= 1'b0;

                if (req_protect_write_r) begin
                    state <= S_IDLE;
                    ready_r <= ready_when_idle;
                end else if (req_write_r) begin
                    // Store-queue enqueue, moved here from the accept cycle so
                    // its enables come from registered request state instead of
                    // the TLB-gated accept.  All inputs are req_*_r registers.
                    if (storeq_merge_lookup) begin
                        // Same-DWORD coalescing: fold into the newest entry.
                        storeq_data[storeq_prev] <= merge32(storeq_data[storeq_prev], req_din_r, req_be_r);
                        storeq_be[storeq_prev] <= storeq_be[storeq_prev] | req_be_r;
                    end else begin
                        storeq_addr[storeq_head] <= req_addr_r[31:2];
                        storeq_data[storeq_head] <= req_din_r;
                        storeq_be[storeq_head] <= req_be_r;
                        storeq_valid[storeq_head] <= 1'b1;
                        storeq_head <= storeq_next_idx(storeq_head);
                    end
                    storeq_count <= storeq_count_wr_next;
                    if (lookup_hit && !req_uncacheable_r) begin
                        plru_set[req_set_r] <= plru_update(rd_plru_r, lookup_way);
                    end
                    // Capture the pipelined store like S_IDLE does: on the
                    // registered opening, not on the TLB-qualified accept.
                    if (lookup_wr_open) begin
                        req_addr_r <= cpu_addr;
                        req_din_r <= cpu_din;
                        req_be_r <= cpu_be;
                        req_write_r <= cpu_write;
                        req_uncacheable_r <= request_uncacheable;
                        req_protect_write_r <= cpu_protect_write;
                        req_tag_r <= cpu_tag;
                        req_set_r <= cpu_set;
                        req_word_r <= cpu_word;
                    end
                    lookup_wr_room_r <= (storeq_count_wr_next <=
                                         STOREQ_CNT_BITS'(STOREQ_DEPTH - 2));
                    if (lookup_wr_accept) begin
                        req_valid_r <= 1'b1;
                        ready_r <= 1'b0;
                    end else begin
                        state <= S_IDLE;
                        ready_r <= (storeq_count_wr_next != STOREQ_DEPTH_VALUE);
                    end
                end else if (req_uncacheable_r) begin
                    if (storeq_empty && !storeq_draining && !mem_valid_r && !mem_busy) begin
                        mem_valid_r <= 1'b1;
                        mem_write_r <= 1'b0;
                        mem_addr_r <= req_addr_r;
                        mem_din_r <= 32'h0;
                        mem_be_r <= req_be_r;
                        mem_burstcount_r <= 8'd1;
                        state <= S_BYPASS_WAIT;
                    end
                end else if (lookup_hit) begin
                    plru_set[req_set_r] <= plru_update(rd_plru_r, lookup_way);
                    state <= S_IDLE;
                    ready_r <= ready_when_idle;
                end else begin
                    fill_set <= req_set_r;
                    fill_tag <= req_tag_r;
                    fill_way <= plru_victim(rd_plru_r);
                    fill_plru_r <= rd_plru_r;
                    fill_count <= LINE_WRAP ? req_word_r : {WORD_OFFSET_BITS{1'b0}};
                    fill_target_word <= req_word_r;
                    fill_requested <= 1'b0;
                    fill_target_returned <= 1'b0;
                    wide_fill_install <= 1'b0;
                    fill_direct_r <= 1'b0;
                    state <= S_FILL;
                end
            end

            S_FILL: begin
                if (!fill_requested && !mem_valid_r && !mem_busy) begin
                    mem_valid_r <= 1'b1;
                    mem_write_r <= 1'b0;
                    mem_addr_r <= LINE_WRAP ? {req_addr_r[31:4], fill_target_word, 2'b00}
                                            : {req_addr_r[31:4], 4'b0000};
                    mem_din_r <= 32'h0;
                    mem_be_r <= 4'hF;
                    mem_burstcount_r <= 8'd4;
                    fill_requested <= 1'b1;
                end

                if (wide_fill_install) begin
                    if (fill_last_word) begin
                        plru_set[fill_set] <= plru_update(fill_plru_r, fill_way);
                        wide_fill_install <= 1'b0;
                        state <= S_IDLE;
                        ready_r <= ready_when_idle;
                    end
                    fill_count <= fill_count + 1'b1;
                end else if (mem_line_resp_valid) begin
                    wide_fill_line <= wide_line_data;
                    wide_fill_install <= 1'b1;
                    dout_r <= wide_line_data[{fill_target_word, 5'b0} +: 32];
                    if (fill_direct_r)
                        fill_resp_valid_r <= 1'b1;
                    else
                        resp_valid_r <= 1'b1;
                    fill_target_returned <= 1'b1;
                end else if (mem_resp_valid) begin
                    if (fill_count == fill_target_word && !fill_target_returned) begin
                        dout_r <= fill_word_data;
                        if (fill_direct_r)
                            fill_resp_valid_r <= 1'b1;
                        else
                            resp_valid_r <= 1'b1;
                        fill_target_returned <= 1'b1;
                    end

                    if (fill_last_word) begin
                        // Do NOT restore the other ways' valid bits from the
                        // fill-START snapshot -- a snoop invalidation landing
                        // DURING this fill must survive (same bug as l1_icache).
                        plru_set[fill_set] <= plru_update(fill_plru_r, fill_way);
                        state <= S_IDLE;
                        ready_r <= ready_when_idle;
                    end
                    fill_count <= fill_count + 1'b1;
                end
            end

            S_BYPASS_WAIT: begin
                if (mem_resp_valid) begin
                    dout_r <= bypass_forward_data;
                    resp_valid_r <= 1'b1;
                    state <= S_IDLE;
                    ready_r <= ready_when_idle;
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

// synthesis translate_off
always_ff @(posedge clk) begin
    if (!reset && state != S_RESET_INIT && cpu_valid && !cpu_ready && !(state == S_IDLE))
        ;
    // A VIPT probe sharing an accepted store's preread resolves while that
    // registered store is in S_LOOKUP. vipt_lookup_store_match forwards the
    // store payload over the preread result, so this is an intentional second
    // legal finalize state rather than a cache ownership collision. A
    // protected (ROM-window) store leaves S_LOOKUP without touching the RAM
    // and is not forwarded, so the probe reads the unchanged data.
    // A probe that preread during a fill resolves in the fill or just after.
    if (!reset && vipt_resolve_valid && state != S_IDLE && state != S_FILL &&
        !(state == S_LOOKUP && req_valid_r && req_write_r))
        $fatal(1, "VIPT resolve while cache is not idle");
end
// synthesis translate_on

endmodule
