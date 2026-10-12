//
// TLB for 80386 Paging Unit
// 32-entry 4-way set-associative TLB with pseudo-LRU, plus a 256-entry direct-mapped sidecar TLB
//
`timescale 1ns/1ns
`include "z486_platform.svh"

module paging_tlb
    import z486_pkg::*;
(
    input               clk,
    input               reset_n,

    // Registered lookup interface (combinational output)
    input        [31:0] linear_addr,
    output reg          hit,
    output reg   [31:0] physical_addr,
    output reg          writable,       // Combined PDE & PTE R/W
    output reg          user,           // Combined PDE & PTE U/S
    output reg          dirty,          // D bit from PTE
    output              is_vga_mem,     // Physical address is in A0000-BFFFF

    // Side-effect-free D2 preread for hardwired loads. The direct-mapped
    // sidecar is a second TLB lookup port; an EX miss simply falls back to the
    // authoritative four-way TLB and page walker.
    input               vipt_preread,
    input        [31:0] vipt_linear_addr,
    output reg          vipt_hit,
    output reg   [31:0] vipt_physical_addr,
    output reg          vipt_writable,
    output reg          vipt_user,
    output reg          vipt_dirty,
    output              vipt_is_vga_mem,

    // Refill the direct sidecar after a registered demand falls back to an
    // authoritative four-way TLB hit.  This path is intentionally separate
    // from the D2 preread: it cannot feed translation back into D2.
    input               vipt_refill_valid,
    input        [31:0] vipt_refill_linear,
    input        [19:0] vipt_refill_pfn,
    input               vipt_refill_writable,
    input               vipt_refill_user,
    input               vipt_refill_dirty,

    // Update interface (from page walker)
    input               update_valid,
    input        [19:0] update_vpn,     // Virtual page number
    input        [19:0] update_pfn,     // Physical frame number
    input               update_writable,
    input               update_user,
    input               update_dirty,

    // Invalidate all entries (on CR3 write)
    input               invalidate_all,

    // Invalidate every cached translation for one linear page (INVLPG).
    input               invalidate_page,
    input        [19:0] invalidate_vpn
);

// 8 sets x 4 ways, one lookup port as on the i486: the registered lookup
// address carries walker, demand and prefetch requests in that priority. Tags
// and PFNs live in MLAB; the small per-entry flags, valid bits and PLRU state
// stay in registers.
// MLABs have one write port and asynchronous reads, so each copy costs about
// 20 ALMs where the register array cost a flip-flop per bit plus an 8:1 read
// mux per bit and port.
reg valid_q    [7:0][3:0];
reg writable_q [7:0][3:0];
reg user_q     [7:0][3:0];
reg dirty_q    [7:0][3:0];
reg vga_mem    [7:0][3:0];

reg [2:0] plru [7:0];

localparam bit TRACE_PAGING_EN = 1'b0;

// Registered lookup address decomposition
wire [19:0] lookup_vpn = linear_addr[31:12];
wire [2:0]  lookup_set = lookup_vpn[2:0];       // Set index: VPN[2:0]
wire [16:0] lookup_tag = lookup_vpn[19:3];       // Tag: VPN[19:3]

// Walker refills always target the registered lookup address
// (paging_unit drives update_vpn from tlb_lookup_addr), so the lookup port's
// hit vector doubles as the refill's existing-entry match.
wire [2:0]  update_set = update_vpn[2:0];
wire [16:0] update_tag = update_vpn[19:3];
wire [2:0]  invalidate_set = invalidate_vpn[2:0];
wire        tlb_write = reset_n && !invalidate_all && !invalidate_page && update_valid;
wire [1:0]  victim_way;

logic [16:0] lookup_tag_q [4];
logic [19:0] lookup_pfn_q [4];

// Quartus 17 does not apply ramstyle to memories declared inside a generate
// block (they stay as registers), so the four copies are written out flat.
`Z486_DISTRIBUTED_RAM reg [36:0] lookup_copy0 [0:7];
`Z486_DISTRIBUTED_RAM reg [36:0] lookup_copy1 [0:7];
`Z486_DISTRIBUTED_RAM reg [36:0] lookup_copy2 [0:7];
`Z486_DISTRIBUTED_RAM reg [36:0] lookup_copy3 [0:7];
always_ff @(posedge clk) begin
    if (tlb_write && victim_way == 2'd0) begin
        lookup_copy0[update_set] <= {update_tag, update_pfn};
    end
    if (tlb_write && victim_way == 2'd1) begin
        lookup_copy1[update_set] <= {update_tag, update_pfn};
    end
    if (tlb_write && victim_way == 2'd2) begin
        lookup_copy2[update_set] <= {update_tag, update_pfn};
    end
    if (tlb_write && victim_way == 2'd3) begin
        lookup_copy3[update_set] <= {update_tag, update_pfn};
    end
end
assign {lookup_tag_q[0], lookup_pfn_q[0]} = lookup_copy0[lookup_set];
assign {lookup_tag_q[1], lookup_pfn_q[1]} = lookup_copy1[lookup_set];
assign {lookup_tag_q[2], lookup_pfn_q[2]} = lookup_copy2[lookup_set];
assign {lookup_tag_q[3], lookup_pfn_q[3]} = lookup_copy3[lookup_set];

// Hit detection - combinational, parallel comparison within selected set
wire hit0 = valid_q[lookup_set][0] && (lookup_tag_q[0] == lookup_tag);
wire hit1 = valid_q[lookup_set][1] && (lookup_tag_q[1] == lookup_tag);
wire hit2 = valid_q[lookup_set][2] && (lookup_tag_q[2] == lookup_tag);
wire hit3 = valid_q[lookup_set][3] && (lookup_tag_q[3] == lookup_tag);

// Compute device classification per way, in parallel with hit detection. This
// avoids putting the selected-PFN mux on cache request routing controls.
assign is_vga_mem = (hit0 && vga_mem[lookup_set][0]) ||
                    (hit1 && vga_mem[lookup_set][1]) ||
                    (hit2 && vga_mem[lookup_set][2]) ||
                    (hit3 && vga_mem[lookup_set][3]);

// Encode hit into 2-bit way index
wire [1:0] hit_way = hit0 ? 2'd0 :
                     hit1 ? 2'd1 :
                     hit2 ? 2'd2 :
                     hit3 ? 2'd3 : 2'd0;

// The D2 port uses one synchronous RAM read followed by an EX tag compare.
// It is maintained as an independent TLB: retaining a translation after the
// four-way TLB replaces it is valid until software executes INVLPG or reloads
// CR3, just as retaining it in any other TLB entry would be.
// 256 entries, direct-mapped on VPN[19:12]: one M10K, and no way select on
// the hit path (it feeds D2 issue through a direct load's hit). Fewer
// entries let a program's read and write pages collide (Quake: 32 entries
// cost 1.9%).
// Validity is an epoch tag stored with each entry, so the RAM holds it and no
// flip-flop array or 256:1 select does: an entry is valid when its epoch
// equals vipt_epoch. A CR3 write advances the epoch; INVLPG writes the
// INVALID code to its index. Live epochs run 0-14, so a wrap could revive an
// entry from 15 flushes earlier: a wrap (and reset, since the RAM keeps its
// contents) starts a scrub that writes INVALID to every index, and the
// sidecar reports misses until the scrub completes.
localparam integer VIPT_TLB_INDEX_BITS = 8;
localparam integer VIPT_TLB_ENTRIES = 1 << VIPT_TLB_INDEX_BITS;
localparam logic [3:0] VIPT_EPOCH_INVALID = 4'hF;
wire [VIPT_TLB_INDEX_BITS-1:0] vipt_preread_index =
    vipt_linear_addr[19:12];
wire [VIPT_TLB_INDEX_BITS-1:0] vipt_refill_index =
    vipt_refill_linear[19:12];
wire vipt_refill_write = vipt_refill_valid && !update_valid;
reg [31:0] vipt_linear_r;
reg        vipt_hazard_r;
reg [3:0]  vipt_epoch;
reg        vipt_scrub;                 // sweeping INVALID over every index
reg [VIPT_TLB_INDEX_BITS-1:0] vipt_scrub_index;
// The write port: INVLPG, then a walker update, then a refill, then the scrub.
wire vipt_scrub_write = vipt_scrub && !invalidate_page && !update_valid &&
                        !vipt_refill_write;
// {epoch, VPN tag[19:8], PFN[19:0], writable, user, dirty, VGA}
`Z486_BLOCK_RAM_NO_RW_CHECK reg [39:0] vipt_tlb [0:VIPT_TLB_ENTRIES-1];
reg [39:0] vipt_tlb_q;
wire       vipt_mutation = invalidate_all || invalidate_page || update_valid ||
                           vipt_refill_write || vipt_scrub_write;

always_ff @(posedge clk) begin
    if (vipt_preread) begin
        vipt_linear_r <= vipt_linear_addr;
        // Any simultaneous mutation conservatively poisons this preread. TLB
        // mutations are rare, and a false collision only takes the normal
        // authoritative lookup; avoiding the live index compares keeps D2 EA
        // formation out of this register input.
        vipt_hazard_r <= vipt_mutation;
        vipt_tlb_q <= vipt_tlb[vipt_preread_index];
    end else if (vipt_mutation) begin
        // A held preread no longer reflects the table.
        vipt_hazard_r <= 1'b1;
    end
end

always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        vipt_epoch <= 4'd0;
        vipt_scrub <= 1'b1;
        vipt_scrub_index <= '0;
    end else begin
        if (invalidate_all) begin
            vipt_epoch <= (vipt_epoch == 4'd14) ? 4'd0 : vipt_epoch + 4'd1;
            if (vipt_epoch == 4'd14) begin
                // Restart: entries written since the scrub began may carry
                // epochs the wrap revives.
                vipt_scrub <= 1'b1;
                vipt_scrub_index <= '0;
            end
        end
        if (vipt_scrub_write && !(invalidate_all && vipt_epoch == 4'd14)) begin
            vipt_scrub_index <= vipt_scrub_index + 1'b1;
            if (&vipt_scrub_index)
                vipt_scrub <= 1'b0;
        end
    end
end

wire vipt_match = !vipt_hazard_r && !vipt_scrub &&
                  (vipt_tlb_q[39:36] == vipt_epoch) &&
                  (vipt_tlb_q[35:24] == vipt_linear_r[31:20]);
assign vipt_is_vga_mem = vipt_match && vipt_tlb_q[0];

always_comb begin
    vipt_hit = vipt_match;
    vipt_physical_addr = {vipt_tlb_q[23:4], vipt_linear_r[11:0]};
    vipt_writable = vipt_tlb_q[3];
    vipt_user = vipt_tlb_q[2];
    vipt_dirty = vipt_tlb_q[1];
    if (!vipt_match) begin
        vipt_physical_addr = vipt_linear_r;
        vipt_writable = 1'b0;
        vipt_user = 1'b0;
        vipt_dirty = 1'b0;
    end
end

always_ff @(posedge clk) begin
    if (invalidate_page)
        vipt_tlb[invalidate_vpn[7:0]] <= {VIPT_EPOCH_INVALID, 36'd0};
    else if (update_valid)
        vipt_tlb[update_vpn[7:0]] <= {vipt_epoch, update_vpn[19:8], update_pfn,
                                      update_writable, update_user,
                                      update_dirty,
                                      is_device_page(update_pfn)};
    else if (vipt_refill_write)
        vipt_tlb[vipt_refill_index] <= {vipt_epoch, vipt_refill_linear[31:20],
                                        vipt_refill_pfn,
                                        vipt_refill_writable,
                                        vipt_refill_user,
                                        vipt_refill_dirty,
                                        is_device_page(vipt_refill_pfn)};
    else if (vipt_scrub_write)
        vipt_tlb[vipt_scrub_index] <= {VIPT_EPOCH_INVALID, 36'd0};
end

// synthesis translate_off
// Reference: the flip-flop valid bits the epoch tag replaces. A hit the
// reference rejects is a bug; an extra miss (a scrub overwrote a fresh entry,
// or INVLPG won the write port over an update) is allowed.
reg [VIPT_TLB_ENTRIES-1:0] vipt_valid_ref;
reg                        vipt_valid_ref_q;
always_ff @(posedge clk) begin
    if (!reset_n)
        vipt_valid_ref <= '0;
    else if (invalidate_all)
        vipt_valid_ref <= '0;
    else if (invalidate_page)
        vipt_valid_ref[invalidate_vpn[7:0]] <= 1'b0;
    else if (update_valid)
        vipt_valid_ref[update_vpn[7:0]] <= 1'b1;
    else if (vipt_refill_write)
        vipt_valid_ref[vipt_refill_index] <= 1'b1;
    if (vipt_preread)
        vipt_valid_ref_q <= vipt_valid_ref[vipt_preread_index];
    if (reset_n && vipt_match && !vipt_valid_ref_q)
        $fatal(1, "sidecar hit on an entry the valid-bit reference rejects (linear %08x)",
               vipt_linear_r);
end
// synthesis translate_on

// Output signals - combinational
always_comb begin
    hit = hit0 | hit1 | hit2 | hit3;

    // Select physical address from matching entry
    physical_addr = {lookup_pfn_q[hit_way], linear_addr[11:0]};
    writable = writable_q[lookup_set][hit_way];
    user = user_q[lookup_set][hit_way];
    dirty = dirty_q[lookup_set][hit_way];

    // If no hit, output linear address (will be overridden by page walker result)
    if (!hit) begin
        physical_addr = linear_addr;
        writable = 1'b1;
        user = 1'b0;
        dirty = 1'b0;
    end
end

// PLRU victim selection for the update set (existing entry wins). The
// existing-entry match is the lookup port's hit vector (see update_set above).
assign victim_way = hit0 ? 2'd0 :
                    hit1 ? 2'd1 :
                    hit2 ? 2'd2 :
                    hit3 ? 2'd3 :
                    plru[update_set][0] ? (plru[update_set][2] ? 2'd3 : 2'd2) :
                                          (plru[update_set][1] ? 2'd1 : 2'd0);

// synthesis translate_off
always @(posedge clk)
    if (reset_n && update_valid && update_vpn != lookup_vpn)
        $fatal(1, "TLB refill vpn %05x differs from the registered lookup %05x",
               update_vpn, lookup_vpn);
// synthesis translate_on


// TLB state update and PLRU management. Tag/PFN writes are in the MLAB
// copies above, enabled by tlb_write and victim_way.
integer s;
always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        // Invalidate all entries on reset
        for (s = 0; s < 8; s = s + 1) begin
            valid_q[s][0] <= 1'b0;
            valid_q[s][1] <= 1'b0;
            valid_q[s][2] <= 1'b0;
            valid_q[s][3] <= 1'b0;
            plru[s] <= 3'b000;
        end
    end else if (invalidate_all) begin
        // CR3 write - flush entire TLB
        for (s = 0; s < 8; s = s + 1) begin
            valid_q[s][0] <= 1'b0;
            valid_q[s][1] <= 1'b0;
            valid_q[s][2] <= 1'b0;
            valid_q[s][3] <= 1'b0;
            plru[s] <= 3'b000;
        end
    end else if (invalidate_page) begin
        // INVLPG clears the whole set: dropping extra translations is always
        // legal, and it needs no tag compare (or tag copy) for a rare event.
        valid_q[invalidate_set][0] <= 1'b0;
        valid_q[invalidate_set][1] <= 1'b0;
        valid_q[invalidate_set][2] <= 1'b0;
        valid_q[invalidate_set][3] <= 1'b0;
    end else begin
        // Update PLRU on hit (point away from accessed way in the hit set)
        if (hit) begin
            case (hit_way)
                2'd0: begin plru[lookup_set][0] <= 1'b1; plru[lookup_set][1] <= 1'b1; end
                2'd1: begin plru[lookup_set][0] <= 1'b1; plru[lookup_set][1] <= 1'b0; end
                2'd2: begin plru[lookup_set][0] <= 1'b0; plru[lookup_set][2] <= 1'b1; end
                2'd3: begin plru[lookup_set][0] <= 1'b0; plru[lookup_set][2] <= 1'b0; end
            endcase
        end

        // Insert new entry from page walker
        if (update_valid) begin
            valid_q[update_set][victim_way]    <= 1'b1;
            writable_q[update_set][victim_way] <= update_writable;
            user_q[update_set][victim_way]     <= update_user;
            dirty_q[update_set][victim_way]    <= update_dirty;
            vga_mem[update_set][victim_way]    <= is_device_page(update_pfn);
            case (victim_way)
                2'd0: begin plru[update_set][0] <= 1'b1; plru[update_set][1] <= 1'b1; end
                2'd1: begin plru[update_set][0] <= 1'b1; plru[update_set][1] <= 1'b0; end
                2'd2: begin plru[update_set][0] <= 1'b0; plru[update_set][2] <= 1'b1; end
                2'd3: begin plru[update_set][0] <= 1'b0; plru[update_set][2] <= 1'b0; end
            endcase

            // synthesis translate_off
            if (TRACE_PAGING_EN)
                $display("TLB UPDATE: vpn=%05x pfn=%05x writable=%b user=%b set=%0d victim_way=%0d",
                         update_vpn, update_pfn, update_writable, update_user, update_set, victim_way);
            // synthesis translate_on
        end
    end
end

endmodule
