//
// Data Access Pipeline
// Direct loads, microcode reads, RMW and direct stores through the L1 probe port and sidecar TLB
//
`include "z486_platform.svh"
`default_nettype none
module data_access
    import z486_pkg::*;
(
    // Clock and reset
    input  logic                    clk,
    input  logic                    reset_n,
    // L1 data cache: probe/resolve port and direct store port
    input  logic                    dcache_vipt_probe_accepted,
    input  logic                    dcache_vipt_probe_direct_accepted,
    input  logic                    dcache_vipt_probe_ready,
    input  logic [31:0]             dcache_vipt_resolve_data,
    input  logic                    dcache_vipt_resolve_hit,
    input  logic                    dcache_vipt_fill_start,      // the miss's fill started
    input  logic                    dcache_vipt_fill_resp_valid, // its missing dword
    input  logic [31:0]             dcache_vipt_fill_resp_data,
    input  logic                    dcache_wr_ready,
    input  logic                    fast_store_accepted,
    output logic [11:0]             dcache_vipt_probe_offset,
    output logic                    dcache_vipt_probe_valid,
    output logic [31:0]             dcache_vipt_resolve_phys_addr,
    output logic                    dcache_vipt_resolve_valid,
    output logic                    dcache_vipt_resolve_fill,    // a miss may fill at once
    output logic [3:0]              fast_store_be,
    output logic                    fast_store_valid,
    output logic [31:0]             fast_store_wdata,
    output logic [31:0]             rmw_fast_phys_r,
    output logic [31:0]             st_phys,
    output logic                    st_route,
    output logic                    st_take,
    // Paging unit and sidecar TLB
    input  logic [31:0]             CR0,
    input  logic                    mem_accepted,
    input  logic                    mem_servicing,
    input  logic                    paging_demand_idle,
    input  logic [1:0]              pg_cpl,
    input  logic                    vipt_tlb_dirty,
    input  logic                    vipt_tlb_hit,
    input  logic                    vipt_tlb_is_vga_mem,
    input  logic [31:0]             vipt_tlb_phys_addr,
    input  logic                    vipt_tlb_user,
    input  logic                    vipt_tlb_writable,
    output logic                    sidecar_bg_pre,
    output logic                    st_tlb_pre,
    output logic [1:0]              ucrd_cpl_r,
    output logic                    ucrd_hit,
    output logic [31:0]             ucrd_linear_r,
    output logic                    ucrd_phys_ok_r,
    output logic [31:0]             ucrd_phys_r,
    output logic                    ucrd_route_pre,
    output logic [1:0]              ucrd_size_r,
    output logic                    ucrd_slow_req_r,
    output logic                    ucrd_slow_submit,
    output logic                    ucrd_slow_wait_r,     // missed: in paging
    output logic                    ucrd_take,            // a microcode read probes now
    output logic                    ucrd_x87_r,
    output hardwired_load_payload_t vipt_load_slow_r,
    output logic [31:0]             vipt_probe_linear,
    output logic                    vipt_slow_addr_owned,
    output logic                    vipt_slow_phys_ok_r,
    output logic [31:0]             vipt_slow_phys_r,
    output logic                    vipt_slow_submit,
    // Address and segmentation units
    input  logic                    ds_flat,
    input  logic [31:0]             ind_linear,
    input  logic                    ind_linear_valid,
    input  logic [31:0]             issue_ind_linear,
    input  logic [1:0]              issue_ind_linear_low,
    input  logic [31:0]             issue_load_linear,
    input  logic [1:0]              issue_load_low,
    input  logic [31:0]             issue_mem_linear,
    input  logic                    pe,
    input  logic                    vm,
    input  logic                    seg_gp_fault,
    input  logic                    ss_flat32,
    input  logic [5:0]              seg_readable,     // per segment: a read needs no microcode check
    // Data unit: load writeback and operands
    input  logic [31:0]             forwarded_esp,
    input  logic [31:0]             mem_wdata,
    input  logic [31:0]             OPR_R,
    input  logic [31:0]             SIGMA,
    output logic                    direct_wb_retire,
    output logic                    fast_opr_commit,
    output logic [31:0]             fast_opr_data,
    output logic                    vipt_load_alu_dst_capture,
    output logic [31:0]             vipt_load_alu_dst_capture_data,
    output logic [2:0]              vipt_load_alu_dst_capture_dst,
    output logic [1:0]              vipt_load_alu_dst_capture_size,
    output logic [4:0]              vipt_load_wb_alu_op_r,
    output logic [31:0]             vipt_load_wb_data,
    output logic [7:0]              vipt_load_wb_dst_onehot_r,
    output logic [7:0]              vipt_load_wb_fwd_mask_r,   // forwarded GPR (not an M3 ALU load)
    output logic [2:0]              vipt_load_wb_dst_r,
    output logic                    vipt_load_wb_is_alu_r,
    output logic [1:0]              vipt_load_wb_size_r,
    output logic [31:0]             vipt_load_wb_target_r,
    output logic                    vipt_load_wb_valid_r,
    // D2 instruction and issue
    input  logic                    d2_vipt_ea_hazard,
    input  logic [31:0]             EIP,
    input  logic                    hardwired_off,
    input  dec_entry_t              i_bus,
    input  logic                    i_issue,
    input  logic                    single_step,
    output logic                    d2_plain_load_overlap_ready,
    output logic                    d2_vipt_candidate,
    output logic                    d2_vipt_load,
    output logic                    d2_vipt_pipe_ready,
    output logic                    d2_vipt_pop,
    output logic                    d2_vipt_ret,
    output logic                    d2_vipt_rmw,
    output logic                    d2_vipt_rmw_candidate,
    output logic                    vipt_issue_load,
    // Microsequencer and core control
    input  logic                    any_fault,
    input  logic                    gp_fault_trigger,
    input  dec_entry_t              i_ex,
    input  logic                    i_first,
    input  logic                    i_rni_delay,
    input  logic                    interrupt_entry,
    input  logic [1:0]              mem_eff_size,
    input  logic                    mem_is_io,
    input  logic                    mem_op_eligible,
    input  logic                    q_flush,
    input  logic                    stall,
    input  logic                    stall_invlpg,
    input  logic                    stall_wio,
    input  logic                    stall_x87_direct,
    input  logic                    uc_active,
    input  logic [11:0]             uc_addr,
    input  logic [11:0]             uc_addr_mem_r,
    input  logic [5:0]              uc_buscode,
    input  logic                    uc_busreq,
    input  logic                    uc_bus_or_dly,
    input  logic                    uc_data_busreq,
    input  logic                    uc_exec,
    input  logic                    uc_is_check_write,
    input  logic                    uc_is_mem_busop,
    input  logic                    uc_is_write,
    input  logic                    uc_p_pure_dly,
    input  logic                    x87_direct_mem_req,
    output logic                    pop_direct_r,
    output logic                    rd_fast_finish,
    output logic                    rd_fast_valid_r,
    output logic                    ret_redirect,
    output logic                    rmw_fallback_delay_r,
    output logic                    rmw_fast_active_r,
    output logic                    stall_fast_store,
    output logic                    stall_rmw_probe,
    output logic                    stall_ucrd,
    output logic                    vipt_load_ex_probed_r,
    output hardwired_load_token_t   vipt_load_ex_r,
    output logic                    vipt_load_exec_block,
    output hardwired_load_token_t   vipt_load_replay_r,
    output logic                    vipt_load_slow_busy,
    output logic                    vipt_load_slow_wait_r
);

wire        vipt_load_ex_hit;
reg        vipt_load_wb_ret_r;         // the retiring load is a RET
wire [31:0] d2_ret_esp;                // the RET's ESP after its pop (and imm16)
wire       d2_vipt_alu;                // ALU register,memory via registered VIPT data
wire       d2_vipt_older_store;         // Current EX uop must enter paging before a younger load
wire [2:0] d2_vipt_dst;
wire [7:0] d2_vipt_dst_onehot;
wire [1:0] d2_vipt_mem_size;
wire [1:0] d2_vipt_write_size;
hardwired_load_result_t d2_vipt_result_kind;
reg        vipt_load_slow_req_r;
// A direct-load miss whose line fill the L1 started from the probe itself
// waits for the missing dword there instead of entering paging.
reg        vipt_load_fill_wait_r;
wire       vipt_load_fill_done = vipt_load_fill_wait_r && dcache_vipt_fill_resp_valid;
assign     ret_redirect = vipt_load_wb_valid_r && vipt_load_wb_ret_r;
reg [31:0] vipt_load_wb_data_r;
reg        vipt_load_overlap_r;        // Plain-load successor owns EX while load completes
assign vipt_load_slow_busy = vipt_load_slow_req_r || vipt_load_slow_wait_r ||
                             vipt_load_fill_wait_r;
wire       vipt_load_busy = vipt_load_ex_r.valid || vipt_load_replay_r.valid ||
                            vipt_load_slow_busy || vipt_load_wb_valid_r;
wire       vipt_load_retire;

// Microcode reads on the probe/resolve pipeline: the set is preread in the RD
// cycle and translation and tag finalize the next (the L1 contract), so no
// live TLB lookup sits in the RD cycle. A miss goes to paging's registered
// path.
reg        ucrd_valid_r;               // probed last cycle; resolves now
// A bus read updates OPR_R no earlier than the routine's next bus or DLY
// uop, and the microcode relies on it: LD_DESCRIPTOR reads the second
// descriptor dword while the uops before its DLY still use the first. A hit
// resolving under any other uop holds its data and commits at that next bus
// or DLY uop (or at the end of the instruction); a bus uop that finds it held
// waits one cycle, as it would wait for the slow path's read.
reg        ucrd_held_r;
reg [31:0] ucrd_held_data_r;
// A first-uStep or x87 read stops every uop after a miss (its DLY may fold
// the RNI), as does a read whose DLY passed the resolve cycle. Otherwise a
// missed read in a routine is a bus read in flight: only bus and DLY uops,
// and the instruction's delay slot, wait for OPR_R.
reg        ucrd_strict_r;
// A microcode-read miss whose line fill the L1 started from the probe waits
// for the missing dword there instead of entering paging.
reg        ucrd_fill_wait_r;
wire       ucrd_fill_done = ucrd_fill_wait_r && dcache_vipt_fill_resp_valid;
wire       ucrd_miss_busy = ucrd_slow_req_r || ucrd_slow_wait_r || ucrd_fill_wait_r;
// In the resolve cycle a uop with side effects runs only when the read's
// translation was already known good in the RD cycle (the sidecar held its
// page), so no fault can follow; a bus operation always waits, keeping
// memory order behind a possible miss. After a miss every uop waits until
// OPR_R is written.
reg        ucrd_tlbok_r;               // the read's translation was known good
// A pure DLY may pass the resolve cycle only when the read cannot leave the
// fast path's translation (known good): after a TLB miss it waits like any
// uop, or a DLY that folds its instruction's RNI (MOV r,m) retires and the
// delay slot copies a stale OPR_R before the slow path returns or faults.
assign stall_ucrd = (ucrd_valid_r && (!ucrd_tlbok_r || (!uc_p_pure_dly && uc_busreq))) ||
                        (ucrd_held_r && uc_bus_or_dly && !uc_p_pure_dly) ||
                        (ucrd_miss_busy && (ucrd_strict_r || uc_bus_or_dly || i_rni_delay));
wire       ucrd_busy = ucrd_valid_r || ucrd_held_r || ucrd_miss_busy;
reg        rd_fast_probed_r;            // D2 preread was accepted
reg [31:0] rd_fast_linear_r;
reg [1:0]  rd_fast_size_r;
reg [1:0]  rd_fast_lane_r;
reg [1:0]  rmw_fast_size_r;
reg [1:0]  rmw_fast_lane_r;
// Direct store port into the L1, shared by WR_FAST and first-uStep stores
// translated in D2 (the sidecar TLB prereads the store's address at issue, so
// the WR cycle has a registered translation and needs no paging cycle).
reg        sidecar_valid_r;            // the sidecar holds a translation for
reg [19:0] sidecar_page_r;             //   this linear page
wire [3:0] st_be;
wire [31:0] st_wdata;
wire       rmw_store_valid = rmw_fast_active_r && i_rni_delay;
wire       rmw_store_accepted = rmw_store_valid && fast_store_accepted;
assign fast_store_valid = rmw_store_valid || st_take;
assign fast_store_be = st_take ? st_be : calc_be(rmw_fast_size_r, rmw_fast_lane_r);
assign fast_store_wdata = st_take ? st_wdata : (SIGMA << {rmw_fast_lane_r, 3'b000});
assign stall_fast_store = rmw_store_valid && !fast_store_accepted;
// A qualified RMW whose D2 preread was denied (for example by an older store
// still in L1 lookup in the same set) holds its overlay entry uStep and
// replays the preread from the registered linear address, instead of taking
// the full original-routine fallback. The select terms are registered.
assign stall_rmw_probe = rd_fast_valid_r && !rd_fast_probed_r && i_first;
assign direct_wb_retire = vipt_load_retire;

function automatic [31:0] format_hardwired_load(
    input [31:0] raw,
    input [1:0] lane,
    input [1:0] mem_size,
    input hardwired_load_result_t result_kind
);
    logic [31:0] shifted;
    begin
        shifted = raw >> {lane, 3'b000};
        case (mem_size)
            2'd0: begin
                case (result_kind)
                    LOAD_RESULT_SIGN_EXTEND:
                        format_hardwired_load = {{24{shifted[7]}}, shifted[7:0]};
                    default:
                        format_hardwired_load = {24'd0, shifted[7:0]};
                endcase
            end
            2'd1: begin
                case (result_kind)
                    LOAD_RESULT_SIGN_EXTEND:
                        format_hardwired_load = {{16{shifted[15]}}, shifted[15:0]};
                    default:
                        format_hardwired_load = {16'd0, shifted[15:0]};
                endcase
            end
            default: format_hardwired_load = shifted;
        endcase
    end
endfunction

assign vipt_load_wb_data = vipt_load_wb_data_r;

// An occupied EX stage may accept only another direct load. If the older load
// misses, the accepted younger token moves to the replay slot on this edge.
assign d2_vipt_pipe_ready = d2_vipt_candidate &&
                                dcache_vipt_probe_ready &&
                                vipt_load_ex_probed_r &&
                                !vipt_load_replay_r.valid &&
                                !vipt_load_slow_busy && !single_step;
// Once a plain load has finalized as a hit, its successor may enter EX on the
// same edge. Data operands consume the following WB value through forwarding;
// an EA dependency is held by d2_vipt_ea_hazard for exactly one hit cycle.
// Misses retain the precise slow-path interlock.
assign d2_plain_load_overlap_ready = vipt_load_ex_r.valid &&
                                vipt_load_ex_probed_r &&
                                vipt_load_ex_hit &&
                                !vipt_load_ex_r.is_alu && !vipt_load_ex_r.is_ret &&
                                !d2_vipt_candidate &&
                                !d2_vipt_ea_hazard &&
                                !vipt_load_replay_r.valid &&
                                !vipt_load_slow_busy && !single_step;

// synthesis translate_off
wire [2:0] vipt_load_wb_norm_dst = (vipt_load_wb_size_r == 2'd0)
                                  ? {1'b0, vipt_load_wb_dst_r[1:0]}
                                  : vipt_load_wb_dst_r;
// synthesis translate_on

wire       vipt_load_overlap_wb = vipt_load_overlap_r &&
                     vipt_load_wb_valid_r && !vipt_load_ex_r.valid &&
                     !vipt_load_replay_r.valid && !vipt_load_slow_busy;
assign vipt_load_exec_block = vipt_load_busy && !vipt_load_overlap_wb;

// The direct load pipeline admits a younger D2 probe while the older token
// resolves in EX. A miss transfers the older token to normal paging and saves
// the already-consumed younger token in one replay slot.
wire vipt_page_enabled = CR0[31];
wire vipt_user_ok = (pg_cpl != 2'd3) || vipt_tlb_user;
wire vipt_translation_ok = !vipt_page_enabled ||
                           (vipt_tlb_hit && vipt_user_ok);
wire [31:0] vipt_resolve_linear = rd_fast_valid_r
                                ? rd_fast_linear_r
                                : ucrd_valid_r
                                ? ucrd_linear_r
                                : vipt_load_ex_r.linear_addr;
wire [31:0] vipt_resolve_phys = vipt_page_enabled
                              ? vipt_tlb_phys_addr : vipt_resolve_linear;
wire vipt_load_ex_contained =
    (vipt_load_ex_r.mem_size == 2'd0) ||
    ((vipt_load_ex_r.mem_size == 2'd1) &&
     (vipt_load_ex_r.lane != 2'd3)) ||
    ((vipt_load_ex_r.mem_size == 2'd2) &&
     (vipt_load_ex_r.lane == 2'd0));
assign vipt_load_ex_hit = vipt_load_ex_r.valid && vipt_load_ex_probed_r &&
                          vipt_load_ex_contained &&
                          vipt_translation_ok &&
                          !vipt_tlb_is_vga_mem && !seg_gp_fault &&
                          dcache_vipt_resolve_hit;
// Capture the destination operand from every registered EX token,
// independently of translation, segmentation, and cache outcome. Plain loads
// use it for byte/word merge forwarding; M3 uses it as the private ALU
// destination. This speculative state has no architectural side effect;
// fault/miss handling gates the later valid/commit token.
wire vipt_load_alu_dst_capture_fast = vipt_load_ex_r.valid &&
                                      vipt_load_ex_probed_r;
wire vipt_load_alu_dst_capture_slow = vipt_load_slow_wait_r &&
                                      !mem_servicing;
assign vipt_load_alu_dst_capture = vipt_load_alu_dst_capture_fast ||
                                 vipt_load_alu_dst_capture_slow ||
                                 vipt_load_fill_done;
assign vipt_load_alu_dst_capture_dst =
    vipt_load_alu_dst_capture_fast ? vipt_load_ex_r.dst
                                   : vipt_load_slow_r.dst;
assign vipt_load_alu_dst_capture_size =
    vipt_load_alu_dst_capture_fast ? vipt_load_ex_r.write_size
                                   : vipt_load_slow_r.write_size;
assign vipt_load_alu_dst_capture_data =
    vipt_load_alu_dst_capture_fast
        ? format_hardwired_load(dcache_vipt_resolve_data,
                               vipt_load_ex_r.lane,
                               vipt_load_ex_r.mem_size,
                               vipt_load_ex_r.result_kind)
        : vipt_load_fill_done
        ? format_hardwired_load(dcache_vipt_fill_resp_data,
                               vipt_load_slow_r.lane,
                               vipt_load_slow_r.mem_size,
                               vipt_load_slow_r.result_kind)
        : format_hardwired_load(OPR_R, 2'd0,
                               vipt_load_slow_r.mem_size,
                               vipt_load_slow_r.result_kind);
// A token whose D2 probe was rejected (an older store owned the RAM read
// port that cycle) probes again from EX at once, instead of through the
// replay slot a cycle later. No younger load is in D2 issue beside an
// unprobed EX token, so it has the probe port.
wire ex_reprobe = vipt_load_ex_r.valid && !vipt_load_ex_probed_r &&
                  !vipt_load_replay_r.valid && !vipt_load_slow_busy &&
                  !mem_servicing && dcache_vipt_probe_ready;
wire vipt_replay_try = vipt_load_replay_r.valid &&
                       !vipt_load_ex_r.valid && !vipt_load_slow_busy &&
                       !mem_servicing &&
                       dcache_vipt_probe_ready;
assign vipt_issue_load = i_issue && d2_vipt_load;
always_ff @(posedge clk) begin
    if (!reset_n || q_flush || any_fault)
        pop_direct_r <= 1'b0;
    else if (i_issue)
        pop_direct_r <= d2_vipt_load && d2_vipt_pop;
end
wire rd_fast_issue = i_issue &&
                     (i_bus.ucode_action == RECIPE_ACTION_RMW_FAST);
wire vipt_issue_rmw = rd_fast_issue && d2_vipt_rmw;
wire vipt_issue_store_wait = vipt_issue_load && d2_vipt_older_store;
// The older load replay has priority; neither replay can coincide with a D2
// issue probe because the replaying instruction still owns EX.
wire rmw_replay_try = stall_rmw_probe && !vipt_replay_try &&
                      !vipt_load_ex_r.valid && !vipt_load_slow_busy &&
                      !mem_servicing && dcache_vipt_probe_ready;
assign vipt_probe_linear = ex_reprobe
                              ? vipt_load_ex_r.linear_addr
                              : vipt_replay_try
                              ? vipt_load_replay_r.linear_addr
                              : stall_rmw_probe
                              ? rd_fast_linear_r
                              : ucrd_route_pre
                              ? ind_linear
                              : issue_load_linear;

assign dcache_vipt_probe_valid = vipt_issue_load || vipt_issue_rmw || ex_reprobe ||
                                 vipt_replay_try || rmw_replay_try || ucrd_take;
assign dcache_vipt_probe_offset = vipt_probe_linear[11:0];
// The sidecar TLB translates the next access in advance, as the i486 TLB
// translates in the access's own clock: an EA instruction that does not
// probe at issue prereads its first address; any other free cycle prereads
// the page of the current ind_linear (not while RD_FAST still needs its own
// translation). A write whose page the sidecar holds can post in its own
// cycle.
assign st_tlb_pre = i_issue && i_bus.ind_is_ea && !d2_vipt_load && !vipt_issue_rmw &&
                    !ex_reprobe && !vipt_replay_try && !rmw_replay_try && !ucrd_take;
assign sidecar_bg_pre = !dcache_vipt_probe_valid && !st_tlb_pre && !rd_fast_valid_r;
always_ff @(posedge clk) begin
    if (!reset_n) begin
        sidecar_valid_r <= 1'b0;
        sidecar_page_r <= 20'd0;
    end else if (st_tlb_pre) begin
        sidecar_valid_r <= 1'b1;
        sidecar_page_r <= issue_mem_linear[31:12];
    end else if (dcache_vipt_probe_valid) begin
        // Every probe request prereads the sidecar TLB, accepted by the L1 or
        // not: a rejected probe's translation is still correct for its page,
        // and the L1's acceptance (an address compare) stays off this enable.
        sidecar_valid_r <= 1'b1;
        sidecar_page_r <= vipt_probe_linear[31:12];
    end else if (sidecar_bg_pre) begin
        sidecar_valid_r <= 1'b1;
        sidecar_page_r <= ind_linear[31:12];
    end
end
// The microcode read's privilege is the one registered with it.
wire ucrd_translation_ok = !vipt_page_enabled ||
                           (vipt_tlb_hit && ((ucrd_cpl_r != 2'd3) || vipt_tlb_user));
assign dcache_vipt_resolve_valid = (((vipt_load_ex_r.valid &&
                                      vipt_load_ex_probed_r) ||
                                     (rd_fast_valid_r &&
                                      rd_fast_probed_r)) &&
                                    vipt_translation_ok &&
                                    !vipt_tlb_is_vga_mem && !seg_gp_fault) ||
                                   (ucrd_valid_r && ucrd_translation_ok &&
                                    !vipt_tlb_is_vga_mem);
assign ucrd_hit = ucrd_valid_r && ucrd_translation_ok && !vipt_tlb_is_vga_mem &&
                dcache_vipt_resolve_hit;
assign dcache_vipt_resolve_phys_addr = vipt_resolve_phys;
// A plain or ALU direct load, or a microcode read, may fill straight from its
// probe on a miss; RET, RMW and x87 reads keep the paging path.
assign dcache_vipt_resolve_fill = !vipt_load_slow_busy &&
    ((vipt_load_ex_r.valid && vipt_load_ex_probed_r &&
      vipt_load_ex_contained && !vipt_load_ex_r.is_ret &&
      !rd_fast_valid_r && !ucrd_valid_r) ||
     (ucrd_valid_r && !ucrd_x87_r));

// A synthetic RNI is required only when the direct pipeline drains. Interior
// load boundaries are represented by their D2 issue and WB commit tokens.
assign vipt_load_retire = vipt_load_wb_valid_r && !vipt_load_overlap_r &&
                          !vipt_load_ex_r.valid &&
                          !vipt_load_replay_r.valid &&
                          !vipt_load_slow_busy;

always_ff @(posedge clk) begin
    if (!reset_n) begin
        vipt_load_ex_r <= '0;
        vipt_load_replay_r <= '0;
        vipt_load_slow_r <= '0;
        vipt_load_slow_req_r <= 1'b0;
        vipt_load_slow_wait_r <= 1'b0;
        vipt_load_fill_wait_r <= 1'b0;
        vipt_slow_phys_ok_r <= 1'b0;
        vipt_slow_phys_r <= 32'd0;
        vipt_load_wb_valid_r <= 1'b0;
        vipt_load_ex_probed_r <= 1'b0;
        vipt_load_wb_data_r <= 32'd0;
        vipt_load_wb_ret_r <= 1'b0;
        vipt_load_wb_target_r <= 32'd0;
        vipt_load_wb_dst_r <= 3'd0;
        vipt_load_wb_dst_onehot_r <= 8'd0;
        vipt_load_wb_fwd_mask_r <= 8'd0;
        vipt_load_wb_size_r <= 2'd2;
        vipt_load_wb_is_alu_r <= 1'b0;
        vipt_load_wb_alu_op_r <= 5'd0;
        vipt_load_overlap_r <= 1'b0;
    end else begin
        vipt_load_wb_valid_r <= 1'b0;
        vipt_load_wb_fwd_mask_r <= 8'd0;

        if (vipt_load_wb_valid_r)
            vipt_load_overlap_r <= 1'b0;
        if (i_issue && d2_plain_load_overlap_ready)
            vipt_load_overlap_r <= 1'b1;

        // The miss's payload and translation are consumed only with a slow
        // request; captured whenever a token resolves (no slow request is
        // pending then) so the fault cone does not reach their enables.
        if (vipt_load_ex_r.valid && !vipt_load_slow_busy) begin
            vipt_load_slow_r.linear_addr <= vipt_load_ex_r.linear_addr;
            vipt_load_slow_r.restart_eip <= vipt_load_ex_r.restart_eip;
            vipt_load_slow_r.dst <= vipt_load_ex_r.dst;
            vipt_load_slow_r.dst_onehot <= vipt_load_ex_r.dst_onehot;
            vipt_load_slow_r.mem_size <= vipt_load_ex_r.mem_size;
            vipt_load_slow_r.lane <= vipt_load_ex_r.lane;
            vipt_load_slow_r.write_size <= vipt_load_ex_r.write_size;
            vipt_load_slow_r.result_kind <= vipt_load_ex_r.result_kind;
            vipt_load_slow_r.is_alu <= vipt_load_ex_r.is_alu;
            vipt_load_slow_r.alu_op <= vipt_load_ex_r.alu_op;
            vipt_load_slow_r.restore_esp <= vipt_load_ex_r.restore_esp;
            vipt_load_slow_r.esp_restore <= vipt_load_ex_r.esp_restore;
            vipt_load_slow_r.is_ret <= vipt_load_ex_r.is_ret;
            vipt_load_slow_r.ret_esp <= vipt_load_ex_r.ret_esp;
        end
        if (vipt_load_ex_r.valid) begin
            vipt_slow_phys_ok_r <= vipt_load_ex_probed_r && vipt_load_ex_contained &&
                                   vipt_translation_ok && !vipt_tlb_is_vga_mem &&
                                   !seg_gp_fault;
            vipt_slow_phys_r <= vipt_resolve_phys;
        end

        // The EX slot normally advances or empties every cycle.
        vipt_load_ex_r.valid <= 1'b0;
        vipt_load_ex_probed_r <= 1'b0;

        if (vipt_replay_try && dcache_vipt_probe_direct_accepted) begin
            vipt_load_ex_r <= vipt_load_replay_r;
            vipt_load_ex_probed_r <= 1'b1;
            vipt_load_replay_r.valid <= 1'b0;
        end

        if (vipt_load_ex_r.valid) begin
            if (!vipt_load_ex_probed_r) begin
                if (ex_reprobe && dcache_vipt_probe_direct_accepted) begin
                    vipt_load_ex_r.valid <= 1'b1;
                    vipt_load_ex_probed_r <= 1'b1;
                end else begin
                    vipt_load_replay_r <= vipt_load_ex_r;
                end
            end else if (vipt_load_ex_hit && !any_fault) begin
                vipt_load_wb_valid_r <= 1'b1;
                // A RET writes its new ESP through the load port and keeps
                // the loaded target for the redirect.
                vipt_load_wb_data_r <= vipt_load_ex_r.is_ret ? vipt_load_ex_r.ret_esp
                                     : format_hardwired_load(
                    dcache_vipt_resolve_data, vipt_load_ex_r.lane,
                    vipt_load_ex_r.mem_size, vipt_load_ex_r.result_kind);
                vipt_load_wb_ret_r <= vipt_load_ex_r.is_ret;
                vipt_load_wb_target_r <= dcache_vipt_resolve_data;
                vipt_load_wb_dst_r <= vipt_load_ex_r.dst;
                vipt_load_wb_dst_onehot_r <= vipt_load_ex_r.dst_onehot;
                vipt_load_wb_fwd_mask_r <= vipt_load_ex_r.is_alu ? 8'd0 : vipt_load_ex_r.dst_onehot;
                vipt_load_wb_size_r <= vipt_load_ex_r.write_size;
                vipt_load_wb_is_alu_r <= vipt_load_ex_r.is_alu;
                vipt_load_wb_alu_op_r <= vipt_load_ex_r.alu_op;
            end else if (!any_fault) begin
                if (dcache_vipt_fill_start && !ucrd_valid_r)
                    vipt_load_fill_wait_r <= 1'b1;
                else
                    vipt_load_slow_req_r <= 1'b1;
            end
        end

        if (vipt_issue_load) begin
            if (vipt_load_ex_r.valid && !vipt_load_ex_hit) begin
                vipt_load_replay_r.valid <= 1'b1;
                vipt_load_replay_r.linear_addr <= issue_load_linear;
                vipt_load_replay_r.restart_eip <= EIP;
                vipt_load_replay_r.dst <= d2_vipt_dst;
                vipt_load_replay_r.dst_onehot <= d2_vipt_dst_onehot;
                vipt_load_replay_r.mem_size <= d2_vipt_mem_size;
                vipt_load_replay_r.lane <= issue_load_low;
                vipt_load_replay_r.write_size <= d2_vipt_write_size;
                vipt_load_replay_r.result_kind <= d2_vipt_result_kind;
                vipt_load_replay_r.is_alu <= d2_vipt_alu;
                vipt_load_replay_r.alu_op <= i_bus.decoded_alu_op;
                vipt_load_replay_r.restore_esp <= d2_vipt_pop;
                vipt_load_replay_r.esp_restore <= forwarded_esp;
                vipt_load_replay_r.is_ret <= d2_vipt_ret;
                vipt_load_replay_r.ret_esp <= d2_ret_esp;
            end else begin
                vipt_load_ex_r.valid <= 1'b1;
                vipt_load_ex_probed_r <= dcache_vipt_probe_accepted &&
                                          !vipt_issue_store_wait;
                vipt_load_ex_r.linear_addr <= issue_load_linear;
                vipt_load_ex_r.restart_eip <= EIP;
                vipt_load_ex_r.dst <= d2_vipt_dst;
                vipt_load_ex_r.dst_onehot <= d2_vipt_dst_onehot;
                vipt_load_ex_r.mem_size <= d2_vipt_mem_size;
                vipt_load_ex_r.lane <= issue_load_low;
                vipt_load_ex_r.write_size <= d2_vipt_write_size;
                vipt_load_ex_r.result_kind <= d2_vipt_result_kind;
                vipt_load_ex_r.is_alu <= d2_vipt_alu;
                vipt_load_ex_r.alu_op <= i_bus.decoded_alu_op;
                vipt_load_ex_r.restore_esp <= d2_vipt_pop;
                vipt_load_ex_r.esp_restore <= forwarded_esp;
                vipt_load_ex_r.is_ret <= d2_vipt_ret;
                vipt_load_ex_r.ret_esp <= d2_ret_esp;
            end
        end

        // Each requester owns only the acceptance of the request it submitted.
        if (vipt_slow_submit && mem_accepted) begin
            vipt_load_slow_req_r <= 1'b0;
            vipt_load_slow_wait_r <= 1'b1;
        end
        // A fill started from the probe returns its missing dword raw, as a
        // hit does.
        if (vipt_load_fill_done) begin
            vipt_load_fill_wait_r <= 1'b0;
            vipt_load_wb_valid_r <= 1'b1;
            vipt_load_wb_data_r <= format_hardwired_load(
                dcache_vipt_fill_resp_data, vipt_load_slow_r.lane,
                vipt_load_slow_r.mem_size, vipt_load_slow_r.result_kind);
            vipt_load_wb_ret_r <= 1'b0;
            vipt_load_wb_target_r <= dcache_vipt_fill_resp_data;
            vipt_load_wb_dst_r <= vipt_load_slow_r.dst;
            vipt_load_wb_dst_onehot_r <= vipt_load_slow_r.dst_onehot;
            vipt_load_wb_fwd_mask_r <= vipt_load_slow_r.is_alu ? 8'd0 : vipt_load_slow_r.dst_onehot;
            vipt_load_wb_size_r <= vipt_load_slow_r.write_size;
            vipt_load_wb_is_alu_r <= vipt_load_slow_r.is_alu;
            vipt_load_wb_alu_op_r <= vipt_load_slow_r.alu_op;
        end
        // Paging assembles crossing reads in OPR_R.  Wait until its ownership
        // drops rather than treating the first fragment as a completed load.
        if (vipt_load_slow_wait_r && !mem_servicing) begin
            vipt_load_slow_wait_r <= 1'b0;
            vipt_load_wb_valid_r <= 1'b1;
            vipt_load_wb_data_r <= vipt_load_slow_r.is_ret ? vipt_load_slow_r.ret_esp
                                 : format_hardwired_load(
                OPR_R, 2'd0, vipt_load_slow_r.mem_size,
                vipt_load_slow_r.result_kind);
            vipt_load_wb_ret_r <= vipt_load_slow_r.is_ret;
            vipt_load_wb_target_r <= OPR_R;
            vipt_load_wb_dst_r <= vipt_load_slow_r.dst;
            vipt_load_wb_dst_onehot_r <= vipt_load_slow_r.dst_onehot;
            vipt_load_wb_fwd_mask_r <= vipt_load_slow_r.is_alu ? 8'd0 : vipt_load_slow_r.dst_onehot;
            vipt_load_wb_size_r <= vipt_load_slow_r.write_size;
            vipt_load_wb_is_alu_r <= vipt_load_slow_r.is_alu;
            vipt_load_wb_alu_op_r <= vipt_load_slow_r.alu_op;
        end

        if (q_flush || any_fault || interrupt_entry) begin
            vipt_load_ex_r.valid <= 1'b0;
            vipt_load_replay_r.valid <= 1'b0;
            vipt_load_slow_req_r <= 1'b0;
            vipt_load_slow_wait_r <= 1'b0;
            vipt_load_fill_wait_r <= 1'b0;
            vipt_load_wb_valid_r <= 1'b0;
            vipt_load_wb_fwd_mask_r <= 8'd0;
            vipt_load_ex_probed_r <= 1'b0;
            vipt_load_overlap_r <= 1'b0;
        end
    end
end

// synthesis translate_off
always_ff @(posedge clk) begin
    if (reset_n && ex_reprobe && (vipt_issue_load || vipt_issue_rmw))
        $fatal(1, "D2 issue probe beside an EX re-probe");
    if (reset_n && i_issue && d2_vipt_load && !dcache_vipt_probe_ready)
        $fatal(1, "VIPT load issued without an accepted D2 preread");
    if (reset_n && vipt_issue_load && vipt_load_ex_r.valid &&
        !vipt_load_ex_hit && vipt_load_replay_r.valid)
        $fatal(1, "VIPT replay token overflow");
    if (reset_n && vipt_load_wb_valid_r &&
        (vipt_load_wb_dst_onehot_r !== (8'h01 << vipt_load_wb_norm_dst)))
        $fatal(1, "VIPT WB destination mask mismatch");
end
// synthesis translate_on

// RD_FAST finalizes the D2 preread in the overlay entry uStep. A hit commits
// the formatted operand to normal OPR_R and retains only the write-qualified
// physical identity. A reject has no architectural side effect and redirects
// to the untouched original routine.
wire rd_fast_contained =
    (rd_fast_size_r == 2'd0) ||
    ((rd_fast_size_r == 2'd1) && (rd_fast_lane_r != 2'd3)) ||
    ((rd_fast_size_r == 2'd2) && (rd_fast_lane_r == 2'd0));
wire rd_fast_page_write_ok = !vipt_page_enabled ||
    (vipt_tlb_hit && vipt_user_ok && vipt_tlb_dirty &&
     (vipt_tlb_writable || ((pg_cpl != 2'd3) && !CR0[16])));
wire rd_fast_hit = rd_fast_valid_r && rd_fast_probed_r &&
                   rd_fast_contained && rd_fast_page_write_ok &&
                   !vipt_tlb_is_vga_mem && !seg_gp_fault &&
                   dcache_vipt_resolve_hit;
assign rd_fast_finish = rd_fast_valid_r && i_first && uc_exec;
// A microcode read hit writes OPR_R as paging does for a single access: the
// operand shifted down from its byte lane and zero-extended from its size.
wire [31:0] ucrd_hit_data = format_hardwired_load(
    ucrd_fill_wait_r ? dcache_vipt_fill_resp_data : dcache_vipt_resolve_data,
    ucrd_linear_r[1:0], ucrd_size_r, LOAD_RESULT_COPY);
// The missing dword of a fill arrives like a hit, unless every uop waits.
wire        ucrd_data_now = ucrd_hit || ucrd_fill_done;
wire        ucrd_hit_hold = ucrd_data_now && !ucrd_x87_r && !uc_bus_or_dly && !i_rni_delay &&
                            !(ucrd_fill_wait_r && ucrd_strict_r);
wire        ucrd_held_commit = ucrd_held_r &&
    (uc_bus_or_dly || i_rni_delay || !uc_active || q_flush || any_fault || interrupt_entry);
assign fast_opr_commit = (rd_fast_finish && rd_fast_hit) || (ucrd_data_now && !ucrd_hit_hold) ||
                         ucrd_held_commit;
assign fast_opr_data = ucrd_held_r ? ucrd_held_data_r
                     : (ucrd_valid_r || ucrd_fill_wait_r) ? ucrd_hit_data
                     : format_hardwired_load(dcache_vipt_resolve_data, rd_fast_lane_r,
                                             rd_fast_size_r, LOAD_RESULT_COPY);
always_ff @(posedge clk) begin
    if (!reset_n) begin
        rd_fast_valid_r <= 1'b0;
        rd_fast_probed_r <= 1'b0;
        rd_fast_linear_r <= 32'd0;
        rd_fast_size_r <= 2'd0;
        rd_fast_lane_r <= 2'd0;
        rmw_fast_active_r <= 1'b0;
        rmw_fast_phys_r <= 32'd0;
        rmw_fast_size_r <= 2'd0;
        rmw_fast_lane_r <= 2'd0;
        rmw_fallback_delay_r <= 1'b0;
    end else begin
        if (rd_fast_issue) begin
            rd_fast_valid_r <= 1'b1;
            rd_fast_probed_r <= d2_vipt_rmw &&
                                dcache_vipt_probe_accepted;
            rd_fast_linear_r <= issue_ind_linear;
            rd_fast_size_r <= i_bus.operand_size;
            rd_fast_lane_r <= issue_ind_linear_low;
        end
        if (rmw_replay_try && dcache_vipt_probe_direct_accepted)
            rd_fast_probed_r <= 1'b1;

        if (rd_fast_finish) begin
            rd_fast_valid_r <= 1'b0;
            rd_fast_probed_r <= 1'b0;
            if (rd_fast_hit) begin
                rmw_fast_active_r <= 1'b1;
                rmw_fast_phys_r <= vipt_resolve_phys;
                rmw_fast_size_r <= rd_fast_size_r;
                rmw_fast_lane_r <= rd_fast_lane_r;
            end else begin
                rmw_fallback_delay_r <= 1'b1;
            end
        end

        // Keep the overlay inert while its registered rejection redirects the
        // two-stage ROM pipeline. The target word is held for one cycle and
        // executes normally after this token is cleared.
        if (rmw_fallback_delay_r &&
            (uc_addr == recipe_fallback_entry(i_ex.entry_point)) &&
            (uc_addr_mem_r == recipe_fallback_entry(i_ex.entry_point)))
            rmw_fallback_delay_r <= 1'b0;

        if (rmw_store_accepted)
            rmw_fast_active_r <= 1'b0;

        if (q_flush || any_fault || interrupt_entry) begin
            rd_fast_valid_r <= 1'b0;
            rd_fast_probed_r <= 1'b0;
            rmw_fast_active_r <= 1'b0;
            rmw_fallback_delay_r <= 1'b0;
        end
    end
end

// A pending microcode-read miss is always older than a pending direct-load
// miss (a microcode read does not start behind one), so it enters paging
// first: their OPR_R results must arrive in program order.
assign      vipt_slow_submit = vipt_load_slow_req_r && !mem_servicing && !ucrd_slow_req_r;
// A fallback token owns stable registered address metadata as soon as it is
// pending.  Present that address to the live TLB while an older request drains;
// submission remains idle-gated above.  This keeps mem_servicing out of the
// live-TLB/cache-address cone without changing request ordering.
assign vipt_slow_addr_owned = vipt_load_slow_req_r;
// A cacheable, non-crossing microcode read (or x87 direct operand read) with
// paging idle probes the cache instead of entering paging; a missed one enters
// paging from its token. Paging would read the same registered ind_linear.
wire        ucrd_uc_read = uc_data_busreq && uc_is_mem_busop &&
    !uc_is_write && !uc_is_check_write && !mem_is_io &&
    (uc_buscode != BUSOP_RD_IND) && !x87_direct_mem_req;
wire [1:0]  ucrd_size_now = x87_direct_mem_req ? 2'd2 : mem_eff_size;
assign ucrd_route_pre = !hardwired_off && mem_op_eligible && (ucrd_uc_read || x87_direct_mem_req) &&
    ind_linear_valid && !vipt_load_slow_req_r && !ucrd_valid_r && !ucrd_held_r &&
    !ucrd_miss_busy && paging_demand_idle &&
    !access_crosses_dword(ucrd_size_now, ind_linear[1:0]) &&
    dcache_vipt_probe_ready && !ex_reprobe;
// A held microcode uop must not probe twice; an x87 direct read is issued
// while x87 holds the pipeline and marks itself issued on acceptance.
assign ucrd_take = ucrd_route_pre && !gp_fault_trigger && (x87_direct_mem_req || !stall);
// A write whose page the sidecar holds posts through the direct store port
// when that translation permits it (present, writable at this privilege,
// dirty, not the VGA aperture) and the L1 takes a store now; otherwise it
// enters paging.
wire        st_route_pre = !hardwired_off && mem_op_eligible && uc_data_busreq && uc_is_mem_busop &&
    uc_is_write && !uc_is_check_write && !mem_is_io &&
    sidecar_valid_r && (sidecar_page_r == ind_linear[31:12]) && ind_linear_valid &&
    !access_crosses_dword(mem_eff_size, ind_linear[1:0]) &&
    paging_demand_idle && !ucrd_busy && !vipt_load_slow_req_r &&
    !rmw_store_valid && dcache_wr_ready;
wire        st_user = (pg_cpl == 2'd3);
wire        st_postable = vipt_page_enabled
    ? (vipt_tlb_hit && !vipt_tlb_is_vga_mem && (!st_user || vipt_tlb_user) &&
       vipt_tlb_dirty && (vipt_tlb_writable || (!st_user && !CR0[16])))
    : !is_device_page(ind_linear[31:12]);
assign st_route = st_route_pre && st_postable;
assign st_take = st_route && !gp_fault_trigger &&
                 !(stall_wio || stall_x87_direct || stall_invlpg || stall_rmw_probe || stall_ucrd);
assign st_phys = vipt_page_enabled ? {vipt_tlb_phys_addr[31:12], ind_linear[11:0]} : ind_linear;
assign st_be = calc_be(mem_eff_size, ind_linear[1:0]);
assign st_wdata = shift_write_data(mem_wdata, mem_eff_size, ind_linear[1:0]);
assign ucrd_slow_submit = ucrd_slow_req_r && !mem_servicing;


// Microcode read token: probe in the RD cycle, resolve the next. A miss
// (TLB, permission, VGA aperture, cache) enters paging from the token, which
// writes OPR_R; every uop waits until it does.
always_ff @(posedge clk) begin
    if (!reset_n) begin
        ucrd_valid_r <= 1'b0;
        ucrd_tlbok_r <= 1'b0;
        ucrd_linear_r <= 32'd0;
        ucrd_size_r <= 2'd0;
        ucrd_cpl_r <= 2'd0;
        ucrd_x87_r <= 1'b0;
        ucrd_phys_ok_r <= 1'b0;
        ucrd_phys_r <= 32'd0;
        ucrd_slow_req_r <= 1'b0;
        ucrd_slow_wait_r <= 1'b0;
        ucrd_held_r <= 1'b0;
        ucrd_held_data_r <= 32'd0;
        ucrd_strict_r <= 1'b0;
        ucrd_fill_wait_r <= 1'b0;
    end else begin
        ucrd_valid_r <= ucrd_take;
        if (ucrd_held_commit)
            ucrd_held_r <= 1'b0;
        if (ucrd_hit_hold) begin
            ucrd_held_r <= 1'b1;
            ucrd_held_data_r <= ucrd_hit_data;
        end
        ucrd_tlbok_r <= ucrd_take && sidecar_valid_r &&
                        (sidecar_page_r == ind_linear[31:12]) && !vipt_tlb_is_vga_mem &&
                        (!vipt_page_enabled ||
                         (vipt_tlb_hit && ((pg_cpl != 2'd3) || vipt_tlb_user)));
        if (ucrd_take) begin
            ucrd_linear_r <= ind_linear;
            ucrd_size_r <= ucrd_size_now;
            ucrd_cpl_r <= pg_cpl;
            ucrd_x87_r <= x87_direct_mem_req;
            ucrd_strict_r <= i_first || x87_direct_mem_req;
        end
        if (ucrd_valid_r && !ucrd_hit) begin
            if (dcache_vipt_fill_start)
                ucrd_fill_wait_r <= 1'b1;
            else
                ucrd_slow_req_r <= 1'b1;
            // A DLY that passed the resolve cycle leaves its wait to the
            // uops after it.
            if (uc_bus_or_dly)
                ucrd_strict_r <= 1'b1;
            ucrd_phys_ok_r <= ucrd_translation_ok && !vipt_tlb_is_vga_mem;
            ucrd_phys_r <= vipt_resolve_phys;
        end
        if (ucrd_slow_submit && mem_accepted) begin
            ucrd_slow_req_r <= 1'b0;
            ucrd_slow_wait_r <= 1'b1;
        end
        if (ucrd_slow_wait_r && !mem_servicing)
            ucrd_slow_wait_r <= 1'b0;
        if (ucrd_fill_done)
            ucrd_fill_wait_r <= 1'b0;
        if (q_flush || any_fault || interrupt_entry) begin
            ucrd_valid_r <= 1'b0;
            ucrd_slow_req_r <= 1'b0;
            ucrd_slow_wait_r <= 1'b0;
            ucrd_fill_wait_r <= 1'b0;
        end
    end
end

// synthesis translate_off
always @(posedge clk)
    if (reset_n && ucrd_take && !dcache_vipt_probe_accepted)
        $fatal(1, "microcode read probe not accepted");
// synthesis translate_on

// Direct register loads share one token after D2.  MOVZX/MOVSX use SRCREG as
// their architectural destination in the original microcode; plain MOV uses
// DSTREG and may name AH/CH/DH/BH.  Crossing operands are admitted here so EX
// can transfer their registered address and metadata to normal paging.
// Both address sizes are eligible: address_unit has already masked a16 offsets
// and added the selected segment base before issue. Complex base+index+disp
// forms retain their separate D2 partial-sum cycle.
// A write uop and its D2 successor can overlap on the edge where paging first
// captures the write.  The store may still need a dirty-bit page walk, so it
// is not yet visible to the cache's store queue.  Accept the younger load token
// but discard that edge's speculative preread, then replay it only after paging
// releases the older request.  The load retains VIPT while cache/store-queue
// ordering sees the accepted store before the replayed lookup.
assign d2_vipt_older_store = uc_active && uc_is_write;
wire d2_vipt_plain_mov = !i_bus.has_0f &&
                         ((i_bus.opcode == 8'h8A) ||
                          (i_bus.opcode == 8'h8B));
wire d2_vipt_movx = i_bus.has_0f && i_bus.data32 &&
                    ((i_bus.opcode == 8'hB6) ||
                     (i_bus.opcode == 8'hB7) ||
                     (i_bus.opcode == 8'hBE) ||
                     (i_bus.opcode == 8'hBF));
assign d2_vipt_alu = i_bus.vipt_alu;
assign d2_vipt_dst = d2_vipt_ret ? 3'd4 :
                     d2_vipt_movx ? i_bus.src_reg_sel : i_bus.dst_reg_sel;
assign d2_vipt_mem_size = d2_vipt_ret ? 2'd2 :
                          d2_vipt_movx ? i_bus.source_size : i_bus.operand_size;
assign d2_vipt_write_size = (d2_vipt_movx || d2_vipt_ret) ? 2'd2
                                                          : i_bus.operand_size;
wire [2:0] d2_vipt_dst_wide = (d2_vipt_write_size == 2'd0)
                            ? {1'b0, d2_vipt_dst[1:0]} : d2_vipt_dst;
assign d2_vipt_dst_onehot = 8'h01 << d2_vipt_dst_wide;
assign d2_vipt_result_kind = !d2_vipt_movx ? LOAD_RESULT_COPY :
    (i_bus.opcode[3] ? LOAD_RESULT_SIGN_EXTEND : LOAD_RESULT_ZERO_EXTEND);
// MOV AL/eAX,moffs shares MOV r,m's recipe; its address is the immediate.
// A ModR/M load has an earlier AGU cycle in which IND takes the ordinary
// segment-limit check; a moffs literal reaches IND on the edge that launches
// the direct token. So only a flat DS (protected mode, page-granular,
// 4 GB limit, not expand-down), where no contained access can violate the
// limit, takes the direct path; others keep the ROM path's precise check.
wire d2_vipt_moffs = !i_bus.has_0f && i_bus.has_moffs &&
                     ((i_bus.opcode == 8'hA0) || (i_bus.opcode == 8'hA1)) &&
                     pe && (i_bus.mem_seg == SEG_DS) && ds_flat;
// POP r is a direct load at the stack address. Its first ROM word still
// executes for its ESP write (its read is suppressed); like moffs, the
// address reaches IND on the token's launch edge, so only a flat 32-bit SS
// takes this path. POP ESP keeps the ROM path.
assign d2_vipt_pop = !i_bus.has_0f && (i_bus.opcode[7:3] == 5'b01011) &&
                   (i_bus.opcode[2:0] != 3'd4) && i_bus.stack_op &&
                   pe && ss_flat32;
// A 32-bit near RET is a direct load of its target: on writeback the load
// port writes ESP (ESP + 4 + imm16) and the target redirects the front end,
// as the hardware branch uStep does; its ROM words do not execute.
wire d2_flat_ss32 = pe && ss_flat32;
assign d2_vipt_ret = !i_bus.has_0f && ((i_bus.opcode == 8'hC3) || (i_bus.opcode == 8'hC2)) &&
                     i_bus.data32 && i_bus.stack_op && d2_flat_ss32;
assign d2_ret_esp = forwarded_esp + 32'd4 +
                    ((i_bus.opcode == 8'hC2) ? {16'd0, i_bus.immediate[15:0]} : 32'd0);
// A direct access skips the microcode's segment check; one whose segment is
// not readable (a null selector) takes the ROM path, which faults.
wire d2_seg_readable = !pe || vm || (i_bus.mem_seg < 4'd6 && seg_readable[i_bus.mem_seg[2:0]]);
assign d2_vipt_candidate = d2_seg_readable && !hardwired_off &&
                           (i_bus.rep_lock == PREFIX_NOREPLOCK) &&
                           (((d2_vipt_plain_mov || d2_vipt_movx || d2_vipt_alu) &&
                             i_bus.has_modrm && (i_bus.modrm[7:6] != 2'b11) &&
                             !i_bus.has_moffs && !i_bus.stack_op) ||
                            d2_vipt_moffs || d2_vipt_pop || d2_vipt_ret) &&
                           !single_step;
// A direct load or RMW waits while an older microcode read resolves or misses:
// its own miss must not reach paging (and OPR_R) ahead of the older read.
assign d2_vipt_load = d2_vipt_candidate && dcache_vipt_probe_ready && !ucrd_route_pre && !ucrd_busy &&
                      !vipt_load_replay_r.valid && !vipt_load_slow_busy &&
                      !rmw_fast_active_r;
wire d2_vipt_rmw_opcode = ((i_bus.opcode == 8'hF6) ||
                            (i_bus.opcode == 8'hF7))
                         ? ((i_bus.modrm[5:3] == 3'b010) ||
                            (i_bus.modrm[5:3] == 3'b011))
                         : (((i_bus.opcode == 8'hFE) ||
                             (i_bus.opcode == 8'hFF)) &&
                            ((i_bus.modrm[5:3] == 3'b000) ||
                             (i_bus.modrm[5:3] == 3'b001)));
assign d2_vipt_rmw_candidate = d2_seg_readable && !hardwired_off &&
                           (i_bus.ucode_action == RECIPE_ACTION_RMW_FAST) &&
                           (((i_bus.opcode == 8'hF6) ||
                             (i_bus.opcode == 8'hF7) ||
                             (i_bus.opcode == 8'hFE) ||
                             (i_bus.opcode == 8'hFF))
                                ? d2_vipt_rmw_opcode : 1'b1) &&
                           (i_bus.rep_lock == PREFIX_NOREPLOCK) &&
                           i_bus.has_modrm &&
                           (i_bus.modrm[7:6] != 2'b11) &&
                           !i_bus.has_moffs && !i_bus.stack_op &&
                           !single_step;
assign d2_vipt_rmw = d2_vipt_rmw_candidate &&
                     dcache_vipt_probe_ready && !ucrd_route_pre && !ucrd_busy &&
                     !vipt_load_replay_r.valid && !vipt_load_slow_busy &&
                     // The retiring WR_FAST owns the demand preread, but a
                     // same-word successor can share it.  Admit the successor
                     // on the acceptance edge so N/N+1/N+2 is truly a
                     // three-cycle first-to-first sequence; backpressure still
                     // keeps the younger instruction out of EX.
                     (!rmw_fast_active_r || rmw_store_accepted) &&
                     !mem_servicing;

endmodule
`default_nettype wire
