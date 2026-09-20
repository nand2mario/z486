`timescale 1ns/1ns

// Testbench for z486 - Protected Mode Test Runner Generic testbench for protected mode tests with configurable segment descriptors and...
// Details: doc/z486/implementation_notes.md#src-24-z486-tests-tb-protected-mode-sv-3

/* verilator lint_off SYNCASYNCNET */

module tb_protected_mode #(
    parameter ENABLE_X87 = 0,
    parameter MEM_SIZE = 1 << 19,
    parameter DCACHE_SET_BITS = 7
);
    // Segment cache array indices (from z486_pkg)
    localparam SEG_ES = 0, SEG_CS = 1, SEG_SS = 2, SEG_DS = 3;
    localparam SEG_FS = 4, SEG_GS = 5, SEG_IDT = 6, SEG_TR = 8, SEG_GDT = 10;
    // Clock and reset
    reg clk = 0;
    always #5 clk <= ~clk;  // 100 MHz clock

    reg reset_n = 0;

    // Test control
    int max_cycles = 10_000_000;
    int cycle = 0;
    bit stop_on_hlt = 1'b1;

    // CPU bus interface (32-bit, ready/valid)
    wire [31:2] addr;       // 4-byte aligned address
    wire [3:0]  be;         // Byte enables
    wire [7:0]  burstcount;
    wire [31:0] dout;       // Data output from CPU
    wire        valid, write, io;
    reg  [31:0] din;        // Data input to CPU
    reg         ready;
    reg         resp_valid;
    reg         intr = 0;
    reg         nmi = 0;
    wire        inta;
    wire        triple_fault_reset;

    // Instantiate the z486 CPU
    z486 #(
        .ENABLE_X87(ENABLE_X87),
        .DCACHE_SET_BITS(DCACHE_SET_BITS)
    ) dut (
        .clk(clk),
        .reset_n(reset_n),
        .addr(addr),
        .be(be),
        .burstcount(burstcount),
        .line_read(),
        .din(din),
        .line_din(128'd0),
        .dout(dout),
        .valid(valid),
        .write(write),
        .io(io),
        .ready(ready),
        .resp_valid(resp_valid),
        .line_resp_valid(1'b0),
        .intr(intr),
        .nmi(nmi),
        .inta(inta),
        .snoop_addr(32'h0),
        .snoop_valid(1'b0),
        .a20_enable(1'b1),
        .cpu_speed_sel(2'd0),
        .single_step(1'b0), // Continuous execution
        .dbg_CS(),
        .dbg_EIP(),
        .dbg_pe(),
        .dbg_vm(),
        .dbg_x87_state(),
        .triple_fault_reset(triple_fault_reset)
    );

`include "m0_profile.svh"

    // The regular tests use 512KB. Snapshot replay overrides this parameter
    // with the captured physical-memory size.
    reg [7:0] mem [0:MEM_SIZE-1];

    // Instruction counting
    wire instruction_boundary = dut.uc_is_rni && dut.uc_active;
    reg prev_instruction_boundary = 0;
    longint instruction_count = 0;
    longint x87_command_count = 0;
    longint x87_direct_load_count = 0;
    longint x87_control_busy_cycles = 0;
    longint x87_executor_busy_cycles = 0;
    longint x87_wait_stall_cycles = 0;
    longint x87_fop_count [0:2047];
    longint x87_direct_load_fop_count [0:2047];
    longint x87_exec_start_count [0:15];
    longint x87_exec_busy_count [0:15];
    longint cpu_entry_count [0:4095];
    longint cpu_entry_cycles [0:4095];
    longint cpu_entry_mem_stall [0:4095];
    longint cpu_entry_x87_stall [0:4095];
    longint cpu_entry_frontend_wait [0:4095];
    longint cpu_opcode_count [0:255];
    longint cpu_opcode_cycles [0:255];
    longint cpu_hardwired_count [0:1];
    longint cpu_hardwired_cycles [0:1];
    longint cpu_commit_count [0:7];
    longint cpu_commit_cycles [0:7];
    longint cpu_stall_cycles [0:9];
    longint cpu_load_chain [0:11];
    longint cpu_load_intervals [0:16];
    longint cpu_load_issue [0:7];
    longint cpu_load_result [0:7];
    longint cpu_load_successor [0:1][0:3];
    longint cpu_load_path_interval [0:2][0:16];
    longint cpu_load_path_successor_count [0:2][0:3];
    longint cpu_load_path_successor_cycles [0:2][0:3];
    longint cpu_load_form_count [0:2][0:1][0:2][0:2][0:3];
    longint cpu_load_form_cycles [0:2][0:1][0:2][0:2][0:3];
    longint cpu_load_next_entry_count [0:2][0:4095];
    longint cpu_load_next_entry_cycles [0:2][0:4095];
    longint cpu_branch_result [0:7];
    longint cpu_rep_elements [0:4];
    longint cpu_x87_wait_reason [0:7];
    longint cpu_store_intervals [0:16];
    longint cpu_store_states [0:9];
    longint cpu_store_next_entry [0:4095];
    longint cpu_store_next_opcode [0:255];
    longint cpu_store_next_entry_gap [0:4095][0:3];
    longint cpu_store_next_opcode_gap [0:255][0:3];
    longint cpu_current_start;
    logic [11:0] cpu_current_entry;
    logic [7:0] cpu_current_opcode;
    logic cpu_current_hardwired;
    logic [2:0] cpu_current_commit;
    logic [7:0] cpu_current_load_dst_mask;
    logic cpu_current_load_direct;
    logic [1:0] cpu_current_load_path;
    logic cpu_current_load_addr32;
    logic [1:0] cpu_current_load_size;
    logic [1:0] cpu_current_load_ea_class;
    logic [1:0] cpu_current_load_alignment;
    logic cpu_profile_active;
    event load_snapshot_x87_state;

    // Snapshot-only CPU profiler. Attribute issue-to-issue time to the older
    // instruction and separately count mutually exclusive pipeline states.
    always @(posedge clk) begin
        if (reset_n && $test$plusargs("profile_cpu")) begin
            if (cpu_profile_active && (cpu_current_entry == 12'h013)) begin
                if (dut.i_issue)
                    cpu_store_states[0] += 1;
                else if (dut.stall_mem)
                    cpu_store_states[1] += 1;
                else if (dut.stall_wio || dut.stall_x87_direct)
                    cpu_store_states[2] += 1;
                else if (dut.stall_d2)
                    cpu_store_states[3] += 1;
                else if (dut.throttle_parked_r)
                    cpu_store_states[4] += 1;
                else if (!dut.d2_valid)
                    cpu_store_states[5] += 1;
                else if (!dut.d2_ready)
                    cpu_store_states[6] += 1;
                else if (dut.hardwired_control_inst.recipe_state.slot_has_work)
                    cpu_store_states[7] += 1;
                else if (!dut.uc_active)
                    cpu_store_states[8] += 1;
                else
                    cpu_store_states[9] += 1;
            end

            if (dut.i_issue) begin
                if (cpu_profile_active) begin
                    cpu_entry_cycles[cpu_current_entry] += cycle - cpu_current_start;
                    cpu_opcode_cycles[cpu_current_opcode] += cycle - cpu_current_start;
                    cpu_hardwired_cycles[cpu_current_hardwired] += cycle - cpu_current_start;
                    cpu_commit_cycles[cpu_current_commit] += cycle - cpu_current_start;
                    if (cpu_current_entry == 12'h019) begin
                        automatic logic load_data_dependency;
                        automatic logic load_ea_dependency;
                        automatic int load_dependency;
                        automatic int load_interval;
                        if ((cycle - cpu_current_start) >= 16)
                            cpu_load_intervals[16] += 1;
                        else
                            cpu_load_intervals[cycle - cpu_current_start] += 1;
                        load_interval = (cycle - cpu_current_start) >= 16
                                      ? 16 : cycle - cpu_current_start;
                        load_data_dependency =
                            |(cpu_current_load_dst_mask &
                              dut.i_bus.recipe_gpr_read_mask);
                        load_ea_dependency =
                            |(cpu_current_load_dst_mask &
                              (dut.i_bus.ea_base_onehot |
                               dut.i_bus.ea_index_onehot));
                        cpu_load_successor[cpu_current_load_direct]
                            [{load_ea_dependency, load_data_dependency}] += 1;
                        load_dependency = {load_ea_dependency,
                                           load_data_dependency};
                        cpu_load_path_interval[cpu_current_load_path]
                                              [load_interval] += 1;
                        cpu_load_path_successor_count[cpu_current_load_path]
                                                     [load_dependency] += 1;
                        cpu_load_path_successor_cycles[cpu_current_load_path]
                                                      [load_dependency] +=
                            cycle - cpu_current_start;
                        cpu_load_form_count[cpu_current_load_path]
                            [cpu_current_load_addr32][cpu_current_load_size]
                            [cpu_current_load_ea_class]
                            [cpu_current_load_alignment] += 1;
                        cpu_load_form_cycles[cpu_current_load_path]
                            [cpu_current_load_addr32][cpu_current_load_size]
                            [cpu_current_load_ea_class]
                            [cpu_current_load_alignment] +=
                                cycle - cpu_current_start;
                        cpu_load_next_entry_count[cpu_current_load_path]
                                                 [dut.i_bus.entry_point] += 1;
                        cpu_load_next_entry_cycles[cpu_current_load_path]
                                                  [dut.i_bus.entry_point] +=
                            cycle - cpu_current_start;
                    end
                    if (cpu_current_entry == 12'h013) begin
                        automatic int store_gap;
                        if ((cycle - cpu_current_start) >= 16)
                            cpu_store_intervals[16] += 1;
                        else
                            cpu_store_intervals[cycle - cpu_current_start] += 1;
                        store_gap = (cycle - cpu_current_start) >= 3
                                  ? 3 : cycle - cpu_current_start;
                        cpu_store_next_entry[dut.i_bus.entry_point] += 1;
                        cpu_store_next_opcode[dut.i_bus.opcode] += 1;
                        cpu_store_next_entry_gap[dut.i_bus.entry_point][store_gap] += 1;
                        cpu_store_next_opcode_gap[dut.i_bus.opcode][store_gap] += 1;
                    end
                end
                cpu_current_start = cycle;
                cpu_current_entry = dut.i_bus.entry_point;
                cpu_current_opcode = dut.i_bus.opcode;
                cpu_current_hardwired = dut.d2_recipe.hardwired;
                cpu_current_commit = dut.d2_recipe.commit_sel;
                cpu_current_load_dst_mask =
                    8'h01 << ((dut.i_bus.operand_size == 2'd0)
                            ? {1'b0, dut.i_bus.dst_reg_sel[1:0]}
                            : dut.i_bus.dst_reg_sel);
                cpu_current_load_direct = dut.d2_vipt_load;
                cpu_entry_count[dut.i_bus.entry_point] += 1;
                cpu_opcode_count[dut.i_bus.opcode] += 1;
                cpu_hardwired_count[dut.d2_recipe.hardwired] += 1;
                cpu_commit_count[dut.d2_recipe.commit_sel] += 1;
                cpu_profile_active = 1'b1;

                if (dut.i_bus.entry_point == 12'h019) begin
                    automatic int load_bytes;
                    automatic int load_terms;
                    cpu_load_issue[0] += 1;
                    if (dut.d2_vipt_candidate)
                        cpu_load_issue[1] += 1;
                    if (dut.d2_vipt_load)
                        cpu_load_issue[2] += 1;
                    if (!dut.i_bus.addr32)
                        cpu_load_issue[3] += 1;
                    if (dut.i_bus.ea_complex)
                        cpu_load_issue[4] += 1;
                    if (dut.i_bus.rep_lock != z486_pkg::PREFIX_NOREPLOCK)
                        cpu_load_issue[5] += 1;
                    if (dut.d2_vipt_candidate && !dut.dcache_vipt_probe_ready)
                        cpu_load_issue[6] += 1;
                    if (dut.d2_vipt_candidate && dut.d2_vipt_older_store)
                        cpu_load_issue[7] += 1;
                    cpu_current_load_path = dut.d2_vipt_load ? 2'd0 :
                        dut.d2_recipe.hardwired ? 2'd1 : 2'd2;
                    cpu_current_load_addr32 = dut.i_bus.addr32;
                    cpu_current_load_size = dut.d2_vipt_mem_size;
                    load_terms = (|dut.i_bus.ea_base_onehot) +
                                 (|dut.i_bus.ea_index_onehot) +
                                 (dut.i_bus.displacement != 32'd0);
                    cpu_current_load_ea_class = dut.i_bus.ea_complex ? 2'd2 :
                                                (load_terms >= 2) ? 2'd1 : 2'd0;
                    load_bytes = 1 << dut.d2_vipt_mem_size;
                    if (({1'b0, dut.issue_ind_linear[11:0]} + load_bytes) > 4096)
                        cpu_current_load_alignment = 2'd3;
                    else if (({1'b0, dut.issue_ind_linear[3:0]} + load_bytes) > 16)
                        cpu_current_load_alignment = 2'd2;
                    else if ((dut.issue_ind_linear & (load_bytes - 1)) != 0)
                        cpu_current_load_alignment = 2'd1;
                    else
                        cpu_current_load_alignment = 2'd0;
                end

                if (dut.i_bus.entry_point == 12'h065) begin
                    cpu_branch_result[0] += 1;
                    if (dut.pf_spec_req)
                        cpu_branch_result[1] += 1;
                end
            end

            if (dut.vipt_load_ex_r.valid && dut.vipt_load_ex_probed_r) begin
                cpu_load_result[0] += 1;
                if (dut.vipt_load_ex_hit)
                    cpu_load_result[1] += 1;
                else if (!dut.vipt_load_ex_contained)
                    cpu_load_result[2] += 1;
                else if (!dut.vipt_translation_ok)
                    cpu_load_result[3] += 1;
                else if (dut.vipt_tlb_is_vga_mem)
                    cpu_load_result[4] += 1;
                else if (dut.seg_gp_fault)
                    cpu_load_result[5] += 1;
                else
                    cpu_load_result[6] += 1;
            end
            if (dut.vipt_load_slow_req_r && dut.mem_accepted)
                cpu_load_result[7] += 1;

            if (dut.jcc_fold_active && dut.uc_exec)
                cpu_branch_result[2] += 1;
            if (dut.branch_ustep_exec) begin
                if (dut.branch_ustep_redirect)
                    cpu_branch_result[3] += 1;
                else
                    cpu_branch_result[4] += 1;
            end
            if (dut.q_flush && dut.pf_spec_owner_r) begin
                if (dut.prefetch_inst.spec_line_match)
                    cpu_branch_result[5] += 1;
                else if (dut.prefetch_inst.spec_data_now)
                    cpu_branch_result[6] += 1;
                else
                    cpu_branch_result[7] += 1;
            end

            // Count completed REP elements at the architectural memory
            // handshake. This is stable across memory latency and DLY holds.
            if (dut.mem_accepted) begin
                if ((cpu_current_entry == 12'h1f8) &&
                    ((dut.uc_addr == 12'h1ff) || (dut.uc_addr == 12'h203)))
                    cpu_rep_elements[0] += 1;
                if ((cpu_current_entry == 12'h218) &&
                    (dut.uc_addr == 12'h222))
                    cpu_rep_elements[1] += 1;
                if ((cpu_current_entry == 12'h235) &&
                    (dut.uc_addr == 12'h23d))
                    cpu_rep_elements[2] += 1;
                if ((cpu_current_entry == 12'h24e) &&
                    (dut.uc_addr == 12'h252))
                    cpu_rep_elements[3] += 1;
                if ((cpu_current_entry == 12'h263) &&
                    (dut.uc_addr == 12'h267))
                    cpu_rep_elements[4] += 1;
            end

            cpu_stall_cycles[0] += dut.stall_mem;
            cpu_stall_cycles[1] += !dut.stall_mem && dut.stall_wio;
            cpu_stall_cycles[2] += !dut.stall_mem && !dut.stall_wio && dut.stall_d2;
            cpu_stall_cycles[3] += !dut.stall_mem && !dut.stall_wio &&
                                   !dut.stall_d2 && dut.stall_x87_direct;
            cpu_stall_cycles[4] += !dut.stall && dut.throttle_parked_r;
            cpu_stall_cycles[5] += !dut.stall && !dut.throttle_parked_r &&
                                   !dut.uc_active && dut.decq_empty && !dut.d2_valid;
            cpu_stall_cycles[6] += !dut.stall && !dut.throttle_parked_r &&
                                   !dut.uc_active && !(dut.decq_empty && !dut.d2_valid);
            cpu_stall_cycles[7] += dut.decoder_fetch_blocked;
            cpu_stall_cycles[8] += dut.mem_servicing;
            cpu_stall_cycles[9] += dut.q_flush;
            if (cpu_profile_active) begin
                cpu_entry_mem_stall[cpu_current_entry] += dut.stall_mem;
                cpu_entry_x87_stall[cpu_current_entry] += dut.stall_x87_direct;
                cpu_entry_frontend_wait[cpu_current_entry] +=
                    !dut.stall && !dut.throttle_parked_r && !dut.uc_active;
            end

            // Classify the cycle where a hardwired load can preload and chain its
            // successor. This is intentionally hierarchical: it is snapshot
            // profiling instrumentation, not part of the synthesized core.
            if (dut.hardwired_control_inst.recipe_state.hardwired &&
                (dut.hardwired_control_inst.recipe_state.commit_sel ==
                 z486_pkg::RECIPE_COMMIT_MEM) &&
                dut.hardwired_control_inst.uc_exec && dut.hardwired_control_inst.uc_next_rni &&
                !dut.hardwired_control_inst.i_issue) begin
                cpu_load_chain[0] += 1;
                if (dut.hardwired_control_inst.chain_start)
                    cpu_load_chain[1] += 1;
                else if (dut.hardwired_control_inst.decq_empty)
                    cpu_load_chain[2] += 1;
                else if (!dut.hardwired_control_inst.d2_push)
                    cpu_load_chain[3] += 1;
                else if (!dut.hardwired_control_inst.issue_recipe.hardwired)
                    cpu_load_chain[4] += 1;
                else if (dut.hardwired_control_inst.issue_recipe.reads_flags &&
                         dut.hardwired_control_inst.recipe_state.writes_flags &&
                         !dut.hardwired_control_inst.issue_recipe.jcc)
                    cpu_load_chain[5] += 1;
                else if (dut.hardwired_control_inst.issue_recipe.uses_ea &&
                         dut.hardwired_control_inst.ea2_conflict)
                    cpu_load_chain[6] += 1;
                else if (dut.hardwired_control_inst.loaduse_conflict)
                    cpu_load_chain[7] += 1;
                else if (dut.hardwired_control_inst.mem_confN)
                    cpu_load_chain[8] += 1;
                else if (dut.hardwired_control_inst.shift_confN)
                    cpu_load_chain[9] += 1;
                else if (!dut.hardwired_control_inst.head_chain_safe)
                    cpu_load_chain[10] += 1;
                else
                    cpu_load_chain[11] += 1;
            end
        end
    end

    generate
    if (ENABLE_X87) begin : gen_snapshot_x87_counters
        logic x87_event_window_active = 1'b0;
        longint x87_event_window_start = 0;
        logic [10:0] x87_event_window_fop = 11'h0;
        logic [11:0] x87_event_window_entry = 12'h0;
        longint x87_event_window_mem = 0;
        longint x87_event_window_port = 0;
        longint x87_event_window_guest_mem = 0;
        longint x87_event_window_wio = 0;
        longint x87_event_window_direct = 0;
        longint x87_event_window_frontend = 0;

        always @(posedge clk) begin
            if (reset_n) begin
                if ($test$plusargs("profile_x87_events")) begin
                    if (dut.i_issue) begin
                        if (x87_event_window_active)
                            $display("X87_EVENT WINDOW %0d %0d %0d %03x %03x %0d %0d %0d %0d %0d %0d",
                                     x87_event_window_start, cycle,
                                     cycle - x87_event_window_start,
                                     x87_event_window_fop,
                                     x87_event_window_entry,
                                     x87_event_window_mem,
                                     x87_event_window_port,
                                     x87_event_window_guest_mem,
                                     x87_event_window_wio,
                                     x87_event_window_direct,
                                     x87_event_window_frontend);
                        x87_event_window_active =
                            (dut.i_bus.opcode >= 8'hd8) &&
                            (dut.i_bus.opcode <= 8'hdf);
                        x87_event_window_start = cycle;
                        x87_event_window_fop = dut.i_bus.fop;
                        x87_event_window_entry = dut.i_bus.entry_point;
                        x87_event_window_mem = 0;
                        x87_event_window_port = 0;
                        x87_event_window_guest_mem = 0;
                        x87_event_window_wio = 0;
                        x87_event_window_direct = 0;
                        x87_event_window_frontend = 0;
                        if (x87_event_window_active)
                            $display("X87_EVENT ISSUE %0d %0d %02x %03x %03x %04x %0d",
                                     cycle, instruction_count,
                                     dut.i_bus.opcode, dut.i_bus.fop,
                                     dut.i_bus.entry_point,
                                     dut.x87.gen_x87.control.control_word,
                                     dut.x87.gen_x87.control.top);
                    end else if (x87_event_window_active) begin
                        x87_event_window_mem += dut.stall_mem;
                        x87_event_window_port +=
                            dut.stall_mem &&
                            ((dut.x87.gen_x87.bridge.state != 0) ||
                             dut.dcache_req_is_x87);
                        x87_event_window_guest_mem +=
                            dut.stall_mem &&
                            (dut.x87.gen_x87.bridge.state == 0) &&
                            !dut.dcache_req_is_x87;
                        x87_event_window_wio += dut.stall_wio;
                        x87_event_window_direct += dut.stall_x87_direct;
                        x87_event_window_frontend +=
                            !dut.stall && !dut.throttle_parked_r &&
                            !dut.uc_active;
                    end
                    if (dut.x87.gen_x87.cmd_valid &&
                        dut.x87.gen_x87.cmd_ready)
                        $display("X87_EVENT ACCEPT %0d protocol %03x",
                                 cycle, dut.x87.gen_x87.cmd_fop);
                    if (dut.x87.direct_valid && dut.x87.direct_ready)
                        $display("X87_EVENT ACCEPT %0d direct %03x",
                                 cycle, dut.i.fop);
                    if (dut.x87.gen_x87.control.v2_exec_start)
                        $display("X87_EVENT EXEC_START %0d %0d %0d %03x %0d %0d %0d",
                                 cycle,
                                 dut.x87.gen_x87.control.v2_exec_op,
                                 dut.x87.gen_x87.control.v2_exec_owner,
                                 dut.x87.gen_x87.control.last_fop,
                                 dut.x87.gen_x87.control.top,
                                 dut.x87.gen_x87.control.arith_dest_index,
                                 dut.x87.gen_x87.control.control_word[9:8]);
                    if (dut.x87.gen_x87.control.v2_exec_done)
                        $display("X87_EVENT EXEC_DONE %0d %0d %0d %03x",
                                 cycle,
                                 dut.x87.gen_x87.control.v2_exec_op,
                                 dut.x87.gen_x87.control.v2_exec_owner,
                                 dut.x87.gen_x87.control.last_fop);
                end
                if (dut.x87.gen_x87.cmd_valid && dut.x87.gen_x87.cmd_ready) begin
                    x87_command_count <= x87_command_count + 1;
                    x87_fop_count[dut.x87.gen_x87.cmd_fop] <=
                        x87_fop_count[dut.x87.gen_x87.cmd_fop] + 1;
                end
                if (dut.x87.direct_valid && dut.x87.direct_ready) begin
                    x87_direct_load_count <= x87_direct_load_count + 1;
                    x87_direct_load_fop_count[dut.i.fop] <=
                        x87_direct_load_fop_count[dut.i.fop] + 1;
                end
                if (!dut.x87_busy_n)
                    x87_control_busy_cycles <= x87_control_busy_cycles + 1;
                if (dut.x87.gen_x87.control.executor.busy)
                    x87_executor_busy_cycles <= x87_executor_busy_cycles + 1;
                if (dut.x87.gen_x87.control.v2_exec_start)
                    x87_exec_start_count[dut.x87.gen_x87.control.v2_exec_op] <=
                        x87_exec_start_count[dut.x87.gen_x87.control.v2_exec_op] + 1;
                if (dut.x87.gen_x87.control.executor.busy)
                    x87_exec_busy_count[dut.x87.gen_x87.control.v2_exec_op] <=
                        x87_exec_busy_count[dut.x87.gen_x87.control.v2_exec_op] + 1;
                if (dut.stall_wio)
                    x87_wait_stall_cycles <= x87_wait_stall_cycles + 1;
                if ($test$plusargs("profile_cpu")) begin
                    if (dut.stall_x87_direct) begin
                        cpu_x87_wait_reason[0] += 1;
                        if (!dut.x87.direct_issued_r)
                            cpu_x87_wait_reason[1] += 1;
                        else if (!dut.x87.direct_data_valid_r)
                            cpu_x87_wait_reason[2] += 1;
                        else if (!dut.x87.direct_ready)
                            cpu_x87_wait_reason[3] += 1;
                    end
                    if (dut.stall_wio) begin
                        cpu_x87_wait_reason[4] += 1;
                        if (!dut.x87_busy_n)
                            cpu_x87_wait_reason[5] += 1;
                        if (dut.x87.gen_x87.control.executor.busy)
                            cpu_x87_wait_reason[6] += 1;
                        if (dut.x87.gen_x87.control.direct_m32_pending)
                            cpu_x87_wait_reason[7] += 1;
                    end
                end
            end
        end
    end
    endgenerate

    generate
    if (ENABLE_X87) begin : gen_snapshot_x87_init
        always @(load_snapshot_x87_state) begin
            dut.x87.gen_x87.control.control_word = x87_control_arg;
            dut.x87.gen_x87.control.status_flags = x87_status_arg;
            dut.x87.gen_x87.control.top = x87_top_arg;
            dut.x87.gen_x87.control.tag_word = x87_tag_arg;
            dut.x87.gen_x87.control.stack_mem.mem[0] = x87_fpr_arg[0];
            dut.x87.gen_x87.control.stack_mem.mem[1] = x87_fpr_arg[1];
            dut.x87.gen_x87.control.stack_mem.mem[2] = x87_fpr_arg[2];
            dut.x87.gen_x87.control.stack_mem.mem[3] = x87_fpr_arg[3];
            dut.x87.gen_x87.control.stack_mem.mem[4] = x87_fpr_arg[4];
            dut.x87.gen_x87.control.stack_mem.mem[5] = x87_fpr_arg[5];
            dut.x87.gen_x87.control.stack_mem.mem[6] = x87_fpr_arg[6];
            dut.x87.gen_x87.control.stack_mem.mem[7] = x87_fpr_arg[7];
        end
    end
    endgenerate

    // The 386-gate path clears the gate-width flag before common stack setup.
    reg cmisc2_executed = 1'b0;
    always @(posedge clk) begin
        if (!reset_n) begin
            cmisc2_executed <= 1'b0;
        end else begin
            if (cmisc2_executed && dut.misc2_flag)
                $fatal(1, "CMISC2 failed to clear misc2_flag");
            cmisc2_executed <= dut.uc_exec && (dut.uc_aluop == 7'h3C);
        end
    end

    // Test result tracking
    reg [7:0] test_status = 8'h00;  // 0x00=running, 0x01=pass, 0xFF=fail
    reg [31:0] test_data = 32'h0;
    reg test_done = 0;
    longint vipt_ea_interlock_cycles = 0;
    longint vipt_store_wait_issues = 0;
    longint vipt_store_replays = 0;
    reg vipt_store_replay_pending = 0;

    // A load-to-AGI regression must both exercise the dependency and prevent
    // the dependent instruction from issuing while its EA input is pending.
    always @(posedge clk) begin
        if (reset_n && dut.d2_vipt_ea_hazard)
            vipt_ea_interlock_cycles <= vipt_ea_interlock_cycles + 1;
        if (reset_n && dut.i_issue && dut.d2_vipt_ea_hazard)
            $fatal(1, "VIPT load-to-EA consumer issued before writeback");
        if (!reset_n || dut.q_flush || dut.any_fault) begin
            vipt_store_replay_pending <= 1'b0;
        end else if (dut.vipt_issue_store_wait) begin
            vipt_store_wait_issues <= vipt_store_wait_issues + 1;
            vipt_store_replay_pending <= 1'b1;
        end else if (vipt_store_replay_pending && dut.vipt_replay_try &&
                     dut.dcache_vipt_probe_accepted) begin
            vipt_store_replays <= vipt_store_replays + 1;
            vipt_store_replay_pending <= 1'b0;
        end
    end

    // Hardware interrupt emulation
    reg [7:0] intr_vector = 8'h20;     // Vector to return on INTA
    reg       intr_request = 0;        // Pending INTR request from signal port
    reg       nmi_request = 0;         // Pending NMI request from signal port
    int       intr_delay = 0;          // Cycles to delay before asserting intr/nmi
    int       signal_delay_cycles = 50;// Configurable cycle delay for signal trigger
    int       signal_delay_instr = 0;  // Optional retired-instruction delay for trigger
    int       intr_instr_remaining = 0;
    int       nmi_instr_remaining = 0;
    reg       intr_hardwired_load_arm = 0;
    int       nmi_pulse_cycles = 1;    // NMI high width in cycles (for deterministic edge delivery)
    int       nmi_hold_count = 0;
    reg       inta_first = 0;          // Track first vs second INTA pair
    reg       inta_resp_pending = 0;
    reg [31:0] inta_resp_data = 32'h0;

    // Configurable memory latency (default 1 cycle, +mem_latency=N for more)
    int mem_latency = 1;
    int rd_wait_count = 0;
    reg [31:0] rd_byte_addr = 32'h0;
    reg [7:0] rd_remaining = 8'd0;
    reg [7:0] rd_index = 8'd0;
    reg rd_io_pending = 1'b0;
    // Store back-pressure (+wr_busy=N: after each write, ready low for 0..N cycles)
    int mem_seed = 1;
    int wr_busy_max = 0;
    int wr_wait_count = 0;
    // Periodic timer-like INTR (+intr_period=P cycles between INTA-completion and next assert,
    // +intr_jit=J adds random 0..J). Level is held until INTA completes, like a PIC.
    int intr_period = 0;
    int intr_jit = 0;
    int intr_gap_cnt = 0;
    int intr_gap_next = 0;
    wire rd_busy = (rd_wait_count != 0) || (rd_remaining != 0) || inta_resp_pending || (wr_wait_count != 0);

    // Memory behavior with configurable latency (ready/valid protocol)
    // Note: din is held stable (not cleared) to allow paging unit to sample it
    // when pg_mem_ready is asserted (which has 1-cycle delay from bus ready)
    always @(posedge clk) begin
        if (ENABLE_X87 && reset_n && valid && io &&
            (dut.i.opcode >= 8'hd8) && (dut.i.opcode <= 8'hdf))
            $fatal(1, "x87 transaction escaped to external I/O: addr=%08x write=%b",
                   {addr, 2'b00}, write);

        ready <= !rd_busy;
        if (wr_wait_count != 0) wr_wait_count <= wr_wait_count - 1;
        if (intr_period > 0 && reset_n) begin
            if (!intr && !inta_resp_pending && !inta_first) begin
                if (intr_gap_cnt >= intr_gap_next) begin
                    intr <= 1'b1;
                    intr_gap_cnt <= 0;
                    intr_gap_next <= intr_period + ((intr_jit > 0) ? int'($urandom % (intr_jit + 1)) : 0);
                end else
                    intr_gap_cnt <= intr_gap_cnt + 1;
            end
        end
        resp_valid <= 1'b0;
        // Don't clear din - hold it stable for page walker timing

        // Handle pending read with latency countdown, then return one DWORD per cycle.
        if (inta_resp_pending) begin
            resp_valid <= 1'b1;
            din <= inta_resp_data;
            inta_resp_pending <= 1'b0;
        end else if (rd_wait_count != 0) begin
            rd_wait_count <= rd_wait_count - 1;
        end else if (rd_remaining != 8'd0) begin
            reg [31:0] byte_addr;
            byte_addr = rd_byte_addr + {22'd0, rd_index, 2'b00};
            if (byte_addr >= MEM_SIZE)
                byte_addr = byte_addr & (MEM_SIZE - 1);

            resp_valid <= 1'b1;
            if (rd_io_pending) begin
                din <= 32'hFFFFFFFF;
            end else begin
                din <= {mem[byte_addr+3], mem[byte_addr+2],
                        mem[byte_addr+1], mem[byte_addr+0]};

                if ($test$plusargs("trace_mem"))
                    $display("MEM RESP @%08x = %08x",
                             byte_addr,
                             {mem[byte_addr+3], mem[byte_addr+2],
                              mem[byte_addr+1], mem[byte_addr+0]});
            end

            rd_index <= rd_index + 8'd1;
            rd_remaining <= rd_remaining - 8'd1;
        end

        if (valid && ready && !rd_busy) begin
            // Coprocessor cycles use reserved 32-bit I/O addresses. The normal
            // I/O trace intentionally truncates to programmed 16-bit ports,
            // which would alias these transactions with test-control ports.
            if ($test$plusargs("trace_x87") && io && addr[31]) begin
                if (write)
                    $display("X87 REQ eip=%08x uc=%03h addr=%08x WR be=%x data=%08x",
                             dut.EIP, dut.uc_addr, {addr, 2'b00}, be, dout);
                else
                    $display("X87 REQ eip=%08x uc=%03h addr=%08x RD be=%x data=ffffffff",
                             dut.EIP, dut.uc_addr, {addr, 2'b00}, be);
            end

            if (inta) begin
                // INTA bus cycle handling
                ready <= 1'b0;
                inta_resp_pending <= 1'b1;
                if (!inta_first) begin
                    inta_first <= 1'b1;
                    inta_resp_data <= 32'h0;
                    if ($test$plusargs("trace_io"))
                        $display("INTA cycle 1 (dummy)");
                end else begin
                    inta_first <= 1'b0;
                    inta_resp_data <= {24'h0, intr_vector};
                    intr <= 1'b0;
                    if ($test$plusargs("trace_io"))
                        $display("INTA cycle 2: vector=0x%02X", intr_vector);
                end
            end else if (!write) begin
                // Read
                reg [7:0] burst_len;
                burst_len = io ? 8'd1 : burstcount;
                rd_byte_addr <= {addr, 2'b00};
                rd_index <= (mem_latency <= 1) ? 8'd1 : 8'd0;
                rd_wait_count <= (mem_latency <= 1) ? 0 : (mem_latency - 1);
                rd_io_pending <= io;

                if (!io) begin
                    reg [31:0] byte_addr;
                    byte_addr = {addr, 2'b00};

                    if (byte_addr >= MEM_SIZE)
                        byte_addr = byte_addr & (MEM_SIZE - 1);

                    if ($test$plusargs("trace_mem"))
                        $display("MEM RD @%08x count=%0d first=%08x", byte_addr, burstcount,
                                 {mem[byte_addr+3], mem[byte_addr+2],
                                  mem[byte_addr+1], mem[byte_addr+0]});
                    if (mem_latency <= 1) begin
                        resp_valid <= 1'b1;
                        din <= {mem[byte_addr+3], mem[byte_addr+2],
                                mem[byte_addr+1], mem[byte_addr+0]};
                    end
                end else if (mem_latency <= 1) begin
                    resp_valid <= 1'b1;
                    din <= 32'hFFFFFFFF;
                end
                rd_remaining <= (mem_latency <= 1 && burst_len > 8'd1) ?
                                (burst_len - 8'd1) : burst_len;
                ready <= (mem_latency <= 1 && burst_len <= 8'd1);
            end else begin
                // Write
                ready <= 1'b1;
                if (!io && wr_busy_max > 0) begin
                    automatic int wn = int'($urandom % (wr_busy_max + 1));
                    wr_wait_count <= wn;
                    ready <= (wn == 0);
                end
                if (io) begin
                // I/O writes - check result ports
                reg [15:0] port;
                port = {addr[15:2], 2'b00};

                // Status port (0xE0) - test result
                if (port == 16'h00E0) begin
                    test_status <= dout[7:0];
                    if (dout[7:0] == 8'h01) begin
                        if (($test$plusargs("expect_vipt_ea_interlock") &&
                             (vipt_ea_interlock_cycles == 0)) ||
                            ($test$plusargs("expect_vipt_store_replay") &&
                             ((vipt_store_wait_issues == 0) ||
                              (vipt_store_replays == 0)))) begin
                            test_status <= 8'hFF;
                            $display("");
                            $display("========================================");
                            $display("  TEST FAILED!");
                            if (vipt_ea_interlock_cycles == 0)
                                $display("  VIPT load-to-EA interlock was not exercised");
                            else
                                $display("  VIPT older-store replay was not exercised");
                            $display("========================================");
                            test_done <= 1;
                        end else begin
                            $display("");
                            $display("========================================");
                            $display("  TEST PASSED!");
                            $display("  Total cycles: %0d", cycle);
                            $display("  Total instructions: %0d", instruction_count);
                            if ($test$plusargs("expect_vipt_ea_interlock"))
                                $display("  VIPT EA interlock cycles: %0d",
                                         vipt_ea_interlock_cycles);
                            if ($test$plusargs("expect_vipt_store_replay"))
                                $display("  VIPT store waits/replays: %0d/%0d",
                                         vipt_store_wait_issues,
                                         vipt_store_replays);
                            $display("========================================");
                            test_done <= 1;
                        end
                    end else if (dout[7:0] == 8'hFF) begin
                        $display("");
                        $display("========================================");
                        $display("  TEST FAILED!");
                        $display("  Failure data: 0x%08X", test_data);
                        $display("  Total cycles: %0d", cycle);
                        $display("  CS:EIP: %04X:%08X", dut.CS, dut.EIP);
                        $display("========================================");
                        test_done <= 1;
                    end
                end

                // Data port (0xE4) - debug/verification data
                if (port == 16'h00E4) begin
                    test_data <= dout;
                    if ($test$plusargs("trace_io"))
                        $display("TEST DATA: 0x%08X", dout);
                end

                // Signal port (0xE8) - trigger hardware interrupts
                // 1 = assert INTR after short delay
                // 2 = pulse NMI after short delay
                // 3 = assert INTR while IF=0 (masked test)
                // 4 = assert INTR during the next hardwired absolute load,
                //     so it becomes pending at the load's retirement boundary
                if (port == 16'h00E8) begin
                    if (dout[7:0] == 8'h01 || dout[7:0] == 8'h03) begin
                        if (signal_delay_instr > 0) begin
                            intr_instr_remaining <= signal_delay_instr;
                            intr_request <= 1'b1;
                        end else begin
                            intr_request <= 1'b1;
                            intr_delay <= signal_delay_cycles;
                        end
                        if ($test$plusargs("trace_io"))
                            $display("SIGNAL: INTR requested (mode=%0d, cyc=%0d, instr=%0d)",
                                     dout[7:0], signal_delay_cycles, signal_delay_instr);
                    end
                    if (dout[7:0] == 8'h02) begin
                        if (signal_delay_instr > 0) begin
                            nmi_instr_remaining <= signal_delay_instr;
                            nmi_request <= 1'b1;
                        end else begin
                            nmi_request <= 1'b1;
                            intr_delay <= signal_delay_cycles;
                        end
                        if ($test$plusargs("trace_io"))
                            $display("SIGNAL: NMI requested (cyc=%0d, instr=%0d)",
                                     signal_delay_cycles, signal_delay_instr);
                    end
                    if (dout[7:0] == 8'h04) begin
                        intr_hardwired_load_arm <= 1'b1;
                        if ($test$plusargs("trace_io"))
                            $display("SIGNAL: INTR armed for hardwired load boundary");
                    end
                end

                // Signal delay (0xEC) - low 16 bits are cycle delay
                if (port == 16'h00EC) begin
                    signal_delay_cycles <= dout[15:0];
                    if ($test$plusargs("trace_io"))
                        $display("SIGNAL CFG: cycle delay=%0d", dout[15:0]);
                end

                // Signal instruction delay (0xF0) - low 16 bits are retired instruction delay
                // 0 means use cycle delay mode.
                if (port == 16'h00F0) begin
                    signal_delay_instr <= dout[15:0];
                    if ($test$plusargs("trace_io"))
                        $display("SIGNAL CFG: instruction delay=%0d", dout[15:0]);
                end

                // Signal vector (0xF4) - low 8 bits are INTR vector
                if (port == 16'h00F4) begin
                    intr_vector <= dout[7:0];
                    if ($test$plusargs("trace_io"))
                        $display("SIGNAL CFG: INTR vector=0x%02X", dout[7:0]);
                end

                // NMI pulse width (0xF8) - low 16 bits = cycles to keep nmi asserted
                if (port == 16'h00F8) begin
                    if (dout[15:0] == 0)
                        nmi_pulse_cycles <= 1;
                    else
                        nmi_pulse_cycles <= dout[15:0];
                    if ($test$plusargs("trace_io"))
                        $display("SIGNAL CFG: NMI pulse cycles=%0d",
                                 (dout[15:0] == 0) ? 1 : dout[15:0]);
                end

                if ($test$plusargs("trace_io"))
                    $display("IO WR port=%04x data=%08x", port, dout);
            end else begin
                // Memory writes
                reg [31:0] byte_addr;
                byte_addr = {addr, 2'b00};

                if (byte_addr < MEM_SIZE) begin
                    if (be[0]) mem[byte_addr+0] <= dout[7:0];
                    if (be[1]) mem[byte_addr+1] <= dout[15:8];
                    if (be[2]) mem[byte_addr+2] <= dout[23:16];
                    if (be[3]) mem[byte_addr+3] <= dout[31:24];
                end

                if ($test$plusargs("trace_mem"))
                    $display("MEM WR @%08x be=%b data=%08x uc=%03h mem_wdata=%08h src=%02h",
                             byte_addr, be, dout, dut.uc_addr, dut.mem_wdata, dut.uc_source);
            end
        end
    end
    end

    // Configuration from plusargs
    string memfile, raw_memfile;
    integer raw_mem_fd, raw_mem_bytes;
    int eip_arg;
    int cr0_arg, cr2_arg, cr3_arg;
    int code_phys_base;  // Physical address where code is loaded (for prefetch)
    int start_protected; // 1: force protected-mode entry state, 0: start in real mode
    int d_init;          // Initial default operand size flag
    bit snapshot_mode;
    bit stop_eip_valid;
    int stop_eip, stop_cs;
    int stop_occurrence, stop_match_count;
    int checksum_count;
    int checksum_start [0:3];
    int checksum_bytes [0:3];

    logic [31:0] init_eax, init_ebx, init_ecx, init_edx;
    logic [31:0] init_esi, init_edi, init_ebp, init_esp, init_eflags;
    logic [15:0] init_ldtr, init_tr;
    logic [31:0] init_gdt_base, init_idt_base;
    logic [19:0] init_gdt_limit, init_idt_limit;
    logic [31:0] ldt_base, tr_base;
    logic [19:0] ldt_limit, tr_limit;
    logic [15:0] ldt_flags, tr_flags;

    logic [15:0] x87_control_arg, x87_status_arg, x87_tag_arg;
    logic [2:0]  x87_top_arg;
    logic [79:0] x87_fpr_arg [0:7];

    // Initial visible segment selectors
    int init_cs, init_ds, init_ss, init_es, init_fs, init_gs;

    // Segment descriptor cache values from plusargs
    int cs_base, cs_limit, cs_flags;
    int ds_base, ds_limit, ds_flags;
    int ss_base, ss_limit, ss_flags;
    int es_base, es_limit, es_flags;
    int fs_base, fs_limit, fs_flags;
    int gs_base, gs_limit, gs_flags;

    // Build seg_desc_t from flags
    // flags[15:12] = type, flags[11] = S, flags[10:9] = DPL, flags[8] = P,
    // flags[7] = D_B, flags[6] = G, flags[5] = A
    function automatic z486_pkg::seg_desc_t build_seg_desc(
        input [31:0] base, input [19:0] limit, input [15:0] flags
    );
        z486_pkg::seg_desc_t desc;
        desc.base       = base;
        desc.limit      = limit;
        desc.seg_type   = flags[15:12];
        desc.S          = flags[11];
        desc.DPL        = flags[10:9];
        desc.P          = flags[8];
        desc.D_B        = flags[7];
        desc.G          = flags[6];
        desc.A          = flags[5];
        return desc;
    endfunction

    // Default protected mode descriptor flags:
    // type=0010 (data RW), S=1, DPL=0, P=1, D_B=1 (32-bit), G=1 (4K), A=1
    localparam DEFAULT_DATA_FLAGS = 16'h21E0;  // Data RW, S=1, DPL=0, P=1, D_B=1, G=1, A=1
    localparam DEFAULT_CODE_FLAGS = 16'hA1E0;  // Code RX, S=1, DPL=0, P=1, D_B=1, G=1, A=1
    // Real-mode: D_B=0 (16-bit), G=0 (byte granularity), P=1, A=0
    // flags: type[15:12] S[11] DPL[10:9] P[8] D_B[7] G[6] A[5]
    localparam RM_DATA_FLAGS = 16'h2900;       // type=0010(RW), S=1, DPL=0, P=1, D_B=0, G=0
    localparam RM_CODE_FLAGS = 16'hA900;       // type=1010(RX), S=1, DPL=0, P=1, D_B=0, G=0

    initial begin
        // Snapshot RAM is already complete, so avoid clearing a 64MB array
        // before loading it. Small instruction tests retain the hex path.
        if ($value$plusargs("raw_mem=%s", raw_memfile)) begin
            raw_mem_fd = $fopen(raw_memfile, "rb");
            if (!raw_mem_fd) begin
                $display("[TB] ERROR: Could not open raw memory %s", raw_memfile);
                $finish;
            end
            raw_mem_bytes = $fread(mem, raw_mem_fd);
            $fclose(raw_mem_fd);
            if (raw_mem_bytes != MEM_SIZE) begin
                $display("[TB] ERROR: Raw memory size %0d, expected %0d",
                         raw_mem_bytes, MEM_SIZE);
                $finish;
            end
            $display("[TB] Loaded %0d raw memory bytes from %s",
                     raw_mem_bytes, raw_memfile);
        end else if ($value$plusargs("mem=%s", memfile)) begin
            for (int i = 0; i < MEM_SIZE; i++)
                mem[i] = 8'h00;
            $readmemh(memfile, mem);
            $display("[TB] Loaded memory from %s", memfile);
        end else begin
            $display("[TB] ERROR: No memory file specified (+mem= or +raw_mem=)");
            $finish;
        end

        snapshot_mode = $test$plusargs("snapshot");
        for (int i = 0; i < 2048; i++)
            x87_fop_count[i] = 0;
        for (int i = 0; i < 2048; i++)
            x87_direct_load_fop_count[i] = 0;
        for (int i = 0; i < 16; i++) begin
            x87_exec_start_count[i] = 0;
            x87_exec_busy_count[i] = 0;
        end
        for (int i = 0; i < 4096; i++) begin
            cpu_entry_count[i] = 0;
            cpu_entry_cycles[i] = 0;
            cpu_entry_mem_stall[i] = 0;
            cpu_entry_x87_stall[i] = 0;
            cpu_entry_frontend_wait[i] = 0;
            cpu_store_next_entry[i] = 0;
            for (int j = 0; j < 4; j++)
                cpu_store_next_entry_gap[i][j] = 0;
        end
        for (int i = 0; i < 256; i++) begin
            cpu_opcode_count[i] = 0;
            cpu_opcode_cycles[i] = 0;
            cpu_store_next_opcode[i] = 0;
            for (int j = 0; j < 4; j++)
                cpu_store_next_opcode_gap[i][j] = 0;
        end
        for (int i = 0; i < 2; i++) begin
            cpu_hardwired_count[i] = 0;
            cpu_hardwired_cycles[i] = 0;
        end
        for (int i = 0; i < 8; i++) begin
            cpu_commit_count[i] = 0;
            cpu_commit_cycles[i] = 0;
        end
        for (int i = 0; i < 10; i++)
            cpu_stall_cycles[i] = 0;
        for (int i = 0; i < 12; i++)
            cpu_load_chain[i] = 0;
        for (int i = 0; i < 17; i++)
            cpu_load_intervals[i] = 0;
        for (int i = 0; i < 8; i++) begin
            cpu_load_issue[i] = 0;
            cpu_load_result[i] = 0;
            cpu_branch_result[i] = 0;
            cpu_x87_wait_reason[i] = 0;
        end
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 4; j++)
                cpu_load_successor[i][j] = 0;
        for (int path = 0; path < 3; path++) begin
            for (int interval = 0; interval < 17; interval++)
                cpu_load_path_interval[path][interval] = 0;
            for (int dependency = 0; dependency < 4; dependency++) begin
                cpu_load_path_successor_count[path][dependency] = 0;
                cpu_load_path_successor_cycles[path][dependency] = 0;
            end
            for (int entry = 0; entry < 4096; entry++) begin
                cpu_load_next_entry_count[path][entry] = 0;
                cpu_load_next_entry_cycles[path][entry] = 0;
            end
            for (int addr32 = 0; addr32 < 2; addr32++)
                for (int size = 0; size < 3; size++)
                    for (int ea_class = 0; ea_class < 3; ea_class++)
                        for (int alignment = 0; alignment < 4; alignment++) begin
                            cpu_load_form_count[path][addr32][size]
                                               [ea_class][alignment] = 0;
                            cpu_load_form_cycles[path][addr32][size]
                                                [ea_class][alignment] = 0;
                        end
        end
        for (int i = 0; i < 5; i++)
            cpu_rep_elements[i] = 0;
        for (int i = 0; i < 17; i++)
            cpu_store_intervals[i] = 0;
        for (int i = 0; i < 10; i++)
            cpu_store_states[i] = 0;
        cpu_current_start = 0;
        cpu_current_entry = 0;
        cpu_current_opcode = 0;
        cpu_current_hardwired = 0;
        cpu_current_commit = 0;
        cpu_current_load_dst_mask = 0;
        cpu_current_load_direct = 0;
        cpu_current_load_path = 0;
        cpu_current_load_addr32 = 0;
        cpu_current_load_size = 0;
        cpu_current_load_ea_class = 0;
        cpu_current_load_alignment = 0;
        cpu_profile_active = 0;

        // Get max cycles
        if ($value$plusargs("cycles=%d", max_cycles))
            $display("[TB] Max cycles: %0d", max_cycles);
        if ($value$plusargs("mem_seed=%d", mem_seed)) void'($urandom(mem_seed));
        if ($value$plusargs("wr_busy=%d", wr_busy_max)) $display("[TB] write busy 0..%0d", wr_busy_max);
        if ($value$plusargs("intr_period=%d", intr_period)) begin
            $display("[TB] periodic INTR period %0d", intr_period);
            intr_gap_next = intr_period;
        end
        void'($value$plusargs("intr_jit=%d", intr_jit));
        if ($value$plusargs("mem_latency=%d", mem_latency))
            $display("[TB] Memory latency: %0d cycles", mem_latency);
        if ($test$plusargs("continue_on_hlt"))
            stop_on_hlt = 1'b0;

        // Get initial EIP (default 0)
        if (!$value$plusargs("eip=%d", eip_arg)) eip_arg = 0;

        // Get control registers
        if (!$value$plusargs("cr0=%d", cr0_arg)) cr0_arg = 32'h80000001;  // PE=1, PG=1
        if (!$value$plusargs("cr2=%d", cr2_arg)) cr2_arg = 32'h00000000;
        if (!$value$plusargs("cr3=%d", cr3_arg)) cr3_arg = 32'h00000000;  // Page dir at 0

        // Get physical address where code is loaded (prefetch bypasses paging)
        if (!$value$plusargs("code_phys_base=%d", code_phys_base)) code_phys_base = 32'h00010000;

        // Start mode (default protected for backward compatibility)
        if (!$value$plusargs("start_protected=%d", start_protected)) start_protected = 1;

        // Get segment descriptor parameters
        // CS
        if (!$value$plusargs("cs_base=%d", cs_base)) cs_base = 32'h10000000;
        if (!$value$plusargs("cs_limit=%d", cs_limit)) cs_limit = 20'hFFFFF;
        if (!$value$plusargs("cs_flags=%d", cs_flags)) cs_flags = DEFAULT_CODE_FLAGS;
        // DS
        if (!$value$plusargs("ds_base=%d", ds_base)) ds_base = 32'h20000000;
        if (!$value$plusargs("ds_limit=%d", ds_limit)) ds_limit = 20'hFFFFF;
        if (!$value$plusargs("ds_flags=%d", ds_flags)) ds_flags = DEFAULT_DATA_FLAGS;
        // SS
        if (!$value$plusargs("ss_base=%d", ss_base)) ss_base = 32'h30000000;
        if (!$value$plusargs("ss_limit=%d", ss_limit)) ss_limit = 20'hFFFFF;
        if (!$value$plusargs("ss_flags=%d", ss_flags)) ss_flags = DEFAULT_DATA_FLAGS;
        // ES
        if (!$value$plusargs("es_base=%d", es_base)) es_base = 32'h00000000;
        if (!$value$plusargs("es_limit=%d", es_limit)) es_limit = 20'hFFFFF;
        if (!$value$plusargs("es_flags=%d", es_flags)) es_flags = DEFAULT_DATA_FLAGS;
        // FS
        if (!$value$plusargs("fs_base=%d", fs_base)) fs_base = 32'h00000000;
        if (!$value$plusargs("fs_limit=%d", fs_limit)) fs_limit = 20'hFFFFF;
        if (!$value$plusargs("fs_flags=%d", fs_flags)) fs_flags = DEFAULT_DATA_FLAGS;
        // GS
        if (!$value$plusargs("gs_base=%d", gs_base)) gs_base = 32'h00000000;
        if (!$value$plusargs("gs_limit=%d", gs_limit)) gs_limit = 20'hFFFFF;
        if (!$value$plusargs("gs_flags=%d", gs_flags)) gs_flags = DEFAULT_DATA_FLAGS;

        // Real-mode: override segment descriptors to 16-bit mode
        if (!start_protected) begin
            if (!$value$plusargs("cs_base=%d", cs_base)) cs_base = code_phys_base;
            if (!$value$plusargs("ds_base=%d", ds_base)) ds_base = 32'h00000000;
            if (!$value$plusargs("ss_base=%d", ss_base)) ss_base = 32'h00000000;
            if (!$value$plusargs("cs_limit=%d", cs_limit)) cs_limit = 20'h0FFFF;
            if (!$value$plusargs("ds_limit=%d", ds_limit)) ds_limit = 20'h0FFFF;
            if (!$value$plusargs("ss_limit=%d", ss_limit)) ss_limit = 20'h0FFFF;
            if (!$value$plusargs("es_limit=%d", es_limit)) es_limit = 20'h0FFFF;
            if (!$value$plusargs("fs_limit=%d", fs_limit)) fs_limit = 20'h0FFFF;
            if (!$value$plusargs("gs_limit=%d", gs_limit)) gs_limit = 20'h0FFFF;
            if (!$value$plusargs("cs_flags=%d", cs_flags)) cs_flags = RM_CODE_FLAGS;
            if (!$value$plusargs("ds_flags=%d", ds_flags)) ds_flags = RM_DATA_FLAGS;
            if (!$value$plusargs("ss_flags=%d", ss_flags)) ss_flags = RM_DATA_FLAGS;
            if (!$value$plusargs("es_flags=%d", es_flags)) es_flags = RM_DATA_FLAGS;
            if (!$value$plusargs("fs_flags=%d", fs_flags)) fs_flags = RM_DATA_FLAGS;
            if (!$value$plusargs("gs_flags=%d", gs_flags)) gs_flags = RM_DATA_FLAGS;
        end

        // Initial segment selectors:
        // - protected start defaults to canonical GDT selectors
        // - real-mode start defaults to CS derived from code physical address
        if (!$value$plusargs("init_cs=%d", init_cs))
            init_cs = start_protected ? 16'h0008 : ((code_phys_base >> 4) & 16'hFFFF);
        if (!$value$plusargs("init_ds=%d", init_ds))
            init_ds = start_protected ? 16'h0010 : 16'h0000;
        if (!$value$plusargs("init_ss=%d", init_ss))
            init_ss = start_protected ? 16'h0018 : 16'h0000;
        if (!$value$plusargs("init_es=%d", init_es))
            init_es = start_protected ? 16'h0020 : 16'h0000;
        if (!$value$plusargs("init_fs=%d", init_fs))
            init_fs = start_protected ? 16'h0028 : 16'h0000;
        if (!$value$plusargs("init_gs=%d", init_gs))
            init_gs = start_protected ? 16'h0030 : 16'h0000;

        // Initial default operand-size mode
        if (!$value$plusargs("d_init=%d", d_init))
            d_init = start_protected ? 1 : 0;

        init_eax = 0; init_ebx = 0; init_ecx = 0; init_edx = 0;
        init_esi = 0; init_edi = 0; init_ebp = 0; init_esp = 32'h0000ff00;
        init_eflags = 32'h00000002;
        void'($value$plusargs("init_eax=%h", init_eax));
        void'($value$plusargs("init_ebx=%h", init_ebx));
        void'($value$plusargs("init_ecx=%h", init_ecx));
        void'($value$plusargs("init_edx=%h", init_edx));
        void'($value$plusargs("init_esi=%h", init_esi));
        void'($value$plusargs("init_edi=%h", init_edi));
        void'($value$plusargs("init_ebp=%h", init_ebp));
        void'($value$plusargs("init_esp=%h", init_esp));
        void'($value$plusargs("init_eflags=%h", init_eflags));

        init_ldtr = 0; init_tr = 0;
        init_gdt_base = 0; init_gdt_limit = 20'h003ff;
        init_idt_base = 0; init_idt_limit = 20'h003ff;
        ldt_base = 0; ldt_limit = 0; ldt_flags = 0;
        tr_base = 0; tr_limit = 0; tr_flags = 0;
        void'($value$plusargs("init_ldtr=%h", init_ldtr));
        void'($value$plusargs("init_tr=%h", init_tr));
        void'($value$plusargs("gdt_base=%h", init_gdt_base));
        void'($value$plusargs("gdt_limit=%h", init_gdt_limit));
        void'($value$plusargs("idt_base=%h", init_idt_base));
        void'($value$plusargs("idt_limit=%h", init_idt_limit));
        void'($value$plusargs("ldt_base=%h", ldt_base));
        void'($value$plusargs("ldt_limit=%h", ldt_limit));
        void'($value$plusargs("ldt_flags=%h", ldt_flags));
        void'($value$plusargs("tr_base=%h", tr_base));
        void'($value$plusargs("tr_limit=%h", tr_limit));
        void'($value$plusargs("tr_flags=%h", tr_flags));

        stop_eip_valid = $value$plusargs("stop_eip=%h", stop_eip);
        if (!$value$plusargs("stop_cs=%h", stop_cs)) stop_cs = init_cs;
        if (!$value$plusargs("stop_occurrence=%d", stop_occurrence)) stop_occurrence = 1;
        if (stop_occurrence < 1) stop_occurrence = 1;
        stop_match_count = 0;
        if (!$value$plusargs("checksum_count=%d", checksum_count)) begin
            checksum_count = 1;
            if (!$value$plusargs("checksum_start=%h", checksum_start[0])) checksum_start[0] = 0;
            if (!$value$plusargs("checksum_bytes=%h", checksum_bytes[0])) checksum_bytes[0] = 0;
        end else begin
            if (checksum_count < 1) checksum_count = 1;
            if (checksum_count > 4) checksum_count = 4;
            for (int i = 0; i < 4; i++) begin
                checksum_start[i] = 0;
                checksum_bytes[i] = 0;
            end
            void'($value$plusargs("checksum0_start=%h", checksum_start[0]));
            void'($value$plusargs("checksum0_bytes=%h", checksum_bytes[0]));
            void'($value$plusargs("checksum1_start=%h", checksum_start[1]));
            void'($value$plusargs("checksum1_bytes=%h", checksum_bytes[1]));
            void'($value$plusargs("checksum2_start=%h", checksum_start[2]));
            void'($value$plusargs("checksum2_bytes=%h", checksum_bytes[2]));
            void'($value$plusargs("checksum3_start=%h", checksum_start[3]));
            void'($value$plusargs("checksum3_bytes=%h", checksum_bytes[3]));
        end

        x87_control_arg = 16'h037f; x87_status_arg = 0;
        x87_tag_arg = 16'hffff; x87_top_arg = 0;
        for (int i = 0; i < 8; i++) x87_fpr_arg[i] = 0;
        void'($value$plusargs("x87_control=%h", x87_control_arg));
        void'($value$plusargs("x87_status=%h", x87_status_arg));
        void'($value$plusargs("x87_tag=%h", x87_tag_arg));
        void'($value$plusargs("x87_top=%h", x87_top_arg));
        void'($value$plusargs("x87_fpr0=%h", x87_fpr_arg[0]));
        void'($value$plusargs("x87_fpr1=%h", x87_fpr_arg[1]));
        void'($value$plusargs("x87_fpr2=%h", x87_fpr_arg[2]));
        void'($value$plusargs("x87_fpr3=%h", x87_fpr_arg[3]));
        void'($value$plusargs("x87_fpr4=%h", x87_fpr_arg[4]));
        void'($value$plusargs("x87_fpr5=%h", x87_fpr_arg[5]));
        void'($value$plusargs("x87_fpr6=%h", x87_fpr_arg[6]));
        void'($value$plusargs("x87_fpr7=%h", x87_fpr_arg[7]));

        $display("[TB] Configuration:");
        $display("[TB]   mode=%s", start_protected ? "protected-start" : "real-start");
        $display("[TB]   EIP=0x%08X CR0=0x%08X CR3=0x%08X", eip_arg, cr0_arg, cr3_arg);
        $display("[TB]   selectors: CS=%04X DS=%04X SS=%04X ES=%04X FS=%04X GS=%04X",
                 init_cs[15:0], init_ds[15:0], init_ss[15:0], init_es[15:0], init_fs[15:0], init_gs[15:0]);
        $display("[TB]   CS: base=0x%08X limit=0x%05X flags=0x%04X", cs_base, cs_limit, cs_flags);
        $display("[TB]   DS: base=0x%08X limit=0x%05X flags=0x%04X", ds_base, ds_limit, ds_flags);
        $display("[TB]   SS: base=0x%08X limit=0x%05X flags=0x%04X", ss_base, ss_limit, ss_flags);

        // Reset sequence
        #20;
        reset_n = 0;
        #10;

        #50;
        reset_n = 1;
        #1;

        // Initialize CPU state immediately after reset release, before the next
        // clock edge. Write the compact hidden-descriptor bank directly.
        dut.CS = init_cs[15:0];
        dut.DS = init_ds[15:0];
        dut.SS = init_ss[15:0];
        dut.ES = init_es[15:0];
        dut.FS = init_fs[15:0];
        dut.GS = init_gs[15:0];
        dut.LDTR = init_ldtr;
        dut.TR = init_tr;

        dut.EIP = eip_arg;

        dut.seg_unit.desc_cache[1] = build_seg_desc(cs_base, cs_limit[19:0], cs_flags);
        dut.seg_unit.desc_cache[3] = build_seg_desc(ds_base, ds_limit[19:0], ds_flags);
        dut.seg_unit.desc_cache[2] = build_seg_desc(ss_base, ss_limit[19:0], ss_flags);
        dut.seg_unit.desc_cache[0] = build_seg_desc(es_base, es_limit[19:0], es_flags);
        dut.seg_unit.desc_cache[4] = build_seg_desc(fs_base, fs_limit[19:0], fs_flags);
        dut.seg_unit.desc_cache[5] = build_seg_desc(gs_base, gs_limit[19:0], gs_flags);
        dut.seg_unit.desc_cache[6] = build_seg_desc(tr_base, tr_limit, tr_flags);
        dut.seg_unit.desc_cache[7] = build_seg_desc(ldt_base, ldt_limit, ldt_flags);
        dut.seg_unit.gdt_base = init_gdt_base;
        dut.seg_unit.gdt_limit = init_gdt_limit;
        dut.seg_unit.idt_base = init_idt_base;
        dut.seg_unit.idt_limit = init_idt_limit;
        dut.seg_unit.desc_cache[1].D_B = d_init[0];

        // Snapshot restore supplies architectural linear state. Ordinary tests
        // retain their existing direct physical-code initialization.
        dut.prefetch_inst.pf_fetch_addr = snapshot_mode ?
                                           cs_base + eip_arg :
                                           code_phys_base + eip_arg;

        // General registers = 0
        dut.EAX = init_eax;
        dut.ECX = init_ecx;
        dut.EDX = init_edx;
        dut.EBX = init_ebx;
        dut.ESP = init_esp;
        dut.EBP = init_ebp;
        dut.ESI = init_esi;
        dut.EDI = init_edi;

        // Flags and control
        dut.EFLAGS = init_eflags;
        dut.CR0 = cr0_arg;
        dut.CR2 = cr2_arg;
        dut.CR3 = cr3_arg;

        if (snapshot_mode)
            -> load_snapshot_x87_state;

        $display("[TB] CPU reset complete, starting execution");
        $display("");
    end

    // Main test loop
    always @(posedge clk) begin
        if (reset_n) begin
            cycle <= cycle + 1;

            if (triple_fault_reset && $test$plusargs("expect_triple_fault")) begin
                $display("");
                $display("========================================");
                $display("  TEST PASSED! Triple-fault reset requested.");
                $display("  Total cycles: %0d", cycle);
                $display("========================================");
                #50;
                $finish;
            end

            // Count architectural issue transfers. RNI can remain asserted on
            // adjacent boundaries, so edge-counting it undercounts throughput.
            prev_instruction_boundary <= instruction_boundary;
            if (dut.i_issue)
                instruction_count <= instruction_count + 1;

            // EIP still names the matching instruction on i_issue.  Counting
            // occurrences permits stable loop/body boundaries as well as a
            // traditional first return-address stop.
            if (stop_eip_valid && dut.i_issue &&
                (dut.CS == stop_cs[15:0]) && (dut.EIP == stop_eip)) begin
                if ((stop_match_count + 1) < stop_occurrence) begin
                    stop_match_count <= stop_match_count + 1;
                end else begin
                    longint unsigned checksum [0:3];
                    for (int range_index = 0; range_index < checksum_count; range_index++) begin
                        checksum[range_index] = 64'hcbf29ce484222325;
                        for (int i = 0; i < checksum_bytes[range_index]; i++) begin
                            if ((checksum_start[range_index] + i) < MEM_SIZE) begin
                                checksum[range_index] = checksum[range_index] ^
                                                        mem[checksum_start[range_index] + i];
                                checksum[range_index] = checksum[range_index] *
                                                        64'h00000100000001b3;
                            end
                        end
                    end
                    $display("");
                    $display("========================================");
                    $display("  SNAPSHOT COMPLETE");
                    $display("  Stop occurrence: %0d", stop_occurrence);
                    $display("  Total cycles: %0d", cycle);
                    $display("  Total instructions: %0d", instruction_count);
                    $display("  CPI: %f", instruction_count ?
                             real'(cycle) / real'(instruction_count) : 0.0);
                    $display("  x87 commands: %0d", x87_command_count);
                    $display("  x87 direct loads: %0d", x87_direct_load_count);
                    $display("  x87 control busy cycles: %0d", x87_control_busy_cycles);
                    $display("  x87 executor busy cycles: %0d", x87_executor_busy_cycles);
                    $display("  x87 WAIT stall cycles: %0d", x87_wait_stall_cycles);
                    if ($test$plusargs("profile_x87")) begin
                    for (int i = 0; i < 2048; i++) begin
                        if (x87_fop_count[i] != 0)
                            $display("X87_FOP protocol %03x %0d", i, x87_fop_count[i]);
                        if (x87_direct_load_fop_count[i] != 0)
                            $display("X87_FOP direct-load %03x %0d", i,
                                     x87_direct_load_fop_count[i]);
                    end
                    for (int i = 0; i < 16; i++) begin
                        if (x87_exec_start_count[i] != 0)
                            $display("X87_EXEC %0d %0d %0d", i,
                                     x87_exec_start_count[i],
                                     x87_exec_busy_count[i]);
                    end
                end
                    if ($test$plusargs("profile_cpu")) begin
                    for (int i = 0; i < 4096; i++) begin
                        if (cpu_entry_count[i] != 0)
                            $display("CPU_ENTRY %03x %0d %0d %0d %0d %0d", i,
                                     cpu_entry_count[i], cpu_entry_cycles[i],
                                     cpu_entry_mem_stall[i], cpu_entry_x87_stall[i],
                                     cpu_entry_frontend_wait[i]);
                    end
                    for (int i = 0; i < 256; i++) begin
                        if (cpu_opcode_count[i] != 0)
                            $display("CPU_OPCODE %02x %0d %0d", i,
                                     cpu_opcode_count[i], cpu_opcode_cycles[i]);
                    end
                    for (int i = 0; i < 2; i++)
                        $display("CPU_HARDWIRED %0d %0d %0d", i,
                                 cpu_hardwired_count[i], cpu_hardwired_cycles[i]);
                    for (int i = 0; i < 8; i++) begin
                        if (cpu_commit_count[i] != 0)
                            $display("CPU_COMMIT %0d %0d %0d", i,
                                     cpu_commit_count[i], cpu_commit_cycles[i]);
                    end
                    for (int i = 0; i < 10; i++)
                        $display("CPU_STATE %0d %0d", i, cpu_stall_cycles[i]);
                    for (int i = 0; i < 12; i++)
                        $display("CPU_LOAD_CHAIN %0d %0d", i,
                                 cpu_load_chain[i]);
                    for (int i = 0; i < 17; i++)
                        $display("CPU_LOAD_INTERVAL %0d %0d", i,
                                 cpu_load_intervals[i]);
                    for (int i = 0; i < 8; i++) begin
                        $display("CPU_LOAD_ISSUE %0d %0d", i,
                                 cpu_load_issue[i]);
                        $display("CPU_LOAD_RESULT %0d %0d", i,
                                 cpu_load_result[i]);
                        $display("CPU_BRANCH_RESULT %0d %0d", i,
                                 cpu_branch_result[i]);
                        $display("CPU_X87_WAIT_REASON %0d %0d", i,
                                 cpu_x87_wait_reason[i]);
                    end
                    for (int i = 0; i < 2; i++)
                        for (int j = 0; j < 4; j++)
                            $display("CPU_LOAD_SUCCESSOR %0d %0d %0d", i, j,
                                     cpu_load_successor[i][j]);
                    for (int path = 0; path < 3; path++) begin
                        for (int interval = 0; interval < 17; interval++) begin
                            if (cpu_load_path_interval[path][interval] != 0)
                                $display("CPU_LOAD_PATH_INTERVAL %0d %0d %0d",
                                         path, interval,
                                         cpu_load_path_interval[path][interval]);
                        end
                        for (int dependency = 0; dependency < 4; dependency++) begin
                            if (cpu_load_path_successor_count[path][dependency] != 0)
                                $display("CPU_LOAD_PATH_SUCCESSOR %0d %0d %0d %0d",
                                         path, dependency,
                                         cpu_load_path_successor_count[path][dependency],
                                         cpu_load_path_successor_cycles[path][dependency]);
                        end
                        for (int entry = 0; entry < 4096; entry++) begin
                            if (cpu_load_next_entry_count[path][entry] != 0)
                                $display("CPU_LOAD_NEXT_ENTRY %0d %03x %0d %0d",
                                         path, entry,
                                         cpu_load_next_entry_count[path][entry],
                                         cpu_load_next_entry_cycles[path][entry]);
                        end
                        for (int addr32 = 0; addr32 < 2; addr32++)
                            for (int size = 0; size < 3; size++)
                                for (int ea_class = 0; ea_class < 3; ea_class++)
                                    for (int alignment = 0; alignment < 4;
                                         alignment++) begin
                                        if (cpu_load_form_count[path][addr32][size]
                                                               [ea_class][alignment] != 0)
                                            $display("CPU_LOAD_FORM %0d %0d %0d %0d %0d %0d %0d",
                                                path, addr32, size, ea_class,
                                                alignment,
                                                cpu_load_form_count[path][addr32][size]
                                                                   [ea_class][alignment],
                                                cpu_load_form_cycles[path][addr32][size]
                                                                    [ea_class][alignment]);
                                    end
                    end
                    for (int i = 0; i < 5; i++)
                        $display("CPU_REP_ELEMENTS %0d %0d", i,
                                 cpu_rep_elements[i]);
                    for (int i = 0; i < 17; i++)
                        $display("CPU_STORE_INTERVAL %0d %0d", i,
                                 cpu_store_intervals[i]);
                    for (int i = 0; i < 10; i++)
                        $display("CPU_STORE_STATE %0d %0d", i,
                                 cpu_store_states[i]);
                    for (int i = 0; i < 4096; i++) begin
                        if (cpu_store_next_entry[i] != 0)
                            $display("CPU_STORE_NEXT_ENTRY %03x %0d %0d %0d %0d",
                                     i, cpu_store_next_entry[i],
                                     cpu_store_next_entry_gap[i][1],
                                     cpu_store_next_entry_gap[i][2],
                                     cpu_store_next_entry_gap[i][3]);
                    end
                    for (int i = 0; i < 256; i++) begin
                        if (cpu_store_next_opcode[i] != 0)
                            $display("CPU_STORE_NEXT_OPCODE %02x %0d %0d %0d %0d",
                                     i, cpu_store_next_opcode[i],
                                     cpu_store_next_opcode_gap[i][1],
                                     cpu_store_next_opcode_gap[i][2],
                                     cpu_store_next_opcode_gap[i][3]);
                    end
                end
                    for (int range_index = 0; range_index < checksum_count; range_index++)
                        $display("  FNV64[%08x+%08x]: %016x",
                                 checksum_start[range_index], checksum_bytes[range_index],
                                 checksum[range_index]);
                    $display("  CS:EIP: %04X:%08X", dut.CS, dut.EIP);
                    $display("========================================");
                    #50;
                    $finish;
                end
            end

            // Test completed
            if (test_done) begin
                #50;
                $finish;
            end

            // Timeout
            if (cycle >= max_cycles) begin
                $display("");
                $display("========================================");
                $display("  TIMEOUT after %0d cycles", max_cycles);
                $display("  Test status: 0x%02X", test_status);
                $display("  Test data: 0x%08X", test_data);
                $display("  Instructions: %0d", instruction_count);
                $display("  CS:EIP: %04X:%08X", dut.CS, dut.EIP);
                $display("  HANGDUMP uc_active=%0d uc_addr=%03x stall=%0d stall_mem=%0d fds=%0d fsds=%0d i_rni_delay=%0d intr_pend=%0d intr_pin=%0d IF=%0d q_flush=%0d",
                         dut.uc_active, dut.uc_addr, dut.stall, dut.stall_mem, dut.fault_delivery_state,
                         dut.fault_suppress_delay_slot, dut.i_rni_delay, dut.interrupt_pending, intr,
                         dut.EFLAGS[9], dut.q_flush);
                $display("  HANGDUMP2 pf_suspended=%0d pf_fault_reported=%0d dec_fetch_blocked=%0d early_redirected=%0d pf_inflight=%0d",
                         dut.prefetch_inst.pf_suspended, dut.prefetch_inst.pf_fault_reported,
                         dut.decoder_fetch_blocked, dut.early_redirected, dut.prefetch_inst.pf_inflight);
                $display("========================================");
                $finish;
            end

            // Hardware interrupt signal generation
            if (intr_hardwired_load_arm && dut.uc_exec &&
                (dut.uc_addr == 12'h019) && dut.recipe_state.hardwired &&
                (dut.recipe_state.commit_sel == z486_pkg::RECIPE_COMMIT_MEM) &&
                dut.d2_vipt_candidate) begin
                intr_hardwired_load_arm <= 1'b0;
                intr <= 1'b1;
                if ($test$plusargs("trace_io"))
                    $display("[TB] INTR asserted during hardwired load at cycle %0d",
                             cycle);
            end

            if (intr_delay > 0) begin
                intr_delay <= intr_delay - 1;
                if (intr_delay == 1) begin
                    if (intr_request) begin
                        intr <= 1'b1;
                        intr_request <= 1'b0;
                        if ($test$plusargs("trace_io"))
                            $display("[TB] INTR asserted at cycle %0d", cycle);
                    end
                    if (nmi_request) begin
                        nmi <= 1'b1;
                        nmi_hold_count <= nmi_pulse_cycles;
                        nmi_request <= 1'b0;
                        if ($test$plusargs("trace_io"))
                            $display("[TB] NMI asserted at cycle %0d", cycle);
                    end
                end
            end

            // Deterministic trigger mode: count retired instructions.
            // This is useful for shadow-window tests (STI/MOV SS).
            if (instruction_boundary && !prev_instruction_boundary) begin
                if (intr_request && intr_instr_remaining > 0) begin
                    intr_instr_remaining <= intr_instr_remaining - 1;
                    if (intr_instr_remaining == 1) begin
                        intr <= 1'b1;
                        intr_request <= 1'b0;
                        if ($test$plusargs("trace_io"))
                            $display("[TB] INTR asserted at retired-instr=%0d (cycle %0d)",
                                     instruction_count + 1, cycle);
                    end
                end
                if (nmi_request && nmi_instr_remaining > 0) begin
                    nmi_instr_remaining <= nmi_instr_remaining - 1;
                    if (nmi_instr_remaining == 1) begin
                        nmi <= 1'b1;
                        nmi_hold_count <= nmi_pulse_cycles;
                        nmi_request <= 1'b0;
                        if ($test$plusargs("trace_io"))
                            $display("[TB] NMI asserted at retired-instr=%0d (cycle %0d)",
                                     instruction_count + 1, cycle);
                    end
                end
            end
            if (nmi_hold_count > 0) begin
                nmi_hold_count <= nmi_hold_count - 1;
                if (nmi_hold_count == 1)
                    nmi <= 1'b0;
            end

            // Check for HLT (can be disabled for HLT-wakeup interrupt tests)
            if (stop_on_hlt && dut.i.opcode == 8'hF4 && instruction_boundary && !prev_instruction_boundary) begin
                $display("");
                $display("========================================");
                $display("  HLT EXECUTED");
                $display("  Test status: 0x%02X", test_status);
                $display("  Test data: 0x%08X", test_data);
                $display("  Cycle: %0d", cycle);
                $display("  CS:EIP: %04X:%08X", dut.CS, dut.EIP);
                $display("========================================");
                #100;
                $finish;
            end

            // Optional progress
            if ($test$plusargs("progress") && (cycle % 1000000 == 0))
                $display("Progress: cycle=%0d instr=%0d", cycle, instruction_count);

            // Trace instructions
            if ($test$plusargs("trace_instr") && instruction_boundary && !prev_instruction_boundary)
                $display("INSTR[%0d]: CS:EIP=%04X:%08X IR=%02X EAX=%08X",
                         instruction_count, dut.CS, dut.EIP, dut.i.opcode, dut.EAX);

            // Trace microcode execution
            if ($test$plusargs("trace_ucode") && !dut.stall)
                $display("UCODE pc=%03h dest=%02h src=%02h bus=%02h aluop=%02h SIGMA=%08h ESP=%08h IND=%08h CS=%04h wdata=%08h",
                         dut.uc_addr, dut.uc_dest, dut.uc_source, dut.uc_buscode, dut.uc_aluop,
                         dut.SIGMA, dut.ESP, dut.IND, dut.CS, dut.mem_wdata);

            // Trace paging
            if ($test$plusargs("trace_paging") && dut.mem_req_upcoming)
                $display("PAGING: linear=%08X servicing=%0d", dut.ind_linear, dut.mem_servicing);
        end
    end

endmodule
