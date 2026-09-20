// Z486 Instruction Decoder

`include "z486_platform.svh"
module decoder
    import z486_pkg::*;
(
    input               clk,
    input               reset_n,

    // Prefetch queue interface (two-cursor protocol)
    input        [63:0] win_d1,         // registered raw window at the D1 cursor
    input        [63:0] win_d1_early,   // speculative next window for entry-ROM preread
    input        [5:0]  d1_avail,       // bytes fetched beyond the D1 cursor
    output       [3:0]  d1_adv,         // D1 cursor advance (0-11 bytes: a prefix,
                                        //   or the whole rest of the instruction
                                        //   at handoff - struct AND literal bytes)
    output       [3:0]  d1_preread_adv, // structural advance without D2 backpressure
    input        [31:0] win_lit,        // 4 bytes at pop_cursor + lit_off
    input        [5:0]  lit_avail,      // bytes fetched beyond that point
    output       [4:0]  lit_off,        // literal offset from the pop cursor
    output              pop_now,        // instruction completed D2: pop its bytes
    output       [4:0]  pop_len,        //   (registered length from the skeleton)

    // Mode signals
    input               D,              // Default operand/address size (CS.D bit)
    input               pe_enable,      // Protected mode enable (CR0.PE)

    // Control signals
    input               q_flush,        // Flush decoder on branch
    input               i_issue,        // D2 transfers into EX

    // Decoded instruction output
    output dec_entry_t  i_bus,          // Decoded instruction
    output              decq_empty,     // Legacy name: unified D2 has no skeleton
    output dec_entry_t  i_bus2,         // Registered D1 skid successor
    output              decq_has2,      // skid successor can complete D2 in one cycle
    output              decq_has_jmp_call, // JMP/CALL rel in flight (halt speculative prefetch)

    // Unified D2 payload and empty-pipe D1 launch
    output dec_entry_t  d2_entry,       // instruction completing D2
    output              d2_push,        // d2_entry is complete this cycle
    output              d1_issue_direct,// handoff has no older D2 skeleton
    output       [11:0] d1_issue_entry_point,
    output dec_entry_t  d1_issue_entry, // live structural entry for empty-pipe launch
    output              fetch_blocked   // decoder is waiting for unfetched bytes
);

typedef enum logic [1:0] {
    LIT_NONE = 2'd0,
    LIT_IMM  = 2'd1,
    LIT_DISP = 2'd2
} lit_kind_t;

// The skeleton: a structurally-complete entry plus the literal plan.
// Literal fields (immediate/displacement) and the entry point are pending.
typedef struct packed {
    dec_entry_t entry;
    lit_kind_t  lit1_kind;
    lit_kind_t  lit2_kind;
    logic [2:0] lit1_size;
    logic [2:0] lit2_size;
    logic       lit1_sign_extend;
    logic       lit2_sign_extend;
    logic       lit1_mirror_disp;
    logic       lit2_mirror_disp;
    logic       fields_fit;          // both literal fields fit the first D2 window
    logic       need_sib;
    logic [2:0] pending_imm_size;
    logic       pending_imm_sign_extend;
} decoder_work_t;

`include "pla_control.svh"
`include "pla_entry.svh"

//=============================================================================
// D1 - structural decode
//=============================================================================

// Prefix state is not part of dec_entry_t until a non-prefix opcode is decoded.
logic        prefix_66;
logic        prefix_67;
logic        prefix_0f;
logic        prefix_rep;
logic [3:0]  prefix_count;
logic [1:0]  prefix_rep_lock;
logic [2:0]  prefix_seg;
reg         code32_r;     // Local CS.D copy; frontend flush hides its one-cycle update latency.

reg            d1_sib;      // SIB sub-cycle pending (cursor held)
decoder_work_t pend_work;   // struct_work parked across the SIB sub-cycle
decoder_work_t skid;        // one registered D1 successor ahead of D2
reg            skid_v;
reg            skel_v;
reg [4:0]      skid_lit_off;
reg            skid_one_d2;
reg [31:0]     skid_raw_lo_r;
reg [31:0]     skid_raw_hi_r;
reg [1:0]      skid_raw_lit_off_r;
reg            skid_raw_valid_r;

// Byte aliases at the D1 cursor.  The cursor does not advance until the
// skeleton handoff, so during the SIB sub-cycle the sib byte is byte 2.
wire [7:0] opcode = win_d1[7:0];
wire [7:0] modrm  = win_d1[15:8];
wire [7:0] sib_b  = win_d1[23:16];
wire       data32 = code32_r ^ prefix_66;
wire       addr32 = code32_r ^ prefix_67;
wire [15:0] entry_rom_sel;

wire consume_prefix = !d1_sib && !prefix_0f && is_prefix(opcode) &&
                      (d1_avail >= 6'd1);
wire consume_0f     = !d1_sib && !prefix_0f && (opcode == 8'h0f) &&
                      (d1_avail >= 6'd1);

decoder_work_t struct_work;
logic [2:0]    struct_len;
// Some simulators do not include signals referenced only inside a task in an
// always_comb sensitivity set. Pass the task's live decode inputs explicitly
// so structural decode is reevaluated when the D1 window advances.
wire [45:0] struct_work_inputs = {
    opcode, modrm, prefix_0f, prefix_rep, prefix_count, data32, addr32,
    prefix_rep_lock, prefix_seg, entry_rom_sel, pe_enable
};
always_comb build_struct_work(struct_work_inputs, struct_work, struct_len);

wire struct_bytes_ok = !d1_sib && (d1_avail >= {3'b000, struct_len});
wire sib_bytes_ok    = d1_sib && (d1_avail >= 6'd3);   // opcode+modrm+sib in view

// skel_free: d2_done's terms are lit_avail / out_full / phase - all
// register-derived, never i_issue (L1) - so the same-edge free is legal.
wire d2_done;
wire d1_slot_ready = !skid_v || i_issue;

wire d1_to_sib  = !consume_prefix && !consume_0f &&
                  struct_bytes_ok && struct_work.need_sib;
wire d1_preread_handoff = !consume_prefix && !consume_0f &&
    ((struct_bytes_ok && !struct_work.need_sib) || sib_bytes_ok);
wire d1_handoff = !consume_prefix && !consume_0f && d1_slot_ready &&
                  ((struct_bytes_ok && !struct_work.need_sib) || sib_bytes_ok);

// Preread the macro-entry PLA from M10K one cycle before D1 handoff. The
// speculative cursor deliberately ignores D2 backpressure; when D1 cannot
// advance, retain the entry corresponding to its current window. This removes
// both the combinational PLA and D2/VIPT readiness from the ucode-ROM address.
logic prefix_0f_early, prefix_rep_early;
always_comb begin
    prefix_0f_early = prefix_0f;
    prefix_rep_early = prefix_rep;
    if (q_flush) begin
        prefix_0f_early = 1'b0;
        prefix_rep_early = 1'b0;
    end else begin
        if (consume_0f)
            prefix_0f_early = 1'b1;
        else if (consume_prefix && ((opcode == 8'hf2) || (opcode == 8'hf3)))
            prefix_rep_early = 1'b1;
        if (d1_preread_handoff) begin
            prefix_0f_early = 1'b0;
            prefix_rep_early = 1'b0;
        end
    end
end

wire [9:0] entry_rom_addr = {win_d1_early[7:0],
                             prefix_rep_early, prefix_0f_early};
`Z486_BLOCK_RAM reg [63:0] entry_rom [0:1023];
initial $readmemh("pla_entry_rom.hex", entry_rom);
reg [63:0] entry_rom_q;
always_ff @(posedge clk)
    entry_rom_q <= entry_rom[entry_rom_addr];

reg [63:0] entry_rom_hold_r;
reg        entry_rom_use_q_r;
always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        entry_rom_hold_r <= 64'd0;
        entry_rom_use_q_r <= 1'b0;
    end else begin
        if (entry_rom_use_q_r)
            entry_rom_hold_r <= entry_rom_q;
        entry_rom_use_q_r <= q_flush || (d1_adv == d1_preread_adv);
    end
end
wire [63:0] entry_rom_current = entry_rom_use_q_r
                              ? entry_rom_q : entry_rom_hold_r;
assign entry_rom_sel = entry_rom_current[{data32, pe_enable}*16 +: 16];

// The first-level group code already encodes data-size and opcode-map mode.
// Keep the ModR/M-dependent second-level entry table in an asynchronous ROM so
// Quartus can constant-fold it as one compact lookup rather than retaining the
// original deep priority PLA on the D1 -> microcode-address cone.
`Z486_DISTRIBUTED_RAM reg [15:0] group_entry_rom [0:1023];
initial $readmemh("pla_group_entry.hex", group_entry_rom);

decoder_work_t handoff_work;
always_comb begin
    handoff_work = d1_sib ? capture_sib(pend_work, sib_b) : struct_work;
end
wire [11:0] handoff_entry_point = recipe_effective_entry(
    handoff_work.entry.entry_point,
    handoff_work.entry.opcode,
    handoff_work.entry.modrm);
decoder_work_t handoff_d2;
always_comb begin
    handoff_d2 = handoff_work;
    handoff_d2.entry.entry_point = handoff_entry_point;
    handoff_d2.entry.ucode_action = recipe_action(handoff_entry_point);
    // Resolve the direct ALU memory-load class in D1. Re-decoding opcode and
    // ModR/M from i_bus put this classification in series with VIPT hit
    // resolution and same-edge chained issue on the microcode-ROM address path.
    handoff_d2.entry.vipt_alu = !handoff_work.entry.has_0f &&
        (handoff_work.entry.opcode[7:6] == 2'b00) &&
        !handoff_work.entry.opcode[2] && handoff_work.entry.opcode[1] &&
        handoff_work.entry.has_modrm &&
        (handoff_work.entry.modrm[7:6] != 2'b11) &&
        (handoff_work.entry.opcode[5:3] != 3'b111);
    // The recipe hazard mask is derived from this registered structural entry
    // during D2. Keeping it out of D1 avoids serializing entry-PLA decode and
    // mask generation on the raw-window-to-skeleton path.
    handoff_d2.entry.recipe_gpr_read_mask = 8'h00;
    {handoff_d2.entry.ea_index_onehot, handoff_d2.entry.ea_base_onehot} =
        dec_ea_onehots(handoff_work.entry);
    // Match the i486 complex-EA rule: a displacement on an address that also
    // reads base and index requires a second D2 addition cycle. POP r/m may
    // add a synthetic post-pop ESP displacement even when none was encoded.
    handoff_d2.entry.ea_complex =
        (|handoff_d2.entry.ea_base_onehot) &&
        (|handoff_d2.entry.ea_index_onehot) &&
        ((handoff_work.lit1_kind == LIT_DISP) ||
         (handoff_work.lit2_kind == LIT_DISP) ||
         handoff_work.entry.ea_uses_post_pop_esp);
    handoff_d2.entry.mem_seg = handoff_work.entry.stack_op ? SEG_SS :
        apply_seg_override_type(
            calc_default_seg_type(handoff_work.entry.modrm,
                                  handoff_work.entry.sib,
                                  handoff_work.entry.has_sib,
                                  handoff_work.entry.addr32),
            handoff_work.entry.seg);
    handoff_d2.fields_fit = (handoff_work.lit2_kind == LIT_NONE) ||
        ({1'b0, handoff_work.lit1_size} + {1'b0, handoff_work.lit2_size} <= 4'd4);
end
// First literal byte, as an offset from the POP cursor (prefix bytes are
// not popped until D2 completes): total length minus the literal bytes.
wire [4:0] handoff_lit_off = handoff_work.entry.length -
                             ({2'b00, handoff_work.lit1_size} +
                              {2'b00, handoff_work.lit2_size});
// D1 cursor advance at handoff: the whole REST of the instruction (struct
// bytes AND the literal bytes D2 will capture), so the cursor lands on the
// next instruction's first byte.  It may pass not-yet-fetched literal
// bytes; d1_avail clamps to 0 in that case and D1 waits for the fill.
wire [4:0] handoff_adv = handoff_work.entry.length -
                         {1'b0, prefix_count} - (prefix_0f ? 5'd1 : 5'd0);
// Prefix and 0F bytes have already advanced the D1 cursor. Relative to the
// registered raw D1 window, the first literal follows only opcode/ModR/M/SIB.
wire [4:0] handoff_raw_lit_off_w = handoff_lit_off -
                                    {1'b0, prefix_count} -
                                    (prefix_0f ? 5'd1 : 5'd0);
wire [1:0] handoff_raw_lit_off = handoff_raw_lit_off_w[1:0];

// The entry PLA result is valid during D1 handoff, one cycle before the
// registered skeleton completes D2. It directly launches an empty D2 pipe.
wire [3:0] handoff_lit_size = {1'b0, handoff_work.lit1_size} +
                              {1'b0, handoff_work.lit2_size};
wire handoff_one_d2 = (handoff_work.lit2_kind == LIT_NONE ||
                       handoff_lit_size <= 4'd4) &&
                      (d1_avail >= {1'b0, handoff_adv});
wire [3:0] handoff_raw_need = (handoff_work.lit2_kind == LIT_NONE ||
                               handoff_lit_size <= 4'd4)
                            ? handoff_lit_size
                            : {1'b0, handoff_work.lit1_size};
wire [5:0] handoff_raw_end = {1'b0, handoff_raw_lit_off_w} +
                             {2'b00, handoff_raw_need};
wire handoff_raw_valid = (d1_avail >= handoff_raw_end);
// Launch the synchronous ROM as soon as structural decode owns an empty D2.
// D2 holds the returned word while late or second literals are captured.
assign d1_issue_direct = d1_handoff && !skel_v;
assign d1_issue_entry_point = handoff_entry_point;
always_comb begin
    d1_issue_entry = handoff_d2.entry;
end

// TIMING: no !q_flush in the advance/pop legs. q_flush carries the deep uc_exec/mem-block cone
assign d1_adv = (consume_prefix || consume_0f) ? 4'd1 :
                d1_handoff ? handoff_adv[3:0] :
                4'd0;
assign d1_preread_adv = (consume_prefix || consume_0f) ? 4'd1 :
                        d1_preread_handoff ? handoff_adv[3:0] :
                        4'd0;

//=============================================================================
// The skeleton register (D1 -> D2 pipeline boundary; Step 2's lookahead)
//=============================================================================

decoder_work_t skel;
reg [4:0]      skel_lit_off;
reg [31:0]     skel_raw_hi_r;    // upper half of the opcode-relative D1 window
reg [1:0]      skel_raw_lit_off_r;
reg [31:0]     skel_lit_r;       // shared low raw half or pop-relative literal
reg            skel_window_valid_r;
reg            skel_window_raw_r; // skel_lit_r is the low half of the D1 window
wire           head_v = skel_v;  // temporary trace compatibility alias

// Register the raw literal window on the D1 handoff edge. The skid keeps its
// own copy so promotion never has to route i_issue through the live prefetch
// aligner and back into the D2 literal register.
wire promote_skid_capture = i_issue && skid_v;
wire promote_handoff_capture = d1_handoff &&
                                ((!skel_v && !i_issue) || (i_issue && !skid_v));
wire incoming_capture = promote_skid_capture || promote_handoff_capture;

always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        code32_r <= 1'b0;
        skel_v <= 1'b0;
        skid_v <= 1'b0;
        d1_sib <= 1'b0;
        prefix_66 <= 1'b0;
        prefix_67 <= 1'b0;
        prefix_0f <= 1'b0;
        prefix_rep <= 1'b0;
        prefix_count <= 4'd0;
        prefix_rep_lock <= PREFIX_NOREPLOCK;
        prefix_seg <= PREFIX_NOSEG;
        skid_raw_lo_r <= 32'd0;
        skid_raw_hi_r <= 32'd0;
        skid_raw_lit_off_r <= 2'd0;
        skid_raw_valid_r <= 1'b0;
    end else if (q_flush) begin
        code32_r <= D;
        skel_v <= 1'b0;
        skid_v <= 1'b0;
        d1_sib <= 1'b0;
        prefix_66 <= 1'b0;
        prefix_67 <= 1'b0;
        prefix_0f <= 1'b0;
        prefix_rep <= 1'b0;
        prefix_count <= 4'd0;
        prefix_rep_lock <= PREFIX_NOREPLOCK;
        prefix_seg <= PREFIX_NOSEG;
    end else begin
        code32_r <= D;
        if (consume_prefix || consume_0f) begin
            if (consume_0f) begin
                prefix_0f <= 1'b1;
            end else begin
                prefix_count <= prefix_count + 4'd1;
                unique case (opcode)
                    8'h66: prefix_66 <= 1'b1;
                    8'h67: prefix_67 <= 1'b1;
                    8'hf0: prefix_rep_lock <= PREFIX_LOCK;
                    8'hf2: begin
                        prefix_rep_lock <= PREFIX_REPNE;
                        prefix_rep <= 1'b1;
                    end
                    8'hf3: begin
                        prefix_rep_lock <= PREFIX_REP;
                        prefix_rep <= 1'b1;
                    end
                    8'h26, 8'h2e, 8'h36, 8'h3e, 8'h64, 8'h65:
                        prefix_seg <= prefix_seg_code(opcode);
                    default: ;
                endcase
            end
        end

        if (d1_to_sib) begin
            pend_work <= struct_work;
            d1_sib <= 1'b1;
        end

        if (d1_handoff) begin
            d1_sib <= 1'b0;
            skid_raw_lo_r <= win_d1[31:0];
            skid_raw_hi_r <= win_d1[63:32];
            skid_raw_lit_off_r <= handoff_raw_lit_off;
            skid_raw_valid_r <= handoff_raw_valid;
            // The skeleton carries the prefix state; clear for the next one.
            prefix_66 <= 1'b0;
            prefix_67 <= 1'b0;
            prefix_0f <= 1'b0;
            prefix_rep <= 1'b0;
            prefix_count <= 4'd0;
            prefix_rep_lock <= PREFIX_NOREPLOCK;
            prefix_seg <= PREFIX_NOSEG;
        end

        // The skid holds one structurally decoded successor while D2 owns the
        // current instruction. On issue it promotes from registered state,
        // keeping the full D1 decoder out of the ROM-address path.
        if (i_issue) begin
            if (skid_v) begin
                skel <= skid;
                skel_v <= 1'b1;
                skel_lit_off <= skid_lit_off;
                if (d1_handoff) begin
                    skid <= handoff_d2;
                    skid_v <= 1'b1;
                    skid_lit_off <= handoff_lit_off;
                    skid_one_d2 <= handoff_one_d2;
                end else begin
                    skid_v <= 1'b0;
                end
            end else if (d1_handoff) begin
                skel <= handoff_d2;
                skel_v <= 1'b1;
                skel_lit_off <= handoff_lit_off;
            end else begin
                skel_v <= 1'b0;
            end
        end else if (d1_handoff) begin
            if (!skel_v) begin
                skel <= handoff_d2;
                skel_v <= 1'b1;
                skel_lit_off <= handoff_lit_off;
            end else begin
                skid <= handoff_d2;
                skid_v <= 1'b1;
                skid_lit_off <= handoff_lit_off;
                skid_one_d2 <= handoff_one_d2;
            end
        end
    end
end

//=============================================================================
// D2 - literal capture + entry resolution
//=============================================================================

// Phase A = fields as handed off; phase B = field A captured (workB holds
// the merged entry), literal window advanced by lit1_size.
reg            d2_phaseB;
decoder_work_t workB;

wire [2:0] sizeA    = skel.lit1_size;
wire [2:0] sizeB    = skel.lit2_size;
wire       has_litA = (skel.lit1_kind != LIT_NONE);
wire       has_litB = (skel.lit2_kind != LIT_NONE);
wire [3:0] sizeAB   = {1'b0, sizeA} + {1'b0, sizeB};

// Once field A is in the registered window, the live literal port is free to
// preread field B while D2 interprets A.
wire d2_window_valid = skel_window_valid_r;
wire prefetch_fieldB = skel_v && d2_window_valid && !d2_phaseB &&
                       has_litB && !skel.fields_fit;
wire [4:0] resident_lit_off = skel_lit_off +
                              ((d2_phaseB || prefetch_fieldB)
                                  ? {2'b00, sizeA} : 5'd0);
assign lit_off = resident_lit_off;

// D1 carries raw bytes, not parsed literals. D2 selects the literal start from
// the registered window; the second field uses the live literal port only when
// both fields do not fit in this 32-bit view.
decoder_work_t capA;
decoder_work_t capAB;
decoder_work_t capB;
wire [31:0] skel_raw_lit_win =
    skel_raw_lit_off_r == 2'd0 ? skel_lit_r :
    skel_raw_lit_off_r == 2'd1 ? {skel_raw_hi_r[7:0],  skel_lit_r[31:8]}  :
    skel_raw_lit_off_r == 2'd2 ? {skel_raw_hi_r[15:0], skel_lit_r[31:16]} :
                                 {skel_raw_hi_r[23:0], skel_lit_r[31:24]};
wire [31:0] skel_lit_win = skel_window_raw_r ? skel_raw_lit_win : skel_lit_r;
wire [31:0] winB = (sizeA == 3'd1) ? { 8'h00, skel_lit_win[31:8]}  :
                   (sizeA == 3'd2) ? {16'h0,  skel_lit_win[31:16]} :
                   (sizeA == 3'd3) ? {24'h0,  skel_lit_win[31:24]} : 32'h0;
always_comb begin
    capA  = capture_literal(skel, skel_lit_win,
                            skel.lit1_kind, skel.lit1_size,
                            skel.lit1_sign_extend, skel.lit1_mirror_disp);
    capAB = capture_literal(capA, winB, capA.lit2_kind, capA.lit2_size,
                            capA.lit2_sign_extend, capA.lit2_mirror_disp);
    capB  = capture_literal(workB, skel_lit_win,
                            workB.lit2_kind, workB.lit2_size,
                            workB.lit2_sign_extend, workB.lit2_mirror_disp);
end

wire       finishing = d2_phaseB || skel.fields_fit;
wire [3:0] need_now  = d2_phaseB ? {1'b0, sizeB} :
                       !has_litA ? 4'd0 :
                       skel.fields_fit ? sizeAB : {1'b0, sizeA};
wire       bytes_ok  = (lit_avail >= {2'b00, need_now});
wire       d2_can    = skel_v && d2_window_valid;
wire       d2_complete = d2_can && finishing;
assign     d2_done   = i_issue;
// Field A may capture while the output side is full (no push happens here).
wire       d2_stepA  = d2_can && !finishing;
wire       d2_late_capture = skel_v && !d2_window_valid && bytes_ok;

// A retained prefetch fault becomes architectural only when one of the decode
// stages is genuinely blocked on bytes, not while the queue can still run or
// D1 is merely backpressured by its skid slot. D1 is younger than a resident
// D2 skeleton: its missing bytes must not fault on the same retirement edge
// that issues that older instruction into EX. Otherwise the registered fetch
// fault cancels the new EX instruction after EIP has already advanced past it.
// A resident D2 which itself needs missing literal bytes must still fault.
wire d1_bytes_blocked = !skid_v &&
                        (d1_sib ? !sib_bytes_ok :
                         (!consume_prefix && !consume_0f && !struct_bytes_ok));
wire d2_bytes_blocked = skel_v && !d2_window_valid && !bytes_ok;
assign fetch_blocked = (!skel_v && d1_bytes_blocked) || d2_bytes_blocked;

decoder_work_t d2_final;
always_comb begin
    d2_final = d2_phaseB ? capB :
               !has_litA ? skel :
               !has_litB ? capA :
               skel.fields_fit ? capAB : capA;
end

always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        d2_phaseB <= 1'b0;
    end else if (q_flush) begin
        d2_phaseB <= 1'b0;
    end else if (incoming_capture) begin
        d2_phaseB <= 1'b0;
    end else if (d2_done) begin
        d2_phaseB <= 1'b0;
    end else if (d2_stepA) begin
        workB <= capA;
        d2_phaseB <= 1'b1;
    end
end

always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        skel_raw_hi_r <= 32'h0;
        skel_raw_lit_off_r <= 2'd0;
        skel_lit_r <= 32'h0;
        skel_window_valid_r <= 1'b0;
        skel_window_raw_r <= 1'b0;
    end else if (q_flush) begin
        skel_window_valid_r <= 1'b0;
    end else if (i_issue && skid_v) begin
        skel_lit_r <= skid_raw_lo_r;
        skel_raw_hi_r <= skid_raw_hi_r;
        skel_raw_lit_off_r <= skid_raw_lit_off_r;
        skel_window_valid_r <= skid_raw_valid_r;
        skel_window_raw_r <= 1'b1;
    end else if (promote_handoff_capture) begin
        skel_lit_r <= win_d1[31:0];
        skel_raw_hi_r <= win_d1[63:32];
        skel_raw_lit_off_r <= handoff_raw_lit_off;
        skel_window_valid_r <= handoff_raw_valid;
        skel_window_raw_r <= 1'b1;
    end else if (d2_stepA) begin
        skel_lit_r <= win_lit;
        skel_window_valid_r <= (lit_avail >= {3'b000, sizeB});
        skel_window_raw_r <= 1'b0;
    end else if (d2_late_capture) begin
        skel_lit_r <= win_lit;
        skel_window_valid_r <= 1'b1;
        skel_window_raw_r <= 1'b0;
    end else if (d2_done) begin
        skel_window_valid_r <= 1'b0;
    end
end

// Literal capture does not change structural EA fields; use the selectors and
// segment index registered at the D1 handoff directly.
dec_entry_t push_entry;
always_comb begin
    // Literal capture changes only these two fields. Keep the full structural
    // entry out of the phase-B mux so its EA and recipe controls remain direct
    // registered skeleton outputs.
    push_entry = skel.entry;
    push_entry.immediate = d2_final.entry.immediate;
    push_entry.displacement = d2_final.entry.displacement;
    // Literals do not affect recipe register dependencies. Use the registered
    // D1 skeleton so a second literal phase cannot enter this hazard path.
    push_entry.recipe_gpr_read_mask = recipe_gpr_read_mask(skel.entry);
end

dec_entry_t skid_entry;
always_comb begin
    skid_entry = skid.entry;
    skid_entry.recipe_gpr_read_mask = recipe_gpr_read_mask(skid.entry);
end

assign d2_entry = push_entry;
assign d2_push  = d2_complete;
assign i_bus = push_entry;
assign decq_empty = !skel_v;
assign i_bus2 = skid_entry;
assign decq_has2 = skel_v && skid_v && skid_one_d2;

assign pop_now = d2_done;
assign pop_len = skel.entry.length;

// synthesis translate_off
// Focused recipe legality checks. The generated inventory is authoritative;
// these assertions verify that its early role agrees with its hazard metadata.
wire [2:0] push_recipe_early = recipe_early_kind(push_entry.entry_point);
recipe_meta_t push_recipe_fc;
assign push_recipe_fc = recipe_metadata(push_entry);
reg recipe_cov_en = 1'b0;
initial recipe_cov_en = $test$plusargs("recipe_cov");
always @(posedge clk) begin
    if (reset_n && d2_done && push_recipe_fc.hardwired) begin
        if (push_recipe_early == RECIPE_EARLY_SEQ)
            $fatal(1, "hardwired recipe missing: entry=%03x opcode=%02x modrm=%02x 0f=%b",
                   push_entry.entry_point, push_entry.opcode, push_entry.modrm,
                   push_entry.has_0f);
        unique case (push_recipe_early)
            RECIPE_EARLY_NONE:
                if (push_recipe_fc.uses_ea || push_recipe_fc.br_rel)
                    $fatal(1, "hardwired recipe NONE role mismatch: entry=%03x fc=%04x",
                           push_entry.entry_point, push_recipe_fc);
            RECIPE_EARLY_EA, RECIPE_EARLY_LOAD, RECIPE_EARLY_STORE,
            RECIPE_EARLY_RMW, RECIPE_EARLY_STACK:
                if (!push_recipe_fc.uses_ea)
                    $fatal(1, "hardwired recipe address role mismatch: entry=%03x kind=%0d fc=%04x",
                           push_entry.entry_point, push_recipe_early, push_recipe_fc);
            RECIPE_EARLY_BRANCH:
                if (!push_recipe_fc.br_rel)
                    $fatal(1, "hardwired recipe branch role mismatch: entry=%03x fc=%04x",
                           push_entry.entry_point, push_recipe_fc);
            default:
                $fatal(1, "hardwired recipe invalid early kind: entry=%03x kind=%0d",
                       push_entry.entry_point, push_recipe_early);
        endcase
        if (recipe_cov_en)
            $display("RECIPE_COV entry=%03x kind=%0d opcode=%02x modrm=%02x fc=%04x",
                     push_entry.entry_point, push_recipe_early,
                     push_entry.opcode, push_entry.modrm, push_recipe_fc);
    end
end

reg D1_EVT = 1'b0;
initial if ($test$plusargs("pf_cur")) D1_EVT = 1'b1;
always @(posedge clk) begin
    if (D1_EVT && d1_handoff)
        $display("%0t D1HO op=%02x modrm=%02x len=%0d adv=%0d(sib=%b slen=%0d) litoff=%0d A=%0d/%0d B=%0d/%0d pcnt=%0d 0f=%b",
                 $time, handoff_work.entry.opcode, handoff_work.entry.modrm,
                 handoff_work.entry.length, d1_adv, d1_sib, struct_len,
                 handoff_lit_off, handoff_work.lit1_kind, handoff_work.lit1_size,
                 handoff_work.lit2_kind, handoff_work.lit2_size,
                 prefix_count, prefix_0f);
    if (D1_EVT && d2_done)
        $display("%0t D2DN op=%02x len=%0d imm=%08x disp=%08x phB=%b",
                 $time, push_entry.opcode, push_entry.length,
                 push_entry.immediate, push_entry.displacement, d2_phaseB);
end
always @(posedge clk) begin
    if (reset_n && d1_handoff && (handoff_raw_lit_off_w[4:2] != 3'b000))
        $fatal(1, "D1: raw literal offset exceeds opcode window: off=%0d op=%02x",
               handoff_raw_lit_off_w, handoff_work.entry.opcode);
    // The literal plan must tile the instruction exactly: pop_len bytes =
    // struct bytes (lit_off) + literal bytes.
    if (reset_n && skel_v &&
        ({1'b0, skel.entry.length} !==
         {1'b0, skel_lit_off} + {3'b000, skel.lit1_size} + {3'b000, skel.lit2_size}))
        $fatal(1, "D2: literal plan does not tile: len=%0d lit_off=%0d A=%0d B=%0d",
               skel.entry.length, skel_lit_off, skel.lit1_size, skel.lit2_size);
    if (reset_n && d2_phaseB && !skel_v)
        $fatal(1, "D2: phase B with no skeleton");
end
// synthesis translate_on

// JMP/CALL rel is unconditionally taken, so its speculative fall-through prefetch is always wasted.
function automatic logic entry_jmp_call(input dec_entry_t e);
    entry_jmp_call = (e.rel_branch_kind == REL_BRANCH_JMP) ||
                     (e.rel_branch_kind == REL_BRANCH_CALL);
endfunction

assign decq_has_jmp_call =
    (d2_push && entry_jmp_call(push_entry)) ||
    (skid_v && skid_one_d2 && entry_jmp_call(skid.entry)) ||
    (d1_handoff && handoff_one_d2 && entry_jmp_call(d1_issue_entry));

//=============================================================================
// Structural Decode
//=============================================================================

task automatic build_struct_work(
    input logic [45:0]    sensitivity_inputs,
    output decoder_work_t w,
    output logic [2:0]    s_len
);
    logic [11:0] ctl_bits;
    logic [15:0] entry_first;
    logic [15:0] entry_final;
    logic [6:0]  group_dec;
    logic        entry_group;
    logic [5:0]  group_code;
    logic        has_modrm;
    logic        has_sib;
    logic [2:0]  disp_size;
    logic [2:0]  imm_total_size;
    logic [2:0]  imm_first_size;
    logic        imm_sign_extend;
    logic        invalid_lock;
    logic        instr_bswap;
    logic        is_setcc;
    logic        is_movzx_movsx;
    logic        is_movzx_word;
    logic        is_xlat;
    logic        is_byte_operand;
    begin
        w = '0;
        s_len = 3'd1;
        ctl_bits = pla_control_opcode_lookup(prefix_0f, opcode);

        w.entry.opcode = opcode;
        w.entry.has_0f = prefix_0f;
        w.entry.has_rep = prefix_rep;
        w.entry.prefix_count = prefix_count;
        w.entry.data32 = data32;
        w.entry.addr32 = addr32;
        w.entry.rep_lock = prefix_rep_lock;
        w.entry.seg = prefix_seg;
        w.entry.has_d_bit = ctl_bits[11] & ~ctl_bits[10] & ctl_bits[9] &
                             ctl_bits[8] & ctl_bits[2] & ctl_bits[1];
        w.entry.has_embedded_register = ~ctl_bits[2];
        w.entry.has_w_bit = ctl_bits[1];
        w.entry.is_pushpop_seg = ctl_bits[3];
        w.entry.update_flags = ctl_bits[4];

        has_modrm = ctl_bits[2] & ~ctl_bits[0];
        imm_sign_extend = 1'b0;
        imm_total_size = 3'd0;
        imm_first_size = 3'd0;

        // A PE/D change flushes the frontend, so a skeleton is never consumed
        // across a mode switch.
        entry_first = entry_rom_sel;
        // Decode group validity and row in parallel with entry_first. This
        // keeps the group select off the first-level entry PLA result.
        group_dec = pla_group_lookup({data32, opcode, pe_enable, prefix_0f});
        group_code = group_dec[5:0];
        entry_group = group_dec[6] && has_modrm;
        entry_final = entry_group ?
            group_entry_rom[{group_code, modrm[5:3],
                             (modrm[7:6] != 2'b11)}] :
            entry_first;
        invalid_lock = check_lock_invalid(prefix_rep_lock, prefix_0f, opcode,
                                          has_modrm, modrm);
        instr_bswap = prefix_0f && (opcode[7:3] == 5'b11001);
        w.entry.boundary_action = invalid_lock ? BOUNDARY_ACTION_NONE :
            decode_boundary_action(prefix_0f, opcode, has_modrm, modrm);
        w.entry.seg_reg_sel = decode_segment_register(prefix_0f, opcode,
                                                       has_modrm, modrm);
        w.entry.cmptest_is_cmp = (opcode[7:2] == 6'b100000) ||
                                 (opcode[7:3] == 5'b00111);
        w.entry.decoded_alu_op = decode_instruction_alu_op(prefix_0f, opcode,
                                                           has_modrm, modrm);
        w.entry.mul_signed = (!prefix_0f &&
                              (opcode == 8'h69 || opcode == 8'h6B)) ||
                             (prefix_0f && opcode == 8'hAF) ||
                             (!prefix_0f && has_modrm &&
                              (opcode == 8'hF6 || opcode == 8'hF7) &&
                              modrm[5:3] == 3'd5);
        w.entry.div_quotient_zf = !prefix_0f && has_modrm &&
                                  (opcode == 8'hF6 || opcode == 8'hF7) &&
                                  modrm[5:3] == 3'd6;
        w.entry.flag_op = decode_flag_op(prefix_0f, opcode);
        if (!prefix_0f && opcode[7:3] == 5'b11011)
            w.entry.fop = {opcode[2:0], modrm};
        w.entry.shift_is_double = prefix_0f &&
            ((opcode == 8'hA4) || (opcode == 8'hA5) ||
             (opcode == 8'hAC) || (opcode == 8'hAD));
        w.entry.shift_right = opcode[3];
        w.entry.shift_operation = modrm[5:3];
        w.entry.port_io = !prefix_0f &&
            ((opcode[7:2] == 6'b011011) ||  // 6C-6F: INS/OUTS
             (opcode[7:2] == 6'b111001) ||  // E4-E7: IN/OUT imm8
             (opcode[7:2] == 6'b111011));   // EC-EF: IN/OUT DX
        if ((!prefix_0f && (opcode[7:4] == 4'h7)) ||
            ( prefix_0f && (opcode[7:4] == 4'h8)))
            w.entry.rel_branch_kind = REL_BRANCH_JCC;
        else if (!prefix_0f && ((opcode == 8'hEB) || (opcode == 8'hE9)))
            w.entry.rel_branch_kind = REL_BRANCH_JMP;
        else if (!prefix_0f && (opcode == 8'hE8))
            w.entry.rel_branch_kind = REL_BRANCH_CALL;
        w.entry.branch_rel8 = !prefix_0f &&
                              ((opcode[7:4] == 4'h7) || (opcode == 8'hEB));
        w.entry.branch_condition = opcode[3:0];
        if (!prefix_0f && (opcode[7:1] == 7'b1110000))
            w.entry.repeat_kind = opcode[0] ? REPEAT_KIND_LOOPE
                                             : REPEAT_KIND_LOOPNE;
        w.entry.entry_point = invalid_lock ? UADDR_INVALID_LOCK :
                              instr_bswap ? UADDR_BSWAP : entry_final[11:0];
        w.entry.stack_op = (invalid_lock || instr_bswap) ? 1'b0 : entry_final[13];
        w.entry.stack_dir = (invalid_lock || instr_bswap) ? 1'b0 : entry_final[12];
        // BSWAP has a fixed r32 operand even in a 16-bit code segment.
        if (instr_bswap)
            w.entry.data32 = 1'b1;

        // Resolve architectural widths once in D1. These exceptions are the
        // same ones that cannot be represented by the generic W-bit rule.
        is_setcc = prefix_0f && (opcode[7:4] == 4'b1001);
        is_movzx_movsx = prefix_0f && (opcode[7:4] == 4'b1011) &&
                          (opcode[2:1] == 2'b11);
        is_movzx_word = is_movzx_movsx && opcode[0];
        is_xlat = !prefix_0f && (opcode == 8'hD7);
        is_byte_operand = is_setcc ? 1'b1 :
                          is_movzx_movsx ? 1'b0 :
                          is_xlat ? 1'b1 :
                          (w.entry.has_embedded_register && w.entry.has_w_bit)
                              ? ~opcode[3] :
                          w.entry.has_w_bit ? ~opcode[0] : 1'b0;
        w.entry.operand_size = is_byte_operand ? 2'd0 :
                               is_movzx_word ? 2'd2 :
                               w.entry.data32 ? 2'd2 : 2'd1;
        w.entry.source_size = is_movzx_movsx
                            ? (opcode[0] ? 2'd1 : 2'd0)
                            : w.entry.operand_size;

        if (has_modrm) begin
            has_sib = addr32 && (modrm[7:6] != 2'b11) && (modrm[2:0] == 3'b100);
            disp_size = has_sib ? 3'd0 : modrm_disp_size(addr32, modrm, 8'h00, 1'b0);
            s_len = 3'd2;

            unique case (ctl_bits[11:10])
                2'b00: imm_total_size = 3'd1;
                2'b01: imm_total_size = data32 ? 3'd4 : 3'd2;
                2'b10: imm_total_size = 3'd0;
                default: begin
                    imm_total_size = 3'd1;
                    imm_sign_extend = 1'b1;
                end
            endcase

            if ((opcode[7:1] == 7'b1111011) && (modrm[5:3] >= 3'd2))
                imm_total_size = 3'd0;

            w.entry.has_modrm = 1'b1;
            w.entry.modrm = modrm;
            w.entry.has_sib = has_sib;
            w.entry.sib = 8'h00;
            w.entry.imm_size = imm_total_size;
            w.need_sib = has_sib;
            w.pending_imm_size = has_sib ? imm_total_size : 3'd0;
            w.pending_imm_sign_extend = has_sib ? imm_sign_extend : 1'b0;
            select_register_fields(prefix_0f, opcode, 1'b1, modrm,
                                   w.entry.src_reg_sel, w.entry.dst_reg_sel);

            if (!has_sib) begin
                if (disp_size != 3'd0) begin
                    w.lit1_kind = LIT_DISP;
                    w.lit1_size = disp_size;
                    // Normalize disp8 while D2 captures the literal so the
                    // live EA path consumes one ready 32-bit addend.
                    w.lit1_sign_extend = (disp_size == 3'd1);
                end
                if (imm_total_size != 3'd0) begin
                    if (w.lit1_kind == LIT_NONE) begin
                        w.lit1_kind = LIT_IMM;
                        w.lit1_size = imm_total_size;
                        w.lit1_sign_extend = imm_sign_extend;
                    end else begin
                        w.lit2_kind = LIT_IMM;
                        w.lit2_size = imm_total_size;
                        w.lit2_sign_extend = imm_sign_extend;
                    end
                end
            end
        end else begin
            has_sib = 1'b0;
            disp_size = 3'd0;
            select_register_fields(prefix_0f, opcode, 1'b0, 8'h00,
                                   w.entry.src_reg_sel, w.entry.dst_reg_sel);

            if (!prefix_0f && opcode[7:2] == 6'b101000) begin
                // MOV AL/eAX,moffs and MOV moffs,AL/eAX.
                w.entry.has_moffs = 1'b1;
                imm_total_size = addr32 ? 3'd4 : 3'd2;
                imm_first_size = imm_total_size;
            end else begin
                unique case (ctl_bits[11:6])
                    6'b100111: imm_total_size = 3'd1;
                    6'b110111: imm_total_size = data32 ? 3'd4 : 3'd2;
                    6'b000111: begin
                        imm_total_size = 3'd1;
                        imm_sign_extend = 1'b1;
                    end
                    6'b010111: imm_total_size = 3'd2;
                    6'b011111: imm_total_size = data32 ? 3'd4 : 3'd2;
                    6'b100010: begin
                        imm_total_size = 3'd1;
                        imm_sign_extend = 1'b1;
                    end
                    6'b101111: imm_total_size = 3'd3;
                    6'b111111: imm_total_size = data32 ? 3'd6 : 3'd4;
                    default:   imm_total_size = 3'd0;
                endcase

                imm_first_size = imm_total_size;
                unique case (ctl_bits[11:6])
                    6'b100010: imm_first_size = 3'd0;
                    6'b101111: imm_first_size = 3'd2;
                    6'b111111: imm_first_size = data32 ? 3'd4 : 3'd2;
                    default: ;
                endcase
            end

            w.entry.imm_size = imm_total_size;
            if (imm_total_size != 3'd0) begin
                if (imm_first_size != 3'd0) begin
                    w.lit1_kind = LIT_IMM;
                    w.lit1_size = imm_first_size;
                    w.lit1_sign_extend = imm_sign_extend && (imm_first_size == 3'd1);
                    w.lit1_mirror_disp = 1'b1;
                end
                if (imm_total_size != imm_first_size) begin
                    if (w.lit1_kind == LIT_NONE) begin
                        w.lit1_kind = LIT_DISP;
                        w.lit1_size = imm_total_size - imm_first_size;
                        w.lit1_sign_extend = imm_sign_extend;
                    end else begin
                        w.lit2_kind = LIT_DISP;
                        w.lit2_size = imm_total_size - imm_first_size;
                        w.lit2_sign_extend = imm_sign_extend;
                    end
                end
            end
        end

        w.entry.length = {1'b0, prefix_count} + (prefix_0f ? 5'd1 : 5'd0) +
                         {2'b00, s_len} + {2'b00, disp_size} +
                         {2'b00, imm_total_size};
        w.entry.ind_is_ea = w.entry.has_modrm || w.entry.stack_op ||
                            w.entry.has_moffs;
    end
endtask

// synthesis translate_off
// Check the generated contents and speculative-cursor alignment anywhere D1
// is live. The X guard excludes only the startup preread.
wire [15:0] pla_entry_now =
    pla_entry_lookup({data32, opcode, prefix_rep, pe_enable, 1'b1, prefix_0f});
always_ff @(posedge clk) begin
    if (reset_n && !q_flush &&
        (^{entry_rom_sel, pla_entry_now} !== 1'bx) &&
        (entry_rom_sel !== pla_entry_now))
        $fatal(1, "entry ROM mismatch: op=%02x rom=%04x pla=%04x",
               opcode, entry_rom_sel, pla_entry_now);
end
// synthesis translate_on

function automatic decoder_work_t capture_sib(input decoder_work_t in,
                                              input logic [7:0]   sib_byte);
    decoder_work_t out;
    logic [2:0] disp_size;
    begin
        out = in;
        disp_size = modrm_disp_size(in.entry.addr32, in.entry.modrm,
                                    sib_byte, in.entry.has_sib);

        out.need_sib = 1'b0;
        out.entry.sib = sib_byte;
        out.entry.ea_uses_post_pop_esp = !in.entry.has_0f &&
            (in.entry.opcode == 8'h8F) && (sib_byte[2:0] == 3'b100);
        out.entry.length = in.entry.length + 5'd1 + {2'b00, disp_size};

        if (disp_size != 3'd0) begin
            out.lit1_kind = LIT_DISP;
            out.lit1_size = disp_size;
            out.lit1_sign_extend = (disp_size == 3'd1);
        end

        if (in.pending_imm_size != 3'd0) begin
            if (out.lit1_kind == LIT_NONE) begin
                out.lit1_kind = LIT_IMM;
                out.lit1_size = in.pending_imm_size;
                out.lit1_sign_extend = in.pending_imm_sign_extend;
            end else begin
                out.lit2_kind = LIT_IMM;
                out.lit2_size = in.pending_imm_size;
                out.lit2_sign_extend = in.pending_imm_sign_extend;
            end
        end

        out.pending_imm_size = 3'd0;
        out.pending_imm_sign_extend = 1'b0;
        capture_sib = out;
    end
endfunction

function automatic decoder_work_t capture_literal(
    input decoder_work_t in,
    input logic [31:0]   win,    // literal bytes at offset 0 of this view
    input lit_kind_t     kind,
    input logic [2:0]    size,
    input logic          sign_extend,
    input logic          mirror_disp
);
    decoder_work_t out;
    logic [31:0] value;
    begin
        out = in;
        value = literal_value(win, size, sign_extend);
        if (kind == LIT_IMM) begin
            out.entry.immediate = value;
            if (mirror_disp)
                out.entry.displacement = value;
        end else if (kind == LIT_DISP) begin
            if (in.lit1_kind == LIT_NONE) begin
                unique case (size)
                    3'd1: out.entry.displacement = {out.entry.displacement[31:8], value[7:0]};
                    3'd2: out.entry.displacement = {out.entry.displacement[31:16], value[15:0]};
                    3'd3: out.entry.displacement = {out.entry.displacement[31:24], value[23:0]};
                    default: out.entry.displacement = value;
                endcase
            end else begin
                out.entry.displacement = value;
            end
        end

        if (kind == out.lit1_kind) begin
            out.lit1_kind = LIT_NONE;
            out.lit1_size = 3'd0;
            out.lit1_sign_extend = 1'b0;
            out.lit1_mirror_disp = 1'b0;
        end else begin
            out.lit2_kind = LIT_NONE;
            out.lit2_size = 3'd0;
            out.lit2_sign_extend = 1'b0;
            out.lit2_mirror_disp = 1'b0;
        end
        capture_literal = out;
    end
endfunction

//=============================================================================
// Helpers
//=============================================================================

function automatic logic [4:0] decode_instruction_alu_op(
    input logic       has_0f_in,
    input logic [7:0] opcode_in,
    input logic       has_modrm_in,
    input logic [7:0] modrm_in
);
    logic [1:0] bit_sel;
    begin
        // Ordinary ALU opcodes encode the operation in opcode[5:3]; group
        // forms encode the same three-bit value in ModR/M.reg.
        decode_instruction_alu_op = opcode_in[7] && has_modrm_in
                                  ? {2'b00, modrm_in[5:3]}
                                  : {2'b00, opcode_in[5:3]};

        if (!has_0f_in && (opcode_in[7:4] == 4'h4))
            decode_instruction_alu_op = {4'b1100, opcode_in[3]};
        else if (!has_0f_in && has_modrm_in &&
                 (((opcode_in == 8'hF6 || opcode_in == 8'hF7) &&
                   (modrm_in[5:3] == 3'd2 || modrm_in[5:3] == 3'd3)) ||
                  ((opcode_in == 8'hFE || opcode_in == 8'hFF) &&
                   (modrm_in[5:3] == 3'd0 || modrm_in[5:3] == 3'd1))))
            decode_instruction_alu_op = {3'b110, modrm_in[4:3]};
        else if ((!has_0f_in && opcode_in == 8'h98) ||
                 (has_0f_in && opcode_in[7:4] == 4'hB &&
                  opcode_in[2:1] == 2'b11)) begin
            if (opcode_in[0] || !opcode_in[5])
                decode_instruction_alu_op = opcode_in[3] ? ALU_SEXT : ALU_ZEXT;
            else
                decode_instruction_alu_op = opcode_in[3] ? ALU_SEXT_B : ALU_ZEXT_B;
        end else if (has_0f_in &&
                     ((opcode_in == 8'hA3) || (opcode_in == 8'hAB) ||
                      (opcode_in == 8'hB3) || (opcode_in == 8'hBB) ||
                      (opcode_in == 8'hBA))) begin
            bit_sel = (opcode_in == 8'hBA) ? modrm_in[4:3] : opcode_in[4:3];
            unique case (bit_sel)
                2'b00: decode_instruction_alu_op = ALU_PASS;
                2'b01: decode_instruction_alu_op = ALU_OR;
                2'b10: decode_instruction_alu_op = ALU_ANDN;
                2'b11: decode_instruction_alu_op = ALU_XOR;
            endcase
        end else if (!has_0f_in &&
                     (opcode_in == 8'h37 || opcode_in == 8'h3F))
            decode_instruction_alu_op = opcode_in[3] ? ALU_AAS : ALU_AAA;
        else if (!has_0f_in &&
                     (opcode_in == 8'h27 || opcode_in == 8'h2F))
            decode_instruction_alu_op = opcode_in[3] ? ALU_DAS : ALU_DAA;
    end
endfunction

function automatic flag_op_t decode_flag_op(
    input logic       has_0f_in,
    input logic [7:0] opcode_in
);
    begin
        decode_flag_op = FLAG_OP_NONE;
        if (!has_0f_in) begin
            unique case (opcode_in)
                8'hF5: decode_flag_op = FLAG_OP_CMC;
                8'hF8: decode_flag_op = FLAG_OP_CLC;
                8'hF9: decode_flag_op = FLAG_OP_STC;
                8'hFA: decode_flag_op = FLAG_OP_CLI;
                8'hFB: decode_flag_op = FLAG_OP_STI;
                8'hFC: decode_flag_op = FLAG_OP_CLD;
                8'hFD: decode_flag_op = FLAG_OP_STD;
                default: ;
            endcase
        end
    end
endfunction

function automatic boundary_action_t decode_boundary_action(
    input logic       has_0f_in,
    input logic [7:0] opcode_in,
    input logic       has_modrm_in,
    input logic [7:0] modrm_in
);
    begin
        decode_boundary_action = BOUNDARY_ACTION_NONE;
        if (!has_0f_in) begin
            unique case (opcode_in)
                8'h9d, 8'hcf: decode_boundary_action = BOUNDARY_ACTION_PRESERVE_RF;
                8'h17:        decode_boundary_action = BOUNDARY_ACTION_LOAD_SS;
                8'h8e: begin
                    if (has_modrm_in && modrm_in[5:3] == 3'd2)
                        decode_boundary_action = BOUNDARY_ACTION_LOAD_SS;
                end
                8'hcc, 8'hcd: decode_boundary_action = BOUNDARY_ACTION_SOFT_INT;
                8'hce:        decode_boundary_action = BOUNDARY_ACTION_INTO;
                8'hfb:        decode_boundary_action = BOUNDARY_ACTION_STI;
                default: ;
            endcase
        end
    end
endfunction

function automatic logic [2:0] decode_segment_register(
    input logic       has_0f_in,
    input logic [7:0] opcode_in,
    input logic       has_modrm_in,
    input logic [7:0] modrm_in
);
    begin
        decode_segment_register = 3'd0;
        if (!has_0f_in) begin
            if (opcode_in[7:5] == 3'b000 && opcode_in[2:1] == 2'b11)
                decode_segment_register = {1'b0, opcode_in[4:3]};
            else begin
                unique case (opcode_in)
                    8'h8c, 8'h8e:
                        if (has_modrm_in)
                            decode_segment_register = modrm_in[5:3];
                    8'hc4: decode_segment_register = 3'd0;
                    8'hc5: decode_segment_register = 3'd3;
                    default: ;
                endcase
            end
        end else begin
            unique case (opcode_in)
                8'ha0, 8'ha1: decode_segment_register = 3'd4;
                8'ha8, 8'ha9: decode_segment_register = 3'd5;
                8'hb2:        decode_segment_register = 3'd2;
                8'hb4:        decode_segment_register = 3'd4;
                8'hb5:        decode_segment_register = 3'd5;
                default: ;
            endcase
        end
    end
endfunction

function automatic logic is_prefix(input logic [7:0] b);
    unique case (b)
        8'h26, 8'h2e, 8'h36, 8'h3e,
        8'h64, 8'h65, 8'h66, 8'h67,
        8'hf0, 8'hf2, 8'hf3: is_prefix = 1'b1;
        default: is_prefix = 1'b0;
    endcase
endfunction

function automatic logic [2:0] prefix_seg_code(input logic [7:0] b);
    unique case (b)
        8'h2e: prefix_seg_code = PREFIX_CS;
        8'h36: prefix_seg_code = PREFIX_SS;
        8'h3e: prefix_seg_code = PREFIX_DS;
        8'h26: prefix_seg_code = PREFIX_ES;
        8'h64: prefix_seg_code = PREFIX_FS;
        8'h65: prefix_seg_code = PREFIX_GS;
        default: prefix_seg_code = PREFIX_NOSEG;
    endcase
endfunction

function automatic logic [31:0] literal_value(
    input logic [31:0] bytes,
    input logic [2:0]  size,
    input logic        sign_extend
);
    begin
        unique case (size)
            3'd1: literal_value = sign_extend ? {{24{bytes[7]}}, bytes[7:0]} :
                                                 {24'h0, bytes[7:0]};
            3'd2: literal_value = {16'h0, bytes[15:0]};
            3'd3: literal_value = {8'h0, bytes[23:0]};
            3'd4: literal_value = bytes;
            default: literal_value = 32'h0;
        endcase
    end
endfunction

function automatic logic [2:0] modrm_disp_size(
    input logic       addr32_in,
    input logic [7:0] modrm_in,
    input logic [7:0] sib_in,
    input logic       has_sib_in
);
    if (modrm_in[7:6] == 2'b11)
        modrm_disp_size = 3'd0;
    else if (modrm_in[7:6] == 2'b01)
        modrm_disp_size = 3'd1;
    else if (modrm_in[7:6] == 2'b10)
        modrm_disp_size = addr32_in ? 3'd4 : 3'd2;
    else if (addr32_in && has_sib_in && sib_in[2:0] == 3'b101)
        modrm_disp_size = 3'd4;
    else if (addr32_in && !has_sib_in && modrm_in[2:0] == 3'b101)
        modrm_disp_size = 3'd4;
    else if (!addr32_in && modrm_in[2:0] == 3'b110)
        modrm_disp_size = 3'd2;
    else
        modrm_disp_size = 3'd0;
endfunction

task automatic select_register_fields(
    input  logic        has_0f_in,
    input  logic [7:0]  opcode_in,
    input  logic        has_modrm_in,
    input  logic [7:0]  modrm_in,
    output logic [2:0]  src_reg_sel_out,
    output logic [2:0]  dst_reg_sel_out
);
    begin
        src_reg_sel_out = has_modrm_in ? modrm_in[5:3] : opcode_in[2:0];
        dst_reg_sel_out = has_modrm_in ? (opcode_in[1] ? modrm_in[5:3] :
                                                        modrm_in[2:0]) :
                                         opcode_in[2:0];

        if (has_0f_in && has_modrm_in) begin
            src_reg_sel_out = modrm_in[5:3];
            dst_reg_sel_out = modrm_in[2:0];
            if (opcode_in == 8'h22 || opcode_in == 8'h23 || opcode_in == 8'h26)
                src_reg_sel_out = modrm_in[2:0];
        end else if (!has_0f_in) begin
            unique casez (opcode_in)
                8'b00???10?: begin
                    src_reg_sel_out = 3'd0;
                    dst_reg_sel_out = 3'd0;
                end
                8'b00???0??: begin
                    src_reg_sel_out = opcode_in[1] ? modrm_in[2:0] : modrm_in[5:3];
                    dst_reg_sel_out = opcode_in[1] ? modrm_in[5:3] : modrm_in[2:0];
                end
                8'h8a, 8'h8b: begin
                    src_reg_sel_out = modrm_in[2:0];
                    dst_reg_sel_out = modrm_in[5:3];
                end
                8'h8c, 8'h8e: begin
                    src_reg_sel_out = modrm_in[5:3];
                    dst_reg_sel_out = modrm_in[2:0];
                end
                8'h62: begin
                    src_reg_sel_out = modrm_in[5:3];
                    dst_reg_sel_out = modrm_in[2:0];
                end
                8'h8f, 8'hfe, 8'hff: dst_reg_sel_out = modrm_in[2:0];
                8'h63: begin
                    src_reg_sel_out = modrm_in[5:3];
                    dst_reg_sel_out = modrm_in[2:0];
                end
                8'h69, 8'h6b: begin
                    src_reg_sel_out = modrm_in[5:3];
                    dst_reg_sel_out = modrm_in[2:0];
                end
                8'b100000??: dst_reg_sel_out = modrm_in[2:0];
                8'hc0, 8'hc1, 8'hd0, 8'hd1,
                8'hd2, 8'hd3: dst_reg_sel_out = modrm_in[2:0];
                8'h86, 8'h87: begin
                    src_reg_sel_out = modrm_in[5:3];
                    dst_reg_sel_out = (modrm_in[7:6] == 2'b11) ? modrm_in[2:0] :
                                                                    modrm_in[5:3];
                end
                8'hd8, 8'hd9, 8'hda, 8'hdb,
                8'hdc, 8'hdd, 8'hde, 8'hdf: begin
                    src_reg_sel_out = modrm_in[5:3];
                    dst_reg_sel_out = modrm_in[2:0];
                end
                8'b10010???: begin
                    src_reg_sel_out = opcode_in[2:0];
                    dst_reg_sel_out = 3'd0;
                end
                8'b101000??, 8'ha8, 8'ha9: begin
                    src_reg_sel_out = 3'd0;
                    dst_reg_sel_out = 3'd0;
                end
                8'hc6, 8'hc7, 8'hf6, 8'hf7: dst_reg_sel_out = modrm_in[2:0];
                default: ;
            endcase
        end
    end
endtask

endmodule
