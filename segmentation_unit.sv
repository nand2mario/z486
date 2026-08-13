// Segmentation Unit for z386 This contains: - Descriptor Cache array (ES/CS/SS/DS/FS/GS/IDT/TR/LDT/GDT) - Segment selection and address...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-segmentation-unit-sv-1
module segmentation_unit
    import z386_pkg::*;
(
    input              clk,
    input              reset_n,

    // Command interface — descriptor cache manipulation
    input              seg_cmd_valid,       // 1 when command should actually execute
    input              stssaf_pulse,        // STSSAF aluop: sideband so it coexists with a busop
    input              ctssaf_pulse,        // CTSSAF aluop: 608 pairs CTSSAF with SDEL, 74B STSSAF with IN=+
    input      [3:0]   seg_cmd,            // SEG_CMD_* command (computed stall-independently)
    input      [3:0]   seg_target,         // Target segment (SEG_ES..SEG_GDT, SEG_NONE)
    input              init_addr32,        // D2 instruction address size for INIT_SEG
    input              init_stack_op,      // D2 stack operation for INIT_SEG
    input              clear_descsw,       // DESSTK mode bit for UPDATE_SEG
    input      [31:0]  seg_data,           // Descriptor command payload
    input      [31:0]  desc_lo,            // Descriptor low DWORD (from TMPC)
    input      [31:0]  desc_hi,            // Descriptor high DWORD (saved at PTOVRR)
    input      [15:0]  slctr,              // SLCTR register (null selector check for SDEL)
    input prot_transition_t transition,    // Protection-driven descriptor updates

    output seg_desc_t  desc_cache [0:7],   // ES..GS, TR, and LDTR hidden descriptors
    output logic [31:0] idt_base,
    output logic [19:0] idt_limit,
    output logic [31:0] gdt_base,
    output logic [19:0] gdt_limit,
    output     [31:0]  lar_result,         // LAR combinational readback (keyed by seg_target)
    output     [31:0]  llim_result,        // LLIM combinational readback (keyed by seg_target)
    output     [31:0]  lbas_result,        // LBAS combinational readback (keyed by seg_target)

    // Segment state (set by commands, used by address translation and z386)
    output reg [3:0]   seg_sel,            // Active segment for memory ops
    output reg         seg_is_io,          // seg_sel == SEG_IO, mirrored at every seg_sel write
                                           // (keeps the 4-bit compare out of the stall/uc_exec cone)
    output reg         is_dtable,          // Accessing GDT/IDT
    output reg         descsw_mode,        // Cross-privilege stack switch active
    output reg         tss_access_flag,    // TSS access flag (for JTSSAF)

    // Address translation — offset → linear address + fault check
    input              pe,                 // CR0.PE - protected mode enable
    input              vm,                 // EFLAGS.VM - V86 mode
    input      [1:0]   cpl,                // Current privilege level
    input      [31:0]  offset,             // Segment offset (EA or IND)
    input      [1:0]   access_size,        // 0=byte, 1=word, 3=dword
    input              check_en,           // Limit check enabled this cycle
    input              is_mem_op,          // Memory operation (needs limit check)
    input              is_write,           // Write operation (needs writable check)

    output     [31:0]  seg_base_pending,   // Next seg_base_r (this cycle's pending base),
                                           // so z386 can pre-register linear_address = base+IND
    output             eff_mask_pending,   // Next (addr_size || is_dtable): 0 => mask offset to 16b
    output             seg_fault,          // Segment limit/protection fault
    output             is_stack_fault      // Fault is on SS (→ #SS not #GP)
);

reg [3:0]   desc_write_seg;     // Tracks target segment for SDES/SDEL
reg         stack_push_mode;    // Stack push direction active (internal; demoted from output)
reg         dt_target_idt;      // Tracks GDTR vs IDTR for SBAS/SLIM_TABLE
reg         addr_size;          // 1=32-bit, 0=16-bit effective address
reg [31:0]  seg_base_r;         // Registered segment base
reg [31:0]  seg_limit_r;        // Registered segment limit

// Full hidden descriptors exist only for the six architectural segment
// registers, TR, and LDTR. GDTR/IDTR contain only base+limit, and SEG_IO is a
// stateless pseudo-segment. Keeping those out of the variable-write array
// avoids synthesizing complete descriptor entries and their write muxes.
function automatic [2:0] desc_index(input [3:0] target);
    case (target)
        SEG_ES, SEG_CS, SEG_SS, SEG_DS, SEG_FS, SEG_GS:
            desc_index = target[2:0];
        SEG_TR:  desc_index = 3'd6;
        SEG_LDT: desc_index = 3'd7;
        default: desc_index = 3'd0;
    endcase
endfunction

function automatic logic desc_target_valid(input [3:0] target);
    desc_target_valid = (target <= SEG_GS) || (target == SEG_TR) ||
                        (target == SEG_LDT);
endfunction

function automatic seg_desc_t desc_for(input [3:0] target);
    seg_desc_t d;
    begin
        d = '0;
        case (target)
            SEG_ES, SEG_CS, SEG_SS, SEG_DS, SEG_FS, SEG_GS:
                d = desc_cache[target[2:0]];
            SEG_TR:  d = desc_cache[6];
            SEG_LDT: d = desc_cache[7];
            SEG_IDT: begin
                d = seg_desc_idt_real_mode();
                d.base = idt_base;
                d.limit = idt_limit;
            end
            SEG_GDT: begin
                d = seg_desc_idt_real_mode();
                d.base = gdt_base;
                d.limit = gdt_limit;
            end
            default: ;
        endcase
        desc_for = d;
    end
endfunction

// Instruction state latched from SEG_CMD_INIT_SEG
reg         i_addr32_r;
reg         i_stack_op_r;

wire [31:0] ES_base = desc_cache[SEG_ES].base;
wire [31:0] CS_base = desc_cache[SEG_CS].base;
wire [31:0] SS_base = desc_cache[SEG_SS].base;
wire [31:0] DS_base = desc_cache[SEG_DS].base;
wire [31:0] FS_base = desc_cache[SEG_FS].base;
wire [31:0] GS_base = desc_cache[SEG_GS].base;
// (the old seg_sel-keyed `seg_base` combinational mux was dead -- seg_base_r is
//  driven from seg_base_for(seg_target); removed to recover the 6-way mux.)

wire [31:0] eff_offset = (addr_size || is_dtable) ? offset : {16'h0, offset[15:0]};

// Pending base: the value seg_base_r will hold next cycle. Mirrors exactly the seg_base_r next-state in the always_ff below (same...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-segmentation-unit-sv-83
reg [31:0] seg_base_pending_c;
reg        addr_size_pending_c;
reg        is_dtable_pending_c;
always_comb begin
    seg_base_pending_c  = seg_base_r;         // hold when no command
    addr_size_pending_c = addr_size;
    is_dtable_pending_c = is_dtable;
    if (seg_cmd_valid) begin
        if (stssaf_pulse && seg_sel == SEG_SS)
            seg_base_pending_c = SS_base;      // (does not change addr_size/is_dtable)
        case (seg_cmd)
            SEG_CMD_INIT_SEG: begin
                seg_base_pending_c  = seg_base_for(seg_target, 1'b0);
                addr_size_pending_c = (init_stack_op && pe) ? desc_cache[SEG_SS].D_B
                                                            : init_addr32;
                is_dtable_pending_c = 1'b0;
            end
            SEG_CMD_UPDATE_SEG: begin
                is_dtable_pending_c = (seg_target == SEG_IDT || seg_target == SEG_GDT);
                if (clear_descsw) begin
                    seg_base_pending_c  = seg_base_for(seg_target, 1'b0);
                    addr_size_pending_c = pe ? desc_cache[SEG_SS].D_B : i_addr32_r;
                end else if (seg_target == SEG_IO) begin
                    // I/O is not a segmented memory offset. Preserve reserved
                    // 32-bit coprocessor ports; IN/OUT sources are already
                    // zero-extended to their architectural port width.
                    seg_base_pending_c  = 32'h0;
                    addr_size_pending_c = 1'b1;
                end else begin
                    seg_base_pending_c  = seg_base_for(seg_target, descsw_mode);
                    addr_size_pending_c = ((i_stack_op_r || stack_push_mode) && pe && seg_target == SEG_SS)
                                          ? (descsw_mode ? desc_cache[SEG_CS].D_B : desc_cache[SEG_SS].D_B)
                                          : i_addr32_r;
                end
            end
            SEG_CMD_DESCSW: begin
                seg_base_pending_c  = CS_base;
                addr_size_pending_c = pe ? desc_cache[SEG_CS].D_B : i_addr32_r;
                is_dtable_pending_c = 1'b0;
            end
            default: ;                         // SPCR/others: keep stssaf-or-hold
        endcase
    end
end
assign seg_base_pending = seg_base_pending_c;
assign eff_mask_pending = addr_size_pending_c || is_dtable_pending_c;

assign is_stack_fault = (seg_sel == SEG_SS);

// Split into subtractor + size comparison to keep access_size off the 32-bit critical path
wire [32:0] base_diff = {1'b0, seg_limit_r} - {1'b0, eff_offset};
wire start_out_of_bounds = base_diff[32];
// access_size encoding (0=byte,1=word,3=dword) == bytes-1, so check remaining < access_size
wire [31:0] diff_res = base_diff[31:0];
wire size_fault = (diff_res[31:3] == 29'd0) && (diff_res[2:0] < access_size);
wire limit_violated = start_out_of_bounds | size_fault;

// Real mode: a boundary-crossing access always faults (even SS -> #SS, e.g.
// POPAD at SP=0xFFFE); only start-out-of-bounds keeps the 16-bit-stack wrap.
wire rm_limit_fault = !pe && (size_fault ||
                      (start_out_of_bounds && !(is_stack_fault && !addr_size)));

wire pm_limit_fault = pe && limit_violated && !is_dtable;

wire seg_writable = (seg_sel == SEG_ES) ? (!desc_cache[SEG_ES].seg_type[3] && desc_cache[SEG_ES].seg_type[1]) :
                    (seg_sel == SEG_CS) ? vm :
                    (seg_sel == SEG_SS) ? 1'b1 :
                    (seg_sel == SEG_DS) ? (!desc_cache[SEG_DS].seg_type[3] && desc_cache[SEG_DS].seg_type[1]) :
                    (seg_sel == SEG_FS) ? (!desc_cache[SEG_FS].seg_type[3] && desc_cache[SEG_FS].seg_type[1]) :
                    (seg_sel == SEG_GS) ? (!desc_cache[SEG_GS].seg_type[3] && desc_cache[SEG_GS].seg_type[1]) :
                    1'b1;  // TR/IDT/GDT/IO: no write check
wire write_fault = pe && is_write && !seg_writable && !is_dtable;

assign seg_fault = check_en && is_mem_op &&
                   (seg_sel != SEG_IO) &&
                   (rm_limit_fault || pm_limit_fault || write_fault);

function automatic [31:0] seg_base_for(input [3:0] sel, input dsw);
    case (sel)
        SEG_ES: seg_base_for = ES_base;
        SEG_CS: seg_base_for = CS_base;
        SEG_SS: seg_base_for = dsw ? CS_base : SS_base;
        SEG_DS: seg_base_for = DS_base;
        SEG_FS: seg_base_for = FS_base;
        SEG_GS: seg_base_for = GS_base;
        SEG_TR: seg_base_for = desc_cache[6].base;
        // Descriptor-table pseudo-segments: the table base is the relocation
        // base (offset = selector index*8 / IDT offset lives in IND).  GDT vs LDT
        // is chosen by the selector's TI bit (slctr[2]); all DESSDT accesses
        // route the selector through SLCTR.
        SEG_GDT: seg_base_for = slctr[2] ? desc_cache[7].base : gdt_base;
        SEG_IDT: seg_base_for = idt_base;
        default: seg_base_for = 32'h0;
    endcase
endfunction

function automatic [20:0] raw_limit_for(input [3:0] sel, input dsw);
    case (sel)
        SEG_ES:  raw_limit_for = {desc_cache[SEG_ES].G, desc_cache[SEG_ES].limit};
        SEG_CS:  raw_limit_for = {desc_cache[SEG_CS].G, desc_cache[SEG_CS].limit};
        SEG_SS:  raw_limit_for = dsw
                               ? {desc_cache[SEG_CS].G, desc_cache[SEG_CS].limit}
                               : {desc_cache[SEG_SS].G, desc_cache[SEG_SS].limit};
        SEG_DS:  raw_limit_for = {desc_cache[SEG_DS].G, desc_cache[SEG_DS].limit};
        SEG_FS:  raw_limit_for = {desc_cache[SEG_FS].G, desc_cache[SEG_FS].limit};
        SEG_GS:  raw_limit_for = {desc_cache[SEG_GS].G, desc_cache[SEG_GS].limit};
        SEG_TR:  raw_limit_for = {desc_cache[6].G, desc_cache[6].limit};
        SEG_LDT: raw_limit_for = {desc_cache[7].G, desc_cache[7].limit};
        default: raw_limit_for = 21'h1F_FFFF;
    endcase
endfunction

function automatic [31:0] expand_raw_limit(input [20:0] raw_limit);
    expand_raw_limit = raw_limit[20]
                     ? {raw_limit[19:0], 12'hFFF}
                     : {12'h000, raw_limit[19:0]};
endfunction

function automatic [31:0] seg_limit_for(input [3:0] sel, input dsw);
    case (sel)
        SEG_ES, SEG_CS, SEG_SS, SEG_DS, SEG_FS, SEG_GS, SEG_TR:
            seg_limit_for = expand_raw_limit(raw_limit_for(sel, dsw));
        default: seg_limit_for = 32'hFFFFFFFF;
    endcase
endfunction

function automatic [3:0] effective_target(input [3:0] target);
    effective_target = (target == SEG_NONE) ? desc_write_seg : target;
endfunction

// LAR/LLIM/LBAS: z386 routes these to IND in same cycle
seg_desc_t lar_desc;
wire [7:0] lar_ar_byte = {lar_desc.P, lar_desc.DPL, lar_desc.S, lar_desc.seg_type};
assign lar_desc = desc_for(seg_target);
assign lar_result = {16'h0, lar_ar_byte, 8'h0};
always_comb begin
    case (seg_target)
        SEG_ES, SEG_CS, SEG_SS, SEG_DS, SEG_FS, SEG_GS, SEG_TR, SEG_LDT:
            llim_result = expand_raw_limit(raw_limit_for(seg_target, 1'b0));
        SEG_IDT: llim_result = {12'h0, idt_limit};
        SEG_GDT: llim_result = {12'h0, gdt_limit};
        default: llim_result = 32'h0;
    endcase
end
assign lbas_result = lar_desc.base;

// Testbench can't force unpacked array struct elements, so use individual regs
`ifdef VERILATOR
seg_desc_t seg_init_es, seg_init_cs, seg_init_ss, seg_init_ds;
seg_desc_t seg_init_fs, seg_init_gs, seg_init_idt, seg_init_tr;
seg_desc_t seg_init_ldt, seg_init_gdt;
initial begin
    seg_init_es  = seg_desc_real_mode(16'h0000);
    seg_init_cs  = seg_desc_reset_cs();
    seg_init_ss  = seg_desc_real_mode(16'h0000);
    seg_init_ds  = seg_desc_real_mode(16'h0000);
    seg_init_fs  = seg_desc_real_mode(16'h0000);
    seg_init_gs  = seg_desc_real_mode(16'h0000);
    seg_init_idt = seg_desc_idt_real_mode();
    seg_init_tr  = seg_desc_real_mode(16'h0000);
    seg_init_ldt = seg_desc_real_mode(16'h0000);
    seg_init_gdt = seg_desc_idt_real_mode();
end
`endif

//=============================================================================
// Segment descriptor cache
//=============================================================================
always_ff @(posedge clk) begin
    if (!reset_n) begin
`ifdef VERILATOR
        desc_cache[0] <= seg_init_es;
        desc_cache[1] <= seg_init_cs;
        desc_cache[2] <= seg_init_ss;
        desc_cache[3] <= seg_init_ds;
        desc_cache[4] <= seg_init_fs;
        desc_cache[5] <= seg_init_gs;
        desc_cache[6] <= seg_init_tr;
        desc_cache[7] <= seg_init_ldt;
        idt_base <= seg_init_idt.base;
        idt_limit <= seg_init_idt.limit;
        gdt_base <= seg_init_gdt.base;
        gdt_limit <= seg_init_gdt.limit;
`else
        desc_cache[0] <= seg_desc_real_mode(16'h0000);
        desc_cache[1] <= seg_desc_reset_cs();
        desc_cache[2] <= seg_desc_real_mode(16'h0000);
        desc_cache[3] <= seg_desc_real_mode(16'h0000);
        desc_cache[4] <= seg_desc_real_mode(16'h0000);
        desc_cache[5] <= seg_desc_real_mode(16'h0000);
        desc_cache[6] <= seg_desc_real_mode(16'h0000);
        desc_cache[7] <= seg_desc_real_mode(16'h0000);
        idt_base <= 32'h0;
        idt_limit <= 20'h003FF;
        gdt_base <= 32'h0;
        gdt_limit <= 20'h003FF;
`endif
    end else if (seg_cmd_valid) begin
        case (seg_cmd)
            SEG_CMD_SBRM: begin
                // Real-mode segment register load: set base = segment × 16.
                // Limit and granularity are NOT reset, thus preserving "unreal mode"
                case (seg_target[2:0])
                    SEG_ES:  desc_cache[0].base <= {12'h0, seg_data[15:0], 4'h0};
                    SEG_CS: begin
                        desc_cache[1].base <= {12'h0, seg_data[15:0], 4'h0};
                        desc_cache[1].D_B <= 1'b0;
                    end
                    SEG_SS:  desc_cache[2].base <= {12'h0, seg_data[15:0], 4'h0};
                    SEG_DS:  desc_cache[3].base <= {12'h0, seg_data[15:0], 4'h0};
                    SEG_FS:  desc_cache[4].base <= {12'h0, seg_data[15:0], 4'h0};
                    SEG_GS:  desc_cache[5].base <= {12'h0, seg_data[15:0], 4'h0};
                    SEG_IDT: idt_base <= {12'h0, seg_data[15:0], 4'h0};
                    default: ;
                endcase
            end

            SEG_CMD_SAR: begin
                automatic logic [3:0] sar_seg;
                automatic logic [2:0] sar_idx;
                sar_seg = effective_target(seg_target);
                sar_idx = desc_index(sar_seg);
                if (desc_target_valid(sar_seg)) begin
                    desc_cache[sar_idx].P        <= seg_data[15];
                    desc_cache[sar_idx].DPL      <= seg_data[14:13];
                    desc_cache[sar_idx].S        <= seg_data[12];
                    desc_cache[sar_idx].seg_type <= seg_data[11:8];
                    desc_cache[sar_idx].G        <= seg_data[7];
                    desc_cache[sar_idx].D_B      <= seg_data[6];
                    desc_cache[sar_idx].A        <= seg_data[8];
                end
            end

            SEG_CMD_SLIM: begin
                automatic logic [3:0] slim_seg;
                slim_seg = effective_target(seg_target);
                if (desc_target_valid(slim_seg))
                    desc_cache[desc_index(slim_seg)].limit <= seg_data[19:0];
            end

            SEG_CMD_SDEH: begin
                automatic logic [3:0] sdeh_seg;
                automatic logic [2:0] sdeh_idx;
                sdeh_seg = effective_target(seg_target);
                sdeh_idx = desc_index(sdeh_seg);
                if (desc_target_valid(sdeh_seg)) begin
                    desc_cache[sdeh_idx].base[31:24]  <= seg_data[31:24];
                    desc_cache[sdeh_idx].G            <= seg_data[23];
                    desc_cache[sdeh_idx].D_B          <= seg_data[22];
                    desc_cache[sdeh_idx].limit[19:16] <= seg_data[19:16];
                end
            end

            SEG_CMD_SDES: begin
                automatic logic [2:0] sdes_idx;
                sdes_idx = desc_index(desc_write_seg);
                desc_cache[sdes_idx].P          <= seg_data[31];
                desc_cache[sdes_idx].DPL        <= seg_data[30:29];
                desc_cache[sdes_idx].S          <= seg_data[28];
                desc_cache[sdes_idx].seg_type   <= seg_data[27:24];
                desc_cache[sdes_idx].base[23:0] <= seg_data[23:0];
                desc_cache[sdes_idx].A          <= seg_data[24];
            end

            SEG_CMD_SDEL: begin
                if (slctr[15:2] == 14'b0) begin
                    automatic seg_desc_t null_desc;
                    null_desc = '0;
                    null_desc.base = 32'hFFFFFFFF;
                    null_desc.limit = 20'hFFFFF;
                    null_desc.P = 1'b1;
                    desc_cache[desc_index(desc_write_seg)] <= null_desc;
                end else begin
                    desc_cache[desc_index(desc_write_seg)].limit[15:0] <= seg_data[15:0];
                    if (desc_write_seg == SEG_TR)
                        desc_cache[6].seg_type[1] <= 1'b1;
                end
            end

            SEG_CMD_DESC: begin
                automatic logic [3:0] full_seg;
                full_seg = effective_target(seg_target);
                if (desc_target_valid(full_seg))
                    desc_cache[desc_index(full_seg)] <= decode_descriptor(desc_lo, desc_hi);
            end

            SEG_CMD_SBAS: begin
                if (dt_target_idt)
                    idt_base <= seg_data;
                else
                    gdt_base <= seg_data;
            end

            SEG_CMD_SLIM_TABLE: begin
                if (dt_target_idt)
                    idt_limit <= {4'h0, seg_data[15:0]};
                else
                    gdt_limit <= {4'h0, seg_data[15:0]};
            end

            default: ;
        endcase

        if (transition.copy_stack_dpl)
            desc_cache[2].DPL <= transition.copy_dpl;

        if (transition.conform_dpl)
            desc_cache[1].DPL <= transition.conform_dpl_value;
    end
end

always_ff @(posedge clk) begin
    if (!reset_n) begin
        dt_target_idt <= 1'b0;
        desc_write_seg <= 4'd0;
    end else if (seg_cmd_valid) begin
        if (seg_target == SEG_IDT)
            dt_target_idt <= 1'b1;
        else if (seg_target == SEG_GDT)
            dt_target_idt <= 1'b0;

        if (seg_target != SEG_NONE &&
            seg_target != SEG_IDT && seg_target != SEG_GDT && seg_target != SEG_IO)
            desc_write_seg <= seg_target;
    end
end

//=============================================================================
// Segment selection, address size, and mode flags
//=============================================================================
always_ff @(posedge clk) begin
    if (!reset_n) begin
        seg_sel <= SEG_DS;
        seg_is_io <= 1'b0;
        is_dtable <= 1'b0;
        addr_size <= 1'b0;
        seg_base_r <= 32'h0;  // DS_base at reset
        seg_limit_r <= 32'hFFFF;
        stack_push_mode <= 1'b0;
        descsw_mode <= 1'b0;
        tss_access_flag <= 1'b0;
        i_addr32_r <= 1'b0;
        i_stack_op_r <= 1'b0;
    end else if (seg_cmd_valid) begin
        // seg_base_r / addr_size / is_dtable next-state is computed ONCE in the always_comb above (seg_base_pending_c / addr_size_pending_c /...
        // Details: doc/z386x/implementation_notes.md#src-24-z386x-segmentation-unit-sv-384
        seg_base_r <= seg_base_pending_c;
        addr_size  <= addr_size_pending_c;
        is_dtable  <= is_dtable_pending_c;
        // STSSAF/CTSSAF arrive as aluop sidebands so the same uop's busop
        // seg_cmd still executes (608: CTSSAF+SDEL dropped the new-stack
        // limit, leaving a stale CS limit for the gate frame pushes).
        // Applied before the case so a same-cycle command wins conflicts.
        if (stssaf_pulse) begin
            stack_push_mode <= 1'b0;
            descsw_mode <= 1'b0;
            tss_access_flag <= 1'b1;
            if (seg_sel == SEG_SS)
                seg_limit_r <= seg_limit_for(SEG_SS, 1'b0);
        end
        if (ctssaf_pulse)
            tss_access_flag <= 1'b0;
        case (seg_cmd)
            SEG_CMD_INIT_SEG: begin
                seg_sel <= seg_target;
                seg_is_io <= (seg_target == SEG_IO);
                seg_limit_r <= seg_limit_for(seg_target, 1'b0);
                i_addr32_r <= init_addr32;
                i_stack_op_r <= init_stack_op;
                stack_push_mode <= 1'b0;
                descsw_mode <= 1'b0;
            end

            SEG_CMD_UPDATE_SEG: begin
                seg_sel <= seg_target;
                seg_is_io <= (seg_target == SEG_IO);
                if (clear_descsw) begin
                    descsw_mode <= 1'b0;
                    seg_limit_r <= seg_limit_for(seg_target, 1'b0);
                end else begin
                    seg_limit_r <= seg_limit_for(seg_target, descsw_mode);
                end
            end

            SEG_CMD_SPCR: begin
                stack_push_mode <= 1'b1;
            end

            SEG_CMD_DESCSW: begin
                stack_push_mode <= 1'b1;
                seg_sel <= SEG_SS;
                seg_is_io <= 1'b0;
                descsw_mode <= 1'b1;
                seg_limit_r <= seg_limit_for(SEG_CS, 1'b0);
            end

            default: ;
        endcase
    end
end

endmodule
