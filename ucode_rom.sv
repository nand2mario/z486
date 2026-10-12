//
// Microcode ROM
// Original 80386 microcode with per-word predecode
//
`include "z486_platform.svh"
module ucode_rom
    import z486_pkg::*;
#(
    parameter INIT_HEX = "ucode.hex"
) (
    // Clock
    input              clk,

    // Port A (EX): the sequencer's address; its word feeds the output register
    input              addr_ce,         // load the port-A address register
    input       [11:0] addr,            // port-A word address
    output      [2:0]  q_kind_early,    // recipe kind of port A's word

    // Port B (D2): the D2 skeleton's first word, read when the skeleton loads
    input              addr_b_ce,       // load the port-B address register
    input       [11:0] addr_b,          // port-B word address (the skeleton's entry)
    output      [2:0]  q_kind_b,        // recipe kind of port B's word

    // Output register: loads port A's word, or port B's at issue (q_sel_b), with predecode
    input              q_ce,            // load the output register
    input              q_sel_b,         // it loads port B's word (registered terms only)
    output      [50:0] q_early,         // the selected word, predecoded, before the register
    output      [50:0] q,               // the executing microword
    output      [5:0]  q_shift_source,
    output      [3:0]  q_shift_source_class,
    output      [1:0]  q_shift2_source,
    output             q_is_shift2,
    output             q_shift_uc_carry,
    output      [5:0]  q_shift_alu_src,
    output      [6:0]  q_shift_aluop,
    output      [1:0]  q_shift_sigma_sel,
    output      [6:0]  q_alu_op_sel,     // {from IR, CMP/TEST, constant ALU op}
    output      [2:0]  q_dly_source,
    output      [8:0]  q_mem_ctrl,
    output      [8:0]  q_ind_ctrl,
    output             q_fpu_f8
);

`Z486_KEEP reg [5:0] q_shift_source_r;
`Z486_KEEP reg [3:0] q_shift_source_class_r;
`Z486_KEEP reg [1:0] q_shift2_source_r;
`Z486_KEEP reg q_is_shift2_r;
`Z486_KEEP reg q_shift_uc_carry_r;
`Z486_KEEP reg [5:0] q_shift_alu_src_r;
`Z486_KEEP reg [6:0] q_shift_aluop_r;
`Z486_KEEP reg [1:0] q_shift_sigma_sel_r;
// Powers up as the pre-decode of the all-zero field (ALUJMP_ALU), like the
// raw field it shadows.
`Z486_KEEP reg [6:0] q_alu_op_sel_r = {2'b10, ALU_PASS};
reg [2:0] q_dly_source_r;
`Z486_KEEP reg [8:0] q_mem_ctrl_r;
reg [8:0] q_ind_ctrl_r;
reg q_fpu_f8_r;

// ALU source 1e is the historical 0x800000f8 coprocessor command port. z486
// also reuses that encoding for SIGMA in non-FPU words, so keep the distinction
// as a compact ROM-delay sideband rather than restoring a wide ALU mux input.
function automatic logic fpu_f8_predecode(input [36:0] w);
    logic [5:0] alusrc;
    logic [6:0] dest;
    logic [5:0] source;
    logic [6:0] aluop;
    logic [5:0] buscode;
    begin
        alusrc = w[36:31];
        dest = w[30:24];
        source = w[23:18];
        aluop = w[17:11];
        buscode = w[5:0];
        fpu_f8_predecode = (alusrc == 6'h1e) &&
            (((dest == DEST_DES_IO) && (buscode == BUSOP_IND_ALU2)) ||
             ((dest == DEST_TMPH) && (source == SRC_SIGMA) &&
              (aluop == ALUJMP_PASS2) && (buscode == 6'h0c)));
    end
endfunction

// Source class for architectural GPR writes in an RNI delay slot. The Intel
// ROM uses only these four sources; zero also covers non-forwarding uops.
function automatic [2:0] dly_source_predecode(input [5:0] s);
    case (s)
        SRC_SIGMA:  dly_source_predecode = 3'd1;
        SRC_OPR_R:  dly_source_predecode = 3'd2;
        SRC_COUNTR: dly_source_predecode = 3'd3;
        SRC_NEG1:   dly_source_predecode = 3'd4;
        default:    dly_source_predecode = 3'd0;
    endcase
endfunction

// SHIFT2 uses only four source fields in the immutable Intel ROM. Register a
// compact selector beside q so the flag-critical barrel path avoids the
// high-fanout general source decoder.
function automatic [1:0] shift2_source_predecode(input [5:0] s);
    case (s)
        SRC_TMPC:   shift2_source_predecode = 2'd0;
        SRC_TMPE:   shift2_source_predecode = 2'd1;
        SRC_SIGMA:  shift2_source_predecode = 2'd2;
        SRC_SRCREG: shift2_source_predecode = 2'd3;
        default:    shift2_source_predecode = 2'd0;
    endcase
endfunction

// Only the BSR loop consumes SHIFT2 carry in the immediately following
// micro-jump. Other shift flags retire through the registered flag sideband.
function automatic logic shift_uc_carry_predecode(input [36:0] w);
    shift_uc_carry_predecode = (w[17:11] == ALUJMP_SHIFT2) &&
                               (w[23:18] == SRC_TMPC) &&
                               (w[36:31] == ALUSRC_TMPB);
endfunction

// Compact result selector for the SIGMA write path. Keep the raw seven-bit
// ALU/jump field out of the barrel-result mux and SHIFT2 flag capture.
// ALU operation for the ALU/jump field, classified ahead of the ALU so the
// op select is one registered 2:1 choice instead of a casez on the raw field:
// [6] the op comes from the instruction (decoded_alu_op), [5] CMP/TEST picks
// CMP or AND by the instruction, else [4:0] is the op.
function automatic [6:0] alu_op_predecode(input [6:0] aluop);
    case (aluop)
        ALUJMP_ALU,
        ALUJMP_INCDEC,
        ALUJMP_SZ_EXT,
        ALUJMP_AAAAAS,
        ALUJMP_DAADAS,
        ALUJMP_SERECO: alu_op_predecode = {2'b10, ALU_PASS};
        ALUJMP_CMPTST: alu_op_predecode = {2'b01, ALU_PASS};
        ALUJMP_AND:    alu_op_predecode = {2'b00, ALU_AND};
        ALUJMP_OR:     alu_op_predecode = {2'b00, ALU_OR};
        ALUJMP_XOR:    alu_op_predecode = {2'b00, ALU_XOR};
        ALUJMP_SIGN:   alu_op_predecode = {2'b00, ALU_SIGN};
        ALUJMP_ADD:    alu_op_predecode = {2'b00, ALU_ADD};
        ALUJMP_ADC:    alu_op_predecode = {2'b00, ALU_ADC};
        ALUJMP_SUB:    alu_op_predecode = {2'b00, ALU_SUBT};
        ALUJMP_CMP:    alu_op_predecode = {2'b00, ALU_CMP};
        ALUJMP_PASS2:  alu_op_predecode = {2'b00, ALU_PASS2};
        default:       alu_op_predecode = {2'b00, ALU_PASS};
    endcase
endfunction

function automatic [1:0] shift_sigma_predecode(input [6:0] aluop);
    case (aluop)
        ALUJMP_SHIFT1:
            shift_sigma_predecode = 2'd1;
        ALUJMP_SHIFT,
        ALUJMP_USTEP_AAD_SHIFT,
        ALUJMP_SHIFT2,
        ALUJMP_BITTST:
            shift_sigma_predecode = 2'd2;
        default:
            shift_sigma_predecode = 2'd0;
    endcase
endfunction

// Compact source class for the barrel path. The immutable ROM uses only this
// subset for SHIFT/SHIFT2 words; encoding it beside q avoids decoding the
// sparse six-bit architectural source field during execution.
function automatic [3:0] shift_source_predecode(input [5:0] s);
    case (s)
        SRC_SIGMA:  shift_source_predecode = 4'd1;
        SRC_DSTREG: shift_source_predecode = 4'd2;
        SRC_SRCREG: shift_source_predecode = 4'd3;
        SRC_IMM:    shift_source_predecode = 4'd4;
        SRC_TMPB:   shift_source_predecode = 4'd5;
        SRC_TMPC:   shift_source_predecode = 4'd6;
        SRC_TMPD:   shift_source_predecode = 4'd7;
        SRC_TMPE:   shift_source_predecode = 4'd8;
        SRC_OPR_R:  shift_source_predecode = 4'd9;
        SRC_COUNTR: shift_source_predecode = 4'd10;
        SRC_NEG1:   shift_source_predecode = 4'd11;
        default:    shift_source_predecode = 4'd0;
    endcase
endfunction

// Predecoded control bits 50:37, computed from the raw 37-bit word in the
// ROM output register stage (one LUT level between RAM output and q_r).
function automatic [13:0] ucode_predecode(input [36:0] w);
    logic [6:0] aluop;
    logic [5:0] buscode;
    logic [1:0] subcode;
    logic [5:0] alusrc;
    logic       mem_busop;
    aluop   = w[17:11];
    buscode = w[5:0];
    subcode = w[7:6];
    alusrc  = w[36:31];
    // Note: BUSOP_WR_D (SGDT/SIDT base store) was historically missing here
    // (and in the retired gen_ucode45.py), so the base store never issued a
    // memory request — EMM386/VCPI context saves silently lost the IDT/GDT
    // base.  Found via TC3-under-EMM386 whole-system debugging, 2026-06-12.
    mem_busop = (buscode == BUSOP_RD_OPR_WORD) ||
                (buscode == BUSOP_RD_BW) || (buscode == BUSOP_RD_D) ||
                (buscode == BUSOP_RD) || (buscode == BUSOP_RD_WORD) ||
                (buscode == BUSOP_RD_IND) || (buscode == BUSOP_WR) ||
                (buscode == BUSOP_WR_OPR) || (buscode == BUSOP_WR_OPR_WORD) ||
                (buscode == BUSOP_WR_WORD) ||
                (buscode == BUSOP_WR_D) || (buscode == BUSOP_CW);
    // bit37: ALU group op (arithmetic flags path)
    ucode_predecode[0] = (aluop == ALUJMP_ALU) || (aluop == ALUJMP_INCDEC) ||
                         (aluop == ALUJMP_CMPTST) || (aluop == ALUJMP_ADC) ||
                         (aluop == ALUJMP_CMP) || (aluop == ALUJMP_AAAAAS) ||
                         (aluop == ALUJMP_DAADAS);
    // bit38: bus op or DLY (uc_bus_or_dly)
    ucode_predecode[1] = mem_busop || (buscode == BUSOP_IACK) || (subcode == 2'b00);
    // bit39: memory bus op (uc_is_mem_busop)
    ucode_predecode[2] = mem_busop;
    // bit40: memory write
    ucode_predecode[3] = (buscode == BUSOP_WR) || (buscode == BUSOP_WR_OPR) ||
                         (buscode == BUSOP_WR_OPR_WORD) ||
                         (buscode == BUSOP_WR_WORD) || (buscode == BUSOP_WR_D);
    // bit41: check write (CW)
    ucode_predecode[4] = (buscode == BUSOP_CW);
    // bit42: word-sized access
    ucode_predecode[5] = (buscode == BUSOP_RD_OPR_WORD) ||
                         (buscode == BUSOP_RD_WORD) ||
                         (buscode == BUSOP_WR_OPR_WORD) ||
                         (buscode == BUSOP_WR_WORD);
    // bit43: descriptor/auxiliary dword access (always 32-bit regardless of
    // operand size); includes the SGDT/SIDT base store
    ucode_predecode[6] = (buscode == BUSOP_RD_D) || (buscode == BUSOP_RD_IND) ||
                         (buscode == BUSOP_WR_D);
    // bit44: JPEREQ with jump offset (coprocessor jump, always taken)
    ucode_predecode[7] = (aluop == ALUJMP_JPEREQ) && (alusrc != 6'h3F);
    // bit45: IO-capable read buscode (issues an IO read when seg_sel == SEG_IO)
    ucode_predecode[8] = (buscode == BUSOP_RD_BW) ||
                         (buscode == BUSOP_RD_WORD) ||
                         (buscode == BUSOP_RD);
    // bit46: IO-capable write buscode (issues an IO write when seg_sel == SEG_IO)
    ucode_predecode[9] = (buscode == BUSOP_WR_WORD) ||
                         (buscode == BUSOP_WR) ||
                         (buscode == BUSOP_WR_OPR_WORD) ||
                         (buscode == BUSOP_WR_OPR);
    // bit47: IACK bus cycle
    ucode_predecode[10] = (buscode == BUSOP_IACK);
    // bit48: pure DLY — waits for the bus but issues no request itself
    //        (optimistic-read grace may release this uop one cycle early)
    ucode_predecode[11] = (subcode == 2'b00) && !mem_busop && (buscode != BUSOP_IACK);
    // bit49: RPT (repeat) opcode
    ucode_predecode[12] = (w[10:8] == 3'b110);
    // bit50: WIO — wait for interrupt/IO (RPT opcode with WIO subcode)
    ucode_predecode[13] = (w[10:8] == 3'b110) && (subcode == 2'b10);
endfunction

// Dedicated copy of bits 47:39 for the paging request cone. Keeping these
// controls beside q avoids routing the wide, high-fanout ucode bus into TLB
// finalization and request selection.
function automatic [8:0] mem_ctrl_predecode(input [36:0] w);
    logic [13:0] p;
    begin
        p = ucode_predecode(w);
        mem_ctrl_predecode = p[10:2];
    end
endfunction

// Compact registered controls for architectural IND updates. The address
// unit needs only an operation class, one source qualifier, one destination
// class, and STSSAF; keep the raw source/destination/ALU/bus fields out of
// that high-fanout cone.
function automatic [8:0] ind_ctrl_predecode(input [36:0] w);
    logic [3:0] op;
    logic [2:0] dest_class;
    logic [5:0] buscode;
    logic [6:0] dest;
    begin
        buscode = w[5:0];
        dest = w[30:24];
        case (buscode)
            BUSOP_IND_PLUS_ALU: op = 4'd1;
            BUSOP_IND_ALU2:     op = 4'd2;
            BUSOP_IND_SRC:      op = 4'd3;
            BUSOP_IND_PLUS:     op = 4'd4;
            BUSOP_IN_PLUS_D:    op = 4'd5;
            BUSOP_LAR:          op = 4'd6;
            BUSOP_LLIM:         op = 4'd7;
            BUSOP_LBAS:         op = 4'd8;
            BUSOP_LPCR:         op = 4'd9;
            default:            op = 4'd0;
        endcase
        if (dest == DEST_DESSTK)
            dest_class = 3'd1;
        else if (dest == DEST_DESCOD)
            dest_class = 3'd2;
        else if ((dest == DEST_DES_ES) || (dest == DEST_DES_OS) ||
                 (dest == DEST_DES_SR))
            dest_class = 3'd3;
        else if (dest == DEST_PFERRC)
            dest_class = 3'd4;
        else if (dest == DEST_LATTTF)
            dest_class = 3'd5;
        else if (dest == DEST_PDBR)
            dest_class = 3'd6;
        else
            dest_class = 3'd0;
        ind_ctrl_predecode = {(w[17:11] == ALUJMP_STSSAF), dest_class,
                              (w[23:18] == SRC_IRF2), op};
    end
endfunction

`ifdef Z486_USE_ALTERA_UCODE_ROM
wire [39:0] q_mem_a;
wire [39:0] q_mem_b;

// Two read ports from one copy. Quartus maps it in the M10K's 4K x 2 mode:
// twenty blocks side by side, no output mux, for any depth up to 4,096.
altsyncram #(
    .operation_mode("BIDIR_DUAL_PORT"),
    .width_a(40),
    .widthad_a(12),
    .numwords_a(UCODE_ROM_DEPTH),
    .width_b(40),
    .widthad_b(12),
    .numwords_b(UCODE_ROM_DEPTH),
    .outdata_reg_a("UNREGISTERED"),
    .outdata_reg_b("UNREGISTERED"),
    .address_reg_b("CLOCK1"),
    .indata_reg_b("CLOCK1"),
    .wrcontrol_wraddress_reg_b("CLOCK1"),
    .address_aclr_a("NONE"),
    .address_aclr_b("NONE"),
    .outdata_aclr_a("NONE"),
    .outdata_aclr_b("NONE"),
    .read_during_write_mode_mixed_ports("DONT_CARE"),
    .power_up_uninitialized("FALSE"),
    .init_file("ucode.mif"),
    .ram_block_type("M10K"),
    .intended_device_family("Cyclone V"),
    .lpm_type("altsyncram")
) microcode_rom_altsyncram (
    .address_a(addr),
    .address_b(addr_b),
    .clock0(clk),
    .clock1(clk),
    .clocken0(addr_ce),
    .clocken1(addr_b_ce),
    .data_a(40'd0),
    .data_b(40'd0),
    .wren_a(1'b0),
    .wren_b(1'b0),
    .q_a(q_mem_a),
    .q_b(q_mem_b),
    .aclr0(1'b0),
    .aclr1(1'b0),
    .addressstall_a(1'b0),
    .addressstall_b(1'b0),
    .clocken2(1'b1),
    .clocken3(1'b1),
    .rden_a(1'b1),
    .rden_b(1'b1),
    .byteena_a(1'b1),
    .byteena_b(1'b1),
    .eccstatus()
);
`else
`ifdef Z486_USE_LOGIC_UCODE_ROM
(* ramstyle = "logic" *) reg [39:0] microcode_rom [0:UCODE_ROM_DEPTH-1];
`else
`Z486_BLOCK_RAM reg [39:0] microcode_rom [0:UCODE_ROM_DEPTH-1];
`endif
reg [39:0] q_mem_a;
reg [39:0] q_mem_b;

initial begin
    $readmemh(INIT_HEX, microcode_rom);
end

always_ff @(posedge clk) begin
    if (addr_ce)
        q_mem_a <= microcode_rom[addr];
    if (addr_b_ce)
        q_mem_b <= microcode_rom[addr_b];
end
`endif

// The executing word comes from port A, or from port B when the issuing
// instruction's first word was read ahead there. The select is registered, so
// the mux adds no late control to the ROM-output registers.
wire [39:0] q_mem = q_sel_b ? q_mem_b : q_mem_a;
reg  [50:0] q_r;

always_ff @(posedge clk) begin
    if (q_ce) begin
        q_r <= {ucode_predecode(q_mem[36:0]), q_mem[36:0]};
        q_shift_source_r <= q_mem[23:18];
        q_shift_source_class_r <= shift_source_predecode(q_mem[23:18]);
        q_shift2_source_r <= shift2_source_predecode(q_mem[23:18]);
        q_is_shift2_r <= (q_mem[17:11] == ALUJMP_SHIFT2);
        q_shift_uc_carry_r <= shift_uc_carry_predecode(q_mem[36:0]);
        q_shift_alu_src_r <= q_mem[36:31];
        q_shift_aluop_r <= q_mem[17:11];
        q_shift_sigma_sel_r <= shift_sigma_predecode(q_mem[17:11]);
        q_alu_op_sel_r <= alu_op_predecode(q_mem[17:11]);
        q_dly_source_r <= dly_source_predecode(q_mem[23:18]);
        q_mem_ctrl_r <= mem_ctrl_predecode(q_mem[36:0]);
        q_ind_ctrl_r <= ind_ctrl_predecode(q_mem[36:0]);
        q_fpu_f8_r <= fpu_f8_predecode(q_mem[36:0]);
    end
end

assign q = q_r;
assign q_early = {ucode_predecode(q_mem[36:0]), q_mem[36:0]};
assign q_kind_early = q_mem_a[39:37];
assign q_kind_b = q_mem_b[39:37];

assign q_shift_source = q_shift_source_r;
assign q_shift_source_class = q_shift_source_class_r;
assign q_shift2_source = q_shift2_source_r;
assign q_is_shift2 = q_is_shift2_r;
assign q_shift_uc_carry = q_shift_uc_carry_r;
assign q_shift_alu_src = q_shift_alu_src_r;
assign q_shift_aluop = q_shift_aluop_r;
assign q_shift_sigma_sel = q_shift_sigma_sel_r;
assign q_alu_op_sel = q_alu_op_sel_r;
assign q_dly_source = q_dly_source_r;
assign q_mem_ctrl = q_mem_ctrl_r;
assign q_ind_ctrl = q_ind_ctrl_r;
assign q_fpu_f8 = q_fpu_f8_r;

// synthesis translate_off
always_ff @(posedge clk)
    if ((^q[36:0] !== 1'bx) &&
        (q_ind_ctrl !== ind_ctrl_predecode(q[36:0])))
        $fatal(1, "IND CONTROL PREDECODE MISMATCH");
always_ff @(posedge clk)
    if ((^q[36:0] !== 1'bx) &&
        (q_shift_sigma_sel !== shift_sigma_predecode(q[17:11])))
        $fatal(1, "SHIFT SIGMA PREDECODE MISMATCH");
always_ff @(posedge clk)
    if ((^q[36:0] !== 1'bx) &&
        (q_alu_op_sel !== alu_op_predecode(q[17:11])))
        $fatal(1, "ALU OP PREDECODE MISMATCH: %h vs %h (field %h)", q_alu_op_sel, alu_op_predecode(q[17:11]), q[17:11]);
// synthesis translate_on

endmodule
