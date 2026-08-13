// x87 data interface and control unit: command decode, microsequencing,
// environment/status state, CPU transfers, and the 8-entry register stack.
// The Intel 80387 block diagram places the stack inside the FPU; keeping its
// RAM here lets stack and transfer control share one owner. Arithmetic is
// delegated to x87_executor.
module x87_control
    import x87_pkg::*, x87_ucode_pkg::*;
(
    input  logic        clk,                    // Core/system clock.
    input  logic        reset,                  // Synchronous active-high reset.

    input  logic        cmd_valid,              // CPU presents an x87 command.
    input  logic [10:0] cmd_fop,                // Encoded ESC opcode and ModR/M.
    output logic        cmd_ready,              // Core can accept cmd_fop now.

    input  logic        direct_m32_valid,       // CPU posts a fault-checked m32 operand.
    input  logic [10:0] direct_m32_fop,         // Memory-form FOP for direct_m32_data.
    input  logic [31:0] direct_m32_data,        // Operand returned by demand memory.
    output logic        direct_m32_ready,       // One-entry direct command queue is available.

    input  logic        word_in_valid,          // Operand-transfer word is valid.
    input  logic  [3:0] word_in_be,             // Valid bytes in word_in_data.
    input  logic [31:0] word_in_data,           // CPU-to-x87 transfer data.
    output logic        word_in_ready,          // Core can accept a transfer word.

    input  logic        read_req_valid,         // CPU requests status or result data.
    input  logic        read_req_data_port,     // 1=result data; 0=status word.
    input  logic  [3:0] read_req_be,            // Requested result-data byte lanes.
    output logic        read_req_ready,         // Core can accept the read request.
    output logic        read_resp_valid,        // read_resp_data is valid.
    output logic [31:0] read_resp_data,         // x87-to-CPU status/result data.

    output logic        busy_n,                 // Active-low arithmetic busy status.
    output logic        pereq,                  // CPU operand-transfer/completion request.
    output logic        error_n,                // Active-low unmasked-error status.
    output logic        queue_safe,             // Masked exceptions permit one queued command.

    // Low-rate hardware diagnostics; sampled only by the MiSTer watchdog.
    output logic [31:0] debug_state             // Packed command/executor progress.
);

typedef enum logic [3:0] {
    RX_NONE,
    RX_CONTROL,
    RX_M32,
    RX_M64,
    RX_M80,
    RX_I16,
    RX_I32,
    RX_I64,
    RX_ENV,
    RX_STATE
} rx_kind_t;

typedef enum logic [1:0] {
    TX_NONE,
    TX_VALUE,
    TX_ENV,
    TX_STATE
} tx_kind_t;

typedef enum logic [1:0] {
    EXEC_NONE,
    EXEC_LOAD,
    EXEC_STORE,
    EXEC_MATH
} convert_owner_t;

logic [2:0] top;
logic [15:0] control_word;
logic [15:0] status_flags;
logic [10:0] last_fop;
logic [15:0] tag_word;
logic        command_pending;
logic [10:0] command_fop;
x87_command_decode_t command_decode;
logic        direct_m32_pending;
logic [10:0] direct_m32_fop_r;
logic [31:0] direct_m32_data_r;

logic [2:0] stack_addr_a;
logic [2:0] stack_addr_b;
logic [2:0] stack_port_addr_a;
logic [2:0] stack_port_addr_b;
logic       stack_write_a;
logic       stack_write_b;
logic [79:0] stack_write_data_a;
logic [79:0] stack_write_data_b;
logic [79:0] stack_read_raw_a;
logic [79:0] stack_read_raw_b;
x87_reg_t    stack_read_data_a;
x87_reg_t    stack_read_data_b;

rx_kind_t rx_kind;
logic [4:0] rx_index;
logic [79:0] rx_payload;
logic [3:0]  rx_byte_count;
logic [159:0] rx_state_shift;

logic [1:0] tx_count;
logic [3:0] tx_last_be;
tx_kind_t tx_kind;
logic [4:0] tx_index;
logic [159:0] tx_state_shift;
logic         tx_generation_done;
logic [2:0]   tx_byte_offset;
logic         transfer_push_valid;
logic [35:0]  transfer_push_data;
logic         transfer_push_ready;
logic         transfer_pop_valid;
logic [35:0]  transfer_pop_data;
logic         transfer_pop_ready;
logic [1:0]   transfer_count;
logic         tx_produce_valid;
logic [35:0]  tx_produce_data;
logic         tx_produce_fire;
logic         tx_consume_fire;
logic         status_read_pending;
logic         command_complete_pulse;
logic [1:0]   pereq_release_hold;
logic         push_pending;
logic [79:0]  push_pending_raw;
logic [1:0]   push_pending_tag;
x87_reg_t     push_pending_value;
logic         memory_math_pending;
logic         memory_math_mul;
logic         memory_math_div;
logic         memory_math_compare;
logic         memory_math_subtract;
logic         memory_math_reverse;
logic         memory_math_pop;
logic         store_pending;
logic         store_integer;
logic [1:0]   store_width;
logic         store_pop;
logic         store_source_empty;
x87_reg_t     store_source;
logic [79:0]  store_source_raw;
logic         pop_pending;
logic         result_write_pending;
logic [2:0]   result_write_index;
logic [79:0]  result_write_raw;
logic                   v2_exec_pending;
x87_exec_op_t     v2_exec_op;
convert_owner_t         v2_exec_owner;
logic             [1:0] v2_exec_size;
logic            [63:0] v2_exec_transfer;
logic                   v2_exec_busy;
logic                   v2_exec_done;
logic             [2:0] v2_exec_commit;
x87_reg_t               v2_exec_result;
x87_reg_t               v2_exec_auxiliary_result;
logic            [63:0] v2_exec_transfer_out;
logic                   v2_exec_invalid;
logic                   v2_exec_inexact;
logic                   v2_exec_divide_by_zero;
logic                   v2_exec_overflow;
logic                   v2_exec_underflow;
logic                   v2_exec_denormal_operand;
logic                   v2_exec_range_incomplete;
logic                   v2_exec_rounded_up;
logic                   v2_compare_unordered;
logic                   v2_compare_less;
logic                   v2_compare_equal;

logic         arith_compare;
logic         arith_quiet_compare;
logic         arith_write_result;
logic [1:0]   arith_pop_count;
logic [2:0]   arith_dest_index;
x87_reg_t     arith_operand_a;
x87_reg_t     arith_operand_b;
logic         trans_cosine;
logic         trans_tangent_pair;
logic         trans_atan2;
logic         fptan_trans_pending;
logic         fptan_div_pending;
logic         fptan_push_after_result;
logic   [7:0] v2_exec_uaddr;

wire [15:0] status_word = {status_flags[15:14], top, status_flags[10:0]};
wire [2:0] st0_index = top;
wire [2:0] cmd_st_index = top + command_fop[2:0];
assign debug_state = {
    busy_n, cmd_ready, command_pending, v2_exec_busy,
    v2_exec_done, v2_exec_pending, v2_exec_owner,
    v2_exec_uaddr, last_fop, rx_kind, (tx_kind != TX_NONE)
};
assign stack_read_data_a = x87_from_m80(stack_read_raw_a);
assign stack_read_data_b = x87_from_m80(stack_read_raw_b);
assign push_pending_value = x87_from_m80(push_pending_raw);
wire command_is_arithmetic = command_decode.arithmetic;
wire v2_exec_math_done = v2_exec_done &&
                            (v2_exec_owner == EXEC_MATH);
wire trans_done = v2_exec_math_done &&
                  (v2_exec_op == X87_ARITH_TRANS);
wire math_done = v2_exec_math_done &&
                 !(trans_done && fptan_trans_pending);
wire math_invalid = v2_exec_invalid;
wire math_inexact = v2_exec_inexact;
wire [74:0] math_result = v2_exec_result;
wire math_divide_by_zero = v2_exec_math_done &&
                           v2_exec_divide_by_zero;
wire math_overflow = v2_exec_math_done && v2_exec_overflow;
wire math_underflow = v2_exec_math_done && v2_exec_underflow;
wire math_denormal_operand = v2_exec_math_done &&
                              v2_exec_denormal_operand;
wire math_range_incomplete = trans_done &&
                             v2_exec_range_incomplete;
wire math_unmasked_exception =
    (math_invalid && !control_word[0]) ||
    (math_denormal_operand && !control_word[1]) ||
    (math_divide_by_zero && !control_word[2]) ||
    (math_overflow && !control_word[3]) ||
    (math_underflow && !control_word[4]) ||
    (math_inexact && !control_word[5]);

function automatic logic fop_reads_status(input logic [10:0] fop);
    return (fop == 11'h7e0) ||
           ((fop[10:8] == 3'd5) && (fop[7:6] != 2'b11) &&
            (fop[5:3] == 3'b111));
endfunction

function automatic logic [2:0] transfer_byte_count(input logic [3:0] be);
    return {2'b00, be[0]} + {2'b00, be[1]} +
           {2'b00, be[2]} + {2'b00, be[3]};
endfunction

function automatic logic [79:0] append_transfer_bytes(
    input logic [79:0] payload,
    input logic [3:0]  byte_offset,
    input logic [31:0] data,
    input logic [3:0]  be
);
    logic [79:0] merged;
    integer lane;
    integer target;
    begin
        merged = payload;
        target = byte_offset;
        for (lane = 0; lane < 4; lane = lane + 1) begin
            if (be[lane] && (target < 10)) begin
                merged[target*8 +: 8] = data[lane*8 +: 8];
                target = target + 1;
            end
        end
        return merged;
    end
endfunction

function automatic logic [31:0] select_transfer_bytes(
    input logic [31:0] data,
    input logic [2:0]  byte_offset,
    input logic [3:0]  be
);
    logic [31:0] selected;
    integer lane;
    integer source;
    begin
        selected = 32'h0;
        source = byte_offset;
        for (lane = 0; lane < 4; lane = lane + 1) begin
            if (be[lane] && (source < 4)) begin
                selected[lane*8 +: 8] = data[source*8 +: 8];
                source = source + 1;
            end
        end
        return selected;
    end
endfunction

// Memory FOPs encode the complete ModR/M byte, but addressing-mode and r/m
// bits are not part of the x87 operation. Preserve register forms verbatim
// and canonicalize memory forms to opcode plus ModR/M.reg for command decode.
function automatic logic [10:0] fop_decode_key(input logic [10:0] fop);
    if (fop[7:6] == 2'b11)
        return fop;
    return {fop[10:8], 2'b00, fop[5:3], 3'b000};
endfunction

function automatic logic m32_is_normal(input logic [31:0] raw);
    return (raw[30:23] != 8'h00) && (raw[30:23] != 8'hff);
endfunction

function automatic logic [79:0] normal_m32_to_m80(input logic [31:0] raw);
    logic [14:0] extended_exp;
    begin
        extended_exp = {7'h0, raw[30:23]} + 15'd16256;
        return {raw[31], extended_exp, 1'b1, raw[22:0], 40'h0};
    end
endfunction


function automatic logic [15:0] architectural_tag_word();
    return tag_word;
endfunction

function automatic logic stack_empty(input logic [2:0] index);
    return tag_word[index*2 +: 2] == 2'b11;
endfunction

function automatic logic [1:0] stack_tag_from_m80(input logic [79:0] value);
    logic [14:0] exponent;
    logic [63:0] significand;
    begin
        exponent = value[78:64];
        significand = value[63:0];
        if ((exponent == 0) && (significand == 0))
            return 2'b01; // Zero.
        if ((exponent != 0) && (exponent != 15'h7fff) && significand[63])
            return 2'b00; // Valid finite value.
        return 2'b10;     // Denormal, infinity, NaN, or unsupported encoding.
    end
endfunction

function automatic logic [159:0] pack_state_pair(
    input logic [79:0] even_value,
    input logic [79:0] odd_value,
    input logic [2:0] even_index
);
    logic [79:0] saved_even;
    logic [79:0] saved_odd;
    begin
        saved_even = stack_empty(even_index)
                   ? x87_to_m80(x87_empty()) : even_value;
        saved_odd = stack_empty(even_index + 3'd1)
                  ? x87_to_m80(x87_empty()) : odd_value;
        return {saved_odd, saved_even};
    end
endfunction

task automatic clear_stack;
    begin
        tag_word <= 16'hffff;
    end
endtask

task automatic write_stack(input logic [2:0] index, input x87_reg_t value);
    begin
        stack_addr_a <= index;
        stack_write_data_a <= x87_to_m80(value);
        stack_write_a <= 1'b1;
        tag_word[index*2 +: 2] <= x87_tag(value);
    end
endtask

task automatic write_stack_raw(
    input logic [2:0] index,
    input logic [79:0] value
);
    begin
        stack_addr_a <= index;
        stack_write_data_a <= value;
        stack_write_a <= 1'b1;
        tag_word[index*2 +: 2] <= stack_tag_from_m80(value);
    end
endtask

task automatic write_stack_raw_tagged(
    input logic [2:0] index,
    input logic [79:0] value,
    input logic [1:0] value_tag
);
    begin
        stack_addr_a <= index;
        stack_write_data_a <= value;
        stack_write_a <= 1'b1;
        tag_word[index*2 +: 2] <= value_tag;
    end
endtask

task automatic raise_stack_fault(input logic overflow);
    begin
        status_flags[0] <= 1'b1; // IE
        status_flags[6] <= 1'b1; // SF
        status_flags[9] <= overflow; // C1 distinguishes overflow/underflow
        if (!control_word[0]) begin
            status_flags[7] <= 1'b1; // ES: unmasked exception summary
            status_flags[15] <= 1'b1; // B
        end
    end
endtask

task automatic raise_invalid;
    begin
        status_flags[0] <= 1'b1;
        if (!control_word[0]) begin
            status_flags[7] <= 1'b1;
            status_flags[15] <= 1'b1;
        end
    end
endtask

task automatic push_raw_tagged(
    input logic [79:0] value,
    input logic [1:0] value_tag
);
    logic [2:0] new_top;
    begin
        new_top = top - 3'd1;
        if (stack_empty(new_top)) begin
            top <= new_top;
            write_stack_raw_tagged(new_top, value, value_tag);
            status_flags[9] <= 1'b0;
        end else begin
            raise_stack_fault(1'b1);
            if (control_word[0]) begin
                top <= new_top;
                write_stack(new_top, x87_indefinite());
            end
        end
    end
endtask

task automatic push_value(input x87_reg_t value);
    begin
        push_raw_tagged(x87_to_m80(value), x87_tag(value));
    end
endtask

task automatic schedule_push(input x87_reg_t value);
    begin
        push_pending_raw <= x87_to_m80(value);
        push_pending_tag <= x87_tag(value);
        push_pending <= 1'b1;
        command_complete_pulse <= 1'b0;
    end
endtask

task automatic schedule_push_raw(input logic [79:0] value);
    begin
        push_pending_raw <= value;
        push_pending_tag <= stack_tag_from_m80(value);
        push_pending <= 1'b1;
        command_complete_pulse <= 1'b0;
    end
endtask

task automatic pop_value;
    begin
        tag_word[top*2 +: 2] <= 2'b11;
        top <= top + 3'd1;
    end
endtask

task automatic start_store(
    input logic integer_store,
    input logic [1:0] width,
    input logic pop
);
    logic source_empty;
    begin
        source_empty = stack_empty(st0_index);
        store_pending <= 1'b1;
        store_integer <= integer_store;
        store_width <= width;
        store_pop <= pop;
        store_source_empty <= source_empty;
        // Store commands request CPU reads only after converted words enter
        // the output FIFO.
        command_complete_pulse <= 1'b0;
        if (source_empty) begin
            raise_stack_fault(1'b0);
            store_source <= x87_indefinite();
            store_source_raw <= x87_to_m80(x87_indefinite());
        end else begin
            store_source <= stack_read_data_a;
            store_source_raw <= stack_read_raw_a;
            status_flags[9] <= 1'b0;
        end
    end
endtask

task automatic accept_environment_word(
    input logic [2:0] word_index,
    input logic [15:0] value
);
    begin
        case (word_index)
            3'd0: control_word <= value;
            3'd1: begin
                status_flags <= value;
                top <= value[13:11];
            end
            3'd2: tag_word <= value;
            default: ; // Instruction and operand pointers live in the 80386.
        endcase
    end
endtask

task automatic commit_state_pair(
    input logic [1:0] pair_index,
    input logic [31:0] final_word
);
    logic [159:0] completed_pair;
    logic [2:0] even_index;
    logic [2:0] odd_index;
    begin
        completed_pair = {final_word, rx_state_shift[127:0]};
        even_index = top + {pair_index, 1'b0};
        odd_index = even_index + 3'd1;
        stack_addr_a <= even_index;
        stack_write_data_a <= completed_pair[79:0];
        stack_write_a <= 1'b1;
        stack_addr_b <= odd_index;
        stack_write_data_b <= completed_pair[159:80];
        stack_write_b <= 1'b1;
    end
endtask

// RPTI may replay a memory command after the x87 has accepted it but before
// the CPU transfers the first operand word. Re-arming that exact empty receive
// transaction is idempotent; unrelated commands remain blocked.
wire core_cmd_direct = direct_m32_pending;
wire core_cmd_valid = core_cmd_direct || cmd_valid;
wire [10:0] core_cmd_fop = core_cmd_direct ? direct_m32_fop_r : cmd_fop;
wire restartable_rx_command = !core_cmd_direct && (rx_kind != RX_NONE) &&
                              (tx_kind == TX_NONE) &&
                              (transfer_count == 2'd0) &&
                              (core_cmd_fop == last_fop);
wire core_cmd_ready = ((rx_kind == RX_NONE) || restartable_rx_command) &&
                   (tx_kind == TX_NONE) &&
                   (transfer_count == 2'd0) &&
                   !command_pending && !stack_write_a && !stack_write_b &&
                   !push_pending &&
                   (!memory_math_pending || restartable_rx_command) &&
                   !store_pending && !pop_pending &&
                   !result_write_pending &&
                   !v2_exec_pending &&
                   !v2_exec_busy && !v2_exec_done &&
                   !fptan_trans_pending && !fptan_div_pending &&
                   !fptan_push_after_result && !read_resp_valid;
// Direct memory commands remain ordered ahead of later bridge commands. Only
// masked-exception mode permits capture while a predecessor is still active.
assign direct_m32_ready = !direct_m32_pending && !cmd_valid && queue_safe;
assign cmd_ready = !direct_m32_pending && core_cmd_ready;
assign word_in_ready = (rx_kind != RX_NONE) && transfer_push_ready;
assign read_req_ready = !read_resp_valid &&
                        (!read_req_data_port ||
                         ((tx_kind != TX_NONE) && transfer_pop_valid));
wire [2:0] rx_fragment_bytes = transfer_byte_count(transfer_pop_data[35:32]);
wire [3:0] rx_byte_count_next = rx_byte_count + rx_fragment_bytes;
wire [79:0] rx_payload_next = append_transfer_bytes(
    rx_payload, rx_byte_count, transfer_pop_data[31:0],
    transfer_pop_data[35:32]);
wire [2:0] tx_entry_bytes = transfer_byte_count(transfer_pop_data[35:32]);
wire [2:0] tx_request_bytes = transfer_byte_count(read_req_be);
wire tx_entry_consumed = tx_byte_offset + tx_request_bytes >= tx_entry_bytes;
wire v2_exec_start = v2_exec_pending && !v2_exec_busy;
assign busy_n = !(((v2_exec_owner == EXEC_MATH) &&
                   (v2_exec_pending || v2_exec_busy)) ||
                  (command_pending && command_is_arithmetic));
// PEREQ releases the 80386 coprocessor-wait microcode as well as requesting
// operand transfers. Keep it asserted throughout an accepted command because
// a one-cycle completion pulse can precede the CPU's CORWAIT sample.
assign pereq = (rx_kind != RX_NONE) ||
               ((tx_kind != TX_NONE) && transfer_pop_valid) ||
               direct_m32_pending || command_pending ||
               stack_write_a || stack_write_b ||
               push_pending || memory_math_pending ||
               store_pending || pop_pending || result_write_pending ||
               v2_exec_pending || v2_exec_busy || v2_exec_done ||
               fptan_trans_pending || fptan_div_pending ||
               fptan_push_after_result ||
               status_read_pending || command_complete_pulse ||
               (pereq_release_hold != 2'b00);
assign error_n = !status_flags[7];
assign queue_safe = &control_word[5:0] && error_n;

// One physical three-word queue serves both protocol directions. A command
// cannot change direction until the queue is empty.
always_comb begin
    tx_produce_valid = (tx_kind != TX_NONE) && !tx_generation_done;
    tx_produce_data = 36'h0;
    case (tx_kind)
        TX_VALUE: begin
            tx_produce_data[31:0] = tx_state_shift[31:0];
            tx_produce_data[35:32] =
                (tx_index + 5'd1 == {3'h0, tx_count}) ? tx_last_be : 4'hf;
        end
        TX_ENV, TX_STATE: begin
            // The environment fields are architecturally 16 bits even when
            // the 32-bit format leaves two padding bytes after each field.
            tx_produce_data[35:32] = (tx_index < 5'd7) ? 4'h3 : 4'hf;
            if (tx_index == 5'd0)
                tx_produce_data[31:0] = {16'h0, control_word};
            else if (tx_index == 5'd1)
                tx_produce_data[31:0] = {16'h0, status_word};
            else if (tx_index == 5'd2)
                tx_produce_data[31:0] =
                    {16'h0, architectural_tag_word()};
            else if (tx_index < 5'd7)
                tx_produce_data[31:0] = 32'h0;
            else
                tx_produce_data[31:0] = tx_state_shift[31:0];
        end
        default: ;
    endcase

    if (core_cmd_direct && core_cmd_valid && core_cmd_ready) begin
        transfer_push_valid = 1'b1;
        transfer_push_data = {4'hf, direct_m32_data_r};
    end else begin
        transfer_push_valid = (rx_kind != RX_NONE)
                            ? word_in_valid : tx_produce_valid;
        transfer_push_data = (rx_kind != RX_NONE)
                           ? {word_in_be, word_in_data} : tx_produce_data;
    end
    transfer_pop_ready = (rx_kind != RX_NONE) ? 1'b1
                       : ((tx_kind != TX_NONE) && read_req_valid &&
                          read_req_ready && read_req_data_port &&
                          tx_entry_consumed);
end

assign tx_produce_fire = (tx_kind != TX_NONE) &&
                         transfer_push_valid && transfer_push_ready;
assign tx_consume_fire = (tx_kind != TX_NONE) &&
                         transfer_pop_valid && transfer_pop_ready;

// Command decode and stack operands are synchronous and return together one
// cycle after command acceptance. This keeps FOP classification out of the
// command-dispatch datapath without adding a protocol cycle.
x87_command_rom command_rom (
    .clk(clk),
    .address(core_cmd_fop),
    .decode(command_decode)
);

x87_transfer_fifo transfer_fifo (
    .clk(clk),
    .reset(reset),
    .clear(1'b0),
    .push_valid(transfer_push_valid),
    .push_data(transfer_push_data),
    .push_ready(transfer_push_ready),
    .pop_valid(transfer_pop_valid),
    .pop_data(transfer_pop_data),
    .pop_ready(transfer_pop_ready),
    .count(transfer_count)
);

// A newly accepted command directly addresses the synchronous read ports. The
// registered addresses retain that selection for writes and multiword state.
always_comb begin
    stack_port_addr_a = stack_addr_a;
    stack_port_addr_b = stack_addr_b;
    if (core_cmd_valid && core_cmd_ready) begin
        stack_port_addr_a = top;
        stack_port_addr_b = ((fop_decode_key(core_cmd_fop) == 11'h530) ||
                             (core_cmd_fop == 11'h1f3))
                          ? top + 3'd1 : top + core_cmd_fop[2:0];
    end
end

x87_stack_mem stack_mem (
    .clk(clk),
    .addr_a(stack_port_addr_a),
    .write_a(stack_write_a),
    .write_data_a(stack_write_data_a),
    .read_data_a(stack_read_raw_a),
    .addr_b(stack_port_addr_b),
    .write_b(stack_write_b),
    .write_data_b(stack_write_data_b),
    .read_data_b(stack_read_raw_b)
);

x87_executor executor (
    .clk(clk),
    .reset(reset),
    .start(v2_exec_start),
    .exec_op(v2_exec_op),
    .integer_size(v2_exec_size),
    .precision_control(control_word[9:8]),
    .rounding_mode(control_word[11:10]),
    .quiet_compare(arith_quiet_compare),
    .trans_cosine(trans_cosine),
    .trans_tangent_pair(trans_tangent_pair),
    .trans_atan2(trans_atan2),
    .operand(arith_operand_a),
    .operand_b(arith_operand_b),
    .transfer_in(v2_exec_transfer),
    .busy(v2_exec_busy),
    .done(v2_exec_done),
    .commit_action(v2_exec_commit),
    .result(v2_exec_result),
    .auxiliary_result(v2_exec_auxiliary_result),
    .transfer_out(v2_exec_transfer_out),
    .invalid(v2_exec_invalid),
    .inexact(v2_exec_inexact),
    .divide_by_zero(v2_exec_divide_by_zero),
    .overflow(v2_exec_overflow),
    .underflow(v2_exec_underflow),
    .denormal_operand(v2_exec_denormal_operand),
    .range_incomplete(v2_exec_range_incomplete),
    .rounded_up(v2_exec_rounded_up),
    .compare_unordered(v2_compare_unordered),
    .compare_less(v2_compare_less),
    .compare_equal(v2_compare_equal),
    .debug_uaddr(v2_exec_uaddr)
);

always_ff @(posedge clk) begin
    if (reset) begin
        control_word <= 16'h037f;
        status_flags <= 16'h0000;
        top <= 3'd0;
        last_fop <= 11'h000;
        command_pending <= 1'b0;
        command_fop <= 11'h000;
        direct_m32_pending <= 1'b0;
        direct_m32_fop_r <= 11'h000;
        direct_m32_data_r <= 32'h0;
        stack_addr_a <= 3'd0;
        stack_addr_b <= 3'd1;
        stack_write_a <= 1'b0;
        stack_write_b <= 1'b0;
        stack_write_data_a <= 80'h0;
        stack_write_data_b <= 80'h0;
        rx_kind <= RX_NONE;
        rx_index <= 5'd0;
        rx_payload <= 80'h0;
        rx_byte_count <= 4'd0;
        rx_state_shift <= '0;
        tx_kind <= TX_NONE;
        tx_count <= 2'd0;
        tx_last_be <= 4'hf;
        tx_index <= 5'd0;
        tx_generation_done <= 1'b0;
        tx_byte_offset <= 3'd0;
        tx_state_shift <= '0;
        status_read_pending <= 1'b0;
        command_complete_pulse <= 1'b0;
        pereq_release_hold <= 2'b00;
        push_pending <= 1'b0;
        push_pending_raw <= 80'h0;
        push_pending_tag <= 2'b11;
        memory_math_pending <= 1'b0;
        memory_math_mul <= 1'b0;
        memory_math_div <= 1'b0;
        memory_math_compare <= 1'b0;
        memory_math_subtract <= 1'b0;
        memory_math_reverse <= 1'b0;
        memory_math_pop <= 1'b0;
        store_pending <= 1'b0;
        store_integer <= 1'b0;
        store_width <= 2'd0;
        store_pop <= 1'b0;
        store_source_empty <= 1'b0;
        store_source <= x87_empty();
        store_source_raw <= 80'h0;
        pop_pending <= 1'b0;
        result_write_pending <= 1'b0;
        result_write_index <= 3'd0;
        result_write_raw <= 80'h0;
        v2_exec_pending <= 1'b0;
        v2_exec_op <= X87_CONVERT_FLD_M32;
        v2_exec_owner <= EXEC_NONE;
        v2_exec_size <= 2'd0;
        v2_exec_transfer <= 64'h0;
        arith_compare <= 1'b0;
        arith_quiet_compare <= 1'b0;
        arith_write_result <= 1'b0;
        arith_pop_count <= 2'd0;
        arith_dest_index <= 3'd0;
        arith_operand_a <= x87_empty();
        arith_operand_b <= x87_empty();
        trans_cosine <= 1'b0;
        trans_tangent_pair <= 1'b0;
        trans_atan2 <= 1'b0;
        fptan_trans_pending <= 1'b0;
        fptan_div_pending <= 1'b0;
        fptan_push_after_result <= 1'b0;
        read_resp_valid <= 1'b0;
        read_resp_data <= 32'h0;
        clear_stack();
    end else begin
        read_resp_valid <= 1'b0;
        command_complete_pulse <= 1'b0;
        stack_write_a <= 1'b0;
        stack_write_b <= 1'b0;
        pereq_release_hold <= {1'b0, pereq_release_hold[1]};

        if (direct_m32_valid && direct_m32_ready) begin
            direct_m32_pending <= 1'b1;
            direct_m32_fop_r <= direct_m32_fop;
            direct_m32_data_r <= direct_m32_data;
        end

        // Transfer conversion and TOP-dependent stack selection are separate
        // cycles. PEREQ keeps the CPU stalled until this commit completes.
        if (push_pending) begin
            if (memory_math_pending) begin
                memory_math_pending <= 1'b0;
                if (stack_empty(st0_index)) begin
                    raise_stack_fault(1'b0);
                    if (memory_math_compare) begin
                        status_flags[14] <= 1'b1;
                        status_flags[10] <= 1'b1;
                        status_flags[8] <= 1'b1;
                    end
                    if (control_word[0]) begin
                        if (!memory_math_compare)
                            write_stack(st0_index, x87_indefinite());
                        if (memory_math_pop)
                            pop_value();
                    end
                end else begin
                    command_complete_pulse <= 1'b0;
                    arith_compare <= memory_math_compare;
                    arith_quiet_compare <= 1'b0;
                    arith_write_result <= !memory_math_compare;
                    arith_pop_count <= {1'b0, memory_math_pop};
                    arith_dest_index <= st0_index;
                    arith_operand_a <= memory_math_reverse
                                     ? push_pending_value : stack_read_data_a;
                    arith_operand_b <= memory_math_reverse
                                     ? stack_read_data_a : push_pending_value;
                    v2_exec_op <= memory_math_div
                        ? X87_ARITH_DIV
                        : memory_math_mul
                        ? X87_ARITH_MUL
                        : memory_math_compare
                        ? X87_ARITH_COMPARE
                        : memory_math_subtract ? X87_ARITH_SUB
                                               : X87_ARITH_ADD;
                    v2_exec_owner <= EXEC_MATH;
                    v2_exec_size <= 2'd0;
                    v2_exec_transfer <= 64'h0;
                    v2_exec_pending <= 1'b1;
                end
            end else begin
                push_raw_tagged(push_pending_raw, push_pending_tag);
            end
            push_pending <= 1'b0;
        end

        if (store_pending) begin
            if (!store_integer && (store_width == 2'd2)) begin
                logic [79:0] store_m80_value;
                store_m80_value = store_source_raw;
                tx_state_shift <= {80'h0, store_m80_value};
                tx_count <= 2'd3;
                tx_last_be <= 4'h3;
                tx_index <= 5'd0;
                tx_kind <= TX_VALUE;
                tx_generation_done <= 1'b0;
                if (store_pop && (!store_source_empty || control_word[0]))
                    pop_pending <= 1'b1;
            end else begin
                v2_exec_op <= store_integer ? X87_CONVERT_FIST
                              : (store_width == 2'd0)
                              ? X87_CONVERT_FST_M32
                              : X87_CONVERT_FST_M64;
                v2_exec_owner <= EXEC_STORE;
                v2_exec_size <= store_width;
                arith_operand_a <= store_source;
                v2_exec_transfer <= 64'h0;
                v2_exec_pending <= 1'b1;
            end
            store_pending <= 1'b0;
        end

        if (v2_exec_start) begin
            v2_exec_pending <= 1'b0;
        end

        if (v2_exec_done) begin
            case (v2_exec_commit)
                X87_COMMIT_PUSH: begin
                    push_pending_raw <= x87_to_m80(v2_exec_result);
                    push_pending_tag <= x87_tag(v2_exec_result);
                    push_pending <= 1'b1;
                    if (v2_exec_invalid)
                        raise_invalid();
                end
                X87_COMMIT_TRANSFER: begin
                    tx_state_shift <= {96'h0, v2_exec_transfer_out};
                    tx_count <= ((store_integer && (store_width == 2'd2)) ||
                                 (!store_integer && (store_width == 2'd1)))
                              ? 2'd2 : 2'd1;
                    tx_last_be <= (store_integer && (store_width == 2'd0))
                                ? 4'h3 : 4'hf;
                    tx_index <= 5'd0;
                    tx_kind <= TX_VALUE;
                    tx_generation_done <= 1'b0;
                    if (v2_exec_invalid)
                        raise_invalid();
                    if (v2_exec_overflow) begin
                        status_flags[3] <= 1'b1;
                        if (!control_word[3]) begin
                            status_flags[7] <= 1'b1;
                            status_flags[15] <= 1'b1;
                        end
                    end
                    if (v2_exec_underflow) begin
                        status_flags[4] <= 1'b1;
                        if (!control_word[4]) begin
                            status_flags[7] <= 1'b1;
                            status_flags[15] <= 1'b1;
                        end
                    end
                    if (v2_exec_inexact) begin
                        status_flags[5] <= 1'b1;
                        if (!control_word[5]) begin
                            status_flags[7] <= 1'b1;
                            status_flags[15] <= 1'b1;
                        end
                    end
                    if (store_pop &&
                        (store_integer
                         ? (!(v2_exec_invalid || store_source_empty) ||
                            control_word[0])
                         : (!store_source_empty || control_word[0])))
                        pop_pending <= 1'b1;
                end
                default: ;
            endcase
        end

        if (pop_pending) begin
            pop_value();
            pop_pending <= 1'b0;
        end

        if (result_write_pending) begin
            write_stack_raw(result_write_index, result_write_raw);
            result_write_pending <= 1'b0;
        end

        // FPTAN commits tan(x) to the old ST0 before pushing the architectural
        // 1.0 result. Keeping these as separate stack-RAM cycles avoids a
        // second write-port dependency on the normal result path.
        if (fptan_push_after_result && !result_write_pending &&
            !push_pending) begin
            schedule_push(x87_one());
            fptan_push_after_result <= 1'b0;
        end

        if (trans_done && fptan_trans_pending) begin
            fptan_trans_pending <= 1'b0;
            status_flags[10] <= v2_exec_range_incomplete; // C2

            if (v2_exec_invalid)
                raise_invalid();
            if (v2_exec_denormal_operand) begin
                status_flags[1] <= 1'b1;
                if (!control_word[1]) begin
                    status_flags[7] <= 1'b1;
                    status_flags[15] <= 1'b1;
                end
            end

            if (v2_exec_range_incomplete) begin
                command_complete_pulse <= 1'b1;
            end else if (v2_exec_invalid) begin
                if (control_word[0]) begin
                    result_write_index <= arith_dest_index;
                    result_write_raw <= x87_to_m80(v2_exec_result);
                    result_write_pending <= 1'b1;
                    fptan_push_after_result <= 1'b1;
                    command_complete_pulse <= 1'b0;
                end
            end else if (v2_exec_denormal_operand && !control_word[1]) begin
                command_complete_pulse <= 1'b1;
            end else begin
                arith_operand_a <= v2_exec_result;
                arith_operand_b <= v2_exec_auxiliary_result;
                v2_exec_op <= X87_ARITH_DIV;
                v2_exec_owner <= EXEC_MATH;
                v2_exec_size <= 2'd0;
                v2_exec_transfer <= 64'h0;
                v2_exec_pending <= 1'b1;
                fptan_div_pending <= 1'b1;
                command_complete_pulse <= 1'b0;
            end
        end

        if (v2_exec_math_done && fptan_div_pending) begin
            fptan_div_pending <= 1'b0;
            if (!math_unmasked_exception)
                fptan_push_after_result <= 1'b1;
        end

        if (math_done) begin
            command_complete_pulse <= 1'b1;
            status_flags[9] <= v2_exec_math_done &&
                               (v2_exec_op != X87_ARITH_DIV) &&
                               (v2_exec_op != X87_ARITH_SQRT) &&
                               v2_exec_rounded_up;

            if (v2_exec_math_done && arith_compare) begin
                status_flags[14] <= v2_compare_equal ||
                                    v2_compare_unordered; // C3
                status_flags[10] <= v2_compare_unordered; // C2
                status_flags[8] <= v2_compare_less ||
                                   v2_compare_unordered;  // C0
            end

            if (trans_done)
                status_flags[10] <= v2_exec_range_incomplete; // C2

            if (math_invalid)
                raise_invalid();

            if (math_denormal_operand) begin
                status_flags[1] <= 1'b1; // DE
                if (!control_word[1]) begin
                    status_flags[7] <= 1'b1;
                    status_flags[15] <= 1'b1;
                end
            end

            if (math_divide_by_zero) begin
                status_flags[2] <= 1'b1; // ZE
                if (!control_word[2]) begin
                    status_flags[7] <= 1'b1;
                    status_flags[15] <= 1'b1;
                end
            end

            if (math_overflow) begin
                status_flags[3] <= 1'b1; // OE
                if (!control_word[3]) begin
                    status_flags[7] <= 1'b1;
                    status_flags[15] <= 1'b1;
                end
            end

            if (math_underflow) begin
                status_flags[4] <= 1'b1; // UE
                if (!control_word[4]) begin
                    status_flags[7] <= 1'b1;
                    status_flags[15] <= 1'b1;
                end
            end

            if (math_inexact) begin
                status_flags[5] <= 1'b1; // PE
                if (!control_word[5]) begin
                    status_flags[7] <= 1'b1;
                    status_flags[15] <= 1'b1;
                end
            end

            // A masked invalid operation retires its indefinite/quiet-NaN
            // result and compare pop. An unmasked invalid leaves the stack.
            if (!math_unmasked_exception && !math_range_incomplete) begin
                if (arith_write_result &&
                    (v2_exec_commit == X87_COMMIT_REPLACE_ST0)) begin
                    result_write_index <= arith_dest_index;
                    result_write_raw <= x87_to_m80(math_result);
                    result_write_pending <= 1'b1;
                    command_complete_pulse <= 1'b0;
                end
                if (arith_pop_count != 0) begin
                    tag_word[top*2 +: 2] <= 2'b11;
                    if (arith_pop_count == 2)
                        tag_word[(top + 3'd1)*2 +: 2] <= 2'b11;
                    top <= top + arith_pop_count;
                end
            end
        end

        if (core_cmd_valid && core_cmd_ready) begin
            last_fop <= core_cmd_fop;
            command_fop <= core_cmd_fop;
            command_pending <= 1'b1;
            stack_addr_a <= top;
            stack_addr_b <= ((fop_decode_key(core_cmd_fop) == 11'h530) ||
                             (core_cmd_fop == 11'h1f3))
                          ? top + 3'd1 : top + core_cmd_fop[2:0];
            if (core_cmd_direct)
                direct_m32_pending <= 1'b0;
        end

        if (word_in_valid && word_in_ready)
            // Keep PEREQ visible through the CPU microcode branch that
            // observes completion, even though the arithmetic side may drain
            // this queued word independently.
            pereq_release_hold <= 2'b11;

        // Stack operands are synchronous RAM outputs captured from the command
        // acceptance cycle. Execute only after both ports have returned.
        if (command_pending) begin
            command_pending <= 1'b0;
            command_complete_pulse <= 1'b1;
            status_read_pending <= command_decode.status_pending;
            rx_kind <= RX_NONE;
            rx_index <= 5'd0;
            rx_payload <= 80'h0;
            rx_byte_count <= 4'd0;
            tx_kind <= TX_NONE;
            tx_index <= 5'd0;
            tx_byte_offset <= 3'd0;
            tx_generation_done <= 1'b0;

            case (command_decode.action)
                X87_CMD_FNINIT: begin
                    control_word <= 16'h037f;
                    status_flags <= 16'h0000;
                    top <= 3'd0;
                    clear_stack();
                end
                X87_CMD_FNCLEX: begin
                    status_flags[7:0] <= 8'h00;
                    status_flags[15] <= 1'b0;
                end
                X87_CMD_FCHS: begin
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0])
                            begin
                                result_write_index <= st0_index;
                                result_write_raw <= x87_to_m80(x87_indefinite());
                                result_write_pending <= 1'b1;
                                command_complete_pulse <= 1'b0;
                            end
                    end else begin
                        result_write_index <= st0_index;
                        result_write_raw <= {
                            !stack_read_raw_a[79], stack_read_raw_a[78:0]};
                        result_write_pending <= 1'b1;
                        command_complete_pulse <= 1'b0;
                    end
                end
                X87_CMD_FABS: begin
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0])
                            begin
                                result_write_index <= st0_index;
                                result_write_raw <= x87_to_m80(x87_indefinite());
                                result_write_pending <= 1'b1;
                                command_complete_pulse <= 1'b0;
                            end
                    end else begin
                        result_write_index <= st0_index;
                        result_write_raw <= {1'b0, stack_read_raw_a[78:0]};
                        result_write_pending <= 1'b1;
                        command_complete_pulse <= 1'b0;
                    end
                end
                X87_CMD_FXAM: begin
                    status_flags[9] <= stack_empty(st0_index)
                                     ? 1'b0 : stack_read_data_a.sign;
                    case (stack_empty(st0_index)
                            ? X87_EMPTY : stack_read_data_a.class_id)
                        X87_NAN: begin
                            status_flags[14] <= 1'b0;
                            status_flags[10] <= 1'b0;
                            status_flags[8] <= 1'b1;
                        end
                        X87_NORMAL: begin
                            status_flags[14] <= 1'b0;
                            status_flags[10] <= 1'b1;
                            status_flags[8] <= 1'b0;
                        end
                        X87_INFINITY: begin
                            status_flags[14] <= 1'b0;
                            status_flags[10] <= 1'b1;
                            status_flags[8] <= 1'b1;
                        end
                        X87_ZERO: begin
                            status_flags[14] <= 1'b1;
                            status_flags[10] <= 1'b0;
                            status_flags[8] <= 1'b0;
                        end
                        X87_EMPTY: begin
                            status_flags[14] <= 1'b1;
                            status_flags[10] <= 1'b0;
                            status_flags[8] <= 1'b1;
                        end
                        default: begin                 // Denormal/unsupported
                            status_flags[14] <= 1'b1;
                            status_flags[10] <= 1'b1;
                            status_flags[8] <= 1'b0;
                        end
                    endcase
                end
                X87_CMD_PUSH_CONST: begin
                    case (command_decode.argument)
                        4'd0: schedule_push(x87_one());       // FLD1
                        4'd1: schedule_push_raw(              // FLDL2T
                            control_word[11:10] == 2'b10
                                ? 80'h4000_d49a784bcd1b8aff
                                : 80'h4000_d49a784bcd1b8afe);
                        4'd2: schedule_push_raw(              // FLDL2E
                            !control_word[10]
                                ? 80'h3fff_b8aa3b295c17f0bc
                                : 80'h3fff_b8aa3b295c17f0bb);
                        4'd3: schedule_push_raw(              // FLDPI
                            !control_word[10]
                                ? 80'h4000_c90fdaa22168c235
                                : 80'h4000_c90fdaa22168c234);
                        4'd4: schedule_push_raw(              // FLDLG2
                            !control_word[10]
                                ? 80'h3ffd_9a209a84fbcff799
                                : 80'h3ffd_9a209a84fbcff798);
                        4'd5: schedule_push_raw(              // FLDLN2
                            !control_word[10]
                                ? 80'h3ffe_b17217f7d1cf79ac
                                : 80'h3ffe_b17217f7d1cf79ab);
                        default: schedule_push(x87_zero(1'b0)); // FLDZ
                    endcase
                end
                X87_CMD_FSQRT: begin
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            result_write_index <= st0_index;
                            result_write_raw <= x87_to_m80(x87_indefinite());
                            result_write_pending <= 1'b1;
                            command_complete_pulse <= 1'b0;
                        end
                    end else begin
                        command_complete_pulse <= 1'b0;
                        arith_compare <= 1'b0;
                        arith_write_result <= 1'b1;
                        arith_pop_count <= 2'd0;
                        arith_dest_index <= st0_index;
                        arith_operand_a <= stack_read_data_a;
                        arith_operand_b <= x87_empty();
                        v2_exec_op <= X87_ARITH_SQRT;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                    end
                end
                X87_CMD_FPTAN: begin
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            result_write_index <= st0_index;
                            result_write_raw <= x87_to_m80(x87_indefinite());
                            result_write_pending <= 1'b1;
                            fptan_push_after_result <= 1'b1;
                            command_complete_pulse <= 1'b0;
                        end
                    end else if (!stack_empty(top - 3'd1)) begin
                        logic [2:0] new_top;

                        new_top = top - 3'd1;
                        raise_stack_fault(1'b1);
                        if (control_word[0]) begin
                            write_stack(top, x87_indefinite());
                            stack_addr_b <= new_top;
                            stack_write_data_b <= x87_to_m80(x87_indefinite());
                            stack_write_b <= 1'b1;
                            tag_word[new_top*2 +: 2] <=
                                x87_tag(x87_indefinite());
                            top <= new_top;
                        end
                    end else begin
                        command_complete_pulse <= 1'b0;
                        arith_compare <= 1'b0;
                        arith_write_result <= 1'b1;
                        arith_pop_count <= 2'd0;
                        arith_dest_index <= st0_index;
                        arith_operand_a <= stack_read_data_a;
                        arith_operand_b <= x87_empty();
                        trans_cosine <= 1'b0;
                        trans_tangent_pair <= 1'b1;
                        trans_atan2 <= 1'b0;
                        v2_exec_op <= X87_ARITH_TRANS;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                        fptan_trans_pending <= 1'b1;
                    end
                end
                X87_CMD_FPATAN: begin
                    if (stack_empty(st0_index) ||
                        stack_empty(top + 3'd1)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            result_write_index <= top + 3'd1;
                            result_write_raw <= x87_to_m80(x87_indefinite());
                            result_write_pending <= 1'b1;
                            tag_word[top*2 +: 2] <= 2'b11;
                            top <= top + 3'd1;
                            command_complete_pulse <= 1'b0;
                        end
                    end else begin
                        command_complete_pulse <= 1'b0;
                        arith_compare <= 1'b0;
                        arith_write_result <= 1'b1;
                        arith_pop_count <= 2'd1;
                        arith_dest_index <= top + 3'd1;
                        arith_operand_a <= stack_read_data_b; // Y = ST(1)
                        arith_operand_b <= stack_read_data_a; // X = ST(0)
                        trans_cosine <= 1'b0;
                        trans_tangent_pair <= 1'b0;
                        trans_atan2 <= 1'b1;
                        v2_exec_op <= X87_ARITH_TRANS;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                    end
                end
                X87_CMD_TRIG: begin                    // FSIN / FCOS
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            result_write_index <= st0_index;
                            result_write_raw <= x87_to_m80(x87_indefinite());
                            result_write_pending <= 1'b1;
                            command_complete_pulse <= 1'b0;
                        end
                    end else begin
                        command_complete_pulse <= 1'b0;
                        arith_compare <= 1'b0;
                        arith_write_result <= 1'b1;
                        arith_pop_count <= 2'd0;
                        arith_dest_index <= st0_index;
                        arith_operand_a <= stack_read_data_a;
                        arith_operand_b <= x87_empty();
                        trans_cosine <= command_decode.argument[0];
                        trans_tangent_pair <= 1'b0;
                        trans_atan2 <= 1'b0;
                        v2_exec_op <= X87_ARITH_TRANS;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                    end
                end
                X87_CMD_FRNDINT: begin
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            result_write_index <= st0_index;
                            result_write_raw <= x87_to_m80(x87_indefinite());
                            result_write_pending <= 1'b1;
                            command_complete_pulse <= 1'b0;
                        end
                    end else begin
                        command_complete_pulse <= 1'b0;
                        arith_compare <= 1'b0;
                        arith_write_result <= 1'b1;
                        arith_pop_count <= 2'd0;
                        arith_dest_index <= st0_index;
                        v2_exec_op <= X87_CONVERT_FRNDINT;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        arith_operand_a <= stack_read_data_a;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                    end
                end
                X87_CMD_FDECSTP: top <= top - 3'd1;
                X87_CMD_FINCSTP: top <= top + 3'd1;
                X87_CMD_TX_ENV: begin                 // FNSTENV/FSTENV
                    tx_kind <= TX_ENV;
                    tx_generation_done <= 1'b0;
                    command_complete_pulse <= 1'b0;
                end
                X87_CMD_TX_STATE: begin               // FNSAVE/FSAVE
                    tx_kind <= TX_STATE;
                    tx_generation_done <= 1'b0;
                    command_complete_pulse <= 1'b0;
                    tx_state_shift <= pack_state_pair(
                        stack_read_raw_a, stack_read_raw_b, top);
                end
                X87_CMD_RX_ENV: rx_kind <= RX_ENV;     // FLDENV
                X87_CMD_RX_STATE: rx_kind <= RX_STATE; // FRSTOR
                X87_CMD_ARITH: begin
                    // Register arithmetic and comparisons use the synchronous
                    // stack outputs captured with this command.
                    if (stack_empty(st0_index) ||
                        (command_decode.needs_sti &&
                         stack_empty(cmd_st_index))) begin
                        raise_stack_fault(1'b0);
                        if (command_decode.compare) begin
                            status_flags[14] <= 1'b1;
                            status_flags[10] <= 1'b1;
                            status_flags[8] <= 1'b1;
                        end
                        if (control_word[0]) begin
                            if (command_decode.write_result)
                                write_stack(
                                    command_decode.dest_sti
                                        ? cmd_st_index : st0_index,
                                    x87_indefinite());
                            if (command_decode.pop_count != 0) begin
                                tag_word[top*2 +: 2] <= 2'b11;
                                if (command_decode.pop_count == 2)
                                    tag_word[(top + 3'd1)*2 +: 2] <= 2'b11;
                                top <= top + command_decode.pop_count;
                            end
                        end
                    end else begin
                        command_complete_pulse <= 1'b0;
                        v2_exec_op <= command_decode.exec_op;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                        arith_compare <= command_decode.compare;
                        arith_quiet_compare <= command_decode.quiet_compare;
                        arith_write_result <= command_decode.write_result;
                        arith_pop_count <= command_decode.pop_count;
                        arith_dest_index <= command_decode.dest_sti
                                          ? cmd_st_index : st0_index;

                        if (!command_decode.needs_sti) begin // FTST uses +0.
                            arith_operand_a <= stack_read_data_a;
                            arith_operand_b <= x87_zero(1'b0);
                        end else if (command_decode.reverse_operands) begin
                            arith_operand_a <= stack_read_data_b;
                            arith_operand_b <= stack_read_data_a;
                        end else begin
                            arith_operand_a <= stack_read_data_a;
                            arith_operand_b <= stack_read_data_b;
                        end
                    end
                end

                // Register stack operations.
                X87_CMD_FLD_ST: begin
                    if (stack_empty(cmd_st_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0])
                            push_value(x87_indefinite());
                    end else begin
                        push_raw_tagged(
                            stack_read_raw_b,
                            tag_word[cmd_st_index*2 +: 2]);
                    end
                end
                X87_CMD_FXCH: begin
                    if (stack_empty(st0_index) || stack_empty(cmd_st_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            stack_addr_a <= st0_index;
                            stack_write_data_a <= x87_to_m80(x87_indefinite());
                            stack_write_a <= 1'b1;
                            if (cmd_st_index != st0_index) begin
                                stack_addr_b <= cmd_st_index;
                                stack_write_data_b <= x87_to_m80(x87_indefinite());
                                stack_write_b <= 1'b1;
                            end
                            tag_word[st0_index*2 +: 2] <= 2'b10;
                            tag_word[cmd_st_index*2 +: 2] <= 2'b10;
                        end
                    end else begin
                        stack_addr_a <= st0_index;
                        stack_write_data_a <= stack_read_raw_b;
                        stack_write_a <= 1'b1;
                        if (cmd_st_index != st0_index) begin
                            stack_addr_b <= cmd_st_index;
                            stack_write_data_b <= stack_read_raw_a;
                            stack_write_b <= 1'b1;
                        end
                        tag_word[st0_index*2 +: 2] <=
                            tag_word[cmd_st_index*2 +: 2];
                        tag_word[cmd_st_index*2 +: 2] <=
                            tag_word[st0_index*2 +: 2];
                    end
                end
                X87_CMD_FFREE:
                    tag_word[cmd_st_index*2 +: 2] <= 2'b11;
                X87_CMD_FSTP_ST: begin
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            write_stack(cmd_st_index, x87_indefinite());
                            pop_value();
                        end
                    end else begin
                        write_stack_raw(cmd_st_index, stack_read_raw_a);
                        pop_value();
                    end
                end

                X87_CMD_MEMORY_MATH: begin
                    memory_math_pending <= 1'b1;
                    memory_math_mul <= command_decode.exec_op == X87_ARITH_MUL;
                    memory_math_div <= command_decode.exec_op == X87_ARITH_DIV;
                    memory_math_compare <= command_decode.compare;
                    memory_math_subtract <= command_decode.exec_op == X87_ARITH_SUB;
                    memory_math_reverse <= command_decode.reverse_operands;
                    memory_math_pop <= command_decode.pop_count != 0;
                    rx_kind <= rx_kind_t'(command_decode.argument);
                end
                X87_CMD_LOAD:
                    rx_kind <= rx_kind_t'(command_decode.argument);
                X87_CMD_STORE:
                    start_store(
                        command_decode.argument[0],
                        command_decode.argument[2:1],
                        command_decode.argument[3]);
                default: ;
            endcase
        end

        if ((rx_kind != RX_NONE) && transfer_pop_valid) begin
            rx_payload <= rx_payload_next;
            rx_byte_count <= rx_byte_count_next;
            case (rx_kind)
                RX_CONTROL: begin
                    if (rx_byte_count_next >= 4'd2) begin
                        control_word <= rx_payload_next[15:0];
                        rx_kind <= RX_NONE;
                    end
                end
                RX_M32: begin
                    if (rx_byte_count_next >= 4'd4) begin
                        if (m32_is_normal(rx_payload_next[31:0])) begin
                            push_pending_raw <= normal_m32_to_m80(
                                rx_payload_next[31:0]);
                            push_pending_tag <= 2'b00;
                            push_pending <= 1'b1;
                        end else begin
                            v2_exec_op <= X87_CONVERT_FLD_M32;
                            v2_exec_owner <= EXEC_LOAD;
                            v2_exec_size <= 2'd0;
                            v2_exec_transfer <= {32'h0,
                                                 rx_payload_next[31:0]};
                            v2_exec_pending <= 1'b1;
                        end
                        rx_kind <= RX_NONE;
                    end
                end
                RX_M64: begin
                    if (rx_byte_count_next >= 4'd8) begin
                        v2_exec_op <= X87_CONVERT_FLD_M64;
                        v2_exec_owner <= EXEC_LOAD;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= rx_payload_next[63:0];
                        v2_exec_pending <= 1'b1;
                        rx_kind <= RX_NONE;
                    end
                end
                RX_M80: begin
                    if (rx_byte_count_next >= 4'd10) begin
                        push_pending_raw <= rx_payload_next;
                        push_pending_tag <= stack_tag_from_m80(rx_payload_next);
                        push_pending <= 1'b1;
                        rx_kind <= RX_NONE;
                    end
                end
                RX_I16: begin
                    if (rx_byte_count_next >= 4'd2) begin
                        v2_exec_op <= X87_CONVERT_FILD;
                        v2_exec_owner <= EXEC_LOAD;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= {48'h0, rx_payload_next[15:0]};
                        v2_exec_pending <= 1'b1;
                        rx_kind <= RX_NONE;
                    end
                end
                RX_I32: begin
                    if (rx_byte_count_next >= 4'd4) begin
                        v2_exec_op <= X87_CONVERT_FILD;
                        v2_exec_owner <= EXEC_LOAD;
                        v2_exec_size <= 2'd1;
                        v2_exec_transfer <= {32'h0, rx_payload_next[31:0]};
                        v2_exec_pending <= 1'b1;
                        rx_kind <= RX_NONE;
                    end
                end
                RX_I64: begin
                    if (rx_byte_count_next >= 4'd8) begin
                        v2_exec_op <= X87_CONVERT_FILD;
                        v2_exec_owner <= EXEC_LOAD;
                        v2_exec_size <= 2'd2;
                        v2_exec_transfer <= rx_payload_next[63:0];
                        v2_exec_pending <= 1'b1;
                        rx_kind <= RX_NONE;
                    end
                end
                RX_ENV, RX_STATE: begin
                    if (rx_index < 5'd7) begin
                        accept_environment_word(
                            rx_index[2:0], transfer_pop_data[15:0]);
                        if ((rx_kind == RX_ENV) && (rx_index == 5'd6))
                            rx_kind <= RX_NONE;
                        else
                            rx_index <= rx_index + 5'd1;
                    end else begin
                        case (rx_index)
                            5'd7, 5'd12, 5'd17, 5'd22:
                                rx_state_shift[31:0] <=
                                    transfer_pop_data[31:0];
                            5'd8, 5'd13, 5'd18, 5'd23:
                                rx_state_shift[63:32] <=
                                    transfer_pop_data[31:0];
                            5'd9, 5'd14, 5'd19, 5'd24:
                                rx_state_shift[95:64] <=
                                    transfer_pop_data[31:0];
                            5'd10, 5'd15, 5'd20, 5'd25:
                                rx_state_shift[127:96] <=
                                    transfer_pop_data[31:0];
                            5'd11: commit_state_pair(
                                2'd0, transfer_pop_data[31:0]);
                            5'd16: commit_state_pair(
                                2'd1, transfer_pop_data[31:0]);
                            5'd21: commit_state_pair(
                                2'd2, transfer_pop_data[31:0]);
                            5'd26: begin
                                commit_state_pair(
                                    2'd3, transfer_pop_data[31:0]);
                                rx_kind <= RX_NONE;
                            end
                            default: ;
                        endcase
                        if (rx_index != 5'd26)
                            rx_index <= rx_index + 5'd1;
                    end
                end
                default: ;
            endcase
        end

        // Output generation runs ahead until the shared queue fills. State
        // stream sequencing advances when a word enters the queue, not when
        // the CPU eventually reads it.
        if (tx_produce_fire) begin
            if (tx_kind == TX_STATE) begin
                case (tx_index)
                    5'd6:
                        tx_state_shift <= pack_state_pair(
                            stack_read_raw_a, stack_read_raw_b, top);
                    5'd9: begin
                        tx_state_shift <= tx_state_shift >> 32;
                        stack_addr_a <= top + 3'd2;
                        stack_addr_b <= top + 3'd3;
                    end
                    5'd11:
                        tx_state_shift <= pack_state_pair(
                            stack_read_raw_a, stack_read_raw_b, top + 3'd2);
                    5'd14: begin
                        tx_state_shift <= tx_state_shift >> 32;
                        stack_addr_a <= top + 3'd4;
                        stack_addr_b <= top + 3'd5;
                    end
                    5'd16:
                        tx_state_shift <= pack_state_pair(
                            stack_read_raw_a, stack_read_raw_b, top + 3'd4);
                    5'd19: begin
                        tx_state_shift <= tx_state_shift >> 32;
                        stack_addr_a <= top + 3'd6;
                        stack_addr_b <= top + 3'd7;
                    end
                    5'd21:
                        tx_state_shift <= pack_state_pair(
                            stack_read_raw_a, stack_read_raw_b, top + 3'd6);
                    default:
                        if (tx_index >= 5'd7)
                            tx_state_shift <= tx_state_shift >> 32;
                endcase
            end else if (tx_kind == TX_VALUE) begin
                tx_state_shift <= tx_state_shift >> 32;
            end

            if (((tx_kind == TX_VALUE) &&
                 (tx_index + 5'd1 == {3'h0, tx_count})) ||
                ((tx_kind == TX_ENV) && (tx_index == 5'd6)) ||
                ((tx_kind == TX_STATE) && (tx_index == 5'd26))) begin
                tx_generation_done <= 1'b1;
            end else begin
                tx_index <= tx_index + 5'd1;
            end
        end

        if (read_req_valid && read_req_ready) begin
            pereq_release_hold <= 2'b11;
            if (!read_req_data_port) begin
                // FNSTSW has register and memory forms. Other f8
                // miscellaneous reads return the control word; environment
                // streams use fc instead.
                read_resp_data <= fop_reads_status(last_fop)
                                ? {16'h0, status_word}
                                : {16'h0, control_word};
                status_read_pending <= 1'b0;
            end else begin
                read_resp_data <= select_transfer_bytes(
                    transfer_pop_data[31:0], tx_byte_offset, read_req_be);
                if (tx_entry_consumed)
                    tx_byte_offset <= 3'd0;
                else
                    tx_byte_offset <= tx_byte_offset + tx_request_bytes;
            end
            read_resp_valid <= 1'b1;
        end

        // Architectural save side effects occur only after the CPU consumes
        // the final queued word, preserving the previous visible ordering.
        if (tx_consume_fire && tx_generation_done &&
            (transfer_count == 2'd1)) begin
            if (tx_kind == TX_ENV)
                control_word[5:0] <= 6'h3f;
            else if (tx_kind == TX_STATE) begin
                control_word <= 16'h037f;
                status_flags <= 16'h0000;
                top <= 3'd0;
                clear_stack();
            end
            tx_kind <= TX_NONE;
            tx_index <= 5'd0;
            tx_generation_done <= 1'b0;
            tx_byte_offset <= 3'd0;
        end
    end
end

endmodule
