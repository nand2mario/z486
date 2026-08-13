// Area-oriented x87 state, transfer, and arithmetic scheduler. This module is
// the sole owner of architectural stack and status updates.
module x87_core
    import x87_pkg::*, x87_ucode_pkg::*;
(
    input  logic        clk,
    input  logic        reset,

    input  logic        cmd_valid,
    input  logic [10:0] cmd_fop,
    output logic        cmd_ready,

    input  logic        word_in_valid,
    input  logic  [3:0] word_in_be,
    input  logic [31:0] word_in_data,
    output logic        word_in_ready,

    input  logic        read_req_valid,
    input  logic        read_req_data_port,
    input  logic  [3:0] read_req_be,
    output logic        read_req_ready,
    output logic        read_resp_valid,
    output logic [31:0] read_resp_data,

    output logic        busy_n,
    output logic        pereq,
    output logic        error_n
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

logic [31:0] tx_words [0:2];
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
logic                   v2_exec_start;
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

wire [15:0] status_word = {status_flags[15:14], top, status_flags[10:0]};
wire [2:0] st0_index = top;
wire [2:0] cmd_st_index = top + command_fop[2:0];
assign stack_read_data_a = x87_from_m80(stack_read_raw_a);
assign stack_read_data_b = x87_from_m80(stack_read_raw_b);
assign push_pending_value = x87_from_m80(push_pending_raw);
wire command_is_arithmetic = fop_is_addsub(command_fop) ||
                             fop_is_compare(command_fop) ||
                             fop_is_mul(command_fop) ||
                             fop_is_div(command_fop) ||
                             fop_is_memory_math(command_fop) ||
                             (command_fop == 11'h1fa) ||
                             (command_fop == 11'h1f2) ||
                             (command_fop == 11'h1f3) ||
                             (command_fop == 11'h1fe) ||
                             (command_fop == 11'h1ff) ||
                             (command_fop == 11'h1fc);
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

wire [10:0] cmd_fop_key = fop_decode_key(cmd_fop);
wire [10:0] command_fop_key = fop_decode_key(command_fop);

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

function automatic logic fop_mask_match(
    input logic [10:0] fop,
    input logic [10:0] value,
    input logic [10:0] mask
);
    return (fop & mask) == value;
endfunction

function automatic logic fop_is_addsub(input logic [10:0] fop);
    logic register_form;
    logic supported_group;
    logic [2:0] operation;
    begin
        register_form = fop[7:6] == 2'b11;
        supported_group = (fop[10:8] == 3'd0) ||
                          (fop[10:8] == 3'd4) ||
                          (fop[10:8] == 3'd6);
        operation = fop[5:3];
        return register_form && supported_group &&
               ((operation == 3'd0) || (operation == 3'd4) ||
                (operation == 3'd5));
    end
endfunction

function automatic logic fop_is_compare_pop2(input logic [10:0] fop);
    return (fop == 11'h2e9) || // FUCOMPP
           (fop == 11'h6d9);   // FCOMPP
endfunction

function automatic logic fop_is_compare(input logic [10:0] fop);
    logic register_form;
    begin
        register_form = fop[7:6] == 2'b11;
        return (register_form && (fop[10:8] == 3'd0) &&
                ((fop[5:3] == 3'd2) || (fop[5:3] == 3'd3))) ||
               (register_form && (fop[10:8] == 3'd5) &&
                ((fop[5:3] == 3'd4) || (fop[5:3] == 3'd5))) ||
               fop_is_compare_pop2(fop) || (fop == 11'h1e4);
    end
endfunction

function automatic logic fop_is_mul(input logic [10:0] fop);
    logic register_form;
    logic supported_group;
    begin
        register_form = fop[7:6] == 2'b11;
        supported_group = (fop[10:8] == 3'd0) ||
                          (fop[10:8] == 3'd4) ||
                          (fop[10:8] == 3'd6);
        return register_form && supported_group &&
               (fop[5:3] == 3'd1);
    end
endfunction

function automatic logic fop_is_div(input logic [10:0] fop);
    logic register_form;
    logic supported_group;
    begin
        register_form = fop[7:6] == 2'b11;
        supported_group = (fop[10:8] == 3'd0) ||
                          (fop[10:8] == 3'd4) ||
                          (fop[10:8] == 3'd6);
        return register_form && supported_group &&
               ((fop[5:3] == 3'd6) || (fop[5:3] == 3'd7));
    end
endfunction

function automatic logic fop_is_memory_math(input logic [10:0] fop);
    logic supported_group;
    begin
        supported_group = (fop[10:8] == 3'd0) || // m32real
                          (fop[10:8] == 3'd2) || // m32int
                          (fop[10:8] == 3'd4) || // m64real
                          (fop[10:8] == 3'd6);   // m16int
        return (fop[7:6] != 2'b11) && supported_group;
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
wire restartable_rx_command = (rx_kind != RX_NONE) &&
                              (tx_kind == TX_NONE) &&
                              (transfer_count == 2'd0) &&
                              (cmd_fop == last_fop);
assign cmd_ready = ((rx_kind == RX_NONE) || restartable_rx_command) &&
                   (tx_kind == TX_NONE) &&
                   (transfer_count == 2'd0) &&
                   !command_pending && !stack_write_a && !stack_write_b &&
                   !push_pending &&
                   (!memory_math_pending || restartable_rx_command) &&
                   !store_pending && !pop_pending &&
                   !result_write_pending &&
                   !v2_exec_pending && !v2_exec_start &&
                   !v2_exec_busy && !v2_exec_done &&
                   !fptan_trans_pending && !fptan_div_pending &&
                   !fptan_push_after_result && !read_resp_valid;
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
assign busy_n = !(((v2_exec_owner == EXEC_MATH) &&
                   (v2_exec_pending || v2_exec_start ||
                    v2_exec_busy)) ||
                  (command_pending && command_is_arithmetic));
// PEREQ releases the 80386 coprocessor-wait microcode as well as requesting
// operand transfers. Keep it asserted throughout an accepted command because
// a one-cycle completion pulse can precede the CPU's CORWAIT sample.
assign pereq = (rx_kind != RX_NONE) ||
               ((tx_kind != TX_NONE) && transfer_pop_valid) ||
               command_pending || stack_write_a || stack_write_b ||
               push_pending || memory_math_pending ||
               store_pending || pop_pending || result_write_pending ||
               v2_exec_pending || v2_exec_start ||
               v2_exec_busy || v2_exec_done ||
               fptan_trans_pending || fptan_div_pending ||
               fptan_push_after_result ||
               status_read_pending || command_complete_pulse ||
               (pereq_release_hold != 2'b00);
assign error_n = !status_flags[7];

// One physical three-word queue serves both protocol directions. A command
// cannot change direction until the queue is empty.
always_comb begin
    tx_produce_valid = (tx_kind != TX_NONE) && !tx_generation_done;
    tx_produce_data = 36'h0;
    case (tx_kind)
        TX_VALUE: begin
            tx_produce_data[31:0] = tx_words[tx_index[1:0]];
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

    transfer_push_valid = (rx_kind != RX_NONE)
                        ? word_in_valid : tx_produce_valid;
    transfer_push_data = (rx_kind != RX_NONE)
                       ? {word_in_be, word_in_data} : tx_produce_data;
    transfer_pop_ready = (rx_kind != RX_NONE) ? 1'b1
                       : ((tx_kind != TX_NONE) && read_req_valid &&
                          read_req_ready && read_req_data_port &&
                          tx_entry_consumed);
end

assign tx_produce_fire = (tx_kind != TX_NONE) &&
                         transfer_push_valid && transfer_push_ready;
assign tx_consume_fire = (tx_kind != TX_NONE) &&
                         transfer_pop_valid && transfer_pop_ready;

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
    if (cmd_valid && cmd_ready) begin
        stack_port_addr_a = top;
        stack_port_addr_b = ((cmd_fop_key == 11'h530) ||
                             (cmd_fop == 11'h1f3))
                          ? top + 3'd1 : top + cmd_fop[2:0];
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
    .compare_equal(v2_compare_equal)
);

always_ff @(posedge clk) begin
    if (reset) begin
        control_word <= 16'h037f;
        status_flags <= 16'h0000;
        top <= 3'd0;
        last_fop <= 11'h000;
        command_pending <= 1'b0;
        command_fop <= 11'h000;
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
        tx_words[0] <= 32'h0;
        tx_words[1] <= 32'h0;
        tx_words[2] <= 32'h0;
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
        v2_exec_start <= 1'b0;
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
        v2_exec_start <= 1'b0;
        pereq_release_hold <= {1'b0, pereq_release_hold[1]};

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
                tx_words[0] <= store_m80_value[31:0];
                tx_words[1] <= store_m80_value[63:32];
                tx_words[2] <= {16'h0, store_m80_value[79:64]};
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

        if (v2_exec_pending && !v2_exec_busy) begin
            v2_exec_start <= 1'b1;
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
                    tx_words[0] <= v2_exec_transfer_out[31:0];
                    tx_words[1] <= v2_exec_transfer_out[63:32];
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

        if (cmd_valid && cmd_ready) begin
            last_fop <= cmd_fop;
            command_fop <= cmd_fop;
            command_pending <= 1'b1;
            stack_addr_a <= top;
            stack_addr_b <= ((cmd_fop_key == 11'h530) ||
                             (cmd_fop == 11'h1f3))
                          ? top + 3'd1 : top + cmd_fop[2:0];
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
            status_read_pending <= fop_reads_status(command_fop) ||
                                   (command_fop_key == 11'h138);
            rx_kind <= RX_NONE;
            rx_index <= 5'd0;
            rx_payload <= 80'h0;
            rx_byte_count <= 4'd0;
            tx_kind <= TX_NONE;
            tx_index <= 5'd0;
            tx_byte_offset <= 3'd0;
            tx_generation_done <= 1'b0;

            case (command_fop_key)
                11'h3e3: begin                         // FNINIT
                    control_word <= 16'h037f;
                    status_flags <= 16'h0000;
                    top <= 3'd0;
                    clear_stack();
                end
                11'h3e2: begin                        // FNCLEX
                    status_flags[7:0] <= 8'h00;
                    status_flags[15] <= 1'b0;
                end
                11'h1e0: begin                        // FCHS
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
                11'h1e1: begin                        // FABS
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
                11'h1e5: begin                        // FXAM
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
                11'h1e8: schedule_push(x87_one());       // FLD1
                11'h1e9: schedule_push_raw(              // FLDL2T
                    control_word[11:10] == 2'b10
                        ? 80'h4000_d49a784bcd1b8aff
                        : 80'h4000_d49a784bcd1b8afe);
                11'h1ea: schedule_push_raw(              // FLDL2E
                    !control_word[10]
                        ? 80'h3fff_b8aa3b295c17f0bc
                        : 80'h3fff_b8aa3b295c17f0bb);
                11'h1eb: schedule_push_raw(              // FLDPI
                    !control_word[10]
                        ? 80'h4000_c90fdaa22168c235
                        : 80'h4000_c90fdaa22168c234);
                11'h1ec: schedule_push_raw(              // FLDLG2
                    !control_word[10]
                        ? 80'h3ffd_9a209a84fbcff799
                        : 80'h3ffd_9a209a84fbcff798);
                11'h1ed: schedule_push_raw(              // FLDLN2
                    !control_word[10]
                        ? 80'h3ffe_b17217f7d1cf79ac
                        : 80'h3ffe_b17217f7d1cf79ab);
                11'h1ee: schedule_push(x87_zero(1'b0));  // FLDZ
                11'h1fa: begin                           // FSQRT
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
                11'h1f2: begin                           // FPTAN
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
                11'h1f3: begin                           // FPATAN
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
                11'h1fe, 11'h1ff: begin                 // FSIN / FCOS
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
                        trans_cosine <= command_fop[0];
                        trans_tangent_pair <= 1'b0;
                        trans_atan2 <= 1'b0;
                        v2_exec_op <= X87_ARITH_TRANS;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                    end
                end
                11'h1fc: begin                           // FRNDINT
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
                11'h1f6: top <= top - 3'd1;           // FDECSTP
                11'h1f7: top <= top + 3'd1;           // FINCSTP
                11'h130: begin                        // FNSTENV/FSTENV
                    tx_kind <= TX_ENV;
                    tx_generation_done <= 1'b0;
                    command_complete_pulse <= 1'b0;
                end
                11'h530: begin                        // FNSAVE/FSAVE
                    tx_kind <= TX_STATE;
                    tx_generation_done <= 1'b0;
                    command_complete_pulse <= 1'b0;
                    tx_state_shift <= pack_state_pair(
                        stack_read_raw_a, stack_read_raw_b, top);
                end
                11'h120: rx_kind <= RX_ENV;            // FLDENV
                11'h520: rx_kind <= RX_STATE;          // FRSTOR
                default: begin
                    // Register arithmetic and comparisons use the synchronous
                    // stack outputs captured with this command.
                    if (fop_is_addsub(command_fop) ||
                        fop_is_mul(command_fop) ||
                        fop_is_div(command_fop) ||
                        fop_is_compare(command_fop)) begin
                        logic needs_sti;
                        logic [1:0] requested_pop;

                        needs_sti = command_fop != 11'h1e4; // FTST uses +0.
                        requested_pop = 2'd0;
                        if (((command_fop[10:8] == 3'd0) &&
                             (command_fop[5:3] == 3'd3)) ||
                            ((command_fop[10:8] == 3'd5) &&
                             (command_fop[5:3] == 3'd5)) ||
                            ((fop_is_addsub(command_fop) ||
                              fop_is_mul(command_fop) ||
                              fop_is_div(command_fop)) &&
                             (command_fop[10:8] == 3'd6)))
                            requested_pop = 2'd1;
                        if (fop_is_compare_pop2(command_fop))
                            requested_pop = 2'd2;

                        if (stack_empty(st0_index) ||
                            (needs_sti && stack_empty(cmd_st_index))) begin
                            raise_stack_fault(1'b0);
                            if (fop_is_compare(command_fop)) begin
                                status_flags[14] <= 1'b1;
                                status_flags[10] <= 1'b1;
                                status_flags[8] <= 1'b1;
                            end
                            if (control_word[0]) begin
                                if (fop_is_addsub(command_fop) ||
                                    fop_is_mul(command_fop) ||
                                    fop_is_div(command_fop))
                                    write_stack(
                                        (command_fop[10:8] == 3'd0)
                                            ? st0_index : cmd_st_index,
                                        x87_indefinite());
                                if (requested_pop != 0) begin
                                    tag_word[top*2 +: 2] <= 2'b11;
                                    if (requested_pop == 2)
                                        tag_word[(top + 3'd1)*2 +: 2] <= 2'b11;
                                    top <= top + requested_pop;
                                end
                            end
                        end else begin
                            command_complete_pulse <= 1'b0;
                            v2_exec_op <= fop_is_div(command_fop)
                                ? X87_ARITH_DIV
                                : fop_is_mul(command_fop)
                                ? X87_ARITH_MUL
                                : fop_is_compare(command_fop)
                                ? X87_ARITH_COMPARE
                                : ((command_fop[5:3] == 3'd4) ||
                                   (command_fop[5:3] == 3'd5))
                                ? X87_ARITH_SUB : X87_ARITH_ADD;
                            v2_exec_owner <= EXEC_MATH;
                            v2_exec_size <= 2'd0;
                            v2_exec_transfer <= 64'h0;
                            v2_exec_pending <= 1'b1;
                            arith_compare <= fop_is_compare(command_fop);
                            arith_quiet_compare <=
                                command_fop[10:8] == 3'd5; // FUCOM[P]
                            arith_write_result <= fop_is_addsub(command_fop) ||
                                                  fop_is_mul(command_fop) ||
                                                  fop_is_div(command_fop);
                            arith_pop_count <= requested_pop;
                            arith_dest_index <=
                                (command_fop[10:8] == 3'd0)
                                    ? st0_index : cmd_st_index;

                            if (command_fop == 11'h1e4) begin
                                arith_operand_a <= stack_read_data_a;
                                arith_operand_b <= x87_zero(1'b0);
                            end else if ((fop_is_addsub(command_fop) ||
                                         fop_is_div(command_fop)) &&
                                        ((command_fop[5:3] == 3'd5) ||
                                         (command_fop[5:3] == 3'd7))) begin
                                arith_operand_a <= stack_read_data_b;
                                arith_operand_b <= stack_read_data_a;
                            end else begin
                                arith_operand_a <= stack_read_data_a;
                                arith_operand_b <= stack_read_data_b;
                            end
                        end
                    end

                    // Register stack operations.
                    else if (fop_mask_match(command_fop, 11'h1c0, 11'h7f8)) begin
                        if (stack_empty(cmd_st_index)) begin          // FLD ST(i)
                            raise_stack_fault(1'b0);
                            if (control_word[0])
                                push_value(x87_indefinite());
                        end else begin
                            push_raw_tagged(
                                stack_read_raw_b,
                                tag_word[cmd_st_index*2 +: 2]);
                        end
                    end
                    else if (fop_mask_match(command_fop, 11'h1c8, 11'h7f8)) begin
                        if (stack_empty(st0_index) || stack_empty(cmd_st_index)) begin
                            raise_stack_fault(1'b0);                  // FXCH ST(i)
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
                    end else if (fop_mask_match(command_fop, 11'h5c0, 11'h7f8)) begin
                        tag_word[cmd_st_index*2 +: 2] <= 2'b11;
                    end
                    else if (fop_mask_match(command_fop, 11'h5d8, 11'h7f8)) begin
                        if (stack_empty(st0_index)) begin             // FSTP ST(i)
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

                    // Memory loads. ModR/M r/m bits do not affect the sidecar.
                    else if (fop_is_memory_math(command_fop)) begin
                        memory_math_pending <= 1'b1;
                        memory_math_mul <= command_fop[5:3] == 3'd1;
                        memory_math_div <= (command_fop[5:3] == 3'd6) ||
                                           (command_fop[5:3] == 3'd7);
                        memory_math_compare <= (command_fop[5:3] == 3'd2) ||
                                               (command_fop[5:3] == 3'd3);
                        memory_math_subtract <= (command_fop[5:3] == 3'd4) ||
                                                (command_fop[5:3] == 3'd5);
                        memory_math_reverse <= (command_fop[5:3] == 3'd5) ||
                                               (command_fop[5:3] == 3'd7);
                        memory_math_pop <= command_fop[5:3] == 3'd3;
                        case (command_fop[10:8])
                            3'd0: rx_kind <= RX_M32;
                            3'd2: rx_kind <= RX_I32;
                            3'd4: rx_kind <= RX_M64;
                            default: rx_kind <= RX_I16;
                        endcase
                    end
                    else if ((command_fop[10:8] == 3'd1) && (command_fop[5:3] == 3'd0))
                        rx_kind <= RX_M32;                            // FLD m32real
                    else if ((command_fop[10:8] == 3'd5) && (command_fop[5:3] == 3'd0))
                        rx_kind <= RX_M64;                            // FLD m64real
                    else if ((command_fop[10:8] == 3'd3) && (command_fop[5:3] == 3'd5))
                        rx_kind <= RX_M80;                            // FLD m80real
                    else if ((command_fop[10:8] == 3'd7) && (command_fop[5:3] == 3'd0))
                        rx_kind <= RX_I16;                            // FILD m16int
                    else if ((command_fop[10:8] == 3'd3) && (command_fop[5:3] == 3'd0))
                        rx_kind <= RX_I32;                            // FILD m32int
                    else if ((command_fop[10:8] == 3'd7) && (command_fop[5:3] == 3'd5))
                        rx_kind <= RX_I64;                            // FILD m64int
                    else if ((command_fop[10:8] == 3'd1) && (command_fop[5:3] == 3'd5))
                        rx_kind <= RX_CONTROL;                        // FLDCW

                    // Real stores.
                    else if ((command_fop[10:8] == 3'd1) &&
                             ((command_fop[5:3] == 3'd2) || (command_fop[5:3] == 3'd3)))
                        start_store(1'b0, 2'd0, command_fop[3]);     // FST[P] m32
                    else if ((command_fop[10:8] == 3'd5) &&
                             ((command_fop[5:3] == 3'd2) || (command_fop[5:3] == 3'd3)))
                        start_store(1'b0, 2'd1, command_fop[3]);     // FST[P] m64
                    else if ((command_fop[10:8] == 3'd3) && (command_fop[5:3] == 3'd7))
                        start_store(1'b0, 2'd2, 1'b1);               // FSTP m80

                    // Integer stores.
                    else if ((command_fop[10:8] == 3'd7) &&
                             ((command_fop[5:3] == 3'd2) || (command_fop[5:3] == 3'd3)))
                        start_store(1'b1, 2'd0, command_fop[3]);     // FIST[P] m16
                    else if ((command_fop[10:8] == 3'd3) &&
                             ((command_fop[5:3] == 3'd2) || (command_fop[5:3] == 3'd3)))
                        start_store(1'b1, 2'd1, command_fop[3]);     // FIST[P] m32
                    else if ((command_fop[10:8] == 3'd7) && (command_fop[5:3] == 3'd7))
                        start_store(1'b1, 2'd2, 1'b1);               // FISTP m64
                end
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
                        v2_exec_op <= X87_CONVERT_FLD_M32;
                        v2_exec_owner <= EXEC_LOAD;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= {32'h0, rx_payload_next[31:0]};
                        v2_exec_pending <= 1'b1;
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
