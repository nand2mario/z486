package x87_ucode_pkg;

typedef enum logic [2:0] {
    X87_FLOW_NEXT,
    X87_FLOW_JUMP,
    X87_FLOW_BRANCH,
    X87_FLOW_LOOP,
    X87_FLOW_WAIT,
    X87_FLOW_FINISH
} x87_flow_t;

typedef struct packed {
    logic [7:0]  target;        // Absolute branch/loop target.
    logic [2:0]  flow;          // Sequencer action for this word.
    logic [4:0]  condition;     // Registered predicate selector.
    logic [3:0]  alu_route;     // Shared add/subtract work route.
    logic [3:0]  shift_route;   // Work normalization/format shift.
    logic [3:0]  prepare;       // Operation-specific initial state.
    logic [3:0]  classify;      // Special-value and range checks.
    logic [3:0]  pack;          // Provisional result/transfer packer.
    logic [4:0]  engine;        // Iterative arithmetic phase.
    logic [3:0]  state;         // Auxiliary exponent/count update.
    logic [1:0]  count;
    logic [1:0]  grs;
    logic [2:0]  commit;        // Retirement action returned with done.
    logic [1:0]  flags;
    logic [2:0]  scratch_read;  // CORDIC limb-RAM read route.
    logic [2:0]  scratch_write; // CORDIC limb-RAM write route.
    logic [3:0]  reserved;
} x87_uop_t;

typedef enum logic [3:0] {
    X87_CONVERT_FRNDINT,
    X87_CONVERT_FLD_M32,
    X87_CONVERT_FLD_M64,
    X87_CONVERT_FILD,
    X87_CONVERT_FST_M32,
    X87_CONVERT_FST_M64,
    X87_CONVERT_FIST,
    X87_ARITH_ADD,
    X87_ARITH_SUB,
    X87_ARITH_COMPARE,
    X87_ARITH_MUL,
    X87_ARITH_DIV,
    X87_ARITH_SQRT,
    X87_ARITH_TRANS
} x87_exec_op_t;

// Synchronous command-ROM actions. The ROM converts the 11-bit ESC/FOP
// encoding into this small control word while the stack RAM reads operands.
typedef enum logic [4:0] {
    X87_CMD_NONE,
    X87_CMD_FNINIT,
    X87_CMD_FNCLEX,
    X87_CMD_FCHS,
    X87_CMD_FABS,
    X87_CMD_FXAM,
    X87_CMD_PUSH_CONST,
    X87_CMD_FSQRT,
    X87_CMD_FPTAN,
    X87_CMD_FPATAN,
    X87_CMD_TRIG,
    X87_CMD_FRNDINT,
    X87_CMD_FDECSTP,
    X87_CMD_FINCSTP,
    X87_CMD_TX_ENV,
    X87_CMD_TX_STATE,
    X87_CMD_RX_ENV,
    X87_CMD_RX_STATE,
    X87_CMD_ARITH,
    X87_CMD_FLD_ST,
    X87_CMD_FXCH,
    X87_CMD_FFREE,
    X87_CMD_FSTP_ST,
    X87_CMD_MEMORY_MATH,
    X87_CMD_LOAD,
    X87_CMD_STORE
} x87_command_action_t;

typedef struct packed {
    logic                status_pending; // Command returns architectural status.
    logic                arithmetic;     // BUSY# follows numeric command lifetime.
    logic          [3:0] argument;       // Transfer kind, width, or constant selector.
    logic          [1:0] pop_count;      // Stack entries retired after success.
    logic                reverse_operands;// Swap ST0 and source arithmetic order.
    logic                needs_sti;      // Second stack operand must be non-empty.
    logic                dest_sti;       // Result replaces ST(i), not ST0.
    logic                write_result;   // Successful arithmetic writes a value.
    logic                quiet_compare;  // QNaN compare does not raise invalid.
    logic                compare;        // Result updates C3/C2/C0.
    x87_exec_op_t         exec_op;        // Numeric executor entry family.
    x87_command_action_t  action;         // Hardwired control/transfer dispatch.
} x87_command_decode_t;

`include "x87_entries.svh"

endpackage
