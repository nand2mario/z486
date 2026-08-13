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
    logic [7:0]  target;
    logic [2:0]  flow;
    logic [4:0]  condition;
    logic [3:0]  alu_route;
    logic [3:0]  shift_route;
    logic [3:0]  prepare;
    logic [3:0]  classify;
    logic [3:0]  pack;
    logic [4:0]  engine;
    logic [3:0]  state;
    logic [1:0]  count;
    logic [1:0]  grs;
    logic [2:0]  commit;
    logic [1:0]  flags;
    logic [9:0]  reserved;
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

`include "x87_entries.svh"

endpackage
