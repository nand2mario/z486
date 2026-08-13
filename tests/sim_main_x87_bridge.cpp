#include "Vtb_x87_bridge.h"
#include "verilated.h"

int main(int argc, char **argv) {
    auto contextp = new VerilatedContext;
    contextp->commandArgs(argc, argv);
    auto top = new Vtb_x87_bridge{contextp};

    while (!contextp->gotFinish() && contextp->time() < 10000) {
        top->eval();
        contextp->timeInc(1);
    }

    int failed = contextp->gotFinish() ? 0 : 1;
    delete top;
    delete contextp;
    return failed;
}
