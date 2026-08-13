#include "Vtb_x87_testfloat.h"
#include "verilated.h"

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vtb_x87_testfloat top;
    while (!Verilated::gotFinish()) {
        top.eval();
        Verilated::timeInc(1);
    }
    top.final();
    return 0;
}
