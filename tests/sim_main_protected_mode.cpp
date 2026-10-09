#include "Vtb_protected_mode.h"
#include "verilated.h"
#include "verilated_vcd_c.h"
#include <cstring>

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vtb_protected_mode* top = new Vtb_protected_mode;

    // commandArgsPlusMatch returns a non-null empty string on no match.
    // Require the exact opt-in: trace_instr/tracefile alone are not VCD requests.
    bool trace_requested = false;
    for (int index = 1; index < argc; index++) {
        if (std::strcmp(argv[index], "+trace") == 0)
            trace_requested = true;
    }
    VerilatedVcdC* tfp = nullptr;
    if (trace_requested) {
        const char* tracefile = "trace.vcd";
        for (int index = 1; index < argc; index++) {
            static constexpr const char prefix[] = "+tracefile=";
            if (std::strncmp(argv[index], prefix, sizeof(prefix) - 1) == 0)
                tracefile = argv[index] + sizeof(prefix) - 1;
        }
        Verilated::traceEverOn(true);
        tfp = new VerilatedVcdC;
        top->trace(tfp, 99);
        tfp->open(tracefile);
    }

    while (!Verilated::gotFinish()) {
        top->eval();
        if (tfp) tfp->dump(Verilated::time());
        Verilated::timeInc(1);
    }

    if (tfp) {
        tfp->close();
        delete tfp;
    }
    delete top;
    return 0;
}
