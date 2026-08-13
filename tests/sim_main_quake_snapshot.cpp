#include "Vtb_protected_mode.h"
#include "verilated.h"
#if VM_TRACE_FST
#include "verilated_fst_c.h"
#endif

#include <cstdlib>
#include <cstring>

static const char* plusarg_value(int argc, char** argv, const char* prefix) {
    const size_t length = std::strlen(prefix);
    for (int index = 1; index < argc; ++index)
        if (std::strncmp(argv[index], prefix, length) == 0)
            return argv[index] + length;
    return nullptr;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vtb_protected_mode* top = new Vtb_protected_mode;
#if VM_TRACE_FST
    VerilatedFstC* trace = nullptr;
    vluint64_t trace_start = 0;
    vluint64_t trace_end = ~vluint64_t{0};
    if (Verilated::commandArgsPlusMatch("trace")) {
        const char* file = plusarg_value(argc, argv, "+tracefile=");
        const char* start = plusarg_value(argc, argv, "+trace_start=");
        const char* end = plusarg_value(argc, argv, "+trace_end=");
        if (start) trace_start = std::strtoull(start, nullptr, 0);
        if (end) trace_end = std::strtoull(end, nullptr, 0);
        Verilated::traceEverOn(true);
        trace = new VerilatedFstC;
        top->trace(trace, 99);
        trace->open(file ? file : "quake_snapshot.fst");
    }
#endif
    while (!Verilated::gotFinish()) {
        top->eval();
#if VM_TRACE_FST
        if (trace && Verilated::time() >= trace_start &&
            Verilated::time() <= trace_end)
            trace->dump(Verilated::time());
#endif
        Verilated::timeInc(1);
    }
#if VM_TRACE_FST
    if (trace) {
        trace->close();
        delete trace;
    }
#endif
    delete top;
    return 0;
}
