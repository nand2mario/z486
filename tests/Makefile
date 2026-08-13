# Makefile for z486 testing
#
# Builds and runs single-instruction tests for the z486 core

# Paths
Z486_DIR := ..
VERILATOR := verilator

# Source files
# Note: muldiv.sv and bus_prefetch.sv are standalone modules that can optionally
# replace the inline mul/div and bus/prefetch code in z486.sv.
# Currently z486.sv uses inline code; uncomment modules below when integrating.
X87_DIR := $(Z486_DIR)/x87
X87_REF_DIR := x87_reference
X87_UCODE_GEN := $(X87_DIR)/x87_ucode.svh \
			$(X87_DIR)/x87_ucode.mif \
			$(X87_DIR)/x87_entries.svh
X87_COMMAND_GEN := $(X87_DIR)/x87_command_decode.svh \
			$(X87_DIR)/x87_command_decode.mif
X87_GEN := $(X87_UCODE_GEN) $(X87_COMMAND_GEN)
X87_RTL := $(X87_DIR)/x87_pkg.sv \
			$(X87_DIR)/x87_ucode_pkg.sv \
			$(X87_DIR)/x87_bridge.sv \
			$(X87_DIR)/x87_cordic_rom.sv \
			$(X87_DIR)/x87_cordic_scratch.sv \
			$(X87_DIR)/x87_ucode_rom.sv \
			$(X87_DIR)/x87_sequencer.sv \
			$(X87_DIR)/x87_executor.sv \
			$(X87_DIR)/x87_transfer_fifo.sv \
			$(X87_DIR)/x87_stack_mem.sv \
			$(X87_DIR)/x87_command_rom.sv \
			$(X87_DIR)/x87_control.sv \
			$(X87_DIR)/x87_unit.sv

RTL_SRCS := $(Z486_DIR)/z486_pkg.sv $(X87_DIR)/x87_pkg.sv \
			$(X87_DIR)/x87_ucode_pkg.sv $(Z486_DIR)/z486.sv \
			$(Z486_DIR)/cpu_throttle.sv $(Z486_DIR)/interrupt_controller.sv \
			$(Z486_DIR)/ucode_rom.sv $(Z486_DIR)/microsequencer.sv $(Z486_DIR)/fast_issue.sv \
			$(Z486_DIR)/data_unit.sv $(Z486_DIR)/address_unit.sv \
			$(Z486_DIR)/shifter.sv $(Z486_DIR)/mul_div.sv \
			$(Z486_DIR)/decoder.sv \
			$(Z486_DIR)/alu.sv $(Z486_DIR)/prefetch.sv \
			$(Z486_DIR)/paging_unit.sv $(Z486_DIR)/paging_tlb.sv $(Z486_DIR)/paging_walker.sv \
			$(Z486_DIR)/memory.sv $(Z486_DIR)/l1_cache.sv $(Z486_DIR)/l1_icache.sv \
			$(filter-out $(X87_DIR)/x87_pkg.sv $(X87_DIR)/x87_ucode_pkg.sv,$(X87_RTL)) \
			$(Z486_DIR)/dsp_mul.sv $(Z486_DIR)/protection.sv $(Z486_DIR)/segmentation_unit.sv
RTL_DEPS := $(Z486_DIR)/ucode_recipes.svh $(X87_GEN)
# Optional modules (not yet integrated):
# RTL_SRCS += $(Z486_DIR)/muldiv.sv $(Z486_DIR)/bus_prefetch.sv
TB_SRCS := tb_z486.sv
CPP_SRCS := sim_main.cpp

# Verilator flags (common, no --top-module — set per target)
VFLAGS_BASE := --timing
VFLAGS_BASE += -Wall -Wno-fatal -Wno-UNOPTFLAT -Wno-SYNCASYNCNET
VFLAGS_BASE += -cc --exe
VFLAGS_BASE += -O3
VFLAGS_BASE += --build
VFLAGS_BASE += -j 0
VFLAGS_BASE += -I$(Z486_DIR)
VFLAGS_BASE += -I$(X87_DIR)
VFLAGS_BASE += -I$(X87_REF_DIR)
VFLAGS_BASE += -CFLAGS "-std=c++14 -O3"

# With VCD trace support (singlestep tests — short runs, need waveforms)
VFLAGS := $(VFLAGS_BASE) --trace --trace-structs
# With FST trace (test386/test186 — millions of cycles, FST is compact)
VFLAGS_FST := $(VFLAGS_BASE) --trace-fst
# Without trace (default test386/test186 build)
VFLAGS_NOTRACE := $(VFLAGS_BASE)

# test386 testbench sources
TB386_SRCS := tb_test386.sv
CPP386_SRCS := sim_main_test386.cpp

# test186 testbench sources (80186 test suite)
TB186_SRCS := tb_test186.sv
CPP186_SRCS := sim_main_test186.cpp

# protected mode testbench sources
TB_PROT_SRCS := tb_protected_mode.sv
CPP_PROT_SRCS := sim_main_protected_mode.cpp
CPP_QUAKE_SNAPSHOT_SRCS := sim_main_quake_snapshot.cpp

# L1 cache unit test
TB_L1_SRCS := tb_l1_cache.sv
CPP_L1_SRCS := sim_main_l1_cache.cpp
TB_L1I_SRCS := tb_l1_icache.sv
CPP_L1I_SRCS := sim_main_l1_icache.cpp
TB_X87_BRIDGE_SRCS := tb_x87_bridge.sv
CPP_X87_BRIDGE_SRCS := sim_main_x87_bridge.cpp
TB_X87_SEQ_SRCS := tb_x87_sequencer.sv
CPP_X87_SEQ_SRCS := sim_main_x87_sequencer.cpp
TB_X87_EXECUTOR_SRCS := tb_x87_executor.sv
CPP_X87_EXECUTOR_SRCS := sim_main_x87_executor.cpp
TB_X87_TESTFLOAT_SRCS := tb_x87_testfloat.sv
CPP_X87_TESTFLOAT_SRCS := sim_main_x87_testfloat.cpp
TB_X87_CORE_SRCS := tb_x87_core.sv
CPP_X87_CORE_SRCS := sim_main_x87_core.cpp
TB_X87_FIFO_SRCS := tb_x87_transfer_fifo.sv
CPP_X87_FIFO_SRCS := sim_main_x87_transfer_fifo.cpp

# PLA4 verification testbench sources
TB_PLA4_SRCS := tb_protection_pla4.sv
CPP_PLA4_SRCS := sim_main_pla4.cpp
PLA4_RTL := $(Z486_DIR)/protection.sv

# Targets
.PHONY: all build test-simple test-full test-386 test-186 test-protected test-pla4 test-l1-cache test-l1-icache test-x87-bridge test-x87-sequencer test-x87-executor test-x87-testfloat test-x87-fifo test-x87-core test-x87-integration test-x87-integer dhrystone clean help

all: build

# Build all testbenches
build: obj_dir/Vtb_z486 obj_dir/Vtb_protected_mode

obj_dir/Vtb_z486: $(RTL_SRCS) $(RTL_DEPS) $(TB_SRCS) $(CPP_SRCS)
	@echo "Building z486 testbench with Verilator..."
	cp -f ../*.hex .
	$(VERILATOR) $(VFLAGS) --top-module tb_z486 $(TB_SRCS) $(RTL_SRCS) $(CPP_SRCS)
	@echo "Build complete: obj_dir/Vtb_z486"

obj_dir/Vtb_protected_mode: $(RTL_SRCS) $(RTL_DEPS) $(TB_PROT_SRCS) $(CPP_PROT_SRCS)
	@echo "Building protected mode testbench with Verilator..."
	cp -f ../*.hex .
	$(VERILATOR) $(VFLAGS) --top-module tb_protected_mode $(TB_PROT_SRCS) $(RTL_SRCS) $(CPP_PROT_SRCS)
	@echo "Build complete: obj_dir/Vtb_protected_mode"

obj_dir_x87/Vtb_protected_mode: $(RTL_SRCS) $(RTL_DEPS) $(TB_PROT_SRCS) $(CPP_PROT_SRCS)
	@echo "Building x87-enabled protected mode testbench with Verilator..."
	cp -f ../*.hex .
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir_x87 -GENABLE_X87=1 --top-module tb_protected_mode \
		$(TB_PROT_SRCS) $(RTL_SRCS) $(CPP_PROT_SRCS)
	@echo "Build complete: obj_dir_x87/Vtb_protected_mode"

obj_dir_quake/Vtb_protected_mode: $(RTL_SRCS) $(RTL_DEPS) $(TB_PROT_SRCS) $(CPP_QUAKE_SNAPSHOT_SRCS)
	@echo "Building 64MB x87 Quake snapshot replay..."
	cp -f ../*.hex .
	$(VERILATOR) $(VFLAGS_NOTRACE) --Mdir obj_dir_quake \
		-GENABLE_X87=1 -GMEM_SIZE=67108864 --top-module tb_protected_mode \
		$(TB_PROT_SRCS) $(RTL_SRCS) $(CPP_QUAKE_SNAPSHOT_SRCS)
	@echo "Build complete: obj_dir_quake/Vtb_protected_mode"

obj_dir_x87_int/Vtb_z486: $(RTL_SRCS) $(RTL_DEPS) $(TB_SRCS) $(CPP_SRCS)
	@echo "Building x87-enabled integer testbench with Verilator..."
	cp -f ../*.hex .
	$(VERILATOR) $(VFLAGS) --Mdir obj_dir_x87_int -GENABLE_X87=1 --top-module tb_z486 \
		$(TB_SRCS) $(RTL_SRCS) $(CPP_SRCS)
	@echo "Build complete: obj_dir_x87_int/Vtb_z486"

obj_dir/Vtb_test386: $(RTL_SRCS) $(RTL_DEPS) $(TB386_SRCS) $(CPP386_SRCS)
	@echo "Building test386 testbench with Verilator..."
	cp -f ../*.hex .
	$(VERILATOR) $(VFLAGS_FST) --top-module tb_test386 $(TB386_SRCS) $(RTL_SRCS) $(CPP386_SRCS)
	@echo "Build complete: obj_dir/Vtb_test386"

obj_dir/Vtb_l1_cache: $(Z486_DIR)/l1_cache.sv $(TB_L1_SRCS) $(CPP_L1_SRCS)
	@echo "Building L1 cache unit test with Verilator..."
	$(VERILATOR) $(VFLAGS_NOTRACE) --top-module tb_l1_cache $(TB_L1_SRCS) $(Z486_DIR)/l1_cache.sv $(CPP_L1_SRCS)
	@echo "Build complete: obj_dir/Vtb_l1_cache"

obj_dir/Vtb_l1_icache: $(Z486_DIR)/l1_icache.sv $(TB_L1I_SRCS) $(CPP_L1I_SRCS)
	@echo "Building L1 instruction cache unit test with Verilator..."
	$(VERILATOR) $(VFLAGS_NOTRACE) --top-module tb_l1_icache $(TB_L1I_SRCS) $(Z486_DIR)/l1_icache.sv $(CPP_L1I_SRCS)
	@echo "Build complete: obj_dir/Vtb_l1_icache"

obj_dir/Vtb_x87_bridge: $(X87_DIR)/x87_bridge.sv $(TB_X87_BRIDGE_SRCS) $(CPP_X87_BRIDGE_SRCS)
	@echo "Building x87 bridge unit test with Verilator..."
	$(VERILATOR) $(VFLAGS_NOTRACE) --top-module tb_x87_bridge $(TB_X87_BRIDGE_SRCS) $(X87_DIR)/x87_bridge.sv $(CPP_X87_BRIDGE_SRCS)
	@echo "Build complete: obj_dir/Vtb_x87_bridge"

$(X87_UCODE_GEN): $(X87_DIR)/build_x87_microcode.py
	cd $(X87_DIR) && ./build_x87_microcode.py

$(X87_COMMAND_GEN): $(X87_DIR)/build_x87_command_decode.py
	cd $(X87_DIR) && ./build_x87_command_decode.py

obj_dir/Vtb_x87_sequencer: $(X87_DIR)/x87_ucode.svh $(X87_DIR)/x87_entries.svh \
		$(X87_DIR)/x87_ucode_pkg.sv $(X87_DIR)/x87_ucode_rom.sv \
		$(X87_DIR)/x87_sequencer.sv $(TB_X87_SEQ_SRCS) $(CPP_X87_SEQ_SRCS)
	@echo "Building x87 sequencer unit test with Verilator..."
	$(VERILATOR) $(VFLAGS_NOTRACE) --top-module tb_x87_sequencer \
		$(X87_DIR)/x87_ucode_pkg.sv $(TB_X87_SEQ_SRCS) \
		$(X87_DIR)/x87_ucode_rom.sv $(X87_DIR)/x87_sequencer.sv \
		$(CPP_X87_SEQ_SRCS)
	@echo "Build complete: obj_dir/Vtb_x87_sequencer"

obj_dir/Vtb_x87_executor: $(X87_DIR)/x87_ucode.svh $(X87_DIR)/x87_entries.svh \
		$(X87_DIR)/x87_pkg.sv $(X87_REF_DIR)/x87_roundint.sv \
		$(X87_REF_DIR)/x87_addsub.sv $(X87_REF_DIR)/x87_mul.sv \
		$(X87_REF_DIR)/x87_divsqrt.sv $(X87_DIR)/x87_cordic_rom.sv \
		$(X87_REF_DIR)/x87_cordic_atan_q80.svh \
		$(X87_REF_DIR)/x87_trans.sv \
		$(X87_DIR)/x87_ucode_pkg.sv $(X87_DIR)/x87_ucode_rom.sv \
		$(X87_DIR)/x87_sequencer.sv $(X87_DIR)/x87_executor.sv \
		$(X87_DIR)/x87_cordic_scratch.sv \
		$(TB_X87_EXECUTOR_SRCS) $(CPP_X87_EXECUTOR_SRCS)
	@echo "Building x87 executor unit test with Verilator..."
	$(VERILATOR) $(VFLAGS_NOTRACE) --top-module tb_x87_executor \
		$(X87_DIR)/x87_pkg.sv $(X87_DIR)/x87_ucode_pkg.sv \
		$(TB_X87_EXECUTOR_SRCS) $(X87_REF_DIR)/x87_roundint.sv \
		$(X87_REF_DIR)/x87_addsub.sv $(X87_REF_DIR)/x87_mul.sv \
		$(X87_REF_DIR)/x87_divsqrt.sv $(X87_DIR)/x87_cordic_rom.sv \
		$(X87_DIR)/x87_cordic_scratch.sv \
		$(X87_REF_DIR)/x87_trans.sv \
		$(X87_DIR)/x87_ucode_rom.sv $(X87_DIR)/x87_sequencer.sv \
		$(X87_DIR)/x87_executor.sv $(CPP_X87_EXECUTOR_SRCS)
	@echo "Build complete: obj_dir/Vtb_x87_executor"

obj_dir_x87_testfloat/Vtb_x87_testfloat: \
		$(X87_DIR)/x87_ucode.svh $(X87_DIR)/x87_entries.svh \
		$(X87_DIR)/x87_pkg.sv $(X87_DIR)/x87_cordic_rom.sv \
		$(X87_DIR)/x87_cordic_scratch.sv \
		$(X87_DIR)/x87_ucode_pkg.sv $(X87_DIR)/x87_ucode_rom.sv \
		$(X87_DIR)/x87_sequencer.sv $(X87_DIR)/x87_executor.sv \
		$(TB_X87_TESTFLOAT_SRCS) $(CPP_X87_TESTFLOAT_SRCS)
	@echo "Building x87 TestFloat testbench with Verilator..."
	$(VERILATOR) $(VFLAGS_NOTRACE) --Mdir obj_dir_x87_testfloat \
		--top-module tb_x87_testfloat \
		$(X87_DIR)/x87_pkg.sv $(X87_DIR)/x87_ucode_pkg.sv \
		$(TB_X87_TESTFLOAT_SRCS) $(X87_DIR)/x87_cordic_rom.sv \
		$(X87_DIR)/x87_cordic_scratch.sv \
		$(X87_DIR)/x87_ucode_rom.sv $(X87_DIR)/x87_sequencer.sv \
		$(X87_DIR)/x87_executor.sv $(CPP_X87_TESTFLOAT_SRCS)
	@echo "Build complete: obj_dir_x87_testfloat/Vtb_x87_testfloat"

obj_dir_x87_core/Vtb_x87_core: $(X87_DIR)/x87_ucode.svh \
		$(X87_DIR)/x87_entries.svh $(X87_DIR)/x87_pkg.sv \
		$(X87_DIR)/x87_cordic_rom.sv \
		$(X87_DIR)/x87_cordic_scratch.sv $(X87_DIR)/x87_ucode_pkg.sv \
		$(X87_DIR)/x87_ucode_rom.sv $(X87_DIR)/x87_sequencer.sv \
		$(X87_DIR)/x87_executor.sv \
		$(X87_DIR)/x87_transfer_fifo.sv \
		$(X87_DIR)/x87_stack_mem.sv \
		$(X87_DIR)/x87_command_rom.sv \
		$(X87_DIR)/x87_control.sv $(TB_X87_CORE_SRCS) \
		$(CPP_X87_CORE_SRCS)
	@echo "Building x87 core unit test with Verilator..."
	$(VERILATOR) $(VFLAGS_NOTRACE) --Mdir obj_dir_x87_core \
		--top-module tb_x87_core $(TB_X87_CORE_SRCS) \
		$(X87_DIR)/x87_pkg.sv $(X87_DIR)/x87_cordic_rom.sv \
		$(X87_DIR)/x87_cordic_scratch.sv \
		$(X87_DIR)/x87_ucode_pkg.sv $(X87_DIR)/x87_ucode_rom.sv \
		$(X87_DIR)/x87_sequencer.sv $(X87_DIR)/x87_executor.sv \
		$(X87_DIR)/x87_transfer_fifo.sv \
		$(X87_DIR)/x87_stack_mem.sv $(X87_DIR)/x87_command_rom.sv \
		$(X87_DIR)/x87_control.sv \
		$(CPP_X87_CORE_SRCS)
	@echo "Build complete: obj_dir_x87_core/Vtb_x87_core"

obj_dir_x87_fifo/Vtb_x87_transfer_fifo: \
		$(X87_DIR)/x87_transfer_fifo.sv $(TB_X87_FIFO_SRCS) \
		$(CPP_X87_FIFO_SRCS)
	@echo "Building x87 transfer FIFO unit test with Verilator..."
	$(VERILATOR) $(VFLAGS_NOTRACE) --Mdir obj_dir_x87_fifo \
		--top-module tb_x87_transfer_fifo $(TB_X87_FIFO_SRCS) \
		$(X87_DIR)/x87_transfer_fifo.sv $(CPP_X87_FIFO_SRCS)
	@echo "Build complete: obj_dir_x87_fifo/Vtb_x87_transfer_fifo"

test: test-simple

# Run tests
test-simple: build
	@echo "Running z486 simple instruction tests..."
	./test_simple.py

test-full: build
	@echo "Running z486 full instruction tests..."
	./test_full_singlestep.py

test-protected: obj_dir/Vtb_protected_mode
	@echo "Running protected mode tests..."
	./test_protected_mode.py

test-386: obj_dir/Vtb_test386
	@echo "Running test386.asm..."
	./test386.py

test-l1-cache: obj_dir/Vtb_l1_cache
	@echo "Running L1 cache unit test..."
	./obj_dir/Vtb_l1_cache

test-l1-icache: obj_dir/Vtb_l1_icache
	@echo "Running L1 instruction cache unit test..."
	./obj_dir/Vtb_l1_icache

test-x87-bridge: obj_dir/Vtb_x87_bridge
	@echo "Running x87 bridge unit test..."
	./obj_dir/Vtb_x87_bridge

test-x87-sequencer: obj_dir/Vtb_x87_sequencer
	@echo "Running x87 sequencer unit test..."
	./obj_dir/Vtb_x87_sequencer

test-x87-executor: obj_dir/Vtb_x87_executor
	@echo "Running x87 executor unit test..."
	./obj_dir/Vtb_x87_executor

TESTFLOAT_GEN ?= $(CURDIR)/../../qemu/subprojects/berkeley-testfloat-3/build/Linux-x86_64-GCC/testfloat_gen
X87_TESTFLOAT_CASES ?= 512

test-x87-testfloat: obj_dir_x87_testfloat/Vtb_x87_testfloat
	@echo "Running x87 arithmetic against Berkeley TestFloat..."
	./run_x87_testfloat.sh "$(TESTFLOAT_GEN)" \
		./obj_dir_x87_testfloat/Vtb_x87_testfloat \
		"$(X87_TESTFLOAT_CASES)"

test-x87-fifo: obj_dir_x87_fifo/Vtb_x87_transfer_fifo
	@echo "Running x87 transfer FIFO unit test..."
	./obj_dir_x87_fifo/Vtb_x87_transfer_fifo

test-x87-core: obj_dir_x87_core/Vtb_x87_core
	@echo "Running x87 core unit test..."
	./obj_dir_x87_core/Vtb_x87_core

test-x87-integration: obj_dir_x87/Vtb_protected_mode
	@echo "Running x87-enabled CPU protocol tests..."
	Z486_ENABLE_X87=1 Z486_TESTBENCH=$(CURDIR)/obj_dir_x87/Vtb_protected_mode ./test_protected_mode.py x87_protocol x87_env32 x87_arithmetic x87_m32_page_fault x87_heap32 x87_qemu_special x87_qemu_exceptions

test-x87-integer: obj_dir_x87_int/Vtb_z486
	@echo "Running integer regressions with x87 enabled..."
	Z486_TESTBENCH=$(CURDIR)/obj_dir_x87_int/Vtb_z486 ./test_simple.py --no-color
	Z486_TESTBENCH=$(CURDIR)/obj_dir_x87_int/Vtb_z486 ./test_singlestep_real.py --no-color
	Z486_TESTBENCH=$(CURDIR)/obj_dir_x87_int/Vtb_z486 ./test_singlestep_protected.py -nt

dhrystone:
	$(MAKE) -C dhrystone compare

# Clean build artifacts
clean:
	rm -rf obj_dir_x87_testfloat
	rm -rf obj_dir_x87_fifo
	rm -rf obj_dir_x87_core
	rm -rf obj_dir_x87
	rm -rf obj_dir_x87_int
	rm -rf obj_dir
	rm -f trace.fst trace.vcd
	rm -f result.txt
	rm -f *.hex

# Help
help:
	@echo "z486 Test Makefile"
	@echo "=================="
	@echo ""
	@echo "Targets:"
	@echo "  make build          - Build all testbenches with Verilator"
	@echo "  make test-simple    - Run default test suite (~13 test files)"
	@echo "  make test-386       - Run test386.asm"
	@echo "  make test-l1-cache  - Run PIPT L1 cache unit test"
	@echo "  make test-l1-icache - Run PIPT L1 instruction cache unit test"
	@echo "  make test-x87-bridge - Run x87 protocol bridge unit test"
	@echo "  make test-x87-core - Run x87 state and transfer unit test"
	@echo "  make test-x87-testfloat - Check arithmetic with Berkeley TestFloat"
	@echo "  make test-x87-integration - Run x87-enabled CPU protocol test"
	@echo "  make dhrystone      - Compare released z386 and current z486 on Dhrystone"
	@echo "  make clean          - Remove build artifacts"
	@echo "  make help           - Show this help message"
