VERILATOR ?= verilator
VERILATOR_COMMON_FLAGS = -Wall -Wno-fatal -Wno-DECLFILENAME -Wno-WIDTH -Wno-UNUSED -Wno-CASEINCOMPLETE -Wno-COMBDLY

VERILATOR_GOLDEN_FLAGS = $(VERILATOR_COMMON_FLAGS) -y golden --cc golden/pistorm_golden.v --top-module pistorm_golden --prefix Vpistorm_golden --Mdir obj_dir_golden --trace

VERILATOR_FLAGS = $(VERILATOR_COMMON_FLAGS) -y . --cc PS32-lite.v pi_interface.v zorro_device.v m68k_interface.v --top-module pistorm --trace
VERILATOR_FLAGS += --Mdir obj_dir
VERILATOR_FLAGS += -CFLAGS "-std=c++17 -O2 -I../tb -I../obj_dir_golden"
VERILATOR_FLAGS += -LDFLAGS "../obj_dir_golden/Vpistorm_golden__ALL.a"
VERILATOR_FLAGS += --exe ../tb/tb_main.cpp ../tb/ps_pi_model.cpp ../tb/amiga_bus_model.cpp ../tb/m68k_timing_checker.cpp ../tb/pcb_components.cpp -o ../tb_pistorm32

all: build

golden_lib:
	mkdir -p golden
	$(VERILATOR) $(VERILATOR_GOLDEN_FLAGS)
	$(MAKE) -C obj_dir_golden -f Vpistorm_golden.mk

version:
	python3 scripts/gen_build_version.py

build: version golden_lib
	$(VERILATOR) $(VERILATOR_FLAGS)
	$(MAKE) -C obj_dir -f Vpistorm.mk

run: build
	./tb_pistorm32

test: build
	./tb_pistorm32

bench: build
	./tb_pistorm32 --bench

trace: build
	./tb_pistorm32 --trace

waveforms: trace
	python3 scripts/vcd2wavedrom.py --preset all

clean:
	rm -rf obj_dir obj_dir_golden tb_pistorm32 sim.vcd outflow work_syn work_pnr build_version.vh

EFINITY_SETUP ?= /home/claude/efinity/2026.1/bin/setup.sh

bitstream: version
	bash -c "if command -v efx_run >/dev/null 2>&1; then efx_run --prj -f compile PS32-lite; elif [ -f $(EFINITY_SETUP) ]; then source $(EFINITY_SETUP) && efx_run --prj -f compile PS32-lite; else echo 'ERROR: efx_run not found'; exit 1; fi"
	gzip -c -9 outflow/PS32-lite.hex.bin > firmware.bin.gz

M68K_CC ?= /opt/amiga/bin/m68k-amigaos-gcc
M68K_CFLAGS ?= -O2 -noixemul -m68020 -s

amiga_tools:
	mkdir -p tools
	$(M68K_CC) $(M68K_CFLAGS) -DMODE_NO_FAST_DSACK tools/ps32_bus_ctrl.c -o tools/PS32_NoFastDSACK
	$(M68K_CC) $(M68K_CFLAGS) -DMODE_TURBO_OFF     tools/ps32_bus_ctrl.c -o tools/PS32_Turbo_OFF
	$(M68K_CC) $(M68K_CFLAGS) -DMODE_TURBO_ON      tools/ps32_bus_ctrl.c -o tools/PS32_Turbo_ON
	$(M68K_CC) $(M68K_CFLAGS)                      tools/ps32_bus_ctrl.c -o tools/PS32_BusCtrl
	$(M68K_CC) $(M68K_CFLAGS) tools/ps32_scope.c -o tools/PS32Scope
	$(M68K_CC) $(M68K_CFLAGS) tools/ps32_stress_test.c -o tools/PS32_StressTest
	$(M68K_CC) $(M68K_CFLAGS) tools/bench_sram.c -o tools/BenchSRAM

.PHONY: all build run test bench trace waveforms clean bitstream amiga_tools
