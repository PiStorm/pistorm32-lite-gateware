VERILATOR ?= verilator
VERILATOR_FLAGS = -Wall -Wno-fatal -Wno-DECLFILENAME -Wno-WIDTH -Wno-UNUSED -Wno-CASEINCOMPLETE -Wno-COMBDLY
VERILATOR_FLAGS += -y . --cc PS32-lite.v pi_interface.v zorro_device.v m68k_interface.v --top-module pistorm --trace
VERILATOR_FLAGS += --Mdir obj_dir
VERILATOR_FLAGS += -CFLAGS "-std=c++17 -O2 -I../tb"
VERILATOR_FLAGS += --exe ../tb/tb_main.cpp ../tb/ps_pi_model.cpp ../tb/amiga_bus_model.cpp ../tb/m68k_timing_checker.cpp ../tb/pcb_components.cpp -o ../tb_pistorm32

all: build

build:
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

clean:
	rm -rf obj_dir tb_pistorm32 sim.vcd outflow work_syn work_pnr

bitstream:
	efx_run --prj -f compile PS32-lite
	gzip -c -9 outflow/PS32-lite.hex.bin > firmware.bin.gz

.PHONY: all build run test bench trace clean bitstream
