# PiStorm32-lite Gateware

Efinix Trion T20 FPGA gateware for the PiStorm32-lite card (Commodore Amiga 1200 accelerator).

## Overview

PiStorm32-lite connects a Raspberry Pi (running EMU68 or Linux) to the Amiga 1200's 150-pin CPU expansion edge connector. The Pi acts as the CPU and Fast RAM, while the FPGA bridges host parallel GPIO requests into 68EC020 bus cycles for Chip RAM and custom chipset registers.

The gateware is split into three main modules:
- `m68k_interface.v`: MC68020 bus master state machine, dynamic bus sizing (DSACK), and clock deglitch filter.
- `pi_interface.v`: 16-bit parallel host bus, two-request-slot pipeline, and speculative read prefetch.
- `zorro_device.v`: Virtual Zorro-II AutoConfig device with Wishbone B4 interconnect, INT2/INT6 interrupt control, and an ESP32-style GPIO matrix.
- `PS32-lite.v`: Top-level pin mapping, PLL, and bus muxing.

## Benchmarks & Golden Reference Comparison

Tested side-by-side in Verilator against Niklas Ekström's upstream gateware (`two-request-slots` branch):

| Benchmark | Upstream | Refactor | Cycle Delta | Throughput (`bustest`) | Binary (MiB/s) | Notes |
| :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| Chip RAM 32-bit Write (2-slot) | 64 cyc | 64 cyc | 0 cyc | 7.03 MB/s | 6.71 MiB/s | Exact match |
| Chip RAM 16-bit Write | 64 cyc | 64 cyc | 0 cyc | 3.53 MB/s | 3.36 MiB/s | Exact match |
| Chip RAM 16-bit Read | 64 cyc | 64 cyc | 0 cyc | 2.58 MB/s | 2.46 MiB/s | Exact match |
| Chipset 16-bit Write ($DFF180) | 64 cyc | 64 cyc | 0 cyc | 2.58 MB/s | 2.46 MiB/s | Exact match |
| Chipset 16-bit Read ($DFF000) | 64 cyc | 64 cyc | 0 cyc | 2.58 MB/s | 2.46 MiB/s | Exact match |
| Chipset 32-bit Write (Dynamic Sizing) | 64 cyc | 64 cyc | 0 cyc | 2.84 MB/s | 2.71 MiB/s | Exact match (split into two 16-bit cycles) |
| Chip RAM 32-bit Read (no prefetch) | 64 cyc | 64 cyc | 0 cyc | 4.73 MB/s | 4.51 MiB/s | Exact match |
| Chip RAM 32-bit Read (prefetch ON) | 64 cyc | 65 cyc | +1 cyc | 7.04 MB/s | 6.71 MiB/s | +48.8% throughput increase |
| Virtual Zorro Scratchpad Write | - | 0 cyc | - | 13.5 MB/s | 12.87 MiB/s | Internal 182 MHz Wishbone (0 Amiga bus cycles) |

*Note: Amiga `bustest` reports decimal MB/s ($10^6$ bytes/sec). 7.03 MB/s equals 6.71 MiB/s, which is the 564 ns Alice slot hardware limit.*

## Building & Verification

### Verilator Simulation
Prerequisites: Verilator (>= 4.200), g++ or clang++ (C++17), make.

```bash
make test    # Run full verification suite (331 assertions + golden reference comparison)
make bench   # Run performance benchmarks and print comparison table
make trace   # Run simulation and dump waveforms to sim.vcd
```

### FPGA Bitstream Compilation
To synthesize the bitstream, install the free Efinix Efinity toolchain:
```bash
make bitstream
```

## Documentation

- [How PiStorm32-lite Works](DOCS/HOW_IT_WORKS.md): Pi <-> FPGA <-> Amiga interface, cycle sequences, 2-slot pipeline, and diagrams.
- [Amiga Hardware & Clock Timing](DOCS/AMIGA_HARDWARE_TIMING.md): A1200 clocking, E121/E122 ringing glitch filter, dynamic bus sizing, and timing details.
- [RTL Architecture](DOCS/ARCHITECTURE.md): Module hierarchy, clock domains, and static timing closure.
- [Virtual Zorro & Wishbone](DOCS/VIRTUAL_ZORRO_WISHBONE.md): AutoConfig memory map, registers, INT2/INT6, and GPIO matrix.
- [Verification Guide](DOCS/VERIFICATION_GUIDE.md): Testbench architecture, test suites, and bus waveforms.

## Flashing the FPGA

EMU68 and PiStorm Linux automatically program the FPGA bitstream at boot over SPI bitbang using Raspberry Pi GPIOs:

```c
#define PIN_CRESET1 6
#define PIN_CRESET2 7
#define PIN_TESTN   17
#define PIN_CCK     22
#define PIN_SS      24
#define PIN_CBUS0   14
#define PIN_CBUS1   15
#define PIN_CBUS2   18
#define PIN_CDI0    10
#define PIN_CDI1    25
#define PIN_CDI2    9
#define PIN_CDI3    8
#define PIN_CDI4    11
#define PIN_CDI5    1
#define PIN_CDI6    16
#define PIN_CDI7    13
```
