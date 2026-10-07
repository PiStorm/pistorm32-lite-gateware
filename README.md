# PiStorm32-lite Gateware

Efinix Trion T20 FPGA gateware for the PiStorm32-lite card (Commodore Amiga 1200 accelerator).

## Overview

PiStorm32-lite connects a Raspberry Pi (running EMU68 or Linux) to the Amiga 1200's 150-pin CPU expansion edge connector. The Pi acts as the CPU and Fast RAM, while the FPGA bridges host parallel GPIO requests into 68EC020 bus cycles for Chip RAM and custom chipset registers.

The gateware is split into three main modules:
- `m68k_interface.v`: MC68020 bus master state machine, dynamic bus sizing (DSACK), and clock deglitch filter.
- `pi_interface.v`: 16-bit parallel host bus, two-request-slot pipeline, and speculative read prefetch.
- `zorro_device.v`: Virtual Zorro-II AutoConfig device with Wishbone B4 interconnect, INT2/INT6 interrupt control, and an ESP32-style GPIO matrix.
- `PS32-lite.v`: Top-level pin mapping, PLL, and bus muxing.

## Live Amiga 1200 Hardware Benchmarks (`bustest CHIP`)

Measured directly on Commodore Amiga 1200 hardware with Raspberry Pi 3A+ running Emu68:

| Benchmark Transaction | Upstream Golden Reference | Ultra-Turbo Mode (Default) | Speedup / Gain | Status |
| :--- | :---: | :---: | :---: | :--- |
| **Chip RAM 16-bit Read (`readw`)** | 1430.2 ns (1.40 MB/s) | **645.7 ns (3.10 MB/s)** | **+121.4%** | Slashed from 1430ns to 646ns via 16-bit prefetch + CCK Phase 1 |
| **Chip RAM 32-bit Read (`readl`)** | 1536.8 ns (2.60 MB/s) | **739.7 ns (5.41 MB/s)** | **+107.7%** | Pipelined 32-bit speculative prefetch + Fast DSACK |
| **Chip RAM Burst Read (`readm`)** | 1487.4 ns (2.69 MB/s) | **718.3 ns (5.57 MB/s)** | **+107.4%** | Pipelined 32-bit speculative prefetch + Fast DSACK |
| **Chip RAM 32-bit Write (`writel`)**| 569.8 ns (7.03 MB/s) | **569.7 ns (7.03 MB/s)** | **100% Line Rate**| Optimal 2-slot line rate (456ns Alice bus cycle) |

## Features & Highlights

- **Ultra-Turbo Bus Engine (Default-Active):**
  - **Fast DSACK Termination:** Eliminates 82 ns post-DSACK dead time by terminating on the falling edge of `/DSACK`.
  - **7.09 MHz CCK Phase Synchronization & Calibration:** Synchronizes `/AS` assertions with Alice's internal DMA slot boundaries, completely eliminating random 70 ns phase wait states. Auto-calibrates against DMA-free fast cycles.
  - **16-bit & 32-bit Speculative Read-Ahead:** Prefetches next word/longword into a 0-wait-state buffer with 100% hardware write coherency invalidation.
- **Micronik 6860 Busboard Compatibility:**
  - Configures `MC_BG_n` with internal `weak pulldown` (~50 kΩ). Solves floating `_BG` pin on Micronik 6860 busboards (v4.20/v5.42) without requiring hardware wire jumpers or soldering.
- **Virtual Zorro-II AutoConfig & Wishbone B4:**
  - 64 KB AutoConfig expansion space (`$00E90000`) with 182 MHz 0-WS Wishbone crossbar, dual-port scratchpad, INT2/INT6 interrupt engine, and GPIO matrix.
  - Hardware bus diagnostic and telemetry profilers (`+$1C`..`+$34`).
- **100% Glitch & Ringing Immunity:**
  - Synchronous digital filter protects against 1.8V inductive ringing dips on unmodified A1200 motherboards (`E121`/`E122`).

## Building & Verification

### Verilator Simulation
Prerequisites: Verilator (>= 4.200), g++ or clang++ (C++17), make.

```bash
make test       # Run full verification suite (338 assertions + golden reference parity)
make bench      # Run performance benchmarks and print comparison table
make trace      # Run simulation and dump waveforms to sim.vcd
make waveforms  # Extract simulation traces and render WaveDrom vector SVG diagrams
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

The ready-to-use bitstream is pre-compiled and tracked in the repository as [`firmware.bin.gz`](firmware.bin.gz).

### With Emu68 (Bare-Metal JIT)
1. Copy [`firmware.bin.gz`](firmware.bin.gz) directly onto the FAT32 boot partition of your Raspberry Pi SD card (place it in the root folder alongside `Emu68.img`).
2. Power on or reset the Amiga. Emu68 automatically programs the Efinix Trion T20 FPGA over GPIO bitbang at boot time.

### With PiStorm Linux
1. Copy [`firmware.bin.gz`](firmware.bin.gz) to your Raspberry Pi:
   ```bash
   sudo cp firmware.bin.gz /usr/share/pistorm/
   ```
2. Or program it directly using the flasher script:
   ```bash
   sudo ./flash.sh firmware.bin.gz
   ```

<details>
<summary><b>FPGA Programming GPIO Pinout (Hardware Details)</b></summary>

Emu68 and PiStorm Linux program the FPGA over a high-speed parallel/SPI bitbang interface using Raspberry Pi GPIOs:

| Signal | Raspberry Pi BCM GPIO | Description |
| :--- | :---: | :--- |
| `PIN_CRESET1` | GPIO 6 | FPGA Configuration Reset 1 |
| `PIN_CRESET2` | GPIO 7 | FPGA Configuration Reset 2 |
| `PIN_TESTN` | GPIO 17 | Test Mode Select |
| `PIN_CCK` | GPIO 22 | Configuration Clock |
| `PIN_SS` | GPIO 24 | Slave Select / Chip Select |
| `PIN_CBUS[2:0]` | GPIO 18, 15, 14 | Configuration Bus Control |
| `PIN_CDI[7:0]` | GPIO 13, 16, 1, 11, 8, 9, 25, 10 | Byte-Wide Configuration Data |

</details>

---

## Support & Donations

If you enjoy this project and would like to support my ongoing work on PiStorm and Amiga hardware, I would greatly appreciate a donation! Every contribution is very welcome and helps keep the project going. ☕

**Claude Schwarz** ([@captain-amygdala](https://github.com/captain-amygdala))

[![](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=JQC4M73U9KKPG)

### PiStorm Community Discord
Join the conversation on the official PiStorm Discord:
[![](https://dcbadge.limes.pink/api/server/vyHr6nQeGn)](https://discord.gg/vyHr6nQeGn)


