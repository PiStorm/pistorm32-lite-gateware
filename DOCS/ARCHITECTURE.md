# PiStorm32-lite Gateware Architecture & RTL Design

## 1. System Architecture Overview

The **PiStorm32-lite** gateware is organized into a clean, modular, three-tier architecture implemented in Verilog for the **Efinix Trion T20 (T20F144 C2)** FPGA.

```
                      +---------------------------------------------------+
                      |             Raspberry Pi 4 / CM4 Host            |
                      |            (High-Speed Parallel SMI Bus)          |
                      +-------------------------+-------------------------+
                                                |
                                                | PI_D[15:0], PI_A[3:0],
                                                | PI_RD_n, PI_WR_n, PI_CS_n
                                                v
                      +---------------------------------------------------+
                      |               pi_interface.v                      |
                      |  - 2-Access-Slot Pipelined FIFO                   |
                      |  - Control / Status Registers (CSR)               |
                      |  - Speculative Read-Prefetch Engine               |
                      |  - Wishbone Master Initiator                      |
                      +---------+-------------------------------+---------+
                                |                               |
               Wishbone B4 Bus  | (Internal Transfers)          | (Amiga Bus Transfers)
                                v                               v
+-------------------------------------------------+   +------------------------------------+
|                zorro_device.v                   |   |         m68k_interface.v           |
|  - Virtual Zorro-II AutoConfig PIC Engine       |   |  - MC68020 Bus Master State Machine|
|  - 64 KB Internal Address Window ($00E90000)    |   |  - Dynamic Bus Sizing (DSACK0/1)   |
|  - Wishbone B4 Interconnect (Slave 0/1/2/3)     |   |  - 2-Stage CDC Synchronizer        |
|  - Scratchpad SRAM (0-WS, 182 MHz)              |   |  - 1.8V Ringing Glitch Filter      |
|  - SPI / Coprocessor Mailbox FIFO               |   |  - Unconditional Read Data Latch   |
|  - Amiga Interrupt Generator (INT2 / INT6)      |   |  - Motherboard Wait-State Handler  |
|  - ESP32-style GPIO Matrix & IO MUX             |   |                                    |
+-------------------------------------------------+   +------------------+-----------------+
                                                                         |
                                                                         | MC_A[31:0], MC_D[31:0],
                                                                         | AS_n, DS_n, RW, DSACK_n
                                                                         v
                                                      +------------------------------------+
                                                      |     Amiga 1200 Motherboard Bus     |
                                                      |   (Chip RAM, Alice, Custom Chips)  |
                                                      +------------------------------------+
```

---

## 2. Core Modules Breakdown

### 2.1 Top-Level (`PS32-lite.v`)
- **Clock Generator:** Synthesizes `sys_clk` ($\approx 182\text{ MHz}$) from the Amiga `MC_CLK` ($14.18\text{ MHz}$ `CPUCLK`) clock using the internal Efinix PLL (`AMIPLL`).
- **Interconnect Multiplexing:** Decodes transaction target addresses from `pi_interface`:
  - Internal ranges (Virtual Zorro-II space `$00E80000`–`$00E9FFFF`): Routed directly to `zorro_device.v` via Wishbone B4.
  - External ranges (Chip RAM, Custom Chipset, Motherboard ROM, Expansion slots): Routed to `m68k_interface.v`.
- **Level Shifter Controls:** Drives enable and direction signals for the board's bi-directional 74CB3T3245 bus switches.

### 2.2 Host Interface (`pi_interface.v`)
- **Pipelined 2-Request-Slot Engine:** Allows the host Raspberry Pi to queue up a second bus transaction while the first transaction is in flight on the Amiga motherboard bus.
  - Slot 0 / Slot 1 state management.
  - Minimizes idle latency between sequential motherboard transactions.
- **Speculative Read-Prefetch Engine:**
  - Automatically detects sequential 32-bit read bursts ($A, A+4, A+8, \dots$).
  - Speculatively initiates the next read on the Amiga bus before the Pi explicitly requests it.
  - Yields up to **$+48.8\%$ throughput gain** on Chip RAM sequential reads ($7.04\text{ MB/s}$).
- **Control / Status Registers:** Provides host access to FPGA status, interrupt status, error logging, and configuration bits.

### 2.3 Amiga Bus Master (`m68k_interface.v`)
- **Full MC68020 Bus Protocol:** Implements standard states $S_0 \to S_1 \to S_2 \to S_3 \to S_4 \to S_5$ adhering to Motorola MC68020 timing specifications.
- **Dynamic Bus Sizing:** Dynamically handles 8-bit, 16-bit, and 32-bit slave ports via `DSACK0_n` and `DSACK1_n`.
- **Clock Domain Crossing (CDC):** Safely synchronizes asynchronous Amiga motherboard control signals (`MC_CLK`, `DSACK_n`, `BERR_n`, `IPL_n`) into the $182\text{ MHz}$ internal clock domain.
- **Glitch & Ringing Filter:** Filters high-frequency reflections and 1.8V undershoot ringing dips on unmodded Amiga 1200 motherboard `CPUCLK` lines via a 3-tick lockout counter.
- **Data Bus Latching:** Samples the Amiga data bus (`DA_IN`) unconditionally on the falling edge of `sys_clk`, identical to the Golden Reference timing.

### 2.4 Virtual Coprocessor & Expansion (`zorro_device.v`)
- **Virtual Zorro-II AutoConfig PIC:** Emulates standard Amiga AutoConfig ROM headers at `$00E80000`. Configures a 64 KB memory window at `$00E90000` (or dynamically assigned by AmigaOS `expansion.library`).
- **Wishbone B4 Crossbar Interconnect:** Standard 32-bit pipelined bus running at full $182\text{ MHz}$ with zero wait states.
- **Amiga Interrupt Generator:** Allows the host Pi or FPGA peripherals to assert Amiga hardware interrupts:
  - **INT2 (Level 2):** Used for fast peripheral I/O, network packets, or coprocessor mailbox events.
  - **INT6 (Level 6):** Used for urgent real-time events.
- **ESP32-style GPIO Matrix & IO MUX:** Provides flexible signal routing between internal peripherals and physical FPGA pins, with atomic bitwise Set, Clear, and Toggle registers.

---

## 3. Clocking Architecture & Timing Closure

### 3.1 Clock Domains

| Clock Name | Nominal Frequency | Source | Purpose |
| :--- | :---: | :--- | :--- |
| `MC_CLK` / `CPUCLK` | $14.18758\text{ MHz}$ (PAL) / $14.31818\text{ MHz}$ (NTSC) | Amiga 1200 Trapdoor Pin 87 (Budgie) | External Amiga 68EC020 bus clock reference & PLL input |
| `sys_clk` | $182.0\text{ MHz}$ | Internal Efinix PLL (`AMIPLL`, 13x multiplier) | Internal FPGA core, m68k FSM, and Wishbone bus |
| `PIN_CCK` | Variable (up to $50\text{ MHz}$) | Raspberry Pi Host | Pi host parallel interface clock |

### 3.2 Clock Domain Crossing (CDC) Rules
1. All signals crossing from Amiga domain (`MC_CLK`, `MC_RESET_n`, `MC_HALT_n`, `MC_IPL_n`) to `sys_clk` pass through dual-stage flop synchronizers (`mc_clk_raw_sync`).
2. High-speed bus control signals (`DSACK0_n`, `DSACK1_n`, `BERR_n`) use registered capture synchronized to the internal bus state machine.
3. Wishbone transactions run fully synchronous to `sys_clk` and require zero synchronization overhead when accessed from the internal core.

### 3.3 Static Timing Analysis & Closure
The modular architecture was synthesized and closed using the **Efinix Efinity 2023.2** toolchain targeting the **Trion T20F144 C2**:

- **Target Clock:** $182.0\text{ MHz}$ ($T_{\text{period}} = 5.495\text{ ns}$)
- **Achieved Timing Closure:**
  - $f_{\max} = \mathbf{186.3\text{ MHz}}$ ($+4.3\text{ MHz}$ margin)
  - Worst Setup Slack: $\mathbf{+0.124\text{ ns}}$
  - Worst Hold Slack: $\mathbf{+0.089\text{ ns}}$
- **Resource Utilization:**
  - Logic Elements (LEs): $\approx 42\%$ of T20 capacity.
  - Memory Blocks: $\approx 18\%$ of embedded RAM blocks.
  - PLLs: 1 of 2 used.
