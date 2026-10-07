# PiStorm32-lite Architecture

Efinix Trion T20 FPGA gateware design for the PiStorm32-lite accelerator.

---

## 1. Block Diagram

```mermaid
flowchart TD
    subgraph HOST["Host System"]
        RPI["Raspberry Pi 4 / CM4 Host<br/><i>(16-bit Parallel GPIO Bus)</i>"]
    end

    subgraph FPGA["PiStorm32-lite Gateware (Efinix Trion T20)"]
        PI_IF["<b>pi_interface.v</b><br/>• 2-Slot Request Queue<br/>• Control and Status Registers (CSR)<br/>• Speculative Read Prefetch Engine<br/>• Address Decoding & Wishbone Initiator"]
        
        ZORRO["<b>zorro_device.v</b><br/>• Virtual Zorro-II AutoConfig ($00E80000)<br/>• 64 KB Internal I/O Window ($00E90000)<br/>• Scratchpad SRAM (4 KB, 0-WS)<br/>• SPI / Mailbox FIFO<br/>• Amiga Interrupts (INT2 / INT6)<br/>• ESP32-style GPIO Matrix"]
        
        M68K["<b>m68k_interface.v</b><br/>• MC68020 Bus Master FSM (S0..S5)<br/>• Ultra-Turbo Fast DSACK & CCK Phase Sync<br/>• Dynamic Bus Sizing (DSACK0/1)<br/>• 14 MHz MC_CLK Lockout Glitch Filter<br/>• CDC Synchronizers & Level Shifter Controls"]
    end

    subgraph AMIGA["Amiga 1200 Hardware"]
        MOTHERBOARD["Amiga 1200 Motherboard Bus<br/><i>(Chip RAM, Alice, Custom Chips, Budgie, Gayle)</i>"]
    end

    RPI -->|"PI_D[15:0], PI_A[2:0]<br/>PI_RD, PI_WR"| PI_IF
    PI_IF -->|"Wishbone B4 Bus (182 MHz)<br/>(Internal $00E9xxxx)"| ZORRO
    PI_IF -->|"Amiga Bus Transfers"| M68K
    M68K -->|"MC_A[31:0], MC_D[31:0]<br/>AS#, DS#, RW, DSACK#"| MOTHERBOARD
```

---

## 2. Module Responsibilities

### `PS32-lite.v` (Top Level)
- Synthesizes `sys_clk` (182 MHz) from `MC_CLK` (14.18 MHz) using the Efinix PLL (`AMIPLL`).
- Decodes target addresses:
  - Internal ranges (`$00E80000`–`$00E9FFFF`): routed to `zorro_device.v`.
  - External ranges (Chip RAM, chipset, ROM): routed to `m68k_interface.v`.
- Generates direction and enable signals for the 74CB3T3245 bus switches.

### `pi_interface.v`
- Implements the 16-bit parallel host bus interface.
- Manages the two-request-slot queue, allowing the Pi to submit a second transaction while the first is running on the Amiga bus.
- Implements the speculative 32-bit read prefetch engine.
- Filters keyboard reset (`PI_KBRESET`) to avoid false Ctrl-Amiga-Amiga triggers during host-initiated resets.

### `m68k_interface.v`
- Executes standard MC68020 bus cycles ($S_0 \to S_1 \to S_2 \to S_3 \to S_4 \to S_5$).
- Ultra-Turbo bus engine: Fast DSACK termination (-82 ns dead time) and 7.09 MHz CCK phase alignment.
- DMA-immune CCK phase auto-calibration locking 100% of reads to Phase 1 (527 ns) and writes to Phase 0 (456 ns).
- Handles dynamic bus sizing via `DSACK0_n` / `DSACK1_n` (8-bit, 16-bit, and 32-bit ports).
- Filters 14.18 MHz `MC_CLK` with a 3-tick lockout counter to prevent false triggers from 1.8V ringing dips.
- Samples read data (`DA_IN`) unconditionally on falling clock edges, matching upstream gateware timing.

### `zorro_device.v`
- Emulates AutoConfig ROM nibbles at `$00E80000` (Manufacturer ID 28020, Product ID 0x32).
- Maps a 64 KB Wishbone B4 memory space at `$00E90000`.
- Wishbone slaves:
  - Slave 0 (`+$0000`): Scratchpad SRAM (4 KB, 0 wait states), `BUS_CTRL` (`+$1C`), and hardware profiler telemetry registers (`+$20`..`+$34`).
  - Slave 1 (`+$1000`): SPI / Coprocessor mailbox FIFO.
  - Slave 2 (`+$2000`): ESP32-style GPIO matrix and IO MUX.
  - Slave 3 (`+$3000`): Interrupt controller (asserts Amiga INT2 and INT6).

---

## 3. Clocking & Timing Closure

### Clock Domains

| Clock | Frequency | Source | Function |
| :--- | :---: | :--- | :--- |
| `MC_CLK` | 14.18758 MHz (PAL) / 14.31818 MHz (NTSC) | Amiga Trapdoor Pin 87 (Budgie) | Amiga bus clock reference & PLL input |
| `sys_clk` | 182.0 MHz | Efinix PLL (`AMIPLL`, 13x multiplier) | Internal FPGA core, FSM, and Wishbone bus |
| `PIN_CCK` | Variable (up to 50 MHz) | Raspberry Pi Host | Pi host parallel interface clock |

### Clock Domain Crossing
- Amiga bus signals (`MC_CLK`, `MC_RESET_n`, `MC_HALT_n`, `MC_IPL_n`) use 2-stage synchronizers (`mc_clk_raw_sync`).
- Control signals from the host Pi pass through registered inputs in `pi_interface.v`.
- Wishbone bus transactions run synchronously on `sys_clk`.

### Static Timing Analysis (Efinix Efinity 2023.2)
Target device: **Trion T20F144 C2**

- Target frequency: 182.0 MHz ($T_{\text{period}} = 5.495\text{ ns}$)
- Achieved $f_{\max}$: **186.3 MHz**
- Setup slack: **+0.124 ns**
- Hold slack: **+0.089 ns**
- Logic utilization: ~42% of T20 capacity
