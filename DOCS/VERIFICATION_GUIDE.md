# Verification Guide

The testbench uses Verilator to co-simulate two implementations side-by-side:
1. `Vpistorm`: The refactored gateware (3-block architecture, prefetch, glitch filter, virtual Zorro).
2. `Vpistorm_golden`: Niklas Ekström's unmodified upstream gateware (`PS32-lite.v` from `two-request-slots` branch).

This verifies that new features introduce no timing regressions on standard Amiga bus cycles ($\Delta = 0$ cycles).

```mermaid
flowchart TD
    TB["<b>Verilator C++ Testbench</b><br/>(tb/tb_main.cpp)"]
    
    TB --> GOLDEN["<b>Golden Reference</b><br/>Vpistorm_golden<br/><i>(Unmodified Upstream)</i>"]
    TB --> REFACTOR["<b>Refactored Gateware</b><br/>Vpistorm<br/><i>(Ultra-Turbo & Prefetch)</i>"]
    
    GOLDEN --> COMP["<b>Side-by-Side Comparison</b><br/>• Cycle Count Check (Delta = 0)<br/>• Throughput & Latency Measurement<br/>• 338 Assertions Passing"]
    REFACTOR --> COMP
```

---

## 1. Testbench Structure (`tb/`)

| File | Role |
| :--- | :--- |
| `tb/tb_main.cpp` | Main simulation driver, 15 test suites, 338 assertions, benchmark reporting |
| `tb/amiga_bus_model.h/.cpp` | Cycle-accurate A1200 motherboard model (Alice 560ns slot, chipset wait states, DSACK) |
| `tb/ps_pi_model.h/.cpp` | Raspberry Pi host model (parallel GPIO bus, 2-slot queue, prefetch testing) |
| `tb/m68k_timing_checker.h/.cpp` | Checks 68020 setup/hold times and bus protocol rules |
| `tb/pcb_components.h/.cpp` | Models 74CB3T3245 bus switches, propagation delays, pull-ups |
| `golden/pistorm_golden.v` | Unmodified upstream source compiled into `obj_dir_golden/` |

---

## 2. Running Tests

```bash
# Run full test suite (338 assertions + golden reference comparison)
make test

# Run standalone benchmark suite and side-by-side performance table
make bench

# Run simulation and dump waveforms to sim.vcd
make trace
```

---

## 3. Test Suites

1. **Reset & Initialization:** Power-on state, PLL lock sync, bus tri-stating.
2. **Basic M68k Cycles:** 8, 16, and 32-bit single transfers (including Test 2.1: Micronik `MC_BG_n` pull-down bus master verification).
3. **Dynamic Bus Sizing:** 8, 16, and 32-bit DSACK handshaking and wait states.
4. **Bus Error & Autovector:** Unmapped access, timeout aborts, and BERR.
5. **Pi SMI Protocol:** Parallel register reads/writes and status flags.
6. **Two-Request-Slot Pipeline:** Back-to-back queued transactions without idle cycles.
7. **Speculative Read Prefetch:** Sequential read acceleration and hit detection.
8. **Prefetch Cache Invalidation:** Flushes on address branches, random reads, and writes.
9. **Virtual Zorro AutoConfig:** AutoConfig ROM nibbles at `$00E80000`.
10. **Wishbone B4 Crossbar:** 0-wait-state transfers to internal slaves ($182\text{ MHz}$).
11. **Amiga Interrupts:** Level 2 (INT2) and Level 6 (INT6) assertion, masking, and clear.
12. **ESP32 GPIO Matrix:** Direct I/O, atomic SET/CLR/TOGGLE, input/output muxing.
13. **Clock Ringing Immunity:** Injects 1.8V dips on falling `MC_CLK` edges to test lockout filter.
14. **Timing Checker:** Protocol, setup/hold verification, and Wishbone Slave 0 `BUS_CTRL` / hardware diagnostic profilers (Test 14.7).
15. **Golden Reference Comparison:** Co-simulation against `Vpistorm_golden`.

---

## 4. Signal Waveforms

### 4.1 68020 Read Cycle (States S0 -> S5)

```mermaid
sequenceDiagram
    autonumber
    participant SYS as "sys_clk (182 MHz)"
    participant CLK as "Amiga MC_CLK (14.18 MHz)"
    participant FSM as "m68k FSM State"
    participant ADDR as "MC_A[31:0]"
    participant AS as "MC_AS_n"
    participant DS as "MC_DS_n"
    participant DSACK as "MC_DSACK[1:0]_n"
    participant DATA as "DA_IN"
    participant LATCH as "mc_data_read"

    Note over FSM: S0: Idle
    CLK->>FSM: Rising edge
    FSM->>ADDR: Drive address ($00040000)
    FSM->>ADDR: ADDR_LE = 1, ADDR_OE_n = 0
    
    Note over FSM: S1: Assert Strobes
    FSM->>AS: MC_AS_n = LOW
    FSM->>DS: MC_DS_n = LOW
    
    Note over FSM: S2-S3: Alice / RAM access
    DSACK-->>FSM: MC_DSACK[1:0]_n = LOW (32-bit port)
    DATA-->>LATCH: RAM drives data
    
    Note over FSM: S4: Latch Data
    CLK->>LATCH: Falling edge: sample DA_IN
    LATCH->>LATCH: mc_data_read <= 0x12345678
    
    Note over FSM: S5: End Cycle
    FSM->>AS: MC_AS_n = HIGH
    FSM->>DS: MC_DS_n = HIGH
    DSACK-->>FSM: DSACK returns HIGH
```

**Simulation Waveform (WaveDrom SVG):**
![68020 Fast DSACK Read Waveform](waveforms/m68k_read_cycle.svg)

---

### 4.2 68020 Pipelined Write Cycle (2-Slot)

```mermaid
sequenceDiagram
    autonumber
    participant Host as "Host Queue"
    participant FSM as "m68k FSM State"
    participant ADDR as "MC_A[31:0]"
    participant RW as "MC_RW"
    participant AS as "MC_AS_n"
    participant DS as "MC_DS_n"
    participant DATA as "DA_OUT[31:0]"
    participant DSACK as "MC_DSACK[1:0]_n"

    Host->>FSM: Slot 0 Write ($00040000, $AABBCCDD)
    FSM->>ADDR: Drive Address
    FSM->>RW: MC_RW = LOW
    FSM->>DATA: DA_OUT = $AABBCCDD, DATA_OE_n = 0
    
    FSM->>AS: MC_AS_n = LOW
    FSM->>DS: MC_DS_n = LOW
    
    Note over DSACK: Alice latches data at 560ns slot
    DSACK-->>FSM: MC_DSACK[1:0]_n = LOW
    
    FSM->>AS: MC_AS_n = HIGH
    FSM->>DS: MC_DS_n = HIGH
    FSM->>DATA: DATA_OE_n = 1
    
    Note over Host,FSM: Slot 0 finishes, Slot 1 starts immediately with 0 idle cycles
```

**Simulation Waveform (WaveDrom SVG):**
![68020 Fast DSACK Write Waveform](waveforms/m68k_write_cycle.svg)

---

### 4.3 Wishbone B4 Internal Access ($00E90000)

```mermaid
sequenceDiagram
    autonumber
    participant Pi as "Raspberry Pi Host"
    participant WB_M as "Wishbone Master (pi_interface)"
    participant WB_S as "Wishbone Slave (zorro_device)"
    participant Amiga as "Amiga Motherboard Lines"

    Pi->>WB_M: Read $00E90204 (GPIO_IN)
    Note over WB_M: Internal address detected
    
    par Internal Wishbone (182 MHz)
        WB_M->>WB_S: wb_cyc = 1, wb_stb = 1, wb_adr = 0x0204
        WB_S-->>WB_M: wb_dat_o = 0x000000A5, wb_ack = 1 (1 clock)
        WB_M-->>Pi: Return data
    and Amiga Bus (Isolated)
        Note over Amiga: MC_AS_n = HIGH, MC_DS_n = HIGH<br/>Level shifters tri-stated<br/>0 Amiga bus cycles
    end
```

**Simulation Waveform (WaveDrom SVG):**
![Wishbone Bus Cycle Waveform](waveforms/wishbone_bus_cycle.svg)

---

### 4.4 Lockout Filter (1.8V Ringing Suppression)

```mermaid
sequenceDiagram
    autonumber
    participant Pin as "Pin MC_CLK (with 1.8V Dip)"
    participant S0 as "mc_clk_raw_sync[0]"
    participant S1 as "mc_clk_raw_sync[1]"
    participant Out as "mc_clk_filtered"

    Note over Pin: MC_CLK falls HIGH to LOW
    Pin->>Pin: Ringing bounce to 1.8V for 4 ns
    
    Note over S0,S1: Synchronizer samples bounce
    S0->>S0: Temporary 1
    S1->>S1: Delayed by 1 sys_clk (5.49 ns)
    
    Note over Out: Lockout counter holds state for 3 ticks (16.5 ns)
    Note over Out: Dip is suppressed, filtered clock stays clean LOW
```

**Simulation Waveform (WaveDrom SVG):**
![Clock Glitch Filter Waveform](waveforms/clock_glitch_filter.svg)

---

### 4.5 Speculative Prefetch Cache Hit (0 Wait States)

The host Pi requests address $A+4$ after a sequential read. Because the FPGA speculatively prefetched the word into its internal buffer, the transfer completes immediately in a single clock cycle with **0 wait states** and **0 physical Amiga motherboard cycles** (`MC_AS_n` remains idle/tri-stated):

**Simulation Waveform (WaveDrom SVG):**
![Speculative Prefetch Cache Hit Waveform](waveforms/prefetch_cache_hit.svg)

---

## 5. Waveform Generation & Viewing

### Extracting & Rendering WaveDrom SVG Diagrams
You can automatically extract key simulation traces from `sim.vcd` and render vector SVG diagrams:
```bash
# Run simulation, extract traces, and render SVGs via wavedrom-cli
make waveforms
```
The resulting SVGs and WaveJSON files are stored in `DOCS/waveforms/`.

### Interactive Viewing (GTKWave)

```bash
make trace
gtkwave sim.vcd
```

Signals of interest:
- Clocks: `TOP.pistorm.sys_clk`, `TOP.pistorm.MC_CLK`, `TOP.pistorm.u_m68k.mc_clk_filtered`
- Amiga Bus: `TOP.pistorm.u_m68k.state`, `TOP.pistorm.MC_A`, `TOP.pistorm.DA_IN`, `TOP.pistorm.MC_AS_n_OUT`, `TOP.pistorm.MC_DSACK_n`
- Wishbone: `TOP.pistorm.wb_cyc`, `TOP.pistorm.wb_stb`, `TOP.pistorm.wb_we`, `TOP.pistorm.wb_adr`, `TOP.pistorm.wb_dat_m2s`, `TOP.pistorm.wb_ack`
