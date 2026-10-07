# How PiStorm32-lite Works

This document explains how the Raspberry Pi, FPGA gateware, and Amiga 1200 motherboard interact during bus operations.

---

## 1. System Overview

The Amiga 1200 exposes the Motorola 68EC020 bus on a 150-pin trapdoor edge connector. When PiStorm32-lite is installed, it requests the bus via `_BR` and takes over bus mastership when Gary/Gayle asserts `_BG`. The motherboard's onboard 68EC020 CPU is tri-stated and stays idle.

The Raspberry Pi runs an emulator/JIT engine (EMU68 bare-metal or PiStorm Linux):
- 680x0 CPU instructions execute on the Pi's ARM core.
- Fast RAM lives in the Pi's LPDDR4 memory and runs at full ARM bus speeds.
- When code accesses Chip RAM (`$00000000`), custom registers (`$00DFF000`), or Kickstart ROM (`$00F80000`), the Pi routes the access over its GPIO bus to the FPGA.
- The FPGA runs a 68020 bus master state machine that drives the physical address, data, and strobe signals to the motherboard via 74CB3T3245 3.3V/5V level shifters.

```mermaid
flowchart TD
    subgraph Host ["Raspberry Pi 4 / CM4"]
        EMU68["EMU68 JIT / Linux Engine"]
        FastRAM["Pi LPDDR4 (Fast RAM)"]
        SMI["Parallel GPIO / SMI"]
        EMU68 <--> FastRAM
        EMU68 -->|Motherboard Access| SMI
    end

    subgraph Hardware ["PiStorm32-Lite Hardware"]
        LevelShifter["74CB3T3245 Level Shifters<br/>(3.3V <-> 5.0V)"]
        
        subgraph FPGA ["Efinix Trion T20 (182 MHz sys_clk)"]
            PI_IF["pi_interface.v<br/>2-Slot FIFO & Prefetch"]
            DEC{"Address Decoder"}
            M68K_IF["m68k_interface.v<br/>68020 Bus Master & Clock Filter"]
            ZORRO_IF["zorro_device.v<br/>Virtual Zorro-II & Wishbone B4"]
            
            PI_IF --> DEC
            DEC -->|External Access| M68K_IF
            DEC -->|Internal $00E90000| ZORRO_IF
        end
    end

    subgraph Amiga ["Amiga 1200 Motherboard"]
        Trapdoor["150-pin CPU Expansion Port"]
        Budgie["Budgie (Clock Gen: 14.18 MHz MC_CLK)"]
        Alice["Alice (560ns Slot Arbitration)"]
        ChipRAM["2 MB Chip RAM"]
        Chipset["Custom Chips: Paula, Lisa, CIAs<br/>($00DFF000 - $00DFF1FE)"]
        
        Trapdoor <--> Alice
        Trapdoor <--> Chipset
        Alice <--> ChipRAM
        Budgie -.->|MC_CLK 14.18 MHz| Trapdoor
    end

    SMI <==>|16-Bit Parallel Data & Control| PI_IF
    M68K_IF <==>|3.3V LVCMOS| LevelShifter
    LevelShifter <==>|5.0V TTL/CMOS| Trapdoor
```

---

## 2. Raspberry Pi Host Interface

The Pi interfaces to the FPGA using a 16-bit parallel bus on standard GPIO pins:

| GPIO Pins | Signal | Direction | Function |
| :--- | :--- | :---: | :--- |
| `GPIO[23:8]` | `PI_D[15:0]` | Bidirectional | Multiplexed data bus |
| `GPIO[26:24]` | `PI_A[2:0]` | Pi $\to$ FPGA | Register address |
| `GPIO6` | `PI_RD` | Pi $\to$ FPGA | Read strobe (active low) |
| `GPIO7` | `PI_WR` | Pi $\to$ FPGA | Write strobe (active low) |
| `GPIO[2:0]` | `PI_IPL[2:0]` | FPGA $\to$ Pi | Current Amiga interrupt level (active-high inverted `_IPL`) |
| `GPIO3` | `PI_TXN_IN_PROGRESS` | FPGA $\to$ Pi | Busy flag: 1 = Bus cycle active on Amiga bus |
| `GPIO4` | `PI_KBRESET` | FPGA $\to$ Pi | Filtered keyboard reset (Ctrl-Amiga-Amiga) |

### FPGA Host Registers (`PI_A[2:0]`)

```
  PI_A = 0 : [ DATA_LO ]  - Data [15:0]
  PI_A = 1 : [ DATA_HI ]  - Data [31:16]
  PI_A = 2 : [ ADDR_LO ]  - Address [15:0]
  PI_A = 3 : [ ADDR_HI ]  - Address [23:16], Size [9:8], R/W [10], FC [13:11] -> Triggers transaction
  PI_A = 4 : [ STATUS  ]  - Read: Status / Write: CONTROL (Reset, Halt, Prefetch, IRQ)
  PI_A = 5 : [ SLOT    ]  - Request slot select (0 or 1)
```

Writing to `ADDR_HI` (`PI_A = 3`) starts the transaction on the FPGA.

---

## 3. Read Transactions

When the Pi needs to read 32 bits from Chip RAM (e.g., `move.l ($00040000), d0`):

```mermaid
sequenceDiagram
    autonumber
    participant Pi as Raspberry Pi (EMU68)
    participant PI_IF as pi_interface.v
    participant M68K as m68k_interface.v
    participant Amiga as Amiga 1200 (Alice / RAM)

    Note over Pi,PI_IF: 1. Pi sets target address
    Pi->>PI_IF: Write ADDR_LO = 0x0000 (PI_A=2)
    Pi->>PI_IF: Write ADDR_HI = 0x0004 | READ | SIZE_32 (PI_A=3)
    Note over PI_IF: Falling edge on PI_WR triggers new_req_valid

    Note over PI_IF,M68K: 2. Dispatch to 68020 FSM
    PI_IF->>M68K: new_req_valid (Slot 0, Addr: 0x00040000, Longword, Read)
    
    Note over M68K,Amiga: 3. MC68020 Bus Cycle (S0 -> S5)
    M68K->>Amiga: Drive MC_A = 0x00040000, R/W = 1
    M68K->>Amiga: Assert MC_AS_n = LOW, MC_DS_n = LOW (S1)
    
    Note over Amiga: Alice syncs to 560ns slot.<br/>RAM drives data.
    Amiga-->>M68K: MC_DSACK[1:0]_n = LOW (32-bit port ready)
    
    Note over M68K: 4. Latch data on falling clock edge
    M68K->>M68K: mc_data_read <= DA_IN (0x12345678)
    M68K->>Amiga: Negate MC_AS_n, MC_DS_n (S5)
    
    Note over M68K,PI_IF: 5. Complete slot
    M68K->>PI_IF: slot_complete_valid (Data: 0x12345678, Normal: 1)
    Note over PI_IF: req_active[0] <= 0, PI_TXN_IN_PROGRESS = 0

    Note over Pi,PI_IF: 6. Pi reads back data
    Pi->>PI_IF: Read DATA_LO (PI_A=0) -> 0x5678
    Pi->>PI_IF: Read DATA_HI (PI_A=1) -> 0x1234
```

---

## 4. Two-Request-Slot Pipeline (Writes)

Chip RAM writes take 560 ns each due to Alice's slot timing. If the Pi waited synchronously for every write to complete, software writes would be blocked by bus latency.

The two-request-slot pipeline removes this delay:
- While **Slot 0** is executing on the motherboard, the Pi writes the next data and address into **Slot 1**.
- As soon as Slot 0 finishes, the FPGA immediately begins Slot 1 with zero idle cycles.
- This keeps the Amiga bus fully saturated at **7.03 MB/s (6.71 MiB/s)**.

```mermaid
sequenceDiagram
    autonumber
    participant Pi as Raspberry Pi
    participant Slot0 as Request Slot 0
    participant Slot1 as Request Slot 1
    participant Bus as Amiga Motherboard Bus

    Pi->>Slot0: Write DATA_LO, DATA_HI, ADDR_LO, ADDR_HI
    Note over Slot0,Bus: Slot 0 active -> cycle starts
    Slot0->>Bus: Amiga Bus Cycle #1 (560 ns)

    Note over Pi,Slot1: Pi switches to Slot 1 immediately:
    Pi->>Slot1: Write DATA_LO, DATA_HI, ADDR_LO, ADDR_HI
    Note over Slot1: Slot 1 queued in FPGA

    Bus-->>Slot0: DSACK acknowledged (Slot 0 done)
    
    Note over Slot1,Bus: FPGA starts Slot 1 with no idle cycles
    Slot1->>Bus: Amiga Bus Cycle #2 (560 ns)
```

---

## 5. Speculative Read Prefetch

Code execution and data copies often read memory in sequential longwords (`$00040000`, `$00040004`, `$00040008`, ...).

With prefetch enabled:
1. When the Pi reads address $A$, the FPGA completes the cycle normally.
2. The FPGA immediately starts a speculative cycle for address $A+4$ on the Amiga bus and buffers the result.
3. When the Pi requests address $A+4$, the FPGA serves the buffered data immediately with zero bus wait states (**Cache Hit**).
4. If the Pi jumps to an unrelated address (branch), the prefetch buffer is invalidated and a normal bus cycle is run (**Cache Miss**).
5. Sequential 32-bit reads speed up from **4.73 MB/s to 7.04 MB/s (+48.8%)**.

```mermaid
flowchart TD
    Req["Pi reads address A"] --> Exec["Run Amiga bus cycle for address A"]
    Exec --> ReturnData["Return Data(A) to Pi"]
    
    ReturnData --> PrefetchOn{"Prefetch enabled &<br/>32-bit read?"}
    PrefetchOn -- No --> Done["Done"]
    
    PrefetchOn -- Yes --> FetchAhead["FPGA starts speculative cycle for (A + 4)"]
    FetchAhead --> Buffer["Save data in prefetch buffer (valid = 1)"]
    
    Buffer --> NextReq["Next read request from Pi (Address B)"]
    NextReq --> HitCheck{"B == (A + 4) &<br/>valid?"}
    
    HitCheck -- "Hit" --> InstantReturn["Return buffered data immediately<br/>(0 wait states, 7.04 MB/s)"]
    HitCheck -- "Miss" --> Invalidate["Flush buffer, run standard cycle for B"]
```

**Speculative Prefetch Simulation Waveform (WaveDrom SVG):**
![Speculative Prefetch Cache Hit Waveform](waveforms/prefetch_cache_hit.svg)

---

## 6. Virtual Zorro-II & Wishbone Interconnect

Addresses in the range `$00E90000`–`$00E9FFFF` are assigned to the virtual Zorro-II expansion card:
- Address decoding happens inside the FPGA.
- The external Amiga bus lines (`AS#`, `DS#`, `ADDR_OE#`, `DATA_OE#`) remain high and tri-stated.
- Transactions are routed to the internal **Wishbone B4 bus** running at **182 MHz** with **0 wait states** ($13.5\text{ MB/s}$).
- The Amiga bus is completely untouched, leaving full bandwidth for chipset DMA (display, sound, blitter).

```mermaid
sequenceDiagram
    autonumber
    participant Pi as Raspberry Pi / Amiga CPU
    participant Dec as Address Decoder
    participant WB as Wishbone Master
    participant Slave as Wishbone Slave (GPIO/Mailbox)
    participant Amiga as Amiga Motherboard Bus

    Pi->>Dec: Read/Write $00E90200
    Note over Dec: Address is internal ($00E9xxxx)
    
    par Internal Wishbone (182 MHz)
        Dec->>WB: wb_cyc = 1, wb_stb = 1, wb_adr = 0x0200
        WB->>Slave: Register access
        Slave-->>WB: wb_ack = 1 (1 clock @ 182 MHz)
        WB-->>Pi: Data ready (282 ns host latency)
    and Amiga Bus (Isolated)
        Note over Amiga: MC_AS_n = HIGH, MC_DS_n = HIGH<br/>Level shifters tri-stated<br/>0 motherboard cycles consumed
    end
```

---

## 7. Motherboard Clock Ringing & The Lockout Filter

Commodore Amiga 1200 motherboards (especially revisions 1D.4 and 2B) have ferrite beads (`E121`, `E122`) on the **14.18 MHz `CPUCLK` (`MC_CLK`)** trace. These cause inductive ringing on the falling clock edge, dipping down to around **1.8V**:

```
14.18 MHz MC_CLK (CPUCLK) Ringing Waveform:
5.0V |-------+
     |        \
2.0V |---------\---[ VIH Threshold ]-----------------------------
     |          \
1.8V |           \   /---\  <-- 1.8V Ringing Dip (false edge risk)
     |            \_/     \
0.8V |---------------------\-[ VIL Threshold ]-------------------
     |                      \
0.0V |                       +-----------------------------------
```

If an unfiltered input buffer samples during the dip, it can register a false clock edge, desynchronizing the bus state machine.

### Implementation in `m68k_interface.v`:
1. `MC_CLK` is sampled by the 182 MHz internal PLL clock (`sys_clk`), giving ~13 samples per 14.18 MHz clock cycle.
2. A 2-stage synchronizer (`mc_clk_raw_sync`) removes metastability.
3. A 3-tick lockout counter (`MC_CLK_LOCKOUT_TICKS = 3`) locks out edge detection for 3 `sys_clk` cycles ($16.5\text{ ns}$) after each transition, blanking out the 1.8V bounce:

```verilog
localparam [2:0] MC_CLK_LOCKOUT_TICKS = 3'd3;

(* async_reg = "true" *) reg [1:0] mc_clk_raw_sync = 2'b00;
reg       mc_clk_filtered = 1'b0;
reg [2:0] mc_clk_lockout  = 3'd0;
reg       rising          = 1'b0;
reg       falling         = 1'b0;

always @(posedge clk) begin
    mc_clk_raw_sync <= {mc_clk_raw_sync[0], MC_CLK};
    rising  <= 1'b0;
    falling <= 1'b0;

    if (mc_clk_lockout != 3'd0) begin
        mc_clk_lockout <= mc_clk_lockout - 3'd1;
    end else begin
        if (mc_clk_raw_sync[0] && !mc_clk_filtered) begin
            rising          <= 1'b1;
            mc_clk_filtered <= 1'b1;
            mc_clk_lockout  <= MC_CLK_LOCKOUT_TICKS;
        end else if (!mc_clk_raw_sync[0] && mc_clk_filtered) begin
            falling         <= 1'b1;
            mc_clk_filtered <= 1'b0;
            mc_clk_lockout  <= MC_CLK_LOCKOUT_TICKS;
        end
    end
end
```

---

## 8. Speculative Prefetch Engine & Hardware Coherency

The FPGA incorporates an autonomous speculative read prefetch engine accelerating Chip RAM reads from 1.4 MB/s to over 3.1 MB/s (`readw`) and 5.6 MB/s (`readm`).

### 16-bit & 32-bit Word Prefetching:
- **32-bit Longword Prefetch:** Whenever a 32-bit read completes normally in Chip RAM (`$000000..$1FFFFF`) or Expansion RAM (`$E00000..$FFFFFF`), the FSM speculatively launches a bus cycle for `address + 4` while the host Pi prepares its next request.
- **16-bit Word Prefetch (`ENABLE_16BIT_PREFETCH`):** Because Amiga 1200 Chip RAM has a 32-bit wide data bus, physical 32-bit reads return two 16-bit words simultaneously:
  - Upper word: `address[1] == 0` (even word)
  - Lower word: `address[1] == 1` (odd word)
  When code reads consecutive 16-bit words (such as rendering text or unrolled copy loops), word 1 is served directly from the FPGA prefetch buffer with **0 physical bus cycles**, halving bus traffic!

### Strict Hardware Write Coherency:
1. **Hardware-Locking:** The prefetch hit qualifier requires `new_req_rw == 1'b1` (Read). A write request can **never** hit the prefetch buffer.
2. **Atomic Invalidation:** Any write request arriving at the FSM instantly and unconditionally invalidates all prefetch tags in the very same clock cycle before dispatching the write:
   ```verilog
   prefetch_valid       <= 1'b0;
   prefetch_word0_avail <= 1'b0;
   prefetch_word1_avail <= 1'b0;
   prefetch_eligible    <= 1'b0;
   req_prefetch_hit     <= 2'b00;
   ```
3. **Safety Filtering:** Prefetching is strictly forbidden in CIA (`$BFE000..$BFFFFF`) and Custom Register space (`$DFF000..$DFFFFF`) to prevent side-effects on hardware strobes.

---

## 9. Fast DSACK Termination & 7.09 MHz CCK Phase Alignment

### Fast DSACK Termination:
The original PiStorm gateware held `/AS` asserted across a redundant `STATE_S4_NOP` state, creating an 82 ns post-DSACK dead time. Fast DSACK terminates the cycle on the falling edge of `MC_CLK` immediately after `/DSACK` assertion, reducing cycle length from 609 ns to 527 ns for reads, and 569 ns to 456 ns for writes.

### 7.09 MHz Colour Clock (CCK) Synchronization:
The Amiga 1200 custom chipset (Alice) operates on a 7.09 MHz bus slot raster. Because the CPU socket only receives a 14.18 MHz clock (`MC_CLK`), there are two possible phases per 7 MHz cycle:
- **Phase 0:** Alice fast write slot (5 wait states = 456 ns)
- **Phase 1:** Alice fast read slot (6 wait states = 527 ns)

Asserting `/AS` on the wrong phase forces Alice to add a full 70 ns wait state penalty. The CCK synchronizer aligns `/AS` assertions to Alice's slot raster and utilizes DMA-immune auto-calibration:
- Only clean fast cycles (`<= 5` wait states on write, or `<= 6` wait states on read) lock the phase.
- Cycles delayed by chipset DMA are ignored, preventing false phase inversion.

---

## 10. Micronik Busboard Hardware Compatibility

Micronik 6860 series Zorro busboards (v4.20 and v5.42) do not route the processor bus `_BG` (Bus Grant) signal from the motherboard edge connector through to the accelerator trapdoor connector.
- **Problem:** Because `_BG` is left floating, standard input buffers with pull-ups see `_BG` as high (inactive), causing the FPGA to believe bus mastership is never granted (`is_bm = 0`).
- **Solution:** The FPGA's `MC_BG_n` pin is configured with an internal **weak pull-down (~50 kΩ)** in `PS32-lite.peri.xml`. If the signal is left unconnected (floating on Micronik busboards), it is gently pulled low into active bus grant state. When installed in standard Amiga motherboards, Gayle drives the pin strongly high or low, easily overriding the 50 kΩ pull-down. This provides 100% plug-and-play compatibility without requiring wire jumpers.

