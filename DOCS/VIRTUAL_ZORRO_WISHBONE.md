# Virtual Zorro-II AutoConfig & Wishbone B4 Architecture

## 1. Executive Overview

The **PiStorm32-lite** gateware features a hardware-emulated **Virtual Zorro-II AutoConfig PIC (Plug-in Card)** coupled to an internal **Wishbone B4 Crossbar Interconnect**.

This subsystem enables the Amiga host CPU and the Raspberry Pi host to exchange data, trigger hardware interrupts, and control peripheral I/O at the FPGA's full $182\text{ MHz}$ internal clock speed without consuming a single cycle on the vintage Amiga motherboard bus.

```
       +-------------------------------------------------------------+
       |                  AmigaOS expansion.library                  |
       +------------------------------+------------------------------+
                                      |
                                      | AutoConfig Probing at $00E80000
                                      v
       +-------------------------------------------------------------+
       |               Virtual Zorro-II AutoConfig PIC               |
       |  - 64 KB Memory Space Assigned (Default: $00E90000)         |
       |  - Zero Motherboard Bus Wait States                         |
       |  - Complete Isolation from Alice / Custom Chipset           |
       +------------------------------+------------------------------+
                                      |
                                      | Wishbone B4 Bus (182 MHz, 0-WS)
                                      v
       +-------------------------------------------------------------+
       |                Wishbone Interconnect Crossbar               |
       +-------+-----------------+-----------------+-----------------+
               |                 |                 |                 |
               v                 v                 v                 v
        [ Slave 0 ]       [ Slave 1 ]       [ Slave 2 ]       [ Slave 3 ]
        Scratchpad SRAM   SPI / Mailbox     ESP32 GPIO Matrix Interrupt Ctrl
        ($0000-$0FFF)     ($1000-$10FF)     ($2000-$20FF)     ($3000-$30FF)
        Fast Buffers      FIFO / Coproc     Atomic IO MUX     Amiga INT2 / 6
```

---

## 2. AutoConfig Specification & Address Allocation

### 2.1 AutoConfig ROM Headers ($00E80000)
When AmigaOS boots, the `expansion.library` queries memory space `$00E80000` to enumerate expansion boards. The PiStorm32-lite gateware synthesizes standard AutoConfig nibbles complying with the Commodore Amiga expansion specification:

- **Board Type:** Zorro-II Memory / I/O Expansion (`$E` in nibble 0).
- **Size Code:** 64 KB (`$01` in size field).
- **Manufacturer ID:** `$5053` ("PS" - PiStorm).
- **Product ID:** `$01` (PiStorm32-lite Virtual Coprocessor).
- **Serial Number:** `$00000001`.
- **Base Address Register:** `$00E80048` / `$00E8004A`.

Once configured by `expansion.library`, the card maps its 64 KB address window to **`$00E90000`–`$00E9FFFF`** (or to any 64 KB aligned base address assigned by the operating system).

---

## 3. Wishbone B4 Memory Map

All internal peripherals reside within the 64 KB configured window:

| Address Offset | Size | Slave Module | Description |
| :--- | :---: | :--- | :--- |
| `+$0000`–`+$0FFF` | 4 KB | **Slave 0: Scratchpad SRAM** | High-speed dual-port shared buffer for DMA / Mailbox packets |
| `+$1000`–`+$101F` | 32 B | **Slave 1: SPI / Coproc FIFO** | High-speed SPI master & coprocessor mailbox registers |
| `+$2000`–`+$20FF` | 256 B | **Slave 2: GPIO Matrix & IO MUX** | ESP32-style pin mapping, atomic SET/CLR/TOGGLE registers |
| `+$3000`–`+$301F` | 32 B | **Slave 3: Interrupt Controller** | Amiga Level 2 (INT2) & Level 6 (INT6) interrupt generation |

### 3.1 Bus Performance & Isolation
- **Clock Frequency:** Full $182.0\text{ MHz}$ (`sys_clk`).
- **Wait States:** **0 Wait States** (single-cycle ACK for all reads and writes).
- **Throughput:** **$13.5\text{ MB/s}$** transfer rate ($3.54\text{ MOps/s}$) over the Pi host parallel interface.
- **Motherboard Bus Isolation:** **100% Isolated.** Zero cycles on `E7M`/Alice; transfers occur purely within the FPGA silicon without loading the Amiga bus.

---

## 4. Peripheral Registers Reference

### 4.1 Slave 0: Scratchpad SRAM (`+$0000`–`+$0FFF`)
A 4 KB block of high-speed dual-ported SRAM accessible by both the Amiga 68020/68030/68040 and the Raspberry Pi host.
- Supports 8-bit, 16-bit, and 32-bit atomic read/write accesses.
- Ideal for zero-copy message ring buffers, disk sector caches, and network packet descriptors.

### 4.2 Slave 1: SPI / Coprocessor Mailbox (`+$1000`–`+$101F`)

| Offset | Register Name | R/W | Description |
| :---: | :--- | :---: | :--- |
| `+$1000` | `COPROC_CTRL` | R/W | Bit 0: Enable, Bit 1: Reset FIFO, Bit 2: Loopback |
| `+$1004` | `COPROC_STATUS` | R | Bit 0: TX Empty, Bit 1: TX Full, Bit 2: RX Empty, Bit 3: RX Full |
| `+$1008` | `COPROC_DATA_TX` | W | Push 32-bit word into transmit FIFO |
| `+$100C` | `COPROC_DATA_RX` | R | Pop 32-bit word from receive FIFO |

### 4.3 Slave 2: ESP32-Style GPIO Matrix & IO MUX (`+$2000`–`+$20FF`)
Provides flexible software-defined pin routing and atomic bit manipulation matching the ESP32 GPIO architecture:

| Offset | Register Name | R/W | Bit Description |
| :---: | :--- | :---: | :--- |
| `+$2000` | `GPIO_OUT_REG` | R/W | Output state of GPIO pins [31:0] |
| `+$2004` | `GPIO_OUT_W1TS` | W | **Write-1-to-Set:** Bits written as 1 set output high; 0 has no effect |
| `+$2008` | `GPIO_OUT_W1TC` | W | **Write-1-to-Clear:** Bits written as 1 clear output low; 0 has no effect |
| `+$200C` | `GPIO_OUT_W1TT` | W | **Write-1-to-Toggle:** Bits written as 1 invert output state |
| `+$2010` | `GPIO_IN_REG` | R | Real-time input state of physical pins [31:0] |
| `+$2014` | `GPIO_ENABLE_REG` | R/W | Direction control (1 = Output, 0 = Input) |
| `+$2018` | `GPIO_ENABLE_W1TS` | W | **Write-1-to-Set Direction:** Atomically sets pins to Output |
| `+$201C` | `GPIO_ENABLE_W1TC` | W | **Write-1-to-Clear Direction:** Atomically sets pins to Input |
| `+$2040`+ | `GPIO_FUNC_IN_SEL_n` | R/W | Maps physical pin `n` to internal peripheral signal |
| `+$2080`+ | `GPIO_FUNC_OUT_SEL_n`| R/W | Maps internal peripheral signal to physical pin `n` (Bit 8: Invert) |

#### Atomic Bit Operations Advantage:
Eliminates read-modify-write race conditions in multi-threaded OS environments (e.g., AmigaOS Exec multitasking or Pi Linux background services). Software never needs to disable interrupts (`Disable()`/`Enable()`) just to toggle an I/O line.

### 4.4 Slave 3: Amiga Interrupt Controller (`+$3000`–`+$301F`)
Allows hardware and software on the Pi host or FPGA peripherals to signal the Amiga CPU directly.

| Offset | Register Name | R/W | Description |
| :---: | :--- | :---: | :--- |
| `+$3000` | `IRQ_STATUS` | R | Bit 0: INT2 Pending, Bit 1: INT6 Pending |
| `+$3004` | `IRQ_ASSERT` | W | Write 1 to Bit 0 asserts **INT2**; Write 1 to Bit 1 asserts **INT6** |
| `+$3008` | `IRQ_CLEAR` | W | Write 1 to Bit 0/1 acknowledges and clears the respective IRQ |
| `+$300C` | `IRQ_MASK` | R/W | Bit 0/1: Enable/mask interrupt propagation to Amiga `_IPL[2:0]` |

- **INT2 (Level 2 Interrupt):** Connected to Amiga `_IPL` lines prioritizing fast I/O, network driver packet arrival, and coprocessor mailbox requests. Handled via standard AmigaOS `AddIntServer(INTB_PORTS, ...)`.
- **INT6 (Level 6 Interrupt):** Highest non-maskable peripheral level; used for urgent real-time timing and fault signaling.

---

## 5. AmigaOS Driver Implementation Guide

### 5.1 AutoConfig Card Enumeration
In AmigaOS C (using `exec/types.h` and `libraries/configvars.h`):

```c
#include <proto/exec.h>
#include <proto/expansion.h>
#include <libraries/configvars.h>

#define PISTORM_MANUFACTURER_ID 0x5053
#define PISTORM_PRODUCT_ID      0x01

struct ConfigDev* cd = NULL;
ULONG* zorro_base = NULL;

if ((ExpansionBase = (struct ExpansionBase*)OpenLibrary("expansion.library", 36))) {
    cd = FindConfigDev(NULL, PISTORM_MANUFACTURER_ID, PISTORM_PRODUCT_ID);
    if (cd) {
        zorro_base = (ULONG*)cd->cd_BoardAddr;
        printf("PiStorm32-lite Virtual Zorro card found at: 0x%08lx\n", (ULONG)zorro_base);
    }
    CloseLibrary((struct Library*)ExpansionBase);
}
```

### 5.2 Atomic Pin Toggling Example
```c
volatile ULONG* gpio_base = (volatile ULONG*)((UBYTE*)zorro_base + 0x2000);
#define GPIO_OUT_W1TT (*(volatile ULONG*)((UBYTE*)gpio_base + 0x000C))
#define GPIO_ENABLE_W1TS (*(volatile ULONG*)((UBYTE*)gpio_base + 0x0018))

// Configure Pin 4 as output
GPIO_ENABLE_W1TS = (1 << 4);

// Atomically toggle Pin 4 (zero wait-states, zero bus cycles)
GPIO_OUT_W1TT = (1 << 4);
```

### 5.3 Installing an INT2 Interrupt Server
```c
#include <hardware/intbits.h>

struct Interrupt pistorm_int2;

BOOL PistormIntHandler(void) {
    volatile ULONG* irq_base = (volatile ULONG*)((UBYTE*)zorro_base + 0x3000);
    ULONG status = irq_base[0]; // Read IRQ_STATUS

    if (status & 0x01) {
        // Handle packet / event from Pi host
        ProcessPiMailbox();

        // Acknowledge and clear INT2
        irq_base[2] = 0x01; // Write IRQ_CLEAR
        return TRUE;
    }
    return FALSE; // Not our interrupt
}

void InstallDriver(void) {
    pistorm_int2.is_Node.ln_Type = NT_INTERRUPT;
    pistorm_int2.is_Node.ln_Pri  = 10;
    pistorm_int2.is_Node.ln_Name = "PiStorm32_INT2";
    pistorm_int2.is_Data         = NULL;
    pistorm_int2.is_Code         = (void (*)())PistormIntHandler;

    AddIntServer(INTB_PORTS, &pistorm_int2);
}
```
