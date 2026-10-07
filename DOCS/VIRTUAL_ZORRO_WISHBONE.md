# Virtual Zorro-II & Wishbone Interconnect

The gateware provides a virtual Zorro-II AutoConfig expansion device and an internal 32-bit Wishbone B4 crossbar running at 182 MHz with 0 wait states.

---

## 1. AutoConfig Specification

AmigaOS enumerates expansion boards at boot via `expansion.library` probing `$00E80000`. The gateware synthesizes standard AutoConfig ROM nibbles:

- **Board Type:** Zorro-II Memory / I/O expansion (`$E` in nibble 0)
- **Size:** 64 KB (`$01` in size field)
- **Manufacturer ID:** 28020 (`$6D74`, registered PiStorm ID)
- **Product ID:** `$32` (PiStorm32)
- **Serial Number:** `$00000001`
- **Base Address Register:** `$00E80048` / `$00E8004A`

Once configured by AmigaOS, the board relocates to its assigned 64 KB base address (typically `$00E90000`). Accesses to this 64 KB space are handled purely inside the FPGA and do not generate any motherboard bus cycles.

---

## 2. Memory Map (64 KB Window)

Base address offsets (relative to configured base, e.g. `$00E90000`):

| Offset | Size | Slave | Function |
| :--- | :---: | :--- | :--- |
| `+$0000`–`+$0FFF` | 4 KB | Slave 0: Scratchpad SRAM | Dual-ported RAM buffer (shared between Pi and Amiga) |
| `+$1000`–`+$101F` | 32 B | Slave 1: SPI / Mailbox FIFO | SPI master and coprocessor message passing |
| `+$2000`–`+$20FF` | 256 B | Slave 2: GPIO Matrix & IO MUX | Pin configuration and atomic bitwise registers |
| `+$3000`–`+$301F` | 32 B | Slave 3: Interrupt Controller | Amiga Level 2 (INT2) and Level 6 (INT6) control |

---

## 3. Peripheral Registers

### Slave 0: Scratchpad SRAM (`+$0000`–`+$0FFF`)
4 KB dual-port SRAM accessible by both the Amiga CPU and Raspberry Pi with 0 wait states. Supports 8, 16, and 32-bit reads and writes.

### Slave 1: SPI / Coprocessor Mailbox (`+$1000`–`+$101F`)

| Offset | Name | R/W | Description |
| :---: | :--- | :---: | :--- |
| `+$1000` | `COPROC_CTRL` | R/W | Bit 0: Enable, Bit 1: FIFO Reset, Bit 2: Loopback |
| `+$1004` | `COPROC_STATUS` | R | Bit 0: TX Empty, Bit 1: TX Full, Bit 2: RX Empty, Bit 3: RX Full |
| `+$1008` | `COPROC_DATA_TX` | W | Write 32-bit word to TX FIFO |
| `+$100C` | `COPROC_DATA_RX` | R | Read 32-bit word from RX FIFO |

### Slave 2: GPIO Matrix & IO MUX (`+$2000`–`+$20FF`)

Supports direct pin control and atomic bit operations:

| Offset | Name | R/W | Description |
| :---: | :--- | :---: | :--- |
| `+$2000` | `GPIO_OUT_REG` | R/W | Output register [31:0] |
| `+$2004` | `GPIO_OUT_W1TS` | W | **Write-1-to-Set:** 1 sets bit to high; 0 has no effect |
| `+$2008` | `GPIO_OUT_W1TC` | W | **Write-1-to-Clear:** 1 clears bit to low; 0 has no effect |
| `+$200C` | `GPIO_OUT_W1TT` | W | **Write-1-to-Toggle:** 1 inverts bit state; 0 has no effect |
| `+$2010` | `GPIO_IN_REG` | R | Input pin state [31:0] |
| `+$2014` | `GPIO_ENABLE_REG` | R/W | Direction (1 = Output, 0 = Input) |
| `+$2018` | `GPIO_ENABLE_W1TS` | W | Set pins to Output atomically |
| `+$201C` | `GPIO_ENABLE_W1TC` | W | Set pins to Input atomically |
| `+$2040`+ | `GPIO_FUNC_IN_SEL_n` | R/W | Route pin `n` to internal peripheral input |
| `+$2080`+ | `GPIO_FUNC_OUT_SEL_n`| R/W | Route internal peripheral to pin `n` (Bit 8: Invert) |

Atomic W1TS/W1TC/W1TT writes avoid read-modify-write race conditions in multitasking environments without needing to disable interrupts.

### Slave 3: Interrupt Controller (`+$3000`–`+$301F`)

| Offset | Name | R/W | Description |
| :---: | :--- | :---: | :--- |
| `+$3000` | `IRQ_STATUS` | R | Bit 0: INT2 pending, Bit 1: INT6 pending |
| `+$3004` | `IRQ_ASSERT` | W | Bit 0: assert INT2, Bit 1: assert INT6 |
| `+$3008` | `IRQ_CLEAR` | W | Bit 0: clear INT2, Bit 1: clear INT6 |
| `+$300C` | `IRQ_MASK` | R/W | Bit 0: enable INT2, Bit 1: enable INT6 |

- **INT2:** Triggers Amiga Level 2 interrupt (Paula, ports/audio). Handled in AmigaOS via `AddIntServer(INTB_PORTS, ...)`.
- **INT6:** Triggers Amiga Level 6 interrupt (CIA-B).

---

## 4. AmigaOS Driver Example

### Finding the Card
```c
#include <proto/exec.h>
#include <proto/expansion.h>
#include <libraries/configvars.h>

#define PISTORM_MANUF_ID 28020
#define PISTORM_PROD_ID  0x32

struct ConfigDev* cd;
volatile ULONG* zorro_base = NULL;

if ((ExpansionBase = (struct ExpansionBase*)OpenLibrary("expansion.library", 36))) {
    cd = FindConfigDev(NULL, PISTORM_MANUF_ID, PISTORM_PROD_ID);
    if (cd) {
        zorro_base = (volatile ULONG*)cd->cd_BoardAddr;
    }
    CloseLibrary((struct Library*)ExpansionBase);
}
```

### Atomic Pin Toggling
```c
#define GPIO_OUT_W1TT     (*(volatile ULONG*)((UBYTE*)zorro_base + 0x200C))
#define GPIO_ENABLE_W1TS  (*(volatile ULONG*)((UBYTE*)zorro_base + 0x2018))

// Set pin 4 as output
GPIO_ENABLE_W1TS = (1 << 4);

// Toggle pin 4 without read-modify-write
GPIO_OUT_W1TT = (1 << 4);
```

### Installing an INT2 Handler
```c
#include <hardware/intbits.h>

struct Interrupt pistorm_int2;

BOOL PistormIntHandler(void) {
    volatile ULONG* irq_base = (volatile ULONG*)((UBYTE*)zorro_base + 0x3000);
    ULONG status = irq_base[0]; // Read IRQ_STATUS

    if (status & 0x01) {
        // Handle event from Pi host
        ProcessEvent();

        // Clear INT2
        irq_base[2] = 0x01; // Write IRQ_CLEAR
        return TRUE;
    }
    return FALSE;
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
