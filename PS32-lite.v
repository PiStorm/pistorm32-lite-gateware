/*
 * PiStorm32-lite Gateware
 *
 * Copyright 2022 Niklas Ekström
 * Copyright 2022-2026 Claude Schwarz
 */

module pistorm (
    // -------------------------------------------------------------------------
    // Physical Raspberry Pi GPIO Signals (Parallel Host Bus)
    // -------------------------------------------------------------------------
    output wire [2:0]   PI_IPL,             // Encoded Amiga interrupt level (GPIO[2..0])
    output wire         PI_TXN_IN_PROGRESS, // Transaction busy flag (GPIO3)
    output wire         PI_KBRESET,         // Filtered keyboard reset (GPIO4, for EMU68)
    input  wire         PI_SER_DAT,         // Serial UART RX from Pi (GPIO5, EMU68 debug)
    input  wire         PI_RD,              // Read strobe, active-low (GPIO6)
    input  wire         PI_WR,              // Write strobe, active-low (GPIO7)
    input  wire [15:0]  PI_D_IN,            // Multiplexed data bus input (GPIO[23..8])
    output wire [15:0]  PI_D_OUT,           // Multiplexed data bus output
    output wire [15:0]  PI_D_OE,            // Data bus output enables
    input  wire [2:0]   PI_A,               // Register address (GPIO[26..24])
    input  wire         PI_SER_CLK,         // Serial UART CLK from Pi (GPIO27, EMU68)

    // -------------------------------------------------------------------------
    // Multiplexed Address/Data (DA) Bus & PCB Bus Switches / Latches
    // -------------------------------------------------------------------------
    input  wire [31:0]  DA_IN,              // 32-bit bidirectional multiplexed bus input
    output wire [31:0]  DA_OUT,             // 32-bit multiplexed bus output
    output wire [31:0]  DA_OE,              // 32-bit output enables
    output wire         ADDR_LE,            // 74LVC573 Address Latch Enable (High = transparent)
    output wire         ADDR_OE_n,          // 74LVC573 Address Output Enable, active low
    output wire         DATA_OE_n,          // 74CBTD3384 Data Bus Switch Output Enable, active low
    output wire         CTRL_OE_n,          // Control Bus Driver Output Enable, active low

    // -------------------------------------------------------------------------
    // Motorola MC68EC020 Amiga 1200 Processor Bus Signals
    // -------------------------------------------------------------------------
    output wire [2:0]   MC_FC_OUT,          // Function Codes (FC2..FC0)
    output wire [2:0]   MC_FC_OE,
    output wire [1:0]   MC_SIZE_OUT,        // Transfer Size (SIZ1..SIZ0)
    output wire [1:0]   MC_SIZE_OE,
    output wire         MC_RW_OUT,          // Read / Write (1 = Read, 0 = Write)
    output wire         MC_RW_OE,
    input  wire         MC_AS_n_IN,         // Address Strobe input (from motherboard)
    output wire         MC_AS_n_OUT,        // Address Strobe output (to motherboard)
    output wire         MC_AS_n_OE,
    output wire         MC_DS_n_OUT,        // Data Strobe output
    output wire         MC_DS_n_OE,
    input  wire [1:0]   MC_DSACK_n,         // Data and Size Acknowledge inputs (DSACK1..DSACK0)
    input  wire [2:0]   MC_IPL_n,           // Interrupt Priority Level inputs (IPL2..IPL0)
    output wire         MC_BR_n_OUT,        // Bus Request output (to Gary/Gayle)
    output wire         MC_BR_n_OE,
    input  wire         MC_BG_n,            // Bus Grant input (from Gary/Gayle)
    input  wire         MC_RESET_n_IN,      // System Reset input
    output wire         MC_RESET_n_OUT,     // System Reset output
    output wire         MC_RESET_n_OE,
    input  wire         MC_HALT_n_IN,       // System Halt input
    output wire         MC_HALT_n_OUT,      // System Halt output
    output wire         MC_HALT_n_OE,
    input  wire         MC_BERR_n,          // Bus Error input
    input  wire         MC_CLK,             // Amiga 14.18 MHz motherboard clock (E1)

    // -------------------------------------------------------------------------
    // Miscellaneous Amiga 1200 Signals
    // -------------------------------------------------------------------------
    output wire         INT2_n_OUT,         // Level 2 Interrupt output (Paula)
    output wire         INT2_n_OE,
    output wire         INT6_n_OUT,         // Level 6 Interrupt output (CIA-B)
    output wire         INT6_n_OE,
    input  wire         KBRESET,            // Keyboard Reset line from keyboard controller

    // -------------------------------------------------------------------------
    // Expansion & Clock Signals
    // -------------------------------------------------------------------------
    input  wire [7:0]   SPARE_IN,           // Auxiliary / expansion header inputs
    output wire [7:0]   SPARE_OUT,          // Auxiliary / expansion header outputs
    output wire [7:0]   SPARE_OE,
    input  wire         AMIPLL_CLKOUT0      // Main FPGA system clock (~182 MHz PLL)
);

// =============================================================================
// Internal Clock & Interconnect Wiring
// =============================================================================

// Main internal FPGA clock driven by high-speed PLL (~182 MHz)
wire clk = AMIPLL_CLKOUT0;

// -----------------------------------------------------------------------------
// Control & Status Interconnect: pi_interface <-> m68k_interface
// -----------------------------------------------------------------------------
wire        request_bm;                     // Host requests bus mastership
wire        drive_reset;                    // Host commands system reset
wire        drive_halt;                     // Host commands CPU halt
wire        pi_drive_int2;                  // Host asserts INT2 from Pi
wire        pi_drive_int6;                  // Host asserts INT6 from Pi
wire        z2_int2;                        // Virtual Zorro device asserts INT2
wire        z2_int6;                        // Virtual Zorro device asserts INT6
wire        drive_int2 = pi_drive_int2 | z2_int2; // Combined Level 2 interrupt drive
wire        drive_int6 = pi_drive_int6 | z2_int6; // Combined Level 6 interrupt drive
wire        increment_execute_slot_pointer; // Slot ping-pong enable
wire        enable_prefetch;                // Speculative read prefetch enable

wire        is_bm;                          // Current bus master status
wire        reset_sync;                     // Synchronized Amiga /RESET
wire        halt_sync;                      // Synchronized Amiga /HALT
wire [2:0]  ipl;                            // Synchronized Amiga /IPL
wire        mc_reset_n_sync;                // Filtered 68020 /RESET
wire [23:0] current_bus_address;            // Live 68020 bus address for status read

// -----------------------------------------------------------------------------
// Request Slot Queue Interconnect: pi_interface -> m68k_interface
// -----------------------------------------------------------------------------
wire [1:0]  req_active;                     // Active request flags for Slot 0 / 1
wire [1:0]  req_internal_intercept;         // Precomputed virtual Zorro access flags
wire        new_req_valid;                  // 1-cycle strobe when Pi writes ADDR_HI
wire        new_req_slot;                   // Target slot ID (0 or 1)
wire [23:0] new_req_addr;                   // Full 24-bit physical address
wire        new_req_rw;                     // 1 = Read, 0 = Write
wire [1:0]  new_req_size;                   // Transfer size: 0=Byte, 1=Word, 3=Long
wire [23:0] req_address_0;                  // Latched address Slot 0
wire [23:0] req_address_1;                  // Latched address Slot 1
wire [31:0] req_data_write_0;               // Write data Slot 0
wire [31:0] req_data_write_1;               // Write data Slot 1
wire [1:0]  req_size_0;                     // Size Slot 0
wire [1:0]  req_size_1;                     // Size Slot 1
wire        req_rw_0;                       // R/W Slot 0
wire        req_rw_1;                       // R/W Slot 1
wire [2:0]  req_fc_0;                       // Function Code Slot 0
wire [2:0]  req_fc_1;                       // Function Code Slot 1

wire        set_execute_slot_valid;         // Slot pointer manual override valid
wire        set_execute_slot_val;           // Slot pointer manual override value

// -----------------------------------------------------------------------------
// Slot Completion Interconnect: m68k_interface -> pi_interface
// -----------------------------------------------------------------------------
wire        slot_complete_valid;            // Combinatorial termination pulse
wire        slot_complete_id;               // Slot ID being completed
wire [31:0] slot_complete_data;             // Read data to buffer in slot
wire        slot_complete_normally;         // 1 = Normal (DSACK), 0 = Bus Error (BERR)

// -----------------------------------------------------------------------------
// Virtual Zorro Configuration & Access Interconnect
// -----------------------------------------------------------------------------
wire        z2_configured;                  // Card has received base address
wire        z2_shutup;                      // Card has received shut-up command
wire [7:0]  z2_base_addr_hi;                // Base address bits [23:16]

wire        z2_access_valid;                // Access request valid from m68k FSM
wire        z2_access_ready;                // Access completion ready from zorro_device
wire        z2_access_wr;                   // 1 = Write, 0 = Read
wire [1:0]  z2_access_size;                 // Size: 0=Byte, 1=Word, 3=Long
wire [23:0] z2_access_addr;                 // Physical address
wire [31:0] z2_access_wr_data;              // Data to write to scratchpad / config
wire [31:0] z2_rd_data;                     // Data read from AutoConfig ROM or IO regs

// -----------------------------------------------------------------------------
// Hardware Diagnostic & Telemetry Interface
// -----------------------------------------------------------------------------
wire [31:0] prefetch_launch_count;
wire [31:0] prefetch_hit_count;
wire [31:0] diag_status;
wire [31:0] diag_bus_capture;
wire [31:0] diag_cycle_timing;
wire [31:0] diag_clock_phase;
wire        prefetch_ctrl_en;
wire        fast_dsack_en;
wire        counter_clear;

// 1. Raspberry Pi Interface Submodule
pi_interface u_pi (
    .clk                            (clk),

    // Raspberry Pi GPIOs
    .PI_A                           (PI_A),
    .PI_RD                          (PI_RD),
    .PI_WR                          (PI_WR),
    .PI_D_IN                        (PI_D_IN),
    .PI_D_OUT                       (PI_D_OUT),
    .PI_D_OE                        (PI_D_OE),
    .PI_IPL                         (PI_IPL),
    .PI_TXN_IN_PROGRESS             (PI_TXN_IN_PROGRESS),
    .PI_KBRESET                     (PI_KBRESET),

    // Status from m68k
    .ipl                            (ipl),
    .halt_sync                      (halt_sync),
    .reset_sync                     (reset_sync),
    .is_bm                          (is_bm),
    .KBRESET                        (KBRESET),
    .mc_reset_n_sync                (mc_reset_n_sync),
    .current_bus_address            (current_bus_address),

    // Control to m68k
    .request_bm                     (request_bm),
    .drive_reset                    (drive_reset),
    .drive_halt                     (drive_halt),
    .drive_int2                     (pi_drive_int2),
    .drive_int6                     (pi_drive_int6),
    .increment_execute_slot_pointer (increment_execute_slot_pointer),
    .enable_prefetch                (enable_prefetch),

    // Virtual Zorro status (for intercept precomputation)
    .z2_configured                  (z2_configured),
    .z2_shutup                      (z2_shutup),
    .z2_base_addr_hi                (z2_base_addr_hi),

    // Slot requests to m68k
    .req_active                     (req_active),
    .req_internal_intercept         (req_internal_intercept),
    .new_req_valid                  (new_req_valid),
    .new_req_slot                   (new_req_slot),
    .new_req_addr                   (new_req_addr),
    .new_req_rw                     (new_req_rw),
    .new_req_size                   (new_req_size),
    .req_address_0                  (req_address_0),
    .req_address_1                  (req_address_1),
    .req_data_write_0               (req_data_write_0),
    .req_data_write_1               (req_data_write_1),
    .req_size_0                     (req_size_0),
    .req_size_1                     (req_size_1),
    .req_rw_0                       (req_rw_0),
    .req_rw_1                       (req_rw_1),
    .req_fc_0                       (req_fc_0),
    .req_fc_1                       (req_fc_1),

    // Slot pointer override
    .set_execute_slot_valid         (set_execute_slot_valid),
    .set_execute_slot_val           (set_execute_slot_val),

    // Slot completion from m68k
    .slot_complete_valid            (slot_complete_valid),
    .slot_complete_id               (slot_complete_id),
    .slot_complete_data             (slot_complete_data),
    .slot_complete_normally         (slot_complete_normally)
);

// 2. Virtual Zorro-II AutoConfig Device Submodule (Wishbone B4 Architecture)
zorro_device #(
    .Z2_MANUF_ID (16'd28020), // 0x6D74
    .Z2_PROD_ID  (8'h32),     // PiStorm32
    .Z2_SERIAL   (32'd1)
) u_zorro (
    .clk                  (clk),
    .reset                (reset_sync || drive_reset),

    // Configuration status
    .z2_configured        (z2_configured),
    .z2_shutup            (z2_shutup),
    .z2_base_addr_hi      (z2_base_addr_hi),

    // Amiga Interrupt Requests
    .z2_int2              (z2_int2),
    .z2_int6              (z2_int6),

    // Access handshake from/to m68k FSM
    .access_valid         (z2_access_valid),
    .access_ready         (z2_access_ready),
    .access_wr            (z2_access_wr),
    .access_size          (z2_access_size),
    .access_addr          (z2_access_addr),
    .access_wr_data       (z2_access_wr_data),
    .access_rd_data       (z2_rd_data),

    // Auxiliary / Expansion Port & Debug Wiring (SPARE[7:0] + EMU68 UART)
    .SPARE_IN             (SPARE_IN),
    .SPARE_OUT            (SPARE_OUT),
    .SPARE_OE             (SPARE_OE),
    .PI_SER_DAT           (PI_SER_DAT),
    .PI_SER_CLK           (PI_SER_CLK),

    // Hardware Diagnostic & Telemetry Interface
    .prefetch_launch_count(prefetch_launch_count),
    .prefetch_hit_count   (prefetch_hit_count),
    .diag_status          (diag_status),
    .diag_bus_capture     (diag_bus_capture),
    .diag_cycle_timing    (diag_cycle_timing),
    .diag_clock_phase     (diag_clock_phase),
    .prefetch_ctrl_en     (prefetch_ctrl_en),
    .fast_dsack_en        (fast_dsack_en),
    .counter_clear        (counter_clear)
);

// 3. MC68020 Bus Master Interface Submodule
m68k_interface u_m68k (
    .clk                            (clk),

    // 68020 physical signals
    .MC_FC_OUT                      (MC_FC_OUT),
    .MC_FC_OE                       (MC_FC_OE),
    .MC_SIZE_OUT                    (MC_SIZE_OUT),
    .MC_SIZE_OE                     (MC_SIZE_OE),
    .MC_RW_OUT                      (MC_RW_OUT),
    .MC_RW_OE                       (MC_RW_OE),
    .MC_AS_n_IN                     (MC_AS_n_IN),
    .MC_AS_n_OUT                    (MC_AS_n_OUT),
    .MC_AS_n_OE                     (MC_AS_n_OE),
    .MC_DS_n_OUT                    (MC_DS_n_OUT),
    .MC_DS_n_OE                     (MC_DS_n_OE),
    .MC_DSACK_n                     (MC_DSACK_n),
    .MC_IPL_n                       (MC_IPL_n),
    .MC_BR_n_OUT                    (MC_BR_n_OUT),
    .MC_BR_n_OE                     (MC_BR_n_OE),
    .MC_BG_n                        (MC_BG_n),
    .MC_RESET_n_IN                  (MC_RESET_n_IN),
    .MC_RESET_n_OUT                 (MC_RESET_n_OUT),
    .MC_RESET_n_OE                  (MC_RESET_n_OE),
    .MC_HALT_n_IN                   (MC_HALT_n_IN),
    .MC_HALT_n_OUT                  (MC_HALT_n_OUT),
    .MC_HALT_n_OE                   (MC_HALT_n_OE),
    .MC_BERR_n                      (MC_BERR_n),
    .MC_CLK                         (MC_CLK),

    // Amiga interrupts
    .INT2_n_OUT                     (INT2_n_OUT),
    .INT2_n_OE                      (INT2_n_OE),
    .INT6_n_OUT                     (INT6_n_OUT),
    .INT6_n_OE                      (INT6_n_OE),

    // Multiplexed DA bus
    .DA_IN                          (DA_IN),
    .DA_OUT                         (DA_OUT),
    .DA_OE                          (DA_OE),
    .ADDR_LE                        (ADDR_LE),
    .ADDR_OE_n                      (ADDR_OE_n),
    .DATA_OE_n                      (DATA_OE_n),
    .CTRL_OE_n                      (CTRL_OE_n),

    // Control from pi_interface
    .request_bm                     (request_bm),
    .drive_reset                    (drive_reset),
    .drive_halt                     (drive_halt),
    .drive_int2                     (drive_int2),
    .drive_int6                     (drive_int6),
    .increment_execute_slot_pointer (increment_execute_slot_pointer),
    .enable_prefetch                (enable_prefetch),

    // Status to pi_interface
    .is_bm                          (is_bm),
    .reset_sync                     (reset_sync),
    .halt_sync                      (halt_sync),
    .ipl                            (ipl),
    .mc_reset_n_sync                (mc_reset_n_sync),
    .current_bus_address            (current_bus_address),

    // Slot requests from pi_interface
    .req_active                     (req_active),
    .req_internal_intercept         (req_internal_intercept),
    .req_address_0                  (req_address_0),
    .req_address_1                  (req_address_1),
    .req_data_write_0               (req_data_write_0),
    .req_data_write_1               (req_data_write_1),
    .req_size_0                     (req_size_0),
    .req_size_1                     (req_size_1),
    .req_rw_0                       (req_rw_0),
    .req_rw_1                       (req_rw_1),
    .req_fc_0                       (req_fc_0),
    .req_fc_1                       (req_fc_1),
    .new_req_valid                  (new_req_valid),
    .new_req_slot                   (new_req_slot),
    .new_req_addr                   (new_req_addr),
    .new_req_rw                     (new_req_rw),
    .new_req_size                   (new_req_size),

    // Slot pointer override
    .set_execute_slot_valid         (set_execute_slot_valid),
    .set_execute_slot_val           (set_execute_slot_val),

    // Slot completion to pi_interface
    .slot_complete_valid            (slot_complete_valid),
    .slot_complete_id               (slot_complete_id),
    .slot_complete_data             (slot_complete_data),
    .slot_complete_normally         (slot_complete_normally),

    // Zorro device interface (Pipelined Handshake)
    .z2_access_valid                (z2_access_valid),
    .z2_access_ready                (z2_access_ready),
    .z2_access_wr                   (z2_access_wr),
    .z2_access_size                 (z2_access_size),
    .z2_access_addr                 (z2_access_addr),
    .z2_access_wr_data              (z2_access_wr_data),
    .z2_rd_data                     (z2_rd_data),

    // Hardware Diagnostic & Telemetry Interface
    .prefetch_launch_count          (prefetch_launch_count),
    .prefetch_hit_count             (prefetch_hit_count),
    .diag_status                    (diag_status),
    .diag_bus_capture               (diag_bus_capture),
    .diag_cycle_timing              (diag_cycle_timing),
    .diag_clock_phase               (diag_clock_phase),
    .prefetch_ctrl_en               (prefetch_ctrl_en),
    .fast_dsack_en                  (fast_dsack_en),
    .counter_clear                  (counter_clear)
);

endmodule
