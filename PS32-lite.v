/*
 * Copyright 2022 Niklas Ekström
 * Copyright 2022 Claude Schwarz
 *
 * Top-Level PiStorm32-lite Module
 */

module pistorm (
    // Raspberry Pi signals
    output [2:0]    PI_IPL,             // GPIO0..2
    output          PI_TXN_IN_PROGRESS, // GPIO3
    output          PI_KBRESET,         // GPIO4 EMU68
    input           PI_SER_DAT,         // GPIO5 EMU68
    input           PI_RD,              // GPIO6
    input           PI_WR,              // GPIO7
    input [15:0]    PI_D_IN,            // GPIO[23..8]
    output [15:0]   PI_D_OUT,
    output [15:0]   PI_D_OE,
    input [2:0]     PI_A,               // GPIO[26..24]
    input           PI_SER_CLK,         // GPIO27 EMU68

    // Shared data and address bus multiplexing
    input [31:0]    DA_IN,
    output [31:0]   DA_OUT,
    output [31:0]   DA_OE,
    output          ADDR_LE,
    output          ADDR_OE_n,
    output          DATA_OE_n,
    output          CTRL_OE_n,

    // MC68EC020 signals
    output [2:0]    MC_FC_OUT,
    output [2:0]    MC_FC_OE,
    output [1:0]    MC_SIZE_OUT,
    output [1:0]    MC_SIZE_OE,
    output          MC_RW_OUT,
    output          MC_RW_OE,
    input           MC_AS_n_IN,
    output          MC_AS_n_OUT,
    output          MC_AS_n_OE,
    output          MC_DS_n_OUT,
    output          MC_DS_n_OE,
    input [1:0]     MC_DSACK_n,
    input [2:0]     MC_IPL_n,
    output          MC_BR_n_OUT,
    output          MC_BR_n_OE,
    input           MC_BG_n,
    input           MC_RESET_n_IN,
    output          MC_RESET_n_OUT,
    output          MC_RESET_n_OE,
    input           MC_HALT_n_IN,
    output          MC_HALT_n_OUT,
    output          MC_HALT_n_OE,
    input           MC_BERR_n,
    input           MC_CLK,

    // Miscellaneous Amiga 1200 signals
    output          INT2_n_OUT,
    output          INT2_n_OE,
    output          INT6_n_OUT,
    output          INT6_n_OE,
    input           KBRESET,

    // EXT Port
    input [7:0]     SPARE_IN,
    output [7:0]    SPARE_OUT,
    output [7:0]    SPARE_OE,

    // PLL Clock
    input           AMIPLL_CLKOUT0
);

// Spare port assignments & EMU68 UART pass-through
assign SPARE_OUT[7:2] = 6'b111111;
assign SPARE_OE       = 8'b11111111;
assign SPARE_OUT[0]   = PI_SER_DAT;
assign SPARE_OUT[1]   = PI_SER_CLK;

// Main clock from PLL
wire clk = AMIPLL_CLKOUT0;

// Interconnect wires: pi_interface <-> m68k_interface
wire        request_bm;
wire        drive_reset;
wire        drive_halt;
wire        drive_int2;
wire        drive_int6;
wire        increment_execute_slot_pointer;
wire        enable_prefetch;

wire        is_bm;
wire        reset_sync;
wire        halt_sync;
wire [2:0]  ipl;
wire        mc_reset_n_sync;
wire [23:0] current_bus_address;

wire [1:0]  req_active;
wire [1:0]  req_internal_intercept;
wire        new_req_valid;
wire        new_req_slot;
wire [23:0] new_req_addr;
wire        new_req_rw;
wire [1:0]  new_req_size;
wire [23:0] req_address_0;
wire [23:0] req_address_1;
wire [31:0] req_data_write_0;
wire [31:0] req_data_write_1;
wire [1:0]  req_size_0;
wire [1:0]  req_size_1;
wire        req_rw_0;
wire        req_rw_1;
wire [2:0]  req_fc_0;
wire [2:0]  req_fc_1;

wire        set_execute_slot_valid;
wire        set_execute_slot_val;

wire        slot_complete_valid;
wire        slot_complete_id;
wire [31:0] slot_complete_data;
wire        slot_complete_normally;

// Interconnect wires: m68k_interface <-> zorro_device
wire        z2_configured;
wire        z2_shutup;
wire [7:0]  z2_base_addr_hi;

wire        z2_access_strobe;
wire        z2_access_wr;
wire [1:0]  z2_access_size;
wire [23:0] z2_access_addr;
wire [31:0] z2_access_wr_data;
wire        z2_access_is_scratchpad;
wire        z2_access_is_io_regs;
wire [31:0] z2_rd_data;

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
    .drive_int2                     (drive_int2),
    .drive_int6                     (drive_int6),
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

// 2. Virtual Zorro-II AutoConfig Device Submodule
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

    // Access handshake from m68k FSM
    .access_strobe        (z2_access_strobe),
    .access_wr            (z2_access_wr),
    .access_size          (z2_access_size),
    .access_addr          (z2_access_addr),
    .access_wr_data       (z2_access_wr_data),
    .access_is_scratchpad (z2_access_is_scratchpad),
    .access_is_io_regs    (z2_access_is_io_regs),
    .access_rd_data       (z2_rd_data)
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

    // Zorro device interface
    .z2_access_strobe               (z2_access_strobe),
    .z2_access_wr                   (z2_access_wr),
    .z2_access_size                 (z2_access_size),
    .z2_access_addr                 (z2_access_addr),
    .z2_access_wr_data              (z2_access_wr_data),
    .z2_access_is_scratchpad        (z2_access_is_scratchpad),
    .z2_access_is_io_regs           (z2_access_is_io_regs),
    .z2_rd_data                     (z2_rd_data)
);

endmodule
