/*
 * PiStorm32-lite Gateware
 *
 * Copyright 2022 Niklas Ekström
 * Copyright 2022 Claude Schwarz
 *
 * 2026 Claude Schwarz Refactor:
 *   - Raspberry Pi 16-Bit Parallel GPIO Interface
 *   - Dual Request-Slot Queue (Pipelined Asynchronous Handshake)
 *   - Precalculated Zorro-II AutoConfig / IO Internal Intercept Logic
 *   - Keyboard Reset Filtering & Synchronized Amiga Interrupt Routing
 */

module pi_interface (
    input  wire        clk,

    // -------------------------------------------------------------------------
    // Physical Raspberry Pi GPIO Signals
    // -------------------------------------------------------------------------
    input  wire [2:0]  PI_A,               // Register address (GPIO[26..24])
    input  wire        PI_RD,              // Read strobe, active low (GPIO6)
    input  wire        PI_WR,              // Write strobe, active low (GPIO7)
    input  wire [15:0] PI_D_IN,            // Multiplexed data bus input (GPIO[23..8])
    output wire [15:0] PI_D_OUT,           // Multiplexed data bus output
    output wire [15:0] PI_D_OE,            // Tri-state output enable per bit
    output wire [2:0]  PI_IPL,             // Encoded interrupt level (GPIO[2..0])
    output wire        PI_TXN_IN_PROGRESS, // Busy indicator (GPIO3)
    output wire        PI_KBRESET,         // Filtered keyboard reset (GPIO4, for EMU68)

    // -------------------------------------------------------------------------
    // Status Inputs (from m68k_interface & Amiga 1200)
    // -------------------------------------------------------------------------
    input  wire [2:0]  ipl,                // Active-low Amiga IPL[2:0] synchronized
    input  wire        halt_sync,          // Amiga /HALT line synchronized
    input  wire        reset_sync,         // Amiga /RESET line synchronized
    input  wire        is_bm,              // PiStorm has bus mastership granted
    input  wire        KBRESET,            // Raw keyboard reset input from Amiga
    input  wire        mc_reset_n_sync,    // Filtered MC68020 /RESET line
    input  wire [23:0] current_bus_address,// Live address from m68k FSM for debugging

    // -------------------------------------------------------------------------
    // Control Outputs (to m68k_interface & Amiga 1200)
    // -------------------------------------------------------------------------
    output wire        request_bm,         // 1 = Request bus mastership (/BR low)
    output wire        drive_reset,        // 1 = Assert Amiga /RESET
    output wire        drive_halt,         // 1 = Assert Amiga /HALT
    output wire        drive_int2,         // 1 = Assert Level 2 interrupt (Paula)
    output wire        drive_int6,         // 1 = Assert Level 6 interrupt (CIA-B)
    output wire        increment_execute_slot_pointer, // 1 = Auto-ping-pong slot
    output wire        enable_prefetch,    // 1 = Enable speculative 32-bit read prefetch

    // -------------------------------------------------------------------------
    // Virtual Zorro Configuration Inputs (for intercept precomputation)
    // -------------------------------------------------------------------------
    input  wire        z2_configured,      // Virtual Zorro card configured
    input  wire        z2_shutup,          // Virtual Zorro card shut up
    input  wire [7:0]  z2_base_addr_hi,    // Configured base address bits [23:16]

    // -------------------------------------------------------------------------
    // Request Slot Queue Interface (to m68k_interface)
    // -------------------------------------------------------------------------
    output reg  [1:0]  req_active = 2'b00,             // Slot 0 / 1 active flags
    output reg  [1:0]  req_internal_intercept = 2'b00, // Precalculated virtual Zorro hit
    output wire        new_req_valid,                  // 1-cycle pulse when PI writes ADDR_HI
    output wire        new_req_slot,                   // Slot ID for the new request
    output wire [23:0] new_req_addr,                   // Full 24-bit physical address
    output wire        new_req_rw,                     // 1 = Read, 0 = Write
    output wire [1:0]  new_req_size,                   // 0 = Byte, 1 = Word, 3 = Longword
    output wire [23:0] req_address_0,
    output wire [23:0] req_address_1,
    output wire [31:0] req_data_write_0,
    output wire [31:0] req_data_write_1,
    output wire [1:0]  req_size_0,
    output wire [1:0]  req_size_1,
    output wire        req_rw_0,
    output wire        req_rw_1,
    output wire [2:0]  req_fc_0,
    output wire [2:0]  req_fc_1,

    // -------------------------------------------------------------------------
    // Slot Pointer Override (when PI writes PI_REG_SLOT)
    // -------------------------------------------------------------------------
    output reg         set_execute_slot_valid = 1'b0,
    output reg         set_execute_slot_val   = 1'b0,

    // -------------------------------------------------------------------------
    // Slot Completion Handshake (from m68k_interface)
    // -------------------------------------------------------------------------
    input  wire        slot_complete_valid,    // 1-cycle completion pulse
    input  wire        slot_complete_id,       // Slot ID (0 or 1) being completed
    input  wire [31:0] slot_complete_data,     // Data read from Amiga bus or Z2 registers
    input  wire        slot_complete_normally  // 1 = Terminated with DSACK, 0 = BERR
);

    // =========================================================================
    // SECTION 1: Raspberry Pi Bus Register Map & Control Register
    //
    // +-------+-----------+-------+----------------------------------------------+
    // | PI_A  | Name      | Access| Function Description                         |
    // +-------+-----------+-------+----------------------------------------------+
    // | 3'd0  | DATA_LO   | R/W   | Read: Data[15:0]  / Write: Data[15:0]        |
    // | 3'd1  | DATA_HI   | R/W   | Read: Data[31:16] / Write: Data[31:16]       |
    // | 3'd2  | ADDR_LO   | R/W   | Read: Addr[15:0]  / Write: Addr[15:0]        |
    // | 3'd3  | ADDR_HI   | R/W   | Read: Addr[23:16] / Write: Addr[23:16],      |
    // |       |           |       |   Size[9:8], R/W[10], FC[13:11], triggers req|
    // | 3'd4  | STATUS    | Read  | {8'd0, Active, TermNorm, IPL[2:0], H, R, BM} |
    // | 3'd4  | CONTROL   | Write | Bit 15: Set/Clr, Bits [6:0]: BM,RST,HLT,etc. |
    // | 3'd5  | SLOT      | Write | Bit 0: Select active slot (0 or 1)           |
    // +-------+-----------+-------+----------------------------------------------+
    // =========================================================================
    localparam [2:0] PI_REG_DATA_LO = 3'd0;
    localparam [2:0] PI_REG_DATA_HI = 3'd1;
    localparam [2:0] PI_REG_ADDR_LO = 3'd2;
    localparam [2:0] PI_REG_ADDR_HI = 3'd3;
    localparam [2:0] PI_REG_STATUS  = 3'd4;
    localparam [2:0] PI_REG_CONTROL = 3'd4;
    localparam [2:0] PI_REG_SLOT    = 3'd5;

    // Pi Control Register:
    //   Bit 0: request_bm                     - Request MC68020 bus mastership
    //   Bit 1: drive_reset                    - Drive Amiga /RESET line low
    //   Bit 2: drive_halt                     - Drive Amiga /HALT line low
    //   Bit 3: drive_int2                     - Assert Amiga INT2 (Paula, Audio/Ports)
    //   Bit 4: drive_int6                     - Assert Amiga INT6 (CIA-B, Timer)
    //   Bit 5: increment_execute_slot_pointer - Ping-pong request slots automatically
    //   Bit 6: enable_prefetch                - Enable speculative 32-bit read prefetch
    reg [14:0] pi_control = 15'b000000000000110;
    assign request_bm                     = pi_control[0];
    assign drive_reset                    = pi_control[1];
    assign drive_halt                     = pi_control[2];
    assign drive_int2                     = pi_control[3];
    assign drive_int6                     = pi_control[4];
    assign increment_execute_slot_pointer = pi_control[5];
    assign enable_prefetch                = pi_control[6];

    // =========================================================================
    // SECTION 2: Dual Request-Slot Queue
    //
    // Provides 2 independent request slots (Slot 0 and Slot 1) to decouple
    // Raspberry Pi software write speed from MC68020 bus execution latency.
    // =========================================================================
    reg [31:0] req_data_write [1:0];
    reg [31:0] req_data_read  [1:0];
    reg [23:0] req_address    [1:0];
    reg [2:0]  req_fc         [1:0];
    reg [1:0]  req_size       [1:0];
    reg        req_rw         [1:0];
    reg [1:0]  req_terminated_normally = 2'b00;

    reg current_pi_slot = 1'b0; // Active slot pointed to by the Raspberry Pi

    // Export slot registers to m68k_interface
    assign req_address_0    = req_address[0];
    assign req_address_1    = req_address[1];
    assign req_data_write_0 = req_data_write[0];
    assign req_data_write_1 = req_data_write[1];
    assign req_size_0       = req_size[0];
    assign req_size_1       = req_size[1];
    assign req_rw_0         = req_rw[0];
    assign req_rw_1         = req_rw[1];
    assign req_fc_0         = req_fc[0];
    assign req_fc_1         = req_fc[1];

    // =========================================================================
    // SECTION 3: Keyboard Reset Filtering & Status Multiplexing
    //
    // Masks keyboard reset during host-driven resets to avoid spuriously
    // reporting a physical keyboard reset (Ctrl-Amiga-Amiga) to EMU68.
    // =========================================================================
    reg mask_reset_for_pi = 1'b1;

    always @(posedge clk) begin
        if (drive_reset)
            mask_reset_for_pi <= 1'b1;
        else if (mc_reset_n_sync)
            mask_reset_for_pi <= 1'b0;
    end

    assign PI_TXN_IN_PROGRESS = req_active[current_pi_slot];
    assign PI_IPL             = ~ipl;
    assign PI_KBRESET         = KBRESET & (mask_reset_for_pi ? 1'b1 : mc_reset_n_sync);

    // =========================================================================
    // SECTION 4: Raspberry Pi Write Synchronizer & Register Decoding
    //
    // Detects falling edge on PI_WR to capture address and data reliably.
    // Precomputes req_internal_intercept in parallel with ADDR_HI write.
    // =========================================================================
    (* async_reg = "true" *) reg [1:0] pi_wr_sync;
    reg [15:0] q_PI_D_IN;

    // Strobe signals to m68k_interface for new request dispatch
    assign new_req_valid = (pi_wr_sync == 2'b10) && (PI_A == PI_REG_ADDR_HI);
    assign new_req_slot  = current_pi_slot;
    assign new_req_addr  = {q_PI_D_IN[7:0], req_address[current_pi_slot][15:0]};
    assign new_req_rw    = q_PI_D_IN[10];
    assign new_req_size  = q_PI_D_IN[9:8];

    always @(posedge clk) begin
        pi_wr_sync <= {pi_wr_sync[0], PI_WR};
        q_PI_D_IN  <= PI_D_IN;

        set_execute_slot_valid <= 1'b0;

        // Capture completed slot transactions from m68k_interface
        if (slot_complete_valid) begin
            req_data_read[slot_complete_id]           <= slot_complete_data;
            req_terminated_normally[slot_complete_id] <= slot_complete_normally;
            req_active[slot_complete_id]              <= 1'b0;
        end

        // Falling edge of PI_WR: execute register write
        if (pi_wr_sync == 2'b10) begin
            case (PI_A)
                PI_REG_DATA_LO: req_data_write[current_pi_slot][15:0] <= q_PI_D_IN;
                PI_REG_DATA_HI: req_data_write[current_pi_slot][31:16] <= q_PI_D_IN;
                PI_REG_ADDR_LO: req_address[current_pi_slot][15:0] <= q_PI_D_IN;
                PI_REG_ADDR_HI: begin
                    req_address[current_pi_slot][23:16] <= q_PI_D_IN[7:0];
                    req_size[current_pi_slot]           <= q_PI_D_IN[9:8];
                    req_rw[current_pi_slot]             <= q_PI_D_IN[10];
                    req_fc[current_pi_slot]             <= q_PI_D_IN[13:11];
                    req_active[current_pi_slot]         <= 1'b1;

                    // Synchronously precalculate if this access targets our virtual Zorro card
                    // Unconfigured card responds at $00E80000..$00E8007E.
                    // Configured card responds at base_addr_hi ($00E9xxxx etc.).
                    req_internal_intercept[current_pi_slot] <= !z2_shutup && (
                        (!z2_configured && (q_PI_D_IN[7:0] == 8'hE8) && (req_address[current_pi_slot][15:7] == 9'd0)) ||
                        (z2_configured && (q_PI_D_IN[7:0] == z2_base_addr_hi))
                    );
                end

                PI_REG_CONTROL: begin
                    // Bit 15 selects set (1) or clear (0) operation on lower 15 bits
                    if (PI_D_IN[15])
                        pi_control <= pi_control | q_PI_D_IN[14:0];
                    else
                        pi_control <= pi_control & ~q_PI_D_IN[14:0];
                end

                PI_REG_SLOT: begin
                    current_pi_slot <= q_PI_D_IN[0];
                    if (!increment_execute_slot_pointer) begin
                        set_execute_slot_valid <= 1'b1;
                        set_execute_slot_val   <= q_PI_D_IN[0];
                    end
                end
            endcase
        end

        // Bus reset invalidates any internal intercepts
        if (reset_sync || drive_reset) begin
            req_internal_intercept <= 2'b00;
        end
    end

    // =========================================================================
    // SECTION 5: Raspberry Pi Read Multiplexer & Bus Driver
    //
    // Asynchronous read data multiplexing driven onto PI_D_OUT when PI_RD is low.
    // =========================================================================
    reg [15:0] pi_data_out;
    assign PI_D_OUT = pi_data_out;

    wire drive_pi_data_out = !PI_RD && PI_WR;
    assign PI_D_OE = {16{drive_pi_data_out}};

    wire [15:0] pi_status = {
        8'd0,
        req_active[current_pi_slot],
        req_terminated_normally[current_pi_slot],
        ipl,
        halt_sync,
        reset_sync,
        is_bm
    };

    reg [31:0] q_req_data_read;
    always @(posedge clk) begin
        q_req_data_read <= req_data_read[current_pi_slot];
    end

    always @(*) begin
        case (PI_A)
            PI_REG_DATA_LO: pi_data_out = q_req_data_read[15:0];
            PI_REG_DATA_HI: pi_data_out = q_req_data_read[31:16];
            PI_REG_ADDR_LO: pi_data_out = current_bus_address[15:0];
            PI_REG_ADDR_HI: pi_data_out = {8'd0, current_bus_address[23:16]};
            PI_REG_STATUS:  pi_data_out = pi_status;
            default:        pi_data_out = 16'bx;
        endcase
    end

endmodule
