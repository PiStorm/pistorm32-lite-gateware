/*
 * PiStorm32-lite Gateware - Virtual Zorro-II AutoConfig & Wishbone Subsystem
 *
 * Copyright 2022 Niklas Ekström
 * Copyright 2022-2026 Claude Schwarz
 */

module zorro_device #(
    parameter [15:0] Z2_MANUF_ID = 16'd28020, // Assigned Manufacturer ID: 28020 (0x6D74)
    parameter [7:0]  Z2_PROD_ID  = 8'h32,     // PiStorm32 Product ID: 0x32
    parameter [31:0] Z2_SERIAL   = 32'd1      // Card Serial Number: 1
)(
    input  wire        clk,
    input  wire        reset,                // Active-high synchronous reset (reset_sync || drive_reset)

    // -------------------------------------------------------------------------
    // Configuration Status Outputs (to pi_interface & m68k_interface)
    // -------------------------------------------------------------------------
    output reg         z2_configured = 1'b0, // High once Kickstart writes base address ($48/$4A)
    output reg         z2_shutup = 1'b0,     // High if Kickstart commands card to shut up ($4C)
    output reg  [7:0]  z2_base_addr_hi = 8'd0,// Base address bits [23:16] for fast address decode

    // -------------------------------------------------------------------------
    // Amiga Interrupt Requests (to Paula & CIA-B via PS32-lite OR-gates)
    // -------------------------------------------------------------------------
    output wire        z2_int2,              // Assert Amiga Level 2 Interrupt (Paula)
    output wire        z2_int6,              // Assert Amiga Level 6 Interrupt (CIA-B)

    // -------------------------------------------------------------------------
    // Internal Access Interface (from/to m68k_interface STATE_INTERNAL_FINISH)
    // -------------------------------------------------------------------------
    input  wire        access_valid,         // 1 = Access request pending
    output reg         access_ready = 1'b0,  // 1 = Access completed (1-cycle pulse)
    input  wire        access_wr,            // 1 = Write transaction, 0 = Read transaction
    input  wire [1:0]  access_size,          // 2'd0 = 8-bit, 2'd1 = 16-bit, 2'd3 = 32-bit
    input  wire [23:0] access_addr,          // Latched Amiga 24-bit physical address
    input  wire [31:0] access_wr_data,       // Latched write data from Pi request buffer
    output reg  [31:0] access_rd_data = 32'd0,// Read data delivered to Pi request buffer

    // -------------------------------------------------------------------------
    // Auxiliary / Expansion Port & Debug Wiring (SPARE[7:0] + EMU68 UART)
    // -------------------------------------------------------------------------
    input  wire [7:0]  SPARE_IN,             // Live physical pin inputs from expansion header
    output wire [7:0]  SPARE_OUT,            // Physical pin output drivers
    output wire [7:0]  SPARE_OE,             // Physical pin output enables (1=Drive, 0=Hi-Z)
    input  wire        PI_SER_DAT,           // EMU68 serial debug data from Pi (GPIO5)
    input  wire        PI_SER_CLK,           // EMU68 serial debug clock from Pi (GPIO27)

    // -------------------------------------------------------------------------
    // Hardware Diagnostic & Telemetry Interface (from/to m68k_interface)
    // -------------------------------------------------------------------------
    input  wire [31:0] prefetch_launch_count,
    input  wire [31:0] prefetch_hit_count,
    input  wire [31:0] diag_status,
    input  wire [31:0] diag_bus_capture,
    input  wire [31:0] diag_cycle_timing,
    input  wire [31:0] diag_clock_phase,
    input  wire        phase_calibrated,
    input  wire        fast_read_phase_cal,
    input  wire        current_cck_phase,
    output reg         prefetch_ctrl_en = 1'b1,
    output reg         fast_dsack_en = 1'b0,
    output reg         cck_sync_en = 1'b0,
    output reg         force_phase_invert = 1'b0,
    output reg         counter_clear = 1'b0
);

    // =========================================================================
    // Card Base Address Registers & Fast Export
    // =========================================================================
    reg [23:0] z2_base_addr = 24'd0;

    always @(*) begin
        z2_base_addr_hi = z2_base_addr[23:16];
    end

    // =========================================================================
    // SECTION 1: AutoConfig ROM Nibble Lookup Table
    //
    // Reference: Amiga Hardware Reference Manual, Appendix E (AutoConfig)
    // AutoConfig ROM space resides at $00E80000..$00E8007E.
    // =========================================================================
    function [3:0] get_ac_nibble;
        input [5:0] idx; // Word index: address[6:1]
        case (idx)
            // Header: er_Type
            // Bits [7:6] = 11: Zorro II board
            // Bit  [5]   = 0:  No address space list
            // Bit  [4]   = 0:  No ROM vector / DIAG list
            // Bits [2:0] = 001: 64KB board size
            6'h00:   get_ac_nibble = 4'b1100; // er_Type [7:4]
            6'h01:   get_ac_nibble = 4'b0001; // er_Type [3:0] (64KB size)

            // Product ID (inverted)
            6'h02:   get_ac_nibble = ~Z2_PROD_ID[7:4]; // Product ID high nibble (~0x3 = 4'hC)
            6'h03:   get_ac_nibble = ~Z2_PROD_ID[3:0]; // Product ID low nibble  (~0x2 = 4'hD)

            // Flags & Reserved (inverted)
            6'h04:   get_ac_nibble = 4'hF;    // er_Flags: no sub-sizing, any 64KB slot valid
            6'h05:   get_ac_nibble = 4'hF;    // Reserved
            6'h06:   get_ac_nibble = 4'hF;    // Reserved
            6'h07:   get_ac_nibble = 4'hF;    // Reserved

            // Manufacturer ID (28020 = 0x6D74, inverted)
            6'h08:   get_ac_nibble = ~Z2_MANUF_ID[15:12]; // ~0x6 = 4'h9
            6'h09:   get_ac_nibble = ~Z2_MANUF_ID[11:8];  // ~0xD = 4'h2
            6'h0A:   get_ac_nibble = ~Z2_MANUF_ID[7:4];   // ~0x7 = 4'h8
            6'h0B:   get_ac_nibble = ~Z2_MANUF_ID[3:0];   // ~0x4 = 4'hB

            // Serial Number (32-bit, inverted)
            6'h0C:   get_ac_nibble = ~Z2_SERIAL[31:28];
            6'h0D:   get_ac_nibble = ~Z2_SERIAL[27:24];
            6'h0E:   get_ac_nibble = ~Z2_SERIAL[23:20];
            6'h0F:   get_ac_nibble = ~Z2_SERIAL[19:16];
            6'h10:   get_ac_nibble = ~Z2_SERIAL[15:12];
            6'h11:   get_ac_nibble = ~Z2_SERIAL[11:8];
            6'h12:   get_ac_nibble = ~Z2_SERIAL[7:4];
            6'h13:   get_ac_nibble = ~Z2_SERIAL[3:0];   // ~0x1 = 4'hE

            // Optional Control/Status nibbles
            6'h20:   get_ac_nibble = 4'h0;    // Interrupt status = 0
            6'h21:   get_ac_nibble = 4'h0;

            default: get_ac_nibble = 4'hF;    // Unused ROM entries return 0xF
        endcase
    endfunction

    // Form AutoConfig nibble pairs for even and odd word accesses
    wire [3:0] ac_nibble_0 = get_ac_nibble(access_addr[6:1]);
    wire [3:0] ac_nibble_1 = get_ac_nibble(access_addr[6:1] + 6'd1);

    // Format AutoConfig ROM output for 8-bit, 16-bit, and 32-bit reads
    wire [7:0]  ac_byte_val = access_addr[0] ? 8'hFF : {ac_nibble_0, 4'h0};
    wire [15:0] ac_word_val = access_addr[0] ? {8'hFF, ac_nibble_1, 4'h0} : {{ac_nibble_0, 4'h0}, 8'hFF};
    wire [31:0] ac_long_val = {{ac_nibble_0, 4'h0}, 8'hFF, {ac_nibble_1, 4'h0}, 8'hFF};

    wire [31:0] ac_read_data = (access_size == 2'd0) ? {24'd0, ac_byte_val} :
                               (access_size == 2'd1) ? {16'd0, ac_word_val} :
                               ac_long_val;

    // =========================================================================
    // SECTION 2: Wishbone B4 Master Bridge & Bus Interconnect
    // =========================================================================
    localparam WB_IDLE = 1'b0;
    localparam WB_WAIT = 1'b1;
    reg wb_state = WB_IDLE;

    reg        wb_cyc     = 1'b0;
    reg        wb_stb     = 1'b0;
    reg        wb_stb_s0  = 1'b0;
    reg        wb_stb_s1  = 1'b0;
    reg        wb_stb_s2  = 1'b0;
    reg        wb_stb_def = 1'b0;
    reg        wb_we      = 1'b0;
    reg [15:0] wb_adr     = 16'd0;
    reg [31:0] wb_dat_m2s = 32'd0;
    reg [3:0]  wb_sel     = 4'b0000;
    wire [31:0] wb_dat_s2m;
    wire        wb_ack;

    // AutoConfig byte/word write extraction (safe against unwritten DATA_HI)
    wire [7:0] ac_wr_byte = (access_wr_data[7:0] != 8'h00) ? access_wr_data[7:0] : access_wr_data[15:8];
    wire [3:0] ac_byte_hi = ac_wr_byte[7:4];
    wire [3:0] ac_byte_lo = ac_wr_byte[3:0];

    reg base_hi_written = 1'b0;
    reg base_lo_written = 1'b0;

    // -------------------------------------------------------------------------
    // Main Handshake & Wishbone Master State Machine
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            z2_configured   <= 1'b0;
            z2_shutup       <= 1'b0;
            z2_base_addr    <= 24'd0;
            base_hi_written <= 1'b0;
            base_lo_written <= 1'b0;
            wb_state        <= WB_IDLE;
            wb_cyc          <= 1'b0;
            wb_stb          <= 1'b0;
            wb_stb_s0       <= 1'b0;
            wb_stb_s1       <= 1'b0;
            wb_stb_s2       <= 1'b0;
            wb_stb_def      <= 1'b0;
            wb_we           <= 1'b0;
            wb_adr          <= 16'd0;
            wb_dat_m2s      <= 32'd0;
            wb_sel          <= 4'b0000;
            access_ready    <= 1'b0;
            access_rd_data  <= 32'd0;
        end else begin
            access_ready <= 1'b0;

            if (!z2_configured) begin
                // -------------------------------------------------------------
                // Unconfigured State: AutoConfig ROM and Address Assignment
                // -------------------------------------------------------------
                if (access_valid && !access_ready) begin
                    access_ready   <= 1'b1;
                    access_rd_data <= ac_read_data;

                    if (access_wr) begin
                        if (access_size == 2'd3 && access_addr[6:1] == 6'h24) begin
                            // 32-bit write to $48 sets both Base High & Base Low
                            z2_base_addr[23:20] <= access_wr_data[31:28];
                            z2_base_addr[19:16] <= access_wr_data[27:24];
                            z2_base_addr[15:0]  <= 16'h0000;
                            z2_configured       <= 1'b1;
                        end else if (access_size == 2'd1 && access_addr[6:1] == 6'h24) begin
                            // 16-bit word write to $48 sets both Base High & Base Low
                            z2_base_addr[23:20] <= access_wr_data[15:12];
                            z2_base_addr[19:16] <= access_wr_data[11:8];
                            z2_base_addr[15:0]  <= 16'h0000;
                            z2_configured       <= 1'b1;
                        end else if (access_addr[6:1] == 6'h24) begin
                            // Byte write to $48 (ec_BaseAddress)
                            z2_base_addr[23:20] <= ac_byte_hi;
                            base_hi_written     <= 1'b1;
                            if (ac_byte_lo != 4'h0) begin
                                // Full byte written to $48 (contains both nibbles, e.g. $E9 from Kickstart)
                                z2_base_addr[19:16] <= ac_byte_lo;
                                z2_base_addr[15:0]  <= 16'h0000;
                                z2_configured       <= 1'b1;
                            end else if (base_lo_written) begin
                                // Low nibble was already latched by prior write to $4A
                                z2_base_addr[15:0]  <= 16'h0000;
                                z2_configured       <= 1'b1;
                            end
                        end else if (access_addr[6:1] == 6'h25) begin
                            // Byte write to $4A (ec_BaseAddress+2, Base Low)
                            z2_base_addr[19:16] <= (ac_byte_hi != 4'h0) ? ac_byte_hi : ac_byte_lo;
                            base_lo_written     <= 1'b1;
                            if (base_hi_written) begin
                                z2_base_addr[15:0]  <= 16'h0000;
                                z2_configured       <= 1'b1;
                            end
                        end else if (access_addr[6:1] == 6'h26) begin
                            // Write to $4C (Shut-up command)
                            z2_shutup <= 1'b1;
                        end
                    end
                end
            end else begin
                // -------------------------------------------------------------
                // Configured State: 64KB I/O via Wishbone B4 Interconnect
                // -------------------------------------------------------------
                case (wb_state)
                    WB_IDLE: begin
                        if (access_valid && !access_ready) begin
                            wb_cyc     <= 1'b1;
                            wb_stb     <= 1'b1;
                            wb_stb_s0  <= (access_addr[15:8] == 8'h00);
                            wb_stb_s1  <= (access_addr[15:8] == 8'h01);
                            wb_stb_s2  <= (access_addr[15:8] == 8'h02);
                            wb_stb_def <= (access_addr[15:8] >= 8'h03);
                            wb_we      <= access_wr;
                            wb_adr     <= access_addr[15:0];

                            // Steer write data and generate Wishbone byte enables
                            case (access_size)
                                2'd0: begin // 8-bit Byte
                                    wb_dat_m2s <= {access_wr_data[7:0], access_wr_data[7:0],
                                                   access_wr_data[7:0], access_wr_data[7:0]};
                                    case (access_addr[1:0])
                                        2'd0: wb_sel <= 4'b1000;
                                        2'd1: wb_sel <= 4'b0100;
                                        2'd2: wb_sel <= 4'b0010;
                                        2'd3: wb_sel <= 4'b0001;
                                    endcase
                                end
                                2'd1: begin // 16-bit Word
                                    wb_dat_m2s <= {access_wr_data[15:0], access_wr_data[15:0]};
                                    if (access_addr[1])
                                        wb_sel <= 4'b0011;
                                    else
                                        wb_sel <= 4'b1100;
                                end
                                default: begin // 32-bit Longword
                                    wb_dat_m2s <= access_wr_data;
                                    wb_sel     <= 4'b1111;
                                end
                            endcase

                            wb_state <= WB_WAIT;
                        end
                    end

                    WB_WAIT: begin
                        if (wb_ack) begin
                            wb_cyc       <= 1'b0;
                            wb_stb       <= 1'b0;
                            wb_stb_s0    <= 1'b0;
                            wb_stb_s1    <= 1'b0;
                            wb_stb_s2    <= 1'b0;
                            wb_stb_def   <= 1'b0;
                            access_ready <= 1'b1;

                            // Demultiplex read data for Pi host buffer
                            case (access_size)
                                2'd0: begin // 8-bit Byte
                                    case (wb_adr[1:0])
                                        2'd0: access_rd_data <= {24'd0, wb_dat_s2m[31:24]};
                                        2'd1: access_rd_data <= {24'd0, wb_dat_s2m[23:16]};
                                        2'd2: access_rd_data <= {24'd0, wb_dat_s2m[15:8]};
                                        2'd3: access_rd_data <= {24'd0, wb_dat_s2m[7:0]};
                                    endcase
                                end
                                2'd1: begin // 16-bit Word
                                    if (wb_adr[1])
                                        access_rd_data <= {16'd0, wb_dat_s2m[15:0]};
                                    else
                                        access_rd_data <= {16'd0, wb_dat_s2m[31:16]};
                                end
                                default: begin // 32-bit Longword
                                    access_rd_data <= wb_dat_s2m;
                                end
                            endcase

                            wb_state <= WB_IDLE;
                        end
                    end
                endcase
            end
        end
    end

    // =========================================================================
    // SECTION 3: Wishbone Address Decoder & Bus Crossbar
    //
    // Note: Slave strobes wb_stb_s0, wb_stb_s1, wb_stb_def are registered
    // directly in the master FSM (WB_IDLE) to eliminate combinational decoding
    // delays on slave clock-enable lines, achieving 182+ MHz timing closure.
    // =========================================================================

    wire [31:0] s0_dat_o;
    wire        s0_ack;
    wire [31:0] s1_dat_o;
    wire        s1_ack;
    wire [31:0] s2_dat_o;
    wire        s2_ack;
    reg         def_ack = 1'b0;

    always @(posedge clk) begin
        if (reset)
            def_ack <= 1'b0;
        else
            def_ack <= wb_stb_def && !def_ack;
    end

    assign wb_ack = s0_ack | s1_ack | s2_ack | def_ack;
    assign wb_dat_s2m = s0_ack ? s0_dat_o :
                        s1_ack ? s1_dat_o :
                        s2_ack ? s2_dat_o :
                        32'h00000000;

    // =========================================================================
    // SECTION 4: Slave 0 - Core Registers & Amiga Interrupt Controller ($0000..$00FF)
    //
    // Register Map:
    //   +$00: MAGIC_ID   - ASCII "PS32" (0x50533332)
    //   +$04: DEV_INFO   - {Manufacturer[15:0], ProductID[7:0], Revision[7:0]}
    //   +$08: STATUS     - {8'h00, BaseAddr[23:16], 15'd0, Configured}
    //   +$0C: SCRATCHPAD - 32-bit R/W register with Wishbone byte enables
    //   +$10: INT_STATUS - {28'd0, s1_irq, 1'b0, int6_pending, int2_pending} (W1C)
    //   +$14: INT_ENABLE - {30'd0, int6_enable, int2_enable} (R/W)
    //   +$18: INT_FORCE  - Software interrupt trigger (Write-only)
    // =========================================================================
    reg [31:0] s0_reg_data = 32'd0;
    reg        s0_ack_reg  = 1'b0;

    reg [31:0] z2_scratchpad = 32'd0;
    reg        int2_pending  = 1'b0;
    reg        int6_pending  = 1'b0;
    reg        int2_enable   = 1'b0;
    reg        int6_enable   = 1'b0;

    wire       s1_irq; // Interrupt line from Slave 1
    wire       s2_irq; // Interrupt line from Slave 2 (GPIO Matrix)
    wire [31:0] gpio_int_cfg; // Interrupt configuration from Slave 2

    assign s0_ack   = s0_ack_reg;
    assign s0_dat_o = s0_reg_data;

    // Active-high interrupt outputs to PS32-lite top-level OR gates
    wire gpio_to_int2 = s2_irq && (!gpio_int_cfg[16]);
    wire gpio_to_int6 = s2_irq && (gpio_int_cfg[16]);

    assign z2_int2 = (int2_pending && int2_enable) | gpio_to_int2;
    assign z2_int6 = (int6_pending && int6_enable) | gpio_to_int6;

    always @(posedge clk) begin
        if (reset) begin
            s0_ack_reg       <= 1'b0;
            s0_reg_data      <= 32'd0;
            z2_scratchpad    <= 32'd0;
            int2_pending     <= 1'b0;
            int6_pending     <= 1'b0;
            int2_enable      <= 1'b0;
            int6_enable      <= 1'b0;
            prefetch_ctrl_en   <= 1'b1;
            fast_dsack_en      <= 1'b0;
            cck_sync_en        <= 1'b0;
            force_phase_invert <= 1'b0;
            counter_clear      <= 1'b0;
        end else begin
            s0_ack_reg    <= 1'b0;
            counter_clear <= 1'b0;

            if (wb_stb_s0 && !s0_ack_reg) begin
                s0_ack_reg <= 1'b1;

                // Read Multiplexer
                case (wb_adr[6:2])
                    5'h00: s0_reg_data <= 32'h50533332; // "PS32"
                    5'h01: s0_reg_data <= {Z2_MANUF_ID, Z2_PROD_ID, 8'h01}; // 0x6D743201
                    5'h02: s0_reg_data <= {8'h00, z2_base_addr[23:16], 15'd0, z2_configured};
                    5'h03: s0_reg_data <= z2_scratchpad;
                    5'h04: s0_reg_data <= {27'd0, s2_irq, s1_irq, 1'b0, int6_pending, int2_pending};
                    5'h05: s0_reg_data <= {30'd0, int6_enable, int2_enable};
                    5'h06: s0_reg_data <= 32'd0;
                    5'h07: s0_reg_data <= {25'd0, current_cck_phase, fast_read_phase_cal, phase_calibrated, force_phase_invert, cck_sync_en, fast_dsack_en, prefetch_ctrl_en}; // +$1C: BUS_CTRL
                    5'h08: s0_reg_data <= diag_status;               // +$20: DIAG_STATUS
                    5'h09: s0_reg_data <= prefetch_launch_count;     // +$24: PREFETCH_LAUNCH_COUNT
                    5'h0A: s0_reg_data <= prefetch_hit_count;        // +$28: PREFETCH_HIT_COUNT
                    5'h0B: s0_reg_data <= diag_bus_capture;          // +$2C: DIAG_BUS_CAPTURE
                    5'h0C: s0_reg_data <= diag_cycle_timing;        // +$30: DIAG_CYCLE_TIMING
                    5'h0D: s0_reg_data <= diag_clock_phase;         // +$34: DIAG_CLOCK_PHASE
                    default: s0_reg_data <= 32'd0;
                endcase

                // Write Handling
                if (wb_we) begin
                    case (wb_adr[6:2])
                        5'h03: begin // Scratchpad write with byte lane enables
                            if (wb_sel[3]) z2_scratchpad[31:24] <= wb_dat_m2s[31:24];
                            if (wb_sel[2]) z2_scratchpad[23:16] <= wb_dat_m2s[23:16];
                            if (wb_sel[1]) z2_scratchpad[15:8]  <= wb_dat_m2s[15:8];
                            if (wb_sel[0]) z2_scratchpad[7:0]   <= wb_dat_m2s[7:0];
                        end
                        5'h04: begin // INT_STATUS: Write-1-to-clear
                            if (wb_sel[0]) begin
                                if (wb_dat_m2s[0]) int2_pending <= 1'b0;
                                if (wb_dat_m2s[1]) int6_pending <= 1'b0;
                            end
                        end
                        5'h05: begin // INT_ENABLE: Read/Write
                            if (wb_sel[0]) begin
                                int2_enable <= wb_dat_m2s[0];
                                int6_enable <= wb_dat_m2s[1];
                            end
                        end
                        5'h06: begin // INT_FORCE: Write-only trigger
                            if (wb_sel[0]) begin
                                if (wb_dat_m2s[0]) int2_pending <= 1'b1;
                                if (wb_dat_m2s[1]) int6_pending <= 1'b1;
                            end
                        end
                        5'h07: begin // PREFETCH_CTRL / BUS_CTRL: Read/Write (+$1C)
                            if (wb_sel[0]) begin
                                prefetch_ctrl_en   <= wb_dat_m2s[0];
                                fast_dsack_en      <= wb_dat_m2s[1];
                                cck_sync_en        <= wb_dat_m2s[2];
                                force_phase_invert <= wb_dat_m2s[3];
                            end
                        end
                        5'h09, 5'h0A: begin // Counter clear trigger on write to +$24 or +$28
                            counter_clear <= 1'b1;
                        end
                    endcase
                end
            end
        end
    end

    // =========================================================================
    // SECTION 5: Slave 1 - Peripheral Slot / SPI Master Drop-in ($0100..$01FF)
    //
    // Register Map (Ready for SPI Master core integration):
    //   +$00: SPI_CTRL   - Control & Status register
    //   +$04: SPI_DATA   - TX/RX Data Register
    //   +$08: SPI_CLKDIV - SCLK Baud Rate Divider
    // =========================================================================
    reg        s1_ack_reg  = 1'b0;
    reg [31:0] s1_reg_data = 32'd0;
    reg [31:0] spi_ctrl    = 32'd0;
    reg [31:0] spi_data    = 32'd0;
    reg [31:0] spi_clkdiv  = 32'd0;

    assign s1_ack   = s1_ack_reg;
    assign s1_dat_o = s1_reg_data;
    assign s1_irq   = spi_ctrl[31]; // Top bit indicates interrupt pending

    always @(posedge clk) begin
        if (reset) begin
            s1_ack_reg  <= 1'b0;
            s1_reg_data <= 32'd0;
            spi_ctrl    <= 32'd0;
            spi_data    <= 32'd0;
            spi_clkdiv  <= 32'd0;
        end else begin
            s1_ack_reg <= 1'b0;

            if (wb_stb_s1 && !s1_ack_reg) begin
                s1_ack_reg <= 1'b1;

                case (wb_adr[4:2])
                    3'h0: s1_reg_data <= spi_ctrl;
                    3'h1: s1_reg_data <= spi_data;
                    3'h2: s1_reg_data <= spi_clkdiv;
                    default: s1_reg_data <= 32'd0;
                endcase

                if (wb_we) begin
                    case (wb_adr[4:2])
                        3'h0: begin
                            if (wb_sel[0]) spi_ctrl[7:0]   <= wb_dat_m2s[7:0];
                            if (wb_sel[1]) spi_ctrl[15:8]  <= wb_dat_m2s[15:8];
                            if (wb_sel[2]) spi_ctrl[23:16] <= wb_dat_m2s[23:16];
                            if (wb_sel[3]) spi_ctrl[31:24] <= wb_dat_m2s[31:24];
                        end
                        3'h1: begin
                            if (wb_sel[0]) spi_data[7:0]   <= wb_dat_m2s[7:0];
                            if (wb_sel[1]) spi_data[15:8]  <= wb_dat_m2s[15:8];
                            if (wb_sel[2]) spi_data[23:16] <= wb_dat_m2s[23:16];
                            if (wb_sel[3]) spi_data[31:24] <= wb_dat_m2s[31:24];
                        end
                        3'h2: begin
                            if (wb_sel[0]) spi_clkdiv[7:0]   <= wb_dat_m2s[7:0];
                            if (wb_sel[1]) spi_clkdiv[15:8]  <= wb_dat_m2s[15:8];
                            if (wb_sel[2]) spi_clkdiv[23:16] <= wb_dat_m2s[23:16];
                            if (wb_sel[3]) spi_clkdiv[31:24] <= wb_dat_m2s[31:24];
                        end
                    endcase
                end
            end
        end
    end

    wire spi_sclk = spi_ctrl[0];
    wire spi_mosi = spi_data[0];
    wire spi_cs_n = spi_ctrl[1];
    wire spi_miso;

    // =========================================================================
    // SECTION 6: Slave 2 - ESP32-Style GPIO Matrix & IO MUX ($0200..$02FF)
    //
    // Flexible I/O multiplexer allowing any expansion port pin (SPARE[7:0]) to
    // be routed to GPIO, Raspberry Pi Debug (EMU68 UART), SPI Master, or IRQ.
    // Supports ESP32-style per-pin function selection, atomic bit-set/clear,
    // direction control, output inversion, and open-drain emulation.
    // =========================================================================
    reg        s2_ack_reg  = 1'b0;
    reg [31:0] s2_reg_data = 32'd0;

    // Slave 2 Registers
    reg [31:0] iomux_ctrl        = 32'd0;        // $00: Mode & Matrix enable
    reg [7:0]  gpio_out          = 8'hFF;        // $08: GPIO output register
    reg [7:0]  gpio_oe           = 8'hFF;        // $14: GPIO direction register (1=out, 0=in)
    reg [7:0]  pin_cfg [7:0];                    // $20..$3C: Per-pin configuration
    reg [31:0] in_mat_sel        = 32'h00000004; // $40: Input Matrix (MISO default Pin 4)
    reg [31:0] gpio_int_cfg_reg  = 32'd0;        // $44: Interrupt configuration
    reg [7:0]  gpio_int_pending  = 8'd0;         // $48: Interrupt status (W1C)
    // -------------------------------------------------------------------------
    // SPARE_IN 2-Stage Synchronizer & Edge Detection
    // -------------------------------------------------------------------------
    reg [7:0] spare_sync_0    = 8'd0;
    reg [7:0] spare_sync_1    = 8'd0;
    reg [7:0] spare_sync_2    = 8'd0;
    reg [7:0] gpio_triggers_q = 8'd0;

    always @(posedge clk) begin
        spare_sync_0    <= SPARE_IN;
        spare_sync_1    <= spare_sync_0;
        spare_sync_2    <= spare_sync_1;
        gpio_triggers_q <= ((gpio_int_cfg_reg[15:8] & (spare_sync_1 & ~spare_sync_2)) |
                            (~gpio_int_cfg_reg[15:8] & spare_sync_1)) & gpio_int_cfg_reg[7:0];
    end

    // Peripheral Input Routing via Input Matrix (using synchronized inputs)
    wire [2:0] miso_pin_sel = in_mat_sel[2:0];
    wire       raw_miso     = spare_sync_1[miso_pin_sel];
    assign     spi_miso     = in_mat_sel[6] ? ~raw_miso : raw_miso;

    assign gpio_int_cfg = gpio_int_cfg_reg;
    assign s2_ack       = s2_ack_reg;
    assign s2_dat_o     = s2_reg_data;
    assign s2_irq       = |gpio_int_pending;

    always @(posedge clk) begin
        if (reset) begin
            s2_ack_reg        <= 1'b0;
            s2_reg_data       <= 32'd0;
            iomux_ctrl        <= 32'd0;
            gpio_out          <= 8'hFF;
            gpio_oe           <= 8'hFF;
            pin_cfg[0]        <= 8'h01; // Pin 0: PI_SER_DAT
            pin_cfg[1]        <= 8'h02; // Pin 1: PI_SER_CLK
            pin_cfg[2]        <= 8'h00; // Pin 2: GPIO
            pin_cfg[3]        <= 8'h00; // Pin 3: GPIO
            pin_cfg[4]        <= 8'h00; // Pin 4: GPIO
            pin_cfg[5]        <= 8'h00; // Pin 5: GPIO
            pin_cfg[6]        <= 8'h00; // Pin 6: GPIO
            pin_cfg[7]        <= 8'h00; // Pin 7: GPIO
            in_mat_sel        <= 32'h00000004;
            gpio_int_cfg_reg  <= 32'd0;
            gpio_int_pending  <= 8'd0;
        end else begin
            s2_ack_reg <= 1'b0;

            // Accumulate pending interrupts from pipelined triggers (supports W1C in write block)
            gpio_int_pending <= gpio_int_pending | gpio_triggers_q;

            if (wb_stb_s2 && !s2_ack_reg) begin
                s2_ack_reg <= 1'b1;

                // Read Multiplexer ($0200..$02FF)
                // Upper 24 bits [31:8]: Only 3 valid 32-bit registers (iomux_ctrl, in_mat_sel, gpio_int_cfg_reg)
                // drastically reducing logic depth and wb_adr fanout for timing closure.
                case (wb_adr[6:2])
                    5'h00:   s2_reg_data[31:8] <= iomux_ctrl[31:8];
                    5'h10:   s2_reg_data[31:8] <= in_mat_sel[31:8];
                    5'h11:   s2_reg_data[31:8] <= gpio_int_cfg_reg[31:8];
                    default: s2_reg_data[31:8] <= 24'd0;
                endcase

                // Lower 8 bits [7:0]: All registers mapped to LSB
                case (wb_adr[6:2])
                    5'h00:   s2_reg_data[7:0] <= iomux_ctrl[7:0];
                    5'h01:   s2_reg_data[7:0] <= spare_sync_1;
                    5'h02:   s2_reg_data[7:0] <= gpio_out;
                    5'h05:   s2_reg_data[7:0] <= gpio_oe;
                    5'h08:   s2_reg_data[7:0] <= pin_cfg[0];
                    5'h09:   s2_reg_data[7:0] <= pin_cfg[1];
                    5'h0A:   s2_reg_data[7:0] <= pin_cfg[2];
                    5'h0B:   s2_reg_data[7:0] <= pin_cfg[3];
                    5'h0C:   s2_reg_data[7:0] <= pin_cfg[4];
                    5'h0D:   s2_reg_data[7:0] <= pin_cfg[5];
                    5'h0E:   s2_reg_data[7:0] <= pin_cfg[6];
                    5'h0F:   s2_reg_data[7:0] <= pin_cfg[7];
                    5'h10:   s2_reg_data[7:0] <= in_mat_sel[7:0];
                    5'h11:   s2_reg_data[7:0] <= gpio_int_cfg_reg[7:0];
                    5'h12:   s2_reg_data[7:0] <= gpio_int_pending;
                    default: s2_reg_data[7:0] <= 8'd0;
                endcase

                // Write Handling
                if (wb_we) begin
                    case (wb_adr[6:2])
                        5'h00: begin // IOMUX_CTRL
                            if (wb_sel[0]) iomux_ctrl[7:0]   <= wb_dat_m2s[7:0];
                            if (wb_sel[1]) iomux_ctrl[15:8]  <= wb_dat_m2s[15:8];
                            if (wb_sel[2]) iomux_ctrl[23:16] <= wb_dat_m2s[23:16];
                            if (wb_sel[3]) iomux_ctrl[31:24] <= wb_dat_m2s[31:24];
                        end
                        5'h02: begin // GPIO_OUT (Direct Write)
                            if (|wb_sel) gpio_out <= wb_dat_m2s[7:0];
                        end
                        5'h03: begin // GPIO_OUT_SET (W1TS - Atomic Bit Set)
                            if (|wb_sel) gpio_out <= gpio_out | wb_dat_m2s[7:0];
                        end
                        5'h04: begin // GPIO_OUT_CLR (W1TC - Atomic Bit Clear)
                            if (|wb_sel) gpio_out <= gpio_out & ~wb_dat_m2s[7:0];
                        end
                        5'h05: begin // GPIO_DIR (Direct Write)
                            if (|wb_sel) gpio_oe <= wb_dat_m2s[7:0];
                        end
                        5'h06: begin // GPIO_DIR_SET (W1TS - Atomic Direction Set)
                            if (|wb_sel) gpio_oe <= gpio_oe | wb_dat_m2s[7:0];
                        end
                        5'h07: begin // GPIO_DIR_CLR (W1TC - Atomic Direction Clear)
                            if (|wb_sel) gpio_oe <= gpio_oe & ~wb_dat_m2s[7:0];
                        end
                        5'h08: if (|wb_sel) pin_cfg[0] <= wb_dat_m2s[7:0]; // PIN_CFG0
                        5'h09: if (|wb_sel) pin_cfg[1] <= wb_dat_m2s[7:0]; // PIN_CFG1
                        5'h0A: if (|wb_sel) pin_cfg[2] <= wb_dat_m2s[7:0]; // PIN_CFG2
                        5'h0B: if (|wb_sel) pin_cfg[3] <= wb_dat_m2s[7:0]; // PIN_CFG3
                        5'h0C: if (|wb_sel) pin_cfg[4] <= wb_dat_m2s[7:0]; // PIN_CFG4
                        5'h0D: if (|wb_sel) pin_cfg[5] <= wb_dat_m2s[7:0]; // PIN_CFG5
                        5'h0E: if (|wb_sel) pin_cfg[6] <= wb_dat_m2s[7:0]; // PIN_CFG6
                        5'h0F: if (|wb_sel) pin_cfg[7] <= wb_dat_m2s[7:0]; // PIN_CFG7
                        5'h10: begin // IN_MAT_SEL
                            if (wb_sel[0]) in_mat_sel[7:0]   <= wb_dat_m2s[7:0];
                            if (wb_sel[1]) in_mat_sel[15:8]  <= wb_dat_m2s[15:8];
                            if (wb_sel[2]) in_mat_sel[23:16] <= wb_dat_m2s[23:16];
                            if (wb_sel[3]) in_mat_sel[31:24] <= wb_dat_m2s[31:24];
                        end
                        5'h11: begin // GPIO_INT_CFG
                            if (wb_sel[0]) gpio_int_cfg_reg[7:0]   <= wb_dat_m2s[7:0];
                            if (wb_sel[1]) gpio_int_cfg_reg[15:8]  <= wb_dat_m2s[15:8];
                            if (wb_sel[2]) gpio_int_cfg_reg[23:16] <= wb_dat_m2s[23:16];
                            if (wb_sel[3]) gpio_int_cfg_reg[31:24] <= wb_dat_m2s[31:24];
                        end
                        5'h12: begin // GPIO_INT_STATUS (W1C)
                            if (|wb_sel) gpio_int_pending <= (gpio_int_pending & ~wb_dat_m2s[7:0]) | gpio_triggers_q;
                        end
                    endcase
                end
            end
        end
    end

    // =========================================================================
    // SECTION 7: Output & Direction Multiplexing Logic
    // =========================================================================
    wire is_matrix_mode = (iomux_ctrl[2:0] == 3'd7) || iomux_ctrl[7];

    // Function multiplexer for each of the 8 pins in Matrix Mode
    function get_pin_func_val;
        input [3:0] func;
        input [2:0] pin_idx;
        case (func)
            4'h0: get_pin_func_val = gpio_out[pin_idx];
            4'h1: get_pin_func_val = PI_SER_DAT;
            4'h2: get_pin_func_val = PI_SER_CLK;
            4'h3: get_pin_func_val = spi_sclk;
            4'h4: get_pin_func_val = spi_mosi;
            4'h5: get_pin_func_val = spi_cs_n;
            4'h6: get_pin_func_val = z2_int2;
            4'h7: get_pin_func_val = z2_int6;
            4'h8: get_pin_func_val = 1'b0;
            4'h9: get_pin_func_val = 1'b1;
            default: get_pin_func_val = 1'b0;
        endcase
    endfunction

    reg [7:0] raw_out;
    reg [7:0] raw_oe;
    reg       func_bit;
    integer p;

    always @(*) begin
        if (is_matrix_mode) begin
            // -----------------------------------------------------------------
            // Full ESP32-Style GPIO Matrix Mode
            // -----------------------------------------------------------------
            for (p = 0; p < 8; p = p + 1) begin
                // 1. Function Selection
                func_bit = get_pin_func_val(pin_cfg[p][3:0], p[2:0]);

                // 2. Inversion
                if (pin_cfg[p][6])
                    func_bit = ~func_bit;
                raw_out[p] = func_bit;

                // 3. Direction / Output Enable Mode
                case (pin_cfg[p][5:4])
                    2'b00: raw_oe[p] = gpio_oe[p]; // Follow GPIO_DIR
                    2'b01: raw_oe[p] = 1'b1;       // Force Output
                    2'b10: raw_oe[p] = 1'b0;       // Force Input (Hi-Z)
                    2'b11: begin                   // Auto Peripheral
                        case (pin_cfg[p][3:0])
                            4'h3, 4'h4, 4'h5: raw_oe[p] = 1'b1; // SPI outputs
                            default: raw_oe[p] = gpio_oe[p];
                        endcase
                    end
                endcase

                // 4. Open-Drain Emulation: if open-drain enabled and output is high, float (OE=0)
                if (pin_cfg[p][7] && raw_out[p])
                    raw_oe[p] = 1'b0;
            end
        end else begin
            // -----------------------------------------------------------------
            // Preset Modes (0 = Default, 1 = All GPIO, 2 = SPI+Debug, 3 = SPI Full)
            // -----------------------------------------------------------------
            case (iomux_ctrl[2:0])
                3'd0: begin // Mode 0: Default (Debug on 0/1, 6 GPIOs on 7:2)
                    raw_out[0] = PI_SER_DAT;
                    raw_oe[0]  = 1'b1;
                    raw_out[1] = PI_SER_CLK;
                    raw_oe[1]  = 1'b1;
                    raw_out[7:2] = gpio_out[7:2];
                    raw_oe[7:2]  = gpio_oe[7:2];
                end

                3'd1: begin // Mode 1: All GPIO (Debug Disabled)
                    raw_out = gpio_out;
                    raw_oe  = gpio_oe;
                end

                3'd2: begin // Mode 2: SPI Master + Debug
                    raw_out[0] = PI_SER_DAT;
                    raw_oe[0]  = 1'b1;
                    raw_out[1] = PI_SER_CLK;
                    raw_oe[1]  = 1'b1;
                    raw_out[2] = spi_cs_n;
                    raw_oe[2]  = 1'b1;
                    raw_out[3] = spi_sclk;
                    raw_oe[3]  = 1'b1;
                    raw_out[4] = spi_mosi;
                    raw_oe[4]  = 1'b1;
                    raw_out[5] = 1'b0;
                    raw_oe[5]  = 1'b0; // Input for MISO
                    raw_out[7:6] = gpio_out[7:6];
                    raw_oe[7:6]  = gpio_oe[7:6];
                end

                3'd3: begin // Mode 3: SPI Master Full (No Debug)
                    raw_out[0] = spi_cs_n;
                    raw_oe[0]  = 1'b1;
                    raw_out[1] = spi_sclk;
                    raw_oe[1]  = 1'b1;
                    raw_out[2] = spi_mosi;
                    raw_oe[2]  = 1'b1;
                    raw_out[3] = 1'b0;
                    raw_oe[3]  = 1'b0; // Input for MISO
                    raw_out[7:4] = gpio_out[7:4];
                    raw_oe[7:4]  = gpio_oe[7:4];
                end

                default: begin
                    raw_out = gpio_out;
                    raw_oe  = gpio_oe;
                end
            endcase
        end
    end

    assign SPARE_OUT = raw_out;
    assign SPARE_OE  = raw_oe;

endmodule
