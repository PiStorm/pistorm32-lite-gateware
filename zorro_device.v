/*
 * PiStorm32-lite Gateware
 *
 * Copyright 2022 Niklas Ekström
 * Copyright 2022 Claude Schwarz
 *
 * 2026 Claude Schwarz Refactor:
 *   - Virtual Zorro-II AutoConfig 64KB I/O Device
 *   - Wishbone B4 Pipelined/Classic Interconnect Architecture
 *   - Timing-Isolated Peripheral Bus with Handshake (access_valid / access_ready)
 *   - Dedicated Amiga Interrupt Subsystem (INT2 Paula / INT6 CIA-B)
 *   - Manufacturer ID 28020 (0x6D74), Product ID 0x32 (PiStorm32), Rev 0x01
 *   - Daisy-Chain Pass-Through: Hides $00E80000 while unconfigured, then
 *     transparently forwards accesses to real external Zorro busboards
 *   - 64KB Address Map:
 *       $0000..$00FF: Slave 0 - Core Registers & Interrupt Management
 *       $0100..$01FF: Slave 1 - Peripheral Slot (SPI Master Drop-in)
 *       $0200..$FFFF: Unmapped space (terminates safely with 0)
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
    output reg  [31:0] access_rd_data = 32'd0// Read data delivered to Pi request buffer
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
    reg        wb_stb_def = 1'b0;
    reg        wb_we      = 1'b0;
    reg [15:0] wb_adr     = 16'd0;
    reg [31:0] wb_dat_m2s = 32'd0;
    reg [3:0]  wb_sel     = 4'b0000;
    wire [31:0] wb_dat_s2m;
    wire        wb_ack;

    // -------------------------------------------------------------------------
    // Main Handshake & Wishbone Master State Machine
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            z2_configured   <= 1'b0;
            z2_shutup       <= 1'b0;
            z2_base_addr    <= 24'd0;
            wb_state        <= WB_IDLE;
            wb_cyc          <= 1'b0;
            wb_stb          <= 1'b0;
            wb_stb_s0       <= 1'b0;
            wb_stb_s1       <= 1'b0;
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
                            z2_base_addr[23:20] <= access_wr_data[31:28] | access_wr_data[27:24];
                            z2_base_addr[19:16] <= access_wr_data[23:20] | access_wr_data[19:16];
                            z2_base_addr[15:0]  <= 16'h0000;
                            z2_configured       <= 1'b1;
                        end else if (access_size == 2'd1 && access_addr[6:1] == 6'h24) begin
                            // 16-bit word write to $48 sets both Base High & Base Low
                            z2_base_addr[23:20] <= access_wr_data[15:12] | access_wr_data[11:8];
                            z2_base_addr[19:16] <= access_wr_data[7:4]   | access_wr_data[3:0];
                            z2_base_addr[15:0]  <= 16'h0000;
                            z2_configured       <= 1'b1;
                        end else if (access_addr[6:1] == 6'h24) begin
                            // 8-bit byte write to $48 (Base High)
                            z2_base_addr[23:20] <= access_wr_data[7:4] | access_wr_data[3:0];
                        end else if (access_addr[6:1] == 6'h25) begin
                            // 8-bit byte write to $4A (Base Low - commits configuration)
                            z2_base_addr[19:16] <= access_wr_data[7:4] | access_wr_data[3:0];
                            z2_base_addr[15:0]  <= 16'h0000;
                            z2_configured       <= 1'b1;
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
                            wb_stb_def <= (access_addr[15:8] >= 8'h02);
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
    reg         def_ack = 1'b0;

    always @(posedge clk) begin
        if (reset)
            def_ack <= 1'b0;
        else
            def_ack <= wb_stb_def && !def_ack;
    end

    assign wb_ack = s0_ack | s1_ack | def_ack;
    assign wb_dat_s2m = s0_ack ? s0_dat_o :
                        s1_ack ? s1_dat_o :
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

    assign s0_ack   = s0_ack_reg;
    assign s0_dat_o = s0_reg_data;

    // Active-high interrupt outputs to PS32-lite top-level OR gates
    assign z2_int2 = int2_pending && int2_enable;
    assign z2_int6 = int6_pending && int6_enable;

    always @(posedge clk) begin
        if (reset) begin
            s0_ack_reg    <= 1'b0;
            s0_reg_data   <= 32'd0;
            z2_scratchpad <= 32'd0;
            int2_pending  <= 1'b0;
            int6_pending  <= 1'b0;
            int2_enable   <= 1'b0;
            int6_enable   <= 1'b0;
        end else begin
            s0_ack_reg <= 1'b0;

            if (wb_stb_s0 && !s0_ack_reg) begin
                s0_ack_reg <= 1'b1;

                // Read Multiplexer
                case (wb_adr[5:2])
                    4'h0: s0_reg_data <= 32'h50533332; // "PS32"
                    4'h1: s0_reg_data <= {Z2_MANUF_ID, Z2_PROD_ID, 8'h01}; // 0x6D743201
                    4'h2: s0_reg_data <= {8'h00, z2_base_addr[23:16], 15'd0, z2_configured};
                    4'h3: s0_reg_data <= z2_scratchpad;
                    4'h4: s0_reg_data <= {28'd0, s1_irq, 1'b0, int6_pending, int2_pending};
                    4'h5: s0_reg_data <= {30'd0, int6_enable, int2_enable};
                    default: s0_reg_data <= 32'd0;
                endcase

                // Write Handling
                if (wb_we) begin
                    case (wb_adr[5:2])
                        4'h3: begin // Scratchpad write with byte lane enables
                            if (wb_sel[3]) z2_scratchpad[31:24] <= wb_dat_m2s[31:24];
                            if (wb_sel[2]) z2_scratchpad[23:16] <= wb_dat_m2s[23:16];
                            if (wb_sel[1]) z2_scratchpad[15:8]  <= wb_dat_m2s[15:8];
                            if (wb_sel[0]) z2_scratchpad[7:0]   <= wb_dat_m2s[7:0];
                        end
                        4'h4: begin // INT_STATUS: Write-1-to-clear
                            if (wb_sel[0]) begin
                                if (wb_dat_m2s[0]) int2_pending <= 1'b0;
                                if (wb_dat_m2s[1]) int6_pending <= 1'b0;
                            end
                        end
                        4'h5: begin // INT_ENABLE: Read/Write
                            if (wb_sel[0]) begin
                                int2_enable <= wb_dat_m2s[0];
                                int6_enable <= wb_dat_m2s[1];
                            end
                        end
                        4'h6: begin // INT_FORCE: Write-only trigger
                            if (wb_sel[0]) begin
                                if (wb_dat_m2s[0]) int2_pending <= 1'b1;
                                if (wb_dat_m2s[1]) int6_pending <= 1'b1;
                            end
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

endmodule
