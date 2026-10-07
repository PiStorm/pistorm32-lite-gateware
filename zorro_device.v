/*
 * PiStorm32-lite Gateware
 *
 * Copyright 2022 Niklas Ekström
 * Copyright 2022 Claude Schwarz
 *
 * 2026 Claude Schwarz Refactor:
 *   - Virtual Zorro-II AutoConfig 64KB I/O Device
 *   - Manufacturer ID 28020 (0x6D74), Product ID 0x32 (PiStorm32), Rev 0x01
 *   - Daisy-Chain Pass-Through: Hides $00E80000 while unconfigured, then
 *     transparently forwards accesses to real external Zorro busboards
 *   - Virtual Register Space: Magic ID "PS32", Device Info, Status, Scratchpad
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
    output reg         z2_configured,        // High once Kickstart writes base address ($48/$4A)
    output reg         z2_shutup,            // High if Kickstart commands card to shut up ($4C)
    output reg  [7:0]  z2_base_addr_hi,      // Base address bits [23:16] for fast address decode

    // -------------------------------------------------------------------------
    // Internal Access Interface (from m68k_interface STATE_INTERNAL_FINISH)
    // -------------------------------------------------------------------------
    input  wire        access_strobe,        // 1-cycle active pulse on access execution
    input  wire        access_wr,            // 1 = Write transaction, 0 = Read transaction
    input  wire [1:0]  access_size,          // 2'd0 = 8-bit, 2'd1 = 16-bit, 2'd3 = 32-bit
    input  wire [23:0] access_addr,          // Latched Amiga 24-bit physical address
    input  wire [31:0] access_wr_data,       // Latched write data from Pi request buffer
    input  wire        access_is_scratchpad, // Latched comparator: access_addr is Scratchpad ($0C)
    input  wire        access_is_io_regs,    // Latched comparator: access_addr is Registers ($00..$0F)
    output wire [31:0] access_rd_data        // Multiplexed read data delivered to Pi request buffer
);

    // =========================================================================
    // Card State & Virtual Registers
    // =========================================================================
    reg [23:0] z2_base_addr  = 24'd0; // Configured base address (e.g. $00E90000)
    reg [31:0] z2_scratchpad = 32'd0; // 32-bit general purpose read/write register

    // Fast combinatorial export of base address high-byte for slot-intercept precomputation
    always @(*) begin
        z2_base_addr_hi = z2_base_addr[23:16];
    end

    // =========================================================================
    // SECTION 1: AutoConfig ROM Nibble Lookup Table
    //
    // Reference: Amiga Hardware Reference Manual, Appendix E (AutoConfig)
    // AutoConfig ROM space resides at $00E80000..$00E8007E.
    // Registers are placed on word boundaries ($00, $02, $04, ...).
    // The 4-bit nibble data is placed in the upper nibble of each byte lane.
    // Inverted nibbles (denoted by ~) are inverted per Commodore specification.
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
    // Odd byte addresses read floating bus pull-up (0xFF)
    wire [7:0]  ac_byte_val = access_addr[0] ? 8'hFF : {ac_nibble_0, 4'h0};
    wire [15:0] ac_word_val = access_addr[0] ? {8'hFF, ac_nibble_1, 4'h0} : {{ac_nibble_0, 4'h0}, 8'hFF};
    wire [31:0] ac_long_val = {{ac_nibble_0, 4'h0}, 8'hFF, {ac_nibble_1, 4'h0}, 8'hFF};

    wire [31:0] ac_read_data = (access_size == 2'd0) ? {24'd0, ac_byte_val} :
                               (access_size == 2'd1) ? {16'd0, ac_word_val} :
                               ac_long_val;

    // =========================================================================
    // SECTION 2: Virtual 64KB I/O Registers
    //
    // Available at base_addr + offset:
    //   +$00: MAGIC_ID   - ASCII "PS32" (0x50533332)
    //   +$04: DEV_INFO   - {Manufacturer[15:0], ProductID[7:0], Revision[7:0]}
    //   +$08: STATUS     - {8'h00, BaseAddr[23:16], 15'd0, Configured}
    //   +$0C: SCRATCHPAD - 32-bit read/write register with byte-lane steering
    // =========================================================================
    reg [31:0] z2_reg_data;
    always @(*) begin
        if (access_is_io_regs) begin
            case (access_addr[3:2])
                2'd0:    z2_reg_data = 32'h50533332; // ASCII "PS32"
                2'd1:    z2_reg_data = {Z2_MANUF_ID, Z2_PROD_ID, 8'h01}; // 0x6D743201
                2'd2:    z2_reg_data = {8'h00, z2_base_addr[23:16], 15'd0, z2_configured};
                2'd3:    z2_reg_data = z2_scratchpad;
                default: z2_reg_data = 32'h00000000;
            endcase
        end else begin
            z2_reg_data = 32'h00000000;
        end
    end

    // Sub-word byte selection for 8-bit reads
    reg [7:0] io_read_byte;
    always @(*) begin
        case (access_addr[1:0])
            2'd0:    io_read_byte = z2_reg_data[31:24];
            2'd1:    io_read_byte = z2_reg_data[23:16];
            2'd2:    io_read_byte = z2_reg_data[15:8];
            2'd3:    io_read_byte = z2_reg_data[7:0];
            default: io_read_byte = 8'h00;
        endcase
    end

    // Sub-word word selection for 16-bit reads
    wire [15:0] io_read_word = access_addr[1] ? z2_reg_data[15:0] : z2_reg_data[31:16];

    // Final aligned I/O read data multiplexer
    wire [31:0] io_read_data = (access_size == 2'd0) ? {24'd0, io_read_byte} :
                               (access_size == 2'd1) ? {16'd0, io_read_word} :
                               z2_reg_data;

    // Route either AutoConfig ROM or 64KB I/O registers based on configuration state
    assign access_rd_data = (!z2_configured) ? ac_read_data : io_read_data;

    // =========================================================================
    // SECTION 3: AutoConfig Write & Register Write Handler
    //
    // AutoConfig Write Registers:
    //   $00E80048 (Base High): bits [23:20] of board base address
    //   $00E8004A (Base Low):  bits [19:16] of board base address (finishes config)
    //   $00E8004C (Shut-up):   disable board until next bus reset
    //
    // Scratchpad Register Write:
    //   Supports byte, word, and longword writes with correct byte lane masks.
    // =========================================================================
    always @(posedge clk) begin
        if (reset) begin
            z2_configured <= 1'b0;
            z2_shutup     <= 1'b0;
            z2_base_addr  <= 24'd0;
            z2_scratchpad <= 32'd0;
        end else if (access_strobe && access_wr) begin
            if (!z2_configured) begin
                // AutoConfig state machine writes
                if (access_size == 2'd3 && access_addr[6:1] == 6'h24) begin
                    // 32-bit long write to $48 sets both Base High & Base Low in one cycle
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
                    // 8-bit byte write to $48 (Base High nibble)
                    z2_base_addr[23:20] <= access_wr_data[7:4] | access_wr_data[3:0];
                end else if (access_addr[6:1] == 6'h25) begin
                    // 8-bit byte write to $4A (Base Low nibble - commits configuration)
                    z2_base_addr[19:16] <= access_wr_data[7:4] | access_wr_data[3:0];
                    z2_base_addr[15:0]  <= 16'h0000;
                    z2_configured       <= 1'b1;
                end else if (access_addr[6:1] == 6'h26) begin
                    // Write to $4C (Shut-up command) - passes daisy-chain to next card
                    z2_shutup <= 1'b1;
                end
            end else begin
                // Configured 64KB I/O writes
                if (access_is_scratchpad) begin
                    case (access_size)
                        2'd0: begin // 8-bit byte write with address byte selection
                            case (access_addr[1:0])
                                2'd0: z2_scratchpad[31:24] <= access_wr_data[7:0];
                                2'd1: z2_scratchpad[23:16] <= access_wr_data[7:0];
                                2'd2: z2_scratchpad[15:8]  <= access_wr_data[7:0];
                                2'd3: z2_scratchpad[7:0]   <= access_wr_data[7:0];
                            endcase
                        end

                        2'd1: begin // 16-bit word write
                            if (access_addr[1])
                                z2_scratchpad[15:0]  <= access_wr_data[15:0];
                            else
                                z2_scratchpad[31:16] <= access_wr_data[15:0];
                        end

                        default: begin // 32-bit longword write
                            z2_scratchpad <= access_wr_data;
                        end
                    endcase
                end
            end
        end
    end

endmodule
