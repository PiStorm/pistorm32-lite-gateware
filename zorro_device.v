/*
 * Copyright 2022 Niklas Ekström
 * Copyright 2022 Claude Schwarz
 *
 * Virtual Zorro-II AutoConfig Device (64KB I/O)
 * Manufacturer ID: 28020 (0x6D74)
 * Product ID:      0x32 (PiStorm32)
 */

module zorro_device #(
    parameter [15:0] Z2_MANUF_ID = 16'd28020, // 0x6D74
    parameter [7:0]  Z2_PROD_ID  = 8'h32,     // PiStorm32 Product ID
    parameter [31:0] Z2_SERIAL   = 32'd1      // Serial Number
)(
    input               clk,
    input               reset,                // reset_sync || drive_reset

    // Configuration status outputs (for intercept precomputation)
    output reg          z2_configured,
    output reg          z2_shutup,
    output reg [7:0]    z2_base_addr_hi,      // z2_base_addr[23:16]

    // Access interface from m68k_interface
    input               access_strobe,        // 1-cycle strobe in STATE_INTERNAL_FINISH
    input               access_wr,            // 1 = write, 0 = read
    input [1:0]         access_size,          // 0->8b, 1->16b, 3->32b
    input [23:0]        access_addr,          // latched internal_addr
    input [31:0]        access_wr_data,       // latched internal_wr_data
    input               access_is_scratchpad, // latched flag for offset $0C
    input               access_is_io_regs,    // latched flag for offset $00..$0F
    output [31:0]       access_rd_data        // internal_read_data
);

reg [23:0] z2_base_addr = 24'd0;
reg [31:0] z2_scratchpad = 32'd0;

always @(*) begin
    z2_base_addr_hi = z2_base_addr[23:16];
end

function [3:0] get_ac_nibble;
    input [5:0] idx;
    case (idx)
        6'h00: get_ac_nibble = 4'b1100; // er_Type[7:4]: Zorro II, no memlist, no rom
        6'h01: get_ac_nibble = 4'b0001; // er_Type[3:0]: 64KB
        6'h02: get_ac_nibble = ~Z2_PROD_ID[7:4]; // Product ID high (~0x3 = 4'hC)
        6'h03: get_ac_nibble = ~Z2_PROD_ID[3:0]; // Product ID low  (~0x2 = 4'hD)
        6'h04: get_ac_nibble = 4'hF; // er_Flags: no bootrom, any space fits (~0x0 = 0xF)
        6'h05: get_ac_nibble = 4'hF;
        6'h06: get_ac_nibble = 4'hF; // Reserved = 0xF
        6'h07: get_ac_nibble = 4'hF;
        6'h08: get_ac_nibble = ~Z2_MANUF_ID[15:12]; // Manufacturer 0x6D74 -> ~0x6 = 4'h9
        6'h09: get_ac_nibble = ~Z2_MANUF_ID[11:8];  // -> ~0xD = 4'h2
        6'h0A: get_ac_nibble = ~Z2_MANUF_ID[7:4];   // -> ~0x7 = 4'h8
        6'h0B: get_ac_nibble = ~Z2_MANUF_ID[3:0];   // -> ~0x4 = 4'hB
        6'h0C: get_ac_nibble = ~Z2_SERIAL[31:28];   // Serial -> 0xF
        6'h0D: get_ac_nibble = ~Z2_SERIAL[27:24];   // -> 0xF
        6'h0E: get_ac_nibble = ~Z2_SERIAL[23:20];   // -> 0xF
        6'h0F: get_ac_nibble = ~Z2_SERIAL[19:16];   // -> 0xF
        6'h10: get_ac_nibble = ~Z2_SERIAL[15:12];   // -> 0xF
        6'h11: get_ac_nibble = ~Z2_SERIAL[11:8];    // -> 0xF
        6'h12: get_ac_nibble = ~Z2_SERIAL[7:4];     // -> 0xF
        6'h13: get_ac_nibble = ~Z2_SERIAL[3:0];     // Serial low: ~0x1 = 4'hE
        6'h20: get_ac_nibble = 4'h0; // Interrupt/Status = 0
        6'h21: get_ac_nibble = 4'h0;
        default: get_ac_nibble = 4'hF;
    endcase
endfunction

wire [3:0] ac_nibble_0 = get_ac_nibble(access_addr[6:1]);
wire [3:0] ac_nibble_1 = get_ac_nibble(access_addr[6:1] + 6'd1);

wire [7:0]  ac_byte_val = access_addr[0] ? 8'hFF : {ac_nibble_0, 4'h0};
wire [15:0] ac_word_val = access_addr[0] ? {8'hFF, ac_nibble_1, 4'h0} : {{ac_nibble_0, 4'h0}, 8'hFF};
wire [31:0] ac_long_val = {{ac_nibble_0, 4'h0}, 8'hFF, {ac_nibble_1, 4'h0}, 8'hFF};

wire [31:0] ac_read_data = (access_size == 2'd0) ? {24'd0, ac_byte_val} :
                           (access_size == 2'd1) ? {16'd0, ac_word_val} :
                           ac_long_val;

reg [31:0] z2_reg_data;
always @(*) begin
    if (access_is_io_regs) begin
        case (access_addr[3:2])
            2'd0: z2_reg_data = 32'h50533332; // ASCII "PS32"
            2'd1: z2_reg_data = {Z2_MANUF_ID, Z2_PROD_ID, 8'h01};
            2'd2: z2_reg_data = {8'h00, z2_base_addr[23:16], 15'd0, z2_configured};
            2'd3: z2_reg_data = z2_scratchpad;
        endcase
    end else begin
        z2_reg_data = 32'h00000000;
    end
end

reg [7:0] io_read_byte;
always @(*) begin
    case (access_addr[1:0])
        2'd0: io_read_byte = z2_reg_data[31:24];
        2'd1: io_read_byte = z2_reg_data[23:16];
        2'd2: io_read_byte = z2_reg_data[15:8];
        2'd3: io_read_byte = z2_reg_data[7:0];
    endcase
end

wire [15:0] io_read_word = access_addr[1] ? z2_reg_data[15:0] : z2_reg_data[31:16];
wire [31:0] io_read_data = (access_size == 2'd0) ? {24'd0, io_read_byte} :
                           (access_size == 2'd1) ? {16'd0, io_read_word} :
                           z2_reg_data;

assign access_rd_data = (!z2_configured) ? ac_read_data : io_read_data;

// Write handling
always @(posedge clk) begin
    if (reset) begin
        z2_configured <= 1'b0;
        z2_shutup     <= 1'b0;
        z2_base_addr  <= 24'd0;
        z2_scratchpad <= 32'd0;
    end else if (access_strobe && access_wr) begin
        if (!z2_configured) begin
            if (access_size == 2'd3 && access_addr[6:1] == 6'h24) begin
                // 32-bit long write to $48 sets both base nibbles
                z2_base_addr[23:20] <= access_wr_data[31:28] | access_wr_data[27:24];
                z2_base_addr[19:16] <= access_wr_data[23:20] | access_wr_data[19:16];
                z2_base_addr[15:0]  <= 16'h0000;
                z2_configured       <= 1'b1;
            end else if (access_size == 2'd1 && access_addr[6:1] == 6'h24) begin
                // 16-bit word write to $48 sets both base nibbles
                z2_base_addr[23:20] <= access_wr_data[15:12] | access_wr_data[11:8];
                z2_base_addr[19:16] <= access_wr_data[7:4] | access_wr_data[3:0];
                z2_base_addr[15:0]  <= 16'h0000;
                z2_configured       <= 1'b1;
            end else if (access_addr[6:1] == 6'h24) begin
                // 8-bit byte write to $48 (Base High)
                z2_base_addr[23:20] <= access_wr_data[7:4] | access_wr_data[3:0];
            end else if (access_addr[6:1] == 6'h25) begin
                // 8-bit byte write to $4A (Base Low)
                z2_base_addr[19:16] <= access_wr_data[7:4] | access_wr_data[3:0];
                z2_base_addr[15:0]  <= 16'h0000;
                z2_configured       <= 1'b1;
            end else if (access_addr[6:1] == 6'h26) begin
                // Write to $4C (Shut-up)
                z2_shutup <= 1'b1;
            end
        end else begin
            // Write to 64KB I/O space
            if (access_is_scratchpad) begin // Scratchpad at offset $0C
                case (access_size)
                    2'd0: begin // Byte write
                        case (access_addr[1:0])
                            2'd0: z2_scratchpad[31:24] <= access_wr_data[7:0];
                            2'd1: z2_scratchpad[23:16] <= access_wr_data[7:0];
                            2'd2: z2_scratchpad[15:8]  <= access_wr_data[7:0];
                            2'd3: z2_scratchpad[7:0]   <= access_wr_data[7:0];
                        endcase
                    end
                    2'd1: begin // Word write
                        if (access_addr[1])
                            z2_scratchpad[15:0] <= access_wr_data[15:0];
                        else
                            z2_scratchpad[31:16] <= access_wr_data[15:0];
                    end
                    default: begin // Long write
                        z2_scratchpad <= access_wr_data;
                    end
                endcase
            end
        end
    end
end

endmodule
