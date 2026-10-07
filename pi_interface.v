/*
 * Copyright 2022 Niklas Ekström
 * Copyright 2022 Claude Schwarz
 *
 * Raspberry Pi Interface & Request Slot Queue
 */

module pi_interface (
    input               clk,

    // Physical Raspberry Pi GPIO signals
    input [2:0]         PI_A,
    input               PI_RD,
    input               PI_WR,
    input [15:0]        PI_D_IN,
    output [15:0]       PI_D_OUT,
    output [15:0]       PI_D_OE,
    output [2:0]        PI_IPL,
    output              PI_TXN_IN_PROGRESS,
    output              PI_KBRESET,

    // Status inputs from m68k / Amiga
    input [2:0]         ipl,
    input               halt_sync,
    input               reset_sync,
    input               is_bm,
    input               KBRESET,
    input               mc_reset_n_sync,
    input [23:0]        current_bus_address,

    // Control outputs to m68k / Amiga
    output              request_bm,
    output              drive_reset,
    output              drive_halt,
    output              drive_int2,
    output              drive_int6,
    output              increment_execute_slot_pointer,
    output              enable_prefetch,

    // Virtual Zorro AutoConfig status inputs (for internal intercept precomputation)
    input               z2_configured,
    input               z2_shutup,
    input [7:0]         z2_base_addr_hi,

    // Slot request outputs to m68k_interface
    output reg [1:0]    req_active = 2'b00,
    output reg [1:0]    req_internal_intercept = 2'b00,
    output              new_req_valid,
    output              new_req_slot,
    output [23:0]       new_req_addr,
    output              new_req_rw,
    output [1:0]        new_req_size,
    output [23:0]       req_address_0,
    output [23:0]       req_address_1,
    output [31:0]       req_data_write_0,
    output [31:0]       req_data_write_1,
    output [1:0]        req_size_0,
    output [1:0]        req_size_1,
    output              req_rw_0,
    output              req_rw_1,
    output [2:0]        req_fc_0,
    output [2:0]        req_fc_1,

    // Slot pointer override (when PI_REG_SLOT is written)
    output reg          set_execute_slot_valid = 1'b0,
    output reg          set_execute_slot_val = 1'b0,

    // Slot completion inputs from m68k_interface
    input               slot_complete_valid,
    input               slot_complete_id,
    input [31:0]        slot_complete_data,
    input               slot_complete_normally
);

// Register addresses
localparam [2:0] PI_REG_DATA_LO = 3'd0;
localparam [2:0] PI_REG_DATA_HI = 3'd1;
localparam [2:0] PI_REG_ADDR_LO = 3'd2;
localparam [2:0] PI_REG_ADDR_HI = 3'd3;
localparam [2:0] PI_REG_STATUS  = 3'd4;
localparam [2:0] PI_REG_CONTROL = 3'd4;
localparam [2:0] PI_REG_SLOT    = 3'd5;

// Pi control register
reg [14:0] pi_control = 15'b000000000000110;
assign request_bm                     = pi_control[0];
assign drive_reset                    = pi_control[1];
assign drive_halt                     = pi_control[2];
assign drive_int2                     = pi_control[3];
assign drive_int6                     = pi_control[4];
assign increment_execute_slot_pointer = pi_control[5];
assign enable_prefetch                = pi_control[6];

// Request slots
reg [31:0] req_data_write [1:0];
reg [31:0] req_data_read [1:0];
reg [23:0] req_address [1:0];
reg [2:0]  req_fc [1:0];
reg [1:0]  req_size [1:0];
reg        req_rw [1:0];
reg [1:0]  req_terminated_normally = 2'b00;

reg current_pi_slot = 1'b0;

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

// Keyboard reset masking
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

// PI Bus write synchronizer
(* async_reg = "true" *) reg [1:0] pi_wr_sync;
reg [15:0] q_PI_D_IN;

assign new_req_valid = (pi_wr_sync == 2'b10) && (PI_A == PI_REG_ADDR_HI);
assign new_req_slot  = current_pi_slot;
assign new_req_addr  = {q_PI_D_IN[7:0], req_address[current_pi_slot][15:0]};
assign new_req_rw    = q_PI_D_IN[10];
assign new_req_size  = q_PI_D_IN[9:8];

always @(posedge clk) begin
    pi_wr_sync <= {pi_wr_sync[0], PI_WR};
    q_PI_D_IN  <= PI_D_IN;

    set_execute_slot_valid <= 1'b0;

    // Slot completion from m68k_interface
    if (slot_complete_valid) begin
        req_data_read[slot_complete_id]           <= slot_complete_data;
        req_terminated_normally[slot_complete_id] <= slot_complete_normally;
        req_active[slot_complete_id]              <= 1'b0;
    end

    // PI Register writes
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

                req_internal_intercept[current_pi_slot] <= !z2_shutup && (
                    (!z2_configured && (q_PI_D_IN[7:0] == 8'hE8) && (req_address[current_pi_slot][15:7] == 9'd0)) ||
                    (z2_configured && (q_PI_D_IN[7:0] == z2_base_addr_hi))
                );
            end
            PI_REG_CONTROL: begin
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

    if (reset_sync || drive_reset) begin
        req_internal_intercept <= 2'b00;
    end
end

// PI Bus read logic
reg [15:0] pi_data_out;
assign PI_D_OUT = pi_data_out;

wire drive_pi_data_out = !PI_RD && PI_WR;
assign PI_D_OE = {16{drive_pi_data_out}};

wire [15:0] pi_status = {8'd0, req_active[current_pi_slot], req_terminated_normally[current_pi_slot], ipl, halt_sync, reset_sync, is_bm};

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
