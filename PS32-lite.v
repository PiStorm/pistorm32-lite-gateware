/*
 * Copyright 2022 Niklas Ekström
 * Copyright 2022 Claude Schwarz
 */
 
module pistorm(
    // Raspberry Pi signals.
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

    // Shared data and address bus multiplexing.
    input [31:0]    DA_IN,
    output [31:0]   DA_OUT,
    output [31:0]   DA_OE,
    output          ADDR_LE,
    output          ADDR_OE_n,
    output          DATA_OE_n,
    output          CTRL_OE_n,

    // MC68EC020 signals.
    // Table 3-2.
    // Table references in this file are to the MC68020UM manual.
    output [2:0]    MC_FC_OUT,      // Can be tri-stated.
    output [2:0]    MC_FC_OE,
    output [1:0]    MC_SIZE_OUT,    // Can be tri-stated.
    output [1:0]    MC_SIZE_OE,
    output          MC_RW_OUT,      // Can be tri-stated.
    output          MC_RW_OE,
    //output          MC_RMC_n,     // Not used.
    input           MC_AS_n_IN,     // Can be tri-stated.
    output          MC_AS_n_OUT,
    output          MC_AS_n_OE,
    output          MC_DS_n_OUT,    // Can be tri-stated.
    output          MC_DS_n_OE,
    input [1:0]     MC_DSACK_n,
    input [2:0]     MC_IPL_n,
    //input           MC_AVEC_n,    // Not used.
    output          MC_BR_n_OUT,    // Open drain.
    output          MC_BR_n_OE,
    input           MC_BG_n,
    input           MC_RESET_n_IN,  // Open drain.
    output          MC_RESET_n_OUT,
    output          MC_RESET_n_OE,
    input           MC_HALT_n_IN,   // Open drain.
    output          MC_HALT_n_OUT,
    output          MC_HALT_n_OE,
    input           MC_BERR_n,
    input           MC_CLK,

    // Miscellaneous Amiga 1200 signals.
    output          INT2_n_OUT, // Open drain.
    output          INT2_n_OE,
    output          INT6_n_OUT, // Open drain.
    output          INT6_n_OE,
    input           KBRESET,
    
    //EXT Port
    input [7:0]    SPARE_IN,
    output [7:0]   SPARE_OUT,
    output [7:0]   SPARE_OE,
    
    //PLL
    //output MC_CLK_TO_PLL,
    input AMIPLL_CLKOUT0
    //input AMIPLL_LOCK
    //input MC_CLK_CLEAN
);

assign SPARE_OUT[7:2] = 6'b111111;
assign SPARE_OE =  8'b11111111;

//EMU68
assign SPARE_OUT[0] = PI_SER_DAT;
assign SPARE_OUT[1] = PI_SER_CLK;

// ## Main clock.
wire clk;

// Connect the PLL.
assign clk = AMIPLL_CLKOUT0;


// Pi control register.
reg [14:0] pi_control = 15'b000000000000110;
wire request_bm = pi_control[0];
wire drive_reset = pi_control[1];
wire drive_halt = pi_control[2];
wire drive_int2 = pi_control[3];
wire drive_int6 = pi_control[4];
wire increment_execute_slot_pointer = pi_control[5];
wire enable_prefetch = pi_control[6];

// ### MC bus signals.
assign MC_BR_n_OUT = 1'b0;
assign MC_BR_n_OE = request_bm;
assign MC_RESET_n_OUT = 1'b0;
assign MC_RESET_n_OE = drive_reset;
assign MC_HALT_n_OUT = 1'b0;
assign MC_HALT_n_OE = drive_halt;
assign INT2_n_OUT = 1'b0;
assign INT2_n_OE = drive_int2;
assign INT6_n_OUT = 1'b0;
assign INT6_n_OE = drive_int6;

reg is_bm;
reg reset_sync;
reg halt_sync;
reg [2:0] ipl;

reg [2:0]   mc_fc;
reg [23:0]  mc_address;
reg [1:0]   mc_size;
reg         mc_rw;
reg [31:0]  mc_data_read;
reg [31:0]  mc_data_write;
reg         mc_as;
reg         mc_ds;


// Universal Amiga 1200 Clock & Glitch Synchronizer
// Protects against 1.8V-2.4V ringing dips on ungemoddeten boards (E123C/E125C 47pF caps)
// and handles Amiga XOR-derived duty cycle asymmetry and cycle-to-cycle jitter.
localparam [2:0] MC_CLK_LOCKOUT_TICKS = 3'd3; // ~16.5 ns lockout at 182 MHz

(* async_reg = "true" *) reg [1:0] mc_clk_raw_sync = 2'b00;
reg       mc_clk_filtered = 1'b0;
reg [2:0] mc_clk_lockout  = 3'd0;
reg       rising          = 1'b0;
reg       falling         = 1'b0;

always @(posedge clk) begin
    mc_clk_raw_sync <= {mc_clk_raw_sync[0], MC_CLK};
    rising  <= 1'b0;
    falling <= 1'b0;

    if (mc_clk_lockout != 3'd0) begin
        mc_clk_lockout <= mc_clk_lockout - 3'd1;
    end else begin
        if (mc_clk_raw_sync[0] && !mc_clk_filtered) begin
            rising          <= 1'b1;
            mc_clk_filtered <= 1'b1;
            mc_clk_lockout  <= MC_CLK_LOCKOUT_TICKS;
        end else if (!mc_clk_raw_sync[0] && mc_clk_filtered) begin
            falling         <= 1'b1;
            mc_clk_filtered <= 1'b0;
            mc_clk_lockout  <= MC_CLK_LOCKOUT_TICKS;
        end
    end
end 


assign MC_FC_OUT = mc_fc;
assign MC_FC_OE = {3{is_bm}};
assign MC_SIZE_OUT = mc_size;
assign MC_SIZE_OE = {2{is_bm}};
assign MC_RW_OUT = mc_rw;
assign MC_RW_OE = is_bm;
assign MC_AS_n_OUT = !mc_as; //Claude
assign MC_AS_n_OE = is_bm;
assign MC_DS_n_OUT = !mc_ds; //Claude
assign MC_DS_n_OE = is_bm;

// Shared bus control.
localparam [1:0] DA_STATE_IDLE = 2'd0;
localparam [1:0] DA_STATE_DATA_TO_FPGA = 2'd1;
localparam [1:0] DA_STATE_FPGA_TO_ADDR = 2'd2;
localparam [1:0] DA_STATE_FPGA_TO_DATA = 2'd3;

reg [1:0] da_state = DA_STATE_IDLE;

reg address_latch_le = 1'b0;

assign ADDR_LE = address_latch_le;
assign ADDR_OE_n = !is_bm;

assign CTRL_OE_n = 1'b0;
assign DATA_OE_n = !(da_state == DA_STATE_DATA_TO_FPGA || da_state == DA_STATE_FPGA_TO_DATA);

// Address scrambling on PCB
wire [31:0] s_mc_address = {8'd0, mc_address};

wire [31:0] da_address = {
    s_mc_address[31], s_mc_address[30], s_mc_address[12], s_mc_address[13], // 31:28
    s_mc_address[ 7], s_mc_address[ 6], s_mc_address[15], s_mc_address[14], // 27:24
    s_mc_address[29], s_mc_address[28], s_mc_address[26], s_mc_address[27], // 23:20
    s_mc_address[16], s_mc_address[17], s_mc_address[ 5], s_mc_address[ 4], // 19:16
    s_mc_address[19], s_mc_address[18], s_mc_address[25], s_mc_address[24], // 15:12
    s_mc_address[ 8], s_mc_address[ 9], s_mc_address[22], s_mc_address[23], // 11:8
    s_mc_address[ 3], s_mc_address[ 2], s_mc_address[21], s_mc_address[20], // 7:4
    s_mc_address[11], s_mc_address[10], s_mc_address[ 0], s_mc_address[ 1]  // 3:0
};
                              
assign DA_OUT = da_state[0] ? mc_data_write : da_address;
wire drive_da_out = is_bm && da_state[1];
assign DA_OE = {32{drive_da_out}};

// ## Access request slots.
// There is currently support for two request slots.

reg [31:0]  req_data_write [1:0];
reg [31:0]  req_data_read [1:0];
reg [23:0]  req_address [1:0];
reg [2:0]   req_fc [1:0];
reg [1:0]   req_size [1:0]; // 0->8b, 1->16b, 3->32b.
reg         req_rw [1:0]; // 0->Write, 1->Read.
reg [1:0]   req_active;
reg [1:0]   req_terminated_normally;

reg         current_pi_slot = 1'b0;
reg         current_execute_slot = 1'b0;
(* syn_preserve = 1 *) reg current_execute_slot_addr = 1'b0;

// ## Speculative 1-Longword Read-Ahead (Prefetch)
reg [23:0]  prefetch_addr = 24'd0;
reg [31:0]  prefetch_data = 32'd0;
reg         prefetch_valid = 1'b0;
reg         prefetch_eligible = 1'b0;
reg         is_prefetch_cycle = 1'b0;

reg [23:0]  next_prefetch_addr = 24'd0;
reg         next_prefetch_allowed = 1'b0;
reg [23:0]  chained_prefetch_addr = 24'd0;
reg         chained_prefetch_allowed = 1'b0;
reg [1:0]   req_prefetch_hit = 2'b00;
reg         can_prefetch = 1'b0;

// ## Virtual Zorro-II AutoConfig Device (64KB I/O)
localparam [15:0] Z2_MANUF_ID = 16'd28020; // 0x6D74
localparam [7:0]  Z2_PROD_ID  = 8'h32;     // PiStorm32 Product ID
localparam [31:0] Z2_SERIAL   = 32'd1;     // Serial Number

reg        z2_configured = 1'b0;
reg        z2_shutup = 1'b0;
reg [23:0] z2_base_addr = 24'd0;
reg [31:0] z2_scratchpad = 32'd0;
reg [1:0]  req_internal_intercept = 2'b00;

reg [23:0] internal_addr = 24'd0;
reg [31:0] internal_wr_data = 32'd0;
reg [1:0]  internal_size = 2'd0;
reg        internal_wr = 1'b0;
reg        internal_slot = 1'b0;
reg        internal_is_scratchpad = 1'b0;
reg        internal_is_io_regs = 1'b0;

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

wire [3:0] ac_nibble_0 = get_ac_nibble(internal_addr[6:1]);
wire [3:0] ac_nibble_1 = get_ac_nibble(internal_addr[6:1] + 6'd1);

wire [7:0]  ac_byte_val = internal_addr[0] ? 8'hFF : {ac_nibble_0, 4'h0};
wire [15:0] ac_word_val = internal_addr[0] ? {8'hFF, ac_nibble_1, 4'h0} : {{ac_nibble_0, 4'h0}, 8'hFF};
wire [31:0] ac_long_val = {{ac_nibble_0, 4'h0}, 8'hFF, {ac_nibble_1, 4'h0}, 8'hFF};

wire [31:0] ac_read_data = (internal_size == 2'd0) ? {24'd0, ac_byte_val} :
                           (internal_size == 2'd1) ? {16'd0, ac_word_val} :
                           ac_long_val;

reg [31:0] z2_reg_data;
always @(*) begin
    if (internal_is_io_regs) begin
        case (internal_addr[3:2])
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
    case (internal_addr[1:0])
        2'd0: io_read_byte = z2_reg_data[31:24];
        2'd1: io_read_byte = z2_reg_data[23:16];
        2'd2: io_read_byte = z2_reg_data[15:8];
        2'd3: io_read_byte = z2_reg_data[7:0];
    endcase
end

wire [15:0] io_read_word = internal_addr[1] ? z2_reg_data[15:0] : z2_reg_data[31:16];
wire [31:0] io_read_data = (internal_size == 2'd0) ? {24'd0, io_read_byte} :
                           (internal_size == 2'd1) ? {16'd0, io_read_word} :
                           z2_reg_data;

wire [31:0] internal_read_data = (!z2_configured) ? ac_read_data : io_read_data;

// Bus arbitration.
reg bg_sync;
reg as_sync;

always @(posedge clk) begin
    bg_sync <= !MC_BG_n;
    as_sync <= !MC_AS_n_IN;

    if (!request_bm) // There may be no bus cycle in progress when request_bm is negated.
        is_bm <= 1'b0;
    else if (bg_sync && !as_sync)
        is_bm <= 1'b1;
end

// Sample RESET, HALT.
always @(posedge clk) begin
    if (falling) begin
        reset_sync <= !MC_RESET_n_IN;
        halt_sync <= !MC_HALT_n_IN;
    end
end

// Synchronize IPL, and handle skew.
(* async_reg = "true" *) reg [2:0] ipl_sync [1:0];

always @(posedge clk) begin
    if (falling) begin
        ipl_sync[0] <= ~MC_IPL_n;
        ipl_sync[1] <= ipl_sync[0];

        if (ipl_sync[0] == ipl_sync[1])
            ipl <= ipl_sync[0];
    end
end

assign PI_TXN_IN_PROGRESS = req_active[current_pi_slot];
//assign PI_IPL_ZERO = ipl == 3'd0;
assign PI_IPL = ~ipl;

// State for current access.
reg [2:0]   fc;
reg [23:0]  address;
reg [1:0]   size;       // 0->8b, 1->16b, 3->32b.
reg         rw;         // 0->Write, 1->Read.
reg [31:0]  data_write;
reg [1:0]   port_width; // 0->8b, 1->16b, 3->32b.
reg [1:0]   left_shift;
reg [1:0]   transfered;
reg         size_le_transfered = 1'b0;

reg [7:0]   data_read_op0;
reg [7:0]   data_read_op1;
reg [7:0]   data_read_op2;
reg [7:0]   data_read_op3;
wire [31:0] data_read = {data_read_op0, data_read_op1, data_read_op2, data_read_op3};

wire [7:0]  op0_wr;
wire [7:0]  op1_wr;
wire [7:0]  op2_wr;
wire [7:0]  op3_wr;
assign {op0_wr, op1_wr, op2_wr, op3_wr} = data_write;

wire [7:0] op0_rd = mc_data_read[31:24];
wire [7:0] op1_rd = mc_data_read[23:16];
wire [7:0] op2_rd = mc_data_read[15:8];
wire [7:0] op3_rd = mc_data_read[7:0];

// ## Pi interface.
localparam [2:0] PI_REG_DATA_LO = 3'd0;
localparam [2:0] PI_REG_DATA_HI = 3'd1;
localparam [2:0] PI_REG_ADDR_LO = 3'd2;
localparam [2:0] PI_REG_ADDR_HI = 3'd3;
localparam [2:0] PI_REG_STATUS = 3'd4;
localparam [2:0] PI_REG_CONTROL = 3'd4;
localparam [2:0] PI_REG_SLOT = 3'd5;

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
        PI_REG_DATA_LO: pi_data_out <= q_req_data_read[15:0];//req_data_read[current_pi_slot][15:0];
        PI_REG_DATA_HI: pi_data_out <= q_req_data_read[31:16];//req_data_read[current_pi_slot][31:16];
        PI_REG_ADDR_LO: pi_data_out <= address[15:0];
        PI_REG_ADDR_HI: pi_data_out <= {8'd0, address[23:16]};
        PI_REG_STATUS: pi_data_out <= pi_status;
        default: pi_data_out <= 16'bx;
    endcase
end

// ## Access state machine (One-Hot Encoded for high fmax)
localparam STATE_BIT_WAIT_ACTIVE_REQUEST    = 0;
localparam STATE_BIT_WAIT_BUS_CYCLE_START   = 1;
localparam STATE_BIT_WAIT_ASSERT_AS         = 2;
localparam STATE_BIT_WAIT_OPEN_DATA_LATCH   = 3;
localparam STATE_BIT_WAIT_TERMINATION       = 4;
localparam STATE_BIT_S4_NOP                 = 5;
localparam STATE_BIT_WAIT_LATCH_DATA        = 6;
localparam STATE_BIT_UPDATE_DATA_READ       = 7;
localparam STATE_BIT_MAYBE_TERMINATE_ACCESS = 8;
localparam STATE_BIT_INTERNAL_FINISH        = 9;

localparam [9:0] STATE_WAIT_ACTIVE_REQUEST    = 10'd1 << STATE_BIT_WAIT_ACTIVE_REQUEST;
localparam [9:0] STATE_WAIT_BUS_CYCLE_START   = 10'd1 << STATE_BIT_WAIT_BUS_CYCLE_START;
localparam [9:0] STATE_WAIT_ASSERT_AS         = 10'd1 << STATE_BIT_WAIT_ASSERT_AS;
localparam [9:0] STATE_WAIT_OPEN_DATA_LATCH   = 10'd1 << STATE_BIT_WAIT_OPEN_DATA_LATCH;
localparam [9:0] STATE_WAIT_TERMINATION       = 10'd1 << STATE_BIT_WAIT_TERMINATION;
localparam [9:0] STATE_S4_NOP                 = 10'd1 << STATE_BIT_S4_NOP;
localparam [9:0] STATE_WAIT_LATCH_DATA        = 10'd1 << STATE_BIT_WAIT_LATCH_DATA;
localparam [9:0] STATE_UPDATE_DATA_READ       = 10'd1 << STATE_BIT_UPDATE_DATA_READ;
localparam [9:0] STATE_MAYBE_TERMINATE_ACCESS = 10'd1 << STATE_BIT_MAYBE_TERMINATE_ACCESS;
localparam [9:0] STATE_INTERNAL_FINISH        = 10'd1 << STATE_BIT_INTERNAL_FINISH;

reg [9:0] state = STATE_WAIT_ACTIVE_REQUEST;

(* async_reg = "true" *) reg [1:0] mc_dsack_n_sync;
reg mc_berr_n_sync;
reg mc_reset_n_sync;
reg [1:0] sync_mc_dsack_n_sync;

// Track and mask self-driven reset to Pi (Issue #4 / SUM1200 keyboard adapter compatibility)
reg mask_reset_for_pi = 1'b1;

always @(posedge clk) begin
    if (drive_reset)
        mask_reset_for_pi <= 1'b1;
    else if (mc_reset_n_sync)
        mask_reset_for_pi <= 1'b0;
end

// Combine active-low resets from keyboard (KBRESET) and 68k bus (mc_reset_n_sync)
// without reflecting the reset we assert ourselves.
assign PI_KBRESET = KBRESET & (mask_reset_for_pi ? 1'b1 : mc_reset_n_sync);

always @(posedge clk) begin
        
    if (falling) begin
        if (state[STATE_BIT_S4_NOP] || state[STATE_BIT_WAIT_LATCH_DATA])
            mc_data_read <= DA_IN;
        mc_dsack_n_sync <= MC_DSACK_n;
        mc_berr_n_sync <= MC_BERR_n;
        mc_reset_n_sync <= MC_RESET_n_IN;
    end
    
end

always @(*) begin
    // Table 5-1.
    case (mc_dsack_n_sync)
        2'b11: port_width <= 2'bx; // Unused.
        2'b10: port_width <= 2'd0;
        2'b01: port_width <= 2'd1;
        2'b00: port_width <= 2'd3;
    endcase
end

wire any_termination = |(~{mc_dsack_n_sync, mc_berr_n_sync, mc_reset_n_sync});
wire terminated_normally = mc_berr_n_sync && mc_reset_n_sync;

(* async_reg = "true" *) reg [1:0] pi_wr_sync;
reg [15:0] q_PI_D_IN;

always @(posedge clk) begin
    pi_wr_sync <= {pi_wr_sync[0], PI_WR};
    q_PI_D_IN <= PI_D_IN;
    
    if (pi_wr_sync == 2'b10) begin
        case (PI_A)
            PI_REG_DATA_LO: req_data_write[current_pi_slot][15:0] <= q_PI_D_IN;
            PI_REG_DATA_HI: req_data_write[current_pi_slot][31:16] <= q_PI_D_IN;
            PI_REG_ADDR_LO: req_address[current_pi_slot][15:0] <= q_PI_D_IN;
            PI_REG_ADDR_HI: begin
                req_address[current_pi_slot][23:16] <= q_PI_D_IN[7:0];
                req_size[current_pi_slot] <= q_PI_D_IN[9:8];
                req_rw[current_pi_slot] <= q_PI_D_IN[10];
                req_fc[current_pi_slot] <= q_PI_D_IN[13:11];
                req_active[current_pi_slot] <= 1'b1;
                req_prefetch_hit[current_pi_slot] <= enable_prefetch && prefetch_valid && q_PI_D_IN[10] &&
                    ({q_PI_D_IN[7:0], req_address[current_pi_slot][15:0]} == prefetch_addr) &&
                    (q_PI_D_IN[9:8] == 2'd3);
                req_internal_intercept[current_pi_slot] <= !z2_shutup && (
                    (!z2_configured && (q_PI_D_IN[7:0] == 8'hE8) && (req_address[current_pi_slot][15:7] == 9'd0)) ||
                    (z2_configured && (q_PI_D_IN[7:0] == z2_base_addr[23:16]))
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
                    current_execute_slot <= q_PI_D_IN[0];
                    current_execute_slot_addr <= q_PI_D_IN[0];
                end
            end
        endcase
    end

    can_prefetch <= enable_prefetch && is_bm && prefetch_eligible && !prefetch_valid && next_prefetch_allowed;

    (* parallel_case, full_case *) case (1'b1)
        state[STATE_BIT_WAIT_ACTIVE_REQUEST]: begin
            if (rising)
                da_state <= DA_STATE_IDLE;

            if (req_active[current_execute_slot]) begin
                if (req_internal_intercept[current_execute_slot]) begin
                    // INTERN BEDIENTES AUTO-CONFIG & VIRTUAL Z2 I/O DEVICE
                    prefetch_valid <= 1'b0;
                    prefetch_eligible <= 1'b0;
                    is_prefetch_cycle <= 1'b0;
                    req_prefetch_hit <= 2'b00;
                    internal_addr <= req_address[current_execute_slot_addr];
                    internal_wr_data <= req_data_write[current_execute_slot];
                    internal_size <= req_size[current_execute_slot];
                    internal_wr <= !req_rw[current_execute_slot];
                    internal_slot <= current_execute_slot;
                    internal_is_scratchpad <= (req_address[current_execute_slot_addr][15:2] == 14'h0003);
                    internal_is_io_regs <= (req_address[current_execute_slot_addr][15:4] == 12'd0);
                    state <= STATE_INTERNAL_FINISH;
                end else if (req_prefetch_hit[current_execute_slot]) begin
                    // PREFETCH HIT: Immediate 0-wait-state delivery from buffer
                    if (current_execute_slot == 1'b0) begin
                        req_data_read[0] <= prefetch_data;
                        req_terminated_normally[0] <= 1'b1;
                        req_active[0] <= 1'b0;
                        req_prefetch_hit[0] <= 1'b0;
                    end else begin
                        req_data_read[1] <= prefetch_data;
                        req_terminated_normally[1] <= 1'b1;
                        req_active[1] <= 1'b0;
                        req_prefetch_hit[1] <= 1'b0;
                    end
                    if (increment_execute_slot_pointer) begin
                        current_execute_slot <= current_execute_slot + 1'd1;
                        current_execute_slot_addr <= current_execute_slot_addr + 1'd1;
                    end
                    prefetch_valid <= 1'b0;
                    if (chained_prefetch_allowed && is_bm) begin
                        address <= chained_prefetch_addr;
                        fc <= 3'd1;
                        size <= 2'd3;
                        rw <= 1'b1;
                        is_prefetch_cycle <= 1'b1;
                        state <= STATE_WAIT_BUS_CYCLE_START;
                    end else begin
                        is_prefetch_cycle <= 1'b0;
                        prefetch_eligible <= 1'b0;
                        state <= STATE_WAIT_ACTIVE_REQUEST;
                    end
                end else begin
                    // Cache Miss or Write: Invalidate prefetch and run normal cycle
                    prefetch_valid <= 1'b0;
                    prefetch_eligible <= 1'b0;
                    req_prefetch_hit <= 2'b00;
                    is_prefetch_cycle <= 1'b0;
                    fc <= req_fc[current_execute_slot];
                    address <= req_address[current_execute_slot_addr];
                    size <= req_size[current_execute_slot];
                    rw <= req_rw[current_execute_slot];
                    data_write <= req_data_write[current_execute_slot];
                    state <= STATE_WAIT_BUS_CYCLE_START;
                end
            end else if (can_prefetch) begin
                // Start speculative read-ahead cycle while idle
                can_prefetch <= 1'b0;
                prefetch_eligible <= 1'b0;
                address <= next_prefetch_addr;
                fc <= 3'd1;
                size <= 2'd3;
                rw <= 1'b1;
                is_prefetch_cycle <= 1'b1;
                state <= STATE_WAIT_BUS_CYCLE_START;
            end
        end
        state[STATE_BIT_WAIT_BUS_CYCLE_START]: begin
            if (rising) begin // Entering S0
                da_state <= DA_STATE_IDLE;
                
                mc_fc <= fc;
                mc_address <= address;
                mc_rw <= rw;

                // Table 5-2.
                case (size)
                    2'd0: mc_size <= 2'b01;
                    2'd1: mc_size <= 2'b10;
                    2'd2: mc_size <= 2'b11;
                    2'd3: mc_size <= 2'b00;
                endcase

                // Table 5-5.
                case (size)
                    2'd0: begin
                        mc_data_write <= {op3_wr, op3_wr, op3_wr, op3_wr};
                    end
                    2'd1: begin
                        case (address[0])
                            1'b0: mc_data_write <= {op2_wr, op3_wr, op2_wr, op3_wr};
                            1'b1: mc_data_write <= {op2_wr, op2_wr, op3_wr, op2_wr};
                        endcase
                    end
                    2'd2: begin
                        case (address[1:0])
                            2'd0: mc_data_write <= {op1_wr, op2_wr, op3_wr, 8'bx};
                            2'd1: mc_data_write <= {op1_wr, op1_wr, op2_wr, op3_wr};
                            2'd2: mc_data_write <= {op1_wr, op2_wr, op1_wr, op2_wr};
                            2'd3: mc_data_write <= {op1_wr, op1_wr, 8'bx, op1_wr};
                        endcase
                    end
                    2'd3: begin
                        case (address[1:0])
                            2'd0: mc_data_write <= {op0_wr, op1_wr, op2_wr, op3_wr};
                            2'd1: mc_data_write <= {op0_wr, op0_wr, op1_wr, op2_wr};
                            2'd2: mc_data_write <= {op0_wr, op1_wr, op0_wr, op1_wr};
                            2'd3: mc_data_write <= {op0_wr, op0_wr, 8'bx, op0_wr};
                        endcase
                    end
                endcase

                state <= STATE_WAIT_ASSERT_AS;
            end
        end
        
        state[STATE_BIT_WAIT_ASSERT_AS]: begin
        da_state <= DA_STATE_FPGA_TO_ADDR;
        address_latch_le <= 1'b1;

            if (falling) begin // S0->S1
                address_latch_le <= 1'b0;
                mc_as <= 1'b1;
                if (rw)
                    mc_ds <= 1'b1;                   
                state <= STATE_WAIT_OPEN_DATA_LATCH;
            end
        end
        
        state[STATE_BIT_WAIT_OPEN_DATA_LATCH]: begin
            if (rising) begin // S1->S2
                if (rw)
                    da_state <= DA_STATE_DATA_TO_FPGA;
                else
                    da_state <= DA_STATE_FPGA_TO_DATA;

                state <= STATE_WAIT_TERMINATION;
            end
        end
        
        state[STATE_BIT_WAIT_TERMINATION]: begin
            if (falling) begin // S2->S3
                if (!rw)
                    mc_ds <= 1'b1;
            end
            if (any_termination)
                state <= STATE_S4_NOP;
        end
         
        state[STATE_BIT_S4_NOP]: begin
            if (falling) begin
                state <= STATE_WAIT_LATCH_DATA;
            end
        end
        
        state[STATE_BIT_WAIT_LATCH_DATA]: begin
            if (falling) begin // S4->S5
                mc_as <= 1'b0;
                mc_ds <= 1'b0;

                left_shift <= address[1:0] & port_width;
                transfered <= port_width - (address[1:0] & port_width);
                size_le_transfered <= (size <= (port_width - (address[1:0] & port_width)));

                if (rw)
                  state <= STATE_UPDATE_DATA_READ;
                else
                  state <= STATE_MAYBE_TERMINATE_ACCESS;
            end
        end
        
        state[STATE_BIT_UPDATE_DATA_READ]: begin
            // Table 5-4.
            case (size)
                2'd0:
                    case (left_shift)
                        2'd0: data_read_op3 <= op0_rd;
                        2'd1: data_read_op3 <= op1_rd;
                        2'd2: data_read_op3 <= op2_rd;
                        2'd3: data_read_op3 <= op3_rd;
                    endcase
                2'd1:
                    case (left_shift)
                        2'd0: data_read_op3 <= op1_rd;
                        2'd1: data_read_op3 <= op2_rd;
                        2'd2: data_read_op3 <= op3_rd;
                        default: data_read_op3 <= 8'bx;
                    endcase
                2'd2:
                    case (left_shift)
                        2'd0: data_read_op3 <= op2_rd;
                        2'd1: data_read_op3 <= op3_rd;
                        default: data_read_op3 <= 8'bx;
                    endcase
                2'd3:
                    case (left_shift)
                        2'd0: data_read_op3 <= op3_rd;
                        default: data_read_op3 <= 8'bx;
                    endcase
            endcase

            case (size)
                2'd1:
                    case (left_shift)
                        2'd0: data_read_op2 <= op0_rd;
                        2'd1: data_read_op2 <= op1_rd;
                        2'd2: data_read_op2 <= op2_rd;
                        2'd3: data_read_op2 <= op3_rd;
                    endcase
                2'd2:
                    case (left_shift)
                        2'd0: data_read_op2 <= op1_rd;
                        2'd1: data_read_op2 <= op2_rd;
                        2'd2: data_read_op2 <= op3_rd;
                        default: data_read_op2 <= 8'bx;
                    endcase
                2'd3:
                    case (left_shift)
                        2'd0: data_read_op2 <= op2_rd;
                        2'd1: data_read_op2 <= op3_rd;
                        default: data_read_op2 <= 8'bx;
                    endcase
            endcase

            case (size)
                2'd2:
                    case (left_shift)
                        2'd0: data_read_op1 <= op0_rd;
                        2'd1: data_read_op1 <= op1_rd;
                        2'd2: data_read_op1 <= op2_rd;
                        2'd3: data_read_op1 <= op3_rd;
                    endcase
                2'd3:
                    case (left_shift)
                        2'd0: data_read_op1 <= op1_rd;
                        2'd1: data_read_op1 <= op2_rd;
                        2'd2: data_read_op1 <= op3_rd;
                        default: data_read_op1 <= 8'bx;
                    endcase
            endcase

            case (size)
                2'd3:
                    case (left_shift)
                        2'd0: data_read_op0 <= op0_rd;
                        2'd1: data_read_op0 <= op1_rd;
                        2'd2: data_read_op0 <= op2_rd;
                        2'd3: data_read_op0 <= op3_rd;
                    endcase
            endcase
            state <= STATE_MAYBE_TERMINATE_ACCESS;
        end
        
        state[STATE_BIT_MAYBE_TERMINATE_ACCESS]: begin      
            if (is_prefetch_cycle) begin
                if (!terminated_normally || size_le_transfered) begin
                    chained_prefetch_addr <= address + 24'd4;
                    if (terminated_normally) begin
                        prefetch_data <= data_read;
                        prefetch_addr <= address;
                        prefetch_valid <= 1'b1;
                        chained_prefetch_allowed <= ((address[23:21] == 3'b000) || (&address[23:19]));
                        req_prefetch_hit[0] <= enable_prefetch && req_active[0] && req_rw[0] &&
                                               (req_address[0] == address) && (req_size[0] == 2'd3);
                        req_prefetch_hit[1] <= enable_prefetch && req_active[1] && req_rw[1] &&
                                               (req_address[1] == address) && (req_size[1] == 2'd3);
                    end else begin
                        prefetch_valid <= 1'b0;
                        req_prefetch_hit <= 2'b00;
                    end
                    is_prefetch_cycle <= 1'b0;
                    prefetch_eligible <= 1'b0;
                    state <= STATE_WAIT_ACTIVE_REQUEST;
                end else begin
                    // Perform another bus cycle if dynamic sizing occurred during prefetch
                    address <= address + {22'd0, transfered + 2'd1};
                    size <= size - (transfered + 2'd1);
                    state <= STATE_WAIT_BUS_CYCLE_START;
                end
            end else begin
                if (!terminated_normally || size_le_transfered) begin
                    if (current_execute_slot == 1'b0) begin
                        req_data_read[0] <= data_read;
                        req_terminated_normally[0] <= terminated_normally;
                        req_active[0] <= 1'b0;
                    end else begin
                        req_data_read[1] <= data_read;
                        req_terminated_normally[1] <= terminated_normally;
                        req_active[1] <= 1'b0;
                    end
                    next_prefetch_addr <= address + 24'd4;
                    if (increment_execute_slot_pointer) begin
                        current_execute_slot <= current_execute_slot + 1'd1;
                        current_execute_slot_addr <= current_execute_slot_addr + 1'd1;
                    end
                    if (terminated_normally && rw && (req_size[current_execute_slot] == 2'd3) &&
                        (req_address[current_execute_slot_addr][1:0] == 2'b00) &&
                        (port_width == 2'd3) &&
                        ((address[23:21] == 3'b000) || (&address[23:19]))) begin
                        prefetch_eligible <= 1'b1;
                        next_prefetch_allowed <= 1'b1;
                    end else begin
                        prefetch_eligible <= 1'b0;
                        prefetch_valid <= 1'b0;
                        next_prefetch_allowed <= 1'b0;
                        req_prefetch_hit <= 2'b00;
                    end
                    state <= STATE_WAIT_ACTIVE_REQUEST;
                end else begin
                    // Perform another bus cycle for this access.
                    address <= address + {22'd0, transfered + 2'd1};
                    size <= size - (transfered + 2'd1);
                    state <= STATE_WAIT_BUS_CYCLE_START;
                end
            end
        end

        state[STATE_BIT_INTERNAL_FINISH]: begin
            if (internal_slot == 1'b0) begin
                req_data_read[0] <= internal_read_data;
                req_terminated_normally[0] <= 1'b1;
                req_active[0] <= 1'b0;
            end else begin
                req_data_read[1] <= internal_read_data;
                req_terminated_normally[1] <= 1'b1;
                req_active[1] <= 1'b0;
            end
            if (increment_execute_slot_pointer) begin
                current_execute_slot <= current_execute_slot + 1'd1;
                current_execute_slot_addr <= current_execute_slot_addr + 1'd1;
            end

            // Handle internal writes
            if (internal_wr) begin
                if (!z2_configured) begin
                    if (internal_size == 2'd3 && internal_addr[6:1] == 6'h24) begin
                        // 32-bit long write to $48 sets both base nibbles
                        z2_base_addr[23:20] <= internal_wr_data[31:28] | internal_wr_data[27:24];
                        z2_base_addr[19:16] <= internal_wr_data[23:20] | internal_wr_data[19:16];
                        z2_base_addr[15:0]  <= 16'h0000;
                        z2_configured       <= 1'b1;
                    end else if (internal_size == 2'd1 && internal_addr[6:1] == 6'h24) begin
                        // 16-bit word write to $48 sets both base nibbles
                        z2_base_addr[23:20] <= internal_wr_data[15:12] | internal_wr_data[11:8];
                        z2_base_addr[19:16] <= internal_wr_data[7:4] | internal_wr_data[3:0];
                        z2_base_addr[15:0]  <= 16'h0000;
                        z2_configured       <= 1'b1;
                    end else if (internal_addr[6:1] == 6'h24) begin
                        // 8-bit byte write to $48 (Base High)
                        z2_base_addr[23:20] <= internal_wr_data[7:4] | internal_wr_data[3:0];
                    end else if (internal_addr[6:1] == 6'h25) begin
                        // 8-bit byte write to $4A (Base Low)
                        z2_base_addr[19:16] <= internal_wr_data[7:4] | internal_wr_data[3:0];
                        z2_base_addr[15:0]  <= 16'h0000;
                        z2_configured       <= 1'b1;
                    end else if (internal_addr[6:1] == 6'h26) begin
                        // Write to $4C (Shut-up)
                        z2_shutup <= 1'b1;
                    end
                end else begin
                    // Write to 64KB I/O space
                    if (internal_is_scratchpad) begin // Scratchpad at offset $0C
                        case (internal_size)
                            2'd0: begin // Byte write
                                case (internal_addr[1:0])
                                    2'd0: z2_scratchpad[31:24] <= internal_wr_data[7:0];
                                    2'd1: z2_scratchpad[23:16] <= internal_wr_data[7:0];
                                    2'd2: z2_scratchpad[15:8]  <= internal_wr_data[7:0];
                                    2'd3: z2_scratchpad[7:0]   <= internal_wr_data[7:0];
                                endcase
                            end
                            2'd1: begin // Word write
                                if (internal_addr[1])
                                    z2_scratchpad[15:0] <= internal_wr_data[15:0];
                                else
                                    z2_scratchpad[31:16] <= internal_wr_data[15:0];
                            end
                            default: begin // Long write
                                z2_scratchpad <= internal_wr_data;
                            end
                        endcase
                    end
                end
            end

            state <= STATE_WAIT_ACTIVE_REQUEST;
        end

        default: state <= STATE_WAIT_ACTIVE_REQUEST;
    endcase

    if (!request_bm || reset_sync || halt_sync || !is_bm) begin
        prefetch_valid <= 1'b0;
        prefetch_eligible <= 1'b0;
        can_prefetch <= 1'b0;
        next_prefetch_allowed <= 1'b0;
        chained_prefetch_allowed <= 1'b0;
        req_prefetch_hit <= 2'b00;
    end

    if (reset_sync || drive_reset) begin
        z2_configured <= 1'b0;
        z2_shutup <= 1'b0;
        z2_base_addr <= 24'd0;
        z2_scratchpad <= 32'd0;
        req_internal_intercept <= 2'b00;
        internal_addr <= 24'd0;
        internal_wr_data <= 32'd0;
        internal_size <= 2'd0;
        internal_wr <= 1'b0;
        internal_slot <= 1'b0;
        internal_is_scratchpad <= 1'b0;
        internal_is_io_regs <= 1'b0;
    end
end

endmodule
