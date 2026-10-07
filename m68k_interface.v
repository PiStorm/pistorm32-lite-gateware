/*
 * Copyright 2022 Niklas Ekström
 * Copyright 2022 Claude Schwarz
 *
 * MC68020 Bus Master Interface, FSM, Arbitration & Prefetch Engine
 */

module m68k_interface (
    input               clk,

    // MC68EC020 signals
    output [2:0]        MC_FC_OUT,
    output [2:0]        MC_FC_OE,
    output [1:0]        MC_SIZE_OUT,
    output [1:0]        MC_SIZE_OE,
    output              MC_RW_OUT,
    output              MC_RW_OE,
    input               MC_AS_n_IN,
    output              MC_AS_n_OUT,
    output              MC_AS_n_OE,
    output              MC_DS_n_OUT,
    output              MC_DS_n_OE,
    input [1:0]         MC_DSACK_n,
    input [2:0]         MC_IPL_n,
    output              MC_BR_n_OUT,
    output              MC_BR_n_OE,
    input               MC_BG_n,
    input               MC_RESET_n_IN,
    output              MC_RESET_n_OUT,
    output              MC_RESET_n_OE,
    input               MC_HALT_n_IN,
    output              MC_HALT_n_OUT,
    output              MC_HALT_n_OE,
    input               MC_BERR_n,
    input               MC_CLK,

    // Amiga 1200 Interrupt lines
    output              INT2_n_OUT,
    output              INT2_n_OE,
    output              INT6_n_OUT,
    output              INT6_n_OE,

    // Shared data and address bus multiplexing
    input [31:0]        DA_IN,
    output [31:0]       DA_OUT,
    output [31:0]       DA_OE,
    output              ADDR_LE,
    output              ADDR_OE_n,
    output              DATA_OE_n,
    output              CTRL_OE_n,

    // Control inputs from pi_interface
    input               request_bm,
    input               drive_reset,
    input               drive_halt,
    input               drive_int2,
    input               drive_int6,
    input               increment_execute_slot_pointer,
    input               enable_prefetch,

    // Status outputs to pi_interface
    output reg          is_bm = 1'b0,
    output reg          reset_sync = 1'b0,
    output reg          halt_sync = 1'b0,
    output reg [2:0]    ipl = 3'b111,
    output reg          mc_reset_n_sync = 1'b1,
    output [23:0]       current_bus_address,

    // Slot request inputs from pi_interface
    input [1:0]         req_active,
    input [1:0]         req_internal_intercept,
    input [23:0]        req_address_0,
    input [23:0]        req_address_1,
    input [31:0]        req_data_write_0,
    input [31:0]        req_data_write_1,
    input [1:0]         req_size_0,
    input [1:0]         req_size_1,
    input               req_rw_0,
    input               req_rw_1,
    input [2:0]         req_fc_0,
    input [2:0]         req_fc_1,
    input               new_req_valid,
    input               new_req_slot,
    input [23:0]        new_req_addr,
    input               new_req_rw,
    input [1:0]         new_req_size,

    // Slot pointer override from pi_interface
    input               set_execute_slot_valid,
    input               set_execute_slot_val,

    // Slot completion outputs to pi_interface (combinational wires for instant completion)
    output              slot_complete_valid,
    output              slot_complete_id,
    output [31:0]       slot_complete_data,
    output              slot_complete_normally,

    // Virtual Zorro device interface
    output reg          z2_access_strobe = 1'b0,
    output reg          z2_access_wr = 1'b0,
    output reg [1:0]    z2_access_size = 2'd0,
    output reg [23:0]   z2_access_addr = 24'd0,
    output reg [31:0]   z2_access_wr_data = 32'd0,
    output reg          z2_access_is_scratchpad = 1'b0,
    output reg          z2_access_is_io_regs = 1'b0,
    input [31:0]        z2_rd_data
);

// Bus control outputs
assign MC_BR_n_OUT    = 1'b0;
assign MC_BR_n_OE     = request_bm;
assign MC_RESET_n_OUT = 1'b0;
assign MC_RESET_n_OE  = drive_reset;
assign MC_HALT_n_OUT  = 1'b0;
assign MC_HALT_n_OE   = drive_halt;
assign INT2_n_OUT     = 1'b0;
assign INT2_n_OE      = drive_int2;
assign INT6_n_OUT     = 1'b0;
assign INT6_n_OE      = drive_int6;

// Bus arbitration
reg bg_sync = 1'b0;
reg as_sync = 1'b0;

always @(posedge clk) begin
    bg_sync <= !MC_BG_n;
    as_sync <= !MC_AS_n_IN;

    if (!request_bm)
        is_bm <= 1'b0;
    else if (bg_sync && !as_sync)
        is_bm <= 1'b1;
end

// Universal Amiga 1200 Clock Filter & Glitch Synchronizer
localparam [2:0] MC_CLK_LOCKOUT_TICKS = 3'd3;

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

// Synchronize inputs on falling edge
(* async_reg = "true" *) reg [2:0] ipl_sync [1:0];
(* async_reg = "true" *) reg [1:0] mc_dsack_n_sync;
reg mc_berr_n_sync;
reg [31:0] mc_data_read;

always @(posedge clk) begin
    if (falling) begin
        reset_sync      <= !MC_RESET_n_IN;
        halt_sync       <= !MC_HALT_n_IN;
        mc_reset_n_sync <= MC_RESET_n_IN;

        ipl_sync[0] <= ~MC_IPL_n;
        ipl_sync[1] <= ipl_sync[0];
        if (ipl_sync[0] == ipl_sync[1])
            ipl <= ipl_sync[0];

        if (state[STATE_BIT_S4_NOP] || state[STATE_BIT_WAIT_LATCH_DATA])
            mc_data_read <= DA_IN;

        mc_dsack_n_sync <= MC_DSACK_n;
        mc_berr_n_sync  <= MC_BERR_n;
    end
end

// Port width decoding
reg [1:0] port_width;
always @(*) begin
    case (mc_dsack_n_sync)
        2'b11: port_width = 2'bx;
        2'b10: port_width = 2'd0;
        2'b01: port_width = 2'd1;
        2'b00: port_width = 2'd3;
    endcase
end

wire any_termination     = |(~{mc_dsack_n_sync, mc_berr_n_sync, mc_reset_n_sync});
wire terminated_normally = mc_berr_n_sync && mc_reset_n_sync;

// Execution slot pointers
reg         current_execute_slot = 1'b0;
(* syn_preserve = 1 *) reg current_execute_slot_addr = 1'b0;
reg         internal_slot = 1'b0;

// Prefetch engine registers
reg [23:0]  prefetch_addr = 24'd0;
reg [31:0]  prefetch_data = 32'd0;
reg         prefetch_valid = 1'b0;
reg         prefetch_eligible = 1'b0;
reg         is_prefetch_cycle = 1'b0;
reg [23:0]  next_prefetch_addr = 24'd0;
reg         next_prefetch_allowed = 1'b0;
reg [23:0]  chained_prefetch_addr = 24'd0;
reg         chained_prefetch_allowed = 1'b0;
reg         can_prefetch = 1'b0;
reg [1:0]   req_prefetch_hit = 2'b00;

// 68020 bus registers
reg [2:0]   mc_fc = 3'd0;
reg [23:0]  mc_address = 24'd0;
reg [1:0]   mc_size = 2'd0;
reg         mc_rw = 1'b1;
reg [31:0]  mc_data_write = 32'd0;
reg         mc_as = 1'b0;
reg         mc_ds = 1'b0;

assign MC_FC_OUT   = mc_fc;
assign MC_FC_OE    = {3{is_bm}};
assign MC_SIZE_OUT = mc_size;
assign MC_SIZE_OE  = {2{is_bm}};
assign MC_RW_OUT   = mc_rw;
assign MC_RW_OE    = is_bm;
assign MC_AS_n_OUT = !mc_as;
assign MC_AS_n_OE  = is_bm;
assign MC_DS_n_OUT = !mc_ds;
assign MC_DS_n_OE  = is_bm;

// Multiplexed bus control
localparam [1:0] DA_STATE_IDLE         = 2'd0;
localparam [1:0] DA_STATE_DATA_TO_FPGA = 2'd1;
localparam [1:0] DA_STATE_FPGA_TO_ADDR = 2'd2;
localparam [1:0] DA_STATE_FPGA_TO_DATA = 2'd3;

reg [1:0] da_state = DA_STATE_IDLE;
reg address_latch_le = 1'b0;

assign ADDR_LE   = address_latch_le;
assign ADDR_OE_n = !is_bm;
assign CTRL_OE_n = 1'b0;
assign DATA_OE_n = !(da_state == DA_STATE_DATA_TO_FPGA || da_state == DA_STATE_FPGA_TO_DATA);

// Address scrambling for PCB routing
wire [31:0] s_mc_address = {8'd0, mc_address};
wire [31:0] da_address = {
    s_mc_address[31], s_mc_address[30], s_mc_address[12], s_mc_address[13],
    s_mc_address[ 7], s_mc_address[ 6], s_mc_address[15], s_mc_address[14],
    s_mc_address[29], s_mc_address[28], s_mc_address[26], s_mc_address[27],
    s_mc_address[16], s_mc_address[17], s_mc_address[ 5], s_mc_address[ 4],
    s_mc_address[19], s_mc_address[18], s_mc_address[25], s_mc_address[24],
    s_mc_address[ 8], s_mc_address[ 9], s_mc_address[22], s_mc_address[23],
    s_mc_address[ 3], s_mc_address[ 2], s_mc_address[21], s_mc_address[20],
    s_mc_address[11], s_mc_address[10], s_mc_address[ 0], s_mc_address[ 1]
};

assign DA_OUT = da_state[0] ? mc_data_write : da_address;
wire drive_da_out = is_bm && da_state[1];
assign DA_OE = {32{drive_da_out}};

// Bus cycle state registers
reg [2:0]   fc = 3'd0;
reg [23:0]  address = 24'd0;
reg [1:0]   size = 2'd0;
reg         rw = 1'b1;
reg [31:0]  data_write = 32'd0;
reg [1:0]   left_shift = 2'd0;
reg [1:0]   transfered = 2'd0;
reg         size_le_transfered = 1'b0;

assign current_bus_address = address;

reg [7:0] data_read_op0;
reg [7:0] data_read_op1;
reg [7:0] data_read_op2;
reg [7:0] data_read_op3;
wire [31:0] data_read = {data_read_op0, data_read_op1, data_read_op2, data_read_op3};

wire [7:0] op0_wr = data_write[31:24];
wire [7:0] op1_wr = data_write[23:16];
wire [7:0] op2_wr = data_write[15:8];
wire [7:0] op3_wr = data_write[7:0];

wire [7:0] op0_rd = mc_data_read[31:24];
wire [7:0] op1_rd = mc_data_read[23:16];
wire [7:0] op2_rd = mc_data_read[15:8];
wire [7:0] op3_rd = mc_data_read[7:0];

// Request multiplexers
wire [23:0] cur_req_addr = current_execute_slot_addr ? req_address_1 : req_address_0;
wire [31:0] cur_req_data = current_execute_slot ? req_data_write_1 : req_data_write_0;
wire [1:0]  cur_req_size = current_execute_slot ? req_size_1 : req_size_0;
wire        cur_req_rw   = current_execute_slot ? req_rw_1 : req_rw_0;
wire [2:0]  cur_req_fc   = current_execute_slot ? req_fc_1 : req_fc_0;
wire        cur_req_act  = current_execute_slot ? req_active[1] : req_active[0];
wire        cur_req_int  = current_execute_slot ? req_internal_intercept[1] : req_internal_intercept[0];
wire        cur_req_hit  = current_execute_slot ? req_prefetch_hit[1] : req_prefetch_hit[0];

// One-Hot State Machine
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

// Combinational slot completion logic (terminates in the EXACT same cycle, preventing re-execution)
wire normal_cycle_terminate    = state[STATE_BIT_MAYBE_TERMINATE_ACCESS] && (!is_prefetch_cycle) && (!terminated_normally || size_le_transfered);
wire internal_finish_terminate = state[STATE_BIT_INTERNAL_FINISH];
wire prefetch_hit_terminate    = state[STATE_BIT_WAIT_ACTIVE_REQUEST] && cur_req_act && (!cur_req_int) && cur_req_hit;

assign slot_complete_valid     = normal_cycle_terminate || internal_finish_terminate || prefetch_hit_terminate;
assign slot_complete_id        = internal_finish_terminate ? internal_slot : current_execute_slot;
assign slot_complete_data      = internal_finish_terminate ? z2_rd_data :
                                 prefetch_hit_terminate    ? prefetch_data :
                                 data_read;
assign slot_complete_normally  = normal_cycle_terminate ? terminated_normally : 1'b1;

wire prefetch_eligible_term = terminated_normally && rw && (cur_req_size == 2'd3) &&
                             (cur_req_addr[1:0] == 2'b00) && (port_width == 2'd3) &&
                             ((address[23:21] == 3'b000) || (&address[23:19]));


always @(posedge clk) begin
    z2_access_strobe <= 1'b0;

    can_prefetch <= enable_prefetch && is_bm && prefetch_eligible && !prefetch_valid && next_prefetch_allowed;

    // Slot pointer override from PI_REG_SLOT
    if (set_execute_slot_valid) begin
        current_execute_slot      <= set_execute_slot_val;
        current_execute_slot_addr <= set_execute_slot_val;
    end

    (* parallel_case, full_case *) case (1'b1)
        state[STATE_BIT_WAIT_ACTIVE_REQUEST]: begin
            if (rising)
                da_state <= DA_STATE_IDLE;

            if (cur_req_act) begin
                if (cur_req_int) begin
                    // Internally serviced Zorro AutoConfig & 64KB I/O
                    prefetch_valid    <= 1'b0;
                    prefetch_eligible <= 1'b0;
                    is_prefetch_cycle <= 1'b0;
                    req_prefetch_hit  <= 2'b00;

                    z2_access_addr          <= cur_req_addr;
                    z2_access_wr_data       <= cur_req_data;
                    z2_access_size          <= cur_req_size;
                    z2_access_wr            <= !cur_req_rw;
                    internal_slot           <= current_execute_slot;
                    z2_access_is_scratchpad <= (cur_req_addr[15:2] == 14'h0003);
                    z2_access_is_io_regs    <= (cur_req_addr[15:4] == 12'd0);

                    state <= STATE_INTERNAL_FINISH;
                end else if (cur_req_hit) begin
                    // Prefetch Hit: immediate 0-wait-state delivery
                    req_prefetch_hit[current_execute_slot] <= 1'b0;
                    if (increment_execute_slot_pointer) begin
                        current_execute_slot      <= current_execute_slot + 1'd1;
                        current_execute_slot_addr <= current_execute_slot_addr + 1'd1;
                    end
                    prefetch_valid <= 1'b0;

                    if (chained_prefetch_allowed && is_bm) begin
                        address           <= chained_prefetch_addr;
                        fc                <= 3'd1;
                        size              <= 2'd3;
                        rw                <= 1'b1;
                        is_prefetch_cycle <= 1'b1;
                        state             <= STATE_WAIT_BUS_CYCLE_START;
                    end else begin
                        is_prefetch_cycle <= 1'b0;
                        prefetch_eligible <= 1'b0;
                        state             <= STATE_WAIT_ACTIVE_REQUEST;
                    end
                end else begin
                    // Cache Miss or Write: run normal Amiga bus cycle
                    prefetch_valid    <= 1'b0;
                    prefetch_eligible <= 1'b0;
                    is_prefetch_cycle <= 1'b0;
                    req_prefetch_hit  <= 2'b00;

                    fc         <= cur_req_fc;
                    address    <= cur_req_addr;
                    size       <= cur_req_size;
                    rw         <= cur_req_rw;
                    data_write <= cur_req_data;
                    state      <= STATE_WAIT_BUS_CYCLE_START;
                end
            end else if (can_prefetch) begin
                // Speculative read-ahead cycle while bus is idle
                can_prefetch      <= 1'b0;
                prefetch_eligible <= 1'b0;
                address           <= next_prefetch_addr;
                fc                <= 3'd1;
                size              <= 2'd3;
                rw                <= 1'b1;
                is_prefetch_cycle <= 1'b1;
                state             <= STATE_WAIT_BUS_CYCLE_START;
            end
        end

        state[STATE_BIT_WAIT_BUS_CYCLE_START]: begin
            if (rising) begin // S0
                da_state <= DA_STATE_IDLE;
                mc_fc    <= fc;
                mc_address <= address;
                mc_rw    <= rw;

                case (size)
                    2'd0: mc_size <= 2'b01;
                    2'd1: mc_size <= 2'b10;
                    2'd2: mc_size <= 2'b11;
                    2'd3: mc_size <= 2'b00;
                endcase

                case (size)
                    2'd0: mc_data_write <= {op3_wr, op3_wr, op3_wr, op3_wr};
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
            da_state         <= DA_STATE_FPGA_TO_ADDR;
            address_latch_le <= 1'b1;

            if (falling) begin // S0->S1
                address_latch_le <= 1'b0;
                mc_as            <= 1'b1;
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
            if (falling)
                state <= STATE_WAIT_LATCH_DATA;
        end

        state[STATE_BIT_WAIT_LATCH_DATA]: begin
            if (falling) begin // S4->S5
                mc_as <= 1'b0;
                mc_ds <= 1'b0;

                left_shift         <= address[1:0] & port_width;
                transfered         <= port_width - (address[1:0] & port_width);
                size_le_transfered <= (size <= (port_width - (address[1:0] & port_width)));

                if (rw)
                    state <= STATE_UPDATE_DATA_READ;
                else
                    state <= STATE_MAYBE_TERMINATE_ACCESS;
            end
        end

        state[STATE_BIT_UPDATE_DATA_READ]: begin
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
                        prefetch_data            <= data_read;
                        prefetch_addr            <= address;
                        prefetch_valid           <= 1'b1;
                        chained_prefetch_allowed <= ((address[23:21] == 3'b000) || (&address[23:19]));
                        req_prefetch_hit[0]      <= enable_prefetch && req_active[0] && req_rw_0 &&
                                                   (req_address_0 == address) && (req_size_0 == 2'd3);
                        req_prefetch_hit[1]      <= enable_prefetch && req_active[1] && req_rw_1 &&
                                                   (req_address_1 == address) && (req_size_1 == 2'd3);
                    end else begin
                        prefetch_valid   <= 1'b0;
                        req_prefetch_hit <= 2'b00;
                    end
                    is_prefetch_cycle <= 1'b0;
                    prefetch_eligible <= 1'b0;
                    state             <= STATE_WAIT_ACTIVE_REQUEST;
                end else begin
                    // Perform another bus cycle if dynamic sizing occurred during prefetch
                    address <= address + {22'd0, transfered + 2'd1};
                    size    <= size - (transfered + 2'd1);
                    state   <= STATE_WAIT_BUS_CYCLE_START;
                end
            end else begin
                if (!terminated_normally || size_le_transfered) begin
                    next_prefetch_addr <= address + 24'd4;
                    if (increment_execute_slot_pointer) begin
                        current_execute_slot      <= current_execute_slot + 1'd1;
                        current_execute_slot_addr <= current_execute_slot_addr + 1'd1;
                    end

                    if (prefetch_eligible_term) begin
                        prefetch_eligible     <= 1'b1;
                        next_prefetch_allowed <= 1'b1;
                    end else begin
                        prefetch_eligible     <= 1'b0;
                        prefetch_valid        <= 1'b0;
                        next_prefetch_allowed <= 1'b0;
                        req_prefetch_hit      <= 2'b00;
                    end
                    state <= STATE_WAIT_ACTIVE_REQUEST;
                end else begin
                    // Perform another bus cycle for this access
                    address <= address + {22'd0, transfered + 2'd1};
                    size    <= size - (transfered + 2'd1);
                    state   <= STATE_WAIT_BUS_CYCLE_START;
                end
            end
        end

        state[STATE_BIT_INTERNAL_FINISH]: begin
            z2_access_strobe <= 1'b1;

            if (increment_execute_slot_pointer) begin
                current_execute_slot      <= current_execute_slot + 1'd1;
                current_execute_slot_addr <= current_execute_slot_addr + 1'd1;
            end

            state <= STATE_WAIT_ACTIVE_REQUEST;
        end

        default: state <= STATE_WAIT_ACTIVE_REQUEST;
    endcase

    if (new_req_valid) begin
        req_prefetch_hit[new_req_slot] <= enable_prefetch && prefetch_valid && new_req_rw &&
            (new_req_addr == prefetch_addr) && (new_req_size == 2'd3);
    end

    if (!request_bm || reset_sync || halt_sync || !is_bm) begin
        prefetch_valid           <= 1'b0;
        prefetch_eligible        <= 1'b0;
        can_prefetch             <= 1'b0;
        next_prefetch_allowed    <= 1'b0;
        chained_prefetch_allowed <= 1'b0;
        req_prefetch_hit         <= 2'b00;
    end

    if (reset_sync || drive_reset) begin
        z2_access_addr          <= 24'd0;
        z2_access_wr_data       <= 32'd0;
        z2_access_size          <= 2'd0;
        z2_access_wr            <= 1'b0;
        internal_slot           <= 1'b0;
        z2_access_is_scratchpad <= 1'b0;
        z2_access_is_io_regs    <= 1'b0;
    end
end

endmodule
