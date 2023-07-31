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
    input AMIPLL_CLKOUT0
    //input AMIPLL_LOCK
    //input MC_CLK_CLEAN
);

assign SPARE_OUT[7:2] = 6'b111111;
assign SPARE_OE =  8'b11111111;

//EMU68
assign SPARE_OUT[0] = PI_SER_DAT;
assign SPARE_OUT[1] = PI_SER_CLK;
assign PI_KBRESET = KBRESET;

// ## Main clock.
wire clk;

// Connect the PLL.
assign clk = AMIPLL_CLKOUT0;

// Synchronize clk_rising/clk_falling with MC_CLK.
(* async_reg = "true" *) reg [12:0] mc_clk_sync;
reg clk_falling, clk_falling_plus_1,clk_falling_plus_2,clk_falling_plus_3,clk_rising,clk_rising_plus_1,clk_rising_plus_2,clk_rising_plus_3;

always @(posedge clk) begin
    mc_clk_sync <= {mc_clk_sync[11:0], MC_CLK};
end

always @(posedge clk) begin
    if ({mc_clk_sync[10], mc_clk_sync[9]} == 2'b01)
        clk_rising <= 1'b1;
    else
        clk_rising <= 1'b0;
    
   if ({mc_clk_sync[10], mc_clk_sync[9]} == 2'b10)
        clk_falling <= 1'b1;
    else
        clk_falling <= 1'b0;   
        
 clk_falling_plus_1 <= clk_falling;
 clk_falling_plus_2 <= clk_falling_plus_1;
 clk_falling_plus_3 <= clk_falling_plus_2;
   
 clk_rising_plus_1 <= clk_rising;
 clk_rising_plus_2 <= clk_rising_plus_1;
 clk_rising_plus_3 <= clk_rising_plus_2;
 
end

// Pi control register.
reg [14:0] pi_control = 15'b000000000000110;
wire request_bm = pi_control[0];
wire drive_reset = pi_control[1];
wire drive_halt = pi_control[2];
wire drive_int2 = pi_control[3];
wire drive_int6 = pi_control[4];
wire increment_execute_slot_pointer = pi_control[5];

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

reg         sync_mc_as;
reg         sync_mc_ds;
reg [2:0]   sync_mc_fc;
reg [1:0]   sync_mc_size;
reg         sync_mc_rw;

always @(negedge MC_CLK) begin
 sync_mc_as <= mc_as;
 sync_mc_ds <= mc_ds;
end

always @(posedge MC_CLK) begin
 sync_mc_fc <= mc_fc;
 sync_mc_size <= mc_size;
 sync_mc_rw <= mc_rw;
end

assign MC_FC_OUT = sync_mc_fc;
assign MC_FC_OE = {3{is_bm}};
assign MC_SIZE_OUT = sync_mc_size;
assign MC_SIZE_OE = {2{is_bm}};
assign MC_RW_OUT = sync_mc_rw;
assign MC_RW_OE = is_bm;
assign MC_AS_n_OUT = !sync_mc_as;
assign MC_AS_n_OE = is_bm;
assign MC_DS_n_OUT = !sync_mc_ds;
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
    if (clk_falling) begin
        reset_sync <= !MC_RESET_n_IN;
        halt_sync <= !MC_HALT_n_IN;
    end
end

// Synchronize IPL, and handle skew.
(* async_reg = "true" *) reg [2:0] ipl_sync [1:0];

always @(posedge clk) begin
    if (clk_falling) begin
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

always @(*) begin
    case (PI_A)
        PI_REG_DATA_LO: pi_data_out <= req_data_read[current_pi_slot][15:0];
        PI_REG_DATA_HI: pi_data_out <= req_data_read[current_pi_slot][31:16];
        PI_REG_ADDR_LO: pi_data_out <= address[15:0];
        PI_REG_ADDR_HI: pi_data_out <= {8'd0, address[23:16]};
        PI_REG_STATUS: pi_data_out <= pi_status;
        default: pi_data_out <= 16'bx;
    endcase
end

// ## Access state machine.
localparam [2:0] STATE_WAIT_ACTIVE_REQUEST = 3'd0;
localparam [2:0] STATE_WAIT_BUS_CYCLE_START = 3'd1;
localparam [2:0] STATE_WAIT_ASSERT_AS = 3'd2;
localparam [2:0] STATE_WAIT_OPEN_DATA_LATCH = 3'd3;
localparam [2:0] STATE_WAIT_TERMINATION = 3'd4;
localparam [2:0] STATE_WAIT_LATCH_DATA = 3'd5;
localparam [2:0] STATE_UPDATE_DATA_READ = 3'd6;
localparam [2:0] STATE_MAYBE_TERMINATE_ACCESS = 3'd7;

reg [2:0] state;

(* async_reg = "true" *) reg [1:0] mc_dsack_n_sync;
reg mc_berr_n_sync;
reg mc_reset_n_sync;

always @(posedge clk) begin
    if (clk_falling) begin
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

always @(posedge clk) begin
    pi_wr_sync <= {pi_wr_sync[0], PI_WR};

    if (pi_wr_sync == 2'b10) begin
        case (PI_A)
            PI_REG_DATA_LO: req_data_write[current_pi_slot][15:0] <= PI_D_IN;
            PI_REG_DATA_HI: req_data_write[current_pi_slot][31:16] <= PI_D_IN;
            PI_REG_ADDR_LO: req_address[current_pi_slot][15:0] <= PI_D_IN;
            PI_REG_ADDR_HI: begin
                req_address[current_pi_slot][23:16] <= PI_D_IN[7:0];
                req_size[current_pi_slot] <= PI_D_IN[9:8];
                req_rw[current_pi_slot] <= PI_D_IN[10];
                req_fc[current_pi_slot] <= PI_D_IN[13:11];
                req_active[current_pi_slot] <= 1'b1;
            end
            PI_REG_CONTROL: begin
                if (PI_D_IN[15])
                    pi_control <= pi_control | PI_D_IN[14:0];
                else
                    pi_control <= pi_control & ~PI_D_IN[14:0];
            end
            PI_REG_SLOT: current_pi_slot <= PI_D_IN[0];
        endcase
    end

    case (state)
        STATE_WAIT_ACTIVE_REQUEST: begin
            if (clk_rising)
                da_state <= DA_STATE_IDLE;

            if (req_active[current_execute_slot]) begin
                fc <= req_fc[current_execute_slot];
                address <= req_address[current_execute_slot];
                size <= req_size[current_execute_slot];
                rw <= req_rw[current_execute_slot];
                data_write <= req_data_write[current_execute_slot];
                state <= STATE_WAIT_BUS_CYCLE_START;
            end
        end
        STATE_WAIT_BUS_CYCLE_START: begin
            if (clk_rising) begin // Entering S0
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
        STATE_WAIT_ASSERT_AS: begin
            if (clk_rising_plus_1) begin
                da_state <= DA_STATE_FPGA_TO_ADDR;
                address_latch_le <= 1'b1;
            end

            if (clk_rising_plus_3)
                address_latch_le <= 1'b0;

            if (clk_falling) begin // S0->S1
                mc_as <= 1'b1;
                if (rw)
                    mc_ds <= 1'b1;

                da_state <= DA_STATE_IDLE;
                state <= STATE_WAIT_OPEN_DATA_LATCH;
            end
        end
        STATE_WAIT_OPEN_DATA_LATCH: begin
            if (clk_rising) begin // S1->S2
                if (rw)
                    da_state <= DA_STATE_DATA_TO_FPGA;
                else
                    da_state <= DA_STATE_FPGA_TO_DATA;

                state <= STATE_WAIT_TERMINATION;
            end
        end
        STATE_WAIT_TERMINATION: begin
            if (clk_falling) begin // S2->S3
                if (!rw)
                    mc_ds <= 1'b1;
            end

            if (!rw)
                da_state <= DA_STATE_FPGA_TO_DATA;

            if (clk_falling_plus_1 && any_termination)
            //if (any_termination)
                state <= STATE_WAIT_LATCH_DATA;
        end
        STATE_WAIT_LATCH_DATA: begin
            if (clk_falling) begin // S4->S5
                mc_as <= 1'b0;
                mc_ds <= 1'b0;

                left_shift <= address[1:0] & port_width;
                transfered <= port_width - (address[1:0] & port_width);

                if (!rw)
                    da_state <= DA_STATE_FPGA_TO_DATA;

                if (rw)
                  state <= STATE_UPDATE_DATA_READ;
                else
                  state <= STATE_MAYBE_TERMINATE_ACCESS;
            end
        end
        STATE_UPDATE_DATA_READ: begin
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
        STATE_MAYBE_TERMINATE_ACCESS: begin
            if (!rw)
                da_state <= DA_STATE_FPGA_TO_DATA;

            if (!terminated_normally || terminated_normally && size <= transfered) begin
                req_data_read[current_execute_slot] <= data_read;
                req_terminated_normally[current_execute_slot] <= terminated_normally;
                req_active[current_execute_slot] <= 1'b0;
                if (increment_execute_slot_pointer)
                    current_execute_slot <= current_execute_slot + 1'd1;
                state <= STATE_WAIT_ACTIVE_REQUEST;
            end else begin
                // Perform another bus cycle for this access.
                address <= address + {22'd0, transfered + 2'd1};
                size <= size - (transfered + 2'd1);
                state <= STATE_WAIT_BUS_CYCLE_START;
            end
        end
    endcase
end

endmodule
