/*
 * PiStorm32-lite Gateware - Amiga 1200 M68k Bus Interface
 *
 * Copyright 2022 Niklas Ekström
 * Copyright 2022-2026 Claude Schwarz
 */

// =============================================================================
// Feature Compilation Flags
// =============================================================================
`define ENABLE_16BIT_PREFETCH

module m68k_interface (
    input  wire        clk,

    // -------------------------------------------------------------------------
    // Motorola MC68EC020 Physical Bus Signals
    // -------------------------------------------------------------------------
    output wire [2:0]  MC_FC_OUT,          // Function Codes: FC2 (User/Super), FC1, FC0 (Prog/Data)
    output wire [2:0]  MC_FC_OE,           // Function Code output enables
    output wire [1:0]  MC_SIZE_OUT,        // Transfer Size: 00=Long, 01=Byte, 10=Word, 11=3 Bytes
    output wire [1:0]  MC_SIZE_OE,         // Transfer Size output enables
    output wire        MC_RW_OUT,          // Bus Direction: 1 = Read, 0 = Write
    output wire        MC_RW_OE,           // Bus Direction output enable
    input  wire        MC_AS_n_IN,         // Address Strobe input from motherboard (when slave)
    output wire        MC_AS_n_OUT,        // Address Strobe output to motherboard (when master)
    output wire        MC_AS_n_OE,         // Address Strobe output enable
    output wire        MC_DS_n_OUT,        // Data Strobe output to motherboard
    output wire        MC_DS_n_OE,         // Data Strobe output enable
    input  wire [1:0]  MC_DSACK_n,         // Data and Size Acknowledge inputs (/DSACK1, /DSACK0)
    input  wire [2:0]  MC_IPL_n,           // Interrupt Priority Level inputs (/IPL2, /IPL1, /IPL0)
    output wire        MC_BR_n_OUT,        // Bus Request output (requests bus from Gary/Gayle)
    output wire        MC_BR_n_OE,         // Bus Request output enable
    input  wire        MC_BG_n,            // Bus Grant input (bus ownership granted by Gary/Gayle)
    input  wire        MC_RESET_n_IN,      // System Reset input
    output wire        MC_RESET_n_OUT,     // System Reset output
    output wire        MC_RESET_n_OE,      // System Reset output enable
    input  wire        MC_HALT_n_IN,       // System Halt input
    output wire        MC_HALT_n_OUT,      // System Halt output
    output wire        MC_HALT_n_OE,       // System Halt output enable
    input  wire        MC_BERR_n,          // Bus Error input (from motherboard or timeout)
    input  wire        MC_CLK,             // Amiga 14.18 MHz motherboard clock (E1)

    // -------------------------------------------------------------------------
    // Amiga 1200 Interrupt Outputs (to Paula & CIA-B)
    // -------------------------------------------------------------------------
    output wire        INT2_n_OUT,         // Level 2 interrupt line (active low)
    output wire        INT2_n_OE,          // Open-drain driver enable for INT2
    output wire        INT6_n_OUT,         // Level 6 interrupt line (active low)
    output wire        INT6_n_OE,          // Open-drain driver enable for INT6

    // -------------------------------------------------------------------------
    // Multiplexed Address/Data (DA) Bus & PCB Bus Switch / Latch Controls
    // -------------------------------------------------------------------------
    input  wire [31:0] DA_IN,              // 32-bit bidirectional multiplexed bus input
    output wire [31:0] DA_OUT,             // 32-bit multiplexed bus output
    output wire [31:0] DA_OE,              // 32-bit output enables
    output wire        ADDR_LE,            // 74LVC573 Address Latch Enable (High = transparent)
    output wire        ADDR_OE_n,          // 74LVC573 Address Output Enable, active low
    output wire        DATA_OE_n,          // 74CBTD3384 Data Bus Switch Output Enable, active low
    output wire        CTRL_OE_n,          // Control Bus Driver Output Enable, active low

    // -------------------------------------------------------------------------
    // Control Inputs (from pi_interface)
    // -------------------------------------------------------------------------
    input  wire        request_bm,         // Request MC68020 bus mastership
    input  wire        drive_reset,        // Drive Amiga /RESET active
    input  wire        drive_halt,         // Drive Amiga /HALT active
    input  wire        drive_int2,         // Drive Amiga INT2 active
    input  wire        drive_int6,         // Drive Amiga INT6 active
    input  wire        increment_execute_slot_pointer, // Ping-pong slot automatically
    input  wire        enable_prefetch,    // Enable speculative read prefetch

    // -------------------------------------------------------------------------
    // Status Outputs (to pi_interface)
    // -------------------------------------------------------------------------
    output reg         is_bm = 1'b0,       // Bus mastership actively granted and held
    output reg         reset_sync = 1'b0,  // Synchronized system reset
    output reg         halt_sync = 1'b0,   // Synchronized system halt
    output reg  [2:0]  ipl = 3'b111,       // Synchronized and debounced IPL
    output reg         mc_reset_n_sync = 1'b1, // Filtered active-high reset flag
    output wire [23:0] current_bus_address,// Live address for debug status

    // -------------------------------------------------------------------------
    // Slot Request Inputs (from pi_interface)
    // -------------------------------------------------------------------------
    input  wire [1:0]  req_active,         // Slot active flags
    input  wire [1:0]  req_internal_intercept, // Precomputed virtual Zorro access flags
    input  wire [23:0] req_address_0,
    input  wire [23:0] req_address_1,
    input  wire [31:0] req_data_write_0,
    input  wire [31:0] req_data_write_1,
    input  wire [1:0]  req_size_0,
    input  wire [1:0]  req_size_1,
    input  wire        req_rw_0,
    input  wire        req_rw_1,
    input  wire [2:0]  req_fc_0,
    input  wire [2:0]  req_fc_1,
    input  wire        req_is_chip_or_custom_0,
    input  wire        req_is_chip_or_custom_1,
    input  wire        new_req_pf_match,
    output wire [7:0]  prefetch_addr_hi,
    input  wire        new_req_valid,      // 1-cycle strobe when Pi writes ADDR_HI
    input  wire        new_req_slot,       // Slot ID for new request
    input  wire [23:0] new_req_addr,       // Physical address for new request
    input  wire        new_req_rw,         // 1 = Read, 0 = Write
    input  wire [1:0]  new_req_size,       // Size for new request

    // -------------------------------------------------------------------------
    // Slot Pointer Override (from pi_interface)
    // -------------------------------------------------------------------------
    input  wire        set_execute_slot_valid,
    input  wire        set_execute_slot_val,

    // -------------------------------------------------------------------------
    // Combinatorial Slot Completion Outputs (to pi_interface)
    // -------------------------------------------------------------------------
    output wire        slot0_complete_valid,   // Slot 0 termination pulse
    output wire        slot1_complete_valid,   // Slot 1 termination pulse
    output wire        slot_complete_valid,    // 1-cycle termination pulse
    output wire        slot_complete_id,       // Slot ID being completed
    output wire [31:0] slot0_complete_data,    // Slot 0 read data returned
    output wire [31:0] slot1_complete_data,    // Slot 1 read data returned
    output wire [31:0] slot_complete_data,     // Read data returned to slot
    output wire        slot_complete_normally, // 1 = Normal (DSACK), 0 = Bus Error (BERR)

    // -------------------------------------------------------------------------
    // Virtual Zorro Device Interface (Pipelined Handshake)
    // -------------------------------------------------------------------------
    output reg         z2_access_valid = 1'b0,         // Access request pending
    input  wire        z2_access_ready,                // Access completion ready from zorro_device
    output reg         z2_access_wr = 1'b0,            // 1 = Write, 0 = Read
    output reg  [1:0]  z2_access_size = 2'd0,          // 0=Byte, 1=Word, 3=Long
    output reg  [23:0] z2_access_addr = 24'd0,         // Physical address
    output reg  [31:0] z2_access_wr_data = 32'd0,      // Write data
    input  wire [31:0] z2_rd_data,                     // Read data from zorro_device

    // -------------------------------------------------------------------------
    // Hardware Diagnostic & Telemetry Interface (to zorro_device)
    // -------------------------------------------------------------------------
    output wire [31:0] prefetch_launch_count,
    output wire [31:0] prefetch_hit_count,
    output wire [31:0] diag_status,
    output wire [31:0] diag_bus_capture,
    output wire [31:0] diag_cycle_timing,
    output wire [31:0] diag_clock_phase,
    output wire        phase_calibrated,
    output wire        fast_read_phase_cal,
    output wire        current_cck_phase,
    input  wire        prefetch_ctrl_en,
    input  wire        fast_dsack_en,
    input  wire        cck_sync_en,
    input  wire        force_phase_invert,
    input  wire        enable_word_prefetch,
    input  wire        counter_clear
);

// Bus control outputs
    // =========================================================================
    // SECTION 1: Bus Arbitration & Auxiliary Control Drivers
    //
    // Arbitration Protocol:
    //   1. Assert /BR (MC_BR_n_OE = request_bm, active low).
    //   2. Wait for /BG (MC_BG_n low) AND current /AS negated (as_sync = 0).
    //   3. Assume bus ownership (is_bm <= 1).
    // =========================================================================
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

    // =========================================================================
    // SECTION 2: Universal Amiga 1200 Clock Filter & Glitch Synchronizer
    //
    // Amiga 1200 motherboard revisions (1D.4, 2B) often suffer from clock line
    // reflections ("ringing" down to 1.8V) and duty cycle asymmetry caused by
    // XOR gate clock buffers.
    //
    // A 3-tick lock-out counter suppresses false edges caused by clock dips,
    // guaranteeing safe, stable rising and falling strobe generation.
    // =========================================================================
    localparam [2:0] MC_CLK_LOCKOUT_TICKS = 3'd3;

    (* async_reg = "true" *) reg [1:0] mc_clk_raw_sync = 2'b00;
    reg       mc_clk_filtered = 1'b0;
    reg [2:0] mc_clk_lockout  = 3'd0;
    reg       rising          = 1'b0; // 1-cycle pulse on filtered 14 MHz rising edge
    reg       falling         = 1'b0; // 1-cycle pulse on filtered 14 MHz falling edge

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

    // Synchronize asynchronous bus control inputs on falling edge of MC_CLK (S4)
    (* async_reg = "true" *) reg [2:0] ipl_sync [1:0];
    (* async_reg = "true" *) reg [1:0] mc_dsack_n_sync;
    reg mc_berr_n_sync;
    reg [31:0] mc_data_read;

    always @(posedge clk) begin
        if (falling) begin
            reset_sync      <= !MC_RESET_n_IN;
            halt_sync       <= !MC_HALT_n_IN;
            mc_reset_n_sync <= MC_RESET_n_IN;

            // Debounce 3-bit interrupt level across 2 falling clock edges
            ipl_sync[0] <= ~MC_IPL_n;
            ipl_sync[1] <= ipl_sync[0];
            if (ipl_sync[0] == ipl_sync[1])
                ipl <= ipl_sync[0];

            // Sample read data from DA bus on every falling clock edge (exact upstream behavior)
            mc_data_read <= DA_IN;

            mc_dsack_n_sync     <= MC_DSACK_n;
            mc_berr_n_sync      <= MC_BERR_n;
            terminated_normally <= MC_BERR_n && MC_RESET_n_IN;
        end
    end

    // =========================================================================
    // SECTION 3: Motorola MC68020 Dynamic Bus Sizing
    //
    // /DSACK1 /DSACK0 Encoding (MC68020UM Table 5-1):
    //   2'b11: No acknowledge (wait state)
    //   2'b10: 8-bit port  (port_width = 2'd0)
    //   2'b01: 16-bit port (port_width = 2'd1)
    //   2'b00: 32-bit port (port_width = 2'd3)
    // =========================================================================
    reg [1:0] port_width;
    always @(*) begin
        case (mc_dsack_n_sync)
            2'b11: port_width = 2'd0; // Wait state / default
            2'b10: port_width = 2'd0; // 8-bit port
            2'b01: port_width = 2'd1; // 16-bit port
            2'b00: port_width = 2'd3; // 32-bit port
        endcase
    end

    wire any_termination     = |(~{mc_dsack_n_sync, mc_berr_n_sync, mc_reset_n_sync});
    reg  terminated_normally = 1'b1;

    // Request slot execution pointers
    reg         current_execute_slot = 1'b0;
    (* syn_preserve = 1 *) reg current_execute_slot_addr = 1'b0;
    (* syn_preserve = 1 *) reg current_execute_slot_complete = 1'b0;
    (* syn_preserve = 1 *) reg current_execute_slot_ctrl = 1'b0;
    (* syn_preserve = 1 *) reg current_execute_slot_bus = 1'b0;

    // =========================================================================
    // SECTION 4: Speculative 32-Bit Read Prefetch Engine Registers
    // =========================================================================
    reg [23:0]  prefetch_addr = 24'd0;        // Address stored in prefetch buffer
    assign prefetch_addr_hi = prefetch_addr[23:16];
    reg [31:0]  prefetch_data = 32'd0;        // Data read during prefetch cycle
    reg         prefetch_valid = 1'b0;       // Buffer holds valid data
`ifdef ENABLE_16BIT_PREFETCH
    reg         prefetch_word0_avail = 1'b0; // Word 0 ($00) available in prefetch buffer
    reg         prefetch_word1_avail = 1'b0; // Word 1 ($02) available in prefetch buffer
`endif
    reg         prefetch_eligible = 1'b0;    // Last cycle qualifies for speculative read-ahead
    reg         is_prefetch_cycle = 1'b0;    // Current bus cycle is speculative
    reg [23:0]  prefetch_target_addr = 24'd0;// Base address + 4 for next prefetch (unified)
    reg         next_prefetch_allowed = 1'b0;// Safety gate preventing illegal prefetch
    reg         chained_prefetch_allowed = 1'b0;
    reg         can_prefetch = 1'b0;         // Trigger prefetch when bus is idle (registered)
    reg         prefetch_launch_pulse = 1'b0;// Single-cycle strobe to decouple 32-bit counter CE
    reg         prefetch_hit_pulse = 1'b0;   // Single-cycle strobe to decouple 32-bit counter CE
    reg [1:0]   req_prefetch_hit = 2'b00;    // Slot 0 / 1 hit flags (locally cached)
    reg [1:0]   latched_port_width = 2'd0;   // Captured during active /AS (S5)
    reg [1:0]   latched_size = 2'd0;         // Captured transfer size (S5)
    reg         is_safe_ram = 1'b0;          // Registered RAM safety check (Chip/Exp RAM)
    reg [23:0]  subcycle_next_addr = 24'd0;  // Precomputed next address for dynamic bus sizing retry
    reg [1:0]   subcycle_next_size = 2'd0;   // Precomputed next size for dynamic bus sizing retry
    reg         s7_s0_hit_pref = 1'b0;       // Pipelined prefetch hit match from State 7
    reg         s7_s1_hit_pref = 1'b0;
    reg         s7_s0_w1_hit   = 1'b0;
    reg         s7_s1_w1_hit   = 1'b0;
    reg         is_chip_or_custom_q = 1'b0;  // Precalculated Chip/Custom address space flag

    // MC68020 physical bus register staging
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

    // =========================================================================
    // SECTION 5: Multiplexed DA Bus Control & PCB Route Deserializer
    // =========================================================================
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

    // PCB trace descrambler mapping FPGA I/O pins to MC68020 address bits
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

    // Active sub-cycle registers for dynamic bus sizing
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

    // Current request multiplexers
    wire [23:0] cur_req_addr = current_execute_slot_addr ? req_address_1 : req_address_0;
    wire [31:0] cur_req_data = current_execute_slot ? req_data_write_1 : req_data_write_0;
    wire [1:0]  cur_req_size = current_execute_slot_bus ? req_size_1 : req_size_0;
    wire        cur_req_rw   = current_execute_slot_bus ? req_rw_1 : req_rw_0;
    wire [2:0]  cur_req_fc   = current_execute_slot_bus ? req_fc_1 : req_fc_0;

    // Direct per-slot request decoding: simple 2:1 muxes directly from registers
    wire cur_slot_active = current_execute_slot_ctrl ? req_active[1] : req_active[0];
    wire cur_slot_int    = current_execute_slot_ctrl ? req_internal_intercept[1] : req_internal_intercept[0];
    wire cur_slot_hit    = current_execute_slot_ctrl ? req_prefetch_hit[1] : req_prefetch_hit[0];

    wire slot0_hit_req   = req_active[0] && (!req_internal_intercept[0]) && req_prefetch_hit[0];
    wire slot1_hit_req   = req_active[1] && (!req_internal_intercept[1]) && req_prefetch_hit[1];

    wire cur_req_act  = cur_slot_active;
    wire cur_req_int  = cur_slot_active && cur_slot_int;
    wire cur_req_hit  = cur_slot_active && (!cur_slot_int) && cur_slot_hit;
    wire cur_req_norm = cur_slot_active && (!cur_slot_int) && (!cur_slot_hit);
    wire do_prefetch  = can_prefetch && !req_active[0] && !req_active[1];

    // =========================================================================
    // SECTION 6: 10-State One-Hot MC68020 Bus Master State Machine
    //
    // States:
    //   0: WAIT_ACTIVE_REQUEST    - Idle / Arbitrate slot request / Prefetch dispatch
    //   1: WAIT_BUS_CYCLE_START   - Synchronize with S0 (rising edge of MC_CLK)
    //   2: WAIT_ASSERT_AS         - Drive address and assert /AS, /DS (S1)
    //   3: WAIT_OPEN_DATA_LATCH   - Latch address into 74LVC573, turn DA bus (S2)
    //   4: WAIT_TERMINATION       - Wait for /DSACK or /BERR / /HALT (S3)
    //   5: S4_NOP                 - Motorola S4/S5 hold time settle cycle
    //   6: WAIT_LATCH_DATA        - Negate /AS, /DS and latch DA read data (S5)
    //   7: UPDATE_DATA_READ       - Dynamic Bus Sizing data realignment (Table 5-4)
    //   8: MAYBE_TERMINATE_ACCESS - Check transfer completion; re-cycle if partial
    //   9: INTERNAL_FINISH        - Zero-wait-state completion for virtual Zorro
    // =========================================================================
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

    // Combinatorial Slot Completion Logic
    // Fires on the exact clock edge the transaction completes, avoiding pipeline bubbles.
    wire normal_cycle_terminate    = state[STATE_BIT_MAYBE_TERMINATE_ACCESS] && (!is_prefetch_cycle) && (!terminated_normally || size_le_transfered);
    wire internal_finish_terminate = state[STATE_BIT_INTERNAL_FINISH] && z2_access_ready;
    wire cycle_term_active         = normal_cycle_terminate || internal_finish_terminate;

    assign slot0_complete_valid    = (cycle_term_active && (!current_execute_slot_complete)) ||
                                     (state[STATE_BIT_WAIT_ACTIVE_REQUEST] && (!current_execute_slot_complete) && slot0_hit_req);
    assign slot1_complete_valid    = (cycle_term_active && ( current_execute_slot_complete)) ||
                                     (state[STATE_BIT_WAIT_ACTIVE_REQUEST] && ( current_execute_slot_complete) && slot1_hit_req);

    wire prefetch_hit_terminate    = state[STATE_BIT_WAIT_ACTIVE_REQUEST] && cur_req_hit;

    assign slot_complete_valid     = slot0_complete_valid || slot1_complete_valid;
    assign slot_complete_id        = current_execute_slot_complete;
`ifdef ENABLE_16BIT_PREFETCH
    wire [31:0] slot0_prefetch_data =
        (req_size_0 == 2'd1) ?
            (req_address_0[1] ? {16'd0, prefetch_data[15:0]} : {16'd0, prefetch_data[31:16]}) :
            prefetch_data;

    wire [31:0] slot1_prefetch_data =
        (req_size_1 == 2'd1) ?
            (req_address_1[1] ? {16'd0, prefetch_data[15:0]} : {16'd0, prefetch_data[31:16]}) :
            prefetch_data;

    assign slot0_complete_data     = state[STATE_BIT_INTERNAL_FINISH]   ? z2_rd_data :
                                     state[STATE_BIT_WAIT_ACTIVE_REQUEST] ? slot0_prefetch_data :
                                     data_read;
    assign slot1_complete_data     = state[STATE_BIT_INTERNAL_FINISH]   ? z2_rd_data :
                                     state[STATE_BIT_WAIT_ACTIVE_REQUEST] ? slot1_prefetch_data :
                                     data_read;
`else
    assign slot0_complete_data     = state[STATE_BIT_INTERNAL_FINISH]   ? z2_rd_data :
                                     state[STATE_BIT_WAIT_ACTIVE_REQUEST] ? prefetch_data :
                                     data_read;
    assign slot1_complete_data     = state[STATE_BIT_INTERNAL_FINISH]   ? z2_rd_data :
                                     state[STATE_BIT_WAIT_ACTIVE_REQUEST] ? prefetch_data :
                                     data_read;
`endif
    assign slot_complete_data      = current_execute_slot_complete ? slot1_complete_data : slot0_complete_data;
    assign slot_complete_normally  = normal_cycle_terminate ? terminated_normally : 1'b1;

    // Effective prefetch enable controlled by both Pi register and Zorro register override
    // Synchronously registered to eliminate cross-chip fanout delay to prefetch CE paths.
    (* syn_keep = "true" *) reg r_enable_prefetch_eff = 1'b0;
    (* syn_keep = "true" *) reg r_enable_16bit_prefetch_eff = 1'b0;

    always @(posedge clk) begin
        r_enable_prefetch_eff       <= enable_prefetch && prefetch_ctrl_en;
        r_enable_16bit_prefetch_eff <= enable_prefetch && prefetch_ctrl_en && enable_word_prefetch;
    end

    wire enable_prefetch_eff = r_enable_prefetch_eff;
`ifdef ENABLE_16BIT_PREFETCH
    wire enable_16bit_prefetch_eff = r_enable_16bit_prefetch_eff;
`endif

    // Prefetch telemetry counters
    reg [31:0] r_prefetch_launch_count = 32'd0;
    reg [31:0] r_prefetch_hit_count    = 32'd0;

    assign prefetch_launch_count = r_prefetch_launch_count;
    assign prefetch_hit_count    = r_prefetch_hit_count;

    reg [1:0]  diag_dsack_at_term = 2'b11;
    reg [1:0]  diag_dsack_at_s4   = 2'b11;
    reg [1:0]  diag_dsack_at_s5   = 2'b11;
    reg [1:0]  diag_port_width_s5 = 2'd0;
    reg [1:0]  diag_size_s5       = 2'd0;
    reg        diag_term_eligible = 1'b0;
    reg        diag_size_le       = 1'b0;
    reg        diag_term_norm     = 1'b0;
    reg        diag_rw            = 1'b0;
    reg [7:0]  diag_addr_lo       = 8'd0;
    reg [7:0]  diag_cycle_count   = 8'd0;

    assign diag_bus_capture = {
        diag_cycle_count,    // [31:24]
        diag_addr_lo,        // [23:16]
        diag_rw,             // [15]
        diag_term_norm,      // [14]
        diag_size_le,        // [13]
        diag_term_eligible,  // [12]
        diag_size_s5,        // [11:10]
        diag_port_width_s5,  // [9:8]
        diag_dsack_at_s5,    // [7:6]
        diag_dsack_at_s4,    // [5:4]
        diag_dsack_at_term,  // [3:2]
        mc_dsack_n_sync      // [1:0]
    };

    assign diag_status = {
        halt_sync,
        reset_sync,
        cur_req_hit,
        cur_req_int,
        cur_req_act,
        terminated_normally,
        mc_dsack_n_sync,
        is_bm,
        enable_prefetch_eff,
        next_prefetch_allowed,
        chained_prefetch_allowed,
        can_prefetch,
        prefetch_valid,
        is_prefetch_cycle,
        req_prefetch_hit,
        prefetch_eligible,
        latched_size[1:0],
        latched_port_width[1:0],
        state[9:0]
    };

    // =========================================================================
    // SECTION 6B: Hardware Bus Cycle Timing Analyzer & Clock Profiler
    // =========================================================================
    reg [11:0] cyc_timer            = 12'd0;
    reg [11:0] diag_as_to_dsack     = 12'd0;
    reg [11:0] diag_as_total        = 12'd0;
    reg [7:0]  diag_dsack_lead_ticks= 8'd0;
    reg [3:0]  diag_term_wait_states= 4'd0;
    reg [3:0]  cyc_wait_states      = 4'd0;
    reg        cyc_dsack_seen       = 1'b0;
    reg        diag_dsack_at_high   = 1'b0;
    reg [7:0]  lead_acc             = 8'd0;
    reg        cck_phase_toggle     = 1'b0;
    reg        as_start_phase       = 1'b0;
    reg        r_phase_calibrated   = 1'b0;
    reg        r_fast_read_phase    = 1'b1; // Default expectation: Phase 1 is fast read

    // 14 MHz Clock Period & Duty Cycle Measurer
    reg [7:0] mc_clk_period_ticks = 8'd0;
    reg [7:0] mc_clk_high_ticks   = 8'd0;
    reg [7:0] mc_clk_low_ticks    = 8'd0;
    reg [7:0] r_period_acc        = 8'd0;
    reg [7:0] r_high_acc          = 8'd0;
    reg [7:0] r_low_acc           = 8'd0;

    always @(posedge clk) begin
        if (rising) begin
            cck_phase_toggle    <= ~cck_phase_toggle;
            mc_clk_period_ticks <= r_period_acc;
            mc_clk_low_ticks    <= r_low_acc;
            r_period_acc        <= 8'd1;
            r_high_acc          <= 8'd1;
            r_low_acc           <= 8'd0;
        end else if (falling) begin
            mc_clk_high_ticks   <= r_high_acc;
            r_period_acc        <= r_period_acc + 8'd1;
            r_low_acc           <= 8'd1;
        end else begin
            if (r_period_acc < 8'hFF) r_period_acc <= r_period_acc + 8'd1;
            if (mc_clk_filtered) begin
                if (r_high_acc < 8'hFF) r_high_acc <= r_high_acc + 8'd1;
            end else begin
                if (r_low_acc < 8'hFF)  r_low_acc  <= r_low_acc + 8'd1;
            end
        end

        // Bus cycle duration & DSACK timing profiler
        if (state[STATE_BIT_WAIT_ASSERT_AS]) begin
            cyc_timer       <= 12'd0;
            cyc_dsack_seen  <= 1'b0;
            cyc_wait_states <= 4'd0;
            lead_acc        <= 8'd0;
            as_start_phase  <= cck_phase_toggle;
        end else if (mc_as) begin
            if (cyc_timer < 12'hFFF)
                cyc_timer <= cyc_timer + 12'd1;

            // Detect raw /DSACK assertion on pins (independent of falling edge)
            if (!cyc_dsack_seen && (MC_DSACK_n != 2'b11 || !MC_BERR_n)) begin
                cyc_dsack_seen     <= 1'b1;
                diag_as_to_dsack   <= cyc_timer;
                diag_dsack_at_high <= mc_clk_filtered;
            end

            // Accumulate lead ticks from raw DSACK until next falling edge
            if (cyc_dsack_seen && !falling && (lead_acc < 8'hFF)) begin
                lead_acc <= lead_acc + 8'd1;
            end

            // Count Amiga 14 MHz falling edges spent in WAIT_TERMINATION
            if (state[STATE_BIT_WAIT_TERMINATION] && falling) begin
                if (cyc_wait_states < 4'hF)
                    cyc_wait_states <= cyc_wait_states + 4'd1;
            end
        end

        // Capture end of cycle when /AS is negated (falling edge in WAIT_LATCH_DATA)
        if (state[STATE_BIT_WAIT_LATCH_DATA] && falling) begin
            diag_as_total         <= cyc_timer;
            diag_term_wait_states <= cyc_wait_states;
            diag_dsack_lead_ticks <= lead_acc;

            // Robust CCK Phase Auto-Calibration for Chip RAM ($000000..$1FFFFF)
            // Alice's fastest slots: Write = 5 wait states (456 ns), Read = 6 wait states (527 ns).
            // A cycle can NEVER complete in <= 5 (write) or <= 6 (read) wait states on the wrong phase.
            // Slow cycles (> 5 write, > 6 read) can occur on either phase due to Alice DMA.
            // Therefore, ONLY clean, fast cycles are used to calibrate the phase.
            if (address[23:21] == 3'b000) begin
                if (!r_phase_calibrated) begin
                    if (!rw && (cyc_wait_states <= 4'd5)) begin
                        // Clean 5-wait-state write detected on as_start_phase.
                        // Optimal read phase is the opposite phase (~as_start_phase).
                        r_fast_read_phase  <= ~as_start_phase;
                        r_phase_calibrated <= 1'b1;
                    end else if (rw && (cyc_wait_states <= 4'd6)) begin
                        // Clean 6-wait-state read detected on as_start_phase.
                        // Optimal read phase is the same phase (as_start_phase).
                        r_fast_read_phase  <= as_start_phase;
                        r_phase_calibrated <= 1'b1;
                    end
                end
            end
        end

        if (counter_clear || reset_sync) begin
            r_phase_calibrated <= 1'b0;
        end
    end

    assign diag_cycle_timing = {
        diag_as_total[9:0],         // [31:22] Total /AS width in 182 MHz clk (each 5.5 ns, up to 5.6 us)
        diag_as_to_dsack[9:0],      // [21:12] /AS assertion to raw /DSACK low in 182 MHz clk
        diag_term_wait_states[3:0], // [11:8]  14 MHz wait states in STATE_WAIT_TERMINATION
        diag_dsack_lead_ticks[3:0], // [7:4]   Lead ticks of raw DSACK before falling edge (capped at 15)
        diag_dsack_at_high,         // [3]     1 = DSACK arrived when MC_CLK was HIGH
        as_start_phase,             // [2]     CCK alternating 14 MHz phase at /AS assert
        diag_rw,                    // [1]     1 = Read, 0 = Write
        any_termination             // [0]     Termination status
    };

    assign diag_clock_phase = {
        diag_cycle_count[7:0],      // [31:24] Cycle sequence number
        mc_clk_period_ticks[7:0],   // [23:16] Total 14 MHz period in 182 MHz clocks (~13)
        mc_clk_high_ticks[7:0],     // [15:8]  High half period in 182 MHz clocks (~6-7)
        mc_clk_low_ticks[7:0]       // [7:0]   Low half period in 182 MHz clocks (~6-7)
    };

    assign phase_calibrated    = r_phase_calibrated;
    assign fast_read_phase_cal = r_fast_read_phase;
    assign current_cck_phase   = cck_phase_toggle;

    // CCK Phase Alignment Logic:
    // Aligns /AS assertion with Alice's 7.09 MHz Colour Clock slot boundary.
    wire effective_fast_read_phase = r_fast_read_phase ^ force_phase_invert;
    wire target_start_phase        = rw ? effective_fast_read_phase : ~effective_fast_read_phase;
    wire is_chip_or_custom         = is_chip_or_custom_q;
    wire sync_to_cck               = cck_sync_en && is_chip_or_custom;

    // Prefetch Safety Filter:
    // Only prefetch from Chip-RAM ($000000..$1FFFFF) or Expansion RAM ($E00000..$FFFFFF).
    // NEVER speculatively prefetch from CIA ($BFExxx) or Custom Registers ($DFFxxx)!
`ifdef ENABLE_16BIT_PREFETCH
    wire is_eligible_size = (latched_size == 2'd3) ||
                            (enable_16bit_prefetch_eff && (latched_size == 2'd1) && (address[1] == 1'b0));
    wire prefetch_eligible_term = terminated_normally && rw && is_eligible_size &&
                                 (address[0] == 1'b0) && (latched_port_width == 2'd3) &&
                                 is_safe_ram;

    // Prefetch hit detection qualification signals (pre-matched at Pi interface)
    wire req_prefetch_qual = enable_prefetch_eff && prefetch_valid && new_req_pf_match;

    wire pf_s0_match_mid = (req_address_0[15:8] == prefetch_addr[15:8]);
    wire pf_s0_match_lo  = (req_address_0[7:2]  == prefetch_addr[7:2]);
    wire pf_s0_match_base = pf_s0_match_mid && pf_s0_match_lo;

    wire pf_s1_match_mid = (req_address_1[15:8] == prefetch_addr[15:8]);
    wire pf_s1_match_lo  = (req_address_1[7:2]  == prefetch_addr[7:2]);
    wire pf_s1_match_base = pf_s1_match_mid && pf_s1_match_lo;

    wire slot0_word0_match = pf_s0_match_base && (req_address_0[1] == prefetch_addr[1]) && prefetch_word0_avail;
    wire slot0_word1_match = pf_s0_match_base && (req_address_0[1] == 1'b1) && prefetch_word1_avail;
    wire slot0_long_match  = pf_s0_match_base && (req_address_0[1:0] == prefetch_addr[1:0]) && prefetch_word0_avail && prefetch_word1_avail;
    wire slot0_addr_match  = (new_req_size == 2'd3) ? slot0_long_match :
                             (req_address_0[1] ? slot0_word1_match : slot0_word0_match);

    wire slot1_word0_match = pf_s1_match_base && (req_address_1[1] == prefetch_addr[1]) && prefetch_word0_avail;
    wire slot1_word1_match = pf_s1_match_base && (req_address_1[1] == 1'b1) && prefetch_word1_avail;
    wire slot1_long_match  = pf_s1_match_base && (req_address_1[1:0] == prefetch_addr[1:0]) && prefetch_word0_avail && prefetch_word1_avail;
    wire slot1_addr_match  = (new_req_size == 2'd3) ? slot1_long_match :
                             (req_address_1[1] ? slot1_word1_match : slot1_word0_match);

    wire slot0_w1_hit = req_active[0] && req_rw_0 && (req_size_0 == 2'd1) &&
                        (req_address_0[23:16] == prefetch_addr[23:16]) && slot0_word1_match;
    wire slot1_w1_hit = req_active[1] && req_rw_1 && (req_size_1 == 2'd1) &&
                        (req_address_1[23:16] == prefetch_addr[23:16]) && slot1_word1_match;

    // Balanced address comparators for State 8 completion
    wire s8_s0_match_hi   = (req_address_0[23:16] == address[23:16]);
    wire s8_s0_match_mid  = (req_address_0[15:8]  == address[15:8]);
    wire s8_s0_match_lo   = (req_address_0[7:2]   == address[7:2]);
    wire s8_s0_match_base = s8_s0_match_hi && s8_s0_match_mid && s8_s0_match_lo;

    wire s8_s1_match_hi   = (req_address_1[23:16] == address[23:16]);
    wire s8_s1_match_mid  = (req_address_1[15:8]  == address[15:8]);
    wire s8_s1_match_lo   = (req_address_1[7:2]   == address[7:2]);
    wire s8_s1_match_base = s8_s1_match_hi && s8_s1_match_mid && s8_s1_match_lo;

    wire s8_s0_hit_pref = enable_prefetch_eff && req_active[0] && req_rw_0 && s8_s0_match_base &&
                          ((req_size_0 == 2'd3 && req_address_0[1:0] == address[1:0]) ||
                           (enable_16bit_prefetch_eff && req_size_0 == 2'd1 && req_address_0[1] == 1'b0));
    wire s8_s1_hit_pref = enable_prefetch_eff && req_active[1] && req_rw_1 && s8_s1_match_base &&
                          ((req_size_1 == 2'd3 && req_address_1[1:0] == address[1:0]) ||
                           (enable_16bit_prefetch_eff && req_size_1 == 2'd1 && req_address_1[1] == 1'b0));

    wire s8_s0_w1_hit   = req_active[0] && req_rw_0 && (req_size_0 == 2'd1) && s8_s0_match_base && req_address_0[1];
    wire s8_s1_w1_hit   = req_active[1] && req_rw_1 && (req_size_1 == 2'd1) && s8_s1_match_base && req_address_1[1];
`else
    wire prefetch_eligible_term = terminated_normally && rw && (latched_size == 2'd3) &&
                                 (address[1:0] == 2'b00) && (latched_port_width == 2'd3) &&
                                 is_safe_ram;

    // Prefetch hit detection qualification signals (pre-matched at Pi interface)
    wire req_prefetch_qual = enable_prefetch_eff && prefetch_valid && new_req_pf_match;
    wire slot0_addr_match  = (req_address_0[15:0] == prefetch_addr[15:0]);
    wire slot1_addr_match  = (req_address_1[15:0] == prefetch_addr[15:0]);
`endif

    // Prefetch Trigger Qualification Signal:
    // Speculative read is launched when enabled, FPGA owns the bus (is_bm),
    // previous cycle qualified as safe (Chip-RAM or Exp-RAM 32-bit read),
    // buffer does not already hold valid data, and next prefetch is allowed.
    // Synchronously registered to break long combinational path into State 0.
    always @(posedge clk) begin
        can_prefetch <= !reset_sync && enable_prefetch_eff && is_bm && prefetch_eligible &&
                        !prefetch_valid && next_prefetch_allowed;
    end

    // =========================================================================
    // SECTION 7: Main FSM Synchronous State Engine
    // =============================================================================
    always @(posedge clk) begin

        // Manual slot execution pointer override from PI_REG_SLOT
        if (set_execute_slot_valid) begin
            current_execute_slot          <= set_execute_slot_val;
            current_execute_slot_addr     <= set_execute_slot_val;
            current_execute_slot_complete <= set_execute_slot_val;
            current_execute_slot_ctrl     <= set_execute_slot_val;
            current_execute_slot_bus      <= set_execute_slot_val;
        end

        // Single-cycle telemetry strobe pulses (default to 0)
        prefetch_launch_pulse <= 1'b0;
        prefetch_hit_pulse    <= 1'b0;

        (* parallel_case, full_case *) case (1'b1)

            // -----------------------------------------------------------------
            // STATE 0: WAIT_ACTIVE_REQUEST
            // Bus master idle state. Arbitrates between:
            //   1. Host Pi queue requests (cur_req_act)
            //      a) Virtual Zorro internal access  -> pipelined Wishbone route
            //      b) Speculative read prefetch hit   -> 0 wait-state instant response
            //      c) Normal Amiga physical access   -> launch S0 bus cycle
            //   2. Speculative prefetch read-ahead   -> read next longword if bus idle
            // -----------------------------------------------------------------
            state[STATE_BIT_WAIT_ACTIVE_REQUEST]: begin
                if (rising)
                    da_state <= DA_STATE_IDLE;

                // Unconditional bus register setup in State 0
                // Decouples Clock Enable from slot arbitration, reducing CE logic depth to 0 LUTs.
                if (do_prefetch) begin
                    address             <= prefetch_target_addr;
                    is_chip_or_custom_q <= 1'b1;
                    fc                  <= 3'd1;
                    size                <= 2'd3;
                    rw                  <= 1'b1;
                end else begin
                    address             <= cur_req_addr;
                    is_chip_or_custom_q <= current_execute_slot_bus ? req_is_chip_or_custom_1 : req_is_chip_or_custom_0;
                    fc                  <= cur_req_fc;
                    size                <= cur_req_size;
                    rw                  <= cur_req_rw;
                end
                data_write <= cur_req_data;

                if (cur_slot_active) begin
                    if (cur_slot_int) begin
                        // ---------------------------------------------------------
                        // Case B: Virtual Zorro AutoConfig & 64KB I/O Space
                        // Routed entirely inside FPGA logic (zorro_device.v);
                        // physical Amiga 68020 bus remains idle/unaffected.
                        // Prefetch state for Amiga Chip RAM is preserved.
                        // ---------------------------------------------------------
                        z2_access_addr    <= cur_req_addr;
                        z2_access_wr_data <= cur_req_data;
                        z2_access_size    <= cur_req_size;
                        z2_access_wr      <= !cur_req_rw;
                        z2_access_valid   <= 1'b1;

                        state <= STATE_INTERNAL_FINISH;

                    end else if (cur_slot_hit) begin
                        // ---------------------------------------------------------
                        // Case C: Speculative Read Prefetch Hit
                        // The requested 32-bit data was already read into FPGA
                        // prefetch buffer during idle cycles. Immediate completion!
                        // ---------------------------------------------------------
                        prefetch_hit_pulse <= 1'b1;
                        req_prefetch_hit[current_execute_slot_ctrl] <= 1'b0;
                        if (increment_execute_slot_pointer) begin
                            current_execute_slot          <= current_execute_slot + 1'd1;
                            current_execute_slot_addr     <= current_execute_slot_addr + 1'd1;
                            current_execute_slot_complete <= current_execute_slot_complete + 1'd1;
                            current_execute_slot_ctrl     <= current_execute_slot_ctrl + 1'd1;
                            current_execute_slot_bus      <= current_execute_slot_bus + 1'd1;
                        end
`ifdef ENABLE_16BIT_PREFETCH
                        if (enable_16bit_prefetch_eff && cur_req_size == 2'd1 && cur_req_addr[1] == 1'b0) begin
                            // Consumed Word 0 ($00). Word 1 ($02) remains available in buffer!
                            prefetch_word0_avail <= 1'b0;
                            prefetch_valid       <= prefetch_word1_avail;

                            // Check if the other slot is already waiting for Word 1
                            if (!current_execute_slot_ctrl) begin
                                if (slot1_w1_hit)
                                    req_prefetch_hit[1] <= 1'b1;
                            end else begin
                                if (slot0_w1_hit)
                                    req_prefetch_hit[0] <= 1'b1;
                            end

                            // Do NOT launch chained prefetch yet because Word 1 is still in the buffer
                            is_prefetch_cycle <= 1'b0;
                            prefetch_eligible <= 1'b0;
                            state             <= STATE_WAIT_ACTIVE_REQUEST;
                        end else begin
                            // Consumed Word 1 ($02) or Longword: buffer is now exhausted
                            prefetch_word0_avail  <= 1'b0;
                            prefetch_word1_avail  <= 1'b0;
                            prefetch_valid        <= 1'b0;
                            is_prefetch_cycle     <= 1'b0;
                            prefetch_eligible     <= chained_prefetch_allowed;
                            next_prefetch_allowed <= chained_prefetch_allowed;
                            state                 <= STATE_WAIT_ACTIVE_REQUEST;
                        end
`else
                        prefetch_valid        <= 1'b0;
                        is_prefetch_cycle     <= 1'b0;
                        prefetch_eligible     <= chained_prefetch_allowed;
                        next_prefetch_allowed <= chained_prefetch_allowed;
                        state                 <= STATE_WAIT_ACTIVE_REQUEST;
`endif

                    end else begin
                        // ---------------------------------------------------------
                        // Case A: Normal Physical MC68020 Bus Access (Miss or Write)
                        // ---------------------------------------------------------
                        prefetch_valid           <= 1'b0;
`ifdef ENABLE_16BIT_PREFETCH
                        prefetch_word0_avail     <= 1'b0;
                        prefetch_word1_avail     <= 1'b0;
`endif
                        prefetch_eligible        <= 1'b0;
                        next_prefetch_allowed    <= 1'b0;
                        chained_prefetch_allowed <= 1'b0;
                        is_prefetch_cycle        <= 1'b0;
                        req_prefetch_hit         <= 2'b00;

                        state                    <= STATE_WAIT_BUS_CYCLE_START;
                    end

                end else if (can_prefetch) begin
                    // ---------------------------------------------------------
                    // Case D: Bus Master is Idle, Launch Speculative Read-Ahead
                    // Speculatively fetch [address + 4] while Pi prepares next op.
                    // ---------------------------------------------------------
                    prefetch_eligible     <= 1'b0;
                    next_prefetch_allowed <= 1'b0;
                    is_prefetch_cycle     <= 1'b1;
                    prefetch_launch_pulse <= 1'b1;
                    state                 <= STATE_WAIT_BUS_CYCLE_START;
                end
            end

            // -----------------------------------------------------------------
            // STATE 1: WAIT_BUS_CYCLE_START (Phase S0)
            // Synchronize with the 14 MHz MC_CLK rising edge.
            // Setup MC68020 SIZE encoding (Table 5-2) and replicate write data
            // across byte lanes according to Motorola MC68020UM Table 5-3.
            // -----------------------------------------------------------------
            state[STATE_BIT_WAIT_BUS_CYCLE_START]: begin
                is_safe_ram <= ((address[23:21] == 3'b000) || (&address[23:19]));
                if (rising && (!sync_to_cck || (cck_phase_toggle == target_start_phase))) begin // Synchronized with S0 & CCK
                    da_state   <= DA_STATE_IDLE;
                    mc_fc      <= fc;
                    mc_address <= address;
                    mc_rw      <= rw;

                    // MC68020 SIZ1 / SIZ0 Encoding:
                    //   2'd0 (1 byte)  -> 2'b01
                    //   2'd1 (2 bytes) -> 2'b10
                    //   2'd2 (3 bytes) -> 2'b11
                    //   2'd3 (4 bytes) -> 2'b00
                    case (size)
                        2'd0: mc_size <= 2'b01;
                        2'd1: mc_size <= 2'b10;
                        2'd2: mc_size <= 2'b11;
                        2'd3: mc_size <= 2'b00;
                    endcase

                    // Write Data Byte Steering / Multiplexing (Motorola Table 5-3)
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

            // -----------------------------------------------------------------
            // STATE 2: WAIT_ASSERT_AS (Phase S1)
            // Drive scrambled address onto DA bus and open 74LVC573 latches.
            // On falling edge of MC_CLK: close latches, assert /AS, and if read,
            // assert /DS immediately (per MC68020 timing specifications).
            // -----------------------------------------------------------------
            state[STATE_BIT_WAIT_ASSERT_AS]: begin
                da_state         <= DA_STATE_FPGA_TO_ADDR;
                address_latch_le <= 1'b1;

                if (falling) begin // S0 -> S1 transition
                    address_latch_le <= 1'b0;
                    mc_as            <= 1'b1;
                    if (rw)
                        mc_ds <= 1'b1;
                    state <= STATE_WAIT_OPEN_DATA_LATCH;
                end
            end

            // -----------------------------------------------------------------
            // STATE 3: WAIT_OPEN_DATA_LATCH (Phase S2)
            // On rising edge of MC_CLK:
            //   - For Read: switch DA bus to input mode (DA_STATE_DATA_TO_FPGA)
            //   - For Write: switch DA bus to output mode (DA_STATE_FPGA_TO_DATA)
            // -----------------------------------------------------------------
            state[STATE_BIT_WAIT_OPEN_DATA_LATCH]: begin
                if (rising) begin // S1 -> S2 transition
                    if (rw)
                        da_state <= DA_STATE_DATA_TO_FPGA;
                    else
                        da_state <= DA_STATE_FPGA_TO_DATA;

                    state <= STATE_WAIT_TERMINATION;
                end
            end

            // -----------------------------------------------------------------
            // STATE 4: WAIT_TERMINATION (Phase S3 / Wait States)
            // For write cycles, assert /DS on falling clock edge (S2->S3).
            // Wait until motherboard asserts /DSACK0, /DSACK1, /BERR, or /RESET.
            // -----------------------------------------------------------------
            state[STATE_BIT_WAIT_TERMINATION]: begin
                if (falling) begin // S2 -> S3 transition
                    if (!rw)
                        mc_ds <= 1'b1;
                end
                if (any_termination) begin
                    diag_dsack_at_term <= mc_dsack_n_sync;
                    if (fast_dsack_en)
                        state <= STATE_WAIT_LATCH_DATA;
                    else
                        state <= STATE_S4_NOP;
                end
            end

            // -----------------------------------------------------------------
            // STATE 5: S4_NOP (Phase S4)
            // Motorola MC68020 S4/S5 hold time settle cycle to guarantee signal
            // integrity and bus stability across legacy Amiga expansion boards.
            // -----------------------------------------------------------------
            state[STATE_BIT_S4_NOP]: begin
                if (falling) begin
                    diag_dsack_at_s4 <= mc_dsack_n_sync;
                    state <= STATE_WAIT_LATCH_DATA;
                end
            end

            // -----------------------------------------------------------------
            // STATE 6: WAIT_LATCH_DATA (Phase S5)
            // On falling edge of MC_CLK:
            //   - Negate /AS and /DS.
            //   - Calculate dynamic bus sizing parameters:
            //       left_shift         = address[1:0] & port_width
            //       transfered         = port_width - left_shift
            //       size_le_transfered = (size <= transfered)
            //   - Route to dynamic read demux or cycle completion check.
            // -----------------------------------------------------------------
            state[STATE_BIT_WAIT_LATCH_DATA]: begin
                if (falling) begin // S4 -> S5 transition
                    mc_as <= 1'b0;
                    mc_ds <= 1'b0;

                    left_shift         <= address[1:0] & port_width;
                    transfered         <= port_width - (address[1:0] & port_width);
                    size_le_transfered <= (size <= (port_width - (address[1:0] & port_width)));
                    subcycle_next_addr <= address + {22'd0, (port_width - (address[1:0] & port_width)) + 2'd1};
                    subcycle_next_size <= size - ((port_width - (address[1:0] & port_width)) + 2'd1);
                    latched_port_width <= port_width;
                    latched_size       <= size;

                    diag_dsack_at_s5   <= mc_dsack_n_sync;
                    diag_port_width_s5 <= port_width;
                    diag_size_s5       <= size;
                    diag_rw            <= rw;
                    diag_addr_lo       <= address[7:0];
                    diag_cycle_count   <= diag_cycle_count + 8'd1;

                    if (rw)
                        state <= STATE_UPDATE_DATA_READ;
                    else
                        state <= STATE_MAYBE_TERMINATE_ACCESS;
                end
            end

            // -----------------------------------------------------------------
            // STATE 7: UPDATE_DATA_READ
            // Motorola MC68020 Dynamic Bus Sizing Read Alignment (Table 5-4):
            // Realigns sampled DA bus bytes (op0_rd..op3_rd) into internal 32-bit
            // register lanes (data_read_op0..op3) based on port width and alignment.
            // -----------------------------------------------------------------
            state[STATE_BIT_UPDATE_DATA_READ]: begin
                // OP3 lane alignment
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

                // OP2 lane alignment
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

                // OP1 lane alignment
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

                // OP0 lane alignment
                case (size)
                    2'd3:
                        case (left_shift)
                            2'd0: data_read_op0 <= op0_rd;
                            2'd1: data_read_op0 <= op1_rd;
                            2'd2: data_read_op0 <= op2_rd;
                            2'd3: data_read_op0 <= op3_rd;
                        endcase
                endcase

                s7_s0_hit_pref <= s8_s0_hit_pref;
                s7_s1_hit_pref <= s8_s1_hit_pref;
                s7_s0_w1_hit   <= s8_s0_w1_hit;
                s7_s1_w1_hit   <= s8_s1_w1_hit;

                state <= STATE_MAYBE_TERMINATE_ACCESS;
            end

            // -----------------------------------------------------------------
            // STATE 8: MAYBE_TERMINATE_ACCESS
            // Determine if full transfer has finished or if another bus cycle
            // is required (e.g. 32-bit access across a 16-bit or 8-bit port).
            // Handles prefetch buffer storage and chaining.
            // -----------------------------------------------------------------
            state[STATE_BIT_MAYBE_TERMINATE_ACCESS]: begin
                if (is_prefetch_cycle) begin
                    if (!terminated_normally || size_le_transfered) begin
                        prefetch_target_addr <= address + 24'd4;
                        if (terminated_normally) begin
                            prefetch_data            <= data_read;
                            prefetch_addr            <= address;
                            prefetch_valid           <= 1'b1;
                            chained_prefetch_allowed <= is_bm && is_safe_ram;
`ifdef ENABLE_16BIT_PREFETCH
                            prefetch_word0_avail     <= 1'b1;
                            prefetch_word1_avail     <= 1'b1;
                            req_prefetch_hit[0]      <= s7_s0_hit_pref;
                            req_prefetch_hit[1]      <= s7_s1_hit_pref;
`else
                            req_prefetch_hit[0]      <= s7_s0_hit_pref;
                            req_prefetch_hit[1]      <= s7_s1_hit_pref;
`endif
                        end else begin
                            prefetch_valid       <= 1'b0;
`ifdef ENABLE_16BIT_PREFETCH
                            prefetch_word0_avail <= 1'b0;
                            prefetch_word1_avail <= 1'b0;
`endif
                            req_prefetch_hit     <= 2'b00;
                        end
                        is_prefetch_cycle <= 1'b0;
                        prefetch_eligible <= 1'b0;
                        state             <= STATE_WAIT_ACTIVE_REQUEST;
                    end else begin
                        // Subsequent sub-cycle required for dynamic sizing during prefetch
                        address <= subcycle_next_addr;
                        size    <= subcycle_next_size;
                        state   <= STATE_WAIT_BUS_CYCLE_START;
                    end
                end else begin
                    if (!terminated_normally || size_le_transfered) begin
                        diag_term_eligible   <= prefetch_eligible_term;
                        diag_size_le         <= size_le_transfered;
                        diag_term_norm       <= terminated_normally;
                        prefetch_target_addr <= address + 24'd4;
                        if (increment_execute_slot_pointer) begin
                            current_execute_slot          <= current_execute_slot + 1'd1;
                            current_execute_slot_addr     <= current_execute_slot_addr + 1'd1;
                            current_execute_slot_complete <= current_execute_slot_complete + 1'd1;
                            current_execute_slot_ctrl     <= current_execute_slot_ctrl + 1'd1;
                            current_execute_slot_bus      <= current_execute_slot_bus + 1'd1;
                        end

                        if (prefetch_eligible_term) begin
`ifdef ENABLE_16BIT_PREFETCH
                            if (enable_16bit_prefetch_eff && (latched_size == 2'd1) && (address[1] == 1'b0)) begin
                                // 16-bit word read of Word 0 completed on 32-bit port (Chip RAM).
                                // Alice drove both Word 0 and Word 1 onto DA[31:0].
                                // mc_data_read holds {Word 0, Word 1}.
                                prefetch_data            <= mc_data_read;
                                prefetch_addr            <= address;
                                prefetch_valid           <= 1'b1;
                                prefetch_word0_avail     <= 1'b0; // Word 0 was just read
                                prefetch_word1_avail     <= 1'b1; // Word 1 is available!
                                prefetch_target_addr     <= address + 24'd4;
                                chained_prefetch_allowed <= is_bm && is_safe_ram;
                                prefetch_eligible        <= 1'b0; // Buffer holds Word 1, do not launch speculative prefetch yet
                                next_prefetch_allowed    <= 1'b0;

                                // Check if other slot is already queued for Word 1 ($address + 2)
                                if (!current_execute_slot_ctrl) begin
                                    req_prefetch_hit[1] <= s7_s1_w1_hit;
                                    req_prefetch_hit[0] <= 1'b0;
                                end else begin
                                    req_prefetch_hit[0] <= s7_s0_w1_hit;
                                    req_prefetch_hit[1] <= 1'b0;
                                end
                            end else begin
                                // 32-bit read finished: qualifies for speculative read-ahead
                                prefetch_eligible        <= 1'b1;
                                next_prefetch_allowed    <= 1'b1;
                                prefetch_valid           <= 1'b0;
                                prefetch_word0_avail     <= 1'b0;
                                prefetch_word1_avail     <= 1'b0;
                                req_prefetch_hit         <= 2'b00;
                            end
`else
                            prefetch_eligible     <= 1'b1;
                            next_prefetch_allowed <= 1'b1;
`endif
                        end else begin
                            prefetch_eligible     <= 1'b0;
                            prefetch_valid        <= 1'b0;
`ifdef ENABLE_16BIT_PREFETCH
                            prefetch_word0_avail  <= 1'b0;
                            prefetch_word1_avail  <= 1'b0;
`endif
                            next_prefetch_allowed <= 1'b0;
                            req_prefetch_hit      <= 2'b00;
                        end
                        state <= STATE_WAIT_ACTIVE_REQUEST;
                    end else begin
                        // Subsequent sub-cycle required for normal partial transfer
                        address <= subcycle_next_addr;
                        size    <= subcycle_next_size;
                        state   <= STATE_WAIT_BUS_CYCLE_START;
                    end
                end
            end

            // -----------------------------------------------------------------
            // STATE 9: INTERNAL_FINISH
            // Pipelined completion state for internal Virtual Zorro card.
            // Waits for z2_access_ready from zorro_device Wishbone subsystem,
            // advances queue slot pointer, and returns to idle.
            // -----------------------------------------------------------------
            state[STATE_BIT_INTERNAL_FINISH]: begin
                if (z2_access_ready) begin
                    z2_access_valid <= 1'b0;

                    if (increment_execute_slot_pointer) begin
                        current_execute_slot          <= current_execute_slot + 1'd1;
                        current_execute_slot_addr     <= current_execute_slot_addr + 1'd1;
                        current_execute_slot_complete <= current_execute_slot_complete + 1'd1;
                        current_execute_slot_ctrl     <= current_execute_slot_ctrl + 1'd1;
                        current_execute_slot_bus      <= current_execute_slot_bus + 1'd1;
                    end

                    state <= STATE_WAIT_ACTIVE_REQUEST;
                end
            end

            default: state <= STATE_WAIT_ACTIVE_REQUEST;
        endcase

        // Asynchronous Request Prefetch Match Detection:
        // Evaluates immediately when a new request is posted by Pi interface.
        // Decoupled per-slot to eliminate cross-slot multiplexer delay.
        if (new_req_valid) begin
            if (!new_req_slot)
                req_prefetch_hit[0] <= req_prefetch_qual && slot0_addr_match;
            else
                req_prefetch_hit[1] <= req_prefetch_qual && slot1_addr_match;
        end

        // Bus loss or Reset safety: flush prefetch cache
        if (!request_bm || reset_sync || !is_bm) begin
            prefetch_valid           <= 1'b0;
`ifdef ENABLE_16BIT_PREFETCH
            prefetch_word0_avail     <= 1'b0;
            prefetch_word1_avail     <= 1'b0;
`endif
            prefetch_eligible        <= 1'b0;
            next_prefetch_allowed    <= 1'b0;
            chained_prefetch_allowed <= 1'b0;
            req_prefetch_hit         <= 2'b00;
        end

        // Hard reset initialization of internal Zorro transfer registers & counters
        if (reset_sync || drive_reset || counter_clear) begin
            diag_cycle_count <= 8'd0;
        end

        if (reset_sync || drive_reset) begin
            z2_access_valid   <= 1'b0;
            z2_access_addr    <= 24'd0;
            z2_access_wr_data <= 32'd0;
            z2_access_size      <= 2'd0;
            z2_access_wr        <= 1'b0;
        end
    end

    // Prefetch telemetry counters (clocked synchronously via single-cycle strobe pulses)
    always @(posedge clk) begin
        if (reset_sync || drive_reset || counter_clear) begin
            r_prefetch_launch_count <= 32'd0;
            r_prefetch_hit_count    <= 32'd0;
        end else begin
            if (prefetch_launch_pulse)
                r_prefetch_launch_count <= r_prefetch_launch_count + 32'd1;
            if (prefetch_hit_pulse)
                r_prefetch_hit_count    <= r_prefetch_hit_count + 32'd1;
        end
    end

endmodule
