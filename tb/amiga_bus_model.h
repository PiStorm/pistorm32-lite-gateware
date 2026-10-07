#ifndef AMIGA_BUS_MODEL_H
#define AMIGA_BUS_MODEL_H

#include <cstdint>
#include <vector>
#include <map>
#include <functional>

#include "m68k_timing_profile.h"
#include "m68k_timing_checker.h"
#include "pcb_components.h"
#include <memory>

// Forward declaration of Verilated DUTs
class Vpistorm;
class Vpistorm_golden;

enum class PortWidth {
    PORT_32BIT = 0, // DSACK_n = 2'b00
    PORT_16BIT = 1, // DSACK_n = 2'b01
    PORT_8BIT  = 2  // DSACK_n = 2'b10
};

template <typename TDut = Vpistorm>
class AmigaBusModelT {
public:
    explicit AmigaBusModelT(TDut* dut, M68kSpeedGrade grade = M68kSpeedGrade::GRADE_A1200_14MHZ);
    ~AmigaBusModelT();

    void reset();

    // Timing profile & checker
    void set_timing_profile(const M68kTimingProfile& profile);
    M68kTimingChecker* get_timing_checker() { return timing_checker_.get(); }

    // PCB Hardware components (74LVC573A & 74CBTD3384)
    PcbLatch74LVC573A* get_pcb_latch() { return &pcb_latch_; }
    PcbSwitch74CBTD3384* get_pcb_data_switch() { return &pcb_data_switch_; }

    // Called on each simulation half-cycle / tick with current physical time (ns)
    void update_on_pll_clock(double now_ns);
    void update_on_mc_clk_edge(bool rising, double now_ns);

    // Address descrambling matching the PiStorm32-lite PCB latch
    static uint32_t descramble_da_address(uint32_t da);
    static uint32_t scramble_address(uint32_t addr);

    // Memory configuration
    void set_port_width_default(PortWidth pw) { default_port_width_ = pw; }
    void set_port_width_region(uint32_t start, uint32_t size, PortWidth pw);
    void set_wait_states(int ws) { wait_states_ = ws; }
    void set_inject_berr_once(bool enable) { inject_berr_once_ = enable; }

    // Memory direct access for verification
    uint8_t  mem_read_8(uint32_t addr);
    uint16_t mem_read_16(uint32_t addr);
    uint32_t mem_read_32(uint32_t addr);
    void mem_write_8(uint32_t addr, uint8_t val);
    void mem_write_16(uint32_t addr, uint16_t val);
    void mem_write_32(uint32_t addr, uint32_t val);

    // Amiga control / status signals
    void set_ipl(uint8_t level); // 0 = no int, 1..7 = int level
    void set_external_reset(bool active); // active = pull MC_RESET_n_IN low
    void set_external_halt(bool active);  // active = pull MC_HALT_n_IN low
    void set_kbreset(bool active);

    // Bus monitor / stats
    uint64_t get_total_bus_cycles() const { return total_bus_cycles_; }
    uint64_t get_read_cycles() const { return read_cycles_; }
    uint64_t get_write_cycles() const { return write_cycles_; }

    bool was_reset_driven() const { return reset_driven_; }
    bool was_halt_driven() const { return halt_driven_; }
    bool was_int2_driven() const { return int2_driven_; }
    bool was_int6_driven() const { return int6_driven_; }

    uint32_t get_last_latched_addr() const { return latched_address_; }

private:
    struct MemoryRegion {
        uint32_t start;
        uint32_t size;
        PortWidth port_width;
    };

    PortWidth get_port_width(uint32_t addr) const;

    TDut* dut_;

    std::vector<uint8_t> ram_;
    std::vector<MemoryRegion> regions_;
    PortWidth default_port_width_ = PortWidth::PORT_32BIT;

    // Bus state
    uint32_t latched_da_ = 0;
    uint32_t latched_address_ = 0;
    bool prev_addr_le_ = false;
    bool prev_as_active_ = false;
    bool prev_ds_active_ = false;
    bool prev_rw_read_ = true;
    bool prev_data_out_active_ = false;
    uint8_t prev_dsack_n_ = 3;

    int wait_states_ = 0;
    int current_wait_cnt_ = 0;
    bool inject_berr_once_ = false;
    bool berr_asserted_ = false;

    // Arbitration
    int bg_delay_cnt_ = 0;

    // Cycle tracking
    bool in_cycle_ = false;
    uint32_t cycle_addr_ = 0;
    bool cycle_rw_ = true;
    uint8_t cycle_size_ = 0;

    // Amiga status
    bool reset_driven_ = false;
    bool halt_driven_ = false;
    bool int2_driven_ = false;
    bool int6_driven_ = false;

    uint64_t total_bus_cycles_ = 0;
    uint64_t read_cycles_ = 0;
    uint64_t write_cycles_ = 0;

    std::unique_ptr<M68kTimingChecker> timing_checker_;
    PcbLatch74LVC573A pcb_latch_;
    PcbSwitch74CBTD3384 pcb_data_switch_;
};

using AmigaBusModel = AmigaBusModelT<Vpistorm>;
using AmigaBusModelGolden = AmigaBusModelT<Vpistorm_golden>;

#endif // AMIGA_BUS_MODEL_H
