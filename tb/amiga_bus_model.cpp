#include "amiga_bus_model.h"
#include "Vpistorm.h"
#include "Vpistorm_golden.h"
#include <iostream>
#include <iomanip>

template <typename TDut>
AmigaBusModelT<TDut>::AmigaBusModelT(TDut* dut, M68kSpeedGrade grade)
    : dut_(dut)
{
    timing_checker_ = std::make_unique<M68kTimingChecker>(M68kTimingProfile::get(grade));

    // Allocate 16MB virtual RAM
    ram_.resize(16 * 1024 * 1024, 0);

    // Fill some test memory patterns
    for (size_t i = 0; i < ram_.size(); i += 4) {
        ram_[i + 0] = (i >> 24) & 0xFF;
        ram_[i + 1] = (i >> 16) & 0xFF;
        ram_[i + 2] = (i >> 8) & 0xFF;
        ram_[i + 3] = i & 0xFF;
    }

    reset();
}

template <typename TDut>
AmigaBusModelT<TDut>::~AmigaBusModelT() = default;

template <typename TDut>
void AmigaBusModelT<TDut>::set_timing_profile(const M68kTimingProfile& profile) {
    timing_checker_->set_profile(profile);
}

template <typename TDut>
void AmigaBusModelT<TDut>::reset() {
    latched_da_ = 0;
    latched_address_ = 0;
    prev_addr_le_ = false;
    prev_as_active_ = false;
    prev_ds_active_ = false;
    prev_rw_read_ = true;
    prev_data_out_active_ = false;
    prev_dsack_n_ = 3;
    pcb_latch_.reset();
    pcb_data_switch_.reset();
    wait_states_ = 0;
    current_wait_cnt_ = 0;
    inject_berr_once_ = false;
    bg_delay_cnt_ = 0;
    in_cycle_ = false;

    total_bus_cycles_ = 0;
    read_cycles_ = 0;
    write_cycles_ = 0;

    reset_driven_ = false;
    halt_driven_ = false;
    int2_driven_ = false;
    int6_driven_ = false;

    // Reset Amiga inputs to DUT
    dut_->MC_AS_n_IN = 1;
    dut_->MC_BG_n = 1;
    dut_->MC_DSACK_n = 3; // 2'b11 (inactive)
    dut_->MC_BERR_n = 1;  // inactive
    dut_->MC_RESET_n_IN = 1;
    dut_->MC_HALT_n_IN = 1;
    dut_->MC_IPL_n = 7;   // level 0 (inverted: 3'b111 = no IRQ)
    dut_->KBRESET = 1;    // active-low: 1 = inactive
    dut_->DA_IN = 0;
    dut_->SPARE_IN = 0;
}

template <typename TDut>
void AmigaBusModelT<TDut>::set_port_width_region(uint32_t start, uint32_t size, PortWidth pw) {
    regions_.push_back({start, size, pw});
}

template <typename TDut>
PortWidth AmigaBusModelT<TDut>::get_port_width(uint32_t addr) const {
    for (const auto& reg : regions_) {
        if (addr >= reg.start && addr < (reg.start + reg.size)) {
            return reg.port_width;
        }
    }
    return default_port_width_;
}

template <typename TDut>
uint32_t AmigaBusModelT<TDut>::descramble_da_address(uint32_t da) {
    uint32_t addr = 0;
    if (da & (1u << 1))  addr |= (1u << 0);
    if (da & (1u << 0))  addr |= (1u << 1);
    if (da & (1u << 6))  addr |= (1u << 2);
    if (da & (1u << 7))  addr |= (1u << 3);
    if (da & (1u << 16)) addr |= (1u << 4);
    if (da & (1u << 17)) addr |= (1u << 5);
    if (da & (1u << 26)) addr |= (1u << 6);
    if (da & (1u << 27)) addr |= (1u << 7);
    if (da & (1u << 11)) addr |= (1u << 8);
    if (da & (1u << 10)) addr |= (1u << 9);
    if (da & (1u << 2))  addr |= (1u << 10);
    if (da & (1u << 3))  addr |= (1u << 11);
    if (da & (1u << 29)) addr |= (1u << 12);
    if (da & (1u << 28)) addr |= (1u << 13);
    if (da & (1u << 24)) addr |= (1u << 14);
    if (da & (1u << 25)) addr |= (1u << 15);
    if (da & (1u << 19)) addr |= (1u << 16);
    if (da & (1u << 18)) addr |= (1u << 17);
    if (da & (1u << 14)) addr |= (1u << 18);
    if (da & (1u << 15)) addr |= (1u << 19);
    if (da & (1u << 4))  addr |= (1u << 20);
    if (da & (1u << 5))  addr |= (1u << 21);
    if (da & (1u << 9))  addr |= (1u << 22);
    if (da & (1u << 8))  addr |= (1u << 23);
    if (da & (1u << 12)) addr |= (1u << 24);
    if (da & (1u << 13)) addr |= (1u << 25);
    if (da & (1u << 21)) addr |= (1u << 26);
    if (da & (1u << 20)) addr |= (1u << 27);
    if (da & (1u << 22)) addr |= (1u << 28);
    if (da & (1u << 23)) addr |= (1u << 29);
    if (da & (1u << 30)) addr |= (1u << 30);
    if (da & (1u << 31)) addr |= (1u << 31);
    return addr;
}

template <typename TDut>
uint32_t AmigaBusModelT<TDut>::scramble_address(uint32_t addr) {
    uint32_t da = 0;
    if (addr & (1u << 0))  da |= (1u << 1);
    if (addr & (1u << 1))  da |= (1u << 0);
    if (addr & (1u << 2))  da |= (1u << 6);
    if (addr & (1u << 3))  da |= (1u << 7);
    if (addr & (1u << 4))  da |= (1u << 16);
    if (addr & (1u << 5))  da |= (1u << 17);
    if (addr & (1u << 6))  da |= (1u << 26);
    if (addr & (1u << 7))  da |= (1u << 27);
    if (addr & (1u << 8))  da |= (1u << 11);
    if (addr & (1u << 9))  da |= (1u << 10);
    if (addr & (1u << 10)) da |= (1u << 2);
    if (addr & (1u << 11)) da |= (1u << 3);
    if (addr & (1u << 12)) da |= (1u << 29);
    if (addr & (1u << 13)) da |= (1u << 28);
    if (addr & (1u << 14)) da |= (1u << 24);
    if (addr & (1u << 15)) da |= (1u << 25);
    if (addr & (1u << 16)) da |= (1u << 19);
    if (addr & (1u << 17)) da |= (1u << 18);
    if (addr & (1u << 18)) da |= (1u << 14);
    if (addr & (1u << 19)) da |= (1u << 15);
    if (addr & (1u << 20)) da |= (1u << 4);
    if (addr & (1u << 21)) da |= (1u << 5);
    if (addr & (1u << 22)) da |= (1u << 9);
    if (addr & (1u << 23)) da |= (1u << 8);
    if (addr & (1u << 24)) da |= (1u << 12);
    if (addr & (1u << 25)) da |= (1u << 13);
    if (addr & (1u << 26)) da |= (1u << 21);
    if (addr & (1u << 27)) da |= (1u << 20);
    if (addr & (1u << 28)) da |= (1u << 22);
    if (addr & (1u << 29)) da |= (1u << 23);
    if (addr & (1u << 30)) da |= (1u << 30);
    if (addr & (1u << 31)) da |= (1u << 31);
    return da;
}

template <typename TDut>
uint8_t AmigaBusModelT<TDut>::mem_read_8(uint32_t addr) {
    return ram_[addr % ram_.size()];
}

template <typename TDut>
uint16_t AmigaBusModelT<TDut>::mem_read_16(uint32_t addr) {
    uint32_t a = addr % ram_.size();
    return (uint16_t(ram_[a]) << 8) | uint16_t(ram_[(a + 1) % ram_.size()]);
}

template <typename TDut>
uint32_t AmigaBusModelT<TDut>::mem_read_32(uint32_t addr) {
    uint32_t a = addr % ram_.size();
    return (uint32_t(ram_[a]) << 24) |
           (uint32_t(ram_[(a + 1) % ram_.size()]) << 16) |
           (uint32_t(ram_[(a + 2) % ram_.size()]) << 8) |
           uint32_t(ram_[(a + 3) % ram_.size()]);
}

template <typename TDut>
void AmigaBusModelT<TDut>::mem_write_8(uint32_t addr, uint8_t val) {
    ram_[addr % ram_.size()] = val;
}

template <typename TDut>
void AmigaBusModelT<TDut>::mem_write_16(uint32_t addr, uint16_t val) {
    uint32_t a = addr % ram_.size();
    ram_[a] = (val >> 8) & 0xFF;
    ram_[(a + 1) % ram_.size()] = val & 0xFF;
}

template <typename TDut>
void AmigaBusModelT<TDut>::mem_write_32(uint32_t addr, uint32_t val) {
    uint32_t a = addr % ram_.size();
    ram_[a] = (val >> 24) & 0xFF;
    ram_[(a + 1) % ram_.size()] = (val >> 16) & 0xFF;
    ram_[(a + 2) % ram_.size()] = (val >> 8) & 0xFF;
    ram_[(a + 3) % ram_.size()] = val & 0xFF;
}

template <typename TDut>
void AmigaBusModelT<TDut>::set_ipl(uint8_t level) {
    uint8_t l = level & 7;
    // MC_IPL_n is active low: 0 -> 111, 7 -> 000
    dut_->MC_IPL_n = (~l) & 7;
}

template <typename TDut>
void AmigaBusModelT<TDut>::set_external_reset(bool active) {
    dut_->MC_RESET_n_IN = active ? 0 : 1;
}

template <typename TDut>
void AmigaBusModelT<TDut>::set_external_halt(bool active) {
    dut_->MC_HALT_n_IN = active ? 0 : 1;
}

template <typename TDut>
void AmigaBusModelT<TDut>::set_kbreset(bool active) {
    dut_->KBRESET = active ? 0 : 1; // active-low: 0 when pressed
}

template <typename TDut>
void AmigaBusModelT<TDut>::update_on_pll_clock(double now_ns) {
    // Monitor open-drain outputs driven by DUT
    if (dut_->MC_RESET_n_OE) reset_driven_ = true;
    if (dut_->MC_HALT_n_OE)  halt_driven_ = true;
    if (dut_->INT2_n_OE)     int2_driven_ = true;
    if (dut_->INT6_n_OE)     int6_driven_ = true;

    // 74LVC573A Address Latch modeling (4x Octal Latches)
    pcb_latch_.update(dut_->DA_OUT, dut_->ADDR_LE != 0, dut_->ADDR_OE_n != 0, now_ns);
    uint32_t prev_addr = latched_address_;
    latched_da_ = pcb_latch_.get_q();
    latched_address_ = descramble_da_address(latched_da_);
    if (latched_address_ != prev_addr && timing_checker_) {
        timing_checker_->on_addr_change(latched_address_, dut_->MC_FC_OUT, dut_->MC_SIZE_OUT, now_ns);
    }
    prev_addr_le_ = (dut_->ADDR_LE != 0);

    // 74CBTD3384 Data Bus Switch modeling (4x 10-bit switches with level shifters)
    bool data_out_active = (!dut_->DATA_OE_n && !dut_->MC_RW_OUT);
    pcb_data_switch_.update_a_to_b(dut_->DA_OUT, dut_->DATA_OE_n != 0, now_ns);
    if (data_out_active != prev_data_out_active_) {
        prev_data_out_active_ = data_out_active;
        if (timing_checker_) {
            if (data_out_active) {
                timing_checker_->on_data_out_change(pcb_data_switch_.get_out(), now_ns);
            } else {
                timing_checker_->on_data_out_disabled(now_ns);
            }
        }
    }

    // Continuous fine-grain edge tracking for MC68020 AC timing checker
    bool rw_read = (dut_->MC_RW_OUT != 0);
    if (rw_read != prev_rw_read_) {
        prev_rw_read_ = rw_read;
        if (timing_checker_) timing_checker_->on_rw_change(rw_read, now_ns);
    }

    bool as_active = (dut_->MC_AS_n_OE && !dut_->MC_AS_n_OUT);
    if (as_active != prev_as_active_) {
        prev_as_active_ = as_active;
        if (timing_checker_) timing_checker_->on_as_change(as_active, now_ns);
    }

    bool ds_active = (dut_->MC_DS_n_OE && !dut_->MC_DS_n_OUT);
    if (ds_active != prev_ds_active_) {
        prev_ds_active_ = ds_active;
        if (timing_checker_) timing_checker_->on_ds_change(ds_active, now_ns);
    }

    uint8_t dsack_n = dut_->MC_DSACK_n;
    if (dsack_n != prev_dsack_n_) {
        prev_dsack_n_ = dsack_n;
        if (timing_checker_) timing_checker_->on_dsack_change(dsack_n, now_ns);
    }

    // Bus master arbitration
    bool br_active = (dut_->MC_BR_n_OE && !dut_->MC_BR_n_OUT);
    if (br_active) {
        if (bg_delay_cnt_ < 4) {
            bg_delay_cnt_++;
        } else {
            dut_->MC_BG_n = 0; // Grant bus to PiStorm
        }
    } else {
        bg_delay_cnt_ = 0;
        dut_->MC_BG_n = 1;
    }
}

template <typename TDut>
void AmigaBusModelT<TDut>::update_on_mc_clk_edge(bool rising, double now_ns) {
    if (timing_checker_) {
        timing_checker_->on_clk_edge(rising, now_ns);
    }

    // Amiga bus cycles are synchronous to MC_CLK
    bool as_active = (dut_->MC_AS_n_OE && !dut_->MC_AS_n_OUT);
    bool ds_active = (dut_->MC_DS_n_OE && !dut_->MC_DS_n_OUT);

    if (as_active) {
        if (!in_cycle_) {
            // Bus cycle started
            in_cycle_ = true;
            cycle_addr_ = latched_address_;
            cycle_rw_ = dut_->MC_RW_OUT != 0;
            cycle_size_ = dut_->MC_SIZE_OUT;
            current_wait_cnt_ = 0;
            total_bus_cycles_++;
            if (cycle_rw_) read_cycles_++; else write_cycles_++;
        }

        // Advance wait states on MC_CLK falling edge (when 68020 samples DSACK)
        if (!rising) {
            if (current_wait_cnt_ < wait_states_) {
                current_wait_cnt_++;
            } else {
                // Termination ready
                if (inject_berr_once_ || berr_asserted_) {
                    berr_asserted_ = true;
                    inject_berr_once_ = false;
                    dut_->MC_BERR_n = 0;
                    dut_->MC_DSACK_n = 3; // Do not assert DSACK on BERR
                } else {
                    dut_->MC_BERR_n = 1;
                    PortWidth pw = get_port_width(cycle_addr_);
                    switch (pw) {
                        case PortWidth::PORT_32BIT: dut_->MC_DSACK_n = 0; break; // 2'b00
                        case PortWidth::PORT_16BIT: dut_->MC_DSACK_n = 1; break; // 2'b01
                        case PortWidth::PORT_8BIT:  dut_->MC_DSACK_n = 2; break; // 2'b10
                    }
                    if (cycle_rw_ && timing_checker_) {
                        timing_checker_->on_sample_read_data(now_ns);
                    }
                }
            }
        }

        // Drive read data when reading
        if (cycle_rw_) {
            PortWidth pw = get_port_width(cycle_addr_);
            uint32_t a = cycle_addr_ % ram_.size();
            uint32_t data = 0;

            if (pw == PortWidth::PORT_32BIT) {
                // 32-bit port aligns to 4-byte boundary
                uint32_t base = a & ~3u;
                data = (uint32_t(ram_[base]) << 24) |
                       (uint32_t(ram_[(base + 1) % ram_.size()]) << 16) |
                       (uint32_t(ram_[(base + 2) % ram_.size()]) << 8) |
                       uint32_t(ram_[(base + 3) % ram_.size()]);
            } else if (pw == PortWidth::PORT_16BIT) {
                // 16-bit port aligns to 2-byte boundary, drives D31..D16
                uint32_t base = a & ~1u;
                uint16_t w = (uint16_t(ram_[base]) << 8) | uint16_t(ram_[(base + 1) % ram_.size()]);
                data = (uint32_t(w) << 16);
            } else {
                // 8-bit port drives D31..D24
                data = (uint32_t(ram_[a]) << 24);
            }
            dut_->DA_IN = data;
            if (timing_checker_) {
                timing_checker_->on_data_in_change(data, now_ns);
            }
        } else {
            // Write cycle: latch write data when DS is active
            if (ds_active && (dut_->MC_DSACK_n != 3 || dut_->MC_BERR_n == 0)) {
                PortWidth pw = get_port_width(cycle_addr_);
                uint32_t wdata = dut_->DA_OUT;
                uint32_t a = cycle_addr_ % ram_.size();

                int bytes_req = 1;
                switch (cycle_size_) {
                    case 1: bytes_req = 1; break; // Byte (01)
                    case 2: bytes_req = 2; break; // Word (10)
                    case 3: bytes_req = 3; break; // 3 Bytes (11)
                    case 0: bytes_req = 4; break; // Longword (00)
                }

                if (pw == PortWidth::PORT_32BIT) {
                    int max_in_cycle = 4 - (a & 3);
                    int n = std::min(bytes_req, max_in_cycle);
                    for (int i = 0; i < n; i++) {
                        uint32_t target = (a + i) % ram_.size();
                        int lane = (a + i) & 3;
                        ram_[target] = (wdata >> (24 - 8 * lane)) & 0xFF;
                    }
                } else if (pw == PortWidth::PORT_16BIT) {
                    int max_in_cycle = 2 - (a & 1);
                    int n = std::min(bytes_req, max_in_cycle);
                    for (int i = 0; i < n; i++) {
                        uint32_t target = (a + i) % ram_.size();
                        int lane = (a + i) & 1;
                        ram_[target] = (wdata >> (24 - 8 * lane)) & 0xFF;
                    }
                } else {
                    // 8-bit port drives D31..D24
                    ram_[a] = (wdata >> 24) & 0xFF;
                }
            }
        }
    } else {
        // AS is negated: end of cycle
        in_cycle_ = false;
        berr_asserted_ = false;
        dut_->MC_DSACK_n = 3; // 2'b11
        dut_->MC_BERR_n = 1;
        dut_->DA_IN = 0;
    }
}

// Explicit template instantiations
template class AmigaBusModelT<Vpistorm>;
template class AmigaBusModelT<Vpistorm_golden>;
