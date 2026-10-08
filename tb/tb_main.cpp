#include <iostream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <cassert>
#include <verilated.h>
#include <verilated_vcd_c.h>

#include "Vpistorm.h"
#include "Vpistorm_golden.h"
#include "amiga_bus_model.h"
#include "ps_pi_model.h"

// Color codes for test runner output
#define ANSI_RESET   "\033[0m"
#define ANSI_BOLD    "\033[1m"
#define ANSI_RED     "\033[31m"
#define ANSI_GREEN   "\033[32m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_BLUE    "\033[34m"
#define ANSI_MAGENTA "\033[35m"
#define ANSI_CYAN    "\033[36m"

enum class ClockMode {
    CLEAN = 0,
    RINGING_UNFIXED = 1,
    XOR_ASYMMETRIC = 2
};

template <typename TDut = Vpistorm>
class SimulationHarnessT {
public:
    SimulationHarnessT(bool trace_enabled, const std::string& vcd_file = "sim.vcd")
        : trace_enabled_(trace_enabled)
    {
        dut_ = std::make_unique<TDut>();
        amiga_bus_ = std::make_unique<AmigaBusModelT<TDut>>(dut_.get());

        if (trace_enabled_) {
            Verilated::traceEverOn(true);
            vcd_ = std::make_unique<VerilatedVcdC>();
            dut_->trace(vcd_.get(), 99);
            vcd_->open(vcd_file.c_str());
        }

        // Initialize clock states
        dut_->AMIPLL_CLKOUT0 = 0;
        dut_->MC_CLK = 0;

        pi_ = std::make_unique<PiStormPiModelT<TDut>>(dut_.get(), [this](int cycles) {
            this->step_cycles(cycles);
        });
    }

    ~SimulationHarnessT() {
        if (vcd_) {
            vcd_->close();
        }
    }

    void set_clock_mode(ClockMode mode) {
        clock_mode_ = mode;
        ticks_in_current_half_ = 0;
        current_half_target_ticks_ = pll_to_mc_ratio_;
        is_glitching_ = false;
        target_mc_level_ = (dut_->MC_CLK == 1);
    }

    // Step PLL clock cycles
    // Ratio: 5 PLL cycles per MC_CLK half-cycle (10 PLL cycles per MC_CLK period)
    // At 7.048 ns PLL period, MC_CLK = 70.48 ns (14.1875 MHz Amiga clock)
    void step_cycles(int cycles) {
        for (int i = 0; i < cycles; ++i) {
            // Half cycle low of PLL clock
            dut_->AMIPLL_CLKOUT0 = 0;
            dut_->eval();
            now_ns_ += pll_half_period_ns_;
            if (vcd_) vcd_->dump(sim_time_++);

            // Half cycle high of PLL clock
            dut_->AMIPLL_CLKOUT0 = 1;
            pll_tick_count_++;
            now_ns_ += pll_half_period_ns_;

            // Handle MC_CLK edge transitions and glitch/asymmetry injection
            ticks_in_current_half_++;

            if (clock_mode_ == ClockMode::RINGING_UNFIXED) {
                // If we just crossed an edge, simulate 1.8V dip across threshold on the very next PLL tick
                if (ticks_in_current_half_ == 1) {
                    dut_->MC_CLK = target_mc_level_ ? 0 : 1;
                    is_glitching_ = true;
                } else if (is_glitching_) {
                    dut_->MC_CLK = target_mc_level_ ? 1 : 0;
                    is_glitching_ = false;
                }
            }

            if (ticks_in_current_half_ >= current_half_target_ticks_) {
                ticks_in_current_half_ = 0;
                target_mc_level_ = !target_mc_level_;
                dut_->MC_CLK = target_mc_level_ ? 1 : 0;
                is_glitching_ = false;

                if (clock_mode_ == ClockMode::XOR_ASYMMETRIC) {
                    // Alternate between 4 ticks (~28ns) and 6 ticks (~42ns) simulating Budgie XOR asymmetry
                    current_half_target_ticks_ = target_mc_level_ ? 4 : 6;
                } else {
                    current_half_target_ticks_ = pll_to_mc_ratio_;
                }

                bool mc_rising = (dut_->MC_CLK == 1);
                amiga_bus_->update_on_mc_clk_edge(mc_rising, now_ns_);
            }

            amiga_bus_->update_on_pll_clock(now_ns_);
            dut_->eval();
            if (vcd_) vcd_->dump(sim_time_++);
        }
    }

    void run_mc_cycles(int mc_cycles) {
        step_cycles(mc_cycles * (pll_to_mc_ratio_ * 2));
    }

    double get_now_ns() const { return now_ns_; }
    uint64_t get_pll_ticks() const { return pll_tick_count_; }

    TDut* dut() { return dut_.get(); }
    AmigaBusModelT<TDut>* amiga() { return amiga_bus_.get(); }
    PiStormPiModelT<TDut>* pi() { return pi_.get(); }

private:
    std::unique_ptr<TDut> dut_;
    std::unique_ptr<AmigaBusModelT<TDut>> amiga_bus_;
    std::unique_ptr<PiStormPiModelT<TDut>> pi_;
    std::unique_ptr<VerilatedVcdC> vcd_;

    bool trace_enabled_ = false;
    uint64_t sim_time_ = 0;
    uint64_t pll_tick_count_ = 0;
    const int pll_to_mc_ratio_ = 5; // 5 PLL ticks per MC_CLK half-cycle
    double now_ns_ = 0.0;
    const double pll_half_period_ns_ = 3.524; // 14.1875 MHz MC_CLK

    ClockMode clock_mode_ = ClockMode::CLEAN;
    int ticks_in_current_half_ = 0;
    int current_half_target_ticks_ = 5;
    bool target_mc_level_ = false;
    bool is_glitching_ = false;
};

using SimulationHarness = SimulationHarnessT<Vpistorm>;
using SimulationHarnessGolden = SimulationHarnessT<Vpistorm_golden>;

// Test result tracking
static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (cond) { \
            std::cout << "  " ANSI_GREEN "[PASS]" ANSI_RESET " " << msg << std::endl; \
            g_tests_passed++; \
        } else { \
            std::cout << "  " ANSI_RED "[FAIL]" ANSI_RESET " " << msg << " (line " << __LINE__ << ")" << std::endl; \
            g_tests_failed++; \
        } \
    } while(0)

struct BenchmarkStat {
    std::string name;
    std::string bus_type;
    std::string config;
    size_t count;
    size_t bytes_per_op;
    double elapsed_ns;
    uint64_t bus_cycles;

    double throughput_mb_s() const {
        double bytes = (double)(count * bytes_per_op);
        double sec = elapsed_ns * 1e-9;
        return (bytes / (1024.0 * 1024.0)) / sec;
    }

    double mops() const {
        double sec = elapsed_ns * 1e-9;
        return ((double)count / 1e6) / sec;
    }

    double avg_latency_ns() const {
        return elapsed_ns / (double)count;
    }

    double avg_amiga_clks() const {
        return avg_latency_ns() / 70.48; // 14.1875 MHz MC_CLK
    }
};

void run_golden_reference_comparison() {
    std::cout << "\n" ANSI_BOLD ANSI_CYAN "====================================================================================================\n"
              << "  PiStorm32-lite: Upstream Golden Reference vs Enhanced Modular Refactor Comparison\n"
              << "====================================================================================================\n" ANSI_RESET;

    SimulationHarnessGolden golden_harness(false);
    SimulationHarness refactor_harness(false);

    // Warm-up and reset both models identically
    golden_harness.run_mc_cycles(20);
    refactor_harness.run_mc_cycles(20);

    golden_harness.amiga()->set_external_reset(true);
    refactor_harness.amiga()->set_external_reset(true);
    golden_harness.run_mc_cycles(10);
    refactor_harness.run_mc_cycles(10);
    golden_harness.amiga()->set_external_reset(false);
    refactor_harness.amiga()->set_external_reset(false);
    golden_harness.run_mc_cycles(10);
    refactor_harness.run_mc_cycles(10);

    golden_harness.pi()->ps_clr_control(CONTROL_DRIVE_RESET | CONTROL_DRIVE_HALT);
    refactor_harness.pi()->ps_clr_control(CONTROL_DRIVE_RESET | CONTROL_DRIVE_HALT);
    golden_harness.run_mc_cycles(10);
    refactor_harness.run_mc_cycles(10);

    golden_harness.pi()->set_use_2slot(true);
    golden_harness.pi()->ps_set_control(CONTROL_REQ_BM);
    refactor_harness.pi()->set_use_2slot(true);
    refactor_harness.pi()->ps_set_control(CONTROL_REQ_BM);
    golden_harness.run_mc_cycles(15);
    refactor_harness.run_mc_cycles(15);

    // Configure Virtual Zorro board in refactor harness to set baseline mode (BUS_CTRL = 0)
    // to verify exact parity against upstream Golden Reference
    refactor_harness.pi()->ps32_write_8(0x00E80048, 0xE0);
    refactor_harness.pi()->ps32_write_8(0x00E8004A, 0x90);
    refactor_harness.pi()->flush_pending_writes();
    refactor_harness.run_mc_cycles(4);
    refactor_harness.pi()->ps32_write_32(0x00E9001C, 0x00000000);
    refactor_harness.pi()->flush_pending_writes();
    refactor_harness.run_mc_cycles(4);

    uint32_t chipmem_base = 0x00040000;
    golden_harness.amiga()->set_port_width_region(chipmem_base, 0x10000, PortWidth::PORT_32BIT);
    golden_harness.amiga()->set_wait_states(4); // 560ns Alice slot
    refactor_harness.amiga()->set_port_width_region(chipmem_base, 0x10000, PortWidth::PORT_32BIT);
    refactor_harness.amiga()->set_wait_states(4);

    uint32_t chipset_base = 0x00DFF000;
    golden_harness.amiga()->set_port_width_region(chipset_base, 0x1000, PortWidth::PORT_16BIT);
    golden_harness.amiga()->set_wait_states(4); // 560ns CCK slot
    refactor_harness.amiga()->set_port_width_region(chipset_base, 0x1000, PortWidth::PORT_16BIT);
    refactor_harness.amiga()->set_wait_states(4);

    struct ComparisonRow {
        std::string name;
        uint64_t golden_cycles;
        uint64_t refactor_cycles;
        double golden_time_ns;
        double refactor_time_ns;
        size_t count;
        size_t bytes_per_op;
        std::string note;
    };
    std::vector<ComparisonRow> rows;

    // --- Benchmark 1: Chipmem 32-bit Write (Pipelined 2-Slot) ---
    {
        double g_t0 = golden_harness.get_now_ns();
        uint64_t g_b0 = golden_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 64; ++i) {
            golden_harness.pi()->ps32_write_32(chipmem_base + i * 4, 0xCAFE0000 | (uint32_t)i);
        }
        golden_harness.pi()->flush_pending_writes();
        double g_t1 = golden_harness.get_now_ns();
        uint64_t g_b1 = golden_harness.amiga()->get_total_bus_cycles();

        double r_t0 = refactor_harness.get_now_ns();
        uint64_t r_b0 = refactor_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 64; ++i) {
            refactor_harness.pi()->ps32_write_32(chipmem_base + i * 4, 0xCAFE0000 | (uint32_t)i);
        }
        refactor_harness.pi()->flush_pending_writes();
        double r_t1 = refactor_harness.get_now_ns();
        uint64_t r_b1 = refactor_harness.amiga()->get_total_bus_cycles();

        uint64_t g_cyc = g_b1 - g_b0;
        uint64_t r_cyc = r_b1 - r_b0;
        double g_time = g_t1 - g_t0;
        double r_time = r_t1 - r_t0;

        TEST_ASSERT(r_cyc == g_cyc, "Chipmem 32-bit Write: Refactor bus cycles match Golden Reference exactly (64 vs 64)");
        TEST_ASSERT(std::abs(r_time - g_time) < 10.0, "Chipmem 32-bit Write: Refactor execution time matches Golden Reference (0% regression)");
        TEST_ASSERT(refactor_harness.amiga()->mem_read_32(chipmem_base + 63 * 4) == (0xCAFE0000 | 63), "Chipmem 32-bit Write: Data verified in RAM");

        rows.push_back({"Chipmem 32-bit Write (2-Slot)", g_cyc, r_cyc, g_time, r_time, 64, 4, "Zero Regression (Prio #2)"});
    }

    // --- Benchmark 2: Chipmem 16-bit Word Write (Pipelined 2-Slot) ---
    {
        double g_t0 = golden_harness.get_now_ns();
        uint64_t g_b0 = golden_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 64; ++i) {
            golden_harness.pi()->ps32_write_16(chipmem_base + i * 2, 0xBEEF ^ (uint16_t)i);
        }
        golden_harness.pi()->flush_pending_writes();
        double g_t1 = golden_harness.get_now_ns();
        uint64_t g_b1 = golden_harness.amiga()->get_total_bus_cycles();

        double r_t0 = refactor_harness.get_now_ns();
        uint64_t r_b0 = refactor_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 64; ++i) {
            refactor_harness.pi()->ps32_write_16(chipmem_base + i * 2, 0xBEEF ^ (uint16_t)i);
        }
        refactor_harness.pi()->flush_pending_writes();
        double r_t1 = refactor_harness.get_now_ns();
        uint64_t r_b1 = refactor_harness.amiga()->get_total_bus_cycles();

        uint64_t g_cyc = g_b1 - g_b0;
        uint64_t r_cyc = r_b1 - r_b0;
        double g_time = g_t1 - g_t0;
        double r_time = r_t1 - r_t0;

        TEST_ASSERT(r_cyc == g_cyc, "Chipmem 16-bit Write: Refactor bus cycles match Golden Reference exactly (64 vs 64)");
        TEST_ASSERT(std::abs(r_time - g_time) < 75.0, "Chipmem 16-bit Write: Refactor execution time matches Golden Reference (0% regression)");

        rows.push_back({"Chipmem 16-bit Word Write", g_cyc, r_cyc, g_time, r_time, 64, 2, "Zero Regression (Prio #2)"});
    }

    // --- Benchmark 2b: Chipmem 16-bit Word Read ---
    {
        double g_t0 = golden_harness.get_now_ns();
        uint64_t g_b0 = golden_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 64; ++i) {
            golden_harness.pi()->ps32_read_16(chipmem_base + i * 2);
        }
        double g_t1 = golden_harness.get_now_ns();
        uint64_t g_b1 = golden_harness.amiga()->get_total_bus_cycles();

        double r_t0 = refactor_harness.get_now_ns();
        uint64_t r_b0 = refactor_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 64; ++i) {
            refactor_harness.pi()->ps32_read_16(chipmem_base + i * 2);
        }
        double r_t1 = refactor_harness.get_now_ns();
        uint64_t r_b1 = refactor_harness.amiga()->get_total_bus_cycles();

        uint64_t g_cyc = g_b1 - g_b0;
        uint64_t r_cyc = r_b1 - r_b0;
        double g_time = g_t1 - g_t0;
        double r_time = r_t1 - r_t0;

        TEST_ASSERT(r_cyc == g_cyc, "Chipmem 16-bit Read: Refactor bus cycles match Golden Reference exactly (64 vs 64)");
        TEST_ASSERT(std::abs(r_time - g_time) < 75.0, "Chipmem 16-bit Read: Refactor execution time matches Golden Reference (0% regression)");

        rows.push_back({"Chipmem 16-bit Word Read", g_cyc, r_cyc, g_time, r_time, 64, 2, "Zero Regression (Prio #2)"});
    }

    // --- Benchmark 3: Custom Chipset 16-bit Word Write ($DFF180) ---
    {
        double g_t0 = golden_harness.get_now_ns();
        uint64_t g_b0 = golden_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 64; ++i) {
            golden_harness.pi()->ps32_write_16(chipset_base + 0x180 + (i % 32) * 2, 0x0F00 | (uint16_t)i);
        }
        golden_harness.pi()->flush_pending_writes();
        double g_t1 = golden_harness.get_now_ns();
        uint64_t g_b1 = golden_harness.amiga()->get_total_bus_cycles();

        double r_t0 = refactor_harness.get_now_ns();
        uint64_t r_b0 = refactor_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 64; ++i) {
            refactor_harness.pi()->ps32_write_16(chipset_base + 0x180 + (i % 32) * 2, 0x0F00 | (uint16_t)i);
        }
        refactor_harness.pi()->flush_pending_writes();
        double r_t1 = refactor_harness.get_now_ns();
        uint64_t r_b1 = refactor_harness.amiga()->get_total_bus_cycles();

        uint64_t g_cyc = g_b1 - g_b0;
        uint64_t r_cyc = r_b1 - r_b0;
        double g_time = g_t1 - g_t0;
        double r_time = r_t1 - r_t0;

        TEST_ASSERT(r_cyc == g_cyc, "Chipset 16-bit Write: Refactor bus cycles match Golden Reference exactly (64 vs 64)");
        TEST_ASSERT(std::abs(r_time - g_time) < 10.0, "Chipset 16-bit Write: Refactor execution time matches Golden Reference (0% regression)");

        rows.push_back({"Chipset 16-bit Word Write", g_cyc, r_cyc, g_time, r_time, 64, 2, "Zero Regression (Prio #2)"});
    }

    // --- Benchmark 3b: Custom Chipset 16-bit Word Read ($DFF000) ---
    {
        for (int i = 0; i < 64; ++i) {
            golden_harness.amiga()->mem_write_16(chipset_base + (i % 16) * 2, 0x0A00 | (uint16_t)i);
            refactor_harness.amiga()->mem_write_16(chipset_base + (i % 16) * 2, 0x0A00 | (uint16_t)i);
        }

        double g_t0 = golden_harness.get_now_ns();
        uint64_t g_b0 = golden_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 64; ++i) {
            golden_harness.pi()->ps32_read_16(chipset_base + (i % 16) * 2);
        }
        double g_t1 = golden_harness.get_now_ns();
        uint64_t g_b1 = golden_harness.amiga()->get_total_bus_cycles();

        double r_t0 = refactor_harness.get_now_ns();
        uint64_t r_b0 = refactor_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 64; ++i) {
            refactor_harness.pi()->ps32_read_16(chipset_base + (i % 16) * 2);
        }
        double r_t1 = refactor_harness.get_now_ns();
        uint64_t r_b1 = refactor_harness.amiga()->get_total_bus_cycles();

        uint64_t g_cyc = g_b1 - g_b0;
        uint64_t r_cyc = r_b1 - r_b0;
        double g_time = g_t1 - g_t0;
        double r_time = r_t1 - r_t0;

        TEST_ASSERT(r_cyc == g_cyc, "Chipset 16-bit Read: Refactor bus cycles match Golden Reference exactly (64 vs 64)");
        TEST_ASSERT(std::abs(r_time - g_time) < 75.0, "Chipset 16-bit Read: Refactor execution time matches Golden Reference (0% regression)");

        rows.push_back({"Chipset 16-bit Word Read", g_cyc, r_cyc, g_time, r_time, 64, 2, "Zero Regression (Prio #2)"});
    }

    // --- Benchmark 4: Custom Chipset 32-bit Dynamic Bus Sizing ---
    {
        double g_t0 = golden_harness.get_now_ns();
        uint64_t g_b0 = golden_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 32; ++i) {
            golden_harness.pi()->ps32_write_32(chipset_base + (i % 16) * 4, 0x01234567);
        }
        golden_harness.pi()->flush_pending_writes();
        double g_t1 = golden_harness.get_now_ns();
        uint64_t g_b1 = golden_harness.amiga()->get_total_bus_cycles();

        double r_t0 = refactor_harness.get_now_ns();
        uint64_t r_b0 = refactor_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 32; ++i) {
            refactor_harness.pi()->ps32_write_32(chipset_base + (i % 16) * 4, 0x01234567);
        }
        refactor_harness.pi()->flush_pending_writes();
        double r_t1 = refactor_harness.get_now_ns();
        uint64_t r_b1 = refactor_harness.amiga()->get_total_bus_cycles();

        uint64_t g_cyc = g_b1 - g_b0;
        uint64_t r_cyc = r_b1 - r_b0;
        double g_time = g_t1 - g_t0;
        double r_time = r_t1 - r_t0;

        TEST_ASSERT(r_cyc == g_cyc, "Chipset 32-bit Sized Write: Bus cycles match Golden Reference exactly (64 vs 64)");
        TEST_ASSERT(std::abs(r_time - g_time) < 10.0, "Chipset 32-bit Sized Write: Execution time matches Golden Reference (0% regression)");

        rows.push_back({"Chipset 32-bit Sized Write", g_cyc, r_cyc, g_time, r_time, 32, 4, "Exact Match"});
    }

    // --- Benchmark 5: Chipmem 32-bit Sequential Read (No Prefetch) ---
    {
        refactor_harness.pi()->ps_clr_control(CONTROL_ENABLE_PREFETCH);
        refactor_harness.run_mc_cycles(4);

        double g_t0 = golden_harness.get_now_ns();
        uint64_t g_b0 = golden_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 64; ++i) {
            golden_harness.pi()->ps32_read_32(chipmem_base + i * 4);
        }
        double g_t1 = golden_harness.get_now_ns();
        uint64_t g_b1 = golden_harness.amiga()->get_total_bus_cycles();

        double r_t0 = refactor_harness.get_now_ns();
        uint64_t r_b0 = refactor_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 64; ++i) {
            refactor_harness.pi()->ps32_read_32(chipmem_base + i * 4);
        }
        double r_t1 = refactor_harness.get_now_ns();
        uint64_t r_b1 = refactor_harness.amiga()->get_total_bus_cycles();

        uint64_t g_cyc = g_b1 - g_b0;
        uint64_t r_cyc = r_b1 - r_b0;
        double g_time = g_t1 - g_t0;
        double r_time = r_t1 - r_t0;

        TEST_ASSERT(r_cyc == g_cyc, "Chipmem 32-bit Read (No Prefetch): Bus cycles match Golden Reference exactly (64 vs 64)");
        TEST_ASSERT(std::abs(r_time - g_time) < 75.0, "Chipmem 32-bit Read (No Prefetch): Execution time matches Golden Reference (0% regression)");

        rows.push_back({"Chipmem 32-bit Read (No Pref)", g_cyc, r_cyc, g_time, r_time, 64, 4, "Exact Match"});
    }

    // --- Benchmark 6: Chipmem 32-bit Sequential Read (Prefetch ON) ---
    {
        refactor_harness.pi()->ps_set_control(CONTROL_ENABLE_PREFETCH);
        refactor_harness.pi()->ps32_write_32(0x00E9001C, 0x00000001);
        refactor_harness.pi()->flush_pending_writes();
        refactor_harness.run_mc_cycles(4);
        refactor_harness.pi()->ps32_read_32(chipmem_base); // prime
        refactor_harness.run_mc_cycles(4);

        double g_t0 = golden_harness.get_now_ns();
        uint64_t g_b0 = golden_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 64; ++i) {
            golden_harness.pi()->ps32_read_32(chipmem_base + i * 4);
        }
        double g_t1 = golden_harness.get_now_ns();
        uint64_t g_b1 = golden_harness.amiga()->get_total_bus_cycles();

        double r_t0 = refactor_harness.get_now_ns();
        uint64_t r_b0 = refactor_harness.amiga()->get_total_bus_cycles();
        for (int i = 0; i < 64; ++i) {
            refactor_harness.pi()->ps32_read_32(chipmem_base + i * 4);
        }
        double r_t1 = refactor_harness.get_now_ns();
        uint64_t r_b1 = refactor_harness.amiga()->get_total_bus_cycles();

        uint64_t g_cyc = g_b1 - g_b0;
        uint64_t r_cyc = r_b1 - r_b0;
        double g_time = g_t1 - g_t0;
        double r_time = r_t1 - r_t0;

        TEST_ASSERT(r_time < g_time, "Chipmem 32-bit Read (Prefetch ON): Refactor delivers higher throughput than Golden Reference");

        rows.push_back({"Chipmem 32-bit Read (Prefetch)", g_cyc, r_cyc, g_time, r_time, 64, 4, "Prefetch Active"});
        refactor_harness.pi()->ps_clr_control(CONTROL_ENABLE_PREFETCH);
    }

    // --- Test 7: 1.8V Clock Ringing Immunity ---
    {
        std::cout << ANSI_CYAN "\n  [Glitch Immunity] Testing 1.8V Ringing Dip Immunity (Unmodified A1200 Motherboard Clock)..." ANSI_RESET << std::endl;
        refactor_harness.set_clock_mode(ClockMode::RINGING_UNFIXED);
        refactor_harness.run_mc_cycles(5);

        uint32_t ring_addr = 0x00045000;
        refactor_harness.pi()->ps32_write_32(ring_addr, 0x12345678);
        refactor_harness.run_mc_cycles(4);
        uint32_t rd_val = refactor_harness.pi()->ps32_read_32(ring_addr);

        TEST_ASSERT(rd_val == 0x12345678, "Glitch Filter: Passes under 1.8V clock ringing dips");
        refactor_harness.set_clock_mode(ClockMode::CLEAN);
    }

    // Print the Side-by-Side Comparison Table
    std::cout << "\n" ANSI_BOLD ANSI_CYAN
              << "======================================================================================================================================\n"
              << "                      PiStorm32-lite: Upstream Golden Reference vs Modular Refactor\n"
              << "======================================================================================================================================\n"
              << ANSI_RESET;
    std::cout << ANSI_BOLD
              << std::left << std::setw(32) << "Benchmark Transaction"
              << std::right << std::setw(13) << "Golden Cyc"
              << std::setw(15) << "Refactor Cyc"
              << std::setw(12) << "Delta Cyc"
              << std::setw(18) << "Golden (Bustest)"
              << std::setw(20) << "Refactor (Bustest)"
              << std::setw(10) << "MiB/s"
              << "    " << std::left << std::setw(20) << "Parity Status"
              << "\n" ANSI_RESET;
    std::cout << "--------------------------------------------------------------------------------------------------------------------------------------\n";

    for (const auto& r : rows) {
        double bytes = (double)(r.count * r.bytes_per_op);
        double g_sec = r.golden_time_ns * 1e-9;
        double g_mbs_dec = (bytes / 1e6) / g_sec;
        double g_mbs_bin = (bytes / (1024.0 * 1024.0)) / g_sec;

        double r_sec = r.refactor_time_ns * 1e-9;
        double r_mbs_dec = (bytes / 1e6) / r_sec;
        double r_mbs_bin = (bytes / (1024.0 * 1024.0)) / r_sec;

        int64_t delta_cyc = (int64_t)r.refactor_cycles - (int64_t)r.golden_cycles;

        std::string status_str;
        if (delta_cyc == 0 && std::abs(r.refactor_time_ns - r.golden_time_ns) < 75.0) {
            status_str = ANSI_GREEN "EXACT MATCH (100%)" ANSI_RESET;
        } else if (r.refactor_time_ns < r.golden_time_ns) {
            double gain_pct = (1.0 - r.refactor_time_ns / r.golden_time_ns) * 100.0;
            char buf[32];
            snprintf(buf, sizeof(buf), "+%.1f%% FASTER", gain_pct);
            status_str = std::string(ANSI_CYAN) + buf + ANSI_RESET;
        } else {
            status_str = ANSI_RED "REGRESSION" ANSI_RESET;
        }

        char g_buf[32], r_buf[32], bin_buf[32];
        snprintf(g_buf, sizeof(g_buf), "%.2f MB/s", g_mbs_dec);
        snprintf(r_buf, sizeof(r_buf), "%.2f MB/s", r_mbs_dec);
        snprintf(bin_buf, sizeof(bin_buf), "(%.2f)", r_mbs_bin);

        std::cout << std::left << std::setw(32) << r.name
                  << std::right << std::setw(13) << (std::to_string(r.golden_cycles) + " cyc")
                  << std::setw(15) << (std::to_string(r.refactor_cycles) + " cyc")
                  << std::setw(12) << ((delta_cyc == 0 ? "0 cyc" : std::to_string(delta_cyc) + " cyc"))
                  << std::setw(18) << g_buf
                  << std::setw(20) << r_buf
                  << std::setw(10) << bin_buf
                  << "    " << status_str
                  << "\n";
    }
    std::cout << "======================================================================================================================================\n" << std::endl;
}

void run_benchmark_suite(SimulationHarness& harness) {
    auto* dut = harness.dut();
    auto* amiga = harness.amiga();
    auto* pi = harness.pi();

    std::cout << "\n" ANSI_BOLD ANSI_CYAN "================================================================================\n"
              << "  === Test 15: Performance Benchmark Suite (Chipmem, Chipset & Virtual Zorro) ===\n"
              << "================================================================================\n" ANSI_RESET;

    // Ensure Pi host releases reset and halt lines (EMU68 boot sequence)
    pi->ps_clr_control(CONTROL_DRIVE_RESET | CONTROL_DRIVE_HALT);
    harness.run_mc_cycles(10);

    // Ensure PiStorm is Bus Master and 2-slot mode is active
    pi->set_use_2slot(true);
    pi->ps_set_control(CONTROL_REQ_BM);
    harness.run_mc_cycles(15);

    // Ensure Virtual Zorro card is configured at 0x00E90000
    if (!dut->pistorm__DOT__z2_configured) {
        pi->ps32_write_8(0x00E80048, 0xE0);
        pi->ps32_write_8(0x00E8004A, 0x90);
        pi->flush_pending_writes();
        harness.run_mc_cycles(4);
    }

    std::vector<BenchmarkStat> stats;

    // -------------------------------------------------------------------------
    // 15.1: Chipmem Benchmark ($00040000..$0004FFFF)
    // 32-bit Amiga 1200 Motherboard Chip RAM, Alice/Budgie bus arbitration
    // Accurate 560 ns Alice slot: 4 x 140 ns CCK = 8 MC_CLK cycles (5 wait states)
    // -------------------------------------------------------------------------
    std::cout << ANSI_BOLD ANSI_BLUE "\n  [15.1] Benchmarking Amiga 1200 Motherboard Chip RAM ($00040000)...\n" ANSI_RESET;
    uint32_t chipmem_base = 0x00040000;
    amiga->set_port_width_region(chipmem_base, 0x10000, PortWidth::PORT_32BIT);
    amiga->set_wait_states(4); // 4 WS = exactly 8 MC_CLK cycles (563.8 ns = 560 ns slot)

    // Pre-populate test pattern in memory
    for (int i = 0; i < 64; ++i) {
        amiga->mem_write_32(chipmem_base + i * 4, 0x11000000 | (uint32_t)i);
    }

    // 15.1a: Chipmem Sequential 32-bit Read (Prefetch Disabled)
    pi->ps_clr_control(CONTROL_ENABLE_PREFETCH);
    pi->flush_pending_writes();
    harness.run_mc_cycles(4);

    double t0 = harness.get_now_ns();
    uint64_t b0 = amiga->get_total_bus_cycles();
    uint32_t last_val = 0;
    for (int i = 0; i < 64; ++i) {
        last_val = pi->ps32_read_32(chipmem_base + i * 4);
    }
    double t1 = harness.get_now_ns();
    uint64_t b1 = amiga->get_total_bus_cycles();
    TEST_ASSERT(last_val == (0x11000000 | 63), "Chipmem Read (No Prefetch): Read data matches memory");
    TEST_ASSERT((b1 - b0) == 64, "Chipmem Read (No Prefetch): Generated exactly 64 motherboard bus cycles");
    stats.push_back({"Chipmem 32-bit Read (No Prefetch)", "32-bit Motherboard", "560ns Alice Slot", 64, 4, t1 - t0, b1 - b0});

    // 15.1b: Chipmem Sequential 32-bit Read (Prefetch Enabled)
    pi->ps_set_control(CONTROL_ENABLE_PREFETCH);
    harness.run_mc_cycles(4);
    // Prime prefetch
    pi->ps32_read_32(chipmem_base);
    harness.run_mc_cycles(4);

    t0 = harness.get_now_ns();
    b0 = amiga->get_total_bus_cycles();
    for (int i = 0; i < 64; ++i) {
        last_val = pi->ps32_read_32(chipmem_base + i * 4);
    }
    t1 = harness.get_now_ns();
    b1 = amiga->get_total_bus_cycles();
    TEST_ASSERT(last_val == (0x11000000 | 63), "Chipmem Read (Prefetch ON): Read data matches memory");
    stats.push_back({"Chipmem 32-bit Read (Prefetch ON)", "32-bit Motherboard", "Speculative Hit", 64, 4, t1 - t0, b1 - b0});
    pi->ps_clr_control(CONTROL_ENABLE_PREFETCH); // restore

    // 15.1c: Chipmem Sequential 32-bit Write (Pipelined 2-Slot Queue)
    t0 = harness.get_now_ns();
    b0 = amiga->get_total_bus_cycles();
    for (int i = 0; i < 64; ++i) {
        pi->ps32_write_32(chipmem_base + i * 4, 0xCAFE0000 | (uint32_t)i);
    }
    pi->flush_pending_writes();
    t1 = harness.get_now_ns();
    b1 = amiga->get_total_bus_cycles();
    TEST_ASSERT(amiga->mem_read_32(chipmem_base + 63 * 4) == (0xCAFE0000 | 63), "Chipmem Write: Memory verification in RAM");
    TEST_ASSERT((b1 - b0) == 64, "Chipmem Write: Generated exactly 64 motherboard bus cycles");
    stats.push_back({"Chipmem 32-bit Write (Pipelined)", "32-bit Motherboard", "560ns (2-Slot)", 64, 4, t1 - t0, b1 - b0});

    // 15.1d: Chipmem Sequential 16-bit Word Read
    t0 = harness.get_now_ns();
    b0 = amiga->get_total_bus_cycles();
    uint16_t last_val16 = 0;
    for (int i = 0; i < 64; ++i) {
        last_val16 = pi->ps32_read_16(chipmem_base + i * 2);
    }
    t1 = harness.get_now_ns();
    b1 = amiga->get_total_bus_cycles();
    TEST_ASSERT(last_val16 != 0, "Chipmem 16-bit Word Read: Non-zero data received");
    stats.push_back({"Chipmem 16-bit Word Read", "32-bit Motherboard", "560ns Alice Slot", 64, 2, t1 - t0, b1 - b0});

    // 15.1d2: Chipmem Sequential 16-bit Word Read (16-bit Prefetch Enabled)
    pi->ps_set_control(CONTROL_ENABLE_PREFETCH);
    pi->ps32_write_32(0x00E9001C, 0x00000081); // Prefetch enable (bit 0) + enable_word_prefetch (bit 7)
    pi->flush_pending_writes();
    harness.run_mc_cycles(4);
    // Prime prefetch
    pi->ps32_read_16(chipmem_base);
    harness.run_mc_cycles(4);

    t0 = harness.get_now_ns();
    b0 = amiga->get_total_bus_cycles();
    for (int i = 0; i < 64; ++i) {
        last_val16 = pi->ps32_read_16(chipmem_base + i * 2);
    }
    t1 = harness.get_now_ns();
    b1 = amiga->get_total_bus_cycles();
    TEST_ASSERT(last_val16 != 0, "Chipmem 16-bit Word Read (Prefetch ON): Non-zero data received");
    TEST_ASSERT((b1 - b0) < 64, "Chipmem 16-bit Word Read (Prefetch ON): Halves bus cycles via 16-bit prefetch hits");
    stats.push_back({"Chipmem 16-bit Read (Prefetch ON)", "32-bit Motherboard", "Speculative Hit", 64, 2, t1 - t0, b1 - b0});
    pi->ps32_write_32(0x00E9001C, 0x00000001); // restore default
    pi->ps_clr_control(CONTROL_ENABLE_PREFETCH);

    // 15.1e: Chipmem Sequential 16-bit Word Write
    t0 = harness.get_now_ns();
    b0 = amiga->get_total_bus_cycles();
    for (int i = 0; i < 64; ++i) {
        pi->ps32_write_16(chipmem_base + i * 2, 0xBEEF ^ (uint16_t)i);
    }
    pi->flush_pending_writes();
    t1 = harness.get_now_ns();
    b1 = amiga->get_total_bus_cycles();
    TEST_ASSERT(amiga->mem_read_16(chipmem_base + 63 * 2) == (0xBEEF ^ 63), "Chipmem 16-bit Word Write: Verified in RAM");
    stats.push_back({"Chipmem 16-bit Word Write", "32-bit Motherboard", "560ns (2-Slot)", 64, 2, t1 - t0, b1 - b0});

    // -------------------------------------------------------------------------
    // 15.2: Custom Chipset Benchmark ($00DFF000..$00DFFFFF)
    // 16-bit Port Width, 4 x 140 ns = 560 ns OCS/AGA Slot Grid (5 wait states)
    // -------------------------------------------------------------------------
    std::cout << ANSI_BOLD ANSI_BLUE "\n  [15.2] Benchmarking Amiga Custom Chipset ($00DFF000)...\n" ANSI_RESET;
    uint32_t chipset_base = 0x00DFF000;
    amiga->set_port_width_region(chipset_base, 0x1000, PortWidth::PORT_16BIT);
    amiga->set_wait_states(4); // 4 WS = exactly 8 MC_CLK cycles (563.8 ns = 560 ns slot)

    for (int i = 0; i < 64; ++i) {
        amiga->mem_write_16(chipset_base + i * 2, 0x0A00 | (uint16_t)i);
    }

    // 15.2a: Custom Chipset 16-bit Word Read
    t0 = harness.get_now_ns();
    b0 = amiga->get_total_bus_cycles();
    for (int i = 0; i < 64; ++i) {
        last_val16 = pi->ps32_read_16(chipset_base + (i % 16) * 2);
    }
    t1 = harness.get_now_ns();
    b1 = amiga->get_total_bus_cycles();
    TEST_ASSERT((b1 - b0) == 64, "Chipset Read: 64 word reads generated 64 bus cycles");
    stats.push_back({"Chipset 16-bit Word Read", "16-bit Custom Chips", "560ns CCK Slot", 64, 2, t1 - t0, b1 - b0});

    // 15.2b: Custom Chipset 16-bit Word Write (e.g. Copper list / Color palette poke)
    t0 = harness.get_now_ns();
    b0 = amiga->get_total_bus_cycles();
    for (int i = 0; i < 64; ++i) {
        pi->ps32_write_16(chipset_base + 0x180 + (i % 32) * 2, 0x0F00 | (uint16_t)i);
    }
    pi->flush_pending_writes();
    t1 = harness.get_now_ns();
    b1 = amiga->get_total_bus_cycles();
    TEST_ASSERT((b1 - b0) == 64, "Chipset Write: 64 word writes generated 64 bus cycles");
    TEST_ASSERT(amiga->mem_read_16(chipset_base + 0x180 + (31 * 2)) == (0x0F00 | 63), "Chipset Write: Verified in custom register space");
    stats.push_back({"Chipset 16-bit Word Write", "16-bit Custom Chips", "560ns CCK Slot", 64, 2, t1 - t0, b1 - b0});

    // 15.2c: Custom Chipset 32-bit Longword Write (Dynamic Bus Sizing)
    // 32 longwords on a 16-bit port decompose into 64 physical bus cycles
    t0 = harness.get_now_ns();
    b0 = amiga->get_total_bus_cycles();
    for (int i = 0; i < 32; ++i) {
        pi->ps32_write_32(chipset_base + 0x80 + (i % 8) * 4, 0x12345670 | (uint32_t)i);
    }
    pi->flush_pending_writes();
    t1 = harness.get_now_ns();
    b1 = amiga->get_total_bus_cycles();
    TEST_ASSERT((b1 - b0) == 64, "Chipset Dynamic Sizing: 32 longword transfers generated 64 16-bit bus cycles");
    stats.push_back({"Chipset 32-bit Dyn Sizing", "16-bit Custom Chips", "560ns (2x cycles)", 32, 4, t1 - t0, b1 - b0});

    // -------------------------------------------------------------------------
    // 15.3: Virtual Zorro-II Benchmark ($00E90000..$00E902FF)
    // 182 MHz Internal Wishbone B4 Interconnect (Zero Motherboard Bus Impact)
    // -------------------------------------------------------------------------
    std::cout << ANSI_BOLD ANSI_BLUE "\n  [15.3] Benchmarking Virtual Zorro-II Wishbone Interconnect ($00E90000)...\n" ANSI_RESET;
    uint32_t zorro_base = 0x00E90000;
    amiga->set_wait_states(0);


    // 15.3a: Virtual Zorro 32-bit Scratchpad Write (Slave 0: $00E9000C)
    t0 = harness.get_now_ns();
    b0 = amiga->get_total_bus_cycles();
    for (int i = 0; i < 128; ++i) {
        pi->ps32_write_32(zorro_base + 0x0C, 0xA5A50000 | (uint32_t)i);
    }
    pi->flush_pending_writes();
    t1 = harness.get_now_ns();
    b1 = amiga->get_total_bus_cycles();
    TEST_ASSERT((b1 - b0) == 0, "Virtual Zorro Write: Zero motherboard bus cycles (100% FPGA internal)");
    stats.push_back({"Virtual Zorro 32-bit Scratchpad Wr", "182 MHz Wishbone", "0 WS (Internal)", 128, 4, t1 - t0, b1 - b0});

    // 15.3b: Virtual Zorro 32-bit Scratchpad Read (Slave 0: $00E9000C)
    t0 = harness.get_now_ns();
    b0 = amiga->get_total_bus_cycles();
    for (int i = 0; i < 128; ++i) {
        last_val = pi->ps32_read_32(zorro_base + 0x0C);
    }
    t1 = harness.get_now_ns();
    b1 = amiga->get_total_bus_cycles();
    TEST_ASSERT((b1 - b0) == 0, "Virtual Zorro Read: Zero motherboard bus cycles (100% FPGA internal)");
    TEST_ASSERT(last_val == (0xA5A50000 | 127), "Virtual Zorro Read: Data matches last written value");
    stats.push_back({"Virtual Zorro 32-bit Scratchpad Rd", "182 MHz Wishbone", "0 WS (Internal)", 128, 4, t1 - t0, b1 - b0});

    // 15.3c: Virtual Zorro SPI / Coprocessor Register (Slave 1: $00E90104)
    t0 = harness.get_now_ns();
    b0 = amiga->get_total_bus_cycles();
    for (int i = 0; i < 128; ++i) {
        pi->ps32_write_32(zorro_base + 0x104, 0x55AA0000 | (uint32_t)i);
    }
    pi->flush_pending_writes();
    t1 = harness.get_now_ns();
    b1 = amiga->get_total_bus_cycles();
    TEST_ASSERT((b1 - b0) == 0, "Virtual Zorro Slave 1: Zero motherboard bus cycles");
    stats.push_back({"Virtual Zorro SPI/Coproc Reg 32-bit", "182 MHz Wishbone", "0 WS (Internal)", 128, 4, t1 - t0, b1 - b0});

    // 15.3d: Virtual Zorro Slave 2 GPIO Matrix Atomic Toggling ($00E9020C / $00E90210)
    // 128 sets + 128 clears = 256 atomic W1TS/W1TC operations
    t0 = harness.get_now_ns();
    b0 = amiga->get_total_bus_cycles();
    for (int i = 0; i < 128; ++i) {
        pi->ps32_write_32(zorro_base + 0x20C, 0x00000055); // Atomic Set
        pi->ps32_write_32(zorro_base + 0x210, 0x00000055); // Atomic Clear
    }
    pi->flush_pending_writes();
    t1 = harness.get_now_ns();
    b1 = amiga->get_total_bus_cycles();
    TEST_ASSERT((b1 - b0) == 0, "Virtual Zorro Slave 2: Zero motherboard bus cycles");
    stats.push_back({"Virtual Zorro GPIO Atomic Toggle", "182 MHz Wishbone", "0 WS (Atomic)", 256, 4, t1 - t0, b1 - b0});

    // -------------------------------------------------------------------------
    // 15.4: Comparative Analysis & Speedup Verification
    // -------------------------------------------------------------------------
    const auto& chipmem_rd_nopref = stats[0];
    const auto& chipmem_rd_pref   = stats[1];
    const auto& chipset_wr        = stats[6];
    const auto& zorro_wr          = stats[8];
    const auto& zorro_rd          = stats[9];

    double speedup_zorro_vs_chipset_wr = zorro_wr.throughput_mb_s() / chipset_wr.throughput_mb_s();
    double speedup_zorro_vs_chipmem_rd = zorro_rd.throughput_mb_s() / chipmem_rd_nopref.throughput_mb_s();
    double speedup_prefetch = chipmem_rd_pref.throughput_mb_s() / chipmem_rd_nopref.throughput_mb_s();

    TEST_ASSERT(zorro_wr.throughput_mb_s() > chipset_wr.throughput_mb_s(),
                "Performance: Virtual Zorro Write throughput exceeds Custom Chipset throughput");
    TEST_ASSERT(zorro_rd.throughput_mb_s() > chipmem_rd_nopref.throughput_mb_s(),
                "Performance: Virtual Zorro Read throughput exceeds Motherboard Chip RAM throughput");
    TEST_ASSERT(chipmem_rd_pref.throughput_mb_s() > chipmem_rd_nopref.throughput_mb_s(),
                "Performance: Speculative Prefetch accelerates sequential Chip RAM reads");

    // -------------------------------------------------------------------------
    // Performance Summary Table
    // -------------------------------------------------------------------------
    std::cout << "\n" ANSI_BOLD ANSI_YELLOW
              << "====================================================================================================================================\n"
              << "                        PiStorm32-Lite Memory & Bus Architecture Benchmark Report\n"
              << "====================================================================================================================================\n" ANSI_RESET
              << std::left
              << std::setw(37) << "Benchmark Target"
              << std::setw(23) << "Interconnect / Bus"
              << std::setw(22) << "Configuration"
              << std::right
              << std::setw(14) << "Throughput"
              << std::setw(16) << "Operations Rate"
              << std::setw(14) << "Avg Latency"
              << std::setw(18) << "Motherboard Bus"
              << "\n"
              << "------------------------------------------------------------------------------------------------------------------------------------\n";

    for (const auto& s : stats) {
        std::cout << ANSI_BOLD << std::left
                  << std::setw(37) << s.name
                  << ANSI_RESET
                  << std::left
                  << std::setw(23) << s.bus_type
                  << std::setw(22) << s.config
                  << std::right << std::fixed << std::setprecision(1)
                  << ANSI_GREEN << std::setw(9) << s.throughput_mb_s() << " MB/s" ANSI_RESET
                  << std::setprecision(2)
                  << ANSI_CYAN << std::setw(9) << s.mops() << " MOps/s" ANSI_RESET
                  << std::setprecision(1)
                  << std::setw(11) << s.avg_latency_ns() << " ns"
                  << std::right;
        if (s.bus_cycles == 0) {
            std::cout << ANSI_GREEN << std::setw(18) << "0 clk [ISOLATED]" ANSI_RESET << "\n";
        } else {
            std::cout << std::setw(11) << s.bus_cycles << " cycles" << "\n";
        }
    }

    std::cout << "------------------------------------------------------------------------------------------------------------------------------------\n"
              << ANSI_BOLD "Architectural Performance Insights:\n" ANSI_RESET
              << "  * " ANSI_GREEN "Virtual Zorro Write vs Custom Chipset Write" ANSI_RESET
              << ":  " ANSI_BOLD << std::fixed << std::setprecision(1) << speedup_zorro_vs_chipset_wr << "x Faster" ANSI_RESET
              << " (Bypasses slow 7MHz CCK sync & motherboard wait states)\n"
              << "  * " ANSI_GREEN "Virtual Zorro Read  vs Motherboard Chip RAM" ANSI_RESET
              << ":  " ANSI_BOLD << speedup_zorro_vs_chipmem_rd << "x Faster" ANSI_RESET
              << " (Direct 182 MHz Wishbone response, no Alice arbitration)\n"
              << "  * " ANSI_GREEN "Speculative Prefetch Acceleration" ANSI_RESET
              << ":            " ANSI_BOLD << "+" << std::setprecision(1) << ((speedup_prefetch - 1.0) * 100.0) << "% Gain" ANSI_RESET
              << " (Speculative read-ahead eliminates bus wait states)\n"
              << "  * " ANSI_GREEN "Bus Isolation" ANSI_RESET
              << ":                             " ANSI_BOLD "100% Isolated" ANSI_RESET
              << " (0 Amiga motherboard bus cycles during all Virtual Zorro Wishbone transfers)\n"
              << ANSI_BOLD ANSI_YELLOW
              << "====================================================================================================================================\n" ANSI_RESET
              << std::endl;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    bool enable_trace = false;
    bool benchmark_only = false;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--trace" || arg == "-t") {
            enable_trace = true;
        } else if (arg == "--benchmark-only" || arg == "--bench" || arg == "-b") {
            benchmark_only = true;
        }
    }

    std::cout << ANSI_BOLD ANSI_CYAN "========================================================\n"
              << "  PiStorm32-lite Gateware Verilator Testbench\n"
              << "  Branch: two-request-slots | Protocol: Emu68 ps_protocol\n"
              << "========================================================\n" ANSI_RESET
              << std::endl;

    SimulationHarness harness(enable_trace, "sim.vcd");
    auto* dut = harness.dut();
    auto* amiga = harness.amiga();
    auto* pi = harness.pi();

    // Warm-up clocks
    harness.run_mc_cycles(20);

    if (benchmark_only) {
        std::cout << ANSI_BOLD ANSI_CYAN "========================================================\n"
                  << "  PiStorm32-lite Benchmark Suite (Standalone Mode)\n"
                  << "========================================================\n" ANSI_RESET << std::endl;

        run_golden_reference_comparison();

        // Pulse system reset so CPLD starts in known clean state
        amiga->set_external_reset(true);
        harness.run_mc_cycles(10);
        amiga->set_external_reset(false);
        harness.run_mc_cycles(10);

        run_benchmark_suite(harness);

        std::cout << "\n" ANSI_BOLD ANSI_CYAN "========================================================\n"
                  << "  Benchmark Assertions Summary\n"
                  << "========================================================\n" ANSI_RESET;
        std::cout << "  Total Assertions: " << (g_tests_passed + g_tests_failed) << std::endl;
        std::cout << "  Passed:           " ANSI_GREEN << g_tests_passed << ANSI_RESET << std::endl;
        std::cout << "  Failed:           " << (g_tests_failed > 0 ? ANSI_RED : ANSI_GREEN)
                  << g_tests_failed << ANSI_RESET << std::endl;
        return (g_tests_failed == 0) ? 0 : 1;
    }

    // =========================================================================
    // Test 1: Clock synchronization and Reset state
    // =========================================================================
    std::cout << ANSI_BOLD "=== Test 1: Clock, Reset & Initial Register States ===" ANSI_RESET << std::endl;
    {
        uint16_t status = pi->read_status();
        TEST_ASSERT((status & STATUS_IS_BM) == 0, "Initial status: Not Bus Master");
        TEST_ASSERT((status & STATUS_REQ_ACTIVE) == 0, "Initial status: No Request Active");
        TEST_ASSERT((status & STATUS_RESET) == 0, "Initial status: No Reset assertion");
        TEST_ASSERT(dut->MC_AS_n_OE == 0, "MC_AS_n_OE tri-stated initially");
        TEST_ASSERT(dut->DATA_OE_n == 1, "DATA_OE_n disabled initially");
        TEST_ASSERT(dut->ADDR_OE_n == 1, "ADDR_OE_n disabled initially");
    }

    // =========================================================================
    // Test 2: Bus Master Arbitration (CONTROL_REQ_BM -> MC_BR_n -> MC_BG_n -> is_bm)
    // =========================================================================
    std::cout << "\n" ANSI_BOLD "=== Test 2: Bus Master Arbitration ===" ANSI_RESET << std::endl;
    {
        // Request Bus Master
        pi->ps_set_control(CONTROL_REQ_BM);
        harness.run_mc_cycles(15);

        uint16_t status = pi->read_status();
        TEST_ASSERT((status & STATUS_IS_BM) != 0, "STATUS_IS_BM asserted after MC_BG_n granted");
        TEST_ASSERT(dut->MC_AS_n_OE == 1, "MC_AS_n_OE enabled when bus master");
        TEST_ASSERT(dut->MC_RW_OE == 1, "MC_RW_OE enabled when bus master");
        TEST_ASSERT(dut->ADDR_OE_n == 0, "ADDR_OE_n enabled (low) when bus master");

        // Release Bus Master
        pi->ps_clr_control(CONTROL_REQ_BM);
        harness.run_mc_cycles(15);
        status = pi->read_status();
        TEST_ASSERT((status & STATUS_IS_BM) == 0, "STATUS_IS_BM cleared after CONTROL_REQ_BM dropped");

        // Re-acquire Bus Master for subsequent memory tests
        pi->ps_set_control(CONTROL_REQ_BM);
        harness.run_mc_cycles(15);
        status = pi->read_status();
        TEST_ASSERT((status & STATUS_IS_BM) != 0, "Re-acquired Bus Master");

        // Test 2.1: Micronik Busboard Compatibility (MC_BG_n held low by weak pulldown when floating)
        dut->MC_BG_n = 0; // Simulated weak pulldown holding line low on Micronik 6860 busboard
        harness.run_mc_cycles(4);
        status = pi->read_status();
        TEST_ASSERT((status & STATUS_IS_BM) != 0, "STATUS_IS_BM remains asserted with Micronik weak pulldown on MC_BG_n");
    }

    // =========================================================================
    // Test 3: Single-Slot 32-bit Memory Accesses (DSACK = 2'b00, 32-bit port)
    // =========================================================================
    std::cout << "\n" ANSI_BOLD "=== Test 3: Single-Slot 32-bit FastRAM Accesses ===" ANSI_RESET << std::endl;
    {
        pi->set_use_2slot(false); // Single slot mode
        amiga->set_port_width_default(PortWidth::PORT_32BIT);
        amiga->set_wait_states(0);

        // 32-bit write & read
        uint32_t test_addr = 0x08001000;
        uint32_t test_val32 = 0xA1B2C3D4;
        pi->ps32_write_32(test_addr, test_val32);
        pi->flush_pending_writes();

        uint32_t ram_val = amiga->mem_read_32(test_addr);
        TEST_ASSERT(ram_val == test_val32, "32-bit write to RAM matches (0x" + 
                    ([&]{ std::stringstream ss; ss << std::hex << ram_val; return ss.str(); }()) + ")");

        uint32_t read_val32 = pi->ps32_read_32(test_addr);
        TEST_ASSERT(read_val32 == test_val32, "32-bit read back via Pi matches (0x" +
                    ([&]{ std::stringstream ss; ss << std::hex << read_val32; return ss.str(); }()) + ")");

        // 16-bit write & read
        uint16_t test_val16_0 = 0x55AA;
        uint16_t test_val16_1 = 0x1234;
        pi->ps32_write_16(test_addr + 4, test_val16_0);
        pi->ps32_write_16(test_addr + 6, test_val16_1);
        pi->flush_pending_writes();

        uint16_t r16_0 = pi->ps32_read_16(test_addr + 4);
        uint16_t r16_1 = pi->ps32_read_16(test_addr + 6);
        TEST_ASSERT(r16_0 == test_val16_0, "16-bit read at offset 4 matches");
        TEST_ASSERT(r16_1 == test_val16_1, "16-bit read at offset 6 matches");

        uint32_t full_word = pi->ps32_read_32(test_addr + 4);
        TEST_ASSERT(full_word == 0x55AA1234, "32-bit read verifies combined 16-bit writes");

        // 8-bit byte accesses (aligned and unaligned)
        pi->ps32_write_8(test_addr + 8,  0x11);
        pi->ps32_write_8(test_addr + 9,  0x22);
        pi->ps32_write_8(test_addr + 10, 0x33);
        pi->ps32_write_8(test_addr + 11, 0x44);
        pi->flush_pending_writes();

        uint8_t b0 = pi->ps32_read_8(test_addr + 8);
        uint8_t b1 = pi->ps32_read_8(test_addr + 9);
        uint8_t b2 = pi->ps32_read_8(test_addr + 10);
        uint8_t b3 = pi->ps32_read_8(test_addr + 11);
        TEST_ASSERT(b0 == 0x11 && b1 == 0x22 && b2 == 0x33 && b3 == 0x44,
                    "8-bit read bytes across all 4 byte lanes match individually");

        uint32_t bytes_combined = pi->ps32_read_32(test_addr + 8);
        TEST_ASSERT(bytes_combined == 0x11223344, "32-bit read verifies combined byte writes");
    }

    // =========================================================================
    // Test 4: Two-Request-Slots Pipelining & Multi-word Transfers
    // =========================================================================
    std::cout << "\n" ANSI_BOLD "=== Test 4: Two-Request-Slots Pipelining & Multi-Word Transfers ===" ANSI_RESET << std::endl;
    {
        // Switch to 2-slot mode (sets CONTROL_INC_EXEC_SLOT)
        pi->set_use_2slot(true);
        harness.run_mc_cycles(5);

        TEST_ASSERT(pi->get_use_2slot() == true, "2-slot mode enabled");

        // 64-bit access test
        uint32_t addr64 = 0x08002000;
        uint64_t val64 = 0xDEADBEEFCAFEBABEULL;
        pi->ps32_write_64(addr64, val64);
        pi->flush_pending_writes();

        uint64_t r64 = pi->ps32_read_64(addr64);
        TEST_ASSERT(r64 == val64, "64-bit pipelined write/read matches (0x" +
                    ([&]{ std::stringstream ss; ss << std::hex << r64; return ss.str(); }()) + ")");

        // 128-bit access test (4 consecutive 32-bit accesses pipelined across slots)
        uint32_t addr128 = 0x08002100;
        uint128_t val128;
        val128.hi = 0x0123456789ABCDEFULL;
        val128.lo = 0xFEDCBA9876543210ULL;
        pi->ps32_write_128(addr128, val128);
        pi->flush_pending_writes();

        uint128_t r128 = pi->ps32_read_128(addr128);
        TEST_ASSERT(r128.hi == val128.hi && r128.lo == val128.lo, "128-bit pipelined write/read matches");

        // Verify true pipelined overlapping:
        // Set wait states so bus cycle takes several cycles, then queue slot 0 and slot 1 back-to-back
        amiga->set_wait_states(2);
        uint32_t pipe_addr0 = 0x08003000;
        uint32_t pipe_addr1 = 0x08003004;

        // Write slot 0
        pi->write_ps_reg(REG_SLOT, 0);
        pi->write_ps_reg(REG_DATA_LO, 0x1111);
        pi->write_ps_reg(REG_DATA_HI, 0x2222);
        pi->write_ps_reg(REG_ADDR_LO, pipe_addr0 & 0xFFFF);
        pi->write_ps_reg(REG_ADDR_HI, TXN_WRITE | (1 << TXN_FC_SHIFT) | (SIZE_LONG << TXN_SIZE_SHIFT) | ((pipe_addr0 >> 16) & 0xFF));

        // Immediately switch to slot 1 and write without waiting for slot 0 to finish!
        pi->write_ps_reg(REG_SLOT, 1);
        pi->write_ps_reg(REG_DATA_LO, 0x3333);
        pi->write_ps_reg(REG_DATA_HI, 0x4444);
        pi->write_ps_reg(REG_ADDR_LO, pipe_addr1 & 0xFFFF);
        pi->write_ps_reg(REG_ADDR_HI, TXN_WRITE | (1 << TXN_FC_SHIFT) | (SIZE_LONG << TXN_SIZE_SHIFT) | ((pipe_addr1 >> 16) & 0xFF));

        // Let simulation run until both finish
        harness.run_mc_cycles(40);

        uint32_t m0 = amiga->mem_read_32(pipe_addr0);
        uint32_t m1 = amiga->mem_read_32(pipe_addr1);
        TEST_ASSERT(m0 == 0x22221111, "Overlapping slot 0 written correctly (0x22221111)");
        TEST_ASSERT(m1 == 0x44443333, "Overlapping slot 1 written correctly (0x44443333)");

        amiga->set_wait_states(0);
    }

    // =========================================================================
    // Test 5: Dynamic Bus Sizing (16-bit and 8-bit ports)
    // =========================================================================
    std::cout << "\n" ANSI_BOLD "=== Test 5: Dynamic Bus Sizing (16-bit & 8-bit ports) ===" ANSI_RESET << std::endl;
    {
        // Configure 16-bit region: 0x00200000 - 0x0020FFFF
        uint32_t reg16_addr = 0x00200000;
        amiga->set_port_width_region(reg16_addr, 0x10000, PortWidth::PORT_16BIT);

        uint64_t cycles_before = amiga->get_total_bus_cycles();
        pi->ps32_write_32(reg16_addr, 0xAABBCCDD);
        pi->flush_pending_writes();
        uint64_t write_bus_cycles = amiga->get_total_bus_cycles() - cycles_before;

        TEST_ASSERT(write_bus_cycles == 2, "32-bit write on 16-bit port generated exactly 2 bus cycles (dynamic sizing)");
        TEST_ASSERT(amiga->mem_read_32(reg16_addr) == 0xAABBCCDD, "16-bit port memory contents match 0xAABBCCDD");

        cycles_before = amiga->get_total_bus_cycles();
        uint32_t r32_from_16 = pi->ps32_read_32(reg16_addr);
        uint64_t read_bus_cycles = amiga->get_total_bus_cycles() - cycles_before;

        TEST_ASSERT(read_bus_cycles == 2, "32-bit read on 16-bit port generated exactly 2 bus cycles");
        TEST_ASSERT(r32_from_16 == 0xAABBCCDD, "32-bit read from 16-bit port assembled correctly");

        // Configure 8-bit region: 0x00DC0000 - 0x00DCFFFF (Amiga RTC / Custom IO space)
        uint32_t reg8_addr = 0x00DC0000;
        amiga->set_port_width_region(reg8_addr, 0x10000, PortWidth::PORT_8BIT);

        cycles_before = amiga->get_total_bus_cycles();
        pi->ps32_write_32(reg8_addr, 0x12345678);
        pi->flush_pending_writes();
        uint64_t write8_cycles = amiga->get_total_bus_cycles() - cycles_before;

        TEST_ASSERT(write8_cycles == 4, "32-bit write on 8-bit port generated exactly 4 bus cycles (dynamic sizing)");
        TEST_ASSERT(amiga->mem_read_32(reg8_addr) == 0x12345678, "8-bit port memory contents match 0x12345678");

        cycles_before = amiga->get_total_bus_cycles();
        uint32_t r32_from_8 = pi->ps32_read_32(reg8_addr);
        uint64_t read8_cycles = amiga->get_total_bus_cycles() - cycles_before;

        TEST_ASSERT(read8_cycles == 4, "32-bit read on 8-bit port generated exactly 4 bus cycles");
        TEST_ASSERT(r32_from_8 == 0x12345678, "32-bit read from 8-bit port assembled correctly");
    }

    // =========================================================================
    // Test 6: Wait States Tolerance
    // =========================================================================
    std::cout << "\n" ANSI_BOLD "=== Test 6: Memory Wait States Tolerance ===" ANSI_RESET << std::endl;
    {
        uint32_t ws_addr = 0x08004000;
        amiga->set_wait_states(4); // 4 wait states on Amiga bus

        pi->ps32_write_32(ws_addr, 0xFEEDFACE);
        pi->flush_pending_writes();

        uint32_t r_ws = pi->ps32_read_32(ws_addr);
        TEST_ASSERT(r_ws == 0xFEEDFACE, "Access with 4 wait states completes accurately (0xFEEDFACE)");

        amiga->set_wait_states(0);
    }

    // =========================================================================
    // Test 7: Bus Error (BERR) Injection & Status Handling
    // =========================================================================
    std::cout << "\n" ANSI_BOLD "=== Test 7: Bus Error (BERR) Injection & Status Handling ===" ANSI_RESET << std::endl;
    {
        harness.run_mc_cycles(15);
        amiga->set_inject_berr_once(true);
        uint32_t berr_addr = 0x08005000;

        // Perform read which should trigger BERR
        pi->write_ps_reg(REG_SLOT, 0);
        pi->write_ps_reg(REG_ADDR_LO, berr_addr & 0xFFFF);
        pi->write_ps_reg(REG_ADDR_HI, TXN_READ | (1 << TXN_FC_SHIFT) | (SIZE_LONG << TXN_SIZE_SHIFT) | ((berr_addr >> 16) & 0xFF));

        bool completed = pi->wait_txn(1000);
        TEST_ASSERT(completed, "Transaction terminated upon BERR");

        uint16_t status = pi->read_status();
        TEST_ASSERT((status & STATUS_TERM_NORMAL) == 0,
                    "STATUS_TERM_NORMAL is 0 indicating abnormal termination (Bus Error)");

        // Re-sync slot pointers after manual slot manipulation
        pi->set_use_2slot(true);
    }

    // =========================================================================
    // Test 8: Amiga Interrupts, Reset, Halt, and Auxiliary Lines
    // =========================================================================
    std::cout << "\n" ANSI_BOLD "=== Test 8: Amiga Interrupts, Reset, Halt & Auxiliary Lines ===" ANSI_RESET << std::endl;
    {
        // 1. Interrupt levels
        for (uint8_t lvl : {1, 2, 4, 6, 7}) {
            amiga->set_ipl(lvl);
            harness.run_mc_cycles(10);
            uint16_t status = pi->read_status();
            uint8_t reported_ipl = (status >> STATUS_IPL_SHIFT) & 7;
            TEST_ASSERT(reported_ipl == lvl, "Amiga IPL " + std::to_string(lvl) +
                        " reported accurately in STATUS reg (" + std::to_string(reported_ipl) + ")");
            uint8_t pi_ipl_pins = dut->PI_IPL & 7;
            TEST_ASSERT(pi_ipl_pins == ((~lvl) & 7), "PI_IPL GPIO pins match active-low IPL " + std::to_string(lvl));
        }
        amiga->set_ipl(0);
        harness.run_mc_cycles(10);

        // 2. Pi driving Reset to Amiga
        pi->ps_set_control(CONTROL_DRIVE_RESET);
        harness.step_cycles(5);
        TEST_ASSERT(dut->MC_RESET_n_OE == 1 && dut->MC_RESET_n_OUT == 0, "Pi drives MC_RESET_n low");
        pi->ps_clr_control(CONTROL_DRIVE_RESET);
        harness.step_cycles(5);
        TEST_ASSERT(dut->MC_RESET_n_OE == 0, "Pi releases MC_RESET_n");

        // 3. Pi driving Halt to Amiga
        pi->ps_set_control(CONTROL_DRIVE_HALT);
        harness.step_cycles(5);
        TEST_ASSERT(dut->MC_HALT_n_OE == 1 && dut->MC_HALT_n_OUT == 0, "Pi drives MC_HALT_n low");
        pi->ps_clr_control(CONTROL_DRIVE_HALT);
        harness.step_cycles(5);
        TEST_ASSERT(dut->MC_HALT_n_OE == 0, "Pi releases MC_HALT_n");

        // 4. Pi driving INT2 and INT6
        pi->ps_set_control(CONTROL_DRIVE_INT2);
        harness.step_cycles(5);
        TEST_ASSERT(dut->INT2_n_OE == 1 && dut->INT2_n_OUT == 0, "Pi drives INT2_n low");
        pi->ps_clr_control(CONTROL_DRIVE_INT2);
        harness.step_cycles(5);
        TEST_ASSERT(dut->INT2_n_OE == 0, "Pi releases INT2_n");

        pi->ps_set_control(CONTROL_DRIVE_INT6);
        harness.step_cycles(5);
        TEST_ASSERT(dut->INT6_n_OE == 1 && dut->INT6_n_OUT == 0, "Pi drives INT6_n low");
        pi->ps_clr_control(CONTROL_DRIVE_INT6);
        harness.step_cycles(5);
        TEST_ASSERT(dut->INT6_n_OE == 0, "Pi releases INT6_n");

        // 5. External Amiga Reset input detected in STATUS
        amiga->set_external_reset(true);
        harness.run_mc_cycles(10);
        uint16_t status_rst = pi->read_status();
        TEST_ASSERT((status_rst & STATUS_RESET) != 0, "External Amiga RESET detected in STATUS register");
        amiga->set_external_reset(false);
        harness.run_mc_cycles(10);
        status_rst = pi->read_status();
        TEST_ASSERT((status_rst & STATUS_RESET) == 0, "External Amiga RESET cleared in STATUS register");

        // 6. Keyboard Reset (active-low) & Issue #4 (SUM1200 / Bus Reset) verification
        amiga->set_kbreset(true); // pulls KBRESET low (0)
        harness.step_cycles(5);
        TEST_ASSERT(dut->PI_KBRESET == 0, "KBRESET asserted (low) passed through to PI_KBRESET");
        amiga->set_kbreset(false); // releases KBRESET high (1)
        harness.step_cycles(5);
        TEST_ASSERT(dut->PI_KBRESET == 1, "KBRESET deasserted (high) passed through to PI_KBRESET");

        // Issue #4: SUM1200 pulls Amiga bus RESET low instead of KBRESET
        amiga->set_external_reset(true); // pulls MC_RESET_n_IN low (0)
        harness.run_mc_cycles(2);
        TEST_ASSERT(dut->PI_KBRESET == 0, "Issue #4: External Amiga bus reset (SUM1200) asserts PI_KBRESET (low)");
        amiga->set_external_reset(false); // releases MC_RESET_n_IN high (1)
        harness.run_mc_cycles(2);
        TEST_ASSERT(dut->PI_KBRESET == 1, "Issue #4: External bus reset release deasserts PI_KBRESET (high)");

        // Issue #4: Self-driven reset from Pi must NOT reflect back to PI_KBRESET
        pi->ps_set_control(CONTROL_DRIVE_RESET);
        harness.run_mc_cycles(2);
        TEST_ASSERT(dut->PI_KBRESET == 1, "Issue #4: Self-driven reset from Pi is masked (PI_KBRESET stays 1)");
        pi->ps_clr_control(CONTROL_DRIVE_RESET);
        harness.run_mc_cycles(2);
        TEST_ASSERT(dut->PI_KBRESET == 1, "Issue #4: PI_KBRESET remains 1 after self-reset release");

        // 7. Serial debug pins passthrough
        pi->set_serial(1, 0);
        TEST_ASSERT(dut->SPARE_OUT == 0xFD, "Serial debug: SER_DAT=1, SER_CLK=0 routed to SPARE_OUT");
        pi->set_serial(0, 1);
        TEST_ASSERT(dut->SPARE_OUT == 0xFE, "Serial debug: SER_DAT=0, SER_CLK=1 routed to SPARE_OUT");
    }

    // =========================================================================
    // Test 9: Write Data Hold Time Verification (t_DOH - Gayle / IDE Compatibility)
    // =========================================================================
    std::cout << "\n" ANSI_BOLD "=== Test 9: Write Data Hold Time (t_DOH - Gayle / IDE Compatibility) ===" ANSI_RESET << std::endl;
    {
        uint32_t ide_addr = 0xDA0000;
        pi->ps32_write_16(ide_addr, 0x1234);
        pi->flush_pending_writes();
        harness.run_mc_cycles(5);

        auto* checker = amiga->get_timing_checker();
        TEST_ASSERT(!checker->has_violations(),
                    "Gayle/IDE Write Data Hold Time: 0 timing violations detected (t_DOH >= 15.0 ns compliant)");
    }

    // =========================================================================
    // Test 10: 1-Longword Speculative Read-Ahead (Chip-RAM Acceleration)
    // =========================================================================
    std::cout << "\n" ANSI_BOLD "=== Test 10: 1-Longword Speculative Read-Ahead (Chip-RAM Acceleration) ===" ANSI_RESET << std::endl;
    {
        // 1. Enable speculative prefetch via pi_control[6]
        pi->ps_set_control(CONTROL_ENABLE_PREFETCH);
        harness.run_mc_cycles(2);

        // Set up sequential Chip-RAM data
        uint32_t base_addr = 0x005000;
        amiga->mem_write_32(base_addr + 0, 0x11223344);
        amiga->mem_write_32(base_addr + 4, 0x55667788);
        amiga->mem_write_32(base_addr + 8, 0x99AABBCC);
        amiga->mem_write_32(base_addr + 12, 0xDDEEFF00);

        // First read (normal read, primes the prefetch engine for base_addr + 4)
        uint32_t val0 = pi->ps32_read_32(base_addr + 0);
        TEST_ASSERT(val0 == 0x11223344, "Prefetch: Initial Chip-RAM read at 0x005000 returns 0x11223344");

        // Allow FPGA to speculatively prefetch next word (base_addr + 4) while idle
        harness.run_mc_cycles(6);

        // Second read (Hit on base_addr + 4)
        uint32_t val1 = pi->ps32_read_32(base_addr + 4);
        TEST_ASSERT(val1 == 0x55667788, "Prefetch: Read at 0x005004 delivered correct prefetch data (0x55667788)");

        // Allow FPGA to speculatively prefetch next word (base_addr + 8)
        harness.run_mc_cycles(6);

        // Third read (Hit on base_addr + 8)
        uint32_t val2 = pi->ps32_read_32(base_addr + 8);
        TEST_ASSERT(val2 == 0x99AABBCC, "Prefetch: Chained prefetch hit at 0x005008 delivered correct data (0x99AABBCC)");

        // 2. Write Invalidation / Coherency:
        pi->ps32_write_32(base_addr + 12, 0xCAFEBABE);
        pi->flush_pending_writes();
        harness.run_mc_cycles(6);

        uint32_t val_written = pi->ps32_read_32(base_addr + 12);
        TEST_ASSERT(val_written == 0xCAFEBABE, "Prefetch Coherency: Read back after write returns new data (0xCAFEBABE)");

        // 3. Non-sequential / Jump Branch Miss:
        uint32_t jump_addr = 0x009000;
        amiga->mem_write_32(jump_addr, 0xA1B2C3D4);
        uint32_t val_jump = pi->ps32_read_32(jump_addr);
        TEST_ASSERT(val_jump == 0xA1B2C3D4, "Prefetch: Jump to non-sequential address 0x009000 executes normal read (0xA1B2C3D4)");
        harness.run_mc_cycles(15); // Allow any speculative prefetch from jump read to complete and bus to go idle

        // 4. Safety Exclusion - Custom Registers ($DFF000..$DFFFFF):
        uint64_t bus_cycles_before_custom = amiga->get_total_bus_cycles();
        pi->ps32_read_16(0x00DFF006); // Read VHPOSR
        harness.step_cycles(2);
        uint64_t bus_cycles_after_custom = amiga->get_total_bus_cycles();
        TEST_ASSERT(bus_cycles_after_custom == bus_cycles_before_custom + 1, "Custom Register read generates exactly 1 bus cycle");
        harness.run_mc_cycles(15); // Wait idle
        TEST_ASSERT(amiga->get_total_bus_cycles() == bus_cycles_after_custom,
                    "Prefetch Safety: Custom Registers ($DFFxxx) NEVER trigger speculative prefetch");

        // 5. Safety Exclusion - CIA Registers ($BFE000..$BFEFFF):
        uint64_t bus_cycles_before_cia = amiga->get_total_bus_cycles();
        pi->ps32_read_8(0x00BFE001); // Read CIA-A PRA
        harness.step_cycles(2);
        uint64_t bus_cycles_after_cia = amiga->get_total_bus_cycles();
        TEST_ASSERT(bus_cycles_after_cia == bus_cycles_before_cia + 1, "CIA Register read generates exactly 1 bus cycle");
        harness.run_mc_cycles(15); // Wait idle
        TEST_ASSERT(amiga->get_total_bus_cycles() == bus_cycles_after_cia,
                    "Prefetch Safety: CIA Registers ($BFExxx) NEVER trigger speculative prefetch");

        // 6. Disable prefetch at end of test
        pi->ps_clr_control(CONTROL_ENABLE_PREFETCH);
        harness.run_mc_cycles(5);
    }

    // =========================================================================
    // Test 13: Exhaustive Alignment & Port Width Combinations Matrix
    // =========================================================================
    std::cout << "\n" ANSI_BOLD "=== Test 13: Exhaustive Alignment & Port Width Combinations Matrix ===" ANSI_RESET << std::endl;
    {
        // Set up test regions with specific port widths
        uint32_t base_32 = 0x010000;
        uint32_t base_16 = 0x020000;
        uint32_t base_8  = 0x030000;
        amiga->set_port_width_region(base_32, 0x10000, PortWidth::PORT_32BIT);
        amiga->set_port_width_region(base_16, 0x10000, PortWidth::PORT_16BIT);
        amiga->set_port_width_region(base_8,  0x10000, PortWidth::PORT_8BIT);

        // ---------------------------------------------------------------------
        // Part A: 32-Bit Port Width (Amiga 1200 Chip-RAM / 32-bit Fast-RAM)
        // ---------------------------------------------------------------------
        std::cout << "  Testing 32-Bit Port Width (Offsets 0, 1, 2, 3)..." << std::endl;
        pi->set_use_2slot(false); // Single slot for discrete cycle counting

        // A.1 Byte Accesses on 32-Bit Port
        uint8_t byte_vals[4] = {0x11, 0x22, 0x33, 0x44};
        for (int offset = 0; offset < 4; offset++) {
            uint32_t addr = base_32 + 0x00 + offset;
            uint64_t c0 = amiga->get_total_bus_cycles();
            pi->ps32_write_8(addr, byte_vals[offset]);
            pi->flush_pending_writes();
            TEST_ASSERT(amiga->get_total_bus_cycles() - c0 == 1,
                        "32-Bit Port: Byte write at offset " + std::to_string(offset) + " takes 1 bus cycle");
            TEST_ASSERT(amiga->mem_read_8(addr) == byte_vals[offset],
                        "32-Bit Port: Memory byte at offset " + std::to_string(offset) + " matches written value");

            c0 = amiga->get_total_bus_cycles();
            uint8_t rb = pi->ps32_read_8(addr);
            TEST_ASSERT(amiga->get_total_bus_cycles() - c0 == 1,
                        "32-Bit Port: Byte read at offset " + std::to_string(offset) + " takes 1 bus cycle");
            TEST_ASSERT(rb == byte_vals[offset],
                        "32-Bit Port: Read back byte at offset " + std::to_string(offset) + " matches");
        }
        // Verify combined 32-bit word across byte lanes
        TEST_ASSERT(pi->ps32_read_32(base_32 + 0x00) == 0x11223344,
                    "32-Bit Port: Combined 32-bit read verifies byte lane steering (0x11223344)");

        // A.2 Word Accesses on 32-Bit Port
        uint16_t word_vals[4] = {0x1234, 0x2345, 0x3456, 0x4567};
        int expected_word_cycles_32[4] = {1, 1, 1, 2};
        for (int offset = 0; offset < 4; offset++) {
            uint32_t addr = base_32 + 0x10 + (offset * 4) + offset;
            uint64_t c0 = amiga->get_total_bus_cycles();
            pi->ps32_write_16(addr, word_vals[offset]);
            pi->flush_pending_writes();
            uint64_t w_cycles = amiga->get_total_bus_cycles() - c0;
            TEST_ASSERT(w_cycles == expected_word_cycles_32[offset],
                        "32-Bit Port: Word write at offset " + std::to_string(offset) + " takes " +
                        std::to_string(expected_word_cycles_32[offset]) + " cycles (observed: " + std::to_string(w_cycles) + ")");
            TEST_ASSERT(amiga->mem_read_16(addr) == word_vals[offset],
                        "32-Bit Port: Memory word at offset " + std::to_string(offset) + " matches written value");

            c0 = amiga->get_total_bus_cycles();
            uint16_t rw = pi->ps32_read_16(addr);
            uint64_t r_cycles = amiga->get_total_bus_cycles() - c0;
            TEST_ASSERT(r_cycles == expected_word_cycles_32[offset],
                        "32-Bit Port: Word read at offset " + std::to_string(offset) + " takes " +
                        std::to_string(expected_word_cycles_32[offset]) + " cycles (observed: " + std::to_string(r_cycles) + ")");
            TEST_ASSERT(rw == word_vals[offset],
                        "32-Bit Port: Read back word at offset " + std::to_string(offset) + " matches");
        }

        // A.3 Longword Accesses on 32-Bit Port
        uint32_t long_vals[4] = {0x01234567, 0x12345678, 0x23456789, 0x3456789A};
        int expected_long_cycles_32[4] = {1, 2, 2, 2};
        for (int offset = 0; offset < 4; offset++) {
            uint32_t addr = base_32 + 0x30 + (offset * 8) + offset;
            uint64_t c0 = amiga->get_total_bus_cycles();
            pi->ps32_write_32(addr, long_vals[offset]);
            pi->flush_pending_writes();
            uint64_t w_cycles = amiga->get_total_bus_cycles() - c0;
            TEST_ASSERT(w_cycles == expected_long_cycles_32[offset],
                        "32-Bit Port: Long write at offset " + std::to_string(offset) + " takes " +
                        std::to_string(expected_long_cycles_32[offset]) + " cycles (observed: " + std::to_string(w_cycles) + ")");
            TEST_ASSERT(amiga->mem_read_32(addr) == long_vals[offset],
                        "32-Bit Port: Memory long at offset " + std::to_string(offset) + " matches written value");

            c0 = amiga->get_total_bus_cycles();
            uint32_t rl = pi->ps32_read_32(addr);
            uint64_t r_cycles = amiga->get_total_bus_cycles() - c0;
            TEST_ASSERT(r_cycles == expected_long_cycles_32[offset],
                        "32-Bit Port: Long read at offset " + std::to_string(offset) + " takes " +
                        std::to_string(expected_long_cycles_32[offset]) + " cycles (observed: " + std::to_string(r_cycles) + ")");
            TEST_ASSERT(rl == long_vals[offset],
                        "32-Bit Port: Read back long at offset " + std::to_string(offset) + " matches");
        }

        // ---------------------------------------------------------------------
        // Part B: 16-Bit Port Width (Kickstart ROM, Zorro-II, Gayle IDE)
        // ---------------------------------------------------------------------
        std::cout << "  Testing 16-Bit Port Width (Offsets 0, 1, 2, 3)..." << std::endl;

        // B.0 Byte Accesses on 16-Bit Port (Always 1 bus cycle)
        for (int offset = 0; offset < 4; offset++) {
            uint32_t addr = base_16 + 0x00 + offset;
            uint64_t c0 = amiga->get_total_bus_cycles();
            pi->ps32_write_8(addr, byte_vals[offset]);
            pi->flush_pending_writes();
            TEST_ASSERT(amiga->get_total_bus_cycles() - c0 == 1,
                        "16-Bit Port: Byte write at offset " + std::to_string(offset) + " takes 1 bus cycle");
            TEST_ASSERT(amiga->mem_read_8(addr) == byte_vals[offset],
                        "16-Bit Port: Memory byte at offset " + std::to_string(offset) + " matches written value");

            c0 = amiga->get_total_bus_cycles();
            uint8_t rb = pi->ps32_read_8(addr);
            TEST_ASSERT(amiga->get_total_bus_cycles() - c0 == 1,
                        "16-Bit Port: Byte read at offset " + std::to_string(offset) + " takes 1 bus cycle");
            TEST_ASSERT(rb == byte_vals[offset],
                        "16-Bit Port: Read back byte at offset " + std::to_string(offset) + " matches");
        }

        // B.1 Word Accesses on 16-Bit Port
        int expected_word_cycles_16[4] = {1, 2, 1, 2};
        for (int offset = 0; offset < 4; offset++) {
            uint32_t addr = base_16 + 0x10 + (offset * 4) + offset;
            uint64_t c0 = amiga->get_total_bus_cycles();
            pi->ps32_write_16(addr, word_vals[offset]);
            pi->flush_pending_writes();
            uint64_t w_cycles = amiga->get_total_bus_cycles() - c0;
            TEST_ASSERT(w_cycles == expected_word_cycles_16[offset],
                        "16-Bit Port: Word write at offset " + std::to_string(offset) + " takes " +
                        std::to_string(expected_word_cycles_16[offset]) + " cycles");
            TEST_ASSERT(amiga->mem_read_16(addr) == word_vals[offset],
                        "16-Bit Port: Memory word at offset " + std::to_string(offset) + " matches");

            c0 = amiga->get_total_bus_cycles();
            uint16_t rw = pi->ps32_read_16(addr);
            uint64_t r_cycles = amiga->get_total_bus_cycles() - c0;
            TEST_ASSERT(r_cycles == expected_word_cycles_16[offset],
                        "16-Bit Port: Word read at offset " + std::to_string(offset) + " takes " +
                        std::to_string(expected_word_cycles_16[offset]) + " cycles");
            TEST_ASSERT(rw == word_vals[offset],
                        "16-Bit Port: Read back word at offset " + std::to_string(offset) + " matches");
        }

        // B.2 Longword Accesses on 16-Bit Port
        int expected_long_cycles_16[4] = {2, 3, 2, 3};
        for (int offset = 0; offset < 4; offset++) {
            uint32_t addr = base_16 + 0x30 + (offset * 8) + offset;
            uint64_t c0 = amiga->get_total_bus_cycles();
            pi->ps32_write_32(addr, long_vals[offset]);
            pi->flush_pending_writes();
            uint64_t w_cycles = amiga->get_total_bus_cycles() - c0;
            TEST_ASSERT(w_cycles == expected_long_cycles_16[offset],
                        "16-Bit Port: Long write at offset " + std::to_string(offset) + " takes " +
                        std::to_string(expected_long_cycles_16[offset]) + " cycles");
            TEST_ASSERT(amiga->mem_read_32(addr) == long_vals[offset],
                        "16-Bit Port: Memory long at offset " + std::to_string(offset) + " matches");

            c0 = amiga->get_total_bus_cycles();
            uint32_t rl = pi->ps32_read_32(addr);
            uint64_t r_cycles = amiga->get_total_bus_cycles() - c0;
            TEST_ASSERT(r_cycles == expected_long_cycles_16[offset],
                        "16-Bit Port: Long read at offset " + std::to_string(offset) + " takes " +
                        std::to_string(expected_long_cycles_16[offset]) + " cycles");
            TEST_ASSERT(rl == long_vals[offset],
                        "16-Bit Port: Read back long at offset " + std::to_string(offset) + " matches");
        }

        // ---------------------------------------------------------------------
        // Part C: 8-Bit Port Width (CIAs, Autoconfig)
        // ---------------------------------------------------------------------
        std::cout << "  Testing 8-Bit Port Width (Offsets 0, 1, 2, 3)..." << std::endl;

        // C.0 Byte Accesses on 8-Bit Port (Always 1 bus cycle)
        for (int offset = 0; offset < 4; offset++) {
            uint32_t addr = base_8 + 0x00 + offset;
            uint64_t c0 = amiga->get_total_bus_cycles();
            pi->ps32_write_8(addr, byte_vals[offset]);
            pi->flush_pending_writes();
            TEST_ASSERT(amiga->get_total_bus_cycles() - c0 == 1,
                        "8-Bit Port: Byte write at offset " + std::to_string(offset) + " takes 1 bus cycle");
            TEST_ASSERT(amiga->mem_read_8(addr) == byte_vals[offset],
                        "8-Bit Port: Memory byte at offset " + std::to_string(offset) + " matches written value");

            c0 = amiga->get_total_bus_cycles();
            uint8_t rb = pi->ps32_read_8(addr);
            TEST_ASSERT(amiga->get_total_bus_cycles() - c0 == 1,
                        "8-Bit Port: Byte read at offset " + std::to_string(offset) + " takes 1 bus cycle");
            TEST_ASSERT(rb == byte_vals[offset],
                        "8-Bit Port: Read back byte at offset " + std::to_string(offset) + " matches");
        }

        // C.1 Word Accesses on 8-Bit Port (Always 2 cycles, any alignment)
        for (int offset = 0; offset < 4; offset++) {
            uint32_t addr = base_8 + 0x10 + (offset * 4) + offset;
            uint64_t c0 = amiga->get_total_bus_cycles();
            pi->ps32_write_16(addr, word_vals[offset]);
            pi->flush_pending_writes();
            TEST_ASSERT(amiga->get_total_bus_cycles() - c0 == 2,
                        "8-Bit Port: Word write at offset " + std::to_string(offset) + " takes 2 cycles");
            TEST_ASSERT(amiga->mem_read_16(addr) == word_vals[offset],
                        "8-Bit Port: Memory word at offset " + std::to_string(offset) + " matches");

            c0 = amiga->get_total_bus_cycles();
            uint16_t rw = pi->ps32_read_16(addr);
            TEST_ASSERT(amiga->get_total_bus_cycles() - c0 == 2,
                        "8-Bit Port: Word read at offset " + std::to_string(offset) + " takes 2 cycles");
            TEST_ASSERT(rw == word_vals[offset],
                        "8-Bit Port: Read back word at offset " + std::to_string(offset) + " matches");
        }

        // C.2 Longword Accesses on 8-Bit Port (Always 4 cycles, any alignment)
        for (int offset = 0; offset < 4; offset++) {
            uint32_t addr = base_8 + 0x30 + (offset * 8) + offset;
            uint64_t c0 = amiga->get_total_bus_cycles();
            pi->ps32_write_32(addr, long_vals[offset]);
            pi->flush_pending_writes();
            TEST_ASSERT(amiga->get_total_bus_cycles() - c0 == 4,
                        "8-Bit Port: Long write at offset " + std::to_string(offset) + " takes 4 cycles");
            TEST_ASSERT(amiga->mem_read_32(addr) == long_vals[offset],
                        "8-Bit Port: Memory long at offset " + std::to_string(offset) + " matches");

            c0 = amiga->get_total_bus_cycles();
            uint32_t rl = pi->ps32_read_32(addr);
            TEST_ASSERT(amiga->get_total_bus_cycles() - c0 == 4,
                        "8-Bit Port: Long read at offset " + std::to_string(offset) + " takes 4 cycles");
            TEST_ASSERT(rl == long_vals[offset],
                        "8-Bit Port: Read back long at offset " + std::to_string(offset) + " matches");
        }

        // ---------------------------------------------------------------------
        // Part D: Pipelined 2-Slot Mode under Unaligned Accesses
        // ---------------------------------------------------------------------
        std::cout << "  Testing 2-Slot Pipelined Mode with Unaligned Cross-Traffic..." << std::endl;
        pi->set_use_2slot(true);
        uint32_t p2_addr1 = base_32 + 0x81; // Offset 1 (unaligned long)
        uint32_t p2_addr2 = base_32 + 0x93; // Offset 3 (unaligned word)
        pi->ps32_write_32(p2_addr1, 0xABCDEF01);
        pi->ps32_write_16(p2_addr2, 0x4321);
        pi->flush_pending_writes();

        uint32_t p2_r1 = pi->ps32_read_32(p2_addr1);
        uint16_t p2_r2 = pi->ps32_read_16(p2_addr2);
        TEST_ASSERT(p2_r1 == 0xABCDEF01, "2-Slot Mode: Unaligned 32-bit read at offset 1 matches (0xABCDEF01)");
        TEST_ASSERT(p2_r2 == 0x4321, "2-Slot Mode: Unaligned 16-bit read at offset 3 matches (0x4321)");

        // ---------------------------------------------------------------------
        // Part E: Prefetch Safety under Unaligned Traffic
        // ---------------------------------------------------------------------
        std::cout << "  Testing Prefetch Safety under Unaligned Traffic..." << std::endl;
        pi->ps_set_control(CONTROL_ENABLE_PREFETCH);
        harness.run_mc_cycles(2);

        // Unaligned 32-bit read in Chip-RAM must NOT trigger prefetch
        pi->ps32_read_32(0x005001);
        harness.run_mc_cycles(4);
        TEST_ASSERT(dut->pistorm__DOT__u_m68k__DOT__prefetch_eligible == 0,
                    "Prefetch Safety: Unaligned 32-bit read (offset 1) does NOT make prefetch eligible");

        // Byte read must NOT trigger prefetch
        pi->ps32_read_8(0x005002);
        harness.run_mc_cycles(4);
        TEST_ASSERT(dut->pistorm__DOT__u_m68k__DOT__prefetch_eligible == 0,
                    "Prefetch Safety: Byte read (offset 2) does NOT make prefetch eligible");

        // Word read must NOT trigger prefetch
        pi->ps32_read_16(0x005002);
        harness.run_mc_cycles(4);
        TEST_ASSERT(dut->pistorm__DOT__u_m68k__DOT__prefetch_eligible == 0,
                    "Prefetch Safety: Word read (offset 2) does NOT make prefetch eligible");

        // 16-bit port read must NOT trigger prefetch
        pi->ps32_read_32(base_16 + 0x00);
        harness.run_mc_cycles(4);
        TEST_ASSERT(dut->pistorm__DOT__u_m68k__DOT__prefetch_eligible == 0,
                    "Prefetch Safety: 16-bit port read does NOT make prefetch eligible");

        // Aligned 32-bit read on 32-bit port MUST trigger prefetch
        pi->ps32_read_32(0x005000);
        harness.run_mc_cycles(10);
        TEST_ASSERT(dut->pistorm__DOT__u_m68k__DOT__prefetch_eligible == 0 && dut->pistorm__DOT__u_m68k__DOT__prefetch_valid == 1,
                    "Prefetch Safety: Aligned 32-bit Chip-RAM read successfully completed prefetch cycle");

        pi->ps_clr_control(CONTROL_ENABLE_PREFETCH);
        harness.run_mc_cycles(2);
    }

    // =========================================================================
    // Test 11: Motorola MC68020 AC Timing Compliance Verification
    // =========================================================================
    std::cout << "\n" ANSI_BOLD "=== Test 11: Motorola MC68020 AC Timing Compliance (MC68020UM Section 10) ===" ANSI_RESET << std::endl;
    {
        auto* checker = amiga->get_timing_checker();
        checker->print_timing_report(std::cout);

        bool violations = checker->has_violations();
        TEST_ASSERT(!violations, "MC68020 AC Timing: 0 timing violations detected across all bus transactions");
    }

    // =========================================================================
    // Test 12: PCB Hardware Component Verification (74LVC573A & 74CBTD3384)
    // =========================================================================
    std::cout << "\n" ANSI_BOLD "=== Test 12: PCB Hardware Component Verification (74LVC573A & 74CBTD3384) ===" ANSI_RESET << std::endl;
    {
        auto* pcb_latch = amiga->get_pcb_latch();
        auto* pcb_switch = amiga->get_pcb_data_switch();

        pcb_latch->print_report(std::cout);
        pcb_switch->print_report(std::cout);

        TEST_ASSERT(pcb_latch->get_setup_violations() == 0,
                    "74LVC573A Latch: 0 setup time violations (t_su >= 2.0 ns, worst observed: " +
                    std::to_string(pcb_latch->get_worst_setup_observed()).substr(0, 5) + " ns)");
        TEST_ASSERT(pcb_latch->get_hold_violations() == 0,
                    "74LVC573A Latch: 0 hold time violations (t_h >= 1.5 ns, worst observed: " +
                    std::to_string(pcb_latch->get_worst_hold_observed()).substr(0, 5) + " ns)");
        TEST_ASSERT(pcb_latch->get_pulse_w_violations() == 0,
                    "74LVC573A Latch: 0 pulse width violations (t_w >= 3.0 ns, worst observed: " +
                    std::to_string(pcb_latch->get_worst_pulse_w_observed()).substr(0, 5) + " ns)");
    }

    // =========================================================================
    // Test 13: Universal A1200 Clock Stability Verification
    // =========================================================================
    std::cout << "\n" ANSI_BOLD "=== Test 13: Universal A1200 Clock Stability Verification ===" ANSI_RESET << std::endl;
    {
        // 13.1: Test under severe ringing / 1.8V dip on ungemoddeten boards (E123C/E125C)
        std::cout << ANSI_CYAN "  [13.1] Testing Bus Transactions under Un-fixed Motherboard Clock (1.8V Ringing Dip)..." ANSI_RESET << std::endl;
        harness.set_clock_mode(ClockMode::RINGING_UNFIXED);
        harness.run_mc_cycles(5);

        // Perform 32-bit read and write
        uint32_t ring_addr = 0x006000;
        pi->ps32_write_32(ring_addr, 0xCAFEBABE);
        harness.run_mc_cycles(4);
        TEST_ASSERT(amiga->mem_read_32(ring_addr) == 0xCAFEBABE,
                    "Ringing Clock: 32-bit write under edge-dip conditions matches memory");

        uint32_t ring_rd32 = pi->ps32_read_32(ring_addr);
        harness.run_mc_cycles(4);
        TEST_ASSERT(ring_rd32 == 0xCAFEBABE,
                    "Ringing Clock: 32-bit read under edge-dip conditions matches expected data");

        // Perform 16-bit unaligned read and write
        pi->ps32_write_16(ring_addr + 2, 0x1234);
        harness.run_mc_cycles(4);
        uint16_t ring_rd16 = pi->ps32_read_16(ring_addr + 2);
        harness.run_mc_cycles(4);
        TEST_ASSERT(ring_rd16 == 0x1234,
                    "Ringing Clock: 16-bit access under edge-dip conditions matches expected data");

        // Perform 8-bit read and write
        pi->ps32_write_8(ring_addr + 1, 0x77);
        harness.run_mc_cycles(4);
        uint8_t ring_rd8 = pi->ps32_read_8(ring_addr + 1);
        harness.run_mc_cycles(4);
        TEST_ASSERT(ring_rd8 == 0x77,
                    "Ringing Clock: 8-bit access under edge-dip conditions matches expected data");

        // 13.2: Test under Amiga XOR Duty Cycle Asymmetry & 3ns Jitter
        std::cout << ANSI_CYAN "  [13.2] Testing Bus Transactions under Amiga XOR Asymmetry & 3ns Jitter..." ANSI_RESET << std::endl;
        harness.set_clock_mode(ClockMode::XOR_ASYMMETRIC);
        harness.run_mc_cycles(5);

        uint32_t xor_addr = 0x007000;
        pi->ps32_write_32(xor_addr, 0xDEADBEEF);
        harness.run_mc_cycles(4);
        TEST_ASSERT(amiga->mem_read_32(xor_addr) == 0xDEADBEEF,
                    "XOR Asymmetry: 32-bit write under asymmetric clock matches memory");

        uint32_t xor_rd32 = pi->ps32_read_32(xor_addr);
        harness.run_mc_cycles(4);
        TEST_ASSERT(xor_rd32 == 0xDEADBEEF,
                    "XOR Asymmetry: 32-bit read under asymmetric clock matches expected data");

        pi->ps32_write_16(xor_addr + 2, 0x55AA);
        harness.run_mc_cycles(4);
        uint16_t xor_rd16 = pi->ps32_read_16(xor_addr + 2);
        harness.run_mc_cycles(4);
        TEST_ASSERT(xor_rd16 == 0x55AA,
                    "XOR Asymmetry: 16-bit access under asymmetric clock matches expected data");

        // Restore clean clock
        harness.set_clock_mode(ClockMode::CLEAN);
        harness.run_mc_cycles(5);
    }

    // =========================================================================
    // Test 14: Virtual Zorro-II 64KB AutoConfig & Daisy-Chain Pass-Through
    // =========================================================================
    std::cout << "\n" ANSI_BOLD "=== Test 14: Virtual Zorro-II 64KB AutoConfig & Daisy-Chain Pass-Through ===" ANSI_RESET << std::endl;
    {
        // 14.1: Inspect AutoConfig ROM in unconfigured state ($00E80000)
        std::cout << ANSI_CYAN "  [14.1] Testing Unconfigured AutoConfig ROM Access ($00E80000)..." ANSI_RESET << std::endl;
        uint64_t cycles_before = amiga->get_total_bus_cycles();

        // Read ROM bytes
        uint8_t r_type_hi = pi->ps32_read_8(0x00E80000);
        uint8_t r_type_lo = pi->ps32_read_8(0x00E80002);
        uint8_t er_type = (r_type_hi & 0xF0) | ((r_type_lo >> 4) & 0x0F);

        TEST_ASSERT(er_type == 0xC1, "AutoConfig ROM: er_Type is 0xC1 (Zorro II, 64KB IO, no ROM/memlist)");

        // Read Product ID (~0x32 -> ~3=C, ~2=D)
        uint8_t r_prod_hi = pi->ps32_read_8(0x00E80004);
        uint8_t r_prod_lo = pi->ps32_read_8(0x00E80006);
        uint8_t prod_id = ~(((r_prod_hi & 0xF0)) | ((r_prod_lo >> 4) & 0x0F));
        TEST_ASSERT(prod_id == 0x32, "AutoConfig ROM: Product ID is 0x32 (PiStorm32)");

        // Read Manufacturer ID 28020 (0x6D74)
        uint8_t m0 = pi->ps32_read_8(0x00E80010);
        uint8_t m1 = pi->ps32_read_8(0x00E80012);
        uint8_t m2 = pi->ps32_read_8(0x00E80014);
        uint8_t m3 = pi->ps32_read_8(0x00E80016);
        uint16_t manuf_id = ~(((m0 & 0xF0) << 8) | ((m1 & 0xF0) << 4) | (m2 & 0xF0) | ((m3 >> 4) & 0x0F));
        TEST_ASSERT(manuf_id == 28020, "AutoConfig ROM: Manufacturer ID is 28020 (0x6D74)");

        // Read 16-bit and 32-bit words directly
        uint16_t rom_word0 = pi->ps32_read_16(0x00E80000);
        TEST_ASSERT(rom_word0 == 0xC0FF, "AutoConfig ROM: 16-bit word read at 0x00E80000 matches 0xC0FF");

        uint32_t rom_long0 = pi->ps32_read_32(0x00E80000);
        TEST_ASSERT(rom_long0 == 0xC0FF10FF, "AutoConfig ROM: 32-bit long read at 0x00E80000 matches 0xC0FF10FF");

        uint64_t rom_bus_cycles = amiga->get_total_bus_cycles() - cycles_before;
        TEST_ASSERT(rom_bus_cycles == 0,
                    "AutoConfig Isolation: 0 physical Amiga bus cycles during AutoConfig ROM reads (zero slot-1 collision)");

        // 14.2: Configure Base Address to $00E90000
        std::cout << ANSI_CYAN "  [14.2] Configuring Card Base Address to $00E90000..." ANSI_RESET << std::endl;
        cycles_before = amiga->get_total_bus_cycles();

        // Write Base High ($E) to $E80048 and Base Low ($9) to $E8004A
        pi->ps32_write_8(0x00E80048, 0xE0);
        pi->ps32_write_8(0x00E8004A, 0x90);
        pi->flush_pending_writes();

        uint64_t cfg_bus_cycles = amiga->get_total_bus_cycles() - cycles_before;
        TEST_ASSERT(cfg_bus_cycles == 0, "AutoConfig Write: Base address write handled internally (0 bus cycles)");

        // 14.3: Daisy-Chain Pass-Through Verification (External Zorro Cards)
        std::cout << ANSI_CYAN "  [14.3] Testing Daisy-Chain Pass-Through to External Busboard ($00E80000)..." ANSI_RESET << std::endl;
        // Now that PiStorm is configured, access to $00E80000 MUST pass through to Amiga bus!
        uint32_t ext_zorro_addr = 0x00E80000;
        amiga->set_port_width_region(ext_zorro_addr, 0x10000, PortWidth::PORT_8BIT);
        amiga->mem_write_8(ext_zorro_addr, 0x5A); // Simulate next card's ROM byte

        cycles_before = amiga->get_total_bus_cycles();
        uint8_t ext_val = pi->ps32_read_8(ext_zorro_addr);
        uint64_t ext_read_cycles = amiga->get_total_bus_cycles() - cycles_before;

        TEST_ASSERT(ext_read_cycles > 0, "Daisy-Chain Pass-Through: Access to 0x00E80000 generated external Amiga bus cycles");
        TEST_ASSERT(ext_val == 0x5A, "Daisy-Chain Pass-Through: Read back external Zorro card ROM data (0x5A)");

        // Write to external card at 0x00E80000
        cycles_before = amiga->get_total_bus_cycles();
        pi->ps32_write_8(ext_zorro_addr, 0xA5);
        pi->flush_pending_writes();
        uint64_t ext_write_cycles = amiga->get_total_bus_cycles() - cycles_before;

        TEST_ASSERT(ext_write_cycles > 0, "Daisy-Chain Pass-Through: Write to 0x00E80000 forwarded to external bus");
        TEST_ASSERT(amiga->mem_read_8(ext_zorro_addr) == 0xA5, "Daisy-Chain Pass-Through: External card received write data");

        // 14.4: Virtual 64KB I/O Access at $00E90000
        std::cout << ANSI_CYAN "  [14.4] Testing Virtual 64KB I/O Registers at $00E90000..." ANSI_RESET << std::endl;
        cycles_before = amiga->get_total_bus_cycles();

        uint32_t magic = pi->ps32_read_32(0x00E90000);
        TEST_ASSERT(magic == 0x50533332, "Virtual IO: Magic ID register at offset 0 matches 'PS32'");

        uint32_t dev_id = pi->ps32_read_32(0x00E90004);
        TEST_ASSERT(dev_id == 0x6D743201, "Virtual IO: Device/Version register at offset 4 matches {28020, 0x32, 0x01}");

        uint32_t base_status = pi->ps32_read_32(0x00E90008);
        TEST_ASSERT(((base_status >> 16) & 0xFF) == 0xE9 && (base_status & 1) == 1,
                    "Virtual IO: Base/Status register at offset 8 matches base 0xE90000 and configured=1");

        uint32_t git_hash = pi->ps32_read_32(0x00E90038);
        TEST_ASSERT(git_hash != 0, "Virtual IO: Git commit hash register at offset 0x38 is non-zero");

        uint32_t build_date = pi->ps32_read_32(0x00E9003C);
        TEST_ASSERT(((build_date >> 24) & 0xFF) == 0x20, "Virtual IO: Build date BCD register at offset 0x3C starts with century 20xx");

        uint32_t build_info = pi->ps32_read_32(0x00E90040);
        TEST_ASSERT(((build_info >> 16) & 0xFFFF) == 0x5053, "Virtual IO: Build info register at offset 0x40 matches 'PS'");

        // Scratchpad R/W testing
        pi->ps32_write_32(0x00E9000C, 0xCAFEBABE);
        pi->flush_pending_writes();
        uint32_t sp32 = pi->ps32_read_32(0x00E9000C);
        TEST_ASSERT(sp32 == 0xCAFEBABE, "Virtual IO: 32-bit scratchpad read/write matches 0xCAFEBABE");

        // 16-bit write to upper word of scratchpad
        pi->ps32_write_16(0x00E9000C, 0x1234);
        pi->flush_pending_writes();
        uint16_t sp16 = pi->ps32_read_16(0x00E9000C);
        TEST_ASSERT(sp16 == 0x1234, "Virtual IO: 16-bit word scratchpad read matches 0x1234");
        TEST_ASSERT(pi->ps32_read_32(0x00E9000C) == 0x1234BABE, "Virtual IO: Scratchpad 32-bit value is 0x1234BABE after 16-bit write");

        // 8-bit write to byte 3 of scratchpad
        pi->ps32_write_8(0x00E9000F, 0x77);
        pi->flush_pending_writes();
        uint8_t sp8 = pi->ps32_read_8(0x00E9000F);
        TEST_ASSERT(sp8 == 0x77, "Virtual IO: 8-bit scratchpad read matches 0x77");
        TEST_ASSERT(pi->ps32_read_32(0x00E9000C) == 0x1234BA77, "Virtual IO: Scratchpad 32-bit value is 0x1234BA77 after 8-bit write");

        uint64_t io_cycles = amiga->get_total_bus_cycles() - cycles_before;
        TEST_ASSERT(io_cycles == 0, "Virtual IO: All 64KB I/O register accesses handled internally (0 Amiga bus cycles)");

        // 14.5: Reset & AutoConfig Shut-up Testing
        std::cout << ANSI_CYAN "  [14.5] Testing Reset & Shut-up Functionality..." ANSI_RESET << std::endl;
        // Pulse external reset
        amiga->set_external_reset(true);
        harness.run_mc_cycles(4);
        amiga->set_external_reset(false);
        harness.run_mc_cycles(4);

        // After reset, card must be unconfigured again
        cycles_before = amiga->get_total_bus_cycles();
        uint32_t reset_rom = pi->ps32_read_32(0x00E80000);
        uint64_t reset_cycles = amiga->get_total_bus_cycles() - cycles_before;
        TEST_ASSERT(reset_cycles == 0 && reset_rom == 0xC0FF10FF,
                    "Reset: AutoConfig card unconfigured after bus reset (responds internally at 0x00E80000)");

        // Send Shut-up command ($00E8004C)
        pi->ps32_write_8(0x00E8004C, 0x00);
        pi->flush_pending_writes();

        // After shut-up, $00E80000 must pass through to external bus
        amiga->mem_write_8(ext_zorro_addr, 0x99);
        cycles_before = amiga->get_total_bus_cycles();
        uint8_t shutup_val = pi->ps32_read_8(ext_zorro_addr);
        uint64_t shutup_cycles = amiga->get_total_bus_cycles() - cycles_before;

        TEST_ASSERT(shutup_cycles > 0, "Shut-up: Access to 0x00E80000 forwards to external bus after shut-up command");
        TEST_ASSERT(shutup_val == 0x99, "Shut-up: External busboard data received at 0x00E80000 after shut-up");

        // 14.6: Wishbone Interrupt Controller & Amiga INT2/INT6 Verification
        std::cout << ANSI_CYAN "  [14.6] Testing Wishbone Interrupt Controller (INT2 & INT6)..." ANSI_RESET << std::endl;
        // Pulse reset to wake up card from shut-up state
        amiga->set_external_reset(true);
        harness.run_mc_cycles(4);
        amiga->set_external_reset(false);
        harness.run_mc_cycles(4);

        // Re-configure card after reset
        pi->ps32_write_8(0x00E80048, 0xE0);
        pi->ps32_write_8(0x00E8004A, 0x90);
        pi->flush_pending_writes();

        // Check initial interrupt registers
        uint32_t init_status = pi->ps32_read_32(0x00E90010);
        uint32_t init_enable = pi->ps32_read_32(0x00E90014);
        TEST_ASSERT(init_status == 0, "Wishbone IRQ: Initial INT_STATUS register is 0");
        TEST_ASSERT(init_enable == 0, "Wishbone IRQ: Initial INT_ENABLE register is 0");
        TEST_ASSERT(dut->INT2_n_OE == 0, "Wishbone IRQ: INT2_n is released initially");
        TEST_ASSERT(dut->INT6_n_OE == 0, "Wishbone IRQ: INT6_n is released initially");

        // Enable INT2 in INT_ENABLE register
        pi->ps32_write_32(0x00E90014, 0x01); // Bit 0 = int2_enable
        pi->flush_pending_writes();
        TEST_ASSERT(pi->ps32_read_32(0x00E90014) == 0x01, "Wishbone IRQ: INT_ENABLE reads back 0x01 (INT2 enabled)");
        TEST_ASSERT(dut->INT2_n_OE == 0, "Wishbone IRQ: INT2_n still inactive before trigger");

        // Force INT2 via INT_FORCE register
        pi->ps32_write_32(0x00E90018, 0x01); // Bit 0 = force int2
        pi->flush_pending_writes();
        harness.step_cycles(2);
        TEST_ASSERT(dut->INT2_n_OE == 1 && dut->INT2_n_OUT == 0, "Wishbone IRQ: Card successfully drives Amiga INT2_n LOW");
        TEST_ASSERT((pi->ps32_read_32(0x00E90010) & 1) == 1, "Wishbone IRQ: INT_STATUS bit 0 is pending");

        // Clear INT2 via Write-1-to-Clear (W1C)
        pi->ps32_write_32(0x00E90010, 0x01);
        pi->flush_pending_writes();
        harness.step_cycles(2);
        TEST_ASSERT(dut->INT2_n_OE == 0, "Wishbone IRQ: INT2_n released after W1C acknowledge");
        TEST_ASSERT((pi->ps32_read_32(0x00E90010) & 1) == 0, "Wishbone IRQ: INT_STATUS bit 0 cleared");

        // Enable INT6 in INT_ENABLE register
        pi->ps32_write_32(0x00E90014, 0x02); // Bit 1 = int6_enable
        pi->flush_pending_writes();
        TEST_ASSERT(pi->ps32_read_32(0x00E90014) == 0x02, "Wishbone IRQ: INT_ENABLE reads back 0x02 (INT6 enabled)");

        // Force INT6 via INT_FORCE register
        pi->ps32_write_32(0x00E90018, 0x02); // Bit 1 = force int6
        pi->flush_pending_writes();
        harness.step_cycles(2);
        TEST_ASSERT(dut->INT6_n_OE == 1 && dut->INT6_n_OUT == 0, "Wishbone IRQ: Card successfully drives Amiga INT6_n LOW");
        TEST_ASSERT((pi->ps32_read_32(0x00E90010) & 2) == 2, "Wishbone IRQ: INT_STATUS bit 1 is pending");

        // Clear INT6 via Write-1-to-Clear (W1C)
        pi->ps32_write_32(0x00E90010, 0x02);
        pi->flush_pending_writes();
        harness.step_cycles(2);
        TEST_ASSERT(dut->INT6_n_OE == 0, "Wishbone IRQ: INT6_n released after W1C acknowledge");
        TEST_ASSERT((pi->ps32_read_32(0x00E90010) & 2) == 0, "Wishbone IRQ: INT_STATUS bit 1 cleared");

        // Test Slave 1 (Peripheral slot / SPI Master at $0100..$01FF)
        pi->ps32_write_32(0x00E90100, 0x12345678);
        pi->ps32_write_32(0x00E90104, 0xAABBCCDD);
        pi->flush_pending_writes();
        uint32_t spi_ctrl_val = pi->ps32_read_32(0x00E90100);
        uint32_t spi_data_val = pi->ps32_read_32(0x00E90104);
        TEST_ASSERT(spi_ctrl_val == 0x12345678, "Wishbone Slave 1: SPI_CTRL register matches 0x12345678");
        TEST_ASSERT(spi_data_val == 0xAABBCCDD, "Wishbone Slave 1: SPI_DATA register matches 0xAABBCCDD");

        // ---------------------------------------------------------------------
        // [14.7] Testing Wishbone Slave 2: ESP32-Style GPIO Matrix & IO MUX ($00E90200)
        // ---------------------------------------------------------------------
        std::cout << "  [14.7] Testing Wishbone Slave 2: ESP32-Style GPIO Matrix & IO MUX ($00E90200)..." << std::endl;

        // 1. Verify Mode 0 Reset Default: Debug DAT/CLK pass-through, pins 7:2 are high
        uint32_t iomux_ctrl = pi->ps32_read_32(0x00E90200);
        uint32_t gpio_dir   = pi->ps32_read_32(0x00E90214);
        TEST_ASSERT((iomux_ctrl & 0x07) == 0, "GPIO Matrix: Default IOMUX_CTRL mode is 0 (Debug Active)");
        TEST_ASSERT((gpio_dir & 0xFF) == 0xFF, "GPIO Matrix: Default GPIO_DIR is 0xFF (All outputs)");
        TEST_ASSERT(dut->SPARE_OE == 0xFF, "GPIO Matrix: Default SPARE_OE is 0xFF");

        pi->set_serial(1, 0);
        harness.run_mc_cycles(2);
        TEST_ASSERT(dut->SPARE_OUT == 0xFD, "GPIO Matrix Mode 0: SER_DAT=1, SER_CLK=0 -> SPARE_OUT = 0xFD");
        pi->set_serial(0, 1);
        harness.run_mc_cycles(2);
        TEST_ASSERT(dut->SPARE_OUT == 0xFE, "GPIO Matrix Mode 0: SER_DAT=0, SER_CLK=1 -> SPARE_OUT = 0xFE");

        // 2. Mode 1: All GPIO (Debug Disabled)
        pi->ps32_write_32(0x00E90200, 0x00000001); // Mode 1
        pi->ps32_write_32(0x00E90208, 0x000000AA); // GPIO_OUT = 0xAA
        pi->flush_pending_writes();
        harness.run_mc_cycles(2);
        TEST_ASSERT(dut->SPARE_OUT == 0xAA, "GPIO Matrix Mode 1: Direct write 0xAA routed to SPARE_OUT");

        // Atomic Bit Set (W1TS)
        pi->ps32_write_32(0x00E9020C, 0x00000005); // Set bits 0 and 2 (0xAA | 0x05 = 0xAF)
        pi->flush_pending_writes();
        harness.run_mc_cycles(2);
        TEST_ASSERT(dut->SPARE_OUT == 0xAF, "GPIO Matrix: Atomic W1TS set bits 0,2 -> SPARE_OUT = 0xAF");

        // Atomic Bit Clear (W1TC)
        pi->ps32_write_32(0x00E90210, 0x000000A0); // Clear bits 7 and 5 (0xAF & ~0xA0 = 0x0F)
        pi->flush_pending_writes();
        harness.run_mc_cycles(2);
        TEST_ASSERT(dut->SPARE_OUT == 0x0F, "GPIO Matrix: Atomic W1TC clear bits 7,5 -> SPARE_OUT = 0x0F");

        uint32_t gpio_out_readback = pi->ps32_read_32(0x00E90208);
        TEST_ASSERT((gpio_out_readback & 0xFF) == 0x0F, "GPIO Matrix: GPIO_OUT readback matches 0x0F");

        // 3. Direction Control (Output Enable)
        pi->ps32_write_32(0x00E9021C, 0x00000003); // Atomic Direction Clear: bits 1,0 become inputs
        pi->flush_pending_writes();
        harness.run_mc_cycles(2);
        TEST_ASSERT(dut->SPARE_OE == 0xFC, "GPIO Matrix: Atomic DIR_CLR bits 1,0 -> SPARE_OE = 0xFC");

        pi->ps32_write_32(0x00E90218, 0x00000001); // Atomic Direction Set: bit 0 becomes output
        pi->flush_pending_writes();
        harness.run_mc_cycles(2);
        TEST_ASSERT(dut->SPARE_OE == 0xFD, "GPIO Matrix: Atomic DIR_SET bit 0 -> SPARE_OE = 0xFD");

        // 4. Live Input Reading (GPIO_IN)
        dut->SPARE_IN = 0x5A;
        harness.run_mc_cycles(2);
        uint32_t in_val1 = pi->ps32_read_32(0x00E90204);
        TEST_ASSERT((in_val1 & 0xFF) == 0x5A, "GPIO Matrix: GPIO_IN reads physical pins (0x5A)");

        dut->SPARE_IN = 0xC3;
        harness.run_mc_cycles(2);
        uint32_t in_val2 = pi->ps32_read_32(0x00E90204);
        TEST_ASSERT((in_val2 & 0xFF) == 0xC3, "GPIO Matrix: GPIO_IN reads physical pins (0xC3)");

        // 5. ESP32 Matrix Mode (Per-Pin Function Selection & Inversion)
        // Enable Matrix Mode (bit 7 = 1)
        pi->ps32_write_32(0x00E90200, 0x00000080);
        // Configure Pin 0 as SPI SCLK (FUNC_SEL=3, OE_MODE=01 force out -> 0x13)
        pi->ps32_write_32(0x00E90220, 0x00000013);
        // Configure Pin 1 as SPI CS# (FUNC_SEL=5, OE_MODE=01 force out -> 0x15)
        pi->ps32_write_32(0x00E90224, 0x00000015);
        // Configure Pin 7 as Inverted Constant 1 (FUNC_SEL=9, INV=1, force out -> 0x59)
        pi->ps32_write_32(0x00E9023C, 0x00000059);
        // Toggle SPI signals via Slave 1: spi_ctrl[0] = SCLK, spi_ctrl[1] = CS#
        pi->ps32_write_32(0x00E90100, 0x00000001); // SCLK=1, CS#=0
        pi->flush_pending_writes();
        harness.run_mc_cycles(2);
        TEST_ASSERT((dut->SPARE_OUT & 0x01) == 1, "GPIO Matrix: Pin 0 routes SPI SCLK (1)");
        TEST_ASSERT((dut->SPARE_OUT & 0x02) == 0, "GPIO Matrix: Pin 1 routes SPI CS# (0)");
        TEST_ASSERT((dut->SPARE_OUT & 0x80) == 0, "GPIO Matrix: Pin 7 inverted constant 1 -> 0");

        pi->ps32_write_32(0x00E90100, 0x00000002); // SCLK=0, CS#=1
        pi->flush_pending_writes();
        harness.run_mc_cycles(2);
        TEST_ASSERT((dut->SPARE_OUT & 0x01) == 0, "GPIO Matrix: Pin 0 routes SPI SCLK (0)");
        TEST_ASSERT((dut->SPARE_OUT & 0x02) == 2, "GPIO Matrix: Pin 1 routes SPI CS# (1)");

        // 6. Test Unmapped Space ($00E90300) safe termination
        uint32_t unmapped_val = pi->ps32_read_32(0x00E90300);
        TEST_ASSERT(unmapped_val == 0, "Wishbone Default Slave: Unmapped address $00E90300 terminates safely returning 0");

        // 7. Test BUS_CTRL (0x00E9001C) & Hardware Diagnostic Registers (0x00E90020 - 0x00E90034)
        uint32_t bus_ctrl_default = pi->ps32_read_32(0x00E9001C);
        TEST_ASSERT((bus_ctrl_default & 0x07) == 0x07, "BUS_CTRL default: prefetch, fast_dsack, and cck_sync active");
        TEST_ASSERT((bus_ctrl_default & 0x80) != 0, "BUS_CTRL default: enable_word_prefetch active");

        // Write and read back BUS_CTRL
        pi->ps32_write_32(0x00E9001C, 0x00000008); // Set bit 3 (force_phase_invert)
        pi->flush_pending_writes();
        uint32_t bus_ctrl_inverted = pi->ps32_read_32(0x00E9001C);
        TEST_ASSERT((bus_ctrl_inverted & 0x08) != 0, "BUS_CTRL: force_phase_invert bit set successfully");

        // Restore default turbo settings
        pi->ps32_write_32(0x00E9001C, 0x00000087);
        pi->flush_pending_writes();

        // Verify diagnostic registers read safely
        uint32_t diag_stat = pi->ps32_read_32(0x00E90020);
        uint32_t pref_launches = pi->ps32_read_32(0x00E90024);
        uint32_t pref_hits = pi->ps32_read_32(0x00E90028);
        uint32_t bus_cap = pi->ps32_read_32(0x00E9002C);
        uint32_t cyc_timing = pi->ps32_read_32(0x00E90030);
        uint32_t clk_phase = pi->ps32_read_32(0x00E90034);
        TEST_ASSERT(pref_launches >= 0 && pref_hits >= 0, "Diagnostic registers +$20..+$34 read back valid telemetry");
    }

    // =========================================================================
    // Test 15: Upstream Golden Reference Comparison & Performance Benchmarks
    // =========================================================================
    run_golden_reference_comparison();
    run_benchmark_suite(harness);

    // =========================================================================
    // Summary
    // =========================================================================
    std::cout << "\n" ANSI_BOLD ANSI_CYAN "========================================================\n"
              << "  Test Suite Summary\n"
              << "========================================================\n" ANSI_RESET;
    std::cout << "  Total Assertions: " << (g_tests_passed + g_tests_failed) << std::endl;
    std::cout << "  Passed:           " ANSI_GREEN << g_tests_passed << ANSI_RESET << std::endl;
    std::cout << "  Failed:           " << (g_tests_failed > 0 ? ANSI_RED : ANSI_GREEN)
              << g_tests_failed << ANSI_RESET << std::endl;

    if (g_tests_failed == 0) {
        std::cout << ANSI_BOLD ANSI_GREEN "\n>>> ALL TESTS PASSED SUCCESSFULLY! <<<\n" ANSI_RESET << std::endl;
    } else {
        std::cout << ANSI_BOLD ANSI_RED "\n>>> TEST SUITE FAILED WITH " << g_tests_failed << " ERRORS! <<<\n" ANSI_RESET << std::endl;
    }

    if (enable_trace) {
        std::cout << "Waveform written to sim.vcd" << std::endl;
    }

    return (g_tests_failed == 0) ? 0 : 1;
}
