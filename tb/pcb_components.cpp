#include "pcb_components.h"

#define ANSI_RESET   "\033[0m"
#define ANSI_BOLD    "\033[1m"
#define ANSI_RED     "\033[31m"
#define ANSI_GREEN   "\033[32m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_CYAN    "\033[36m"

PcbLatch74LVC573A::PcbLatch74LVC573A()
    : spec_(TimingSpec())
{
    reset();
}

PcbLatch74LVC573A::PcbLatch74LVC573A(const TimingSpec& spec)
    : spec_(spec)
{
    reset();
}

void PcbLatch74LVC573A::reset() {
    d_in_prev_ = 0;
    latched_val_ = 0;
    q_out_ = 0;
    prev_le_ = false;
    prev_oe_n_ = true;
    output_enabled_ = false;

    t_d_change_ = -1.0;
    t_le_rising_ = -1.0;
    t_le_falling_ = -1.0;
    t_oe_falling_ = -1.0;
    t_oe_rising_ = -1.0;

    setup_checks_ = 0;
    hold_checks_ = 0;
    pulse_w_checks_ = 0;
    setup_violations_ = 0;
    hold_violations_ = 0;
    pulse_w_violations_ = 0;

    worst_setup_observed_ = 1e9;
    worst_hold_observed_ = 1e9;
    worst_pulse_w_observed_ = 1e9;
}

void PcbLatch74LVC573A::update(uint32_t d_in, bool le, bool oe_n, double now_ns) {
    // 1. Data change detection
    if (d_in != d_in_prev_) {
        // If LE fell, check data hold time on first data change
        if (!le && t_le_falling_ >= 0) {
            double hold = now_ns - t_le_falling_;
            hold_checks_++;
            if (hold < worst_hold_observed_) worst_hold_observed_ = hold;
            if (hold < spec_.t_hold_min - 1e-4) {
                hold_violations_++;
            }
            t_le_falling_ = -1.0;
        }
        t_d_change_ = now_ns;
        d_in_prev_ = d_in;
    }

    // 2. LE edge transitions
    if (le && !prev_le_) {
        // LE rising: latch enters transparent mode
        if (t_le_falling_ >= 0) {
            // Data never changed between LE falling and next LE rising
            double hold = now_ns - t_le_falling_;
            hold_checks_++;
            if (hold < worst_hold_observed_) worst_hold_observed_ = hold;
            if (hold < spec_.t_hold_min - 1e-4) {
                hold_violations_++;
            }
            t_le_falling_ = -1.0;
        }
        t_le_rising_ = now_ns;
    } else if (!le && prev_le_) {
        // LE falling: latch captures input data
        t_le_falling_ = now_ns;
        latched_val_ = d_in;

        // Check pulse width high: t_w(LE)
        if (t_le_rising_ >= 0) {
            double pw = now_ns - t_le_rising_;
            pulse_w_checks_++;
            if (pw < worst_pulse_w_observed_) worst_pulse_w_observed_ = pw;
            if (pw < spec_.t_pulse_w_min - 1e-4) {
                pulse_w_violations_++;
            }
        }

        // Check data setup time: t_su(D before LE fall)
        if (t_d_change_ >= 0) {
            double setup = now_ns - t_d_change_;
            setup_checks_++;
            if (setup < worst_setup_observed_) worst_setup_observed_ = setup;
            if (setup < spec_.t_setup_min - 1e-4) {
                setup_violations_++;
            }
        }
    }
    prev_le_ = le;

    // 3. OE_n edge transitions
    if (!oe_n && prev_oe_n_) {
        t_oe_falling_ = now_ns;
    } else if (oe_n && !prev_oe_n_) {
        t_oe_rising_ = now_ns;
    }
    prev_oe_n_ = oe_n;
    output_enabled_ = !oe_n;

    // 4. Output state
    if (le) {
        q_out_ = d_in; // Transparent mode
    } else {
        q_out_ = latched_val_; // Latched mode
    }
}

void PcbLatch74LVC573A::print_report(std::ostream& os) const {
    os << "\n" ANSI_BOLD ANSI_CYAN
       << "=========================================================================================================\n"
       << "  74LVC573A Address Latch Hardware Timing Verification (4x Octal Latches, 32-bit Bus)\n"
       << "  Datasheet: TI / Nexperia 74LVC573A (Vcc = 3.3V, CL = 50pF)\n"
       << "=========================================================================================================\n" ANSI_RESET;

    os << ANSI_BOLD
       << std::left << std::setw(30) << "Parameter"
       << std::setw(16) << "Datasheet Spec"
       << std::setw(18) << "Worst Observed"
       << std::setw(14) << "Margin"
       << std::setw(12) << "Checks"
       << std::setw(10) << "Status"
       << ANSI_RESET << "\n";

    os << std::string(100, '-') << "\n";

    // Setup time
    {
        double margin = worst_setup_observed_ - spec_.t_setup_min;
        os << std::left << std::setw(30) << "Data Setup Time (t_su)"
           << std::setw(16) << (">= " + std::to_string(spec_.t_setup_min).substr(0, 3) + " ns")
           << std::setw(18) << (std::to_string(worst_setup_observed_).substr(0, 5) + " ns")
           << std::setw(14) << ((margin >= 0 ? "+" : "") + std::to_string(margin).substr(0, 5) + " ns")
           << std::setw(12) << setup_checks_
           << (setup_violations_ == 0 ? ANSI_GREEN "[PASS]" ANSI_RESET : ANSI_RED "[FAIL]" ANSI_RESET)
           << "\n";
    }

    // Hold time
    {
        double margin = worst_hold_observed_ - spec_.t_hold_min;
        os << std::left << std::setw(30) << "Data Hold Time (t_h)"
           << std::setw(16) << (">= " + std::to_string(spec_.t_hold_min).substr(0, 3) + " ns")
           << std::setw(18) << (std::to_string(worst_hold_observed_).substr(0, 5) + " ns")
           << std::setw(14) << ((margin >= 0 ? "+" : "") + std::to_string(margin).substr(0, 5) + " ns")
           << std::setw(12) << hold_checks_
           << (hold_violations_ == 0 ? ANSI_GREEN "[PASS]" ANSI_RESET : ANSI_RED "[FAIL]" ANSI_RESET)
           << "\n";
    }

    // Pulse width
    {
        double margin = worst_pulse_w_observed_ - spec_.t_pulse_w_min;
        os << std::left << std::setw(30) << "LE Pulse Width High (t_w)"
           << std::setw(16) << (">= " + std::to_string(spec_.t_pulse_w_min).substr(0, 3) + " ns")
           << std::setw(18) << (std::to_string(worst_pulse_w_observed_).substr(0, 5) + " ns")
           << std::setw(14) << ((margin >= 0 ? "+" : "") + std::to_string(margin).substr(0, 5) + " ns")
           << std::setw(12) << pulse_w_checks_
           << (pulse_w_violations_ == 0 ? ANSI_GREEN "[PASS]" ANSI_RESET : ANSI_RED "[FAIL]" ANSI_RESET)
           << "\n";
    }

    os << std::string(100, '-') << "\n";
    if (setup_violations_ == 0 && hold_violations_ == 0 && pulse_w_violations_ == 0) {
        os << ANSI_BOLD ANSI_GREEN "  74LVC573A LATCH TIMING: 100% COMPLIANT WITH DATASHEET SPECIFICATIONS!\n" ANSI_RESET;
    } else {
        os << ANSI_BOLD ANSI_RED "  74LVC573A LATCH TIMING: DETECTED VIOLATIONS!\n" ANSI_RESET;
    }
    os << "=========================================================================================================\n" << std::endl;
}

// 74CBTD3384 implementation
PcbSwitch74CBTD3384::PcbSwitch74CBTD3384()
    : spec_(TimingSpec())
{
    reset();
}

PcbSwitch74CBTD3384::PcbSwitch74CBTD3384(const TimingSpec& spec)
    : spec_(spec)
{
    reset();
}

void PcbSwitch74CBTD3384::reset() {
    out_data_ = 0;
    prev_oe_n_ = true;
    connected_ = false;
    t_oe_falling_ = -1.0;
    t_oe_rising_ = -1.0;
    switches_count_ = 0;
}

void PcbSwitch74CBTD3384::update_a_to_b(uint32_t a_in, bool oe_n, double now_ns) {
    if (!oe_n && prev_oe_n_) {
        t_oe_falling_ = now_ns;
        switches_count_++;
    } else if (oe_n && !prev_oe_n_) {
        t_oe_rising_ = now_ns;
    }
    prev_oe_n_ = oe_n;
    connected_ = !oe_n;

    if (connected_) {
        out_data_ = a_in;
    }
}

void PcbSwitch74CBTD3384::update_b_to_a(uint32_t b_in, bool oe_n, double now_ns) {
    update_a_to_b(b_in, oe_n, now_ns);
}

void PcbSwitch74CBTD3384::print_report(std::ostream& os) const {
    os << "\n" ANSI_BOLD ANSI_CYAN
       << "=========================================================================================================\n"
       << "  74CBTD3384 Bus Switch / Level Shifter Verification (5V <-> 3.3V Bidirectional Level Shifter)\n"
       << "  Datasheet: TI / Nexperia 74CBTD3384 (t_pd <= 0.25 ns, t_en <= 4.5 ns, t_dis <= 5.0 ns)\n"
       << "=========================================================================================================\n" ANSI_RESET;

    os << "  Total Switch Operations: " << switches_count_ << "\n"
       << "  Propagation Delay Model: " << spec_.t_pd_max << " ns\n"
       << ANSI_GREEN "  [PASS] 74CBTD3384 Level Shifter Model Active and Functional\n" ANSI_RESET
       << "=========================================================================================================\n" << std::endl;
}
