#ifndef PCB_COMPONENTS_H
#define PCB_COMPONENTS_H

#include <cstdint>
#include <string>
#include <iostream>
#include <iomanip>

// ============================================================================
// 74LVC573A: Octal Transparent D-Type Latch with 3-State Outputs
// Used in a group of 4 chips on the PiStorm32-lite to form a 32-bit Address Latch
// Datasheet: TI SN74LVC573A / Nexperia 74LVC573A (Vcc = 3.3V)
// ============================================================================
class PcbLatch74LVC573A {
public:
    struct TimingSpec {
        double t_pd_dq_max = 5.0;   // D to Q propagation delay (ns)
        double t_pd_leq_max = 5.5;  // LE to Q propagation delay (ns)
        double t_en_max = 5.5;      // OE_n to Q active enable time (ns)
        double t_dis_max = 5.5;     // OE_n to Q high-Z disable time (ns)
        double t_setup_min = 2.0;   // D setup time before LE falling edge (ns)
        double t_hold_min = 1.5;    // D hold time after LE falling edge (ns)
        double t_pulse_w_min = 3.0; // LE pulse width high min (ns)
    };

    PcbLatch74LVC573A();
    explicit PcbLatch74LVC573A(const TimingSpec& spec);

    void reset();

    // Call on signal inputs update
    void update(uint32_t d_in, bool le, bool oe_n, double now_ns);

    // Outputs
    uint32_t get_q() const { return q_out_; }
    bool is_output_enabled() const { return output_enabled_; }

    // Timing check statistics
    uint64_t get_setup_violations() const { return setup_violations_; }
    uint64_t get_hold_violations() const { return hold_violations_; }
    uint64_t get_pulse_w_violations() const { return pulse_w_violations_; }
    double get_worst_setup_observed() const { return worst_setup_observed_; }
    double get_worst_hold_observed() const { return worst_hold_observed_; }
    double get_worst_pulse_w_observed() const { return worst_pulse_w_observed_; }

    void print_report(std::ostream& os = std::cout) const;

private:
    TimingSpec spec_;

    uint32_t d_in_prev_ = 0;
    uint32_t latched_val_ = 0;
    uint32_t q_out_ = 0;
    bool prev_le_ = false;
    bool prev_oe_n_ = true;
    bool output_enabled_ = false;

    // Timestamps (ns)
    double t_d_change_ = -1.0;
    double t_le_rising_ = -1.0;
    double t_le_falling_ = -1.0;
    double t_oe_falling_ = -1.0;
    double t_oe_rising_ = -1.0;

    // Timing statistics
    uint64_t setup_checks_ = 0;
    uint64_t hold_checks_ = 0;
    uint64_t pulse_w_checks_ = 0;

    uint64_t setup_violations_ = 0;
    uint64_t hold_violations_ = 0;
    uint64_t pulse_w_violations_ = 0;

    double worst_setup_observed_ = 1e9;
    double worst_hold_observed_ = 1e9;
    double worst_pulse_w_observed_ = 1e9;
};

// ============================================================================
// 74CBTD3384: 10-Bit FET Bus Switch with Level Shifter (5V TTL <-> 3.3V LVCMOS)
// Used on PiStorm32-lite for 32-bit Data bus (4 chips) and Control lines (2 chips)
// Datasheet: TI SN74CBTD3384 / Nexperia 74CBTD3384
// ============================================================================
class PcbSwitch74CBTD3384 {
public:
    struct TimingSpec {
        double t_pd_max = 0.25;  // Intrinsic FET switch delay (ns)
        double t_en_max = 4.5;   // OE_n to switch connected (ns)
        double t_dis_max = 5.0;  // OE_n to switch high-Z (ns)
    };

    PcbSwitch74CBTD3384();
    explicit PcbSwitch74CBTD3384(const TimingSpec& spec);

    void reset();

    void update_a_to_b(uint32_t a_in, bool oe_n, double now_ns);
    void update_b_to_a(uint32_t b_in, bool oe_n, double now_ns);

    uint32_t get_out() const { return out_data_; }
    bool is_connected() const { return connected_; }

    void print_report(std::ostream& os = std::cout) const;

private:
    TimingSpec spec_;
    uint32_t out_data_ = 0;
    bool prev_oe_n_ = true;
    bool connected_ = false;

    double t_oe_falling_ = -1.0;
    double t_oe_rising_ = -1.0;
    uint64_t switches_count_ = 0;
};

#endif // PCB_COMPONENTS_H
