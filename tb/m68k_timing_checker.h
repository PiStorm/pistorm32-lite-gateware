#ifndef M68K_TIMING_CHECKER_H
#define M68K_TIMING_CHECKER_H

#include "m68k_timing_profile.h"
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>

struct TimingMetric {
    std::string param_id;
    std::string name;
    double limit_ns;
    bool is_min_limit; // true if limit is a minimum required time, false if maximum allowable
    double worst_observed_ns;
    uint64_t violation_count;
    uint64_t sample_count;

    double get_margin() const {
        if (sample_count == 0) return 0.0;
        return is_min_limit ? (worst_observed_ns - limit_ns) : (limit_ns - worst_observed_ns);
    }

    bool is_passing() const {
        return violation_count == 0;
    }
};

class M68kTimingChecker {
public:
    explicit M68kTimingChecker(const M68kTimingProfile& profile);

    void set_profile(const M68kTimingProfile& profile);
    const M68kTimingProfile& get_profile() const { return profile_; }

    // Signal transition observers called with physical time in nanoseconds
    void on_clk_edge(bool rising, double now_ns);
    void on_addr_change(uint32_t new_addr, uint8_t new_fc, uint8_t new_size, double now_ns);
    void on_as_change(bool asserted_low, double now_ns);
    void on_ds_change(bool asserted_low, double now_ns);
    void on_rw_change(bool read_high, double now_ns);
    void on_data_out_change(uint32_t data, double now_ns);
    void on_data_out_disabled(double now_ns);
    void on_data_in_change(uint32_t data, double now_ns);
    void on_dsack_change(uint8_t dsack_n, double now_ns);
    void on_berr_change(bool asserted_low, double now_ns);

    // Called on sampling edge (falling clock edge when cycle terminates)
    void on_sample_read_data(double now_ns);

    // Reporting
    void print_timing_report(std::ostream& os = std::cout) const;
    bool has_violations() const;
    uint64_t get_total_violations() const;

private:
    void record_sample(const std::string& param_id, double val_ns);

    M68kTimingProfile profile_;

    // Timestamps of past signal transitions (nanoseconds)
    double t_clk_rising_ = -1.0;
    double t_clk_falling_ = -1.0;
    double t_prev_clk_rising_ = -1.0;
    double t_prev_clk_falling_ = -1.0;

    double t_addr_valid_ = -1.0;
    double t_as_asserted_ = -1.0;
    double t_as_negated_ = -1.0;
    double t_prev_as_negated_ = -1.0;

    double t_ds_asserted_ = -1.0;
    double t_ds_negated_ = -1.0;

    double t_rw_high_ = -1.0;
    double t_rw_low_ = -1.0;

    double t_data_out_valid_ = -1.0;
    double t_data_in_valid_ = -1.0;

    double t_dsack_asserted_ = -1.0;
    double t_dsack_negated_ = -1.0;

    bool current_rw_is_read_ = true;
    bool as_is_low_ = false;
    bool ds_is_low_ = false;

    std::vector<TimingMetric> metrics_;
};

#endif // M68K_TIMING_CHECKER_H
