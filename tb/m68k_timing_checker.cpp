#include "m68k_timing_checker.h"
#include <cmath>

#define ANSI_RESET   "\033[0m"
#define ANSI_BOLD    "\033[1m"
#define ANSI_RED     "\033[31m"
#define ANSI_GREEN   "\033[32m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_CYAN    "\033[36m"

M68kTimingChecker::M68kTimingChecker(const M68kTimingProfile& profile) {
    set_profile(profile);
}

void M68kTimingChecker::set_profile(const M68kTimingProfile& profile) {
    profile_ = profile;
    metrics_.clear();

    metrics_.push_back({"#1",   "Clock Cycle Time (t_cyc)",                  profile_.t_cyc,         true,  1e9, 0, 0});
    metrics_.push_back({"#2",   "Clock Pulse Width High (t_PWH)",            profile_.t_pulse_w_min, true,  1e9, 0, 0});
    metrics_.push_back({"#3",   "Clock Pulse Width Low (t_PWL)",             profile_.t_pulse_w_min, true,  1e9, 0, 0});
    metrics_.push_back({"#6",   "Clock High to Addr Valid (t_CHAV)",         profile_.t6_CHAV_max,   false, -1e9, 0, 0});
    metrics_.push_back({"#9",   "Clock Low to AS Asserted (t_CLSAL)",        profile_.t9_CLSAL_max,  false, -1e9, 0, 0});
    metrics_.push_back({"#9A",  "AS to DS Read Skew (t_SASKW)",              profile_.t9A_SASKW_max, false, -1e9, 0, 0});
    metrics_.push_back({"#9B",  "AS Asserted to DS Asserted Write (t_ASWDS)",profile_.t9B_ASWDS_min, true,  1e9, 0, 0});
    metrics_.push_back({"#11",  "Addr Valid to AS Asserted (t_AVSL)",        profile_.t11_AVSL_min,  true,  1e9, 0, 0});
    metrics_.push_back({"#12",  "Clock Low to AS Negated (t_CLSH)",          profile_.t12_CLSH_max,  false, -1e9, 0, 0});
    metrics_.push_back({"#13",  "AS Negated to Addr Invalid (t_SHAI)",       profile_.t13_SHAI_min,  true,  1e9, 0, 0});
    metrics_.push_back({"#14",  "AS Width Asserted (t_SASH)",                profile_.t14_SASH_min,  true,  1e9, 0, 0});
    metrics_.push_back({"#15",  "AS Width Negated (t_SSNH)",                 profile_.t15_SSNH_min,  true,  1e9, 0, 0});
    metrics_.push_back({"#21",  "R/W High to AS Asserted Read (t_RHAS)",     profile_.t21_RHAS_min,  true,  1e9, 0, 0});
    metrics_.push_back({"#22",  "R/W Low to DS Asserted Write (t_RLDS)",     profile_.t22_RLDS_min,  true,  1e9, 0, 0});
    metrics_.push_back({"#25",  "Data-Out Hold Time after DS (t_DOH)",       profile_.t25_SHDOI_min, true,  1e9, 0, 0});
    metrics_.push_back({"#26",  "Data-Out Valid to DS Asserted (t_DOVDS)",   profile_.t26_DOVDS_min, true,  1e9, 0, 0});
    metrics_.push_back({"#27",  "Data-In Valid to Clock Low (t_DICL)",       profile_.t27_DICL_min,  true,  1e9, 0, 0});
    metrics_.push_back({"#28",  "AS Negated to DSACK Negated (t_SHDAI)",     profile_.t28_SHDAI_max, false, -1e9, 0, 0});
    metrics_.push_back({"#47A", "Async Input Setup to Clock Low (t_ASIS)",   profile_.t47A_ASIS_min, true,  1e9, 0, 0});
}

void M68kTimingChecker::record_sample(const std::string& param_id, double val_ns) {
    for (auto& m : metrics_) {
        if (m.param_id == param_id) {
            m.sample_count++;
            if (m.is_min_limit) {
                if (val_ns < m.worst_observed_ns) m.worst_observed_ns = val_ns;
                if (val_ns < m.limit_ns - 1e-4) m.violation_count++;
            } else {
                if (val_ns > m.worst_observed_ns) m.worst_observed_ns = val_ns;
                if (val_ns > m.limit_ns + 1e-4) m.violation_count++;
            }
            return;
        }
    }
}

void M68kTimingChecker::on_clk_edge(bool rising, double now_ns) {
    if (rising) {
        if (t_clk_rising_ >= 0) {
            double cyc = now_ns - t_clk_rising_;
            record_sample("#1", cyc);
        }
        if (t_clk_falling_ >= 0) {
            double pwl = now_ns - t_clk_falling_;
            record_sample("#3", pwl);
        }
        t_prev_clk_rising_ = t_clk_rising_;
        t_clk_rising_ = now_ns;
    } else {
        // Falling edge
        if (t_clk_rising_ >= 0) {
            double pwh = now_ns - t_clk_rising_;
            record_sample("#2", pwh);
        }
        t_prev_clk_falling_ = t_clk_falling_;
        t_clk_falling_ = now_ns;

        // Check asynchronous input setup for DSACK if asserted recently
        if (t_dsack_asserted_ >= 0 && (now_ns - t_dsack_asserted_) < (profile_.t_cyc * 1.5)) {
            double setup = now_ns - t_dsack_asserted_;
            record_sample("#47A", setup);
        }
    }
}

void M68kTimingChecker::on_addr_change(uint32_t /*new_addr*/, uint8_t /*new_fc*/, uint8_t /*new_size*/, double now_ns) {
    t_addr_valid_ = now_ns;
    if (t_clk_rising_ >= 0) {
        double delay = now_ns - t_clk_rising_;
        if (delay >= 0 && delay < profile_.t_cyc) {
            record_sample("#6", delay);
        }
    }

    if (t_as_negated_ >= 0) {
        double hold = now_ns - t_as_negated_;
        record_sample("#13", hold);
    }
}

void M68kTimingChecker::on_as_change(bool asserted_low, double now_ns) {
    as_is_low_ = asserted_low;
    if (asserted_low) {
        // AS falling edge (asserted)
        t_as_asserted_ = now_ns;

        if (t_addr_valid_ >= 0) {
            double tavsl = now_ns - t_addr_valid_;
            record_sample("#11", tavsl);
        }

        if (t_clk_falling_ >= 0) {
            double tclsal = now_ns - t_clk_falling_;
            if (tclsal >= 0 && tclsal < profile_.t_cyc) {
                record_sample("#9", tclsal);
            }
        }

        if (current_rw_is_read_ && t_rw_high_ >= 0) {
            double trhas = now_ns - t_rw_high_;
            record_sample("#21", trhas);
        }

        if (t_prev_as_negated_ >= 0) {
            double tssnh = now_ns - t_prev_as_negated_;
            record_sample("#15", tssnh);
        }
    } else {
        // AS rising edge (negated)
        t_prev_as_negated_ = t_as_negated_;
        t_as_negated_ = now_ns;

        if (t_as_asserted_ >= 0) {
            double tsash = now_ns - t_as_asserted_;
            record_sample("#14", tsash);
        }

        if (t_clk_falling_ >= 0) {
            double tclsh = now_ns - t_clk_falling_;
            if (tclsh >= 0 && tclsh < profile_.t_cyc) {
                record_sample("#12", tclsh);
            }
        }
    }
}

void M68kTimingChecker::on_ds_change(bool asserted_low, double now_ns) {
    ds_is_low_ = asserted_low;
    if (asserted_low) {
        t_ds_asserted_ = now_ns;
        if (!current_rw_is_read_) {
            // Write cycle
            if (t_as_asserted_ >= 0) {
                double taswds = now_ns - t_as_asserted_;
                record_sample("#9B", taswds);
            }
            if (t_rw_low_ >= 0) {
                double trlds = now_ns - t_rw_low_;
                record_sample("#22", trlds);
            }
            if (t_data_out_valid_ >= 0) {
                double tdovds = now_ns - t_data_out_valid_;
                record_sample("#26", tdovds);
            }
        } else {
            // Read cycle: AS to DS assertion skew (#9A)
            if (t_as_asserted_ >= 0) {
                double skew = std::abs(now_ns - t_as_asserted_);
                record_sample("#9A", skew);
            }
        }
    } else {
        t_ds_negated_ = now_ns;
    }
}

void M68kTimingChecker::on_rw_change(bool read_high, double now_ns) {
    current_rw_is_read_ = read_high;
    if (read_high) {
        t_rw_high_ = now_ns;
    } else {
        t_rw_low_ = now_ns;
    }
}

void M68kTimingChecker::on_data_out_change(uint32_t /*data*/, double now_ns) {
    t_data_out_valid_ = now_ns;
}

void M68kTimingChecker::on_data_out_disabled(double now_ns) {
    if (!current_rw_is_read_ && t_ds_negated_ >= 0) {
        double tdoh = now_ns - t_ds_negated_;
        if (tdoh >= 0 && tdoh < profile_.t_cyc * 2) {
            record_sample("#25", tdoh);
        }
    }
}

void M68kTimingChecker::on_data_in_change(uint32_t /*data*/, double now_ns) {
    t_data_in_valid_ = now_ns;
}

void M68kTimingChecker::on_dsack_change(uint8_t dsack_n, double now_ns) {
    if (dsack_n != 3) {
        t_dsack_asserted_ = now_ns;
    } else {
        t_dsack_negated_ = now_ns;
        if (t_as_negated_ >= 0) {
            double tshdai = now_ns - t_as_negated_;
            if (tshdai >= 0 && tshdai < profile_.t_cyc * 2) {
                record_sample("#28", tshdai);
            }
        }
    }
}

void M68kTimingChecker::on_berr_change(bool /*asserted_low*/, double /*now_ns*/) {
}

void M68kTimingChecker::on_sample_read_data(double now_ns) {
    if (t_data_in_valid_ >= 0) {
        double tdicl = now_ns - t_data_in_valid_;
        record_sample("#27", tdicl);
    }
}

bool M68kTimingChecker::has_violations() const {
    return get_total_violations() > 0;
}

uint64_t M68kTimingChecker::get_total_violations() const {
    uint64_t total = 0;
    for (const auto& m : metrics_) {
        total += m.violation_count;
    }
    return total;
}

void M68kTimingChecker::print_timing_report(std::ostream& os) const {
    os << "\n" ANSI_BOLD ANSI_CYAN
       << "=========================================================================================================\n"
       << "  Motorola MC68020 AC Timing Compliance Report (Profile: " << profile_.name << ")\n"
       << "  Reference: MC68020UM.pdf Section 10, Table 10-1\n"
       << "=========================================================================================================\n" ANSI_RESET;

    os << ANSI_BOLD
       << std::left << std::setw(6)  << "Num"
       << std::setw(42) << "Characteristic"
       << std::setw(14) << "Datasheet Limit"
       << std::setw(16) << "Worst Observed"
       << std::setw(12) << "Margin"
       << std::setw(10) << "Status"
       << ANSI_RESET << "\n";

    os << std::string(105, '-') << "\n";

    for (const auto& m : metrics_) {
        if (m.sample_count == 0) continue;

        os << std::left << std::setw(6) << m.param_id
           << std::setw(42) << m.name;

        std::string limit_str = (m.is_min_limit ? ">= " : "<= ") +
                                ([&]{ std::stringstream ss; ss << std::fixed << std::setprecision(1) << m.limit_ns << " ns"; return ss.str(); }());
        os << std::setw(14) << limit_str;

        std::string obs_str = ([&]{ std::stringstream ss; ss << std::fixed << std::setprecision(1) << m.worst_observed_ns << " ns"; return ss.str(); }());
        os << std::setw(16) << obs_str;

        double margin = m.get_margin();
        std::string margin_str = (margin >= 0 ? "+" : "") +
                                 ([&]{ std::stringstream ss; ss << std::fixed << std::setprecision(1) << margin << " ns"; return ss.str(); }());
        os << std::setw(12) << margin_str;

        if (m.is_passing()) {
            os << ANSI_GREEN "[PASS]" ANSI_RESET "\n";
        } else {
            os << ANSI_RED "[VIOLATION (" << m.violation_count << ")]" ANSI_RESET "\n";
        }
    }

    os << std::string(105, '-') << "\n";
    if (!has_violations()) {
        os << ANSI_BOLD ANSI_GREEN "  AC TIMING VERIFICATION: 100% COMPLIANT WITH MOTOROLA MC68020 SPECIFICATION!\n" ANSI_RESET;
    } else {
        os << ANSI_BOLD ANSI_RED "  AC TIMING VERIFICATION: DETECTED " << get_total_violations() << " VIOLATION(S)!\n" ANSI_RESET;
    }
    os << "=========================================================================================================\n" << std::endl;
}
