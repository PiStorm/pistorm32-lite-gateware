#ifndef M68K_TIMING_PROFILE_H
#define M68K_TIMING_PROFILE_H

#include <string>
#include <cstdint>

// Speed Grade enumeration
enum class M68kSpeedGrade {
    GRADE_16MHZ,  // 16.67 MHz standard (Amiga baseline / standard 68020)
    GRADE_20MHZ,  // 20.00 MHz
    GRADE_25MHZ,  // 25.00 MHz
    GRADE_33MHZ,  // 33.33 MHz
    GRADE_A1200_14MHZ, // Amiga 1200 stock 14.18 MHz
    GRADE_A1200_28MHZ  // Amiga 1200 28.36 MHz
};

// AC Timing Parameters structure containing timing limits in nanoseconds (double)
// All parameters directly match Table 10-1 from MC68020UM.pdf
struct M68kTimingProfile {
    std::string name;
    double freq_mhz;
    double t_cyc;         // #1 Cycle time (ns)
    double t_pulse_w_min; // #2, #3 Clock pulse width min (ns)
    double t_pulse_w_max; // #2, #3 Clock pulse width max (ns)
    double t_rf_max;      // #4, #5 Rise/Fall max (ns)

    // Address & Control
    double t6_CHAV_max;   // #6 Clock High to Addr/FC/Size/RMC valid max (ns)
    double t8_CHAZ_min;   // #8 Clock High to Addr/FC invalid min (ns)
    double t9_CLSAL_min;  // #9 Clock Low to AS, DS asserted min (ns)
    double t9_CLSAL_max;  // #9 Clock Low to AS, DS asserted max (ns)
    double t9A_SASKW_max; // #9A AS to DS assertion skew max (+/- ns)
    double t9B_ASWDS_min; // #9B AS asserted to DS asserted (write) min (ns)
    double t11_AVSL_min;  // #11 Address/FC valid to AS asserted min (ns)
    double t12_CLSH_max;  // #12 Clock Low to AS, DS negated max (ns)
    double t13_SHAI_min;  // #13 AS, DS negated to Address invalid min (ns)
    double t14_SASH_min;  // #14 AS width asserted min (ns)
    double t14A_DSW_min;  // #14A DS width asserted (write) min (ns)
    double t15_SSNH_min;  // #15 AS, DS width negated min (ns)

    // Read / Write
    double t17_SHRWI_min; // #17 AS, DS negated to R/W invalid min (ns)
    double t18_CHRH_max;  // #18 Clock High to R/W High max (ns)
    double t20_CHRL_max;  // #20 Clock High to R/W Low max (ns)
    double t21_RHAS_min;  // #21 R/W High to AS asserted (read) min (ns)
    double t22_RLDS_min;  // #22 R/W Low to DS asserted (write) min (ns)
    double t23_CHDOV_max; // #23 Clock High to Data-Out valid max (ns)
    double t25_SHDOI_min; // #25 AS, DS negated to Data-Out invalid min (ns)
    double t26_DOVDS_min; // #26 Data-Out valid to DS asserted (write) min (ns)

    // Data-In & DSACK
    double t27_DICL_min;  // #27 Data-In setup to Clock Low min (ns)
    double t28_SHDAI_max; // #28 AS, DS negated to DSACK/BERR negated max (ns)
    double t29_SHDII_min; // #29 AS, DS negated to Data-In invalid min (ns)
    double t30_CLDII_min; // #30 Clock Low to Data-In invalid hold min (ns)
    double t31_DSACKDV_max; // #31 DSACK asserted to Data-In valid max (ns)
    double t31A_DSACKSKW_max;// #31A DSACK skew max (ns)

    // Arbitration
    double t33_CLBGA_max; // #33 Clock Low to BG asserted max (ns)
    double t34_CLBGN_max; // #34 Clock Low to BG negated max (ns)

    // Asynchronous Setup / Hold
    double t47A_ASIS_min; // #47A Asynchronous input setup time min (ns)
    double t47B_ASIH_min; // #47B Asynchronous input hold time min (ns)
    double t48_DSACKBERR_max; // #48 DSACK asserted to BERR asserted max (ns)
    double t53_CHDOI_min; // #53 Data-Out hold from Clock High min (ns)

    static M68kTimingProfile get(M68kSpeedGrade grade) {
        M68kTimingProfile p;
        switch (grade) {
            case M68kSpeedGrade::GRADE_16MHZ:
                p.name = "MC68020-16.67MHz";
                p.freq_mhz = 16.666667;
                p.t_cyc = 60.0;
                p.t_pulse_w_min = 24.0;
                p.t_pulse_w_max = 95.0;
                p.t_rf_max = 5.0;
                p.t6_CHAV_max = 30.0;
                p.t8_CHAZ_min = 0.0;
                p.t9_CLSAL_min = 3.0;
                p.t9_CLSAL_max = 30.0;
                p.t9A_SASKW_max = 15.0;
                p.t9B_ASWDS_min = 37.0;
                p.t11_AVSL_min = 15.0;
                p.t12_CLSH_max = 30.0;
                p.t13_SHAI_min = 15.0;
                p.t14_SASH_min = 100.0;
                p.t14A_DSW_min = 40.0;
                p.t15_SSNH_min = 40.0;
                p.t17_SHRWI_min = 15.0;
                p.t18_CHRH_max = 30.0;
                p.t20_CHRL_max = 30.0;
                p.t21_RHAS_min = 15.0;
                p.t22_RLDS_min = 75.0;
                p.t23_CHDOV_max = 30.0;
                p.t25_SHDOI_min = 15.0;
                p.t26_DOVDS_min = 15.0;
                p.t27_DICL_min = 5.0;
                p.t28_SHDAI_max = 80.0;
                p.t29_SHDII_min = 0.0;
                p.t30_CLDII_min = 15.0;
                p.t31_DSACKDV_max = 50.0;
                p.t31A_DSACKSKW_max = 15.0;
                p.t33_CLBGA_max = 30.0;
                p.t34_CLBGN_max = 30.0;
                p.t47A_ASIS_min = 5.0;
                p.t47B_ASIH_min = 15.0;
                p.t48_DSACKBERR_max = 30.0;
                p.t53_CHDOI_min = 0.0;
                break;

            case M68kSpeedGrade::GRADE_20MHZ:
                p.name = "MC68020-20.00MHz";
                p.freq_mhz = 20.0;
                p.t_cyc = 50.0;
                p.t_pulse_w_min = 20.0;
                p.t_pulse_w_max = 54.0;
                p.t_rf_max = 5.0;
                p.t6_CHAV_max = 25.0;
                p.t8_CHAZ_min = 0.0;
                p.t9_CLSAL_min = 3.0;
                p.t9_CLSAL_max = 25.0;
                p.t9A_SASKW_max = 10.0;
                p.t9B_ASWDS_min = 32.0;
                p.t11_AVSL_min = 10.0;
                p.t12_CLSH_max = 25.0;
                p.t13_SHAI_min = 10.0;
                p.t14_SASH_min = 85.0;
                p.t14A_DSW_min = 38.0;
                p.t15_SSNH_min = 38.0;
                p.t17_SHRWI_min = 10.0;
                p.t18_CHRH_max = 25.0;
                p.t20_CHRL_max = 25.0;
                p.t21_RHAS_min = 10.0;
                p.t22_RLDS_min = 60.0;
                p.t23_CHDOV_max = 25.0;
                p.t25_SHDOI_min = 10.0;
                p.t26_DOVDS_min = 10.0;
                p.t27_DICL_min = 5.0;
                p.t28_SHDAI_max = 65.0;
                p.t29_SHDII_min = 0.0;
                p.t30_CLDII_min = 15.0;
                p.t31_DSACKDV_max = 43.0;
                p.t31A_DSACKSKW_max = 10.0;
                p.t33_CLBGA_max = 25.0;
                p.t34_CLBGN_max = 25.0;
                p.t47A_ASIS_min = 5.0;
                p.t47B_ASIH_min = 15.0;
                p.t48_DSACKBERR_max = 20.0;
                p.t53_CHDOI_min = 0.0;
                break;

            case M68kSpeedGrade::GRADE_25MHZ:
                p.name = "MC68020-25.00MHz";
                p.freq_mhz = 25.0;
                p.t_cyc = 40.0;
                p.t_pulse_w_min = 19.0;
                p.t_pulse_w_max = 61.0;
                p.t_rf_max = 4.0;
                p.t6_CHAV_max = 25.0;
                p.t8_CHAZ_min = 0.0;
                p.t9_CLSAL_min = 3.0;
                p.t9_CLSAL_max = 18.0;
                p.t9A_SASKW_max = 10.0;
                p.t9B_ASWDS_min = 27.0;
                p.t11_AVSL_min = 6.0;
                p.t12_CLSH_max = 15.0;
                p.t13_SHAI_min = 10.0;
                p.t14_SASH_min = 70.0;
                p.t14A_DSW_min = 30.0;
                p.t15_SSNH_min = 30.0;
                p.t17_SHRWI_min = 10.0;
                p.t18_CHRH_max = 20.0;
                p.t20_CHRL_max = 20.0;
                p.t21_RHAS_min = 5.0;
                p.t22_RLDS_min = 50.0;
                p.t23_CHDOV_max = 25.0;
                p.t25_SHDOI_min = 5.0;
                p.t26_DOVDS_min = 5.0;
                p.t27_DICL_min = 5.0;
                p.t28_SHDAI_max = 50.0;
                p.t29_SHDII_min = 0.0;
                p.t30_CLDII_min = 10.0;
                p.t31_DSACKDV_max = 32.0;
                p.t31A_DSACKSKW_max = 10.0;
                p.t33_CLBGA_max = 20.0;
                p.t34_CLBGN_max = 20.0;
                p.t47A_ASIS_min = 5.0;
                p.t47B_ASIH_min = 10.0;
                p.t48_DSACKBERR_max = 18.0;
                p.t53_CHDOI_min = 0.0;
                break;

            case M68kSpeedGrade::GRADE_33MHZ:
                p.name = "MC68020-33.33MHz";
                p.freq_mhz = 33.333333;
                p.t_cyc = 30.0;
                p.t_pulse_w_min = 14.0;
                p.t_pulse_w_max = 66.0;
                p.t_rf_max = 3.0;
                p.t6_CHAV_max = 21.0;
                p.t8_CHAZ_min = 0.0;
                p.t9_CLSAL_min = 3.0;
                p.t9_CLSAL_max = 15.0;
                p.t9A_SASKW_max = 10.0;
                p.t9B_ASWDS_min = 22.0;
                p.t11_AVSL_min = 5.0;
                p.t12_CLSH_max = 15.0;
                p.t13_SHAI_min = 5.0;
                p.t14_SASH_min = 50.0;
                p.t14A_DSW_min = 25.0;
                p.t15_SSNH_min = 23.0;
                p.t17_SHRWI_min = 5.0;
                p.t18_CHRH_max = 15.0;
                p.t20_CHRL_max = 15.0;
                p.t21_RHAS_min = 5.0;
                p.t22_RLDS_min = 35.0;
                p.t23_CHDOV_max = 18.0;
                p.t25_SHDOI_min = 5.0;
                p.t26_DOVDS_min = 5.0;
                p.t27_DICL_min = 5.0;
                p.t28_SHDAI_max = 40.0;
                p.t29_SHDII_min = 0.0;
                p.t30_CLDII_min = 10.0;
                p.t31_DSACKDV_max = 17.0;
                p.t31A_DSACKSKW_max = 10.0;
                p.t33_CLBGA_max = 20.0;
                p.t34_CLBGN_max = 20.0;
                p.t47A_ASIS_min = 5.0;
                p.t47B_ASIH_min = 10.0;
                p.t48_DSACKBERR_max = 15.0;
                p.t53_CHDOI_min = 0.0;
                break;

            case M68kSpeedGrade::GRADE_A1200_14MHZ:
                p = get(M68kSpeedGrade::GRADE_16MHZ);
                p.name = "Amiga1200-14.18MHz";
                p.freq_mhz = 14.18758;
                p.t_cyc = 70.48;
                p.t_pulse_w_min = 30.0;
                p.t_pulse_w_max = 110.0;
                p.t14_SASH_min = 110.0;
                p.t15_SSNH_min = 45.0;
                break;

            case M68kSpeedGrade::GRADE_A1200_28MHZ:
                p = get(M68kSpeedGrade::GRADE_25MHZ);
                p.name = "Amiga1200-28.36MHz";
                p.freq_mhz = 28.37516;
                p.t_cyc = 35.24;
                p.t_pulse_w_min = 15.0;
                p.t_pulse_w_max = 55.0;
                p.t14_SASH_min = 60.0;
                p.t15_SSNH_min = 25.0;
                break;
        }
        return p;
    }
};

#endif // M68K_TIMING_PROFILE_H
