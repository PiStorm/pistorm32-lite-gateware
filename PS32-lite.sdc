# PLL Constraints
#################
create_clock -period 5.4945 AMIPLL_CLKOUT0
create_clock -period 71.4286 MC_CLK

set_false_path -from AMIPLL_CLKOUT0 -to MC_CLK
set_false_path -from MC_CLK -to AMIPLL_CLKOUT0

# Asynchronous input signals (sampled/synchronized in fabric)
set_false_path -from [get_ports {KBRESET}]
set_false_path -from [get_ports {MC_RESET_n_IN}]
set_false_path -from [get_ports {MC_HALT_n_IN}]
set_false_path -from [get_ports {MC_IPL_n[*]}]
set_false_path -from [get_ports {MC_BERR_n}]
set_false_path -from [get_ports {MC_BG_n}]
set_false_path -from [get_ports {MC_AS_n_IN}]
set_false_path -from [get_ports {SPARE_IN[*]}]

# Static / Asynchronous output signals
set_false_path -to [get_ports {SPARE_OUT[*]}]
set_false_path -to [get_ports {SPARE_OE[*]}]
set_false_path -to [get_ports {PI_KBRESET}]

# Quasi-static configuration register bits from Raspberry Pi
set_false_path -from [get_cells {*pi_control*}]
