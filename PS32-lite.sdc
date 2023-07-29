# PLL Constraints
#################
create_clock -period 5.4945 AMIPLL_CLKOUT0
create_clock -period 71.4286 MC_CLK

set_false_path -from AMIPLL_CLKOUT0 -to MC_CLK
set_false_path -from MC_CLK -to AMIPLL_CLKOUT0
