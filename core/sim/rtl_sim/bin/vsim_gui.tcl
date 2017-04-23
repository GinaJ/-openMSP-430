 
vcd dumpports sim:/tb_openMSP430/dut/* -file dumpports_rtl.openMSP430.vcd -unique

do ../bin/vsim_wave.do

# run 1 ms
run -all

vcd dumpportsflush
#quit -f
