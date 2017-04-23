
#=============================================================================#
#                              Configuration                                  #
#=============================================================================#

set DESIGN_NAME      "openMSP430"

set NETLIST_FILES    [list "../../../gate/verilog/$DESIGN_NAME.gate.v"]

set LIBRARY_FILES    [list "../../../gate/verilog/CORE65GPSVT_tmax.v"]



#=============================================================================#
#                           Read Design & Technology files                    #
#=============================================================================#

# Rules to be ignored
set_rules B7  ignore    ;# undriven module output pin
set_rules B8  ignore    ;# unconnected module input pin
set_rules B9  ignore    ;# undriven module internal net
set_rules B10 ignore    ;# unconnected module internal net
set_rules N20 ignore    ;# underspecified UDP
set_rules N21 ignore    ;# unsupported UDP entry
set_rules N23 ignore    ;# inconsistent UDP


# Reset TMAX
reset_all
build -force
read_netlist -delete

set_netlist -sequential_modeling

# Read gate level netlist
foreach design_file $NETLIST_FILES {
    read_netlist $design_file
}

# Read library files
foreach lib_file $LIBRARY_FILES {
    read_netlist $lib_file
}

# Remove unused net connections
remove_net_connection -all

# Build the model
run_build_model $DESIGN_NAME


#=============================================================================#
#                                    Run DRC                                  #
#=============================================================================#

# Allow ATPG to use nonscan cell values loaded by the last shift.
#set_drc -load_nonscan_cells

# Report settings
report_settings drc

# Run DRC
#run_drc $SPF_FILE
run_drc

#=============================================================================#
#                               Fault Simulation                              #
#=============================================================================#

set_patterns -external dumpports_rtl.openMSP430.vcd -sensitive -strobe_period { 25 ns } -strobe_offset { 95 ns }
set_simulation -num_processes 0
run_simulation -sequential
set_faults -model stuck
add_faults -all
run_fault_sim -sequential
 
# # Create reports
report_summaries
write_faults output_fault_list.txt -all -replace
report_faults -level {100 1} -verbose > report_faults_verbose.txt
report_faults -level {5 100} > report_faults.txt

# 
quit
