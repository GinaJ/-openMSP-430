#!/bin/bash

root_dir=$(cd $(dirname $0); pwd)

#logic simulation
(
cd ${root_dir}/core/sim/rtl_sim/run
./run mytest
)

# fault simulation
(
cd ${root_dir}/core/sim/rtl_sim/run
tmax -shell ../bin/tmax.tcl | tee tmax.log
)

