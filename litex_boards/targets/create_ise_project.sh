# xtclsh create_ise_project.sh
# xtclsh needs to be in PATH. This can be done by
# source /opt/Xilinx/14.7/ISE_DS/settings64.sh
file mkdir build2
cd build2
project new pano_logic_g2
# files from pano_logic_g2.prj
xfile add [pwd]/../../../../pythondata-cpu-vexriscv/pythondata_cpu_vexriscv/verilog/VexRiscv.v
xfile add [pwd]/../build/pano_logic_g2/gateware/pano_logic_g2.v
project set family Spartan6
project set device xc6slx100
project set package fgg484
project set speed -2
project set top pano_logic_g2
project close

