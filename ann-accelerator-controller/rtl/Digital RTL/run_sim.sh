#!/bin/bash
# Compile + run the start/act_sat RTL (sv_files_start_sat). Run from this folder: bash run_sim.sh
STD=/tools/kits/tower/libs/TS18PM/tsl18fs191svt_wtn_Rev_2022.03/lib/verilog
OPT="-full64 -sverilog -kdb -debug_access+all -timescale=1ns/1ps"
# 1) adder tree alone
vcs $OPT antenna_sim_stub.sv addr_tree_Nir_flow.sv tb_addr_tree_Nir_flow.sv -top tb_addr_tree_Nir_flow -o simv_tree -l vcs_tree_compile.log
./simv_tree -l sim_tree.log
# 2) tree -> DSA -> act_sat (std-cell models for the DSA's BUF_X2)
vcs $OPT antenna_sim_stub.sv addr_tree_Nir_flow.sv dsa.sv act_sat.sv tb_tree_dsa_act.sv -v $STD/tsl18fs191svt_wtn.v -v $STD/tsl18fs191svt_wtn_udp.v -top tb_tree_dsa_act -o simv_chain -l vcs_chain_compile.log
./simv_chain -l sim_chain.log
# 3) wrapper elaboration check (no stimulus: the SA stub has no outputs)
vcs $OPT antenna_sim_stub.sv SA_macro.sv addr_tree_Nir_flow.sv dsa.sv act_sat.sv sa_to_adder_tree_wrapper.sv -v $STD/tsl18fs191svt_wtn.v -v $STD/tsl18fs191svt_wtn_udp.v -top sa_to_adder_tree_wrapper -o simv_wrapper -l vcs_wrapper_compile.log
