//------------------------------------------------------------------------------
// nemo_top_wrapper file list (paths relative to NEMO_RTL/)
//   vlib work
//   vlog -sv -work work -f nemo_top_wrapper.f
// Run vsim from NEMO_RTL/ so the default WEIGHT_PULSE_LUT_FILE path resolves.
//------------------------------------------------------------------------------

// `include "../common/macros.svh" is relative to the including file
+incdir+ann-accelerator-controller/rtl/Controller
+incdir+ann-accelerator-controller/rtl/Input_Buffer

// Shared architecture package (must be first)
ann-accelerator-controller/rtl/common/ann_arch_pkg.sv

// Parallel Interface
ann-accelerator-controller/rtl/Parallel_Interface/parallel_interface_pkg.sv
ann-accelerator-controller/rtl/Parallel_Interface/parallel_interface.sv

// Input Buffer
ann-accelerator-controller/rtl/Input_Buffer/input_buffer_pkg.sv
ann-accelerator-controller/rtl/Input_Buffer/input_buffer.sv

// Controller
ann-accelerator-controller/rtl/Controller/controller_pkg.sv
ann-accelerator-controller/rtl/Controller/controller.sv

// Top-level wrapper
nemo_top_wrapper.sv
