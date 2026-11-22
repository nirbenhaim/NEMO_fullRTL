`timescale 1ns/1ps
// -------------------------------------------------------------------------
// SA_macro
//
// Black-box stub for the analog SA (sub-array) IP, per the interface handoff
// from Dima (Interface_SA_inital.xlsx) and the 16x64 SA architecture diagram
// (Drawing_16x64_SA - control for inputs.vsdx). No internal behavior --
// this exists purely so sa_to_adder_tree_wrapper.sv has something concrete
// to instantiate 16 copies of and wire up structurally.
//
// Pin list transcribed directly from the Excel interface file:
//   - analog bias/supply pins (vssx, vccx, vccy, vcch_wl, vcch_bl, delta_vb,
//     lvr_vb, lvr_vcm, tdc_vb, v_erase, v_read, v_write, vtc_vb, vth, bl_vdc)
//   - bl_pad: bidirectional analog pad
//   - digital control pins (reset, s0, s1, s2, clk, pulse)
//   - outputs: in_valid (1 bit, shared by all 4 ADC channels of this SA),
//     ADC_0..ADC_3 (7 bits each)
//
// All of these are single named ports (not arrays/vectors), consistent
// with the port-naming convention used across this project (see
// addr_tree_Nir_flow.sv) to avoid DC/Innovus's double-indexed
// escaped-port-name issue -- AND, specifically for the ADC outputs below,
// to avoid a second issue: a Liberty pin group named with brackets does
// NOT automatically form a bus DC can link against a Verilog vector port
// (that needs an explicit Liberty bus() group). Bit-blasting ADC_0..3 into
// 28 flat scalar ports, matching SA_macro_default.lib's pins 1:1, sidesteps
// that entirely and stays consistent with the flat-signal style used
// everywhere else in this project.
//
// NOTE: this is a structural stub only. Real geometry (LEF) and timing
// (LIB) require characterization data from the analog team -- pending.
// -------------------------------------------------------------------------

module SA_macro (
    // ---- analog bias / supply pins ----
    input  logic vssx,
    input  logic vccx,
    input  logic vccy,
    input  logic vcch_wl,
    input  logic vcch_bl,
    input  logic delta_vb,
    input  logic lvr_vb,
    input  logic lvr_vcm,
    inout  wire  bl_pad,
    input  logic tdc_vb,
    input  logic v_erase,
    input  logic v_read,
    input  logic v_write,
    input  logic vtc_vb,
    input  logic vth,
    input  logic bl_vdc,

    // ---- digital control pins ----
    input  logic reset,
    input  logic s0,
    input  logic s1,
    input  logic s2,
    input  logic clk,
    input  logic pulse,

    // ---- outputs ----
    output logic in_valid,   // shared adc_result_valid for all 4 ADCs below

    output logic ADC_0_6, ADC_0_5, ADC_0_4, ADC_0_3, ADC_0_2, ADC_0_1, ADC_0_0,
    output logic ADC_1_6, ADC_1_5, ADC_1_4, ADC_1_3, ADC_1_2, ADC_1_1, ADC_1_0,
    output logic ADC_2_6, ADC_2_5, ADC_2_4, ADC_2_3, ADC_2_2, ADC_2_1, ADC_2_0,
    output logic ADC_3_6, ADC_3_5, ADC_3_4, ADC_3_3, ADC_3_2, ADC_3_1, ADC_3_0
);

endmodule
