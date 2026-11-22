`timescale 1ns/1ps
// -------------------------------------------------------------------------
// sa_to_adder_tree_wrapper
//
// Instantiates 16 copies of the black-box SA_macro IP and wires ONE ADC
// channel from each of the 16 SAs into a single addr_tree_Nir_flow instance
// -- i.e. this is one "AT<k>" from the 16x64 SA architecture diagram
// (Drawing_16x64_SA - control for inputs.vsdx), where AT<k> takes ADC_<k>
// from all 16 SAs. Which k this wrapper represents is a compile-time
// parameter (ADC_INDEX), so the same wrapper source builds all 4 real AT
// instances (AT0..AT3) just by changing the parameter -- confirmed with
// Dima that the full 16-SA wrapper (not a single instance) is what's
// wanted, "to see the big picture of a PE."
//
// Per Dima's instructions: ONLY the ADC-output-to-Adder-Tree connection is
// in scope right now. Every other SA pin (power/bias, reset, s0-s2, clk,
// pulse, bl_pad) is treated as a SHARED/broadcast signal, fanned out
// identically to all 16 SA instances and brought out as a single wrapper
// port each -- these will be driven by the controller in a later phase.
// This is an assumption (flagged for Dima to confirm), not a spec'd fact:
// it is entirely possible some of these (e.g. bl_pad) are meant to be
// distinct per-SA rather than shared.
//
// This module is purely structural (instances + wiring, no behavior), so
// it maps directly and mechanically to a CDL netlist -- see
// sa_to_adder_tree_wrapper.cdl.
//
// 2026-11-10 (agreed with Dima): the tree starts on a single 'start' input
// from an external clock-cycle delay, not on the SAs' in_valid outputs, so
// SA in_valid is left unconnected here. act_sat (saturation to 8 bits) is
// added after the DSA, started by act_start from the controller.
// -------------------------------------------------------------------------

module sa_to_adder_tree_wrapper #(
    parameter int ADC_INDEX = 0   // which ADC channel (0-3) this AT instance taps
)(
    // ---- shared analog bias / supply pins (broadcast to all 16 SAs) ----
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

    // ---- shared digital control pins (broadcast to all 16 SAs) ----
    input  logic reset,
    input  logic s0,
    input  logic s1,
    input  logic s2,
    input  logic clk,
    input  logic pulse,

    // ---- Adder Tree outputs, passed straight through ----
    output logic        out_valid,
    output logic [10:0] sum,

    // ---- start of a tree calculation (from the external clock-cycle delay) ----
    input  logic        start,

    // ---- DSA (Digital Shift Accumulator) output ----
    output logic [18:0] acc_out,

    // ---- activation (saturation) ----
    input  logic        act_start,
    output logic [7:0]  act_out,
    output logic        act_valid
);

    // reset fans out broadcast to all 16 SA_macro instances plus the
    // tree's rst_n inverter (18 loads) and has no antenna protection
    // anywhere else in the design -- verifyProcessAntenna flags it.
    ANTENNA_18_SVT_WTN ant_reset (.I(reset));

    // Per-SA taps of the selected ADC channel
    logic [6:0] tap0,  tap1,  tap2,  tap3,  tap4,  tap5,  tap6,  tap7;
    logic [6:0] tap8,  tap9,  tap10, tap11, tap12, tap13, tap14, tap15;

    genvar gi;
    generate
        for (gi = 0; gi < 16; gi = gi + 1) begin : GEN_SA
            // Flat scalar ADC bits (matches SA_macro's flat scalar ports --
            // see SA_macro.sv header comment for why these aren't vectors)
            logic sa_adc0_6, sa_adc0_5, sa_adc0_4, sa_adc0_3, sa_adc0_2, sa_adc0_1, sa_adc0_0;
            logic sa_adc1_6, sa_adc1_5, sa_adc1_4, sa_adc1_3, sa_adc1_2, sa_adc1_1, sa_adc1_0;
            logic sa_adc2_6, sa_adc2_5, sa_adc2_4, sa_adc2_3, sa_adc2_2, sa_adc2_1, sa_adc2_0;
            logic sa_adc3_6, sa_adc3_5, sa_adc3_4, sa_adc3_3, sa_adc3_2, sa_adc3_1, sa_adc3_0;
            // Internal 7-bit vector (not a port, so no library-matching
            // concern) holding whichever ADC channel this AT taps.
            logic [6:0] GEN_SA_tap;

            SA_macro u_sa (
                .vssx     (vssx),
                .vccx     (vccx),
                .vccy     (vccy),
                .vcch_wl  (vcch_wl),
                .vcch_bl  (vcch_bl),
                .delta_vb (delta_vb),
                .lvr_vb   (lvr_vb),
                .lvr_vcm  (lvr_vcm),
                .bl_pad   (bl_pad),
                .tdc_vb   (tdc_vb),
                .v_erase  (v_erase),
                .v_read   (v_read),
                .v_write  (v_write),
                .vtc_vb   (vtc_vb),
                .vth      (vth),
                .bl_vdc   (bl_vdc),
                .reset    (reset),
                .s0       (s0),
                .s1       (s1),
                .s2       (s2),
                .clk      (clk),
                .pulse    (pulse),
                .in_valid (),             // not used: the tree starts on 'start'
                .ADC_0_6  (sa_adc0_6), .ADC_0_5 (sa_adc0_5), .ADC_0_4 (sa_adc0_4), .ADC_0_3 (sa_adc0_3),
                .ADC_0_2  (sa_adc0_2), .ADC_0_1 (sa_adc0_1), .ADC_0_0 (sa_adc0_0),
                .ADC_1_6  (sa_adc1_6), .ADC_1_5 (sa_adc1_5), .ADC_1_4 (sa_adc1_4), .ADC_1_3 (sa_adc1_3),
                .ADC_1_2  (sa_adc1_2), .ADC_1_1 (sa_adc1_1), .ADC_1_0 (sa_adc1_0),
                .ADC_2_6  (sa_adc2_6), .ADC_2_5 (sa_adc2_5), .ADC_2_4 (sa_adc2_4), .ADC_2_3 (sa_adc2_3),
                .ADC_2_2  (sa_adc2_2), .ADC_2_1 (sa_adc2_1), .ADC_2_0 (sa_adc2_0),
                .ADC_3_6  (sa_adc3_6), .ADC_3_5 (sa_adc3_5), .ADC_3_4 (sa_adc3_4), .ADC_3_3 (sa_adc3_3),
                .ADC_3_2  (sa_adc3_2), .ADC_3_1 (sa_adc3_1), .ADC_3_0 (sa_adc3_0)
            );

            // compile-time selection of which ADC channel this AT taps --
            // only one branch survives elaboration, so the CDL netlist
            // shows a plain direct connection, not a mux
            case (ADC_INDEX)
                0: assign GEN_SA_tap = {sa_adc0_6, sa_adc0_5, sa_adc0_4, sa_adc0_3, sa_adc0_2, sa_adc0_1, sa_adc0_0};
                1: assign GEN_SA_tap = {sa_adc1_6, sa_adc1_5, sa_adc1_4, sa_adc1_3, sa_adc1_2, sa_adc1_1, sa_adc1_0};
                2: assign GEN_SA_tap = {sa_adc2_6, sa_adc2_5, sa_adc2_4, sa_adc2_3, sa_adc2_2, sa_adc2_1, sa_adc2_0};
                3: assign GEN_SA_tap = {sa_adc3_6, sa_adc3_5, sa_adc3_4, sa_adc3_3, sa_adc3_2, sa_adc3_1, sa_adc3_0};
            endcase

            // Antenna protection at the SA-macro side of the long SA-to-tree
            // route. addr_tree_Nir_flow already has ANTENNA_18_SVT_WTN on
            // its SA_OUT* ports, but P&R shows that single
            // far-end (tree-side) diode isn't enough for these particular
            // net lengths -- verifyProcessAntenna flags GEN_SA[i].GEN_SA_tap[1:6]
            // on every instance. Automatic
            // diode insertion (route_antenna_diode_insertion) can't fix it
            // either: it tries to place near the tree-side pin, which sits
            // inside the packed u_addr_tree fence with no room left. Adding
            // a second diode here, at the source end in the open macro
            // channel, gives near-end protection that's currently missing
            // entirely.
            genvar gt;
            for (gt = 0; gt < 7; gt = gt + 1) begin : GEN_TAP_ANT
                ANTENNA_18_SVT_WTN ant_tap (.I(GEN_SA_tap[gt]));
            end
        end
    endgenerate

    // NOTE: see below -- tap arrays are wired from GEN_SA in the
    // block above via the generate-local names; flattened here for the
    // addr_tree_Nir_flow port connections (16 separate ports, not arrays,
    // per this project's established convention).
    assign tap0  = GEN_SA[0].GEN_SA_tap;
    assign tap1  = GEN_SA[1].GEN_SA_tap;
    assign tap2  = GEN_SA[2].GEN_SA_tap;
    assign tap3  = GEN_SA[3].GEN_SA_tap;
    assign tap4  = GEN_SA[4].GEN_SA_tap;
    assign tap5  = GEN_SA[5].GEN_SA_tap;
    assign tap6  = GEN_SA[6].GEN_SA_tap;
    assign tap7  = GEN_SA[7].GEN_SA_tap;
    assign tap8  = GEN_SA[8].GEN_SA_tap;
    assign tap9  = GEN_SA[9].GEN_SA_tap;
    assign tap10 = GEN_SA[10].GEN_SA_tap;
    assign tap11 = GEN_SA[11].GEN_SA_tap;
    assign tap12 = GEN_SA[12].GEN_SA_tap;
    assign tap13 = GEN_SA[13].GEN_SA_tap;
    assign tap14 = GEN_SA[14].GEN_SA_tap;
    assign tap15 = GEN_SA[15].GEN_SA_tap;

    addr_tree_Nir_flow u_addr_tree (
        .clk        (clk),
        .rst_n      (~reset),
        .start      (start),
        .SA_OUT0    (tap0),  .SA_OUT1  (tap1),  .SA_OUT2  (tap2),  .SA_OUT3  (tap3),
        .SA_OUT4    (tap4),  .SA_OUT5  (tap5),  .SA_OUT6  (tap6),  .SA_OUT7  (tap7),
        .SA_OUT8    (tap8),  .SA_OUT9  (tap9),  .SA_OUT10 (tap10), .SA_OUT11 (tap11),
        .SA_OUT12   (tap12), .SA_OUT13 (tap13), .SA_OUT14 (tap14), .SA_OUT15 (tap15),
        .out_valid  (out_valid),
        .sum        (sum)
    );

    // DSA consumes the tree's output directly -- vec_in/tree_result_valid
    // tap sum/out_valid before they reach the top-level pins above.
    logic [10:0] dsa_vec_out;   // unused pass-through from dsa

    dsa u_dsa (
        .clk               (clk),
        .rst_n             (~reset),
        .tree_result_valid (out_valid),
        .vec_in            (sum),
        .acc_out           (acc_out),
        .vec_out           (dsa_vec_out)
    );

    // Activation: saturate the 19-bit accumulator to 8 bits.
    act_sat u_act_sat (
        .clk       (clk),
        .rst_n     (~reset),
        .act_start (act_start),
        .acc_in    (acc_out),
        .act_out   (act_out),
        .act_valid (act_valid)
    );

    // acc_out runs from inside the DSA fence out to the chip's top-level
    // pins with zero antenna protection so far (unlike sum/tap,
    // which already have at least one diode each) -- verifyProcessAntenna
    // flagged acc_out[1] and acc_out[18] after adding DSA.
    genvar ga;
    generate
        for (ga = 0; ga < 19; ga = ga + 1) begin : GEN_ACC_ANT
            ANTENNA_18_SVT_WTN ant_acc_out (.I(acc_out[ga]));
        end
    endgenerate

    // New top-level ports (2026-11-10): verifyProcessAntenna flagged start,
    // act_start (11 pins) and act_out[*] -- long M3 runs from the die-edge
    // pins to the fences. Same fix as acc_out: a diode on each port net at
    // the wrapper level. start already has one inside the tree (far end);
    // this is the second one.
    ANTENNA_18_SVT_WTN ant_start     (.I(start));
    ANTENNA_18_SVT_WTN ant_act_start (.I(act_start));
    ANTENNA_18_SVT_WTN ant_act_valid (.I(act_valid));
    genvar gb;
    generate
        for (gb = 0; gb < 8; gb = gb + 1) begin : GEN_ACT_ANT
            ANTENNA_18_SVT_WTN ant_act_out (.I(act_out[gb]));
        end
    endgenerate

endmodule
