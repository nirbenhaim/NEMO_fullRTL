`timescale 1ns/1ps
// -------------------------------------------------------------------------
// addr_tree_Nir_flow
//
// One lane of the "Adder Trees" block from the Inference Flow spec. Sums
// N=16 IN_WIDTH=7-bit ADC samples (SA_OUT0..SA_OUT15) that arrive from 16
// independent sub-arrays (SAs), using a balanced binary adder tree
// pipelined with ONE REGISTER STAGE PER LEVEL:
//   level 1 : 16 -> 8 partial sums,  8-bit  (7+7)
//   level 2 :  8 -> 4 partial sums,  9-bit  (8+8)
//   level 3 :  4 -> 2 partial sums, 10-bit  (9+9)
//   level 4 :  2 -> 1 final sum,    11-bit  (10+10)  = 'sum'
// A matching 1-bit valid is registered alongside the data at every level,
// so out_valid is high exactly 4 cycles after start, one cycle per level
// -- matching tree_level1_valid .. tree_level4_valid / tree_result_valid
// in the spec waveform.
//
// Inputs are 16 SEPARATE named ports (SA_OUT0..SA_OUT15), one per sub-array,
// rather than a bus/array -- matches piso2.sv's out0/out1 convention so
// each SA's pin is individually named and trackable, and avoids the
// escaped double-indexed netlist port names that broke DC/Innovus SDC
// matching earlier in this project (array-of-vector ports flatten to
// escaped name[i][j] in the gate netlist; get_ports can't match that
// pattern).
//
// Start condition (2026-11-10, agreed with Dima): everything works on
// clock cycles, not on ready signals. The SAs' own in_valid outputs are NOT
// used. A single 'start' input comes from an external clock-cycle delay
// (outside this block), sized from the ADC latency/skew so the 16 ADC
// outputs are settled. When start = 1 at the capture edge, SA_OUT0..15 are
// valid by contract and are taken in by level 1. start must be high for
// exactly 1 cycle per sample.
//
// Capture edge: level 1 (the stage that takes in SA_OUT0..15 and start)
// samples on posedge clk. A commented negedge line is right below it: to
// capture on the falling edge, swap the comment between the two lines.
// Levels 2-4 always stay on posedge. With negedge capture, out_valid comes
// 3 cycles after start instead of 4 (level 1 -> level 2 gets half a cycle).
//
// Antenna protection: an ANTENNA_18_SVT_WTN diode cell is instantiated on
// every input and output signal, mirroring piso2.sv's approach. This
// replaces automatic NanoRoute diode insertion in the P&R script.
//
// Latency    : 4 cycles, start to out_valid (posedge capture).
// Throughput : 1 new result per clock (fully pipelined, back-to-back OK).
// Reset is asynchronous, active-low.
// -------------------------------------------------------------------------

module addr_tree_Nir_flow #(
    parameter int N         = 16,
    parameter int IN_WIDTH  = 7,
    parameter int SUM_WIDTH = IN_WIDTH + $clog2(N)   // 11
)(
    input  logic               clk,
    input  logic               rst_n,

    // from the external clock-cycle delay: SA_OUT0..15 are valid now
    input  logic               start,

    input  logic [IN_WIDTH-1:0] SA_OUT0,
    input  logic [IN_WIDTH-1:0] SA_OUT1,
    input  logic [IN_WIDTH-1:0] SA_OUT2,
    input  logic [IN_WIDTH-1:0] SA_OUT3,
    input  logic [IN_WIDTH-1:0] SA_OUT4,
    input  logic [IN_WIDTH-1:0] SA_OUT5,
    input  logic [IN_WIDTH-1:0] SA_OUT6,
    input  logic [IN_WIDTH-1:0] SA_OUT7,
    input  logic [IN_WIDTH-1:0] SA_OUT8,
    input  logic [IN_WIDTH-1:0] SA_OUT9,
    input  logic [IN_WIDTH-1:0] SA_OUT10,
    input  logic [IN_WIDTH-1:0] SA_OUT11,
    input  logic [IN_WIDTH-1:0] SA_OUT12,
    input  logic [IN_WIDTH-1:0] SA_OUT13,
    input  logic [IN_WIDTH-1:0] SA_OUT14,
    input  logic [IN_WIDTH-1:0] SA_OUT15,

    output logic                       out_valid,     // = tree_result_valid
    output logic [SUM_WIDTH-1:0]       sum            // = treeN_sum
);

    // -------------------------------------------------------------------------
    // Antenna protection (mirrors piso2.sv)
    // -------------------------------------------------------------------------
    genvar gi;
    generate
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA0_ANT
            ANTENNA_18_SVT_WTN ant_sa0 (.I(SA_OUT0[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA1_ANT
            ANTENNA_18_SVT_WTN ant_sa1 (.I(SA_OUT1[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA2_ANT
            ANTENNA_18_SVT_WTN ant_sa2 (.I(SA_OUT2[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA3_ANT
            ANTENNA_18_SVT_WTN ant_sa3 (.I(SA_OUT3[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA4_ANT
            ANTENNA_18_SVT_WTN ant_sa4 (.I(SA_OUT4[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA5_ANT
            ANTENNA_18_SVT_WTN ant_sa5 (.I(SA_OUT5[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA6_ANT
            ANTENNA_18_SVT_WTN ant_sa6 (.I(SA_OUT6[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA7_ANT
            ANTENNA_18_SVT_WTN ant_sa7 (.I(SA_OUT7[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA8_ANT
            ANTENNA_18_SVT_WTN ant_sa8 (.I(SA_OUT8[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA9_ANT
            ANTENNA_18_SVT_WTN ant_sa9 (.I(SA_OUT9[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA10_ANT
            ANTENNA_18_SVT_WTN ant_sa10 (.I(SA_OUT10[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA11_ANT
            ANTENNA_18_SVT_WTN ant_sa11 (.I(SA_OUT11[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA12_ANT
            ANTENNA_18_SVT_WTN ant_sa12 (.I(SA_OUT12[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA13_ANT
            ANTENNA_18_SVT_WTN ant_sa13 (.I(SA_OUT13[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA14_ANT
            ANTENNA_18_SVT_WTN ant_sa14 (.I(SA_OUT14[gi]));
        end
        for (gi = 0; gi < IN_WIDTH; gi = gi + 1) begin : GEN_SA15_ANT
            ANTENNA_18_SVT_WTN ant_sa15 (.I(SA_OUT15[gi]));
        end

        ANTENNA_18_SVT_WTN ant_clk        (.I(clk));
        ANTENNA_18_SVT_WTN ant_rst        (.I(rst_n));

        ANTENNA_18_SVT_WTN ant_start      (.I(start));

        for (gi = 0; gi < SUM_WIDTH; gi = gi + 1) begin : GEN_SUM_ANT
            ANTENNA_18_SVT_WTN ant_sum (.I(sum[gi]));
        end
        ANTENNA_18_SVT_WTN ant_out_valid (.I(out_valid));
    endgenerate

    // ---- level 1 : 16 -> 8, 7+7 -> 8 bit  (input capture stage) ----
    logic [IN_WIDTH:0]   level1_sum [0:7];
    logic                level1_valid;

    always_ff @(posedge clk or negedge rst_n) begin      // capture on rising edge
//  always_ff @(negedge clk or negedge rst_n) begin      // capture on falling edge: swap the comment with the line above
        if (!rst_n) begin
            for (int k = 0; k < 8; k++) level1_sum[k] <= '0;
            level1_valid <= 1'b0;
        end
        else begin
            level1_sum[0] <= SA_OUT0  + SA_OUT1;
            level1_sum[1] <= SA_OUT2  + SA_OUT3;
            level1_sum[2] <= SA_OUT4  + SA_OUT5;
            level1_sum[3] <= SA_OUT6  + SA_OUT7;
            level1_sum[4] <= SA_OUT8  + SA_OUT9;
            level1_sum[5] <= SA_OUT10 + SA_OUT11;
            level1_sum[6] <= SA_OUT12 + SA_OUT13;
            level1_sum[7] <= SA_OUT14 + SA_OUT15;
            level1_valid  <= start;
        end
    end

    // ---- level 2 : 8 -> 4, 8+8 -> 9 bit ----
    logic [IN_WIDTH+1:0] level2_sum [0:3];
    logic                level2_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int k = 0; k < 4; k++) level2_sum[k] <= '0;
            level2_valid <= 1'b0;
        end
        else begin
            for (int k = 0; k < 4; k++)
                level2_sum[k] <= level1_sum[2*k] + level1_sum[2*k+1];
            level2_valid <= level1_valid;
        end
    end

    // ---- level 3 : 4 -> 2, 9+9 -> 10 bit ----
    logic [IN_WIDTH+2:0] level3_sum [0:1];
    logic                level3_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int k = 0; k < 2; k++) level3_sum[k] <= '0;
            level3_valid <= 1'b0;
        end
        else begin
            for (int k = 0; k < 2; k++)
                level3_sum[k] <= level2_sum[2*k] + level2_sum[2*k+1];
            level3_valid <= level2_valid;
        end
    end

    // ---- level 4 : 2 -> 1, 10+10 -> 11 bit  = final sum ----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum       <= '0;
            out_valid <= 1'b0;
        end
        else begin
            sum       <= level3_sum[0] + level3_sum[1];
            out_valid <= level3_valid;
        end
    end

endmodule
