`timescale 1ns/1ps

// Full digital chain: adder tree -> DSA -> act_sat.
// 'start' is pulsed once per bit-plane (8 per inference, MSB plane first),
// every 4 cycles, standing in for the external clock-cycle delay.
// After the last plane, act_start is pulsed once.
//   inference 1: planes b7..b3 = 0, b2 = 1, b1 = 2, b0 = 3 on all 16 SAs
//                sums 0,0,0,0,0,16,32,48 -> acc = 16*4 + 32*2 + 48 = 176 -> act = 176
//   inference 2: every plane = 0,1,2,...,15 -> sum 120 per plane
//                acc = 120 * 255 = 30600 -> act = 255 (saturated)
// The DSA is cleared between inferences by rst_n (as in the wrapper today).

module tb_tree_dsa_act;

    logic clk = 0;
    logic rst_n;
    logic start;
    logic act_start;

    logic [6:0] SA_OUT0, SA_OUT1, SA_OUT2,  SA_OUT3,  SA_OUT4,  SA_OUT5,  SA_OUT6,  SA_OUT7;
    logic [6:0] SA_OUT8, SA_OUT9, SA_OUT10, SA_OUT11, SA_OUT12, SA_OUT13, SA_OUT14, SA_OUT15;

    logic        out_valid;
    logic [10:0] sum;
    logic [18:0] acc_out;
    logic [10:0] vec_out;
    logic [7:0]  act_out;
    logic        act_valid;

    addr_tree_Nir_flow u_tree (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (start),
        .SA_OUT0   (SA_OUT0),  .SA_OUT1  (SA_OUT1),  .SA_OUT2  (SA_OUT2),  .SA_OUT3  (SA_OUT3),
        .SA_OUT4   (SA_OUT4),  .SA_OUT5  (SA_OUT5),  .SA_OUT6  (SA_OUT6),  .SA_OUT7  (SA_OUT7),
        .SA_OUT8   (SA_OUT8),  .SA_OUT9  (SA_OUT9),  .SA_OUT10 (SA_OUT10), .SA_OUT11 (SA_OUT11),
        .SA_OUT12  (SA_OUT12), .SA_OUT13 (SA_OUT13), .SA_OUT14 (SA_OUT14), .SA_OUT15 (SA_OUT15),
        .out_valid (out_valid),
        .sum       (sum)
    );

    dsa u_dsa (
        .clk               (clk),
        .rst_n             (rst_n),
        .tree_result_valid (out_valid),
        .vec_in            (sum),
        .acc_out           (acc_out),
        .vec_out           (vec_out)
    );

    act_sat u_act (
        .clk       (clk),
        .rst_n     (rst_n),
        .act_start (act_start),
        .acc_in    (acc_out),
        .act_out   (act_out),
        .act_valid (act_valid)
    );

    always #50 clk = ~clk;   // 100 ns clock: 50 ns high / 50 ns low

    initial begin
        $dumpfile("wave_chain.vcd");
        $dumpvars(0, tb_tree_dsa_act);

        // ---------------- reset ----------------
        rst_n = 0;
        start = 0;
        act_start = 0;
        SA_OUT0 = 0; SA_OUT1 = 0; SA_OUT2  = 0; SA_OUT3  = 0; SA_OUT4  = 0; SA_OUT5  = 0; SA_OUT6  = 0; SA_OUT7  = 0;
        SA_OUT8 = 0; SA_OUT9 = 0; SA_OUT10 = 0; SA_OUT11 = 0; SA_OUT12 = 0; SA_OUT13 = 0; SA_OUT14 = 0; SA_OUT15 = 0;
        repeat (3) @(posedge clk);
        @(posedge clk) #1;
        rst_n = 1;
        repeat (2) @(posedge clk);

        // ================= inference 1 : act = 176 =================
        // planes b7..b3 : all SAs = 0
        @(posedge clk) #1; start = 1;
        @(posedge clk) #1; start = 0;
        repeat (2) @(posedge clk);
        @(posedge clk) #1; start = 1;
        @(posedge clk) #1; start = 0;
        repeat (2) @(posedge clk);
        @(posedge clk) #1; start = 1;
        @(posedge clk) #1; start = 0;
        repeat (2) @(posedge clk);
        @(posedge clk) #1; start = 1;
        @(posedge clk) #1; start = 0;
        repeat (2) @(posedge clk);
        @(posedge clk) #1; start = 1;
        @(posedge clk) #1; start = 0;
        repeat (2) @(posedge clk);
        // plane b2 : all SAs = 1 (sum 16)
        @(posedge clk) #1;
        SA_OUT0 = 1; SA_OUT1 = 1; SA_OUT2  = 1; SA_OUT3  = 1; SA_OUT4  = 1; SA_OUT5  = 1; SA_OUT6  = 1; SA_OUT7  = 1;
        SA_OUT8 = 1; SA_OUT9 = 1; SA_OUT10 = 1; SA_OUT11 = 1; SA_OUT12 = 1; SA_OUT13 = 1; SA_OUT14 = 1; SA_OUT15 = 1;
        start = 1;
        @(posedge clk) #1; start = 0;
        repeat (2) @(posedge clk);
        // plane b1 : all SAs = 2 (sum 32)
        @(posedge clk) #1;
        SA_OUT0 = 2; SA_OUT1 = 2; SA_OUT2  = 2; SA_OUT3  = 2; SA_OUT4  = 2; SA_OUT5  = 2; SA_OUT6  = 2; SA_OUT7  = 2;
        SA_OUT8 = 2; SA_OUT9 = 2; SA_OUT10 = 2; SA_OUT11 = 2; SA_OUT12 = 2; SA_OUT13 = 2; SA_OUT14 = 2; SA_OUT15 = 2;
        start = 1;
        @(posedge clk) #1; start = 0;
        repeat (2) @(posedge clk);
        // plane b0 : all SAs = 3 (sum 48)
        @(posedge clk) #1;
        SA_OUT0 = 3; SA_OUT1 = 3; SA_OUT2  = 3; SA_OUT3  = 3; SA_OUT4  = 3; SA_OUT5  = 3; SA_OUT6  = 3; SA_OUT7  = 3;
        SA_OUT8 = 3; SA_OUT9 = 3; SA_OUT10 = 3; SA_OUT11 = 3; SA_OUT12 = 3; SA_OUT13 = 3; SA_OUT14 = 3; SA_OUT15 = 3;
        start = 1;
        @(posedge clk) #1; start = 0;
        // last result: out_valid 4 cycles after start, acc 1 cycle later
        repeat (3) @(posedge clk);
        @(posedge clk) #1; act_start = 1;
        @(posedge clk) #1; act_start = 0;
        repeat (3) @(posedge clk);

        // clear the DSA (controller's job later; rst_n today)
        @(posedge clk) #1; rst_n = 0;
        @(posedge clk) #1; rst_n = 1;
        repeat (2) @(posedge clk);

        // ================= inference 2 : act = 255 (saturated) =================
        @(posedge clk) #1;
        SA_OUT0 = 0; SA_OUT1 = 1; SA_OUT2  = 2;  SA_OUT3  = 3;  SA_OUT4  = 4;  SA_OUT5  = 5;  SA_OUT6  = 6;  SA_OUT7  = 7;
        SA_OUT8 = 8; SA_OUT9 = 9; SA_OUT10 = 10; SA_OUT11 = 11; SA_OUT12 = 12; SA_OUT13 = 13; SA_OUT14 = 14; SA_OUT15 = 15;
        // planes b7..b0 : same samples every plane (sum 120)
        start = 1;
        @(posedge clk) #1; start = 0;
        repeat (2) @(posedge clk);
        @(posedge clk) #1; start = 1;
        @(posedge clk) #1; start = 0;
        repeat (2) @(posedge clk);
        @(posedge clk) #1; start = 1;
        @(posedge clk) #1; start = 0;
        repeat (2) @(posedge clk);
        @(posedge clk) #1; start = 1;
        @(posedge clk) #1; start = 0;
        repeat (2) @(posedge clk);
        @(posedge clk) #1; start = 1;
        @(posedge clk) #1; start = 0;
        repeat (2) @(posedge clk);
        @(posedge clk) #1; start = 1;
        @(posedge clk) #1; start = 0;
        repeat (2) @(posedge clk);
        @(posedge clk) #1; start = 1;
        @(posedge clk) #1; start = 0;
        repeat (2) @(posedge clk);
        @(posedge clk) #1; start = 1;
        @(posedge clk) #1; start = 0;
        repeat (3) @(posedge clk);
        @(posedge clk) #1; act_start = 1;
        @(posedge clk) #1; act_start = 0;
        repeat (3) @(posedge clk);

        $finish;
    end

endmodule
