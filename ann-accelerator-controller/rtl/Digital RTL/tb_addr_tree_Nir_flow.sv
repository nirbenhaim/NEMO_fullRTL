`timescale 1ns/1ps

module tb_addr_tree_Nir_flow;

    localparam int N         = 16;
    localparam int IN_WIDTH  = 7;
    localparam int SUM_WIDTH = IN_WIDTH + $clog2(N);

    logic clk = 0;
    logic rst_n;

    logic start;

    logic [IN_WIDTH-1:0] SA_OUT0, SA_OUT1, SA_OUT2,  SA_OUT3,  SA_OUT4,  SA_OUT5,  SA_OUT6,  SA_OUT7;
    logic [IN_WIDTH-1:0] SA_OUT8, SA_OUT9, SA_OUT10, SA_OUT11, SA_OUT12, SA_OUT13, SA_OUT14, SA_OUT15;

    logic out_valid;
    logic [SUM_WIDTH-1:0] sum;

    addr_tree_Nir_flow #(
        .N(N),
        .IN_WIDTH(IN_WIDTH)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),
        .SA_OUT0    (SA_OUT0),
        .SA_OUT1    (SA_OUT1),
        .SA_OUT2    (SA_OUT2),
        .SA_OUT3    (SA_OUT3),
        .SA_OUT4    (SA_OUT4),
        .SA_OUT5    (SA_OUT5),
        .SA_OUT6    (SA_OUT6),
        .SA_OUT7    (SA_OUT7),
        .SA_OUT8    (SA_OUT8),
        .SA_OUT9    (SA_OUT9),
        .SA_OUT10   (SA_OUT10),
        .SA_OUT11   (SA_OUT11),
        .SA_OUT12   (SA_OUT12),
        .SA_OUT13   (SA_OUT13),
        .SA_OUT14   (SA_OUT14),
        .SA_OUT15   (SA_OUT15),
        .out_valid  (out_valid),
        .sum        (sum)
    );

    always #50 clk = ~clk;   // 100 ns clock: 50 ns high / 50 ns low

    initial begin
        $dumpfile("wave.vcd");
        $dumpvars(0, tb_addr_tree_Nir_flow);

        // reset
        rst_n = 0;
        start = 0;
        SA_OUT0 = '0; SA_OUT1 = '0; SA_OUT2  = '0; SA_OUT3  = '0;
        SA_OUT4 = '0; SA_OUT5 = '0; SA_OUT6  = '0; SA_OUT7  = '0;
        SA_OUT8 = '0; SA_OUT9 = '0; SA_OUT10 = '0; SA_OUT11 = '0;
        SA_OUT12 = '0; SA_OUT13 = '0; SA_OUT14 = '0; SA_OUT15 = '0;
        repeat (3) @(posedge clk);
        #1 rst_n = 1;
        repeat (2) @(posedge clk);

        // ============ run 1 : samples 0,1,2,...,15  -> sum = 120 ============
        @(posedge clk) #1;
        SA_OUT0 = 7'd0;  SA_OUT1 = 7'd1;  SA_OUT2  = 7'd2;  SA_OUT3  = 7'd3;
        SA_OUT4 = 7'd4;  SA_OUT5 = 7'd5;  SA_OUT6  = 7'd6;  SA_OUT7  = 7'd7;
        SA_OUT8 = 7'd8;  SA_OUT9 = 7'd9;  SA_OUT10 = 7'd10; SA_OUT11 = 7'd11;
        SA_OUT12 = 7'd12; SA_OUT13 = 7'd13; SA_OUT14 = 7'd14; SA_OUT15 = 7'd15;
        start = 1;
        @(posedge clk) #1;
        start = 0;
        repeat (6) @(posedge clk);

        // ============ run 2 : samples 100,99,98,...,85  -> sum = 1480 ============
        @(posedge clk) #1;
        SA_OUT0 = 7'd100; SA_OUT1 = 7'd99; SA_OUT2  = 7'd98; SA_OUT3  = 7'd97;
        SA_OUT4 = 7'd96;  SA_OUT5 = 7'd95; SA_OUT6  = 7'd94; SA_OUT7  = 7'd93;
        SA_OUT8 = 7'd92;  SA_OUT9 = 7'd91; SA_OUT10 = 7'd90; SA_OUT11 = 7'd89;
        SA_OUT12 = 7'd88; SA_OUT13 = 7'd87; SA_OUT14 = 7'd86; SA_OUT15 = 7'd85;
        start = 1;
        @(posedge clk) #1;
        start = 0;
        repeat (6) @(posedge clk);

        $finish;
    end

endmodule
