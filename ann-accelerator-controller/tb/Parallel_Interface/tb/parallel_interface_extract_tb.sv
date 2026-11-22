//------------------------------------------------------------------------------
// Parallel Interface - Field Extraction Testbench
//------------------------------------------------------------------------------
// Tests: data = host_data[31:24], address = host_data[23:8] (packed binary
//       {PE[15:14], SA[13:10], ROW[9:6], COL[5:0]}), cmd = host_cmd, and
//       valid = (host_cmd != HIZ) && host_data[7:0] == 0 && fields in range
//       for several host_data / host_cmd patterns (ann_address host layout).
//------------------------------------------------------------------------------

`timescale 1ns/1ps

`include "../../../rtl/common/macros.svh"

import ann_arch_pkg::*;
import parallel_interface_pkg::*;

module parallel_interface_extract_tb;

    logic clk, reset;
    logic [WORD_WIDTH-1:0]        host_data;
    logic [CMD_WIDTH-1:0]         host_cmd;
    logic                         valid;
    logic [ACT_WIDTH-1:0]         data;
    logic [WEIGHT_ADDR_WIDTH-1:0] address;
    logic [CMD_WIDTH-1:0]         cmd;

    parallel_interface dut (
        .clk(clk),
        .reset(reset),
        .host_data(host_data),
        .host_cmd(host_cmd),
        .valid(valid),
        .data(data),
        .address(address),
        .cmd(cmd)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    int pass_count, fail_count;

    // Reference model: address field is host_data[23:8] unchanged
    function automatic logic [WEIGHT_ADDR_WIDTH-1:0] exp_packed_addr(input logic [WORD_WIDTH-1:0] w);
        return w[WORD_ADDR_MSB:WORD_ADDR_LSB];
    endfunction

    // Reference model for valid
    function automatic logic exp_valid(input logic [WORD_WIDTH-1:0] w, input logic [CMD_WIDTH-1:0] c);
        return (c != CMD_HIZ) && (w[WORD_RSVD_WIDTH-1:0] == '0);
    endfunction

    task automatic log_decode(input string tag);
        $display("  %s", tag);
        $display("    host_data hex : 0x%08h", host_data);
        $display("    host_data bin : %08b-%02b-%04b-%04b-%06b-%08b",
                 host_data[31:24], host_data[23:22], host_data[21:18], host_data[17:14],
                 host_data[13:8], host_data[7:0]);
        $display("    host_cmd      : %b", host_cmd);
        $display("    PI outputs    : data=0x%02h addr=0x%04h cmd=%b valid=%0d", data, address, cmd, valid);
        $display("    decoded idx   : PE=%0d SA=%0d row=%0d col=%0d",
                 get_pe_id(address), get_sa_id(address), get_row_id(address), get_col_id(address));
    endtask

    // Generic check: data / address / cmd / valid against the reference model
    task automatic check_word(input string tag, input logic [WORD_WIDTH-1:0] w,
                              input logic [CMD_WIDTH-1:0] c, input logic exp_v);
        host_data = w;
        host_cmd  = c;
        #1;
        log_decode(tag);
        if (data == w[WORD_DATA_MSB:WORD_DATA_LSB] && address == exp_packed_addr(w) &&
            cmd == c && valid === exp_v && exp_v == exp_valid(w, c))
            begin pass_count++; $display("    RESULT: PASS (exp data=%02h addr=%04h valid=%0d)",
                                         w[WORD_DATA_MSB:WORD_DATA_LSB], exp_packed_addr(w), exp_v); end
        else
            begin fail_count++; $display("    RESULT: FAIL (exp data=%02h addr=%04h valid=%0d)",
                                         w[WORD_DATA_MSB:WORD_DATA_LSB], exp_packed_addr(w), exp_v); end
    endtask

    initial begin
        reset = 0;
        host_data = 32'd0;
        host_cmd = CMD_HIZ;
        pass_count = 0;
        fail_count = 0;
        #20;

        $display("----------------------------------------");
        $display("Parallel Interface Field Extraction Tests");
        $display("Summary: data=host_data[31:24], addr=host_data[23:8] {PE,SA,ROW,COL}, rsvd=host_data[7:0], cmd=host_cmd.");
        $display("----------------------------------------");

        // Pattern 1: idle (HIZ -> never valid)
        check_word("Test: idle", 32'd0, CMD_HIZ, 1'b0);

        // Pattern 2: data MSB byte only (address/reserved zero); command shown on cmd port
        check_word("Test: data 0xA5 addr=0 cmd=READ", {8'hA5, 24'd0}, CMD_READ, 1'b1);

        // Pattern 3: packed address built from binary fields
        begin
            logic [WEIGHT_ADDR_WIDTH-1:0] par;
            par = pack_ann_addr(2'd1, 4'd9, 4'd5, 6'd42);
            check_word($sformatf("Test: addr decode PE=1 SA=9 ROW=5 COL=42 (arg=%04h)", par),
                       build_host_ann_word(8'd0, par), CMD_READ, 1'b1);
            if (get_pe_id(address) == 2'd1 && get_sa_id(address) == 4'd9 &&
                get_row_id(address) == 4'd5 && get_col_id(address) == 6'd42 &&
                extract_address(host_data) == par && extract_data(host_data) == 8'd0 &&
                host_word_is_valid(host_data))
                begin pass_count++; $display("    RESULT: PASS (field round-trip)"); end
            else
                begin fail_count++; $display("    RESULT: FAIL (field round-trip)"); end
        end

        // Pattern 4: cmd port only (payload zero); any non-HIZ cmd with a clean word is valid
        check_word("Test: cmd 101 (payload zero)", 32'd0, 3'b101, 1'b1);

        // Pattern 5: all ones in host_data; reserved bits non-zero -> valid must be 0
        check_word("Test: host_data all 1s (rsvd != 0)", 32'hFFFFFFFF, 3'b111, 1'b0);

        // Pattern 6: PROG-style payload
        check_word("Test: PROG packed addr (arg=FF00) data=0x0F",
                   build_host_ann_word(8'h0F, 16'hFF00), CMD_PROG, 1'b1);

        // Pattern 7: highest address (PE=3 SA=15 ROW=15 COL=63) with INF pixel byte
        check_word("Test: INF max addr PE=3 SA=15 ROW=15 COL=63 data=0xC3",
                   build_host_ann_word(8'hC3, pack_ann_addr(2'd3, 4'd15, 4'd15, 6'd63)), CMD_INF, 1'b1);

        // Pattern 8: well-formed word but HIZ command -> not valid
        check_word("Test: well-formed word with cmd=HIZ",
                   build_host_ann_word(8'h5A, 16'h1234), CMD_HIZ, 1'b0);

        // Pattern 9..16: single reserved bit set -> not valid (walk bits 0..7)
        for (int b = 0; b < WORD_RSVD_WIDTH; b++) begin
            logic [WORD_WIDTH-1:0] w;
            w = build_host_ann_word(8'h3C, 16'h0421) | (32'd1 << b);
            check_word($sformatf("Test: reserved bit %0d set (ERASE)", b), w, CMD_ERASE, 1'b0);
            if (host_word_is_valid(w))
                begin fail_count++; $display("    RESULT: FAIL (host_word_is_valid should be 0)"); end
        end

        #10;
        $display("----------------------------------------");
        $display("SUMMARY: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0)
            $display("RESULT: PASS (all %0d checks passed)", pass_count);
        else
            $display("RESULT: FAIL (%0d passed, %0d failed)", pass_count, fail_count);
        $finish;
    end

endmodule
