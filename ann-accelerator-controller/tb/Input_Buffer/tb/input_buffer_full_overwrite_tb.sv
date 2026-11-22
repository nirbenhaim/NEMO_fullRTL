//------------------------------------------------------------------------------
// Input Buffer - Full Load then Overwrite Behavior Testbench
//------------------------------------------------------------------------------
// Goal:
//   1) Fill all BUFFER_SIZE (256) buffer locations (16 SA blocks x 16 rows).
//   2) Show full-buffer contents (one line per SA) and verify via buf_data,
//      with data_lanes gated to 0 during CTRL_WEIGHT_READ.
//   3) Load 10 new pixels to addresses 0..9 and dump all rows after each write
//      to visualize replacement behavior; also overwrite the last entry (255).
//   4) Verify overwritten/untouched entries via buf_data and via the
//      CTRL_COMPUTE data_lanes (all 8 bit-planes).
//------------------------------------------------------------------------------

`timescale 1ns/1ps

`include "../../../rtl/common/macros.svh"

import ann_arch_pkg::*;
import input_buffer_pkg::*;

module input_buffer_full_overwrite_tb;

    localparam int CLK_PERIOD = 10;
    localparam int NUM_NEW_PX = 10;

    logic clk, rst_n;
    logic [BUFFER_DATA_WIDTH-1:0] data_in;
    logic [2:0]                   reg_ctrl;
    logic                         buf_read_write;
    logic [BUFFER_ADDR_WIDTH-1:0] buf_reg_add;
    logic [BUF_BIT_SEL_WIDTH-1:0] bit_sel;
    logic                         ready;
    logic [BUFFER_DATA_WIDTH-1:0] buf_data;
    logic [NUM_DATA_LINES-1:0]    data_lanes;

    int pass_count, fail_count;
    logic [BUFFER_DATA_WIDTH-1:0] expected [0:BUFFER_SIZE-1];
    logic [BUFFER_DATA_WIDTH-1:0] new_px [0:NUM_NEW_PX-1];

    input_buffer dut (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(data_in),
        .ready(ready),
        .reg_ctrl(reg_ctrl),
        .buf_read_write(buf_read_write),
        .buf_reg_add(buf_reg_add),
        .bit_sel(bit_sel),
        .buf_data(buf_data),
        .data_lanes(data_lanes)
    );

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // One line per sub-array: entries [sa*SA_ROWS .. sa*SA_ROWS+SA_ROWS-1]
    task automatic dump_all_rows(input string tag);
        string line;
        $display("%s", tag);
        for (int s = 0; s < NUM_SA; s++) begin
            line = $sformatf("  SA%02d [%3d..%3d]:", s, s*SA_ROWS, s*SA_ROWS + SA_ROWS - 1);
            for (int r = 0; r < SA_ROWS; r++)
                line = {line, $sformatf(" %02h", dut.buffer_reg[s*SA_ROWS + r])};
            $display("%s", line);
        end
    endtask

    // Inputs driven at negedge, written at the next posedge
    task automatic write_byte(input int addr, input logic [7:0] val);
        begin
            @(negedge clk);
            reg_ctrl = CTRL_DATA_LOAD;
            buf_read_write = 1'b1;
            buf_reg_add = BUFFER_ADDR_WIDTH'(addr);
            data_in = val;
            @(posedge clk);
        end
    endtask

    task automatic check_addr(input int idx);
        begin
            @(negedge clk);
            reg_ctrl = CTRL_WEIGHT_READ;
            buf_read_write = 1'b0;
            buf_reg_add = BUFFER_ADDR_WIDTH'(idx);
            #1;
            if (buf_data === expected[idx] && ready === 1'b1 && data_lanes === '0) begin
                pass_count++;
            end else begin
                fail_count++;
                $display("  FAIL check addr=%0d exp=%02h got=%02h ready=%0d lanes_zero=%0d",
                         idx, expected[idx], buf_data, ready, (data_lanes === '0));
            end
        end
    endtask

    // CTRL_COMPUTE: every lane carries bit b of its entry
    task automatic check_lanes(input string tag);
        logic [NUM_DATA_LINES-1:0] exp_v;
        @(negedge clk);
        reg_ctrl = CTRL_COMPUTE;
        buf_read_write = 1'b0;
        for (int b = 0; b < BUFFER_DATA_WIDTH; b++) begin
            for (int i = 0; i < NUM_DATA_LINES; i++)
                exp_v[i] = expected[i][b];
            bit_sel = BUF_BIT_SEL_WIDTH'(b);
            #1;
            if (data_lanes === exp_v) begin
                pass_count++;
            end else begin
                fail_count++;
                $display("  FAIL lanes (%s) bit_sel=%0d exp=%064h got=%064h", tag, b, exp_v, data_lanes);
            end
        end
        @(negedge clk);
        reg_ctrl = CTRL_IDLE;
        bit_sel = '0;
    endtask

    initial begin
        rst_n = 0;
        reg_ctrl = CTRL_IDLE;
        buf_read_write = 1'b0;
        buf_reg_add = '0;
        data_in = 8'd0;
        bit_sel = 3'd0;
        pass_count = 0;
        fail_count = 0;

        for (int i = 0; i < BUFFER_SIZE; i++) expected[i] = 8'd0;
        for (int i = 0; i < NUM_NEW_PX; i++) new_px[i] = 8'(8'hA0 + i);

        repeat (2) @(posedge clk);
        @(negedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        $display("------------------------------------------------------------");
        $display("Input Buffer Full Overwrite Behavior Test");
        $display("Phase 1: Fill all %0d addresses (%0d SA x %0d rows)", BUFFER_SIZE, NUM_SA, SA_ROWS);
        $display("Phase 2: Write %0d new bytes to addr 0..%0d, dump rows each write; overwrite addr %0d",
                 NUM_NEW_PX, NUM_NEW_PX - 1, BUFFER_MAX_ADDR);
        $display("------------------------------------------------------------");

        // Phase 1: fill all entries
        for (int a = 0; a < BUFFER_SIZE; a++) begin
            logic [7:0] v;
            v = 8'((a * 3) + 8'h11);
            write_byte(a, v);
            expected[a] = v;
        end

        dump_all_rows($sformatf("After full load (%0d/%0d):", BUFFER_SIZE, BUFFER_SIZE));

        // Quick verification after full load
        for (int a = 0; a < BUFFER_SIZE; a++) begin
            check_addr(a);
        end
        check_lanes("after full load");

        // Phase 2: overwrite first NUM_NEW_PX addresses
        $display("");
        $display("Overwrite phase: loading %0d new pixels to addr 0..%0d", NUM_NEW_PX, NUM_NEW_PX - 1);
        for (int i = 0; i < NUM_NEW_PX; i++) begin
            write_byte(i, new_px[i]);
            expected[i] = new_px[i];
            @(negedge clk);
            reg_ctrl = CTRL_IDLE;
            buf_read_write = 1'b0;
            dump_all_rows($sformatf("After overwrite #%0d (addr=%0d, data=%02h):", i+1, i, new_px[i]));
        end

        // Overwrite the last entry (top of the address range)
        write_byte(BUFFER_MAX_ADDR, 8'hEE);
        expected[BUFFER_MAX_ADDR] = 8'hEE;
        @(negedge clk);
        reg_ctrl = CTRL_IDLE;
        buf_read_write = 1'b0;
        dump_all_rows($sformatf("After overwrite of last entry (addr=%0d, data=EE):", BUFFER_MAX_ADDR));

        // Verify overwritten and untouched ranges
        for (int a = 0; a < BUFFER_SIZE; a++) begin
            check_addr(a);
        end
        check_lanes("after overwrite");

        reg_ctrl = CTRL_IDLE;
        buf_read_write = 1'b0;

        // TB REPORT at end of log
        $display("");
        $display("//==============================================================================");
        $display("// TB REPORT  input_buffer_full_overwrite_tb");
        $display("// Full %0d-entry load then overwrite %0d pixels (addr 0..%0d) + addr %0d, row dumps after each write",
                 BUFFER_SIZE, NUM_NEW_PX, NUM_NEW_PX - 1, BUFFER_MAX_ADDR);
        $display("//==============================================================================");
        $display("SUMMARY: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");

        # (2*CLK_PERIOD);
        $finish;
    end

endmodule
