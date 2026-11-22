//------------------------------------------------------------------------------
// Input Buffer - Reset Behavior Focused Testbench
//------------------------------------------------------------------------------
// Focus:
//   1) Reset while loading bytes into buffer (all 256 entries cleared).
//   2) Full 256-entry load, lane gating outside CTRL_COMPUTE, bit-serial baseline.
//   3) Reset while reading bit-serial outputs (data_lanes) in CTRL_COMPUTE.
//------------------------------------------------------------------------------

`timescale 1ns/1ps

`include "../../../rtl/common/macros.svh"

import ann_arch_pkg::*;
import input_buffer_pkg::*;

module input_buffer_reset_behavior_tb;

    localparam int CLK_PERIOD = 10;
    localparam int RESET_ADDR = 20;

    // Wave helpers (explicitly show load trigger and pixel counter)
    logic       tb_pi_load_trigger;
    logic [8:0] tb_load_pixel_count;

    logic clk, rst_n;
    logic [BUFFER_DATA_WIDTH-1:0] data_in;
    logic [2:0]                   reg_ctrl;
    logic                         buf_read_write;
    logic [BUFFER_ADDR_WIDTH-1:0] buf_reg_add;
    logic [BUF_BIT_SEL_WIDTH-1:0] bit_sel;
    logic                         ready;
    logic [BUFFER_DATA_WIDTH-1:0] buf_data;
    logic [NUM_DATA_LINES-1:0]    data_lanes;

    logic [BUFFER_DATA_WIDTH-1:0] load_vals [0:BUFFER_SIZE-1];
    int pass_count, fail_count;

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

    assign tb_pi_load_trigger = (reg_ctrl == CTRL_DATA_LOAD) && buf_read_write;

    task automatic check(input bit cond, input string pass_msg, input string fail_msg);
        begin
            if (cond) begin
                pass_count++;
                $display("  PASS: %s", pass_msg);
            end else begin
                fail_count++;
                $display("  FAIL: %s", fail_msg);
            end
        end
    endtask

    // Write one byte (inputs driven at negedge, captured at the next posedge)
    task automatic write_byte(input int addr, input logic [7:0] val);
        @(negedge clk);
        reg_ctrl = CTRL_DATA_LOAD;
        buf_read_write = 1'b1;
        buf_reg_add = BUFFER_ADDR_WIDTH'(addr);
        data_in = val;
        @(posedge clk);
        tb_load_pixel_count = tb_load_pixel_count + 1'b1;
    endtask

    // Read every entry through buf_data (CTRL_WEIGHT_READ); count non-zero entries
    task automatic count_nonzero_entries(output int nz, output int first_nz);
        nz = 0;
        first_nz = -1;
        @(negedge clk);
        reg_ctrl = CTRL_WEIGHT_READ;
        buf_read_write = 1'b0;
        for (int a = 0; a < BUFFER_SIZE; a++) begin
            buf_reg_add = BUFFER_ADDR_WIDTH'(a);
            #1;
            if (buf_data !== 8'h00) begin
                nz++;
                if (first_nz < 0) first_nz = a;
            end
        end
        reg_ctrl = CTRL_IDLE;
    endtask

    function automatic logic [NUM_DATA_LINES-1:0] exp_plane(input int b);
        logic [NUM_DATA_LINES-1:0] v;
        for (int i = 0; i < NUM_DATA_LINES; i++)
            v[i] = load_vals[i][b];
        return v;
    endfunction

    initial begin
        int nz, first_nz;

        // Init
        rst_n = 0;
        reg_ctrl = CTRL_IDLE;
        buf_read_write = 0;
        buf_reg_add = 0;
        data_in = 0;
        bit_sel = 0;
        tb_load_pixel_count = 0;
        pass_count = 0;
        fail_count = 0;

        for (int i = 0; i < BUFFER_SIZE; i++) begin
            load_vals[i] = 8'((i + 1) * 8'h13); // deterministic pattern
            if (load_vals[i] == 8'h00) load_vals[i] = 8'h5A;
        end

        repeat (2) @(posedge clk);
        @(negedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        $display("----------------------------------------------------");
        $display("Input Buffer Reset Behavior Test (%0d entries, %0d lanes)", BUFFER_SIZE, NUM_DATA_LINES);
        $display("----------------------------------------------------");

        // ------------------------------------------------------------
        // A) Reset while loading
        // ------------------------------------------------------------
        $display("A) Reset while DATA_LOAD is active");
        tb_load_pixel_count = 0;

        // Write first 4 pixels (addr 16..19) plus the last entry of the buffer
        for (int i = 0; i < 4; i++)
            write_byte(16 + i, load_vals[i]);
        write_byte(BUFFER_MAX_ADDR, 8'h3C);

        // Write addr20, then assert reset while DATA_LOAD is still driven
        write_byte(RESET_ADDR, 8'hA5);
        @(negedge clk);
        check(dut.buffer_reg[RESET_ADDR] == 8'hA5 && dut.buffer_reg[BUFFER_MAX_ADDR] == 8'h3C,
              $sformatf("addr %0d / %0d written before reset", RESET_ADDR, BUFFER_MAX_ADDR),
              $sformatf("pre-reset writes missing (addr%0d=%02x addr%0d=%02x)", RESET_ADDR,
                        dut.buffer_reg[RESET_ADDR], BUFFER_MAX_ADDR, dut.buffer_reg[BUFFER_MAX_ADDR]));

        rst_n = 1'b0;
        tb_load_pixel_count = 0;
        @(posedge clk);
        @(negedge clk);
        // Prevent immediate rewrite of addr20 after reset deassertion.
        reg_ctrl = CTRL_IDLE;
        buf_read_write = 1'b0;
        rst_n = 1'b1;
        @(posedge clk);

        // Verify address cleared by reset
        @(negedge clk);
        reg_ctrl = CTRL_WEIGHT_READ;
        buf_read_write = 1'b0;
        buf_reg_add = RESET_ADDR[BUFFER_ADDR_WIDTH-1:0];
        #1;
        check(
            buf_data == 8'h00,
            $sformatf("addr %0d cleared to 0 after reset during load", RESET_ADDR),
            $sformatf("addr %0d expected 0x00, got 0x%02x", RESET_ADDR, buf_data)
        );
        count_nonzero_entries(nz, first_nz);
        check(nz == 0,
              $sformatf("all %0d entries read 0x00 after reset during load", BUFFER_SIZE),
              $sformatf("%0d entries non-zero after reset (first at %0d)", nz, first_nz));

        // ------------------------------------------------------------
        // B) Load all 256 pixels, lane gating, bit-serial baseline
        // ------------------------------------------------------------
        $display("B) Full %0d-entry load, lane gating and bit-serial baseline before reset", BUFFER_SIZE);
        tb_load_pixel_count = 0;
        for (int i = 0; i < BUFFER_SIZE; i++)
            write_byte(i, load_vals[i]);

        // Lanes stay 0 while loading / reading through buf_data
        @(negedge clk);
        bit_sel = 3'd0;
        reg_ctrl = CTRL_DATA_LOAD;
        buf_read_write = 1'b0;   // DATA_LOAD with read: no write, no lanes
        #1;
        check(data_lanes == '0, "data_lanes = 0 in CTRL_DATA_LOAD",
              $sformatf("data_lanes non-zero in CTRL_DATA_LOAD: %064h", data_lanes));
        reg_ctrl = CTRL_WEIGHT_READ;
        #1;
        check(data_lanes == '0, "data_lanes = 0 in CTRL_WEIGHT_READ",
              $sformatf("data_lanes non-zero in CTRL_WEIGHT_READ: %064h", data_lanes));
        reg_ctrl = CTRL_IDLE;
        #1;
        check(data_lanes == '0, "data_lanes = 0 in CTRL_IDLE",
              $sformatf("data_lanes non-zero in CTRL_IDLE: %064h", data_lanes));

        @(negedge clk);
        reg_ctrl = CTRL_COMPUTE;
        buf_read_write = 1'b0;
        tb_load_pixel_count = 0;
        buf_reg_add = 8'd24;
        bit_sel = 3'd0;
        #1;
        check(
            data_lanes === exp_plane(0),
            "bit-serial plane 0 matches loaded bytes on all 256 lanes before reset",
            $sformatf("bit-serial plane 0 mismatch before reset: exp=%064h got=%064h", exp_plane(0), data_lanes)
        );

        // ------------------------------------------------------------
        // C) Reset during bit-serial
        // ------------------------------------------------------------
        $display("C) Reset while COMPUTE/bit-serial is active");
        @(negedge clk);
        bit_sel = 3'd3;
        #1;
        check(
            data_lanes === exp_plane(3),
            "bit-serial plane 3 matches loaded bytes just before reset",
            $sformatf("bit-serial plane 3 mismatch: exp=%064h got=%064h", exp_plane(3), data_lanes)
        );
        @(negedge clk);
        rst_n = 1'b0;
        #1;
        check(
            data_lanes == '0,
            "data_lanes forced low during reset",
            $sformatf("data_lanes not all zero during reset: %064h", data_lanes)
        );
        @(negedge clk);
        rst_n = 1'b1;
        #1;
        // Still in CTRL_COMPUTE after reset: buffer is empty so every plane is 0
        begin
            bit all_zero;
            all_zero = 1;
            for (int b = 0; b < BUFFER_DATA_WIDTH; b++) begin
                bit_sel = BUF_BIT_SEL_WIDTH'(b);
                #1;
                if (data_lanes !== '0) all_zero = 0;
            end
            check(all_zero, "data_lanes all 0 for every bit-plane after reset (still in COMPUTE)",
                  "data_lanes non-zero after reset in COMPUTE");
        end

        // Confirm memory cleared after reset by reading every entry
        buf_reg_add = 8'd24;
        reg_ctrl = CTRL_WEIGHT_READ;
        #1;
        check(
            buf_data == 8'h00,
            "buffer content cleared after reset during compute",
            $sformatf("buffer[24] expected 0x00 after reset, got 0x%02x", buf_data)
        );
        count_nonzero_entries(nz, first_nz);
        check(nz == 0,
              $sformatf("all %0d entries read 0x00 after reset during compute", BUFFER_SIZE),
              $sformatf("%0d entries non-zero after reset (first at %0d)", nz, first_nz));

        // TB REPORT: printed last so it appears at the end of target/input_buffer/<tb>_log.txt
        $display("");
        $display("//==============================================================================");
        $display("// TB REPORT  input_buffer_reset_behavior_tb");
        $display("// Reset during CTRL_DATA_LOAD and during COMPUTE/bit-serial (%0d entries, %0d lanes)",
                 BUFFER_SIZE, NUM_DATA_LINES);
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
