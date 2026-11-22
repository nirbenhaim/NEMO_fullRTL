//------------------------------------------------------------------------------
// Input Buffer - Bit-Serial Read (data_lanes) Testbench
//------------------------------------------------------------------------------
// Tests:
//   (1) Load all BUFFER_SIZE (256) entries, one SA block (16 entries) at a time.
//   (2) CTRL_COMPUTE: for every bit_sel 0..7, data_lanes[i] == entry[i][bit_sel]
//       for all 256 lanes, and the lanes do not depend on buf_reg_add.
//   (3) Lane gating: data_lanes == 0 for IDLE / DATA_LOAD / RESULT_OUT /
//       WEIGHT_READ and for CTRL_COMPUTE with buf_read_write=1.
//   (4) Overwrite one SA block (entries 128..143) and re-check all lanes.
//------------------------------------------------------------------------------

`timescale 1ns/1ps

`include "../../../rtl/common/macros.svh"

import ann_arch_pkg::*;
import input_buffer_pkg::*;

module input_buffer_bit_serial_tb;

    localparam int CLK_PERIOD = 10;

    // Wave: 8 latched data_in values - first 8 entries of the block being loaded
    logic [7:0] tb_captured_din_0, tb_captured_din_1, tb_captured_din_2, tb_captured_din_3;
    logic [7:0] tb_captured_din_4, tb_captured_din_5, tb_captured_din_6, tb_captured_din_7;

    // Wave helpers
    logic       tb_pi_load_trigger;   // High while TB drives data-load writes (PI-like load phase)
    logic [8:0] tb_load_pixel_count;  // Counts loaded pixels: 0..BUFFER_SIZE

    logic clk, rst_n;
    logic [BUFFER_DATA_WIDTH-1:0] data_in;
    logic [2:0]                   reg_ctrl;
    logic                         buf_read_write;
    logic [BUFFER_ADDR_WIDTH-1:0] buf_reg_add;
    logic [BUF_BIT_SEL_WIDTH-1:0] bit_sel;
    logic                         ready;
    logic [BUFFER_DATA_WIDTH-1:0] buf_data;
    logic [NUM_DATA_LINES-1:0]    data_lanes;

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

    int pass_count, fail_count;
    logic [BUFFER_DATA_WIDTH-1:0] stored [0:BUFFER_SIZE-1];   // expected entry contents

    task automatic capture_lane(input int i, input logic [7:0] v);
        begin
            case (i)
                0: tb_captured_din_0 = v;
                1: tb_captured_din_1 = v;
                2: tb_captured_din_2 = v;
                3: tb_captured_din_3 = v;
                4: tb_captured_din_4 = v;
                5: tb_captured_din_5 = v;
                6: tb_captured_din_6 = v;
                7: tb_captured_din_7 = v;
                default: ;
            endcase
        end
    endtask

    // Expected bit-plane b over all lanes
    function automatic logic [NUM_DATA_LINES-1:0] exp_plane(input int b);
        logic [NUM_DATA_LINES-1:0] v;
        for (int i = 0; i < NUM_DATA_LINES; i++)
            v[i] = stored[i][b];
        return v;
    endfunction

    // Write one SA block (SA_ROWS entries starting at sa*SA_ROWS); inputs change at negedge
    task automatic load_block(input int sa, input int seed);
        for (int r = 0; r < SA_ROWS; r++) begin
            int a;
            a = sa * SA_ROWS + r;
            @(negedge clk);
            reg_ctrl = CTRL_DATA_LOAD;
            buf_read_write = 1'b1;
            stored[a] = 8'(a * seed + sa + 1);
            buf_reg_add = BUFFER_ADDR_WIDTH'(a);
            data_in = stored[a];
            @(posedge clk);
            capture_lane(r, data_in);
            tb_load_pixel_count = tb_load_pixel_count + 1'b1;
        end
        @(negedge clk);
        reg_ctrl = CTRL_IDLE;
        buf_read_write = 1'b0;
    endtask

    // CTRL_COMPUTE: check all lanes for each bit-plane, with a few buf_reg_add values
    task automatic check_all_planes(input string tag);
        logic [NUM_DATA_LINES-1:0] exp_v;
        int addr_pick [0:2];
        addr_pick[0] = 0; addr_pick[1] = 77; addr_pick[2] = BUFFER_MAX_ADDR;
        @(negedge clk);
        reg_ctrl = CTRL_COMPUTE;
        buf_read_write = 1'b0;
        for (int b = 0; b < BUFFER_DATA_WIDTH; b++) begin
            exp_v = exp_plane(b);
            for (int k = 0; k < 3; k++) begin
                bit_sel = BUF_BIT_SEL_WIDTH'(b);
                buf_reg_add = BUFFER_ADDR_WIDTH'(addr_pick[k]);
                #1;
                if (data_lanes === exp_v) begin
                    pass_count++;
                    if (k == 0)
                        $display("  Test: bit_sel=%0d (%s) buf_reg_add=%0d | Expected: %064h | Actual: %064h | PASS",
                                 b, tag, buf_reg_add, exp_v, data_lanes);
                end else begin
                    fail_count++;
                    $display("  Test: bit_sel=%0d (%s) buf_reg_add=%0d | Expected: %064h | Actual: %064h | FAIL",
                             b, tag, buf_reg_add, exp_v, data_lanes);
                end
            end
            @(negedge clk);
        end
        // buf_data still returns the full byte at buf_reg_add in COMPUTE
        buf_reg_add = 8'd200;
        #1;
        if (buf_data === stored[200] && ready === 1'b1) pass_count++;
        else begin
            fail_count++;
            $display("  FAIL (%s): COMPUTE buf_data[200]=%02h exp_v=%02h ready=%0d", tag, buf_data, stored[200], ready);
        end
        @(negedge clk);
        reg_ctrl = CTRL_IDLE;
    endtask

    // Lanes must be 0 outside (CTRL_COMPUTE && read)
    task automatic check_gated(input logic [2:0] ctrl, input logic rw, input string name);
        bit ok;
        ok = 1;
        @(negedge clk);
        reg_ctrl = ctrl;
        buf_read_write = rw;
        buf_reg_add = 8'd5;
        data_in = stored[5];   // DATA_LOAD/write rewrites entry 5 with its own value (no change)
        for (int b = 0; b < BUFFER_DATA_WIDTH; b++) begin
            bit_sel = BUF_BIT_SEL_WIDTH'(b);
            #1;
            if (data_lanes !== '0) ok = 0;
        end
        if (ok) begin
            pass_count++;
            $display("  Test: lane gating reg_ctrl=%s rw=%0d | Expected: all lanes 0 | PASS", name, rw);
        end else begin
            fail_count++;
            $display("  Test: lane gating reg_ctrl=%s rw=%0d | Expected: all lanes 0 | Actual: %064h | FAIL",
                     name, rw, data_lanes);
        end
        @(negedge clk);
        reg_ctrl = CTRL_IDLE;
        buf_read_write = 1'b0;
    endtask

    initial begin
        rst_n = 0;
        reg_ctrl = CTRL_IDLE;
        buf_read_write = 0;
        buf_reg_add = 0;
        data_in = 0;
        bit_sel = 0;
        pass_count = 0;
        fail_count = 0;
        tb_load_pixel_count = '0;
        for (int i = 0; i < BUFFER_SIZE; i++) stored[i] = '0;
        tb_captured_din_0 = '0; tb_captured_din_1 = '0; tb_captured_din_2 = '0; tb_captured_din_3 = '0;
        tb_captured_din_4 = '0; tb_captured_din_5 = '0; tb_captured_din_6 = '0; tb_captured_din_7 = '0;
        #(2*CLK_PERIOD);
        @(negedge clk);
        rst_n = 1;
        #(2*CLK_PERIOD);

        $display("----------------------------------------");
        $display("Input Buffer Bit-Serial (data_lanes) Tests");
        $display("Summary: %0d pixels (%0d SA blocks x %0d rows), %0d lanes, %0d bit-planes",
                 BUFFER_SIZE, NUM_SA, SA_ROWS, NUM_DATA_LINES, BUFFER_DATA_WIDTH);
        $display("----------------------------------------");

        // ---- (1) Load all 256 entries, one SA block at a time ----
        $display("--- Load: entries 0-%0d (one block per SA) ---", BUFFER_MAX_ADDR);
        for (int sa = 0; sa < NUM_SA; sa++)
            load_block(sa, 13);

        // ---- (2) Bit-serial lanes over the whole depth ----
        $display("--- COMPUTE: all %0d lanes, bit_sel 0-7, buf_reg_add in {0,77,255} ---", NUM_DATA_LINES);
        check_all_planes("full 256");

        // ---- (3) Lane gating ----
        $display("--- Lane gating (lanes = 0 outside CTRL_COMPUTE read) ---");
        check_gated(CTRL_IDLE,        1'b0, "IDLE");
        check_gated(CTRL_DATA_LOAD,   1'b1, "DATA_LOAD");
        check_gated(CTRL_RESULT_OUT,  1'b0, "RESULT_OUT");
        check_gated(CTRL_WEIGHT_READ, 1'b0, "WEIGHT_READ");
        check_gated(CTRL_COMPUTE,     1'b1, "COMPUTE(rw=1)");

        // ---- (4) Overwrite SA 8 (entries 128..143) and recheck ----
        $display("--- Overwrite SA 8 (entries 128-143) and recheck all lanes ---");
        tb_load_pixel_count = '0;
        load_block(8, 29);
        check_all_planes("after SA8 overwrite");

        #(2*CLK_PERIOD);
        // TB REPORT: printed last so it appears at the end of target/input_buffer/<tb>_log.txt
        $display("");
        $display("//==============================================================================");
        $display("// TB REPORT  input_buffer_bit_serial_tb");
        $display("// Bit-serial data_lanes: %0d entries x %0d bit-planes, lane gating, SA-block overwrite",
                 BUFFER_SIZE, BUFFER_DATA_WIDTH);
        $display("//==============================================================================");
        $display("SUMMARY: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0)
            $display("RESULT: PASS (bit-serial checks)");
        else
            $display("RESULT: FAIL (%0d passed, %0d failed)", pass_count, fail_count);
        $finish;
    end

endmodule
