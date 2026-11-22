//------------------------------------------------------------------------------
// Controller INF -> Input Buffer Flow Testbench
//------------------------------------------------------------------------------
// Shows:
//   1) Host INF packets entering PI (one-cycle strobes, some with HIZ gaps).
//   2) The INF start beat moves S_IDLE -> S_COLLECT_DATA (its data is not stored),
//      then INPUT_VECTOR_LEN (256) pixel beats are written to buffer entries
//      0..255 in order (pixel k -> entry k).
//   3) inf_start is high exactly in the cycle the 256th pixel is accepted.
//   4) S_COMPUTE lasts T_INF_COMPUTE (2176) cycles = 16 column positions x
//      (8 bit-steps x 16 + boundary 8).  Every cycle is compared with a
//      reference model of the schedule (NEMO CS-IMC Architecture Brief
//      Rev. 1.0 Fig. 3/4, MSB first): col_sel = k held for 8 planes,
//      buf_bit_sel = 7, 6, ..., 0 (next plane driven in the last cycle of each
//      bit-step), pulses = INF in cycles 0..9 of a bit-step, tree_start in
//      cycle 10, dsa_clear in the first cycle of each column, data_lanes =
//      the selected plane of every entry, compute_done in the last cycle.
//   5) S_RESULT for one cycle, then back to S_IDLE.
// Two inference runs with different pixel patterns are performed.
//------------------------------------------------------------------------------

`timescale 1ns/1ps

`include "../../../../../rtl/common/macros.svh"

import ann_arch_pkg::*;
import controller_pkg::*;
import input_buffer_pkg::*;
import parallel_interface_pkg::*;

module controller_inf_buffer_flow_tb;

    localparam int CLK_PERIOD = 5;
    localparam string LOG_FILE = "target/Controller/inf/controller_inf_buffer_flow_tb_log.txt";
    localparam int NUM_RUNS   = 2;
    localparam int DETAIL_BEATS = 4;   // detailed log for the first/last N beats of a run

    logic        clk, rst_n, reset;
    logic [WORD_WIDTH-1:0]        host_data;
    logic [2:0]                   host_cmd;
    logic                         valid;
    logic [ACT_WIDTH-1:0]         pi_data;
    logic [WEIGHT_ADDR_WIDTH-1:0] address;
    logic [CMD_WIDTH-1:0]         cmd;
    logic        ann_reset, op_done, busy;
    logic [WORD_WIDTH-1:0]        ann_address;
    logic [2:0]                   pulses;
    logic [BUFFER_ADDR_WIDTH-1:0] buf_reg_add;
    logic [2:0]                   buf_reg_ctrl;
    logic                         buf_read_write;
    logic [BUF_BIT_SEL_WIDTH-1:0] buf_bit_sel;
    logic [BUFFER_DATA_WIDTH-1:0] buf_data_out, buf_data;
    logic [NUM_DATA_LINES-1:0]    data_lanes;
    logic                         buf_ready;
    logic [ANN_WEIGHT_WIDTH-1:0]  weight_read_data_mock;
    logic                         dsa_clear, compute_done, inf_start, tree_start;
    logic [COL_SEL_WIDTH-1:0]     col_sel;

    int fd;
    int pass_count, fail_count;

    // Cycle-level event monitor (sampled at posedge, inputs driven at negedge)
    int cyc;
    int dsa_clear_count, compute_done_count, compute_cycle_count, inf_start_count, tree_start_count;
    int inf_start_cyc, compute_done_cyc, first_compute_cyc;
    bit in_compute_d;

    parallel_interface u_pi (
        .clk(clk), .reset(reset), .host_data(host_data), .host_cmd(host_cmd),
        .valid(valid), .data(pi_data), .address(address), .cmd(cmd)
    );

    ann_controller dut (
        .clk(clk), .rst_n(rst_n),
        .valid(valid), .data(pi_data), .address(address), .cmd(cmd),
        .ann_reset(ann_reset),
        .op_done(op_done), .ann_address(ann_address), .pulses(pulses),
        .weight_read_data(weight_read_data_mock),
        .buf_reg_add(buf_reg_add), .buf_reg_ctrl(buf_reg_ctrl), .buf_read_write(buf_read_write),
        .buf_bit_sel(buf_bit_sel),
        .buf_data_out(buf_data_out), .buf_ready(buf_ready), .buf_data(buf_data),
        .inf_start(inf_start), .dsa_clear(dsa_clear), .tree_start(tree_start), .col_sel(col_sel),
        .compute_done(compute_done), .prog_fail(), .op_timeout(), .read_data(), .read_done(),
        .busy(busy)
    );

    input_buffer u_input_buffer (
        .clk(clk), .rst_n(rst_n), .data_in(buf_data_out),
        .ready(buf_ready), .reg_ctrl(buf_reg_ctrl), .buf_read_write(buf_read_write),
        .buf_reg_add(buf_reg_add), .bit_sel(buf_bit_sel), .buf_data(buf_data),
        .data_lanes(data_lanes)
    );

    assign weight_read_data_mock = buf_data[ANN_WEIGHT_WIDTH-1:0];

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // Not used by the INF flow, keep low.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) op_done <= 1'b0;
        else op_done <= 1'b0;
    end

    //--------------------------------------------------------------------------
    // Pixel patterns (bijective in k so a mis-ordered write is detected)
    //--------------------------------------------------------------------------
    function automatic logic [7:0] pixel_of(input int run, input int k);
        if (run == 0)
            return 8'(k) ^ 8'h5A;
        else
            return 8'(k * 7 + 3);
    endfunction

    // Expected bit-plane b of the whole input vector (lane k = pixel k bit b)
    function automatic logic [NUM_DATA_LINES-1:0] exp_plane(input int run, input int b);
        logic [NUM_DATA_LINES-1:0] v;
        logic [7:0] px;
        for (int k = 0; k < NUM_DATA_LINES; k++) begin
            px = pixel_of(run, k);
            v[k] = px[b];
        end
        return v;
    endfunction

    // Host-side address for pixel k: SA = k / SA_ROWS, ROW = k % SA_ROWS
    function automatic logic [WEIGHT_ADDR_WIDTH-1:0] pixel_addr(input int k);
        return pack_ann_addr(PE_ID_WIDTH'(0), SA_ID_WIDTH'(k / SA_ROWS),
                             ROW_ID_WIDTH'(k % SA_ROWS), COL_ID_WIDTH'(0));
    endfunction

    //--------------------------------------------------------------------------
    // Check / log helpers
    //--------------------------------------------------------------------------
    task automatic check(input bit cond, input string msg);
        if (cond) begin
            pass_count++;
        end else begin
            fail_count++;
            $fdisplay(fd, "  FAIL: %s", msg);
        end
    endtask

    task automatic dump_buffer(input string tag);
        $fdisplay(fd, "%s", tag);
        for (int s = 0; s < NUM_SA; s++) begin
            $fwrite(fd, "  SA%02d [%3d..%3d]:", s, s*SA_ROWS, s*SA_ROWS + SA_ROWS - 1);
            for (int r = 0; r < SA_ROWS; r++)
                $fwrite(fd, " %02h", u_input_buffer.buffer_reg[s*SA_ROWS + r]);
            $fwrite(fd, "\n");
        end
    endtask

    task automatic log_beat(input string label);
        $fdisplay(fd, "%s", label);
        $fdisplay(fd, "  host_data: 0x%08h (data=%02h pe=%0d sa=%0d row=%0d col=%0d rsvd=%02h), cmd=%b",
            host_data, word_data(host_data), get_pe_id(word_addr(host_data)),
            get_sa_id(word_addr(host_data)), get_row_id(word_addr(host_data)),
            get_col_id(word_addr(host_data)), host_data[7:0], host_cmd);
        $fdisplay(fd, "  PI->CTRL : valid=%0d data=0x%02h addr=0x%04h cmd=%b", valid, pi_data, address, cmd);
        $fdisplay(fd, "  CTRL     : state=%s busy=%0d buf_ctrl=%0d buf_rw=%0d buf_addr=%0d buf_din=%02h pulses=%b bit_sel=%0d inf_start=%0d",
            dut.state.name(), busy, buf_reg_ctrl, buf_read_write, buf_reg_add, buf_data_out, pulses, buf_bit_sel, inf_start);
    endtask

    //--------------------------------------------------------------------------
    // Event monitor: dsa_clear / compute_done / COMPUTE window timing
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            cyc <= 0;
        end else begin
            cyc <= cyc + 1;
            if (inf_start) begin
                inf_start_count++;
                inf_start_cyc = cyc;
            end
            if (dsa_clear)  dsa_clear_count++;
            if (tree_start) tree_start_count++;
            if (compute_done) begin
                compute_done_count++;
                compute_done_cyc = cyc;
            end
            if (buf_reg_ctrl == CTRL_COMPUTE) begin
                compute_cycle_count++;
                if (!in_compute_d)
                    first_compute_cyc = cyc;
            end
            in_compute_d = (buf_reg_ctrl == CTRL_COMPUTE);
        end
    end

    task automatic clear_monitor();
        dsa_clear_count     = 0;
        inf_start_count     = 0;
        tree_start_count    = 0;
        compute_done_count  = 0;
        compute_cycle_count = 0;
        inf_start_cyc       = -1;
        compute_done_cyc    = -1;
        first_compute_cyc   = -1;
    endtask

    //--------------------------------------------------------------------------
    // One complete inference: start beat + 256 pixels + COMPUTE + RESULT
    //--------------------------------------------------------------------------
    task automatic run_inference(input int run);
        int  fails_before;
        logic [NUM_DATA_LINES-1:0] exp_lanes;

        fails_before = fail_count;
        clear_monitor();

        $fdisplay(fd, "");
        $fdisplay(fd, "==== INF run %0d: pattern %s ====", run,
                  (run == 0) ? "pixel[k] = k ^ 0x5A" : "pixel[k] = 7*k + 3");

        // ---- Start beat (IDLE -> COLLECT_DATA, data not stored) ----
        @(negedge clk);
        host_data = build_host_ann_word(8'hEE, '0);
        host_cmd  = CMD_INF;
        #1;
        log_beat("Start beat (INF, data 0xEE must NOT be stored):");
        check(dut.state == S_IDLE, "start beat: controller not in S_IDLE");
        check(valid === 1'b1, "start beat: PI valid not asserted");
        check(buf_reg_ctrl == CTRL_IDLE && !buf_read_write,
              "start beat: controller wrote the buffer in S_IDLE");
        check(!inf_start && !dsa_clear, "start beat: inf_start/dsa_clear asserted");

        // ---- 256 pixel beats ----
        for (int k = 0; k < INPUT_VECTOR_LEN; k++) begin
            @(negedge clk);
            host_data = build_host_ann_word(pixel_of(run, k), pixel_addr(k));
            host_cmd  = CMD_INF;
            #1;
            if (k < DETAIL_BEATS || k >= INPUT_VECTOR_LEN - DETAIL_BEATS)
                log_beat($sformatf("Pixel beat %0d:", k));
            check(dut.state == S_COLLECT_DATA,
                  $sformatf("beat %0d: state=%s, expected S_COLLECT_DATA", k, dut.state.name()));
            check(valid === 1'b1, $sformatf("beat %0d: PI valid not asserted", k));
            check(buf_reg_ctrl == CTRL_DATA_LOAD && buf_read_write,
                  $sformatf("beat %0d: buffer not in DATA_LOAD/write (ctrl=%0d rw=%0d)", k, buf_reg_ctrl, buf_read_write));
            check(buf_reg_add == BUFFER_ADDR_WIDTH'(k),
                  $sformatf("beat %0d: buf_reg_add=%0d expected %0d", k, buf_reg_add, k));
            check(buf_data_out == pixel_of(run, k),
                  $sformatf("beat %0d: buf_data_out=%02h expected %02h", k, buf_data_out, pixel_of(run, k)));
            check(inf_start === (k == INPUT_VECTOR_LEN - 1),
                  $sformatf("beat %0d: inf_start=%0d expected %0d", k, inf_start, (k == INPUT_VECTOR_LEN - 1)));
            check(!dsa_clear, $sformatf("beat %0d: dsa_clear during collect", k));
            check(data_lanes == '0, $sformatf("beat %0d: data_lanes not 0 during collect", k));
            check(pulses == PULSE_MODE_HIZ, $sformatf("beat %0d: pulses=%b during collect", k, pulses));

            // HIZ gap after every 4th beat (not after the last): valid is a strobe,
            // the pixel counter must hold while no beat is presented.
            if ((k % 4) == 3 && k != INPUT_VECTOR_LEN - 1) begin
                @(negedge clk);
                host_data = '0;
                host_cmd  = CMD_HIZ;
                #1;
                check(!valid && !inf_start && dut.state == S_COLLECT_DATA &&
                      buf_reg_add == BUFFER_ADDR_WIDTH'(k + 1),
                      $sformatf("gap after beat %0d: unexpected valid/inf_start/state/addr (addr=%0d)", k, buf_reg_add));
            end
        end

        // ---- Release host, controller must now be in COMPUTE ----
        @(negedge clk);
        host_data = '0;
        host_cmd  = CMD_HIZ;
        #1;

        dump_buffer("After collect phase (buffer contents, one line per SA):");
        begin
            int bad;
            bad = 0;
            for (int k = 0; k < BUFFER_SIZE; k++) begin
                if (u_input_buffer.buffer_reg[k] !== pixel_of(run, k)) begin
                    bad++;
                    if (bad <= 8)
                        $fdisplay(fd, "  FAIL: entry %0d = %02h expected %02h",
                                  k, u_input_buffer.buffer_reg[k], pixel_of(run, k));
                end
            end
            check(bad == 0, $sformatf("%0d/%0d buffer entries hold the wrong pixel", bad, BUFFER_SIZE));
            if (bad == 0)
                $fdisplay(fd, "PASS: all %0d buffer entries hold their pixel (pixel k -> entry k)", BUFFER_SIZE);
        end

        // ---- COMPUTE: T_INF_COMPUTE cycles, compared with a schedule model ----
        $fdisplay(fd, "");
        $fdisplay(fd, "Compute phase (logged: column 0 bit-steps 0-1, column 0 boundary, last 4 cycles)");
        for (int c = 0; c < T_INF_COMPUTE; c++) begin
            int col, w, step, cy, bnd, e_plane, e_bit_sel, e_col;
            bit e_inf, e_dsa, e_tree, in_bnd;
            if (c > 0) begin
                @(negedge clk);
                #1;
            end
            // reference schedule
            col    = c / T_INF_COLUMN;
            w      = c % T_INF_COLUMN;
            in_bnd = (w >= ACT_WIDTH * T_INF_STEP);
            step   = in_bnd ? ACT_WIDTH - 1 : w / T_INF_STEP;
            cy     = in_bnd ? 0 : w % T_INF_STEP;
            bnd    = in_bnd ? w - ACT_WIDTH * T_INF_STEP : 0;
            e_plane   = (ACT_WIDTH - 1) - step;                                  // MSB first
            if (in_bnd)                           e_bit_sel = ACT_WIDTH - 1;     // next column's MSB
            else if (cy == T_INF_STEP - 1)        e_bit_sel = (step == ACT_WIDTH - 1) ? ACT_WIDTH - 1 : e_plane - 1;
            else                                  e_bit_sel = e_plane;
            e_inf  = !in_bnd && (cy < T_INF_SETTLE + T_INF_ADC);
            e_tree = !in_bnd && (cy == INF_TREE_START_CYC);
            e_dsa  = !in_bnd && (step == 0) && (cy == 0);
            e_col  = (in_bnd && bnd >= T_COL_ACT + T_COL_WB && col < NUM_COL_POS - 1) ? col + 1 : col;
            exp_lanes = exp_plane(run, e_bit_sel);

            if ((col == 0 && (w < 2 * T_INF_STEP || in_bnd)) || c >= T_INF_COMPUTE - 4)
                $fdisplay(fd, "  cyc%0d col=%0d %s%0d state=%s col_sel=%0d bit_sel=%0d pulses=%b tree_start=%0d dsa_clear=%0d compute_done=%0d",
                          c, col, in_bnd ? "bnd" : "step", in_bnd ? bnd : step, dut.state.name(),
                          col_sel, buf_bit_sel, pulses, tree_start, dsa_clear, compute_done);
            check(dut.state == S_COMPUTE,
                  $sformatf("compute cyc %0d: state=%s expected S_COMPUTE", c, dut.state.name()));
            check(buf_reg_ctrl == CTRL_COMPUTE && !buf_read_write,
                  $sformatf("compute cyc %0d: buffer ctrl=%0d rw=%0d", c, buf_reg_ctrl, buf_read_write));
            check(col_sel == COL_SEL_WIDTH'(e_col),
                  $sformatf("compute cyc %0d: col_sel=%0d expected %0d", c, col_sel, e_col));
            check(buf_bit_sel == BUF_BIT_SEL_WIDTH'(e_bit_sel),
                  $sformatf("compute cyc %0d: bit_sel=%0d expected %0d", c, buf_bit_sel, e_bit_sel));
            check(pulses == (e_inf ? PULSE_MODE_INF : PULSE_MODE_HIZ),
                  $sformatf("compute cyc %0d: pulses=%b expected %s", c, pulses, e_inf ? "INF" : "HIZ"));
            check(tree_start === e_tree && dsa_clear === e_dsa,
                  $sformatf("compute cyc %0d: tree_start=%0d/%0d dsa_clear=%0d/%0d (got/expected)",
                            c, tree_start, e_tree, dsa_clear, e_dsa));
            check(data_lanes === exp_lanes,
                  $sformatf("compute cyc %0d: data_lanes mismatch (plane %0d)", c, e_bit_sel));
            check(compute_done === (c == T_INF_COMPUTE - 1),
                  $sformatf("compute cyc %0d: compute_done=%0d", c, compute_done));
            check(!inf_start, $sformatf("compute cyc %0d: inf_start asserted", c));
            check(busy, $sformatf("compute cyc %0d: busy low", c));
        end

        // ---- RESULT (1 cycle) ----
        @(negedge clk);
        #1;
        $fdisplay(fd, "  after compute: state=%s buf_ctrl=%0d compute_done=%0d lanes=%064h",
                  dut.state.name(), buf_reg_ctrl, compute_done, data_lanes);
        check(dut.state == S_RESULT, $sformatf("expected S_RESULT after compute, got %s", dut.state.name()));
        check(buf_reg_ctrl == CTRL_RESULT_OUT, "S_RESULT: buf_reg_ctrl != CTRL_RESULT_OUT");
        check(data_lanes == '0, "S_RESULT: data_lanes not gated to 0");
        check(!compute_done && !dsa_clear && !tree_start, "S_RESULT: compute_done/dsa_clear/tree_start asserted");
        check(col_sel == '0, "S_RESULT: col_sel back to 0");

        // ---- back to IDLE ----
        @(negedge clk);
        #1;
        check(dut.state == S_IDLE, $sformatf("expected S_IDLE after S_RESULT, got %s", dut.state.name()));
        check(!busy, "S_IDLE: busy still high");
        check(data_lanes == '0, "S_IDLE: data_lanes not gated to 0");

        // A couple of idle cycles so the monitor sees the end of the window
        repeat (2) @(negedge clk);

        // ---- Strobe timing summary ----
        $fdisplay(fd, "Strobe timing: inf_start@%0d (x%0d) first_compute@%0d compute_cycles=%0d compute_done@%0d (x%0d) dsa_clear x%0d tree_start x%0d",
                  inf_start_cyc, inf_start_count, first_compute_cyc, compute_cycle_count,
                  compute_done_cyc, compute_done_count, dsa_clear_count, tree_start_count);
        check(inf_start_count == 1, $sformatf("inf_start asserted %0d times (expected 1)", inf_start_count));
        check(dsa_clear_count == NUM_COL_POS,
              $sformatf("dsa_clear asserted %0d times (expected %0d, one per column)", dsa_clear_count, NUM_COL_POS));
        check(tree_start_count == NUM_COL_POS * ACT_WIDTH,
              $sformatf("tree_start asserted %0d times (expected %0d)", tree_start_count, NUM_COL_POS * ACT_WIDTH));
        check(compute_done_count == 1, $sformatf("compute_done asserted %0d times (expected 1)", compute_done_count));
        check(first_compute_cyc == inf_start_cyc + 1,
              "COMPUTE did not start the cycle right after inf_start");
        check(compute_cycle_count == T_INF_COMPUTE,
              $sformatf("COMPUTE lasted %0d cycles (expected %0d)", compute_cycle_count, T_INF_COMPUTE));
        check(compute_done_cyc == first_compute_cyc + T_INF_COMPUTE - 1,
              "compute_done not in the last COMPUTE cycle");

        if (fail_count == fails_before)
            $fdisplay(fd, "PASS: INF run %0d (256 pixels stored, 16 columns x 8 planes MSB first on data_lanes, strobe timing)", run);
        else
            $fdisplay(fd, "FAIL: INF run %0d had %0d failing checks", run, fail_count - fails_before);
    endtask

    initial begin
        rst_n = 0; reset = 1;
        host_data = 0; host_cmd = CMD_HIZ;
        pass_count = 0; fail_count = 0;
        cyc = 0;
        in_compute_d = 0;
        clear_monitor();

        fd = $fopen(LOG_FILE, "w");
        if (!fd) begin
            $error("Cannot open log file: %s", LOG_FILE);
            $finish;
        end

        $fdisplay(fd, "// Controller INF buffer-flow behavior");
        $fdisplay(fd, "// INF start beat -> S_COLLECT_DATA; %0d pixel beats -> entries 0..%0d; COMPUTE %0d cycles",
                  INPUT_VECTOR_LEN, INPUT_VECTOR_LEN - 1, T_INF_COMPUTE);
        $fdisplay(fd, "// = %0d column positions x (%0d bit-steps x %0d + boundary %0d), planes MSB first (7..0)",
                  NUM_COL_POS, ACT_WIDTH, T_INF_STEP, T_COL_BND);
        $fdisplay(fd, "// inf_start: cycle of the last accepted pixel; dsa_clear: first cycle of each column; compute_done: last COMPUTE cycle.");
        $fdisplay(fd, "");

        repeat(5) @(posedge clk);
        @(negedge clk);
        rst_n = 1; reset = 0;
        repeat(2) @(posedge clk);

        for (int run = 0; run < NUM_RUNS; run++)
            run_inference(run);

        // Final summary
        $fdisplay(fd, "");
        $fdisplay(fd, "//==============================================================================");
        $fdisplay(fd, "// TB REPORT  controller_inf_buffer_flow_tb");
        $fdisplay(fd, "// %0d INF runs: %0d pixels -> buffer entries, %0d columns x %0d planes (MSB first) on %0d data_lanes, strobe timing",
                  NUM_RUNS, INPUT_VECTOR_LEN, NUM_COL_POS, ACT_WIDTH, NUM_DATA_LINES);
        $fdisplay(fd, "//==============================================================================");
        $fdisplay(fd, "SUMMARY: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0)
            $fdisplay(fd, "RESULT: PASS");
        else
            $fdisplay(fd, "RESULT: FAIL");

        $fclose(fd);
        $finish;
    end

endmodule
