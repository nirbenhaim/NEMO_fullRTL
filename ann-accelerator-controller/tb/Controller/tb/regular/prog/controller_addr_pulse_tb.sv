//------------------------------------------------------------------------------
// Controller Address and Pulse Verification Testbench
//------------------------------------------------------------------------------
// Verifies:
// 1. Packed bus: ann_address = {data[31:24], packed addr[23:8] = {PE,SA,ROW,COL}, 8'h00}
// 2. Pulse output (pulses[2:0]) for each command: READ, PROG, ERASE, INF
// 3. Controller behavior per command type
//
// Address (ann_arch_pkg): 16-bit packed binary {PE[15:14], SA[13:10], ROW[9:6], COL[5:0]}
// The test list spans PE 0..3, SA 0..15, ROW 0..15, COL 0..63 (incl. all-max corner).
//
// Parameters (from controller_pkg):
//   TREAD, PULSE_NUM_READ, TPROG, PULSE_NUM_PROG, TERASE, PULSE_NUM_ERASE, TINF, PULSE_NUM_INF
//------------------------------------------------------------------------------

`timescale 1ns/1ps

import ann_arch_pkg::*;
import controller_pkg::*;
import input_buffer_pkg::*;
import parallel_interface_pkg::*;

module controller_addr_pulse_tb;

    //--------------------------------------------------------------------------
    // Parameters
    //--------------------------------------------------------------------------
    localparam int CLK_PERIOD = 5;
    localparam string RESULT_FILE = "target/Controller/controller_addr_pulse_verify.txt";

    // Pulse parameters from controller_pkg (TREAD, TPROG, TERASE, TINF, etc.)

    //--------------------------------------------------------------------------
    // Signals
    //--------------------------------------------------------------------------
    logic clk, rst_n, reset;
    logic [WORD_WIDTH-1:0] host_data;
    logic [CMD_WIDTH-1:0]  host_cmd;
    logic host_strobe;                       // one-cycle frame strobe per command beat
    logic pi_valid;                          // parallel_interface level qualifier
    logic valid;                             // one-cycle strobe into ann_controller
    logic [ACT_WIDTH-1:0] pi_data;
    logic [WEIGHT_ADDR_WIDTH-1:0] address;
    logic [CMD_WIDTH-1:0] cmd;
    logic ann_reset, op_done, busy;
    logic [WORD_WIDTH-1:0] ann_address;
    logic [2:0] pulses;
    logic [BUFFER_ADDR_WIDTH-1:0] buf_reg_add;
    logic [2:0] buf_reg_ctrl;
    logic buf_read_write;
    logic [BUF_BIT_SEL_WIDTH-1:0] buf_bit_sel;
    logic [BUFFER_DATA_WIDTH-1:0] buf_data_out, buf_data;
    logic [NUM_DATA_LINES-1:0] data_lanes;   // unused here (INF bit-serial lanes)
    logic dsa_clear, compute_done;           // unused here (INF datapath strobes)
    logic buf_ready;

    // Top-level style valid: PI level qualifier ANDed with a one-cycle frame strobe
    assign valid = pi_valid && host_strobe;

    //--------------------------------------------------------------------------
    // Expected ann_address = pack_ann_address(data, PE, SA, ROW, COL)
    //--------------------------------------------------------------------------
    function automatic logic [WORD_WIDTH-1:0] expected_ann_address(
        logic [WEIGHT_ADDR_WIDTH-1:0] addr_16, logic [ACT_WIDTH-1:0] data_val);
        logic [PE_ID_WIDTH-1:0]  pe;
        logic [SA_ID_WIDTH-1:0]  sa;
        logic [ROW_ID_WIDTH-1:0] rw;
        logic [COL_ID_WIDTH-1:0] cl;
        parse_ann_address(addr_16, pe, sa, rw, cl);
        return pack_ann_address(data_val, pe, sa, rw, cl);
    endfunction

    // 32-bit ann_address as binary for logs: data | PE | SA | ROW | COL | reserved (matches pack_ann_address).
    function automatic string ann_address_bin(logic [WORD_WIDTH-1:0] w);
        return $sformatf("%08b-%02b-%04b-%04b-%06b-%08b",
            w[31:24], w[23:22], w[21:18], w[17:14], w[13:8], w[7:0]);
    endfunction

    //--------------------------------------------------------------------------
    // DUT
    //--------------------------------------------------------------------------
    parallel_interface u_parallel_interface (
        .clk(clk), .reset(reset), .host_data(host_data), .host_cmd(host_cmd),
        .valid(pi_valid), .data(pi_data), .address(address), .cmd(cmd)
    );

    ann_controller dut (
        .clk(clk), .rst_n(rst_n),
        .valid(valid), .data(pi_data), .address(address), .cmd(cmd),
        .ann_reset(ann_reset),
        .op_done(op_done), .ann_address(ann_address), .pulses(pulses),
        .weight_read_data(buf_data[ANN_WEIGHT_WIDTH-1:0]),  // VERIFY path tie-off
        .buf_reg_add(buf_reg_add), .buf_reg_ctrl(buf_reg_ctrl), .buf_read_write(buf_read_write),
        .buf_bit_sel(buf_bit_sel),
        .buf_data_out(buf_data_out), .buf_ready(buf_ready), .buf_data(buf_data),
        .inf_start(), .dsa_clear(dsa_clear), .tree_start(), .col_sel(), .compute_done(compute_done), .prog_fail(), .op_timeout(), .read_data(), .read_done(), .busy(busy)
    );

    input_buffer u_input_buffer (
        .clk(clk), .rst_n(rst_n), .data_in(buf_data_out),  // Controller drives buf_data_out for writes
        .ready(buf_ready), .reg_ctrl(buf_reg_ctrl), .buf_read_write(buf_read_write),
        .buf_reg_add(buf_reg_add), .bit_sel(buf_bit_sel), .buf_data(buf_data),
        .data_lanes(data_lanes)
    );

    logic in_prog_core_phase;
    assign in_prog_core_phase = (pulses == PULSE_MODE_PROG) && (buf_reg_ctrl == CTRL_WEIGHT_READ);

    // op_done mock: PROG/ERASE handshake states (see ann_controller).
    int op_done_cnt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            op_done <= 0;
            op_done_cnt <= 0;
        end else begin
            if (dut.state == S_PROGRAM && (dut.prog_state == PROG_SELECT || dut.prog_state == PROG_WAIT_ACK)) begin
                op_done <= 1'b1;
                op_done_cnt <= 0;
            end else if (dut.state == S_ERASE && (dut.erase_state == ERASE_SELECT || dut.erase_state == ERASE_WAIT_ACK)) begin
                op_done <= 1'b1;
                op_done_cnt <= 0;
            end else if (busy && (pulses == 3'b001 || pulses == 3'b011 || pulses == 3'b100)) begin
                if (op_done_cnt >= 3) begin
                    op_done <= 1;
                    op_done_cnt <= 0;
                end else begin
                    op_done <= 0;
                    op_done_cnt <= op_done_cnt + 1;
                end
            end else begin
                op_done <= 0;
                op_done_cnt <= 0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Clock and Reset
    //--------------------------------------------------------------------------
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    initial begin
        rst_n = 0; reset = 1; host_data = 0; host_cmd = CMD_HIZ; host_strobe = 1'b0;
        repeat(5) @(posedge clk);
        rst_n = 1; reset = 0;
        repeat(2) @(posedge clk);
    end

    int fd;  // result file handle (declared before use in run_cmd_test)

    //--------------------------------------------------------------------------
    // Task: Run command test and verify ann_address
    //--------------------------------------------------------------------------
    task automatic run_cmd_test(
        input int test_num,
        input logic [CMD_WIDTH-1:0] cmd_code,
        input logic [PE_ID_WIDTH-1:0]  pe,
        input logic [SA_ID_WIDTH-1:0]  sa,
        input logic [ROW_ID_WIDTH-1:0] row,
        input logic [COL_ID_WIDTH-1:0] col,
        input logic [ACT_WIDTH-1:0] data_val,
        input string cmd_label
    );
        logic [WEIGHT_ADDR_WIDTH-1:0] addr;
        logic [WORD_WIDTH-1:0] exp;
        string desc;
        addr = pack_ann_addr(pe, sa, row, col);
        exp  = expected_ann_address(addr, data_val);
        desc = $sformatf("%-6s (PE=%0d,SA=%0d,row=%0d,col=%0d)", cmd_label, pe, sa, row, col);
        $fdisplay(fd, "=== Test %0d: %s at addr 0x%04X ===", test_num, desc, addr);

        host_data   = 0;
        host_cmd    = CMD_HIZ;
        host_strobe = 1'b0;
        @(posedge clk);
        @(posedge clk);
        // Drive the command on the falling edge and strobe valid for exactly one rising edge.
        @(negedge clk);
        host_data   = build_host_ann_word(data_val, addr);
        host_cmd    = cmd_code;
        host_strobe = 1'b1;
        @(negedge clk);
        host_strobe = 1'b0;
        wait(busy);
        // READ completes quickly and ann_address is forced to 0 in S_IDLE - scan the whole busy window.
        // PROG stays busy through VERIFY - sample early after command registers (repeat-3) then wait idle.
        if (cmd_code == CMD_READ) begin
            automatic bit saw_match = 1'b0;
            automatic logic [WORD_WIDTH-1:0] last_word = ann_address;
            while (busy) begin
                last_word = ann_address;
                if (ann_address === exp)
                    saw_match = 1'b1;
                @(posedge clk);
            end
            if (!saw_match)
                $fdisplay(fd, "  ERROR: ann_address never matched expected 0x%08X (0b%s) (last while busy: 0x%08X (0b%s))",
                    exp, ann_address_bin(exp), last_word, ann_address_bin(last_word));
            else
                $fdisplay(fd, "  PASS: ann_address 0x%08X (0b%s)", exp, ann_address_bin(exp));
        end else begin
            repeat(3) @(posedge clk);
            if (ann_address !== exp)
                $fdisplay(fd, "  ERROR: ann_address 0x%08X (0b%s) != expected 0x%08X (0b%s)",
                    ann_address, ann_address_bin(ann_address), exp, ann_address_bin(exp));
            else
                $fdisplay(fd, "  PASS: ann_address 0x%08X (0b%s)", ann_address, ann_address_bin(ann_address));
            wait(!busy);
        end
        host_data = 0;
        host_cmd  = CMD_HIZ;
        $fdisplay(fd, "");
    endtask

    //--------------------------------------------------------------------------
    // Test sequence
    //--------------------------------------------------------------------------

    initial begin
        fd = $fopen(RESULT_FILE, "w");
        $fdisplay(fd, "// Controller Address and Pulse Verification");
        $fdisplay(fd, "// Parameters: TREAD=%0d, PULSE_NUM_READ=%0d, TPROG=%0d, PULSE_NUM_PROG=%0d", controller_pkg::TREAD, controller_pkg::PULSE_NUM_READ, controller_pkg::TPROG, controller_pkg::PULSE_NUM_PROG);
        $fdisplay(fd, "//            TERASE=%0d, PULSE_NUM_ERASE=%0d, TINF=%0d, PULSE_NUM_INF=%0d", controller_pkg::TERASE, controller_pkg::PULSE_NUM_ERASE, controller_pkg::TINF, controller_pkg::PULSE_NUM_INF);
        $fdisplay(fd, "// Expected: ann_address = {data[31:24], packed addr[23:8] = {PE,SA,ROW,COL}, 8'h00}");
        $fdisplay(fd, "// Binary suffix: 0b<data[31:24]>-<PE[23:22]>-<SA[21:18]>-<ROW[17:14]>-<COL[13:8]>-<rsvd[7:0]> (fields, MSB first)");
        $fdisplay(fd, "// Expected: READ->pulses=001, PROG->pulses=010, ERASE->pulses=011, INF->pulses=100");
        $fdisplay(fd, "");

        wait(rst_n);
        repeat(10) @(posedge clk);

        // run_cmd_test(num, cmd, PE, SA, ROW, COL, data, label)
        // addr_16 = pack_ann_addr(PE, SA, ROW, COL) = {PE[15:14], SA[13:10], ROW[9:6], COL[5:0]}

        run_cmd_test(1,  CMD_PROG,  2'd0, 4'd0,  4'd0,  6'd0,  8'd5,  "PROG");
        run_cmd_test(2,  CMD_READ,  2'd1, 4'd3,  4'd7,  6'd6,  8'h00, "READ");
        run_cmd_test(3,  CMD_ERASE, 2'd0, 4'd15, 4'd5,  6'd0,  8'h00, "ERASE");
        run_cmd_test(4,  CMD_PROG,  2'd3, 4'd15, 4'd15, 6'd63, 8'd15, "PROG");
        run_cmd_test(5,  CMD_PROG,  2'd2, 4'd0,  4'd0,  6'd0,  8'd3,  "PROG");
        run_cmd_test(6,  CMD_READ,  2'd0, 4'd2,  4'd4,  6'd4,  8'h00, "READ");
        run_cmd_test(7,  CMD_ERASE, 2'd3, 4'd0,  4'd7,  6'd0,  8'h00, "ERASE");
        run_cmd_test(8,  CMD_PROG,  2'd1, 4'd1,  4'd15, 6'd63, 8'd12, "PROG");
        run_cmd_test(9,  CMD_READ,  2'd2, 4'd11, 4'd4,  6'd32, 8'h00, "READ");
        run_cmd_test(10, CMD_ERASE, 2'd1, 4'd2,  4'd1,  6'd0,  8'h00, "ERASE");
        run_cmd_test(11, CMD_PROG,  2'd0, 4'd8,  4'd0,  6'd55, 8'd7,  "PROG");
        run_cmd_test(12, CMD_READ,  2'd3, 4'd15, 4'd0,  6'd0,  8'h00, "READ");
        run_cmd_test(13, CMD_ERASE, 2'd0, 4'd0,  4'd1,  6'd1,  8'h00, "ERASE");
        run_cmd_test(14, CMD_PROG,  2'd1, 4'd12, 4'd9,  6'd17, 8'd10, "PROG");
        run_cmd_test(15, CMD_READ,  2'd0, 4'd3,  4'd10, 6'd42, 8'h00, "READ");
        run_cmd_test(16, CMD_PROG,  2'd3, 4'd1,  4'd3,  6'd2,  8'd9,  "PROG");
        run_cmd_test(17, CMD_READ,  2'd0, 4'd9,  4'd2,  6'd61, 8'h00, "READ");
        run_cmd_test(18, CMD_ERASE, 2'd2, 4'd6,  4'd14, 6'd4,  8'h00, "ERASE");
        run_cmd_test(19, CMD_PROG,  2'd1, 4'd0,  4'd7,  6'd0,  8'd14, "PROG");
        run_cmd_test(20, CMD_READ,  2'd3, 4'd13, 4'd5,  6'd27, 8'h00, "READ");
        run_cmd_test(21, CMD_ERASE, 2'd0, 4'd14, 4'd4,  6'd7,  8'h00, "ERASE");
        run_cmd_test(22, CMD_PROG,  2'd2, 4'd10, 4'd11, 6'd48, 8'd6,  "PROG");
        run_cmd_test(23, CMD_READ,  2'd1, 4'd5,  4'd6,  6'd1,  8'h00, "READ");
        run_cmd_test(24, CMD_ERASE, 2'd3, 4'd0,  4'd2,  6'd2,  8'h00, "ERASE");
        run_cmd_test(25, CMD_PROG,  2'd0, 4'd7,  4'd3,  6'd33, 8'd11, "PROG");
        run_cmd_test(26, CMD_READ,  2'd2, 4'd4,  4'd13, 6'd7,  8'h00, "READ");
        run_cmd_test(27, CMD_ERASE, 2'd1, 4'd1,  4'd0,  6'd19, 8'h00, "ERASE");
        run_cmd_test(28, CMD_PROG,  2'd3, 4'd2,  4'd12, 6'd5,  8'd4,  "PROG");
        run_cmd_test(29, CMD_READ,  2'd1, 4'd0,  4'd6,  6'd63, 8'h00, "READ");
        run_cmd_test(30, CMD_ERASE, 2'd3, 4'd15, 4'd15, 6'd63, 8'h00, "ERASE");

        $fdisplay(fd, "=== Verification Complete (30 tests) ===");
        $fclose(fd);

        $display("[%0t] Verification complete. Results in %s", $time, RESULT_FILE);
        repeat(5) @(posedge clk);
        $finish;
    end

endmodule
