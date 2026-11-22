//------------------------------------------------------------------------------
// Host READ reorder check: program 8 weights at 8 addresses, then CMD_READ in
// permuted order. Report: target/Controller/read/controller_host_read_reorder_report.txt
//------------------------------------------------------------------------------

`timescale 1ns/1ps

`include "rtl/common/macros.svh"

import ann_arch_pkg::*;
import controller_pkg::*;
import input_buffer_pkg::*;
import parallel_interface_pkg::*;

module controller_host_read_reorder_tb;

    localparam int N_WEIGHTS = 8;
    localparam int CLK_PERIOD = 5;
    localparam string REPORT_FILE = "target/Controller/read/controller_host_read_reorder_report.txt";

    logic        clk, rst_n, reset;
    logic [31:0] host_data;
    logic [2:0]  host_cmd;
    logic        valid;
    logic [7:0]  pi_data;
    logic [WEIGHT_ADDR_WIDTH-1:0] address;
    logic [CMD_WIDTH-1:0] cmd;
    logic        ann_reset, op_done, busy;
    logic [31:0] ann_address;
    logic [2:0]  pulses;
    logic [BUFFER_ADDR_WIDTH-1:0] buf_reg_add;
    logic [2:0]  buf_reg_ctrl;
    logic        buf_read_write;
    logic [2:0]  buf_bit_sel;
    logic [7:0]  buf_data_out, buf_data;
    logic [NUM_DATA_LINES-1:0] data_lanes;
    logic        buf_ready;
    logic        dsa_clear, compute_done;  // unused here (INF datapath strobes)

    // Eight distinct ANN addresses (unique decoded cells)
    // Packed layout {PE[15:14], SA[13:10], ROW[9:6], COL[5:0]} (ann_arch_pkg)
    logic [WEIGHT_ADDR_WIDTH-1:0] prog_addr [0:N_WEIGHTS-1];
    logic [7:0]  prog_data [0:N_WEIGHTS-1]; // use [3:0] as weight nibble
    // Read sequence: visit program indices in this order (not 0..7)
    int          read_perm [0:N_WEIGHTS-1];

    logic [3:0] ann_weight_matrix [0:NUM_PE-1][0:NUM_SA-1][0:SA_ROWS-1][0:SA_COLS-1];
    logic [3:0] weight_read_data_mock;
    logic [3:0] actual_from_ann;
    logic [PE_ID_WIDTH-1:0]  dec_blk;
    logic [SA_ID_WIDTH-1:0]  dec_sb;
    logic [ROW_ID_WIDTH-1:0] dec_row;
    logic [COL_ID_WIDTH-1:0] dec_col;

    always_comb begin
        ann_address_decode(ann_address, dec_blk, dec_sb, dec_row, dec_col);
        actual_from_ann = ann_weight_matrix[dec_blk][dec_sb][dec_row][dec_col];
    end

    assign weight_read_data_mock = actual_from_ann;

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
        .inf_start(), .dsa_clear(dsa_clear), .tree_start(), .col_sel(), .compute_done(compute_done), .prog_fail(), .op_timeout(), .read_data(), .read_done(), .busy(busy)
    );

    input_buffer u_buf (
        .clk(clk), .rst_n(rst_n), .data_in(buf_data_out),
        .ready(buf_ready), .reg_ctrl(buf_reg_ctrl), .buf_read_write(buf_read_write),
        .buf_reg_add(buf_reg_add), .bit_sel(buf_bit_sel), .buf_data(buf_data),
        .data_lanes(data_lanes)
    );

    logic in_prog_core_phase;
    assign in_prog_core_phase = (pulses == PULSE_MODE_PROG) && (buf_reg_ctrl == CTRL_WEIGHT_READ);

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
            end else if (busy && pulses == PULSE_MODE_READ) begin
                if (op_done_cnt >= PULSE_TOTAL_READ - 1) begin
                    op_done <= 1;
                    op_done_cnt <= 0;
                end else begin
                    op_done <= 0;
                    op_done_cnt <= op_done_cnt + 1;
                end
            end else if (busy && (pulses == PULSE_MODE_ERASE || pulses == PULSE_MODE_INF)) begin
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

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int b = 0; b < NUM_PE; b++)
                for (int sb = 0; sb < NUM_SA; sb++)
                    for (int r = 0; r < SA_ROWS; r++)
                        for (int c = 0; c < SA_COLS; c++)
                            ann_weight_matrix[b][sb][r][c] <= 4'b0;
        end else if (in_prog_core_phase) begin
            automatic logic [PE_ID_WIDTH-1:0]  lb;
            automatic logic [SA_ID_WIDTH-1:0]  lsb;
            automatic logic [ROW_ID_WIDTH-1:0] lr;
            automatic logic [COL_ID_WIDTH-1:0] lc;
            ann_address_decode(ann_address, lb, lsb, lr, lc);
            ann_weight_matrix[lb][lsb][lr][lc] <= ann_address[27:24];
        end
    end

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    initial begin
        rst_n = 0; reset = 1; host_data = 0; host_cmd = CMD_HIZ;
        repeat(5) @(posedge clk);
        rst_n = 1; reset = 0;
        repeat(4) @(posedge clk);
    end

    task automatic wait_idle(int maxcyc);
        int n;
        n = 0;
        while (busy && n < maxcyc) begin
            @(posedge clk);
            n++;
        end
        if (busy)
            $fatal(1, "wait_idle timeout");
    endtask

    task automatic pulse_host_ann(input logic [2:0] c, input logic [7:0] d, input logic [WEIGHT_ADDR_WIDTH-1:0] a);
        host_data = build_host_ann_word(d, a);
        host_cmd  = c;
        @(posedge clk);
        host_data = 0;
        host_cmd  = CMD_HIZ;
        @(posedge clk);
    endtask

    function automatic string ann_word_bin32(logic [31:0] w);
        return $sformatf("%08b-%02b-%04b-%04b-%06b-%08b",
            w[31:24], w[23:22], w[21:18], w[17:14], w[13:8], w[7:0]);
    endfunction

    int fd;
    int pass_c, fail_c;

    task automatic log_one_read(int step_num, int prog_ix, logic [WEIGHT_ADDR_WIDTH-1:0] addr, logic [3:0] exp_w);
        logic [31:0] host_pkt;
        logic [31:0] saw_word;
        logic [3:0]  saw_adc;
        bit          saw_read_state;
        int          to;
        host_pkt = build_host_ann_word(8'h00, addr);
        $fdisplay(fd, "=== READ step %0d (program order index %0d) ===", step_num, prog_ix);
        $fdisplay(fd, "// Host command line: CMD_READ  data_byte=0x%02X  ann_addr=0x%04X",
                  8'h00, addr);
        $fdisplay(fd, "// Host 32b packet (build_host_ann_word): 0x%08h", host_pkt);
        $fdisplay(fd, "// Expected mock memristor weight at this cell: %0d (4b)", exp_w);

        pulse_host_ann(CMD_READ, 8'h00, addr);
        saw_word = '0;
        saw_adc  = '0;
        saw_read_state = 0;
        to = 0;
        begin
            bit logged_s_read_line;
            logged_s_read_line = 0;
            while (!busy && to < 10) begin
                @(posedge clk);
                to++;
            end
            to = 0;
            while (busy) begin
                if (dut.state == S_READ) begin
                    saw_read_state = 1;
                    saw_word = ann_address;
                    saw_adc  = weight_read_data_mock;
                    if (!logged_s_read_line) begin
                        logged_s_read_line = 1;
                        $fdisplay(fd, "[%0t] DUT in S_READ: pulses=%03b ann_address=0x%08h (0b%s) weight_read_data(mock)=%0d",
                                  $time, pulses, ann_address, ann_word_bin32(ann_address), weight_read_data_mock);
                    end
                end
                @(posedge clk);
                to++;
                if (to > 500_000) break;
            end
        end
        if (!saw_read_state) begin
            fail_c++;
            $fdisplay(fd, "FAIL: never entered S_READ");
        end else if (saw_adc !== exp_w) begin
            fail_c++;
            $fdisplay(fd, "FAIL: weight_read_data %0d != expected programmed %0d", saw_adc, exp_w);
        end else if (saw_word !== host_pkt) begin
            fail_c++;
            $fdisplay(fd, "FAIL: ann_address 0x%08h != expected packed host 0x%08h", saw_word, host_pkt);
        end else if (saw_word !== {8'h00, addr, 8'h00}) begin
            // Independent of package helpers: {data[31:24], addr[23:8], rsvd[7:0]=0}
            fail_c++;
            $fdisplay(fd, "FAIL: ann_address 0x%08h != {data,addr16,8'h00} 0x%08h", saw_word, {8'h00, addr, 8'h00});
        end else begin
            pass_c++;
            $fdisplay(fd, "PASS: READ returned consistent ann_address and mock ADC data");
        end
        $fdisplay(fd, "");
    endtask

    initial begin
        pass_c = 0;
        fail_c = 0;

        // Distinct addresses and weights
        //                       pack_ann_addr(PE, SA, ROW, COL)
        prog_addr[0] = pack_ann_addr(2'd0, 4'd0,  4'd0,  6'd0);  prog_data[0] = {4'h0, 4'd1};  // 0x0000
        prog_addr[1] = pack_ann_addr(2'd3, 4'd15, 4'd15, 6'd63); prog_data[1] = {4'h0, 4'd2};  // 0xFFFF
        prog_addr[2] = pack_ann_addr(2'd1, 4'd2,  4'd3,  6'd4);  prog_data[2] = {4'h0, 4'd3};  // 0x48C4
        prog_addr[3] = pack_ann_addr(2'd2, 4'd7,  4'd8,  6'd32); prog_data[3] = {4'h0, 4'd4};  // 0x9E20
        prog_addr[4] = pack_ann_addr(2'd0, 4'd15, 4'd0,  6'd63); prog_data[4] = {4'h0, 4'd5};  // 0x3C3F
        prog_addr[5] = pack_ann_addr(2'd3, 4'd0,  4'd15, 6'd0);  prog_data[5] = {4'h0, 4'd6};  // 0xC3C0
        prog_addr[6] = pack_ann_addr(2'd0, 4'd1,  4'd7,  6'd63); prog_data[6] = {4'h0, 4'd7};  // 0x05FF
        prog_addr[7] = pack_ann_addr(2'd1, 4'd9,  4'd12, 6'd21); prog_data[7] = {4'h0, 4'd8};  // 0x6715

        // Not sequential: e.g. 4,0,7,2,1,6,3,5
        read_perm[0] = 4;
        read_perm[1] = 0;
        read_perm[2] = 7;
        read_perm[3] = 2;
        read_perm[4] = 1;
        read_perm[5] = 6;
        read_perm[6] = 3;
        read_perm[7] = 5;

        fd = $fopen(REPORT_FILE, "w");
        if (!fd) begin $error("Cannot open %s", REPORT_FILE); $finish; end

        $fdisplay(fd, "//==============================================================================");
        $fdisplay(fd, "// controller_host_read_reorder_tb");
        $fdisplay(fd, "// Program %0d weights, then CMD_READ in non-sequential order (by program index).", N_WEIGHTS);
        $fdisplay(fd, "// ann_address fields: data[31:24]-PE[23:22]-SA[21:18]-ROW[17:14]-COL[13:8]-rsvd[7:0] (binary in samples)");
        $fdisplay(fd, "// Packed address (16b): {PE[15:14], SA[13:10], ROW[9:6], COL[5:0]}");
        $fdisplay(fd, "// Pulse: READ=001. Mock ADC = ann_weight_matrix cell selected by ann_address decode.");
        $fdisplay(fd, "//==============================================================================");
        $fdisplay(fd, "");

        wait(rst_n);

        $fdisplay(fd, "//==============================================================================");
        $fdisplay(fd, "// PHASE 1: Programmed weights (CMD_PROG via host, PROG+VERIFY in DUT)");
        $fdisplay(fd, "//==============================================================================");
        $fdisplay(fd, "// ix | addr   | pe  | sa  | row | col | weight | host_prog_packet (hex)");
        $fdisplay(fd, "//----+--------+-----+-----+-----+-----+--------+----------------------------");
        for (int i = 0; i < N_WEIGHTS; i++) begin
            automatic logic [PE_ID_WIDTH-1:0]  pb;
            automatic logic [SA_ID_WIDTH-1:0]  psb;
            automatic logic [ROW_ID_WIDTH-1:0] pr;
            automatic logic [COL_ID_WIDTH-1:0] pc;
            logic [31:0] pkt;
            parse_ann_address(prog_addr[i], pb, psb, pr, pc);
            pkt = build_host_ann_word(prog_data[i], prog_addr[i]);
            $fdisplay(fd, "// %0d  | 0x%04h | %0d   | %0d   | %0d   | %0d   | %0d      | 0x%08h",
                      i, prog_addr[i], pb, psb, pr, pc, prog_data[i][3:0], pkt);
        end
        $fdisplay(fd, "");

        for (int i = 0; i < N_WEIGHTS; i++) begin
            pulse_host_ann(CMD_PROG, prog_data[i], prog_addr[i]);
            wait_idle(500_000);
        end

        $fdisplay(fd, "// After programming — mock matrix at programmed cells:");
        for (int i = 0; i < N_WEIGHTS; i++) begin
            automatic logic [PE_ID_WIDTH-1:0]  pb;
            automatic logic [SA_ID_WIDTH-1:0]  psb;
            automatic logic [ROW_ID_WIDTH-1:0] pr;
            automatic logic [COL_ID_WIDTH-1:0] pc;
            parse_ann_address(prog_addr[i], pb, psb, pr, pc);
            $fdisplay(fd, "//   ix%0d addr=0x%04h -> weight=%0d (expect %0d)",
                      i, prog_addr[i],
                      ann_weight_matrix[pb][psb][pr][pc], prog_data[i][3:0]);
            if (ann_weight_matrix[pb][psb][pr][pc] !== prog_data[i][3:0]) begin
                $fclose(fd);
                $fatal(1, "programming mismatch at ix %0d", i);
            end
        end
        $fdisplay(fd, "");

        $fdisplay(fd, "//==============================================================================");
        $fdisplay(fd, "// PHASE 2: Host READ requests (order permuted)");
        $fdisplay(fd, "//==============================================================================");
        $fdisplay(fd, "// read_step uses program indices in order: %0d %0d %0d %0d %0d %0d %0d %0d",
                  read_perm[0], read_perm[1], read_perm[2], read_perm[3],
                  read_perm[4], read_perm[5], read_perm[6], read_perm[7]);
        $fdisplay(fd, "");

        for (int s = 0; s < N_WEIGHTS; s++) begin
            int ix;
            ix = read_perm[s];
            log_one_read(s + 1, ix, prog_addr[ix], prog_data[ix][3:0]);
            wait_idle(500_000);
        end

        $fdisplay(fd, "// Summary: PASS=%0d FAIL=%0d", pass_c, fail_c);
        $fclose(fd);
        $display("[%0t] controller_host_read_reorder_tb: PASS=%0d FAIL=%0d -> %s",
                 $time, pass_c, fail_c, REPORT_FILE);
        if (fail_c != 0)
            $fatal(1, "controller_host_read_reorder_tb failures");
        $finish;
    end

endmodule
