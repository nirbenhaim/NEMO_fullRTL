//------------------------------------------------------------------------------
// PI + ann_controller + input_buffer integration.
// Log: target/Controller/tb_pi_controller_integration.txt
//------------------------------------------------------------------------------

`timescale 1ns/1ps

`include "../../../rtl/common/macros.svh"

import ann_arch_pkg::*;
import controller_pkg::*;
import input_buffer_pkg::*;
import parallel_interface_pkg::*;

module parallel_interface_controller_integration_tb;

    localparam int CLK_PERIOD = 5;
    localparam string LOG_FILE = "target/Controller/tb_pi_controller_integration.txt";

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
    logic        dsa_clear, compute_done;  // INF datapath strobes (not checked here)

    parallel_interface u_pi (
        .clk(clk), .reset(reset), .host_data(host_data), .host_cmd(host_cmd),
        .valid(valid), .data(pi_data), .address(address), .cmd(cmd)
    );

    ann_controller dut (
        .clk(clk), .rst_n(rst_n),
        .valid(valid), .data(pi_data), .address(address), .cmd(cmd),
        .ann_reset(ann_reset),
        .op_done(op_done), .ann_address(ann_address), .pulses(pulses),
        .weight_read_data(buf_data[3:0]),
        .buf_reg_add(buf_reg_add), .buf_reg_ctrl(buf_reg_ctrl), .buf_read_write(buf_read_write),
        .buf_bit_sel(buf_bit_sel),
        .buf_data_out(buf_data_out), .buf_ready(buf_ready), .buf_data(buf_data),
        .inf_start(), .dsa_clear(dsa_clear), .tree_start(), .col_sel(), .compute_done(compute_done), .prog_fail(), .op_timeout(), .read_data(), .read_done(), .busy(busy)
    );

    input_buffer u_input_buffer (
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
            end else if (busy && (pulses == PULSE_MODE_READ || pulses == PULSE_MODE_ERASE || pulses == PULSE_MODE_INF)) begin
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

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    initial begin
        rst_n = 0; reset = 1; host_data = 0; host_cmd = CMD_HIZ;
        repeat(5) @(posedge clk);
        rst_n = 1; reset = 0;
        repeat(2) @(posedge clk);
    end

    // Expected ann_address word, built by hand from the packed address
    // {PE[15:14], SA[13:10], ROW[9:6], COL[5:0]} into
    // {data[31:24], PE[23:22], SA[21:18], ROW[17:14], COL[13:8], rsvd[7:0]=0}
    // (independent of the package pack helpers, then cross-checked against them).
    function automatic logic [31:0] exp_word(logic [WEIGHT_ADDR_WIDTH-1:0] a, logic [7:0] d);
        logic [1:0] pe;
        logic [3:0] sa, rw;
        logic [5:0] cl;
        logic [31:0] w;
        pe = a[15:14];
        sa = a[13:10];
        rw = a[9:6];
        cl = a[5:0];
        w  = {d, pe, sa, rw, cl, 8'h00};
        if (w !== pack_ann_address(d, pe, sa, rw, cl))
            $error("exp_word: hand-packed 0x%08h != pack_ann_address 0x%08h", w,
                   pack_ann_address(d, pe, sa, rw, cl));
        return w;
    endfunction

    function automatic string cmd_name(input logic [2:0] c);
        case (c)
            CMD_READ:  return "CMD_READ";
            CMD_PROG:  return "CMD_PROG";
            CMD_ERASE: return "CMD_ERASE";
            CMD_INF:   return "CMD_INF";
            CMD_HIZ:   return "CMD_HIZ";
            default:   return "CMD_???";
        endcase
    endfunction

    function automatic string pulse_name(input logic [2:0] p);
        case (p)
            PULSE_MODE_HIZ:   return "HIZ";
            PULSE_MODE_READ:  return "READ";
            PULSE_MODE_PROG:  return "PROG";
            PULSE_MODE_ERASE: return "ERASE";
            PULSE_MODE_INF:   return "INF";
            default:          return "???";
        endcase
    endfunction

    int fd;
    int pass_c, fail_c;

    // Hierarchical peek at buffer RAM for post-transaction snapshot (simulation only).
    // 256 entries = 16 rows x 16 (row r = sub-array r, column = row within that SA).
    task automatic dump_buffer_snapshot(input string ctx);
        int rr, cc;
        string line;
        $fdisplay(fd, "  // input_buffer after %s — buffer_reg[0:%0d], 16x16 row-major (hex):", ctx, BUFFER_SIZE - 1);
        for (rr = 0; rr < BUFFER_SIZE / 16; rr++) begin
            line = $sformatf("  //   row%02d ", rr);
            for (cc = 0; cc < 16; cc++)
                line = {line, $sformatf(" %02h", u_input_buffer.buffer_reg[rr * 16 + cc])};
            $fdisplay(fd, "%s", line);
        end
    endtask

    task automatic log_host_and_pi(
        input logic [31:0] host_pkt,
        input logic [2:0]  host_c,
        input logic [WEIGHT_ADDR_WIDTH-1:0] par_a
    );
        logic [WEIGHT_ADDR_WIDTH-1:0] pi_addr;
        logic        pi_valid;
        pi_valid = (host_c != CMD_HIZ) && host_word_is_valid(host_pkt);
        pi_addr  = extract_address(host_pkt);
        $fdisplay(fd, "  // Host packet: host_cmd=%s (%b)", cmd_name(host_c), host_c);
        $fdisplay(fd, "  //   host_data:");
        $fdisplay(fd, "  //     %08b-%02b-%04b-%04b-%06b-%08b",
            host_pkt[31:24], host_pkt[23:22], host_pkt[21:18], host_pkt[17:14], host_pkt[13:8], host_pkt[7:0]);
        $fdisplay(fd, "  //     data[7:0] - PE[1:0] - SA[3:0] - ROW[3:0] - COL[5:0] - rsvd[7:0]");
        $fdisplay(fd, "  //   host_data (hex): 0x%08h", host_pkt);
        $fdisplay(fd, "  // PI -> controller:");
        $fdisplay(fd, "  //   valid=%b  data(bin)=%08b  cmd=%s (%b)",
            pi_valid, host_pkt[31:24], cmd_name(host_c), host_c);
        $fdisplay(fd, "  //   address[15:0]:");
        $fdisplay(fd, "  //     %02b-%04b-%04b-%06b",
            pi_addr[15:14], pi_addr[13:10], pi_addr[9:6], pi_addr[5:0]);
        $fdisplay(fd, "  //     PE[1:0] - SA[3:0] - ROW[3:0] - COL[5:0]");
        $fdisplay(fd, "  //   address (hex): 0x%04h  |  parallel_addr idx: pe=%0d sa=%0d row=%0d col=%0d",
            pi_addr, get_pe_id(par_a), get_sa_id(par_a), get_row_id(par_a), get_col_id(par_a));
        if (pi_addr !== par_a)
            $fdisplay(fd, "  //   WARNING: extract_address(host_data)=0x%04h != requested 0x%04h", pi_addr, par_a);
    endtask

    task automatic wait_until_idle(int maxcyc);
        int n;
        n = 0;
        while (busy) begin
            @(posedge clk);
            n++;
            if (n > maxcyc)
                break;
        end
    endtask

    // valid = (host_cmd != HIZ). Holding a command high for the whole transaction re-arms INF (and
    // breaks check_cmd's busy wait) every time the DUT returns to IDLE between COLLECT/COMPUTE rows.
    task automatic check_cmd(
        input logic [CMD_WIDTH-1:0] c,
        input logic [WEIGHT_ADDR_WIDTH-1:0] a,
        input logic [7:0] d,
        input logic [2:0] exp_pulse,
        input string ctx
    );
        logic [31:0] exp, saw_word, host_pkt;
        logic [2:0]  saw_pulse;
        int to;
        int busy_cycles;
        bit pulse_ok, word_ok;
        host_pkt = build_host_ann_word(d, a);
        exp = exp_word(a, d);
        wait_until_idle(500_000);
        repeat(8) @(posedge clk);
        host_data = 0;
        host_cmd  = CMD_HIZ;
        @(posedge clk);
        $fdisplay(fd, "");
        $fdisplay(fd, "//==============================================================================");
        $fdisplay(fd, "// %s", ctx);
        $fdisplay(fd, "//==============================================================================");
        host_data = host_pkt;
        host_cmd  = c;
        log_host_and_pi(host_pkt, c, a);
        $fdisplay(fd, "  // PI holds one-cycle beat: @(posedge) then host_cmd=HIZ (valid deasserts).");
        @(posedge clk);
        host_data = '0;
        host_cmd  = CMD_HIZ;
        @(posedge clk);
        wait (busy);
        saw_word = '0;
        saw_pulse = '0;
        pulse_ok = 0;
        word_ok = 0;
        to = 0;
        busy_cycles = 0;
        while (busy) begin
            busy_cycles++;
            if (pulses === exp_pulse)
                pulse_ok = 1;
            if (pulses !== 3'b000)
                saw_pulse = pulses;
            if (ann_address !== '0)
                saw_word = ann_address;
            if (ann_address === exp)
                word_ok = 1;
            @(posedge clk);
            to++;
            if (to > 500_000)
                break;
        end
        $fdisplay(fd, "  // DUT: controller busy for %0d posedge cycles (then idle)", busy_cycles);
        $fdisplay(fd, "  //   pulses: expected %s (%b)  |  last non-zero saw %s (%b)  pulse_ok=%0b",
            pulse_name(exp_pulse), exp_pulse, pulse_name(saw_pulse), saw_pulse, pulse_ok);
        $fdisplay(fd, "  //   ann_address: exp=0x%08h  saw_last_nonzero=0x%08h  saw_match_exp=%0b",
            exp, saw_word, word_ok);
        dump_buffer_snapshot(ctx);
        $fdisplay(fd, "  // Return to idle: busy=%0b", busy);
        if (!pulse_ok || !word_ok) begin
            fail_c++;
            $fdisplay(fd, "FAIL %s  pulse_ok=%0b word_ok=%0b  saw_pulse=%b saw_word=0x%h  exp_word=0x%h",
                      ctx, pulse_ok, word_ok, saw_pulse, saw_word, exp);
        end else begin
            pass_c++;
            $fdisplay(fd, "PASS %s", ctx);
        end
        if (busy) begin
            fail_c++;
            $fdisplay(fd, "FAIL %s: busy stuck high after case", ctx);
        end
        host_data = '0;
        host_cmd  = CMD_HIZ;
    endtask

    // One full input vector: 1 + INPUT_VECTOR_LEN (=257) consecutive clocks with INF (first beat
    // enters COLLECT from IDLE; then INPUT_VECTOR_LEN counted valid beats, one pixel per clock,
    // before COMPUTE). host_cmd returns to HIZ right after the last pixel beat so the DUT does not
    // re-arm INF when it returns to IDLE. Monitor must run in parallel with the drive burst or
    // COLLECT words are missed.
    task automatic check_inf_row8(
        input logic [WEIGHT_ADDR_WIDTH-1:0] a,
        input logic [7:0] d,
        input string ctx
    );
        logic [31:0] exp, saw_word, host_pkt;
        logic [2:0]  saw_pulse;
        int to;
        int busy_cycles;
        bit pulse_ok, word_ok;
        host_pkt = build_host_ann_word(d, a);
        exp = exp_word(a, d);
        wait_until_idle(500_000);
        repeat(8) @(posedge clk);
        host_data = '0;
        host_cmd  = CMD_HIZ;
        @(posedge clk);
        $fdisplay(fd, "");
        $fdisplay(fd, "//==============================================================================");
        $fdisplay(fd, "// %s  (INF: %0d clk host_cmd=CMD_INF, same host_data each cycle)", ctx, 1 + INPUT_VECTOR_LEN);
        $fdisplay(fd, "//==============================================================================");
        host_data = host_pkt;
        host_cmd  = CMD_INF;
        log_host_and_pi(host_pkt, CMD_INF, a);
        $fdisplay(fd, "  // INF: buffer loads %0d pixels (CTRL_DATA_LOAD), then COMPUTE issues INF pulses.", INPUT_VECTOR_LEN);
        saw_word = '0;
        saw_pulse = '0;
        pulse_ok = 0;
        word_ok = 0;
        to = 0;
        busy_cycles = 0;
        fork
            begin : drive_inf_row
                repeat(1 + INPUT_VECTOR_LEN) @(posedge clk);
                host_data = '0;
                host_cmd  = CMD_HIZ;
                @(posedge clk);
            end
            begin : mon_inf_row
                wait (busy);
                while (busy) begin
                    busy_cycles++;
                    if (pulses === PULSE_MODE_INF)
                        pulse_ok = 1;
                    if (pulses !== 3'b000)
                        saw_pulse = pulses;
                    if (ann_address !== '0)
                        saw_word = ann_address;
                    if (ann_address === exp)
                        word_ok = 1;
                    @(posedge clk);
                    to++;
                    if (to > 500_000)
                        break;
                end
            end
        join
        $fdisplay(fd, "  // DUT: controller busy for %0d posedge cycles (INF COLLECT + COMPUTE + ...)", busy_cycles);
        $fdisplay(fd, "  //   pulses: expected %s (%b)  |  last non-zero saw %s (%b)  pulse_ok=%0b",
            pulse_name(PULSE_MODE_INF), PULSE_MODE_INF, pulse_name(saw_pulse), saw_pulse, pulse_ok);
        $fdisplay(fd, "  //   ann_address: exp=0x%08h  saw_last_nonzero=0x%08h  saw_match_exp=%0b  (INF may toggle word during COLLECT vs COMPUTE)",
            exp, saw_word, word_ok);
        dump_buffer_snapshot(ctx);
        $fdisplay(fd, "  // Return to idle: busy=%0b", busy);
        if (!pulse_ok || !word_ok) begin
            fail_c++;
            $fdisplay(fd, "FAIL %s  saw_inf_pulse=%0b saw_match_word=%0b saw_pulse=%b saw_word=0x%h exp_word=0x%h",
                      ctx, pulse_ok, word_ok, saw_pulse, saw_word, exp);
        end else begin
            pass_c++;
            $fdisplay(fd, "PASS %s", ctx);
        end
        if (busy) begin
            fail_c++;
            $fdisplay(fd, "FAIL %s: busy stuck high after case", ctx);
        end
        host_data = '0;
        host_cmd  = CMD_HIZ;
    endtask

    // Negative case: non-zero reserved bits host_data[7:0] -> parallel_interface.valid stays low,
    // controller must stay idle (no busy, no pulses).
    task automatic check_rsvd_reject(
        input logic [CMD_WIDTH-1:0] c,
        input logic [WEIGHT_ADDR_WIDTH-1:0] a,
        input logic [7:0] d,
        input logic [7:0] rsvd,
        input string ctx
    );
        logic [31:0] host_pkt;
        bit saw_valid, saw_busy, saw_pulse;
        host_pkt = build_host_ann_word(d, a) | {24'h0, rsvd};
        wait_until_idle(500_000);
        repeat(8) @(posedge clk);
        host_data = '0;
        host_cmd  = CMD_HIZ;
        @(posedge clk);
        $fdisplay(fd, "");
        $fdisplay(fd, "//==============================================================================");
        $fdisplay(fd, "// %s  (non-zero reserved bits host_data[7:0]=0x%02h -> expect reject)", ctx, rsvd);
        $fdisplay(fd, "//==============================================================================");
        host_data = host_pkt;
        host_cmd  = c;
        log_host_and_pi(host_pkt, c, a);
        saw_valid = 0;
        saw_busy  = 0;
        saw_pulse = 0;
        #1;
        if (valid === 1'b1)
            saw_valid = 1;
        @(posedge clk);
        host_data = '0;
        host_cmd  = CMD_HIZ;
        repeat(20) begin
            if (busy === 1'b1)
                saw_busy = 1;
            if (pulses !== 3'b000)
                saw_pulse = 1;
            @(posedge clk);
        end
        $fdisplay(fd, "  // PI valid during beat=%0b  |  controller busy seen=%0b  pulses seen=%0b",
            saw_valid, saw_busy, saw_pulse);
        if (saw_valid || saw_busy || saw_pulse) begin
            fail_c++;
            $fdisplay(fd, "FAIL %s  valid=%0b busy=%0b pulses=%0b (malformed word was accepted)",
                      ctx, saw_valid, saw_busy, saw_pulse);
        end else begin
            pass_c++;
            $fdisplay(fd, "PASS %s", ctx);
        end
    endtask

    initial begin
        pass_c = 0;
        fail_c = 0;
        fd = $fopen(LOG_FILE, "w");
        if (!fd) begin $error("Cannot open log"); $finish; end
        $fdisplay(fd, "// PI + controller + input_buffer integration (verbose report)");
        $fdisplay(fd, "//");
        $fdisplay(fd, "// Per case: host 32b + host_cmd | PI->controller (valid,data,address,cmd) |");
        $fdisplay(fd, "//   busy cycle count | pulses + ann_address (exp vs saw) | input_buffer 16x16 dump | idle.");
        $fdisplay(fd, "// Address (16b packed): {PE[15:14], SA[13:10], ROW[9:6], COL[5:0]}");
        $fdisplay(fd, "// ann_address / host_data: {data[31:24], PE[23:22], SA[21:18], ROW[17:14], COL[13:8], rsvd[7:0]=0}");
        $fdisplay(fd, "");
        wait(rst_n);
        repeat(6) @(posedge clk);

        // Addresses: pack_ann_addr(PE, SA, ROW, COL) -> {PE[15:14], SA[13:10], ROW[9:6], COL[5:0]}
        check_cmd(CMD_READ, pack_ann_addr(2'd3, 4'd15, 4'd15, 6'd63), 8'h00, PULSE_MODE_READ, "01_READ");   // 0xFFFF
        @(posedge clk);

        check_cmd(CMD_PROG, pack_ann_addr(2'd0, 4'd0, 4'd0, 6'd0), 8'h0C, PULSE_MODE_PROG, "02_PROG");       // 0x0000
        @(posedge clk);

        check_cmd(CMD_ERASE, pack_ann_addr(2'd2, 4'd5, 4'd9, 6'd33), 8'h00, PULSE_MODE_ERASE, "03_ERASE");   // 0x9661
        @(posedge clk);

        check_inf_row8(pack_ann_addr(2'd1, 4'd10, 4'd3, 6'd21), 8'hAA, "04_INF");                           // 0x68D5
        @(posedge clk);

        check_cmd(CMD_READ, pack_ann_addr(2'd0, 4'd1, 4'd0, 6'd4), 8'h00, PULSE_MODE_READ, "05_READ");       // 0x0404
        @(posedge clk);

        check_cmd(CMD_PROG, pack_ann_addr(2'd1, 4'd0, 4'd15, 6'd8), 8'h0E, PULSE_MODE_PROG, "06_PROG");      // 0x43C8
        @(posedge clk);

        check_cmd(CMD_ERASE, pack_ann_addr(2'd0, 4'd9, 4'd1, 6'd0), 8'h00, PULSE_MODE_ERASE, "07_ERASE");    // 0x2440
        @(posedge clk);

        check_cmd(CMD_READ, pack_ann_addr(2'd3, 4'd0, 4'd0, 6'd0), 8'h00, PULSE_MODE_READ, "08_READ");       // 0xC000
        @(posedge clk);

        check_cmd(CMD_PROG, pack_ann_addr(2'd0, 4'd1, 4'd7, 6'd63), 8'h0F, PULSE_MODE_PROG, "09_PROG");      // 0x05FF
        @(posedge clk);

        check_cmd(CMD_ERASE, pack_ann_addr(2'd0, 4'd0, 4'd3, 6'd60), 8'h00, PULSE_MODE_ERASE, "10_ERASE");   // 0x00FC
        @(posedge clk);

        check_rsvd_reject(CMD_PROG, pack_ann_addr(2'd2, 4'd11, 4'd6, 6'd42), 8'h05, 8'h01, "11_RSVD_REJECT"); // 0xADAA, rsvd=0x01
        @(posedge clk);

        $fdisplay(fd, "// Summary: PASS=%0d FAIL=%0d", pass_c, fail_c);
        $fclose(fd);
        $display("[%0t] parallel_interface_controller_integration_tb: PASS=%0d FAIL=%0d -> %s",
                 $time, pass_c, fail_c, LOG_FILE);
        if (fail_c != 0)
            $fatal(1, "parallel_interface_controller_integration_tb failures");
        $finish;
    end

endmodule
