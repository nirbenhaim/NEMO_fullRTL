//------------------------------------------------------------------------------
// nemo_top_wrapper_tb.sv
//------------------------------------------------------------------------------
// End-to-end test of nemo_top_wrapper <-> ann_controller integration.
// The host is modelled at the pads (SPI: sclk / mosi / csn); a behavioural
// model of the analog Y-Flash array and of the digital datapath sits on the
// core boundaries.
//
//   T1  idle state after reset
//   T2  PROG via SPI at the max address  (PE3 SA15 ROW15 COL63, weight 0xA)
//   T3  PROG via SPI at a second address (PE1 SA2  ROW5  COL7,  weight 0x3)
//   T4  READ via SPI
//   T5  ERASE via SPI (cell 1 cleared, cell 2 untouched)
//   T6  frame with non-zero reserved bits is rejected
//   T7  HIZ frame does nothing
//   T8  INF (Rev 5 schedule, NEMO CS-IMC Architecture Brief Rev. 1.0 §6 with
//       MSB-first bit order): start frame + 256 pixel frames over SPI, then
//       16 column positions x 8 bit-planes.  Checked every array-read cycle:
//       col_sel = column k (held for 8 planes), planes in the order b7..b0,
//       ann_input_bits = that plane of every pixel.  Also checked: number and
//       timing of start / dsa_clear / tree_result_valid / act_start, compute
//       length 16 x 136 cycles, and all 256 outputs against an independent
//       reference (LSB-first sum of per-SA 7-bit-clipped tree results).
//   T9  irq once per inference, 64-word MISO readback of the 256 outputs,
//       re-read, and the 2048-bit PISO stream
//   T10 second INF: result_valid cleared at inf_start, new 256 outputs
//   MISO status bits are also checked on idle / busy frames in T7 / T8.
//   T11 SPI frame aborted after 20 bits; next frame still decoded correctly
//   T12 stuck cell: PROG gives up after the erase retries -> prog_fail on MISO,
//       cleared by the next successful PROG
//   T13 reset during an operation: synchronised release, clean restart
//   T14 READ result returned on MISO in the frame after the READ
//   T15 analog core never answers op_done -> watchdog abort, error bit
//   T16 host stops sending pixels mid-INF -> watchdog abort, error bit
//   Every test also checks: exactly one controller beat per SPI frame.
//
// Mock analog core:
//   op_done          = 1 (core always ready)
//   PROG pulse       : yf_cell[addr] <= ann_core_word[27:24]  (ideal programming)
//   ERASE pulse      : yf_cell[addr] <= 0
//   weight_read_data = yf_cell[addr of ann_core_word]
//   Inference read   : for PE pe, ADC/tree g and column position col_sel, SA s
//                      returns min(sum_r yf_cell[pe,s,r,g*16+col_sel] *
//                      ann_input_bits[s*16+r], 127)   (7-bit ADC, unity FS)
//
// Mock digital datapath (Roman's blocks, behavioural):
//   adder tree : on 'start' samples the 16 ADC codes of each tree, sums them;
//                tree_result_valid 4 cycles later (TREE_LATENCY)
//   DSA        : sync clear on dsa_clear; acc <= (acc << 1) + tree on
//                tree_result_valid (MSB-first accumulation)
//   act_sat    : on act_start, ACT_LATENCY cycles later act_out = sat8(acc >>
//                ACT_SHIFT) for all 16 trees, act_valid high for 4 cycles
//
// MISO response frame (35 bits, MSB first) = {result[31:0], error, busy, valid}
//   after a READ frame: {8'h00, addr[15:0], data[7:0], error, busy, read_valid}
// irq is a level: set by a new result, cleared by the read frame that returns it.
//------------------------------------------------------------------------------

`timescale 1ns/1ps

import ann_arch_pkg::*;
import parallel_interface_pkg::*;
import controller_pkg::*;

module nemo_top_wrapper_tb;

    //--------------------------------------------------------------------------
    // Timing
    //--------------------------------------------------------------------------
    localparam time CLK_HALF  = 5ns;    // 100 MHz simulation clock (all checks in cycles)
    localparam time SCLK_HALF = 50ns;   // 10 MHz SPI clock (asynchronous to clk)
    localparam int  TREE_LATENCY     = T_INF_TREE;   // adder tree: start -> tree_result_valid
    localparam int  ACT_LATENCY      = 2;            // act_start -> act_valid (within T_COL_ACT)
    localparam int  ACT_SHIFT        = 10;           // mock requantisation: act = sat8(acc >> 10)
    localparam int  OP_DONE_TO       = 1000;    // shortened watchdogs for simulation
    localparam int  COLLECT_TO       = 20000;   // > one SPI frame (~390 clk)

    localparam int  STEPS_PER_INF    = NUM_COL_POS * ACT_WIDTH;     // 128 bit-steps
    localparam int  READ_CYC_PER_COL = ACT_WIDTH * TINF;            // 80 INF cycles per column
    localparam int  RESULT_WORDS     = OUTPUT_BITS / WORD_WIDTH;    // 64

    localparam logic [2:0] P_HIZ   = 3'b000;
    localparam logic [2:0] P_READ  = 3'b001;
    localparam logic [2:0] P_PROG  = 3'b010;
    localparam logic [2:0] P_ERASE = 3'b011;
    localparam logic [2:0] P_INF   = 3'b100;

    //--------------------------------------------------------------------------
    // DUT signals
    //--------------------------------------------------------------------------
    logic        clk = 0, rst_n = 0;
    logic        sclk = 0, mosi = 0, csn = 1;
    logic        miso, irq, piso_out, dft0_buff, dft1_buff;
    logic        scan_en = 0;

    logic                          ann_reset;
    logic [WORD_WIDTH-1:0]         ann_core_word;
    logic [2:0]                    pulses;
    logic [INPUT_VECTOR_LEN-1:0]   ann_input_bits;
    logic [COL_SEL_WIDTH-1:0]      col_sel;
    logic                          op_done;
    logic [ANN_WEIGHT_WIDTH-1:0]   weight_read_data;

    logic                          dsa_clear, start, act_start;
    logic [BIT_SEL_WIDTH-1:0]      bit_idx;
    logic [PE_ID_WIDTH-1:0]        pe_sel;

    logic [NUM_TREES-1:0]           tree_result_valid;
    logic [NUM_TREES*ACC_WIDTH-1:0] acc_out;
    logic [NUM_TREES-1:0]           act_valid = '0;
    logic [RESULT_BITS-1:0]         act_out   = '0;

    always #(CLK_HALF) clk = ~clk;

    nemo_top_wrapper #(
        .OP_DONE_TIMEOUT (OP_DONE_TO),
        .COLLECT_TIMEOUT (COLLECT_TO)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .sclk(sclk), .mosi(mosi), .miso(miso), .csn(csn),
        .irq(irq), .scan_en(scan_en), .piso_out(piso_out),
        .dft0_buff(dft0_buff), .dft1_buff(dft1_buff),
        .ann_reset(ann_reset), .ann_core_word(ann_core_word), .pulses(pulses),
        .ann_input_bits(ann_input_bits), .col_sel(col_sel),
        .op_done(op_done), .weight_read_data(weight_read_data),
        .dsa_clear(dsa_clear), .start(start), .act_start(act_start),
        .bit_idx(bit_idx), .pe_sel(pe_sel),
        .tree_result_valid(tree_result_valid), .acc_out(acc_out),
        .act_valid(act_valid), .act_out(act_out)
    );

    //--------------------------------------------------------------------------
    // Mock analog Y-Flash array
    //--------------------------------------------------------------------------
    logic [ANN_WEIGHT_WIDTH-1:0] yf_cell [0:TOTAL_WEIGHT_LOCATIONS-1];
    logic [ANN_WEIGHT_WIDTH-1:0] yf_save [0:TOTAL_WEIGHT_LOCATIONS-1];   // T8-T10 backdoor load
    ann_addr_t core_addr;

    assign core_addr        = word_addr(ann_core_word);
    bit    op_done_en = 1;          // T15: core stops answering
    assign op_done          = op_done_en;
    assign weight_read_data = yf_cell[core_addr];

    initial for (int i = 0; i < TOTAL_WEIGHT_LOCATIONS; i++) yf_cell[i] = '0;

    ann_addr_t a_stuck = '0;
    bit        stuck_en = 0;      // T12: this cell ignores PROG pulses

    always @(posedge clk) begin
        if (pulses == P_PROG && !(stuck_en && core_addr == a_stuck))
            yf_cell[core_addr] <= ann_core_word[WORD_DATA_LSB +: ANN_WEIGHT_WIDTH];
        else if (pulses == P_ERASE)
            yf_cell[core_addr] <= '0;
    end

    // Per-SA ADC code: min(sum_r W * bit, 127) for column 'col' of PE 'pe'
    function automatic int adc_code(input int pe, input int sa, input int col,
                                    input logic [INPUT_VECTOR_LEN-1:0] bits);
        int p;
        p = 0;
        for (int r = 0; r < SA_ROWS; r++)
            if (bits[sa*SA_ROWS + r])
                p += int'(yf_cell[pack_ann_addr(PE_ID_WIDTH'(pe), SA_ID_WIDTH'(sa),
                                                ROW_ID_WIDTH'(r), COL_ID_WIDTH'(col))]);
        return (p > 127) ? 127 : p;
    endfunction

    // Adder-tree input of tree t (= pe*4 + g) at column position k
    function automatic int tree_sum(input int t, input int k,
                                    input logic [INPUT_VECTOR_LEN-1:0] bits);
        int pe, g, s;
        pe = t / ADCS_PER_SA;
        g  = t % ADCS_PER_SA;
        s  = 0;
        for (int sa = 0; sa < NUM_SA; sa++)
            s += adc_code(pe, sa, g * COL_MUX_RATIO + k, bits);
        return s;
    endfunction

    //--------------------------------------------------------------------------
    // Mock digital datapath (Roman): 16 adder trees + DSA + act_sat
    //--------------------------------------------------------------------------
    logic [TREE_LATENCY-1:0] tree_pipe = '0;
    int                      tree_val [TREE_LATENCY][NUM_TREES];
    logic [ACC_WIDTH-1:0]    acc_m    [NUM_TREES];

    initial for (int t = 0; t < NUM_TREES; t++) acc_m[t] = '0;

    always @(posedge clk) begin
        // DSA first (uses the tree output of the previous cycle):
        // sync clear, MSB-first shift-and-add on each tree result
        if (dsa_clear)
            for (int t = 0; t < NUM_TREES; t++) acc_m[t] <= '0;
        else if (tree_result_valid[0])
            for (int t = 0; t < NUM_TREES; t++)
                acc_m[t] <= (acc_m[t] << 1) + ACC_WIDTH'(tree_val[TREE_LATENCY-1][t]);
        // adder tree: sample on start, result TREE_LATENCY cycles later
        for (int s = TREE_LATENCY - 1; s > 0; s--) tree_val[s] = tree_val[s-1];
        if (start)
            for (int t = 0; t < NUM_TREES; t++) tree_val[0][t] = tree_sum(t, int'(col_sel), ann_input_bits);
        tree_pipe <= {tree_pipe[TREE_LATENCY-2:0], start};
    end
    assign tree_result_valid = {NUM_TREES{tree_pipe[TREE_LATENCY-1]}};
    always_comb for (int t = 0; t < NUM_TREES; t++) acc_out[t*ACC_WIDTH +: ACC_WIDTH] = acc_m[t];

    function automatic logic [7:0] sat8(input longint v);
        return (v > 255) ? 8'd255 : 8'(v);
    endfunction

    int         irq_cnt = 0;
    bit         irq_q   = 0;
    bit         chk_rv = 0;
    int         rv_clear_ok = 0, rv_clear_bad = 0;   // result_valid after inf_start
    int         act_lat_err = 0;

    always @(posedge clk) begin
        if (act_start) begin
            logic [RESULT_BITS-1:0] a;
            for (int t = 0; t < NUM_TREES; t++)
                a[t*8 +: 8] = sat8(longint'(acc_m[t]) >> ACT_SHIFT);
            fork
                begin
                    repeat (ACT_LATENCY - 1) @(posedge clk);
                    act_out   <= a;
                    act_valid <= '1;
                    repeat (4) @(posedge clk);
                    act_valid <= '0;
                end
            join_none
        end
        if (irq && !irq_q) irq_cnt++;     // irq is a level: count rising edges
        irq_q = irq;
        if (chk_rv) begin
            if (dut.result_valid) rv_clear_bad++; else rv_clear_ok++;
            chk_rv = 0;
        end
        if (dut.ctrl_inf_start) chk_rv = 1;
    end

    //--------------------------------------------------------------------------
    // Independent reference: LSB-first sum of the clipped tree results
    //   out[pe*64 + g*16 + k] = sat8( (sum_b tree(pe,g,k,b) << b) >> ACT_SHIFT )
    //--------------------------------------------------------------------------
    logic [7:0]          pix [0:INPUT_VECTOR_LEN-1];
    logic [OUTPUT_BITS-1:0] exp_out, prev_out;

    function automatic logic [INPUT_VECTOR_LEN-1:0] plane_of(input int b);
        logic [INPUT_VECTOR_LEN-1:0] v;
        for (int i = 0; i < INPUT_VECTOR_LEN; i++) v[i] = pix[i][b];
        return v;
    endfunction

    task automatic compute_reference();
        logic [INPUT_VECTOR_LEN-1:0] pl [ACT_WIDTH];
        for (int b = 0; b < ACT_WIDTH; b++) pl[b] = plane_of(b);
        for (int t = 0; t < NUM_TREES; t++)
            for (int k = 0; k < NUM_COL_POS; k++) begin
                longint acc;
                int o;
                acc = 0;
                for (int b = 0; b < ACT_WIDTH; b++) acc += longint'(tree_sum(t, k, pl[b])) << b;
                o = (t / ADCS_PER_SA) * SA_COLS + (t % ADCS_PER_SA) * COL_MUX_RATIO + k;
                exp_out[o*8 +: 8] = sat8(acc >> ACT_SHIFT);
            end
        // the reference must exercise the datapath, not just saturate
        begin
            int n_sat, n_zero, n_mid;
            n_sat = 0; n_zero = 0; n_mid = 0;
            for (int o = 0; o < NUM_OUTPUTS; o++)
                if (exp_out[o*8 +: 8] == 8'd255) n_sat++;
                else if (exp_out[o*8 +: 8] == 8'd0) n_zero++;
                else n_mid++;
            $display("  reference: %0d outputs in 1..254, %0d saturated, %0d zero", n_mid, n_sat, n_zero);
            check(n_mid >= NUM_OUTPUTS * 3 / 4,
                  $sformatf("reference spread: %0d/%0d outputs unsaturated (>= 75%%)", n_mid, NUM_OUTPUTS));
        end
    endtask

    //--------------------------------------------------------------------------
    // Check bookkeeping
    //--------------------------------------------------------------------------
    int n_pass = 0, n_fail = 0;

    function automatic void check(input bit cond, input string msg);
        if (cond) begin
            n_pass++;
            $display("  PASS: %s", msg);
        end else begin
            n_fail++;
            $display("  FAIL: %s  (t=%0t)", msg, $time);
        end
    endfunction

    //--------------------------------------------------------------------------
    // Monitors (posedge sampling = values held during the previous cycle)
    //--------------------------------------------------------------------------
    int unsigned cyc = 0;
    int ctrl_beats = 0, host_frames = 0;
    int prog_cyc = 0, read_cyc = 0, erase_cyc = 0, inf_cyc = 0;
    int dsa_clear_cnt = 0, act_start_cnt = 0, compute_done_cnt = 0, inf_start_cnt = 0;
    int unsigned t_compute_done = 0, t_act_start = 0;
    int start_cnt = 0, tv_cnt = 0;
    int unsigned t_first_inf = 0, t_first_start = 0, t_last_tv = 0;
    int ann_reset_seen = 0;
    int ops_done = 0;          // completed controller operations (busy 1 -> 0)
    bit busy_q   = 0;
    ann_addr_t                   last_read_addr;
    logic [ANN_WEIGHT_WIDTH-1:0] last_read_data;

    // PROG-phase observation
    bit          watch_prog = 0;
    ann_addr_t   exp_addr;
    logic [7:0]  exp_payload;
    int          prog_word_err = 0, prog_sel_err = 0;

    // INF-phase observation (T8 / T10)
    bit          watch_inf = 0;
    int          inf_idx = 0;                     // array-read (INF) cycles seen in this inference
    int          plane_err = 0, col_err = 0, lane_idle_err = 0;
    int          dsa_clear_ctx_err = 0, inf_start_ctx_err = 0, act_start_ctx_err = 0;
    int          start_ctx_err = 0, col_hold_err = 0;
    int          tv_in_col = 0;
    int          col_changes = 0;
    logic [COL_SEL_WIDTH-1:0] col_q = '0;

    // PISO observation (T9)
    int          piso_n = 0;
    logic [OUTPUT_BITS-1:0] piso_bits;

    always @(posedge clk) begin
        cyc++;
        if (dut.ctrl_valid)        ctrl_beats++;
        if (dut.host_valid)        host_frames++;
        if (ann_reset)             ann_reset_seen++;
        if (pulses == P_PROG)      prog_cyc++;
        if (pulses == P_READ) begin
            read_cyc++;
            last_read_addr = core_addr;
            last_read_data = weight_read_data;
        end
        if (busy_q && !dut.ctrl_busy) ops_done++;
        busy_q = dut.ctrl_busy;
        if (pulses == P_ERASE)     erase_cyc++;
        if (pulses == P_INF)       inf_cyc++;

        if (dut.ctrl_inf_start) begin
            inf_start_cnt++;
            // must coincide with acceptance of the last pixel
            if (!(dut.ctrl_valid && dut.u_ann_controller.pixel_cnt == INPUT_VECTOR_LEN - 1))
                inf_start_ctx_err++;
        end
        if (dut.ctrl_compute_done) begin
            compute_done_cnt++;
            t_compute_done = cyc;
        end
        if (pulses == P_INF && t_first_inf == 0) t_first_inf = cyc;
        if (dsa_clear)                             dsa_clear_cnt++;   // counted always (T16)

        // PROG: core word / PE select / row select while pulses are applied
        if (watch_prog && pulses == P_PROG) begin
            if (ann_core_word !== pack_ann_word(exp_payload, exp_addr)) prog_word_err++;
            if (pe_sel !== get_pe_id(exp_addr) ||
                dut.row_sel_internal !== get_row_id(exp_addr))      prog_sel_err++;
        end

        // INF: column sweep, MSB-first planes, datapath strobes
        if (watch_inf) begin
            if (pulses == P_INF) begin
                int k, b;
                k = inf_idx / READ_CYC_PER_COL;                          // column position
                b = (ACT_WIDTH - 1) - (inf_idx / TINF) % ACT_WIDTH;     // MSB first
                if (col_sel !== COL_SEL_WIDTH'(k)) col_err++;
                if (bit_idx !== BIT_SEL_WIDTH'(b) || ann_input_bits !== plane_of(b)) begin
                    plane_err++;
                    if (plane_err <= 4)
                        $display("  [col %0d plane %0d] bit_idx=%0d col_sel=%0d lanes mismatch=%0b",
                                 k, b, bit_idx, col_sel, ann_input_bits !== plane_of(b));
                end
                // dsa_clear exactly in the first array-read cycle of each column
                if (dsa_clear !== ((inf_idx % READ_CYC_PER_COL) == 0)) dsa_clear_ctx_err++;
                inf_idx++;
            end else begin
                if (dsa_clear) dsa_clear_ctx_err++;
                if (!dut.ctrl_busy && ann_input_bits !== '0) lane_idle_err++;
            end
            // col_sel moves only between columns, by +1
            if (col_sel !== col_q) begin
                col_changes++;
                if (col_sel !== col_q + 1'b1 && col_sel !== '0) col_hold_err++;
                col_q = col_sel;
            end
            if (dsa_clear) tv_in_col = 0;
            if (start) begin
                if (start_cnt == 0) t_first_start = cyc;
                start_cnt++;
                // start in the cycle right after the last conversion cycle
                if (pulses !== P_HIZ || dut.u_ann_controller.inf_cyc != INF_TREE_START_CYC) start_ctx_err++;
            end
            if (tree_result_valid[0]) begin
                tv_cnt++;
                tv_in_col++;
                t_last_tv = cyc;
            end
            if (act_start) begin
                act_start_cnt++;
                t_act_start = cyc;
                // one cycle after the 8th tree result of the column
                if (!(tv_in_col == ACT_WIDTH && t_act_start - t_last_tv == 1)) act_start_ctx_err++;
            end
        end

        // PISO: record the stream while it runs
        if (dut.piso_running && piso_n < OUTPUT_BITS) begin
            piso_bits[OUTPUT_BITS - 1 - piso_n] = piso_out;
            piso_n++;
        end
    end

    task automatic clear_inf_monitor();
        dsa_clear_cnt = 0; act_start_cnt = 0; compute_done_cnt = 0; inf_start_cnt = 0;
        start_cnt = 0; tv_cnt = 0; t_first_inf = 0; inf_idx = 0;
        plane_err = 0; col_err = 0; lane_idle_err = 0;
        dsa_clear_ctx_err = 0; inf_start_ctx_err = 0; act_start_ctx_err = 0;
        start_ctx_err = 0; col_hold_err = 0; col_changes = 0; col_q = '0;
        tv_in_col = 0;
    endtask

    // Checks common to every inference (T8, T10)
    task automatic check_inference(input string tag);
        check(inf_start_cnt == 1 && inf_start_ctx_err == 0,
              $sformatf("%s: inf_start once, at acceptance of pixel 255", tag));
        check(inf_idx == NUM_COL_POS * READ_CYC_PER_COL,
              $sformatf("%s: %0d array-read (INF) cycles = 16 columns x 8 planes x %0d", tag, inf_idx, TINF));
        check(col_err == 0,      $sformatf("%s: col_sel = column k during all 8 planes of column k", tag));
        check(plane_err == 0,    $sformatf("%s: planes MSB first (b7..b0) per column, lanes = that bit of every pixel", tag));
        check(col_hold_err == 0 && col_changes == NUM_COL_POS,
              $sformatf("%s: col_sel advanced 0 -> 15 in steps of 1 (%0d changes incl. return to 0)", tag, col_changes));
        check(lane_idle_err == 0, $sformatf("%s: ann_input_bits = 0 outside the inference", tag));
        check(dsa_clear_cnt == NUM_COL_POS && dsa_clear_ctx_err == 0,
              $sformatf("%s: dsa_clear %0d times, each in the first array-read cycle of a column", tag, dsa_clear_cnt));
        check(start_cnt == STEPS_PER_INF && start_ctx_err == 0,
              $sformatf("%s: start %0d times (one per bit-step), each at bit-step cycle %0d", tag, start_cnt,
                        INF_TREE_START_CYC));
        check(t_first_start - t_first_inf == INF_TREE_START_CYC,
              $sformatf("%s: first start %0d cycles after the first plane (settle %0d + ADC %0d)", tag,
                        t_first_start - t_first_inf, T_INF_SETTLE, T_INF_ADC));
        check(tv_cnt == STEPS_PER_INF, $sformatf("%s: %0d tree results", tag, tv_cnt));
        check(act_start_cnt == NUM_COL_POS && act_start_ctx_err == 0,
              $sformatf("%s: act_start %0d times, each 1 cycle after the 8th tree result of its column", tag,
                        act_start_cnt));
        check(compute_done_cnt == 1, $sformatf("%s: compute_done once", tag));
        check(t_compute_done - t_first_inf == T_INF_COMPUTE - 1,
              $sformatf("%s: compute phase %0d cycles (brief Table 7: 16 x %0d = %0d)", tag,
                        t_compute_done - t_first_inf + 1, T_INF_COLUMN, T_INF_COMPUTE));
        check(dut.result_reg === exp_out,
              $sformatf("%s: output buffer = independent reference for all %0d outputs", tag, NUM_OUTPUTS));
        if (dut.result_reg !== exp_out) begin
            int nbad;
            nbad = 0;
            for (int o = 0; o < NUM_OUTPUTS; o++)
                if (dut.result_reg[o*8 +: 8] !== exp_out[o*8 +: 8]) begin
                    if (nbad < 6) $display("    out[%0d] = %0d expected %0d", o, dut.result_reg[o*8 +: 8], exp_out[o*8 +: 8]);
                    nbad++;
                end
            $display("    %0d outputs differ", nbad);
        end
    endtask

    //--------------------------------------------------------------------------
    // Host SPI model: 35-bit frame {host_data[31:0], host_cmd[2:0]}, MSB first,
    // data changes while sclk is low, DUT samples on sclk rising edge.
    //--------------------------------------------------------------------------
    // MISO bits are sampled on every sclk rising edge (SPI mode 0) into miso_rx.
    logic [WORD_WIDTH+2:0] miso_rx;

    task automatic spi_send(input logic [WORD_WIDTH-1:0] data, input logic [2:0] cmd);
        logic [WORD_WIDTH+2:0] frame;
        frame = {data, cmd};
        csn = 0;
        #(SCLK_HALF);
        for (int b = WORD_WIDTH + 2; b >= 0; b--) begin
            mosi = frame[b];
            #(SCLK_HALF); sclk = 1;
            miso_rx[b] = miso;              // host samples MISO on the rising edge
            #(SCLK_HALF); sclk = 0;
        end
        #(SCLK_HALF);
        csn  = 1;
        mosi = 0;
        #(4 * SCLK_HALF);
    endtask

    // Send only the first nbits of a frame, then release csn (aborted frame)
    task automatic spi_send_partial(input logic [WORD_WIDTH-1:0] data, input logic [2:0] cmd,
                                    input int nbits);
        logic [WORD_WIDTH+2:0] frame;
        frame = {data, cmd};
        csn = 0;
        #(SCLK_HALF);
        for (int b = WORD_WIDTH + 2; b > WORD_WIDTH + 2 - nbits; b--) begin
            mosi = frame[b];
            #(SCLK_HALF); sclk = 1;
            #(SCLK_HALF); sclk = 0;
        end
        #(SCLK_HALF);
        csn  = 1;
        mosi = 0;
        #(4 * SCLK_HALF);
    endtask

    // Read the 2048-bit inference result: 65 HIZ frames, the payload of frame
    // i selects the word returned by frame i+1 (frame 0 only selects word 0).
    logic [OUTPUT_BITS-1:0] rd_result;
    logic [2:0]             rd_status;

    task automatic read_result();
        spi_send(build_host_ann_word(8'd0, '0), CMD_HIZ);          // select word 0
        for (int k = 0; k < RESULT_WORDS; k++) begin
            spi_send(build_host_ann_word(8'((k + 1) % RESULT_WORDS), '0), CMD_HIZ);
            rd_result[k*WORD_WIDTH +: WORD_WIDTH] = miso_rx[WORD_WIDTH+2:3];
            rd_status = miso_rx[2:0];
        end
    endtask

    // Wait until one more controller operation has completed since ops0
    // (counted by the monitor, so a short op that ends during spi_send is not missed)
    task automatic wait_op_done(input string what, input int ops0);
        int t;
        t = 0;
        while (ops_done == ops0 && t < 20000) begin @(posedge clk); t++; end
        check(ops_done == ops0 + 1, {what, ": controller ran and returned to idle"});
        repeat (5) @(posedge clk);
    endtask

    task automatic idle_cycles(input int n);
        repeat (n) @(posedge clk);
    endtask

    // Backdoor: random 4-bit weights in every cell (65,536 cells - programming
    // them over SPI would take far too long); the old contents are restored
    // after T10 so the later PROG/READ/ERASE tests see their own cells.
    task automatic load_random_weights(input int seed);
        int s;
        s = seed;
        for (int i = 0; i < TOTAL_WEIGHT_LOCATIONS; i++) begin
            yf_save[i] = yf_cell[i];
            yf_cell[i] = ANN_WEIGHT_WIDTH'($urandom(s));
            s = s * 1103515245 + 12345 + i;
        end
    endtask

    task automatic restore_weights();
        for (int i = 0; i < TOTAL_WEIGHT_LOCATIONS; i++) yf_cell[i] = yf_save[i];
    endtask

    //--------------------------------------------------------------------------
    // Stimulus
    //--------------------------------------------------------------------------
    ann_addr_t a1, a2, a3, a_inf;
    int beats0, frames0, p0, r0, e0, i0, ops0;

    initial begin
        $display("==============================================================");
        $display(" nemo_top_wrapper <-> ann_controller integration test");
        $display(" Geometry: %0d PE x %0d SA x %0dx%0d, input vector %0d",
                 NUM_PE, NUM_SA, SA_ROWS, SA_COLS, INPUT_VECTOR_LEN);
        $display(" Inference: %0d column positions x %0d planes (MSB first), %0d compute cycles",
                 NUM_COL_POS, ACT_WIDTH, T_INF_COMPUTE);
        $display("==============================================================");

        a1    = pack_ann_addr(2'd3, 4'd15, 4'd15, 6'd63);
        a2    = pack_ann_addr(2'd1, 4'd2,  4'd5,  6'd7);
        a_inf = pack_ann_addr(2'd2, 4'd0,  4'd0,  6'd0);

        rst_n = 0;
        idle_cycles(10);
        rst_n = 1;
        idle_cycles(10);

        //---------------------------------------------------------------- T1
        $display("\n[T1] Idle after reset");
        check(ann_core_word === '0,   "ann_core_word = 0 in idle");
        check(pulses === P_HIZ,       "pulses = HIZ in idle");
        check(ann_input_bits === '0,  "ann_input_bits = 0 in idle");
        check(col_sel === '0,         "col_sel = 0 in idle");
        check(!dut.ctrl_busy,         "controller not busy");
        check(dsa_clear === 0 && start === 0 && act_start === 0, "datapath strobes low");

        //---------------------------------------------------------------- T2
        $display("\n[T2] PROG over SPI: PE3 SA15 ROW15 COL63, weight 0xA (addr 0x%04h)", a1);
        beats0 = ctrl_beats; frames0 = host_frames; p0 = prog_cyc; r0 = read_cyc;
        exp_addr = a1; exp_payload = 8'h0A; watch_prog = 1;
        prog_word_err = 0; prog_sel_err = 0;
        ops0 = ops_done;
        spi_send(build_host_ann_word(8'h0A, a1), CMD_PROG);
        wait_op_done("PROG #1", ops0);
        watch_prog = 0;
        check(host_frames - frames0 == 1, "exactly one host frame received");
        check(ctrl_beats  - beats0  == 1, "exactly one controller beat (no repeats)");
        check(prog_cyc - p0 > 0,          $sformatf("PROG pulses applied (%0d cycles)", prog_cyc - p0));
        check(read_cyc - r0 > 0,          $sformatf("VERIFY read pulses applied (%0d cycles)", read_cyc - r0));
        check(prog_word_err == 0,         "ann_core_word = {0x0A, addr, 0x00} during PROG");
        check(prog_sel_err == 0,          "pe_sel = 3 and row_sel = 15 during PROG");
        check(yf_cell[a1] == 4'hA,           $sformatf("mock yf_cell[0x%04h] = 0x%0h (expected 0xA)", a1, yf_cell[a1]));

        //---------------------------------------------------------------- T3
        $display("\n[T3] PROG over SPI: PE1 SA2 ROW5 COL7, weight 0x3 (addr 0x%04h)", a2);
        beats0 = ctrl_beats;
        exp_addr = a2; exp_payload = 8'h03; watch_prog = 1;
        prog_word_err = 0; prog_sel_err = 0;
        ops0 = ops_done;
        spi_send(build_host_ann_word(8'h03, a2), CMD_PROG);
        wait_op_done("PROG #2", ops0);
        watch_prog = 0;
        check(ctrl_beats - beats0 == 1,   "exactly one controller beat");
        check(prog_word_err == 0,         "ann_core_word = {0x03, addr, 0x00} during PROG");
        check(prog_sel_err == 0,          "pe_sel = 1 and row_sel = 5 during PROG");
        check(yf_cell[a2] == 4'h3,           $sformatf("mock yf_cell[0x%04h] = 0x%0h (expected 0x3)", a2, yf_cell[a2]));
        check(yf_cell[a1] == 4'hA,           "first cell unchanged");

        //---------------------------------------------------------------- T4
        $display("\n[T4] READ over SPI at addr 0x%04h", a1);
        beats0 = ctrl_beats; r0 = read_cyc;
        ops0 = ops_done;
        last_read_addr = '1; last_read_data = '0;
        spi_send(build_host_ann_word(8'h00, a1), CMD_READ);
        wait_op_done("READ", ops0);
        check(last_read_addr === a1, $sformatf("READ addressed 0x%04h on the core bus", last_read_addr));
        check(last_read_data === 4'hA, $sformatf("mock ADC returned 0x%0h for that cell (expected 0xA)", last_read_data));
        check(ctrl_beats - beats0 == 1,   "exactly one controller beat");
        check(read_cyc - r0 > 0,          $sformatf("READ pulses applied (%0d cycles)", read_cyc - r0));

        //---------------------------------------------------------------- T5
        $display("\n[T5] ERASE over SPI at addr 0x%04h", a1);
        beats0 = ctrl_beats; e0 = erase_cyc;
        ops0 = ops_done;
        spi_send(build_host_ann_word(8'h00, a1), CMD_ERASE);
        wait_op_done("ERASE", ops0);
        check(ctrl_beats - beats0 == 1,   "exactly one controller beat");
        check(erase_cyc - e0 > 0,         $sformatf("ERASE pulses applied (%0d cycles)", erase_cyc - e0));
        check(yf_cell[a1] == 4'h0,           "erased cell = 0");
        check(yf_cell[a2] == 4'h3,           "other cell untouched (0x3)");

        //---------------------------------------------------------------- T6
        $display("\n[T6] PROG frame with reserved bits != 0 is rejected");
        beats0 = ctrl_beats; frames0 = host_frames; p0 = prog_cyc;
        spi_send(build_host_ann_word(8'h05, a2) | 32'h0000_0001, CMD_PROG);
        idle_cycles(200);
        check(host_frames - frames0 == 1, "frame received by SPI slave");
        check(ctrl_beats - beats0 == 0,   "no controller beat (parallel_interface valid = 0)");
        check(!dut.ctrl_busy && prog_cyc == p0, "no PROG activity");
        check(yf_cell[a2] == 4'h3,           "cell unchanged");

        //---------------------------------------------------------------- T7
        $display("\n[T7] HIZ frame does nothing");
        beats0 = ctrl_beats; frames0 = host_frames;
        spi_send(build_host_ann_word(8'hFF, a2), CMD_HIZ);
        idle_cycles(200);
        check(host_frames - frames0 == 1, "frame received by SPI slave");
        check(ctrl_beats - beats0 == 0,   "no controller beat");
        check(pulses === P_HIZ && !dut.ctrl_busy, "controller idle");
        check(miso_rx === 35'h0, $sformatf("MISO response before any inference = 0 (got 0x%09h)", miso_rx));

        //---------------------------------------------------------------- T8
        $display("\n[T8] INF over SPI: start frame + %0d pixel frames, column sweep, MSB first", INPUT_VECTOR_LEN);
        load_random_weights(17);
        for (int k = 0; k < INPUT_VECTOR_LEN; k++) pix[k] = 8'(k * 37 + 11) ^ 8'h5A;
        compute_reference();
        beats0 = ctrl_beats; frames0 = host_frames; i0 = inf_cyc; ops0 = ops_done;
        clear_inf_monitor();
        watch_inf = 1;

        spi_send(build_host_ann_word(8'h00, a_inf), CMD_INF);         // start beat
        for (int k = 0; k < INPUT_VECTOR_LEN; k++)
            spi_send(build_host_ann_word(pix[k], a_inf), CMD_INF);    // pixel beats
        check(miso_rx[1] === 1'b1 && miso_rx[0] === 1'b0,
              $sformatf("MISO status during collection: busy=1 valid=0 (got busy=%0b valid=%0b)",
                        miso_rx[1], miso_rx[0]));
        wait_op_done("INF", ops0);
        idle_cycles(20);
        watch_inf = 0;

        check(host_frames - frames0 == INPUT_VECTOR_LEN + 1,
              $sformatf("%0d frames received", host_frames - frames0));
        check(ctrl_beats - beats0 == INPUT_VECTOR_LEN + 1,
              $sformatf("%0d controller beats (1 start + %0d pixels)", ctrl_beats - beats0, INPUT_VECTOR_LEN));
        begin
            int buf_err;
            buf_err = 0;
            for (int k = 0; k < INPUT_VECTOR_LEN; k++)
                if (dut.u_input_buffer.buffer_reg[k] !== pix[k]) buf_err++;
            check(buf_err == 0, $sformatf("all %0d input-buffer entries hold their pixel (%0d errors)",
                                          INPUT_VECTOR_LEN, buf_err));
        end
        check(inf_cyc - i0 == NUM_COL_POS * READ_CYC_PER_COL,
              $sformatf("INF pulses for %0d cycles (expected %0d)", inf_cyc - i0, NUM_COL_POS * READ_CYC_PER_COL));
        check_inference("INF #1");
        check(!dut.ctrl_busy,             "controller back to idle");

        //---------------------------------------------------------------- T9
        $display("\n[T9] irq, MISO readback of the 256 outputs (64 words), PISO stream");
        idle_cycles(50);
        check(irq_cnt == 1, $sformatf("irq raised once after the inference (%0d)", irq_cnt));
        idle_cycles(500);
        check(irq === 1'b1, "irq is a level: still high 500 cycles later (not yet read)");
        spi_send(build_host_ann_word(8'h01, '0), CMD_HIZ); // returns word 0, selects word 1
        check(irq === 1'b0, "irq cleared by the read frame");
        check(miso_rx === {exp_out[31:0], 3'b001},
              $sformatf("word 0 = 0x%08h status %03b (expected 0x%08h / 001)",
                        miso_rx[34:3], miso_rx[2:0], exp_out[31:0]));
        read_result();
        check(rd_result === exp_out, "full 2048-bit result over MISO = reference");
        check(piso_n == OUTPUT_BITS && piso_bits === exp_out,
              $sformatf("PISO streamed %0d bits = output buffer, byte 255 bit 7 first", piso_n));
        check(rd_status === 3'b001, "status error=0 busy=0 valid=1");
        read_result();
        check(rd_result === exp_out, "second read returns the same result (sticky)");
        check(!dut.ctrl_busy, "read frames do not start a controller operation");

        //---------------------------------------------------------------- T10
        $display("\n[T10] Second INF: new pixels, the new 256 outputs replace the old ones");
        prev_out = exp_out;
        for (int k = 0; k < INPUT_VECTOR_LEN; k++) pix[k] = (k == INPUT_VECTOR_LEN - 1) ? 8'hFF : 8'(k);
        compute_reference();
        check(exp_out !== prev_out, "reference changes with the new pixels");
        ops0 = ops_done; rv_clear_ok = 0; rv_clear_bad = 0;
        clear_inf_monitor();
        watch_inf = 1;
        spi_send(build_host_ann_word(8'h00, a_inf), CMD_INF);
        for (int k = 0; k < INPUT_VECTOR_LEN - 1; k++)
            spi_send(build_host_ann_word(pix[k], a_inf), CMD_INF);
        piso_n = 0;
        check(miso_rx === {prev_out[31:0], 3'b011},
              "during collection MISO still returns the old result word 0 (busy=1 valid=1)");
        spi_send(build_host_ann_word(pix[INPUT_VECTOR_LEN-1], a_inf), CMD_INF);   // last pixel -> inf_start
        wait_op_done("INF #2", ops0);
        idle_cycles(20);
        watch_inf = 0;
        check(rv_clear_ok == 1 && rv_clear_bad == 0, "result_valid cleared by inf_start");
        check_inference("INF #2");
        idle_cycles(50);
        check(irq_cnt == 2 && irq === 1'b1, "irq raised (level) for the second inference");
        read_result();
        check(irq === 1'b0, "irq cleared by the read frames");
        check(rd_result === exp_out && rd_status === 3'b001,
              $sformatf("MISO returns the new 2048-bit result, status %03b", rd_status));
        restore_weights();

        //---------------------------------------------------------------- T11
        $display("\n[T11] SPI frame aborted after 20 bits, then a normal PROG frame");
        a3 = pack_ann_addr(2'd0, 4'd9, 4'd3, 6'd42);
        beats0 = ctrl_beats; frames0 = host_frames;
        spi_send_partial(build_host_ann_word(8'h0F, a2), CMD_ERASE, 20);
        idle_cycles(100);
        check(host_frames == frames0 && ctrl_beats == beats0, "aborted frame produced no beat");
        ops0 = ops_done;
        spi_send(build_host_ann_word(8'h06, a3), CMD_PROG);
        wait_op_done("PROG after abort", ops0);
        check(host_frames - frames0 == 1 && ctrl_beats - beats0 == 1,
              "next frame received as exactly one beat (bit counter realigned)");
        check(yf_cell[a3] == 4'h6, $sformatf("cell[0x%04h] = 0x%0h (expected 0x6)", a3, yf_cell[a3]));
        check(yf_cell[a2] == 4'h3, "cell named in the aborted ERASE frame untouched");

        //---------------------------------------------------------------- T12
        $display("\n[T12] Stuck cell: PROG gives up -> prog_fail reported on MISO");
        a_stuck = pack_ann_addr(2'd2, 4'd7, 4'd12, 6'd5);
        stuck_en = 1;
        ops0 = ops_done; e0 = erase_cyc; p0 = prog_cyc;
        spi_send(build_host_ann_word(8'h07, a_stuck), CMD_PROG);
        wait_op_done("PROG on stuck cell (must terminate)", ops0);
        check(erase_cyc - e0 > 0, $sformatf("ERASE recovery attempted (%0d erase cycles, %0d prog cycles)",
                                             erase_cyc - e0, prog_cyc - p0));
        check(dut.ctrl_prog_fail === 1'b1, "controller prog_fail set");
        spi_send(build_host_ann_word(8'h00, '0), CMD_HIZ);
        check(miso_rx[2] === 1'b1, $sformatf("MISO status bit2 (prog_fail) = %0b", miso_rx[2]));
        stuck_en = 0;
        ops0 = ops_done;
        spi_send(build_host_ann_word(8'h09, a2), CMD_PROG);
        wait_op_done("good PROG after failure", ops0);
        check(dut.ctrl_prog_fail === 1'b0 && yf_cell[a2] == 4'h9, "next good PROG clears prog_fail");
        spi_send(build_host_ann_word(8'h00, '0), CMD_HIZ);
        check(miso_rx[2] === 1'b0, "MISO prog_fail bit cleared");

        //---------------------------------------------------------------- T13
        $display("\n[T13] Reset asserted in the middle of a PROG");
        ops0 = ops_done;
        fork
            spi_send(build_host_ann_word(8'h0C, a3), CMD_PROG);
            begin
                wait (dut.ctrl_busy === 1'b1);
                idle_cycles(3);
                #2ns rst_n = 0;               // asynchronous assertion
                #1ns;
                check(dut.rst_n_clk === 1'b0, "rst_n_clk asserted immediately (async)");
                idle_cycles(4);
                #2ns rst_n = 1;               // release between clock edges
                @(posedge clk); #1ns;
                check(dut.rst_n_clk === 1'b0, "rst_n_clk still low 1 clk after release");
                @(posedge clk); #1ns;
                check(dut.rst_n_clk === 1'b1, "rst_n_clk released on the 2nd clk edge (synchronised)");
            end
        join
        idle_cycles(20);
        check(!dut.ctrl_busy && pulses === P_HIZ && irq === 1'b0, "after reset: controller idle, irq low");
        spi_send(build_host_ann_word(8'h00, '0), CMD_HIZ);
        check(miso_rx === 35'h0, "after reset: MISO result/status cleared");
        ops0 = ops_done;
        spi_send(build_host_ann_word(8'h0C, a3), CMD_PROG);
        wait_op_done("PROG after reset", ops0);
        check(yf_cell[a3] == 4'hC, "PROG after reset works (cell = 0xC)");

        //---------------------------------------------------------------- T14
        $display("\n[T14] READ result returned on MISO (cell 0x%04h holds 0xC)", a3);
        ops0 = ops_done;
        spi_send(build_host_ann_word(8'h00, a3), CMD_READ);
        wait_op_done("READ", ops0);
        spi_send(build_host_ann_word(8'h00, '0), CMD_HIZ);       // collect the READ result
        check(miso_rx === {8'h00, a3, 8'h0C, 3'b001},
              $sformatf("READ response: addr=0x%04h data=0x%02h status=%03b (expected 0x%04h / 0x0C / 001)",
                        miso_rx[26:11], miso_rx[10:3], miso_rx[2:0], a3));
        spi_send(build_host_ann_word(8'h00, '0), CMD_HIZ);
        check(miso_rx === 35'h0, "following frame returns the inference result again (none since reset)");
        ops0 = ops_done;
        spi_send(build_host_ann_word(8'h00, a2), CMD_READ);
        wait_op_done("READ #2", ops0);
        spi_send(build_host_ann_word(8'h00, '0), CMD_HIZ);
        check(miso_rx[26:11] === a2 && miso_rx[10:3] === 8'h09,
              $sformatf("second READ: addr=0x%04h data=0x%02h (expected 0x%04h / 0x09)",
                        miso_rx[26:11], miso_rx[10:3], a2));

        //---------------------------------------------------------------- T15
        $display("\n[T15] Analog core never answers op_done -> watchdog after %0d cycles", OP_DONE_TO);
        op_done_en = 0;
        ops0 = ops_done; p0 = prog_cyc;
        spi_send(build_host_ann_word(8'h05, a3), CMD_PROG);
        wait_op_done("PROG without op_done (must terminate)", ops0);
        check(dut.ctrl_op_timeout === 1'b1, "controller op_timeout set");
        check(prog_cyc == p0, "no PROG pulses applied (stuck before PROG_WRITE)");
        spi_send(build_host_ann_word(8'h00, '0), CMD_HIZ);
        check(miso_rx[2:1] === 2'b10, $sformatf("MISO error=1 busy=0 (got %02b)", miso_rx[2:1]));
        op_done_en = 1;
        ops0 = ops_done;
        spi_send(build_host_ann_word(8'h05, a3), CMD_PROG);
        wait_op_done("PROG after op_done restored", ops0);
        check(!dut.ctrl_op_timeout && yf_cell[a3] == 4'h5, "next PROG works and clears the error");

        //---------------------------------------------------------------- T16
        $display("\n[T16] Host stops after 10 pixels -> watchdog after %0d idle cycles", COLLECT_TO);
        ops0 = ops_done; i0 = inf_cyc; dsa_clear_cnt = 0;
        spi_send(build_host_ann_word(8'h00, a_inf), CMD_INF);
        for (int k = 0; k < 10; k++)
            spi_send(build_host_ann_word(8'(k), a_inf), CMD_INF);
        check(dut.ctrl_busy === 1'b1, "controller busy collecting");
        wait_op_done("abandoned INF (must terminate)", ops0);
        check(dut.ctrl_op_timeout === 1'b1, "controller op_timeout set");
        check(inf_cyc == i0 && dsa_clear_cnt == 0, "no compute phase, no dsa_clear");
        spi_send(build_host_ann_word(8'h00, '0), CMD_HIZ);
        check(miso_rx[2:1] === 2'b10, $sformatf("MISO error=1 busy=0 (got %02b)", miso_rx[2:1]));
        ops0 = ops_done;
        spi_send(build_host_ann_word(8'h00, a3), CMD_READ);
        wait_op_done("READ after INF abort", ops0);
        check(!dut.ctrl_op_timeout, "next command clears the error");

        //---------------------------------------------------------------- end
        check(ann_reset_seen == 0, "ann_reset never asserted");

        $display("\n==============================================================");
        $display(" SUMMARY: %0d passed, %0d failed", n_pass, n_fail);
        $display(" RESULT: %s", (n_fail == 0) ? "PASS" : "FAIL");
        $display("==============================================================");
        $finish;
    end

    // Global watchdog
    initial begin
        #40ms;
        $display("  FAIL: global timeout");
        $display(" RESULT: FAIL");
        $finish;
    end

endmodule
