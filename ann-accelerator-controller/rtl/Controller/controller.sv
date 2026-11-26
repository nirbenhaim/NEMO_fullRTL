//------------------------------------------------------------------------------
// ANN Controller
//------------------------------------------------------------------------------
// INF flow: S_IDLE --INF--> S_COLLECT_DATA (256 pixels -> input buffer)
//           -> S_COMPUTE (column x bit-plane sweep, T_INF_COMPUTE = 2176 cycles)
//           -> S_RESULT -> S_IDLE
// S_COMPUTE follows NEMO CS-IMC Architecture Brief Rev. 1.0 §6.2-6.3 with the
// bit order MSB first (2026-11-18): for each column position k = 0..15 the
// column mux (col_sel = k) is held while planes b7..b0 are applied, one
// 16-cycle bit-step each, followed by an 8-cycle column boundary.  See the
// schedule constants in controller_pkg.
//------------------------------------------------------------------------------


`include "../common/macros.svh"

import ann_arch_pkg::*;
import controller_pkg::*;
import input_buffer_pkg::*;
import parallel_interface_pkg::*;

module ann_controller #(
    // Packed {PE,SA,ROW,COL} address width; must equal ann_arch_pkg::WEIGHT_ADDR_WIDTH (16)
    parameter int ADDR_WIDTH   = controller_pkg::DEFAULT_ADDR_WIDTH,
    // Weight precision; must be 1..BUFFER_DATA_WIDTH (4 for Y-Flash)
    parameter int WEIGHT_WIDTH = controller_pkg::DEFAULT_WEIGHT_WIDTH,
    parameter bit USE_WEIGHT_PULSE_LUT = 1'b1,
    parameter string WEIGHT_PULSE_LUT_FILE = "target/Controller/programming_inputs/weight_pulse_lut.mem",
    // Watchdogs (clk cycles): abort to S_IDLE and raise op_timeout when exceeded
    parameter int OP_DONE_TIMEOUT = controller_pkg::DEFAULT_OP_DONE_TIMEOUT,  // wait for op_done
    parameter int COLLECT_TIMEOUT = controller_pkg::DEFAULT_COLLECT_TIMEOUT,  // gap between pixel beats
    // Pulse generation configuration
    parameter int TREAD = controller_pkg::TREAD,
    parameter int TPROG = controller_pkg::TPROG,
    parameter int TERASE = controller_pkg::TERASE,
    parameter int TINF = controller_pkg::TINF,
    parameter int PULSE_GAP = controller_pkg::PULSE_GAP,
    parameter int PULSE_NUM_READ = controller_pkg::PULSE_NUM_READ,
    parameter int PULSE_NUM_PROG = controller_pkg::PULSE_NUM_PROG,
    parameter int PULSE_NUM_ERASE = controller_pkg::PULSE_NUM_ERASE,
    // The inference sequencer always processes every column and bit plane.
    parameter int PULSE_NUM_INF = controller_pkg::PULSE_NUM_INF,
    parameter int MAX_PROG_RETRIES = controller_pkg::MAX_PROG_RETRIES
) (
    //======================================================================
    // Global
    //======================================================================
    input  logic                          clk,
    input  logic                          rst_n,

    //======================================================================
    // Controller <-> Parallel Interface
    //======================================================================
    input  logic                          valid,     // One-cycle strobe: new command/data beat
    input  logic [ACT_WIDTH-1:0]          data,      // data byte from parallel interface
    input  logic [ADDR_WIDTH-1:0]         address,   // packed {PE,SA,ROW,COL} from parallel interface
    input  logic [CMD_WIDTH-1:0]          cmd,       // Command from parallel interface

    //======================================================================
    // Controller <-> ANN Core
    //======================================================================
    // ann_address (ann_arch_pkg word format, same as host word):
    //   [31:24] data byte (quantized weight in [27:24] for PROG)
    //   [23:8]  packed binary {PE[1:0], SA[3:0], ROW[3:0], COL[5:0]}
    //   [7:0]   reserved (0)
    // pulses: operation / pulse mode toward core (READ/PROG/ERASE/INF)
    //======================================================================
    output logic                          ann_reset,
    // Core handshake: asserted to advance PROG_SELECT->PROG_WRITE, PROG_WAIT_ACK->PROG_COMPLETE,
    // ERASE_SELECT->ERASE_PULSE, ERASE_WAIT_ACK->ERASE_COMPLETE (see program/erase sub-FSMs).
    input  logic                          op_done,
    output logic [WORD_WIDTH-1:0]         ann_address,
    output logic [2:0]                    pulses,

    // ADC/quantizer input for verification read (from ANN core)
    input  logic [WEIGHT_WIDTH-1:0]       weight_read_data,  // Read weight from memristor during VERIFY

    //======================================================================
    // Controller <-> Input Buffer
    //======================================================================
    output logic [BUFFER_ADDR_WIDTH-1:0]  buf_reg_add,
    output logic [2:0]                    buf_reg_ctrl,     // buffer control signals
    output logic                          buf_read_write,   // 1 = write, 0 = read
    output logic [BUF_BIT_SEL_WIDTH-1:0]  buf_bit_sel,      // bit-plane on the array rows (MSB first: 7 -> 0 per column)
    output logic [BUFFER_DATA_WIDTH-1:0]  buf_data_out,     // Captured data for buffer write (from parallel interface)
    input  logic                          buf_ready,
    input  logic [BUFFER_DATA_WIDTH-1:0]  buf_data,         // Data from input buffer (full byte at current addr)

    //======================================================================
    // Controller -> Digital datapath (adder tree / DSA / ReLU)
    //======================================================================
    output logic                          inf_start,        // 1-cycle: last pixel accepted (a new inference starts)
    output logic                          dsa_clear,        // 1-cycle: first cycle of each column position (16 per inference)
    output logic                          tree_start,       // 1-cycle per bit-step: ADC codes valid, adder tree samples
    output logic [COL_SEL_WIDTH-1:0]      col_sel,          // column-mux select, common to all muxes; held for 8 planes
    output logic                          compute_done,     // 1-cycle: last cycle of the last column (COMPUTE -> RESULT)

    //======================================================================
    // Error status to higher-level
    //======================================================================
    output logic                          prog_fail,        // sticky: last PROG gave up (erase retries exhausted); cleared by next command
    output logic                          op_timeout,       // sticky: last command aborted by a watchdog; cleared by next command

    //======================================================================
    // READ result to higher-level
    //======================================================================
    output logic [WEIGHT_WIDTH-1:0]       read_data,        // weight sampled at the end of the last READ
    output logic                          read_done,        // 1-cycle: read_data updated (end of S_READ)

    //======================================================================
    // Status to higher-level
    //======================================================================
    output logic                          busy
);

    //--------------------------------------------------------------------------
    // Elaboration-time consistency checks
    //--------------------------------------------------------------------------
    localparam int PROG_RETRY_CNT_WIDTH = (MAX_PROG_RETRIES < 1)
                                        ? 1 : $clog2(MAX_PROG_RETRIES + 1);
    localparam int CTRL_PULSE_TOTAL_READ =
        pulse_train_total(TREAD, PULSE_NUM_READ, PULSE_GAP);
    localparam int CTRL_PULSE_TOTAL_PROG =
        pulse_train_total(TPROG, PULSE_NUM_PROG, PULSE_GAP);
    localparam int CTRL_PULSE_TOTAL_ERASE =
        pulse_train_total(TERASE, PULSE_NUM_ERASE, PULSE_GAP);

    initial begin
        if (ADDR_WIDTH != WEIGHT_ADDR_WIDTH)
            $fatal(1, "ann_controller: ADDR_WIDTH (%0d) must equal ann_arch_pkg::WEIGHT_ADDR_WIDTH (%0d)",
                   ADDR_WIDTH, WEIGHT_ADDR_WIDTH);
        if (WEIGHT_WIDTH < 1 || WEIGHT_WIDTH > BUFFER_DATA_WIDTH)
            $fatal(1, "ann_controller: WEIGHT_WIDTH (%0d) must be 1..%0d",
                   WEIGHT_WIDTH, BUFFER_DATA_WIDTH);
        if (BUFFER_ADDR_WIDTH > ADDR_WIDTH)
            $fatal(1, "ann_controller: BUFFER_ADDR_WIDTH (%0d) exceeds ADDR_WIDTH (%0d)",
                   BUFFER_ADDR_WIDTH, ADDR_WIDTH);
        if (TREAD < 1 || TPROG < 1 || TERASE < 1 || TINF < 1 ||
            PULSE_NUM_READ < 1 || PULSE_NUM_PROG < 1 || PULSE_NUM_ERASE < 1 ||
            PULSE_GAP < 0)
            $fatal(1, "ann_controller: pulse widths/counts must be positive and PULSE_GAP non-negative");
        if (CTRL_PULSE_TOTAL_READ > 255 || CTRL_PULSE_TOTAL_PROG > 255 ||
            CTRL_PULSE_TOTAL_ERASE > 255)
            $fatal(1, "ann_controller: READ/PROG/ERASE pulse trains must fit the 8-bit pulse counter");
        if (TINF + T_INF_TREE + T_INF_SHIFT + T_INF_DRIVE > 255)
            $fatal(1, "ann_controller: TINF exceeds the inference cycle counter range");
        if (PULSE_NUM_INF != NUM_COL_POS * ACT_WIDTH)
            $fatal(1, "ann_controller: PULSE_NUM_INF (%0d) must match the fixed inference schedule (%0d)",
                   PULSE_NUM_INF, NUM_COL_POS * ACT_WIDTH);
        if (MAX_PROG_RETRIES < 0)
            $fatal(1, "ann_controller: MAX_PROG_RETRIES must be non-negative");
    end

    //--------------------------------------------------------------------------
    // FSM States (using package type)
    //--------------------------------------------------------------------------
    controller_state_t state, next_state;
    prog_sequence_state_t prog_state, next_prog_state;

    //--------------------------------------------------------------------------
    // VERIFY sub-FSM
    //--------------------------------------------------------------------------
    typedef enum logic [1:0] {
        VERIFY_IDLE,
        VERIFY_WAIT,
        VERIFY_CHECK,
        VERIFY_DONE
    } verify_state_t;

    verify_state_t verify_state, verify_next;

    erase_state_t erase_state, erase_next;

    //--------------------------------------------------------------------------
    // VERIFY and ERASE sub-FSM Signals
    //--------------------------------------------------------------------------
    // Retry counter: ERASE rounds in verify-failure recovery (0..3); abort when >= 3
    logic [1:0] retry_cnt;

    // Re-program counter: PROG retries without ERASE when read < expected (0..MAX_PROG_RETRIES)
    logic [PROG_RETRY_CNT_WIDTH-1:0] prog_retry_cnt;

    // S_VERIFY read pulse train: VERIFY_IDLE is cycle 0; VERIFY_WAIT uses verify_pulse_idx 1..PULSE_TOTAL_READ-1
    logic [7:0] verify_pulse_idx;

    // Outputs (or internal signals connected to outputs)
    logic        weight_read_en;      // enable read in VERIFY (internal; use weight_read_data input port)
    logic [WEIGHT_WIDTH-1:0] expected_weight;     // from buffer/host
    // VERIFY_CHECK: set when mismatch forces entry to S_ERASE (max reprog or over-programmed)
    logic        verify_failure_starts_erase;
    // erase_failure_flag: pulsed for one cycle in ERASE_COMPLETE when verify-failure
    // recovery has exhausted erase retries (retry_cnt >= 3) and the FSM returns to S_IDLE.
    logic        erase_failure_flag;

    // Asserted when under-programmed verify requests a stronger re-program pulse
    logic        program_stronger;

    // Host CMD_ERASE from S_IDLE: after erase sub-FSM, return to idle (no PROG/VERIFY).
    // Verify-failure path clears this when entering S_ERASE from S_VERIFY.
    logic        erase_from_host;

    //--------------------------------------------------------------------------
    // Direct Address and Data from Parallel Interface (for new commands)
    //--------------------------------------------------------------------------
    logic [ADDR_WIDTH-1:0] address_reg;   // Current packed address from parallel interface
    logic [ACT_WIDTH-1:0]  data_reg;      // Current data from parallel interface (captured with address)
    logic [WEIGHT_ADDR_WIDTH-1:0] weight_addr_reg;  // Parsed ANN address

    // Address parsing outputs (binary PE / SA / row / col indices; debug/waves)
    logic [PE_ID_WIDTH-1:0]      pe_id;
    logic [SA_ID_WIDTH-1:0]      sa_id;
    logic [ROW_ID_WIDTH-1:0]     row_id;
    logic [COL_ID_WIDTH-1:0]     col_id;

    // Scratch input-buffer slot for single-weight PROG/VERIFY (low bits of packed address)
    logic [BUFFER_ADDR_WIDTH-1:0] prog_buf_addr;
    assign prog_buf_addr = address_reg[BUFFER_ADDR_WIDTH-1:0];

    //--------------------------------------------------------------------------
    // Data Collection Tracking for INF Command
    //--------------------------------------------------------------------------
    // INF: one start beat (S_IDLE) followed by INPUT_VECTOR_LEN pixel beats;
    // pixel k is written to input-buffer entry k (= SA k/SA_ROWS, row k%SA_ROWS).
    logic [BUFFER_ADDR_WIDTH-1:0] pixel_cnt;   // next input-buffer entry (0..INPUT_VECTOR_LEN-1)
    logic                         last_pixel;  // pixel_cnt points at the last entry

    assign last_pixel = (pixel_cnt == INPUT_VECTOR_LEN - 1);

    //--------------------------------------------------------------------------
    // S_COMPUTE sequencer: column position (outer) x bit-step (inner, MSB first)
    //--------------------------------------------------------------------------
    localparam int INF_STEP_CYCLES = TINF + T_INF_TREE + T_INF_SHIFT + T_INF_DRIVE;
    localparam int INF_CYC_PARAM_WIDTH =
        $clog2((INF_STEP_CYCLES > T_COL_BND) ? INF_STEP_CYCLES : T_COL_BND);

    logic [COL_SEL_WIDTH-1:0]     col_cnt;     // current column position k (0..15)
    logic [BUF_BIT_SEL_WIDTH-1:0] step_cnt;    // bit-step within the column (0..7); plane = 7 - step_cnt
    logic [INF_CYC_PARAM_WIDTH-1:0] inf_cyc;   // cycle within the bit-step or the boundary
    logic                         inf_bnd;     // 1 = in the column boundary (after the 8th bit-step)

    logic [BUF_BIT_SEL_WIDTH-1:0] cur_plane;   // plane of the current bit-step
    logic [BUF_BIT_SEL_WIDTH-1:0] next_plane;  // plane driven in the last cycle of the bit-step
    logic                         in_compute;
    logic                         step_last;   // last cycle of a bit-step
    logic                         bnd_last;    // last cycle of a column boundary
    logic                         col_last;    // current column is the last one
    logic                         compute_last;// last cycle of S_COMPUTE
    logic                         inf_read;    // array being read (settle + conversion)

    localparam logic [BUF_BIT_SEL_WIDTH-1:0] PLANE_MSB = BUF_BIT_SEL_WIDTH'(ACT_WIDTH - 1);

    assign in_compute   = (state == S_COMPUTE);
    assign cur_plane    = PLANE_MSB - step_cnt;                          // 7, 6, ..., 0
    assign next_plane   = (step_cnt == PLANE_MSB) ? PLANE_MSB            // after b0: b7 of the next column
                                                  : cur_plane - 1'b1;
    assign step_last    = !inf_bnd && (inf_cyc == INF_CYC_PARAM_WIDTH'(INF_STEP_CYCLES - 1));
    assign bnd_last     =  inf_bnd && (inf_cyc == INF_CYC_PARAM_WIDTH'(T_COL_BND - 1));
    assign col_last     = (col_cnt == COL_SEL_WIDTH'(NUM_COL_POS - 1));
    assign compute_last = in_compute && bnd_last && col_last;
    assign inf_read     = in_compute && !inf_bnd && (inf_cyc < INF_CYC_PARAM_WIDTH'(TINF));

    //--------------------------------------------------------------------------
    // Pulse Timing (per spec: T*N cycles per mode)
    //--------------------------------------------------------------------------
    logic [7:0] pulse_cnt;     // Cycle counter for pulse duration
    logic       pulse_done;    // Asserted when pulse_cnt reaches total for current mode
    logic [7:0] pulse_total;   // Total cycles for current mode (T*N or max(8, T*N) for INF)

    //--------------------------------------------------------------------------
    // Weight Count Detection and Tracking (kept for ERASE/VERIFY compatibility)
    //--------------------------------------------------------------------------
    logic [WEIGHT_COUNT_WIDTH-1:0] weight_count_reg;  // Legacy: number of weights to program (default 640)
    logic                           weight_count_valid;  // Weight count is ready
    logic [WEIGHT_COUNT_WIDTH-1:0] buffer_idx_reg;  // Current buffer index (0 to weight_count-1)
    
    // Matrix dimensions (for mapping algorithm - kept for ERASE/VERIFY)
    logic [5:0] matrix_rows;  // Number of rows (default: 10)
    logic [6:0] matrix_cols;  // Number of columns (default: 64)
    logic                           weight_prog_done;       // All weights programmed flag

    // Buffer address and weight selection (kept for ERASE/VERIFY)
    logic [BUFFER_ADDR_WIDTH-1:0]  buf_addr_reg;
    // Selects lower (0) or upper (1) weight field of buffer byte via buffer_idx_reg[0]
    logic                           weight_sel;

    // Weight data extraction from buffer
    logic [WEIGHT_WIDTH-1:0] weight_from_buffer;

    // Expected weight for verification (from buffer)
    assign expected_weight = weight_from_buffer;


    // One LUT entry per weight level (2^WEIGHT_WIDTH = 16 for 4-bit weights)
    logic [7:0] weight_pulse_cycles_lut [0:(1<<WEIGHT_WIDTH)-1];

    initial
        $readmemh(WEIGHT_PULSE_LUT_FILE, weight_pulse_cycles_lut);

    // LUT row for expected_weight: repeat count for the (TPROG x PULSE_NUM_PROG) macro train
    wire [7:0] lut_entry_expected_weight;
    assign lut_entry_expected_weight = weight_pulse_cycles_lut[expected_weight];

    //--------------------------------------------------------------------------
    // Address Parsing: Direct address from parallel interface (all commands)
    //--------------------------------------------------------------------------
    `comb(
        // All commands use direct address from host packet
        parse_ann_address(address_reg, pe_id, sa_id, row_id, col_id);
        weight_addr_reg = address_reg;
    )

    //--------------------------------------------------------------------------
    // Packed ANN core bus: host data byte + packed binary address
    //--------------------------------------------------------------------------
    logic [ACT_WIDTH-1:0] data_byte_for_ann;
    always_comb begin
        data_byte_for_ann = '0;
        unique case (state)
            S_PROGRAM,
            S_VERIFY,
            S_READ,
            S_ERASE: data_byte_for_ann = data_reg;
            S_COLLECT_DATA: data_byte_for_ann = data;
            default: data_byte_for_ann = '0;
        endcase
    end

    always_comb begin
        if (state == S_IDLE)
            ann_address = '0;
        else
            ann_address = pack_ann_word(data_byte_for_ann, weight_addr_reg);
    end

    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    // Pulse total and done (burst train: N bursts of T cycles, (N-1) gaps of PULSE_GAP)
    //--------------------------------------------------------------------------
    `comb(
        automatic int Tp;
        automatic int Np;
        pulse_total = 8'd0;
        if (state == S_READ)
            pulse_total = CTRL_PULSE_TOTAL_READ[7:0];
        else if (state == S_PROGRAM && prog_state == PROG_WRITE) begin
            if (!USE_WEIGHT_PULSE_LUT) begin
                Tp = TPROG;
                Np = PULSE_NUM_PROG;
                pulse_total = pulse_train_total(Tp, Np, PULSE_GAP)[7:0];
            end else if (prog_retry_cnt > 2'd0) begin
                Tp = 1;
                Np = 1;
                pulse_total = pulse_train_total(Tp, Np, PULSE_GAP)[7:0];
            end else begin
                // First PROG: R copies of macro train, PULSE_GAP HIZ between copies (R from LUT)
                automatic int Rlut;
                automatic int Mmacro;
                Mmacro = pulse_train_total(TPROG, PULSE_NUM_PROG, PULSE_GAP);
                Rlut = int'(weight_pulse_cycles_lut[expected_weight]);
                if (Rlut < 1)
                    Rlut = 1;
                pulse_total = 8'(pulse_lut_macro_repeat_total(Rlut, Mmacro, PULSE_GAP));
            end
        end
        else if (state == S_ERASE && erase_state == ERASE_PULSE)
            pulse_total = CTRL_PULSE_TOTAL_ERASE[7:0];
        // S_COMPUTE has its own sequencer (col_cnt / step_cnt / inf_cyc)

        pulse_done = (pulse_total > 8'd0) && (pulse_cnt >= pulse_total - 1'b1);
    )

    //--------------------------------------------------------------------------
    // 3-bit Pulse Output to ANN Core (pulsed: active mode only during burst high phases)
    //--------------------------------------------------------------------------
    `comb(
        automatic int Tv;
        automatic int Tpp;
        automatic int Npp;
        automatic logic rd_on;
        automatic logic pg_on;
        automatic logic er_on;
        pulses = 3'b000;
        // S_READ
        if (state == S_READ) begin
            rd_on = pulse_train_active(int'(pulse_cnt), TREAD, PULSE_NUM_READ, PULSE_GAP);
            pulses = rd_on ? PULSE_MODE_READ : PULSE_MODE_HIZ;
        end
        else if (state == S_PROGRAM && prog_state == PROG_WRITE) begin
            if (!USE_WEIGHT_PULSE_LUT) begin
                Tpp = TPROG;
                Npp = PULSE_NUM_PROG;
                pg_on = pulse_train_active(int'(pulse_cnt), Tpp, Npp, PULSE_GAP);
            end else if (prog_retry_cnt > 2'd0) begin
                Tpp = 1;
                Npp = 1;
                pg_on = pulse_train_active(int'(pulse_cnt), Tpp, Npp, PULSE_GAP);
            end else begin
                automatic int Mmacro;
                automatic int Rlut;
                Mmacro = pulse_train_total(TPROG, PULSE_NUM_PROG, PULSE_GAP);
                Rlut = int'(weight_pulse_cycles_lut[expected_weight]);
                if (Rlut < 1)
                    Rlut = 1;
                pg_on = pulse_lut_macro_repeat_active(int'(pulse_cnt), Rlut, Mmacro, TPROG, PULSE_NUM_PROG, PULSE_GAP);
            end
            pulses = pg_on ? PULSE_MODE_PROG : PULSE_MODE_HIZ;
        end
        else if (state == S_ERASE && erase_state == ERASE_PULSE) begin
            er_on = pulse_train_active(int'(pulse_cnt), TERASE, PULSE_NUM_ERASE, PULSE_GAP);
            pulses = er_on ? PULSE_MODE_ERASE : PULSE_MODE_HIZ;
        end
        else if (state == S_VERIFY && (verify_state == VERIFY_IDLE || verify_state == VERIFY_WAIT)) begin
            Tv = (verify_state == VERIFY_IDLE) ? 0 : int'(verify_pulse_idx);
            rd_on = pulse_train_active(Tv, TREAD, PULSE_NUM_READ, PULSE_GAP);
            pulses = rd_on ? PULSE_MODE_READ : PULSE_MODE_HIZ;
        end
        else if (state == S_COMPUTE) begin
            // INF while the array is read in each bit-step (settle + conversion)
            pulses = inf_read ? PULSE_MODE_INF : PULSE_MODE_HIZ;
        end
    )
    
    //--------------------------------------------------------------------------
    // Buffer Address and Weight Selection Mapping
    //--------------------------------------------------------------------------
    `comb(
        // For PROG/VERIFY: use prog_buf_addr; else use buffer_idx for legacy
        buf_addr_reg = (state == S_PROGRAM || state == S_VERIFY) ? prog_buf_addr : buffer_idx_reg[BUFFER_ADDR_WIDTH:1];
        weight_sel   = buffer_idx_reg[0];

        // Extract weight data from buffer
        // For PROG and VERIFY (direct address): weight in buf_data[WEIGHT_WIDTH-1:0] at prog_buf_addr
        if (state == S_PROGRAM || state == S_VERIFY) begin
            weight_from_buffer = buf_data[WEIGHT_WIDTH-1:0];
        end else begin
            // Legacy: use weight_sel for buffer_idx mapping
            if (weight_sel == 1'b0) begin
                weight_from_buffer = buf_data[WEIGHT_WIDTH-1:0];
            end else begin
                weight_from_buffer = buf_data[BUFFER_DATA_WIDTH-1 -: WEIGHT_WIDTH];
            end
        end
    )

    //--------------------------------------------------------------------------
    // Buffer Data Output Assignment
    //--------------------------------------------------------------------------
    // Output captured data_reg for PROG command, or current data for INF command
    `comb(
        // For PROG state: use captured data_reg (written during PROG_HIZ/PROG_SELECT)
        // For INF state (S_COLLECT_DATA): use current data from parallel interface
        if (state == S_PROGRAM) begin
            buf_data_out = data_reg;  // Use captured data for PROG
        end else begin
            buf_data_out = data;  // Use current data for INF (S_COLLECT_DATA)
        end
    )

    //--------------------------------------------------------------------------
    // Digital datapath strobes (adder tree / DSA / ReLU)
    //--------------------------------------------------------------------------
    // inf_start   : the cycle the last pixel is accepted (a new inference starts;
    //               the top level invalidates the previous result).
    // dsa_clear   : first cycle of every column position.  The DSA has one
    //               accumulator per tree (4 per PE), reused for each column,
    //               so it is cleared 16 times per inference, before the first
    //               tree result of the column (cycle 14).
    // tree_start  : cycle TINF of every bit-step, when the ADC
    //               codes of the current plane are valid (128 per inference).
    // col_sel     : column position k for the whole column; during the last
    //               T_COL_ADV boundary cycles it already shows k+1 so the mux
    //               switches and the array settles before the next column.
    // compute_done: last cycle of S_COMPUTE.
    assign inf_start    = (state == S_COLLECT_DATA) && valid && last_pixel;
    assign dsa_clear    = in_compute && !inf_bnd && (step_cnt == '0) && (inf_cyc == '0);
    assign tree_start   = in_compute && !inf_bnd && (inf_cyc == INF_CYC_PARAM_WIDTH'(TINF));
    assign col_sel      = !in_compute ? '0 :
                          (inf_bnd && !col_last &&
                           inf_cyc >= INF_CYC_PARAM_WIDTH'(T_COL_ACT + T_COL_WB)) ? col_cnt + 1'b1 : col_cnt;
    assign compute_done = compute_last;

    //--------------------------------------------------------------------------
    // Watchdogs: op_done handshakes and pixel-beat gaps
    //--------------------------------------------------------------------------
    localparam int TIMEOUT_MAX = (OP_DONE_TIMEOUT > COLLECT_TIMEOUT) ? OP_DONE_TIMEOUT : COLLECT_TIMEOUT;
    localparam int TIMEOUT_W   = $clog2(TIMEOUT_MAX + 1);

    logic                 wait_op_done_st;   // in a state that waits for op_done
    logic                 wait_pixel_st;     // collecting pixels, waiting for a beat
    logic [TIMEOUT_W-1:0] wd_cnt;            // cycles spent waiting
    logic                 timeout_hit;       // abort this cycle

    assign wait_op_done_st = (state == S_PROGRAM && (prog_state  == PROG_SELECT  || prog_state  == PROG_WAIT_ACK)) ||
                             (state == S_ERASE   && (erase_state == ERASE_SELECT || erase_state == ERASE_WAIT_ACK));
    assign wait_pixel_st   = (state == S_COLLECT_DATA);

    assign timeout_hit = (wait_op_done_st && !op_done && (wd_cnt >= OP_DONE_TIMEOUT - 1)) ||
                         (wait_pixel_st   && !valid   && (wd_cnt >= COLLECT_TIMEOUT - 1));

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wd_cnt <= '0;
        else if ((wait_op_done_st && !op_done) || (wait_pixel_st && !valid))
            wd_cnt <= timeout_hit ? '0 : wd_cnt + 1'b1;
        else
            wd_cnt <= '0;                     // progress made (or not waiting)
    end

    //--------------------------------------------------------------------------
    // Error status (sticky until the next command is accepted in S_IDLE)
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prog_fail  <= 1'b0;
            op_timeout <= 1'b0;
        end else begin
            if (state == S_IDLE && valid && cmd != CMD_HIZ) begin
                prog_fail  <= 1'b0;
                op_timeout <= 1'b0;
            end
            if (erase_failure_flag) prog_fail  <= 1'b1;
            if (timeout_hit)        op_timeout <= 1'b1;
        end
    end

    //--------------------------------------------------------------------------
    // READ result: weight sampled in the last cycle of the READ pulse train
    // (same sampling assumption as VERIFY: weight_read_data valid by then)
    //--------------------------------------------------------------------------
    assign read_done = (state == S_READ) && pulse_done;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            read_data <= '0;
        else if (read_done)
            read_data <= weight_read_data;
    end

    //--------------------------------------------------------------------------
    // State register, address tracking, and data collection
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= S_IDLE;
            prog_state         <= PROG_HIZ;
            address_reg        <= '0;
            data_reg           <= '0;
            pixel_cnt          <= '0;
            col_cnt            <= '0;
            step_cnt           <= '0;
            inf_cyc            <= '0;
            inf_bnd            <= 1'b0;
            pulse_cnt          <= '0;

            // Keep these for ERASE/VERIFY compatibility
            buffer_idx_reg     <= '0;
            weight_count_reg   <= DEFAULT_NUM_WEIGHTS;  // Default: 640 weights
            weight_count_valid <= 1'b0;
            weight_prog_done   <= 1'b0;
            matrix_rows        <= DEFAULT_MATRIX_ROWS;  // Default: 10 rows
            matrix_cols        <= DEFAULT_MATRIX_COLS;  // Default: 64 columns
            erase_from_host    <= 1'b0;
        end else begin
            state <= next_state;
            prog_state <= next_prog_state;

            // Host-initiated erase vs verify-driven erase
            if (state == S_VERIFY && next_state == S_ERASE)
                erase_from_host <= 1'b0;
            else if (state == S_IDLE && valid && cmd == CMD_ERASE)
                erase_from_host <= 1'b1;
            else if (state == S_ERASE && next_state == S_IDLE)
                erase_from_host <= 1'b0;
            
            // Capture address and data from parallel interface when valid command arrives
            if (valid && state == S_IDLE) begin
                address_reg <= address;
                data_reg <= data;  // Capture data along with address for PROG command
            end
            
            // Data collection counter for INF command (S_COLLECT_DATA state)
            if (state == S_COLLECT_DATA) begin
                if (valid) begin
                    // One pixel per valid beat; wrap after the last entry
                    pixel_cnt <= last_pixel ? '0 : pixel_cnt + 1'b1;
                end
            end else if (state == S_IDLE && next_state == S_COLLECT_DATA) begin
                // Initialize counter when entering collection state
                pixel_cnt <= '0;
            end

            // S_COMPUTE sequencer: bit-step cycles -> 8 bit-steps -> boundary
            // -> next column (separate chain so the entry reset is not
            // shadowed by the S_COLLECT_DATA branch above)
            if (state != S_COMPUTE && next_state == S_COMPUTE) begin
                col_cnt  <= '0;
                step_cnt <= '0;
                inf_cyc  <= '0;
                inf_bnd  <= 1'b0;
            end else if (state == S_COMPUTE) begin
                if (step_last) begin                       // end of a bit-step
                    inf_cyc <= '0;
                    if (step_cnt == PLANE_MSB)
                        inf_bnd  <= 1'b1;                  // 8th bit-step done -> boundary
                    else
                        step_cnt <= step_cnt + 1'b1;
                end else if (bnd_last) begin               // end of the column boundary
                    inf_cyc  <= '0;
                    inf_bnd  <= 1'b0;
                    step_cnt <= '0;
                    col_cnt  <= col_last ? '0 : col_cnt + 1'b1;
                end else begin
                    inf_cyc <= inf_cyc + 1'b1;
                end
            end

            // Pulse timing counter: reset on enter, increment until done
            if ((state != S_READ && next_state == S_READ) ||
                (state == S_PROGRAM && prog_state != PROG_WRITE && next_prog_state == PROG_WRITE) ||
                (state == S_ERASE && erase_state != ERASE_PULSE && erase_next == ERASE_PULSE)) begin
                pulse_cnt <= 8'd0;
            end else if (((state == S_READ && next_state == S_READ) ||
                         (state == S_PROGRAM && prog_state == PROG_WRITE && next_prog_state == PROG_WRITE) ||
                         (state == S_ERASE && erase_state == ERASE_PULSE && erase_next == ERASE_PULSE)) && !pulse_done) begin
                pulse_cnt <= pulse_cnt + 1'b1;
            end

            // Keep ERASE/VERIFY logic for compatibility (unchanged)
            // Weight count detection: For now, use default 640 weights
            // TODO: In the future, can count buffer writes or use explicit count signal
            if (state == S_IDLE && next_state == S_RESET) begin
                // Initialize with default values when starting programming
                weight_count_reg <= DEFAULT_NUM_WEIGHTS;
                weight_count_valid <= 1'b1;
                matrix_rows <= DEFAULT_MATRIX_ROWS;
                matrix_cols <= DEFAULT_MATRIX_COLS;
                buffer_idx_reg <= '0;
                weight_prog_done <= 1'b0;
            end
            
            // Reset buffer index when entering RESET state
            if (state == S_IDLE && next_state == S_RESET) begin
                buffer_idx_reg <= '0;
            end
            
            // Advance buffer index after successful verification when
            // transitioning back into PROGRAM state.
            if (state == S_VERIFY && next_state == S_PROGRAM) begin
                if (buffer_idx_reg < (weight_count_reg - 1)) begin
                    buffer_idx_reg <= buffer_idx_reg + 1'b1;
                end else begin
                    weight_prog_done <= 1'b1;
                end
            end
            
            // Check completion during programming (direct address: one weight per PROG command)
            if (state == S_PROGRAM && prog_state == PROG_COMPLETE) begin
                weight_prog_done <= 1'b1;  // Direct address flow: one weight per command
            end
        end
    end

    //--------------------------------------------------------------------------
    // VERIFY, ERASE, and PROGRAM sub-FSM state registers
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            verify_state     <= VERIFY_IDLE;
            erase_state      <= ERASE_HIZ;
            retry_cnt        <= 2'd0;
            prog_retry_cnt   <= 2'd0;
            verify_pulse_idx <= 8'd0;
            program_stronger <= 1'b0;
        end else begin
            verify_state <= verify_next;
            erase_state  <= erase_next;
            // prog_state is updated in the main state register block above

            // -------- retry_cnt and prog_retry_cnt: reset on success (VERIFY_DONE) --------
            if (state == S_VERIFY && verify_state == VERIFY_DONE) begin
                retry_cnt      <= 2'd0;
                prog_retry_cnt <= '0;
                program_stronger <= 1'b0;
            end
            // -------- prog_retry_cnt: increment when re-progging without ERASE --------
            else if (state == S_VERIFY && verify_state == VERIFY_CHECK && weight_read_data < expected_weight && prog_retry_cnt < MAX_PROG_RETRIES) begin
                prog_retry_cnt <= prog_retry_cnt + 1'b1;
            end
            // -------- prog_retry_cnt: reset when going to ERASE (over-programmed or max re-prog exceeded) --------
            else if (state == S_VERIFY && verify_state == VERIFY_CHECK && (weight_read_data > expected_weight || prog_retry_cnt >= MAX_PROG_RETRIES)) begin
                prog_retry_cnt <= '0;
            end
            // -------- retry_cnt: increment on ERASE_COMPLETE; prog_retry_cnt reset --------
            else if (state == S_ERASE && erase_state == ERASE_COMPLETE) begin
                retry_cnt      <= retry_cnt + 1'b1;
                prog_retry_cnt <= '0;
            end
            // -------- all retry state: fresh for every host command --------
            // (The former legacy clear at PROG_COMPLETE was removed: it reset
            //  retry_cnt on every re-program, so erase-retry exhaustion could
            //  never be reached and a cell that never verifies looped forever.)
            else if (state == S_IDLE) begin
                retry_cnt        <= 2'd0;
                prog_retry_cnt   <= '0;
                program_stronger <= 1'b0;
            end

            // -------- verify pulse index (aligned with PULSE_TOTAL_READ burst train) --------
            if (state != S_VERIFY) begin
                verify_pulse_idx <= 8'd0;
            end else if (verify_state == VERIFY_IDLE) begin
                verify_pulse_idx <= 8'd1;
            end else if (verify_state == VERIFY_WAIT && verify_next == VERIFY_WAIT) begin
                verify_pulse_idx <= verify_pulse_idx + 8'd1;
            end

            // -------- program_stronger flag: set in VERIFY_CHECK if needed --------
            if (state == S_VERIFY && verify_state == VERIFY_CHECK) begin
                if (weight_read_data != expected_weight && expected_weight > weight_read_data) begin
                    program_stronger <= 1'b1;
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // Main FSM: outputs + next-state logic
    //--------------------------------------------------------------------------
    `comb(
       
        ann_reset        = 1'b0;


        // Default buffer address (will be overridden in each state)
        buf_reg_add      = buf_addr_reg;
        buf_reg_ctrl     = CTRL_IDLE;
        buf_read_write   = 1'b0;   // 0 = read, 1 = write
        buf_bit_sel      = '0;

        busy             = 1'b0;

        next_state       = state;
        next_prog_state  = prog_state;
        // Sub-FSMs hold by default (prevents latches outside their parent state)
        verify_next      = verify_state;
        erase_next       = erase_state;
        weight_read_en   = 1'b0;

        verify_failure_starts_erase = 1'b0;
        erase_failure_flag = 1'b0;

        unique case (state)

            //--------------------------------------------------------------
            // IDLE: wait for valid signal from parallel interface
            //--------------------------------------------------------------
            S_IDLE: begin
                busy            = 1'b0;
                buf_reg_add     = '0;
                buf_reg_ctrl    = CTRL_IDLE;
                buf_read_write  = 1'b0;
                
                // When valid signal arrives, decode command and proceed
                if (valid) begin
                    unique case (cmd)
                        CMD_PROG: begin
                            // Weight programming - store weight in buffer, then program
                            next_state = S_PROGRAM;
                            next_prog_state = PROG_HIZ;
                        end
                        CMD_ERASE: begin
                            // Weight erase - use direct address
                            next_state = S_ERASE;
                        end
                        CMD_READ: begin
                            // Read weight from memristor - use direct address
                            next_state = S_READ;
                        end
                        CMD_INF: begin
                            // Inference - collect data first
                            next_state = S_COLLECT_DATA;
                        end
                        default: begin
                            next_state = S_IDLE;
                        end
                    endcase
                end else begin
                    next_state = S_IDLE;
                end
            end

            //--------------------------------------------------------------
            // RESET: Reset ANN core and buffer (kept for compatibility)
            //--------------------------------------------------------------
            S_RESET: begin
                busy            = 1'b1;
                ann_reset       = 1'b1;
                buf_reg_ctrl    = CTRL_IDLE;
                buf_read_write  = 1'b0;
                
                // After reset, proceed to programming
                next_state = S_PROGRAM;
                next_prog_state = PROG_HIZ;
            end

            //--------------------------------------------------------------
            // PROGRAM: program weights into ANN (buffer load + program sub-FSM)
            //--------------------------------------------------------------
            S_PROGRAM: begin
                busy            = 1'b1;
                
                // For new PROG command: store weight in buffer, then read from buffer for programming
                // Buffer address: low BUFFER_ADDR_WIDTH bits of ANN address as temporary scratch slot
                // Weight must be stored in buffer for verification/reprogramming
                // Use address_reg (captured from parallel interface in S_IDLE)
                if (prog_state == PROG_HIZ || prog_state == PROG_SELECT) begin
                    // First, store weight data from parallel interface to buffer
                    // Use prog_buf_addr (low bits of the packed address) as scratch slot
                    buf_reg_add     = prog_buf_addr;
                    buf_reg_ctrl    = CTRL_DATA_LOAD;
                    buf_read_write  = 1'b1;              // Write to buffer
                end else begin
                    // Read weight from buffer for programming
                    buf_reg_add     = prog_buf_addr;
                    buf_reg_ctrl    = CTRL_WEIGHT_READ;  
                    buf_read_write  = 1'b0;              // Read from buffer
                end
                
                // Programming sequence sub-state machine
                unique case (prog_state)
                    PROG_HIZ: begin
                        next_prog_state = PROG_SELECT;
                    end
                    
                    PROG_SELECT: begin
                        next_prog_state = op_done ? PROG_WRITE : PROG_SELECT;
                    end
                    
                    PROG_WRITE: begin
                        // Hold PROG pulses until train done; then wait core op_done
                        if (pulse_done)
                            next_prog_state = PROG_WAIT_ACK;
                        else
                            next_prog_state = PROG_WRITE;
                    end
                    
                    PROG_WAIT_ACK: begin
                        next_prog_state = op_done ? PROG_COMPLETE : PROG_WAIT_ACK;
                    end
                    
                    PROG_COMPLETE: begin
                        // Weight programming complete, proceed to verify
                        next_state = S_VERIFY;
                        next_prog_state = PROG_HIZ;  // Reset for next weight (if any)
                    end
                    
                    default: begin
                        next_prog_state = PROG_HIZ;
                    end
                endcase
            end

            //--------------------------------------------------------------
            // VERIFY: Verify programmed weights (uses direct address)
            //--------------------------------------------------------------
            S_VERIFY: begin
                busy            = 1'b1;
                buf_reg_add     = prog_buf_addr;   // Read expected weight from buffer
                buf_reg_ctrl    = CTRL_WEIGHT_READ;
                buf_read_write  = 1'b0;               // Read from buffer
                weight_read_en  = (verify_state == VERIFY_IDLE) ? 1'b1 : 1'b0;
                
                // VERIFY sub-FSM logic
                unique case (verify_state)
                    VERIFY_IDLE: begin
                        // Initiate read; single-cycle train skips WAIT
                        if (CTRL_PULSE_TOTAL_READ <= 1)
                            verify_next = VERIFY_CHECK;
                        else
                            verify_next = VERIFY_WAIT;
                    end
                    
                    VERIFY_WAIT: begin
                        // One cycle per burst-train index (see verify_pulse_idx seq block)
                        if (verify_pulse_idx == CTRL_PULSE_TOTAL_READ[7:0] - 8'd1)
                            verify_next = VERIFY_CHECK;
                        else
                            verify_next = VERIFY_WAIT;
                    end
                    
                    VERIFY_CHECK: begin
                        // Compare read weight with expected weight
                        if (weight_read_data == expected_weight) begin
                            verify_next = VERIFY_DONE;  // Match: continue to next weight
                        end else if (weight_read_data < expected_weight) begin
                            // Under-programmed: re-apply PROG pulses (no ERASE)
                            if (prog_retry_cnt < MAX_PROG_RETRIES) begin
                                next_state = S_PROGRAM;
                                next_prog_state = PROG_HIZ;
                            end else begin
                                // Max re-prog attempts exceeded, fall back to ERASE
                                verify_failure_starts_erase = 1'b1;
                                next_state = S_ERASE;
                            end
                            verify_next = VERIFY_IDLE;
                        end else begin
                            // Over-programmed: ERASE then retry
                            verify_failure_starts_erase = 1'b1;
                            next_state = S_ERASE;
                            verify_next = VERIFY_IDLE;
                        end
                    end
                    
                    VERIFY_DONE: begin
                        // Direct address flow: one weight per PROG command, return to idle
                        next_state = S_IDLE;
                        verify_next = VERIFY_IDLE;
                    end
                    
                    default: begin
                        verify_next = VERIFY_IDLE;
                    end
                endcase
            end

            //--------------------------------------------------------------
            // ERASE: Erase weights if verification fails
            //--------------------------------------------------------------
            S_ERASE: begin
                busy            = 1'b1;
                buf_reg_ctrl    = CTRL_IDLE;
                buf_read_write  = 1'b0;
                
                // ERASE sub-FSM logic
                unique case (erase_state)
                    ERASE_HIZ: begin
                        // Erase sequence: outputs high-Z / idle phase
                        erase_next = ERASE_SELECT;
                    end
                    
                    ERASE_SELECT: begin
                        // Address path setup; wait core op_done before pulse phase
                        erase_next = op_done ? ERASE_PULSE : ERASE_SELECT;
                    end
                    
                    ERASE_PULSE: begin
                        // Hold erase pulse for Terase*Pulse_num_erase cycles (per spec)
                        if (pulse_done)
                            erase_next = ERASE_WAIT_ACK;
                        else
                            erase_next = ERASE_PULSE;
                    end
                    
                    ERASE_WAIT_ACK: begin
                        // Pulse train finished; wait op_done before completing erase
                        erase_next = op_done ? ERASE_COMPLETE : ERASE_WAIT_ACK;
                    end
                    
                    ERASE_COMPLETE: begin
                        // Host CMD_ERASE: finish after erase; verify-failure path reprograms
                        if (erase_from_host) begin
                            next_state = S_IDLE;
                            next_prog_state = PROG_HIZ;
                            erase_next = ERASE_HIZ;
                        end else if (retry_cnt < 3) begin
                            // Retry: go back to PROGRAM (with adaptive voltage if program_stronger set)
                            next_state = S_PROGRAM;
                            next_prog_state = PROG_HIZ;
                            erase_next = ERASE_HIZ;
                        end else begin
                            // Erase retry budget exhausted; abort verify/recovery and return to idle
                            erase_failure_flag = 1'b1;
                            next_state = S_IDLE;
                            next_prog_state = PROG_HIZ;
                            erase_next = ERASE_HIZ;
                        end
                    end
                    
                    default: begin
                        erase_next = ERASE_HIZ;
                    end
                endcase
            end

            //--------------------------------------------------------------
            // READ: Read weight from memristor at specified address
            //--------------------------------------------------------------
            S_READ: begin
                busy            = 1'b1;
                buf_reg_ctrl    = CTRL_IDLE;
                buf_read_write  = 1'b0;
                
                // Pulses held for Tread*Pulse_num_read cycles (per spec)
                if (pulse_done) begin
                    next_state = S_IDLE;
                end else begin
                    next_state = S_READ;
                end
            end

            //--------------------------------------------------------------
            // COLLECT_DATA: Collect data for inference (INF command)
            //--------------------------------------------------------------
            S_COLLECT_DATA: begin
                busy            = 1'b1;
                
                // Write pixel data to input buffer
                // Use the current counter directly so each valid beat maps 1:1 to its entry.
                buf_reg_add     = pixel_cnt;
                buf_reg_ctrl    = CTRL_DATA_LOAD;
                buf_read_write  = valid;             // Write only on an accepted beat (no junk on idle cycles)
                
                // Collect INPUT_VECTOR_LEN pixels (full input vector)
                if (valid && last_pixel) begin
                    // Whole vector collected, proceed to computation
                    next_state = S_COMPUTE;
                end else begin
                    next_state = S_COLLECT_DATA;
                end
            end

            //--------------------------------------------------------------
            // COMPUTE: column x bit-plane sweep (INF command)
            //--------------------------------------------------------------
            // 16 column positions x (8 bit-steps of T_INF_STEP + boundary of
            // T_COL_BND) = T_INF_COMPUTE cycles.  Bit-planes MSB first; the
            // next plane is driven in the last cycle of each bit-step, and the
            // MSB of the next column stays on the rows through the boundary.
            S_COMPUTE: begin
                busy            = 1'b1;
                buf_reg_add     = '0;                 // Lanes cover the whole buffer; address unused
                buf_reg_ctrl    = CTRL_COMPUTE;
                buf_read_write  = 1'b0;               // Read from buffer
                buf_bit_sel     = inf_bnd   ? PLANE_MSB  :
                                  step_last ? next_plane : cur_plane;

                if (compute_last) begin
                    next_state = S_RESULT;
                end else begin
                    next_state = S_COMPUTE;
                end
            end

            //--------------------------------------------------------------
            // RESULT: Output classification results
            //--------------------------------------------------------------
            S_RESULT: begin
                busy            = 1'b1;
                buf_reg_ctrl    = CTRL_RESULT_OUT;
                buf_read_write  = 1'b0;
                
                // After result output complete, return to idle
                // For now, assume output completes immediately (can be extended)
                next_state = S_IDLE;
            end

            default: begin
                next_state = S_IDLE;
                next_prog_state = PROG_HIZ;
            end

        endcase

        // Watchdog abort: return to idle from any waiting state
        if (timeout_hit) begin
            next_state      = S_IDLE;
            next_prog_state = PROG_HIZ;
            verify_next     = VERIFY_IDLE;
            erase_next      = ERASE_HIZ;
        end
    )

endmodule