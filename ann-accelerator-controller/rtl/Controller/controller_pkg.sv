//------------------------------------------------------------------------------
// Controller Package 
//------------------------------------------------------------------------------


package controller_pkg;

    //--------------------------------------------------------------------------
    // Geometry / word format and command types (used internally; importers must
    // import ann_arch_pkg and parallel_interface_pkg themselves)
    //--------------------------------------------------------------------------
    import ann_arch_pkg::*;
    import parallel_interface_pkg::*;

    //--------------------------------------------------------------------------
    // Controller FSM States
    //--------------------------------------------------------------------------
    typedef enum logic [3:0] {
        S_IDLE          = 4'd0,  // Idle state
        S_RESET         = 4'd1,  // Reset ANN core and buffer
        S_PROGRAM       = 4'd2,  // Program weights into ANN
        S_VERIFY        = 4'd3,  // Verify programmed weights
        S_ERASE         = 4'd4,  // Erase weights 
        S_READ          = 4'd5,  // Read weight from memristor
        S_COLLECT_DATA  = 4'd6,  // Collect data for inference (INF command)
        S_COMPUTE       = 4'd7,  // Inference computation phase
        S_RESULT        = 4'd8   // Output classification results
    } controller_state_t;

    //--------------------------------------------------------------------------
    // ANN Architecture
    //--------------------------------------------------------------------------
    // NUM_PE / NUM_SA / SA_ROWS / SA_COLS, the packed weight-address layout
    // {PE, SA, ROW, COL} and the 32-bit word format live in ann_arch_pkg
    // (rtl/common/ann_arch_pkg.sv) so the controller, parallel interface, input
    // buffer and nemo_top_wrapper all share one definition.
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // Legacy weight-count tracking (kept for ERASE/VERIFY compatibility)
    //--------------------------------------------------------------------------
    localparam int DEFAULT_MATRIX_ROWS = 10;  // legacy 640-weight case (10x64)
    localparam int DEFAULT_MATRIX_COLS = 64;
    localparam int DEFAULT_NUM_WEIGHTS = DEFAULT_MATRIX_ROWS * DEFAULT_MATRIX_COLS;  // 640

    localparam int WEIGHT_COUNT_WIDTH = $clog2(TOTAL_WEIGHT_LOCATIONS);  // 16 for 65536
    localparam int MIN_WEIGHT_COUNT   = 1;
    localparam int MAX_WEIGHT_COUNT   = TOTAL_WEIGHT_LOCATIONS;

    // Max re-program attempts without ERASE (when read < expected)
    localparam int MAX_PROG_RETRIES = 3;

    //--------------------------------------------------------------------------
    // Watchdog timeouts (clk cycles) – defaults for ann_controller parameters
    //--------------------------------------------------------------------------
    // OP_DONE : max wait for the analog core's op_done in PROG/ERASE handshakes
    // COLLECT : max gap between two pixel beats while collecting an INF vector
    localparam int DEFAULT_OP_DONE_TIMEOUT = 4096;      // ~41 us @ 100 MHz
    localparam int DEFAULT_COLLECT_TIMEOUT = 1 << 20;   // ~10.5 ms @ 100 MHz

    //--------------------------------------------------------------------------
    // Default Parameters
    //--------------------------------------------------------------------------
    localparam int DEFAULT_ADDR_WIDTH   = WEIGHT_ADDR_WIDTH;  // 16: packed {PE,SA,ROW,COL}
    localparam int DEFAULT_WEIGHT_WIDTH = ANN_WEIGHT_WIDTH;   // 4

    //--------------------------------------------------------------------------
    // Pulse Mode Encoding (5 modes, 3 bits) - matches host cmd_t
    //--------------------------------------------------------------------------
    // Used for pulse output: each bit that is 1 in the mode drives a pulse
    typedef enum logic [2:0] {
        PULSE_MODE_HIZ   = 3'b000,
        PULSE_MODE_READ  = 3'b001,
        PULSE_MODE_PROG  = 3'b010,
        PULSE_MODE_ERASE = 3'b011,
        PULSE_MODE_INF   = 3'b100
    } pulse_mode_t;

    //--------------------------------------------------------------------------
    // Pulse Generator Parameters
    //--------------------------------------------------------------------------
    // T*: cycles each burst holds the active pulse mode (READ=001, PROG=010, etc.)
    // PULSE_NUM_*: number of bursts (repeats)
    // PULSE_GAP: HIZ (000) cycles between bursts (none after the last burst)
    // Total train length = N*T + max(0,N-1)*PULSE_GAP
    localparam int TREAD           = 2;  // Read pulse width (clock cycles)
    localparam int PULSE_NUM_READ  = 1;  // Number of read pulses
    localparam int TPROG           = 5;  // Program pulse width
    localparam int PULSE_NUM_PROG  = 1;  // Number of program pulses
    localparam int TERASE          = 2;  // Erase pulse width
    localparam int PULSE_NUM_ERASE = 1;  // Number of erase pulses
    localparam int PULSE_GAP       = 1;  // Idle cycles between bursts

    //--------------------------------------------------------------------------
    // Inference schedule (S_COMPUTE)
    //--------------------------------------------------------------------------
    // Source: NEMO CS-IMC Architecture Brief Rev. 1.0, §6.2-6.3 (Fig. 3, 4,
    // Table 7), with the bit order changed to MSB first (design decision
    // 2026-11-18): for column position k = 0..15 the column mux is set to k
    // and held while the 8 bit-planes b7, b6, ..., b0 are applied; only then
    // does the mux advance to k+1.  The DSA therefore accumulates
    // acc = (acc << 1) + tree, and needs no bit index.
    //
    // One bit-step (Fig. 3), cycles counted from 0:
    //   0..1   array settle           (plane on the rows, pulses = INF)
    //   2..9   ADC conversion         (pulses = INF)
    //   10     tree_start             (ADC codes valid -> adder tree samples)
    //   10..13 adder-tree reduction (4 stages)
    //   14     shift-and-add          (tree_result_valid -> DSA)
    //   15     drive the next plane   (bit_sel changes to the next plane)
    // Column boundary (Fig. 4), after the 8th bit-step:
    //   activation (2) -> register write-back (4) -> column advance + settle (2)
    // All counts are clk cycles of the 10-20 MHz core clock (brief §4.1).
    localparam int T_INF_SETTLE    = 2;   // array settle after a plane is driven
    localparam int T_INF_ADC       = 8;   // ADC conversion (VTC/SAR-TDC phases 5:2:1)
    localparam int T_INF_TREE      = 4;   // adder-tree pipeline depth
    localparam int T_INF_SHIFT     = 1;   // DSA shift-and-add
    localparam int T_INF_DRIVE     = 1;   // drive the next bit-plane
    localparam int T_INF_STEP      = T_INF_SETTLE + T_INF_ADC + T_INF_TREE +
                                     T_INF_SHIFT + T_INF_DRIVE;          // 16
    localparam int T_COL_ACT       = 2;   // neuron complete + activation
    localparam int T_COL_WB        = ADCS_PER_SA;                        // 4 register writes
    localparam int T_COL_ADV       = 2;   // column mux advance + array settle
    localparam int T_COL_BND       = T_COL_ACT + T_COL_WB + T_COL_ADV;   // 8
    localparam int T_INF_COLUMN    = ACT_WIDTH * T_INF_STEP + T_COL_BND; // 136
    localparam int T_INF_COMPUTE   = NUM_COL_POS * T_INF_COLUMN;         // 2176 (brief Table 7)
    localparam int INF_TREE_START_CYC = T_INF_SETTLE + T_INF_ADC;        // 10
    localparam int INF_CYC_WIDTH   = $clog2((T_INF_STEP > T_COL_BND) ? T_INF_STEP : T_COL_BND); // 4

    // INF pulse width per bit-step: pulses = INF while the array is read
    // (settle + conversion).  Kept under the old name for the wave scripts.
    localparam int TINF            = T_INF_SETTLE + T_INF_ADC;           // 10
    // Aliases kept for testbench reports / wave scripts (Rev 5 meaning):
    //   PULSE_NUM_INF   = INF pulses per inference (one per bit-step) = 128
    //   PULSE_TOTAL_INF = S_COMPUTE length in cycles                  = 2176
    localparam int PULSE_NUM_INF   = NUM_COL_POS * ACT_WIDTH;
    localparam int PULSE_TOTAL_INF = T_INF_COMPUTE;

    // Total cycles for a burst train: N*T + (N-1)*G (returns 0 if N < 1)
    function automatic int pulse_train_total(input int T, input int N, input int G);
        int t_safe;
        t_safe = (T < 1) ? 1 : T;
        if (N < 1)
            return 0;
        return N * t_safe + (N - 1) * G;
    endfunction

    // True when cycle_idx falls inside an active burst (not a HIZ gap)
    function automatic logic pulse_train_active(
        input int cycle_idx,
        input int T,
        input int N,
        input int G
    );
        int t_safe, chunk, r, base;
        t_safe = (T < 1) ? 1 : T;
        if (N < 1 || cycle_idx < 0)
            return 1'b0;
        chunk = t_safe + G;
        for (r = 0; r < N; r++) begin
            base = r * chunk;
            if (cycle_idx >= base && cycle_idx < base + t_safe)
                return 1'b1;
        end
        return 1'b0;
    endfunction

    // Total cycles for R back-to-back copies of one macro train (length Mmacro),
    // separated by G HIZ cycles (no gap after the last copy). Used for LUT PROG.
    function automatic int pulse_lut_macro_repeat_total(input int R, input int Mmacro, input int G);
        int Rc;
        Rc = (R < 1) ? 1 : R;
        if (Mmacro < 1)
            return 0;
        return Rc * Mmacro + (Rc - 1) * G;
    endfunction

    // True when cyc is inside an active burst of the R-fold LUT macro train
    function automatic logic pulse_lut_macro_repeat_active(
        input int cyc,
        input int R,
        input int Mmacro,
        input int T,
        input int N,
        input int G
    );
        int Rc;
        int cum;
        int b;
        Rc = (R < 1) ? 1 : R;
        if (Mmacro < 1 || cyc < 0)
            return 1'b0;
        cum = 0;
        for (b = 0; b < Rc; b++) begin
            if (cyc >= cum && cyc < cum + Mmacro)
                return pulse_train_active(cyc - cum, T, N, G);
            cum += Mmacro;
            if (b < Rc - 1) begin
                if (cyc >= cum && cyc < cum + G)
                    return 1'b0;
                cum += G;
            end
        end
        return 1'b0;
    endfunction

    localparam int PULSE_TOTAL_READ     = pulse_train_total(TREAD, PULSE_NUM_READ, PULSE_GAP);
    localparam int PULSE_TOTAL_PROG     = pulse_train_total(TPROG, PULSE_NUM_PROG, PULSE_GAP);
    localparam int PULSE_TOTAL_ERASE    = pulse_train_total(TERASE, PULSE_NUM_ERASE, PULSE_GAP);
    // (INF no longer uses the burst-train generator: S_COMPUTE runs the
    //  column x bit-plane schedule above, T_INF_COMPUTE cycles in total.)

    //--------------------------------------------------------------------------
    // ANN core word helpers (ann_address output == host word format)
    //--------------------------------------------------------------------------
    localparam int ADDR_OUT_WIDTH = WORD_WIDTH;  // 32

    // Pack data byte with binary PE/SA/row/col indices into the 32-bit word
    function automatic logic [WORD_WIDTH-1:0] pack_ann_address(
        input logic [ACT_WIDTH-1:0]    data_byte,
        input logic [PE_ID_WIDTH-1:0]  pe_id,
        input logic [SA_ID_WIDTH-1:0]  sa_id,
        input logic [ROW_ID_WIDTH-1:0] row_id,
        input logic [COL_ID_WIDTH-1:0] col_id
    );
        return pack_ann_word(data_byte, pack_ann_addr(pe_id, sa_id, row_id, col_id));
    endfunction

    // Decode the 32-bit ANN core word into binary PE/SA/row/col indices
    function automatic void ann_address_decode(
        input  logic [WORD_WIDTH-1:0]   word,
        output logic [PE_ID_WIDTH-1:0]  o_pe_id,
        output logic [SA_ID_WIDTH-1:0]  o_sa_id,
        output logic [ROW_ID_WIDTH-1:0] o_row_id,
        output logic [COL_ID_WIDTH-1:0] o_col_id
    );
        o_pe_id  = get_pe_id(word_addr(word));
        o_sa_id  = get_sa_id(word_addr(word));
        o_row_id = get_row_id(word_addr(word));
        o_col_id = get_col_id(word_addr(word));
    endfunction

    //--------------------------------------------------------------------------
    // Programming Sequence Sub-States
    //--------------------------------------------------------------------------
    typedef enum logic [2:0] {
        PROG_HIZ,
        PROG_SELECT,     // wait op_done (core/setup) before PROG_WRITE
        PROG_WRITE,
        PROG_WAIT_ACK,   // after pulse_done, wait op_done before PROG_COMPLETE
        PROG_COMPLETE
    } prog_sequence_state_t;

    // Erase sub-FSM (S_ERASE only)
    typedef enum logic [2:0] {
        ERASE_HIZ,
        ERASE_SELECT,    // wait op_done before ERASE_PULSE
        ERASE_PULSE,
        ERASE_WAIT_ACK,  // after pulse_done, wait op_done before ERASE_COMPLETE
        ERASE_COMPLETE
    } erase_state_t;

    //--------------------------------------------------------------------------
    // Address Parsing Function for Direct Address Interface
    //--------------------------------------------------------------------------
    // Split the packed parallel-interface address into binary fields.
    // Layout (ann_arch_pkg): {PE[15:14], SA[13:10], ROW[9:6], COL[5:0]}
    function automatic void parse_ann_address(
        input  logic [WEIGHT_ADDR_WIDTH-1:0] address,
        output logic [PE_ID_WIDTH-1:0]       pe_id,
        output logic [SA_ID_WIDTH-1:0]       sa_id,
        output logic [ROW_ID_WIDTH-1:0]      row_id,
        output logic [COL_ID_WIDTH-1:0]      col_id
    );
        pe_id  = get_pe_id(address);
        sa_id  = get_sa_id(address);
        row_id = get_row_id(address);
        col_id = get_col_id(address);
    endfunction

endpackage
