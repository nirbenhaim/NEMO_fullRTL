//------------------------------------------------------------------------------
// Input Buffer Package
//------------------------------------------------------------------------------
// Core-level input buffer (architecture drawing: "INPUT BUFFER 256x8-BIT INT8").
// One entry per global input row: entry i feeds row (i % SA_ROWS) of
// sub-array (i / SA_ROWS) in every PE.  All entries are applied in parallel,
// bit-serially, one bit-plane at a time, MSB first (b7 -> b0) for every
// column position (order chosen by the controller via bit_sel).
//------------------------------------------------------------------------------

package input_buffer_pkg;

    // Geometry (used internally; importers must import ann_arch_pkg themselves)
    import ann_arch_pkg::*;

    //--------------------------------------------------------------------------
    // Data Organization Constants (derived from ann_arch_pkg)
    //--------------------------------------------------------------------------
    localparam int BUFFER_SIZE       = INPUT_VECTOR_LEN;          // 256 entries
    localparam int BUFFER_ADDR_WIDTH = $clog2(BUFFER_SIZE);       // 8
    localparam int BUFFER_DATA_WIDTH = ACT_WIDTH;                 // 8 (INT8)
    localparam int BUFFER_MAX_ADDR   = BUFFER_SIZE - 1;           // 255
    localparam int BUF_BIT_SEL_WIDTH = $clog2(BUFFER_DATA_WIDTH); // 3

    //--------------------------------------------------------------------------
    // Control Signal Encodings (reg_ctrl)
    //--------------------------------------------------------------------------
    typedef enum logic [2:0] {
        CTRL_IDLE         = 3'd0,
        CTRL_DATA_LOAD    = 3'd1,  // Data load (input vector / PROG scratch weight)
        CTRL_COMPUTE      = 3'd2,  // Compute mode (drives bit-serial lanes)
        CTRL_RESULT_OUT   = 3'd3,  // Result output
        CTRL_WEIGHT_READ  = 3'd4   // Weight read (weight programming)
    } buffer_ctrl_t;

    //--------------------------------------------------------------------------
    // Weight Storage Organization (scratch weight for PROG/VERIFY)
    //--------------------------------------------------------------------------
    localparam int WEIGHTS_PER_LOC   = BUFFER_DATA_WIDTH / ANN_WEIGHT_WIDTH;  // 2
    localparam int WEIGHT_BITS       = ANN_WEIGHT_WIDTH;                      // 4
    localparam int WEIGHT0_LSB       = 0;                                     // [3:0]
    localparam int WEIGHT0_MSB       = WEIGHT0_LSB + WEIGHT_BITS - 1;
    localparam int WEIGHT1_LSB       = WEIGHT0_MSB + 1;                       // [7:4]
    localparam int WEIGHT1_MSB       = WEIGHT1_LSB + WEIGHT_BITS - 1;

    //--------------------------------------------------------------------------
    // Output Data Lines (one bit-serial lane per global input row)
    //--------------------------------------------------------------------------
    localparam int NUM_DATA_LINES    = BUFFER_SIZE;  // 256 lanes
    localparam int DATA_LINE_WIDTH   = 1;            // 1 bit per lane per cycle

    //--------------------------------------------------------------------------
    // Helper Functions
    //--------------------------------------------------------------------------

    // True if addr is within the buffer depth
    function automatic logic is_valid_addr(logic [BUFFER_ADDR_WIDTH-1:0] addr);
        return (int'(addr) < BUFFER_SIZE);
    endfunction

    // Sub-array index fed by a buffer entry
    function automatic logic [SA_ID_WIDTH-1:0] get_sa_from_addr(logic [BUFFER_ADDR_WIDTH-1:0] addr);
        return SA_ID_WIDTH'(int'(addr) / SA_ROWS);
    endfunction

    // Row (within the sub-array) fed by a buffer entry
    function automatic logic [ROW_ID_WIDTH-1:0] get_row_from_addr(logic [BUFFER_ADDR_WIDTH-1:0] addr);
        return ROW_ID_WIDTH'(int'(addr) % SA_ROWS);
    endfunction

    // Lower weight field of a buffer word (weight 0)
    function automatic logic [WEIGHT_BITS-1:0] extract_weight0(logic [BUFFER_DATA_WIDTH-1:0] data);
        return data[WEIGHT0_MSB:WEIGHT0_LSB];
    endfunction

    // Upper weight field of a buffer word (weight 1)
    function automatic logic [WEIGHT_BITS-1:0] extract_weight1(logic [BUFFER_DATA_WIDTH-1:0] data);
        return data[WEIGHT1_MSB:WEIGHT1_LSB];
    endfunction

endpackage
