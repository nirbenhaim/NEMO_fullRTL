//------------------------------------------------------------------------------
// ANN Architecture Package
//------------------------------------------------------------------------------
// Single source of truth for the NEMO CS-IMC array geometry and for the 32-bit
// word format shared by the host (SPI frame), the parallel interface, the
// controller and the analog ANN core.  Values follow nemo_top_wrapper.sv and
// the Drawing_16x64_SA architecture:
//
//   1 Core = NUM_PE PEs            (4)
//   1 PE   = NUM_SA sub-arrays     (16)
//   1 SA   = SA_ROWS x SA_COLS     (16 x 64 Y-Flash array)
//   Weight = ANN_WEIGHT_WIDTH bits (4, Y-Flash conductance level)
//   Data   = ACT_WIDTH bits INT8, applied bit-serially, MSB first (b7 -> b0);
//            all 8 planes go to one column position before the mux advances
//
// Input mapping: the NUM_SA sub-arrays of a PE are stacked along the input
// (row) dimension and summed by the PE adder tree, so one PE consumes
// NUM_SA*SA_ROWS = 256 inputs and produces SA_COLS = 64 outputs.  All PEs share
// the same input vector (core = 256 inputs x 256 outputs).
//
// Must be compiled before every other package in this project.
//------------------------------------------------------------------------------

package ann_arch_pkg;

    //--------------------------------------------------------------------------
    // Primary knobs (must match nemo_top_wrapper.sv / architecture drawing)
    //--------------------------------------------------------------------------
    localparam int NUM_PE           = 4;    // PEs per core
    localparam int NUM_SA           = 16;   // sub-arrays per PE
    localparam int SA_ROWS          = 16;   // rows per sub-array
    localparam int SA_COLS          = 64;   // columns per sub-array
    localparam int ANN_WEIGHT_WIDTH = 4;    // weight precision (bits)
    localparam int ACT_WIDTH        = 8;    // activation / payload width (INT8)

    //--------------------------------------------------------------------------
    // Packed binary weight address {PE, SA, ROW, COL}  (COL at LSB)
    //--------------------------------------------------------------------------
    localparam int PE_ID_WIDTH  = $clog2(NUM_PE);    // 2
    localparam int SA_ID_WIDTH  = $clog2(NUM_SA);    // 4
    localparam int ROW_ID_WIDTH = $clog2(SA_ROWS);   // 4
    localparam int COL_ID_WIDTH = $clog2(SA_COLS);   // 6

    localparam int WEIGHT_ADDR_WIDTH = PE_ID_WIDTH + SA_ID_WIDTH +
                                       ROW_ID_WIDTH + COL_ID_WIDTH;   // 16

    localparam int COL_ID_LSB = 0;
    localparam int COL_ID_MSB = COL_ID_LSB + COL_ID_WIDTH - 1;        // 5
    localparam int ROW_ID_LSB = COL_ID_MSB + 1;                       // 6
    localparam int ROW_ID_MSB = ROW_ID_LSB + ROW_ID_WIDTH - 1;        // 9
    localparam int SA_ID_LSB  = ROW_ID_MSB + 1;                       // 10
    localparam int SA_ID_MSB  = SA_ID_LSB + SA_ID_WIDTH - 1;          // 13
    localparam int PE_ID_LSB  = SA_ID_MSB + 1;                        // 14
    localparam int PE_ID_MSB  = PE_ID_LSB + PE_ID_WIDTH - 1;          // 15

    //--------------------------------------------------------------------------
    // Derived array geometry
    //--------------------------------------------------------------------------
    localparam int INPUT_VECTOR_LEN       = NUM_SA * SA_ROWS;                    // 256
    localparam int OUTPUTS_PER_PE         = SA_COLS;                             // 64
    localparam int NUM_OUTPUTS            = NUM_PE * SA_COLS;                    // 256
    localparam int TOTAL_WEIGHT_LOCATIONS = NUM_PE * NUM_SA * SA_ROWS * SA_COLS; // 65536

    // Digital datapath (Controller_vs_datapath_review): each SA has ADCS_PER_SA
    // SAR ADCs behind 16:1 column muxes; adder tree t of a PE sums ADC t of all
    // 16 SAs -> ADCS_PER_SA trees per PE, NUM_TREES result channels per core.
    localparam int ADCS_PER_SA      = 4;
    localparam int COL_MUX_RATIO    = SA_COLS / ADCS_PER_SA;      // 16
    localparam int NUM_TREES        = NUM_PE * ADCS_PER_SA;       // 16
    localparam int ACC_WIDTH        = 19;                         // DSA accumulator
    localparam int RESULT_BITS      = NUM_TREES * ACT_WIDTH;      // 128 (act_out of ONE column position)

    // Column sweep (NEMO CS-IMC Architecture Brief Rev. 1.0 §6.2): the column
    // mux select is common to all 4 muxes of every SA in every PE.  At column
    // position k, ADC/tree g of a PE converts column g*COL_MUX_RATIO + k
    // (neuron g*16 + k of that PE).  One inference sweeps all NUM_COL_POS
    // positions, so it produces NUM_COL_POS x NUM_TREES = NUM_OUTPUTS results.
    localparam int NUM_COL_POS      = COL_MUX_RATIO;              // 16 column positions
    localparam int COL_SEL_WIDTH    = $clog2(NUM_COL_POS);        // 4
    localparam int OUTPUT_BITS      = NUM_OUTPUTS * ACT_WIDTH;    // 2048 (256 x UINT8 per inference)

    localparam int INPUT_IDX_WIDTH  = $clog2(INPUT_VECTOR_LEN);   // 8
    localparam int OUTPUT_IDX_WIDTH = $clog2(NUM_OUTPUTS);        // 8
    localparam int BIT_SEL_WIDTH    = $clog2(ACT_WIDTH);          // 3

    //--------------------------------------------------------------------------
    // 32-bit word format (host_data from SPI  ==  controller ann_address)
    //--------------------------------------------------------------------------
    //   [31:24] payload  (weight in [27:24] for PROG, pixel byte for INF)
    //   [23:22] PE   (binary)
    //   [21:18] SA   (binary)
    //   [17:14] ROW  (binary)
    //   [13:8]  COL  (binary)
    //   [7:0]   reserved, must be zero
    //--------------------------------------------------------------------------
    localparam int WORD_WIDTH      = 32;
    localparam int WORD_DATA_MSB   = WORD_WIDTH - 1;                   // 31
    localparam int WORD_DATA_LSB   = WORD_WIDTH - ACT_WIDTH;           // 24
    localparam int WORD_ADDR_MSB   = WORD_DATA_LSB - 1;                // 23
    localparam int WORD_ADDR_LSB   = WORD_DATA_LSB - WEIGHT_ADDR_WIDTH; // 8
    localparam int WORD_RSVD_WIDTH = WORD_ADDR_LSB;                    // 8 -> [7:0]

    typedef logic [WEIGHT_ADDR_WIDTH-1:0] ann_addr_t;
    typedef logic [WORD_WIDTH-1:0]        ann_word_t;

    //--------------------------------------------------------------------------
    // Address helpers
    //--------------------------------------------------------------------------

    // Build packed address from binary field indices
    function automatic ann_addr_t pack_ann_addr(
        input logic [PE_ID_WIDTH-1:0]  pe_id,
        input logic [SA_ID_WIDTH-1:0]  sa_id,
        input logic [ROW_ID_WIDTH-1:0] row_id,
        input logic [COL_ID_WIDTH-1:0] col_id
    );
        ann_addr_t a;
        a = '0;
        a[PE_ID_MSB:PE_ID_LSB]   = pe_id;
        a[SA_ID_MSB:SA_ID_LSB]   = sa_id;
        a[ROW_ID_MSB:ROW_ID_LSB] = row_id;
        a[COL_ID_MSB:COL_ID_LSB] = col_id;
        return a;
    endfunction

    function automatic logic [PE_ID_WIDTH-1:0] get_pe_id(input ann_addr_t addr);
        return addr[PE_ID_MSB:PE_ID_LSB];
    endfunction

    function automatic logic [SA_ID_WIDTH-1:0] get_sa_id(input ann_addr_t addr);
        return addr[SA_ID_MSB:SA_ID_LSB];
    endfunction

    function automatic logic [ROW_ID_WIDTH-1:0] get_row_id(input ann_addr_t addr);
        return addr[ROW_ID_MSB:ROW_ID_LSB];
    endfunction

    function automatic logic [COL_ID_WIDTH-1:0] get_col_id(input ann_addr_t addr);
        return addr[COL_ID_MSB:COL_ID_LSB];
    endfunction

    // True when every field indexes an existing PE / SA / row / column
    // (always true while all dimensions are powers of two).
    function automatic logic addr_in_range(input ann_addr_t addr);
        return (int'(get_pe_id(addr))  < NUM_PE)  &&
               (int'(get_sa_id(addr))  < NUM_SA)  &&
               (int'(get_row_id(addr)) < SA_ROWS) &&
               (int'(get_col_id(addr)) < SA_COLS);
    endfunction

    //--------------------------------------------------------------------------
    // Word helpers
    //--------------------------------------------------------------------------

    // {payload, packed address, reserved=0}
    function automatic ann_word_t pack_ann_word(
        input logic [ACT_WIDTH-1:0] data_byte,
        input ann_addr_t            addr
    );
        ann_word_t w;
        w = '0;
        w[WORD_DATA_MSB:WORD_DATA_LSB] = data_byte;
        w[WORD_ADDR_MSB:WORD_ADDR_LSB] = addr;
        return w;
    endfunction

    function automatic logic [ACT_WIDTH-1:0] word_data(input ann_word_t w);
        return w[WORD_DATA_MSB:WORD_DATA_LSB];
    endfunction

    function automatic ann_addr_t word_addr(input ann_word_t w);
        return w[WORD_ADDR_MSB:WORD_ADDR_LSB];
    endfunction

    function automatic logic word_rsvd_is_zero(input ann_word_t w);
        return (w[WORD_RSVD_WIDTH-1:0] == '0);
    endfunction

    //--------------------------------------------------------------------------
    // Weight-matrix mapping (core-level W[input][output] -> packed address)
    //--------------------------------------------------------------------------
    //   output column o : PE  = o / SA_COLS,  COL = o % SA_COLS
    //   input  row    i : SA  = i / SA_ROWS,  ROW = i % SA_ROWS
    //--------------------------------------------------------------------------
    function automatic ann_addr_t matrix_coords_to_ann_addr(
        input int unsigned in_idx,    // 0 .. INPUT_VECTOR_LEN-1
        input int unsigned out_idx    // 0 .. NUM_OUTPUTS-1
    );
        return pack_ann_addr(PE_ID_WIDTH'(out_idx / SA_COLS),
                             SA_ID_WIDTH'(in_idx  / SA_ROWS),
                             ROW_ID_WIDTH'(in_idx % SA_ROWS),
                             COL_ID_WIDTH'(out_idx % SA_COLS));
    endfunction

endpackage
