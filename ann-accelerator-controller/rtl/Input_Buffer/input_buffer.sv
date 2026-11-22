//------------------------------------------------------------------------------
// Input Buffer - SystemVerilog Implementation
//------------------------------------------------------------------------------
// BUFFER_SIZE (256) x BUFFER_DATA_WIDTH (8) register file.
//   - Write port : controller loads one byte per cycle (CTRL_DATA_LOAD).
//   - buf_data   : full byte at buf_reg_add (weight read / verify).
//   - data_lanes : during CTRL_COMPUTE, lane i = buffer_reg[i][bit_sel], i.e.
//                  the whole input vector is applied in parallel, one bit-plane
//                  at a time, as selected by the controller (MSB first: b7 ->
//                  b0 for every column position).  Lane i drives row
//                  (i % SA_ROWS) of sub-array (i / SA_ROWS) in every PE.
//------------------------------------------------------------------------------

`include "../common/macros.svh"

import ann_arch_pkg::*;
import input_buffer_pkg::*;

module input_buffer(
    //======================================================================
    // Clock & Reset
    //======================================================================
    input  logic                          clk,
    input  logic                          rst_n,

    //======================================================================
    // Connection to Parallel Interface
    //======================================================================
    input  logic [BUFFER_DATA_WIDTH-1:0]  data_in,         // Incoming data

    //======================================================================
    // Connection to Controller
    //======================================================================
    output logic                          ready,           // Buffer is ready to accept new data
    input  logic [2:0]                    reg_ctrl,        // Register control
    input  logic                          buf_read_write,  // 1 = write, 0 = read
    input  logic [BUFFER_ADDR_WIDTH-1:0]  buf_reg_add,     // entry address
    input  logic [BUF_BIT_SEL_WIDTH-1:0]  bit_sel,         // Bit-plane index for bit-serial output (controller drives MSB first)

    //======================================================================
    // Output to Controller (full byte at current address, for weight read/verify)
    //======================================================================
    output logic [BUFFER_DATA_WIDTH-1:0]  buf_data,

    //======================================================================
    // Output data lanes to ANN Core (1 bit per input row per cycle)
    //======================================================================
    output logic [NUM_DATA_LINES-1:0]     data_lanes
);

    //--------------------------------------------------------------------------
    // Internal Storage
    //--------------------------------------------------------------------------
    logic [BUFFER_DATA_WIDTH-1:0] buffer_reg [0:BUFFER_SIZE-1];

    // Internal control signals derived from reg_ctrl / buf_read_write
    logic                           write_en;    // Asserted for CTRL_DATA_LOAD writes
    logic                           read_en;     // Asserted for COMPUTE / RESULT_OUT / WEIGHT_READ reads
    logic                           compute_en;  // Asserted for CTRL_COMPUTE reads (drives lanes)
    logic [BUFFER_ADDR_WIDTH-1:0]   addr;        // Current buffer address (= buf_reg_add)

    //--------------------------------------------------------------------------
    // Control Signal Decoding
    //--------------------------------------------------------------------------
    assign write_en   = buf_read_write && (reg_ctrl == CTRL_DATA_LOAD);
    assign read_en    = ~buf_read_write && ((reg_ctrl == CTRL_COMPUTE) ||
                                             (reg_ctrl == CTRL_RESULT_OUT) ||
                                             (reg_ctrl == CTRL_WEIGHT_READ));
    assign compute_en = ~buf_read_write && (reg_ctrl == CTRL_COMPUTE);
    assign addr       = buf_reg_add;

    //--------------------------------------------------------------------------
    // Write Path: Store data from parallel interface
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize all buffer locations to zero
            for (int i = 0; i < BUFFER_SIZE; i++) begin
                buffer_reg[i] <= '0;
            end
        end else begin
            // Write data to buffer when write is enabled
            if (write_en && is_valid_addr(addr)) begin
                buffer_reg[addr] <= data_in;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Read Path: buf_data = full byte at addr (for controller weight read/verify)
    //--------------------------------------------------------------------------
    `comb(
        buf_data = '0;
        if (read_en && is_valid_addr(addr))
            buf_data = buffer_reg[addr];
    )

    //--------------------------------------------------------------------------
    // Read Path: data_lanes = bit 'bit_sel' of every entry (bit-serial; order set by the controller)
    //--------------------------------------------------------------------------
    // Only driven in CTRL_COMPUTE so the array rows stay quiet while the
    // buffer is used as PROG/VERIFY scratch storage.
    //--------------------------------------------------------------------------
    `comb(
        for (int i = 0; i < NUM_DATA_LINES; i++)
            data_lanes[i] = compute_en ? buffer_reg[i][bit_sel] : 1'b0;
    )

    //--------------------------------------------------------------------------
    // Ready Signal Logic
    //--------------------------------------------------------------------------

    `comb(

        ready = 1'b0;

        // Ready for write operations during data load
        if (reg_ctrl == CTRL_DATA_LOAD && buf_read_write) begin
            // Buffer is always ready to accept writes (address valid)
            ready = is_valid_addr(addr);
        end
        // Ready for read operations during compute/result/weight read phase
        else if ((reg_ctrl == CTRL_COMPUTE ||
                  reg_ctrl == CTRL_RESULT_OUT ||
                  reg_ctrl == CTRL_WEIGHT_READ) && ~buf_read_write) begin
            // Data is available immediately for reading
            ready = is_valid_addr(addr);
        end
        // Not ready in idle or invalid control states
        else begin
            ready = 1'b0;
        end
    )

endmodule
