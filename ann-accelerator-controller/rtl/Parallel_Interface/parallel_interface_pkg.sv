//------------------------------------------------------------------------------
// Parallel Interface Package
//------------------------------------------------------------------------------

package parallel_interface_pkg;

    // Geometry + word format (used internally; importers must import
    // ann_arch_pkg themselves - package imports are not re-exported)
    import ann_arch_pkg::*;

    //--------------------------------------------------------------------------
    // Command Signal Definitions (5 commands, 3 bits)
    //--------------------------------------------------------------------------
    localparam int CMD_WIDTH = 3;

    typedef enum logic [2:0] {
        CMD_HIZ   = 3'b000,  // High-Z (idle, no pulses)
        CMD_READ  = 3'b001,  // Read weight value from memristor (verification read)
        CMD_PROG  = 3'b010,  // Program weight at specified address
        CMD_ERASE = 3'b011,  // Erase weight at specified address
        CMD_INF   = 3'b100   // Inference/classification - apply input data for matrix multiplication
    } cmd_t;

    //--------------------------------------------------------------------------
    // Host data word = same layout as controller ann_address (ann_arch_pkg)
    //--------------------------------------------------------------------------
    //   host_data[31:24] = data byte (weight / pixel byte)
    //   host_data[23:8]  = packed binary address {PE[1:0], SA[3:0], ROW[3:0], COL[5:0]}
    //   host_data[7:0]   = reserved, must be zero
    //
    // Command is not inside the 32-bit word; parallel_interface exposes an
    // explicit host_cmd[2:0] port (SPI frame = {host_data[31:0], host_cmd[2:0]}).
    //
    // Controller receives address[WEIGHT_ADDR_WIDTH-1:0] = host_data[23:8]
    // unchanged (same packed layout as the controller's weight address).
    //--------------------------------------------------------------------------

    // Host word is well-formed: reserved bits zero and every field in range
    function automatic logic host_word_is_valid(input logic [WORD_WIDTH-1:0] host_data);
        return word_rsvd_is_zero(host_data) && addr_in_range(word_addr(host_data));
    endfunction

    // Build host_data word from packed address + data byte (TB / software)
    function automatic logic [WORD_WIDTH-1:0] build_host_ann_word(
        input logic [ACT_WIDTH-1:0]         data_byte,
        input logic [WEIGHT_ADDR_WIDTH-1:0] parallel_addr
    );
        return pack_ann_word(data_byte, parallel_addr);
    endfunction

    // Extract data byte from host_data[31:24]
    function automatic logic [ACT_WIDTH-1:0] extract_data(logic [WORD_WIDTH-1:0] host_data);
        return word_data(host_data);
    endfunction

    // Extract packed address from host_data[23:8]
    function automatic logic [WEIGHT_ADDR_WIDTH-1:0] extract_address(logic [WORD_WIDTH-1:0] host_data);
        return word_addr(host_data);
    endfunction

    // Deprecated stub: command comes from host_cmd port; always returns HIZ
    function automatic logic [2:0] extract_cmd(logic [WORD_WIDTH-1:0] host_data);
        return CMD_HIZ;
    endfunction

endpackage
