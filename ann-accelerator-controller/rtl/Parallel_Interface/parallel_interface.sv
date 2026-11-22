//------------------------------------------------------------------------------
// Parallel Interface Module
//------------------------------------------------------------------------------
// Host presents the same 32-bit layout as ann_address (see ann_arch_pkg):
//   host_data[31:24] = data byte
//   host_data[23:8]  = packed binary {PE, SA, ROW, COL}
//   host_data[7:0]   = reserved (must be zero)
//
// Command is a separate synchronous input host_cmd[2:0] (parallel_interface_pkg::cmd_t).
// Decoded outputs match what ann_controller expects: data, address, cmd.
//
// valid is a level qualifier (command active + well-formed word).  It does not
// know when a new host word arrives; the top level ANDs it with its own
// one-cycle frame strobe before driving ann_controller.valid.
//------------------------------------------------------------------------------

import ann_arch_pkg::*;
import parallel_interface_pkg::*;

module parallel_interface(
    //======================================================================
    // Global
    //======================================================================
    input  logic                         clk,
    input  logic                         reset,      // Active high reset

    //======================================================================
    // Host Interface
    //======================================================================
    input  logic [WORD_WIDTH-1:0]        host_data,  // ann_address format
    input  logic [CMD_WIDTH-1:0]         host_cmd,   // command (HIZ when idle / no transaction)

    //======================================================================
    // Controller Interface
    //======================================================================
    output logic                         valid,      // Valid command/data ready
    output logic [ACT_WIDTH-1:0]         data,       // host_data[31:24]
    output logic [WEIGHT_ADDR_WIDTH-1:0] address,    // host_data[23:8] packed {PE,SA,ROW,COL}
    output logic [CMD_WIDTH-1:0]         cmd         // host_cmd
);

    assign data    = extract_data(host_data);
    assign address = extract_address(host_data);
    assign cmd     = host_cmd;

    // Command is valid only when cmd is active and the host word is well-formed.
    assign valid = (host_cmd != CMD_HIZ) && host_word_is_valid(host_data);

endmodule
