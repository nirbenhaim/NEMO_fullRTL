`timescale 1ns/1ps
// -------------------------------------------------------------------------
// act_sat (activation: saturation)
//
// Sits after the DSA. Turns the 19-bit accumulator into the 8-bit value
// that goes on to the next layer:
//   act = |acc_in[18:8] ? 8'hFF : acc_in[7:0]
// If any of the upper bits 18..8 is 1, the value is >= 256 and does not fit
// in 8 bits, so the output saturates at 255. Otherwise it is the low 8 bits.
// (Decided 2026-11-10: saturation only. ReLU is left for later; it is a
// no-op today because acc is never negative.)
//
// Registered output: when act_start = 1 at a rising edge (driven by the
// controller after the last bit-plane), the result is stored; on the next
// cycle act_out holds it and act_valid is high for one cycle.
// Reset is asynchronous, active-low (same as the adder tree).
// -------------------------------------------------------------------------
module act_sat #(
    parameter int ACC_WIDTH = 19,
    parameter int ACT_WIDTH = 8
)(
    input  logic                  clk,
    input  logic                  rst_n,
    input  logic                  act_start,
    input  logic [ACC_WIDTH-1:0]  acc_in,

    output logic [ACT_WIDTH-1:0]  act_out,
    output logic                  act_valid
);

    logic                 overflow;
    logic [ACT_WIDTH-1:0] act_next;

    assign overflow = |acc_in[ACC_WIDTH-1:ACT_WIDTH];               // value >= 256
    assign act_next = overflow ? {ACT_WIDTH{1'b1}} : acc_in[ACT_WIDTH-1:0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            act_out   <= '0;
            act_valid <= 1'b0;
        end
        else begin
            act_valid <= act_start;
            if (act_start)
                act_out <= act_next;
        end
    end

endmodule
