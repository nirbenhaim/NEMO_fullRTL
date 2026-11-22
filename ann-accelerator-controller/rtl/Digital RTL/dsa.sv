// -------------------------------------------------------------------------
// dsa (Digital Shift Accumulator)
//
// Sized per the 16x64 SA architecture diagram: takes the adder tree's
// 11-bit sum (SUM_WIDTH from addr_tree_Nir_flow) and accumulates it with a
// left-shift each cycle, implementing the bit-serial weighted
// reconstruction ("DIGITAL SHIFT x 2^8" in the diagram) driven by the
// 8-cycle bit-serial controller elsewhere in the PE. After 8 shift-
// accumulate cycles the running sum reaches full INT8-weighted precision,
// hence ACC_WIDTH = VEC_WIDTH + 8 = 19 (matches the diagram's 11-BIT in /
// 19-BIT out labels on this stage, upstream of the ReLU activation block).
//
// tree_result_valid ties directly to addr_tree_Nir_flow's out_valid: the
// tree only produces one real partial sum every 4 cycles (its own pipeline
// latency), so the accumulate step here must only fire on the cycles the
// tree actually says its output is valid -- not on every clock edge.
// -------------------------------------------------------------------------
module dsa #(
    parameter int VEC_WIDTH = 11,
    parameter int ACC_WIDTH = 19
)(
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     tree_result_valid,
    input  logic [VEC_WIDTH-1:0]     vec_in,

    output logic [ACC_WIDTH-1:0]     acc_out,
    output logic [VEC_WIDTH-1:0]     vec_out
);

    logic [ACC_WIDTH-1:0] acc_reg;

    // -------------------------------------------------
    // Input buffering (explicit standard-cell buffers)
    // -------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < VEC_WIDTH; i++) begin : GEN_VEC_BUF
            BUF_X2_18_SVT_WTN buff (
                .A (vec_in[i]),
                .Q (vec_out[i])
            );
        end
    endgenerate

    // -------------------------------------------------
    // DSA / accumulator logic
    // Uses *buffered* vec_out
    // -------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            acc_reg <= '0;
        end
        else if (tree_result_valid) begin
            acc_reg <= (acc_reg << 1)
                     + {{(ACC_WIDTH-VEC_WIDTH){1'b0}}, vec_out};
        end
    end

    assign acc_out = acc_reg;

endmodule
