// =============================================================================
//  nemo_top_wrapper.sv
//  NEMO CS-IMC – Top-Level Digital Wrapper  (Rev 4 – controller RTL integrated)
// =============================================================================
//
//  CHANGES vs Rev 3
//  ────────────────
//  [9]  Architecture parameters now come from ann_arch_pkg
//       (ann-accelerator-controller/rtl/common/ann_arch_pkg.sv), the single
//       source of truth shared with the controller, parallel interface and
//       input buffer.  Values unchanged: 4 PE x 16 SA x 16 rows x 64 cols,
//       4-bit weights, INT8 activations.
//  [10] parallel_interface / ann_controller now use the packed binary host
//       word natively → the one-hot reconstruction (FIX #8) and the
//       pi_address_packed bypass are removed; host_data is passed directly.
//  [11] ann_controller.valid = host_valid (1-cycle frame strobe) AND
//       pi_valid_out (cmd != HIZ, reserved bits zero, fields in range).
//  [12] ann_core_word is the packed binary word {payload, PE, SA, ROW, COL,
//       8'h00} (same layout as host_data).  A one-hot tail for 16 SA / 16 rows
//       / 64 cols would need 100 bits; the core has its own row/col decoders.
//       pe_sel / row_sel_internal are decoded from the binary fields.
//  [13] New port ann_input_bits[255:0]: bit-serial activations from the
//       256x8 input buffer (lane i → SA i/16, row i%16, shared by all PEs).
//       Replaces the 8 unconnected D0-D7 lanes.
//  [14] dsa_clear / activation_start are driven from controller strobes
//       (dsa_clear, compute_done) instead of being inferred from host_cmd
//       and bit_idx wrap-around.
//  [15] SPI frame handshake: frame_done_sclk was a level that stayed high
//       until the next sclk edge, so host_valid could be high for many clk
//       cycles per frame (multiple beats).  Replaced by a frame holding
//       register + toggle synchroniser → exactly one host_valid per frame,
//       and the captured frame is stable for a whole frame time.
//  [16] Input-buffer address widened to 8 bits (256 entries).
//  [17] WEIGHT_PULSE_LUT_FILE exposed as a wrapper parameter.
//  [18] MISO readback rewritten (SPI mode 0).  Old logic cleared its load
//       flag one clk after irq while the bus was idle (host read zeros) or
//       reloaded on every sclk edge during a frame (host read bit 31 x32),
//       with an unsynchronised clk→sclk crossing.  Now every frame returns
//       {result[31:0], 1'b0, busy, result_valid}; see MISO section.
//  [19] Reset synchroniser: rst_n is asserted asynchronously and released
//       synchronously to clk (rst_n_clk) for every clk-domain block.
//  [20] SPI receiver bit counter is reset asynchronously by csn, so a frame
//       aborted mid-way (csn released early) no longer misaligns later frames.
//  [21] irq pad is a level: set when a new result is captured, cleared after
//       the host completes an SPI frame that returned it (was a 10 ns pulse).
//  [22] MISO status bit [2] = error = prog_fail | op_timeout (controller gave
//       up programming a weight, or a watchdog aborted the last command);
//       cleared when the next command is accepted.
//  [23] READ result returned to the host: the response to the frame that
//       follows a READ frame is {8'h00, addr[15:0], data[7:0], status}.
//  [24] Controller watchdogs (OP_DONE_TIMEOUT / COLLECT_TIMEOUT) exposed as
//       wrapper parameters.
//
//  CHANGES from Controller_vs_datapath_review (Roman, 2026-11-16)
//  ──────────────────────────────────────────────────────────────
//  [25] New output 'start': adder-tree sample strobe = plane-active
//       (pulses == INF) delayed by ADC_DELAY cycles (parameter, value from
//       Dima; placeholder 2).  Without it the tree never samples.
//  [26] act_start (renamed from activation_start) now pulses one cycle after
//       the 8th tree_result_valid, i.e. the earliest correct cycle for any
//       ADC delay.  Replaces the fixed TREE_DSA_LATENCY delay (wrong if N > 2).
//  [27] Datapath ports flattened (no 2-D unpacked array ports) and widened
//       to NUM_TREES = 16 channels (4 trees x 4 PEs):
//         tree_result_valid[15:0], acc_out[16*19-1:0], act_valid[15:0],
//         act_out[16*8-1:0]   (tree t at [t*W +: W]).  Renamed to the
//       datapath's names (act_valid / act_out).
//  [28] irq = all 16 act_valid; the inference result is 128 bits.  PISO
//       shifts 128 bits; MISO returns it as 4 x 32-bit words (word selected
//       by the payload of the preceding HIZ frame, see MISO section).
//  [29] Clock comment: system clock per datapath flow is 10 MHz (100 ns).
//       All timing in this file is in clk cycles.
//  Already satisfied from the review: 256 input lanes; full PE/SA/ROW/COL
//  addressing; MISO clock-domain crossing.
//
//  CHANGES Rev 5 – column sweep + MSB first  (Nir, 2026-11-18)
//  ──────────────────────────────────────────────────────────
//  Follows NEMO CS-IMC Architecture Brief Rev. 1.0 §6 and the 2026-11-18
//  decision on bit order (sketch "WhatsApp Image 2026-11-18 at 16.58.03").
//  [30] Bit-plane order MSB first (review §4, option A): for every column
//       position the planes b7, b6, ..., b0 are applied in that order, so the
//       datapath DSA acc <= (acc << 1) + sum is correct unchanged.  Closes the
//       open bit-order item of the review.
//  [31] Column sweep: new output col_sel[3:0] to the analog core = the
//       column-mux select, common to the 4 muxes of every SA in every PE.  It
//       holds column position k (0..15) for all 8 planes, then advances.
//       One inference = 16 column positions (was: a single position).
//  [32] Bit-step timing per brief Fig. 3: each plane is held for settle (2) +
//       conversion (8) cycles (pulses = INF), the tree samples at cycle 10.
//       'start' is now the controller's tree_start strobe; the ADC_DELAY
//       parameter is removed (the ADC timing is in controller_pkg).
//  [33] dsa_clear pulses at the start of every column position (16 per
//       inference): the DSA's 4 accumulators per PE are reused per column.
//       act_start still pulses one cycle after the 8th tree_result_valid,
//       now once per column (16 per inference).
//  [34] Output buffer 256 x 8 (brief Fig. 1): after each column the 16
//       act_out bytes are written to out[pe*64 + g*16 + k] (tree t = pe*4+g,
//       column position k).  irq is raised once, when all 16 columns are in.
//  [35] Result returned over MISO as 64 x 32-bit words (word j = bytes
//       4j+3..4j), word selected by payload[5:0] of the preceding HIZ frame.
//       PISO streams the 2048-bit result, byte 255 bit 7 first.
//  [36] The previous result is invalidated by the controller's inf_start
//       (last pixel accepted) instead of dsa_clear (now per column).
//
//  ARCHITECTURE (from Drawing_16x64_SA diagram – ground truth)
//  ─────────────────────────────────────────────────────────────
//   1 Core  =  4 PEs
//   1 PE    = 16 SAs
//   1 SA    = 16 rows  ×  64 cols   (Y-Flash 16×64 array)
//   Weight  =  4 bits  (Y-Flash conductance level)
//   Data    =  8 bits INT8, streamed bit-serially (8 cycles / vector)
//
//  HOST WORD FORMAT (packed binary – from SPI frame)
//  ─────────────────────────────────────────────────
//   host_data[31:24]  payload  (8-bit weight nibble or pixel byte)
//   host_data[23:22]  PE       (2-bit binary, 0-3)
//   host_data[21:18]  SA       (4-bit binary, 0-15)
//   host_data[17:14]  ROW      (4-bit binary, 0-15)
//   host_data[13:8]   COL      (6-bit binary, 0-63)
//   host_data[7:0]    reserved / zero (non-zero → frame rejected)
//
//  HOST COMMAND SEQUENCES (one SPI frame per line)
//  ───────────────────────────────────────────────
//   PROG  : {weight, addr, PROG}            → program + verify one weight
//   ERASE : {x,      addr, ERASE}           → erase one weight
//   READ  : {x,      addr, READ}            → read pulse train at addr
//   INF   : {x, addr, INF} then 256 frames {pixel_k, x, non-HIZ cmd}
//           (k = 0..255, pixel k → SA k/16, row k%16) → 16 column positions
//           x 8 bit-planes (MSB first) = 2176 compute cycles → 256 results
//
// =============================================================================

`timescale 1ns/1ps

module nemo_top_wrapper #(
    parameter string WEIGHT_PULSE_LUT_FILE =
        "ann-accelerator-controller/target/Controller/programming_inputs/weight_pulse_lut.mem",
    // Controller watchdogs in clk cycles (FIX #24)
    parameter int OP_DONE_TIMEOUT = controller_pkg::DEFAULT_OP_DONE_TIMEOUT,  // analog op_done wait
    parameter int COLLECT_TIMEOUT = controller_pkg::DEFAULT_COLLECT_TIMEOUT   // gap between pixel frames
    // (ADC_DELAY removed in Rev 5: the ADC conversion time is the controller's
    //  T_INF_SETTLE + T_INF_ADC per bit-step, see controller_pkg)
) (

    // -------------------------------------------------------------------------
    //  PAD: Clock & Reset
    // -------------------------------------------------------------------------
    input  logic        clk,            // system clock (10 MHz / 100 ns in the datapath flow)
    input  logic        rst_n,          // active-low async chip reset  (pad 34)

    // -------------------------------------------------------------------------
    //  PAD: SPI host interface  (pads 30-33)
    //  35-bit frame MSB-first: [34:3] host_data[31:0], [2:0] host_cmd[2:0]
    // -------------------------------------------------------------------------
    input  logic        sclk,           // SPI clock from host
    input  logic        mosi,           // host → chip data
    output logic        miso,           // chip → host readback
    input  logic        csn,            // SPI chip-select, active-low

    // -------------------------------------------------------------------------
    //  PAD: Control outputs to host  (pads 35-36)
    // -------------------------------------------------------------------------
    output logic        irq,            // inference-done interrupt to host
    input  logic        scan_en,        // DFT scan-enable

    // -------------------------------------------------------------------------
    //  PAD: PISO serialised output  (pad 29)
    // -------------------------------------------------------------------------
    output logic        piso_out,       // serialised DSA result stream

    // -------------------------------------------------------------------------
    //  PAD: DFT output taps  (pads 27-28)  – driven by analog blocks
    // -------------------------------------------------------------------------
    output logic        dft0_buff,      // VTC/TDC test tap
    output logic        dft1_buff,      // TDC/DSA test tap

    // =========================================================================
    //  ANALOG ANN CORE BOUNDARY
    // =========================================================================
    output logic                                         ann_reset,
    output logic [ann_arch_pkg::WORD_WIDTH-1:0]          ann_core_word,  // {payload[31:24], PE,SA,ROW,COL [23:8], 8'h00}
    output logic [2:0]                                   pulses,         // HIZ/READ/PROG/ERASE/INF
    output logic [ann_arch_pkg::INPUT_VECTOR_LEN-1:0]    ann_input_bits, // bit-serial activations, lane i → SA i/16, row i%16
    output logic [ann_arch_pkg::COL_SEL_WIDTH-1:0]       col_sel,        // column-mux select (all muxes, all SAs/PEs) – FIX #31

    input  logic                                         op_done,
    input  logic [ann_arch_pkg::ANN_WEIGHT_WIDTH-1:0]    weight_read_data,

    // =========================================================================
    //  DIGITAL DATAPATH BOUNDARY  (Adder Tree / DSA / ReLU)
    // =========================================================================
    // Packed per-tree buses: tree t (0..15, t = PE*4 + ADC) at [t*W +: W]
    output logic                                                         dsa_clear,  // sync clear of the DSA accumulators, start of every column (16x)
    output logic                                                         start,      // tree sample strobe, once per bit-step (128 per inference)
    output logic                                                         act_start,  // start activation / saturation, once per column (16x)
    output logic [ann_arch_pkg::BIT_SEL_WIDTH-1:0]                       bit_idx,    // plane on the rows (MSB first); not used by the datapath
    output logic [ann_arch_pkg::PE_ID_WIDTH-1:0]                         pe_sel,     // not used by the datapath (future output mux)

    input  logic [ann_arch_pkg::NUM_TREES-1:0]                           tree_result_valid,
    input  logic [ann_arch_pkg::NUM_TREES*ann_arch_pkg::ACC_WIDTH-1:0]   acc_out,
    input  logic [ann_arch_pkg::NUM_TREES-1:0]                           act_valid,
    input  logic [ann_arch_pkg::NUM_TREES*ann_arch_pkg::ACT_WIDTH-1:0]   act_out
);

    // =========================================================================
    //  Local architecture parameters  (all from ann_arch_pkg – FIX #9)
    // =========================================================================
    localparam int NUM_PE          = ann_arch_pkg::NUM_PE;            // 4
    localparam int NUM_SA          = ann_arch_pkg::NUM_SA;            // 16
    localparam int SA_ROWS         = ann_arch_pkg::SA_ROWS;           // 16
    localparam int SA_COLS         = ann_arch_pkg::SA_COLS;           // 64

    localparam int PE_ID_WIDTH     = ann_arch_pkg::PE_ID_WIDTH;       // 2
    localparam int SA_ID_WIDTH     = ann_arch_pkg::SA_ID_WIDTH;       // 4
    localparam int ROW_ID_WIDTH    = ann_arch_pkg::ROW_ID_WIDTH;      // 4
    localparam int COL_ID_WIDTH    = ann_arch_pkg::COL_ID_WIDTH;      // 6

    // Packed binary address {PE, SA, ROW, COL}
    localparam int ADDR_WIDTH      = ann_arch_pkg::WEIGHT_ADDR_WIDTH; // 16

    localparam int WEIGHT_WIDTH    = ann_arch_pkg::ANN_WEIGHT_WIDTH;  // 4
    localparam int BUFFER_DATA_W   = input_buffer_pkg::BUFFER_DATA_WIDTH;  // 8
    localparam int BUFFER_ADDR_W   = input_buffer_pkg::BUFFER_ADDR_WIDTH;  // 8
    localparam int BIT_SEL_W       = input_buffer_pkg::BUF_BIT_SEL_WIDTH;  // 3
    localparam int NUM_LANES       = input_buffer_pkg::NUM_DATA_LINES;     // 256

    localparam int WORD_W          = ann_arch_pkg::WORD_WIDTH;             // 32
    localparam int CMD_W           = parallel_interface_pkg::CMD_WIDTH;    // 3

    // SPI: 35-bit frame (32-bit data + 3-bit cmd), MSB first
    localparam int SPI_FRAME_BITS  = WORD_W + CMD_W;                  // 35
    localparam int SPI_CNT_W       = $clog2(SPI_FRAME_BITS);          // 6

    // Inference result: 256 outputs x 8 bit = 2048 bits (output buffer, PISO, MISO)
    localparam int NUM_TREES       = ann_arch_pkg::NUM_TREES;           // 16 (act_out channels)
    localparam int ADCS_PER_SA     = ann_arch_pkg::ADCS_PER_SA;         // 4 trees per PE
    localparam int ACT_W           = ann_arch_pkg::ACT_WIDTH;           // 8
    localparam int NUM_COL_POS     = ann_arch_pkg::NUM_COL_POS;         // 16 column positions
    localparam int COL_SEL_W       = ann_arch_pkg::COL_SEL_WIDTH;       // 4
    localparam int OUT_BITS        = ann_arch_pkg::OUTPUT_BITS;         // 2048
    localparam int PISO_CNT_W      = $clog2(OUT_BITS + 1);              // 12
    localparam int RESULT_WORDS    = OUT_BITS / WORD_W;                 // 64 MISO words
    localparam int RSP_WORD_W      = (RESULT_WORDS > 1) ? $clog2(RESULT_WORDS) : 1;  // 6
    localparam int COLS_CNT_W      = $clog2(NUM_COL_POS + 1);           // 5

    // Bit-planes per column = tree results to count before act_start
    localparam int NUM_PLANES      = ann_arch_pkg::ACT_WIDTH;           // 8
    localparam int PLANE_CNT_W     = $clog2(NUM_PLANES);                // 3

    // Pulse-mode encoding on 'pulses' (controller_pkg::pulse_mode_t)
    localparam logic [2:0] PULSE_HIZ   = 3'b000;
    localparam logic [2:0] PULSE_PROG  = 3'b010;
    localparam logic [2:0] PULSE_ERASE = 3'b011;

    // =========================================================================
    //  Internal signal declarations
    // =========================================================================

    // Reset synchroniser (FIX #19)
    logic                      rst_meta, rst_n_clk;   // rst_n_clk: clk-domain reset

    // SPI / host bus
    logic                      spi_cnt_rst;       // csn | reset -> realign bit counter (FIX #20)
    logic [SPI_FRAME_BITS-1:0] spi_sreg;          // sclk domain shift register
    logic [SPI_FRAME_BITS-1:0] spi_frame;         // sclk domain: last complete frame (stable)
    logic [SPI_CNT_W-1:0]      spi_bit_cnt;       // counts 0..34 in sclk domain
    logic                      frame_tgl_sclk;    // toggles once per complete frame
    logic                      ft_s0, ft_s1, ft_s2; // 2-FF sync + edge detect in clk domain
    logic                      frame_done_clk;    // 1-cycle pulse per frame

    logic [WORD_W-1:0] host_data;        // packed binary word from SPI frame
    logic [CMD_W-1:0]  host_cmd;
    logic              host_valid;       // 1-cycle strobe per received frame

    // parallel_interface outputs
    logic [ADDR_WIDTH-1:0]    pi_addr_out;
    logic [BUFFER_DATA_W-1:0] pi_data_out;
    logic [CMD_W-1:0]         pi_cmd_out;
    logic                     pi_valid_out;
    logic                     ctrl_valid;

    // Input buffer control (controller → buffer)
    logic [BUFFER_ADDR_W-1:0] buf_reg_add;
    logic [2:0]               buf_reg_ctrl;
    logic                     buf_read_write;
    logic [BIT_SEL_W-1:0]     buf_bit_sel;
    logic [BUFFER_DATA_W-1:0] buf_data_to_buf;

    // Input buffer status (buffer → controller)
    logic                     buf_ready_sig;
    logic [BUFFER_DATA_W-1:0] buf_data_from_buf;

    // Bit-serial lanes (one per global input row)
    logic [NUM_LANES-1:0]     pixel_bit_lanes;

    // Controller status / datapath strobes
    logic        ctrl_busy;
    logic        ctrl_inf_start;                   // last pixel accepted: new inference
    logic        ctrl_dsa_clear;                   // start of every column position
    logic        ctrl_tree_start;                  // ADC codes valid, once per bit-step
    logic [COL_SEL_W-1:0] ctrl_col_sel;            // column-mux select
    logic        ctrl_compute_done;
    logic        ctrl_prog_fail;
    logic        ctrl_op_timeout;
    logic        ctrl_error;                       // prog_fail | op_timeout
    logic [WEIGHT_WIDTH-1:0] ctrl_read_data;
    logic        ctrl_read_done;

    // READ result for the host (FIX #23)
    logic [ADDR_WIDTH-1:0]   read_addr_reg;    // address of the last READ
    logic                    read_valid;       // a READ has completed since reset
    logic                    rsp_is_read;      // next MISO frame returns the READ result
    logic [RSP_WORD_W-1:0]   rsp_word;         // result word returned by the next frame

    // Decoded fields of the word currently driven to the core
    logic [ADDR_WIDTH-1:0] core_addr;

    // DSA coordination
    logic [PLANE_CNT_W-1:0] tv_cnt;                  // tree results counted in this column (0..7)
    logic [COL_SEL_W-1:0]   cap_col;                 // column position whose act_out is pending

    // Column capture / IRQ / PISO
    logic        col_capture;        // 1 clk: all trees report act_valid -> store the column
    logic [COLS_CNT_W-1:0] cols_captured;  // columns stored in this inference (0..16)
    logic        irq_pulse;          // 1 clk when the 16th column is stored
    logic        irq_flag;           // level on the irq pad (FIX #21)
    logic        word_irq;           // irq_flag value captured with miso_word
    logic        frame_had_irq;      // current frame returns the pending result
    logic        all_pe_valid;
    logic        all_pe_valid_prev;
    logic [PISO_CNT_W-1:0]      piso_cnt;
    logic        piso_running;

    // MISO readback (FIX #18)
    logic                      csn_s0, csn_s1, csn_s2; // csn synchronised into clk
    logic [OUT_BITS-1:0]       result_reg;       // output buffer 256 x 8: byte o at [o*8 +: 8]
    logic                      result_valid;
    logic [SPI_FRAME_BITS-1:0] miso_word;        // response frame, stable while csn low
    logic [SPI_CNT_W-1:0]      miso_idx;         // sclk-domain bit index
    logic                      miso_idx_rst;

    // Internal formerly-external signals
    logic        pulse_internal;
    logic        set_internal;
    logic        rst_dsa_internal;
    logic [ROW_ID_WIDTH-1:0] row_sel_internal;
    logic        clk_sipo_internal;
    logic        sipo_cs_internal;
    logic [3:0]  dft_s_sel;
    logic [2:0]  dft_b_sel;
    logic        _unused;

    // =========================================================================
    //  Reset synchroniser  (FIX #19)
    //  Asynchronous assertion, release synchronised to clk.  All clk-domain
    //  logic (controller, input buffer, parallel interface, wrapper glue)
    //  uses rst_n_clk.  The sclk-domain SPI logic keeps the raw rst_n:
    //  sclk is not free-running, so the host must keep sclk idle while
    //  rst_n is released (standard SPI reset requirement).
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rst_meta  <= 1'b0;
            rst_n_clk <= 1'b0;
        end else begin
            rst_meta  <= 1'b1;
            rst_n_clk <= rst_meta;
        end
    end

    // =========================================================================
    //  SPI Slave  (FIX #15: frame holding register + toggle synchroniser)
    // =========================================================================

    // --- sclk domain: bit counter, reset by csn high (FIX #20) ---
    // csn high asynchronously clears the counter, so every frame starts at
    // bit 0 even if the previous frame was aborted before its 35th bit.
    assign spi_cnt_rst = csn | ~rst_n;

    always_ff @(posedge sclk or posedge spi_cnt_rst) begin
        if (spi_cnt_rst)
            spi_bit_cnt <= '0;
        else if (spi_bit_cnt == SPI_CNT_W'(SPI_FRAME_BITS - 1))
            spi_bit_cnt <= '0;
        else
            spi_bit_cnt <= spi_bit_cnt + 1'b1;
    end

    // --- sclk domain: shift register, frame capture, frame toggle ---
    // On the 35th bit the complete frame is copied into spi_frame, which then
    // stays stable for at least one full frame time while the clk domain
    // samples it.  frame_tgl_sclk flips once per frame.
    always_ff @(posedge sclk or negedge rst_n) begin
        if (!rst_n) begin
            spi_sreg       <= '0;
            spi_frame      <= '0;
            frame_tgl_sclk <= 1'b0;
        end else if (!csn) begin
            spi_sreg <= {spi_sreg[SPI_FRAME_BITS-2:0], mosi};
            if (spi_bit_cnt == SPI_CNT_W'(SPI_FRAME_BITS - 1)) begin
                spi_frame      <= {spi_sreg[SPI_FRAME_BITS-2:0], mosi};
                frame_tgl_sclk <= ~frame_tgl_sclk;   // exactly 35 bits received
            end
        end
    end

    // --- clk domain: 2-FF synchroniser + edge detect on the frame toggle ---
    always_ff @(posedge clk or negedge rst_n_clk) begin
        if (!rst_n_clk) begin
            ft_s0 <= 1'b0;
            ft_s1 <= 1'b0;
            ft_s2 <= 1'b0;
        end else begin
            ft_s0 <= frame_tgl_sclk;
            ft_s1 <= ft_s0;
            ft_s2 <= ft_s1;
        end
    end
    assign frame_done_clk = ft_s1 ^ ft_s2;   // one clk pulse per received frame

    // --- clk domain: latch frame on frame_done ---
    always_ff @(posedge clk or negedge rst_n_clk) begin
        if (!rst_n_clk) begin
            host_data  <= '0;
            host_cmd   <= parallel_interface_pkg::CMD_HIZ;
            host_valid <= 1'b0;
        end else begin
            host_valid <= 1'b0;
            if (frame_done_clk) begin
                host_data  <= spi_frame[SPI_FRAME_BITS-1 -: WORD_W];  // bits [34:3]
                host_cmd   <= spi_frame[CMD_W-1:0];                    // bits [2:0]
                host_valid <= 1'b1;
            end
        end
    end

    // =========================================================================
    //  Parallel Interface instantiation  (FIX #10: packed host word, no remap)
    // =========================================================================
    parallel_interface u_parallel_interface (
        .clk        (clk),
        .reset      (~rst_n_clk),          // active-high reset
        .host_data  (host_data),
        .host_cmd   (host_cmd),
        // Outputs
        .valid      (pi_valid_out),        // cmd != HIZ && word well-formed (level)
        .data       (pi_data_out),
        .address    (pi_addr_out),         // packed {PE,SA,ROW,COL}
        .cmd        (pi_cmd_out)
    );

    // One beat per received frame, only if the frame is well-formed (FIX #11)
    assign ctrl_valid = host_valid & pi_valid_out;

    // =========================================================================
    //  Input Buffer instantiation  (256 x 8, 256 bit-serial lanes)
    // =========================================================================
    input_buffer u_input_buffer (
        .clk            (clk),
        .rst_n          (rst_n_clk),
        .data_in        (buf_data_to_buf),
        .bit_sel        (buf_bit_sel),
        .reg_ctrl       (buf_reg_ctrl),
        .buf_read_write (buf_read_write),
        .buf_reg_add    (buf_reg_add),
        .ready          (buf_ready_sig),
        .buf_data       (buf_data_from_buf),
        .data_lanes     (pixel_bit_lanes)
    );

    // Bit-serial activations to the array rows (FIX #13)
    assign ann_input_bits = pixel_bit_lanes;

    // =========================================================================
    //  ANN Controller instantiation
    // =========================================================================
    ann_controller #(
        .ADDR_WIDTH           (ADDR_WIDTH),
        .WEIGHT_WIDTH         (WEIGHT_WIDTH),
        .USE_WEIGHT_PULSE_LUT (1'b1),
        .WEIGHT_PULSE_LUT_FILE(WEIGHT_PULSE_LUT_FILE),
        .OP_DONE_TIMEOUT      (OP_DONE_TIMEOUT),
        .COLLECT_TIMEOUT      (COLLECT_TIMEOUT)
    ) u_ann_controller (
        .clk              (clk),
        .rst_n            (rst_n_clk),
        // From parallel_interface
        .valid            (ctrl_valid),
        .data             (pi_data_out),
        .address          (pi_addr_out),
        .cmd              (pi_cmd_out),
        // Analog core
        .op_done          (op_done),
        .weight_read_data (weight_read_data),
        .ann_reset        (ann_reset),
        .ann_address      (ann_core_word),
        .pulses           (pulses),
        // Input buffer
        .buf_reg_add      (buf_reg_add),
        .buf_reg_ctrl     (buf_reg_ctrl),
        .buf_read_write   (buf_read_write),
        .buf_bit_sel      (buf_bit_sel),
        .buf_data_out     (buf_data_to_buf),
        .buf_ready        (buf_ready_sig),
        .buf_data         (buf_data_from_buf),
        // Digital datapath strobes / column sweep
        .inf_start        (ctrl_inf_start),
        .dsa_clear        (ctrl_dsa_clear),
        .tree_start       (ctrl_tree_start),
        .col_sel          (ctrl_col_sel),
        .compute_done     (ctrl_compute_done),
        // Error status
        .prog_fail        (ctrl_prog_fail),
        .op_timeout       (ctrl_op_timeout),
        // READ result
        .read_data        (ctrl_read_data),
        .read_done        (ctrl_read_done),
        // Status
        .busy             (ctrl_busy)
    );

    // Packed address currently driven to the core (binary fields, FIX #12)
    assign core_addr = ann_arch_pkg::word_addr(ann_core_word);

    // =========================================================================
    //  DSA / Adder-tree coordination  (FIX #14)
    // =========================================================================

    // dsa_clear (FIX #33): first cycle of every column position.  The DSA's
    // accumulators are reused for each column, so they are cleared 16 times
    // per inference, each time before the column's first tree result.
    assign dsa_clear = ctrl_dsa_clear;

    // col_sel (FIX #31): column-mux select to the analog core
    assign col_sel = ctrl_col_sel;

    // bit_idx: plane currently on the rows (7 -> 0 for every column)
    assign bit_idx = buf_bit_sel;

    // pe_sel: PE field of the packed core address
    assign pe_sel = ann_arch_pkg::get_pe_id(core_addr);

    // start (FIX #32): adder-tree sample strobe = controller tree_start, i.e.
    // T_INF_SETTLE + T_INF_ADC cycles after each plane is driven, when the
    // conversion of that plane has finished.  One pulse per bit-step.
    assign start = ctrl_tree_start;

    // act_start (FIX #26/#33): one cycle after the 8th tree_result_valid of a
    // column.  All trees run in lock-step, so tree 0 is representative.  The
    // count is cleared by dsa_clear at the start of every column.  The column
    // position is latched with it, to address the output buffer when the
    // activation result comes back.
    always_ff @(posedge clk or negedge rst_n_clk) begin
        if (!rst_n_clk) begin
            tv_cnt    <= '0;
            act_start <= 1'b0;
            cap_col   <= '0;
        end else begin
            act_start <= tree_result_valid[0] && (tv_cnt == PLANE_CNT_W'(NUM_PLANES - 1));
            if (tree_result_valid[0] && (tv_cnt == PLANE_CNT_W'(NUM_PLANES - 1)))
                cap_col <= ctrl_col_sel;
            if (ctrl_dsa_clear)
                tv_cnt <= '0;
            else if (tree_result_valid[0])
                tv_cnt <= tv_cnt + 1'b1;              // wraps 7 -> 0
        end
    end

    // =========================================================================
    //  Output buffer 256 x 8 (FIX #34)
    //  col_capture: all 16 trees report act_valid (rising edge).  The 16
    //  act_out bytes of column position cap_col are stored at
    //      out[pe*64 + g*16 + cap_col]   for tree t = pe*4 + g
    //  (neuron g*16 + k of PE pe, matching the column-group mapping of the
    //  architecture: ADC/tree g serves columns g*16 .. g*16+15).
    //  irq_pulse when the 16th column of the inference has been stored.
    // =========================================================================
    assign all_pe_valid = &act_valid;
    assign col_capture  = all_pe_valid & ~all_pe_valid_prev;

    always_ff @(posedge clk or negedge rst_n_clk) begin
        if (!rst_n_clk) begin
            all_pe_valid_prev <= 1'b0;
            irq_pulse         <= 1'b0;
            cols_captured     <= '0;
            result_reg        <= '0;
        end else begin
            all_pe_valid_prev <= all_pe_valid;
            irq_pulse         <= 1'b0;
            if (ctrl_inf_start) begin
                cols_captured <= '0;                       // new inference
            end else if (col_capture) begin
                for (int t = 0; t < NUM_TREES; t++)
                    result_reg[((t / ADCS_PER_SA) * SA_COLS + (t % ADCS_PER_SA) * NUM_COL_POS
                                + int'(cap_col)) * ACT_W +: ACT_W] <= act_out[t * ACT_W +: ACT_W];
                cols_captured <= cols_captured + 1'b1;
                if (cols_captured == COLS_CNT_W'(NUM_COL_POS - 1))
                    irq_pulse <= 1'b1;                     // all 16 columns stored
            end
        end
    end

    // =========================================================================
    //  PISO readout controller
    //  Streams the 2048-bit output buffer on irq_pulse, MSB first
    //  (byte 255 bit 7 first, byte 0 bit 0 last).  Reads result_reg by index,
    //  so no second 2048-bit register is needed.
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n_clk) begin
        if (!rst_n_clk) begin
            piso_cnt     <= '0;
            piso_running <= 1'b0;
        end else begin
            if (irq_pulse && !piso_running) begin
                piso_cnt     <= '0;
                piso_running <= 1'b1;
            end else if (piso_running) begin
                piso_cnt <= piso_cnt + 1'b1;
                if (piso_cnt == PISO_CNT_W'(OUT_BITS - 1))
                    piso_running <= 1'b0;
            end
        end
    end

    assign piso_out = piso_running ? result_reg[OUT_BITS - 1 - int'(piso_cnt)] : 1'b0;

    // =========================================================================
    //  MISO readback (SPI mode 0)  – FIX #18
    // =========================================================================
    //  Every 35-bit host frame clocks out a 35-bit response, MSB first:
    //  Default response (inference result, 256 x 8 = 2048 bits = 64 words, FIX #35):
    //     [34:3] result word j = output bytes 4j+3 .. 4j (byte 4j in [10:3]);
    //            output byte o = neuron o: o = pe*64 + g*16 + k
    //            (PE pe, tree g, column position k).
    //            j = payload[5:0] (host_data[29:24]) of the PRECEDING HIZ
    //            frame (0 after reset / after any other command).  Reading all
    //            64 words: HIZ(j=0), HIZ(j=1)->word0, ..., HIZ(j=0)->word63.
    //     [2]    error       prog_fail | op_timeout of the last command (FIX #22)
    //     [1]    busy        controller busy when the frame started
    //     [0]    valid       result_valid: set when all 16 columns are stored,
    //                        cleared by inf_start (FIX #36)
    //  Response to the frame AFTER a valid READ frame (FIX #23):
    //     [34:27] 8'h00
    //     [26:11] address of the READ  {PE, SA, ROW, COL}
    //     [10:3]  read data (weight, zero-extended to 8 bits)
    //     [2:1]   error, busy        (as above)
    //     [0]     valid = a READ has completed since reset
    //  The READ completes ~10 clk after its frame ends: keep csn high for
    //  >= 20 clk before the collecting frame, or re-read while busy = 1.
    //  A HIZ frame is a pure read (and switches back to the default response).
    //
    //  Timing (mode 0): bit 34 is on miso as soon as csn falls; each further
    //  bit changes on the falling edge of sclk; the host samples on the rising
    //  edge.  miso = 0 while csn is high.
    //
    //  CDC: miso_word (clk domain) is only updated while the bus is idle
    //  (csn high, 2-FF synchronised), so it is stable for the whole frame.
    //  Requirement: csn falling edge to first sclk rising edge >= 4 clk
    //  periods (400 ns at 10 MHz, 40 ns at 100 MHz).
    //
    //  irq (FIX #21): irq_flag is set when a result is captured and cleared
    //  when a complete frame that started while it was set has been received,
    //  i.e. after the host has read the result.  A new result always wins.
    // =========================================================================

    // --- clk domain: result capture + transmit word ---
    always_ff @(posedge clk or negedge rst_n_clk) begin
        if (!rst_n_clk) begin
            csn_s0        <= 1'b1;
            csn_s1        <= 1'b1;
            csn_s2        <= 1'b1;
            result_valid  <= 1'b0;
            miso_word     <= '0;
            irq_flag      <= 1'b0;
            word_irq      <= 1'b0;
            frame_had_irq <= 1'b0;
            read_addr_reg <= '0;
            read_valid    <= 1'b0;
            rsp_is_read   <= 1'b0;
            rsp_word      <= '0;
        end else begin
            csn_s0 <= csn;
            csn_s1 <= csn_s0;
            csn_s2 <= csn_s1;

            // result_reg is written column by column (output-buffer block above)
            if (irq_pulse) begin
                result_valid <= 1'b1;          // all 16 columns stored
            end else if (ctrl_inf_start) begin
                result_valid <= 1'b0;          // new inference: old result is stale
            end

            // READ result capture and response selection (FIX #23)
            if (ctrl_read_done) begin
                read_addr_reg <= core_addr;
                read_valid    <= 1'b1;
            end
            if (host_valid) begin              // response type follows the last frame
                rsp_is_read <= pi_valid_out && (host_cmd == parallel_interface_pkg::CMD_READ);
                rsp_word    <= (host_cmd == parallel_interface_pkg::CMD_HIZ)
                               ? host_data[ann_arch_pkg::WORD_DATA_LSB +: RSP_WORD_W] : '0;
            end

            if (csn_s1) begin                  // bus idle → safe to update
                if (rsp_is_read) begin
                    miso_word <= {8'h00, read_addr_reg, 8'(ctrl_read_data),
                                  ctrl_error, ctrl_busy, read_valid};
                    word_irq  <= 1'b0;         // this frame does not carry the result
                end else begin
                    miso_word <= {result_reg[rsp_word*WORD_W +: WORD_W],
                                  ctrl_error, ctrl_busy, result_valid};
                    word_irq  <= irq_flag;
                end
            end

            // frame start (synchronised csn falling edge): does it carry the result?
            if (csn_s2 && !csn_s1)
                frame_had_irq <= word_irq;

            // irq level: a new result sets it; a completed read frame clears it
            if (irq_pulse)
                irq_flag <= 1'b1;
            else if (frame_done_clk && frame_had_irq)
                irq_flag <= 1'b0;
            if (frame_done_clk)
                frame_had_irq <= 1'b0;
        end
    end

    assign irq        = irq_flag;
    assign ctrl_error = ctrl_prog_fail | ctrl_op_timeout;

    // --- sclk domain: bit index, advanced on each falling edge of sclk ---
    assign miso_idx_rst = csn | ~rst_n;

    always_ff @(negedge sclk or posedge miso_idx_rst) begin
        if (miso_idx_rst)
            miso_idx <= '0;
        else if (miso_idx != SPI_CNT_W'(SPI_FRAME_BITS))
            miso_idx <= miso_idx + 1'b1;
    end

    assign miso = (!csn && (miso_idx < SPI_CNT_W'(SPI_FRAME_BITS)))
                  ? miso_word[SPI_FRAME_BITS - 1 - miso_idx]
                  : 1'b0;

    // =========================================================================
    //  Internal signals (formerly external pads in Feb tape-out)
    // =========================================================================

    // HV program pulse – high during PROG_WRITE (pulses == PROG encoding)
    assign pulse_internal   = (pulses == PULSE_PROG);

    // Set/latch strobe – one cycle after op_done during select phase
    assign set_internal     = op_done & ((pulses == PULSE_PROG) | (pulses == PULSE_ERASE));

    // DSA accumulator reset – alias of dsa_clear (per column)
    assign rst_dsa_internal = dsa_clear;

    // Row mux select (s0_p..s2_p) – ROW field of the packed core address
    assign row_sel_internal = ann_arch_pkg::get_row_id(core_addr);

    // SIPO clock/cs – gate clk to SIPO only during active pulse phases
    assign clk_sipo_internal = clk & (pulses != PULSE_HIZ);
    assign sipo_cs_internal  = (pulses != PULSE_HIZ);

    // =========================================================================
    //  DFT – scan_en disables DFT mux selects
    //  Values will come from an SPI test register (not yet implemented)
    // =========================================================================
    assign dft_s_sel = 4'b0000;   // replace with SPI test register output
    assign dft_b_sel = 3'b000;

    assign dft0_buff = 1'b0;      // connect to analog DFT tap when available
    assign dft1_buff = 1'b0;      // connect to analog DFT tap when available

    // =========================================================================
    //  Suppress unused-signal lint warnings
    // =========================================================================
    assign _unused = ^tree_result_valid[NUM_TREES-1:1] ^ ^acc_out
                   ^ ctrl_compute_done ^ cols_captured[COLS_CNT_W-1]
                   ^ scan_en
                   ^ ^dft_s_sel ^ ^dft_b_sel
                   ^ pulse_internal ^ set_internal ^ rst_dsa_internal
                   ^ clk_sipo_internal ^ sipo_cs_internal
                   ^ ^row_sel_internal;

endmodule : nemo_top_wrapper

// =============================================================================
//  End of nemo_top_wrapper.sv
// =============================================================================
