# NEMO digital datapath RTL: one AT slice (Roman)

SystemVerilog for one "AT slice" of the PE: 16 SA macros â†’ 16-input adder tree â†’ shift-accumulator (DSA) â†’ saturation.
This is the exact RTL used for the current layout (lab: `~/Backend180/NEMO_SipoPiso/syn/tsl018_syn_2223/sv_files_start_sat_ant/`, version of 2026-11-10).
Technology: Tower 0.18 Âµm (tsl18), clock 100 ns (10 MHz), core 1.8 V.

## Hierarchy

```
sa_to_adder_tree_wrapper          (top of the slice)
â”œâ”€â”€ SA_macro        x16           black-box stub of Dima's analog SA (no behavior)
â”œâ”€â”€ addr_tree_Nir_flow  u_addr_tree   16 x 7-bit -> 11-bit sum, pipelined, latency 4 cycles
â”œâ”€â”€ dsa                 u_dsa         acc <= (acc << 1) + sum on each out_valid, 19 bits
â””â”€â”€ act_sat             u_act_sat     act_out = |acc[18:8] ? 255 : acc[7:0], registered
```

## Files

| File | What it is |
|---|---|
| `sa_to_adder_tree_wrapper.sv` | Top of the slice. Instantiates 16 `SA_macro`, the tree, the DSA and `act_sat`. |
| `addr_tree_Nir_flow.sv` | Adder tree. Samples `SA_OUT0..15` when `start` = 1; `out_valid` 4 cycles later. |
| `dsa.sv` | Digital shift-accumulator. Accumulates on `tree_result_valid` (= tree `out_valid`). |
| `act_sat.sv` | Saturation to 8 bits. `act_valid` 1 cycle after `act_start`. ReLU not included yet. |
| `SA_macro.sv` | Placeholder for the analog SA: ports only, no logic. |
| `antenna_sim_stub.sv` | **Simulation only.** Empty model of the antenna-diode cell `ANTENNA_18_SVT_WTN`. Do not synthesize it. |
| `tb_addr_tree_Nir_flow.sv` | Testbench: adder tree alone. |
| `tb_tree_dsa_act.sv` | Testbench: tree â†’ DSA â†’ act_sat, 8 bit-planes Ã— 2 inferences (results 176 and 255). |
| `run_sim.sh` | VCS commands for both testbenches and a wrapper elaboration check. |

## Slice interface (signals that talk to the controller)

| Signal | Dir | Meaning |
|---|---|---|
| `clk`, `reset` | in | 100 ns clock; `reset` is active high at the wrapper (internally `rst_n = ~reset`). |
| `start` | in | 1-cycle pulse per bit-plane: the tree samples the 16 SA outputs on that edge. |
| `out_valid`, `sum[10:0]` | out | Tree result for one bit-plane, 4 cycles after `start`. |
| `acc_out[18:0]` | out | DSA accumulator (final 1 cycle after the 8th `out_valid`). |
| `act_start` | in | 1-cycle pulse after the 8th plane is accumulated. |
| `act_out[7:0]`, `act_valid` | out | Saturated result, valid 1 cycle after `act_start`; held until the next `act_start`. |

All ports are flat (scalars or 1-D vectors). 2-D array ports are avoided on purpose: they break the SDC / P&R flow.

## Simulation

On the lab (VCS): `bash run_sim.sh`. Run it from this folder.
The DSA instantiates one Tower standard cell (`BUF_X2_18_SVT_WTN`), so the script points VCS at the Tower standard-cell
Verilog models with `-v`. Those models are part of the Tower PDK and are **not** included here.

## Notes

- Uses Tower standard cells by name: `ANTENNA_18_SVT_WTN` (antenna diodes, physical-only) and `BUF_X2_18_SVT_WTN` (in `dsa.sv`).
- **Do not commit Tower PDK files (.lib, .lef, .gds, Verilog models) to GitHub.** They are under the Tower NDA.
- Open questions (see the controller review document):
  - Bit-plane order: the DSA (`acc << 1`) needs the MSB plane first; the controller currently sends LSB first.
  - `dsa` reset is still tied to the wrapper `reset`. It should become a separate `dsa_clear` / `rst_n` from the controller.
  - One slice gives 1 result. Parameter `ADC_INDEX` (0-3) picks which ADC channel of the 16 SAs the slice taps,
    so one PE = 4 slices = 4 results, and a 4-PE core gives 16 results.
