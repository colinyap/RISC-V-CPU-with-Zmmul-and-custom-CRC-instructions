# Multicycle RISC-V CPU with Zmmul and Custom CRC Instructions

A synthesizable, multicycle 32-bit RISC-V processor written in Verilog-2001. The core implements the RV32I base integer instruction set used by this project, the multiplication subset of the RISC-V M extension (`Zmmul`), and three custom instructions for accelerating a 16-bit CRC.

This processor was developed for the **ChampionCHIP Competition — Malaysian Edition**, hosted by **ChipInventor** and **ASEM**.

> **Current recommended version:** [`RISC_V Complete (Optimised and Reformatted)`](RISC_V%20Complete%20%28Optimised%20and%20Reformatted%29)

## What the project contains

The repository keeps the main stages of the processor's development rather than replacing the earlier work:

| Directory | Purpose |
|---|---|
| [`RV32I Basic`](RV32I%20Basic) | Original block-based RV32I processor and its testbench. |
| [`RISC_V Complete`](RISC_V%20Complete) | Reformatted RV32I core with Zmmul and custom CRC extensions integrated. |
| [`RISC_V Complete (Optimised)`](RISC_V%20Complete%20%28Optimised%29) | Optimized multi-module implementation. |
| [`RISC_V Complete (Optimised and Reformatted)`](RISC_V%20Complete%20%28Optimised%20and%20Reformatted%29) | Optimized processor with the CPU datapath, control, multiplier and CRC logic consolidated into one large module; instruction memory, data memory and address decoding remain separate. |

This layout makes it possible to inspect the design at each stage: the initial RV32I implementation, extension integration, PPA-oriented optimization, and the final consolidated presentation.

## Design lineage

The project began with the block-oriented implementation under **`RV32I Basic`**. That implementation separates functions such as the controller, ALU, load/store units, immediate extension, register storage and multiplexers into individual RTL blocks.

Before the Zmmul and CRC extensions were integrated, the original RV32I processor was **reformatted into the `rv32i_core` module**. The reformat did not introduce a different ISA or execution model. It consolidated the base processor's register file, controller, immediate generation, ALU, branch comparison and load/store alignment logic into a single, explicit core module with:

- a shared instruction/data memory interface;
- a four-state multicycle controller;
- parameterized reset values;
- status and debug outputs; and
- a clean R-type extension interface.

The extension interface made it possible to add the iterative multiplier and CRC execution unit without embedding extension-specific behavior throughout the base decoder. The optimized-and-reformatted version later inlines those extension blocks into one large processor module while continuing to keep IMEM, DMEM and address decoding external.

## Architecture

### Execution model

The processor uses four architectural control states:

```text
FETCH → DECODE → EXECUTE → WRITEBACK → FETCH
```

Ordinary RV32I instructions and CRC instructions take **four cycles**. Multiply instructions use the same outer state machine but hold the processor in `EXECUTE` until the iterative multiplier completes, resulting in **22 cycles per multiply instruction** in this implementation.

This is a deliberately compact multicycle design, not a pipelined processor. It avoids pipeline hazards, forwarding and speculation while sharing the datapath across instruction classes.

### High-level datapath

```text
                     ┌──────────────────────────┐
 Instruction/Data ──►│                          │──► Address
       Rdata          │       RV32 core          │──► WriteData
                     │                          │──► MemWrite / ByteStrobe
                     │  Register file           │
                     │  Immediate generator     │
                     │  ALU / branch logic      │
                     │  Load/store alignment    │
                     │  Four-state controller   │
                     │  Radix-4 multiplier      │
                     │  CRC-16 execution logic  │
                     └──────────────────────────┘
```

The final consolidated processor module is [`RV32_single_optimised.v`](RISC_V%20Complete%20%28Optimised%20and%20Reformatted%29/rtl%20modules/RV32_single_optimised.v). It contains no CPU leaf-module instances. The SoC-level `top` connects it to:

- [`RISCV_IMEM_OPTIMISED.v`](RISC_V%20Complete%20%28Optimised%20and%20Reformatted%29/rtl%20modules/RISCV_IMEM_OPTIMISED.v)
- [`RISCV_DMEM_OPTIMISED.v`](RISC_V%20Complete%20%28Optimised%20and%20Reformatted%29/rtl%20modules/RISCV_DMEM_OPTIMISED.v)
- [`address_decoder_optimised.v`](RISC_V%20Complete%20%28Optimised%20and%20Reformatted%29/rtl%20modules/address_decoder_optimised.v)

## Supported instructions

### RV32I base integer instructions

The core implements the following 37 base instructions:

| Class | Instructions |
|---|---|
| Upper immediate | `LUI`, `AUIPC` |
| Jumps | `JAL`, `JALR` |
| Branches | `BEQ`, `BNE`, `BLT`, `BGE`, `BLTU`, `BGEU` |
| Loads | `LB`, `LH`, `LW`, `LBU`, `LHU` |
| Stores | `SB`, `SH`, `SW` |
| Immediate ALU | `ADDI`, `SLTI`, `SLTIU`, `XORI`, `ORI`, `ANDI`, `SLLI`, `SRLI`, `SRAI` |
| Register ALU | `ADD`, `SUB`, `SLL`, `SLT`, `SLTU`, `XOR`, `SRL`, `SRA`, `OR`, `AND` |

### Zmmul

The multiplication subset of the M extension is supported:

| Instruction | Result |
|---|---|
| `MUL` | Low 32 bits of the product |
| `MULH` | High 32 bits of signed × signed multiplication |
| `MULHSU` | High 32 bits of signed × unsigned multiplication |
| `MULHU` | High 32 bits of unsigned × unsigned multiplication |

Division and remainder instructions are **not** implemented. Encodings occupying those M-extension slots are rejected by the extension controller after the multiplier transaction behavior used by this design.

### Custom CRC instructions

Three custom R-type operations accelerate an MSB-first, 16-bit CRC:

| Instruction | Data consumed from `rs1` | CRC seed |
|---|---:|---|
| `CRCB` | `rs1[7:0]` | `rs2[15:0]` |
| `CRCH` | `rs1[15:0]` | `rs2[15:0]` |
| `CRCW` | `rs1[31:0]` | `rs2[15:0]` |

The default polynomial is `16'h1021`. For each input bit, processed most-significant bit first, the unit performs:

```text
feedback = crc[15] XOR data_bit
crc      = (crc << 1) XOR (feedback ? 16'h1021 : 16'h0000)
```

The 16-bit result is zero-extended when written to the destination register. There is no implicit initial seed, reflection or final XOR; software supplies the current CRC value through `rs2`.

The project encoding uses the standard R-type `OP` opcode (`7'b0110011`) with `funct7 = 7'b1000000`:

| Instruction | `funct3` |
|---|---:|
| `CRCB` | `000` |
| `CRCH` | `001` |
| `CRCW` | `010` |

## Multiplier optimization

The original extension used a signed, sequential radix-4 modified Booth multiplier. The optimized design preserves its interface and exact instruction latency while replacing variable-position 64-bit partial-product accumulation with a fixed-window recurrence:

- 34-bit upper accumulator;
- 32-bit lower product register;
- one previous-bit register for Booth recoding;
- 16 radix-4 update iterations; and
- carry-save high-half correction for `MULHSU` and `MULHU`.

The optimization reduces arithmetic width and unnecessary shifting without turning multiplication into a single-cycle combinational path. Start, ready, busy, enable-stall, restart and result-holding behavior are preserved.

## Optimization results

The original complete implementation and optimized implementation were synthesized through the same screening flow using Yosys 0.68, ABC and the Sky130 HD typical library at 1.8 V and 25 °C. A processor wrapper exposing the real memory and status interfaces was used because the repository's closed simulation `top` has no externally observable datapath outputs and could otherwise be optimized away.

| Metric | Original complete core | Optimized core | Change |
|---|---:|---:|---:|
| Mapped standard-cell area | 86,788.24 µm² | 81,144.07 µm² | **−6.50%** |
| ABC combinational-delay estimate | 10.131 ns | 7.814 ns | **−22.87%** |
| Flip-flops | 1,388 | 1,357 | −31 |
| Ordinary/CRC instruction cycles | 4 | 4 | Unchanged |
| Multiply instruction cycles | 22 | 22 | Unchanged |

The selected design improved both area and estimated delay across the tested 5, 10 and 20 fF output loads. More area-aggressive alternatives were rejected when they caused disproportionate timing loss; for example, disabling timing-driven sizing reduced area by a further 3.76% but increased estimated delay by 50.9%.

These are **pre-layout screening results**, not post-route timing or power measurements. The delay figure is not an achieved processor clock period, and the area figure is summed standard-cell area rather than die area. Power, routed setup/hold timing, clock-tree effects, congestion, DRC and LVS require a complete physical implementation with declared constraints.

## Memory system

The supplied simulation SoC uses separate instruction and data memories behind a simple read-data selector:

- Reset PC: `0x0040_0000`
- Reset stack pointer (`x2`): `0x0000_00FC`
- Reset global pointer (`x3`): `0x0000_00E0`
- Data memory: 32 × 32-bit words with byte write strobes
- Instruction/data interface: zero-wait-state shared address/read-data path

The included DMEM indexes words using `Address[6:2]`; addresses therefore alias across the small simulation memory. The address decoder selects instruction memory during fetch and for data reads in the `0x0040_0000`–`0x0040_0FFF` region. This is a compact competition/simulation memory system rather than a cache, bus protocol or general-purpose SoC interconnect.

The instruction memory contains two compile-time forms:

- normal simulation: the complete self-checking firmware image;
- ``SYNTHESIS`` defined: a minimal three-instruction arithmetic image.

## Unsupported and intentionally limited behavior

This is an unprivileged competition core, not a machine-mode or operating-system processor.

- No CSRs, privilege modes, traps, interrupts, MMU or caches.
- `ECALL`, `EBREAK` and other `SYSTEM` encodings do not trap. The core exposes a `syscall` status indication during execution and otherwise treats them as non-trapping operations.
- `FENCE`/`FENCE.I` have no ordering mechanism and behave as no-ops in this memory system.
- No divide/remainder unit.
- No memory-ready or bus-error handshake.
- No misalignment trap. The RTL's existing byte-lane behavior is retained: odd halfword accesses do not perform a split transaction, while word accesses use the selected memory word.
- Illegal encodings report `illegal_instruction`, but the signal is diagnostic rather than a trap request. Some malformed encodings retain the original datapath side effects; consult the RTL before treating it as a formal compliance target.

These are documented characteristics, not unfinished privileged features silently claimed as supported.

## Repository guide

### 1. RV32I Basic

```text
RV32I Basic/
├── RV32I_Basic.v
├── RV32I_Basic_TB.v
└── rtl modules/
    ├── controller.v
    ├── RISCV_ALU.v
    ├── LSU_READ.v
    ├── LSU_STORE.v
    ├── RISCV_UNIFIED_MEM.v
    └── ...
```

This is the original block-based implementation. Its processor logic was later reformatted into `rv32i_core` to provide a clearer integration boundary for the extensions.

### 2. RISC_V Complete

```text
RISC_V Complete/
├── RV32.v
├── tb_RV32.v
└── rtl files/
    ├── rv32i_core.v
    ├── RadixBooth.v
    ├── MultiplierWrapper.v
    ├── xicrc_exec.v
    ├── ext_secondarycontrol.v
    ├── RISCV_iMEM.v
    ├── RISCV_DMEM.v
    └── address_decoder.v
```

This is the readable, modular implementation of the complete processor.

### 3. Optimised

[`RV32_optimized.v`](RISC_V%20Complete%20%28Optimised%29/RV32_optimized.v) contains the selected optimized multi-module design and [`tb_RV32.v`](RISC_V%20Complete%20%28Optimised%29/tb_RV32.v) contains its firmware testbench.

### 4. Optimised and Reformatted

```text
RISC_V Complete (Optimised and Reformatted)/
├── RV32_optimised_reformatted.v
├── TB_RV32_optimised_reformatted.v
└── rtl modules/
    ├── RV32_single_optimised.v
    ├── RISCV_IMEM_OPTIMISED.v
    ├── RISCV_DMEM_OPTIMISED.v
    ├── address_decoder_optimised.v
    ├── MUX2_32.v
    └── top.v
```

This is the recommended presentation when the processor itself must be one consolidated RTL module while memories and address selection remain separate.

## Running the supplied simulations

### Optimized multi-module version

From the repository root:

```bash
iverilog -g2001 -s testbench \
  -o rv32_optimized_sim \
  "RISC_V Complete (Optimised)/RV32_optimized.v" \
  "RISC_V Complete (Optimised)/tb_RV32.v"

vvp rv32_optimized_sim
```

### Optimized and reformatted version

```bash
iverilog -g2001 -s testbench \
  -o rv32_reformatted_sim \
  "RISC_V Complete (Optimised and Reformatted)/RV32_optimised_reformatted.v" \
  "RISC_V Complete (Optimised and Reformatted)/TB_RV32_optimised_reformatted.v"

vvp rv32_reformatted_sim
```

The testbench generates `testbench.vcd`, runs the firmware and reports whether execution reached the pass or fail loop. Open the waveform with GTKWave if desired:

```bash
gtkwave testbench.vcd
```

Do **not** define `SYNTHESIS` when running the full firmware testbench; that macro selects the minimal synthesis-oriented IMEM contents.

## Verification performed during development

The selected optimized RTL was checked using several complementary methods:

- formal binary equivalence of the changed multiplier and correction logic;
- directed and randomized multiplier protocol testing;
- independent arithmetic and CRC reference checks;
- cycle-by-cycle original-versus-optimized comparisons;
- all 37 base, four Zmmul and three CRC operation classes;
- malformed-encoding and reset-interruption tests;
- full simulation-firmware comparison;
- generic post-synthesis regression; and
- functional regression of the exact Sky130-mapped Verilog using models generated from the same Liberty library.

The largest external-core regression exercised 13,920 instructions and 141,936 comparisons. The mapped-netlist regression exercised 5,959 instructions and 67,772 output comparisons for both the baseline and selected design.

Verification establishes strong compatibility evidence, not mathematical proof of the entire ISA, exhaustive four-state equivalence or physical signoff. Both the reference and optimized designs can share an original architectural bug; differential testing does not replace an independent compliance suite.

## Notes for synthesis and integration

- The RTL targets Verilog-2001.
- Compile only one implementation variant at a time; several files intentionally define modules with the same names.
- A separate top-level should connect `RV32_single_optimised` to IMEM, DMEM and the address decoder when using the consolidated source files.
- Replace the case-based instruction image with the memory implementation required by the target platform.
- Add realistic clock and I/O constraints before interpreting synthesis timing.
- Preserve the processor's reset distinction: the base core uses an asynchronous active-low reset, while the sequential multiplier samples its active-low reset on a clock edge.
- The ChipInventor-generated top-level file may be regenerated by the platform; preserve RTL changes in the corresponding source blocks before exporting again.

## Project scope

The aim of this repository is specific: build a compact multicycle RV32 processor, extend it with useful multiplication and CRC operations, then improve its implementation without changing its architectural behavior. It is intended as an educational and competition design whose internal decisions can be read directly in Verilog.

It is not presented as a Linux-capable core, a fully privileged RISC-V implementation or a tapeout-ready hard macro.

## Acknowledgements

Developed for the **ChampionCHIP Competition — Malaysian Edition**, hosted by **ChipInventor** and **ASEM**.

RISC-V is an open standard maintained by RISC-V International. “RISC-V” and related marks belong to their respective owners.

## License

No license file is currently included in this repository. Until a license is added, normal copyright restrictions apply even though the source is publicly visible. Add an explicit open-source license before inviting reuse, modification or redistribution.
