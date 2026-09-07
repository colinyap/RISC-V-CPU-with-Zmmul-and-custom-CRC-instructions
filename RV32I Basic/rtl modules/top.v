

//  ---------- INLCUDED BLOCK: extend_riscv  ---------- 
module extend_riscv(
  input [31:0] input_data,
  output reg [31:0] output_data,
  input [2:0] immsrc
);

  reg [11:0] imm;
  reg [19:0] imm20bit;
always @(*) begin
  imm = 0;
  imm20bit = 0;
  output_data = 0;
  case (immsrc)
    0: begin
      imm = input_data[31:20];
      output_data = {{20{imm[11]}}, imm[11:0]};
    end
    1: begin
      imm = {input_data[31:25], input_data[11:7]};
      output_data = {{20{imm[11]}}, imm[11:0]};
    end
    2: begin
      imm = {input_data[31], input_data[7], input_data[30:25], input_data[11:8]};
      output_data = {{19{imm[11]}}, imm[11:0], 1'b0};
    end
    4: begin
      imm20bit = input_data[31:12];
      output_data = {imm20bit[19:0], {12{1'b0}}};
    end
    3: begin
      imm20bit = {input_data[31], input_data[19:12], input_data[20], input_data[30:21]};
      output_data = {{11{imm20bit[19]}}, imm20bit[19:0], 1'b0};
    end
    default: begin
      imm = 0;
      imm20bit = 0;
      output_data = 0;
    end
  endcase

end

endmodule



//  ---------- INLCUDED BLOCK: riscv_4kbReg  ---------- 
module riscv_4kbReg(input  wire       enable_reg_write, reset,  
               input  wire [4:0]  reg_addr1, reg_addr2, addr_write, 
               input  wire [31:0] write_data, 
               output wire [31:0] rd1_data, rd2_data,
               input  wire        clk);
					
  (* ram_style = "distributed" *) reg [31:0] rf[31:0];

	always @(posedge clk, negedge reset) begin
    if(reset == 1'b0) begin
      rf[0] <= 32'h00000000;
      rf[2] <= 32'h000000FC; //Stack Pointer, pointed to the top of the memeory by deafault
      rf[3] <= 32'h000000E0; //Global Pointer, pointed in the middle of the memory by default
    end
		else begin
      if (enable_reg_write && addr_write != 5'b0) 
			  rf[addr_write] <= write_data;
      else
        rf[addr_write] <= rf[addr_write];
    end
  end

	assign rd1_data = (reg_addr1 != 5'b0) ? rf[reg_addr1] : 32'b0; 
	assign rd2_data = (reg_addr2 != 5'b0) ? rf[reg_addr2] : 32'b0;
endmodule



//  ---------- INLCUDED BLOCK: controller  ---------- 
// ============================================================================
//  controller.v
//  Four-state Mealy control unit for a multicycle RV32I core.
//
//  States          : FETCH -> DECODE -> EXECUTE -> WRITEBACK -> FETCH
//  Coverage        : all 37 computational/control RV32I instructions.
//                    FENCE is a NOP; ECALL/EBREAK pulse SysCall (no traps).
//  Language        : Verilog-2001 (iverilog -g2001 / Verilator clean)
//
//  Mealy: outputs depend on state AND (op, funct3, Flags). In particular
//  PCWrite in EXECUTE is a function of the ALU flags in the same cycle.
//
//  Memory model assumed: asynchronous read (RD combinational from Adr), with
//  the external Instr and Data registers acting as the capture stage - i.e.
//  exactly what the reference datapath draws. See spec doc section 7 for the
//  one change required to move onto a real synchronous-read SRAM macro.
//
//  Control-signal encodings (unchanged from the reference datapath):
//    AdrSrc     0=PC              1=Result
//    ALUSrcA   00=PC             01=OldPC          10=A (rs1)
//    ALUSrcB   00=WriteData(rs2) 01=ImmExt         10=constant 4
//    ResultSrc 00=ALUOut         01=LSU load data  10=ALUResult
// ============================================================================

module controller #(
    // 0 = uniform 4 cycles for every instruction (CPI = 4.00)
    // 1 = skip WRITEBACK for stores and branches, which have nothing to
    //     commit there (CPI ~ 3.4 on typical code). Costs 2 gates.
    parameter EARLY_EXIT = 0
) (
    input  wire        clk,
    input  wire        rst_n,        // asynchronous, active-low

    // ---- instruction fields (from the Instr register) ----------------------
    input  wire [6:0]  op,           // Instr[6:0]
    input  wire [2:0]  funct3,       // Instr[14:12]
    input  wire        funct7b5,     // Instr[30]

    // ---- ALU status flags, valid in the same cycle as the ALU operation ----
    // {N, Z, C, V} for the operation A - B:
    //   N = result[31]
    //   Z = (result == 0)
    //   C = carry-out of (A + ~B + 1)  -> C=1 means A >= B unsigned
    //   V = (A[31] ^ B[31]) & (A[31] ^ result[31])
    input  wire [3:0]  Flags,

    // ---- stall hooks. Tie both to 1'b1 if unused. --------------------------
    // ex_done   : drive low ONLY for multi-cycle compute units (MUL/DIV/CRC).
    //             Never drive it low for a load or a store - MemWrite is
    //             combinational in EXECUTE and would issue the write twice.
    // mem_ready : drive low while a slow peripheral has not returned data.
    input  wire        ex_done,
    input  wire        mem_ready,

    // ---- datapath control --------------------------------------------------
    output reg         PCWrite,
    output reg         AdrSrc,
    output reg         MemWrite,     // 1 cycle only, in EXECUTE, stores only
    output reg         MemRead,      // SRAM chip-enable; tie off if unused
    output reg         IRWrite,      // enables Instr AND OldPC
    output reg         RegWrite,
    output reg  [1:0]  ResultSrc,
    output reg  [1:0]  ALUSrcA,
    output reg  [1:0]  ALUSrcB,
    output reg  [3:0]  ALUControl,   // widened from 3 bits: 11 ops needed
    output reg  [2:0]  ImmSrc,       // widened from 2 bits: U-type needed

    // ---- LSU control -------------------------------------------------------
    output wire [2:0]  LSUCtrl,      // = funct3 during a load/store, else 0

    // ---- status / debug ----------------------------------------------------
    output wire [1:0]  StateOut,     // for trace instrumentation
    output reg         SysCall,      // ECALL/EBREAK seen in EXECUTE
    output wire        Illegal       // unrecognised opcode in EXECUTE
);

    // ------------------------------------------------------------------------
    // Constants
    // ------------------------------------------------------------------------
    localparam [1:0] S_FETCH  = 2'd0,
                     S_DECODE = 2'd1,
                     S_EXEC   = 2'd2,
                     S_WB     = 2'd3;

    localparam [6:0] OP_LOAD   = 7'b0000011,   // LB LH LW LBU LHU
                     OP_MISCM  = 7'b0001111,   // FENCE
                     OP_IMM    = 7'b0010011,   // ADDI..SRAI
                     OP_AUIPC  = 7'b0010111,
                     OP_STORE  = 7'b0100011,   // SB SH SW
                     OP_REG    = 7'b0110011,   // ADD..AND
                     OP_LUI    = 7'b0110111,
                     OP_BRANCH = 7'b1100011,   // BEQ..BGEU
                     OP_JALR   = 7'b1100111,
                     OP_JAL    = 7'b1101111,
                     OP_SYSTEM = 7'b1110011;   // ECALL EBREAK

    localparam [1:0] ALUOP_ADD   = 2'b00,   // address / target / link / PC+4
                     ALUOP_SUB   = 2'b01,   // branch comparison
                     ALUOP_FUNCT = 2'b10,   // decode from funct3/funct7
                     ALUOP_PASSB = 2'b11;   // LUI

    localparam [3:0] ALU_ADD   = 4'd0,  ALU_SUB  = 4'd1,
                     ALU_AND   = 4'd2,  ALU_OR   = 4'd3,
                     ALU_XOR   = 4'd4,  ALU_SLT  = 4'd5,
                     ALU_SLTU  = 4'd6,  ALU_SLL  = 4'd7,
                     ALU_SRL   = 4'd8,  ALU_SRA  = 4'd9,
                     ALU_PASSB = 4'd10;

    localparam [2:0] IMM_I = 3'd0, IMM_S = 3'd1, IMM_B = 3'd2,
                     IMM_J = 3'd3, IMM_U = 3'd4;

    // ------------------------------------------------------------------------
    // Block 1 of 3 : state register
    // ------------------------------------------------------------------------
    reg [1:0] state, next_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_FETCH;
        else        state <= next_state;
    end

    assign StateOut = state;

    // ------------------------------------------------------------------------
    // Block 2 of 3 : next-state logic
    // ------------------------------------------------------------------------
    always @(*) begin
        next_state = state;                       // default: hold (no latch)
        case (state)
            S_FETCH : next_state = S_DECODE;
            S_DECODE: next_state = S_EXEC;
            S_EXEC  : begin
                if (!ex_done)
                    next_state = S_EXEC;          // multi-cycle unit busy
                else if (EARLY_EXIT != 0 &&
                         (op == OP_STORE || op == OP_BRANCH))
                    next_state = S_FETCH;         // nothing to commit
                else
                    next_state = S_WB;
            end
            S_WB    : next_state = mem_ready ? S_FETCH : S_WB;
            default : next_state = S_FETCH;
        endcase
    end

    // ------------------------------------------------------------------------
    // Branch condition (Mealy on Flags + funct3)
    // ------------------------------------------------------------------------
    wire flag_n = Flags[3];
    wire flag_z = Flags[2];
    wire flag_c = Flags[1];
    wire flag_v = Flags[0];

    reg BranchTaken;
    always @(*) begin
        case (funct3)
            3'b000 : BranchTaken =   flag_z;              // BEQ
            3'b001 : BranchTaken =  ~flag_z;              // BNE
            3'b100 : BranchTaken =  (flag_n ^ flag_v);    // BLT
            3'b101 : BranchTaken = ~(flag_n ^ flag_v);    // BGE
            3'b110 : BranchTaken =  ~flag_c;              // BLTU
            3'b111 : BranchTaken =   flag_c;              // BGEU
            default: BranchTaken = 1'b0;                  // 010/011 reserved
        endcase
    end

    // ------------------------------------------------------------------------
    // Block 3 of 3 : output logic (Mealy)
    // Every output is given a side-effect-free default at the top of the
    // block, so no path through the case can leave one unassigned.
    // ------------------------------------------------------------------------
    reg [1:0] ALUOp;

    always @(*) begin
        PCWrite   = 1'b0;
        AdrSrc    = 1'b0;
        MemWrite  = 1'b0;
        MemRead   = 1'b0;
        IRWrite   = 1'b0;
        RegWrite  = 1'b0;
        ResultSrc = 2'b00;
        ALUSrcA   = 2'b00;
        ALUSrcB   = 2'b00;
        ALUOp     = ALUOP_ADD;
        SysCall   = 1'b0;

        case (state)

        // ---- FETCH : Instr <= mem[PC], OldPC <= PC, PC <= PC + 4 ----------
        S_FETCH: begin
            AdrSrc    = 1'b0;          // Adr = PC
            MemRead   = 1'b1;
            IRWrite   = 1'b1;          // captures Instr and OldPC
            ALUSrcA   = 2'b00;         // PC
            ALUSrcB   = 2'b10;         // 4
            ALUOp     = ALUOP_ADD;
            ResultSrc = 2'b10;         // ALUResult
            PCWrite   = 1'b1;
        end

        // ---- DECODE : A <= rs1, WriteData <= rs2 (unconditional capture),
        //      and the otherwise-idle ALU precomputes the branch/JAL target
        //      ALUOut <= OldPC + ImmExt. No architectural side effects. -----
        S_DECODE: begin
            ALUSrcA = 2'b01;           // OldPC
            ALUSrcB = 2'b01;           // ImmExt
            ALUOp   = ALUOP_ADD;
        end

        // ---- EXECUTE : ALU work, memory access issued, PC redirected ------
        S_EXEC: begin
            case (op)

            OP_REG: begin                          // ADD SUB SLL SLT SLTU
                ALUSrcA = 2'b10;                   // XOR SRL SRA OR AND
                ALUSrcB = 2'b00;                   // ALUOut <= rs1 op rs2
                ALUOp   = ALUOP_FUNCT;
            end

            OP_IMM: begin                          // ADDI..SRAI
                ALUSrcA = 2'b10;
                ALUSrcB = 2'b01;                   // ALUOut <= rs1 op imm
                ALUOp   = ALUOP_FUNCT;
            end

            OP_LOAD: begin                         // LB LH LW LBU LHU
                ALUSrcA   = 2'b10;
                ALUSrcB   = 2'b01;
                ALUOp     = ALUOP_ADD;             // ALUResult = rs1 + imm
                ResultSrc = 2'b10;
                AdrSrc    = 1'b1;                  // Adr = ALUResult
                MemRead   = 1'b1;                  // Data <= mem[Adr]
            end

            OP_STORE: begin                        // SB SH SW
                ALUSrcA   = 2'b10;
                ALUSrcB   = 2'b01;
                ALUOp     = ALUOP_ADD;
                ResultSrc = 2'b10;
                AdrSrc    = 1'b1;
                MemWrite  = 1'b1;                  // commits at this edge
            end

            OP_BRANCH: begin                       // BEQ BNE BLT BGE BLTU BGEU
                ALUSrcA   = 2'b10;
                ALUSrcB   = 2'b00;
                ALUOp     = ALUOP_SUB;             // sets N,Z,C,V
                ResultSrc = 2'b00;                 // ALUOut = OldPC + imm_b
                PCWrite   = BranchTaken;           // <-- the Mealy edge
            end

            OP_JAL: begin
                ALUSrcA   = 2'b01;                 // OldPC
                ALUSrcB   = 2'b10;                 // 4
                ALUOp     = ALUOP_ADD;             // ALUOut <= link (PC+4)
                ResultSrc = 2'b00;                 // PC <= ALUOut = target
                PCWrite   = 1'b1;
            end

            OP_JALR: begin
                ALUSrcA   = 2'b10;                 // rs1
                ALUSrcB   = 2'b01;                 // imm_i
                ALUOp     = ALUOP_ADD;
                ResultSrc = 2'b10;                 // PC <= ALUResult
                PCWrite   = 1'b1;                  // datapath forces PC[0]=0
            end

            OP_LUI: begin
                ALUSrcA = 2'b00;                   // don't care
                ALUSrcB = 2'b01;                   // ImmExt (U)
                ALUOp   = ALUOP_PASSB;             // ALUOut <= imm
            end

            OP_AUIPC: begin
                ALUSrcA = 2'b01;                   // OldPC
                ALUSrcB = 2'b01;                   // ImmExt (U)
                ALUOp   = ALUOP_ADD;               // ALUOut <= PC + imm
            end

            OP_SYSTEM: SysCall = 1'b1;             // ECALL/EBREAK hook
            OP_MISCM : ;                           // FENCE = NOP
            default  : ;                           // illegal = NOP
            endcase
        end

        // ---- WRITEBACK : the only state that writes the register file -----
        S_WB: begin
            case (op)

            OP_REG, OP_IMM, OP_LUI, OP_AUIPC, OP_JAL: begin
                ResultSrc = 2'b00;                 // ALUOut
                RegWrite  = 1'b1;
            end

            OP_LOAD: begin
                ResultSrc = 2'b01;                 // LSU-aligned load data
                RegWrite  = 1'b1;
            end

            OP_JALR: begin                         // link computed here: the
                ALUSrcA   = 2'b01;                 // ALUOut slot was used by
                ALUSrcB   = 2'b10;                 // the jump target
                ALUOp     = ALUOP_ADD;             // ALUResult = OldPC + 4
                ResultSrc = 2'b10;
                RegWrite  = 1'b1;
            end

            default: ;                             // STORE/BRANCH/FENCE/SYSTEM
            endcase
        end

        default: ;
        endcase
    end

    // ------------------------------------------------------------------------
    // ALU decoder
    // ------------------------------------------------------------------------
    always @(*) begin
        case (ALUOp)
            ALUOP_ADD  : ALUControl = ALU_ADD;
            ALUOP_SUB  : ALUControl = ALU_SUB;
            ALUOP_PASSB: ALUControl = ALU_PASSB;
            default    : begin                     // ALUOP_FUNCT
                case (funct3)
                    // op[5] gates SUB so that ADDI with imm[10]=1 stays an ADD
                    3'b000 : ALUControl = (funct7b5 & op[5]) ? ALU_SUB  : ALU_ADD;
                    3'b001 : ALUControl = ALU_SLL;
                    3'b010 : ALUControl = ALU_SLT;
                    3'b011 : ALUControl = ALU_SLTU;
                    3'b100 : ALUControl = ALU_XOR;
                    // SRAI/SRA are distinguished by imm[10]/funct7[5] alone
                    3'b101 : ALUControl = funct7b5 ? ALU_SRA : ALU_SRL;
                    3'b110 : ALUControl = ALU_OR;
                    default: ALUControl = ALU_AND;
                endcase
            end
        endcase
    end

    // ------------------------------------------------------------------------
    // Immediate decoder - a pure function of the opcode
    // ------------------------------------------------------------------------
    always @(*) begin
        case (op)
            OP_LOAD, OP_IMM, OP_JALR: ImmSrc = IMM_I;
            OP_STORE                : ImmSrc = IMM_S;
            OP_BRANCH               : ImmSrc = IMM_B;
            OP_JAL                  : ImmSrc = IMM_J;
            OP_LUI,  OP_AUIPC       : ImmSrc = IMM_U;
            default                 : ImmSrc = IMM_I;
        endcase
    end

// ------------------------------------------------------------------------
// LSU control and illegal instruction detection
// ------------------------------------------------------------------------
wire is_mem = (op == OP_LOAD) || (op == OP_STORE);
assign LSUCtrl = is_mem ? funct3 : 3'b000;

reg legal_op;
reg legal_encoding;

always @(*) begin
    case (op)
        OP_LOAD, OP_MISCM, OP_IMM, OP_AUIPC, OP_STORE, OP_REG,
        OP_LUI, OP_BRANCH, OP_JALR, OP_JAL, OP_SYSTEM:
            legal_op = 1'b1;

        default:
            legal_op = 1'b0;
    endcase
end


// Check funct3/funct7 legality
always @(*) begin
    legal_encoding = 1'b1;

    case (op)

        // Loads:
        // LB=000 LH=001 LW=010 LBU=100 LHU=101
        OP_LOAD: begin
            case (funct3)
                3'b000,
                3'b001,
                3'b010,
                3'b100,
                3'b101: legal_encoding = 1'b1;

                default: legal_encoding = 1'b0;
            endcase
        end


        // Stores:
        // SB=000 SH=001 SW=010
        OP_STORE: begin
            case (funct3)
                3'b000,
                3'b001,
                3'b010: legal_encoding = 1'b1;

                default: legal_encoding = 1'b0;
            endcase
        end


        // Register operations
        OP_REG: begin
            case (funct3)

                // ADD/SUB
                3'b000:
                    legal_encoding =
                    (funct7b5 == 1'b0) ||
                    (funct7b5 == 1'b1);

                // SLL
                3'b001:
                    legal_encoding = (funct7b5 == 1'b0);

                // SLT
                3'b010:
                    legal_encoding = (funct7b5 == 1'b0);

                // SLTU
                3'b011:
                    legal_encoding = (funct7b5 == 1'b0);

                // XOR
                3'b100:
                    legal_encoding = (funct7b5 == 1'b0);

                // SRL/SRA
                3'b101:
                    legal_encoding = 1'b1;

                // OR
                3'b110:
                    legal_encoding = (funct7b5 == 1'b0);

                // AND
                3'b111:
                    legal_encoding = (funct7b5 == 1'b0);

                default:
                    legal_encoding = 1'b0;
            endcase
        end


        // Immediate ALU instructions
        OP_IMM: begin
            case (funct3)

                // ADDI
                3'b000:
                    legal_encoding = 1'b1;

                // SLLI
                3'b001:
                    legal_encoding = (funct7b5 == 1'b0);

                // SLTI
                3'b010:
                    legal_encoding = 1'b1;

                // SLTIU
                3'b011:
                    legal_encoding = 1'b1;

                // XORI
                3'b100:
                    legal_encoding = 1'b1;

                // SRLI/SRAI
                3'b101:
                    legal_encoding = 1'b1;

                // ORI
                3'b110:
                    legal_encoding = 1'b1;

                // ANDI
                3'b111:
                    legal_encoding = 1'b1;

                default:
                    legal_encoding = 1'b0;
            endcase
        end


        // JALR must have funct3 = 000
        OP_JALR:
            legal_encoding = (funct3 == 3'b000);


        // Branch instructions
        OP_BRANCH: begin
            case (funct3)
                3'b000, // BEQ
                3'b001, // BNE
                3'b100, // BLT
                3'b101, // BGE
                3'b110, // BLTU
                3'b111: // BGEU
                    legal_encoding = 1'b1;

                default:
                    legal_encoding = 1'b0;
            endcase
        end


        default:
            legal_encoding = 1'b1;

    endcase
end


assign Illegal = (state == S_EXEC) & ~(legal_op && legal_encoding);
endmodule



//  ---------- INLCUDED BLOCK: funct7_5  ---------- 
module funct7_5 ( input [6:0] funct7,
                 output wire funct7_5);
  assign funct7_5 = funct7[5];
endmodule //Just extracts bit 5 of funct7



//  ---------- INLCUDED BLOCK: LSU_READ  ---------- 
//  ---------- INCLUDED BLOCK: LSU_READ ----------
module LSU_READ (
    input [31:0] data,
    input [2:0] LSUctrl,
    output reg [31:0] data_out,

    // New input added below existing ports
  	input [31:0] address
);

    localparam b  = 3'b000;   // LB
    localparam h  = 3'b001;   // LH
    localparam w  = 3'b010;   // LW
    localparam bu = 3'b100;   // LBU
    localparam hu = 3'b101;   // LHU
  wire [1:0] address_offset;
  assign address_offset = address [1:0];
    reg [15:0] half;
    reg [7:0] b_yte;

    always @(*) begin
        half     = 16'b0;
        b_yte    = 8'b0;
        data_out = 32'b0;

        case (LSUctrl)

            // ---------------- LW ----------------
            w: begin
                data_out = data;
            end


            // ---------------- LH ----------------
            h: begin
                case (address_offset)
                    2'b00: half = data[15:0];
                    2'b10: half = data[31:16];

                    // Misaligned halfword
                    default: half = 16'b0;
                endcase

                data_out = {{16{half[15]}}, half};
            end


            // ---------------- LB ----------------
            b: begin
                case (address_offset)
                    2'b00: b_yte = data[7:0];
                    2'b01: b_yte = data[15:8];
                    2'b10: b_yte = data[23:16];
                    2'b11: b_yte = data[31:24];
                endcase

                data_out = {{24{b_yte[7]}}, b_yte};
            end


            // ---------------- LBU ----------------
            bu: begin
                case (address_offset)
                    2'b00: b_yte = data[7:0];
                    2'b01: b_yte = data[15:8];
                    2'b10: b_yte = data[23:16];
                    2'b11: b_yte = data[31:24];
                endcase

                data_out = {24'b0, b_yte};
            end


            // ---------------- LHU ----------------
            hu: begin
                case (address_offset)
                    2'b00: half = data[15:0];
                    2'b10: half = data[31:16];

                    // Misaligned halfword
                    default: half = 16'b0;
                endcase

                data_out = {16'b0, half};
            end


            default: begin
                half     = 16'b0;
                b_yte    = 8'b0;
                data_out = 32'b0;
            end

        endcase
    end

endmodule



//  ---------- INLCUDED BLOCK: RISCV_UNIFIED_MEM  ---------- 
//  ---------- INCLUDED BLOCK: RISCV_UNIFIED_MEM ----------
//  4 KB unified instruction/data SRAM, pre-loaded with a firmware image that
//  executes all 37 RV32I instructions once, plus FENCE / ECALL / EBREAK.
//
//  Memory map:  0x000 - 0x0DF   program (56 words)
//               0x400 - 0x40F   data scratch (written by SW/SB/SH)
//
//  Program ends in a JAL x0,0 self-loop at 0x0DC; the testbench uses that
//  as the "done" marker.
//
//  NOTE: the initial block is simulation-only firmware. Strip it (or replace
//  it with a $readmemh / real ROM macro) before synthesis.

module RISCV_UNIFIED_MEM (
    input [31:0] i_Data,
    input [31:0] i_Address,
    input i_Write_Enable,
    input i_Clk,
    output [31:0] o_Data,

    // New input added below existing ports
    input [3:0] i_Byte_Enable
);

    /* 4KB SRAM
       1024 words x 32 bits = 4096 bytes
    */
  reg [31:0] r_Contents [0:63];


    /* ------------------------------------------------------------------
       FIRMWARE  -  every RV32I instruction, executed once, in order.
       Register plan:  x5 = 5      x6 = -3     x28 = 0x400 (data base)
                       x31 = fail flag (must stay 0)
       ------------------------------------------------------------------ */
    integer w;
    initial begin
        for (w = 0; w < 1024; w = w + 1)
            r_Contents[w] = 32'h00000000;

        r_Contents[10'd0  ] = 32'h00500293;  // 0000h  ADDI  x5,x0,5
        r_Contents[10'd1  ] = 32'hFFD00313;  // 0004h  ADDI  x6,x0,-3
        r_Contents[10'd2  ] = 32'h40000E13;  // 0008h  ADDI  x28,x0,0x400
        r_Contents[10'd3  ] = 32'h00000F93;  // 000Ch  ADDI  x31,x0,0        (clear fail flag)
        r_Contents[10'd4  ] = 32'h006283B3;  // 0010h  ADD   x7,x5,x6
        r_Contents[10'd5  ] = 32'h40628433;  // 0014h  SUB   x8,x5,x6
        r_Contents[10'd6  ] = 32'h0062F4B3;  // 0018h  AND   x9,x5,x6
        r_Contents[10'd7  ] = 32'h0062E533;  // 001Ch  OR    x10,x5,x6
        r_Contents[10'd8  ] = 32'h0062C5B3;  // 0020h  XOR   x11,x5,x6
        r_Contents[10'd9  ] = 32'h00529633;  // 0024h  SLL   x12,x5,x5
        r_Contents[10'd10 ] = 32'h005356B3;  // 0028h  SRL   x13,x6,x5
        r_Contents[10'd11 ] = 32'h40535733;  // 002Ch  SRA   x14,x6,x5
        r_Contents[10'd12 ] = 32'h005327B3;  // 0030h  SLT   x15,x6,x5
        r_Contents[10'd13 ] = 32'h00533833;  // 0034h  SLTU  x16,x6,x5
        r_Contents[10'd14 ] = 32'h00032893;  // 0038h  SLTI  x17,x6,0
        r_Contents[10'd15 ] = 32'h00133913;  // 003Ch  SLTIU x18,x6,1
        r_Contents[10'd16 ] = 32'hFFF2C993;  // 0040h  XORI  x19,x5,-1
        r_Contents[10'd17 ] = 32'h0082EA13;  // 0044h  ORI   x20,x5,8
        r_Contents[10'd18 ] = 32'h00C37A93;  // 0048h  ANDI  x21,x6,12
        r_Contents[10'd19 ] = 32'h00329B13;  // 004Ch  SLLI  x22,x5,3
        r_Contents[10'd20 ] = 32'h00435B93;  // 0050h  SRLI  x23,x6,4
        r_Contents[10'd21 ] = 32'h40435C13;  // 0054h  SRAI  x24,x6,4
        r_Contents[10'd22 ] = 32'h12345CB7;  // 0058h  LUI   x25,0x12345
        r_Contents[10'd23 ] = 32'h678C8C93;  // 005Ch  ADDI  x25,x25,0x678   -> 0x12345678
        r_Contents[10'd24 ] = 32'h00001D17;  // 0060h  AUIPC x26,0x1
        r_Contents[10'd25 ] = 32'h019E2023;  // 0064h  SW    x25,0(x28)
        r_Contents[10'd26 ] = 32'h000E2D83;  // 0068h  LW    x27,0(x28)
        r_Contents[10'd27 ] = 32'h006E2223;  // 006Ch  SW    x6,4(x28)
        r_Contents[10'd28 ] = 32'h004E0203;  // 0070h  LB    x4,4(x28)
        r_Contents[10'd29 ] = 32'h004E4E83;  // 0074h  LBU   x29,4(x28)
        r_Contents[10'd30 ] = 32'h004E1F03;  // 0078h  LH    x30,4(x28)
        r_Contents[10'd31 ] = 32'h004E5183;  // 007Ch  LHU   x3,4(x28)
        r_Contents[10'd32 ] = 32'h005E0423;  // 0080h  SB    x5,8(x28)
        r_Contents[10'd33 ] = 32'h019E1623;  // 0084h  SH    x25,12(x28)
        r_Contents[10'd34 ] = 32'h00528463;  // 0088h  BEQ   x5,x5,+8        (taken)
        r_Contents[10'd35 ] = 32'h001F8F93;  // 008Ch    ADDI x31,x31,1      (skipped)
        r_Contents[10'd36 ] = 32'h00629463;  // 0090h  BNE   x5,x6,+8        (taken)
        r_Contents[10'd37 ] = 32'h001F8F93;  // 0094h    ADDI x31,x31,1      (skipped)
        r_Contents[10'd38 ] = 32'h00534463;  // 0098h  BLT   x6,x5,+8        (taken)
        r_Contents[10'd39 ] = 32'h001F8F93;  // 009Ch    ADDI x31,x31,1      (skipped)
        r_Contents[10'd40 ] = 32'h0062D463;  // 00A0h  BGE   x5,x6,+8        (taken)
        r_Contents[10'd41 ] = 32'h001F8F93;  // 00A4h    ADDI x31,x31,1      (skipped)
        r_Contents[10'd42 ] = 32'h0062E463;  // 00A8h  BLTU  x5,x6,+8        (taken)
        r_Contents[10'd43 ] = 32'h001F8F93;  // 00ACh    ADDI x31,x31,1      (skipped)
        r_Contents[10'd44 ] = 32'h00537463;  // 00B0h  BGEU  x6,x5,+8        (taken)
        r_Contents[10'd45 ] = 32'h001F8F93;  // 00B4h    ADDI x31,x31,1      (skipped)
        r_Contents[10'd46 ] = 32'h008000EF;  // 00B8h  JAL   x1,+8           (taken)
        r_Contents[10'd47 ] = 32'h001F8F93;  // 00BCh    ADDI x31,x31,1      (skipped)
        r_Contents[10'd48 ] = 32'h0D000167;  // 00C0h  JALR  x2,0xD0(x0)     -> word 52
        r_Contents[10'd49 ] = 32'h001F8F93;  // 00C4h    ADDI x31,x31,1      (skipped)
        r_Contents[10'd50 ] = 32'h001F8F93;  // 00C8h    ADDI x31,x31,1      (skipped)
        r_Contents[10'd51 ] = 32'h001F8F93;  // 00CCh    ADDI x31,x31,1      (skipped)
        r_Contents[10'd52 ] = 32'h0000000F;  // 00D0h  FENCE                 (NOP)
        r_Contents[10'd53 ] = 32'h00000073;  // 00D4h  ECALL                 (NOP)
        r_Contents[10'd54 ] = 32'h00100073;  // 00D8h  EBREAK                (NOP)
        r_Contents[10'd55 ] = 32'h0000006F;  // 00DCh  JAL   x0,0            (self-loop = done)
    end


    /* Synchronous Write */
    always @(posedge i_Clk) begin

        if (i_Write_Enable) begin

            if (i_Byte_Enable[0])
              r_Contents[i_Address[7:2]][7:0]
                    <= i_Data[7:0];

            if (i_Byte_Enable[1])
              r_Contents[i_Address[7:2]][15:8]
                    <= i_Data[15:8];

            if (i_Byte_Enable[2])
              r_Contents[i_Address[7:2]][23:16]
                    <= i_Data[23:16];

            if (i_Byte_Enable[3])
              r_Contents[i_Address[7:2]][31:24]
                    <= i_Data[31:24];

        end
    end


    /* Asynchronous Read */
  assign o_Data = r_Contents[i_Address[7:2]];

endmodule



//  ---------- INLCUDED BLOCK: instruction_splitter  ---------- 
module instruction_splitter (input [31:0] instruction,
                            output [6:0] opcode,
                             output [2:0] funct3,
                             output [6:0] funct7,
                             output [4:0] rs1,
                             output [4:0] rs2,
                             output [4:0] rd);
  assign opcode = instruction [6:0];
  assign funct3 = instruction [14:12];
  assign funct7 = instruction [31:25];
  assign rs1 = instruction [19:15];
  assign rs2 = instruction [24:20];
  assign rd = instruction [11:7];
endmodule



//  ---------- INLCUDED BLOCK: mux3to1_32bit  ---------- 
module mux3to1_32bit (
    input [31:0] in0,
    input [31:0] in1,
    input [31:0] in2,
  	input [1:0] sel,
    output reg [31:0] out
);

  always @ (*)
begin
    case(sel)
        2'b00: out = in0;
        2'b01: out = in1;
        2'b10: out = in2;
        default: out = 32'h00000000; // Default case, output all zeros
    endcase
end

endmodule



//  ---------- INLCUDED BLOCK: RISCV_ALU  ---------- 
/* ALU Operations */
`define ALU_ADD   4'd0
`define ALU_SUB   4'd1
`define ALU_AND   4'd2
`define ALU_OR    4'd3
`define ALU_XOR   4'd4
`define ALU_SLT   4'd5
`define ALU_SLTU  4'd6
`define ALU_SLL   4'd7
`define ALU_SRL   4'd8
`define ALU_SRA   4'd9
`define ALU_PASSB 4'd10

module RISCV_ALU (
    input  signed [31:0] i_SrcA,
    input  signed [31:0] i_SrcB,
    input         [3:0]  i_ALUControl,  
    
    output reg signed [31:0] o_ALUResult,
    output            [3:0]  o_Flags
);

    // -----------------------------------------
    // Adder / Subtractor Logic
    // -----------------------------------------
    wire        w_is_sub;
    wire [32:0] w_B_inv;
    wire [32:0] w_sum;
    
    wire        w_flag_n;
    wire        w_flag_v;
    wire        w_flag_c;
    wire        w_flag_z;

    // Subtraction is active for SUB, SLT, and SLTU
    assign w_is_sub = (i_ALUControl == `ALU_SUB) || 
                      (i_ALUControl == `ALU_SLT) || 
                      (i_ALUControl == `ALU_SLTU);

    // Hardware implementation of A + ~B + 1. 
    assign w_B_inv = w_is_sub ? {1'b0, ~i_SrcB} : {1'b0, i_SrcB};
    assign w_sum   = {1'b0, i_SrcA} + w_B_inv + w_is_sub;

    // Internal flags extracted from the adder core
    assign w_flag_n = w_sum[31];
    assign w_flag_c = w_sum[32]; // Standard carry out
    assign w_flag_v = (i_SrcA[31] ^ i_SrcB[31]) & (i_SrcA[31] ^ w_sum[31]);

    // -----------------------------------------
    // Main ALU Logic
    // -----------------------------------------
    always @ (*) begin
        case (i_ALUControl)
            `ALU_ADD:    o_ALUResult = w_sum[31:0];
            `ALU_SUB:    o_ALUResult = w_sum[31:0];
            `ALU_AND:    o_ALUResult = i_SrcA & i_SrcB;
            `ALU_OR:     o_ALUResult = i_SrcA | i_SrcB;
            `ALU_XOR:    o_ALUResult = i_SrcA ^ i_SrcB;
            
            // SLT: 1 if Signed A < B (Evaluated as N ^ V)
            `ALU_SLT:    o_ALUResult = {31'b0, (w_flag_n ^ w_flag_v)};
            
            // SLTU: 1 if Unsigned A < B (Carry=0 means A < B during A+~B+1)
            `ALU_SLTU:   o_ALUResult = {31'b0, ~w_flag_c};
            
            // Shifts (using Verilog shift operators, ensuring unsigned where needed)
            `ALU_SLL:    o_ALUResult = i_SrcA << i_SrcB[4:0];
            `ALU_SRL:    o_ALUResult = $unsigned(i_SrcA) >> i_SrcB[4:0];
            `ALU_SRA:    o_ALUResult = i_SrcA >>> i_SrcB[4:0]; 
            
            // LUI Pass-through
            `ALU_PASSB:  o_ALUResult = i_SrcB;
            
            default:     o_ALUResult = w_sum[31:0];
        endcase
    end

    // -----------------------------------------
    // Output Flags Packing {N, Z, C, V}
    // -----------------------------------------
    assign w_flag_z = (o_ALUResult == 32'b0);
    
    // N = o_ALUResult[31] (Sign of final output)
    // Z = w_flag_z (Zero)
    // C = w_flag_c (Carry out from adder)
    // V = w_flag_v (Overflow from adder)
    assign o_Flags = {o_ALUResult[31], w_flag_z, w_flag_c, w_flag_v};

endmodule



//  ---------- INLCUDED BLOCK: mux_2to1  ---------- 
module mux_2to1 (
    input [31:0] data_0,
    input [31:0] data_1,
    input select,
    output reg [31:0] out
);

  always @ (*) begin
    if (select)
        out = data_1;
    else
        out = data_0;
end

endmodule



//  ---------- INLCUDED BLOCK: dff_32bit_sync_rstn  ---------- 
module dff_32bit_sync_rstn (
    input wire clk,          // Clock signal
    input wire rstn,         // Synchronous active-low reset
    input wire [31:0] d,     // 32-bit data input
    output reg [31:0] q      // 32-bit data output
);

    // Trigger ONLY on the positive edge of the clock
    always @(posedge clk) begin
        if (!rstn) begin
            q <= 32'b0;      // Reset output to 0 when rstn is low at the clock edge
        end else begin
            q <= d;          // Pass input to output
        end
    end

endmodule



//  ---------- INLCUDED BLOCK: dff_32bit_sync_rstn_test  ---------- 
module dff_32bit_sync_rstn_test (
    input wire clk,              // Clock signal
    input wire rstn,             // Synchronous active-low reset
    input wire test_mode,        // Test mode enable signal
    input wire [31:0] test_val,  // 32-bit test data input
    input wire [31:0] d,         // 32-bit normal data input
  	output reg [31:0] q,          // 32-bit data output
    input wire en               // Clock/Data enable signal

);

    // Trigger ONLY on the positive edge of the clock
    always @(posedge clk) begin
        if (!rstn) begin
            q <= 32'b0;          // Reset has the highest priority
        end else if (test_mode) begin
            q <= test_val;       // Load test value when test_mode is high
        end else if (en) begin
          q <= {d[31:1], 1'b0};              // Normal operation when enabled: pass d to q
        end
        // If not reset, not in test mode, and en is 0, q retains its previous value automatically
    end

endmodule



//  ---------- INLCUDED BLOCK: dual_dff_32bit_sync_negedge_rstn  ---------- 
module dual_dff_32bit_sync_negedge_rstn (
    input wire clk,             // Clock signal
    input wire rstn,            // Synchronous active-low reset (negedge triggered)
    input wire [31:0] d1,       // 32-bit data input 1
    input wire [31:0] d2,       // 32-bit data input 2
    output reg [31:0] q1,       // 32-bit data output 1
    output reg [31:0] q2        // 32-bit data output 2
);

    always @(posedge clk) begin
        if (!rstn) begin
            q1 <= 32'b0;
            q2 <= 32'b0;
        end else begin
            q1 <= d1;
            q2 <= d2;
        end
    end

endmodule



//  ---------- INLCUDED BLOCK: dff_32bit_sync_negedge_rst  ---------- 
module dff_32bit_sync_negedge_rst (
    input wire clk,              // Clock signal
    input wire rstn,             // Synchronous active-low reset
    input wire en,               // Enable signal
    input wire [31:0] d,         // 32-bit data input
    output reg [31:0] q          // 32-bit data output
);

    // Trigger on the positive edge of the clock
    always @(posedge clk) begin
        if (!rstn) begin
            q <= 32'b0;          // Synchronous active-low reset
        end else if (en) begin
            q <= d;              // Load input d when enabled
        end
        // If en is low and not resetting, q holds its current value automatically
    end

endmodule



//  ---------- INLCUDED BLOCK: LSU_STORE  ---------- 
//  ---------- INCLUDED BLOCK: LSU_STORE ----------
module LSU_STORE (
    input [31:0] data,
    input [2:0] LSUctrl,
    output reg [31:0] data_out,

    // New ports
  	input [31:0] address,
    output reg [3:0] byte_enable
);

    localparam b = 3'b000;   // SB
    localparam h = 3'b001;   // SH
    localparam w = 3'b010;   // SW
  	wire [1:0] address_offset;
  	assign address_offset = address[1:0];

    always @(*) begin

        data_out    = 32'b0;
        byte_enable = 4'b0000;

        case (LSUctrl)

            // ---------------- SB ----------------
            b: begin

                case (address_offset)

                    2'b00: begin
                        data_out = {
                            24'b0,
                            data[7:0]
                        };

                        byte_enable = 4'b0001;
                    end


                    2'b01: begin
                        data_out = {
                            16'b0,
                            data[7:0],
                            8'b0
                        };

                        byte_enable = 4'b0010;
                    end


                    2'b10: begin
                        data_out = {
                            8'b0,
                            data[7:0],
                            16'b0
                        };

                        byte_enable = 4'b0100;
                    end


                    2'b11: begin
                        data_out = {
                            data[7:0],
                            24'b0
                        };

                        byte_enable = 4'b1000;
                    end

                endcase
            end


            // ---------------- SH ----------------
            h: begin

                case (address_offset)

                    2'b00: begin
                        data_out = {
                            16'b0,
                            data[15:0]
                        };

                        byte_enable = 4'b0011;
                    end


                    2'b10: begin
                        data_out = {
                            data[15:0],
                            16'b0
                        };

                        byte_enable = 4'b1100;
                    end


                    // Misaligned halfword
                    default: begin
                        data_out    = 32'b0;
                        byte_enable = 4'b0000;
                    end

                endcase
            end


            // ---------------- SW ----------------
            w: begin
                data_out    = data;
                byte_enable = 4'b1111;
            end


            default: begin
                data_out    = 32'b0;
                byte_enable = 4'b0000;
            end

        endcase
    end

endmodule



//  ---------- INLCUDED BLOCK: dff_32bit_sync_rst_n  ---------- 
module dff_32bit_sync_rst_n (
    input wire clk,          // Clock signal
    input wire rstn,         // Synchronous active-low reset
    input wire [31:0] d,     // 32-bit data input
    output reg [31:0] q,     // 32-bit data output
    input wire en            // Enable signal added to the end
);

    // Trigger ONLY on the positive edge of the clock
    always @(posedge clk) begin
        if (!rstn) begin
            q <= 32'b0;      // Reset output to 0 when rstn is low at the clock edge
        end else if (en) begin
            q <= d;          // Pass input to output when enabled
        end
    end

endmodule


// Automatically generated by ChipInventor Cloud EDA Tool - 3.15
// Careful: this file (hdl.v) will be automatically replaced
// when you ask tool to generate top Verilog code by clicking
// at BLOCKS button.

module top (

  input wire CLK,
  input wire rst_n,
  input wire Test,
  input wire [31:0] Test_data

);

//Internal Wires
 wire [31:0] w_1;
 wire [2:0] w_2;
 wire [31:0] w_3;
 wire w_4;
 wire [4:0] w_5;
 wire [4:0] w_6;
 wire [4:0] w_7;
 wire [31:0] w_8;
 wire [31:0] w_9;
 wire [31:0] w_10;
 wire [31:0] w_11;
 wire [31:0] w_12;
 wire [3:0] w_13;
 wire [31:0] w_14;
 wire [3:0] w_16;
 wire [6:0] w_18;
 wire [2:0] w_19;
 wire [6:0] w_20;
 wire w_21;
 wire [31:0] w_22;
 wire [31:0] w_23;
 wire w_26;
 wire [31:0] w_27;
 wire [31:0] w_29;
 wire [1:0] w_30;
 wire [31:0] w_31;
 wire [1:0] w_32;
 wire [31:0] w_33;
 wire [31:0] w_34;
 wire [1:0] w_35;
 wire [31:0] w_38;
 wire [2:0] w_39;
 wire w_41;
 wire w_42;
 wire w_43;
 wire [31:0] w_46;
 wire [31:0] w_49;
 wire [3:0] w_50;

//Instances of Modules
extend_riscv blk2465_9 (
         .input_data (w_1),
         .immsrc (w_2),
         .output_data (w_3)
     );

riscv_4kbReg blk2466_23 (
         .clk (CLK),
         .reset (rst_n),
         .enable_reg_write (w_4),
         .reg_addr1 (w_5),
         .reg_addr2 (w_6),
         .addr_write (w_7),
         .write_data (w_8),
         .rd1_data (w_9),
         .rd2_data (w_10)
     );

RISCV_ALU blk2479_50 (
         .i_SrcA (w_11),
         .i_SrcB (w_12),
         .i_ALUControl (w_13),
         .o_ALUResult (w_14),
         .o_Flags (w_16)
     );

instruction_splitter blk2475_63 (
         .rs1 (w_5),
         .rs2 (w_6),
         .rd (w_7),
         .instruction (w_1),
         .opcode (w_18),
         .funct3 (w_19),
         .funct7 (w_20)
     );

dff_32bit_sync_negedge_rst blk2487_66 (
         .clk (CLK),
         .rstn (rst_n),
         .en (w_21),
         .d (w_22),
         .q (w_23)
     );

mux_2to1 blk2482_77 (
         .data_0 (w_22),
         .data_1 (w_8),
         .select (w_26),
         .out (w_27)
     );

mux3to1_32bit blk2478_78 (
         .out (w_11),
         .in1 (w_23),
         .in0 (w_22),
         .in2 (w_29),
         .sel (w_30)
     );

mux3to1_32bit blk2478_79 (
         .in2 (32'd4),
         .in1 (w_3),
         .out (w_12),
         .in0 (w_31),
         .sel (w_32)
     );

mux3to1_32bit blk2478_80 (
         .out (w_8),
         .in2 (w_14),
         .in0 (w_33),
         .in1 (w_34),
         .sel (w_35)
     );

LSU_READ blk2470_81 (
         .data_out (w_34),
         .data (w_38),
         .LSUctrl (w_39),
         .address (w_33)
     );

dff_32bit_sync_rstn_test blk2485_84 (
         .clk (CLK),
         .rstn (rst_n),
         .test_mode (Test),
         .test_val (Test_data [31:0]),
         .q (w_22),
         .d (w_8),
         .en (w_41)
     );

funct7_5 blk2468_85 (
         .funct7 (w_20),
         .funct7_5 (w_42)
     );

controller #(.EARLY_EXIT(0)) blk2467_86 (
         .clk (CLK),
         .rst_n (rst_n),
         .mem_ready (1'b1),
         .ex_done (1'b1),
         .ImmSrc (w_2),
         .RegWrite (w_4),
         .ALUControl (w_13),
         .Flags (w_16),
         .op (w_18),
         .funct3 (w_19),
         .IRWrite (w_21),
         .AdrSrc (w_26),
         .ALUSrcA (w_30),
         .ALUSrcB (w_32),
         .ResultSrc (w_35),
         .LSUCtrl (w_39),
         .PCWrite (w_41),
         .funct7b5 (w_42),
         .MemWrite (w_43)
     );

dff_32bit_sync_rstn blk2484_89 (
         .clk (CLK),
         .rstn (rst_n),
         .q (w_38),
         .d (w_46)
     );

dff_32bit_sync_rstn blk2484_90 (
         .clk (CLK),
         .rstn (rst_n),
         .d (w_14),
         .q (w_33)
     );

dff_32bit_sync_rst_n blk2489_91 (
         .clk (CLK),
         .rstn (rst_n),
         .q (w_1),
         .en (w_21),
         .d (w_46)
     );

LSU_STORE blk2488_92 (
         .address (w_8),
         .LSUctrl (w_39),
         .data (w_31),
         .data_out (w_49),
         .byte_enable (w_50)
     );

dual_dff_32bit_sync_negedge_rstn blk2486_94 (
         .clk (CLK),
         .rstn (rst_n),
         .d1 (w_9),
         .d2 (w_10),
         .q1 (w_29),
         .q2 (w_31)
     );

RISCV_UNIFIED_MEM blk2472_95 (
         .i_Clk (CLK),
         .i_Address (w_27),
         .i_Write_Enable (w_43),
         .o_Data (w_46),
         .i_Data (w_49),
         .i_Byte_Enable (w_50)
     );


endmodule
