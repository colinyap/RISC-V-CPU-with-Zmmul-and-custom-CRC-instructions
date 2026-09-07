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