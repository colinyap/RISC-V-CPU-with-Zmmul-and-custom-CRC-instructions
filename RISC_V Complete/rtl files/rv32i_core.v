// RV32I multicycle processor core
//
// This file intentionally contains exactly one synthesizable module.  The
// register file, controller, immediate generator, ALU and load/store alignment
// logic are all implemented inside the core; no leaf modules are instantiated.
//
// Memory is external and uses one simple shared interface. Address selects the
// current instruction or data access, Rdata returns a word, and stores drive
// WriteData, MemWrite, and ByteStrobe. The external address decoder handles all
// memory and MMIO selection.
//
// R-type extension interface:
//   The current instruction, decoded register addresses, and source operands
//   are exposed continuously.  An external extension controller decides when
//   to act by examining ext_instruction together with debug_state (2'd2 means
//   EXECUTE).  Internally, a non-base OP encoding waits for ext_ready, allowing
//   either a combinational or multicycle extension.  Set ext_illegal on
//   completion to reject an unsupported encoding, and ext_writeback to write
//   ext_result to rd.  Tie ext_ready=1, ext_illegal=1, ext_writeback=0 and
//   ext_result=0 when no extension block is connected.

module rv32i_core #(
    parameter [31:0] RESET_PC = 32'h0040_0000,
    parameter [31:0] RESET_SP = 32'h0000_00FC,
    parameter [31:0] RESET_GP = 32'h0000_00E0
) (
    input  wire        clk,
    input  wire        rst_n,

    // Shared instruction/data memory interface.
    output wire [31:0] Address,
    output wire [31:0] WriteData,
    input  wire [31:0] Rdata,
    output wire        MemWrite,
    output wire [3:0]  ByteStrobe,

    // Continuously exposed instruction and R-type extension hook.
    output wire [31:0] ext_instruction,
    output wire [6:0]  ext_funct7,
    output wire [2:0]  ext_funct3,
    output wire [4:0]  ext_rs1_addr,
    output wire [4:0]  ext_rs2_addr,
    output wire [4:0]  ext_rd_addr,
    output wire [31:0] ext_rs1_data,
    output wire [31:0] ext_rs2_data,
    input  wire        ext_ready,
    input  wire [31:0] ext_result,
    input  wire        ext_writeback,
    input  wire        ext_illegal,

    // Status hooks.  These are one-cycle pulses unless the relevant external
    // transaction is deliberately stalled.
    output wire        syscall,
    output wire        illegal_instruction,
    output wire [1:0]  debug_state,
    output wire [31:0] debug_pc
);

    localparam [1:0] S_FETCH  = 2'd0;
    localparam [1:0] S_DECODE = 2'd1;
    localparam [1:0] S_EXEC   = 2'd2;
    localparam [1:0] S_WB     = 2'd3;

    localparam [6:0] OP_LOAD   = 7'b0000011;
    localparam [6:0] OP_MISCM  = 7'b0001111;
    localparam [6:0] OP_IMM    = 7'b0010011;
    localparam [6:0] OP_AUIPC  = 7'b0010111;
    localparam [6:0] OP_STORE  = 7'b0100011;
    localparam [6:0] OP_REG    = 7'b0110011;
    localparam [6:0] OP_LUI    = 7'b0110111;
    localparam [6:0] OP_BRANCH = 7'b1100011;
    localparam [6:0] OP_JALR   = 7'b1100111;
    localparam [6:0] OP_JAL    = 7'b1101111;
    localparam [6:0] OP_SYSTEM = 7'b1110011;

    reg [1:0]  state;
    reg [31:0] pc;
    reg [31:0] old_pc;
    reg [31:0] instruction;
    reg [31:0] operand_a;
    reg [31:0] operand_b;
    reg [31:0] result_reg;
    reg        extension_writeback_reg;
    reg        extension_illegal_reg;
    reg [31:0] registers [0:31];
    integer reset_index;

    wire [6:0] opcode = instruction[6:0];
    wire [4:0] rd     = instruction[11:7];
    wire [2:0] funct3 = instruction[14:12];
    wire [4:0] rs1    = instruction[19:15];
    wire [4:0] rs2    = instruction[24:20];
    wire [6:0] funct7 = instruction[31:25];

    wire [31:0] rs1_value = (rs1 == 5'd0) ? 32'd0 : registers[rs1];
    wire [31:0] rs2_value = (rs2 == 5'd0) ? 32'd0 : registers[rs2];

    wire [31:0] imm_i = {{20{instruction[31]}}, instruction[31:20]};
    wire [31:0] imm_s = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
    wire [31:0] imm_b = {{19{instruction[31]}}, instruction[31], instruction[7],
                         instruction[30:25], instruction[11:8], 1'b0};
    wire [31:0] imm_u = {instruction[31:12], 12'b0};
    wire [31:0] imm_j = {{11{instruction[31]}}, instruction[31], instruction[19:12],
                         instruction[20], instruction[30:21], 1'b0};

    // Full funct7 checks are important here: encodings reserved for extensions
    // must not accidentally execute as a base ADD, shift, etc.
    reg base_rtype;
    always @(*) begin
        base_rtype = 1'b0;
        case (funct3)
            3'b000: base_rtype = (funct7 == 7'b0000000) || (funct7 == 7'b0100000);
            3'b101: base_rtype = (funct7 == 7'b0000000) || (funct7 == 7'b0100000);
            default: base_rtype = (funct7 == 7'b0000000);
        endcase
    end

    wire extension_rtype = (opcode == OP_REG) && !base_rtype;
    wire memory_op = (opcode == OP_LOAD) || (opcode == OP_STORE);
    wire [31:0] effective_addr = operand_a + ((opcode == OP_STORE) ? imm_s : imm_i);

    // FETCH uses the PC; a load/store in EXECUTE uses its effective address.
    assign Address  = ((state == S_EXEC) && memory_op) ? effective_addr : pc;
    assign MemWrite = (state == S_EXEC) && (opcode == OP_STORE);

    reg [31:0] store_data;
    reg [3:0]  store_strobes;
    always @(*) begin
        store_data    = 32'd0;
        store_strobes = 4'b0000;
        case (funct3)
            3'b000: begin // SB
                case (effective_addr[1:0])
                    2'd0: begin store_data = {24'd0, operand_b[7:0]};       store_strobes = 4'b0001; end
                    2'd1: begin store_data = {16'd0, operand_b[7:0], 8'd0}; store_strobes = 4'b0010; end
                    2'd2: begin store_data = {8'd0, operand_b[7:0], 16'd0}; store_strobes = 4'b0100; end
                    2'd3: begin store_data = {operand_b[7:0], 24'd0};       store_strobes = 4'b1000; end
                endcase
            end
            3'b001: begin // SH; a misaligned halfword produces no byte strobes.
                case (effective_addr[1:0])
                    2'd0: begin store_data = {16'd0, operand_b[15:0]}; store_strobes = 4'b0011; end
                    2'd2: begin store_data = {operand_b[15:0], 16'd0}; store_strobes = 4'b1100; end
                    default: begin store_data = 32'd0; store_strobes = 4'b0000; end
                endcase
            end
            3'b010: begin store_data = operand_b; store_strobes = 4'b1111; end // SW
            default: begin store_data = 32'd0; store_strobes = 4'b0000; end
        endcase
    end
    assign WriteData  = store_data;
    assign ByteStrobe = MemWrite ? store_strobes : 4'b0000;

    reg [31:0] load_value;
    reg [15:0] selected_half;
    reg [7:0]  selected_byte;
    always @(*) begin
        case (effective_addr[1:0])
            2'd0: selected_byte = Rdata[7:0];
            2'd1: selected_byte = Rdata[15:8];
            2'd2: selected_byte = Rdata[23:16];
            default: selected_byte = Rdata[31:24];
        endcase
        case (effective_addr[1:0])
            2'd0: selected_half = Rdata[15:0];
            2'd2: selected_half = Rdata[31:16];
            default: selected_half = 16'd0;
        endcase
        case (funct3)
            3'b000: load_value = {{24{selected_byte[7]}}, selected_byte};
            3'b001: load_value = {{16{selected_half[15]}}, selected_half};
            3'b010: load_value = Rdata;
            3'b100: load_value = {24'd0, selected_byte};
            3'b101: load_value = {16'd0, selected_half};
            default: load_value = 32'd0;
        endcase
    end

    // These observation signals are always driven from the current registered
    // instruction, regardless of opcode legality or processor state.
    assign ext_instruction = instruction;
    assign ext_funct7      = funct7;
    assign ext_funct3      = funct3;
    assign ext_rs1_addr    = rs1;
    assign ext_rs2_addr    = rs2;
    assign ext_rd_addr     = rd;
    assign ext_rs1_data    = rs1_value;
    assign ext_rs2_data    = rs2_value;

    reg opcode_legal;
    reg encoding_legal;
    always @(*) begin
        case (opcode)
            OP_LOAD, OP_MISCM, OP_IMM, OP_AUIPC, OP_STORE, OP_REG,
            OP_LUI, OP_BRANCH, OP_JALR, OP_JAL, OP_SYSTEM: opcode_legal = 1'b1;
            default: opcode_legal = 1'b0;
        endcase

        encoding_legal = 1'b1;
        case (opcode)
            OP_LOAD: begin
                case (funct3)
                    3'b000, 3'b001, 3'b010, 3'b100, 3'b101: encoding_legal = 1'b1;
                    default: encoding_legal = 1'b0;
                endcase
            end
            OP_STORE: begin
                case (funct3)
                    3'b000, 3'b001, 3'b010: encoding_legal = 1'b1;
                    default: encoding_legal = 1'b0;
                endcase
            end
            OP_IMM: begin
                case (funct3)
                    3'b001: encoding_legal = (funct7 == 7'b0000000);
                    3'b101: encoding_legal = (funct7 == 7'b0000000) || (funct7 == 7'b0100000);
                    default: encoding_legal = 1'b1;
                endcase
            end
            OP_BRANCH: begin
                case (funct3)
                    3'b000, 3'b001, 3'b100, 3'b101, 3'b110, 3'b111: encoding_legal = 1'b1;
                    default: encoding_legal = 1'b0;
                endcase
            end
            OP_JALR: encoding_legal = (funct3 == 3'b000);
            OP_REG:  encoding_legal = base_rtype; // Extension overrides this on completion.
            default: encoding_legal = 1'b1;
        endcase
    end

    assign syscall = (state == S_EXEC) && (opcode == OP_SYSTEM);
    assign illegal_instruction = (state == S_EXEC) &&
        ((!opcode_legal) ||
         ((!extension_rtype) && !encoding_legal) ||
         (extension_rtype && ext_ready && ext_illegal));
    assign debug_state = state;
    assign debug_pc    = pc;

    reg [31:0] base_alu_result;
    always @(*) begin
        case (funct3)
            3'b000: base_alu_result = (funct7[5] && (opcode == OP_REG)) ?
                                      (operand_a - operand_b) :
                                      (operand_a + ((opcode == OP_IMM) ? imm_i : operand_b));
            3'b001: base_alu_result = operand_a << ((opcode == OP_IMM) ? instruction[24:20] : operand_b[4:0]);
            3'b010: base_alu_result = ($signed(operand_a) < $signed((opcode == OP_IMM) ? imm_i : operand_b)) ? 32'd1 : 32'd0;
            3'b011: base_alu_result = (operand_a < ((opcode == OP_IMM) ? imm_i : operand_b)) ? 32'd1 : 32'd0;
            3'b100: base_alu_result = operand_a ^ ((opcode == OP_IMM) ? imm_i : operand_b);
            3'b101: begin
                if (funct7[5])
                    base_alu_result = $signed(operand_a) >>> ((opcode == OP_IMM) ? instruction[24:20] : operand_b[4:0]);
                else
                    base_alu_result = operand_a >> ((opcode == OP_IMM) ? instruction[24:20] : operand_b[4:0]);
            end
            3'b110: base_alu_result = operand_a | ((opcode == OP_IMM) ? imm_i : operand_b);
            default: base_alu_result = operand_a & ((opcode == OP_IMM) ? imm_i : operand_b);
        endcase
    end

    reg branch_taken;
    always @(*) begin
        case (funct3)
            3'b000: branch_taken = (operand_a == operand_b);
            3'b001: branch_taken = (operand_a != operand_b);
            3'b100: branch_taken = ($signed(operand_a) <  $signed(operand_b));
            3'b101: branch_taken = ($signed(operand_a) >= $signed(operand_b));
            3'b110: branch_taken = (operand_a <  operand_b);
            3'b111: branch_taken = (operand_a >= operand_b);
            default: branch_taken = 1'b0;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_FETCH;
            pc          <= RESET_PC;
            old_pc      <= 32'd0;
            instruction <= 32'd0;
            operand_a   <= 32'd0;
            operand_b   <= 32'd0;
            result_reg  <= 32'd0;
            extension_writeback_reg <= 1'b0;
            extension_illegal_reg   <= 1'b0;
            for (reset_index = 0; reset_index < 32; reset_index = reset_index + 1)
                registers[reset_index] <= 32'd0;
            registers[2] <= RESET_SP;
            registers[3] <= RESET_GP;
        end else begin
            registers[0] <= 32'd0;
            case (state)
                S_FETCH: begin
                    instruction <= Rdata;
                    old_pc      <= pc;
                    pc          <= pc + 32'd4;
                    state       <= S_DECODE;
                end

                S_DECODE: begin
                    operand_a <= rs1_value;
                    operand_b <= rs2_value;
                    state     <= S_EXEC;
                end

                S_EXEC: begin
                    case (opcode)
                        OP_LOAD: begin
                            result_reg <= load_value;
                            state      <= S_WB;
                        end
                        OP_STORE: state <= S_WB;
                        OP_REG: begin
                            if (base_rtype) begin
                                result_reg <= base_alu_result;
                                state      <= S_WB;
                            end else if (ext_ready) begin
                                result_reg              <= ext_result;
                                extension_writeback_reg <= ext_writeback;
                                extension_illegal_reg   <= ext_illegal;
                                state                   <= S_WB;
                            end
                        end
                        OP_IMM: begin result_reg <= base_alu_result; state <= S_WB; end
                        OP_LUI: begin result_reg <= imm_u; state <= S_WB; end
                        OP_AUIPC: begin result_reg <= old_pc + imm_u; state <= S_WB; end
                        OP_BRANCH: begin
                            if (branch_taken) pc <= old_pc + imm_b;
                            state <= S_WB;
                        end
                        OP_JAL: begin
                            result_reg <= old_pc + 32'd4;
                            pc         <= old_pc + imm_j;
                            state      <= S_WB;
                        end
                        OP_JALR: begin
                            result_reg <= old_pc + 32'd4;
                            pc         <= (operand_a + imm_i) & 32'hFFFF_FFFE;
                            state      <= S_WB;
                        end
                        default: state <= S_WB; // FENCE, SYSTEM and illegal encodings are non-trapping NOPs.
                    endcase
                end

                S_WB: begin
                    if (rd != 5'd0) begin
                        case (opcode)
                            OP_LOAD, OP_IMM, OP_LUI, OP_AUIPC, OP_JAL, OP_JALR:
                                registers[rd] <= result_reg;
                            OP_REG: begin
                                if (base_rtype || (!extension_illegal_reg && extension_writeback_reg))
                                    registers[rd] <= result_reg;
                            end
                            default: ;
                        endcase
                    end
                    state <= S_FETCH;
                end

                default: state <= S_FETCH;
            endcase
        end
    end

endmodule