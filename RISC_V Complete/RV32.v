

//  ---------- INLCUDED BLOCK: MUX2_32  ---------- 
module MUX2_32 (
  input [31:0] A, 
  input [31:0] B, 
  input S, 
  output [31:0] Z
);
  assign Z = (S) ? B : A;
endmodule



//  ---------- INLCUDED BLOCK: RadixBooth  ---------- 
//==========================================================================
// RadixBooth : signed 32x32 -> 64 sequential radix-4 (modified Booth)
//              multiplier.  16 iterations, 17 clocks from start to ready.
//
// Handshake
//   - reset is SYNCHRONOUS and ACTIVE LOW.
//   - Pulse `start` high for exactly one clock with `en` high.  inputA /
//     inputB are sampled on that same edge (no set-up cycle needed).
//   - `busy` goes high on the start edge and low again when the result
//     is latched.  `ready` and `result` update on the SAME clock edge,
//     so `result` is valid on the first cycle `ready` is high, and it
//     holds until the next `start`.
//   - `en` low stalls the whole machine (state is held, not lost).
//
// Both operands are interpreted as two's-complement signed.
//==========================================================================
module RadixBooth (inputA, inputB, result, clk, reset, en, start, ready, busy, start_posedge);

    input      [31:0] inputA;   // multiplicand (signed)
    input      [31:0] inputB;   // multiplier   (signed)
    input             clk;
    input             reset;    // synchronous, active low
    input             en;       // global enable / stall
    input             start;    // one-cycle pulse
    output reg [63:0] result;   // signed product, valid when ready
    output reg        ready;
    output reg        busy;
  	output reg        start_posedge;

    reg [31:0] M;    // multiplicand
    reg [32:0] Q;    // {multiplier, implicit b(-1) = 0}
    reg [63:0] Acc;  // accumulating sum
    reg [5:0]  i;    // UNSIGNED iteration counter, 0,2,...,32
  	reg start_prev;

    //----------------------------------------------------------------------
    // The four Booth multiples, built at full 64-bit width.
    //
    // Negating here (rather than taking a 32-bit two's complement and then
    // sign-extending) is what makes M = 0x80000000 work: -(-2^31) needs 33
    // bits and does not fit back into 32.  Everything is exact mod 2^64,
    // and the true product fits in 64 bits, so the final sum is exact.
    //----------------------------------------------------------------------
    wire [63:0] ext_M   = {{32{M[31]}}, M};
    wire [63:0] ext_2M  =  ext_M << 1;
    wire [63:0] ext_nM  = -ext_M;
    wire [63:0] ext_n2M = -ext_2M;

    // Booth triple (b[i+1], b[i], b[i-1]); guarded so the final state never
    // indexes past the end of Q.
    wire [2:0] code = (i < 6'd32) ? {Q[i+2], Q[i+1], Q[i]} : 3'b000;

    reg [63:0] sel;
    always @* begin
        case (code)
            3'b001, 3'b010: sel = ext_M;    // +1 * M
            3'b011:         sel = ext_2M;   // +2 * M
            3'b100:         sel = ext_n2M;  // -2 * M
            3'b101, 3'b110: sel = ext_nM;   // -1 * M
            default:        sel = 64'd0;    // 000, 111
        endcase
    end

    wire [63:0] partialProd = sel << i;

    //----------------------------------------------------------------------
    // Control / datapath
    //----------------------------------------------------------------------
    always @(posedge clk) begin
        if (!reset) begin
            M      <= 32'd0;
            Q      <= 33'd0;
            Acc    <= 64'd0;
            i      <=  6'd0;
            result <= 64'd0;
            ready  <=  1'b0;
            busy   <=  1'b0;
          	start_prev <= 1'b0;
        end
        else if (en) begin
          start_prev <= start;
          if (start && !start_prev) begin
                M     <= inputA;
                Q     <= {inputB, 1'b0};
                Acc   <= 64'd0;
                i     <=  6'd0;
                ready <=  1'b0;      // clear stale ready from last op
                busy  <=  1'b1;
            end
            else if (busy) begin
                if (i == 6'd32) begin
                    result <= Acc;   // result and ready land on the same edge
                    ready  <= 1'b1;
                    busy   <= 1'b0;
                end
                else begin
                    Acc <= Acc + partialProd;
                    i   <= i + 6'd2;
                end
            end
        end
    end
  always @(*) begin
    start_posedge = (start && !start_prev) ? 1:0;
  end

endmodule



//  ---------- INLCUDED BLOCK: rv32i_core  ---------- 
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



//  ---------- INLCUDED BLOCK: MultiplierWrapper  ---------- 
module MultiplierWrapper (input [63:0] raw_input,
                          output reg [31:0] out,
                          input [2:0] funct3,
                          input [31:0] rs1,
                          input [31:0] rs2);
  
  localparam MUL = 0;
  localparam MULH = 1;
  localparam MULHSU = 2;
  localparam MULHU = 3;
  
  reg [63:0] raw_out;
  
  always @(*) begin
    raw_out = raw_input;
    out = raw_input [63:32];
    case (funct3)
      MUL: begin
        out = raw_input [31:0];
      end
      MULH: begin
        out = raw_input [63:32];
      end
      MULHSU: begin
        if(rs2[31]) begin
          raw_out = raw_input + {rs1, 32'b0};
          out = raw_out [63:32];
        end
      end
      MULHU: begin
        if(rs1[31] && rs2[31]) begin
          raw_out = raw_input + {rs1, 32'b0} + {rs2, 32'b0};
          out = raw_out [63:32];
        end else if (rs1[31]) begin
          raw_out = raw_input + {rs2, 32'b0};
          out = raw_out [63:32];
        end else if (rs2[31]) begin
          raw_out = raw_input + {rs1, 32'b0};
          out = raw_out [63:32];
        end
      end
      default: begin
        raw_out = 0;
        out = 0;
      end
 endcase
 end
 endmodule



//  ---------- INLCUDED BLOCK: xicrc_exec  ---------- 
// Architectural behaviour from the supplied block guide:
//   CRCB -> process  8 bits of rs1_data
//   CRCH -> process 16 bits of rs1_data
//   CRCW -> process 32 bits of rs1_data
//   result is 16 bits
module xicrc_exec #(
    parameter [15:0] POLY = 16'h1021
) (
    input  wire [31:0] rs1_data,
    input  wire [31:0] rs2_data,
    input  wire [1:0]  crc_sel,      // 00=CRCB, 01=CRCH, 10=CRCW
    output reg  [15:0] crc_result
);

    integer i;
    reg [15:0] crc8_result;
    reg [15:0] crc16_result;
    reg [15:0] crc32_result;

    // One MSB-first CRC update step.
    // feedback=1 means the generator polynomial is XORed after the shift.
    function [15:0] crc_next_bit;
        input [15:0] crc;
        input        data_bit;
        reg          feedback;
        begin
            feedback    = crc[15] ^ data_bit;
            crc_next_bit = crc << 1;
            if (feedback)
                crc_next_bit = crc_next_bit ^ POLY;
        end
    endfunction

    always @(*) begin
        // Three explicit combinational paths, matching the block-guide model.
        crc8_result  = rs2_data[15:0];
        crc16_result = rs2_data[15:0];
        crc32_result = rs2_data[15:0];

        for (i = 7; i >= 0; i = i - 1)
            crc8_result = crc_next_bit(crc8_result, rs1_data[i]);

        for (i = 15; i >= 0; i = i - 1)
            crc16_result = crc_next_bit(crc16_result, rs1_data[i]);

        for (i = 31; i >= 0; i = i - 1)
            crc32_result = crc_next_bit(crc32_result, rs1_data[i]);

        case (crc_sel)
            2'b00:   crc_result = crc8_result;
            2'b01:   crc_result = crc16_result;
            2'b10:   crc_result = crc32_result;
            default: crc_result = 16'h0000;
        endcase
    end

endmodule



//  ---------- INLCUDED BLOCK: zeroextend_16to32  ---------- 
module zeroextend_16to32 (input [15:0] in,
                          output [31:0] out);
  assign out = {16'b0, in};
endmodule



//  ---------- INLCUDED BLOCK: ext_secondarycontrol  ---------- 
module ext_secondarycontrol (input[31:0] instruction,
                             output reg write_sel,
                             output reg mul_start,
                             output reg [1:0] crc_sel,
                             input mul_ready,
                             output reg ext_ready,
                             output reg extwrite_en,
                             input [1:0] state,
                            output reg illegal,
                            input start_posedge);
  								
  localparam EXECUTE = 2;
  localparam WRITEBACK = 3;
  localparam MULTIPLY = 7'b0000001;
  localparam CRC = 7'b1000000;
  
wire [2:0] funct3   = instruction[14:12];
wire [6:0] funct7   = instruction[31:25];
wire [6:0] opcode   = instruction[6:0];

  
  always @(*) begin
    crc_sel = 0;
    write_sel = 0;
    mul_start = 0;
    ext_ready = 0;
    extwrite_en = 0;
    if (state == EXECUTE || state == WRITEBACK) begin
      if (opcode == 7'b0110011) begin
        case (funct7)
          MULTIPLY: begin
            write_sel = 0;
            mul_start = 1;
            if (mul_ready && !start_posedge) begin
              ext_ready = 1;
              extwrite_en = 1;
            end
          end
          CRC: begin
            crc_sel = funct3[1:0];
            ext_ready = 1;
            extwrite_en = 1;
            write_sel = 1;
          end
          default: begin
            crc_sel = 0;
            write_sel = 0;
            mul_start = 0;
            ext_ready = 1;
            extwrite_en = 0;
          end
        endcase
      end
    end
  end
  always @(*) begin
    	illegal = 0;
    if (!(opcode == 7'b0110011) || (!(funct7 == MULTIPLY) && !(funct7 == CRC))) begin
       illegal = 1;
   	 end
    if ((funct7 == MULTIPLY) && (!(funct3 == 0) && !(funct3 ==1) && !(funct3 ==2) && !(funct3 == 3))) begin
         illegal = 1;
       end
    if ((funct7 == CRC) && (!(funct3 == 0) && !(funct3 ==1) && !(funct3 ==2))) begin
         illegal = 1;
           end
  end
endmodule



//  ---------- INLCUDED BLOCK: RISCV_DMEM  ---------- 
//  
module RISCV_DMEM(
    input [31:0] i_Data,
    input [31:0] i_Address,
    input i_Write_Enable,
    input i_Clk,
    output [31:0] o_Data,

    // New input added below existing ports
    input [3:0] i_Byte_Enable
);

    /* 128B RAM
    */
  reg [31:0] r_Contents [0:31];

    /* Synchronous Write */
    always @(posedge i_Clk) begin

        if (i_Write_Enable) begin

            if (i_Byte_Enable[0])
              r_Contents[i_Address[6:2]][7:0]
                    <= i_Data[7:0];

            if (i_Byte_Enable[1])
              r_Contents[i_Address[6:2]][15:8]
                    <= i_Data[15:8];

            if (i_Byte_Enable[2])
              r_Contents[i_Address[6:2]][23:16]
                    <= i_Data[23:16];

            if (i_Byte_Enable[3])
              r_Contents[i_Address[6:2]][31:24]
                    <= i_Data[31:24];

        end
    end


    /* Asynchronous Read */
  assign o_Data = r_Contents[i_Address[6:2]];

endmodule



//  ---------- INLCUDED BLOCK: RISCV_iMEM  ---------- 
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

module RISCV_iMEM(
    input [31:0] i_Address,
    input i_Clk,
  output reg [31:0] r_Instruction
);
`ifdef SYNTHESIS
always @(*) begin
  case (i_Address)

        32'h00400000: r_Instruction = 32'h00A00293; // addi x5, x0, 10
        32'h00400004: r_Instruction = 32'h00500313; // addi x6, x0, 5
        32'h00400008: r_Instruction = 32'h006283B3; // add  x7, x5, x6

        default:
            r_Instruction = 32'h00000013; // NOP

    endcase
end
`else
  always @(*) begin
    case (i_Address)
32'h00400000: r_Instruction = 32'h123452B7;
32'h00400004: r_Instruction = 32'h12345337;
32'h00400008: r_Instruction = 32'h3E629463;
32'h0040000C: r_Instruction = 32'h00001297;
32'h00400010: r_Instruction = 32'h3E028063;
32'h00400014: r_Instruction = 32'h00A00293;
32'h00400018: r_Instruction = 32'hFFD28313;
32'h0040001C: r_Instruction = 32'h00700393;
32'h00400020: r_Instruction = 32'h3C731863;
32'h00400024: r_Instruction = 32'h006283B3;
32'h00400028: r_Instruction = 32'h01100E13;
32'h0040002C: r_Instruction = 32'h3DC39263;
32'h00400030: r_Instruction = 32'h405E03B3;
32'h00400034: r_Instruction = 32'h3A639E63;
32'h00400038: r_Instruction = 32'h0FF00293;
32'h0040003C: r_Instruction = 32'h0F02C313;
32'h00400040: r_Instruction = 32'h00F00393;
32'h00400044: r_Instruction = 32'h3A731663;
32'h00400048: r_Instruction = 32'h7002E313;
32'h0040004C: r_Instruction = 32'h7FF00393;
32'h00400050: r_Instruction = 32'h3A731063;
32'h00400054: r_Instruction = 32'h0F02F313;
32'h00400058: r_Instruction = 32'h0F000393;
32'h0040005C: r_Instruction = 32'h38731A63;
32'h00400060: r_Instruction = 32'h0AA00293;
32'h00400064: r_Instruction = 32'h05500313;
32'h00400068: r_Instruction = 32'h0062C3B3;
32'h0040006C: r_Instruction = 32'h0FF00E13;
32'h00400070: r_Instruction = 32'h39C39063;
32'h00400074: r_Instruction = 32'h0062E3B3;
32'h00400078: r_Instruction = 32'h37C39C63;
32'h0040007C: r_Instruction = 32'h0062F3B3;
32'h00400080: r_Instruction = 32'h36039863;
32'h00400084: r_Instruction = 32'h00100293;
32'h00400088: r_Instruction = 32'h00429313;
32'h0040008C: r_Instruction = 32'h01000393;
32'h00400090: r_Instruction = 32'h36731063;
32'h00400094: r_Instruction = 32'h0023D313;
32'h00400098: r_Instruction = 32'h00400E13;
32'h0040009C: r_Instruction = 32'h35C31A63;
32'h004000A0: r_Instruction = 32'hFF000293;
32'h004000A4: r_Instruction = 32'h4022D313;
32'h004000A8: r_Instruction = 32'hFFC00393;
32'h004000AC: r_Instruction = 32'h34731263;
32'h004000B0: r_Instruction = 32'h00100293;
32'h004000B4: r_Instruction = 32'h00400313;
32'h004000B8: r_Instruction = 32'h006293B3;
32'h004000BC: r_Instruction = 32'h01000E13;
32'h004000C0: r_Instruction = 32'h33C39863;
32'h004000C4: r_Instruction = 32'h00200313;
32'h004000C8: r_Instruction = 32'h006E53B3;
32'h004000CC: r_Instruction = 32'h00400E93;
32'h004000D0: r_Instruction = 32'h33D39063;
32'h004000D4: r_Instruction = 32'hFF000293;
32'h004000D8: r_Instruction = 32'h4062D3B3;
32'h004000DC: r_Instruction = 32'hFFC00E93;
32'h004000E0: r_Instruction = 32'h31D39863;
32'h004000E4: r_Instruction = 32'h00A00293;
32'h004000E8: r_Instruction = 32'h0142A313;
32'h004000EC: r_Instruction = 32'h00100393;
32'h004000F0: r_Instruction = 32'h30731063;
32'h004000F4: r_Instruction = 32'hFF600293;
32'h004000F8: r_Instruction = 32'h0142B313;
32'h004000FC: r_Instruction = 32'h2E031A63;
32'h00400100: r_Instruction = 32'h00A00293;
32'h00400104: r_Instruction = 32'h01400313;
32'h00400108: r_Instruction = 32'h0062A3B3;
32'h0040010C: r_Instruction = 32'h00100E13;
32'h00400110: r_Instruction = 32'h2FC39063;
32'h00400114: r_Instruction = 32'h005333B3;
32'h00400118: r_Instruction = 32'h2C039C63;
32'h0040011C: r_Instruction = 32'h0FC10417;
32'h00400120: r_Instruction = 32'hEE440413;
32'h00400124: r_Instruction = 32'h12345337;
32'h00400128: r_Instruction = 32'h67830313;
32'h0040012C: r_Instruction = 32'h00642023;
32'h00400130: r_Instruction = 32'h0000B3B7;
32'h00400134: r_Instruction = 32'hABB38393;
32'h00400138: r_Instruction = 32'h00741223;
32'h0040013C: r_Instruction = 32'h0CC00E13;
32'h00400140: r_Instruction = 32'h01C40423;
32'h00400144: r_Instruction = 32'h00042E83;
32'h00400148: r_Instruction = 32'h2A6E9463;
32'h0040014C: r_Instruction = 32'h00441F03;
32'h00400150: r_Instruction = 32'hFFFFBFB7;
32'h00400154: r_Instruction = 32'hABBF8F93;
32'h00400158: r_Instruction = 32'h29FF1C63;
32'h0040015C: r_Instruction = 32'h00445F03;
32'h00400160: r_Instruction = 32'h0000BFB7;
32'h00400164: r_Instruction = 32'hABBF8F93;
32'h00400168: r_Instruction = 32'h29FF1463;
32'h0040016C: r_Instruction = 32'h00840F03;
32'h00400170: r_Instruction = 32'hFCC00F93;
32'h00400174: r_Instruction = 32'h27FF1E63;
32'h00400178: r_Instruction = 32'h00844F03;
32'h0040017C: r_Instruction = 32'h0CC00F93;
32'h00400180: r_Instruction = 32'h27FF1863;
32'h00400184: r_Instruction = 32'h00500293;
32'h00400188: r_Instruction = 32'h00A00313;
32'h0040018C: r_Instruction = 32'h00500393;
32'h00400190: r_Instruction = 32'hFF600E13;
32'h00400194: r_Instruction = 32'hFF600E93;
32'h00400198: r_Instruction = 32'h00728463;
32'h0040019C: r_Instruction = 32'h2540006F;
32'h004001A0: r_Instruction = 32'h01DE0463;
32'h004001A4: r_Instruction = 32'h24C0006F;
32'h004001A8: r_Instruction = 32'h00629463;
32'h004001AC: r_Instruction = 32'h2440006F;
32'h004001B0: r_Instruction = 32'h01C29463;
32'h004001B4: r_Instruction = 32'h23C0006F;
32'h004001B8: r_Instruction = 32'h0062C463;
32'h004001BC: r_Instruction = 32'h2340006F;
32'h004001C0: r_Instruction = 32'h005E4463;
32'h004001C4: r_Instruction = 32'h22C0006F;
32'h004001C8: r_Instruction = 32'h00535463;
32'h004001CC: r_Instruction = 32'h2240006F;
32'h004001D0: r_Instruction = 32'h01C2D463;
32'h004001D4: r_Instruction = 32'h21C0006F;
32'h004001D8: r_Instruction = 32'h0062E463;
32'h004001DC: r_Instruction = 32'h2140006F;
32'h004001E0: r_Instruction = 32'h01C2E463;
32'h004001E4: r_Instruction = 32'h20C0006F;
32'h004001E8: r_Instruction = 32'h00537463;
32'h004001EC: r_Instruction = 32'h2040006F;
32'h004001F0: r_Instruction = 32'h005E7463;
32'h004001F4: r_Instruction = 32'h1FC0006F;
32'h004001F8: r_Instruction = 32'h00800F6F;
32'h004001FC: r_Instruction = 32'h1F40006F;
32'h00400200: r_Instruction = 32'h00000F97;
32'h00400204: r_Instruction = 32'h010F8F93;
32'h00400208: r_Instruction = 32'h000F8067;
32'h0040020C: r_Instruction = 32'h1E40006F;
32'h00400210: r_Instruction = 32'h00100013;
32'h00400214: r_Instruction = 32'h1C001E63;
32'h00400218: r_Instruction = 32'hDEADC2B7;
32'h0040021C: r_Instruction = 32'hEEF28293;
32'h00400220: r_Instruction = 32'h00028313;
32'h00400224: r_Instruction = 32'h00030393;
32'h00400228: r_Instruction = 32'h00038F93;
32'h0040022C: r_Instruction = 32'hDEADC2B7;
32'h00400230: r_Instruction = 32'hEEF28293;
32'h00400234: r_Instruction = 32'h1A5F9E63;
32'h00400238: r_Instruction = 32'h000185B7;
32'h0040023C: r_Instruction = 32'h6A058593;
32'h00400240: r_Instruction = 32'h00200613;
32'h00400244: r_Instruction = 32'hEE6B36B7;
32'h00400248: r_Instruction = 32'h80068693;
32'h0040024C: r_Instruction = 32'h000312B7;
32'h00400250: r_Instruction = 32'hD4028293;
32'h00400254: r_Instruction = 32'hDCD65337;
32'h00400258: r_Instruction = 32'hFFF00393;
32'h0040025C: r_Instruction = 32'h00100E13;
32'h00400260: r_Instruction = 32'h02C58533;
32'h00400264: r_Instruction = 32'h18551663;
32'h00400268: r_Instruction = 32'h02C68533;
32'h0040026C: r_Instruction = 32'h18651263;
32'h00400270: r_Instruction = 32'h02C59533;
32'h00400274: r_Instruction = 32'h16051E63;
32'h00400278: r_Instruction = 32'h02C69533;
32'h0040027C: r_Instruction = 32'h16751A63;
32'h00400280: r_Instruction = 32'h02C6B533;
32'h00400284: r_Instruction = 32'h17C51663;
32'h00400288: r_Instruction = 32'h02C6A533;
32'h0040028C: r_Instruction = 32'h16751263;
32'h00400290: r_Instruction = 32'h02D62533;
32'h00400294: r_Instruction = 32'h15C51E63;
32'h00400298: r_Instruction = 32'h000028B7;
32'h0040029C: r_Instruction = 32'hE8288893;
32'h004002A0: r_Instruction = 32'h00010437;
32'h004002A4: r_Instruction = 32'hFFF40413;
32'h004002A8: r_Instruction = 32'h01200493;
32'h004002AC: r_Instruction = 32'h03400913;
32'h004002B0: r_Instruction = 32'h05600993;
32'h004002B4: r_Instruction = 32'h07800A13;
32'h004002B8: r_Instruction = 32'h09000A93;
32'h004002BC: r_Instruction = 32'h0AB00B13;
32'h004002C0: r_Instruction = 32'h0CD00B93;
32'h004002C4: r_Instruction = 32'h0EF00C13;
32'h004002C8: r_Instruction = 32'h80848433;
32'h004002CC: r_Instruction = 32'h80890433;
32'h004002D0: r_Instruction = 32'h80898433;
32'h004002D4: r_Instruction = 32'h808A0433;
32'h004002D8: r_Instruction = 32'h808A8433;
32'h004002DC: r_Instruction = 32'h808B0433;
32'h004002E0: r_Instruction = 32'h808B8433;
32'h004002E4: r_Instruction = 32'h808C0433;
32'h004002E8: r_Instruction = 32'h11141463;
32'h004002EC: r_Instruction = 32'h000102B7;
32'h004002F0: r_Instruction = 32'hFFF28293;
32'h004002F4: r_Instruction = 32'h00001337;
32'h004002F8: r_Instruction = 32'h23430313;
32'h004002FC: r_Instruction = 32'h000053B7;
32'h00400300: r_Instruction = 32'h67838393;
32'h00400304: r_Instruction = 32'h00009E37;
32'h00400308: r_Instruction = 32'h0ABE0E13;
32'h0040030C: r_Instruction = 32'h0000DEB7;
32'h00400310: r_Instruction = 32'hDEFE8E93;
32'h00400314: r_Instruction = 32'h805312B3;
32'h00400318: r_Instruction = 32'h805392B3;
32'h0040031C: r_Instruction = 32'h805E12B3;
32'h00400320: r_Instruction = 32'h805E92B3;
32'h00400324: r_Instruction = 32'h0D129663;
32'h00400328: r_Instruction = 32'h00010537;
32'h0040032C: r_Instruction = 32'hFFF50513;
32'h00400330: r_Instruction = 32'h123455B7;
32'h00400334: r_Instruction = 32'h67858593;
32'h00400338: r_Instruction = 32'h90ABD637;
32'h0040033C: r_Instruction = 32'hDEF60613;
32'h00400340: r_Instruction = 32'h80A5A533;
32'h00400344: r_Instruction = 32'h80A62533;
32'h00400348: r_Instruction = 32'h0B151463;
32'h0040034C: r_Instruction = 32'h00000297;
32'h00400350: r_Instruction = 32'h0AC28293;
32'h00400354: r_Instruction = 32'h0002A303;
32'h00400358: r_Instruction = 32'h0042A383;
32'h0040035C: r_Instruction = 32'h00010537;
32'h00400360: r_Instruction = 32'hFFF50513;
32'h00400364: r_Instruction = 32'h80A32533;
32'h00400368: r_Instruction = 32'h80A3A533;
32'h0040036C: r_Instruction = 32'h000028B7;
32'h00400370: r_Instruction = 32'hE8288893;
32'h00400374: r_Instruction = 32'h07151E63;
32'h00400378: r_Instruction = 32'h01400293;
32'h0040037C: r_Instruction = 32'h00A00313;
32'h00400380: r_Instruction = 32'h006283B3;
32'h00400384: r_Instruction = 32'h40628E33;
32'h00400388: r_Instruction = 32'h03C38EB3;
32'h0040038C: r_Instruction = 32'h12C00F13;
32'h00400390: r_Instruction = 32'h07EE9063;
32'h00400394: r_Instruction = 32'h00000297;
32'h00400398: r_Instruction = 32'h06C28293;
32'h0040039C: r_Instruction = 32'h0FC10317;
32'h004003A0: r_Instruction = 32'hC7430313;
32'h004003A4: r_Instruction = 32'h00300393;
32'h004003A8: r_Instruction = 32'h0002AE03;
32'h004003AC: r_Instruction = 32'h01C32023;
32'h004003B0: r_Instruction = 32'h00428293;
32'h004003B4: r_Instruction = 32'h00430313;
32'h004003B8: r_Instruction = 32'hFFF38393;
32'h004003BC: r_Instruction = 32'hFE0396E3;
32'h004003C0: r_Instruction = 32'h0FC10317;
32'h004003C4: r_Instruction = 32'hC5030313;
32'h004003C8: r_Instruction = 32'h00032E03;
32'h004003CC: r_Instruction = 32'h11111EB7;
32'h004003D0: r_Instruction = 32'h111E8E93;
32'h004003D4: r_Instruction = 32'h01DE1E63;
32'h004003D8: r_Instruction = 32'h00832E03;
32'h004003DC: r_Instruction = 32'h33333EB7;
32'h004003E0: r_Instruction = 32'h333E8E93;
32'h004003E4: r_Instruction = 32'h01DE1663;
32'h004003E8: r_Instruction = 32'h00000213;
32'h004003EC: r_Instruction = 32'h0000006F;
32'h004003F0: r_Instruction = 32'hFFF00213;
32'h004003F4: r_Instruction = 32'h0000006F;
32'h004003F8: r_Instruction = 32'h12345678;
32'h004003FC: r_Instruction = 32'h90ABCDEF;
32'h00400400: r_Instruction = 32'h11111111;
32'h00400404: r_Instruction = 32'h22222222;
32'h00400408: r_Instruction = 32'h33333333;
default:
r_Instruction = 32'h00000013; // NOP
    endcase
  end
  `endif

endmodule



//  ---------- INLCUDED BLOCK: address_decoder  ---------- 
module address_decoder (output reg rData_sel,
                        input [1:0] state,
                        input [31:0] Address
);
  localparam FETCH = 0;
  
  always @(*) begin
    if (state == FETCH || (Address >= 32'h0040_0000 && Address <= 32'h0040_0FFF)) begin
      rData_sel = 0;
    end else begin
      rData_sel = 1;
    end
  end
endmodule


// Automatically generated by ChipInventor Cloud EDA Tool - 3.15
// Careful: this file (hdl.v) will be automatically replaced
// when you ask tool to generate top Verilog code by clicking
// at BLOCKS button.

module top (

  input wire clk,
  input wire rst_n,
  output wire syscall,
  output wire illegal_instruct,
  output wire [31:0] debug_PC,
  output wire [1:0] debug_state

);

//Internal Wires
 wire [31:0] w_1;
 wire w_2;
 wire [31:0] w_3;
 wire w_4;
 wire w_5;
 wire [31:0] w_6;
 wire [31:0] w_9;
 wire w_10;
 wire [3:0] w_11;
 wire [31:0] w_12;
 wire [2:0] w_13;
 wire [31:0] w_14;
 wire [31:0] w_17;
 wire [1:0] w_20;
 wire [63:0] w_24;
 wire [31:0] w_25;
 wire [1:0] w_26;
 wire [15:0] w_27;
 wire [31:0] w_28;
 wire w_29;
 wire [31:0] w_30;
 wire [31:0] w_31;
 wire w_32;
 wire w_33;
 wire w_34;
 wire w_35;

//Interface Assigns
assign debug_state [1:0] = w_20;

//Instances of Modules
rv32i_core blk3708_24 (
         .clk (clk),
         .rst_n (rst_n),
         .syscall (syscall),
         .illegal_instruction (illegal_instruct),
         .debug_pc (debug_PC [31:0]),
         .debug_state (w_20),
         .Rdata (w_1),
         .ext_ready (w_2),
         .ext_result (w_3),
         .ext_writeback (w_4),
         .ext_illegal (w_5),
         .Address (w_6),
         .WriteData (w_9),
         .MemWrite (w_10),
         .ByteStrobe (w_11),
         .ext_instruction (w_12),
         .ext_funct3 (w_13),
         .ext_rs1_data (w_14),
         .ext_rs2_data (w_17)
     );

MultiplierWrapper blk4140_28 (
         .funct3 (w_13),
         .rs1 (w_14),
         .rs2 (w_17),
         .raw_input (w_24),
         .out (w_25)
     );

xicrc_exec blk4146_34 (
         .rs1_data (w_14),
         .rs2_data (w_17),
         .crc_sel (w_26),
         .crc_result (w_27)
     );

zeroextend_16to32 blk4147_35 (
         .in (w_27),
         .out (w_28)
     );

MUX2_32 blk1779_37 (
         .Z (w_3),
         .A (w_25),
         .B (w_28),
         .S (w_29)
     );

MUX2_32 blk1779_50 (
         .Z (w_1),
         .A (w_30),
         .B (w_31),
         .S (w_32)
     );

RISCV_DMEM blk4191_54 (
         .i_Clk (clk),
         .i_Address (w_6),
         .i_Data (w_9),
         .i_Write_Enable (w_10),
         .i_Byte_Enable (w_11),
         .o_Data (w_31)
     );

RISCV_iMEM blk4195_67 (
         .i_Clk (clk),
         .i_Address (w_6),
         .r_Instruction (w_30)
     );

RadixBooth blk2514_72 (
         .clk (clk),
         .reset (rst_n),
         .en (1'b1),
         .inputA (w_14),
         .inputB (w_17),
         .result (w_24),
         .start (w_33),
         .ready (w_34),
         .start_posedge (w_35)
     );

ext_secondarycontrol blk4154_73 (
         .ext_ready (w_2),
         .extwrite_en (w_4),
         .illegal (w_5),
         .instruction (w_12),
         .state (w_20),
         .crc_sel (w_26),
         .write_sel (w_29),
         .mul_start (w_33),
         .mul_ready (w_34),
         .start_posedge (w_35)
     );

address_decoder blk4199_74 (
         .Address (w_6),
         .state (w_20),
         .rData_sel (w_32)
     );


endmodule
