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