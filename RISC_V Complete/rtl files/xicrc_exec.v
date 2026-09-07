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