// Extracted unchanged from RV32_optimized.v.
module RISCV_DMEM_OPTIMISED(
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