// Extracted unchanged from RV32_optimized.v.
module address_decoder_optimised(output reg rData_sel,
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