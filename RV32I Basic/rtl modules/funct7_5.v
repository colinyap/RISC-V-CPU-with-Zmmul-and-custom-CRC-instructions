module funct7_5 ( input [6:0] funct7,
                 output wire funct7_5);
  assign funct7_5 = funct7[5];
endmodule //Just extracts bit 5 of funct7