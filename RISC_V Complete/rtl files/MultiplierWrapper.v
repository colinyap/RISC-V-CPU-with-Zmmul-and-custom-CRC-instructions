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