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