module extend_riscv(
  input [31:0] input_data,
  output reg [31:0] output_data,
  input [2:0] immsrc
);

  reg [11:0] imm;
  reg [19:0] imm20bit;
always @(*) begin
  imm = 0;
  imm20bit = 0;
  output_data = 0;
  case (immsrc)
    0: begin
      imm = input_data[31:20];
      output_data = {{20{imm[11]}}, imm[11:0]};
    end
    1: begin
      imm = {input_data[31:25], input_data[11:7]};
      output_data = {{20{imm[11]}}, imm[11:0]};
    end
    2: begin
      imm = {input_data[31], input_data[7], input_data[30:25], input_data[11:8]};
      output_data = {{19{imm[11]}}, imm[11:0], 1'b0};
    end
    4: begin
      imm20bit = input_data[31:12];
      output_data = {imm20bit[19:0], {12{1'b0}}};
    end
    3: begin
      imm20bit = {input_data[31], input_data[19:12], input_data[20], input_data[30:21]};
      output_data = {{11{imm20bit[19]}}, imm20bit[19:0], 1'b0};
    end
    default: begin
      imm = 0;
      imm20bit = 0;
      output_data = 0;
    end
  endcase

end

endmodule