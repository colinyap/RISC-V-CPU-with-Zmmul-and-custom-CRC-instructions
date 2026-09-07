module riscv_4kbReg(input  wire       enable_reg_write, reset,  
               input  wire [4:0]  reg_addr1, reg_addr2, addr_write, 
               input  wire [31:0] write_data, 
               output wire [31:0] rd1_data, rd2_data,
               input  wire        clk);
					
  (* ram_style = "distributed" *) reg [31:0] rf[31:0];

	always @(posedge clk, negedge reset) begin
    if(reset == 1'b0) begin
      rf[0] <= 32'h00000000;
      rf[2] <= 32'h000000FC; //Stack Pointer, pointed to the top of the memeory by deafault
      rf[3] <= 32'h000000E0; //Global Pointer, pointed in the middle of the memory by default
    end
		else begin
      if (enable_reg_write && addr_write != 5'b0) 
			  rf[addr_write] <= write_data;
      else
        rf[addr_write] <= rf[addr_write];
    end
  end

	assign rd1_data = (reg_addr1 != 5'b0) ? rf[reg_addr1] : 32'b0; 
	assign rd2_data = (reg_addr2 != 5'b0) ? rf[reg_addr2] : 32'b0;
endmodule