//  ---------- INCLUDED BLOCK: LSU_STORE ----------
module LSU_STORE (
    input [31:0] data,
    input [2:0] LSUctrl,
    output reg [31:0] data_out,

    // New ports
  	input [31:0] address,
    output reg [3:0] byte_enable
);

    localparam b = 3'b000;   // SB
    localparam h = 3'b001;   // SH
    localparam w = 3'b010;   // SW
  	wire [1:0] address_offset;
  	assign address_offset = address[1:0];

    always @(*) begin

        data_out    = 32'b0;
        byte_enable = 4'b0000;

        case (LSUctrl)

            // ---------------- SB ----------------
            b: begin

                case (address_offset)

                    2'b00: begin
                        data_out = {
                            24'b0,
                            data[7:0]
                        };

                        byte_enable = 4'b0001;
                    end


                    2'b01: begin
                        data_out = {
                            16'b0,
                            data[7:0],
                            8'b0
                        };

                        byte_enable = 4'b0010;
                    end


                    2'b10: begin
                        data_out = {
                            8'b0,
                            data[7:0],
                            16'b0
                        };

                        byte_enable = 4'b0100;
                    end


                    2'b11: begin
                        data_out = {
                            data[7:0],
                            24'b0
                        };

                        byte_enable = 4'b1000;
                    end

                endcase
            end


            // ---------------- SH ----------------
            h: begin

                case (address_offset)

                    2'b00: begin
                        data_out = {
                            16'b0,
                            data[15:0]
                        };

                        byte_enable = 4'b0011;
                    end


                    2'b10: begin
                        data_out = {
                            data[15:0],
                            16'b0
                        };

                        byte_enable = 4'b1100;
                    end


                    // Misaligned halfword
                    default: begin
                        data_out    = 32'b0;
                        byte_enable = 4'b0000;
                    end

                endcase
            end


            // ---------------- SW ----------------
            w: begin
                data_out    = data;
                byte_enable = 4'b1111;
            end


            default: begin
                data_out    = 32'b0;
                byte_enable = 4'b0000;
            end

        endcase
    end

endmodule