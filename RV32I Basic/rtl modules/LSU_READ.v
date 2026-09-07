//  ---------- INCLUDED BLOCK: LSU_READ ----------
module LSU_READ (
    input [31:0] data,
    input [2:0] LSUctrl,
    output reg [31:0] data_out,

    // New input added below existing ports
  	input [31:0] address
);

    localparam b  = 3'b000;   // LB
    localparam h  = 3'b001;   // LH
    localparam w  = 3'b010;   // LW
    localparam bu = 3'b100;   // LBU
    localparam hu = 3'b101;   // LHU
  wire [1:0] address_offset;
  assign address_offset = address [1:0];
    reg [15:0] half;
    reg [7:0] b_yte;

    always @(*) begin
        half     = 16'b0;
        b_yte    = 8'b0;
        data_out = 32'b0;

        case (LSUctrl)

            // ---------------- LW ----------------
            w: begin
                data_out = data;
            end


            // ---------------- LH ----------------
            h: begin
                case (address_offset)
                    2'b00: half = data[15:0];
                    2'b10: half = data[31:16];

                    // Misaligned halfword
                    default: half = 16'b0;
                endcase

                data_out = {{16{half[15]}}, half};
            end


            // ---------------- LB ----------------
            b: begin
                case (address_offset)
                    2'b00: b_yte = data[7:0];
                    2'b01: b_yte = data[15:8];
                    2'b10: b_yte = data[23:16];
                    2'b11: b_yte = data[31:24];
                endcase

                data_out = {{24{b_yte[7]}}, b_yte};
            end


            // ---------------- LBU ----------------
            bu: begin
                case (address_offset)
                    2'b00: b_yte = data[7:0];
                    2'b01: b_yte = data[15:8];
                    2'b10: b_yte = data[23:16];
                    2'b11: b_yte = data[31:24];
                endcase

                data_out = {24'b0, b_yte};
            end


            // ---------------- LHU ----------------
            hu: begin
                case (address_offset)
                    2'b00: half = data[15:0];
                    2'b10: half = data[31:16];

                    // Misaligned halfword
                    default: half = 16'b0;
                endcase

                data_out = {16'b0, half};
            end


            default: begin
                half     = 16'b0;
                b_yte    = 8'b0;
                data_out = 32'b0;
            end

        endcase
    end

endmodule