module mux_2to1 (
    input [31:0] data_0,
    input [31:0] data_1,
    input select,
    output reg [31:0] out
);

  always @ (*) begin
    if (select)
        out = data_1;
    else
        out = data_0;
end

endmodule