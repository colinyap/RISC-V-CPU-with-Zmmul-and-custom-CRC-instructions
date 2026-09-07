module dual_dff_32bit_sync_negedge_rstn (
    input wire clk,             // Clock signal
    input wire rstn,            // Synchronous active-low reset (negedge triggered)
    input wire [31:0] d1,       // 32-bit data input 1
    input wire [31:0] d2,       // 32-bit data input 2
    output reg [31:0] q1,       // 32-bit data output 1
    output reg [31:0] q2        // 32-bit data output 2
);

    always @(posedge clk) begin
        if (!rstn) begin
            q1 <= 32'b0;
            q2 <= 32'b0;
        end else begin
            q1 <= d1;
            q2 <= d2;
        end
    end

endmodule