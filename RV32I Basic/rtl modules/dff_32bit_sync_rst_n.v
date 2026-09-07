module dff_32bit_sync_rst_n (
    input wire clk,          // Clock signal
    input wire rstn,         // Synchronous active-low reset
    input wire [31:0] d,     // 32-bit data input
    output reg [31:0] q,     // 32-bit data output
    input wire en            // Enable signal added to the end
);

    // Trigger ONLY on the positive edge of the clock
    always @(posedge clk) begin
        if (!rstn) begin
            q <= 32'b0;      // Reset output to 0 when rstn is low at the clock edge
        end else if (en) begin
            q <= d;          // Pass input to output when enabled
        end
    end

endmodule