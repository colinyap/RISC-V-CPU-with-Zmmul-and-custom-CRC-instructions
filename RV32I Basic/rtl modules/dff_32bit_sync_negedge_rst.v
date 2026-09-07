module dff_32bit_sync_negedge_rst (
    input wire clk,              // Clock signal
    input wire rstn,             // Synchronous active-low reset
    input wire en,               // Enable signal
    input wire [31:0] d,         // 32-bit data input
    output reg [31:0] q          // 32-bit data output
);

    // Trigger on the positive edge of the clock
    always @(posedge clk) begin
        if (!rstn) begin
            q <= 32'b0;          // Synchronous active-low reset
        end else if (en) begin
            q <= d;              // Load input d when enabled
        end
        // If en is low and not resetting, q holds its current value automatically
    end

endmodule