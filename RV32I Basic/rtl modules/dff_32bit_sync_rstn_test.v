module dff_32bit_sync_rstn_test (
    input wire clk,              // Clock signal
    input wire rstn,             // Synchronous active-low reset
    input wire test_mode,        // Test mode enable signal
    input wire [31:0] test_val,  // 32-bit test data input
    input wire [31:0] d,         // 32-bit normal data input
  	output reg [31:0] q,          // 32-bit data output
    input wire en               // Clock/Data enable signal

);

    // Trigger ONLY on the positive edge of the clock
    always @(posedge clk) begin
        if (!rstn) begin
            q <= 32'b0;          // Reset has the highest priority
        end else if (test_mode) begin
            q <= test_val;       // Load test value when test_mode is high
        end else if (en) begin
          q <= {d[31:1], 1'b0};              // Normal operation when enabled: pass d to q
        end
        // If not reset, not in test mode, and en is 0, q retains its previous value automatically
    end

endmodule