//==========================================================================
// RadixBooth : signed 32x32 -> 64 sequential radix-4 (modified Booth)
//              multiplier.  16 iterations, 17 clocks from start to ready.
//
// Handshake
//   - reset is SYNCHRONOUS and ACTIVE LOW.
//   - Pulse `start` high for exactly one clock with `en` high.  inputA /
//     inputB are sampled on that same edge (no set-up cycle needed).
//   - `busy` goes high on the start edge and low again when the result
//     is latched.  `ready` and `result` update on the SAME clock edge,
//     so `result` is valid on the first cycle `ready` is high, and it
//     holds until the next `start`.
//   - `en` low stalls the whole machine (state is held, not lost).
//
// Both operands are interpreted as two's-complement signed.
//==========================================================================
module RadixBooth (inputA, inputB, result, clk, reset, en, start, ready, busy, start_posedge);

    input      [31:0] inputA;   // multiplicand (signed)
    input      [31:0] inputB;   // multiplier   (signed)
    input             clk;
    input             reset;    // synchronous, active low
    input             en;       // global enable / stall
    input             start;    // one-cycle pulse
    output reg [63:0] result;   // signed product, valid when ready
    output reg        ready;
    output reg        busy;
  	output reg        start_posedge;

    reg [31:0] M;    // multiplicand
    reg [32:0] Q;    // {multiplier, implicit b(-1) = 0}
    reg [63:0] Acc;  // accumulating sum
    reg [5:0]  i;    // UNSIGNED iteration counter, 0,2,...,32
  	reg start_prev;

    //----------------------------------------------------------------------
    // The four Booth multiples, built at full 64-bit width.
    //
    // Negating here (rather than taking a 32-bit two's complement and then
    // sign-extending) is what makes M = 0x80000000 work: -(-2^31) needs 33
    // bits and does not fit back into 32.  Everything is exact mod 2^64,
    // and the true product fits in 64 bits, so the final sum is exact.
    //----------------------------------------------------------------------
    wire [63:0] ext_M   = {{32{M[31]}}, M};
    wire [63:0] ext_2M  =  ext_M << 1;
    wire [63:0] ext_nM  = -ext_M;
    wire [63:0] ext_n2M = -ext_2M;

    // Booth triple (b[i+1], b[i], b[i-1]); guarded so the final state never
    // indexes past the end of Q.
    wire [2:0] code = (i < 6'd32) ? {Q[i+2], Q[i+1], Q[i]} : 3'b000;

    reg [63:0] sel;
    always @* begin
        case (code)
            3'b001, 3'b010: sel = ext_M;    // +1 * M
            3'b011:         sel = ext_2M;   // +2 * M
            3'b100:         sel = ext_n2M;  // -2 * M
            3'b101, 3'b110: sel = ext_nM;   // -1 * M
            default:        sel = 64'd0;    // 000, 111
        endcase
    end

    wire [63:0] partialProd = sel << i;

    //----------------------------------------------------------------------
    // Control / datapath
    //----------------------------------------------------------------------
    always @(posedge clk) begin
        if (!reset) begin
            M      <= 32'd0;
            Q      <= 33'd0;
            Acc    <= 64'd0;
            i      <=  6'd0;
            result <= 64'd0;
            ready  <=  1'b0;
            busy   <=  1'b0;
          	start_prev <= 1'b0;
        end
        else if (en) begin
          start_prev <= start;
          if (start && !start_prev) begin
                M     <= inputA;
                Q     <= {inputB, 1'b0};
                Acc   <= 64'd0;
                i     <=  6'd0;
                ready <=  1'b0;      // clear stale ready from last op
                busy  <=  1'b1;
            end
            else if (busy) begin
                if (i == 6'd32) begin
                    result <= Acc;   // result and ready land on the same edge
                    ready  <= 1'b1;
                    busy   <= 1'b0;
                end
                else begin
                    Acc <= Acc + partialProd;
                    i   <= i + 6'd2;
                end
            end
        end
    end
  always @(*) begin
    start_posedge = (start && !start_prev) ? 1:0;
  end

endmodule