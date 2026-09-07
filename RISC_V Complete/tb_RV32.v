//==========================================================================
// tb_top.v -- minimal self-checking testbench for the RV32I SoC.
//
// Runs the pre-synthesis firmware image in RISCV_iMEM (the `else` branch of
// the `ifdef SYNTHESIS -- do NOT define SYNTHESIS when compiling this).
//
// The firmware ends in one of two self-loops:
//     0x004003E8  addi x4, x0,  0    ; PASS marker
//     0x004003EC  jal  x0, 0         ; park here
//     0x004003F0  addi x4, x0, -1    ; FAIL marker
//     0x004003F4  jal  x0, 0         ; park here
//
// The TB waits for the PC to park, then reports x4 and which loop was hit.
//
//   iverilog -g2001 -o sim tb_top.v hdl.v && ./sim
//==========================================================================
`timescale 1ns / 1ps

module testbench;

    localparam [31:0] PASS_LOOP = 32'h004003EC;
    localparam [31:0] FAIL_LOOP = 32'h004003F4;
    localparam integer TIMEOUT_CYCLES = 200000;

    reg clk;
    reg rst_n;

    integer cycles;
    reg     done;

    top dut (
        .clk   (clk),
        .rst_n (rst_n)
    );

    // 100 MHz
    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial begin
      $dumpfile("testbench.vcd");
      $dumpvars(0, testbench);
    end

    // Reset: hold low across a few edges, release off the negedge so the
    // core never sees reset deassert coincident with a rising edge.
    initial begin
        rst_n  = 1'b0;
        cycles = 0;
        done   = 1'b0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        $display("[%0t] reset released", $time);
    end

    // Watch the PC for the end-of-program self-loop.
    always @(posedge clk) begin
        if (rst_n) begin
            cycles <= cycles + 1;

            if (!done && dut.blk3708_24.pc == PASS_LOOP) begin
                done <= 1'b1;
                report(1'b1);
            end
            else if (!done && dut.blk3708_24.pc == FAIL_LOOP) begin
                done <= 1'b1;
                report(1'b0);
            end
            else if (cycles > TIMEOUT_CYCLES) begin
                $display("");
                $display("==================================================");
                $display("  TIMEOUT after %0d cycles -- PC never parked", cycles);
                $display("  last PC    = 0x%08h", dut.blk3708_24.pc);
                $display("  last instr = 0x%08h", dut.blk3708_24.instruction);
                $display("  state      = %0d", dut.blk3708_24.state);
                $display("==================================================");
                $finish;
            end
        end
    end

    task report;
        input passed;
        begin
            // Let the marker instruction retire into x4 before reading it.
            repeat (8) @(posedge clk);
            $display("");
            $display("==================================================");
            if (passed && dut.blk3708_24.registers[4] == 32'h0000_0000)
                $display("  RESULT: PASS   (x4 = 0x%08h)",
                         dut.blk3708_24.registers[4]);
            else if (passed)
                $display("  RESULT: PASS loop reached but x4 = 0x%08h (expected 0)",
                         dut.blk3708_24.registers[4]);
            else
                $display("  RESULT: FAIL   (x4 = 0x%08h, PC parked at 0x%08h)",
                         dut.blk3708_24.registers[4], dut.blk3708_24.pc);
            $display("  cycles run : %0d", cycles);
            $display("==================================================");
            $display("");
            dump_regs;
            $finish;
        end
    endtask

    task dump_regs;
        integer r;
        begin
            $display("register file:");
            for (r = 0; r < 32; r = r + 1)
                $display("  x%0d%s= 0x%08h", r, (r < 10) ? "  " : " ",
                         dut.blk3708_24.registers[r]);
        end
    endtask

endmodule