//==========================================================================
// testbench.v -- minimal self-checking testbench for the optimised RV32 SoC.
//
// Targets the `top` built from RV32_single_optimised + address_decoder_optimised
// + RISCV_DMEM_OPTIMISED + RISCV_IMEM_OPTIMISED + MUX2_32.
//
// Runs the pre-synthesis firmware image in RISCV_IMEM_OPTIMISED (the `else`
// branch) -- do NOT define SYNTHESIS when compiling this.
//
// The firmware ends in one of two self-loops:
//     0x004003E8  addi x4, x0,  0    ; PASS marker
//     0x004003EC  jal  x0, 0         ; park here
//     0x004003F0  addi x4, x0, -1    ; FAIL marker
//     0x004003F4  jal  x0, 0         ; park here
//
//   iverilog -g2001 -o sim testbench.v hdl.v && ./sim
//==========================================================================
`timescale 1ns / 1ps

module testbench;

    localparam [31:0] PASS_LOOP = 32'h004003EC;
    localparam [31:0] FAIL_LOOP = 32'h004003F4;
    localparam integer TIMEOUT_CYCLES = 200000;

    reg clk;
    reg rst_n;

    // top-level observation ports
    wire        syscall;
    wire        illegal;
    wire [31:0] debug_pc;
    wire [31:0] Address;

    integer cycles;
    integer syscall_count;
    integer illegal_count;
    reg     done;

    top dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .syscall  (syscall),
        .illegal  (illegal),
        .debug_pc (debug_pc),
        .Address  (Address)
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
        rst_n         = 1'b0;
        cycles        = 0;
        syscall_count = 0;
        illegal_count = 0;
        done          = 1'b0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        $display("[%0t] reset released", $time);
    end

    // Count status pulses. syscall is expected (the firmware executes
    // ECALL/EBREAK); illegal firing means a decode problem.
    always @(posedge clk) begin
        if (rst_n && !done) begin
            if (syscall) syscall_count <= syscall_count + 1;
            if (illegal) begin
                illegal_count <= illegal_count + 1;
                $display("[%0t] ILLEGAL at pc=0x%08h instr=0x%08h",
                         $time, debug_pc, dut.blk4224_79.instruction);
            end
        end
    end

    // Watch the PC for the end-of-program self-loop.
    always @(posedge clk) begin
        if (rst_n) begin
            cycles <= cycles + 1;

            if (!done && debug_pc == PASS_LOOP) begin
                done <= 1'b1;
                report(1'b1);
            end
            else if (!done && debug_pc == FAIL_LOOP) begin
                done <= 1'b1;
                report(1'b0);
            end
            else if (cycles > TIMEOUT_CYCLES) begin
                $display("");
                $display("==================================================");
                $display("  TIMEOUT after %0d cycles -- PC never parked", cycles);
                $display("  last PC    = 0x%08h", debug_pc);
                $display("  last instr = 0x%08h", dut.blk4224_79.instruction);
                $display("  state      = %0d", dut.blk4224_79.state);
                $display("  Address    = 0x%08h", Address);
                $display("  mul_busy   = %b  mul_ready = %b  mul_i = %0d",
                         dut.blk4224_79.mul_busy,
                         dut.blk4224_79.mul_ready,
                         dut.blk4224_79.mul_i);
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
            if (passed && dut.blk4224_79.registers[4] == 32'h0000_0000)
                $display("  RESULT: PASS   (x4 = 0x%08h)",
                         dut.blk4224_79.registers[4]);
            else if (passed)
                $display("  RESULT: PASS loop reached but x4 = 0x%08h (expected 0)",
                         dut.blk4224_79.registers[4]);
            else
                $display("  RESULT: FAIL   (x4 = 0x%08h, PC parked at 0x%08h)",
                         dut.blk4224_79.registers[4], debug_pc);
            $display("  cycles run : %0d", cycles);
            $display("  syscalls   : %0d", syscall_count);
            $display("  illegals   : %0d", illegal_count);
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
                         dut.blk4224_79.registers[r]);
        end
    endtask

endmodule