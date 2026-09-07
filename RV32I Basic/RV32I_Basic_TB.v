// ============================================================================
//  testbench.v  -  run the firmware in RISCV_UNIFIED_MEM to completion and
//                    check the final architectural state.
//
//  The program ends in JAL x0,0 (self-loop) at 0x0DC. The bench waits for the
//  PC to reach it, then compares the register file and the data scratch words.
//
//  iverilog -g2001 -o sim.vvp testbench.v hdl.v && vvp sim.vvp
// ============================================================================

`timescale 1ns/1ps

module testbench;

    reg        CLK = 1'b0;
    reg        rst_n = 1'b0;
    reg        Test = 1'b0;
    reg [31:0] Test_data = 32'h0;

    top dut (.CLK(CLK), .rst_n(rst_n), .Test(Test), .Test_data(Test_data));

    always #5 CLK = ~CLK;

    // hierarchical handles into the core
    // regs : dut.blk2466_23.rf[n]      mem : dut.blk2472_95.r_Contents[word]
    // PC   : dut.w_22                  (NOTE: w_18 is opcode, 7 bits;
    //                                   AdrSrc is w_26, 1 bit)

    localparam [31:0] DONE_PC = 32'h0000_00DC;   // the JAL x0,0 self-loop

    integer errors = 0;

    task chk;
        input [8*10:1] name;
        input [31:0]   got, exp;
        begin
            if (got !== exp) begin
                errors = errors + 1;
                $display("  FAIL  %0s got=%h exp=%h", name, got, exp);
            end else begin
                $display("  pass  %0s = %h", name, got);
            end
        end
    endtask

    initial begin
        $dumpfile("testbench.vcd");
        $dumpvars(0, testbench);

        rst_n = 1'b0;
        repeat (3) @(posedge CLK);
        #1 rst_n = 1'b1;

        // run until the program reaches its self-loop
        while (dut.w_22 !== DONE_PC) @(posedge CLK);
        repeat (8) @(posedge CLK);
        #1;

        $display("\nFirmware reached the done self-loop (PC=%h) at t=%0t\n",
                 dut.w_22, $time);

        $display("-- ALU results --");
        chk("x5  ADDI ", dut.blk2466_23.rf[5],  32'h0000_0005);
        chk("x6  ADDI ", dut.blk2466_23.rf[6],  32'hFFFF_FFFD);
        chk("x7  ADD  ", dut.blk2466_23.rf[7],  32'h0000_0002);
        chk("x8  SUB  ", dut.blk2466_23.rf[8],  32'h0000_0008);
        chk("x9  AND  ", dut.blk2466_23.rf[9],  32'h0000_0005);
        chk("x10 OR   ", dut.blk2466_23.rf[10], 32'hFFFF_FFFD);
        chk("x11 XOR  ", dut.blk2466_23.rf[11], 32'hFFFF_FFF8);
        chk("x12 SLL  ", dut.blk2466_23.rf[12], 32'h0000_00A0);
        chk("x13 SRL  ", dut.blk2466_23.rf[13], 32'h07FF_FFFF);
        chk("x14 SRA  ", dut.blk2466_23.rf[14], 32'hFFFF_FFFF);
        chk("x15 SLT  ", dut.blk2466_23.rf[15], 32'h0000_0001);
        chk("x16 SLTU ", dut.blk2466_23.rf[16], 32'h0000_0000);
        chk("x17 SLTI ", dut.blk2466_23.rf[17], 32'h0000_0001);
        chk("x18 SLTIU", dut.blk2466_23.rf[18], 32'h0000_0000);
        chk("x19 XORI ", dut.blk2466_23.rf[19], 32'hFFFF_FFFA);
        chk("x20 ORI  ", dut.blk2466_23.rf[20], 32'h0000_000D);
        chk("x21 ANDI ", dut.blk2466_23.rf[21], 32'h0000_000C);
        chk("x22 SLLI ", dut.blk2466_23.rf[22], 32'h0000_0028);
        chk("x23 SRLI ", dut.blk2466_23.rf[23], 32'h0FFF_FFFF);
        chk("x24 SRAI ", dut.blk2466_23.rf[24], 32'hFFFF_FFFF);
        chk("x25 LUI  ", dut.blk2466_23.rf[25], 32'h1234_5678);
        chk("x26 AUIPC", dut.blk2466_23.rf[26], 32'h0000_1060);

        $display("-- loads --");
        chk("x27 LW   ", dut.blk2466_23.rf[27], 32'h1234_5678);
        chk("x4  LB   ", dut.blk2466_23.rf[4],  32'hFFFF_FFFD);
        chk("x29 LBU  ", dut.blk2466_23.rf[29], 32'h0000_00FD);
        chk("x30 LH   ", dut.blk2466_23.rf[30], 32'hFFFF_FFFD);
        chk("x3  LHU  ", dut.blk2466_23.rf[3],  32'h0000_FFFD);

        // With the 64-word memory and the [7:2] address slice, x28=0x400
        // aliases to word 0, so the four stores land on words 0-3 (the first
        // four instructions, already executed and never re-fetched). The two
        // SWs overwrite whole words; SB merges into 0x40000E13 -> 0x40000E05;
        // SH merges into 0x00000F93 -> 0x00005678 (upper half already zero).
        $display("-- stores --");
        chk("m400 SW  ", dut.blk2472_95.r_Contents[6'd0], 32'h1234_5678);
        chk("m404 SW  ", dut.blk2472_95.r_Contents[6'd1], 32'hFFFF_FFFD);
        chk("m408 SB  ", dut.blk2472_95.r_Contents[6'd2], 32'h4000_0E05);
        chk("m40C SH  ", dut.blk2472_95.r_Contents[6'd3], 32'h0000_5678);

        $display("-- control flow --");
        chk("x1  JAL  ", dut.blk2466_23.rf[1],  32'h0000_00BC);  // link
        chk("x2  JALR ", dut.blk2466_23.rf[2],  32'h0000_00C4);  // link
        chk("x31 fail ", dut.blk2466_23.rf[31], 32'h0000_0000);  // 0 = every
                                                                 // branch/jump
                                                                 // was taken
        $display("\n========================================");
        if (errors == 0) $display(" RESULT: PASS - all 37 RV32I instructions OK");
        else             $display(" RESULT: FAIL - %0d mismatches", errors);
        $display("========================================");
        $finish;
    end

    initial begin
        #50000;
        $display(" RESULT: FAIL - timeout, PC=%h", dut.w_22);
        $finish;
    end

endmodule