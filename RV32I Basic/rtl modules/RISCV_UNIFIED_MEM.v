//  ---------- INCLUDED BLOCK: RISCV_UNIFIED_MEM ----------
//  4 KB unified instruction/data SRAM, pre-loaded with a firmware image that
//  executes all 37 RV32I instructions once, plus FENCE / ECALL / EBREAK.
//
//  Memory map:  0x000 - 0x0DF   program (56 words)
//               0x400 - 0x40F   data scratch (written by SW/SB/SH)
//
//  Program ends in a JAL x0,0 self-loop at 0x0DC; the testbench uses that
//  as the "done" marker.
//
//  NOTE: the initial block is simulation-only firmware. Strip it (or replace
//  it with a $readmemh / real ROM macro) before synthesis.

module RISCV_UNIFIED_MEM (
    input [31:0] i_Data,
    input [31:0] i_Address,
    input i_Write_Enable,
    input i_Clk,
    output [31:0] o_Data,

    // New input added below existing ports
    input [3:0] i_Byte_Enable
);

    /* 4KB SRAM
       1024 words x 32 bits = 4096 bytes
    */
  reg [31:0] r_Contents [0:63];


    /* ------------------------------------------------------------------
       FIRMWARE  -  every RV32I instruction, executed once, in order.
       Register plan:  x5 = 5      x6 = -3     x28 = 0x400 (data base)
                       x31 = fail flag (must stay 0)
       ------------------------------------------------------------------ */
    integer w;
    initial begin
        for (w = 0; w < 1024; w = w + 1)
            r_Contents[w] = 32'h00000000;

        r_Contents[10'd0  ] = 32'h00500293;  // 0000h  ADDI  x5,x0,5
        r_Contents[10'd1  ] = 32'hFFD00313;  // 0004h  ADDI  x6,x0,-3
        r_Contents[10'd2  ] = 32'h40000E13;  // 0008h  ADDI  x28,x0,0x400
        r_Contents[10'd3  ] = 32'h00000F93;  // 000Ch  ADDI  x31,x0,0        (clear fail flag)
        r_Contents[10'd4  ] = 32'h006283B3;  // 0010h  ADD   x7,x5,x6
        r_Contents[10'd5  ] = 32'h40628433;  // 0014h  SUB   x8,x5,x6
        r_Contents[10'd6  ] = 32'h0062F4B3;  // 0018h  AND   x9,x5,x6
        r_Contents[10'd7  ] = 32'h0062E533;  // 001Ch  OR    x10,x5,x6
        r_Contents[10'd8  ] = 32'h0062C5B3;  // 0020h  XOR   x11,x5,x6
        r_Contents[10'd9  ] = 32'h00529633;  // 0024h  SLL   x12,x5,x5
        r_Contents[10'd10 ] = 32'h005356B3;  // 0028h  SRL   x13,x6,x5
        r_Contents[10'd11 ] = 32'h40535733;  // 002Ch  SRA   x14,x6,x5
        r_Contents[10'd12 ] = 32'h005327B3;  // 0030h  SLT   x15,x6,x5
        r_Contents[10'd13 ] = 32'h00533833;  // 0034h  SLTU  x16,x6,x5
        r_Contents[10'd14 ] = 32'h00032893;  // 0038h  SLTI  x17,x6,0
        r_Contents[10'd15 ] = 32'h00133913;  // 003Ch  SLTIU x18,x6,1
        r_Contents[10'd16 ] = 32'hFFF2C993;  // 0040h  XORI  x19,x5,-1
        r_Contents[10'd17 ] = 32'h0082EA13;  // 0044h  ORI   x20,x5,8
        r_Contents[10'd18 ] = 32'h00C37A93;  // 0048h  ANDI  x21,x6,12
        r_Contents[10'd19 ] = 32'h00329B13;  // 004Ch  SLLI  x22,x5,3
        r_Contents[10'd20 ] = 32'h00435B93;  // 0050h  SRLI  x23,x6,4
        r_Contents[10'd21 ] = 32'h40435C13;  // 0054h  SRAI  x24,x6,4
        r_Contents[10'd22 ] = 32'h12345CB7;  // 0058h  LUI   x25,0x12345
        r_Contents[10'd23 ] = 32'h678C8C93;  // 005Ch  ADDI  x25,x25,0x678   -> 0x12345678
        r_Contents[10'd24 ] = 32'h00001D17;  // 0060h  AUIPC x26,0x1
        r_Contents[10'd25 ] = 32'h019E2023;  // 0064h  SW    x25,0(x28)
        r_Contents[10'd26 ] = 32'h000E2D83;  // 0068h  LW    x27,0(x28)
        r_Contents[10'd27 ] = 32'h006E2223;  // 006Ch  SW    x6,4(x28)
        r_Contents[10'd28 ] = 32'h004E0203;  // 0070h  LB    x4,4(x28)
        r_Contents[10'd29 ] = 32'h004E4E83;  // 0074h  LBU   x29,4(x28)
        r_Contents[10'd30 ] = 32'h004E1F03;  // 0078h  LH    x30,4(x28)
        r_Contents[10'd31 ] = 32'h004E5183;  // 007Ch  LHU   x3,4(x28)
        r_Contents[10'd32 ] = 32'h005E0423;  // 0080h  SB    x5,8(x28)
        r_Contents[10'd33 ] = 32'h019E1623;  // 0084h  SH    x25,12(x28)
        r_Contents[10'd34 ] = 32'h00528463;  // 0088h  BEQ   x5,x5,+8        (taken)
        r_Contents[10'd35 ] = 32'h001F8F93;  // 008Ch    ADDI x31,x31,1      (skipped)
        r_Contents[10'd36 ] = 32'h00629463;  // 0090h  BNE   x5,x6,+8        (taken)
        r_Contents[10'd37 ] = 32'h001F8F93;  // 0094h    ADDI x31,x31,1      (skipped)
        r_Contents[10'd38 ] = 32'h00534463;  // 0098h  BLT   x6,x5,+8        (taken)
        r_Contents[10'd39 ] = 32'h001F8F93;  // 009Ch    ADDI x31,x31,1      (skipped)
        r_Contents[10'd40 ] = 32'h0062D463;  // 00A0h  BGE   x5,x6,+8        (taken)
        r_Contents[10'd41 ] = 32'h001F8F93;  // 00A4h    ADDI x31,x31,1      (skipped)
        r_Contents[10'd42 ] = 32'h0062E463;  // 00A8h  BLTU  x5,x6,+8        (taken)
        r_Contents[10'd43 ] = 32'h001F8F93;  // 00ACh    ADDI x31,x31,1      (skipped)
        r_Contents[10'd44 ] = 32'h00537463;  // 00B0h  BGEU  x6,x5,+8        (taken)
        r_Contents[10'd45 ] = 32'h001F8F93;  // 00B4h    ADDI x31,x31,1      (skipped)
        r_Contents[10'd46 ] = 32'h008000EF;  // 00B8h  JAL   x1,+8           (taken)
        r_Contents[10'd47 ] = 32'h001F8F93;  // 00BCh    ADDI x31,x31,1      (skipped)
        r_Contents[10'd48 ] = 32'h0D000167;  // 00C0h  JALR  x2,0xD0(x0)     -> word 52
        r_Contents[10'd49 ] = 32'h001F8F93;  // 00C4h    ADDI x31,x31,1      (skipped)
        r_Contents[10'd50 ] = 32'h001F8F93;  // 00C8h    ADDI x31,x31,1      (skipped)
        r_Contents[10'd51 ] = 32'h001F8F93;  // 00CCh    ADDI x31,x31,1      (skipped)
        r_Contents[10'd52 ] = 32'h0000000F;  // 00D0h  FENCE                 (NOP)
        r_Contents[10'd53 ] = 32'h00000073;  // 00D4h  ECALL                 (NOP)
        r_Contents[10'd54 ] = 32'h00100073;  // 00D8h  EBREAK                (NOP)
        r_Contents[10'd55 ] = 32'h0000006F;  // 00DCh  JAL   x0,0            (self-loop = done)
    end


    /* Synchronous Write */
    always @(posedge i_Clk) begin

        if (i_Write_Enable) begin

            if (i_Byte_Enable[0])
              r_Contents[i_Address[7:2]][7:0]
                    <= i_Data[7:0];

            if (i_Byte_Enable[1])
              r_Contents[i_Address[7:2]][15:8]
                    <= i_Data[15:8];

            if (i_Byte_Enable[2])
              r_Contents[i_Address[7:2]][23:16]
                    <= i_Data[23:16];

            if (i_Byte_Enable[3])
              r_Contents[i_Address[7:2]][31:24]
                    <= i_Data[31:24];

        end
    end


    /* Asynchronous Read */
  assign o_Data = r_Contents[i_Address[7:2]];

endmodule