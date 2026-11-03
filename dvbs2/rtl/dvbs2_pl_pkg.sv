// PL framing helpers shared by the TX framer and the RX.
package dvbs2_pl_pkg;

  localparam logic [25:0] SOF      = 26'h18D2E82;
  localparam logic [63:0] PLSC_SCR = 64'h719D83C953422DFA;

  // 90 PLHEADER bits, bit 89 = first symbol: SOF + RM(64,7)-coded PLS code,
  // scrambled. Symbol k is pi/2-BPSK: even k (1+j)(1-2y)/sqrt2, odd k
  // (-1+j)(1-2y)/sqrt2.
  function automatic logic [89:0] plheader(input logic [4:0] modcod, input logic pilots);
    logic [31:0] c;
    logic [63:0] y;
    logic [5:0]  b;
    logic [31:0] g [6];
    g[0] = 32'h55555555; g[1] = 32'h33333333; g[2] = 32'h0F0F0F0F;
    g[3] = 32'h00FF00FF; g[4] = 32'h0000FFFF; g[5] = 32'hFFFFFFFF;
    b = {modcod, 1'b0};          // b[0] = FECFRAME size (normal)
    c = '0;
    for (int k = 0; k < 6; k++)
      if (b[5 - k]) c ^= g[k];
    for (int i = 0; i < 32; i++) begin
      y[63 - 2 * i] = c[31 - i];
      y[62 - 2 * i] = c[31 - i] ^ pilots;
    end
    plheader = {SOF, y ^ PLSC_SCR};
  endfunction

endpackage
