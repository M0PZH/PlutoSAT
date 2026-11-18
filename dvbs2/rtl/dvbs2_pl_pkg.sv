// PL framing helpers shared by the TX framer and the RX.
package dvbs2_pl_pkg;

  localparam logic [25:0] SOF      = 26'h18D2E82;
  localparam logic [63:0] PLSC_SCR = 64'h719D83C953422DFA;

  // 90 PLHEADER bits, bit 89 = first symbol: SOF + RM(64,7)-coded PLS code,
  // scrambled. Symbol k is pi/2-BPSK: even k (1+j)(1-2y)/sqrt2, odd k
  // (-1+j)(1-2y)/sqrt2.
  function automatic logic [89:0] plheader(input logic [4:0] modcod, input logic pilots,
                                           input logic short_fr = 1'b0);
    logic [31:0] c;
    logic [63:0] y;
    logic [5:0]  b;
    logic [31:0] g [6];
    g[0] = 32'h55555555; g[1] = 32'h33333333; g[2] = 32'h0F0F0F0F;
    g[3] = 32'h00FF00FF; g[4] = 32'h0000FFFF; g[5] = 32'hFFFFFFFF;
    b = {modcod, short_fr};      // b[0] = FECFRAME size (1: short)
    c = '0;
    for (int k = 0; k < 6; k++)
      if (b[5 - k]) c ^= g[k];
    for (int i = 0; i < 32; i++) begin
      y[63 - 2 * i] = c[31 - i];
      y[62 - 2 * i] = c[31 - i] ^ pilots;
    end
    plheader = {SOF, y ^ PLSC_SCR};
  endfunction

  // MODCOD 12..17: 8PSK
  function automatic logic is_psk8(input logic [4:0] modcod);
    is_psk8 = modcod >= 5'd12;
  endfunction

  // 90-symbol slots of a data PLFRAME: QPSK 360 / 90 (short), 8PSK 240 / 60
  function automatic logic [8:0] plslots(input logic [4:0] modcod, input logic short_fr);
    case ({is_psk8(modcod), short_fr})
      2'b00:   plslots = 9'd360;
      2'b01:   plslots = 9'd90;
      2'b10:   plslots = 9'd240;
      default: plslots = 9'd60;
    endcase
  endfunction

  // symbols of a data PLFRAME: header, slots, a pilot block after every 16 slots
  function automatic logic [15:0] plframe_len(input logic [4:0] modcod, input logic pilots,
                                              input logic short_fr);
    logic [8:0] s;
    s = plslots(modcod, short_fr);
    plframe_len = 16'd90 + 16'(s) * 16'd90 + (pilots ? 16'(((s - 9'd1) >> 4) * 9'd36) : 16'd0);
  endfunction

  // LLR words of a data frame (QPSK 2, 8PSK 3 LLRs a word): Nldpc / 2 or / 3
  function automatic logic [14:0] llr_words(input logic [4:0] modcod, input logic short_fr);
    llr_words = 15'(plslots(modcod, short_fr)) * 15'd90;
  endfunction

endpackage
