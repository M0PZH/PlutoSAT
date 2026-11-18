// DVB-S2 PL framing for QPSK and 8PSK, normal and short frames (EN 302 307-1 5.5).
//
//   PLHEADER  : SOF + RM(64,7)-coded PLS, pi/2-BPSK, 90 symbols
//   payload   : S slots of 90 symbols (QPSK 360 / short 90, 8PSK 240 /
//               short 60), a 36-symbol pilot block after every 16 slots
//               when pilots are on
//   dummy     : MODCOD 0 header + 36 slots of (1+j)/sqrt2, sent whenever no
//               complete FECFRAME is waiting, so the carrier never stops
//   scrambling: Gold sequence n (x states supplied by software), restarted
//               after each header
//
// Symbols are carried as a phase index p (symbol exp(j pi p / 4)): the
// header, pilots, dummy bodies and QPSK are odd p, 8PSK any p; the PL
// scrambler's rotation by j^R adds 2R.

module dvbs2_plframer
  import dvbs2_tables_pkg::*;
(
  input  logic        clk,
  input  logic        rst,
  input  logic        enable,
  input  logic [17:0] gold_xa,       // x LFSR state for Gold code n
  input  logic [17:0] gold_xb,       // x LFSR state for n + 131072

  input  logic        frame_avail,
  input  logic [6:0]  frame_cfg,     // {psk8, short, pilots, rate_idx}
  output logic        frame_pop,
  input  logic        sym_valid,
  input  logic [2:0]  sym_data,      // phase index
  output logic        sym_rd,

  output logic        m_valid,
  input  logic        m_ready,
  output logic [2:0]  m_ph,          // phase index
  output logic        m_sof,         // first header symbol
  output logic        m_dummy,       // symbol belongs to a dummy frame

  output logic        stat_data_frame,   // pulse per data PLFRAME started
  output logic        stat_dummy_frame   // pulse per dummy PLFRAME started
);

  localparam logic [25:0] SOF = 26'h18D2E82;
  localparam logic [63:0] PLSC_SCR = 64'h719D83C953422DFA;

  function automatic logic [89:0] plheader(input logic [4:0] modcod, input logic pilots,
                                           input logic short_fr);
    logic [31:0] c;
    logic [63:0] y;
    logic [5:0]  b;
    logic [31:0] g [6];
    g[0] = 32'h55555555; g[1] = 32'h33333333; g[2] = 32'h0F0F0F0F;
    g[3] = 32'h00FF00FF; g[4] = 32'h0000FFFF; g[5] = 32'hFFFFFFFF;
    b = {modcod, short_fr};      // b[5] = MSB of MODCOD ... b[0] = FECFRAME size
    c = '0;
    for (int k = 0; k < 6; k++)
      if (b[5 - k]) c ^= g[k];
    for (int i = 0; i < 32; i++) begin
      y[63 - 2 * i] = c[31 - i];
      y[62 - 2 * i] = c[31 - i] ^ pilots;
    end
    plheader = {SOF, y ^ PLSC_SCR};
  endfunction

  typedef enum logic [1:0] {ST_IDLE, ST_HDR, ST_DATA, ST_PILOT} state_t;
  state_t state;

  logic [89:0] hdr;
  logic [6:0]  hdr_i;
  logic [8:0]  slot;
  logic [6:0]  sym_i;         // symbol in slot / pilot block
  logic [8:0]  nslots_m1;
  logic        dummy, pilots;

  logic [17:0] xa, xb, ya, yb;
  wire  [1:0]  rot = {xb[0] ^ yb[0], xa[0] ^ ya[0]};

  wire out_free = !m_valid || m_ready;
  wire need_data = (state == ST_DATA) && !dummy;
  wire can_emit = out_free && (state != ST_IDLE) && (!need_data || sym_valid);

  assign sym_rd = can_emit && need_data;

  // unscrambled body symbol; pilot / dummy: (1+j)/sqrt2
  wire [2:0] bph = need_data ? sym_data : 3'd1;

  // MODCOD of a frame configuration: QPSK rate_idx + 1, 8PSK 12..17
  function automatic logic [4:0] modcod_of(input logic [6:0] c);
    if (!c[6]) modcod_of = 5'(c[3:0] + 4'd1);
    else case (c[3:0])
      4'd4:    modcod_of = 5'd12;
      4'd5:    modcod_of = 5'd13;
      4'd6:    modcod_of = 5'd14;
      4'd8:    modcod_of = 5'd15;
      4'd9:    modcod_of = 5'd16;
      default: modcod_of = 5'd17;
    endcase
  endfunction
  // slots - 1: QPSK 360 / 90 (short), 8PSK 240 / 60
  function automatic logic [8:0] nslots_m1_of(input logic [6:0] c);
    case (c[6:5])
      2'b00:   nslots_m1_of = 9'd359;
      2'b01:   nslots_m1_of = 9'd89;
      2'b10:   nslots_m1_of = 9'd239;
      default: nslots_m1_of = 9'd59;
    endcase
  endfunction

  always_ff @(posedge clk) begin
    frame_pop        <= 1'b0;
    stat_data_frame  <= 1'b0;
    stat_dummy_frame <= 1'b0;
    if (rst) begin
      state   <= ST_IDLE;
      m_valid <= 1'b0;
    end else begin
      if (m_valid && m_ready) m_valid <= 1'b0;

      case (state)
        ST_IDLE: begin
          // decide the next frame: data if a full FECFRAME is waiting
          if (enable && !frame_pop) begin
            if (frame_avail) begin
              dummy     <= 1'b0;
              pilots    <= frame_cfg[4];
              hdr       <= plheader(modcod_of(frame_cfg), frame_cfg[4], frame_cfg[5]);
              nslots_m1 <= nslots_m1_of(frame_cfg);
              frame_pop <= 1'b1;
              stat_data_frame <= 1'b1;
            end else begin
              dummy     <= 1'b1;
              pilots    <= 1'b0;
              hdr       <= plheader(5'd0, 1'b0, 1'b0);
              nslots_m1 <= 9'd35;
              stat_dummy_frame <= 1'b1;
            end
            hdr_i <= '0;
            state <= ST_HDR;
          end
        end

        ST_HDR: if (can_emit) begin
          // pi/2-BPSK: even symbols (1+j)(1-2y)/sqrt2 (p = 1, 5), odd
          // (-1+j)(1-2y)/sqrt2 (p = 3, 7)
          m_valid <= 1'b1;
          m_ph    <= {hdr[89], hdr_i[0], 1'b1};
          m_sof   <= (hdr_i == 0);
          m_dummy <= dummy;
          hdr     <= {hdr[88:0], 1'b0};
          hdr_i   <= hdr_i + 7'd1;
          if (hdr_i == 7'd89) begin
            state <= ST_DATA;
            slot  <= '0;
            sym_i <= '0;
            xa <= gold_xa; xb <= gold_xb; ya <= PLS_YA_INIT; yb <= PLS_YB_INIT;
          end
        end

        ST_DATA, ST_PILOT: if (can_emit) begin
          m_valid <= 1'b1;
          m_sof   <= 1'b0;
          m_dummy <= dummy;
          m_ph    <= bph + {rot, 1'b0};    // multiply by j^rot
          xa <= {xa[0] ^ xa[7], xa[17:1]};
          xb <= {xb[0] ^ xb[7], xb[17:1]};
          ya <= {ya[0] ^ ya[5] ^ ya[7] ^ ya[10], ya[17:1]};
          yb <= {yb[0] ^ yb[5] ^ yb[7] ^ yb[10], yb[17:1]};

          sym_i <= sym_i + 7'd1;
          if (state == ST_DATA && sym_i == 7'd89) begin
            sym_i <= '0;
            slot  <= slot + 9'd1;
            if (slot == nslots_m1)
              state <= ST_IDLE;
            else if (pilots && slot[3:0] == 4'd15)
              state <= ST_PILOT;
          end else if (state == ST_PILOT && sym_i == 7'd35) begin
            sym_i <= '0;
            state <= ST_DATA;
          end
        end

        default: state <= ST_IDLE;
      endcase
    end
  end

endmodule
