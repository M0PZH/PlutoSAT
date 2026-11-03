// DVB-S2 PL framing for QPSK normal frames (EN 302 307-1 5.5).
//
//   PLHEADER  : SOF + RM(64,7)-coded PLS, pi/2-BPSK, 90 symbols
//   payload   : 360 slots of 90 QPSK symbols, a 36-symbol pilot block after
//               every 16 slots when pilots are on
//   dummy     : MODCOD 0 header + 36 slots of (1+j)/sqrt2, sent whenever no
//               complete FECFRAME is waiting, so the carrier never stops
//   scrambling: Gold sequence n (x states supplied by software), restarted
//               after each header
//
// Every symbol here is (+-a, +-a), so symbols are carried as two sign bits
// (1 = negative); the rotation by j^R of the PL scrambler maps that set onto
// itself.

module dvbs2_plframer
  import dvbs2_tables_pkg::*;
(
  input  logic        clk,
  input  logic        rst,
  input  logic        enable,
  input  logic [17:0] gold_xa,       // x LFSR state for Gold code n
  input  logic [17:0] gold_xb,       // x LFSR state for n + 131072

  input  logic        frame_avail,
  input  logic [4:0]  frame_cfg,     // {pilots, rate_idx}
  output logic        frame_pop,
  input  logic        sym_valid,
  input  logic [1:0]  sym_data,
  output logic        sym_rd,

  output logic        m_valid,
  input  logic        m_ready,
  output logic        m_si,          // I sign
  output logic        m_sq,          // Q sign
  output logic        m_sof,         // first header symbol
  output logic        m_dummy,       // symbol belongs to a dummy frame

  output logic        stat_data_frame,   // pulse per data PLFRAME started
  output logic        stat_dummy_frame   // pulse per dummy PLFRAME started
);

  localparam logic [25:0] SOF = 26'h18D2E82;
  localparam logic [63:0] PLSC_SCR = 64'h719D83C953422DFA;

  function automatic logic [89:0] plheader(input logic [4:0] modcod, input logic pilots);
    logic [31:0] c;
    logic [63:0] y;
    logic [5:0]  b;
    logic [31:0] g [6];
    g[0] = 32'h55555555; g[1] = 32'h33333333; g[2] = 32'h0F0F0F0F;
    g[3] = 32'h00FF00FF; g[4] = 32'h0000FFFF; g[5] = 32'hFFFFFFFF;
    b = {modcod, 1'b0};          // b[5] = MSB of MODCOD ... b[0] = FECFRAME size (normal)
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

  // unscrambled body symbol
  logic bsi, bsq;
  always_comb begin
    if (need_data) {bsi, bsq} = sym_data;
    else           {bsi, bsq} = 2'b00;   // pilot / dummy: (1+j)/sqrt2
  end

  logic [8:0] nslots_cfg;
  logic       next_pilots;
  logic [4:0] next_modcod;

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
              hdr       <= plheader(5'(frame_cfg[3:0] + 4'd1), frame_cfg[4]);
              nslots_m1 <= 9'd359;
              frame_pop <= 1'b1;
              stat_data_frame <= 1'b1;
            end else begin
              dummy     <= 1'b1;
              pilots    <= 1'b0;
              hdr       <= plheader(5'd0, 1'b0);
              nslots_m1 <= 9'd35;
              stat_dummy_frame <= 1'b1;
            end
            hdr_i <= '0;
            state <= ST_HDR;
          end
        end

        ST_HDR: if (can_emit) begin
          // pi/2-BPSK: even symbols (1+j)(1-2y)/sqrt2, odd (-1+j)(1-2y)/sqrt2
          m_valid <= 1'b1;
          m_si    <= hdr[89] ^ hdr_i[0];
          m_sq    <= hdr[89];
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
          case (rot)   // multiply by j^rot
            2'd0: begin m_si <= bsi;  m_sq <= bsq;  end
            2'd1: begin m_si <= !bsq; m_sq <= bsi;  end
            2'd2: begin m_si <= !bsi; m_sq <= !bsq; end
            2'd3: begin m_si <= bsq;  m_sq <= !bsi; end
          endcase
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
