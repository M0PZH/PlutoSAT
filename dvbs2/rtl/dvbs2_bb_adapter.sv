// DMA packet -> BBFRAME byte stream for dvbs2_tx.
//
// One AXI-Stream packet (one DMA transfer) per BBFRAME, 32-bit words, first
// byte in bits [7:0]:
//   word 0      : header {8'hB5, 16'h0, 3'b0, pilots, rate_idx[3:0]}
//   words 1..   : the Kbch/8 BBFRAME bytes, zero padded to a word boundary,
//                 plus at most one more padding word (64-bit DMA alignment)
// A packet with a bad header is dropped. A packet that ends early is padded
// with zeros to Kbch/8 bytes so the encoder never loses frame alignment;
// further extra words are discarded. Both count as errors.

module dvbs2_bb_adapter
  import dvbs2_tables_pkg::*;
(
  input  logic        clk,
  input  logic        rst,

  input  logic        s_valid,
  output logic        s_ready,
  input  logic [31:0] s_data,
  input  logic        s_last,

  output logic        m_valid,
  input  logic        m_ready,
  output logic [7:0]  m_data,
  output logic [4:0]  m_user,
  output logic        m_last,

  output logic        err_hdr,    // pulse: bad header, packet dropped
  output logic        err_len     // pulse: packet length != Kbch/8 bytes
);

  typedef enum logic [2:0] {ST_HDR, ST_DATA, ST_PAD, ST_DROP, ST_TAIL} state_t;
  state_t state;

  logic [31:0] word;
  logic [1:0]  bidx;
  logic        word_valid, word_last;
  logic [12:0] nbytes_m1, bcnt;
  logic [4:0]  cfg;
  logic        tail_extra;   // one padding word already seen

  wire hdr_ok = (s_data[31:24] == 8'hB5) && (s_data[3:0] < 4'(NUM_RATES));
  wire out_fire = m_valid && m_ready;
  wire frame_end = out_fire && (bcnt == nbytes_m1);

  assign m_user = cfg;
  assign m_last = (bcnt == nbytes_m1);
  assign m_data = (state == ST_PAD) ? 8'h00 : word[8 * bidx +: 8];
  assign m_valid = (state == ST_PAD) || (state == ST_DATA && word_valid);

  // take a new word in ST_HDR / ST_DROP, or when the current one is used up
  assign s_ready = (state == ST_HDR) || (state == ST_DROP) || (state == ST_TAIL) ||
                   (state == ST_DATA && (!word_valid ||
                                         (out_fire && bidx == 2'd3 && !frame_end && !word_last)));

  always_ff @(posedge clk) begin
    err_hdr <= 1'b0;
    err_len <= 1'b0;
    if (rst) begin
      state <= ST_HDR;
      word_valid <= 1'b0;
    end else begin
      case (state)
        ST_HDR: if (s_valid) begin
          if (hdr_ok && !s_last) begin
            cfg       <= {s_data[4], s_data[3:0]};
            nbytes_m1 <= 13'(KBCH[s_data[3:0]] / 8 - 1);
            bcnt      <= '0;
            word_valid <= 1'b0;
            state     <= ST_DATA;
          end else begin
            err_hdr <= 1'b1;
            if (!s_last) state <= ST_DROP;
          end
        end

        ST_DROP: if (s_valid && s_last) state <= ST_HDR;

        ST_DATA: begin
          if (out_fire) begin
            bidx <= bidx + 2'd1;
            bcnt <= bcnt + 13'd1;
            if (bidx == 2'd3) word_valid <= 1'b0;
            if (frame_end) begin
              word_valid <= 1'b0;
              // the packet may carry one more (alignment) word
              tail_extra <= 1'b0;
              state <= word_last ? ST_HDR : ST_TAIL;
            end
          end
          if (s_valid && s_ready) begin
            word       <= s_data;
            word_last  <= s_last;
            word_valid <= 1'b1;
            bidx       <= '0;
          end
          // packet ended before the frame: pad with zeros
          if (word_valid && word_last && out_fire && bidx == 2'd3 && !frame_end) begin
            err_len <= 1'b1;
            state   <= ST_PAD;
          end
        end

        ST_TAIL: if (s_valid) begin
          tail_extra <= 1'b1;
          if (s_last) begin
            err_len <= tail_extra;      // more than one extra word
            state   <= ST_HDR;
          end
        end

        ST_PAD: if (out_fire) begin
          bcnt <= bcnt + 13'd1;
          if (frame_end) state <= ST_HDR;
        end

        default: state <= ST_HDR;
      endcase
    end
  end

endmodule
