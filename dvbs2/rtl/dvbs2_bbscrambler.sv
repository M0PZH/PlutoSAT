// BBFRAME byte input -> bit serialiser + BB scrambler (EN 302 307-1 5.2.2).
//
// Input: one BBFRAME per packet, Kbch/8 bytes, MSB of each byte first.
// s_user carries the frame configuration {pilots, rate_idx[3:0]}; it is
// sampled on the first byte of each frame and must be constant per frame.
// s_last is only checked: the frame length is taken from the rate.
//
// Output: one bit per beat, m_user = frame config, m_last on bit Kbch-1.

module dvbs2_bbscrambler
  import dvbs2_tables_pkg::*;
(
  input  logic       clk,
  input  logic       rst,

  input  logic       s_valid,
  output logic       s_ready,
  input  logic [7:0] s_data,
  input  logic [4:0] s_user,
  input  logic       s_last,

  output logic       m_valid,
  input  logic       m_ready,
  output logic       m_data,
  output logic [4:0] m_user,
  output logic       m_last,

  output logic       err_len    // pulse: s_last not on the frame's last byte
);

  localparam logic [14:0] PRBS_INIT = 15'b100101010000000;

  logic [7:0]  sreg;
  logic [2:0]  bit_idx;
  logic        have_byte;
  logic [15:0] bit_cnt;       // bit index within the BBFRAME
  logic [15:0] kbch_m1;
  logic [12:0] byte_cnt;      // bytes accepted in this frame
  logic [12:0] nbytes_m1;
  logic [14:0] prbs;
  logic        in_frame;
  logic [4:0]  cfg;

  wire prbs_bit = prbs[1] ^ prbs[0];
  wire out_fire = m_valid && m_ready;
  wire last_bit_of_byte = (bit_idx == 3'd7);

  assign m_valid = have_byte;
  assign m_data  = sreg[7] ^ prbs_bit;
  assign m_user  = cfg;
  assign m_last  = (bit_cnt == kbch_m1);

  // a new byte can be taken when the register is empty or its last bit leaves now
  assign s_ready = !have_byte || (out_fire && last_bit_of_byte);

  always_ff @(posedge clk) begin
    err_len <= 1'b0;
    if (rst) begin
      have_byte <= 1'b0;
      in_frame  <= 1'b0;
      bit_idx   <= '0;
      bit_cnt   <= '0;
      prbs      <= PRBS_INIT;
    end else begin
      if (out_fire) begin
        sreg    <= {sreg[6:0], 1'b0};
        bit_idx <= bit_idx + 3'd1;
        prbs    <= {prbs_bit, prbs[14:1]};
        if (m_last) begin
          bit_cnt  <= '0;
          in_frame <= 1'b0;
          prbs     <= PRBS_INIT;
        end else begin
          bit_cnt <= bit_cnt + 16'd1;
        end
        if (last_bit_of_byte)
          have_byte <= 1'b0;
      end

      if (s_valid && s_ready) begin
        sreg      <= s_data;
        bit_idx   <= '0;
        have_byte <= 1'b1;
        // first byte of a frame: either nothing in progress, or the byte
        // completing the previous frame is leaving right now
        if (!in_frame || (out_fire && m_last)) begin
          cfg       <= s_user;
          kbch_m1   <= 16'(KBCH[s_user[3:0]] - 1);
          nbytes_m1 <= 13'(KBCH[s_user[3:0]] / 8 - 1);
          byte_cnt  <= 13'd1;
          in_frame  <= 1'b1;
          err_len   <= s_last;  // a one-byte frame is always wrong
        end else begin
          byte_cnt <= byte_cnt + 13'd1;
          err_len  <= s_last != (byte_cnt == nbytes_m1);
        end
      end
    end
  end

endmodule
