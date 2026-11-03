// DVB-S2 BCH outer encoder, normal FECFRAME, bit serial (EN 302 307-1 5.3.1).
//
// Passes Kbch info bits through while dividing by g(x), then appends the
// 16*t parity bits. The parity LFSR is 192 bits wide and left aligned, so
// t = 12/10/8 share one register: the polynomial is shifted up for t < 12
// and the unused low bits stay zero.

module dvbs2_bch_enc
  import dvbs2_tables_pkg::*;
(
  input  logic       clk,
  input  logic       rst,

  input  logic       s_valid,
  output logic       s_ready,
  input  logic       s_data,
  input  logic [4:0] s_user,
  input  logic       s_last,   // last info bit (Kbch-1)

  output logic       m_valid,
  input  logic       m_ready,
  output logic       m_data,
  output logic [4:0] m_user,
  output logic       m_last    // last BCH codeword bit (Nbch-1)
);

  logic [191:0] lfsr;
  logic         parity_phase;
  logic [7:0]   par_cnt;
  logic [7:0]   par_len_m1;
  logic [4:0]   cfg;
  logic         first;        // next info bit starts a frame

  function automatic logic [191:0] poly_aligned(input logic [3:0] rate);
    case (BCH_T[rate])
      8:       poly_aligned = {BCH_POLY_T8, 64'd0};
      10:      poly_aligned = {BCH_POLY_T10, 32'd0};
      default: poly_aligned = BCH_POLY_T12;
    endcase
  endfunction

  logic [191:0] poly;
  wire [3:0] rate_now = first ? s_user[3:0] : cfg[3:0];

  assign s_ready = !parity_phase && m_ready;
  assign m_valid = parity_phase || s_valid;
  assign m_data  = parity_phase ? lfsr[191] : s_data;
  assign m_user  = (parity_phase || !first) ? cfg : s_user;
  assign m_last  = parity_phase && (par_cnt == par_len_m1);

  always_comb poly = poly_aligned(rate_now);

  always_ff @(posedge clk) begin
    if (rst) begin
      lfsr         <= '0;
      parity_phase <= 1'b0;
      first        <= 1'b1;
      par_cnt      <= '0;
    end else if (!parity_phase) begin
      if (s_valid && s_ready) begin
        if (first) begin
          cfg        <= s_user;
          par_len_m1 <= 8'(16 * BCH_T[s_user[3:0]] - 1);
          first      <= 1'b0;
        end
        // divide; the register is all-zero at every frame start because
        // shifting out the parity bits empties it
        lfsr <= {lfsr[190:0], 1'b0} ^ ((lfsr[191] ^ s_data) ? poly : '0);
        if (s_last) begin
          parity_phase <= 1'b1;
          par_cnt      <= '0;
        end
      end
    end else if (m_ready) begin
      lfsr    <= {lfsr[190:0], 1'b0};
      par_cnt <= par_cnt + 8'd1;
      if (m_last) begin
        parity_phase <= 1'b0;
        first        <= 1'b1;
      end
    end
  end

endmodule
