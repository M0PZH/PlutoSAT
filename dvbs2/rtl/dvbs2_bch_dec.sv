// DVB-S2 BCH decoder, normal FECFRAMEs (model: model/bchdec.py, bit exact).
//
// Input : the Kldpc hard decisions of a frame from dvbs2_ldpc_dec (BB-
//         scrambled BBFRAME + BCH parity) as bytes, MSB first, s_last on the
//         last; s_ok / s_iters: the LDPC result; s_rate / s_tag: the frame's
//         code rate and tag (sampled with each byte; constant in a frame),
//         out again as m_rate / m_tag.
// Output: the Kbch BBFRAME bits, corrected and BB-descrambled, as bytes;
//         m_ok on the last byte: the frame is a BCH codeword now (no errors
//         or <= t corrected), m_nerr: errors located (deg Lambda), m_raw_err:
//         input bits that differ from s_chan (the channel's hard decisions,
//         i.e. pre-FEC errors if the frame is good).
//
//   IN    bytes into the buffer, odd syndromes S_1..S_2t-1 8 bits/clock
//   SQ    S_2k = S_k^2
//   BM    simplified inversionless Berlekamp-Massey, t steps (~30 clocks each)
//   CI    Chien start terms Lambda_j alpha^(-j(n-1))
//   CH    Chien search 8 positions/clock, corrections and BB descrambling
//         written back into the buffer
//   OUT   Kbch/8 bytes, one per 2 clocks
// Not ready during SQ..OUT (about n/8 + Kbch/4 + 500 clocks per frame).

module dvbs2_bch_dec
  import dvbs2_bch_pkg::*;
  import dvbs2_ldpc_dec_pkg::*;
#(
  parameter int TW = 8                     // tag width
) (
  input  logic        clk,
  input  logic        rst,

  input  logic        s_valid,
  output logic        s_ready,
  input  logic [7:0]  s_data,
  input  logic        s_last,
  input  logic        s_ok,
  input  logic [5:0]  s_iters,
  input  logic [7:0]  s_chan,
  input  logic [3:0]  s_rate,
  input  logic [TW-1:0] s_tag,

  output logic        m_valid,
  input  logic        m_ready,
  output logic [7:0]  m_data,
  output logic        m_last,
  output logic        m_ok,          // BCH: codeword
  output logic        m_ldpc_ok,     // LDPC converged
  output logic [5:0]  m_iters,
  output logic [4:0]  m_nerr,
  output logic [15:0] m_raw_err,
  output logic [3:0]  m_rate,
  output logic [TW-1:0] m_tag,

  output logic [15:0] st_frames,
  output logic [15:0] st_corrected,  // frames with errors corrected
  output logic [15:0] st_failed      // frames not decodable
);

  function automatic logic [15:0] gmul(input logic [15:0] a, input logic [15:0] b);
    logic [15:0] r;
    r = '0;
    for (int i = 15; i >= 0; i--) begin
      r = {r[14:0], 1'b0} ^ (r[15] ? BCH_PRIM[15:0] : 16'h0000);
      if (b[i]) r = r ^ a;
    end
    return r;
  endfunction

  // ------------------------------------------------------------ rate
  // the frame's rate, taken with its bytes; the syndromes do not depend on
  // it, the first use (t in B_D) is 13 clocks after the last byte
  logic [3:0]    rate_idx;
  logic [TW-1:0] tag;
  logic [15:0] nb, kb;                       // bytes: Kldpc / 8, Kbch / 8
  logic [3:0]  t;
  logic [15:0] cinit [12];
  always_ff @(posedge clk) begin
    nb <= BCH_N[rate_idx] >> 3;
    kb <= LD_KBCH[rate_idx] >> 3;
    t  <= BCH_T[rate_idx];
    for (int j = 0; j < 12; j++) cinit[j] <= BCH_CINIT[rate_idx][j];
  end

  // ------------------------------------------------------------ buffer (2 x BRAM36)
  logic        bw_en;
  logic [12:0] bw_a, br_a;
  logic [7:0]  bw_d, br_q;
  (* ram_style = "block" *) logic [7:0] buf_m [8192];
  always_ff @(posedge clk) begin
    if (bw_en) buf_m[bw_a] <= bw_d;
    br_q <= buf_m[br_a];
  end

  // ------------------------------------------------------------ state
  typedef enum logic [3:0] {B_IN, B_SQ, B_D, B_U, B_U2, B_CI, B_CH, B_CHEND, B_OUT, B_OUT2} st_t;
  st_t st;

  logic [15:0] S [1:24];
  logic [15:0] lam [14], bb [14], lnew [14];
  logic [15:0] gamma, d;
  logic [4:0]  L;
  logic [3:0]  r, j;
  logic [15:0] term [12];
  logic [15:0] k;                            // byte index
  logic        ch_v;                         // CH: byte k-1 read, its mask in ch_mask
  logic [7:0]  ch_mask;
  logic [15:0] ch_k;
  logic [14:0] prbs;
  logic [4:0]  roots;
  logic        ok, ldpc_ok;
  logic [5:0]  iters;
  logic [15:0] raw;

  assign s_ready = (st == B_IN) && !rst;

  logic [7:0]  cw;                           // CH: corrected (descrambled) byte
  logic [14:0] cpr;
  always_comb begin
    bw_en = 1'b0;
    bw_a  = 13'(k);
    bw_d  = s_data;
    br_a  = 13'(k);
    cw    = br_q ^ ch_mask;
    cpr   = prbs;
    if (st == B_IN) bw_en = s_valid;
    if (st == B_CH && ch_v) begin
      if (ch_k < kb)
        for (int i = 0; i < 8; i++) begin
          cw[7 - i] = cw[7 - i] ^ (cpr[1] ^ cpr[0]);
          cpr = {cpr[1] ^ cpr[0], cpr[14:1]};
        end
      bw_en = 1'b1;
      bw_a  = 13'(ch_k);
      bw_d  = cw;
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      st <= B_IN;
      k <= '0;
      m_valid <= 1'b0;
      st_frames <= '0; st_corrected <= '0; st_failed <= '0;
      for (int i = 1; i <= 24; i++) S[i] <= '0;
    end else begin
      if (m_valid && m_ready) m_valid <= 1'b0;

      case (st)
        // ---------------------------------------------------- bytes in, syndromes
        B_IN: if (s_valid) begin
          rate_idx <= s_rate;
          tag <= s_tag;
          for (int jj = 0; jj < 12; jj++) begin : syn
            logic [15:0] v;
            v = gmul(S[2 * jj + 1], BCH_SYN_MUL[jj][8]);
            for (int b = 0; b < 8; b++) if (s_data[b]) v = v ^ BCH_SYN_MUL[jj][b];
            S[2 * jj + 1] <= v;
          end
          k <= k + 16'd1;
          raw <= (k == 0 ? 16'd0 : raw) + 16'($countones(s_data ^ s_chan));
          if (s_last) begin
            ldpc_ok <= s_ok;
            iters <= s_iters;
            j <= 4'd1;
            st <= B_SQ;
          end
        end

        B_SQ: begin
          S[2 * j] <= gmul(S[j], S[j]);
          if (j == 4'd12) begin
            for (int i = 0; i < 14; i++) begin
              lam[i] <= (i == 0) ? 16'd1 : 16'd0;
              bb[i]  <= (i == 0) ? 16'd1 : 16'd0;
            end
            gamma <= 16'd1;
            L <= '0;
            r <= '0;
            j <= '0;
            d <= '0;
            st <= B_D;
          end else j <= j + 4'd1;
        end

        // ---------------------------------------------------- Berlekamp-Massey
        B_D: begin : bd
          logic [5:0] si;
          si = 6'(2 * r + 1) - 6'(j);
          if (si >= 6'd1 && si <= 6'd24) d <= d ^ gmul(lam[j], S[5'(si)]);
          if (j == t) begin
            j <= '0;
            st <= B_U;
          end else j <= j + 4'd1;
        end

        B_U: begin
          lnew[j] <= gmul(gamma, lam[j]) ^ (j != 0 ? gmul(d, bb[j - 4'd1]) : 16'd0);
          if (j == t + 4'd1) st <= B_U2;
          else j <= j + 4'd1;
        end

        B_U2: begin
          if (d != 0 && 5'(L) <= 5'(r)) begin
            for (int i = 0; i < 14; i++) bb[i] <= (i == 0) ? 16'd0 : lam[i - 1];
            L <= 5'(2 * r + 1) - L;
            gamma <= d;
          end else
            for (int i = 0; i < 14; i++) bb[i] <= (i < 2) ? 16'd0 : bb[i - 2];
          for (int i = 0; i < 14; i++) if (i <= int'(t) + 1) lam[i] <= lnew[i];
          d <= '0;
          j <= '0;
          if (r + 4'd1 == t) begin
            j <= 4'd1;
            st <= B_CI;
          end else begin
            r <= r + 4'd1;
            st <= B_D;
          end
        end

        // ---------------------------------------------------- Chien search
        B_CI: begin
          term[j - 4'd1] <= gmul(lam[j], cinit[j - 4'd1]);
          if (j == 4'd12) begin
            k <= '0;
            ch_v <= 1'b0;
            roots <= '0;
            prbs <= 15'b100101010000000;
            st <= B_CH;
          end else j <= j + 4'd1;
        end

        B_CH: begin
          // byte k: read it, its 8 root flags; byte k-1: written back (always_comb)
          if (ch_v) roots <= roots + 5'($countones(ch_mask));
          if (k != nb) begin : ch
            logic [7:0] mk;
            for (int i = 0; i < 8; i++) begin
              logic [15:0] v;
              v = lam[0];
              for (int jj = 0; jj < 12; jj++) v = v ^ gmul(term[jj], BCH_CH_MUL[jj][i]);
              mk[7 - i] = (v == 16'd0);
            end
            for (int jj = 0; jj < 12; jj++) term[jj] <= gmul(term[jj], BCH_CH_MUL[jj][8]);
            ch_mask <= mk;
            ch_k <= k;
            ch_v <= 1'b1;
            k <= k + 16'd1;
          end else begin
            ch_v <= 1'b0;
            st <= B_CHEND;
          end
        end

        B_CHEND: begin
          ok <= L <= 5'(t) && roots == L;
          k <= '0;
          st <= B_OUT;
        end

        // ---------------------------------------------------- output
        B_OUT: st <= B_OUT2;                 // br_q <= buffer[k]

        B_OUT2: if (!m_valid || m_ready) begin
          m_valid   <= 1'b1;
          m_data    <= br_q;
          m_last    <= (k + 16'd1 == kb);
          m_ok      <= ok;
          m_ldpc_ok <= ldpc_ok;
          m_iters   <= iters;
          m_nerr    <= L;
          m_raw_err <= raw;
          m_rate    <= rate_idx;
          m_tag     <= tag;
          if (k + 16'd1 == kb) begin
            st_frames <= st_frames + 16'd1;
            if (!ok) st_failed <= st_failed + 16'd1;
            else if (L != 0) st_corrected <= st_corrected + 16'd1;
            k <= '0;
            for (int i = 1; i <= 24; i++) S[i] <= '0;
            st <= B_IN;
          end else begin
            k <= k + 16'd1;
            st <= B_OUT;
          end
        end

        default: st <= B_IN;
      endcase

      // BB descrambler: advanced by 8 bits per info byte written back
      if (st == B_CH && ch_v && ch_k < kb) begin : adv
        logic [14:0] pr;
        pr = prbs;
        for (int i = 0; i < 8; i++) pr = {pr[1] ^ pr[0], pr[14:1]};
        prbs <= pr;
      end
    end
  end

endmodule
