// Polyphase RRC pulse shaper / integer interpolator.
//
//   y[n*SPS + p] = sum_{j<TPP} h[p + SPS*j] * x[n - j],   p = 0 .. SPS-1
//
// SPS (samples per symbol, 2..MAX_SPS) is a register, so one filter covers
// every symbol rate that divides the DAC rate (12 MS/s / 8, 12, 24, 36, 48 ...).
// Coefficients are written by software (coef_*), phase-major:
//   coef_addr = p*TPP + j  holds h[p + SPS*j]
// MACS multipliers per rail work on one output for TPP/MACS cycles.
// Output = round(sum / 2^ACC_SHIFT), saturated to 16 bit.

module dvbs2_rrc_interp #(
  parameter int TPP       = 16,      // taps per phase
  parameter int MACS      = 4,       // multipliers per rail
  parameter int MAX_SPS   = 64,
  parameter int COEF_W    = 18,
  parameter int ACC_SHIFT = 17,
  parameter int CAW       = $clog2(MAX_SPS * TPP)
) (
  input  logic               clk,
  input  logic               rst,
  input  logic [6:0]         sps,

  input  logic               coef_we,
  input  logic [CAW-1:0]     coef_addr,
  input  logic [COEF_W-1:0]  coef_data,

  input  logic               s_valid,
  output logic               s_ready,
  input  logic signed [15:0] s_i,
  input  logic signed [15:0] s_q,

  output logic               m_valid,
  input  logic               m_ready,
  output logic signed [15:0] m_i,
  output logic signed [15:0] m_q
);

  localparam int K   = TPP / MACS;           // cycles per output
  localparam int KW  = (K > 1) ? $clog2(K) : 1;
  localparam int BAW = $clog2(MAX_SPS * K);  // bank address width
  localparam int ACC_W = 16 + COEF_W + $clog2(TPP) + 1;
  localparam int OFIFO = 8;

  // ---------------------------------------------------------------- coefficient banks
  // bank l holds taps j with j % MACS == l, at address p*K + j/MACS
  logic signed [COEF_W-1:0] coef_mem [MACS][MAX_SPS * K];
  logic signed [COEF_W-1:0] coef_rd  [MACS];
  logic [BAW-1:0]           bank_raddr;

  wire [$clog2(TPP)-1:0] w_j = coef_addr[$clog2(TPP)-1:0];
  wire [CAW-1:0]         w_p = coef_addr >> $clog2(TPP);

  for (genvar l = 0; l < MACS; l++) begin : g_bank
    initial for (int a = 0; a < MAX_SPS * K; a++) coef_mem[l][a] = '0;
    always_ff @(posedge clk) begin
      if (coef_we && (w_j % MACS) == l)
        coef_mem[l][BAW'(w_p * K + w_j / MACS)] <= coef_data;
      coef_rd[l] <= coef_mem[l][bank_raddr];
    end
  end

  // ---------------------------------------------------------------- symbol history
  logic signed [15:0] hist_i [TPP];
  logic signed [15:0] hist_q [TPP];

  // ---------------------------------------------------------------- issue control
  logic [6:0]  ph;           // current phase
  logic [KW-1:0] k;          // current step within an output
  logic        have_sym;     // history holds the symbol being interpolated
  logic [3:0]  ofifo_n;      // samples in output FIFO
  logic [3:0]  inflight;     // outputs started but not yet in the FIFO

  wire start_ok = have_sym && (ofifo_n + inflight < OFIFO);
  wire issue    = have_sym && ((k != 0) || start_ok);
  wire last_k   = (k == KW'(K - 1));
  wire out_done;             // an output enters the FIFO this cycle

  assign s_ready = !have_sym;
  assign bank_raddr = BAW'(ph * K + k);

  // stage 1 (aligned with coefficient read data)
  logic               s1_v, s1_first, s1_last;
  logic signed [15:0] s1_i [MACS], s1_q [MACS];
  // stage 2: products
  logic               s2_v, s2_first, s2_last;
  logic signed [16+COEF_W-1:0] s2_pi [MACS], s2_pq [MACS];
  // stage 3: per-lane accumulators
  logic               s3_v;
  logic signed [ACC_W-1:0] acc_i [MACS], acc_q [MACS];

  always_ff @(posedge clk) begin
    if (rst) begin
      have_sym <= 1'b0;
      ph <= '0; k <= '0;
      for (int j = 0; j < TPP; j++) begin hist_i[j] <= '0; hist_q[j] <= '0; end
    end else begin
      if (s_valid && s_ready) begin
        for (int j = TPP - 1; j > 0; j--) begin
          hist_i[j] <= hist_i[j - 1];
          hist_q[j] <= hist_q[j - 1];
        end
        hist_i[0] <= s_i;
        hist_q[0] <= s_q;
        have_sym  <= 1'b1;
      end
      if (issue) begin
        k <= last_k ? '0 : k + 1'b1;
        if (last_k) begin
          if (ph == sps - 7'd1) begin
            ph <= '0;
            have_sym <= 1'b0;    // all phases issued: take the next symbol
          end else begin
            ph <= ph + 7'd1;
          end
        end
      end
    end
  end

  // ---------------------------------------------------------------- MAC pipeline
  always_ff @(posedge clk) begin
    if (rst) begin
      s1_v <= 1'b0; s2_v <= 1'b0; s3_v <= 1'b0;
    end else begin
      s1_v     <= issue;
      s1_first <= (k == 0);
      s1_last  <= last_k;
      for (int l = 0; l < MACS; l++) begin
        s1_i[l] <= hist_i[l + MACS * k];
        s1_q[l] <= hist_q[l + MACS * k];
      end

      s2_v     <= s1_v;
      s2_first <= s1_first;
      s2_last  <= s1_last;
      for (int l = 0; l < MACS; l++) begin
        s2_pi[l] <= s1_i[l] * coef_rd[l];
        s2_pq[l] <= s1_q[l] * coef_rd[l];
      end

      s3_v <= s2_v && s2_last;
      if (s2_v)
        for (int l = 0; l < MACS; l++) begin
          acc_i[l] <= (s2_first ? '0 : acc_i[l]) + ACC_W'(s2_pi[l]);
          acc_q[l] <= (s2_first ? '0 : acc_q[l]) + ACC_W'(s2_pq[l]);
        end
    end
  end

  // ---------------------------------------------------------------- lane sum, round, saturate
  function automatic logic signed [15:0] round_sat(input logic signed [ACC_W-1:0] a);
    logic signed [ACC_W-1:0] r;
    r = (a + (ACC_W'(1) <<< (ACC_SHIFT - 1))) >>> ACC_SHIFT;
    if (r > 32767)       round_sat = 16'sd32767;
    else if (r < -32768) round_sat = -16'sd32768;
    else                 round_sat = r[15:0];
  endfunction

  logic signed [ACC_W-1:0] sum_i, sum_q;
  always_comb begin
    sum_i = '0; sum_q = '0;
    for (int l = 0; l < MACS; l++) begin
      sum_i += acc_i[l];
      sum_q += acc_q[l];
    end
  end

  // stage 4: lane sum (registered); stage 5: round and saturate
  logic               s4_v, s5_v;
  logic signed [ACC_W-1:0] s4_si, s4_sq;
  logic signed [15:0] s5_i, s5_q;
  always_ff @(posedge clk) begin
    if (rst) begin
      s4_v <= 1'b0;
      s5_v <= 1'b0;
    end else begin
      s4_v  <= s3_v;
      s4_si <= sum_i;
      s4_sq <= sum_q;
      s5_v  <= s4_v;
      s5_i  <= round_sat(s4_si);
      s5_q  <= round_sat(s4_sq);
    end
  end
  assign out_done = s5_v;

  // ---------------------------------------------------------------- output FIFO
  logic signed [15:0] of_i [OFIFO], of_q [OFIFO];
  logic [2:0] of_w, of_r;
  wire  of_pop = m_valid && m_ready;

  assign m_valid = (ofifo_n != 0);
  assign m_i = of_i[of_r];
  assign m_q = of_q[of_r];

  always_ff @(posedge clk) begin
    if (rst) begin
      of_w <= '0; of_r <= '0; ofifo_n <= '0; inflight <= '0;
    end else begin
      if (out_done) begin
        of_i[of_w] <= s5_i;
        of_q[of_w] <= s5_q;
        of_w <= of_w + 3'd1;
      end
      if (of_pop) of_r <= of_r + 3'd1;
      ofifo_n  <= ofifo_n + (out_done ? 4'd1 : 4'd0) - (of_pop ? 4'd1 : 4'd0);
      inflight <= inflight + ((issue && k == 0) ? 4'd1 : 4'd0) - (out_done ? 4'd1 : 4'd0);
    end
  end

endmodule
