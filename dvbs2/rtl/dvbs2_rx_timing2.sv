// DVB-S2 RX symbol timing recovery for two receive slots (model:
// model/rxfixed.py, class Timing, once per slot). One engine (the MAC
// pipeline, gain, TED and loop arithmetic of dvbs2_rx_timing) is
// time-multiplexed per symbol between two independent slots.
//
// Per slot: the sample buffer (64 x 32 LUTRAM), the coefficient bank (the
// slot is the MSB of the coefficient RAM address: 2048 x 18, one RAMB36),
// the loop gains, the loop state (k, mu, integ, gacc, g, prev), the
// integrator preset and the output register. The engine picks a slot in
// S_IDLE (round robin among the slots that have the samples for their next
// symbol and a free output register), loads that slot's state into the
// working registers in the same clock and writes it back in S_LOOP3. Per
// slot the arithmetic is exactly that of dvbs2_rx_timing, so each slot's
// output equals a dvbs2_rx_timing fed with that slot's samples
// (sim/rx/test_rx2.py).
//
// One symbol of either slot takes NT + 17 clocks (42 for NT = 25), with no
// switching overhead: the sum of the two symbol rates is at most clk / 42
// (2.38 MBd at 100 MHz).
//
// rst resets everything; rst_s[s] resets slot s alone (its samples, loop
// state, output register; its coefficient bank is kept), while the other
// slot keeps running. A slot reset during the slot's own symbol cancels
// that symbol (no output, no write-back). integ_init[s] is loaded at either
// reset (TIMING_INTEG, model: timing_integ_preset()). Coefficients
// (coef_we, coef_slot, coef_addr) can be written at any time, like those of
// dvbs2_rx_timing; write a slot's bank while that slot is held in reset.

module dvbs2_rx_timing2 #(
  parameter int NT = 25          // taps per phase (32 phases, 5-bit index)
) (
  input  logic               clk,
  input  logic               rst,
  input  logic [1:0]         rst_s,

  input  logic [15:0]        kp [2],
  input  logic [15:0]        ki [2],
  input  logic signed [31:0] integ_init [2],
  input  logic               coef_we,
  input  logic               coef_slot,
  input  logic [9:0]         coef_addr,     // {phase[4:0], tap[4:0]}
  input  logic signed [17:0] coef_data,

  input  logic [1:0]         s_valid,
  output logic [1:0]         s_ready,
  input  logic signed [15:0] s_i [2],
  input  logic signed [15:0] s_q [2],

  output logic [1:0]         m_valid,
  input  logic [1:0]         m_ready,
  output logic signed [15:0] m_i [2],
  output logic signed [15:0] m_q [2],

  output logic [16:0]        agc_gain [2],
  output logic signed [31:0] loop_integ [2]
);

  localparam int  MU_F     = 24;
  localparam int  AGC_SH   = 12;
  localparam logic [32:0] P_TARGET = 33'd1 << 25;
  localparam logic [16:0] G_INIT = 17'd4096;
  localparam logic [16:0] G_MAX  = 17'h1FFFF;

  // ------------------------------------------------------------------ storage
  logic [31:0] xbuf0 [64], xbuf1 [64];          // {q, i} per slot
  (* ram_style = "block" *) logic signed [17:0] cbuf [2048];   // {slot, phase, tap}

  always_ff @(posedge clk) if (coef_we) cbuf[{coef_slot, coef_addr}] <= coef_data;

  logic [15:0] wcnt [2];        // samples written

  // per-slot loop state (written back after each symbol)
  logic [15:0]        b_k [2];  // newest sample of the on-time window
  logic [MU_F-1:0]    b_mu [2];
  logic signed [31:0] b_integ [2];
  logic [31:0]        b_gacc [2];
  logic [16:0]        b_g [2];
  logic signed [15:0] b_prev_i [2], b_prev_q [2];

  wire [1:0] srst = rst_s | {2{rst}};

  for (genvar s = 0; s < 2; s++) begin : g_in
    wire [15:0] base = b_k[s] - 16'(NT);
    assign s_ready[s] = !srst[s] && (16'(wcnt[s] - base) < 16'd63);
    always_ff @(posedge clk) begin
      if (srst[s]) wcnt[s] <= '0;
      else if (s_valid[s] && s_ready[s]) wcnt[s] <= wcnt[s] + 16'd1;
    end
    assign agc_gain[s]   = b_g[s];
    assign loop_integ[s] = b_integ[s];
  end
  always_ff @(posedge clk) begin
    if (s_valid[0] && s_ready[0]) xbuf0[wcnt[0][5:0]] <= {s_q[0], s_i[0]};
    if (s_valid[1] && s_ready[1]) xbuf1[wcnt[1][5:0]] <= {s_q[1], s_i[1]};
  end

  // ------------------------------------------------------------------ helpers
  function automatic logic signed [17:0] sat18(input logic signed [39:0] a);
    if (a > 40'sd131071)       sat18 = 18'sd131071;
    else if (a < -40'sd131072) sat18 = -18'sd131072;
    else                       sat18 = a[17:0];
  endfunction

  function automatic logic signed [15:0] sat16(input logic signed [39:0] a);
    if (a > 40'sd32767)       sat16 = 16'sd32767;
    else if (a < -40'sd32768) sat16 = -16'sd32768;
    else                      sat16 = a[15:0];
  endfunction

  // ------------------------------------------------------------------ control
  typedef enum logic [3:0] {S_IDLE, S_MAC, S_MF, S_GAIN, S_GAIN2, S_OUT, S_TP, S_TED, S_TED2,
                            S_LOOP0, S_LOOP1, S_LOOP2, S_LOOP3} state_t;
  state_t state;

  logic               cs;             // slot being processed
  logic               last;           // last slot started (round robin)
  logic               cs_kill;        // the current slot was reset while being processed

  // working copies of the current slot's state
  logic [15:0]        k;
  logic [MU_F-1:0]    mu;
  logic signed [31:0] integ;
  logic [31:0]        gacc;
  logic [16:0]        g;
  logic signed [15:0] prev_i, prev_q;
  logic [15:0]        kp_w, ki_w;

  // MAC pipeline: issue (t) -> read data (a) -> products (b) -> accumulate (c)
  logic [5:0]         t;
  logic               iss;
  logic [4:0]         ph;
  logic               a_v, b_v;
  logic               a_on, a_mid, b_on, b_mid;
  logic signed [15:0] a_xi, a_xq;
  logic signed [17:0] a_c, a_cp;
  logic signed [33:0] b_oi, b_oq, b_mi, b_mq;
  logic signed [39:0] acc_oi, acc_oq, acc_mi, acc_mq;

  logic [31:0] xrd;
  logic signed [17:0] crd;
  logic        rd_v, rd_on, rd_mid;

  // filter outputs
  logic signed [17:0] mf_oi, mf_oq, mf_mi, mf_mq;
  logic signed [15:0] y_oi, y_oq, y_mi, y_mq;
  logic signed [33:0] ted;
  logic signed [35:0] pg_oi, pg_oq, pg_mi, pg_mq;
  logic signed [16:0] d_i, d_q;
  logic signed [33:0] t_i, t_q;
  logic signed [32:0] sq_i, sq_q;
  logic signed [39:0] pr_i, pr_p;
  logic signed [40:0] pr_g;
  logic signed [33:0] mu_new;
  logic signed [21:0] es;
  logic signed [32:0] pwr_err;
  logic signed [17:0] agc_err;

  // slot eligibility: the samples for the next symbol, the output register free
  logic [1:0] elig;
  for (genvar s = 0; s < 2; s++) begin : g_el
    assign elig[s] = !srst[s] && ($signed(16'(wcnt[s] - b_k[s])) > 0) && !m_valid[s];
  end
  wire pick = (elig[0] && elig[1]) ? !last : elig[1];

  wire signed [39:0] gs = 40'($signed({1'b0, g}));
  wire [5:0] xa = 6'(k - 16'(t));

  // read stage (LUTRAM + BRAM, one clock), data and product stages
  always_ff @(posedge clk) begin
    rd_v   <= iss;
    rd_on  <= iss && (t < 6'(NT));
    rd_mid <= iss && (t != 0);
    xrd    <= cs ? xbuf1[xa] : xbuf0[xa];
    crd    <= cbuf[{cs, ph, t[4:0]}];
    // data stage: keep the previous coefficient for the mid-point filter
    a_v   <= rd_v;
    a_on  <= rd_on;
    a_mid <= rd_mid;
    a_xi  <= $signed(xrd[15:0]);
    a_xq  <= $signed(xrd[31:16]);
    a_c   <= crd;
    if (rd_v) a_cp <= a_c;
    // product stage
    b_v   <= a_v;
    b_on  <= a_on;
    b_mid <= a_mid;
    b_oi  <= 34'(a_c) * 34'(a_xi);
    b_oq  <= 34'(a_c) * 34'(a_xq);
    b_mi  <= 34'(a_cp) * 34'(a_xi);
    b_mq  <= 34'(a_cp) * 34'(a_xq);
  end

  // data stage: a_c = c[t], a_cp = c[t-1], a_x = x[k-t]
  //   on-time  sum_t c[t] x[k-t],     t = 0 .. NT-1
  //   mid      sum_t c[t-1] x[k-t],   t = 1 .. NT     (= sum_j c[j] x[k-1-j])

  always_ff @(posedge clk) begin
    if (state != S_IDLE && srst[cs]) cs_kill <= 1'b1;

    if (rst) begin
      state   <= S_IDLE;
      iss     <= 1'b0;
      last    <= 1'b1;
      cs      <= 1'b0;
      cs_kill <= 1'b0;
    end else begin
      case (state)
        S_IDLE: if (elig != 0) begin
          cs      <= pick;
          last    <= pick;
          cs_kill <= 1'b0;
          k       <= b_k[pick];
          mu      <= b_mu[pick];
          integ   <= b_integ[pick];
          gacc    <= b_gacc[pick];
          g       <= b_g[pick];
          prev_i  <= b_prev_i[pick];
          prev_q  <= b_prev_q[pick];
          kp_w    <= kp[pick];
          ki_w    <= ki[pick];
          ph      <= b_mu[pick][MU_F-1 -: 5];
          t       <= '0;
          iss     <= 1'b1;
          acc_oi <= '0; acc_oq <= '0; acc_mi <= '0; acc_mq <= '0;
          state   <= S_MAC;
        end

        S_MAC: begin
          if (iss) begin
            if (t == 6'(NT)) iss <= 1'b0;
            else             t <= t + 6'd1;
          end
          if (b_v) begin
            if (b_on)  begin acc_oi <= acc_oi + 40'(b_oi); acc_oq <= acc_oq + 40'(b_oq); end
            if (b_mid) begin acc_mi <= acc_mi + 40'(b_mi); acc_mq <= acc_mq + 40'(b_mq); end
          end
          if (!iss && !rd_v && !a_v && !b_v) state <= S_MF;
        end

        S_MF: begin
          mf_oi <= sat18((acc_oi + 40'sd8192) >>> 14);
          mf_oq <= sat18((acc_oq + 40'sd8192) >>> 14);
          mf_mi <= sat18((acc_mi + 40'sd8192) >>> 14);
          mf_mq <= sat18((acc_mq + 40'sd8192) >>> 14);
          state <= S_GAIN;
        end

        S_GAIN: begin
          pg_oi <= 36'(mf_oi) * 36'(gs);
          pg_oq <= 36'(mf_oq) * 36'(gs);
          pg_mi <= 36'(mf_mi) * 36'(gs);
          pg_mq <= 36'(mf_mq) * 36'(gs);
          state <= S_GAIN2;
        end

        S_GAIN2: begin
          y_oi <= sat16(40'(pg_oi + 36'sd8192) >>> 14);
          y_oq <= sat16(40'(pg_oq + 36'sd8192) >>> 14);
          y_mi <= sat16(40'(pg_mi + 36'sd8192) >>> 14);
          y_mq <= sat16(40'(pg_mq + 36'sd8192) >>> 14);
          state <= S_OUT;
        end

        S_OUT: begin
          // (the output register is free: the slot was only picked with it free)
          d_i <= 17'(prev_i) - 17'(y_oi);
          d_q <= 17'(prev_q) - 17'(y_oq);
          prev_i <= y_oi;
          prev_q <= y_oq;
          state <= S_TP;
        end

        S_TP: begin
          t_i  <= 34'(d_i) * 34'(y_mi);
          t_q  <= 34'(d_q) * 34'(y_mq);
          sq_i <= 33'(y_oi) * 33'(y_oi);
          sq_q <= 33'(y_oq) * 33'(y_oq);
          state <= S_TED;
        end

        S_TED: begin
          ted <= t_i + t_q;
          pwr_err <= $signed(P_TARGET) - sq_i - sq_q;
          state <= S_TED2;
        end

        S_TED2: begin
          es <= 22'(ted >>> 12);
          if ((pwr_err >>> 9) > 33'sd131071)       agc_err <= 18'sd131071;
          else if ((pwr_err >>> 9) < -33'sd131072) agc_err <= -18'sd131072;
          else                                     agc_err <= 18'(pwr_err >>> 9);
          state <= S_LOOP0;
        end

        S_LOOP0: begin
          pr_i <= 40'(es) * 40'($signed({1'b0, ki_w}));
          pr_p <= 40'(es) * 40'($signed({1'b0, kp_w}));
          pr_g <= 41'($signed({1'b0, g})) * 41'(agc_err);
          state <= S_LOOP1;
        end

        S_LOOP1: begin : agc
          logic signed [40:0] ga;
          integ <= integ + 32'(pr_i >>> 16);
          ga = $signed({9'd0, gacc}) + (pr_g >>> AGC_SH);
          if (ga < 41'sd32768)
            ga = 41'sd32768;
          else if (ga > $signed({9'd0, G_MAX, 15'h7FFF}))
            ga = $signed({9'd0, G_MAX, 15'h7FFF});
          gacc <= ga[31:0];
          state <= S_LOOP2;
        end

        S_LOOP2: begin
          mu_new <= $signed({10'd0, mu}) + 34'(pr_p >>> 8) + 34'(integ);
          state <= S_LOOP3;
        end

        S_LOOP3: state <= S_IDLE;     // write-back below

        default: state <= S_IDLE;
      endcase
    end
  end

  // output registers (a slot reset clears its own)
  for (genvar s = 0; s < 2; s++) begin : g_out
    always_ff @(posedge clk) begin
      if (m_valid[s] && m_ready[s]) m_valid[s] <= 1'b0;
      if (!rst && state == S_OUT && cs == s[0] && !cs_kill) begin
        m_valid[s] <= 1'b1;
        m_i[s] <= y_oi;
        m_q[s] <= y_oq;
      end
      if (srst[s]) m_valid[s] <= 1'b0;
    end
  end

  // write-back of the current slot's state (S_LOOP3), per-slot reset
  always_ff @(posedge clk) begin
    for (int s = 0; s < 2; s++) begin
      if (srst[s]) begin
        b_k[s]      <= 16'(NT);
        b_mu[s]     <= '0;
        b_integ[s]  <= integ_init[s];
        b_g[s]      <= G_INIT;
        b_gacc[s]   <= 32'(G_INIT) << 15;
        b_prev_i[s] <= '0;
        b_prev_q[s] <= '0;
      end else if (state == S_LOOP3 && cs == s[0] && !cs_kill) begin : wb
        logic signed [9:0] st;
        st = 10'(mu_new >>> MU_F);
        if (st < -10'sd1)     st = -10'sd1;
        else if (st > 10'sd1) st = 10'sd1;
        b_k[s]      <= k + 16'(st) + 16'd2;
        b_mu[s]     <= mu_new[MU_F-1:0];
        b_g[s]      <= gacc[31:15];
        b_gacc[s]   <= gacc;
        b_integ[s]  <= integ;
        b_prev_i[s] <= prev_i;
        b_prev_q[s] <= prev_q;
      end
    end
  end

endmodule
