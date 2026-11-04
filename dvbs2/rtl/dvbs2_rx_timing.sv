// DVB-S2 RX symbol timing recovery (model: model/rxfixed.py, class Timing).
//
// Input: complex samples at ~2 samples/symbol. A 32-phase polyphase RRC
// matched filter (25 taps per phase) is evaluated twice per symbol, at the
// on-time point (newest sample k) and one sample earlier (mid point); the
// phase index is the top 5 bits of the fractional delay mu. Gardner TED,
// PI loop: k advances by 2 + {-1, 0, +1} per symbol.
//
// The AGC gain multiplies the filter outputs; it is updated per symbol from
// the on-time symbol power (target 2^25).
//
// integ_init: the loop integrator's value at reset (Q.24 samples per symbol
// beyond 2), for a known symbol-rate offset (model: timing_integ_preset()).
//
// One symbol takes NT + 17 clocks (42 for NT = 25): up to clk / 42 symbols
// per second, e.g. 2.4 MBd at 100 MHz.

module dvbs2_rx_timing #(
  parameter int NT = 25          // taps per phase (32 phases, 5-bit index)
) (
  input  logic               clk,
  input  logic               rst,

  input  logic [15:0]        kp,
  input  logic [15:0]        ki,
  input  logic signed [31:0] integ_init,
  input  logic               coef_we,
  input  logic [9:0]         coef_addr,     // {phase[4:0], tap[4:0]}
  input  logic signed [17:0] coef_data,

  input  logic               s_valid,
  output logic               s_ready,
  input  logic signed [15:0] s_i,
  input  logic signed [15:0] s_q,

  output logic               m_valid,
  input  logic               m_ready,
  output logic signed [15:0] m_i,
  output logic signed [15:0] m_q,

  output logic [16:0]        agc_gain,
  output logic signed [31:0] loop_integ
);

  localparam int  MU_F     = 24;
  localparam int  AGC_SH   = 12;
  localparam logic [32:0] P_TARGET = 33'd1 << 25;
  localparam logic [16:0] G_INIT = 17'd4096;
  localparam logic [16:0] G_MAX  = 17'h1FFFF;

  // ------------------------------------------------------------------ storage
  logic [31:0] xbuf [64];                       // {q, i}
  (* ram_style = "block" *) logic signed [17:0] cbuf [1024];

  always_ff @(posedge clk) if (coef_we) cbuf[coef_addr] <= coef_data;

  logic [15:0] wcnt;          // samples written
  logic [15:0] k;             // newest sample of the on-time window

  wire [15:0] base = k - 16'(NT);
  assign s_ready = !rst && (16'(wcnt - base) < 16'd63);

  always_ff @(posedge clk) begin
    if (rst) wcnt <= '0;
    else if (s_valid && s_ready) begin
      xbuf[wcnt[5:0]] <= {s_q, s_i};
      wcnt <= wcnt + 16'd1;
    end
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

  logic [MU_F-1:0]    mu;
  logic signed [31:0] integ;
  logic [31:0]        gacc;
  logic [16:0]        g;
  logic signed [15:0] prev_i, prev_q;

  assign agc_gain   = g;
  assign loop_integ = integ;

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

  wire avail = $signed(16'(wcnt - k)) > 0;
  wire signed [39:0] gs = 40'($signed({1'b0, g}));

  always_ff @(posedge clk) begin
    // read stage (LUTRAM + BRAM, one clock)
    rd_v   <= iss;
    rd_on  <= iss && (t < 6'(NT));
    rd_mid <= iss && (t != 0);
    xrd    <= xbuf[6'(k - 16'(t))];
    crd    <= cbuf[{ph, t[4:0]}];
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
    if (rst) begin
      state   <= S_IDLE;
      iss     <= 1'b0;
      k       <= 16'(NT);
      mu      <= '0;
      integ   <= integ_init;
      g       <= G_INIT;
      gacc    <= 32'(G_INIT) << 15;
      prev_i  <= '0;
      prev_q  <= '0;
      m_valid <= 1'b0;
    end else begin
      if (m_valid && m_ready) m_valid <= 1'b0;

      case (state)
        S_IDLE: if (avail) begin
          ph    <= mu[MU_F-1 -: 5];
          t     <= '0;
          iss   <= 1'b1;
          acc_oi <= '0; acc_oq <= '0; acc_mi <= '0; acc_mq <= '0;
          state <= S_MAC;
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

        S_OUT: if (!m_valid || m_ready) begin
          m_valid <= 1'b1;
          m_i <= y_oi;
          m_q <= y_oq;
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
          pr_i <= 40'(es) * 40'($signed({1'b0, ki}));
          pr_p <= 40'(es) * 40'($signed({1'b0, kp}));
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

        S_LOOP3: begin : loop3
          logic signed [9:0] st;
          st = 10'(mu_new >>> MU_F);
          if (st < -10'sd1)     st = -10'sd1;
          else if (st > 10'sd1) st = 10'sd1;
          k <= k + 16'(st) + 16'd2;
          mu <= mu_new[MU_F-1:0];
          g  <= gacc[31:15];
          state <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
