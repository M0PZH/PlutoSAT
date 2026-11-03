// DDC for the DVB-S2 receiver (model: model/rxfixed.py, ddc()): NCO mix,
// then a decimating FIR, ADC rate -> 2 samples/symbol.
//
//   m[n] = x[n] * exp(j*2*pi*n*finc/2^32)            (bypassed when finc = 0)
//   y[k] = sat16(round(sum_{i<ntaps} h[i] m[k*dec + dec-1 - i] / 2^shift))
//
// Two taps per clock (two copies of the sample buffer): an output takes
// ntaps/2 clocks, outputs are computed back to back, so the input rate may
// be up to clk * dec / (ntaps/2), e.g. ntaps = 12*dec: clk/6.
// m_* has no backpressure; an output request while two are pending is
// dropped and counted.

module dvbs2_rx_ddc #(
  parameter string LUT_FILE = "nco_lut.mem"
) (
  input  logic               clk,
  input  logic               rst,

  input  logic [31:0]        finc,
  input  logic [5:0]         dec,           // 1..32
  input  logic [9:0]         ntaps,         // even, 2..512
  input  logic [4:0]         shift,
  input  logic               coef_we,
  input  logic [8:0]         coef_addr,
  input  logic signed [17:0] coef_data,

  input  logic               s_valid,
  output logic               s_ready,
  input  logic signed [15:0] s_i,
  input  logic signed [15:0] s_q,

  output logic               m_valid,
  output logic signed [15:0] m_i,
  output logic signed [15:0] m_q,
  output logic [15:0]        st_dropped
);

  // ------------------------------------------------------------ mixer
  logic        mx_v;
  logic signed [15:0] mx_i, mx_q;
  logic [31:0] ph;
  logic        rot_ov;
  logic signed [15:0] rot_oi, rot_oq;
  logic [7:0]  rot_ot;

  assign s_ready = !rst;

  dvbs2_rx_rot #(.LUT_FILE(LUT_FILE), .TW(8)) u_rot (
    .clk, .in_v(s_valid && finc != 0), .in_i(s_i), .in_q(s_q), .in_ph(ph), .in_tag(8'd0),
    .out_v(rot_ov), .out_i(rot_oi), .out_q(rot_oq), .out_tag(rot_ot));

  always_ff @(posedge clk) begin
    if (rst) begin
      ph <= '0;
      mx_v <= 1'b0;
    end else begin
      if (s_valid) ph <= ph + finc;
      // bypass when finc = 0 (same latency is not needed: order is kept)
      mx_v <= (s_valid && finc == 0) || rot_ov;
      mx_i <= rot_ov ? rot_oi : s_i;
      mx_q <= rot_ov ? rot_oq : s_q;
    end
  end

  // ------------------------------------------------------------ sample buffers
  logic [31:0] xa [1024], xb [1024];
  logic [31:0] xa_q, xb_q;
  logic [31:0] n;                            // samples written
  logic [5:0]  dcnt;
  logic [17:0] he [256], ho [256];
  logic signed [17:0] he_q, ho_q;

  always_ff @(posedge clk) begin
    if (coef_we) begin
      if (coef_addr[0]) ho[coef_addr[8:1]] <= coef_data;
      else              he[coef_addr[8:1]] <= coef_data;
    end
  end

  // pending outputs: index of the newest sample of each
  logic [31:0] pend [2];
  logic [1:0]  npend;
  logic        trig;

  always_ff @(posedge clk) begin
    trig <= 1'b0;
    if (rst) begin
      n <= '0;
      dcnt <= '0;
    end else if (mx_v) begin
      xa[n[9:0]] <= {mx_q, mx_i};
      xb[n[9:0]] <= {mx_q, mx_i};
      n <= n + 32'd1;
      if (dcnt + 6'd1 >= dec) begin
        dcnt <= '0;
        trig <= 1'b1;
      end else dcnt <= dcnt + 6'd1;
    end
  end

  // ------------------------------------------------------------ MAC engine
  logic        busy;
  logic [31:0] base;                         // newest sample index of the current output
  logic [8:0]  mi;                           // tap pair index
  logic        r_v, r_first, r_last, r_oka, r_okb;
  logic        p_v, p_first, p_last;
  logic signed [33:0] p_ai, p_aq, p_bi, p_bq;
  logic signed [42:0] acc_i, acc_q;
  logic        a_v;

  wire [8:0] npairs_m1 = 9'((ntaps >> 1) - 10'd1);

  always_ff @(posedge clk) begin
    if (rst) begin
      busy <= 1'b0;
      npend <= '0;
      r_v <= 1'b0; p_v <= 1'b0; a_v <= 1'b0;
      m_valid <= 1'b0;
      st_dropped <= '0;
    end else begin : eng
      logic [1:0]  np;
      logic [31:0] p0, p1;
      np = npend; p0 = pend[0]; p1 = pend[1];
      // queue a new output request (newest sample index n - 1 after the write)
      if (trig) begin
        if (np == 2'd2) st_dropped <= st_dropped + 16'd1;
        else if (np == 2'd1) begin p1 = n - 32'd1; np = 2'd2; end
        else begin p0 = n - 32'd1; np = 2'd1; end
      end
      // issue reads: taps 2m (buffer A) and 2m + 1 (buffer B)
      r_v <= 1'b0;
      if (busy) begin
        xa_q <= xa[10'(base - 32'(mi) * 2)];
        xb_q <= xb[10'(base - 32'(mi) * 2 - 32'd1)];
        he_q <= he[8'(mi)];
        ho_q <= ho[8'(mi)];
        r_v <= 1'b1;
        r_first <= mi == 0;
        r_last  <= mi == npairs_m1;
        // samples before the first one are zero
        r_oka <= base >= 32'(mi) * 2;
        r_okb <= base >= 32'(mi) * 2 + 32'd1;
        if (mi != npairs_m1) mi <= mi + 9'd1;
        else if (np != 0) begin
          // next output back to back
          base <= p0;
          mi <= '0;
          p0 = p1;
          np = np - 2'd1;
        end else busy <= 1'b0;
      end else if (np != 0) begin
        busy <= 1'b1;
        base <= p0;
        mi <= '0;
        p0 = p1;
        np = np - 2'd1;
      end
      npend <= np; pend[0] <= p0; pend[1] <= p1;
      // products
      p_v <= r_v; p_first <= r_first; p_last <= r_last;
      p_ai <= r_oka ? 34'($signed(xa_q[15:0]))  * 34'(he_q) : '0;
      p_aq <= r_oka ? 34'($signed(xa_q[31:16])) * 34'(he_q) : '0;
      p_bi <= r_okb ? 34'($signed(xb_q[15:0]))  * 34'(ho_q) : '0;
      p_bq <= r_okb ? 34'($signed(xb_q[31:16])) * 34'(ho_q) : '0;
      // accumulate
      a_v <= p_v && p_last;
      if (p_v) begin
        acc_i <= (p_first ? 43'sd0 : acc_i) + 43'(p_ai) + 43'(p_bi);
        acc_q <= (p_first ? 43'sd0 : acc_q) + 43'(p_aq) + 43'(p_bq);
      end
      // round, saturate
      m_valid <= a_v;
      if (a_v) begin : rs
        logic signed [42:0] ri, rq;
        ri = shift == 0 ? acc_i : (acc_i + (43'sd1 <<< (shift - 5'd1))) >>> shift;
        rq = shift == 0 ? acc_q : (acc_q + (43'sd1 <<< (shift - 5'd1))) >>> shift;
        m_i <= ri > 43'sd32767 ? 16'sd32767 : ri < -43'sd32768 ? -16'sd32768 : ri[15:0];
        m_q <= rq > 43'sd32767 ? 16'sd32767 : rq < -43'sd32768 ? -16'sd32768 : rq[15:0];
      end
    end
  end

endmodule
