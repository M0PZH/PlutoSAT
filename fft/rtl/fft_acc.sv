// Power and averaging of the FFT output (model: model/fftspec.py).
//
// Per output bin k (natural order, one per clock or slower):
//   p = re^2 + im^2                                  (32-bit)
//   A[j] = (first frame ? 0 : A[j]) + p,  j = k ^ 2048 (FFT-shifted address)
// in a 4096 x 40 accumulator RAM. In the last frame of an average the result
// word min((A[j] + p) >> rsh, 2^32 - 1) is written instead:
//   DOUBLE_BUF = 0: back into the accumulator RAM, which is also the result
//                   buffer (read port shared with rd_addr when idle);
//   DOUBLE_BUF = 1: into a separate 4096 x 32 result RAM (read port rd_addr
//                   only), so the next average can run while the ARM reads,
//                   unless keep (the previous result is still unread) is set
//                   when the last frame starts: then the average is dropped.
// rd_data is valid two clocks after rd_addr.

module fft_acc #(
  parameter bit DOUBLE_BUF = 1'b0
) (
  input  logic        clk,
  input  logic        srst,

  input  logic        restart,        // an average starts: frame counter to 0
  input  logic [7:0]  nm1,            // frames per average - 1
  input  logic [3:0]  rsh,            // result right shift, 0..8
  input  logic        keep,           // DOUBLE_BUF: do not overwrite the result

  input  logic        s_valid,
  input  logic [31:0] s_data,         // {im, re}
  input  logic        s_last,

  output logic        frame_end,      // pulse: a frame is accumulated
  output logic        avg_end,        // pulse: the last frame is written
  output logic        avg_pub,        // with avg_end: the result was written
  output logic        sat,            // pulse: a result word saturated
  output logic        order_err,      // pulse: tlast not at bin 4095

  input  logic [11:0] rd_addr,
  output logic [31:0] rd_data
);

  // ------------------------------------------------------------ bin / frame counters
  logic [11:0] k;
  logic [7:0]  f;
  logic        wr_res;
  always_ff @(posedge clk) begin
    order_err <= 1'b0;
    if (srst || restart) begin
      k <= '0; f <= '0; wr_res <= 1'b1;
    end else if (s_valid) begin
      k <= k + 1'b1;
      order_err <= s_last != (k == 12'hFFF);
      if (k == 12'hFFF) f <= (f == nm1) ? 8'd0 : f + 1'b1;
      if (k == 12'h000 && f == nm1) wr_res <= !(DOUBLE_BUF && keep);
    end
  end

  // ------------------------------------------------------------ pipeline
  // stage 0: input register, accumulator read address
  logic               v0, first0, lastf0, lastk0;
  logic [11:0]        j0;
  logic signed [15:0] re0, im0;
  always_ff @(posedge clk) begin
    v0 <= s_valid && !srst;
    j0 <= {~k[11], k[10:0]};
    re0 <= s_data[15:0];
    im0 <= s_data[31:16];
    first0 <= f == 8'd0;
    lastf0 <= f == nm1;
    lastk0 <= k == 12'hFFF;
  end

  logic [39:0] acc_q2;
  logic        acc_we;
  logic [11:0] acc_wa;
  logic [39:0] acc_wd;
  wire  [11:0] acc_ra = (DOUBLE_BUF || v0) ? j0 : rd_addr;

  xpm_memory_sdpram #(
    .ADDR_WIDTH_A(12), .ADDR_WIDTH_B(12),
    .BYTE_WRITE_WIDTH_A(40), .WRITE_DATA_WIDTH_A(40), .READ_DATA_WIDTH_B(40),
    .MEMORY_SIZE(4096 * 40), .MEMORY_PRIMITIVE("block"),
    .READ_LATENCY_B(2), .CLOCKING_MODE("common_clock"), .WRITE_MODE_B("read_first")
  ) u_acc (
    .clka(clk), .ena(1'b1), .wea(acc_we), .addra(acc_wa), .dina(acc_wd),
    .injectsbiterra(1'b0), .injectdbiterra(1'b0),
    .clkb(clk), .rstb(1'b0), .enb(1'b1), .regceb(1'b1), .addrb(acc_ra), .doutb(acc_q2),
    .sbiterrb(), .dbiterrb(), .sleep(1'b0));

  // stages 1..3: |X|^2 in two DSP48 (input, multiplier, output registers);
  // the accumulator word arrives with stage 2
  logic               v1, v2, v3, first1, first2, first3, lastf1, lastf2, lastf3;
  logic               lastk1, lastk2, lastk3;
  logic [11:0]        j1, j2, j3;
  logic signed [15:0] re1, im1;
  logic [31:0]        rr2, ii2;
  logic [31:0]        p3;
  logic [39:0]        acc3;
  always_ff @(posedge clk) begin
    v1 <= v0 && !srst; j1 <= j0; first1 <= first0; lastf1 <= lastf0; lastk1 <= lastk0;
    re1 <= re0; im1 <= im0;
    v2 <= v1 && !srst; j2 <= j1; first2 <= first1; lastf2 <= lastf1; lastk2 <= lastk1;
    rr2 <= 32'(re1 * re1);
    ii2 <= 32'(im1 * im1);
    v3 <= v2 && !srst; j3 <= j2; first3 <= first2; lastf3 <= lastf2; lastk3 <= lastk2;
    p3 <= rr2 + ii2;
    acc3 <= acc_q2;
  end

  // stage 4: accumulate; 5: shift; 6: saturate, write
  logic        v4, v5, v6, lastf4, lastf5, lastf6, lastk4, lastk5, lastk6, wres6;
  logic [11:0] j4, j5, j6;
  logic [39:0] sum4, sum5, sum6, sh5;
  logic [31:0] res6;
  logic        sat6;
  always_ff @(posedge clk) begin
    v4 <= v3 && !srst; j4 <= j3; lastf4 <= lastf3; lastk4 <= lastk3;
    sum4 <= (first3 ? 40'd0 : acc3) + 40'(p3);
    v5 <= v4 && !srst; j5 <= j4; lastf5 <= lastf4; lastk5 <= lastk4;
    sum5 <= sum4;
    sh5 <= sum4 >> rsh;
    v6 <= v5 && !srst; j6 <= j5; lastf6 <= lastf5; lastk6 <= lastk5;
    sum6 <= sum5;
    sat6 <= sh5[39:32] != 8'd0;
    res6 <= (sh5[39:32] != 8'd0) ? 32'hFFFF_FFFF : sh5[31:0];
    wres6 <= wr_res;
  end

  always_comb begin
    acc_wa = j6;
    acc_we = v6 && !(DOUBLE_BUF && lastf6);
    acc_wd = (lastf6 && !DOUBLE_BUF) ? {8'd0, res6} : sum6;
  end

  assign frame_end = v6 && lastk6;
  assign avg_end   = v6 && lastk6 && lastf6;
  assign avg_pub   = wres6;
  assign sat       = v6 && lastf6 && wres6 && sat6;

  // ------------------------------------------------------------ result buffer
  if (DOUBLE_BUF) begin : g_res
    logic [31:0] res_q;
    xpm_memory_sdpram #(
      .ADDR_WIDTH_A(12), .ADDR_WIDTH_B(12),
      .BYTE_WRITE_WIDTH_A(32), .WRITE_DATA_WIDTH_A(32), .READ_DATA_WIDTH_B(32),
      .MEMORY_SIZE(4096 * 32), .MEMORY_PRIMITIVE("block"),
      .READ_LATENCY_B(2), .CLOCKING_MODE("common_clock"), .WRITE_MODE_B("read_first")
    ) u_res (
      .clka(clk), .ena(1'b1), .wea(v6 && lastf6 && wres6), .addra(j6), .dina(res6),
      .injectsbiterra(1'b0), .injectdbiterra(1'b0),
      .clkb(clk), .rstb(1'b0), .enb(1'b1), .regceb(1'b1), .addrb(rd_addr), .doutb(res_q),
      .sbiterrb(), .dbiterrb(), .sleep(1'b0));
    assign rd_data = res_q;
  end else begin : g_nores
    assign rd_data = acc_q2[31:0];
  end

endmodule
