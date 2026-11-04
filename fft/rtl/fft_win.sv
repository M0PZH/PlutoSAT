// Window: y = (x * w[m] + 2^16) >> 17 for I and Q (model: model/fftspec.py).
// Half table w[m] = w[4095 - m] = WIN[m], m < 2048, 18-bit unsigned
// (2^17 = 1.0, values above 2^17 overflow), one 36 Kb block RAM written by
// the ARM and initialised from WIN_FILE (Blackman-Harris). Latency 6 clocks.
// Transform length N = 4096 >> len: sample n of a frame takes the 4096-point
// window at m = n * 2^len (every 2^len-th sample, the same window shape:
// Blackman-Harris keeps its -92 dB sidelobes and ENBW 2.005 bins).

module fft_win #(
  parameter string WIN_FILE = "fft_win.mem"
) (
  input  logic               clk,
  input  logic               srst,

  input  logic               we,
  input  logic [10:0]        wa,
  input  logic [17:0]        wd,

  input  logic [1:0]         len,            // N = 4096 >> len

  input  logic               in_v,
  input  logic [11:0]        in_n,           // sample index in the frame, < N
  input  logic signed [15:0] in_i,
  input  logic signed [15:0] in_q,

  output logic               out_v,
  output logic               out_last,
  output logic signed [15:0] out_i,
  output logic signed [15:0] out_q
);

  // stage 0: input register, 4096-point index m = n << len, table address
  // (mirrored in the upper half)
  logic               v0, l0;
  logic [11:0]        m0;
  logic signed [15:0] i0, q0;
  always_ff @(posedge clk) begin
    v0 <= in_v && !srst;
    m0 <= in_n << len;
    l0 <= (in_n | ~(12'hFFF >> len)) == 12'hFFF;
    i0 <= in_i;
    q0 <= in_q;
  end
  wire [10:0] ra = m0[11] ? ~m0[10:0] : m0[10:0];

  logic [17:0] w2;
  xpm_memory_sdpram #(
    .ADDR_WIDTH_A(11), .ADDR_WIDTH_B(11),
    .BYTE_WRITE_WIDTH_A(18), .WRITE_DATA_WIDTH_A(18), .READ_DATA_WIDTH_B(18),
    .MEMORY_SIZE(2048 * 18), .MEMORY_PRIMITIVE("block"),
    .MEMORY_INIT_FILE(WIN_FILE), .USE_MEM_INIT(1),
    .READ_LATENCY_B(2), .CLOCKING_MODE("common_clock"), .WRITE_MODE_B("read_first")
  ) u_ram (
    .clka(clk), .ena(1'b1), .wea(we), .addra(wa), .dina(wd),
    .injectsbiterra(1'b0), .injectdbiterra(1'b0),
    .clkb(clk), .rstb(1'b0), .enb(1'b1), .regceb(1'b1), .addrb(ra), .doutb(w2),
    .sbiterrb(), .dbiterrb(), .sleep(1'b0));

  // stages 1, 2: wait for the table (data aligned with stage 2)
  logic               v1, v2, l1, l2;
  logic signed [15:0] i1, q1, i2, q2;
  always_ff @(posedge clk) begin
    v1 <= v0 && !srst; l1 <= l0; i1 <= i0; q1 <= q0;
    v2 <= v1 && !srst; l2 <= l1; i2 <= i1; q2 <= q1;
  end

  // stages 3..5: DSP48 input, multiplier and output registers
  logic               v3, v4, v5, l3, l4, l5;
  logic signed [18:0] w3;
  logic signed [15:0] i3, q3;
  logic signed [34:0] mi4, mq4, pi5, pq5;
  always_ff @(posedge clk) begin
    v3 <= v2 && !srst; l3 <= l2;
    w3 <= $signed({1'b0, w2});
    i3 <= i2; q3 <= q2;
    v4 <= v3 && !srst; l4 <= l3;
    mi4 <= i3 * w3;
    mq4 <= q3 * w3;
    v5 <= v4 && !srst; l5 <= l4;
    pi5 <= mi4 + 35'sd65536;
    pq5 <= mq4 + 35'sd65536;
  end

  assign out_v    = v5;
  assign out_last = l5;
  assign out_i    = pi5[32:17];
  assign out_q    = pq5[32:17];

endmodule
