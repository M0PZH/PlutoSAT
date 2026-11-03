// Pipelined complex rotator: (i + jq) * exp(j*2*pi*phase/2^32), 1024-entry
// {sin, cos} LUT (the TX NCO table), rounded by 2^15 and saturated.
// Latency 3; a tag travels with each sample.
module dvbs2_rx_rot #(
  parameter string LUT_FILE = "nco_lut.mem",
  parameter int    TW       = 4
) (
  input  logic               clk,
  input  logic               in_v,
  input  logic signed [15:0] in_i,
  input  logic signed [15:0] in_q,
  input  logic [31:0]        in_ph,
  input  logic [TW-1:0]      in_tag,
  output logic               out_v,
  output logic signed [15:0] out_i,
  output logic signed [15:0] out_q,
  output logic [TW-1:0]      out_tag
);
  (* rom_style = "block" *) logic [31:0] lut [1024];
  initial $readmemh(LUT_FILE, lut);

  logic               v1, v2;
  logic signed [15:0] i1, q1;
  logic [31:0]        cs1;
  logic [TW-1:0]      t1, t2;
  logic signed [32:0] re2, im2;

  function automatic logic signed [15:0] rs15(input logic signed [32:0] a);
    logic signed [32:0] r;
    r = (a + 33'sd16384) >>> 15;
    if (r > 33'sd32767)       rs15 = 16'sd32767;
    else if (r < -33'sd32768) rs15 = -16'sd32768;
    else                      rs15 = r[15:0];
  endfunction

  always_ff @(posedge clk) begin
    v1  <= in_v;
    i1  <= in_i;
    q1  <= in_q;
    t1  <= in_tag;
    cs1 <= lut[in_ph[31:22]];
    v2  <= v1;
    t2  <= t1;
    re2 <= 33'(i1) * 33'($signed(cs1[15:0])) - 33'(q1) * 33'($signed(cs1[31:16]));
    im2 <= 33'(i1) * 33'($signed(cs1[31:16])) + 33'(q1) * 33'($signed(cs1[15:0]));
    out_v   <= v2;
    out_tag <= t2;
    out_i   <= rs15(re2);
    out_q   <= rs15(im2);
  end
endmodule
