// File-driven testbench for rx_awgn_axi.
//   +adc=<file>  input samples "i0 q0 i1 q1" (hex), one per +adc_div clocks
//   +cmd=<file>  W <addr> <data> | R <addr> | A <n> | E | D <n>  (as tb_nbiq_axi)
//   +out=<file>  "R addr data" lines, then "O cycle i0 q0 i1 q1" for every
//                output sample (cycle: ADC clocks since the noise generator reset)
`timescale 1ns/1ps
module tb_rx_awgn;
  logic clk = 0, aresetn = 0, adc_clk = 0;
  always #5 clk = ~clk;
  always #2 adc_clk = ~adc_clk;                // 250 MHz

  logic [7:0]  s_axi_awaddr, s_axi_araddr;
  logic        s_axi_awvalid = 0, s_axi_awready, s_axi_wvalid = 0, s_axi_wready;
  logic [31:0] s_axi_wdata, s_axi_rdata;
  logic [1:0]  s_axi_bresp, s_axi_rresp;
  logic        s_axi_bvalid, s_axi_bready = 1, s_axi_arvalid = 0, s_axi_arready;
  logic        s_axi_rvalid, s_axi_rready = 1;

  logic [3:0]  in_valid = 0, out_valid;
  logic [15:0] in_i0, in_q0, in_i1, in_q1, out_i0, out_q0, out_i1, out_q1;

  rx_awgn_axi dut (.s_axi_aclk(clk), .s_axi_aresetn(aresetn), .*);

  int fa, fo, adc_div, adc_limit = 0, adc_sent = 0, adc_ph = 0;
  longint cyc;
  logic [15:0] a0, a1, a2, a3;
  always @(posedge adc_clk) begin
    cyc <= dut.a_rst ? 0 : cyc + 1;
    in_valid <= 0;
    if (adc_sent < adc_limit) begin
      if (adc_ph == 0) begin
        if ($fscanf(fa, "%h %h %h %h", a0, a1, a2, a3) != 4) $fatal(1, "ADC file short");
        in_i0 <= a0; in_q0 <= a1; in_i1 <= a2; in_q1 <= a3;
        in_valid <= 4'hF;
        adc_sent <= adc_sent + 1;
      end
      adc_ph <= (adc_ph + 1 == adc_div) ? 0 : adc_ph + 1;
    end
    if (out_valid[0]) $fdisplay(fo, "O %0d %h %h %h %h", cyc, out_i0, out_q0, out_i1, out_q1);
    if (out_valid != 0 && out_valid != 4'hF) $fatal(1, "valids not aligned");
  end

  // driven and sampled at negedges (a handshake seen there happens at the next posedge)
  task automatic axi_write(input logic [7:0] a, input logic [31:0] d);
    @(negedge clk);
    s_axi_awaddr = a; s_axi_awvalid = 1;
    s_axi_wdata = d;  s_axi_wvalid = 1;
    while (s_axi_awvalid || s_axi_wvalid) begin
      automatic bit aw = s_axi_awvalid && s_axi_awready;
      automatic bit w  = s_axi_wvalid && s_axi_wready;
      @(negedge clk);
      if (aw) s_axi_awvalid = 0;
      if (w)  s_axi_wvalid = 0;
    end
    while (!s_axi_bvalid) @(negedge clk);
    @(negedge clk);
  endtask

  task automatic axi_read(input logic [7:0] a, output logic [31:0] d);
    @(negedge clk);
    s_axi_araddr = a; s_axi_arvalid = 1;
    while (!s_axi_arready) @(negedge clk);
    @(negedge clk);
    s_axi_arvalid = 0;
    while (!s_axi_rvalid) @(negedge clk);
    d = s_axi_rdata;
    @(negedge clk);
  endtask

  string fcmd, fout, fadc, op;
  int fc, rc, n;
  logic [31:0] ca, cd, rd;
  initial begin
    if (!$value$plusargs("cmd=%s", fcmd)) $fatal(1, "+cmd");
    if (!$value$plusargs("out=%s", fout)) $fatal(1, "+out");
    if (!$value$plusargs("adc=%s", fadc)) $fatal(1, "+adc");
    if (!$value$plusargs("adc_div=%d", adc_div)) adc_div = 4;
    fc = $fopen(fcmd, "r");
    fo = $fopen(fout, "w");
    fa = $fopen(fadc, "r");
    repeat (20) @(posedge clk);
    aresetn = 1;
    repeat (40) @(posedge clk);
    while ($fscanf(fc, "%s", op) == 1) begin
      case (op)
        "W": begin rc = $fscanf(fc, "%h %h", ca, cd); axi_write(8'(ca), cd); end
        "R": begin rc = $fscanf(fc, "%h", ca); axi_read(8'(ca), rd); $fdisplay(fo, "R %h %h", ca, rd); end
        "A": begin rc = $fscanf(fc, "%d", n); adc_limit = adc_limit + n; end
        "E": begin while (adc_sent < adc_limit) @(posedge clk); repeat (200) @(posedge clk); end
        "D": begin rc = $fscanf(fc, "%d", n); repeat (n) @(posedge clk); end
        default: $fatal(1, "bad command %s", op);
      endcase
    end
    $fclose(fo);
    $finish;
  end
endmodule
