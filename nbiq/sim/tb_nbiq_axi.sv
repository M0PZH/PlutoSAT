// File-driven testbench for nbiq_axi.
//   +adc=<file>  ADC samples "i0 q0 i1 q1" (16-bit hex), sent one per
//                +adc_div adc_clk cycles (adc_clk period +adc_ps ps)
//   +cmd=<file>  commands, one per line:
//                  W <addr> <data>   AXI-Lite write (hex)
//                  R <addr>          AXI-Lite read, "R addr data" to +out
//                  A <n>             start sending the next n ADC samples (background)
//                  E                 wait until the ADC samples sent so far are done
//                  D <n>             wait n clocks
//   +out=<file>  read results and "T ch data" for every tap sample ({q, i}) as
//                they come, then "M addr data" for every memory word written
//   +stall=<0..100>  AXI3 slave ready/response stall probability (%)
// Parameter DW: the AXI master's data width (32 or 64, verilator -GDW=..).
`timescale 1ns/1ps
module tb_nbiq_axi #(parameter int DW = 32);
  localparam int BEATS = 64 / (DW / 8);
  logic clk = 0, aresetn = 0;
  always #5 clk = ~clk;
  logic adc_clk = 0;
  int   adc_ps;
  real  adc_half;                              // ns
  initial begin
    if (!$value$plusargs("adc_ps=%d", adc_ps)) adc_ps = 20833;
    adc_half = adc_ps / 2000.0;
    forever #(adc_half) adc_clk = ~adc_clk;
  end

  logic [15:0] s_axi_awaddr, s_axi_araddr;
  logic        s_axi_awvalid = 0, s_axi_awready, s_axi_wvalid = 0, s_axi_wready;
  logic [31:0] s_axi_wdata, s_axi_rdata;
  logic [1:0]  s_axi_bresp, s_axi_rresp;
  logic        s_axi_bvalid, s_axi_bready = 1, s_axi_arvalid = 0, s_axi_arready;
  logic        s_axi_rvalid, s_axi_rready = 1;

  logic [31:0] m_axi_awaddr;
  logic [DW-1:0] m_axi_wdata;
  logic [DW/8-1:0] m_axi_wstrb;
  logic [3:0]  m_axi_awlen, m_axi_awcache;
  logic [2:0]  m_axi_awsize;
  logic [1:0]  m_axi_awburst, m_axi_bresp = 0;
  logic        m_axi_awvalid, m_axi_awready, m_axi_wlast, m_axi_wvalid, m_axi_wready;
  logic        m_axi_bvalid = 0, m_axi_bready;
  logic        irq;
  logic        tap_valid;
  logic [2:0]  tap_ch;
  logic [15:0] tap_i, tap_q;

  logic        adc_valid = 0;
  logic [15:0] adc_data_i0, adc_data_q0, adc_data_i1, adc_data_q1;

  nbiq_axi #(.LUT_FILE("../rtl/gen/nbiq_lut.mem"), .DW(DW)) dut (
    .s_axi_aclk(clk), .s_axi_aresetn(aresetn), .*);

  // ---------------------------------------------------------------- AXI3 slave
  int stall;
  logic [31:0] mem [logic [31:0]];
  logic [31:0] aq [$], wq [$];              // accepted burst addresses / data words
  int          wbeat = 0, nb_pend = 0;
  always @(posedge clk) begin
    m_axi_awready <= ($urandom % 100) >= stall;
    m_axi_wready  <= ($urandom % 100) >= stall;
    if (m_axi_awvalid && m_axi_awready) begin
      if (m_axi_awlen != BEATS - 1 || m_axi_awsize != $clog2(DW / 8) || m_axi_awburst != 1)
        $fatal(1, "burst type");
      if (m_axi_awaddr[5:0] != 0) $fatal(1, "burst alignment %h", m_axi_awaddr);
      aq.push_back(m_axi_awaddr);
    end
    if (m_axi_wvalid && m_axi_wready) begin
      if (m_axi_wstrb != '1) $fatal(1, "wstrb");
      if (m_axi_wlast != (wbeat == BEATS - 1)) $fatal(1, "wlast at beat %0d", wbeat);
      for (int l = 0; l < DW / 32; l++) wq.push_back(m_axi_wdata[32 * l +: 32]);
      wbeat = (wbeat + 1) % BEATS;
    end
    while (aq.size() > 0 && wq.size() >= 16) begin
      automatic logic [31:0] a = aq.pop_front();
      for (int k = 0; k < 16; k++) mem[a + 32'(4 * k)] = wq.pop_front();
      nb_pend++;
    end
    if (m_axi_bvalid && m_axi_bready) m_axi_bvalid <= 0;
    else if (!m_axi_bvalid && nb_pend > 0 && ($urandom % 100) >= stall) begin
      m_axi_bvalid <= 1; nb_pend--;
    end
  end

  // ---------------------------------------------------------------- ADC source
  int fa, adc_div, adc_limit = 0, adc_sent = 0, adc_ph = 0;
  logic [15:0] a0, a1, a2, a3;
  always @(posedge adc_clk) begin
    adc_valid <= 0;
    if (adc_sent < adc_limit) begin
      if (adc_ph == 0) begin
        if ($fscanf(fa, "%h %h %h %h", a0, a1, a2, a3) != 4) $fatal(1, "ADC file short");
        adc_data_i0 <= a0; adc_data_q0 <= a1; adc_data_i1 <= a2; adc_data_q1 <= a3;
        adc_valid <= 1;
        adc_sent <= adc_sent + 1;
      end
      adc_ph <= (adc_ph + 1 == adc_div) ? 0 : adc_ph + 1;
    end
  end

  // ---------------------------------------------------------------- commands
  // driven and sampled at negedges (a handshake seen there happens at the next posedge)
  task automatic axi_write(input logic [15:0] a, input logic [31:0] d);
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

  task automatic axi_read(input logic [15:0] a, output logic [31:0] d);
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
  int fc, fo = 0, rc, n;

  always @(posedge clk)
    if (tap_valid && fo != 0) $fdisplay(fo, "T %0d %h", tap_ch, {tap_q, tap_i});

  logic [31:0] ca, cd, rd;
  int timeout_us;
  initial begin
    if (!$value$plusargs("timeout_us=%d", timeout_us)) timeout_us = 100000;
    #(timeout_us * 1000.0);
    $fatal(1, "timeout");
  end

  initial begin
    if (!$value$plusargs("cmd=%s", fcmd)) $fatal(1, "+cmd");
    if (!$value$plusargs("out=%s", fout)) $fatal(1, "+out");
    if (!$value$plusargs("adc=%s", fadc)) $fatal(1, "+adc");
    if (!$value$plusargs("adc_div=%d", adc_div)) adc_div = 4;
    if (!$value$plusargs("stall=%d", stall)) stall = 0;
    fc = $fopen(fcmd, "r");
    fo = $fopen(fout, "w");
    fa = $fopen(fadc, "r");
    repeat (20) @(posedge clk);
    aresetn <= 1;
    repeat (40) @(posedge clk);
    while ($fscanf(fc, "%s", op) == 1) begin
      case (op)
        "W": begin rc = $fscanf(fc, "%h %h", ca, cd); axi_write(16'(ca), cd); end
        "R": begin rc = $fscanf(fc, "%h", ca); axi_read(16'(ca), rd); $fdisplay(fo, "R %h %h", ca, rd); end
        "A": begin rc = $fscanf(fc, "%d", n); adc_limit = adc_limit + n; end
        "E": begin while (adc_sent < adc_limit) @(posedge clk); repeat (2000) @(posedge clk); end
        "D": begin rc = $fscanf(fc, "%d", n); repeat (n) @(posedge clk); end
        default: $fatal(1, "bad command %s", op);
      endcase
    end
    foreach (mem[a]) $fdisplay(fo, "M %h %h", a, mem[a]);
    $fclose(fo);
    $finish;
  end
endmodule
