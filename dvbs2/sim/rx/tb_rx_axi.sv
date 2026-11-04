// Testbench for dvbs2_rx_axi: configures the core over AXI-Lite, plays ADC
// samples on a separate 48 MHz clock (one every +adcdiv clocks), writes the
// DMA packets ("hhhhhhhhhhhhhhhh last") and finally the status registers.
//   +ddccoef= +rrccoef= +in= +out= +regs= +adcdiv= +stall= +hold=
//   register values: +modcod= +pilots= +xa= +xb= +finc= +dec= +ntaps= +shift= +cicr= +cicshift=
//                    +tkp= +tki= +tinteg0= +thresh= +pkp= +pki= +maxit= +plscfg= (PLS_CFG, 0: default)
//                    +llrq=1 (LLRQ_CTRL: through the DDR LLR queue, axi_mem +axistall= +axilat=)
//                    +llrqcfg= (LLRQ_CFG, 0: default)
`timescale 1ps/1ps
module tb_rx_axi;
  logic clk = 0, adc_clk = 0, aresetn = 0;
  always #5000 clk = ~clk;                   // 100 MHz
  always #10417 adc_clk = ~adc_clk;          // 48 MHz

  logic [7:0]  awaddr, araddr;
  logic        awvalid = 0, awready, wvalid = 0, wready, bvalid, bready = 1;
  logic [31:0] wdata, rdata;
  logic        arvalid = 0, arready, rvalid, rready = 1;
  logic [1:0]  bresp, rresp;
  logic [63:0] tdata;
  logic        tvalid, tready = 0, tlast;
  logic        adc_valid = 0;
  logic [15:0] adc_i, adc_q;
  logic [31:0] m_axi_awaddr, m_axi_araddr;
  logic [3:0]  m_axi_awlen, m_axi_arlen, m_axi_awcache, m_axi_arcache;
  logic [2:0]  m_axi_awsize, m_axi_arsize;
  logic [1:0]  m_axi_awburst, m_axi_arburst, m_axi_bresp, m_axi_rresp;
  logic        m_axi_awvalid, m_axi_awready, m_axi_wlast, m_axi_wvalid, m_axi_wready, m_axi_bvalid, m_axi_bready;
  logic        m_axi_arvalid, m_axi_arready, m_axi_rlast, m_axi_rvalid, m_axi_rready;
  logic [63:0] m_axi_wdata, m_axi_rdata;
  logic [7:0]  m_axi_wstrb;

  dvbs2_rx_axi #(.LUT_FILE("../../rtl/gen/nco_lut.mem"), .LDPC_ROM_FILE("../../rtl/gen/ldpc_dec_rom.mem"),
                 .PKT_WORDS(1024)) dut (
    .s_axi_aclk(clk), .s_axi_aresetn(aresetn),
    .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
    .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
    .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
    .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
    .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
    .m_axis_tdata(tdata), .m_axis_tvalid(tvalid), .m_axis_tready(tready), .m_axis_tlast(tlast),
    .adc_clk(adc_clk), .adc_valid(adc_valid), .adc_data_i(adc_i), .adc_data_q(adc_q), .m_axi_awaddr,
    .m_axi_awlen, .m_axi_awsize, .m_axi_awburst, .m_axi_awcache, .m_axi_awvalid, .m_axi_awready, .m_axi_wdata,
    .m_axi_wstrb, .m_axi_wlast, .m_axi_wvalid, .m_axi_wready, .m_axi_bresp, .m_axi_bvalid, .m_axi_bready,
    .m_axi_araddr, .m_axi_arlen, .m_axi_arsize, .m_axi_arburst, .m_axi_arcache, .m_axi_arvalid,
    .m_axi_arready, .m_axi_rdata, .m_axi_rresp, .m_axi_rlast, .m_axi_rvalid, .m_axi_rready);

  axi_mem u_mem (
    .clk, .rst(!aresetn), .awaddr(m_axi_awaddr), .awlen(m_axi_awlen), .awsize(m_axi_awsize),
    .awburst(m_axi_awburst), .awvalid(m_axi_awvalid), .awready(m_axi_awready),
    .wdata(m_axi_wdata), .wstrb(m_axi_wstrb), .wlast(m_axi_wlast), .wvalid(m_axi_wvalid), .wready(m_axi_wready),
    .bresp(m_axi_bresp), .bvalid(m_axi_bvalid), .bready(m_axi_bready),
    .araddr(m_axi_araddr), .arlen(m_axi_arlen), .arsize(m_axi_arsize), .arburst(m_axi_arburst),
    .arvalid(m_axi_arvalid), .arready(m_axi_arready), .rdata(m_axi_rdata), .rresp(m_axi_rresp),
    .rlast(m_axi_rlast), .rvalid(m_axi_rvalid), .rready(m_axi_rready));

  // the DUT's awready/wready/arready are high while it is idle
  task automatic axi_write(input logic [7:0] a, input logic [31:0] d);
    @(posedge clk);
    awaddr <= a; wdata <= d; awvalid <= 1; wvalid <= 1;
    @(posedge clk);
    awvalid <= 0; wvalid <= 0;
    while (!bvalid) @(posedge clk);
    @(posedge clk);
  endtask

  task automatic axi_read(input logic [7:0] a, output logic [31:0] d);
    @(posedge clk);
    araddr <= a; arvalid <= 1;
    @(posedge clk);
    arvalid <= 0;
    while (!rvalid) @(posedge clk);
    d = rdata;
    @(posedge clk);
  endtask

  string fin, fout, fregs, fddc, frrc;
  int fi, fo, fr, fc, rc, adcdiv, stall, v, n_in;
  logic [31:0] r;
  logic [17:0] cv;
  logic [15:0] vi, vq;
  logic eof = 0, started = 0;
  longint lv;

  function automatic int arg(input string name, input int dflt);
    int x;
    if (!$value$plusargs({name, "=%d"}, x)) x = dflt;
    return x;
  endfunction

  initial begin
    if (!$value$plusargs("in=%s", fin)) $fatal(1, "+in");
    if (!$value$plusargs("out=%s", fout)) $fatal(1, "+out");
    if (!$value$plusargs("regs=%s", fregs)) $fatal(1, "+regs");
    if (!$value$plusargs("ddccoef=%s", fddc)) $fatal(1, "+ddccoef");
    if (!$value$plusargs("rrccoef=%s", frrc)) $fatal(1, "+rrccoef");
    adcdiv = arg("adcdiv", 4);
    stall = arg("stall", 0);
    fi = $fopen(fin, "r");
    fo = $fopen(fout, "w");
    repeat (8) @(posedge clk);
    aresetn <= 1;
    repeat (20) @(posedge clk);
    axi_read(8'h04, r);
    if (r != 32'h44565232) $fatal(1, "bad ID %h", r);
    axi_write(8'h14, arg("modcod", 4) | (arg("pilots", 1) << 8));
    axi_write(8'h18, arg("xa", 1));
    axi_write(8'h1C, arg("xb", 0));
    if (!$value$plusargs("finc=%d", lv)) lv = 0;
    axi_write(8'h20, 32'(lv));
    axi_write(8'h24, arg("dec", 1) | (arg("ntaps", 2) << 8) | (arg("shift", 16) << 24));
    axi_write(8'h4C, arg("cicr", 0) | (arg("cicshift", 0) << 8));
    axi_read(8'h4C, r);
    if (r != 32'(arg("cicr", 0) | (arg("cicshift", 0) << 8))) $fatal(1, "DDC_CIC reads %h", r);
    axi_write(8'h38, arg("tkp", 0) | (arg("tki", 0) << 16));
    axi_write(8'h5C, arg("tinteg0", 0));
    axi_write(8'h3C, arg("thresh", 0));
    axi_write(8'h40, arg("pkp", 0));
    axi_write(8'h44, arg("pki", 0));
    axi_write(8'h48, arg("maxit", 30));
    if (arg("plscfg", 0) != 0) axi_write(8'hD4, arg("plscfg", 0));
    if (arg("llrqcfg", 0) != 0) axi_write(8'hE4, arg("llrqcfg", 0));
    axi_write(8'hDC, arg("llrq", 0));
    fc = $fopen(fddc, "r");
    axi_write(8'h28, 0);
    for (int a = 0; a < arg("ntaps", 2); a++) begin
      rc = $fscanf(fc, "%h", cv);
      axi_write(8'h2C, 32'(cv));
    end
    $fclose(fc);
    fc = $fopen(frrc, "r");
    axi_write(8'h30, 0);
    for (int a = 0; a < 1024; a++) begin
      rc = $fscanf(fc, "%h", cv);
      axi_write(8'h34, 32'(cv));
    end
    $fclose(fc);
    axi_write(8'h10, 32'h2);                  // soft reset after configuration
    repeat (40) @(posedge clk);
    axi_write(8'h10, 32'h1);                  // enable
    axi_write(8'hA0, arg("snapsrc", 0) << 1 | 1);   // constellation snapshot: the first 1024
    started = 1;
  end

  // ADC samples
  int ph = 0;
  always @(posedge adc_clk) begin
    adc_valid <= 0;
    if (started && !eof) begin
      if (ph == 0) begin
        rc = $fscanf(fi, "%h %h", vi, vq);
        if (rc == 2) begin adc_valid <= 1; adc_i <= vi; adc_q <= vq; n_in++; end
        else eof <= 1;
      end
      ph = (ph + 1 == adcdiv) ? 0 : ph + 1;
    end
  end

  // DMA side
  int idle = 0;
  longint cyc = 0, hold;
  initial if (!$value$plusargs("hold=%d", hold)) hold = 0;
  always @(posedge clk) begin
    cyc++;
    tready <= cyc >= hold && ($urandom % 100) >= stall;   // hold: no DMA yet
    if (tvalid && tready) begin
      if (tlast) $display("packet end at cycle %0d (hold %0d)", cyc, hold);
      $fdisplay(fo, "%016h %0d", tdata, tlast);
      idle = 0;
    end else idle++;
  end

  initial begin
    wait (eof);
    wait (idle > 3000000);
    fr = $fopen(fregs, "w");
    for (int a = 'h50; a <= 'hF0; a += 4) begin
      if (a == 'hA8) continue;
      axi_read(8'(a), r);
      $fdisplay(fr, "%02h %08h", a, r);
    end
    axi_read(8'h00, r);
    $fdisplay(fr, "%02h %08h", 0, r);
    // PLS_SEEN: write 1 to clear
    axi_write(8'hD0, 32'hFFFF_FFFE);
    axi_read(8'hD0, r);
    $fdisplay(fr, "w0 %08h", r);
    axi_write(8'hD0, 32'h0000_0001);
    axi_read(8'hD0, r);
    $fdisplay(fr, "w1 %08h", r);
    axi_write(8'hA4, 0);
    for (int a = 0; a < 1024; a++) begin
      axi_read(8'hA8, r);
      $fdisplay(fr, "s%0d %08h", a, r);
    end
    $fclose(fr);
    $fclose(fo);
    $display("tb_rx_axi: %0d ADC samples", n_in);
    $finish;
  end
endmodule
