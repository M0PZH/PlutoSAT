// Testbench for dvbs2_rx_axi (two receive slots): configures the slots over
// AXI-Lite, plays ADC samples on a separate 48 MHz clock (one every +adcdiv
// clocks), writes the DMA packets ("hhhhhhhhhhhhhhhh last") and finally the
// registers of both slot banks and the shared bank ("aaa vvvvvvvv").
//   +in= +out= +regs= +adcdiv= +stall= +hold=
//   slot s = 0, 1 runs with +en<s>=1 (default: slot 0 only), registers of its bank:
//     +ddccoef<s>= +rrccoef<s>= (files) +modcod<s>= +pilots<s>= +xa<s>= +xb<s>= +finc<s>=
//     +dec<s>= +ntaps<s>= +shift<s>= +cicr<s>= +cicshift<s>= +tkp<s>= +tki<s>= +tinteg0<s>=
//     +thresh<s>= +pkp<s>= +pki<s>= +maxit<s>= +plscfg<s>= (PLS_CFG, 0: default)
//     +rst<s>at=<n>: soft reset of the slot (its CTRL = 2, then 1) after n ADC samples
//   +llrqcfg= (LLRQ_CFG, 0: default; the LLR queue on axi_mem +axistall= +axilat=)
//   +snapslot= +snapsrc= constellation snapshot of that slot (armed through its bank)
//   +v1=1: slot 0 configured as 1.x software does it (bank 0 alone: CTRL = 2, then 1),
//          no write to the shared bank
`timescale 1ps/1ps
module tb_rx_axi;
  logic clk = 0, adc_clk = 0, aresetn = 0;
  always #5000 clk = ~clk;                   // 100 MHz
  always #10417 adc_clk = ~adc_clk;          // 48 MHz

  logic [9:0]  awaddr, araddr;
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

  // the DUT's awready/wready/arready are high while it is idle; one access at a time
  semaphore bus = new(1);
  task automatic axi_write(input logic [9:0] a, input logic [31:0] d);
    bus.get(1);
    @(posedge clk);
    awaddr <= a; wdata <= d; awvalid <= 1; wvalid <= 1;
    @(posedge clk);
    awvalid <= 0; wvalid <= 0;
    while (!bvalid) @(posedge clk);
    @(posedge clk);
    bus.put(1);
  endtask

  task automatic axi_read(input logic [9:0] a, output logic [31:0] d);
    bus.get(1);
    @(posedge clk);
    araddr <= a; arvalid <= 1;
    @(posedge clk);
    arvalid <= 0;
    while (!rvalid) @(posedge clk);
    d = rdata;
    @(posedge clk);
    bus.put(1);
  endtask

  string fin, fout, fregs, fddc, frrc;
  int fi, fo, fr, fc, rc, adcdiv, stall, v, n_in, v1;
  logic [31:0] r;
  logic [17:0] cv;
  logic [15:0] vi, vq;
  logic eof = 0, started = 0;
  logic [1:0] en;
  longint lv;

  function automatic int arg(input string name, input int dflt);
    int x;
    if (!$value$plusargs({name, "=%d"}, x)) x = dflt;
    return x;
  endfunction

  function automatic int sarg(input string name, input int s, input int dflt);
    return arg($sformatf("%s%0d", name, s), dflt);
  endfunction

  task automatic config_slot(input int s);
    logic [9:0] b;
    string nm;
    b = 10'(s * 'h100);
    axi_write(b + 10'h14, sarg("modcod", s, 4) | (sarg("pilots", s, 1) << 8));
    axi_write(b + 10'h18, sarg("xa", s, 1));
    axi_write(b + 10'h1C, sarg("xb", s, 0));
    nm = $sformatf("finc%0d=%%d", s);
    if (!$value$plusargs(nm, lv)) lv = 0;
    axi_write(b + 10'h20, 32'(lv));
    axi_write(b + 10'h24, sarg("dec", s, 1) | (sarg("ntaps", s, 2) << 8) | (sarg("shift", s, 16) << 24));
    axi_write(b + 10'h4C, sarg("cicr", s, 0) | (sarg("cicshift", s, 0) << 8));
    axi_read(b + 10'h4C, r);
    if (r != 32'(sarg("cicr", s, 0) | (sarg("cicshift", s, 0) << 8))) $fatal(1, "DDC_CIC reads %h", r);
    axi_write(b + 10'h38, sarg("tkp", s, 0) | (sarg("tki", s, 0) << 16));
    axi_write(b + 10'h5C, sarg("tinteg0", s, 0));
    axi_write(b + 10'h3C, sarg("thresh", s, 0));
    axi_write(b + 10'h40, sarg("pkp", s, 0));
    axi_write(b + 10'h44, sarg("pki", s, 0));
    axi_write(b + 10'h48, sarg("maxit", s, 30));
    if (sarg("plscfg", s, 0) != 0) axi_write(b + 10'hD4, sarg("plscfg", s, 0));
    nm = $sformatf("ddccoef%0d=%%s", s);
    if (!$value$plusargs(nm, fddc)) $fatal(1, "+ddccoef%0d", s);
    nm = $sformatf("rrccoef%0d=%%s", s);
    if (!$value$plusargs(nm, frrc)) $fatal(1, "+rrccoef%0d", s);
    fc = $fopen(fddc, "r");
    axi_write(b + 10'h28, 0);
    for (int a = 0; a < sarg("ntaps", s, 2); a++) begin
      rc = $fscanf(fc, "%h", cv);
      axi_write(b + 10'h2C, 32'(cv));
    end
    $fclose(fc);
    fc = $fopen(frrc, "r");
    axi_write(b + 10'h30, 0);
    for (int a = 0; a < 1024; a++) begin
      rc = $fscanf(fc, "%h", cv);
      axi_write(b + 10'h34, 32'(cv));
    end
    $fclose(fc);
  endtask

  initial begin
    if (!$value$plusargs("in=%s", fin)) $fatal(1, "+in");
    if (!$value$plusargs("out=%s", fout)) $fatal(1, "+out");
    if (!$value$plusargs("regs=%s", fregs)) $fatal(1, "+regs");
    adcdiv = arg("adcdiv", 4);
    stall = arg("stall", 0);
    v1 = arg("v1", 0);
    en = {arg("en1", 0) != 0, arg("en0", 1) != 0};
    fi = $fopen(fin, "r");
    fo = $fopen(fout, "w");
    repeat (8) @(posedge clk);
    aresetn <= 1;
    repeat (20) @(posedge clk);
    axi_read(v1 ? 10'h004 : 10'h204, r);
    if (r != 32'h44565232) $fatal(1, "bad ID %h", r);
    if (!v1 && arg("llrqcfg", 0) != 0) axi_write(10'h2E4, arg("llrqcfg", 0));
    for (int s = 0; s < 2; s++)
      if (en[s]) config_slot(s);
    if (v1) begin
      axi_write(10'h010, 32'h2);              // 1.x: soft reset after configuration, then enable
      repeat (40) @(posedge clk);
      axi_write(10'h010, 32'h1);
    end else begin
      axi_write(10'h210, 32'h2);              // soft reset of the receiver after configuration
      repeat (40) @(posedge clk);
      for (int s = 0; s < 2; s++)             // enable through the slots' banks
        if (en[s]) axi_write(10'(s * 'h100 + 'h10), 32'h1);
    end
    // constellation snapshot of the first 1024 symbols
    axi_write(10'(arg("snapslot", 0) * 'h100 + 'hA0), arg("snapsrc", 0) << 1 | 1);
    started = 1;
  end

  // per-slot soft reset after n ADC samples (the other slot runs on)
  for (genvar s = 0; s < 2; s++) begin : g_rst
    initial begin
      int n;
      n = arg($sformatf("rst%0dat", s), -1);
      if (n >= 0) begin
        wait (started && n_in >= n);
        axi_write(10'(s * 'h100 + 'h10), 32'h2);
        repeat (40) @(posedge clk);
        axi_write(10'(s * 'h100 + 'h10), 32'h1);
        $display("slot %0d reset at ADC sample %0d", s, n);
      end
    end
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
    for (int a = 0; a < 'h300; a += 4) begin
      if (a[7:0] == 'hA8) continue;
      axi_read(10'(a), r);
      $fdisplay(fr, "%03h %08h", a, r);
    end
    for (int s = 0; s < 2; s++) begin
      if (!en[s]) continue;
      // PLS_SEEN: write 1 to clear
      axi_write(10'(s * 'h100 + 'hD0), 32'hFFFF_FFFE);
      axi_read(10'(s * 'h100 + 'hD0), r);
      $fdisplay(fr, "w0_%0d %08h", s, r);
      axi_write(10'(s * 'h100 + 'hD0), 32'h0000_0001);
      axi_read(10'(s * 'h100 + 'hD0), r);
      $fdisplay(fr, "w1_%0d %08h", s, r);
    end
    axi_write(10'h2A4, 0);
    for (int a = 0; a < 1024; a++) begin
      axi_read(10'(arg("snapslot", 0) * 'h100 + 'hA8), r);
      $fdisplay(fr, "s%0d %08h", a, r);
    end
    $fclose(fr);
    $fclose(fo);
    $display("tb_rx_axi: %0d ADC samples", n_in);
    $finish;
  end
endmodule
