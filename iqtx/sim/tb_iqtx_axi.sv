// File-driven testbench for iqtx_axi.
//   +cmd=<file>  commands, one per line (hex numbers):
//                  W <addr> <data>   AXI-Lite write
//                  R <addr>          AXI-Lite read, "R addr data" to +out
//                  M <addr> <data>   write a 32-bit word into the memory model
//                  D <n>             wait n clocks (decimal)
//                  S <mask> <val>    wait until (STATUS & mask) == val (timeout: error)
//                  P <addr> <min>    wait until register addr >= min (mod 2^32: (int)(r - min) >= 0)
//                  V <mode>          DVB-S2 input: 0 zero, 1 a counter pattern
//                  E                 end
//   +out=<file>  "R addr data" reads; "O data" every DAC sample taken from the
//                IQ FIFO; "X x r st" every envelope sample (x = {Q, I} in,
//                r = ramp value, st = state); "T data" every DAC output sample
//                with +trace=1
//   +stall=<0..100>  AXI3 slave stall probability (%)
`timescale 1ns/1ps
module tb_iqtx_axi;
  logic clk = 0, aresetn = 0;
  always #5 clk = ~clk;
  logic dac_clk = 0;
  always #10.4166 dac_clk = ~dac_clk;          // 48 MHz, one sample per 4: 12 MS/s

  logic [15:0] s_axi_awaddr, s_axi_araddr;
  logic        s_axi_awvalid = 0, s_axi_awready, s_axi_wvalid = 0, s_axi_wready;
  logic [31:0] s_axi_wdata, s_axi_rdata;
  logic [1:0]  s_axi_bresp, s_axi_rresp;
  logic        s_axi_bvalid, s_axi_bready = 1, s_axi_arvalid = 0, s_axi_arready;
  logic        s_axi_rvalid, s_axi_rready = 1;

  logic [31:0] m_axi_araddr;
  logic [3:0]  m_axi_arlen, m_axi_arcache;
  logic [2:0]  m_axi_arsize;
  logic [1:0]  m_axi_arburst, m_axi_rresp = 0;
  logic        m_axi_arvalid, m_axi_arready = 0, m_axi_rlast = 0, m_axi_rvalid = 0, m_axi_rready;
  logic [63:0] m_axi_rdata;
  logic        irq;

  logic        dac_valid = 0;
  logic [15:0] dvb_i = 0, dvb_q = 0, dac_out_i, dac_out_q;

  iqtx_axi #(.FIR_FILE("../rtl/gen/iqtx_fir.mem"), .NCO_FILE("../rtl/gen/iqtx_nco.mem"),
             .NORM_FILE("../rtl/gen/iqtx_norm.mem"), .RAMP_FILE("../rtl/gen/iqtx_ramp.mem")) dut (
    .s_axi_aclk(clk), .s_axi_aresetn(aresetn), .*);

  // ---------------------------------------------------------------- DAC clock
  int dcnt = 0, dvb_mode = 0;
  always @(posedge dac_clk) begin
    dcnt <= (dcnt + 1) % 4;
    dac_valid <= dcnt == 0;
    if (dcnt == 0) begin
      dvb_i <= dvb_mode ? dvb_i + 16'd3 | 16'd1 : 16'd0;
      dvb_q <= dvb_mode ? 16'h0100 : 16'd0;
    end
  end

  // ---------------------------------------------------------------- AXI3 read slave
  int stall;
  logic [31:0] mem [logic [31:0]];
  logic [31:0] arq [$];
  int          beat = 0, lat = 0;
  function automatic logic [31:0] rdw(input logic [31:0] a);
    return mem.exists(a) ? mem[a] : 32'hDEAD_BEEF;
  endfunction
  always @(posedge clk) begin
    m_axi_arready <= ($urandom % 100) >= stall;
    if (m_axi_arvalid && m_axi_arready) begin
      arq.push_back(m_axi_araddr);
      if (m_axi_arlen != 7 || m_axi_arsize != 3 || m_axi_araddr[5:0] != 0) begin
        $display("bad burst %h len %d size %d", m_axi_araddr, m_axi_arlen, m_axi_arsize);
        $finish;
      end
    end
    if (m_axi_rvalid && m_axi_rready) begin
      m_axi_rvalid <= 1'b0;
      if (beat == 7) begin beat <= 0; void'(arq.pop_front()); lat <= 0; end
      else beat <= beat + 1;
    end else if (arq.size() > 0 && !m_axi_rvalid) begin
      if (lat < 20) lat <= lat + 1;
      else if (($urandom % 100) >= stall) begin
        logic [31:0] a;
        a = arq[0] + 8 * beat;
        m_axi_rdata <= {rdw(a + 4), rdw(a)};
        m_axi_rlast <= beat == 7;
        m_axi_rvalid <= 1'b1;
      end
    end
  end

  // ---------------------------------------------------------------- monitors
  int fo, trace;
  always @(posedge dac_clk)
    if (dut.d_pop && dut.d_valid) $fdisplay(fo, "O %h", dut.d_data);
  always @(posedge dac_clk)
    if (trace && dac_valid) $fdisplay(fo, "T %h", {dac_out_q, dac_out_i});
  always @(posedge clk)
    if (dut.ep[3]) $fdisplay(fo, "X %h %h %0d", dut.x0, dut.r1, dut.st);

  // ---------------------------------------------------------------- AXI-Lite
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

  initial begin
    string cmdf, outf, op;
    int fc, n, t;
    logic [31:0] a, d, v;
    if (!$value$plusargs("cmd=%s", cmdf)) $fatal(1, "+cmd");
    if (!$value$plusargs("out=%s", outf)) $fatal(1, "+out");
    if (!$value$plusargs("stall=%d", stall)) stall = 0;
    if (!$value$plusargs("trace=%d", trace)) trace = 0;
    fo = $fopen(outf, "w");
    fc = $fopen(cmdf, "r");
    repeat (20) @(posedge clk);
    aresetn <= 1;
    repeat (40) @(posedge clk);
    while ($fscanf(fc, "%s", op) == 1) begin
      case (op)
        "W": begin n = $fscanf(fc, "%h %h", a, d); axi_write(a[15:0], d); end
        "R": begin n = $fscanf(fc, "%h", a); axi_read(a[15:0], d); $fdisplay(fo, "R %h %h", a, d); end
        "M": begin n = $fscanf(fc, "%h %h", a, d); mem[a] = d; end
        "D": begin n = $fscanf(fc, "%d", t); repeat (t) @(posedge clk); end
        "P": begin
          n = $fscanf(fc, "%h %h", a, v);
          t = 0;
          do begin
            axi_read(a[15:0], d);
            t++;
            if (t > 400000) begin $display("timeout %h = %h", a, d); $fatal(1, "timeout"); end
          end while ($signed(d - v) < 0);
        end
        "V": begin n = $fscanf(fc, "%d", t); dvb_mode = t; end
        "S": begin
          n = $fscanf(fc, "%h %h", a, v);
          t = 0;
          do begin
            axi_read(16'h0010, d);
            t++;
            if (t > 200000) begin $display("timeout STATUS %h", d); $fatal(1, "timeout"); end
          end while ((d & a) != v);
        end
        "E": break;
        default: $fatal(1, "bad command %s", op);
      endcase
    end
    repeat (100) @(posedge clk);
    $fclose(fo);
    $finish;
  end
endmodule
