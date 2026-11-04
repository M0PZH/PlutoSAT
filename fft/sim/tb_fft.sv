// xsim testbench for fft_axi (driven by test_fft.py through files).
//
// +DIR=<dir>  holds the inputs:
//   adc.hex   one ADC sample per line: {q1, i1, q0, i0} (64-bit hex); one is
//             played per adc_valid (adc_clk 250 MHz, valid at 12 MSPS:
//             12 strobes per 250 clocks, evenly spread)
//   tap.hex   one nbiq tap beat per s_axi_aclk clock: {valid, ch[2:0], i, q}
//             (36-bit hex), looped; with +TAP_DIV=d one beat every d clocks
//             (idle in between)
//   cmd.txt   commands, one per line:
//               W addr data        AXI write
//               R addr             AXI read, logged
//               P addr mask value  poll until (read & mask) == value
//               I                  wait for irq
//               C n                wait n clocks
//               D n                read the first n result words, logged
//               K p c              m_axis sink: tready with probability p %
//                                  per clock, for c more packets (-1: no limit;
//                                  0: tready low); reset: K 100 -1
//               Q                  end
// and gets ev.txt: the event log (DUT probes and command results):
//   G t src ch nm1 scale rsh len  an average starts (configuration taken)
//   F t                         first sample of a frame enters the window
//   S i q                       a captured sample (hex)
//   E t pub                     an average ends (pub: result published)
//   X t                         abort
//   W t                         adc.hex wrapped around
//   R addr data, D t (then n lines w data, other events may come
//   in between), I t, Q t
//   A t last data               an accepted m_axis beat (64-bit hex)
//   B t                         CMD.STR_ABORT
//   H n                         clocks the FFT output was held (before Q)
// The sink checks the AXI4-Stream rules (tvalid held, tdata / tlast
// stable until accepted) except right after CMD.STR_ABORT.
`timescale 1ns/1ps

module tb_fft;
  parameter bit    DOUBLE_BUF = 1'b0;
  parameter string WIN_FILE   = "fft_win.mem";

  logic clk = 1'b0, adc_clk = 1'b0, aresetn = 1'b0;
  always #5 clk = ~clk;
  always #2 adc_clk = ~adc_clk;

  logic [15:0] awaddr, araddr;
  logic        awvalid, awready, wvalid, wready, bvalid, bready;
  logic        arvalid, arready, rvalid, rready;
  logic [31:0] wdata, rdata;
  logic [1:0]  bresp, rresp;
  logic        irq;

  logic        adc_valid = 1'b0;
  logic [15:0] i0 = '0, q0 = '0, i1 = '0, q1 = '0;
  logic        tap_valid = 1'b0;
  logic [2:0]  tap_ch = '0;
  logic [15:0] tap_i = '0, tap_q = '0;
  logic [63:0] m_tdata;
  logic        m_tvalid, m_tready = 1'b0, m_tlast;

  fft_axi #(.WIN_FILE(WIN_FILE), .DOUBLE_BUF(DOUBLE_BUF)) dut (
    .s_axi_aclk(clk), .s_axi_aresetn(aresetn),
    .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
    .s_axi_wdata(wdata), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
    .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
    .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
    .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
    .irq,
    .adc_clk, .adc_valid, .adc_data_i0(i0), .adc_data_q0(q0), .adc_data_i1(i1), .adc_data_q1(q1),
    .tap_valid, .tap_ch, .tap_i, .tap_q,
    .m_axis_tdata(m_tdata), .m_axis_tvalid(m_tvalid), .m_axis_tready(m_tready),
    .m_axis_tlast(m_tlast));

  string dir;
  integer ev;
  localparam int ADC_MAX = 1 << 20, TAP_MAX = 1 << 20;
  logic [63:0] adc_mem [ADC_MAX];
  logic [35:0] tap_mem [TAP_MAX];
  int adc_n, tap_n;

  // ---------------------------------------------------------------- stimulus
  int ph = 0, am = 0;
  always @(posedge adc_clk) begin
    adc_valid <= 1'b0;
    if (aresetn && adc_n > 0) begin
      ph = ph + 12;
      if (ph >= 250) begin
        ph = ph - 250;
        if (am >= adc_n) begin am = 0; $fdisplay(ev, "W %0d", $time); end   // wrapped
        adc_valid <= 1'b1;
        {q1, i1, q0, i0} <= adc_mem[am];
        am = am + 1;
      end
    end
  end

  int tm = 0, tdiv = 1, tph = 0;
  always @(posedge clk) begin
    if (tph == 0) begin
      {tap_valid, tap_ch, tap_i, tap_q} <= tap_mem[tm];
      tm = (tm + 1 == tap_n) ? 0 : tm + 1;
    end else
      tap_valid <= 1'b0;
    tph = (tph + 1 == tdiv) ? 0 : tph + 1;
  end

  // ---------------------------------------------------------------- m_axis sink
  int k_p = 100, k_c = -1;
  logic        pv = 1'b0, pr = 1'b0, pl, sab = 1'b0;
  logic [63:0] pd;
  always @(posedge clk) begin
    if (m_tvalid && m_tready) begin
      $fdisplay(ev, "A %0d %0d %016x", $time, m_tlast, m_tdata);
      if (m_tlast && k_c > 0) k_c = k_c - 1;
    end
    if (dut.cmd_sabort) $fdisplay(ev, "B %0d", $time);      // after a beat taken now
    if (pv && !pr && !sab && (!m_tvalid || m_tdata !== pd || m_tlast !== pl)) begin
      $display("FATAL: m_axis beat changed before it was accepted at %0t", $time);
      $finish;
    end
    pv <= m_tvalid; pr <= m_tready; pd <= m_tdata; pl <= m_tlast;
    sab <= dut.cmd_sabort;
    m_tready <= k_c != 0 && ($urandom % 100) < k_p;
  end

  // ---------------------------------------------------------------- probes
  logic go_d = 1'b0;
  int holds = 0;
  always @(posedge clk) begin
    go_d <= dut.go && !dut.srst;
    if (go_d)
      $fdisplay(ev, "G %0d %0d %0d %0d %06x %0d %0d", $time, dut.a_src, dut.a_ch, dut.a_nm1, dut.a_scale,
                dut.a_rsh, dut.a_len);
    if (dut.p_v) begin
      if (dut.p_n == 0) $fdisplay(ev, "F %0d", $time);
      $fdisplay(ev, "S %04x %04x", dut.p_i, dut.p_q);
    end
    if (dut.avg_end && !dut.srst) $fdisplay(ev, "E %0d %0d", $time, dut.avg_pub);
    if (dut.cmd_abort) $fdisplay(ev, "X %0d", $time);
    if (dut.m_v && !dut.m_rdy) holds++;
  end

  // ---------------------------------------------------------------- AXI master
  task automatic axi_w(input logic [15:0] a, input logic [31:0] d);
    @(posedge clk);
    awaddr <= a; awvalid <= 1'b1; wdata <= d; wvalid <= 1'b1;
    fork
      begin do @(posedge clk); while (!awready); awvalid <= 1'b0; end
      begin do @(posedge clk); while (!wready); wvalid <= 1'b0; end
    join
    do @(posedge clk); while (!bvalid);
  endtask

  task automatic axi_r(input logic [15:0] a, output logic [31:0] d);
    @(posedge clk);
    araddr <= a; arvalid <= 1'b1;
    do @(posedge clk); while (!arready);
    arvalid <= 1'b0;
    do @(posedge clk); while (!rvalid);
    d = rdata;
  endtask

  assign bready = 1'b1;
  assign rready = 1'b1;

  // ---------------------------------------------------------------- commands
  initial begin
    integer cf, r;
    string op;
    logic [31:0] a, d, m, v;
    int n;
    awaddr = '0; awvalid = 1'b0; wdata = '0; wvalid = 1'b0; araddr = '0; arvalid = 1'b0;
    if (!$value$plusargs("DIR=%s", dir)) dir = ".";
    if (!$value$plusargs("ADC_N=%d", adc_n)) adc_n = 0;
    if (!$value$plusargs("TAP_N=%d", tap_n)) tap_n = 1;
    if (!$value$plusargs("TAP_DIV=%d", tdiv)) tdiv = 1;
    for (int k = 0; k < TAP_MAX; k++) tap_mem[k] = '0;
    if (adc_n > 0) $readmemh({dir, "/adc.hex"}, adc_mem, 0, adc_n - 1);
    if (tap_n > 1) $readmemh({dir, "/tap.hex"}, tap_mem, 0, tap_n - 1);
    ev = $fopen({dir, "/ev.txt"}, "w");
    cf = $fopen({dir, "/cmd.txt"}, "r");
    repeat (20) @(posedge clk);
    aresetn <= 1'b1;
    repeat (40) @(posedge clk);
    forever begin
      r = $fscanf(cf, "%s", op);
      if (r != 1 || op == "Q") break;
      case (op)
        "W": begin r = $fscanf(cf, "%h %h", a, d); axi_w(a[15:0], d); end
        "R": begin r = $fscanf(cf, "%h", a); axi_r(a[15:0], d); $fdisplay(ev, "R %04x %08x", a[15:0], d); end
        "P": begin
          r = $fscanf(cf, "%h %h %h", a, m, v);
          n = 0;
          do begin
            axi_r(a[15:0], d);
            if (++n > 2000000) begin $display("FATAL: poll %h timeout", a); $finish; end
          end while ((d & m) != v);
        end
        "I": begin
          repeat (8) @(posedge clk);       // a preceding DONE clear reaches irq
          n = 0;
          while (!irq) begin
            @(posedge clk);
            if (++n > 50000000) begin $display("FATAL: irq timeout"); $finish; end
          end
          $fdisplay(ev, "I %0d", $time);
        end
        "C": begin r = $fscanf(cf, "%d", n); repeat (n) @(posedge clk); end
        "K": begin r = $fscanf(cf, "%d %d", k_p, k_c); end
        "D": begin
          r = $fscanf(cf, "%d", n);
          $fdisplay(ev, "D %0d", $time);
          for (int j = 0; j < n; j++) begin
            axi_r(16'h4000 + 16'(4 * j), d);
            $fdisplay(ev, "w %08x", d);
          end
        end
        default: begin $display("FATAL: bad command %s", op); $finish; end
      endcase
    end
    $fdisplay(ev, "H %0d", holds);
    $fdisplay(ev, "Q %0d", $time);
    $fclose(ev);
    $display("DONE at %0t", $time);
    $finish;
  end

endmodule
