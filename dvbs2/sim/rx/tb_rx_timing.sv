// File-driven testbench for dvbs2_rx_timing (run by sim/rx/test_rx.py).
//   +coef=<file>  1024 lines of 18-bit hex coefficients ({phase, tap})
//   +in=<file>    input samples, "iiii qqqq" hex per line
//   +out=<file>   output symbols, same format
//   +kp=, +ki=    loop gains; +stall=<0..99> percent of idle input/output cycles
`timescale 1ns/1ps
module tb_rx_timing;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [15:0] kp, ki;
  logic coef_we = 0;
  logic [9:0] coef_addr;
  logic signed [17:0] coef_data;
  logic s_valid = 0, s_ready;
  logic signed [15:0] s_i, s_q;
  logic m_valid, m_ready = 0;
  logic signed [15:0] m_i, m_q;
  logic [16:0] agc_gain;
  logic signed [31:0] loop_integ;

  dvbs2_rx_timing dut (.*);

  string fin, fout, fcoef;
  int fi, fo, fc, stall, n_in, n_out, rc;
  logic [17:0] cv;
  logic [15:0] vi, vq;
  logic eof = 0;

  initial begin
    if (!$value$plusargs("in=%s", fin)) $fatal(1, "+in");
    if (!$value$plusargs("out=%s", fout)) $fatal(1, "+out");
    if (!$value$plusargs("coef=%s", fcoef)) $fatal(1, "+coef");
    if (!$value$plusargs("kp=%d", kp)) kp = 0;
    if (!$value$plusargs("ki=%d", ki)) ki = 0;
    if (!$value$plusargs("stall=%d", stall)) stall = 0;
    fc = $fopen(fcoef, "r");
    fi = $fopen(fin, "r");
    fo = $fopen(fout, "w");
    repeat (4) @(posedge clk);
    for (int a = 0; a < 1024; a++) begin
      rc = $fscanf(fc, "%h", cv);
      coef_we <= 1; coef_addr <= 10'(a); coef_data <= cv;
      @(posedge clk);
    end
    coef_we <= 0;
    rst <= 0;
  end

  // input driver
  always @(posedge clk) begin
    if (!rst) begin
      if (s_valid && s_ready) s_valid <= 0;
      if ((!s_valid || s_ready) && !eof && ($urandom % 100) >= stall) begin
        rc = $fscanf(fi, "%h %h", vi, vq);
        if (rc == 2) begin
          s_valid <= 1; s_i <= vi; s_q <= vq; n_in++;
        end else eof <= 1;
      end
    end
  end

  // output sink
  int idle = 0;
  always @(posedge clk) begin
    m_ready <= ($urandom % 100) >= stall;
    if (m_valid && m_ready) begin
      $fdisplay(fo, "%04h %04h", m_i, m_q);
      n_out++;
      idle = 0;
    end else idle++;
    if (eof && idle > 2000) begin
      $display("tb_rx_timing: %0d samples in, %0d symbols out, gain %0d integ %0d", n_in, n_out, agc_gain, loop_integ);
      $fclose(fo);
      $finish;
    end
  end
endmodule
