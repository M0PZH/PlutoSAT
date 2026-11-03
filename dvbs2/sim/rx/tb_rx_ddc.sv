// File-driven testbench for dvbs2_rx_ddc.
//   +coef=<file> (ntaps lines, 18-bit hex) +in= +out= +dec= +ntaps= +shift= +finc= +period=
`timescale 1ns/1ps
module tb_rx_ddc;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [31:0] finc;
  logic [5:0]  dec;
  logic [9:0]  ntaps;
  logic [4:0]  shift;
  logic coef_we = 0;
  logic [8:0] coef_addr;
  logic signed [17:0] coef_data;
  logic s_valid = 0, s_ready;
  logic signed [15:0] s_i, s_q, m_i, m_q;
  logic m_valid;
  logic [15:0] st_dropped;

  dvbs2_rx_ddc #(.LUT_FILE("../../rtl/gen/nco_lut.mem")) dut (.*);

  string fin, fout, fcoef;
  int fi, fo, fc, period, rc, tmp, n_in, n_out;
  logic [17:0] cv;
  logic [15:0] vi, vq;
  logic eof = 0;
  longint f64;

  initial begin
    if (!$value$plusargs("in=%s", fin)) $fatal(1, "+in");
    if (!$value$plusargs("out=%s", fout)) $fatal(1, "+out");
    if (!$value$plusargs("coef=%s", fcoef)) $fatal(1, "+coef");
    if (!$value$plusargs("dec=%d", tmp)) tmp = 1;
    dec = 6'(tmp);
    if (!$value$plusargs("ntaps=%d", tmp)) tmp = 2;
    ntaps = 10'(tmp);
    if (!$value$plusargs("shift=%d", tmp)) tmp = 16;
    shift = 5'(tmp);
    if (!$value$plusargs("finc=%d", f64)) f64 = 0;
    finc = 32'(f64);
    if (!$value$plusargs("period=%d", period)) period = 8;
    fc = $fopen(fcoef, "r");
    fi = $fopen(fin, "r");
    fo = $fopen(fout, "w");
    repeat (4) @(posedge clk);
    for (int a = 0; a < int'(ntaps); a++) begin
      rc = $fscanf(fc, "%h", cv);
      coef_we <= 1; coef_addr <= 9'(a); coef_data <= cv;
      @(posedge clk);
    end
    coef_we <= 0;
    rst <= 0;
  end

  int ph = 0;
  always @(posedge clk) begin
    s_valid <= 0;
    if (!rst && !eof) begin
      if (ph == 0) begin
        rc = $fscanf(fi, "%h %h", vi, vq);
        if (rc == 2) begin s_valid <= 1; s_i <= vi; s_q <= vq; n_in++; end
        else eof <= 1;
      end
      ph = (ph + 1 == period) ? 0 : ph + 1;
    end
  end

  int idle = 0;
  always @(posedge clk) begin
    if (m_valid) begin
      $fdisplay(fo, "%04h %04h", m_i, m_q);
      n_out++;
      idle = 0;
    end else idle++;
    if (eof && idle > 5000) begin
      $display("tb_rx_ddc: %0d in, %0d out, dropped %0d", n_in, n_out, st_dropped);
      $fclose(fo);
      $finish;
    end
  end
endmodule
