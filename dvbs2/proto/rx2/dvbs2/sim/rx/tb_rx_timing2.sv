// PROTOTYPE testbench for dvbs2_rx_timing2: two slots, each from its own
// files (+in0/+in1, +coef0/+coef1, +out0/+out1, +kp0.. +ki1), random
// per-slot input gaps and output stalls (+stall0, +stall1, percent).
`timescale 1ns/1ps
module tb_rx_timing2;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [1:0] rst_s = 0;
  logic [15:0] kp [2], ki [2];
  logic coef_we = 0, coef_slot = 0;
  logic [9:0] coef_addr;
  logic signed [17:0] coef_data;
  logic [1:0] s_valid = 0, s_ready;
  logic signed [15:0] s_i [2], s_q [2];
  logic [1:0] m_valid, m_ready = 0;
  logic signed [15:0] m_i [2], m_q [2];
  logic [16:0] agc_gain [2];
  logic signed [31:0] loop_integ [2];

  dvbs2_rx_timing2 dut (.*);

  string fin [2], fout [2], fcoef [2];
  int fi [2], fo [2], fc, stall [2], n_in [2], n_out [2], rc, cyc = 0, busy = 0;
  logic [17:0] cv;
  logic [15:0] vi, vq;
  logic [1:0] eof = 0;

  initial begin
    if (!$value$plusargs("in0=%s", fin[0])) $fatal(1, "+in0");
    if (!$value$plusargs("in1=%s", fin[1])) $fatal(1, "+in1");
    if (!$value$plusargs("out0=%s", fout[0])) $fatal(1, "+out0");
    if (!$value$plusargs("out1=%s", fout[1])) $fatal(1, "+out1");
    if (!$value$plusargs("coef0=%s", fcoef[0])) $fatal(1, "+coef0");
    if (!$value$plusargs("coef1=%s", fcoef[1])) $fatal(1, "+coef1");
    if (!$value$plusargs("kp0=%d", kp[0])) kp[0] = 0;
    if (!$value$plusargs("ki0=%d", ki[0])) ki[0] = 0;
    if (!$value$plusargs("kp1=%d", kp[1])) kp[1] = 0;
    if (!$value$plusargs("ki1=%d", ki[1])) ki[1] = 0;
    if (!$value$plusargs("stall0=%d", stall[0])) stall[0] = 0;
    if (!$value$plusargs("stall1=%d", stall[1])) stall[1] = 0;
    for (int s = 0; s < 2; s++) begin
      fi[s] = $fopen(fin[s], "r");
      fo[s] = $fopen(fout[s], "w");
      n_in[s] = 0; n_out[s] = 0;
    end
    repeat (4) @(posedge clk);
    for (int s = 0; s < 2; s++) begin
      fc = $fopen(fcoef[s], "r");
      for (int a = 0; a < 1024; a++) begin
        rc = $fscanf(fc, "%h", cv);
        coef_we <= 1; coef_slot <= s[0]; coef_addr <= 10'(a); coef_data <= cv;
        @(posedge clk);
      end
      $fclose(fc);
    end
    coef_we <= 0;
    rst <= 0;
  end

  for (genvar s = 0; s < 2; s++) begin : g_drv
    always @(posedge clk) begin
      if (!rst) begin
        if (s_valid[s] && s_ready[s]) s_valid[s] <= 0;
        if ((!s_valid[s] || s_ready[s]) && !eof[s] && ($urandom % 100) >= stall[s]) begin
          rc = $fscanf(fi[s], "%h %h", vi, vq);
          if (rc == 2) begin
            s_valid[s] <= 1; s_i[s] <= vi; s_q[s] <= vq; n_in[s]++;
          end else eof[s] <= 1;
        end
      end
    end
    always @(posedge clk) begin
      m_ready[s] <= ($urandom % 100) >= stall[s];
      if (m_valid[s] && m_ready[s]) begin
        $fdisplay(fo[s], "%04h %04h", m_i[s], m_q[s]);
        n_out[s]++;
      end
    end
  end

  int idle = 0;
  always @(posedge clk) begin
    cyc++;
    if (dut.state != 0) busy++;
    if (|(m_valid & m_ready)) idle = 0; else idle++;
    if (eof == 2'b11 && idle > 4000) begin
      $display("tb_rx_timing2: slot0 %0d in %0d out, slot1 %0d in %0d out, %0d clocks, engine busy %0d",
               n_in[0], n_out[0], n_in[1], n_out[1], cyc, busy);
      $fclose(fo[0]); $fclose(fo[1]);
      $finish;
    end
  end
endmodule
