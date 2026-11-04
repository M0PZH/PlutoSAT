// File-driven testbench for dvbs2_rx_timing2 (run by sim/rx/test_rx2.py):
// two slots, each from its own files.
//   +in0= +in1=      input samples "iiii qqqq"
//   +out0= +out1=    output symbols "iiii qqqq"; a line "reset" where the slot was reset
//   +coef0= +coef1=  1024 coefficients each ({phase, tap})
//   +kp0= +ki0= +kp1= +ki1= +ia0= +ia1= (integrator preset)
//   +stall0= +stall1= percent of idle input/output cycles per slot
//   +rst<s>at=<n>    reset slot s (rst_s) after n of its samples, during one of
//                    its symbols; its input starts again from the beginning of its file
`timescale 1ns/1ps
module tb_rx_timing2;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [1:0] rst_s = 0;
  logic [15:0] kp [2], ki [2];
  logic signed [31:0] integ_init [2];
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

  string fin [2], fout [2], fcoef [2], nm;
  int fi [2], fo [2], fc, stall [2], n_in [2], n_out [2], rst_at [2], rst_cnt [2], rc, tmp, cyc = 0, busy = 0;
  logic [17:0] cv;
  logic [15:0] vi, vq;
  logic [1:0] eof = 0;

  initial begin
    for (int s = 0; s < 2; s++) begin
      nm = $sformatf("in%0d=%%s", s);    if (!$value$plusargs(nm, fin[s])) $fatal(1, "+in");
      nm = $sformatf("out%0d=%%s", s);   if (!$value$plusargs(nm, fout[s])) $fatal(1, "+out");
      nm = $sformatf("coef%0d=%%s", s);  if (!$value$plusargs(nm, fcoef[s])) $fatal(1, "+coef");
      nm = $sformatf("kp%0d=%%d", s);    if (!$value$plusargs(nm, tmp)) tmp = 0; kp[s] = 16'(tmp);
      nm = $sformatf("ki%0d=%%d", s);    if (!$value$plusargs(nm, tmp)) tmp = 0; ki[s] = 16'(tmp);
      nm = $sformatf("ia%0d=%%d", s);    if (!$value$plusargs(nm, tmp)) tmp = 0; integ_init[s] = 32'(tmp);
      nm = $sformatf("stall%0d=%%d", s); if (!$value$plusargs(nm, stall[s])) stall[s] = 0;
      nm = $sformatf("rst%0dat=%%d", s); if (!$value$plusargs(nm, rst_at[s])) rst_at[s] = -1;
      fi[s] = $fopen(fin[s], "r");
      fo[s] = $fopen(fout[s], "w");
      n_in[s] = 0; n_out[s] = 0; rst_cnt[s] = 0;
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
      if (rst_cnt[s] != 0) begin
        rst_cnt[s]--;
        if (rst_cnt[s] == 0) rst_s[s] <= 0;
      end else if (!rst) begin
        if (s_valid[s] && s_ready[s]) s_valid[s] <= 0;
        // (when the engine is in that slot's MAC phase: the symbol is cancelled)
        if (n_in[s] >= rst_at[s] && rst_at[s] >= 0 && dut.state == 1 && dut.cs == s[0]) begin
          // per-slot reset, the input from the start of the file again
          rst_at[s] = -1;
          rst_s[s] <= 1;
          rst_cnt[s] = 3;
          s_valid[s] <= 0;
          $fdisplay(fo[s], "reset");
          $rewind(fi[s]);
        end else if ((!s_valid[s] || s_ready[s]) && !eof[s] && ($urandom % 100) >= stall[s]) begin
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

  // symbols cancelled by a slot reset (the reset hit that slot's symbol)
  int kills = 0;
  always @(posedge clk) if (dut.state == 12 && dut.cs_kill) kills++;

  int idle = 0;
  always @(posedge clk) begin
    cyc++;
    if (dut.state != 0) busy++;
    if (|(m_valid & m_ready)) idle = 0; else idle++;
    if (eof == 2'b11 && idle > 4000) begin
      $display("tb_rx_timing2: slot0 %0d in %0d out, slot1 %0d in %0d out, %0d clocks, engine busy %0d, cancelled %0d",
               n_in[0], n_out[0], n_in[1], n_out[1], cyc, busy, kills);
      $fclose(fo[0]); $fclose(fo[1]);
      $finish;
    end
  end
endmodule
