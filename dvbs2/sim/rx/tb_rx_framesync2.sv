// File-driven testbench for dvbs2_rx_framesync2 (run by sim/rx/test_rx2.py):
// two slots from their own files.
//   +in0= +in1=   symbols "iiii qqqq"
//   +out0= +out1= "iiii qqqq sof dummy"; a line "reset" where the slot was reset
//   +modcod0= +pilots0= +thresh0= (and slot 1), +stall0= +stall1= percent
//   (per-slot input gaps; the output backpressure uses slot 0's)
//   +plsbusy0= +plscode0= +plsconf0= (and slot 1): a PLS decoder stand-in per
//   slot as in tb_rx_framesync (0: none)
//   +rst<s>at=<n>  reset slot s after n of its symbols; its input starts again
//   ty_v2 = 0 (no carrier feedback), as tb_rx_framesync
`timescale 1ns/1ps
module tb_rx_framesync2;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [1:0] rst_s = 0;
  logic [4:0] modcod2 [2];
  logic [1:0] pilots2;
  logic [31:0] thresh2 [2];
  logic [1:0] s_valid2 = 0, s_ready2;
  logic signed [15:0] s_i2 [2], s_q2 [2];
  logic m_valid, m_ready = 0, m_sof, m_dummy, m_found, m_slot;
  logic signed [15:0] m_i, m_q;
  logic [1:0] ty_v2 = 0;
  logic ty_dummy = 0;
  logic [1:0] pls_v2 = 0, pls_conf2 = 0, pls_busy2 = 0;
  logic [6:0] pls_code2 [2];
  logic [1:0] fs_state2 [2];
  logic [15:0] cnt_found2 [2], cnt_flywheel2 [2], cnt_lost2 [2];

  dvbs2_rx_framesync2 dut (.*);

  string fin [2], fout [2], nm;
  int fi [2], fo [2], stall [2], n_in [2], n_out [2], rst_at [2], rst_cnt [2], rc, tmp, cyc = 0;
  int pls_clk [2], pls_code_a [2], pls_conf_a [2], pls_k [2], pls_left [2];
  logic [15:0] vi, vq;
  logic [1:0] eof = 0;

  initial begin
    for (int s = 0; s < 2; s++) begin
      nm = $sformatf("in%0d=%%s", s);     if (!$value$plusargs(nm, fin[s])) $fatal(1, "+in");
      nm = $sformatf("out%0d=%%s", s);    if (!$value$plusargs(nm, fout[s])) $fatal(1, "+out");
      nm = $sformatf("modcod%0d=%%d", s); if (!$value$plusargs(nm, tmp)) tmp = 4; modcod2[s] = 5'(tmp);
      nm = $sformatf("pilots%0d=%%d", s); if (!$value$plusargs(nm, tmp)) tmp = 1; pilots2[s] = tmp[0];
      nm = $sformatf("thresh%0d=%%d", s); if (!$value$plusargs(nm, tmp)) tmp = 0; thresh2[s] = 32'(tmp);
      nm = $sformatf("stall%0d=%%d", s);  if (!$value$plusargs(nm, stall[s])) stall[s] = 0;
      nm = $sformatf("plsbusy%0d=%%d", s); if (!$value$plusargs(nm, pls_clk[s])) pls_clk[s] = 0;
      nm = $sformatf("plscode%0d=%%d", s); if (!$value$plusargs(nm, pls_code_a[s])) pls_code_a[s] = 0;
      nm = $sformatf("plsconf%0d=%%d", s); if (!$value$plusargs(nm, pls_conf_a[s])) pls_conf_a[s] = 0;
      nm = $sformatf("rst%0dat=%%d", s);  if (!$value$plusargs(nm, rst_at[s])) rst_at[s] = -1;
      fi[s] = $fopen(fin[s], "r");
      fo[s] = $fopen(fout[s], "w");
      n_in[s] = 0; n_out[s] = 0; rst_cnt[s] = 0; pls_k[s] = -1; pls_left[s] = 0;
      pls_code2[s] = '0;
    end
    repeat (4) @(posedge clk);
    rst = 0;
  end

  for (genvar s = 0; s < 2; s++) begin : g_drv
    always @(posedge clk) begin
      if (rst_cnt[s] != 0) begin
        rst_cnt[s]--;
        if (rst_cnt[s] == 0) rst_s[s] <= 0;
      end else if (!rst) begin
        if (s_valid2[s] && s_ready2[s]) s_valid2[s] <= 0;
        if (n_in[s] >= rst_at[s] && rst_at[s] >= 0) begin
          // per-slot reset (the testbench's PLS stand-in of the slot too), the input again
          rst_at[s] = -1;
          rst_s[s] <= 1;
          rst_cnt[s] = 2;
          s_valid2[s] <= 0;
          pls_k[s] = -1;
          $fdisplay(fo[s], "reset");
          $rewind(fi[s]);
        end else if ((!s_valid2[s] || s_ready2[s]) && !eof[s] && ($urandom % 100) >= stall[s]) begin
          rc = $fscanf(fi[s], "%h %h", vi, vq);
          if (rc == 2) begin
            s_valid2[s] <= 1; s_i2[s] <= vi; s_q2[s] <= vq; n_in[s]++;
          end else eof[s] <= 1;
        end
      end
    end

    // PLS decoder stand-in (busy falls in the clock res_v rises, as in dvbs2_rx_pls)
    always @(posedge clk) begin
      pls_v2[s] <= 0;
      if (rst_s[s]) begin
        pls_busy2[s] <= 0;
        pls_k[s] = -1;
      end else begin
        if (pls_clk[s] != 0 && m_valid && m_ready && m_slot == s[0]) begin
          if (m_sof) pls_k[s] = 0;
          else if (pls_k[s] >= 0) pls_k[s]++;
          if (pls_k[s] == 89) begin
            pls_k[s] = -1;
            pls_busy2[s] <= 1;
            pls_left[s] = pls_clk[s];
          end
        end
        if (pls_busy2[s]) begin
          if (pls_left[s] == 0) begin
            pls_busy2[s] <= 0;
            pls_v2[s] <= 1;
            pls_code2[s] <= 7'(pls_code_a[s]);
            pls_conf2[s] <= pls_conf_a[s][0];
          end else pls_left[s]--;
        end
      end
    end
  end

  int idle = 0, swaps = 0, holds = 0;
  always @(posedge clk) begin
    cyc++;
    if (dut.ctx_swap) swaps++;
    if (dut.pls_hold && dut.st == 0) holds++;
    m_ready <= ($urandom % 100) >= stall[0];
    if (m_valid && m_ready) begin
      $fdisplay(fo[m_slot], "%04h %04h %0d %0d", m_i, m_q, m_sof, m_dummy);
      n_out[m_slot]++;
      idle = 0;
    end else idle++;
    if (eof == 2'b11 && idle > 4000) begin
      for (int s = 0; s < 2; s++)
        $display("slot %0d: %0d in, %0d out, found %0d flywheel %0d lost %0d state %0d", s, n_in[s], n_out[s],
                 cnt_found2[s], cnt_flywheel2[s], cnt_lost2[s], fs_state2[s]);
      $display("tb_rx_framesync2: clocks %0d swaps %0d hold %0d", cyc, swaps, holds);
      $fclose(fo[0]); $fclose(fo[1]);
      $finish;
    end
  end
endmodule
