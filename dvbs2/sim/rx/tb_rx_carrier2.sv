// File-driven testbench for dvbs2_rx_carrier2 (run by sim/rx/test_rx2.py):
// two slots from their own files.
//   +in0= +in1=   "iiii qqqq sof dummy found"
//   +out0= +out1= "ll qq first last"; a line "reset" where the slot was reset
//   +modcod0= +pilots0= +xa0= +xb0= +kp0= +ki0= (and slot 1)
//   +stall0= +stall1= percent of idle input cycles per slot; +ostall= output
//   +rst<s>at=<n>  reset slot s after n of its symbols; its input starts again
// At the end, per slot: "slot s: .. frames .. freq .. amp .. energy .. gain .. acq .. fa .. retype .."
`timescale 1ns/1ps
module tb_rx_carrier2;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [1:0] rst_s = 0;
  logic [4:0] modcod2 [2];
  logic [1:0] pilots2;
  logic [17:0] gold_xa2 [2], gold_xb2 [2], pll_kp2 [2], pll_ki2 [2];
  logic [1:0] s_valid2 = 0, s_ready2, s_sof2, s_dummy2, s_found2;
  logic signed [15:0] s_i2 [2], s_q2 [2];
  logic [1:0] ty_v2;
  logic ty_dummy;
  logic m_valid, m_ready = 0, m_first, m_last, m_slot;
  logic [7:0] m_llr_i, m_llr_q;
  logic [31:0] st_freq2 [2], st_amp2 [2];
  logic [35:0] st_energy2 [2];
  logic [17:0] st_gain2 [2];
  logic [15:0] st_frames2 [2], st_acq2 [2], st_fa2 [2], st_retype2 [2];
  logic [1:0] st_acquired2;
  logic snap_v, snap_slot;
  logic signed [15:0] snap_i, snap_q;

  dvbs2_rx_carrier2 #(.LUT_FILE("../../rtl/gen/nco_lut.mem")) dut (.*);

  string fin [2], fout [2], nm;
  int fi [2], fo [2], stall [2], ostall, n_in [2], n_out [2], rst_at [2], rst_cnt [2], rc, tmp, a, b, c, cyc = 0;
  logic [15:0] vi, vq;
  logic [1:0] eof = 0;

  initial begin
    for (int s = 0; s < 2; s++) begin
      nm = $sformatf("in%0d=%%s", s);     if (!$value$plusargs(nm, fin[s])) $fatal(1, "+in");
      nm = $sformatf("out%0d=%%s", s);    if (!$value$plusargs(nm, fout[s])) $fatal(1, "+out");
      nm = $sformatf("modcod%0d=%%d", s); if (!$value$plusargs(nm, tmp)) tmp = 4; modcod2[s] = 5'(tmp);
      nm = $sformatf("pilots%0d=%%d", s); if (!$value$plusargs(nm, tmp)) tmp = 1; pilots2[s] = tmp[0];
      nm = $sformatf("xa%0d=%%d", s);     if (!$value$plusargs(nm, tmp)) tmp = 1; gold_xa2[s] = 18'(tmp);
      nm = $sformatf("xb%0d=%%d", s);     if (!$value$plusargs(nm, tmp)) tmp = 0; gold_xb2[s] = 18'(tmp);
      nm = $sformatf("kp%0d=%%d", s);     if (!$value$plusargs(nm, tmp)) tmp = 0; pll_kp2[s] = 18'(tmp);
      nm = $sformatf("ki%0d=%%d", s);     if (!$value$plusargs(nm, tmp)) tmp = 0; pll_ki2[s] = 18'(tmp);
      nm = $sformatf("stall%0d=%%d", s);  if (!$value$plusargs(nm, stall[s])) stall[s] = 0;
      nm = $sformatf("rst%0dat=%%d", s);  if (!$value$plusargs(nm, rst_at[s])) rst_at[s] = -1;
      fi[s] = $fopen(fin[s], "r");
      fo[s] = $fopen(fout[s], "w");
      n_in[s] = 0; n_out[s] = 0; rst_cnt[s] = 0;
    end
    if (!$value$plusargs("ostall=%d", ostall)) ostall = 0;
    repeat (4) @(posedge clk);
    rst <= 0;
  end

  for (genvar s = 0; s < 2; s++) begin : g_drv
    always @(posedge clk) begin
      if (rst_cnt[s] != 0) begin
        rst_cnt[s]--;
        if (rst_cnt[s] == 0) rst_s[s] <= 0;
      end else if (!rst) begin
        if (s_valid2[s] && s_ready2[s]) s_valid2[s] <= 0;
        if (n_in[s] >= rst_at[s] && rst_at[s] >= 0) begin
          rst_at[s] = -1;
          rst_s[s] <= 1;
          rst_cnt[s] = 2;
          s_valid2[s] <= 0;
          $fdisplay(fo[s], "reset");
          $rewind(fi[s]);
        end else if ((!s_valid2[s] || s_ready2[s]) && !eof[s] && ($urandom % 100) >= stall[s]) begin
          rc = $fscanf(fi[s], "%h %h %d %d %d", vi, vq, a, b, c);
          if (rc == 5) begin
            s_valid2[s] <= 1; s_i2[s] <= vi; s_q2[s] <= vq;
            s_sof2[s] <= a[0]; s_dummy2[s] <= b[0]; s_found2[s] <= c[0]; n_in[s]++;
          end else eof[s] <= 1;
        end
      end
    end
  end

  int idle = 0, swaps = 0, dropped = 0, pend = 0;
  always @(posedge clk) begin
    cyc++;
    if (dut.rp != 0) pend++;            // a slot reset waiting for the end of a job
    if (dut.ctx_swap) swaps++;
    if (dut.stale && dut.of_cnt != 0 && (!m_valid || m_ready)) dropped++;
    m_ready <= ($urandom % 100) >= ostall;
    if (m_valid && m_ready) begin
      $fdisplay(fo[m_slot], "%02h %02h %0d %0d", m_llr_i, m_llr_q, m_first, m_last);
      n_out[m_slot]++;
      idle = 0;
    end else idle++;
    if (eof == 2'b11 && idle > 20000) begin
      for (int s = 0; s < 2; s++)
        $display("slot %0d: %0d in, %0d out, frames %0d freq %0d amp %0d energy %0d gain %0d acq %0d fa %0d retype %0d acquired %0d",
                 s, n_in[s], n_out[s], st_frames2[s], $signed(st_freq2[s]), st_amp2[s], st_energy2[s], st_gain2[s],
                 st_acq2[s], st_fa2[s], st_retype2[s], st_acquired2[s]);
      $display("tb_rx_carrier2: clocks %0d swaps %0d dropped %0d reset pending %0d", cyc, swaps, dropped, pend);
      $fclose(fo[0]); $fclose(fo[1]);
      $finish;
    end
  end
endmodule
