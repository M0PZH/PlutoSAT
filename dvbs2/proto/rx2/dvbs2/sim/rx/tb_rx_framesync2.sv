// PROTOTYPE testbench for dvbs2_rx_framesync2: two slots from their own files
// (+in0/+in1 "iiii qqqq", +out0/+out1 "iiii qqqq sof dummy"), per-slot
// +modcod0.. +pilots0.. +thresh0.., +stall (percent, per-slot input gaps and
// output backpressure). ty_v2 = 0 (no carrier feedback), as tb_rx_framesync.
`timescale 1ns/1ps
module tb_rx_framesync2;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [4:0] modcod2 [2];
  logic [1:0] pilots2;
  logic [31:0] thresh2 [2];
  logic [1:0] s_valid2 = 0, s_ready2;
  logic signed [15:0] s_i2 [2], s_q2 [2];
  logic m_valid, m_ready = 0, m_sof, m_dummy, m_found, m_slot;
  logic signed [15:0] m_i, m_q;
  logic [1:0] ty_v2 = 0;
  logic ty_dummy = 0;
  logic [1:0] fs_state2 [2];
  logic [15:0] cnt_found2 [2], cnt_flywheel2 [2], cnt_lost2 [2];

  dvbs2_rx_framesync2 dut (.*);

  string fin [2], fout [2], nm;
  int fi [2], fo [2], stall, n_in [2], n_out [2], rc, tmp, cyc = 0;
  logic [15:0] vi, vq;
  logic [1:0] eof = 0;

  initial begin
    for (int s = 0; s < 2; s++) begin
      nm = $sformatf("in%0d=%%s", s);  if (!$value$plusargs(nm, fin[s])) $fatal(1, "+in");
      nm = $sformatf("out%0d=%%s", s); if (!$value$plusargs(nm, fout[s])) $fatal(1, "+out");
      nm = $sformatf("modcod%0d=%%d", s); if (!$value$plusargs(nm, tmp)) tmp = 4; modcod2[s] = 5'(tmp);
      nm = $sformatf("pilots%0d=%%d", s); if (!$value$plusargs(nm, tmp)) tmp = 1; pilots2[s] = tmp[0];
      nm = $sformatf("thresh%0d=%%d", s); if (!$value$plusargs(nm, tmp)) tmp = 0; thresh2[s] = 32'(tmp);
      fi[s] = $fopen(fin[s], "r");
      fo[s] = $fopen(fout[s], "w");
      n_in[s] = 0; n_out[s] = 0;
    end
    if (!$value$plusargs("stall=%d", stall)) stall = 0;
    repeat (4) @(posedge clk);
    rst = 0;
  end

  for (genvar s = 0; s < 2; s++) begin : g_drv
    always @(posedge clk) begin
      if (!rst) begin
        if (s_valid2[s] && s_ready2[s]) s_valid2[s] <= 0;
        if ((!s_valid2[s] || s_ready2[s]) && !eof[s] && ($urandom % 100) >= stall) begin
          rc = $fscanf(fi[s], "%h %h", vi, vq);
          if (rc == 2) begin
            s_valid2[s] <= 1; s_i2[s] <= vi; s_q2[s] <= vq; n_in[s]++;
          end else eof[s] <= 1;
        end
      end
    end
  end

  int idle = 0;
  always @(posedge clk) begin
    cyc++;
    m_ready <= ($urandom % 100) >= stall;
    if (m_valid && m_ready) begin
      $fdisplay(fo[m_slot], "%04h %04h %0d %0d", m_i, m_q, m_sof, m_dummy);
      n_out[m_slot]++;
      idle = 0;
    end else idle++;
    if (eof == 2'b11 && idle > 4000) begin
      for (int s = 0; s < 2; s++)
        $display("slot %0d: %0d in, %0d out, found %0d flywheel %0d lost %0d", s, n_in[s], n_out[s],
                 cnt_found2[s], cnt_flywheel2[s], cnt_lost2[s]);
      $display("clocks %0d", cyc);
      $fclose(fo[0]); $fclose(fo[1]);
      $finish;
    end
  end
endmodule
