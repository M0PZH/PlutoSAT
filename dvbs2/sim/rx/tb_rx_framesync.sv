// File-driven testbench for dvbs2_rx_framesync.
//   +in=<file> symbols "iiii qqqq"; +out=<file> "iiii qqqq sof dummy"
//   +modcod= +pilots= +thresh= +stall=<percent>
`timescale 1ns/1ps
module tb_rx_framesync;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [4:0] modcod;
  logic pilots;
  logic [31:0] thresh;
  logic s_valid = 0, s_ready;
  logic signed [15:0] s_i, s_q;
  logic m_valid, m_ready = 0;
  logic signed [15:0] m_i, m_q;
  logic m_sof, m_dummy, m_found;
  logic ty_v = 0, ty_dummy = 0;
  logic [1:0] fs_state;
  logic [15:0] cnt_found, cnt_flywheel, cnt_lost;

  dvbs2_rx_framesync dut (.*);

  string fin, fout;
  int fi, fo, stall, n_in, n_out, rc, tmp;
  logic [15:0] vi, vq;
  logic eof = 0;

  initial begin
    if (!$value$plusargs("in=%s", fin)) $fatal(1, "+in");
    if (!$value$plusargs("out=%s", fout)) $fatal(1, "+out");
    if (!$value$plusargs("modcod=%d", tmp)) tmp = 4;
    modcod = 5'(tmp);
    if (!$value$plusargs("pilots=%d", tmp)) tmp = 1;
    pilots = tmp[0];
    if (!$value$plusargs("thresh=%d", thresh)) thresh = 0;
    if (!$value$plusargs("stall=%d", stall)) stall = 0;
    fi = $fopen(fin, "r");
    fo = $fopen(fout, "w");
    repeat (4) @(posedge clk);
    rst <= 0;
  end

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

  int idle = 0;
  always @(posedge clk) begin
    m_ready <= ($urandom % 100) >= stall;
    if (m_valid && m_ready) begin
      $fdisplay(fo, "%04h %04h %0d %0d", m_i, m_q, m_sof, m_dummy);
      n_out++;
      idle = 0;
    end else idle++;
    if (eof && idle > 2000) begin
      $display("tb_rx_framesync: %0d in, %0d out, found %0d flywheel %0d lost %0d state %0d",
               n_in, n_out, cnt_found, cnt_flywheel, cnt_lost, fs_state);
      $fclose(fo);
      $finish;
    end
  end
endmodule
