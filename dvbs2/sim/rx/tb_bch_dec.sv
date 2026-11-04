// File-driven testbench for dvbs2_bch_dec.
//   +in=<file> "dd last ok iters chan rate tag" (Kldpc/8 bytes per frame);
//   +out=<file> "dd last ok ldpc_ok iters nerr raw rate tag"; +stall=<percent>
`timescale 1ns/1ps
module tb_bch_dec;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [3:0] s_rate, m_rate;
  logic [7:0] s_tag, m_tag;
  logic s_valid = 0, s_ready, s_last, s_ok;
  logic [7:0] s_data, s_chan;
  logic [15:0] m_raw_err;
  logic [5:0] s_iters;
  logic m_valid, m_ready = 0, m_last, m_ok, m_ldpc_ok;
  logic [7:0] m_data;
  logic [5:0] m_iters;
  logic [4:0] m_nerr;
  logic [15:0] st_frames, st_corrected, st_failed;

  dvbs2_bch_dec dut (.*);

  string fin, fout;
  int fi, fo, stall, rc, tmp, n_in, n_out, a, b, c, d, e, ra, tg;
  logic eof = 0;

  initial begin
    if (!$value$plusargs("in=%s", fin)) $fatal(1, "+in");
    if (!$value$plusargs("out=%s", fout)) $fatal(1, "+out");
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
        rc = $fscanf(fi, "%h %d %d %d %h %d %d", a, b, c, d, e, ra, tg);
        if (rc == 7) begin
          s_valid <= 1; s_data <= 8'(a); s_last <= b[0]; s_ok <= c[0]; s_iters <= 6'(d); s_chan <= 8'(e); n_in++;
          s_rate <= 4'(ra); s_tag <= 8'(tg);
        end else eof <= 1;
      end
    end
  end

  int idle = 0;
  always @(posedge clk) begin
    m_ready <= ($urandom % 100) >= stall;
    if (m_valid && m_ready) begin
      $fdisplay(fo, "%02h %0d %0d %0d %02h %02h %04h %0h %02h", m_data, m_last, m_ok, m_ldpc_ok, m_iters, m_nerr, m_raw_err,
                m_rate, m_tag);
      n_out++;
      idle = 0;
    end else idle++;
    if (eof && idle > 100000) begin
      $display("tb_bch_dec: %0d in, %0d out, frames %0d corrected %0d failed %0d",
               n_in, n_out, st_frames, st_corrected, st_failed);
      $fclose(fo);
      $finish;
    end
  end
endmodule
