// File-driven testbench for dvbs2_ldpc_dec.
//   +in=<file> "ii qq first" per word; +out=<file> "dd last ok iters"
//   +rate= +maxit= +stall=<output %> +gap=<input idle clocks per word>
`timescale 1ns/1ps
module tb_ldpc_dec;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [3:0] rate_idx;
  logic [5:0] max_it;
  logic s_valid = 0, s_ready, s_first;
  logic [7:0] s_llr_i, s_llr_q;
  logic m_valid, m_ready = 0, m_last, m_ok;
  logic [7:0] m_data;
  logic [5:0] m_iters;
  logic [7:0] m_chan;
  logic [15:0] st_frames, st_failed;

  dvbs2_ldpc_dec #(.ROM_FILE("../../rtl/gen/ldpc_dec_rom.mem")) dut (.*);

  string fin, fout;
  int fi, fo, stall, gap, rc, tmp, a, n_in, n_out, cnt;
  logic [7:0] vi, vq;
  logic eof = 0;

  initial begin
    if (!$value$plusargs("in=%s", fin)) $fatal(1, "+in");
    if (!$value$plusargs("out=%s", fout)) $fatal(1, "+out");
    if (!$value$plusargs("rate=%d", tmp)) tmp = 3;
    rate_idx = 4'(tmp);
    if (!$value$plusargs("maxit=%d", tmp)) tmp = 30;
    max_it = 6'(tmp);
    if (!$value$plusargs("stall=%d", stall)) stall = 0;
    if (!$value$plusargs("gap=%d", gap)) gap = 0;
    fi = $fopen(fin, "r");
    fo = $fopen(fout, "w");
    repeat (4) @(posedge clk);
    rst <= 0;
  end

  always @(posedge clk) begin
    if (!rst) begin
      if (s_valid && s_ready) begin s_valid <= 0; cnt = gap; end
      if (!s_valid || s_ready) begin
        if (cnt > 0) cnt--;
        else if (!eof) begin
          rc = $fscanf(fi, "%h %h %d", vi, vq, a);
          if (rc == 3) begin
            s_valid <= 1; s_llr_i <= vi; s_llr_q <= vq; s_first <= a[0]; n_in++;
          end else eof <= 1;
        end
      end
    end
  end

  int idle = 0;
  always @(posedge clk) begin
    m_ready <= ($urandom % 100) >= stall;
    if (m_valid && m_ready) begin
      $fdisplay(fo, "%02h %0d %0d %02h %02h", m_data, m_last, m_ok, m_iters, m_chan);
      n_out++;
      idle = 0;
    end else idle++;
    if (eof && idle > 3000000) begin
      $display("tb_ldpc_dec: %0d words in, %0d bytes out, frames %0d failed %0d t %0t",
               n_in, n_out, st_frames, st_failed, $time);
      $fclose(fo);
      $finish;
    end
  end
endmodule
