// File-driven testbench for dvbs2_ldpc_dec.
//   +in=<file> "ii qq first rate maxit tag" per word (rate, maxit, tag: taken on
//   a frame's first word); +out=<file> "dd last ok iters chan rate tag"
//   +stall=<output %> +gap=<input idle clocks per word> +itcap=<n> (it_cap, enabled if > 0)
`timescale 1ns/1ps
module tb_ldpc_dec;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [3:0] s_rate, m_rate;
  logic [5:0] s_max_it, it_cap;
  logic [7:0] s_tag, m_tag;
  logic it_cap_en;
  logic s_valid = 0, s_ready, s_first;
  logic [7:0] s_llr_i, s_llr_q;
  logic m_valid, m_ready = 0, m_last, m_ok;
  logic [7:0] m_data;
  logic [5:0] m_iters;
  logic [7:0] m_chan;
  logic [15:0] st_frames, st_failed;

  dvbs2_ldpc_dec #(.ROM_FILE("../../rtl/gen/ldpc_dec_rom.mem")) dut (.*);

  string fin, fout;
  int fi, fo, stall, gap, rc, tmp, a, ra, mi, tg, n_in, n_out, cnt;
  logic [7:0] vi, vq;
  logic eof = 0;

  initial begin
    if (!$value$plusargs("in=%s", fin)) $fatal(1, "+in");
    if (!$value$plusargs("out=%s", fout)) $fatal(1, "+out");
    if (!$value$plusargs("itcap=%d", tmp)) tmp = 0;
    it_cap = 6'(tmp);
    it_cap_en = tmp > 0;
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
          rc = $fscanf(fi, "%h %h %d %d %d %d", vi, vq, a, ra, mi, tg);
          if (rc == 6) begin
            s_valid <= 1; s_llr_i <= vi; s_llr_q <= vq; s_first <= a[0]; n_in++;
            s_rate <= 4'(ra); s_max_it <= 6'(mi); s_tag <= 8'(tg);
          end else eof <= 1;
        end
      end
    end
  end

  int idle = 0;
  always @(posedge clk) begin
    m_ready <= ($urandom % 100) >= stall;
    if (m_valid && m_ready) begin
      $fdisplay(fo, "%02h %0d %0d %02h %02h %0h %02h", m_data, m_last, m_ok, m_iters, m_chan, m_rate, m_tag);
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
