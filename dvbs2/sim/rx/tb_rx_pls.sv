// File-driven testbench for dvbs2_rx_pls.
//   +in=<file> "iiii qqqq sof fhat(hex) acquired", one symbol every +period= clocks
//   +out=<file> one line per decode: "code conf car grid best second freq"
//   +modcod= +pilots= +h= +step=
// At the end: "tb_rx_pls: count .. data .. dcode .. diff .. .. seen .. .. grids .. skipped .."
`timescale 1ns/1ps
module tb_rx_pls;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [4:0] modcod;
  logic pilots;
  logic [5:0] cfg_h;
  logic [9:0] cfg_step;
  logic s_v = 0, s_sof = 0, acquired = 0;
  logic signed [15:0] s_i, s_q;
  logic [31:0] fhat = 0;
  logic [31:0] seen_clr = 0;
  logic res_v, res_conf, res_car, res_grid, res_valid;
  logic [6:0] res_code, data_code;
  logic [17:0] res_best, res_second;
  logic [31:0] st_seen;
  logic [15:0] res_freq, st_count, st_data, st_diff_modcod, st_diff_pilots, st_skipped;
  logic data_conf, data_car, data_grid, data_valid, dec_busy;

  dvbs2_rx_pls dut (.*);

  string fin, fout;
  int fi, fo, period, rc, tmp, a, c, n_in, n_out;
  logic [15:0] vi, vq;
  logic [31:0] fh;
  logic eof = 0;

  initial begin
    if (!$value$plusargs("in=%s", fin)) $fatal(1, "+in");
    if (!$value$plusargs("out=%s", fout)) $fatal(1, "+out");
    if (!$value$plusargs("period=%d", period)) period = 40;
    if (!$value$plusargs("modcod=%d", tmp)) tmp = 4;
    modcod = 5'(tmp);
    if (!$value$plusargs("pilots=%d", tmp)) tmp = 1;
    pilots = tmp[0];
    if (!$value$plusargs("h=%d", tmp)) tmp = 32;
    cfg_h = 6'(tmp);
    if (!$value$plusargs("step=%d", tmp)) tmp = 262;
    cfg_step = 10'(tmp);
    fi = $fopen(fin, "r");
    fo = $fopen(fout, "w");
    repeat (4) @(posedge clk);
    rst <= 0;
  end

  int ph = 0;
  always @(posedge clk) begin
    s_v <= 0;
    if (!rst && !eof) begin
      if (ph == 0) begin
        rc = $fscanf(fi, "%h %h %d %h %d", vi, vq, a, fh, c);
        if (rc == 5) begin
          s_v <= 1; s_i <= vi; s_q <= vq; s_sof <= a[0]; fhat <= fh; acquired <= c[0]; n_in++;
        end else eof <= 1;
      end
      ph = (ph + 1 == period) ? 0 : ph + 1;
    end
  end

  int idle = 0;
  always @(posedge clk) begin
    if (res_v) begin
      $fdisplay(fo, "%02h %0d %0d %0d %05h %05h %04h", res_code, res_conf, res_car, res_grid,
                res_best, res_second, res_freq);
      n_out++;
    end
    if (dut.busy || !eof) idle = 0;
    else idle++;
    if (eof && idle > 1000) begin
      $display("tb_rx_pls: %0d in, count %0d data %0d dcode %02h dvalid %0d diff %0d %0d seen %08h skipped %0d",
               n_in, st_count, st_data, data_code, data_valid, st_diff_modcod, st_diff_pilots, st_seen, st_skipped);
      $fclose(fo);
      $finish;
    end
  end
endmodule
