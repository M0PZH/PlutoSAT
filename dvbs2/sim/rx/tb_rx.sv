// File-driven testbench for the receiver core dvbs2_rx.
//   +coef= +in= (samples "iiii qqqq") +out= (carrier LLRs "ll qq first last")
//   +bytes= (BBFRAME bytes "dd last ok iters") +maxit=
//   +period=<clocks per input sample> (no backpressure on the input)
//   +modcod= +pilots= +thresh= +xa= +xb= +tkp= +tki= +tinteg0= +pkp= +pki= +stall=<output %>
//   +pls=<file> PLS decodes "code conf car grid best second freq" +plsh= +plsstep=
`timescale 1ns/1ps
module tb_rx;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic in_valid = 0;
  logic signed [15:0] in_i, in_q;
  logic [15:0] in_overflows;
  logic coef_we = 0;
  logic [9:0] coef_addr;
  logic signed [17:0] coef_data;
  logic [15:0] timing_kp, timing_ki;
  logic signed [31:0] timing_integ0;
  logic [4:0] modcod;
  logic pilots;
  logic [31:0] fs_thresh;
  logic [17:0] gold_xa, gold_xb, pll_kp, pll_ki;
  logic m_valid, m_ready = 0;
  logic [7:0] m_data;
  logic m_last, m_ok, m_ldpc_ok;
  logic [4:0] m_nerr;
  logic [15:0] m_raw_err;
  logic snap_c_v, snap_t_v;
  logic signed [15:0] snap_c_i, snap_c_q, snap_t_i, snap_t_q;
  logic [15:0] st_bch_corrected, st_bch_failed;
  logic [5:0] m_iters, ldpc_max_it;
  logic [15:0] st_ldpc_frames, st_ldpc_failed;
  logic [16:0] st_agc_gain;
  logic signed [31:0] st_timing_integ;
  logic [1:0] st_fs_state;
  logic [15:0] st_fs_found, st_fs_flywheel, st_fs_lost, st_frames;
  logic [31:0] st_freq, st_amp;
  logic [35:0] st_energy;
  logic [17:0] st_gain;
  logic [15:0] st_acq, st_fa, st_retype;
  logic [5:0] pls_h;
  logic [9:0] pls_step;
  logic [31:0] pls_seen_clr = 0;
  logic st_pls_v;
  logic [6:0] st_pls_code, st_pls_dcode;
  logic [3:0] st_pls_flags, st_pls_dflags;
  logic [17:0] st_pls_best, st_pls_second;
  logic [31:0] st_pls_seen;
  logic [15:0] st_pls_freq, st_pls_count, st_pls_dcount, st_pls_diff_modcod, st_pls_diff_pilots;
  logic [15:0] st_pls_skipped;

  dvbs2_rx #(.LUT_FILE("../../rtl/gen/nco_lut.mem"), .LDPC_ROM_FILE("../../rtl/gen/ldpc_dec_rom.mem")) dut (
    .in_clk(clk), .in_rst(rst), .clk, .rst, .*);

  string fin, fout, fcoef, fbytes, fpls;
  int fi, fo, fc, fb, fp, stall, period, rc, tmp, n_in, n_out, n_bytes;
  logic [17:0] cv;
  logic [15:0] vi, vq;
  logic eof = 0;

  initial begin
    if (!$value$plusargs("in=%s", fin)) $fatal(1, "+in");
    if (!$value$plusargs("out=%s", fout)) $fatal(1, "+out");
    if (!$value$plusargs("coef=%s", fcoef)) $fatal(1, "+coef");
    if (!$value$plusargs("bytes=%s", fbytes)) $fatal(1, "+bytes");
    if (!$value$plusargs("maxit=%d", tmp)) tmp = 30;
    ldpc_max_it = 6'(tmp);
    if (!$value$plusargs("period=%d", period)) period = 24;
    if (!$value$plusargs("stall=%d", stall)) stall = 0;
    if (!$value$plusargs("modcod=%d", tmp)) tmp = 4;
    modcod = 5'(tmp);
    if (!$value$plusargs("pilots=%d", tmp)) tmp = 1;
    pilots = tmp[0];
    if (!$value$plusargs("thresh=%d", fs_thresh)) fs_thresh = 0;
    if (!$value$plusargs("xa=%d", tmp)) tmp = 1;
    gold_xa = 18'(tmp);
    if (!$value$plusargs("xb=%d", tmp)) tmp = 0;
    gold_xb = 18'(tmp);
    if (!$value$plusargs("tkp=%d", tmp)) tmp = 0;
    timing_kp = 16'(tmp);
    if (!$value$plusargs("tki=%d", tmp)) tmp = 0;
    timing_ki = 16'(tmp);
    if (!$value$plusargs("tinteg0=%d", tmp)) tmp = 0;
    timing_integ0 = 32'(tmp);
    if (!$value$plusargs("pkp=%d", tmp)) tmp = 0;
    pll_kp = 18'(tmp);
    if (!$value$plusargs("pki=%d", tmp)) tmp = 0;
    pll_ki = 18'(tmp);
    if (!$value$plusargs("plsh=%d", tmp)) tmp = dvbs2_rx_pkg::PLS_H_DEFAULT;
    pls_h = 6'(tmp);
    if (!$value$plusargs("plsstep=%d", tmp)) tmp = dvbs2_rx_pkg::PLS_STEP_DEFAULT;
    pls_step = 10'(tmp);
    if (!$value$plusargs("pls=%s", fpls)) fpls = "/dev/null";
    fp = $fopen(fpls, "w");
    fc = $fopen(fcoef, "r");
    fi = $fopen(fin, "r");
    fo = $fopen(fout, "w");
    fb = $fopen(fbytes, "w");
    repeat (4) @(posedge clk);
    for (int a = 0; a < 1024; a++) begin
      rc = $fscanf(fc, "%h", cv);
      coef_we <= 1; coef_addr <= 10'(a); coef_data <= cv;
      @(posedge clk);
    end
    coef_we <= 0;
    repeat (8) @(posedge clk);
    rst <= 0;
  end

  // input stream: one sample every `period` clocks
  int ph = 0;
  always @(posedge clk) begin
    in_valid <= 0;
    if (!rst && !eof) begin
      if (ph == 0) begin
        rc = $fscanf(fi, "%h %h", vi, vq);
        if (rc == 2) begin
          in_valid <= 1; in_i <= vi; in_q <= vq; n_in++;
        end else eof <= 1;
      end
      ph = (ph + 1 == period) ? 0 : ph + 1;
    end
  end

  // carrier LLRs (tap inside the core)
  always @(posedge clk)
    if (dut.l_valid && dut.l_ready) begin
      $fdisplay(fo, "%02h %02h %0d %0d", dut.l_i, dut.l_q, dut.l_first, dut.l_last);
      n_out++;
    end

  always @(posedge clk)
    if (st_pls_v)
      $fdisplay(fp, "%02h %0d %0d %0d %05h %05h %04h", st_pls_code, st_pls_flags[0], st_pls_flags[1],
                st_pls_flags[2], st_pls_best, st_pls_second, st_pls_freq);

  int idle = 0;
  always @(posedge clk) begin
    m_ready <= ($urandom % 100) >= stall;
    if (m_valid && m_ready) begin
      $fdisplay(fb, "%02h %0d %0d %02h %0d %02h %04h", m_data, m_last, m_ok, m_iters, m_ldpc_ok, m_nerr, m_raw_err);
      n_bytes++;
    end
    if (m_valid || dut.l_valid || dut.u_ldpc.dst != 0 || !eof) idle = 0;
    else idle++;
    if (eof && idle > 200000) begin
      $display("tb_rx: %0d samples, overflows %0d, %0d LLR words, frames %0d, fs found %0d flywheel %0d lost %0d, agc %0d, freq %0d, amp %0d, gain %0d, %0d bytes, ldpc frames %0d failed %0d, pls %0d skipped %0d, t %0t",
               n_in, in_overflows, n_out, st_frames, st_fs_found, st_fs_flywheel, st_fs_lost,
               st_agc_gain, $signed(st_freq), st_amp, st_gain, n_bytes, st_ldpc_frames, st_ldpc_failed, st_pls_count, st_pls_skipped, $time);
      $fclose(fo);
      $fclose(fb);
      $fclose(fp);
      $finish;
    end
  end
endmodule
