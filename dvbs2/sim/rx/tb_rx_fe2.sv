// File-driven testbench of the two-slot receive front end (run by
// sim/rx/test_rx2.py): per slot an input FIFO (512 samples, as in dvbs2_rx),
// then the shared engines dvbs2_rx_timing2 -> dvbs2_rx_framesync2 ->
// dvbs2_rx_carrier2, and one dvbs2_rx_pls per slot on the carrier's input
// (as the integrated core will connect them).
//   +coef0= +coef1=  RRC coefficients (1024 each)
//   +in0= +in1=      samples "iiii qqqq", one every +period0= / +period1= clocks
//                    (no backpressure: samples arriving with the FIFO full are lost)
//   +out0= +out1=    LLRs "ll qq first last"; "reset" where the slot was reset
//   +pls0= +pls1=    PLS decodes "code conf car grid best second freq"; "reset" likewise
//   +modcod<s>= +pilots<s>= +thresh<s>= +xa<s>= +xb<s>= +tkp<s>= +tki<s>= +tinteg<s>=
//   +pkp<s>= +pki<s>= +plsh<s>= +plsstep<s>=  per slot; +stall= output percent
//   +rst<s>at=<n>    reset slot s (every engine, its FIFO and PLS decoder) after n
//                    of its samples; its input starts again from the beginning
`timescale 1ns/1ps
module tb_rx_fe2;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [1:0] rst_s = 0;
  wire  [1:0] srst = rst_s | {2{rst}};

  // configuration
  logic [15:0] t_kp [2], t_ki [2];
  logic signed [31:0] t_integ [2];
  logic [4:0] modcod2 [2];
  logic [1:0] pilots2;
  logic [31:0] thresh2 [2];
  logic [17:0] gold_xa2 [2], gold_xb2 [2], pll_kp2 [2], pll_ki2 [2];
  logic [5:0] pls_h [2];
  logic [9:0] pls_step [2];
  logic coef_we = 0, coef_slot = 0;
  logic [9:0] coef_addr;
  logic signed [17:0] coef_data;

  // ---------------------------------------------------------------- input FIFOs
  logic [1:0] in_valid = 0, w_ready, f_valid, f_ready;
  logic signed [15:0] in_i [2], in_q [2];
  logic [31:0] f_data [2];
  logic signed [15:0] f_i [2], f_q [2];
  int overflows [2];
  for (genvar s = 0; s < 2; s++) begin : g_fifo
    dvbs2_async_fifo #(.W(32), .AW(9)) u_in_fifo (
      .wclk(clk), .wrst(srst[s]), .w_valid(in_valid[s]), .w_ready(w_ready[s]), .w_data({in_q[s], in_i[s]}),
      .rclk(clk), .rrst(srst[s]), .r_valid(f_valid[s]), .r_ready(f_ready[s]), .r_data(f_data[s]));
    assign f_i[s] = $signed(f_data[s][15:0]);
    assign f_q[s] = $signed(f_data[s][31:16]);
  end

  // ---------------------------------------------------------------- timing
  logic [1:0] t_valid, t_ready;
  logic signed [15:0] t_i [2], t_q [2];
  logic [16:0] agc_gain [2];
  logic signed [31:0] loop_integ [2];
  dvbs2_rx_timing2 u_timing (
    .clk, .rst, .rst_s, .kp(t_kp), .ki(t_ki), .integ_init(t_integ), .coef_we, .coef_slot, .coef_addr, .coef_data,
    .s_valid(f_valid), .s_ready(f_ready), .s_i(f_i), .s_q(f_q),
    .m_valid(t_valid), .m_ready(t_ready), .m_i(t_i), .m_q(t_q), .agc_gain, .loop_integ);

  // ---------------------------------------------------------------- frame sync
  logic y_valid, y_ready, y_sof, y_dummy, y_found, y_slot;
  logic signed [15:0] y_i, y_q;
  logic [1:0] ty_v2, pls_v2, pls_conf2, pls_busy2;
  logic ty_dummy;
  logic [6:0] pls_code2 [2];
  logic [1:0] fs_state2 [2];
  logic [15:0] cnt_found2 [2], cnt_flywheel2 [2], cnt_lost2 [2];
  dvbs2_rx_framesync2 u_fs (
    .clk, .rst, .rst_s, .modcod2, .pilots2, .thresh2,
    .s_valid2(t_valid), .s_ready2(t_ready), .s_i2(t_i), .s_q2(t_q),
    .m_valid(y_valid), .m_ready(y_ready), .m_i(y_i), .m_q(y_q), .m_sof(y_sof), .m_dummy(y_dummy),
    .m_found(y_found), .m_slot(y_slot), .ty_v2, .ty_dummy, .pls_v2, .pls_conf2, .pls_code2, .pls_busy2,
    .fs_state2, .cnt_found2, .cnt_flywheel2, .cnt_lost2);

  // ---------------------------------------------------------------- carrier
  logic [1:0] c_valid, c_ready;
  logic m_valid, m_ready = 0, m_first, m_last, m_slot;
  logic [7:0] m_llr_i, m_llr_q;
  logic [31:0] st_freq2 [2], st_amp2 [2];
  logic [35:0] st_energy2 [2];
  logic [17:0] st_gain2 [2];
  logic [15:0] st_frames2 [2], st_acq2 [2], st_fa2 [2], st_retype2 [2];
  logic [1:0] st_acquired2;
  logic snap_v, snap_slot;
  logic signed [15:0] snap_i, snap_q;
  logic signed [15:0] y_i2 [2], y_q2 [2];
  assign y_i2[0] = y_i; assign y_i2[1] = y_i;
  assign y_q2[0] = y_q; assign y_q2[1] = y_q;
  assign c_valid = {y_valid && y_slot, y_valid && !y_slot};
  assign y_ready = c_ready[y_slot];
  dvbs2_rx_carrier2 #(.LUT_FILE("../../rtl/gen/nco_lut.mem")) u_carrier (
    .clk, .rst, .rst_s, .modcod2, .pilots2, .gold_xa2, .gold_xb2, .pll_kp2, .pll_ki2,
    .s_valid2(c_valid), .s_ready2(c_ready), .s_i2(y_i2), .s_q2(y_q2),
    .s_sof2({2{y_sof}}), .s_dummy2({2{y_dummy}}), .s_found2({2{y_found}}),
    .ty_v2, .ty_dummy, .m_valid, .m_ready, .m_llr_i, .m_llr_q, .m_first, .m_last, .m_slot,
    .st_freq2, .st_amp2, .st_energy2, .st_gain2, .st_frames2, .st_acq2, .st_fa2, .st_retype2, .st_acquired2,
    .snap_v, .snap_i, .snap_q, .snap_slot);

  // ---------------------------------------------------------------- PLS decoders, one per slot
  logic [6:0] p_code [2], p_dcode [2];
  logic [1:0] p_car, p_grid, p_valid, p_dconf, p_dcar, p_dgrid, p_dvalid;
  logic [17:0] p_best [2], p_second [2];
  logic [15:0] p_freq [2], p_count [2], p_data [2], p_dmod [2], p_dpil [2], p_skipped [2];
  logic [31:0] p_seen [2];
  for (genvar s = 0; s < 2; s++) begin : g_pls
    dvbs2_rx_pls u_pls (
      .clk, .rst(srst[s]), .modcod(modcod2[s]), .pilots(pilots2[s]), .cfg_h(pls_h[s]), .cfg_step(pls_step[s]),
      .s_v(y_valid && y_ready && y_slot == s[0]), .s_i(y_i), .s_q(y_q), .s_sof(y_sof),
      .fhat(st_freq2[s]), .acquired(st_acquired2[s]), .seen_clr(32'd0),
      .res_v(pls_v2[s]), .res_code(pls_code2[s]), .res_conf(pls_conf2[s]), .res_car(p_car[s]),
      .res_grid(p_grid[s]), .res_best(p_best[s]), .res_second(p_second[s]), .res_freq(p_freq[s]),
      .res_valid(p_valid[s]), .st_count(p_count[s]),
      .data_code(p_dcode[s]), .data_conf(p_dconf[s]), .data_car(p_dcar[s]), .data_grid(p_dgrid[s]),
      .data_valid(p_dvalid[s]), .st_data(p_data[s]), .st_diff_modcod(p_dmod[s]), .st_diff_pilots(p_dpil[s]),
      .st_seen(p_seen[s]), .st_skipped(p_skipped[s]), .dec_busy(pls_busy2[s]));
  end

  // ---------------------------------------------------------------- stimulus
  string fin [2], fout [2], fpls [2], fcoef [2], nm;
  int fi [2], fo [2], fp [2], fc, stall, period [2], ph [2], n_in [2], n_out [2], rst_at [2], rst_cnt [2];
  int rc, tmp, cyc = 0;
  logic [17:0] cv;
  logic [15:0] vi, vq;
  logic [1:0] eof = 0;

  initial begin
    for (int s = 0; s < 2; s++) begin
      nm = $sformatf("in%0d=%%s", s);   if (!$value$plusargs(nm, fin[s])) $fatal(1, "+in");
      nm = $sformatf("out%0d=%%s", s);  if (!$value$plusargs(nm, fout[s])) $fatal(1, "+out");
      nm = $sformatf("pls%0d=%%s", s);  if (!$value$plusargs(nm, fpls[s])) fpls[s] = "/dev/null";
      nm = $sformatf("coef%0d=%%s", s); if (!$value$plusargs(nm, fcoef[s])) $fatal(1, "+coef");
      nm = $sformatf("period%0d=%%d", s); if (!$value$plusargs(nm, period[s])) period[s] = 40;
      nm = $sformatf("modcod%0d=%%d", s); if (!$value$plusargs(nm, tmp)) tmp = 4; modcod2[s] = 5'(tmp);
      nm = $sformatf("pilots%0d=%%d", s); if (!$value$plusargs(nm, tmp)) tmp = 1; pilots2[s] = tmp[0];
      nm = $sformatf("thresh%0d=%%d", s); if (!$value$plusargs(nm, tmp)) tmp = 0; thresh2[s] = 32'(tmp);
      nm = $sformatf("xa%0d=%%d", s);     if (!$value$plusargs(nm, tmp)) tmp = 1; gold_xa2[s] = 18'(tmp);
      nm = $sformatf("xb%0d=%%d", s);     if (!$value$plusargs(nm, tmp)) tmp = 0; gold_xb2[s] = 18'(tmp);
      nm = $sformatf("tkp%0d=%%d", s);    if (!$value$plusargs(nm, tmp)) tmp = 0; t_kp[s] = 16'(tmp);
      nm = $sformatf("tki%0d=%%d", s);    if (!$value$plusargs(nm, tmp)) tmp = 0; t_ki[s] = 16'(tmp);
      nm = $sformatf("tinteg%0d=%%d", s); if (!$value$plusargs(nm, tmp)) tmp = 0; t_integ[s] = 32'(tmp);
      nm = $sformatf("pkp%0d=%%d", s);    if (!$value$plusargs(nm, tmp)) tmp = 0; pll_kp2[s] = 18'(tmp);
      nm = $sformatf("pki%0d=%%d", s);    if (!$value$plusargs(nm, tmp)) tmp = 0; pll_ki2[s] = 18'(tmp);
      nm = $sformatf("plsh%0d=%%d", s);   if (!$value$plusargs(nm, tmp)) tmp = dvbs2_rx_pkg::PLS_H_DEFAULT;
      pls_h[s] = 6'(tmp);
      nm = $sformatf("plsstep%0d=%%d", s); if (!$value$plusargs(nm, tmp)) tmp = dvbs2_rx_pkg::PLS_STEP_DEFAULT;
      pls_step[s] = 10'(tmp);
      nm = $sformatf("rst%0dat=%%d", s);  if (!$value$plusargs(nm, rst_at[s])) rst_at[s] = -1;
      fi[s] = $fopen(fin[s], "r");
      fo[s] = $fopen(fout[s], "w");
      fp[s] = $fopen(fpls[s], "w");
      n_in[s] = 0; n_out[s] = 0; rst_cnt[s] = 0; ph[s] = 0; overflows[s] = 0;
    end
    if (!$value$plusargs("stall=%d", stall)) stall = 0;
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
    repeat (8) @(posedge clk);
    rst <= 0;
  end

  for (genvar s = 0; s < 2; s++) begin : g_drv
    // one sample every period clocks; a full FIFO loses it
    always @(posedge clk) begin
      in_valid[s] <= 0;
      if (in_valid[s] && !w_ready[s]) overflows[s]++;
      if (rst_cnt[s] != 0) begin
        rst_cnt[s]--;
        if (rst_cnt[s] == 0) rst_s[s] <= 0;
      end else if (!rst && !eof[s]) begin
        if (n_in[s] == rst_at[s]) begin
          rst_at[s] = -1;
          rst_s[s] <= 1;
          rst_cnt[s] = 3;
          ph[s] = 0;
          $fdisplay(fo[s], "reset");
          $fdisplay(fp[s], "reset");
          $rewind(fi[s]);
        end else begin
          if (ph[s] == 0) begin
            rc = $fscanf(fi[s], "%h %h", vi, vq);
            if (rc == 2) begin
              in_valid[s] <= 1; in_i[s] <= vi; in_q[s] <= vq; n_in[s]++;
            end else eof[s] <= 1;
          end
          ph[s] = (ph[s] + 1 == period[s]) ? 0 : ph[s] + 1;
        end
      end
    end

    always @(posedge clk)
      if (pls_v2[s])
        $fdisplay(fp[s], "%02h %0d %0d %0d %05h %05h %04h", pls_code2[s], pls_conf2[s], p_car[s], p_grid[s],
                  p_best[s], p_second[s], p_freq[s]);
  end

  // the frame sync waiting for a slot's PLS decode (clocks, longest wait)
  int hold_n [2] = '{0, 0}, hold_run [2] = '{0, 0}, hold_max [2] = '{0, 0};
  always @(posedge clk)
    for (int s = 0; s < 2; s++)
      if (u_fs.st == 0 && (s == u_fs.cur ? u_fs.pls_hold && t_valid[s] : u_fs.hold_oth && t_valid[s])) begin
        hold_n[s]++; hold_run[s]++;
        if (hold_run[s] > hold_max[s]) hold_max[s] = hold_run[s];
      end else hold_run[s] = 0;

  int idle = 0, busy_t = 0;
  always @(posedge clk) begin
    cyc++;
    if (u_timing.state != 0) busy_t++;
    m_ready <= ($urandom % 100) >= stall;
    if (m_valid && m_ready) begin
      $fdisplay(fo[m_slot], "%02h %02h %0d %0d", m_llr_i, m_llr_q, m_first, m_last);
      n_out[m_slot]++;
    end
    if (m_valid || |f_valid || |t_valid || y_valid || |pls_busy2 || eof != 2'b11) idle = 0;
    else idle++;
    if (eof == 2'b11 && idle > 50000) begin
      for (int s = 0; s < 2; s++)
        $display("slot %0d: %0d samples, overflows %0d, %0d LLR words, frames %0d, fs found %0d flywheel %0d lost %0d, agc %0d, freq %0d, amp %0d, energy %0d, gain %0d, acq %0d, pls %0d skipped %0d, pls hold %0d max %0d,",
                 s, n_in[s], overflows[s], n_out[s], st_frames2[s], cnt_found2[s], cnt_flywheel2[s], cnt_lost2[s],
                 agc_gain[s], $signed(st_freq2[s]), st_amp2[s], st_energy2[s], st_gain2[s], st_acq2[s],
                 p_count[s], p_skipped[s], hold_n[s], hold_max[s]);
      $display("tb_rx_fe2: %0d clocks, timing engine busy %0d", cyc, busy_t);
      for (int s = 0; s < 2; s++) begin
        $fclose(fo[s]); $fclose(fp[s]);
      end
      $finish;
    end
  end
endmodule
