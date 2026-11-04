// File-driven testbench for the two-slot receiver core dvbs2_rx (run by
// sim/rx/test_rx.py and test_rx2.py). Per slot s (a slot without +in<s> is idle):
//   +in<s>=      samples "iiii qqqq", one every +period<s>= clocks (no backpressure:
//                samples arriving with the slot's FIFO full are lost)
//   +coef<s>=    RRC coefficients (1024)
//   +out<s>=     carrier LLRs "ll qq first last"; "reset" where the slot was reset
//   +bytes<s>=   BBFRAME bytes "dd last ok iters ldpc_ok nerr raw rate modcod pilots slot";
//                "reset" likewise
//   +pls<s>=     PLS decodes "code conf car grid best second freq"; "reset" likewise
//   +modcod<s>= +pilots<s>= +thresh<s>= +xa<s>= +xb<s>= +tkp<s>= +tki<s>= +tinteg0<s>=
//   +pkp<s>= +pki<s>= +plsh<s>= +plsstep<s>= +maxit<s>=
//   +rst<s>at=<n> reset slot s (rst_s) after n of its samples; its input starts again
//                 from the beginning
// Common: +stall=<output %>, LLR queue +llrqnfr= +llrqqhi= +llrqitlo= (axi_mem: +axistall=
// +axilat=), +prof=N: every N clocks the input FIFOs' fill and the engines' busy share.
// Prints a status line per slot ("slot s: ...") and one for the core.
`timescale 1ns/1ps
module tb_rx;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [1:0] rst_s = 0;
  logic [1:0] in_valid = 0;
  logic signed [15:0] in_i [2], in_q [2];
  logic [15:0] in_overflows [2];
  logic coef_we = 0, coef_slot = 0;
  logic [9:0] coef_addr;
  logic signed [17:0] coef_data;
  logic [15:0] timing_kp [2], timing_ki [2];
  logic signed [31:0] timing_integ0 [2];
  logic [4:0] modcod [2];
  logic [1:0] pilots;
  logic [31:0] fs_thresh [2];
  logic [17:0] gold_xa [2], gold_xb [2], pll_kp [2], pll_ki [2];
  logic [5:0] ldpc_max_it [2], pls_h [2];
  logic [9:0] pls_step [2];
  logic [31:0] pls_seen_clr [2] = '{0, 0};
  logic m_valid, m_ready = 0;
  logic [7:0] m_data;
  logic m_last, m_ok, m_ldpc_ok;
  logic [5:0] m_iters;
  logic [4:0] m_nerr;
  logic [15:0] m_raw_err;
  logic [3:0] m_rate;
  logic [4:0] m_modcod;
  logic m_pilots, m_slot;
  logic snap_c_v, snap_c_slot;
  logic [1:0] snap_t_v;
  logic signed [15:0] snap_c_i, snap_c_q, snap_t_i [2], snap_t_q [2];
  logic [16:0] st_agc_gain [2];
  logic signed [31:0] st_timing_integ [2];
  logic [1:0] st_fs_state [2];
  logic [15:0] st_fs_found [2], st_fs_flywheel [2], st_fs_lost [2], st_frames [2];
  logic [31:0] st_freq [2], st_amp [2];
  logic [35:0] st_energy [2];
  logic [17:0] st_gain [2];
  logic [15:0] st_acq [2], st_fa [2], st_retype [2];
  logic [15:0] st_ldpc_frames [2], st_ldpc_failed [2], st_bch_corrected [2], st_bch_failed [2];
  logic [1:0] st_pls_v;
  logic [6:0] st_pls_code [2], st_pls_dcode [2];
  logic [3:0] st_pls_flags [2], st_pls_dflags [2];
  logic [17:0] st_pls_best [2], st_pls_second [2];
  logic [31:0] st_pls_seen [2];
  logic [15:0] st_pls_freq [2], st_pls_count [2], st_pls_dcount [2], st_pls_diff_modcod [2], st_pls_diff_pilots [2];
  logic [15:0] st_pls_skipped [2];
  logic axi_rst = 1, llrq_en = 1;
  logic [31:0] llrq_base = 32'h3E80_0000;
  logic [4:0] llrq_nfr;
  logic [5:0] llrq_qhi, llrq_it_lo;
  logic [5:0] st_llrq_depth, st_llrq_depth_max;
  logic [15:0] st_llrq_frames, st_llrq_dropped [2], st_llrq_axi_err;
  logic st_llrq_busy;
  logic [31:0] m_axi_awaddr, m_axi_araddr;
  logic [3:0]  m_axi_awlen, m_axi_arlen, m_axi_awcache, m_axi_arcache;
  logic [2:0]  m_axi_awsize, m_axi_arsize;
  logic [1:0]  m_axi_awburst, m_axi_arburst, m_axi_bresp, m_axi_rresp;
  logic        m_axi_awvalid, m_axi_awready, m_axi_wlast, m_axi_wvalid, m_axi_wready, m_axi_bvalid, m_axi_bready;
  logic        m_axi_arvalid, m_axi_arready, m_axi_rlast, m_axi_rvalid, m_axi_rready;
  logic [63:0] m_axi_wdata, m_axi_rdata;
  logic [7:0]  m_axi_wstrb;

  axi_mem u_mem (
    .clk, .rst(axi_rst), .awaddr(m_axi_awaddr), .awlen(m_axi_awlen), .awsize(m_axi_awsize),
    .awburst(m_axi_awburst), .awvalid(m_axi_awvalid), .awready(m_axi_awready),
    .wdata(m_axi_wdata), .wstrb(m_axi_wstrb), .wlast(m_axi_wlast), .wvalid(m_axi_wvalid), .wready(m_axi_wready),
    .bresp(m_axi_bresp), .bvalid(m_axi_bvalid), .bready(m_axi_bready),
    .araddr(m_axi_araddr), .arlen(m_axi_arlen), .arsize(m_axi_arsize), .arburst(m_axi_arburst),
    .arvalid(m_axi_arvalid), .arready(m_axi_arready), .rdata(m_axi_rdata), .rresp(m_axi_rresp),
    .rlast(m_axi_rlast), .rvalid(m_axi_rvalid), .rready(m_axi_rready));

  dvbs2_rx #(.LUT_FILE("../../rtl/gen/nco_lut.mem"), .LDPC_ROM_FILE("../../rtl/gen/ldpc_dec_rom.mem")) dut (.*);

  function automatic int arg(input string name, input int dflt);
    int x;
    if (!$value$plusargs({name, "=%d"}, x)) x = dflt;
    return x;
  endfunction

  string fin [2], fout [2], fbytes [2], fpls [2], fcoef [2], nm;
  int fi [2], fo [2], fb [2], fp [2], fc, stall, period [2], ph [2], n_in [2], n_out [2], n_bytes [2];
  int rst_at [2], rst_cnt [2], rc;
  logic [1:0] act = 0, eof = 0;
  logic [17:0] cv;
  logic [15:0] vi, vq;

  initial begin
    for (int s = 0; s < 2; s++) begin
      nm = $sformatf("in%0d=%%s", s);
      act[s] = $value$plusargs(nm, fin[s]) != 0;
      nm = $sformatf("out%0d=%%s", s);   if (!$value$plusargs(nm, fout[s])) fout[s] = "/dev/null";
      nm = $sformatf("bytes%0d=%%s", s); if (!$value$plusargs(nm, fbytes[s])) fbytes[s] = "/dev/null";
      nm = $sformatf("pls%0d=%%s", s);   if (!$value$plusargs(nm, fpls[s])) fpls[s] = "/dev/null";
      nm = $sformatf("coef%0d=%%s", s);  if (!$value$plusargs(nm, fcoef[s])) fcoef[s] = "";
      period[s]   = arg($sformatf("period%0d", s), 24);
      modcod[s]   = 5'(arg($sformatf("modcod%0d", s), 4));
      pilots[s]   = arg($sformatf("pilots%0d", s), 1) != 0;
      fs_thresh[s] = 32'(arg($sformatf("thresh%0d", s), 0));
      gold_xa[s]  = 18'(arg($sformatf("xa%0d", s), 1));
      gold_xb[s]  = 18'(arg($sformatf("xb%0d", s), 0));
      timing_kp[s] = 16'(arg($sformatf("tkp%0d", s), 0));
      timing_ki[s] = 16'(arg($sformatf("tki%0d", s), 0));
      timing_integ0[s] = 32'(arg($sformatf("tinteg0%0d", s), 0));
      pll_kp[s]   = 18'(arg($sformatf("pkp%0d", s), 0));
      pll_ki[s]   = 18'(arg($sformatf("pki%0d", s), 0));
      pls_h[s]    = 6'(arg($sformatf("plsh%0d", s), dvbs2_rx_pkg::PLS_H_DEFAULT));
      pls_step[s] = 10'(arg($sformatf("plsstep%0d", s), dvbs2_rx_pkg::PLS_STEP_DEFAULT));
      ldpc_max_it[s] = 6'(arg($sformatf("maxit%0d", s), 30));
      rst_at[s]   = arg($sformatf("rst%0dat", s), -1);
      if (act[s]) fi[s] = $fopen(fin[s], "r");
      fo[s] = $fopen(fout[s], "w");
      fb[s] = $fopen(fbytes[s], "w");
      fp[s] = $fopen(fpls[s], "w");
      n_in[s] = 0; n_out[s] = 0; n_bytes[s] = 0; rst_cnt[s] = 0; ph[s] = 0;
    end
    stall = arg("stall", 0);
    llrq_nfr = 5'(arg("llrqnfr", 16));
    llrq_qhi = 6'(arg("llrqqhi", 63));
    llrq_it_lo = 6'(arg("llrqitlo", 63));
    repeat (4) @(posedge clk);
    axi_rst <= 0;
    for (int s = 0; s < 2; s++) begin
      if (fcoef[s] == "") continue;
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
    // one sample every period clocks
    always @(posedge clk) begin
      in_valid[s] <= 0;
      if (rst_cnt[s] != 0) begin
        rst_cnt[s]--;
        if (rst_cnt[s] == 0) rst_s[s] <= 0;
      end else if (!rst && act[s] && !eof[s]) begin
        if (n_in[s] == rst_at[s]) begin
          rst_at[s] = -1;
          rst_s[s] <= 1;
          rst_cnt[s] = 3;
          ph[s] = 0;
          $fdisplay(fo[s], "reset");
          $fdisplay(fp[s], "reset");
          $fdisplay(fb[s], "reset");
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
      if (st_pls_v[s])
        $fdisplay(fp[s], "%02h %0d %0d %0d %05h %05h %04h", st_pls_code[s], st_pls_flags[s][0], st_pls_flags[s][1],
                  st_pls_flags[s][2], st_pls_best[s], st_pls_second[s], st_pls_freq[s]);
  end

  // carrier LLRs (tap inside the core)
  always @(posedge clk)
    if (dut.l_valid && dut.l_ready) begin
      $fdisplay(fo[dut.l_slot], "%02h %02h %0d %0d", dut.l_i, dut.l_q, dut.l_first, dut.l_last);
      n_out[dut.l_slot]++;
    end

  // clocks the frame sync waited for a slot's PLS decode, and the longest wait
  int hold_n [2] = '{0, 0}, hold_run [2] = '{0, 0}, hold_max [2] = '{0, 0};
  always @(posedge clk)
    for (int s = 0; s < 2; s++)
      if (dut.u_fs.st == 0 && (s == dut.u_fs.cur ? dut.u_fs.pls_hold && dut.t_valid[s]
                                                 : dut.u_fs.hold_oth && dut.t_valid[s])) begin
        hold_n[s]++; hold_run[s]++;
        if (hold_run[s] > hold_max[s]) hold_max[s] = hold_run[s];
      end else hold_run[s] = 0;

  // +prof=N: every N clocks the input FIFOs' fill and the engines' busy share
  int prof, pr_t, pr_f, pr_c, pr_tb;
  initial if (!$value$plusargs("prof=%d", prof)) prof = 0;
  always @(posedge clk) if (prof != 0) begin
    pr_t += int'(dut.u_timing.state != 0);
    pr_f += int'(dut.u_fs.st != 0);
    pr_c += int'(dut.u_carrier.st != 0);
    pr_tb += int'(dut.y_valid && !dut.y_ready);
    if (cyc % prof == 0) begin
      $display("prof %0d: fifo %0d %0d, timing %0d%%, fs %0d%%, carrier %0d%%, fs out held %0d%%, ovf %0d %0d",
               cyc, dut.g_in[0].u_in_fifo.wp - dut.g_in[0].u_in_fifo.rp, dut.g_in[1].u_in_fifo.wp - dut.g_in[1].u_in_fifo.rp,
               100 * pr_t / prof, 100 * pr_f / prof, 100 * pr_c / prof, 100 * pr_tb / prof, in_overflows[0], in_overflows[1]);
      pr_t = 0; pr_f = 0; pr_c = 0; pr_tb = 0;
    end
  end

  int idle = 0;
  longint cyc = 0;
  always @(posedge clk) begin
    cyc++;
    m_ready <= ($urandom % 100) >= stall;
    if (m_valid && m_ready) begin
      $fdisplay(fb[m_slot], "%02h %0d %0d %02h %0d %02h %04h %0h %02h %0d %0d", m_data, m_last, m_ok, m_iters,
                m_ldpc_ok, m_nerr, m_raw_err, m_rate, m_modcod, m_pilots, m_slot);
      n_bytes[m_slot]++;
    end
    if (m_valid || dut.l_valid || dut.u_ldpc.dst != 0 || st_llrq_busy || |dut.f_valid || |dut.t_valid ||
        dut.y_valid || |dut.pls_busy2 || (eof | ~act) != 2'b11) idle = 0;
    else idle++;
    if (idle > 200000) begin
      for (int s = 0; s < 2; s++) begin
        $display("slot %0d: %0d samples, overflows %0d, %0d LLR words, frames %0d, fs found %0d flywheel %0d lost %0d, agc %0d, freq %0d, amp %0d, energy %0d, gain %0d, %0d bytes, ldpc frames %0d failed %0d, bch corrected %0d failed %0d, pls %0d skipped %0d, pls hold %0d max %0d, llrq dropped %0d,",
                 s, n_in[s], in_overflows[s], n_out[s], st_frames[s], st_fs_found[s], st_fs_flywheel[s], st_fs_lost[s],
                 st_agc_gain[s], $signed(st_freq[s]), st_amp[s], st_energy[s], st_gain[s], n_bytes[s],
                 st_ldpc_frames[s], st_ldpc_failed[s], st_bch_corrected[s], st_bch_failed[s],
                 st_pls_count[s], st_pls_skipped[s], hold_n[s], hold_max[s], st_llrq_dropped[s]);
        $fclose(fo[s]); $fclose(fb[s]); $fclose(fp[s]);
      end
      $display("tb_rx: llrq frames %0d depth max %0d axi err %0d, %0d clocks, t %0t",
               st_llrq_frames, st_llrq_depth_max, st_llrq_axi_err, cyc, $time);
      $finish;
    end
  end
endmodule
