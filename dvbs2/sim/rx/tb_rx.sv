// File-driven testbench for the receiver core dvbs2_rx.
//   +coef= +in= (samples "iiii qqqq") +out= (carrier LLRs "ll qq first last")
//   +bytes= (BBFRAME bytes "dd last ok iters ldpc_ok nerr raw rate modcod pilots slot") +maxit=
//   +period=<clocks per input sample> (no backpressure on the input)
//   +modcod= +pilots= +thresh= +xa= +xb= +tkp= +tki= +tinteg0= +pkp= +pki= +stall=<output %>
//   +pls=<file> PLS decodes "code conf car grid best second freq" +plsh= +plsstep=
//   +llrq=1: the LDPC input through the DDR LLR queue (axi_mem: +axistall= +axilat=),
//   +llrqnfr= +llrqqhi= +llrqitlo=
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
  logic [3:0] m_rate;
  logic [4:0] m_modcod;
  logic m_pilots, m_slot;
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
  logic axi_rst = 1, llrq_en;
  logic [31:0] llrq_base = 32'h3E80_0000;
  logic [4:0] llrq_nfr;
  logic [5:0] llrq_qhi, llrq_it_lo;
  logic [5:0] st_llrq_depth, st_llrq_depth_max;
  logic [15:0] st_llrq_frames, st_llrq_dropped0, st_llrq_dropped1, st_llrq_axi_err;
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
    if (!$value$plusargs("llrq=%d", tmp)) tmp = 0;
    llrq_en = tmp[0];
    if (!$value$plusargs("llrqnfr=%d", tmp)) tmp = 16;
    llrq_nfr = 5'(tmp);
    if (!$value$plusargs("llrqqhi=%d", tmp)) tmp = 63;
    llrq_qhi = 6'(tmp);
    if (!$value$plusargs("llrqitlo=%d", tmp)) tmp = 63;
    llrq_it_lo = 6'(tmp);
    fp = $fopen(fpls, "w");
    fc = $fopen(fcoef, "r");
    fi = $fopen(fin, "r");
    fo = $fopen(fout, "w");
    fb = $fopen(fbytes, "w");
    repeat (4) @(posedge clk);
    axi_rst <= 0;
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

  // clocks the frame sync waited for the PLS decoder, and the longest wait
  int hold_n = 0, hold_run = 0, hold_max = 0;
  always @(posedge clk)
    if (dut.u_fs.pls_hold) begin
      hold_n++; hold_run++;
      if (hold_run > hold_max) hold_max = hold_run;
    end else hold_run = 0;

  int idle = 0;
  always @(posedge clk) begin
    m_ready <= ($urandom % 100) >= stall;
    if (m_valid && m_ready) begin
      $fdisplay(fb, "%02h %0d %0d %02h %0d %02h %04h %0h %02h %0d %0d", m_data, m_last, m_ok, m_iters, m_ldpc_ok,
                m_nerr, m_raw_err, m_rate, m_modcod, m_pilots, m_slot);
      n_bytes++;
    end
    if (m_valid || dut.l_valid || dut.u_ldpc.dst != 0 || st_llrq_busy || !eof) idle = 0;
    else idle++;
    if (eof && idle > 200000) begin
      $display("tb_rx: %0d samples, overflows %0d, %0d LLR words, frames %0d, fs found %0d flywheel %0d lost %0d, agc %0d, freq %0d, amp %0d, energy %0d, gain %0d, %0d bytes, ldpc frames %0d failed %0d, pls %0d skipped %0d, pls hold %0d max %0d, llrq frames %0d dropped %0d depth max %0d axi err %0d, t %0t",
               n_in, in_overflows, n_out, st_frames, st_fs_found, st_fs_flywheel, st_fs_lost,
               st_agc_gain, $signed(st_freq), st_amp, st_energy, st_gain, n_bytes, st_ldpc_frames, st_ldpc_failed, st_pls_count, st_pls_skipped, hold_n, hold_max,
               st_llrq_frames, st_llrq_dropped0, st_llrq_depth_max, st_llrq_axi_err, $time);
      $fclose(fo);
      $fclose(fb);
      $fclose(fp);
      $finish;
    end
  end
endmodule
