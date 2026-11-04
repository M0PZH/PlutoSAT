// Testbench for the DDR LLR queue: dvbs2_rx_llrq -> dvbs2_ldpc_dec, the
// queue's AXI master on axi_mem.
//   +in=<file> words "gap ii qq first slot modcod pilots": offered gap clocks
//       after the previous word was taken (backpressure delays the rest)
//   +out=<file> decoder bytes "dd last ok iters chan rate tag", tag = {0, slot, pilots, modcod}
//   +nfr= +qhi= +itlo= +maxit0= +maxit1= +base=<hex> +stall=<decoder output %>
//   +rstat=<clock>: soft reset (16 clocks); the input then skips to the next first word
//   +axistall= +axilat= +memlo= +memhi= (axi_mem)
`timescale 1ns/1ps
module tb_llrq;
  logic clk = 0, rst = 1, arst = 1;
  always #5 clk = ~clk;

  logic [31:0] base;
  logic [4:0]  nfr;
  logic [5:0]  max_it0, max_it1, qhi, it_lo;
  logic        s_valid = 0, s_ready, s_first, s_slot, s_pilots;
  logic [7:0]  s_llr_i, s_llr_q;
  logic [4:0]  s_modcod;
  logic        q_valid, q_ready, q_first, q_slot, q_pilots, it_cap_en;
  logic [7:0]  q_llr_i, q_llr_q;
  logic [3:0]  q_rate;
  logic [5:0]  q_max_it, it_cap;
  logic [4:0]  q_modcod;
  logic [5:0]  st_depth, st_depth_max;
  logic [15:0] st_frames, st_dropped0, st_dropped1, st_axi_err;
  logic        st_busy;

  logic [31:0] awaddr, araddr;
  logic [3:0]  awlen, arlen, awcache, arcache;
  logic [2:0]  awsize, arsize;
  logic [1:0]  awburst, arburst, bresp, rresp;
  logic        awvalid, awready, wlast, wvalid, wready, bvalid, bready;
  logic        arvalid, arready, rlast, rvalid, rready;
  logic [63:0] wdata, rdata;
  logic [7:0]  wstrb;

  dvbs2_rx_llrq dut (
    .clk, .arst, .rst, .en(1'b1), .base, .nfr, .max_it0, .max_it1, .qhi, .it_lo, .clr_dropped(2'b00),
    .s_valid, .s_ready, .s_llr_i, .s_llr_q, .s_first, .s_slot, .s_modcod, .s_pilots,
    .m_valid(q_valid), .m_ready(q_ready), .m_llr_i(q_llr_i), .m_llr_q(q_llr_q), .m_first(q_first),
    .m_rate(q_rate), .m_max_it(q_max_it), .m_slot(q_slot), .m_modcod(q_modcod), .m_pilots(q_pilots),
    .it_cap_en, .it_cap,
    .st_depth, .st_depth_max, .st_frames, .st_dropped0, .st_dropped1, .st_axi_err, .st_busy,
    .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize), .m_axi_awburst(awburst),
    .m_axi_awcache(awcache), .m_axi_awvalid(awvalid), .m_axi_awready(awready),
    .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast), .m_axi_wvalid(wvalid), .m_axi_wready(wready),
    .m_axi_bresp(bresp), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
    .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize), .m_axi_arburst(arburst),
    .m_axi_arcache(arcache), .m_axi_arvalid(arvalid), .m_axi_arready(arready),
    .m_axi_rdata(rdata), .m_axi_rresp(rresp), .m_axi_rlast(rlast), .m_axi_rvalid(rvalid), .m_axi_rready(rready));

  axi_mem u_mem (
    .clk, .rst(arst), .awaddr, .awlen, .awsize, .awburst, .awvalid, .awready,
    .wdata, .wstrb, .wlast, .wvalid, .wready, .bresp, .bvalid, .bready,
    .araddr, .arlen, .arsize, .arburst, .arvalid, .arready, .rdata, .rresp, .rlast, .rvalid, .rready);

  logic       m_valid, m_ready = 0, m_last, m_ok;
  logic [7:0] m_data, m_chan, m_tag;
  logic [5:0] m_iters;
  logic [3:0] m_rate;
  logic [15:0] l_frames, l_failed;

  dvbs2_ldpc_dec #(.ROM_FILE("../../rtl/gen/ldpc_dec_rom.mem")) u_ldpc (
    .clk, .rst,
    .s_valid(q_valid), .s_ready(q_ready), .s_llr_i(q_llr_i), .s_llr_q(q_llr_q), .s_first(q_first),
    .s_rate(q_rate), .s_max_it(q_max_it), .s_tag({1'b0, q_slot, q_pilots, q_modcod}),
    .it_cap_en, .it_cap,
    .m_valid, .m_ready, .m_data, .m_last, .m_ok, .m_iters, .m_chan, .m_rate, .m_tag,
    .st_frames(l_frames), .st_failed(l_failed));

  string fin, fout;
  int fi, fo, stall, rc, tmp, gap, a_i, a_q, a_f, a_s, a_m, a_p, n_in = 0, n_out = 0, cnt = 0;
  longint rstat, cyc = 0, n_stall = 0, run = 0, max_run = 0;
  logic eof = 0, skipping = 0, have = 0;

  function automatic int arg(input string name, input int dflt);
    int x;
    if (!$value$plusargs({name, "=%d"}, x)) x = dflt;
    return x;
  endfunction

  initial begin
    if (!$value$plusargs("in=%s", fin)) $fatal(1, "+in");
    if (!$value$plusargs("out=%s", fout)) $fatal(1, "+out");
    if (!$value$plusargs("base=%h", base)) base = 32'h3E80_0000;
    if (!$value$plusargs("rstat=%d", rstat)) rstat = -1;
    nfr = 5'(arg("nfr", 16));
    qhi = 6'(arg("qhi", 63));
    it_lo = 6'(arg("itlo", 63));
    max_it0 = 6'(arg("maxit0", 30));
    max_it1 = 6'(arg("maxit1", 30));
    stall = arg("stall", 0);
    fi = $fopen(fin, "r");
    fo = $fopen(fout, "w");
    repeat (4) @(posedge clk);
    arst <= 0;
    repeat (4) @(posedge clk);
    rst <= 0;
  end

  // input words: each one gap clocks after the previous one was taken
  always @(posedge clk) begin
    cyc++;
    if (cyc == rstat) begin
      rst <= 1;
      s_valid <= 0;
      skipping <= 1;
    end else if (cyc == rstat + 16) rst <= 0;
    if (!rst && !arst) begin
      if (s_valid && !s_ready) begin
        n_stall++; run++;
        if (run > max_run) max_run = run;
      end else run = 0;
      if (s_valid && s_ready) s_valid <= 0;
      if (!s_valid || s_ready) begin
        if (!have && !eof) begin
          rc = $fscanf(fi, "%d %h %h %d %d %d %d", gap, a_i, a_q, a_f, a_s, a_m, a_p);
          if (rc == 7) begin have = 1; cnt = gap; end
          else eof <= 1;
        end
        if (have && skipping && a_f == 0) have = 0;              // after a reset: to a frame start
        else if (have) begin
          skipping <= 0;
          if (cnt > 1) cnt--;
          else begin
            s_valid <= 1; s_llr_i <= 8'(a_i); s_llr_q <= 8'(a_q); s_first <= a_f[0];
            s_slot <= a_s[0]; s_modcod <= 5'(a_m); s_pilots <= a_p[0];
            have = 0; n_in++;
          end
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
    end
    if (!eof || st_busy || q_valid || m_valid || u_ldpc.dst != 0 || s_valid) idle = 0;
    else idle++;
    if (idle > 100000) begin
      $display("tb_llrq: words=%0d stall=%0d stall_max=%0d bytes=%0d lframes=%0d lfailed=%0d qframes=%0d drop0=%0d drop1=%0d depth_max=%0d axi_err=%0d wb=%0d rb=%0d outb=%0d outr=%0d t=%0t",
               n_in, n_stall, max_run, n_out, l_frames, l_failed, st_frames, st_dropped0, st_dropped1,
               st_depth_max, st_axi_err, u_mem.n_wb, u_mem.n_rb, u_mem.max_b, u_mem.max_r, $time);
      $fclose(fo);
      $finish;
    end
  end
endmodule
