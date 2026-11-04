// Spectrum analyser: averaged power spectra (4096 bins) of one selectable
// source, computed in the PL; the ARM reads the result over AXI-Lite.
//
// Source (rx0 / rx1 at the ADC rate, or one nbiq channel) -> 4096
// consecutive samples -> window (table, Blackman-Harris at reset) -> Xilinx
// FFT (radix-2 Lite, 16-bit, scaled, convergent rounding) -> |X|^2 ->
// sum over NAVG frames (40-bit) -> >> RSHIFT, saturated to 32 bits.
// Model: model/fftspec.py, details: README.md.
//
// Registers (32-bit, byte addresses):
//   0x000 VERSION     0x0001_0000
//   0x004 ID          "FFTS" (0x46465453)
//   0x008 CTRL        [0] RUN: averages back to back, [1] IRQ_EN,
//                     [5:4] SRC (0 rx0, 1 rx1, 2 nbiq), [10:8] NB_CH (0..4)
//   0x00C CMD         (wo) [0] START: clear DONE, start one average if idle;
//                     [1] ABORT: reset the FFT and the sequencer, clear DONE
//   0x010 NAVG        [8:0] spectra per average, 1..256 (0 counts as 1)
//   0x014 SCALE       [23:0] FFT scale schedule, 2 bits per radix-2 stage,
//                     stage 0 in [1:0] (reset 0x555400; total shift 7)
//   0x018 RSHIFT      [3:0] result right shift, 0..8 (larger values: 8)
//   0x01C STATUS      (ro) [0] BUSY, [1] DONE, [2] CAPT (loading samples),
//                     [15:8] FRAME (frame of the average being captured)
//   0x020 DONE        [0] result ready; write 1 to clear (interrupt source)
//   0x024 FLAGS       sticky, write 1 to clear: [0] FFT_OVF, [1] SAT,
//                     [2] LOST (sample lost / frame order error), [3] DROP
//   0x028 FRAMES      (ro) FFT frames accumulated since reset
//   0x02C AVGS        (ro) results published since reset
//   0x030 DROPS       (ro) averages dropped (DOUBLE_BUF: result still unread)
//   0x034 RES_CFG     (ro) of the published result: [1:0] SRC, [6:4] NB_CH,
//                     [11:8] RSHIFT, [24:16] NAVG
//   0x038 RES_SCALE   (ro) [23:0] scale schedule of the published result
//   0x03C INFO        (ro) [3:0] log2 FFT length (12), [4] DOUBLE_BUF
//   0x2000 + 4 n : WIN table w[n] = w[4095 - n], n < 2048 (wo, 18-bit unsigned,
//                  2^17 = 1.0; write while idle)
//   0x4000 + 4 j : RESULT word j (ro), FFT-shifted: frequency (j - 2048) fs / 4096
//
// Configuration (CTRL source and channel, NAVG, SCALE, RSHIFT) is taken when
// an average starts; RES_CFG / RES_SCALE describe the result in the buffer.
// START (or RUN) starts an average when the core is idle; with RUN set and
// DOUBLE_BUF = 0 the next average waits until DONE is cleared. With
// DOUBLE_BUF = 1 averages run back to back; one finishing while DONE is still
// set is dropped (FLAGS.DROP, DROPS). irq = DONE & IRQ_EN (level).
//
// Inputs: adc_* in the adc_clk domain (AD9363 l_clk, up to 250 MHz; 16-bit
// two's complement, all four lanes qualified by adc_valid), crossed to
// s_axi_aclk with an asynchronous FIFO; tap_* in the s_axi_aclk domain (the
// nbiq channel outputs, interleaved, one sample per clock with tap_valid).

module fft_axi #(
  parameter string WIN_FILE   = "fft_win.mem",
  parameter bit    DOUBLE_BUF = 1'b0
) (
  input  logic        s_axi_aclk,
  input  logic        s_axi_aresetn,
  input  logic [15:0] s_axi_awaddr,
  input  logic        s_axi_awvalid,
  output logic        s_axi_awready,
  input  logic [31:0] s_axi_wdata,
  input  logic        s_axi_wvalid,
  output logic        s_axi_wready,
  output logic [1:0]  s_axi_bresp,
  output logic        s_axi_bvalid,
  input  logic        s_axi_bready,
  input  logic [15:0] s_axi_araddr,
  input  logic        s_axi_arvalid,
  output logic        s_axi_arready,
  output logic [31:0] s_axi_rdata,
  output logic [1:0]  s_axi_rresp,
  output logic        s_axi_rvalid,
  input  logic        s_axi_rready,

  output logic        irq,

  input  logic        adc_clk,
  input  logic        adc_valid,
  input  logic [15:0] adc_data_i0,
  input  logic [15:0] adc_data_q0,
  input  logic [15:0] adc_data_i1,
  input  logic [15:0] adc_data_q1,

  input  logic        tap_valid,
  input  logic [2:0]  tap_ch,
  input  logic [15:0] tap_i,
  input  logic [15:0] tap_q
);

  wire clk = s_axi_aclk;
  assign s_axi_bresp = 2'b00;
  assign s_axi_rresp = 2'b00;

  localparam logic [31:0] VERSION = 32'h0001_0000;
  localparam logic [31:0] ID      = 32'h4646_5453;

  // ---------------------------------------------------------------- reset
  logic [3:0] srst_cnt = '1;
  logic       rst = 1'b1, aresetn_q = 1'b0;
  always_ff @(posedge clk) begin
    aresetn_q <= s_axi_aresetn;
    if (!aresetn_q)         srst_cnt <= '1;
    else if (srst_cnt != 0) srst_cnt <= srst_cnt - 1'b1;
    rst <= !aresetn_q || (srst_cnt != 0);
  end

  // ---------------------------------------------------------------- registers
  logic        run, irq_en;
  logic [1:0]  src;
  logic [2:0]  nbch;
  logic [8:0]  navg;
  logic [23:0] scale;
  logic [3:0]  rshift;
  logic        cmd_start, cmd_abort, done_clr;
  logic [3:0]  flags_clr;
  logic        win_we;
  logic [10:0] win_wa;
  logic [17:0] win_wd;

  logic        aw_hold, w_hold;
  logic [15:0] awaddr_q;
  logic [31:0] wdata_q;
  assign s_axi_awready = !aw_hold;
  assign s_axi_wready  = !w_hold;

  always_ff @(posedge clk) begin
    cmd_start <= 1'b0;
    cmd_abort <= 1'b0;
    done_clr  <= 1'b0;
    flags_clr <= '0;
    win_we    <= 1'b0;
    if (rst) begin
      aw_hold <= 1'b0; w_hold <= 1'b0; s_axi_bvalid <= 1'b0;
      run <= 1'b0; irq_en <= 1'b0; src <= '0; nbch <= '0;
      navg <= 9'd1; scale <= 24'h555400; rshift <= '0;
      win_wa <= '0; win_wd <= '0;
    end else begin
      if (s_axi_awvalid && s_axi_awready) begin aw_hold <= 1'b1; awaddr_q <= s_axi_awaddr; end
      if (s_axi_wvalid && s_axi_wready)   begin w_hold  <= 1'b1; wdata_q  <= s_axi_wdata;  end
      if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
      if (aw_hold && w_hold && !s_axi_bvalid) begin
        aw_hold <= 1'b0; w_hold <= 1'b0; s_axi_bvalid <= 1'b1;
        if (awaddr_q[14]) ;                                   // result: read-only
        else if (awaddr_q[13]) begin
          win_we <= 1'b1;
          win_wa <= awaddr_q[12:2];
          win_wd <= wdata_q[17:0];
        end else
          case (awaddr_q[7:2])
            6'h02: begin
              run <= wdata_q[0]; irq_en <= wdata_q[1];
              src <= wdata_q[5:4]; nbch <= wdata_q[10:8];
            end
            6'h03: begin cmd_start <= wdata_q[0]; cmd_abort <= wdata_q[1]; end
            6'h04: navg <= wdata_q[8:0];
            6'h05: scale <= wdata_q[23:0];
            6'h06: rshift <= (wdata_q[3:0] > 4'd8) ? 4'd8 : wdata_q[3:0];
            6'h08: done_clr <= wdata_q[0];
            6'h09: flags_clr <= wdata_q[3:0];
            default: ;
          endcase
      end
    end
  end

  // ABORT: reset the FFT core (aresetn >= 2 clocks) and the data path
  logic [3:0] abort_cnt = '1;
  logic       srst = 1'b1;
  always_ff @(posedge clk) begin
    if (rst || cmd_abort) abort_cnt <= '1;
    else if (abort_cnt != 0) abort_cnt <= abort_cnt - 1'b1;
    srst <= rst || cmd_abort || (abort_cnt != 0);
  end

  // ---------------------------------------------------------------- ADC -> clk
  (* ASYNC_REG = "TRUE" *) logic [1:0] rst_s;
  logic        ad_v;
  logic [63:0] ad_d;
  always_ff @(posedge adc_clk) begin
    rst_s <= {rst_s[0], rst};
    ad_v <= adc_valid;
    ad_d <= {adc_data_q1, adc_data_i1, adc_data_q0, adc_data_i0};
  end

  logic        a_valid;
  logic [63:0] a_data;
  dvbs2_async_fifo #(.W(64), .AW(4)) u_adc_fifo (
    .wclk(adc_clk), .wrst(rst_s[1]), .w_valid(ad_v), .w_ready(), .w_data(ad_d),
    .rclk(clk), .rrst(rst), .r_valid(a_valid), .r_ready(1'b1), .r_data(a_data));

  // ---------------------------------------------------------------- source select
  logic [1:0]  a_src;                  // configuration of the running average
  logic [2:0]  a_ch;
  logic [7:0]  a_nm1;
  logic [23:0] a_scale;
  logic [3:0]  a_rsh;

  logic        t_v;
  logic [2:0]  t_ch;
  logic [15:0] t_i, t_q;
  // first registers after the FIFO's LUTRAM read port: plain flip-flops
  (* shreg_extract = "no" *) logic        s_v;
  (* shreg_extract = "no" *) logic [15:0] s_i, s_q;
  always_ff @(posedge clk) begin
    t_v <= tap_valid; t_ch <= tap_ch; t_i <= tap_i; t_q <= tap_q;
    s_v <= a_src[1] ? (t_v && t_ch == a_ch) : a_valid;
    s_i <= a_src[1] ? t_i : (a_src[0] ? a_data[47:32] : a_data[15:0]);
    s_q <= a_src[1] ? t_q : (a_src[0] ? a_data[63:48] : a_data[31:16]);
  end

  // ---------------------------------------------------------------- sequencer
  typedef enum logic [2:0] {C_IDLE, C_CFG, C_WAIT, C_CAPT, C_DRAIN} cst_t;
  cst_t        cst;
  logic        done;
  logic [7:0]  f_in;
  logic [11:0] n_in;
  logic [4:0]  guard;
  logic        cfg_v, cfg_r, fft_rdy;
  logic        p_v;
  logic [11:0] p_n;
  logic [15:0] p_i, p_q;
  logic        avg_end, avg_pub;

  wire go = (cst == C_IDLE) && (cmd_start || (run && (DOUBLE_BUF || !done)));

  // A frame is in the FFT from its last input sample until its last output
  // sample. The radix-2 Lite core cannot load while it processes or unloads,
  // and its input FIFO would take the first samples of the next frame early:
  // the next frame is captured only when the core is empty.
  logic in_core, w_v, w_last, m_v, m_last;
  always_ff @(posedge clk)
    if (srst) in_core <= 1'b0;
    else if (w_v && w_last && fft_rdy) in_core <= 1'b1;
    else if (m_v && m_last) in_core <= 1'b0;

  always_ff @(posedge clk) begin
    p_v <= 1'b0;
    p_n <= n_in; p_i <= s_i; p_q <= s_q;
    if (srst) begin
      cst <= C_IDLE; cfg_v <= 1'b0; f_in <= '0; n_in <= '0; guard <= '0;
    end else begin
      if (guard != 0) guard <= guard - 1'b1;
      case (cst)
        C_IDLE:
          if (go) begin
            a_src <= src; a_ch <= nbch;
            a_nm1 <= (navg == 0) ? 8'd0 : (navg > 9'd256) ? 8'd255 : 8'(navg - 1'b1);
            a_scale <= scale; a_rsh <= rshift;
            f_in <= '0;
            cfg_v <= 1'b1;
            cst <= C_CFG;
          end
        C_CFG:
          if (cfg_r) begin cfg_v <= 1'b0; guard <= '1; cst <= C_WAIT; end
        C_WAIT:                          // window pipeline and FFT empty
          if (guard == 0 && !in_core && fft_rdy) begin n_in <= '0; cst <= C_CAPT; end
        C_CAPT:
          if (s_v) begin
            p_v <= 1'b1;
            n_in <= n_in + 1'b1;
            if (n_in == 12'hFFF) begin
              guard <= '1;
              if (f_in == a_nm1) cst <= C_DRAIN;
              else begin f_in <= f_in + 1'b1; cst <= C_WAIT; end
            end
          end
        C_DRAIN:
          if (avg_end) cst <= C_IDLE;
        default: cst <= C_IDLE;
      endcase
    end
  end

  // ---------------------------------------------------------------- window, FFT
  logic [15:0] w_i, w_q;
  fft_win #(.WIN_FILE(WIN_FILE)) u_win (
    .clk, .srst, .we(win_we), .wa(win_wa), .wd(win_wd),
    .in_v(p_v), .in_n(p_n), .in_i(p_i), .in_q(p_q),
    .out_v(w_v), .out_last(w_last), .out_i(w_i), .out_q(w_q));

  logic [31:0] m_d;
  logic        ev_ovf, ev_tl_unexp, ev_tl_miss;
  fft_xfft u_fft (
    .aclk(clk), .aresetn(!srst),
    .s_axis_config_tdata({7'd0, a_scale, 1'b1}),            // [0] forward
    .s_axis_config_tvalid(cfg_v), .s_axis_config_tready(cfg_r),
    .s_axis_data_tdata({w_q, w_i}), .s_axis_data_tvalid(w_v),
    .s_axis_data_tready(fft_rdy), .s_axis_data_tlast(w_last),
    .m_axis_data_tdata(m_d), .m_axis_data_tuser(), .m_axis_data_tvalid(m_v),
    .m_axis_data_tready(1'b1), .m_axis_data_tlast(m_last),
    .m_axis_status_tdata(), .m_axis_status_tvalid(), .m_axis_status_tready(1'b1),
    .event_frame_started(), .event_tlast_unexpected(ev_tl_unexp),
    .event_tlast_missing(ev_tl_miss), .event_fft_overflow(ev_ovf),
    .event_status_channel_halt(), .event_data_in_channel_halt(),
    .event_data_out_channel_halt());

  // ---------------------------------------------------------------- power, average
  logic        frame_end, sat_p, order_err;
  logic [11:0] rd_addr;
  logic [31:0] rd_data;
  fft_acc #(.DOUBLE_BUF(DOUBLE_BUF)) u_acc (
    .clk, .srst, .restart(go), .nm1(a_nm1), .rsh(a_rsh), .keep(done),
    .s_valid(m_v), .s_data(m_d), .s_last(m_last),
    .frame_end, .avg_end, .avg_pub, .sat(sat_p), .order_err,
    .rd_addr, .rd_data);

  // ---------------------------------------------------------------- status
  logic [3:0]  flags;
  logic [31:0] frames, avgs, drops;
  logic [24:0] res_cfg;
  logic [23:0] res_scale;
  always_ff @(posedge clk) begin
    if (rst) begin
      done <= 1'b0; flags <= '0; frames <= '0; avgs <= '0; drops <= '0;
      res_cfg <= '0; res_scale <= '0;
    end else begin
      if (done_clr || cmd_start || cmd_abort) done <= 1'b0;
      if (avg_end && !srst) begin
        if (avg_pub) begin
          done <= 1'b1;
          avgs <= avgs + 1'b1;
          res_cfg <= {9'(a_nm1) + 9'd1, 4'd0, a_rsh, 1'b0, a_ch, 2'd0, a_src};
          res_scale <= a_scale;
        end else
          drops <= drops + 1'b1;
      end
      if (frame_end && !srst) frames <= frames + 1'b1;
      flags <= (flags & ~flags_clr)
             | {avg_end && !avg_pub && !srst,
                (w_v && !fft_rdy) || ev_tl_unexp || ev_tl_miss || order_err,
                sat_p, ev_ovf};
    end
  end

  always_ff @(posedge clk) irq <= done && irq_en;

  // ---------------------------------------------------------------- AXI-Lite read
  logic [2:0]  ar_p;
  logic [15:0] ar_q;
  assign rd_addr = ar_q[13:2];
  always_ff @(posedge clk) begin
    if (rst) begin
      s_axi_arready <= 1'b1; s_axi_rvalid <= 1'b0; ar_p <= '0; ar_q <= '0;
    end else begin
      ar_p <= {ar_p[1:0], 1'b0};
      if (s_axi_arvalid && s_axi_arready) begin
        s_axi_arready <= 1'b0;
        ar_p[0] <= 1'b1;
        ar_q <= s_axi_araddr;
      end
      if (ar_p[2]) begin                 // result RAM data valid now
        s_axi_rvalid <= 1'b1;
        s_axi_rdata <= '0;
        if (ar_q[14]) s_axi_rdata <= rd_data;
        else if (!ar_q[13])
          case (ar_q[7:2])
            6'h00: s_axi_rdata <= VERSION;
            6'h01: s_axi_rdata <= ID;
            6'h02: s_axi_rdata <= {21'd0, nbch, 2'd0, src, 2'd0, irq_en, run};
            6'h04: s_axi_rdata <= {23'd0, navg};
            6'h05: s_axi_rdata <= {8'd0, scale};
            6'h06: s_axi_rdata <= {28'd0, rshift};
            6'h07: s_axi_rdata <= {16'd0, f_in, 5'd0, cst == C_CAPT, done, cst != C_IDLE};
            6'h08: s_axi_rdata <= {31'd0, done};
            6'h09: s_axi_rdata <= {28'd0, flags};
            6'h0A: s_axi_rdata <= frames;
            6'h0B: s_axi_rdata <= avgs;
            6'h0C: s_axi_rdata <= drops;
            6'h0D: s_axi_rdata <= {7'd0, res_cfg};
            6'h0E: s_axi_rdata <= {8'd0, res_scale};
            6'h0F: s_axi_rdata <= {27'd0, DOUBLE_BUF, 4'd12};
            default: ;
          endcase
      end
      if (s_axi_rvalid && s_axi_rready) begin
        s_axi_rvalid <= 1'b0;
        s_axi_arready <= 1'b1;
      end
    end
  end

endmodule
