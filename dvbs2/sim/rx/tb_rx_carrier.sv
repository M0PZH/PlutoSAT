// File-driven testbench for dvbs2_rx_carrier.
//   +in=<file> "iiii qqqq sof dummy found"; +out=<file> "ll qq first last"
//   +modcod= +pilots= +xa= +xb= +kp= +ki= +stall=<percent>
`timescale 1ns/1ps
module tb_rx_carrier;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;

  logic [4:0] modcod;
  logic pilots;
  logic [17:0] gold_xa, gold_xb, pll_kp, pll_ki;
  logic s_valid = 0, s_ready;
  logic signed [15:0] s_i, s_q;
  logic s_sof, s_dummy, s_found;
  logic ty_v, ty_dummy;
  logic [15:0] st_acq, st_fa, st_retype;
  logic st_acquired;
  logic snap_v;
  logic signed [15:0] snap_i, snap_q;
  logic m_valid, m_ready = 0;
  logic [7:0] m_llr_i, m_llr_q;
  logic m_first, m_last;
  logic [31:0] st_freq, st_amp;
  logic [35:0] st_energy;
  logic [17:0] st_gain;
  logic [15:0] st_frames;

  dvbs2_rx_carrier #(.LUT_FILE("../../rtl/gen/nco_lut.mem")) dut (.*);

  string fin, fout;
  int fi, fo, stall, n_in, n_out, rc, tmp, a, b, c;
  logic [15:0] vi, vq;
  logic eof = 0;

  initial begin
    if (!$value$plusargs("in=%s", fin)) $fatal(1, "+in");
    if (!$value$plusargs("out=%s", fout)) $fatal(1, "+out");
    if (!$value$plusargs("modcod=%d", tmp)) tmp = 4;
    modcod = 5'(tmp);
    if (!$value$plusargs("pilots=%d", tmp)) tmp = 1;
    pilots = tmp[0];
    if (!$value$plusargs("xa=%d", tmp)) tmp = 1;
    gold_xa = 18'(tmp);
    if (!$value$plusargs("xb=%d", tmp)) tmp = 0;
    gold_xb = 18'(tmp);
    if (!$value$plusargs("kp=%d", tmp)) tmp = 0;
    pll_kp = 18'(tmp);
    if (!$value$plusargs("ki=%d", tmp)) tmp = 0;
    pll_ki = 18'(tmp);
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
        rc = $fscanf(fi, "%h %h %d %d %d", vi, vq, a, b, c);
        if (rc == 5) begin
          s_valid <= 1; s_i <= vi; s_q <= vq; s_sof <= a[0]; s_dummy <= b[0]; s_found <= c[0]; n_in++;
        end else eof <= 1;
      end
    end
  end

  // +trace: frequency updates (debug)
  int trace;
  initial trace = $test$plusargs("trace");
  logic [31:0] fhat_q;
  always @(posedge clk) if (trace != 0) begin
    fhat_q <= dut.fhat;
    if (dut.fhat != fhat_q) $display("F %0d %0d %s", $signed(dut.fhat), n_in, dut.st.name());
    if (dut.st.name() == "M_FLL_END") $display("FF %0d nhdr %0d acq %0d", $signed(dut.ff), dut.nhdr, dut.acquired);
    if (dut.st.name() == "M_FLL_CORW2") $display("ST %0d ang %0d R %0d %0d", dut.fst, $signed(dut.p_ang), $signed(dut.cor_x), $signed(dut.cor_y));
  end

  int idle = 0;
  always @(posedge clk) begin
    m_ready <= ($urandom % 100) >= stall;
    if (m_valid && m_ready) begin
      $fdisplay(fo, "%02h %02h %0d %0d", m_llr_i, m_llr_q, m_first, m_last);
      n_out++;
      idle = 0;
    end else idle++;
    if (eof && idle > 20000) begin
      $display("tb_rx_carrier: %0d in, %0d out, frames %0d freq %0d amp %0d energy %0d gain %0d acq %0d fa %0d retype %0d",
               n_in, n_out, st_frames, $signed(st_freq), st_amp, st_energy, st_gain, st_acq, st_fa, st_retype);
      $fclose(fo);
      $finish;
    end
  end
endmodule
