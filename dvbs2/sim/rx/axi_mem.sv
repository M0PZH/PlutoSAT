// AXI3 slave memory model (64-bit, one ID, in-order responses) for the RX
// LLR queue testbenches.
//
// Random ready stalls on AW, W and AR, random latency of each write
// response and each read burst, random R valid gaps (+axistall=<%>,
// +axilat=<max clocks>). Write data becomes visible only when its write
// response is sent (the latest the interconnect may make it visible), so a
// read issued before the response sees the old contents: a master that
// reads its data back before the response would get wrong data. Never
// written words read as 0xDEAD_BEEF_xxxx_xxxx.
//
// Checks ($fatal): the address window [+memlo, +memhi) (hex), 8-byte beats,
// INCR, no burst across a 4 KB boundary, wlast on the burst's last beat,
// valid and payload held until ready.
`timescale 1ps/1ps
module axi_mem (
  input  logic        clk,
  input  logic        rst,
  input  logic [31:0] awaddr,
  input  logic [3:0]  awlen,
  input  logic [2:0]  awsize,
  input  logic [1:0]  awburst,
  input  logic        awvalid,
  output logic        awready,
  input  logic [63:0] wdata,
  input  logic [7:0]  wstrb,
  input  logic        wlast,
  input  logic        wvalid,
  output logic        wready,
  output logic [1:0]  bresp,
  output logic        bvalid,
  input  logic        bready,
  input  logic [31:0] araddr,
  input  logic [3:0]  arlen,
  input  logic [2:0]  arsize,
  input  logic [1:0]  arburst,
  input  logic        arvalid,
  output logic        arready,
  output logic [63:0] rdata,
  output logic [1:0]  rresp,
  output logic        rlast,
  output logic        rvalid,
  input  logic        rready
);
  int stall, lat;
  longint lo, hi;
  initial begin
    if (!$value$plusargs("axistall=%d", stall)) stall = 0;
    if (!$value$plusargs("axilat=%d", lat)) lat = 0;
    if (!$value$plusargs("memlo=%h", lo)) lo = 0;
    if (!$value$plusargs("memhi=%h", hi)) hi = 64'h1_0000_0000;
  end

  logic [63:0] mem [longint];
  longint cyc = 0;

  // write side
  logic [35:0] aw_q [$];                     // {len, addr} accepted, waiting for their W burst
  int          wl_q [$];                     // beats of each W burst (wlast seen)
  logic [63:0] wd_q [$];                     // W beats not yet visible
  int          w_n = 0;                      // beats of the W burst in progress
  longint      b_due [$];                    // write responses: when each may be sent
  logic [35:0] b_aw [$];                     // ... its {len, addr}
  // read side
  longint      r_due [$];
  logic [35:0] r_ar [$];
  int          r_n = 0;
  // statistics
  int n_wb = 0, n_rb = 0, max_b = 0, max_r = 0;

  function automatic void check_burst(input string ch, input logic [31:0] a, input logic [3:0] len,
                                      input logic [2:0] size, input logic [1:0] burst);
    if (size != 3 || burst != 1) $fatal(1, "axi_mem: %s size %0d burst %0d", ch, size, burst);
    if (a[2:0] != 0) $fatal(1, "axi_mem: %s address %h not 8-byte aligned", ch, a);
    if (longint'(a) < lo || longint'(a) + 8 * (longint'(len) + 1) > hi)
      $fatal(1, "axi_mem: %s %h + %0d beats outside [%h, %h)", ch, a, len + 1, lo, hi);
    if (32'(a[11:0]) + 8 * (32'(len) + 1) > 4096) $fatal(1, "axi_mem: %s %h len %0d crosses 4 KB", ch, a, len);
  endfunction

  // valid/payload stability
  logic        aw_hold, w_hold, ar_hold;
  logic [35:0] aw_p, ar_p;
  logic [72:0] w_p;
  always @(posedge clk) begin
    if (aw_hold && (!awvalid || {awlen, awaddr} != aw_p)) $fatal(1, "axi_mem: AW changed before awready");
    if (w_hold && (!wvalid || {wlast, wstrb, wdata} != w_p)) $fatal(1, "axi_mem: W changed before wready");
    if (ar_hold && (!arvalid || {arlen, araddr} != ar_p)) $fatal(1, "axi_mem: AR changed before arready");
    aw_hold <= !rst && awvalid && !awready; aw_p <= {awlen, awaddr};
    w_hold  <= !rst && wvalid && !wready;   w_p  <= {wlast, wstrb, wdata};
    ar_hold <= !rst && arvalid && !arready; ar_p <= {arlen, araddr};
  end

  always @(posedge clk) begin
    cyc++;
    if (rst) begin
      awready <= 0; wready <= 0; arready <= 0; bvalid <= 0; rvalid <= 0;
      bresp <= 0; rresp <= 0;
    end else begin
      // AW
      if (awvalid && awready) begin
        check_burst("AW", awaddr, awlen, awsize, awburst);
        aw_q.push_back({awlen, awaddr});
        n_wb++;
      end
      // W
      if (wvalid && wready) begin
        if (wstrb != 8'hFF) $fatal(1, "axi_mem: wstrb %h", wstrb);
        wd_q.push_back(wdata);
        w_n++;
        if (wlast) begin wl_q.push_back(w_n); w_n = 0; end
        else if (w_n > 16) $fatal(1, "axi_mem: W burst without wlast");
      end
      // a burst with both its AW and all of its W: schedule the response
      while (aw_q.size() > 0 && wl_q.size() > 0) begin
        automatic logic [35:0] a = aw_q.pop_front();
        automatic int n = wl_q.pop_front();
        if (n != int'(a[35:32]) + 1) $fatal(1, "axi_mem: W burst of %0d beats for awlen %0d", n, a[35:32]);
        b_due.push_back(cyc + (lat > 0 ? longint'($urandom % (lat + 1)) : 0));
        b_aw.push_back(a);
      end
      if (b_due.size() > max_b) max_b = b_due.size();
      // B: the data becomes visible with the response
      if (bvalid && bready) bvalid <= 0;
      if ((!bvalid || bready) && b_due.size() > 0 && b_due[0] <= cyc && ($urandom % 100) >= stall) begin
        automatic logic [35:0] a = b_aw.pop_front();
        void'(b_due.pop_front());
        for (int k = 0; k <= int'(a[35:32]); k++) mem[longint'(a[31:0]) + 8 * k] = wd_q.pop_front();
        bvalid <= 1; bresp <= 0;
      end
      // AR
      if (arvalid && arready) begin
        check_burst("AR", araddr, arlen, arsize, arburst);
        r_ar.push_back({arlen, araddr});
        r_due.push_back(cyc + (lat > 0 ? longint'($urandom % (lat + 1)) : 0));
        n_rb++;
      end
      if (r_ar.size() > max_r) max_r = r_ar.size();
      // R
      if (rvalid && rready) rvalid <= 0;
      if ((!rvalid || rready) && r_ar.size() > 0 && r_due[0] <= cyc && ($urandom % 100) >= stall) begin
        automatic logic [35:0] a = r_ar[0];
        automatic longint ad = longint'(a[31:0]) + 8 * r_n;
        rdata <= mem.exists(ad) ? mem[ad] : {32'hDEADBEEF, ad[31:0]};
        rresp <= 0;
        rlast <= r_n == int'(a[35:32]);
        rvalid <= 1;
        if (r_n == int'(a[35:32])) begin
          r_n = 0;
          void'(r_ar.pop_front());
          void'(r_due.pop_front());
        end else r_n++;
      end
      awready <= ($urandom % 100) >= stall;
      wready  <= ($urandom % 100) >= stall;
      arready <= ($urandom % 100) >= stall;
    end
  end
endmodule
