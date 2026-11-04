// Narrowband channel samples -> one DDR ring buffer per channel, through an
// AXI3 write master of DW = 32 or 64 bits (64 for a Zynq S_AXI_HP port in
// its default 64-bit mode, 32 for S_AXI_GP).
//
// Each channel collects its samples ({Q, I}, 16 bits each) in a 64-entry
// staging buffer and writes them as 64-byte bursts (16 beats at DW 32, 8 at
// DW 64; lane l of a beat holds sample 2 k + l, i.e. little-endian) to
// base | (wptr & mask); wptr (bytes, wraps at 2^32) advances when a burst's
// response is back, so everything below it is in memory. base must be
// aligned to mask + 1, a power of two >= 4096. A sample that finds its
// staging buffer full is dropped and counted. While a channel is disabled
// its staging buffer and wptr are cleared. Channels with a full burst are
// served round-robin.

module nbiq_wr #(
  parameter int NCH = 5,
  parameter int DW  = 32                     // 32 or 64
) (
  input  logic               clk,
  input  logic               rst,
  input  logic [NCH-1:0]     en,
  input  logic [31:0]        base [NCH],
  input  logic [31:0]        mask [NCH],

  input  logic               s_valid,
  input  logic [2:0]         s_ch,
  input  logic [31:0]        s_data,

  output logic [31:0]        wptr [NCH],
  output logic [15:0]        st_ovf [NCH],

  output logic [31:0]        m_axi_awaddr,
  output logic [3:0]         m_axi_awlen,
  output logic [2:0]         m_axi_awsize,
  output logic [1:0]         m_axi_awburst,
  output logic [3:0]         m_axi_awcache,
  output logic               m_axi_awvalid,
  input  logic               m_axi_awready,
  output logic [DW-1:0]      m_axi_wdata,
  output logic [DW/8-1:0]    m_axi_wstrb,
  output logic               m_axi_wlast,
  output logic               m_axi_wvalid,
  input  logic               m_axi_wready,
  input  logic [1:0]         m_axi_bresp,
  input  logic               m_axi_bvalid,
  output logic               m_axi_bready
);

  localparam int L     = DW / 32;            // samples per beat
  localparam int LB    = $clog2(L);
  localparam int BEATS = 16 / L;              // beats per 64-byte burst
  localparam int AW    = 6 - LB;              // lane address bits per channel

  initial assert (DW == 32 || DW == 64) else $fatal(1, "nbiq_wr: DW must be 32 or 64");

  assign m_axi_awlen   = 4'(BEATS - 1);
  assign m_axi_awsize  = 3'(2 + LB);
  assign m_axi_awburst = 2'd1;
  assign m_axi_awcache = 4'b0011;
  assign m_axi_wstrb   = '1;
  assign m_axi_bready  = 1'b1;

  // ------------------------------------------------------------ staging
  logic [6:0] wc [NCH], rc [NCH];             // written / read, mod 128

  // burst state
  typedef enum logic [1:0] {W_IDLE, W_BURST, W_RESP} wst_t;
  wst_t        st;
  logic [2:0]  bc, rr;
  logic [5:0]  rb;                            // rc[bc] at the burst start
  logic        aw_done, w_done, kill;
  logic [3:0]  beat;

  wire s_ok = s_valid && en[s_ch] && 7'(wc[s_ch] - rc[s_ch]) != 7'd64;

  // one 64/L-deep buffer per lane and channel: sample n of a channel goes to
  // lane n % L, so a beat reads all lanes at one address (bursts start at
  // multiples of 16 samples)
  for (genvar l = 0; l < L; l++) begin : g_lane
    (* ram_style = "distributed" *) logic [31:0] stg [NCH * 64 / L];
    always_ff @(posedge clk) begin
      if (s_ok && 32'(wc[s_ch][5:0]) % L == l)
        stg[{s_ch, AW'(wc[s_ch][5:0] >> LB)}] <= s_data;
    end
    assign m_axi_wdata[32 * l +: 32] = stg[{bc, AW'(32'(rb >> LB) + 32'(beat))}];
  end
  assign m_axi_wlast = beat == 4'(BEATS - 1);

  // round-robin: the first channel with a full burst from rr on
  logic [NCH-1:0] rdy;
  logic           pick;
  logic [2:0]     pc;
  always_comb begin
    for (int c = 0; c < NCH; c++) rdy[c] = en[c] && 7'(wc[c] - rc[c]) >= 7'd16;
    pick = 1'b0; pc = '0;
    for (int c = NCH - 1; c >= 0; c--)
      if (rdy[c]) begin pick = 1'b1; pc = 3'(c); end
    for (int c = NCH - 1; c >= 0; c--)
      if (rdy[c] && 3'(c) >= rr) pc = 3'(c);
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      st <= W_IDLE;
      m_axi_awvalid <= 1'b0;
      m_axi_wvalid <= 1'b0;
      rr <= '0;
      for (int c = 0; c < NCH; c++) st_ovf[c] <= '0;   // wc, rc, wptr: cleared by !en
    end else begin
      // staging writes
      if (s_ok) wc[s_ch] <= wc[s_ch] + 7'd1;
      else if (s_valid && en[s_ch]) st_ovf[s_ch] <= st_ovf[s_ch] + 16'd1;

      if (st != W_IDLE && !en[bc]) kill <= 1'b1;

      case (st)
        W_IDLE: if (pick) begin
          bc <= pc; rr <= (pc == 3'(NCH - 1)) ? 3'd0 : pc + 3'd1;
          rb <= rc[pc][5:0];
          m_axi_awaddr <= base[pc] | (wptr[pc] & mask[pc]);
          m_axi_awvalid <= 1'b1;
          m_axi_wvalid <= 1'b1;
          aw_done <= 1'b0; w_done <= 1'b0; kill <= 1'b0;
          beat <= '0;
          st <= W_BURST;
        end

        W_BURST: begin
          if (m_axi_awvalid && m_axi_awready) begin
            m_axi_awvalid <= 1'b0;
            aw_done <= 1'b1;
          end
          if (m_axi_wvalid && m_axi_wready) begin
            beat <= beat + 4'd1;
            if (m_axi_wlast) begin
              m_axi_wvalid <= 1'b0;
              w_done <= 1'b1;
            end
          end
          if ((aw_done || (m_axi_awvalid && m_axi_awready)) &&
              (w_done || (m_axi_wvalid && m_axi_wready && m_axi_wlast)))
            st <= W_RESP;
        end

        W_RESP: if (m_axi_bvalid) begin
          if (!kill && en[bc]) begin
            rc[bc] <= rc[bc] + 7'd16;
            wptr[bc] <= wptr[bc] + 32'd64;
          end
          st <= W_IDLE;
        end

        default: st <= W_IDLE;
      endcase

      for (int c = 0; c < NCH; c++)
        if (!en[c]) begin wc[c] <= '0; rc[c] <= '0; wptr[c] <= '0; end
    end
  end

endmodule
