// Narrowband channel samples -> one DDR ring buffer per channel, through a
// 32-bit AXI3 write master (Zynq S_AXI_GP0).
//
// Each channel collects its samples ({Q, I}, 16 bits each) in a 64-entry
// staging buffer and writes them as 16-beat bursts (64 bytes) to
// base | (wptr & mask); wptr (bytes, wraps at 2^32) advances when a burst's
// response is back, so everything below it is in memory. base must be
// aligned to mask + 1, a power of two >= 4096. A sample that finds its
// staging buffer full is dropped and counted. While a channel is disabled
// its staging buffer and wptr are cleared.

module nbiq_wr #(
  parameter int NCH = 4
) (
  input  logic               clk,
  input  logic               rst,
  input  logic [NCH-1:0]     en,
  input  logic [31:0]        base [NCH],
  input  logic [31:0]        mask [NCH],

  input  logic               s_valid,
  input  logic [1:0]         s_ch,
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
  output logic [31:0]        m_axi_wdata,
  output logic [3:0]         m_axi_wstrb,
  output logic               m_axi_wlast,
  output logic               m_axi_wvalid,
  input  logic               m_axi_wready,
  input  logic [1:0]         m_axi_bresp,
  input  logic               m_axi_bvalid,
  output logic               m_axi_bready
);

  assign m_axi_awlen   = 4'd15;
  assign m_axi_awsize  = 3'd2;
  assign m_axi_awburst = 2'd1;
  assign m_axi_awcache = 4'b0011;
  assign m_axi_wstrb   = 4'hF;
  assign m_axi_bready  = 1'b1;

  // ------------------------------------------------------------ staging
  (* ram_style = "distributed" *) logic [31:0] stg [256];
  logic [6:0] wc [NCH], rc [NCH];             // written / read, mod 128

  // burst state
  typedef enum logic [1:0] {W_IDLE, W_BURST, W_RESP} wst_t;
  wst_t        st;
  logic [1:0]  bc, rr;
  logic        aw_done, w_done, kill;
  logic [3:0]  beat;

  wire s_ok = s_valid && en[s_ch] && 7'(wc[s_ch] - rc[s_ch]) != 7'd64;
  always_ff @(posedge clk) begin
    if (s_ok) stg[{s_ch, wc[s_ch][5:0]}] <= s_data;
  end

  assign m_axi_wdata = stg[{bc, 6'(rc[bc][5:0] + 6'(beat))}];
  assign m_axi_wlast = beat == 4'd15;

  logic       pick;
  logic [1:0] pc;
  always_comb begin
    pick = 1'b0; pc = '0;
    for (int k = NCH - 1; k >= 0; k--)
      if (en[2'(rr + 2'(k))] && 7'(wc[2'(rr + 2'(k))] - rc[2'(rr + 2'(k))]) >= 7'd16) begin
        pick = 1'b1; pc = 2'(rr + 2'(k));
      end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      st <= W_IDLE;
      m_axi_awvalid <= 1'b0;
      m_axi_wvalid <= 1'b0;
      rr <= '0;
      for (int c = 0; c < NCH; c++) begin
        wc[c] <= '0; rc[c] <= '0; wptr[c] <= '0; st_ovf[c] <= '0;
      end
    end else begin
      // staging writes
      if (s_ok) wc[s_ch] <= wc[s_ch] + 7'd1;
      else if (s_valid && en[s_ch]) st_ovf[s_ch] <= st_ovf[s_ch] + 16'd1;

      if (st != W_IDLE && !en[bc]) kill <= 1'b1;

      case (st)
        W_IDLE: if (pick) begin
          bc <= pc; rr <= pc + 2'd1;
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
