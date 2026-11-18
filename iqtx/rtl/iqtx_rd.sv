// DDR ring reader of the IQ transmitter: 64-byte AXI3 read bursts (8 beats
// of 64 bits, a Zynq S_AXI_HP port in its default 64-bit mode) from
// base | (rptr & mask) into a 32-beat (64-sample) prefetch FIFO in LUTRAM.
//
// rptr counts the bytes fetched (wraps at 2^32) and advances by 64 when a
// burst's last beat is in the FIFO. A burst is issued when run is set, no
// burst is in flight, the FIFO has room for it and the ring holds at least
// 64 bytes: (wptr & ~63) - rptr in 64 .. mask + 1 (a write pointer behind
// rptr or more than a ring ahead is ignored). drop (a pulse, while !busy)
// empties the FIFO and sets rptr to wptr & ~63: whatever the ring still
// holds is discarded.
//
// Samples ({Q, I}, 32 bits) leave the FIFO lowest address first (lane 0 of
// a beat = bits [31:0]).

module iqtx_rd (
  input  logic        clk,
  input  logic        rst,
  input  logic        run,
  input  logic        drop,
  input  logic [31:0] base,
  input  logic [31:0] mask,
  input  logic [31:0] wptr,
  output logic [31:0] rptr,
  output logic        busy,

  output logic        s_valid,
  input  logic        s_ready,
  output logic [31:0] s_data,

  output logic [31:0] m_axi_araddr,
  output logic [3:0]  m_axi_arlen,
  output logic [2:0]  m_axi_arsize,
  output logic [1:0]  m_axi_arburst,
  output logic [3:0]  m_axi_arcache,
  output logic        m_axi_arvalid,
  input  logic        m_axi_arready,
  input  logic [63:0] m_axi_rdata,
  input  logic        m_axi_rlast,
  input  logic        m_axi_rvalid,
  output logic        m_axi_rready
);

  assign m_axi_arlen   = 4'd7;
  assign m_axi_arsize  = 3'd3;
  assign m_axi_arburst = 2'd1;
  assign m_axi_arcache = 4'b0011;
  assign m_axi_rready  = 1'b1;

  (* ram_style = "distributed" *) logic [63:0] mem [32];
  logic [5:0] wr, rd;                 // beats written / read, mod 64
  logic       lane;

  wire [5:0]  used  = wr - rd;
  wire [31:0] avail = {wptr[31:6], 6'd0} - rptr;
  wire        go    = run && !busy && used <= 6'd24 &&
                      avail >= 32'd64 && avail <= mask + 32'd1;

  assign s_valid = used != 6'd0;
  assign s_data  = lane ? mem[rd[4:0]][63:32] : mem[rd[4:0]][31:0];

  always_ff @(posedge clk) begin
    if (m_axi_rvalid && busy) mem[wr[4:0]] <= m_axi_rdata;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      wr <= '0; rd <= '0; lane <= 1'b0;
      rptr <= '0; busy <= 1'b0; m_axi_arvalid <= 1'b0;
    end else begin
      if (go) begin
        busy <= 1'b1;
        m_axi_araddr <= base | (rptr & mask);
        m_axi_arvalid <= 1'b1;
      end
      if (m_axi_arvalid && m_axi_arready) m_axi_arvalid <= 1'b0;
      if (m_axi_rvalid && busy) begin
        wr <= wr + 6'd1;
        if (m_axi_rlast) begin
          busy <= 1'b0;
          rptr <= rptr + 32'd64;
        end
      end
      if (s_valid && s_ready) begin
        lane <= !lane;
        if (lane) rd <= rd + 6'd1;
      end
      if (drop) begin
        wr <= '0; rd <= '0; lane <= 1'b0;
        rptr <= {wptr[31:6], 6'd0};
      end
    end
  end

endmodule
