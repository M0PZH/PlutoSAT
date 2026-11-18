// Verilog-2001 top of the narrowband IQ transmitter for Vivado block-design
// module reference. See rtl/iqtx_axi.sv for the register map. Add rtl/*.sv,
// rtl/gen/iqtx_*.mem and dvbs2/rtl/dvbs2_async_fifo.sv with this file.
// m_axi: 64-bit AXI3 read-only master (write channels tied off), for a Zynq
// S_AXI_HP port in its default 64-bit mode (fishball: S_AXI_HP0).
// dvb_i/q: the DVB-S2 TX output (dvbs2_tx dac_out_i/q); dac_out_i/q: to the
// AD9361 core's DAC channel 0 (the TX source mux is in this core).

module iqtx_bd #(
  parameter FIR_FILE  = "iqtx_fir.mem",
  parameter NCO_FILE  = "iqtx_nco.mem",
  parameter NORM_FILE = "iqtx_norm.mem",
  parameter RAMP_FILE = "iqtx_ramp.mem"
) (
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi:m_axi, ASSOCIATED_RESET s_axi_aresetn" *)
  input  wire        s_axi_aclk,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  input  wire        s_axi_aresetn,
  input  wire [15:0] s_axi_awaddr,
  input  wire [2:0]  s_axi_awprot,
  input  wire        s_axi_awvalid,
  output wire        s_axi_awready,
  input  wire [31:0] s_axi_wdata,
  input  wire [3:0]  s_axi_wstrb,
  input  wire        s_axi_wvalid,
  output wire        s_axi_wready,
  output wire [1:0]  s_axi_bresp,
  output wire        s_axi_bvalid,
  input  wire        s_axi_bready,
  input  wire [15:0] s_axi_araddr,
  input  wire [2:0]  s_axi_arprot,
  input  wire        s_axi_arvalid,
  output wire        s_axi_arready,
  output wire [31:0] s_axi_rdata,
  output wire [1:0]  s_axi_rresp,
  output wire        s_axi_rvalid,
  input  wire        s_axi_rready,

  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWID" *)
  (* X_INTERFACE_PARAMETER = "PROTOCOL AXI3, DATA_WIDTH 64, ID_WIDTH 6, READ_WRITE_MODE READ_WRITE, MAX_BURST_LENGTH 16, NUM_WRITE_OUTSTANDING 1, NUM_READ_OUTSTANDING 1" *)
  output wire [5:0]  m_axi_awid,
  output wire [31:0] m_axi_awaddr,
  output wire [3:0]  m_axi_awlen,
  output wire [2:0]  m_axi_awsize,
  output wire [1:0]  m_axi_awburst,
  output wire [1:0]  m_axi_awlock,
  output wire [3:0]  m_axi_awcache,
  output wire [2:0]  m_axi_awprot,
  output wire [3:0]  m_axi_awqos,
  output wire        m_axi_awvalid,
  input  wire        m_axi_awready,
  output wire [5:0]  m_axi_wid,
  output wire [63:0] m_axi_wdata,
  output wire [7:0]  m_axi_wstrb,
  output wire        m_axi_wlast,
  output wire        m_axi_wvalid,
  input  wire        m_axi_wready,
  input  wire [5:0]  m_axi_bid,
  input  wire [1:0]  m_axi_bresp,
  input  wire        m_axi_bvalid,
  output wire        m_axi_bready,
  output wire [5:0]  m_axi_arid,
  output wire [31:0] m_axi_araddr,
  output wire [3:0]  m_axi_arlen,
  output wire [2:0]  m_axi_arsize,
  output wire [1:0]  m_axi_arburst,
  output wire [1:0]  m_axi_arlock,
  output wire [3:0]  m_axi_arcache,
  output wire [2:0]  m_axi_arprot,
  output wire [3:0]  m_axi_arqos,
  output wire        m_axi_arvalid,
  input  wire        m_axi_arready,
  input  wire [5:0]  m_axi_rid,
  input  wire [63:0] m_axi_rdata,
  input  wire [1:0]  m_axi_rresp,
  input  wire        m_axi_rlast,
  input  wire        m_axi_rvalid,
  output wire        m_axi_rready,

  (* X_INTERFACE_INFO = "xilinx.com:signal:interrupt:1.0 irq INTERRUPT" *)
  (* X_INTERFACE_PARAMETER = "SENSITIVITY LEVEL_HIGH" *)
  output wire        irq,

  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 dac_clk CLK" *)
  input  wire        dac_clk,
  input  wire        dac_valid,
  input  wire [15:0] dvb_i,
  input  wire [15:0] dvb_q,
  output wire [15:0] dac_out_i,
  output wire [15:0] dac_out_q
);

  assign m_axi_awid    = 6'd0;
  assign m_axi_awaddr  = 32'd0;
  assign m_axi_awlen   = 4'd0;
  assign m_axi_awsize  = 3'd3;
  assign m_axi_awburst = 2'd1;
  assign m_axi_awlock  = 2'd0;
  assign m_axi_awcache = 4'd0;
  assign m_axi_awprot  = 3'd0;
  assign m_axi_awqos   = 4'd0;
  assign m_axi_awvalid = 1'b0;
  assign m_axi_wid     = 6'd0;
  assign m_axi_wdata   = 64'd0;
  assign m_axi_wstrb   = 8'd0;
  assign m_axi_wlast   = 1'b0;
  assign m_axi_wvalid  = 1'b0;
  assign m_axi_bready  = 1'b1;
  assign m_axi_arid    = 6'd0;
  assign m_axi_arlock  = 2'd0;
  assign m_axi_arprot  = 3'd0;
  assign m_axi_arqos   = 4'd0;

  iqtx_axi #(.FIR_FILE(FIR_FILE), .NCO_FILE(NCO_FILE), .NORM_FILE(NORM_FILE),
             .RAMP_FILE(RAMP_FILE)) i_iqtx_axi (
    .s_axi_aclk    (s_axi_aclk),
    .s_axi_aresetn (s_axi_aresetn),
    .s_axi_awaddr  (s_axi_awaddr),
    .s_axi_awvalid (s_axi_awvalid),
    .s_axi_awready (s_axi_awready),
    .s_axi_wdata   (s_axi_wdata),
    .s_axi_wvalid  (s_axi_wvalid),
    .s_axi_wready  (s_axi_wready),
    .s_axi_bresp   (s_axi_bresp),
    .s_axi_bvalid  (s_axi_bvalid),
    .s_axi_bready  (s_axi_bready),
    .s_axi_araddr  (s_axi_araddr),
    .s_axi_arvalid (s_axi_arvalid),
    .s_axi_arready (s_axi_arready),
    .s_axi_rdata   (s_axi_rdata),
    .s_axi_rresp   (s_axi_rresp),
    .s_axi_rvalid  (s_axi_rvalid),
    .s_axi_rready  (s_axi_rready),
    .m_axi_araddr  (m_axi_araddr),
    .m_axi_arlen   (m_axi_arlen),
    .m_axi_arsize  (m_axi_arsize),
    .m_axi_arburst (m_axi_arburst),
    .m_axi_arcache (m_axi_arcache),
    .m_axi_arvalid (m_axi_arvalid),
    .m_axi_arready (m_axi_arready),
    .m_axi_rdata   (m_axi_rdata),
    .m_axi_rresp   (m_axi_rresp),
    .m_axi_rlast   (m_axi_rlast),
    .m_axi_rvalid  (m_axi_rvalid),
    .m_axi_rready  (m_axi_rready),
    .irq           (irq),
    .dac_clk       (dac_clk),
    .dac_valid     (dac_valid),
    .dvb_i         (dvb_i),
    .dvb_q         (dvb_q),
    .dac_out_i     (dac_out_i),
    .dac_out_q     (dac_out_q)
  );

endmodule
