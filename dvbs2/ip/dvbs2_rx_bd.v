// Verilog-2001 top of the DVB-S2 receiver (two receive slots) for Vivado
// block-design module reference (a module reference top cannot be
// SystemVerilog). See dvbs2_rx_axi.sv for the register map (10 address
// bits: slot banks 0x000, 0x100, shared 0x200) and the DMA packet format. Add
// rtl/*.sv, rtl/gen/*.sv and rtl/gen/{nco_lut,ldpc_dec_rom}.mem to the
// project together with this file.
// m_axi: the LLR queue's 64-bit AXI3 master (dvbs2_rx_llrq.sv), for a Zynq
// S_AXI_HP port in its default 64-bit mode (fishball: S_AXI_HP1).

module dvbs2_rx_bd #(
  // plain file names: Vivado finds .mem files that are part of the project
  parameter LUT_FILE      = "nco_lut.mem",
  parameter LDPC_ROM_FILE = "ldpc_dec_rom.mem"
) (
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi:m_axis:m_axi, ASSOCIATED_RESET s_axi_aresetn" *)
  input  wire        s_axi_aclk,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  input  wire        s_axi_aresetn,
  input  wire [9:0]  s_axi_awaddr,
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
  input  wire [9:0]  s_axi_araddr,
  input  wire [2:0]  s_axi_arprot,
  input  wire        s_axi_arvalid,
  output wire        s_axi_arready,
  output wire [31:0] s_axi_rdata,
  output wire [1:0]  s_axi_rresp,
  output wire        s_axi_rvalid,
  input  wire        s_axi_rready,

  output wire [63:0] m_axis_tdata,
  output wire        m_axis_tvalid,
  input  wire        m_axis_tready,
  output wire        m_axis_tlast,

  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 adc_clk CLK" *)
  input  wire        adc_clk,
  input  wire        adc_valid,
  input  wire [15:0] adc_data_i,
  input  wire [15:0] adc_data_q,

  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWID" *)
  (* X_INTERFACE_PARAMETER = "PROTOCOL AXI3, DATA_WIDTH 64, ID_WIDTH 6, READ_WRITE_MODE READ_WRITE, MAX_BURST_LENGTH 16, NUM_WRITE_OUTSTANDING 4, NUM_READ_OUTSTANDING 4" *)
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
  output wire        m_axi_rready
);

  assign m_axi_awid   = 6'd0;
  assign m_axi_awlock = 2'd0;
  assign m_axi_awprot = 3'd0;
  assign m_axi_awqos  = 4'd0;
  assign m_axi_wid    = 6'd0;
  assign m_axi_arid   = 6'd0;
  assign m_axi_arlock = 2'd0;
  assign m_axi_arprot = 3'd0;
  assign m_axi_arqos  = 4'd0;

  dvbs2_rx_axi #(
    .LUT_FILE      (LUT_FILE),
    .LDPC_ROM_FILE (LDPC_ROM_FILE)
  ) i_dvbs2_rx_axi (
    .s_axi_aclk    (s_axi_aclk),
    .s_axi_aresetn (s_axi_aresetn),
    .s_axi_awaddr  (s_axi_awaddr),
    .s_axi_awvalid (s_axi_awvalid),
    .s_axi_awready (s_axi_awready),
    .s_axi_wdata   (s_axi_wdata),
    .s_axi_wstrb   (s_axi_wstrb),
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
    .m_axis_tdata  (m_axis_tdata),
    .m_axis_tvalid (m_axis_tvalid),
    .m_axis_tready (m_axis_tready),
    .m_axis_tlast  (m_axis_tlast),
    .adc_clk       (adc_clk),
    .adc_valid     (adc_valid),
    .adc_data_i    (adc_data_i),
    .adc_data_q    (adc_data_q),
    .m_axi_awaddr  (m_axi_awaddr),
    .m_axi_awlen   (m_axi_awlen),
    .m_axi_awsize  (m_axi_awsize),
    .m_axi_awburst (m_axi_awburst),
    .m_axi_awcache (m_axi_awcache),
    .m_axi_awvalid (m_axi_awvalid),
    .m_axi_awready (m_axi_awready),
    .m_axi_wdata   (m_axi_wdata),
    .m_axi_wstrb   (m_axi_wstrb),
    .m_axi_wlast   (m_axi_wlast),
    .m_axi_wvalid  (m_axi_wvalid),
    .m_axi_wready  (m_axi_wready),
    .m_axi_bresp   (m_axi_bresp),
    .m_axi_bvalid  (m_axi_bvalid),
    .m_axi_bready  (m_axi_bready),
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
    .m_axi_rready  (m_axi_rready)
  );

endmodule
