// Verilog-2001 top for Vivado block-design module reference (a module
// reference top cannot be SystemVerilog). See dvbs2_tx_axi.sv for the
// register map. Add rtl/*.sv, rtl/gen/dvbs2_tables_pkg.sv and the two
// rtl/gen/*.mem files to the project together with this file.


module dvbs2_tx_bd #(
  // plain file names: Vivado finds .mem files that are part of the project
  parameter LDPC_ROM_FILE = "ldpc_rom.mem",
  parameter NCO_LUT_FILE  = "nco_lut.mem"
) (
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi:s_axis, ASSOCIATED_RESET s_axi_aresetn" *)
  input  wire        s_axi_aclk,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  input  wire        s_axi_aresetn,
  input  wire [7:0]  s_axi_awaddr,
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
  input  wire [7:0]  s_axi_araddr,
  input  wire [2:0]  s_axi_arprot,
  input  wire        s_axi_arvalid,
  output wire        s_axi_arready,
  output wire [31:0] s_axi_rdata,
  output wire [1:0]  s_axi_rresp,
  output wire        s_axi_rvalid,
  input  wire        s_axi_rready,

  input  wire [31:0] s_axis_tdata,
  input  wire        s_axis_tvalid,
  output wire        s_axis_tready,
  input  wire        s_axis_tlast,

  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 dac_clk CLK" *)
  input  wire        dac_clk,
  input  wire        dac_valid,
  input  wire [15:0] dac_in_i,
  input  wire [15:0] dac_in_q,
  output wire [15:0] dac_out_i,
  output wire [15:0] dac_out_q
);

  dvbs2_tx_axi #(
    .LDPC_ROM_FILE (LDPC_ROM_FILE),
    .NCO_LUT_FILE  (NCO_LUT_FILE)
  ) i_dvbs2_tx_axi (
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
    .s_axis_tdata  (s_axis_tdata),
    .s_axis_tvalid (s_axis_tvalid),
    .s_axis_tready (s_axis_tready),
    .s_axis_tlast  (s_axis_tlast),
    .dac_clk       (dac_clk),
    .dac_valid     (dac_valid),
    .dac_in_i      (dac_in_i),
    .dac_in_q      (dac_in_q),
    .dac_out_i     (dac_out_i),
    .dac_out_q     (dac_out_q));

endmodule
