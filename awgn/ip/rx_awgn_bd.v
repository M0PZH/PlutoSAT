// Verilog-2001 top of the self-test noise source for Vivado block-design
// module reference. See rtl/rx_awgn_axi.sv for the registers. Sits between
// axi_ad9361's receive outputs and everything that uses them; all four
// lanes are delayed by the same 6 ADC clocks.

module rx_awgn_bd (
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn" *)
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

  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 adc_clk CLK" *)
  input  wire        adc_clk,
  input  wire        adc_valid_i0, adc_valid_q0, adc_valid_i1, adc_valid_q1,
  input  wire [15:0] adc_data_i0, adc_data_q0, adc_data_i1, adc_data_q1,
  output wire        out_valid_i0, out_valid_q0, out_valid_i1, out_valid_q1,
  output wire [15:0] out_data_i0, out_data_q0, out_data_i1, out_data_q1
);

  rx_awgn_axi i_rx_awgn_axi (
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
    .adc_clk       (adc_clk),
    .in_valid      ({adc_valid_q1, adc_valid_i1, adc_valid_q0, adc_valid_i0}),
    .in_i0         (adc_data_i0),
    .in_q0         (adc_data_q0),
    .in_i1         (adc_data_i1),
    .in_q1         (adc_data_q1),
    .out_valid     ({out_valid_q1, out_valid_i1, out_valid_q0, out_valid_i0}),
    .out_i0        (out_data_i0),
    .out_q0        (out_data_q0),
    .out_i1        (out_data_i1),
    .out_q1        (out_data_q1)
  );

endmodule
