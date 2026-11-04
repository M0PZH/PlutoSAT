// Verilog-2001 top of the spectrum analyser for Vivado block-design module
// reference. See rtl/fft_axi.sv for the register map and README.md for the
// integration. Add rtl/fft_win.sv, rtl/fft_acc.sv, rtl/fft_axi.sv,
// rtl/gen/fft_win.mem, dvbs2/rtl/dvbs2_async_fifo.sv and this file, and
// create the xfft instance first (ip/fft_xfft.tcl: fft_create_xfft).
//
// s_axi: AXI-Lite slave, 64 KB. adc_*: AD9363 receive samples (l_clk domain).
// tap_*: nbiq channel outputs, s_axi_aclk domain. irq: level, "average done".

module fft_bd #(
  parameter WIN_FILE   = "fft_win.mem",
  parameter DOUBLE_BUF = 0
) (
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn" *)
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

  (* X_INTERFACE_INFO = "xilinx.com:signal:interrupt:1.0 irq INTERRUPT" *)
  (* X_INTERFACE_PARAMETER = "SENSITIVITY LEVEL_HIGH" *)
  output wire        irq,

  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 adc_clk CLK" *)
  input  wire        adc_clk,
  input  wire        adc_valid,
  input  wire [15:0] adc_data_i0,
  input  wire [15:0] adc_data_q0,
  input  wire [15:0] adc_data_i1,
  input  wire [15:0] adc_data_q1,

  input  wire        tap_valid,
  input  wire [2:0]  tap_ch,
  input  wire [15:0] tap_i,
  input  wire [15:0] tap_q
);

  fft_axi #(.WIN_FILE(WIN_FILE), .DOUBLE_BUF(DOUBLE_BUF)) i_fft_axi (
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
    .irq           (irq),
    .adc_clk       (adc_clk),
    .adc_valid     (adc_valid),
    .adc_data_i0   (adc_data_i0),
    .adc_data_q0   (adc_data_q0),
    .adc_data_i1   (adc_data_i1),
    .adc_data_q1   (adc_data_q1),
    .tap_valid     (tap_valid),
    .tap_ch        (tap_ch),
    .tap_i         (tap_i),
    .tap_q         (tap_q)
  );

endmodule
