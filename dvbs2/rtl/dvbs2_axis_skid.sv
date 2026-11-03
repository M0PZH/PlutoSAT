// AXI-Stream register slice (full-throughput skid buffer).
// Breaks the combinational valid/ready paths between pipeline blocks.

module dvbs2_axis_skid #(
  parameter int W = 8
) (
  input  logic         clk,
  input  logic         rst,

  input  logic         s_valid,
  output logic         s_ready,
  input  logic [W-1:0] s_data,

  output logic         m_valid,
  input  logic         m_ready,
  output logic [W-1:0] m_data
);

  logic [W-1:0] skid_data;
  logic         skid_valid;

  assign s_ready = !skid_valid;

  always_ff @(posedge clk) begin
    if (rst) begin
      m_valid    <= 1'b0;
      skid_valid <= 1'b0;
    end else begin
      if (m_ready || !m_valid) begin
        if (skid_valid) begin
          m_data     <= skid_data;
          m_valid    <= 1'b1;
          skid_valid <= 1'b0;
        end else begin
          m_data  <= s_data;
          m_valid <= s_valid;
        end
      end else if (s_valid && s_ready) begin
        skid_data  <= s_data;
        skid_valid <= 1'b1;
      end
    end
  end

endmodule
