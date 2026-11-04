// Simple dual-port RAM (one write port, one registered read port).
module nbiq_sdpram #(
  parameter int W = 48,
  parameter int AW = 9
) (
  input  logic          clk,
  input  logic          we,
  input  logic [AW-1:0] wa,
  input  logic [W-1:0]  wd,
  input  logic [AW-1:0] ra,
  output logic [W-1:0]  rq
);
  (* ram_style = "block" *) logic [W-1:0] mem [1 << AW];
  always_ff @(posedge clk) if (we) mem[wa] <= wd;
  always_ff @(posedge clk) rq <= mem[ra];
endmodule
