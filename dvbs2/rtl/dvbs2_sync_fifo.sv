// Synchronous first-word-fall-through FIFO: 2^AW words in a RAM (block RAM
// or LUTRAM, RAM_STYLE) plus the output register, so 2^AW + 1 words in all.
// The RAM is read into the output register (a block RAM's own output latch)
// whenever that is empty or being emptied; a word written in one clock is
// at the output (r_valid) in the next.

module dvbs2_sync_fifo #(
  parameter int    W         = 32,
  parameter int    AW        = 9,
  parameter string RAM_STYLE = "block"       // "block" or "distributed"
) (
  input  logic         clk,
  input  logic         rst,

  input  logic         w_valid,
  output logic         w_ready,
  input  logic [W-1:0] w_data,

  output logic         r_valid,
  input  logic         r_ready,
  output logic [W-1:0] r_data
);

  logic [AW:0] wp, rp;
  wire         rd = (wp != rp) && (!r_valid || r_ready);

  assign w_ready = (wp - rp) != (AW+1)'(1 << AW);

  if (RAM_STYLE == "block") begin : g_bram
    (* ram_style = "block" *) logic [W-1:0] mem [1 << AW];
    always_ff @(posedge clk) begin
      if (w_valid && w_ready) mem[wp[AW-1:0]] <= w_data;
      if (rd) r_data <= mem[rp[AW-1:0]];
    end
  end else begin : g_lutram
    (* ram_style = "distributed" *) logic [W-1:0] mem [1 << AW];
    always_ff @(posedge clk) begin
      if (w_valid && w_ready) mem[wp[AW-1:0]] <= w_data;
      if (rd) r_data <= mem[rp[AW-1:0]];
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      wp <= '0; rp <= '0; r_valid <= 1'b0;
    end else begin
      if (w_valid && w_ready) wp <= wp + 1'b1;
      if (rd) begin
        rp <= rp + 1'b1;
        r_valid <= 1'b1;
      end else if (r_ready) r_valid <= 1'b0;
    end
  end

endmodule
