// Dual-clock FIFO with Gray-coded pointers (first-word fall-through).
// Both resets must be asserted together for at least a few cycles of the
// slower clock.

module dvbs2_async_fifo #(
  parameter int W  = 32,
  parameter int AW = 5          // depth = 2^AW
) (
  input  logic         wclk,
  input  logic         wrst,
  input  logic         w_valid,
  output logic         w_ready,
  input  logic [W-1:0] w_data,

  input  logic         rclk,
  input  logic         rrst,
  output logic         r_valid,
  input  logic         r_ready,
  output logic [W-1:0] r_data
);

  logic [W-1:0] mem [1 << AW];

  function automatic logic [AW:0] bin2gray(input logic [AW:0] b);
    return b ^ (b >> 1);
  endfunction

  // write side
  logic [AW:0] wbin, wgray;
  (* ASYNC_REG = "TRUE" *) logic [AW:0] rgray_w1, rgray_w2;
  wire  [AW:0] wbin_next = wbin + 1'b1;
  assign w_ready = (wgray != {~rgray_w2[AW:AW-1], rgray_w2[AW-2:0]});

  always_ff @(posedge wclk) begin
    if (wrst) begin
      wbin <= '0; wgray <= '0; rgray_w1 <= '0; rgray_w2 <= '0;
    end else begin
      {rgray_w2, rgray_w1} <= {rgray_w1, rgray_r};
      if (w_valid && w_ready) begin
        mem[wbin[AW-1:0]] <= w_data;
        wbin  <= wbin_next;
        wgray <= bin2gray(wbin_next);
      end
    end
  end

  // read side
  logic [AW:0] rbin, rgray_r;
  (* ASYNC_REG = "TRUE" *) logic [AW:0] wgray_r1, wgray_r2;
  wire  [AW:0] rbin_next = rbin + 1'b1;
  assign r_valid = (rgray_r != wgray_r2);
  assign r_data  = mem[rbin[AW-1:0]];

  always_ff @(posedge rclk) begin
    if (rrst) begin
      rbin <= '0; rgray_r <= '0; wgray_r1 <= '0; wgray_r2 <= '0;
    end else begin
      {wgray_r2, wgray_r1} <= {wgray_r1, wgray};
      if (r_valid && r_ready) begin
        rbin    <= rbin_next;
        rgray_r <= bin2gray(rbin_next);
      end
    end
  end

endmodule
