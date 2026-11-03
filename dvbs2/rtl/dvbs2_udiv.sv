// Sequential unsigned divider: q = n / d (restoring, NW clocks). d = 0 gives
// all ones.
module dvbs2_udiv #(
  parameter int NW = 44,
  parameter int DW = 36
) (
  input  logic          clk,
  input  logic          rst,
  input  logic          start,
  input  logic [NW-1:0] n,
  input  logic [DW-1:0] d,
  output logic          done,
  output logic [NW-1:0] q
);
  logic          busy;
  logic [NW-1:0] nr;
  logic [DW:0]   r;
  logic [DW-1:0] dr;
  logic [$clog2(NW+1)-1:0] cnt;

  always_ff @(posedge clk) begin
    done <= 1'b0;
    if (rst) busy <= 1'b0;
    else if (start && !busy) begin
      busy <= 1'b1;
      nr   <= n;
      dr   <= d;
      r    <= '0;
      cnt  <= '0;
    end else if (busy) begin : step
      logic [DW:0] t;
      t = {r[DW-1:0], nr[NW-1]};
      if (t >= {1'b0, dr}) begin
        r <= t - {1'b0, dr};
        q <= {q[NW-2:0], 1'b1};
      end else begin
        r <= t;
        q <= {q[NW-2:0], 1'b0};
      end
      nr <= {nr[NW-2:0], 1'b0};
      cnt <= cnt + 1'b1;
      if (cnt == NW - 1) begin
        busy <= 1'b0;
        done <= 1'b1;
      end
    end
  end
endmodule
