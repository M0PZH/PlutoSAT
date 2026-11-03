// Sequential vectoring CORDIC (model: rxfixed.cordic): angle of (x, y) in
// turns * 2^16 and magnitude * K. Inputs are first shifted right until
// max(|x|, |y|) < 2^22; the magnitude is shifted back. 21 clocks.
module dvbs2_rx_cordic
  import dvbs2_rx_pkg::*;
(
  input  logic               clk,
  input  logic               rst,
  input  logic               start,
  input  logic signed [49:0] x,
  input  logic signed [49:0] y,
  output logic               done,
  output logic signed [15:0] ang,
  output logic [31:0]        mag
);
  logic               busy;
  logic [1:0]         ph;
  logic [4:0]         it;
  logic [5:0]         sh;
  logic signed [49:0] xin, yin;
  logic signed [26:0] cx, cy;
  logic signed [20:0] acc;

  function automatic logic [5:0] nshift(input logic signed [49:0] a, input logic signed [49:0] b);
    logic [49:0] ma, mb, mx;
    ma = a[49] ? 50'(-a) : 50'(a);
    mb = b[49] ? 50'(-b) : 50'(b);
    mx = ma > mb ? ma : mb;
    nshift = 0;
    for (int i = 0; i < 50; i++)
      if (mx[i]) nshift = (i + 1 > 22) ? 6'(i + 1 - 22) : 6'd0;
  endfunction

  always_ff @(posedge clk) begin
    done <= 1'b0;
    if (rst) begin
      busy <= 1'b0;
    end else if (start && !busy) begin
      busy <= 1'b1;
      ph   <= 2'd0;
      xin  <= x;
      yin  <= y;
      sh   <= nshift(x, y);
    end else if (busy) begin
      case (ph)
        2'd0: begin : pre
          logic signed [49:0] xs, ys;
          xs = xin >>> sh;
          ys = yin >>> sh;
          if (xs < 0) begin
            cx  <= 27'(-xs) <<< 2;
            cy  <= 27'(-ys) <<< 2;
            acc <= 21'sd1 <<< 19;
          end else begin
            cx  <= 27'(xs) <<< 2;
            cy  <= 27'(ys) <<< 2;
            acc <= '0;
          end
          it <= '0;
          ph <= 2'd1;
        end
        2'd1: begin
          if (cy >= 0) begin
            cx  <= cx + (cy >>> it);
            cy  <= cy - (cx >>> it);
            acc <= acc + 21'(CORDIC_ATAN[it]);
          end else begin
            cx  <= cx - (cy >>> it);
            cy  <= cy + (cx >>> it);
            acc <= acc - 21'(CORDIC_ATAN[it]);
          end
          if (it == 5'(CORDIC_IT - 1)) ph <= 2'd2;
          else                         it <= it + 5'd1;
        end
        default: begin
          ang  <= 16'((acc + 21'sd8) >>> 4);
          mag  <= 32'(cx >>> 2) << sh;
          done <= 1'b1;
          busy <= 1'b0;
        end
      endcase
    end
  end
endmodule
