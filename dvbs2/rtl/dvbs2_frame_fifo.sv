// FECFRAME buffer between the FEC encoders and the PL framer.
//
// The framer only starts a data PLFRAME when a complete FECFRAME is stored,
// so a slow or bursty producer can never underrun a frame half way; when no
// frame is ready the framer sends a dummy PLFRAME instead. Bits are stored
// as QPSK symbol pairs {b0, b1}. A small side FIFO holds the configuration
// of every complete frame.
//
// DEPTH is in symbols; the default (1.5 normal frames) lets the encoders
// fill the next frame while the current one is being transmitted.

module dvbs2_frame_fifo #(
  parameter int DEPTH = 49152,
  parameter int AW = $clog2(DEPTH)
) (
  input  logic       clk,
  input  logic       rst,

  // FEC side (bits)
  input  logic       s_valid,
  output logic       s_ready,
  input  logic       s_data,
  input  logic [4:0] s_user,
  input  logic       s_last,

  // framer side
  output logic       frame_avail,   // a complete frame is stored
  output logic [4:0] frame_cfg,     // its configuration
  input  logic       frame_pop,     // framer takes that frame (once per frame)
  output logic       sym_valid,     // FWFT symbol pair
  output logic [1:0] sym_data,      // {b0 (I sign), b1 (Q sign)}
  input  logic       sym_rd,

  output logic [AW:0] level         // symbols stored
);

  // ---------------------------------------------------------------- symbol RAM
  logic [1:0] ram [DEPTH];
  logic [AW-1:0] wptr, rptr;
  logic [AW:0]   count;             // symbols in RAM (not yet read out)
  logic          half;              // first bit of a pair received
  logic          b0;
  logic          ram_we;
  logic [1:0]    ram_dout;
  logic          rd_issue, inflight;

  // ---------------------------------------------------------------- frame cfg FIFO
  logic [4:0] cfg_q [4];
  logic [1:0] cfg_w, cfg_r;
  logic [2:0] cfg_n;

  wire cfg_push = s_valid && s_ready && s_last;
  assign frame_avail = (cfg_n != 0);
  assign frame_cfg   = cfg_q[cfg_r];

  // accept a bit when there is room for its pair and for the frame's cfg
  assign s_ready = (count < (AW+1)'(DEPTH) || half) && (cfg_n != 3'd4);
  assign ram_we  = s_valid && s_ready && half;

  always_ff @(posedge clk) begin
    if (ram_we) ram[wptr] <= {b0, s_data};
    if (rd_issue) ram_dout <= ram[rptr];
  end

  assign level = count;

  // FWFT read side
  assign rd_issue = !inflight && (!sym_valid || sym_rd) && (count != 0);

  always_ff @(posedge clk) begin
    if (rst) begin
      wptr <= '0; rptr <= '0; count <= '0; half <= 1'b0;
      inflight <= 1'b0; sym_valid <= 1'b0;
      cfg_w <= '0; cfg_r <= '0; cfg_n <= '0;
    end else begin
      if (s_valid && s_ready) begin
        half <= !half;
        if (!half) b0 <= s_data;
      end
      if (ram_we) wptr <= (wptr == AW'(DEPTH - 1)) ? '0 : wptr + 1'b1;
      if (rd_issue) rptr <= (rptr == AW'(DEPTH - 1)) ? '0 : rptr + 1'b1;
      count <= count + (ram_we ? 1'b1 : 1'b0) - (rd_issue ? 1'b1 : 1'b0);

      if (sym_valid && sym_rd) sym_valid <= 1'b0;
      inflight <= rd_issue;
      if (inflight) begin
        sym_data  <= ram_dout;
        sym_valid <= 1'b1;
      end

      if (cfg_push) begin
        cfg_q[cfg_w] <= s_user;
        cfg_w <= cfg_w + 1'b1;
      end
      if (frame_pop) cfg_r <= cfg_r + 1'b1;
      cfg_n <= cfg_n + (cfg_push ? 3'd1 : 3'd0) - (frame_pop ? 3'd1 : 3'd0);
    end
  end

endmodule
