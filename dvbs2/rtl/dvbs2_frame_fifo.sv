// FECFRAME buffer between the FEC encoders and the PL framer, with the 8PSK
// bit interleaver and the symbol mapping.
//
// The framer only starts a data PLFRAME when a complete FECFRAME is stored,
// so a slow or bursty producer can never underrun a frame half way; when no
// frame is ready the framer sends a dummy PLFRAME instead. A small side FIFO
// holds the configuration of every complete frame.
//
// Storage: a ring of 98304 bits (1.5 normal FECFRAMEs) in three 32K x 1
// banks; ring bit a is bank a mod 3, row a / 3. Every frame starts at a
// multiple of 3 (64800 and 16200 are), i.e. at bank 0 of a row.
//   QPSK: bit i of the frame at ring bit base + i; symbol k = bits 2k, 2k+1.
//   8PSK: the block interleaver of EN 302 307-1 5.3.3 (3 columns of N/3 bits,
//         written column-wise, read row-wise): bit i = c N/3 + k at ring bit
//         base + 3k + c, so symbol k is one row of the three banks.
// A frame reserves its N bits when its first bit is accepted (8PSK writes
// the whole region from the start), so with 1.5 frames of storage the
// encoders start a normal frame once half of the previous one is sent.
//
// Output: the symbol's phase index p (symbol exp(j pi p / 4)) before PL
// scrambling: QPSK (b0, b1) -> 1, 7, 3, 5; 8PSK v = (b0 b1 b2) -> 1, 0, 4, 5,
// 2, 7, 3, 6, the first column being the LSB of v for rate 3/5.
//
// s_user / frame_cfg = {psk8, short, pilots, rate_idx[3:0]}.

module dvbs2_frame_fifo #(
  parameter int ROWS = 32768
) (
  input  logic       clk,
  input  logic       rst,

  // FEC side (bits)
  input  logic       s_valid,
  output logic       s_ready,
  input  logic       s_data,
  input  logic [6:0] s_user,
  input  logic       s_last,

  // framer side
  output logic       frame_avail,   // a complete frame is stored
  output logic [6:0] frame_cfg,     // its configuration
  input  logic       frame_pop,     // framer takes that frame (once per frame)
  output logic       sym_valid,     // FWFT symbol
  output logic [2:0] sym_data,      // phase index p
  input  logic       sym_rd,

  output logic [16:0] level,        // bits stored or reserved / 2 (QPSK symbols)
  output logic [2:0]  frames        // complete frames not yet popped (0..4)
);

  localparam int RAW = $clog2(ROWS);
  localparam int RING = 3 * ROWS;

  function automatic logic [16:0] flen(input logic [6:0] c);   // FECFRAME bits
    flen = c[5] ? 17'd16200 : 17'd64800;
  endfunction
  function automatic logic [14:0] ncol(input logic [6:0] c);   // 8PSK column length
    ncol = c[5] ? 15'd5400 : 15'd21600;
  endfunction

  // ---------------------------------------------------------------- banks
  logic           b_we [3];
  logic [RAW-1:0] b_wa;
  logic [RAW-1:0] b_ra [3];
  logic           b_q [3];
  for (genvar j = 0; j < 3; j++) begin : g_bank
    (* ram_style = "block" *) logic mem [ROWS];
    always_ff @(posedge clk) begin
      if (b_we[j]) mem[b_wa] <= s_data;
      b_q[j] <= mem[b_ra[j]];
    end
  end

  // ---------------------------------------------------------------- frame cfg FIFO
  // 8 entries: up to 4 complete frames not popped + the one being read
  logic [6:0] cfg_q [8];
  logic [2:0] cfg_w, cfg_r, cfg_rd;   // write, framer pop, symbol reader
  logic [2:0] cfg_n;

  // ---------------------------------------------------------------- write side
  logic           w_in;               // a frame is being written
  logic [6:0]     w_cfg;
  logic [RAW-1:0] w_base;             // row of the frame's first bit
  logic [RAW-1:0] w_row;              // QPSK: row of the next bit; 8PSK: base + k
  logic [1:0]     w_bank;             // QPSK: bank of the next bit; 8PSK: column c
  logic [14:0]    w_k;                // 8PSK: k of the next bit
  logic [17:0]    used;               // bits reserved (frames being written or not read)
  logic [17:0]    avail;              // bits of complete frames not read yet

  wire [6:0]  cur_cfg = w_in ? w_cfg : s_user;
  wire        room = w_in || (used + 18'(flen(s_user)) <= 18'(RING));
  assign s_ready = room && (cfg_n != 3'd4);
  wire        take = s_valid && s_ready;
  wire        cfg_push = take && s_last;
  wire [RAW-1:0] row_now  = w_in ? w_row : w_base;
  wire [1:0]     bank_now = w_in ? w_bank : 2'd0;
  wire [14:0]    k_now    = w_in ? w_k : 15'd0;

  assign b_wa = row_now;
  always_comb for (int j = 0; j < 3; j++) b_we[j] = take && bank_now == 2'(j);

  assign frame_avail = (cfg_n != 0);
  assign frame_cfg   = cfg_q[cfg_r];
  assign frames      = cfg_n;
  assign level       = 17'(used >> 1);

  // ---------------------------------------------------------------- read side
  logic [RAW-1:0] r_row;
  logic [1:0]     r_bank;
  logic [16:0]    r_left;             // bits of the reader's frame not read yet
  logic           r_new;              // the reader's next symbol starts a frame
  logic           inflight;
  logic [6:0]     rq_cfg;             // cfg of the symbol in flight
  logic [1:0]     rq_bank;
  wire  [6:0]     r_cfg = cfg_q[cfg_rd];
  wire  [1:0]     r_step = r_cfg[6] ? 2'd3 : 2'd2;
  wire            rd_issue = !inflight && (!sym_valid || sym_rd) && (avail != 0);

  always_comb
    for (int j = 0; j < 3; j++)
      b_ra[j] = (2'(j) >= r_bank) ? r_row : r_row + 1'b1;

  // the symbol's bits in order: bank r_bank, r_bank + 1, r_bank + 2 (mod 3)
  logic [2:0] rbits;
  always_comb begin
    case (rq_bank)
      2'd0:    rbits = {b_q[0], b_q[1], b_q[2]};
      2'd1:    rbits = {b_q[1], b_q[2], b_q[0]};
      default: rbits = {b_q[2], b_q[0], b_q[1]};
    endcase
  end

  function automatic logic [2:0] phase(input logic [6:0] c, input logic [2:0] b);
    logic [2:0] v;
    phase = 3'd1;
    if (!c[6]) begin
      case (b[2:1])
        2'd0:    phase = 3'd1;
        2'd1:    phase = 3'd7;
        2'd2:    phase = 3'd3;
        default: phase = 3'd5;
      endcase
    end else begin
      v = (c[3:0] == 4'd4) ? {b[0], b[1], b[2]} : b;
      case (v)
        3'd0:    phase = 3'd1;
        3'd1:    phase = 3'd0;
        3'd2:    phase = 3'd4;
        3'd3:    phase = 3'd5;
        3'd4:    phase = 3'd2;
        3'd5:    phase = 3'd7;
        3'd6:    phase = 3'd3;
        default: phase = 3'd6;
      endcase
    end
  endfunction

  always_ff @(posedge clk) begin
    if (rst) begin
      w_in <= 1'b0; w_base <= '0; used <= '0; avail <= '0;
      r_row <= '0; r_bank <= '0; r_new <= 1'b1;
      inflight <= 1'b0; sym_valid <= 1'b0;
      cfg_w <= '0; cfg_r <= '0; cfg_rd <= '0; cfg_n <= '0;
    end else begin : seq
      logic [17:0] u, a;
      u = used;
      a = avail;

      // -------- write
      if (take) begin
        if (!w_in) begin
          w_in  <= 1'b1;
          w_cfg <= s_user;
          u = u + 18'(flen(s_user));
        end
        if (cur_cfg[6]) begin
          // next k of the column; at its end the next column from k = 0
          if (k_now == ncol(cur_cfg) - 15'd1) begin
            w_k    <= '0;
            w_row  <= row_now - RAW'(ncol(cur_cfg) - 15'd1);
            w_bank <= bank_now + 2'd1;
          end else begin
            w_k    <= k_now + 15'd1;
            w_row  <= row_now + 1'b1;
            w_bank <= bank_now;
          end
        end else begin
          w_bank <= (bank_now == 2'd2) ? 2'd0 : bank_now + 2'd1;
          w_row  <= (bank_now == 2'd2) ? row_now + 1'b1 : row_now;
        end
        if (s_last) begin
          w_in   <= 1'b0;
          w_base <= w_base + RAW'(flen(cur_cfg) / 3);
          cfg_q[cfg_w] <= cur_cfg;
          cfg_w <= cfg_w + 1'b1;
          a = a + 18'(flen(cur_cfg));
        end
      end

      // -------- read (FWFT)
      if (sym_valid && sym_rd) sym_valid <= 1'b0;
      inflight <= rd_issue;
      if (rd_issue) begin
        rq_cfg  <= r_cfg;
        rq_bank <= r_bank;
        u = u - 18'(r_step);
        a = a - 18'(r_step);
        if ((r_new ? flen(r_cfg) : r_left) == 17'(r_step)) begin
          r_new  <= 1'b1;
          cfg_rd <= cfg_rd + 1'b1;
        end else begin
          r_new  <= 1'b0;
          r_left <= (r_new ? flen(r_cfg) : r_left) - 17'(r_step);
        end
        // QPSK: bank 0 -> 2, 1 -> 0 of the next row, 2 -> 1 of the next row
        if (r_cfg[6]) begin
          r_row  <= r_row + 1'b1;
          r_bank <= 2'd0;
        end else begin
          r_bank <= (r_bank == 2'd0) ? 2'd2 : (r_bank == 2'd1) ? 2'd0 : 2'd1;
          if (r_bank != 2'd0) r_row <= r_row + 1'b1;
        end
      end
      if (inflight) begin
        sym_data  <= phase(rq_cfg, rbits);
        sym_valid <= 1'b1;
      end

      used  <= u;
      avail <= a;
      if (frame_pop) cfg_r <= cfg_r + 1'b1;
      cfg_n <= cfg_n + (cfg_push ? 3'd1 : 3'd0) - (frame_pop ? 3'd1 : 3'd0);
    end
  end

endmodule
