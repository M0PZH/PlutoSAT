// DVB-S2 LDPC inner encoder, normal FECFRAME, bit serial (EN 302 307-1 5.3.2).
//
// Info bits pass straight through. For every info bit i_m equal to one, each
// address-table entry x of its 360-bit group m/360 toggles parity bit
//   p[(x + (m mod 360) * q) mod Np]
// in a 1-bit parity RAM. The ROM stores entries pre-split as x = r + c0*q,
// so the address is r + ((c0 + s) mod 360) * q: an add, a compare and a
// small multiply. Updates are pipelined at one per cycle with a forwarding
// path for back-to-back updates of the same address. Afterwards the parity
// bits are read out through the final accumulator p_i ^= p_(i-1) and the
// RAM is cleared for the next frame.
//
// Throughput: about (Kldpc + ones * column_weight) cycles plus 2*Np cycles
// per frame, i.e. ~1.5 ms per frame at 100 MHz for rate 3/5.

module dvbs2_ldpc_enc
  import dvbs2_tables_pkg::*;
#(
  parameter string ROM_FILE = "ldpc_rom.mem"
) (
  input  logic       clk,
  input  logic       rst,

  input  logic       s_valid,
  output logic       s_ready,
  input  logic       s_data,
  input  logic [4:0] s_user,
  input  logic       s_last,   // last BCH codeword bit = last LDPC info bit

  output logic       m_valid,
  input  logic       m_ready,
  output logic       m_data,
  output logic [4:0] m_user,
  output logic       m_last    // bit 64799
);

  localparam int NLDPC = 64800;
  localparam int PAR_DEPTH = 49152;   // >= max Np = 48600 (rate 1/4)

  typedef enum logic [2:0] {ST_START, ST_SCAN, ST_INFO, ST_ISSUE, ST_DRAIN, ST_PAR} state_t;
  state_t state, after_issue;

  logic [4:0]  cfg;
  logic [7:0]  q;
  logic [15:0] np_m1;

  // ---------------------------------------------------------------- tables
  (* rom_style = "block" *) logic [17:0] rom [LDPC_ROM_DEPTH];
  initial $readmemh(ROM_FILE, rom);
  logic [LDPC_ROM_AW-1:0] rom_addr;
  logic [17:0]            rom_dout;
  always_ff @(posedge clk) rom_dout <= rom[rom_addr];

  wire        e_last = rom_dout[17];
  wire [7:0]  e_r    = rom_dout[16:9];
  wire [8:0]  e_c0   = rom_dout[8:0];

  // ---------------------------------------------------------------- parity RAM
  logic par_ram [PAR_DEPTH];
  initial for (int i = 0; i < PAR_DEPTH; i++) par_ram[i] = 1'b0;
  logic [15:0] ram_raddr, ram_waddr;
  logic        ram_we, ram_wdata, ram_dout;
  always_ff @(posedge clk) begin
    if (ram_we) par_ram[ram_waddr] <= ram_wdata;
    ram_dout <= par_ram[ram_raddr];
  end

  // ---------------------------------------------------------------- group bookkeeping
  logic [LDPC_ROM_AW-1:0] grp_base, scan_ptr;
  logic [3:0]             grp_len, scan_cnt;
  logic                   scan_wait;
  logic [8:0]             s_cnt;       // m mod 360 of the next info bit
  logic                   grp_end_pending, frame_end_pending;

  // issue state for one '1' info bit
  logic [LDPC_ROM_AW-1:0] iss_ptr;
  logic [3:0]             iss_left;
  logic [8:0]             iss_s;

  // ---------------------------------------------------------------- update pipeline
  logic        pa_v, pb_v, pc_v, pd_v;
  logic [8:0]  pa_s;
  logic [8:0]  pb_c;
  logic [7:0]  pb_r;
  logic [15:0] pc_addr, pd_addr;
  logic        pd_fwd, pd_fwd_data;
  wire         pipe_busy = pa_v || pb_v || pc_v || pd_v;

  // ---------------------------------------------------------------- parity readout
  logic [15:0] rd_i;          // next parity index to read
  logic        rd_inflight;
  logic        pout_valid, pout_bit, pout_last, acc;
  logic [15:0] pout_idx;

  wire info_accept = (state == ST_INFO) && s_valid && m_ready;

  assign s_ready = (state == ST_INFO) && m_ready;
  assign m_user  = cfg;
  always_comb begin
    if (state == ST_PAR) begin
      m_valid = pout_valid;
      m_data  = pout_bit;
      m_last  = pout_valid && pout_last;
    end else begin
      m_valid = (state == ST_INFO) && s_valid;
      m_data  = s_data;
      m_last  = 1'b0;
    end
  end

  // ROM address and RAM read address muxes
  always_comb begin
    rom_addr = scan_ptr;
    if (state == ST_ISSUE) rom_addr = iss_ptr;
    ram_raddr = (state == ST_PAR) ? rd_i : pc_addr;
  end

  // stage D/E: read-modify-write (toggle) with same-cycle forwarding
  wire cur_bit = pd_fwd ? pd_fwd_data : ram_dout;
  always_comb begin
    ram_we    = 1'b0;
    ram_waddr = pd_addr;
    ram_wdata = !cur_bit;
    if (pd_v) begin
      ram_we = 1'b1;
    end else if (state == ST_PAR && rd_inflight) begin
      ram_we    = 1'b1;          // clear as we read out
      ram_waddr = rd_i - 16'd1;
      ram_wdata = 1'b0;
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      state <= ST_START;
      pa_v <= 1'b0; pb_v <= 1'b0; pc_v <= 1'b0; pd_v <= 1'b0;
      pout_valid <= 1'b0;
      rd_inflight <= 1'b0;
      scan_wait <= 1'b0;
    end else begin
      // -------- pipeline (always advances)
      // A -> B: ROM data valid one cycle after the address
      pb_v <= pa_v;
      if (pa_v) begin
        pb_c <= (10'(e_c0) + 10'(pa_s) >= 10'd360) ? 9'(e_c0 + pa_s - 9'd360) : 9'(e_c0 + pa_s);
        pb_r <= e_r;
      end
      // B -> C: address = r + c*q
      pc_v <= pb_v;
      if (pb_v) pc_addr <= 16'(pb_c) * 16'(q) + 16'(pb_r);
      // C -> D: RAM read issued with pc_addr; note forwarding from the write
      // happening in this same cycle (stage D of the previous update)
      pd_v <= pc_v;
      if (pc_v) begin
        pd_addr     <= pc_addr;
        pd_fwd      <= pd_v && (pd_addr == pc_addr);
        pd_fwd_data <= !cur_bit;
      end

      pa_v <= 1'b0;

      case (state)
        ST_START: begin
          if (s_valid) begin
            cfg      <= s_user;
            q        <= 8'(Q[s_user[3:0]]);
            np_m1    <= 16'(NLDPC - KLDPC[s_user[3:0]] - 1);
            grp_base <= LDPC_ROM_AW'(LDPC_ROM_BASE[s_user[3:0]]);
            scan_ptr <= LDPC_ROM_AW'(LDPC_ROM_BASE[s_user[3:0]]);
            scan_cnt <= '0;
            scan_wait <= 1'b0;
            s_cnt    <= '0;
            state    <= ST_SCAN;
          end
        end

        // find the number of entries in the group starting at grp_base
        ST_SCAN: begin
          scan_ptr  <= scan_ptr + 1'b1;
          scan_wait <= 1'b1;
          if (scan_wait) begin
            scan_cnt <= scan_cnt + 4'd1;
            if (e_last) begin
              grp_len   <= scan_cnt + 4'd1;
              scan_wait <= 1'b0;
              state     <= ST_INFO;
            end
          end
        end

        ST_INFO: begin
          if (info_accept) begin
            grp_end_pending   <= (s_cnt == 9'd359);
            frame_end_pending <= s_last;
            iss_ptr  <= grp_base;
            iss_left <= grp_len;
            iss_s    <= s_cnt;
            s_cnt    <= (s_cnt == 9'd359) ? 9'd0 : s_cnt + 9'd1;
            if (s_cnt == 9'd359) begin
              grp_base <= grp_base + LDPC_ROM_AW'(grp_len);
              scan_ptr <= grp_base + LDPC_ROM_AW'(grp_len);
              scan_cnt <= '0;
            end
            if (s_data)
              state <= ST_ISSUE;
            else if (s_last)
              state <= ST_DRAIN;
            else if (s_cnt == 9'd359)
              state <= ST_SCAN;
          end
        end

        ST_ISSUE: begin
          pa_v     <= 1'b1;
          pa_s     <= iss_s;
          iss_ptr  <= iss_ptr + 1'b1;
          iss_left <= iss_left - 4'd1;
          if (iss_left == 4'd1)
            state <= frame_end_pending ? ST_DRAIN : (grp_end_pending ? ST_SCAN : ST_INFO);
        end

        ST_DRAIN: begin
          if (!pipe_busy && !pa_v) begin
            rd_i        <= '0;
            pout_idx    <= '0;
            acc         <= 1'b0;
            rd_inflight <= 1'b0;
            state       <= ST_PAR;
          end
        end

        ST_PAR: begin
          if (m_ready && pout_valid) begin
            pout_valid <= 1'b0;
            if (pout_last) state <= ST_START;
          end
          // read data -> output register
          if (rd_inflight) begin
            rd_inflight <= 1'b0;
            pout_bit    <= acc ^ ram_dout;
            acc         <= acc ^ ram_dout;
            pout_last   <= (pout_idx == np_m1);
            pout_idx    <= pout_idx + 16'd1;
            pout_valid  <= 1'b1;
          end else if ((!pout_valid || m_ready) && pout_idx <= np_m1 && !(pout_valid && pout_last)) begin
            rd_inflight <= 1'b1;     // issue read of rd_i (ram_raddr = rd_i)
            rd_i        <= rd_i + 16'd1;
          end
        end

        default: state <= ST_START;
      endcase
    end
  end

endmodule
