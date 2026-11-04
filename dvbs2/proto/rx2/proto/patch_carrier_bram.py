"""PROTOTYPE: move the carrier's seg and zbuf buffers to block RAM (same behaviour)."""
import sys
src, dst = sys.argv[1], sys.argv[2]
t = open(src).read()

def rep(old, new, count=1):
    global t
    n = t.count(old)
    assert n == count, (old, n)
    t = t.replace(old, new)

# seg: write port in its own process (one clock later), read port outside the FSM
rep("""  (* ram_style = "block" *) logic [31:0] seg [2048];   // pending data symbols
  logic [31:0] seg_rd;""",
"""  (* ram_style = "block" *) logic [31:0] seg [2048];   // pending data symbols
  logic [31:0] seg_rd;
  logic        sw_v;
  logic [10:0] sw_a;
  logic [31:0] sw_d;
  always_ff @(posedge clk) if (sw_v) seg[sw_a] <= sw_d;""")
rep("""            seg[nd] <= {s1q, s1i};""", """            sw_v <= 1'b1; sw_a <= nd; sw_d <= {s1q, s1i};""")
rep("""            seg_rd  <= seg[sk];\n""", "")
rep("""    zw_v      <= 1'b0;
""", """    zw_v      <= 1'b0;
    sw_v      <= 1'b0;
    zr_v      <= 1'b0;
""")
# zbuf: one read port (BRAM), M_HZ and M_FLL_ROT
rep("""  logic [31:0] hz_w;""", """  logic [31:0] hz_w;
  logic [31:0] zb_q;
  logic        zr_v;
  logic [31:0] zr_ph;
  logic [6:0]  zr_tag;
  wire  [9:0]  zb_ra = (st == M_HZ) ? 10'({wb, fk}) : 10'({fh, fk});
  always_ff @(posedge clk) zb_q <= zbuf[zb_ra];
  wire seg_issue = (st == M_SEG_EMIT) && (sk != nd) && (6'(of_cnt) + inflight < 6'd22);
  always_ff @(posedge clk) if (seg_issue) seg_rd <= seg[sk];""")
rep("""          hz_w <= zbuf[10'({wb, fk})];\n""", "")
for a in ("hz_w[15:0]", "hz_w[31:16]"):
    t = t.replace(a, a.replace("hz_w", "zb_q"))
rep("""            rot_v   <= 1'b1;
            rot_i   <= $signed(zbuf[10'({fh, fk})][15:0]);
            rot_q   <= $signed(zbuf[10'({fh, fk})][31:16]);
            rot_ph  <= fph;
            rot_tag <= {1'b0, fk};""",
"""            zr_v    <= 1'b1;
            zr_ph   <= fph;
            zr_tag  <= fk;""")
# the rotator input one clock later, from the BRAM output
lines = t.split("\n")
# find the end of the main FSM always_ff: the 'end' closing 'end else begin' + case
idx = t.rfind("      endcase\n    end\n  end\n")
assert idx > 0
t = t[:idx] + """      endcase
    end
    if (zr_v) begin
      rot_v   <= 1'b1;
      rot_i   <= $signed(zb_q[15:0]);
      rot_q   <= $signed(zb_q[31:16]);
      rot_ph  <= zr_ph;
      rot_tag <= {1'b0, zr_tag};
    end
  end
""" + t[idx + len("      endcase\n    end\n  end\n"):]
open(dst, "w").write(t)
