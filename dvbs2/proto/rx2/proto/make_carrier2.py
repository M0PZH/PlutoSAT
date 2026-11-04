"""PROTOTYPE (rx2 study): derive a two-slot dvbs2_rx_carrier2 from dvbs2_rx_carrier
mechanically. The FSM's registers get a shadow copy that holds the other slot's
context; in M_IDLE the two are swapped in one clock (ctx_swap) when the other
slot has a symbol and the current one has none or was just served. seg/zbuf get
the slot as address MSB, the output words carry the slot (m_slot).
Upper bound: every register the FSM assigns is shadowed except one-clock pulses
and unit operands (consumed inside a job)."""
import re
import sys

src, dst = sys.argv[1], sys.argv[2]
t = open(src).read()


def rep(old, new, count=1):
    global t
    n = t.count(old)
    assert n == count, (old[:60], n)
    t = t.replace(old, new)


# ---------------------------------------------------------------- registers to shadow
start = t.index("  always_ff @(posedge clk) begin\n    rot_v     <= 1'b0;")
fsm = t[start:t.index("\nendmodule")]
lhs = set(re.findall(r"\b([A-Za-z_]\w*)\s*(?:\[[^\]]*\])*\s*<=", fsm))
decl = {}
for m in re.finditer(r"^\s*(logic|st_t)\s*(signed)?\s*((?:\[[^\]]*\]\s*)*)([^;]*);", t, re.M):
    for nm in m.group(4).split(","):
        nm = nm.strip()
        mm = re.match(r"([A-Za-z_]\w*)\s*(\[[^\]]*\])?", nm)
        if mm:
            decl[mm.group(1)] = (m.group(1), m.group(2) or "", m.group(3).strip(), mm.group(2) or "")
TRANSIENT = {"st", "rot_v", "cor_start", "div_start", "lp_v", "seg_rv", "ty_v", "ty_dummy", "zw_v", "sw_v",
             "zr_v", "rot_i", "rot_q", "rot_ph", "rot_tag", "lp_i", "lp_q", "lp_g", "lp_zero", "cor_x", "cor_y",
             "div_n", "div_d", "zw_a", "zw_d", "sw_a", "sw_d", "zr_ph", "zr_tag", "seg_rph"}
ctx = sorted(n for n in lhs if n not in TRANSIENT)
for n in ctx:
    assert n in decl and decl[n][3] == "", n
bits = 0
for n in ctx:
    w = decl[n][2]
    if decl[n][0] == "st_t":
        bits += 6
    elif w:
        a, b = re.match(r"\[(\d+):(\d+)\]", w).groups()
        bits += int(a) - int(b) + 1
    else:
        bits += 1
print(f"{len(ctx)} context registers, {bits} bits")

SLOTLOGIC = '''  assign modcod  = modcod2[cur];
  assign pilots  = pilots2[cur];
  assign gold_xa = gold_xa2[cur];
  assign gold_xb = gold_xb2[cur];
  assign pll_kp  = pll_kp2[cur];
  assign pll_ki  = pll_ki2[cur];
  assign ctx_swap = !rst && st == M_IDLE && s_valid2[!cur] && (!s_valid2[cur] || served);
  assign s_valid = s_valid2[cur] && !ctx_swap;
  assign s_i = s_i2[cur];
  assign s_q = s_q2[cur];
  assign s_sof = s_sof2[cur];
  assign s_dummy = s_dummy2[cur];
  assign s_found = s_found2[cur];
  assign s_ready2[0] = s_ready && !cur && !ctx_swap;
  assign s_ready2[1] = s_ready && cur && !ctx_swap;
  assign ty_v2[0] = ty_v && !cur;
  assign ty_v2[1] = ty_v && cur;
  always_ff @(posedge clk) begin
    if (rst) begin
      cur <= 1'b0; served <= 1'b0;
    end else if (ctx_swap) begin
      cur <= !cur; served <= 1'b0;
    end else if (s_valid && s_ready) served <= 1'b1;
  end
'''
# ---------------------------------------------------------------- ports
hdr_start = t.index("module dvbs2_rx_carrier\n")
body_start = t.index(");\n", t.index("output logic signed [15:0] snap_q")) + 3
t = t[:hdr_start] + """module dvbs2_rx_carrier2
  import dvbs2_pl_pkg::*;
  import dvbs2_rx_pkg::*;
#(
  parameter string LUT_FILE = "nco_lut.mem"
) (
  input  logic               clk,
  input  logic               rst,

  input  logic [4:0]         modcod2 [2],
  input  logic [1:0]         pilots2,
  input  logic [17:0]        gold_xa2 [2],
  input  logic [17:0]        gold_xb2 [2],
  input  logic [17:0]        pll_kp2 [2],
  input  logic [17:0]        pll_ki2 [2],

  input  logic [1:0]         s_valid2,
  output logic [1:0]         s_ready2,
  input  logic signed [15:0] s_i2 [2],
  input  logic signed [15:0] s_q2 [2],
  input  logic [1:0]         s_sof2,
  input  logic [1:0]         s_dummy2,
  input  logic [1:0]         s_found2,

  output logic [1:0]         ty_v2,
  output logic               ty_dummy,

  output logic               m_valid,
  input  logic               m_ready,
  output logic [7:0]         m_llr_i,
  output logic [7:0]         m_llr_q,
  output logic               m_first,
  output logic               m_last,
  output logic               m_slot,

  output logic [31:0]        st_freq2 [2],
  output logic [31:0]        st_amp2 [2],
  output logic [35:0]        st_energy2 [2],
  output logic [17:0]        st_gain2 [2],
  output logic [15:0]        st_frames2 [2],
  output logic [15:0]        st_acq2 [2],
  output logic [15:0]        st_fa2 [2],
  output logic [15:0]        st_retype2 [2],

  output logic               snap_v,
  output logic signed [15:0] snap_i,
  output logic signed [15:0] snap_q,
  output logic               snap_slot
);

  // ---------------------------------------------------------------- slot selection
  logic        cur;                         // slot whose context is in the working registers
  logic        served;                      // a symbol of cur was taken since the last swap
  logic        ctx_swap;
  logic [4:0]  modcod;
  logic        pilots;
  logic [17:0] gold_xa, gold_xb, pll_kp, pll_ki;
  logic        s_valid, s_ready, s_sof, s_dummy, s_found, ty_v;
  logic signed [15:0] s_i, s_q;
  logic [31:0] st_freq, st_amp;
  logic [35:0] st_energy;
  logic [17:0] st_gain;
  logic [15:0] st_acq, st_fa, st_retype;
""" + t[body_start:]

# per-slot header pattern (registered per slot, selected by cur)
rep("""  logic [89:0] hdr_d, hdr_z;
  always_ff @(posedge clk) begin
    hdr_d <= plheader(modcod, pilots);
    hdr_z <= plheader(5'd0, 1'b0);
  end""", """  logic [89:0] hdr_d, hdr_z, hdr_ds [2];
  always_ff @(posedge clk) begin
    hdr_ds[0] <= plheader(modcod2[0], pilots2[0]);
    hdr_ds[1] <= plheader(modcod2[1], pilots2[1]);
    hdr_z <= plheader(5'd0, 1'b0);
  end
  assign hdr_d = hdr_ds[cur];""")

# per-slot buffers: slot = address MSB
rep("logic [31:0] zbuf [640];", "logic [31:0] zbuf [2048];")
rep("(* ram_style = \"block\" *) logic [31:0] seg [2048];", "(* ram_style = \"block\" *) logic [31:0] seg [4096];")
rep("always_ff @(posedge clk) if (sw_v) seg[sw_a] <= sw_d;", "always_ff @(posedge clk) if (sw_v) seg[{cur, sw_a}] <= sw_d;")
rep("always_ff @(posedge clk) if (zw_v) zbuf[zw_a] <= zw_d;", "always_ff @(posedge clk) if (zw_v) zbuf[{cur, zw_a}] <= zw_d;")
rep("always_ff @(posedge clk) zb_q <= zbuf[zb_ra];", "always_ff @(posedge clk) zb_q <= zbuf[{cur, zb_ra}];")
rep("always_ff @(posedge clk) if (seg_issue) seg_rd <= seg[sk];", "always_ff @(posedge clk) if (seg_issue) seg_rd <= seg[{cur, sk}];")

# output FIFO with the slot, per-slot output counters
rep("logic [15:0] ofifo [32];", "logic [16:0] ofifo [32];\n  logic        lp_tag, l1_tag, of_tag, hd;")
rep("logic [14:0] oc_out;", "logic [14:0] oc_out [2];\n  logic [15:0] st_frames [2];\n  assign hd = ofifo[of_rp[4:0]][16];")
rep("""      oc_out <= '0;
      st_frames <= '0;""", """      oc_out[0] <= '0; oc_out[1] <= '0;
      st_frames[0] <= '0; st_frames[1] <= '0;""")
rep("ofifo[of_wp[4:0]] <= of_din;", "ofifo[of_wp[4:0]] <= {of_tag, of_din};")
rep("""        {m_llr_q, m_llr_i} <= ofifo[of_rp[4:0]];
        m_first <= (oc_out == 0);
        m_last  <= (oc_out == 15'd32399);
        of_rp <= of_rp + 6'd1;
        if (oc_out == 15'd32399) begin
          oc_out <= '0;
          st_frames <= st_frames + 16'd1;
        end else oc_out <= oc_out + 15'd1;""", """        {m_slot, m_llr_q, m_llr_i} <= ofifo[of_rp[4:0]];
        m_first <= (oc_out[hd] == 0);
        m_last  <= (oc_out[hd] == 15'd32399);
        of_rp <= of_rp + 6'd1;
        if (oc_out[hd] == 15'd32399) begin
          oc_out[hd] <= '0;
          st_frames[hd] <= st_frames[hd] + 16'd1;
        end else oc_out[hd] <= oc_out[hd] + 15'd1;""")
rep("""      l1_v    <= lp_v;""", """      l1_v    <= lp_v;
      l1_tag  <= lp_tag;
      of_tag  <= l1_tag;""")
rep("  assign snap_q = lp_q;", "  assign snap_q = lp_q;\n  assign snap_slot = lp_tag;\n  always_ff @(posedge clk) lp_tag <= cur;")

# ---------------------------------------------------------------- shadows + swap
decls, cap, rest = [], [], []
for n in ctx:
    ty, sg, w, _ = decl[n]
    decls.append(f"  {ty} {sg + ' ' if sg else ''}{w + ' ' if w else ''}{n}_sh;")
    cap.append(f"      {n}_sh <= {n};")
    rest.append(f"      {n} <= {n}_sh;")
shadow = "\n  // ---------------------------------------------------------------- other slot's context\n" + \
    "\n".join(decls) + "\n  always_ff @(posedge clk) begin\n    if (ctx_swap || rst) begin\n" + "\n".join(cap) + \
    "\n    end\n  end\n"
anchor = "  always_ff @(posedge clk) begin\n    rot_v     <= 1'b0;"
t = t.replace(anchor, shadow + "\n" + anchor)
rep("""    if (zr_v) begin
      rot_v   <= 1'b1;
      rot_i   <= $signed(zb_q[15:0]);
      rot_q   <= $signed(zb_q[31:16]);
      rot_ph  <= zr_ph;
      rot_tag <= {1'b0, zr_tag};
    end
""", """    if (zr_v) begin
      rot_v   <= 1'b1;
      rot_i   <= $signed(zb_q[15:0]);
      rot_q   <= $signed(zb_q[31:16]);
      rot_ph  <= zr_ph;
      rot_tag <= {1'b0, zr_tag};
    end
    if (ctx_swap) begin
""" + "\n".join(rest) + "\n    end\n")

# per-slot status: working registers for cur, the shadows for the other slot
stat = []
for o, r in [("st_freq2", "fhat"), ("st_amp2", "A"), ("st_energy2", "E"), ("st_gain2", "G"),
             ("st_acq2", "n_acq"), ("st_fa2", "n_fa"), ("st_retype2", "n_retype")]:
    stat.append(f"  assign {o}[0] = cur ? {r}_sh : {r};\n  assign {o}[1] = cur ? {r} : {r}_sh;")
stat.append("  assign st_frames2[0] = st_frames[0];\n  assign st_frames2[1] = st_frames[1];")
stat.append(SLOTLOGIC)
t = t.replace("\nendmodule", "\n" + "\n".join(stat) + "\n\nendmodule")
t = t.replace("// (model: model/rxfixed.py, class Carrier; see there for the algorithm).",
              "// (model: model/rxfixed.py, class Carrier; see there for the algorithm).\n// PROTOTYPE two-slot version generated by rx2/proto/make_carrier2.py.", 1)
open(dst, "w").write(t)
