"""PROTOTYPE (rx2 study, resource numbers only, not simulated): two-slot frame
sync by the same shadow-register swap as make_carrier2.py. Swap in S_IN when
the output register is empty and the other slot has a symbol."""
import re
import sys

src, dst = sys.argv[1], sys.argv[2]
t = open(src).read()


def rep(old, new, count=1):
    global t
    n = t.count(old)
    assert n == count, (old[:60], n)
    t = t.replace(old, new)


start = t.index("  always_ff @(posedge clk) begin\n    if (rst) begin\n      st <= S_IN;")
fsm = t[start:t.index("\nendmodule")]
lhs = set(re.findall(r"\b([A-Za-z_]\w*)\s*(?:\[[^\]]*\])*\s*<=", fsm))
decl = {}
for m in re.finditer(r"^\s*(logic|st_t)\s*(signed)?\s*((?:\[[^\]]*\]\s*)*)([^;]*);", t, re.M):
    for nm in m.group(4).split(","):
        nm = nm.strip()
        mm = re.match(r"([A-Za-z_]\w*)\s*(\[[^\]]*\])?", nm)
        if mm:
            decl[mm.group(1)] = (m.group(1), m.group(2) or "", m.group(3).strip(), mm.group(2) or "")
TRANSIENT = {"st", "m_valid", "m_i", "m_q", "m_sof", "m_dummy", "m_found", "d_we", "d_wdata", "sbuf",
             "rd_v", "iss", "cnt_found", "cnt_flywheel", "cnt_lost", "rd_ok", "rd_nd", "rd_nz", "ad_i", "ad_q",
             "az_i", "az_q", "ty_p", "ty_pc"}
ctx = sorted(n for n in lhs if n not in TRANSIENT and n in decl and decl[n][3] == "")
skipped = sorted(n for n in lhs if n not in TRANSIENT and n not in ctx)
bits = 0
for n in ctx:
    w = decl[n][2]
    bits += 5 if decl[n][0] == "st_t" else (int(re.match(r"\[(\d+):(\d+)\]", w).group(1)) -
                                            int(re.match(r"\[(\d+):(\d+)\]", w).group(2)) + 1 if w else 1)
print(f"{len(ctx)} context registers, {bits} bits; not shadowed: {skipped}")

rep("module dvbs2_rx_framesync\n", "module dvbs2_rx_framesync2\n")
rep("""  input  logic [4:0]         modcod,
  input  logic               pilots,
  input  logic [31:0]        thresh,

  input  logic               s_valid,
  output logic               s_ready,
  input  logic signed [15:0] s_i,
  input  logic signed [15:0] s_q,
""", """  input  logic [4:0]         modcod2 [2],
  input  logic [1:0]         pilots2,
  input  logic [31:0]        thresh2 [2],

  input  logic [1:0]         s_valid2,
  output logic [1:0]         s_ready2,
  input  logic signed [15:0] s_i2 [2],
  input  logic signed [15:0] s_q2 [2],
""")
rep("""  output logic               m_found,       // with m_sof: detected (not flywheel)

  input  logic               ty_v,
  input  logic               ty_dummy,

  output logic [1:0]         fs_state,      // 0 search, 1 check, 2 locked
  output logic [15:0]        cnt_found,
  output logic [15:0]        cnt_flywheel,
  output logic [15:0]        cnt_lost
);""", """  output logic               m_found,       // with m_sof: detected (not flywheel)
  output logic               m_slot,

  input  logic [1:0]         ty_v2,
  input  logic               ty_dummy,

  output logic [1:0]         fs_state2 [2],
  output logic [15:0]        cnt_found2 [2],
  output logic [15:0]        cnt_flywheel2 [2],
  output logic [15:0]        cnt_lost2 [2]
);
  logic        cur, served, ctx_swap, s_valid, s_ready, ty_v, pilots;
  logic [4:0]  modcod;
  logic [31:0] thresh;
  logic signed [15:0] s_i, s_q;
  logic [1:0]  fs_state;
  logic [15:0] cnt_found [2], cnt_flywheel [2], cnt_lost [2];
  logic [15:0] cnt_found_w, cnt_flywheel_w, cnt_lost_w;
  assign modcod = modcod2[cur];
  assign pilots = pilots2[cur];
  assign thresh = thresh2[cur];
  assign s_i = s_i2[cur];
  assign s_q = s_q2[cur];
  assign ty_v = ty_v2[cur];
  assign m_slot = cur;""")
# per-slot coefficient registers
rep("""  logic [89:0] hdr_d, hdr_z;
  logic [89:1] neg_d, neg_z;                 // c_i = -1
  logic [31:0] len_d;""", """  logic [89:0] hdr_d, hdr_z;
  logic [89:1] neg_d, neg_z;                 // c_i = -1
  logic [31:0] len_d;
  logic [89:1] neg_ds [2];
  logic [31:0] len_ds [2];
  for (genvar s = 0; s < 2; s++) begin : g_hdr
    logic [89:0] h;
    always_ff @(posedge clk) begin
      h <= plheader(modcod2[s], pilots2[s]);
      len_ds[s] <= pilots2[s] ? 32'd33282 : 32'd32490;
    end
    for (genvar i = 1; i < 90; i++) begin : g_i
      always_ff @(posedge clk) neg_ds[s][i] <= h[89 - i] ^ h[90 - i] ^ i[0];
    end
  end""")
t = t.replace("neg_d[i] <= hdr_d[89 - i] ^ hdr_d[90 - i] ^ i[0];", "")
t = t.replace("len_d <= pilots ? 32'd33282 : 32'd32490;", "")
t = t.replace("    hdr_d <= plheader(modcod, pilots);\n", "")
rep("  localparam logic [31:0] LEN_Z = 32'd3330;", "  localparam logic [31:0] LEN_Z = 32'd3330;\n  assign neg_d = neg_ds[cur];\n  assign len_d = len_ds[cur];")
rep("logic [31:0] sbuf [128];", "logic [31:0] sbuf [256];")
rep("logic [23:0] mem [128];", "logic [23:0] mem [256];")
rep("if (d_we) mem[n[6:0]] <= d_wdata;", "if (d_we) mem[{cur, n[6:0]}] <= d_wdata;")
rep("rd[l] <= mem[ra];", "rd[l] <= mem[{cur, ra}];")
rep("sbuf[n[6:0]] <= {s_q, s_i};", "sbuf[{cur, n[6:0]}] <= {s_q, s_i};")
rep("sv = sbuf[o[6:0]];", "sv = sbuf[{cur, o[6:0]}];")
# counters per slot (by cur)
a0 = t.index("  always_ff @(posedge clk) begin\n    if (rst) begin\n      st <= S_IN;")
head, body = t[:a0], t[a0:]
for c in ("cnt_found", "cnt_flywheel", "cnt_lost"):
    body = re.sub(rf"\b{c}\b(?!\[|2|_w)", f"{c}[cur]", body)
t = head + body
rep("cnt_found[cur] <= '0; cnt_flywheel[cur] <= '0; cnt_lost[cur] <= '0;",
    "for (int s = 0; s < 2; s++) begin cnt_found[s] <= '0; cnt_flywheel[s] <= '0; cnt_lost[s] <= '0; end")
# shadows
decls, cap, rest = [], [], []
for n in ctx:
    ty, sg, w, _ = decl[n]
    decls.append(f"  {ty} {sg + ' ' if sg else ''}{w + ' ' if w else ''}{n}_sh;")
    cap.append(f"      {n}_sh <= {n};")
    rest.append(f"      {n} <= {n}_sh;")
anchor = "  always_ff @(posedge clk) begin\n    if (rst) begin\n      st <= S_IN;"
t = t.replace(anchor, "\n".join(decls) + """
  logic ty_p_sh, ty_pc_sh;
  always_ff @(posedge clk) begin
    if (ctx_swap || rst) begin
""" + "\n".join(cap) + """
      ty_p_sh <= ty_p; ty_pc_sh <= ty_pc;
    end
    if (!ctx_swap && !rst && ty_v2[!cur]) begin ty_p_sh <= 1'b1; ty_pc_sh <= ty_dummy; end
  end
""" + anchor)
idx = t.rindex("\nendmodule")
end_fsm = t.rindex("  end\n", 0, idx)
t = t[:end_fsm] + "    if (ctx_swap) begin\n" + "\n".join(rest) + \
    "\n      ty_p <= ty_p_sh; ty_pc <= ty_pc_sh;\n    end\n" + t[end_fsm:]
t = t[:t.rindex("\nendmodule")] + """
  assign ctx_swap = !rst && st == S_IN && !m_valid && s_valid2[!cur] && (!s_valid2[cur] || served);
  assign s_valid = s_valid2[cur] && !ctx_swap;
  assign s_ready2[0] = s_ready && !cur && !ctx_swap;
  assign s_ready2[1] = s_ready && cur && !ctx_swap;
  assign fs_state2[0] = cur ? fst_sh : fst;
  assign fs_state2[1] = cur ? fst : fst_sh;
  assign cnt_found2 = cnt_found;
  assign cnt_flywheel2 = cnt_flywheel;
  assign cnt_lost2 = cnt_lost;
  always_ff @(posedge clk) begin
    if (rst) begin cur <= 1'b0; served <= 1'b0; end
    else if (ctx_swap) begin cur <= !cur; served <= 1'b0; end
    else if (s_valid && s_ready) served <= 1'b1;
  end

endmodule
"""
open(dst, "w").write(t)
