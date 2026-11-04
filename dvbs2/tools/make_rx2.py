"""Generate the two-slot RX engines (second DATV receive slot,
docs/rx2_design.md section 5) from the single-slot RTL:

    rtl/dvbs2_rx_framesync.sv -> rtl/dvbs2_rx_framesync2.sv
    rtl/dvbs2_rx_carrier.sv   -> rtl/dvbs2_rx_carrier2.sv

    cd dvbs2 && .venv/bin/python tools/make_rx2.py [--check]

(--check: compare with the files in rtl/ instead of writing them; the
generated files are committed, sim/rx/test_rx2.py runs the check.)

Method (context swap): every register of the engine's FSM that holds a
slot's state from one symbol to the next (the context) gets a shadow copy
that holds the other slot's context. At the engine's input state (frame
sync S_IN, carrier M_IDLE) the two are swapped in one clock (ctx_swap) when
the other slot has a symbol and the current one has none or was just
served. Per-symbol temporaries and the engine's own pipeline are not
shadowed: the lists below are hand-reviewed, and every register the FSM
assigns must be in exactly one of them, so a change of the single-slot RTL
stops the generator until the new registers are classified. Simulation
with RX2_SCRAMBLE defined overwrites the unshadowed temporaries with random
values at every swap (the tests run that way): a temporary that is in fact
read before it is written in a symbol shows up as a mismatch.

Each slot's buffers get the slot as an address bit, its configuration is
selected by the loaded slot, its status outputs come from the working or
the shadow register. Inputs that pulse at any time (frame sync: ty_v from
the carrier, pls_v from the PLS decoder) reach the slot's context wherever
it is (working, shadow, or both in a swap clock).

Per-slot reset (rst_s[s], a level or a pulse): the shadow slot's context is
reset at once; the loaded slot's when the engine is back at its input state
(the current symbol or header job finishes first, so the engine's units and
pipelines are idle; remembered in rp). A slot in reset takes no symbols and
is not swapped in. The other slot keeps running. The carrier drops the
output words of a reset slot still in its output FIFO (an epoch bit per
slot); the next word of that slot starts a new frame (m_first).
"""
import re
import sys
import os

HERE = os.path.dirname(os.path.abspath(__file__))
RTL = os.path.join(HERE, "..", "rtl")


class Src:
    def __init__(self, path):
        self.name = os.path.basename(path)
        self.t = open(path).read()

    def rep(self, old, new, count=1):
        n = self.t.count(old)
        assert n == count, (self.name, old[:70], n)
        self.t = self.t.replace(old, new)

    def resub(self, pat, new, count=1, flags=0):
        self.t, n = re.subn(pat, new, self.t, flags=flags)
        assert n == count, (self.name, pat[:70], n)


def declarations(t):
    """name -> (type, signed, packed dims, unpacked dims) of the `logic` / `st_t`
    declarations at module level."""
    decl = {}
    for m in re.finditer(r"^\s*(logic|st_t)\s+(signed\s+)?((?:\[[^\]]*\]\s*)*)([^;=]*);", t, re.M):
        for nm in m.group(4).split(","):
            mm = re.match(r"\s*([A-Za-z_]\w*)\s*((?:\[[^\]]*\]\s*)*)$", nm)
            if mm:
                decl[mm.group(1)] = (m.group(1), (m.group(2) or "").strip(), m.group(3).strip(),
                                     mm.group(2).strip())
    # ports
    for m in re.finditer(r"^\s*(?:input|output)\s+logic\s+(signed\s+)?((?:\[[^\]]*\]\s*)*)([A-Za-z_]\w*)"
                         r"\s*((?:\[[^\]]*\]\s*)*)[,)]", t, re.M):
        decl.setdefault(m.group(3), ("logic", (m.group(1) or "").strip(), m.group(2).strip(), m.group(4).strip()))
    return decl


def width(d):
    if d[0] == "st_t":
        return None
    if not d[2]:
        return 1
    w = 1
    for a, b in re.findall(r"\[(\d+):(\d+)\]", d[2]):
        w *= int(a) - int(b) + 1
    return w


def lhs_names(block):
    """Names assigned (non-blocking) in block: `name <=` or `name[..] <=` at the
    start of a statement (not a comparison inside an expression)."""
    out = set()
    block = re.sub(r"//[^\n]*", "", block)
    for m in re.finditer(r"\b([A-Za-z_]\w*)\s*(?:\[(?:[^\[\]]|\[[^\]]*\])*\])*\s*<=", block):
        before = block[:m.start()].rstrip()
        if not before or before.endswith((";", "begin", "end", "else", ")", ":")):
            out.add(m.group(1))
    return out


def block_at(t, start):
    """The text of the begin ... end block whose `begin` is the first one at or
    after `start` (begin/end nesting, case/endcase do not matter here)."""
    i = t.index("begin", start)
    depth = 0
    for m in re.finditer(r"\b(begin|end)\b", t[i:]):
        depth += 1 if m.group(1) == "begin" else -1
        if depth == 0:
            return i, i + m.end()
    raise ValueError("unbalanced")


def reset_values(t, rst_if):
    """name -> value of the plain assignments `name <= value;` in the block
    `rst_if` (e.g. "if (rst) begin") of the text t."""
    a, b = block_at(t, t.index(rst_if))
    vals = {}
    for m in re.finditer(r"\b([A-Za-z_]\w*)\s*<=\s*([^;]+);", t[a:b]):
        vals[m.group(1)] = m.group(2).strip()
    return vals, (a, b)


def classify(fsm_lhs, decl, context, temp, transient):
    sets = [set(context), set(temp), set(transient)]
    for i in range(3):
        for j in range(i + 1, 3):
            assert not sets[i] & sets[j], sets[i] & sets[j]
    unk = sorted(fsm_lhs - sets[0] - sets[1] - sets[2])
    assert not unk, f"registers assigned by the FSM and not classified (context, temp, transient): {unk}"
    gone = sorted((sets[0] | sets[1]) - fsm_lhs)
    assert not gone, f"classified registers the FSM no longer assigns: {gone}"
    for n in context:
        assert n in decl and not decl[n][3], f"context register {n}: not a scalar declaration"


def shadow_decls(ctx, decl):
    out = []
    for n in ctx:
        ty, sg, w, _ = decl[n]
        out.append(f"  {ty} {sg + ' ' if sg else ''}{w + ' ' if w else ''}{n}_sh;")
    return "\n".join(out)


def ctx_bits(ctx, decl, st_bits):
    return sum(width(decl[n]) or st_bits for n in ctx)


def ctx_comment(ctx, decl, st_bits, temp):
    bits = ctx_bits(ctx, decl, st_bits)
    lines = [f"  // context: {len(ctx)} registers, {bits} bits (shadowed):"]
    line = "  //  "
    for n in ctx:
        item = f" {n}({width(decl[n]) or st_bits})"
        if len(line) + len(item) > 100:
            lines.append(line)
            line = "  //  "
        line += item
    lines.append(line)
    lines.append("  // per-symbol / per-job temporaries (not shadowed, scrambled at swaps in RX2_SCRAMBLE):")
    line = "  //  "
    for n in sorted(temp):
        item = f" {n}"
        if len(line) + len(item) > 100:
            lines.append(line)
            line = "  //  "
        line += item
    lines.append(line)
    return "\n".join(lines)


def scramble(temp, decl):
    out = ["`ifdef RX2_SCRAMBLE"]
    for n in sorted(temp):
        d = decl[n]
        if d[3]:
            continue
        if d[0] == "st_t":
            out.append(f"      {n} <= st_t'($urandom);")
        else:
            out.append(f"      {n} <= $bits({n})'({{$urandom, $urandom}});")
    out.append("`endif")
    return "\n".join(out)


def subst_slot(text, names, idx):
    """Replace each word in `names` by name2[idx] (per-slot input arrays)."""
    for n in names:
        text = re.sub(rf"\b{n}\b(?!2)", f"{n}2[{idx}]", text)
    return text


def rename_lhs(text, regs, suffix="_sh"):
    for n in regs:
        text = re.sub(rf"\b{n}\b(\s*<=)", rf"{n}{suffix}\1", text)
    return text


HEADER = "// GENERATED by tools/make_rx2.py from {src} -- do not edit; edit the\n" \
         "// single-slot module (or the generator) and run the generator again.\n"


# =============================================================================
# frame sync
# =============================================================================

FS_CONTEXT = [
    "n", "pi_", "pq", "fst", "best_v", "best_m", "best_n", "win_v", "win_m", "win_n", "win_c",
    "cwin_m", "cwin_c", "pred", "miss", "pk", "nm", "pend_v", "pend_hs", "pend_c", "pend_f", "nchk",
    "chk_alt", "csum", "have_last", "last_n", "ty_p", "ty_pc", "pr_v", "pr_ok", "pr_dummy", "pw_on",
    "pw_cnt", "nbad", "cnt_found", "cnt_flywheel", "cnt_lost", "nodz",
]
# written and read within one symbol (S_IN .. S_OUT)
FS_TEMP = [
    "ci", "cq", "p1", "p2", "p3", "p4", "pdi", "pdq", "t", "Ad_i", "Ad_q", "Az_i", "Az_q",
    "sqd_i", "sqd_q", "sqz_i", "sqz_q", "md", "mz", "thresh_q", "thr", "nm4", "nm10", "inwin",
    "ad_i", "ad_q", "az_i", "az_q", "rd_ok", "rd_nd", "rd_nz",
]
# the engine itself: state, correlator pipeline, memory writes, output register
# (which carries its slot)
FS_TRANSIENT = ["st", "iss", "rd_v", "d_we", "d_wdata", "sbuf", "m_valid", "m_i", "m_q", "m_sof", "m_dummy",
                "m_found", "m_slot"]

# pulse inputs: the source's text, which the generator reuses for the other slot
FS_PULSES = """      if (ty_v) begin
        ty_p <= 1'b1; ty_pc <= ty_dummy;
      end
      if (pls_v) begin
        pr_v     <= 1'b1;
        pr_dummy <= pls_code[6:2] == 5'd0;
        pr_ok    <= pls_conf && (pls_code[6:2] == 5'd0 || pls_code == {modcod, 1'b0, pilots});
      end
"""
FS_PULSE_IN = ["ty_v", "pls_v", "pls_conf", "pls_code", "modcod", "pilots"]


def make_framesync2():
    s = Src(os.path.join(RTL, "dvbs2_rx_framesync.sv"))
    t0 = s.t
    decl = declarations(t0)
    fsm_start = t0.index("  always_ff @(posedge clk) begin\n    if (rst) begin\n      st <= S_IN;")
    fsm = t0[fsm_start:t0.rindex("endmodule")]
    classify(lhs_names(fsm), decl, FS_CONTEXT, FS_TEMP, FS_TRANSIENT)
    rv, _ = reset_values(fsm, "if (rst) begin")
    ctx = sorted(FS_CONTEXT)
    assert FS_PULSES in t0

    # ------------------------------------------------------------- header comment, ports
    s.rep("// DVB-S2 RX PLHEADER detection (model: model/rxfixed.py, class FrameSync).\n",
          HEADER.format(src="rtl/dvbs2_rx_framesync.sv") +
          "//\n// Two-slot DVB-S2 RX PLHEADER detection: one dvbs2_rx_framesync engine shared by\n"
          "// two receive slots by a context swap per symbol (tools/make_rx2.py explains\n"
          "// the method). Per slot it does exactly what dvbs2_rx_framesync does (model:\n"
          "// model/rxfixed.py, class FrameSync; tests: sim/rx/test_rx2.py). The output\n"
          "// register carries the slot (m_slot). The text below is the single-slot\n"
          "// module's.\n//\n")
    s.rep("module dvbs2_rx_framesync\n", "module dvbs2_rx_framesync2\n")
    s.rep("""  input  logic               clk,
  input  logic               rst,

  input  logic [4:0]         modcod,
  input  logic               pilots,
  input  logic [31:0]        thresh,

  input  logic               s_valid,
  output logic               s_ready,
  input  logic signed [15:0] s_i,
  input  logic signed [15:0] s_q,
""", """  input  logic               clk,
  input  logic               rst,
  input  logic [1:0]         rst_s,          // per-slot reset

  input  logic [4:0]         modcod2 [2],
  input  logic [1:0]         pilots2,
  input  logic [31:0]        thresh2 [2],

  input  logic [1:0]         s_valid2,
  output logic [1:0]         s_ready2,
  input  logic signed [15:0] s_i2 [2],
  input  logic signed [15:0] s_q2 [2],
""")
    s.rep("""  output logic               m_found,       // with m_sof: detected (not flywheel)

  input  logic               ty_v,
  input  logic               ty_dummy,

  // PLS decoder: result (pulse, with the code), decoding
  input  logic               pls_v,
  input  logic               pls_conf,
  input  logic [6:0]         pls_code,       // {MODCOD, short frame, pilots}
  input  logic               pls_busy,

  output logic [1:0]         fs_state,      // 0 search, 1 check, 2 locked
  output logic [15:0]        cnt_found,
  output logic [15:0]        cnt_flywheel,
  output logic [15:0]        cnt_lost
);
""", """  output logic               m_found,       // with m_sof: detected (not flywheel)
  output logic               m_slot,

  input  logic [1:0]         ty_v2,          // per slot (the carrier reports one at a time)
  input  logic               ty_dummy,

  // PLS decoders (one per slot): result (pulse, with the code), decoding
  input  logic [1:0]         pls_v2,
  input  logic [1:0]         pls_conf2,
  input  logic [6:0]         pls_code2 [2],
  input  logic [1:0]         pls_busy2,

  output logic [1:0]         fs_state2 [2],
  output logic [15:0]        cnt_found2 [2],
  output logic [15:0]        cnt_flywheel2 [2],
  output logic [15:0]        cnt_lost2 [2]
);

  // ------------------------------------------------------------ slot selection
  logic        cur;                         // slot whose context is in the working registers
  logic        served;                      // a symbol of cur was taken since the last swap
  logic        ctx_swap;
  logic [1:0]  rp;                          // reset of the loaded slot pending (applied in S_IN)
  logic        rl;                          // the loaded slot is reset in this clock
  logic [1:0]  srs;                         // a slot is in reset (its output is dropped)
  logic        cur_rst;                     // the loaded slot is in reset
  logic        av_cur, av_oth;              // the loaded / other slot can take a symbol
  logic        s_valid, s_ready, ty_v, pls_v, pls_conf, pls_busy, pilots;
  logic [4:0]  modcod;
  logic [6:0]  pls_code;
  logic [31:0] thresh;
  logic signed [15:0] s_i, s_q;
  logic [1:0]  fs_state;
  logic [15:0] cnt_found, cnt_flywheel, cnt_lost;
  assign modcod   = modcod2[cur];
  assign pilots   = pilots2[cur];
  assign thresh   = thresh2[cur];
  assign s_i      = s_i2[cur];
  assign s_q      = s_q2[cur];
  assign ty_v     = ty_v2[cur];
  assign pls_v    = pls_v2[cur];
  assign pls_conf = pls_conf2[cur];
  assign pls_code = pls_code2[cur];
  assign pls_busy = pls_busy2[cur];
""")
    # per-slot header coefficients (registered per slot, selected by cur)
    s.rep("  logic [89:0] hdr_d, hdr_z;\n", "  logic [89:0] hdr_z;\n")
    s.rep("""  always_ff @(posedge clk) begin
    hdr_d <= plheader(modcod, pilots);
    hdr_z <= plheader(5'd0, 1'b0);
    len_d <= pilots ? 32'd33282 : 32'd32490;
    for (int i = 1; i < 90; i++) begin
      // y_k = hdr[89 - k]; c_i = b_i b_(i-1) (-1 if i odd)
      neg_d[i] <= hdr_d[89 - i] ^ hdr_d[90 - i] ^ i[0];
      neg_z[i] <= hdr_z[89 - i] ^ hdr_z[90 - i] ^ i[0];
    end
  end
""", """  logic [89:1] neg_ds [2];
  logic [31:0] len_ds [2];
  always_ff @(posedge clk) begin
    hdr_z <= plheader(5'd0, 1'b0);
    for (int i = 1; i < 90; i++) begin
      // y_k = hdr[89 - k]; c_i = b_i b_(i-1) (-1 if i odd)
      neg_z[i] <= hdr_z[89 - i] ^ hdr_z[90 - i] ^ i[0];
    end
  end
  for (genvar s = 0; s < 2; s++) begin : g_hdr
    logic [89:0] h;
    logic [89:1] nd;
    logic [31:0] ld;
    always_ff @(posedge clk) begin
      h  <= plheader(modcod2[s], pilots2[s]);
      ld <= pilots2[s] ? 32'd33282 : 32'd32490;
      for (int i = 1; i < 90; i++) nd[i] <= h[89 - i] ^ h[90 - i] ^ i[0];
    end
    assign neg_ds[s] = nd;
    assign len_ds[s] = ld;
  end
  assign neg_d = neg_ds[cur];
  assign len_d = len_ds[cur];
""")
    # per-slot buffers: the slot is the address MSB
    s.rep("logic [31:0] sbuf [128];", "logic [31:0] sbuf [256];")
    s.rep("logic [23:0] mem [128];", "logic [23:0] mem [256];")
    s.rep("if (d_we) mem[n[6:0]] <= d_wdata;", "if (d_we) mem[{cur, n[6:0]}] <= d_wdata;")
    s.rep("rd[l] <= mem[ra];", "rd[l] <= mem[{cur, ra}];")
    s.rep("sbuf[n[6:0]] <= {s_q, s_i};", "sbuf[{cur, n[6:0]}] <= {s_q, s_i};")
    s.rep("sv = sbuf[o[6:0]];", "sv = sbuf[{cur, o[6:0]}];")
    # the output register carries its slot, so a swap need not wait for it (a
    # slot reset clears it only if it holds that slot's symbol)
    s.rep("      m_valid <= 1'b0;\n      cnt_found <= '0;", "      if (rst || m_slot == cur) m_valid <= 1'b0;\n      cnt_found <= '0;")
    s.rep("""            m_valid <= 1'b1;
            m_i <= $signed(sv[15:0]);""", """            m_valid <= !cur_rst;       // (a symbol of a slot in reset is dropped)
            m_slot  <= cur;
            m_i <= $signed(sv[15:0]);""")
    s.rep("      if (m_valid && m_ready) m_valid <= 1'b0;\n",
          "      if (m_valid && (m_ready || srs[m_slot])) m_valid <= 1'b0;\n")
    s.rep("  assign s_ready  = (st == S_IN) && !rst && !pls_hold;\n",
          "  assign s_ready  = (st == S_IN) && !rl && !pls_hold;\n")

    # ------------------------------------------------------------- reset of the working registers
    s.rep("  always_ff @(posedge clk) begin\n    if (rst) begin\n      st <= S_IN;",
          "  always_ff @(posedge clk) begin\n    if (rl) begin\n      st <= S_IN;")

    # ------------------------------------------------------------- shadows
    rsh = {n: rv[n] for n in ctx if n in rv}
    pulses_cur = rename_lhs(subst_slot(FS_PULSES, FS_PULSE_IN, "cur"), ctx).replace("\n      ", "\n        ")
    pulses_oth = rename_lhs(subst_slot(FS_PULSES, FS_PULSE_IN, "!cur"), ctx)
    pulses_in = subst_slot(FS_PULSES, FS_PULSE_IN, "!cur")
    shadow = "\n  // ------------------------------------------------------------ the other slot's context\n" + \
        ctx_comment(ctx, decl, 5, FS_TEMP) + "\n" + shadow_decls(ctx, decl) + """
  always_ff @(posedge clk) begin
    if (rst || !ctx_swap && rst_s[!cur]) begin
      // the shadow slot's reset
""" + "\n".join(f"      {n}_sh <= {v};" for n, v in rsh.items()) + """
    end else if (ctx_swap) begin
      if (cur_rst) begin
        // the slot swapped out is in reset
""" + "\n".join(f"        {n}_sh <= {v};" for n, v in rsh.items()) + """
      end else begin
""" + "\n".join(f"        {n}_sh <= {n};" for n in ctx) + """
        // (its pulses in this clock)
""" + pulses_cur + """      end
    end else begin
      // pulses for the slot in the shadow
""" + pulses_oth + """    end
  end

"""
    s.rep("  always_ff @(posedge clk) begin\n    if (rl) begin\n      st <= S_IN;",
          shadow + "  always_ff @(posedge clk) begin\n    if (rl) begin\n      st <= S_IN;")
    # swap: restore last in the FSM process (wins over the FSM's assignments)
    end_fsm = s.t.rindex("  end\n", 0, s.t.rindex("\nendmodule"))
    restore = "    if (ctx_swap) begin\n" + "\n".join(f"      {n} <= {n}_sh;" for n in ctx) + \
        "\n      // (pulses for the slot swapped in, in this clock)\n" + pulses_in + \
        scramble(FS_TEMP, decl) + "\n    end\n"
    s.t = s.t[:end_fsm] + restore + s.t[end_fsm:]

    stat = []
    for o, r in [("fs_state2", "fst"), ("cnt_found2", "cnt_found"), ("cnt_flywheel2", "cnt_flywheel"),
                 ("cnt_lost2", "cnt_lost")]:
        stat.append(f"  assign {o}[0] = cur ? {r}_sh : {r};\n  assign {o}[1] = cur ? {r} : {r}_sh;")
    s.t = s.t[:s.t.rindex("\nendmodule")] + """
  // ------------------------------------------------------------ slot selection
""" + "\n".join(stat) + """
  // the other slot's hold for its PLS decode (as pls_hold)
  wire hold_oth = pw_on_sh && pw_cnt_sh == '0 && pls_busy2[!cur];
  assign srs      = rst_s | rp;
  assign cur_rst  = srs[cur];
  assign rl       = rst || (st == S_IN && cur_rst);
  assign av_cur   = s_valid2[cur] && !pls_hold && !cur_rst;
  assign av_oth   = s_valid2[!cur] && !hold_oth && !rst_s[!cur];
  assign ctx_swap = !rst && st == S_IN && av_oth && (!av_cur || served);
  assign s_valid  = s_valid2[cur] && !ctx_swap && !cur_rst;
  assign s_ready2[0] = s_ready && !cur && !ctx_swap && !cur_rst;
  assign s_ready2[1] = s_ready && cur && !ctx_swap && !cur_rst;
  always_ff @(posedge clk) begin
    if (rst) begin
      cur <= 1'b0; served <= 1'b0; rp <= '0;
    end else begin
      if (ctx_swap) begin
        cur <= !cur; served <= 1'b0;
      end else if (s_valid && s_ready) served <= 1'b1;
      // a reset of the loaded slot during a symbol waits for S_IN
      if (st == S_IN) rp[cur] <= 1'b0;
      else if (rst_s[cur]) rp[cur] <= 1'b1;
    end
  end

endmodule
"""
    return s.t


# =============================================================================
# carrier
# =============================================================================

CR_CONTEXT = [
    "phi", "fhat", "phip", "fp", "nhdr", "in_frame", "typ", "p", "flen", "r_body", "xa", "xb", "ya", "yb",
    "idle", "Hi", "Hq", "Hai", "Haq", "Pi", "Pq", "Esum", "Pe", "A", "E", "nblk", "stat_rst", "G",
    "seg_pend", "owner_ended", "nd", "th0", "len0", "oc_push", "fmute", "smute", "acquired", "nconf",
    "big", "favg", "navg", "n_fa", "n_acq", "n_retype", "wb", "ap_v", "ap_typ", "prev_ok", "since",
    "cflip", "c90", "dth",
]
# written and read within one symbol or one header / known-block / segment job
# (M_IDLE back to M_IDLE)
CR_TEMP = [
    "si", "sq", "ssof", "sdum", "sfnd", "s1i", "s1q", "zi", "zq", "zai", "zaq", "z_hdr", "z_pil", "z_dum",
    "z_soft", "ci", "cq", "pll_e", "sq_zi", "sq_zq", "p_ki", "p_kp", "hh", "hha", "fk", "fk_got", "fst",
    "ff", "fph", "fh", "fh_last", "Ri", "Rq", "Si", "Sq", "zki", "zkq", "zmi", "zmq", "zm_ok", "pr_a",
    "pr_b", "pr_c", "pr_d", "p_ang", "f_c", "fsh", "fkick", "theta", "mag", "bsum", "blen", "blk_opens",
    "p_a", "p_e", "a_new", "e_new", "p_aa", "p_acg", "dl_q", "p_dl", "incr", "fa_kp", "fa_big", "fa_k",
    "fa_stepk", "fa_jump", "sph", "sk", "sk_done", "inflight", "p_dd", "ret_pad", "ret_blk", "seg_cont",
    "zrbuf", "hz1_si", "hz1_sq", "hz1_a", "hz2_a", "hz2_i", "hz2_q", "zr2_ph", "zr2_tag",
]
# the engine itself: state, unit operands and strobes, buffer write ports, the
# LLR pipeline input (its words carry the slot), the header z and zbuf read pipelines
CR_TRANSIENT = [
    "st", "rot_v", "rot_i", "rot_q", "rot_ph", "rot_tag", "cor_start", "cor_x", "cor_y", "div_start",
    "div_n", "div_d", "lp_v", "lp_zero", "lp_i", "lp_q", "lp_g", "lp_mute", "seg_rv", "seg_rph", "ty_v",
    "ty_dummy",
    "zw_v", "zw_a", "zw_d", "sw_v", "sw_a", "sw_d", "zr_v", "zr_ph", "zr_tag", "zr2_v", "hz1_v", "hz2_v",
]


# BRAM packing of the carrier's per-slot buffers (rx2_design.md section 8):
# zbuf 2 -> 1.5 tiles, seg 4 -> 3 tiles
ZBUF_SPLIT = True
SEG_PACK = True


def make_carrier2():
    s = Src(os.path.join(RTL, "dvbs2_rx_carrier.sv"))
    t0 = s.t
    decl = declarations(t0)
    fsm_start = t0.index("  always_ff @(posedge clk) begin\n    rot_v     <= 1'b0;")
    fsm = t0[fsm_start:t0.rindex("endmodule")]
    classify(lhs_names(fsm), decl, CR_CONTEXT, CR_TEMP, CR_TRANSIENT)
    rv, _ = reset_values(fsm, "if (rst) begin")
    ctx = sorted(CR_CONTEXT)

    s.rep("// DVB-S2 RX carrier recovery, PL descrambling and QPSK soft demapping\n",
          HEADER.format(src="rtl/dvbs2_rx_carrier.sv") +
          "//\n// Two-slot carrier: one dvbs2_rx_carrier engine shared by two receive slots by\n"
          "// a context swap in M_IDLE (tools/make_rx2.py explains the method). Per slot it\n"
          "// does exactly what dvbs2_rx_carrier does (model: model/rxfixed.py, class\n"
          "// Carrier; tests: sim/rx/test_rx2.py). seg and zbuf hold both slots' data (the\n"
          "// slot is an address bit), the LLR words carry their slot through the LLR\n"
          "// pipeline and the output FIFO (m_slot; m_first / m_last per slot), ty_v2 and\n"
          "// the status outputs are per slot. The text below is the single-slot module's.\n//\n"
          "// DVB-S2 RX carrier recovery, PL descrambling and QPSK soft demapping\n")
    s.rep("module dvbs2_rx_carrier\n", "module dvbs2_rx_carrier2\n")
    # ------------------------------------------------------------- ports
    a = s.t.index("  input  logic               clk,\n")
    b = s.t.index(");\n", s.t.index("output logic signed [15:0] snap_q")) + 3
    s.t = s.t[:a] + """  input  logic               clk,
  input  logic               rst,
  input  logic [1:0]         rst_s,        // per-slot reset

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
  output logic [1:0]         st_acquired2,

  output logic               snap_v,
  output logic signed [15:0] snap_i,
  output logic signed [15:0] snap_q,
  output logic               snap_slot
);

  // ---------------------------------------------------------------- slot selection
  logic        cur;                         // slot whose context is in the working registers
  logic        served;                      // a symbol of cur was taken since the last swap
  logic        ctx_swap;
  logic [1:0]  rp;                          // reset of the loaded slot pending (applied in M_IDLE)
  logic        rl;                          // the loaded slot is reset in this clock
  logic [1:0]  srs;                         // a slot is in reset (its output is dropped)
  logic        cur_rst;                     // the loaded slot is in reset
  logic [1:0]  app, app_d;                  // a slot's context is reset in this / the last clock
  logic [1:0]  ep;                          // output epoch per slot (words of an older one are dropped)
  logic [4:0]  modcod;
  logic        pilots;
  logic [17:0] gold_xa, gold_xb, pll_kp, pll_ki;
  logic        s_valid, s_ready, s_sof, s_dummy, s_found, ty_v;
  logic signed [15:0] s_i, s_q;
  logic [31:0] st_freq, st_amp;
  logic [35:0] st_energy;
  logic [17:0] st_gain;
  logic [15:0] st_acq, st_fa, st_retype;
  logic        st_acquired;
""" + s.t[b:]

    # per-slot header pattern (registered per slot, selected by cur)
    s.rep("""  logic [89:0] hdr_d, hdr_z;
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

    # per-slot buffers, the slot as an address bit
    if ZBUF_SPLIT:
        # zbuf: banks 0..3 of both slots in one RAMB36 (1024 x 32, address
        # {bank[1:0], slot, k}), bank 4 of both in a RAMB18 (256 x 32, {slot, k});
        # the bank select is registered with the read, the choice made at zb_q
        s.rep("logic [31:0] zbuf [640];                  // 5 banks of header s1, then z / 2 (16-bit pairs)",
              "// 5 banks of header s1, then z / 2 (16-bit pairs), per slot: banks 0..3 in zbuf, bank 4 in zbuf4\n"
              "  (* ram_style = \"block\" *) logic [31:0] zbuf [1024];\n"
              "  (* ram_style = \"block\" *) logic [31:0] zbuf4 [256];\n"
              "  logic [31:0] zb_q0a, zb_q0b;\n  logic        zb_s1;")
        s.rep("always_ff @(posedge clk) if (zw_v) zbuf[zw_a] <= zw_d;",
              "always_ff @(posedge clk) if (zw_v && !zw_a[9]) zbuf[{zw_a[8:7], cur, zw_a[6:0]}] <= zw_d;\n"
              "  always_ff @(posedge clk) if (zw_v && zw_a[9]) zbuf4[{cur, zw_a[6:0]}] <= zw_d;")
        s.rep("    zb_q0 <= zbuf[zb_ra];\n    zb_q  <= zb_q0;\n",
              "    zb_q0a <= zbuf[{zb_ra[8:7], cur, zb_ra[6:0]}];\n"
              "    zb_q0b <= zbuf4[{cur, zb_ra[6:0]}];\n"
              "    zb_s1  <= zb_ra[9];               // (banks are 0..4)\n"
              "    zb_q   <= zb_s1 ? zb_q0b : zb_q0a;\n")
    else:
        # zbuf: 5 banks of 128 per slot, address {bank, slot, k} (1280 words)
        s.rep("logic [31:0] zbuf [640];", "logic [31:0] zbuf [1280];")
        s.rep("always_ff @(posedge clk) if (zw_v) zbuf[zw_a] <= zw_d;",
              "always_ff @(posedge clk) if (zw_v) zbuf[{zw_a[9:7], cur, zw_a[6:0]}] <= zw_d;")
        s.rep("    zb_q0 <= zbuf[zb_ra];", "    zb_q0 <= zbuf[{zb_ra[9:7], cur, zb_ra[6:0]}];")
    if SEG_PACK:
        # seg: 1440 words per slot in 3 RAMB36: k < 1024 in seg (2048 x 32,
        # {slot, k[9:0]}), 1024 <= k < 1440 in seg1 (1024 x 32, {slot, k[8:0]});
        # the half read is registered with the read, the choice is made at seg_rd
        s.rep("(* ram_style = \"block\" *) logic [31:0] seg [2048];   // pending data symbols",
              "(* ram_style = \"block\" *) logic [31:0] seg [2048];   // pending data symbols, k < 1024\n"
              "  (* ram_style = \"block\" *) logic [31:0] seg1 [1024];  // ... 1024 <= k < 1440\n"
              "  logic [31:0] seg_rd0, seg_rd1;\n  logic        seg_rs;")
        s.rep("  logic [31:0] seg_rd;\n", "  logic [31:0] seg_rd;\n  assign seg_rd = seg_rs ? seg_rd1 : seg_rd0;\n")
        s.rep("always_ff @(posedge clk) if (sw_v) seg[sw_a] <= sw_d;",
              "always_ff @(posedge clk) if (sw_v && !sw_a[10]) seg[{cur, sw_a[9:0]}] <= sw_d;\n"
              "  always_ff @(posedge clk) if (sw_v && sw_a[10]) seg1[{cur, sw_a[8:0]}] <= sw_d;")
        s.rep("always_ff @(posedge clk) if (seg_issue) seg_rd <= seg[sk];",
              "always_ff @(posedge clk) if (seg_issue) begin\n"
              "    seg_rd0 <= seg[{cur, sk[9:0]}];\n"
              "    seg_rd1 <= seg1[{cur, sk[8:0]}];\n"
              "    seg_rs  <= sk[10];\n"
              "  end")
    else:
        s.rep("(* ram_style = \"block\" *) logic [31:0] seg [2048];",
              "(* ram_style = \"block\" *) logic [31:0] seg [4096];")
        s.rep("always_ff @(posedge clk) if (sw_v) seg[sw_a] <= sw_d;",
              "always_ff @(posedge clk) if (sw_v) seg[{cur, sw_a}] <= sw_d;")
        s.rep("always_ff @(posedge clk) if (seg_issue) seg_rd <= seg[sk];",
              "always_ff @(posedge clk) if (seg_issue) seg_rd <= seg[{cur, sk}];")

    # output FIFO: words carry {epoch, slot}; per-slot output counters
    s.rep("logic [15:0] ofifo [32];",
          "logic [17:0] ofifo [32];\n  logic        lp_tag, l1_tag, of_tag, lp_ep, l1_ep, of_ep, hd, stale;")
    s.rep("logic [14:0] oc_out;", "logic [14:0] oc_out [2];\n  logic [15:0] st_frames [2];\n"
          "  assign hd = ofifo[of_rp[4:0]][16];\n"
          "  // a word of a slot in reset, or reset since it was written, is dropped\n"
          "  assign stale = ofifo[of_rp[4:0]][17] != ep[hd] || srs[hd];")
    s.rep("""      if (m_valid && m_ready) m_valid <= 1'b0;
      if ((!m_valid || m_ready) && of_cnt != 0) begin""", """      if (m_valid && (m_ready || srs[m_slot])) m_valid <= 1'b0;
      if ((!m_valid || m_ready) && of_cnt != 0) begin""")
    s.rep("""      oc_out <= '0;
      st_frames <= '0;""", """      oc_out[0] <= '0; oc_out[1] <= '0;
      st_frames[0] <= '0; st_frames[1] <= '0;""")
    s.rep("ofifo[of_wp[4:0]] <= of_din;", "ofifo[of_wp[4:0]] <= {of_ep, of_tag, of_din};")
    s.rep("""      if ((!m_valid || m_ready) && of_cnt != 0) begin
        m_valid <= 1'b1;
        {m_llr_q, m_llr_i} <= ofifo[of_rp[4:0]];
        m_first <= (oc_out == 0);
        m_last  <= (oc_out == 15'd32399);
        of_rp <= of_rp + 6'd1;
        if (oc_out == 15'd32399) begin
          oc_out <= '0;
          st_frames <= st_frames + 16'd1;
        end else oc_out <= oc_out + 15'd1;
      end
    end""", """      if ((!m_valid || m_ready) && of_cnt != 0 && stale) of_rp <= of_rp + 6'd1;   // dropped
      else if ((!m_valid || m_ready) && of_cnt != 0) begin
        m_valid <= 1'b1;
        {m_slot, m_llr_q, m_llr_i} <= ofifo[of_rp[4:0]][16:0];
        m_first <= (oc_out[hd] == 0);
        m_last  <= (oc_out[hd] == 15'd32399);
        of_rp <= of_rp + 6'd1;
        if (oc_out[hd] == 15'd32399) begin
          oc_out[hd] <= '0;
          st_frames[hd] <= st_frames[hd] + 16'd1;
        end else oc_out[hd] <= oc_out[hd] + 15'd1;
      end
      // a slot reset: its next word starts a frame
      for (int k = 0; k < 2; k++) if (app[k]) begin
        oc_out[k] <= '0;
        st_frames[k] <= '0;
      end
    end""")
    s.rep("""      l1_v    <= lp_v;""", """      l1_v    <= lp_v;
      l1_tag  <= lp_tag;
      of_tag  <= l1_tag;
      l1_ep   <= lp_ep;
      of_ep   <= l1_ep;""")
    s.rep("  assign snap_q = lp_q;", "  assign snap_q = lp_q;\n  assign snap_slot = lp_tag;\n"
          "  // (lp_* are written by the FSM in the clock it runs for cur)\n"
          "  always_ff @(posedge clk) begin\n    lp_tag <= cur;\n    lp_ep  <= ep[cur];\n  end")
    s.rep("  assign s_ready = (st == M_IDLE) && !rst;", "  assign s_ready = (st == M_IDLE) && !rl;")

    # ------------------------------------------------------------- reset of the working registers
    a = s.t.index("  always_ff @(posedge clk) begin\n    rot_v     <= 1'b0;")
    i = s.t.index("    if (rst) begin\n      st <= M_IDLE;", a)
    s.t = s.t[:i] + "    if (rl) begin\n      st <= M_IDLE;" + s.t[i + len("    if (rst) begin\n      st <= M_IDLE;"):]

    # ------------------------------------------------------------- shadows
    rsh = {n: rv[n] for n in ctx if n in rv}
    shadow = "\n  // ---------------------------------------------------------------- the other slot's context\n" + \
        ctx_comment(ctx, decl, 6, CR_TEMP) + "\n" + shadow_decls(ctx, decl) + """
  always_ff @(posedge clk) begin
    if (rst || !ctx_swap && rst_s[!cur] || ctx_swap && cur_rst) begin
      // the shadow slot's reset (or the slot swapped out is in reset)
""" + "\n".join(f"      {n}_sh <= {v};" for n, v in rsh.items()) + """
    end else if (ctx_swap) begin
""" + "\n".join(f"      {n}_sh <= {n};" for n in ctx) + """
    end
  end
"""
    anchor = "  always_ff @(posedge clk) begin\n    rot_v     <= 1'b0;"
    s.rep(anchor, shadow + "\n" + anchor)
    end_fsm = s.t.rindex("  end\n", 0, s.t.rindex("\nendmodule"))
    restore = "    if (ctx_swap) begin\n" + "\n".join(f"      {n} <= {n}_sh;" for n in ctx) + "\n" + \
        scramble(CR_TEMP, decl) + "\n    end\n"
    s.t = s.t[:end_fsm] + restore + s.t[end_fsm:]

    stat = []
    for o, r in [("st_freq2", "fhat"), ("st_amp2", "A"), ("st_energy2", "E"), ("st_gain2", "G"),
                 ("st_acq2", "n_acq"), ("st_fa2", "n_fa"), ("st_retype2", "n_retype"),
                 ("st_acquired2", "acquired")]:
        stat.append(f"  assign {o}[0] = cur ? {r}_sh : {r};\n  assign {o}[1] = cur ? {r} : {r}_sh;")
    stat.append("  assign st_frames2[0] = st_frames[0];\n  assign st_frames2[1] = st_frames[1];")
    s.t = s.t[:s.t.rindex("\nendmodule")] + """
  // ---------------------------------------------------------------- slot selection
""" + "\n".join(stat) + """
  assign modcod   = modcod2[cur];
  assign pilots   = pilots2[cur];
  assign gold_xa  = gold_xa2[cur];
  assign gold_xb  = gold_xb2[cur];
  assign pll_kp   = pll_kp2[cur];
  assign pll_ki   = pll_ki2[cur];
  assign s_i      = s_i2[cur];
  assign s_q      = s_q2[cur];
  assign s_sof    = s_sof2[cur];
  assign s_dummy  = s_dummy2[cur];
  assign s_found  = s_found2[cur];
  assign ty_v2[0] = ty_v && !cur;
  assign ty_v2[1] = ty_v && cur;
  assign srs      = rst_s | rp;
  assign cur_rst  = srs[cur];
  assign rl       = rst || (st == M_IDLE && cur_rst);
  assign ctx_swap = !rst && st == M_IDLE && s_valid2[!cur] && !rst_s[!cur] &&
                    (!(s_valid2[cur] && !cur_rst) || served);
  assign s_valid  = s_valid2[cur] && !ctx_swap && !cur_rst;
  assign s_ready2[0] = s_ready && !cur && !ctx_swap && !cur_rst;
  assign s_ready2[1] = s_ready && cur && !ctx_swap && !cur_rst;
  assign app[0]   = cur ? rst_s[0] : rl;
  assign app[1]   = cur ? rl : rst_s[1];
  always_ff @(posedge clk) begin
    if (rst) begin
      cur <= 1'b0; served <= 1'b0; rp <= '0;
      ep <= '0; app_d <= '0;
    end else begin
      app_d <= app;
      for (int k = 0; k < 2; k++) if (app[k] && !app_d[k]) ep[k] <= !ep[k];
      if (ctx_swap) begin
        cur <= !cur; served <= 1'b0;
      end else if (s_valid && s_ready) served <= 1'b1;
      // a reset of the loaded slot during a job waits for M_IDLE
      if (st == M_IDLE) rp[cur] <= 1'b0;
      else if (rst_s[cur]) rp[cur] <= 1'b1;
    end
  end
`ifndef SYNTHESIS
  // (a segment never holds more than 1440 symbols)
  always @(posedge clk) if (sw_v && sw_a >= 11'd1440) $error("carrier2: segment longer than 1440");
`endif

endmodule
"""
    return s.t


def main():
    check = "--check" in sys.argv
    bad = []
    for name, fn in [("dvbs2_rx_framesync2.sv", make_framesync2), ("dvbs2_rx_carrier2.sv", make_carrier2)]:
        text = fn()
        path = os.path.join(RTL, name)
        if check:
            if not os.path.exists(path) or open(path).read() != text:
                bad.append(name)
        else:
            open(path, "w").write(text)
            print("wrote", os.path.normpath(path))
    if bad:
        print("out of date (run tools/make_rx2.py):", " ".join(bad))
        sys.exit(1)


if __name__ == "__main__":
    main()
