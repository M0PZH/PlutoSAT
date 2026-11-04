import re, sys
t = open(sys.argv[1]).read()
# main FSM block
start = t.index("  always_ff @(posedge clk) begin\n    rot_v     <= 1'b0;")
end_ = t.index("\nendmodule")
fsm = t[start:end_]
lhs = set(re.findall(r"\b([A-Za-z_]\w*)\s*(?:\[[^\]]*\])*\s*<=", fsm))
decl = {}
for m in re.finditer(r"^\s*(logic|st_t)\s*(signed)?\s*((?:\[[^\]]*\]\s*)*)([^;]*);", t, re.M):
    names = m.group(4)
    for nm in names.split(","):
        nm = nm.strip()
        if not nm: continue
        mm = re.match(r"([A-Za-z_]\w*)\s*(\[[^\]]*\])?", nm)
        decl[mm.group(1)] = (m.group(1), m.group(2) or "", m.group(3).strip(), mm.group(2) or "")
for n in sorted(lhs):
    d = decl.get(n)
    print(n, d)
