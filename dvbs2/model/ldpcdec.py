"""DVB-S2 LDPC decoder, normal and short FECFRAMEs: bit-exact model of rtl/dvbs2_ldpc_dec.sv.

Layered offset min-sum on the quasi-cyclic form of the parity-check
matrix (EN 302 307-1 5.3.2 tables):

  check c = j + q*t      (row group / layer j = 0..q-1, row t = 0..359)
  info bit 360*g + m     is in check (x + m*q) mod M for each table entry x of
                         column group g  ->  layer x mod q, shift t0 = x div q,
                         row t holds m = (t - t0) mod 360
  parity bit i = j + q*m (stored as parity column group j, m = 0..359):
                         check c holds p_c and p_(c-1):  layer j -> parity
                         groups j (shift 0) and j-1 (shift 0); layer 0 ->
                         group q-1 with shift 1, absent in row t = 0

P lanes process the checks t = b + S*l (l = 0..P-1, S = 360/P) of sub-step
b together, one edge per clock: phase A reads the APP values L of all edges
(Q = L - R_old, min1/min2/index/signs), phase B writes L += R_new - R_old,
edge by edge (so circulants of one column group that share a layer, which
the DVB-S2 tables have, update their variables one after the other). Each
column group is stored as S words of P values: word w, lane i holds
m = w + S*i; an edge reads word (b - t0) mod S rotated by (b - t0) div S.

Stops after an iteration with all checks satisfied and no APP sign change.

Short FECFRAMEs: some codes (1/4, 1/2, 3/4, 4/5, 5/6) have layers of
different check degrees; their layers are padded to the largest degree
with absent edges (PAD_G in the RTL edge ROM: never the minimum, sign 0, no
parity contribution, no write), so every layer of a code has the same
number of edges.
"""

import numpy as np

import txmodel as m

P = 18
S = 360 // P

# fixed point (LLR input: int8, 4 LSBs per unit LLR)
WA = 9               # APP L
WQ = 9               # Q = L - R
RMAX = 63            # check message magnitude
MAX_IT = 30
# Channel LLR clip at the decoder input. A check sends at most RMAX - OFFSET
# = 62, so a channel LLR beyond that can never be overruled where few checks
# vote: a degree-1 (the last parity bit) or degree-2 (parity) variable whose
# channel LLR is wrong at full scale (an impulse at high SNR, where the LLR
# gain makes every |LLR| 127) stays wrong, and the frame does not converge
# (BCH then finds no error: only parity bits are wrong). At 61 every variable
# can be corrected; at the waterfall |LLR| > 61 is rare (mean 8 Es/N0).
LCH = 61


def sat(a, w):
    lim = 1 << (w - 1)
    return np.clip(a, -lim, lim - 1)


OFFSET = 1           # offset min-sum: |R| = max(min - 1 LSB, 0)
PAD_G = 255          # column group of an absent (padding) edge in the RTL ROM


def norm(mag):
    """Offset min-sum correction of the check-to-variable magnitude."""
    return np.maximum(mag - OFFSET, 0)


class Code:
    """Layer structure of one code rate."""

    def __init__(self, rate_idx, short=False):
        L = (m.LDPC_SHORT if short else m.LDPC)[m.RATES[rate_idx]]
        self.N = m.nldpc(short)
        self.q = q = L["q"]
        self.M = L["nparity"]
        self.K = L["kldpc"]
        self.kbch = L["kbch"]
        self.ng = self.K // 360                  # info column groups
        layers = [[] for _ in range(q)]
        for g, row in enumerate(L["table"]):
            for x in row:
                layers[x % q].append((g, x // q, False))
        # pad the info edges to one degree (irregular short codes); the two
        # parity edges come last (the RTL generates them, they are not in
        # its edge ROM)
        dinfo = max(len(lay) for lay in layers)
        for lay in layers:
            lay.extend([(PAD_G, 0, "pad")] * (dinfo - len(lay)))
        for j in range(q):
            if j == 0:
                layers[j].append((self.ng + q - 1, 1, True))   # p_(c-1), absent at t = 0
            else:
                layers[j].append((self.ng + j - 1, 0, False))
            layers[j].append((self.ng + j, 0, False))
        self.layers = layers
        self.dup = [len({e[0] for e in lay if e[2] != "pad"}) != len([e for e in lay if e[2] != "pad"])
                    for lay in layers]

    def perm(self):
        """Codeword position of (column group, m) for all groups: (ng+q, 360)."""
        g = np.arange(self.ng)[:, None] * 360 + np.arange(360)[None, :]
        jj = np.arange(self.q)[:, None] + self.q * np.arange(360)[None, :]
        return np.vstack([g, self.K + jj])


class Decoder:
    def __init__(self, rate_idx, short=False):
        self.code = Code(rate_idx, short)
        c = self.code
        self.pos = c.perm()

    def decode(self, llr, max_it=MAX_IT):
        """llr: Nldpc int8 (codeword order). Returns (hard bits, iterations,
        converged)."""
        c = self.code
        L = np.clip(np.asarray(llr, np.int64)[self.pos], -LCH, LCH)   # (groups, 360)
        q = c.q
        # compressed check state per layer: arrays over t (360)
        m1 = [np.zeros(360, np.int64) for _ in range(q)]
        m2 = [np.zeros(360, np.int64) for _ in range(q)]
        ix = [np.zeros(360, np.int64) for _ in range(q)]
        st = [np.zeros(360, np.int64) for _ in range(q)]
        sg = [np.zeros((len(c.layers[j]), 360), np.int64) for j in range(q)]
        it = 0
        conv = False
        while it < max_it:
            it += 1
            unsat = 0
            flips = 0
            for j in range(q):
                lay = c.layers[j]
                if c.dup[j]:
                    steps = [b + S * np.arange(P) for b in range(S)]
                else:
                    steps = [np.arange(360)]
                for T in steps:
                    u, f = self._substep(L, lay, T, m1[j], m2[j], ix[j], st[j], sg[j])
                    unsat += u
                    flips += f
            if unsat == 0 and flips == 0:
                conv = True
                break
        hard = np.zeros(c.N, np.uint8)
        hard[self.pos.ravel()] = (L.ravel() < 0).astype(np.uint8)
        return hard, it, conv

    @staticmethod
    def _substep(L, lay, T, m1, m2, ix, st, sg):
        d = len(lay)
        n = len(T)
        Q = np.empty((d, n), np.int64)
        Ro = np.empty((d, n), np.int64)
        hs = np.zeros(n, np.int64)
        valid = np.ones((d, n), bool)
        idx = []
        for e, (G, t0, special) in enumerate(lay):
            mm = (T - t0) % 360
            idx.append(mm)
            if special == "pad":
                valid[e] = False
                lv = np.zeros(n, np.int64)
            else:
                if special:
                    valid[e] = T != 0
                lv = L[G, mm]
            mag = norm(np.where(ix[T] == e, m2[T], m1[T]))
            ro = np.where((st[T] ^ sg[e, T]) == 1, -mag, mag)
            Ro[e] = ro
            Q[e] = sat(lv - ro, WQ)
            hs ^= np.where(valid[e], (lv < 0).astype(np.int64), 0)
        sgn = (Q < 0).astype(np.int64)
        mag = np.minimum(np.abs(Q), RMAX)
        mag = np.where(valid, mag, RMAX + 1)              # absent edge: never the minimum
        sgn = np.where(valid, sgn, 0)
        i1 = np.argmin(mag, axis=0)                        # first minimum
        a1 = mag[i1, np.arange(n)]
        mag2 = mag.copy()
        mag2[i1, np.arange(n)] = RMAX + 1
        a2 = np.minimum(mag2.min(axis=0), RMAX)
        stot = np.bitwise_xor.reduce(sgn, axis=0)
        m1[T] = a1
        m2[T] = a2
        ix[T] = i1
        st[T] = stot
        flips = 0
        for e, (G, t0, special) in enumerate(lay):
            sg[e, T] = sgn[e]
            mm = idx[e]
            magn = norm(np.where(i1 == e, a2, a1))
            rn = np.where((stot ^ sgn[e]) == 1, -magn, magn)
            v = valid[e]
            if special == "pad":
                continue
            old = L[G, mm[v]]
            new = sat(old + rn[v] - Ro[e][v], WA)
            flips += int(np.count_nonzero((old < 0) != (new < 0)))
            L[G, mm[v]] = new
        return int(hs.sum()), flips


# ---------------------------------------------------------------------------
# tables for the RTL
# ---------------------------------------------------------------------------

def write_rtl_tables(gen_dir=None):
    """rtl/gen/ldpc_dec_rom.mem (the info edges of every layer of every code,
    padded: {k0[4:0], w0[4:0], G[7:0]}, t0 = k0*S + w0; normal rates 0..10,
    then short 0..9; the two parity edges of each layer, the last ones, are
    generated by the RTL: layer j to parity groups j - 1 (layer 0: q - 1 with
    shift 1, the special edge) and j) and
    rtl/gen/dvbs2_ldpc_dec_pkg.sv (per-code parameters, indexed {short,
    rate_idx[3:0]}; unused indices repeat the nearest code)."""
    import os
    gen_dir = gen_dir or os.path.join(m.HERE, "..", "rtl", "gen")
    rom, rows = [], {}
    for short in (False, True):
        for ri in range(10 if short else 11):
            c = Code(ri, short)
            base = len(rom)
            for lay in c.layers:
                for G, t0, special in lay[:-2]:
                    rom.append(((t0 // S) << 13) | ((t0 % S) << 8) | G)
            rows[16 * short + ri] = (base, c.q, c.ng, len(c.layers[0]), c.kbch, c.K, c.N)
    with open(os.path.join(gen_dir, "ldpc_dec_rom.mem"), "w") as f:
        for v in rom:
            f.write(f"{v:05x}\n")
    full = [rows[(i & 16) + min(i & 15, 9 if i & 16 else 10)] for i in range(32)]
    L = ["// Generated by model/ldpcdec.py write_rtl_tables(): LDPC decoder parameters,",
         "// indexed {short, rate_idx[3:0]} (unused indices repeat the nearest code).",
         "package dvbs2_ldpc_dec_pkg;",
         f"  localparam int LD_P = {P};",
         f"  localparam int LD_S = {S};",
         f"  localparam int LD_ROM_DEPTH = {len(rom)};",
         f"  localparam int LD_RMAX = {RMAX};",
         f"  localparam int LD_OFFSET = {OFFSET};",
         f"  localparam int LD_LCH = {LCH};"]
    for name, k in (("BASE", 0), ("Q", 1), ("NG", 2), ("DEG", 3), ("KBCH", 4), ("KLDPC", 5), ("N", 6)):
        L.append(f"  localparam logic [15:0] LD_{name} [32] = '{{" +
                 ", ".join(f"16'd{r[k]}" for r in full) + "};")
    L += ["endpackage", ""]
    with open(os.path.join(gen_dir, "dvbs2_ldpc_dec_pkg.sv"), "w") as f:
        f.write("\n".join(L))


# ---------------------------------------------------------------------------
# quick experiments
# ---------------------------------------------------------------------------

def awgn_llr(code_bits, esn0_db, rng):
    """QPSK (b0 -> I, b1 -> Q) over AWGN, LLRs quantised like the RX carrier block
    (int8, 4 LSBs per unit LLR)."""
    s = (1 - 2 * code_bits.astype(np.float64)) / np.sqrt(2)
    nvar = 10 ** (-esn0_db / 10)
    y = s + rng.standard_normal(len(s)) * np.sqrt(nvar / 2)
    llr = 2 * np.sqrt(2) * y / nvar * 4
    return np.clip(np.round(llr), -127, 127).astype(np.int8)


def compare(rate_idx, esn0_db, nframes, seed=0):
    import rxmodel as rf
    rng = np.random.default_rng(seed)
    kb = m.LDPC[m.RATES[rate_idx]]["kbch"]
    dec = Decoder(rate_idx)
    llrs, bbs = [], []
    ok_model = 0
    its = []
    for _ in range(nframes):
        bb = rng.integers(0, 2, kb).astype(np.uint8)
        cw = m.fec_frame(bb, rate_idx)
        llr = awgn_llr(cw, esn0_db, rng)
        hard, it, conv = dec.decode(llr)
        its.append(it)
        ok_model += int(np.array_equal(hard, cw))
        llrs.append(llr)
        bbs.append(bb)
    out = rf.gr_ldpc_decode_llr(np.array(llrs), rate_idx)
    ok_gr = 0
    for k, bb in enumerate(bbs):
        # gr output is descrambled BBFRAME; compare against the BB-scrambled input
        ok_gr += int(np.array_equal(out[k * kb:(k + 1) * kb], bb))
    print(f"{m.RATES[rate_idx]} Es/N0 {esn0_db} dB: model {ok_model}/{nframes} frames "
          f"(mean {np.mean(its):.1f} it), gr-dvbs2rx {ok_gr}/{nframes}")
    return ok_model, ok_gr


if __name__ == "__main__":
    write_rtl_tables()
