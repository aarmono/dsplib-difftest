#!/usr/bin/env python3
"""blobcov.py - dynamic machine-code coverage of REAL dsplibs.o functions under an existing test harness.

Runs the (32-bit, PIE) *blob* test binary of a chunk under valgrind/callgrind, one process per scenario, with
--toggle-collect=<symbol> so only the function(s) under test (and their callees) are collected, then merges the
per-instruction execution counts and the per-conditional-jump taken/executed counts across scenarios and prints,
for every function, which blob instructions / branch edges the harness NEVER exercised, with the disassembly
(objdump -d -M intel) annotated with function-relative offsets (and the matching dsplibs.o section offset).

  blobcov.py run    --binary test/.build/v34/v34_receiver_test_blob --scenarios "0 1 2 3" --func receiver --out DIR
  blobcov.py report --binary test/.build/v34/v34_receiver_test_blob --func receiver --dir DIR [--json out.json]
  blobcov.py run ... --no-report      (only collect; report later, possibly with a different --func subset)

See tools/README.md.  Standard library only (python3 >= 3.8), needs valgrind + objdump + nm.
"""
import argparse, collections, concurrent.futures, glob, json, os, re, shutil, signal, struct, subprocess, sys, time
from blobpath import BLOB_O   # noqa: E402  (the blob oracle's location: test/blob.sh)

# --demangle=no: a C++ symbol is matched by its MANGLED name (--toggle-collect=_ZN5QueueIfE4readEPfj); C names are unaffected
CG_FLAGS = ["--tool=callgrind", "--demangle=no", "--dump-instr=yes", "--collect-jumps=yes", "--compress-pos=no",
            "--compress-strings=no", "--collect-atstart=no"]


def sh(cmd, **kw):
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, **kw)


# --------------------------------------------------------------------------------------------- symbols / disasm
def read_symbols(path):
    """nm -S: list of dicts name, addr, size, type (only sized FUNC-ish text symbols t/T/w)."""
    out = []
    r = sh(["nm", "-S", "--defined-only", path])
    for ln in r.stdout.splitlines():
        p = ln.split()
        if len(p) == 4 and p[2] in "tTwW":
            out.append(dict(name=p[3], addr=int(p[0], 16), size=int(p[1], 16), type=p[2]))
    return out


def resolve_func(symbols, name=None, addr=None, size=None):
    if addr is not None:
        for s in symbols:
            if s["addr"] == addr:
                return dict(name=name or s["name"], addr=addr, size=size or s["size"], type=s["type"], sym=s["name"])
        if size is None:
            sys.exit("--addr %#x matches no sized text symbol; give --size" % addr)
        cont = [s for s in symbols if s["addr"] <= addr < s["addr"] + s["size"]]
        return dict(name=name or "sub_%x" % addr, addr=addr, size=size, type="?",
                    sym=cont[0]["name"] if cont else None)
    c = [s for s in symbols if s["name"] == name]
    if not c:
        sys.exit("symbol %r not found in binary (nm -S); use --addr/--size" % name)
    if len(c) > 1:
        sys.exit("symbol %r is ambiguous (%s); use --addr" % (name, ", ".join("%#x" % s["addr"] for s in c)))
    s = c[0]
    if size:
        s = dict(s, size=size)
    return dict(name=name, addr=s["addr"], size=s["size"], type=s["type"], sym=name)


INSN_RE = re.compile(r"^\s*([0-9a-f]+):\t((?:[0-9a-f]{2} )+)\s*\t?(.*)$")


def disassemble(path, start, end):
    """-> list of dict(addr,size,text,mn,ops,bytes).  Uses objdump -d -w -M intel on [start,end)."""
    r = sh(["objdump", "-d", "-w", "-M", "intel", "--start-address=%#x" % start, "--stop-address=%#x" % end, path])
    insns = []
    for ln in r.stdout.splitlines():
        m = INSN_RE.match(ln)
        if not m:
            m2 = re.match(r"^\s*([0-9a-f]+):\t((?:[0-9a-f]{2} ?)+)\s*$", ln)   # continuation with no text
            if m2 and insns:
                insns[-1]["size"] += len(m2.group(2).split())
            continue
        a = int(m.group(1), 16)
        nb = len(m.group(2).split())
        text = m.group(3).strip()
        text = re.sub(r"\s*#.*$", "", text)
        mn, _, ops = text.partition(" ")
        if mn in ("rep", "repz", "repnz", "repe", "repne", "lock", "data16", "cs", "ds", "es", "ss", "bnd"):
            parts = text.split(None, 2)
            if len(parts) > 1:
                mn = parts[0] + " " + parts[1]
                ops = parts[2] if len(parts) > 2 else ""
        insns.append(dict(addr=a, size=nb, text=text, mn=mn, ops=ops.strip()))
    return insns


def is_jcc(mn):
    return mn.startswith("j") and mn not in ("jmp",) or mn in ("loop", "loope", "loopne", "loopz", "loopnz")


def is_uncond_end(mn):
    return mn in ("jmp", "ret", "retn", "ud2", "hlt") or mn.startswith("ret")


def direct_target(ops):
    m = re.match(r"^([0-9a-f]+)(?:\s|$)", ops)
    return int(m.group(1), 16) if m else None


PAD_RE = re.compile(r"^(nop|xchg\s+ax,ax|lea\s+(\w+),\[\2(\+eiz\*\d)?\+0x0\]|mov\s+(\w+),\4|\.byte 0x90|data16.*|cs nop.*)", re.I)


def is_padding(ins):
    t = ins["text"]
    return bool(PAD_RE.match(t)) or t.startswith("nop") or t.startswith("xchg   ax,ax") or \
        re.match(r"^lea\s+\w+,\[\w+(\+eiz\*\d)?(\+0x0)?\]$", t) is not None or \
        re.match(r"^mov\s+(\w+),\1$", t) is not None or ins["mn"].startswith("nop")


# --------------------------------------------------------------------------------------------- ELF helpers
def elf_sections(path):
    r = sh(["readelf", "-SW", path])
    secs = []
    for ln in r.stdout.splitlines():
        m = re.match(r"^\s*\[\s*\d+\]\s+(\S+)\s+\S+\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)", ln)
        if m:
            secs.append((m.group(1), int(m.group(2), 16), int(m.group(3), 16), int(m.group(4), 16)))
    return secs


class Elf:
    def __init__(self, path):
        self.path = path
        self.secs = elf_sections(path)
        self.fh = open(path, "rb")

    def read(self, vaddr, n):
        for nm, a, off, sz in self.secs:
            if a and a <= vaddr and vaddr + n <= a + sz:
                self.fh.seek(off + vaddr - a)
                return self.fh.read(n)
        return None


# --------------------------------------------------------------------------------------------- callgrind parsing
class Prof:
    """Counts of ONE callgrind run (all its part files), restricted to the wanted object."""

    def __init__(self):
        self.ir = collections.Counter()          # addr -> executions
        self.taken = collections.Counter()       # (src, tgt) -> taken count   (jcnd + jump)
        self.jexec = collections.Counter()       # (src, tgt) -> "executed" as reported on jcnd lines
        self.total = 0

    def merge(self, o):
        self.ir.update(o.ir)
        self.taken.update(o.taken)
        self.jexec.update(o.jexec)
        self.total += o.total


def parse_callgrind(files, obj_real):
    p = Prof()
    obj_base = os.path.basename(obj_real)
    for fn in files:
        try:
            fh = open(fn, errors="replace")
        except OSError:
            continue
        cur_ob = None
        skip_next = False          # the cost line following calls=
        pend = None                # (kind, tgt, taken, exec) waiting for the source-position line
        with fh:
            for ln in fh:
                if not ln or ln[0] == "#":
                    continue
                c = ln[0]
                if c == "0":        # cost or position line: "0xaddr line [cost]"
                    f = ln.split()
                    if pend is not None:
                        if cur_ob:
                            src = int(f[0], 16)
                            kind, tgt, tk, ex = pend
                            p.taken[(src, tgt)] += tk
                            if kind == "c":
                                p.jexec[(src, tgt)] += ex
                        pend = None
                        continue
                    if skip_next:
                        skip_next = False
                        continue
                    if cur_ob and len(f) >= 3:
                        a = int(f[0], 16)
                        n = int(f[2])
                        p.ir[a] += n
                        p.total += n
                    continue
                if ln.startswith("ob="):
                    name = ln[3:].strip()
                    name = re.sub(r"^\(\d+\)\s*", "", name)
                    cur_ob = os.path.basename(name) == obj_base and \
                        os.path.realpath(name) == obj_real or (os.path.basename(name) == obj_base and not os.path.exists(name))
                elif ln.startswith("calls="):
                    skip_next = True
                elif ln.startswith("jcnd="):
                    m = re.match(r"jcnd=(\d+)/(\d+)\s+(0x[0-9a-f]+)", ln)
                    if m:
                        pend = ("c", int(m.group(3), 16), int(m.group(1)), int(m.group(2)))
                elif ln.startswith("jump="):
                    m = re.match(r"jump=(\d+)\s+(0x[0-9a-f]+)", ln)
                    if m:
                        pend = ("j", int(m.group(2), 16), int(m.group(1)), 0)
                # fn=, fl=, fi=, fe=, cfn=, cfi=, cob=, events:, etc: ignored
    return p


# --------------------------------------------------------------------------------------------- analysis
def analyse(func, insns, prof, elf, per_scen=None, ctx=6):
    lo, hi = func["addr"], func["addr"] + func["size"]
    byaddr = {i["addr"]: k for k, i in enumerate(insns)}
    n = len(insns)
    ir = [prof.ir.get(i["addr"], 0) for i in insns]
    for k, i in enumerate(insns):
        i["ir"] = ir[k]
        i["off"] = i["addr"] - lo
    pad = [is_padding(i) for i in insns]
    # ---- indirect jmp tables
    tables = []
    for k, i in enumerate(insns):
        if i["mn"] == "jmp" and "[" in i["ops"]:
            m = re.search(r"\[(?:(\w+)\*4\+|\w+\+\w+\*4\+)?(0x[0-9a-f]+)(?:\+\w+\*4)?\]", i["ops"])
            if not m:
                continue
            base = int(m.group(2), 16)
            cnt = None
            for j in range(k - 1, max(k - 8, -1), -1):       # cmp reg,N ; ja default
                mm = re.match(r"cmp\s+\w+,(0x[0-9a-f]+|\d+)$", insns[j]["text"])
                if mm and any(insns[q]["mn"] in ("ja", "jae", "jg", "jb") for q in range(j + 1, k)):
                    cnt = int(mm.group(1), 0) + 1
                    break
            ent = []
            for e in range(cnt or 256):
                b = elf.read(base + 4 * e, 4)
                if b is None:
                    break
                v = struct.unpack("<I", b)[0]
                if not (lo <= v < hi) or v not in byaddr:
                    break
                ent.append(v)
            if ent:
                tables.append(dict(at=i["off"], base=base, targets=ent, bounded=cnt is not None))
    # ---- leaders, conditional jumps
    cond = []
    for k, i in enumerate(insns):
        if is_jcc(i["mn"]):
            t = direct_target(i["ops"])
            fall = insns[k + 1]["addr"] if k + 1 < n else None
            taken = prof.taken.get((i["addr"], t), 0) if t is not None else 0
            cond.append(dict(idx=k, off=i["off"], text=i["text"], tgt=t, fall=fall, ir=i["ir"], taken=taken,
                             nt=i["ir"] - taken, tgt_in=(t is not None and lo <= t < hi)))
    leaders = {lo}
    for c in cond:
        if c["tgt_in"]:
            leaders.add(c["tgt"])
        if c["fall"]:
            leaders.add(c["fall"])
    for k, i in enumerate(insns):
        if i["mn"] == "jmp":
            t = direct_target(i["ops"])
            if t is not None and lo <= t < hi:
                leaders.add(t)
            if k + 1 < n:
                leaders.add(insns[k + 1]["addr"])
        elif is_uncond_end(i["mn"]) and k + 1 < n:
            leaders.add(insns[k + 1]["addr"])
    for t in tables:
        leaders.update(t["targets"])
    # ---- unreached ranges (maximal runs ir==0, then split by padding / non-padding)
    ranges = []
    k = 0
    while k < n:
        if ir[k] == 0:
            j = k
            kind = pad[k]
            while j < n and ir[j] == 0 and pad[j] == kind:
                j += 1
            ranges.append(dict(a=k, b=j, pad=kind))
            k = j
        else:
            k += 1
    # incoming "never exercised" edges into code: map target addr -> list of descriptors
    entries = collections.defaultdict(list)
    for c in cond:
        if c["ir"] > 0:
            if c["taken"] == 0 and c["tgt_in"]:
                entries[c["tgt"]].append("%s@+%#x NOT TAKEN (executed %d x)" % (c["text"].split()[0], c["off"], c["ir"]))
            if c["nt"] == 0 and c["fall"]:
                entries[c["fall"]].append("%s@+%#x ALWAYS TAKEN (%d x); fall-through never" % (c["text"].split()[0], c["off"], c["ir"]))
    for t in tables:
        jk = byaddr.get(lo + t["at"])
        for ci, tg in enumerate(t["targets"]):
            if insns[byaddr[tg]]["ir"] == 0:
                entries[tg].append("switch case %d of jmp@+%#x (table %#x)" % (ci, t["at"], t["base"]))
    calls_ = lambda a, b: [ins["text"].split(None, 1)[1] for ins in insns[a:b] if ins["mn"] == "call"]
    for r in ranges:
        a, b = r["a"], r["b"]
        r["off"] = insns[a]["off"]
        r["end"] = insns[b - 1]["off"] + insns[b - 1]["size"]
        r["bytes"] = r["end"] - r["off"]
        r["n"] = b - a
        ent = []
        for q in range(a, b):
            ent += entries.get(insns[q]["addr"], [])
        if a > 0 and ir[a - 1] > 0 and not is_uncond_end(insns[a - 1]["mn"]) and not is_jcc(insns[a - 1]["mn"]) and not r["pad"]:
            ent.append("falls in from executed +%#x (%s) - call that never returned?" % (insns[a - 1]["off"], insns[a - 1]["text"]))
        r["entries"] = ent
        r["calls"] = calls_(a, b)
        notes = []
        for q in range(a, b):
            mn = insns[q]["mn"]
            if mn in ("div", "idiv"):
                notes.append("%s@+%#x (possible #DE trap)" % (mn, insns[q]["off"]))
            if mn in ("ud2", "hlt", "int3", "int"):
                notes.append("%s@+%#x" % (mn, insns[q]["off"]))
            if mn.split()[0] in ("rep", "repz", "repnz", "repe", "repne") or mn in ("movsb", "movsw", "movsd", "stosb", "stosw", "stosd", "scasb", "cmpsb", "lodsb"):
                notes.append("string op %s@+%#x" % (mn, insns[q]["off"]))
        r["notes"] = notes
        r["asm"] = [(insns[q]["off"], insns[q]["addr"], insns[q]["text"]) for q in range(a, min(b, a + ctx))]
        r["asm_all"] = [(insns[q]["off"], insns[q]["addr"], insns[q]["text"]) for q in range(a, b)]
        r["last"] = (insns[b - 1]["off"], insns[b - 1]["text"])
    # ---- edges (never exercised) inside reached code
    edges = []
    for c in cond:
        if c["ir"] == 0:
            continue
        for kind, cnt, dest in (("taken", c["taken"], c["tgt"]), ("fall-through", c["nt"], c["fall"])):
            if cnt != 0 or dest is None:
                continue
            dk = byaddr.get(dest)
            dr = insns[dk]["ir"] if dk is not None else None
            dest_in = dk is not None
            edges.append(dict(off=c["off"], jcc=c["text"], kind=kind, executed=c["ir"],
                              dest_off=(dest - lo) if dest_in else None, dest=dest,
                              dest_ir=dr, dest_text=insns[dk]["text"] if dest_in else "(outside function)",
                              dest_asm=[(insns[q]["off"], insns[q]["text"]) for q in range(dk, min(n, dk + ctx))] if dest_in else [],
                              dest_unreached=(dr == 0) if dest_in else None))
    # ---- hot blocks
    blocks = []
    starts = sorted(a for a in leaders if a in byaddr)
    for bi, a in enumerate(starts):
        k0 = byaddr[a]
        k1 = byaddr[starts[bi + 1]] if bi + 1 < len(starts) else n
        if ir[k0] > 0:
            blocks.append(dict(off=a - lo, n=k1 - k0, count=ir[k0], weight=ir[k0] * (k1 - k0), first=insns[k0]["text"],
                               last=insns[k1 - 1]["text"]))
    blocks.sort(key=lambda b: -b["weight"])
    # ---- summary
    nb = sum(i["size"] for i in insns)
    ex = [k for k in range(n) if ir[k] > 0]
    padn = sum(1 for k in range(n) if pad[k] and ir[k] == 0)
    padb = sum(insns[k]["size"] for k in range(n) if pad[k] and ir[k] == 0)
    real_n = n - padn
    real_b = nb - padb
    ex_b = sum(insns[k]["size"] for k in ex)
    ncond = len(cond)
    ex_edges = 0
    for c in cond:
        if c["ir"] > 0:
            ex_edges += (1 if c["taken"] > 0 else 0) + (1 if c["nt"] > 0 else 0)
    per = {}
    if per_scen:
        allset = set(insns[k]["addr"] for k in ex)
        for s, pr in per_scen.items():
            mine = set(insns[k]["addr"] for k in range(n) if pr.ir.get(insns[k]["addr"], 0) > 0)
            others = set()
            for s2, pr2 in per_scen.items():
                if s2 != s:
                    others |= set(insns[k]["addr"] for k in range(n) if pr2.ir.get(insns[k]["addr"], 0) > 0)
            ed = 0
            for c in cond:
                if pr.taken.get((insns[c["idx"]]["addr"], c["tgt"]), 0) > 0 and c["tgt"] is not None:
                    ed += 1
            per[s] = dict(insns=len(mine), only_here=len(mine - others), calls=pr.ir.get(lo, 0), taken_edges=ed)
    return dict(name=func["name"], addr=lo, size=func["size"], n_insns=n, bytes=nb, executed_insns=len(ex),
                executed_bytes=ex_b, padding_unreached_insns=padn, padding_unreached_bytes=padb,
                pct_insns=100.0 * len(ex) / max(1, n), pct_insns_nopad=100.0 * len(ex) / max(1, real_n),
                pct_bytes_nopad=100.0 * ex_b / max(1, real_b),
                calls=ir[0] if ir else 0, cond_branches=ncond, edges_total=2 * ncond, edges_exercised=ex_edges,
                unreached_ranges=ranges, unexercised_edges=edges, switch_tables=tables,
                hot_blocks=blocks[:10], per_scenario=per, insns=insns)


# --------------------------------------------------------------------------------------------- report
def fmt_asm(lst, indent="      "):
    return "\n".join("%s+%#06x  %s" % (indent, o, t) for o, t in lst)


def print_report(res, args, dso_off=None):
    w = sys.stdout.write
    w("=" * 100 + "\n")
    w("FUNCTION %s   blob addr %#x (+0 == .o section offset %s)   size %d B   %d instructions\n" %
      (res["name"], res["addr"], ("%#x" % dso_off) if dso_off is not None else "?", res["size"], res["n_insns"]))
    w("  calls (entry executions):            %d\n" % res["calls"])
    w("  instructions executed:               %d / %d = %.1f%%   (excluding %d unreached alignment-pad insns/%d B: %.1f%% insns, %.1f%% bytes)\n" %
      (res["executed_insns"], res["n_insns"], res["pct_insns"], res["padding_unreached_insns"],
       res["padding_unreached_bytes"], res["pct_insns_nopad"], res["pct_bytes_nopad"]))
    w("  conditional branches: %d -> %d edges; exercised edges: %d (edges inside unreached code count as unexercised)\n" %
      (res["cond_branches"], res["edges_total"], res["edges_exercised"]))
    ue = res["unexercised_edges"]
    w("  never-exercised edges of REACHED branches: %d  (dest block itself unreached: %d;  dest reached by another path: %d)\n" %
      (len(ue), sum(1 for e in ue if e["dest_unreached"]), sum(1 for e in ue if e["dest_unreached"] is False)))
    real = [r for r in res["unreached_ranges"] if not r["pad"]]
    w("  unreached code ranges: %d (%d insns, %d B) + %d pure-padding runs\n" %
      (len(real), sum(r["n"] for r in real), sum(r["bytes"] for r in real), sum(1 for r in res["unreached_ranges"] if r["pad"])))
    if res["per_scenario"]:
        w("  per-scenario: " + "; ".join("s%s: %d insns (%d only here)" % (s, d["insns"], d["only_here"])
                                        for s, d in sorted(res["per_scenario"].items(), key=lambda kv: kv[0])) + "\n")
    w("\n-- A. NEVER-EXERCISED EDGES (branch executed, one direction never seen) --\n")
    if not ue:
        w("   (none)\n")
    for e in sorted(ue, key=lambda e: e["off"]):
        w("  [%s] +%#06x  %-34s  %s NEVER  (branch ran %d x)\n" %
          ("BLOCK" if e["dest_unreached"] else ("EDGE " if e["dest_unreached"] is False else "?    "),
           e["off"], e["jcc"], e["kind"], e["executed"]))
        if e["dest_off"] is not None:
            w("      -> +%#06x %s; dest block executed %d x\n" % (e["dest_off"], "(UNREACHED)" if e["dest_unreached"] else "", e["dest_ir"]))
            if not args.brief:
                w(fmt_asm(e["dest_asm"][:args.ctx]) + "\n")
    w("\n-- B. UNREACHED CODE RANGES (function-relative; grouped contiguous runs, execution count 0) --\n")
    for r in res["unreached_ranges"]:
        tag = "PAD  " if r["pad"] else "CODE "
        if r["pad"] and not args.show_pad:
            continue
        w("  %s +%#06x..+%#06x  %4d B  %3d insns   entered by: %s\n" %
          (tag, r["off"], r["end"], r["bytes"], r["n"], "; ".join(r["entries"]) if r["entries"] else "(nothing reachable: only from other unreached code / indirect)"))
        if r["calls"]:
            w("        calls: %s\n" % ", ".join(r["calls"]))
        if r["notes"]:
            w("        notes: %s\n" % ", ".join(r["notes"]))
        if not r["pad"] and not args.brief:
            lst = r["asm_all"] if args.full else r["asm"]
            w("\n".join("        +%#06x  %s" % (o, t) for o, a, t in lst) + "\n")
            if len(lst) < r["n"]:
                w("        ... (%d more, last: +%#06x %s)\n" % (r["n"] - len(lst), r["last"][0], r["last"][1]))
    if res["switch_tables"]:
        w("\n-- C. SWITCH TABLES --\n")
        for t in res["switch_tables"]:
            hit = [ci for ci, tg in enumerate(t["targets"])]
            w("  jmp@+%#x table %#x: %d entries%s\n" % (t["at"], t["base"], len(t["targets"]), "" if t["bounded"] else " (unbounded scan)"))
    w("\n-- D. HOTTEST BLOCKS (by executions x length) --\n")
    for b in res["hot_blocks"][:args.hot]:
        w("  +%#06x  x%-10d %3d insns   %s ... %s\n" % (b["off"], b["count"], b["n"], b["first"], b["last"]))
    w("\n")


# --------------------------------------------------------------------------------------------- runner
def run_scenarios(a):
    binary = os.path.abspath(a.binary)
    if not os.path.exists(binary):
        sys.exit("binary not found: " + binary)
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    syms = read_symbols(binary)
    funcs = [resolve_func(syms, name=f) for f in (a.func or [])]
    for ad in (a.addr or []):
        funcs.append(resolve_func(syms, addr=int(ad, 0), size=a.size))
    toggles = []
    for f in funcs:
        sym = f.get("sym") or f["name"]
        if not any(s["name"] == sym for s in syms):
            sys.exit("no symbol to toggle-collect on for %s; use --collect-all" % f["name"])
        toggles.append(sym)
    scen = a.scenarios.split()
    # resume: skip scenarios that already have an 'ok' meta entry
    meta_path = os.path.join(out, "meta.json")
    meta = json.load(open(meta_path)) if os.path.exists(meta_path) else {}
    meta.setdefault("runs", {})
    meta["binary"] = binary
    todo = [s for s in scen if not (meta["runs"].get(s, {}).get("status") in ("ok", "partial") and a.resume)]
    if not todo:
        print("nothing to run (all scenarios already collected; drop --resume to redo)", file=sys.stderr)
    env = dict(os.environ)
    for kv in a.env or []:
        k, _, v = kv.partition("=")
        env[k] = v

    def one(s):
        for old in glob.glob(os.path.join(out, "cg.s%s.*" % s)):
            os.remove(old)
        cmd = ["valgrind", "--error-exitcode=0", "-q"] + CG_FLAGS + (["--trace-children=yes"] if a.trace_children else [])
        if a.collect_all:
            cmd[cmd.index("--collect-atstart=no")] = "--collect-atstart=yes"
        for t in toggles:
            cmd.append("--toggle-collect=" + t)
        cmd += ["--callgrind-out-file=%s/cg.s%s.%%p" % (out, s), binary] + [x.replace("{s}", s) for x in a.args_template.split()]
        t0 = time.time()
        log = open(os.path.join(out, "run.s%s.log" % s), "w")
        pr = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=env, cwd=a.cwd or os.path.dirname(binary),
                              stdin=subprocess.DEVNULL, start_new_session=True)
        status = "ok"
        try:
            rc = pr.wait(timeout=a.timeout or None)
        except subprocess.TimeoutExpired:
            status = "partial"    # dump what we have, then stop
            sh(["callgrind_control", "-d", str(pr.pid)])
            time.sleep(4)
            try:
                os.killpg(pr.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            pr.wait()
            rc = None
        log.close()
        return s, dict(status=status, rc=rc, secs=round(time.time() - t0, 1), cmd=" ".join(cmd))

    t_all = time.time()
    with concurrent.futures.ThreadPoolExecutor(max_workers=a.jobs) as ex:
        futs = [ex.submit(one, s) for s in todo]
        try:
            for f in concurrent.futures.as_completed(futs):
                s, m = f.result()
                meta["runs"][s] = m
                json.dump(meta, open(meta_path, "w"), indent=1)
                print("scenario %s: %s rc=%s %.0fs" % (s, m["status"], m["rc"], m["secs"]), file=sys.stderr)
        except KeyboardInterrupt:
            for f in futs:
                f.cancel()
            raise
    meta["funcs"] = [f["name"] for f in funcs]
    json.dump(meta, open(meta_path, "w"), indent=1)
    print("collected in %.0fs into %s" % (time.time() - t_all, out), file=sys.stderr)
    return out


def do_report(a, outdir):
    binary = os.path.abspath(a.binary)
    obj_real = os.path.realpath(binary)
    syms = read_symbols(binary)
    funcs = [resolve_func(syms, name=f) for f in (a.func or [])]
    for ad in (a.addr or []):
        funcs.append(resolve_func(syms, addr=int(ad, 0), size=a.size))
    if not funcs:
        sys.exit("no --func / --addr given")
    meta = json.load(open(os.path.join(outdir, "meta.json"))) if os.path.exists(os.path.join(outdir, "meta.json")) else {}
    per_scen = {}
    for f in sorted(glob.glob(os.path.join(outdir, "cg.s*"))):
        m = re.match(r".*/cg\.s([^.]+)\.", f)
        if m:
            per_scen.setdefault(m.group(1), []).append(f)
    profs = {s: parse_callgrind(fs, obj_real) for s, fs in per_scen.items()}
    if not profs:
        sys.exit("no callgrind output files in " + outdir)
    total = Prof()
    for p in profs.values():
        total.merge(p)
    elf = Elf(binary)
    blob_o = a.blob_obj or BLOB_O
    o_syms = {}
    if os.path.exists(blob_o):
        r = sh(["readelf", "-sW", blob_o])
        for ln in r.stdout.splitlines():
            p = ln.split()
            if len(p) >= 8 and p[3] == "FUNC":
                o_syms.setdefault(p[7], []).append((int(p[1], 16), int(p[2])))
    results = []
    bad = {s: m for s, m in meta.get("runs", {}).items() if m.get("status") != "ok" or m.get("rc") not in (0,)}
    if bad:
        sys.stderr.write("NOTE: scenarios with non-ok status/exit code (coverage of these is a LOWER bound or the scenario "
                         "crashed/trapped by design): %s\n" % ", ".join("s%s=%s/rc%s" % (s, m.get("status"), m.get("rc")) for s, m in sorted(bad.items())))
    print("blobcov: %s   scenarios collected: %s" % (binary, " ".join("s" + s for s in sorted(profs, key=lambda x: (len(x), x)))))
    for f in funcs:
        insns = disassemble(binary, f["addr"], f["addr"] + f["size"])
        if not insns:
            sys.exit("objdump produced nothing for %s" % f["name"])
        for i in insns:
            pass
        res = analyse(f, insns, total, elf, per_scen=profs, ctx=a.ctx)
        ooff = None
        for (v, sz) in o_syms.get(f.get("sym") or f["name"], []):
            if sz == f["size"]:
                ooff = v
        res["blob_o_offset"] = ooff
        print_report(res, a, ooff)
        results.append(res)
    if a.json:
        jr = []
        for r in results:
            d = dict(r)
            insns = d.pop("insns")
            if a.json_insns:
                d["insns"] = [dict(off=i["off"], size=i["size"], text=i["text"], ir=i["ir"]) for i in insns]
            for rg in d["unreached_ranges"]:
                rg.pop("asm", None)
            jr.append(d)
        json.dump(dict(binary=binary, scenarios=sorted(profs), run_meta=meta.get("runs", {}), functions=jr), open(a.json, "w"), indent=1)
        print("json written: " + a.json)
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for nm in ("run", "report"):
        p = sub.add_parser(nm)
        p.add_argument("--binary", required=True, help="blob test binary (e.g. test/.build/v34/v34_receiver_test_blob)")
        p.add_argument("--func", action="append", help="symbol name (repeatable)")
        p.add_argument("--addr", action="append", help="linked-binary address of a function (nm address, hex) (repeatable)")
        p.add_argument("--size", type=lambda x: int(x, 0), help="byte size for --addr")
        p.add_argument("--blob-obj", help="dsplibs.o (for .o-relative offsets); default BLOB_O (test/blob.sh)")
        p.add_argument("--ctx", type=int, default=6, help="instructions of disassembly to print per unreached block")
        p.add_argument("--full", action="store_true", help="print every instruction of each unreached range")
        p.add_argument("--brief", action="store_true", help="omit disassembly listings")
        p.add_argument("--show-pad", action="store_true", help="also list pure-padding unreached runs")
        p.add_argument("--hot", type=int, default=6, help="number of hottest blocks to list")
        p.add_argument("--json", help="write machine-readable JSON here")
        p.add_argument("--json-insns", action="store_true", help="include the per-instruction table in the JSON")
    r = sub.choices["run"]
    r.add_argument("--scenarios", required=True, help='space separated scenario ids, passed as argv[1] ("{s}" in --args-template)')
    r.add_argument("--args-template", default="{s}", help="argv appended after the binary; default '{s}'")
    r.add_argument("--out", required=True, help="directory for callgrind files (put it under a scratch dir; can be large)")
    r.add_argument("--jobs", type=int, default=6, help="parallel scenarios")
    r.add_argument("--timeout", type=float, default=0, help="seconds per scenario; on expiry dump partial data (status=partial, LOWER-bound coverage) and kill")
    r.add_argument("--env", action="append", help="K=V for the harness (repeatable)")
    r.add_argument("--cwd", help="working dir (default: binary's dir)")
    r.add_argument("--no-trace-children", dest="trace_children", action="store_false", default=True,
                   help="default traces children (scenarios that fork a probe child)")
    r.add_argument("--collect-all", action="store_true", help="collect everything (no toggle); only for --addr without a symbol; HUGE")
    r.add_argument("--resume", action="store_true", help="skip scenarios already collected in --out")
    r.add_argument("--no-report", action="store_true")
    q = sub.choices["report"]
    q.add_argument("--dir", required=True, help="directory with cg.s<scen>.* files")
    a = ap.parse_args()
    if a.cmd == "run":
        out = run_scenarios(a)
        if not a.no_report:
            do_report(a, out)
    else:
        do_report(a, a.dir)


if __name__ == "__main__":
    main()
