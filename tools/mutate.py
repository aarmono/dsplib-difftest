#!/usr/bin/env python3
"""mutate.py - shared, directed mutation-testing driver for the dsplibs.o port chunks.

A "mutant" is the port source with one small edit inside one of the named functions.  The chunk's differential
test (real blob vs port) must KILL it: the mutant port's stdout differs from the stored blob stdout of some
scenario, or the scenario exits non-zero (assertion, self-check, signal, timeout).  Survivors are either
equivalent mutants or harness weaknesses; this tool's job is to make that triage small and directed.

  tools/mutate.py list    ...   generate + count mutants only (no build, no run)
  tools/mutate.py run     ...   prepare (once) / generate / compile / TCE / run / write JSONL
  tools/mutate.py report  ...   survivor report from an out-dir's JSONL

Typical:
  tools/mutate.py run --chunk v34_tier5c --port src/v34/v34_detectors.c --port-env PORT_T5C \
        --func tone_detect --func dftupdate ... --out /tmp/m5c [--sample 200] [--gcov] [--jobs 6]
  tools/mutate.py run ... --survivors-of /tmp/m5c_old/results.jsonl --out /tmp/m5c_new     # extended harness

See tools/README.md section 4 for the method, the operator classes / tags, the equivalence pre-filter (TCE) and
the blob-directed SIGN / compare-signedness heuristics and their limits.  Python >= 3.8, stdlib only.
"""
import argparse, collections, concurrent.futures as cf, hashlib, json, os, random, re, shlex, shutil, signal, subprocess, sys, threading, time

HERE = os.path.dirname(os.path.abspath(__file__))
from blobpath import BLOB_O   # noqa: E402  (the blob oracle's location: test/blob.sh)
sys.path.insert(0, HERE)

# ====================================================================================================================
# 1. C lexer (just enough for mutation: comments/strings/preprocessor skipped, offsets + lines kept)
# ====================================================================================================================
PUNCT3 = ("<<=", ">>=", "...")
PUNCT2 = ("::", "->", "++", "--", "<<", ">>", "<=", ">=", "==", "!=", "&&", "||", "+=", "-=", "*=", "/=", "%=", "&=", "|=", "^=", "##")
ASSIGN_OPS = {"=", "+=", "-=", "*=", "/=", "%=", "&=", "|=", "^=", "<<=", ">>="}
REL_OPS = {"<", "<=", ">", ">="}
EQ_OPS = {"==", "!="}
KEYWORDS = {"if", "else", "for", "while", "do", "switch", "case", "default", "return", "break", "continue", "goto", "sizeof",
            "typedef", "struct", "union", "enum"}
TYPE_KW = {"int", "char", "short", "long", "signed", "unsigned", "void", "float", "double", "_Bool"}
QUAL_KW = {"const", "volatile", "static", "inline", "extern", "register", "restrict", "__restrict", "__inline", "__inline__"}
STD_TYPES = {"int8_t", "uint8_t", "int16_t", "uint16_t", "int32_t", "uint32_t", "int64_t", "uint64_t", "intptr_t", "uintptr_t",
             "size_t", "ssize_t", "ptrdiff_t", "s8_t", "u8_t", "s16_t", "u16_t", "s32_t", "u32_t", "ptr_t", "long_double"}
SIGN_FLIP = {"int8_t": "uint8_t", "uint8_t": "int8_t", "int16_t": "uint16_t", "uint16_t": "int16_t",
             "int32_t": "uint32_t", "uint32_t": "int32_t", "int64_t": "uint64_t", "uint64_t": "int64_t",
             "intptr_t": "uintptr_t", "uintptr_t": "intptr_t", "s8_t": "u8_t", "u8_t": "s8_t", "s16_t": "u16_t", "u16_t": "s16_t",
             "s32_t": "u32_t", "u32_t": "s32_t", "size_t": "ptrdiff_t", "ptrdiff_t": "size_t"}
# multi-keyword type spellings -> sign-flipped spelling
SIGN_FLIP_KW = {"int": "unsigned", "signed": "unsigned", "signed int": "unsigned", "unsigned": "int", "unsigned int": "int",
                "short": "unsigned short", "short int": "unsigned short", "signed short": "unsigned short",
                "unsigned short": "short", "unsigned short int": "short", "char": "unsigned char", "unsigned char": "signed char",
                "signed char": "unsigned char", "long": "unsigned long", "unsigned long": "long"}
WIDTH_FLIP = {"int16_t": "int32_t", "uint16_t": "uint32_t", "int32_t": "int16_t", "uint32_t": "uint16_t",
              "int8_t": "int16_t", "uint8_t": "uint16_t", "short": "int", "unsigned short": "unsigned", "int": "short"}
PTRISH_TYPES = {"uintptr_t", "intptr_t", "size_t", "ptrdiff_t", "ssize_t"}
ACCESSOR_RE = re.compile(r"^([SU])(8|16|32)$|^PTR$")
ACC_SWAP_WIDTH = {("S", 16): "S32", ("U", 16): "U32", ("S", 32): "S16", ("U", 32): "U16", ("S", 8): "S16", ("U", 8): "U16"}


class Tok:
    __slots__ = ("kind", "text", "s", "e", "line", "i")

    def __init__(self, kind, text, s, e, line):
        self.kind, self.text, self.s, self.e, self.line, self.i = kind, text, s, e, line, -1

    def __repr__(self):
        return "Tok(%s %r @%d)" % (self.kind, self.text, self.line)


def lex(src):
    toks, n, i, line = [], len(src), 0, 1
    bol = True
    while i < n:
        c = src[i]
        if c == "\n":
            line += 1
            i += 1
            bol = True
            continue
        if c in " \t\r\f\v":
            i += 1
            continue
        if src.startswith("/*", i):
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
            line += src.count("\n", i, j)
            i = j
            continue
        if src.startswith("//", i):
            j = src.find("\n", i)
            i = n if j < 0 else j
            continue
        if c == "#" and bol:
            j = i
            while True:
                k = src.find("\n", j)
                if k < 0:
                    j = n
                    break
                if src[k - 1] == "\\" or (src[k - 1] == "\r" and k > 1 and src[k - 2] == "\\"):
                    j = k + 1
                    continue
                j = k
                break
            line += src.count("\n", i, j)
            i = j
            continue
        bol = False
        if c.isalpha() or c == "_":
            j = i + 1
            while j < n and (src[j].isalnum() or src[j] == "_"):
                j += 1
            toks.append(Tok("id", src[i:j], i, j, line))
            i = j
            continue
        if c.isdigit() or (c == "." and i + 1 < n and src[i + 1].isdigit()):
            j = i + 1
            while j < n and (src[j].isalnum() or src[j] in "._" or (src[j] in "+-" and src[j - 1] in "eEpP" and not src[i:j].lower().startswith("0x") or (src[j] in "+-" and src[j - 1] in "pP"))):
                j += 1
            t = src[i:j]
            isf = (("." in t) or (re.search(r"[eE]", t) and not t.lower().startswith("0x")) or (t.lower().startswith("0x") and "p" in t.lower()))
            toks.append(Tok("float" if isf else "num", t, i, j, line))
            i = j
            continue
        if c == '"' or c == "'":
            j = i + 1
            while j < n and src[j] != c:
                j += 2 if src[j] == "\\" else 1
            j += 1
            toks.append(Tok("str" if c == '"' else "chr", src[i:j], i, j, line))
            line += src.count("\n", i, j)
            i = j
            continue
        t = None
        for p in PUNCT3:
            if src.startswith(p, i):
                t = p
                break
        if t is None:
            for p in PUNCT2:
                if src.startswith(p, i):
                    t = p
                    break
        if t is None:
            t = c
        toks.append(Tok("p", t, i, i + len(t), line))
        i += len(t)
    for k, t in enumerate(toks):
        t.i = k
    return toks


def match_parens(toks):
    """index of the matching closer for every (, [, { token"""
    m, st = {}, []
    for t in toks:
        if t.kind == "p" and t.text in "([{" and len(t.text) == 1:
            st.append(t.i)
        elif t.kind == "p" and t.text in ")]}" and len(t.text) == 1:
            if st:
                j = st.pop()
                m[j] = t.i
                m[t.i] = j
    return m


def num_value(text):
    t = text.rstrip("uUlL")
    try:
        return int(t, 16) if t.lower().startswith("0x") else (int(t, 8) if (len(t) > 1 and t[0] == "0" and t.isdigit()) else int(t))
    except ValueError:
        return None


# ====================================================================================================================
# 2. source model: functions, type groups, casts, declarations, accessors
# ====================================================================================================================
class Func:
    def __init__(self, name, head_i, lp, rp, lb, rb):
        self.name, self.head_i, self.lp, self.rp, self.lb, self.rb = name, head_i, lp, rp, lb, rb
        self.params = []          # list of param names in order
        self.ptr_vars = set()     # identifiers declared with a pointer type
        self.vars = {}            # ident -> (param index, delta) resolved base pointers (single assignment)


class Model:
    def __init__(self, src):
        self.src = src
        self.toks = lex(src)
        self.mp = match_parens(self.toks)
        self.typedefs = set(STD_TYPES)
        self.accessors = set()
        for mo in re.finditer(r"^[ \t]*#[ \t]*define[ \t]+([A-Za-z_]\w*)\(([^)]*)\)", src, re.M):
            if ACCESSOR_RE.match(mo.group(1)) and len([a for a in mo.group(2).split(",") if a.strip()]) == 2:
                self.accessors.add(mo.group(1))
        for mo in re.finditer(r"\btypedef\b[^;]*?\b(\w+)(?:\[[^\]]*\])?\s*;", src):
            self.typedefs.add(mo.group(1))
        for mo in re.finditer(r"\btemplate\s*<([^<>]*)>", src):            # C++: template parameter names are type names
            for pm in re.finditer(r"\b(?:class|typename)\s+(\w+)", mo.group(1)):
                self.typedefs.add(pm.group(1))
        self.funcs = self._find_funcs()
        self.cast_close = {}      # index of ')' closing a cast -> index of its '('
        self.type_groups = {}     # start index -> (end index inclusive) of type groups found by scan
        self._scan_casts_decls()
        self.line_starts = [0] + [m.end() for m in re.finditer("\n", src)]

    # ---- line helpers
    def line_of(self, pos):
        import bisect
        return bisect.bisect_right(self.line_starts, pos)

    def line_text(self, ln):
        a = self.line_starts[ln - 1]
        b = self.line_starts[ln] - 1 if ln < len(self.line_starts) else len(self.src)
        return self.src[a:b]

    def is_type_tok(self, t):
        return t.kind == "id" and (t.text in TYPE_KW or t.text in QUAL_KW or t.text in self.typedefs)

    def _find_funcs(self):
        toks, mp = self.toks, self.mp
        funcs, depth, i, n = [], 0, 0, len(toks)
        while i < n:
            t = toks[i]
            if t.kind == "p" and t.text == "{":
                # function body if the previous token is ')' whose '(' is preceded by an identifier, at depth 0
                if depth == 0 and i > 0 and toks[i - 1].text == ")" and (i - 1) in mp:
                    lp = mp[i - 1]
                    if lp > 0 and toks[lp - 1].kind == "id" and toks[lp - 1].text not in KEYWORDS and toks[lp - 1].text not in ("__attribute__",):
                        rb = mp.get(i)
                        if rb is not None:
                            f = Func(self._qualified_name(lp - 1), lp - 1, lp, i - 1, i, rb)
                            self._parse_params(f)
                            if "::" in f.name:                  # C++ member / overload: unique key  Class::method/nparams
                                f.name = "%s/%d" % (f.name, len([p for p in f.params if p is not None]))
                            funcs.append(f)
                            i = rb + 1
                            continue
                depth += 1
            elif t.kind == "p" and t.text == "}":
                depth -= 1
            i += 1
        return {f.name: f for f in funcs}

    def _qualified_name(self, k):
        """C: the identifier before '('.  C++: `Scrambler<H, O>::process` -> 'Scrambler::process' (template arguments dropped,
        a destructor keeps its '~'); a plain function keeps its name."""
        toks = self.toks
        name = toks[k].text
        if k > 0 and toks[k - 1].text == "~":
            name = "~" + name
            k -= 1
        parts = [name]
        j = k - 1
        while j >= 1 and toks[j].text == "::":
            j -= 1
            if toks[j].text == ">":                        # skip a balanced <...>
                d = 0
                while j >= 0:
                    if toks[j].text == ">":
                        d += 1
                    elif toks[j].text == "<":
                        d -= 1
                        if d == 0:
                            break
                    j -= 1
                j -= 1
            if j < 0 or toks[j].kind != "id":
                break
            parts.insert(0, toks[j].text)
            j -= 1
        return "::".join(parts)

    def _parse_params(self, f):
        toks = self.toks
        args, cur, d = [], [], 0
        for k in range(f.lp + 1, f.rp):
            t = toks[k]
            if t.text in "([" and t.kind == "p":
                d += 1
            if t.text in ")]" and t.kind == "p":
                d -= 1
            if t.text == "," and d == 0 and t.kind == "p":
                args.append(cur)
                cur = []
            else:
                cur.append(t)
        if cur:
            args.append(cur)
        for a in args:
            ids = [t for t in a if t.kind == "id" and not self.is_type_tok(t) and t.text not in ("struct", "union", "enum")]
            f.params.append(ids[-1].text if ids else None)
            if any(t.text == "*" for t in a) and ids:
                f.ptr_vars.add(ids[-1].text)

    def type_group_at(self, i, lo=0, hi=None):
        """maximal run of type tokens starting at i (returns end index exclusive or None).  struct/union/enum X counted."""
        toks = self.toks
        hi = len(toks) if hi is None else hi
        j = i
        if j < hi and toks[j].kind == "id" and toks[j].text in ("struct", "union", "enum"):
            j += 2
        while j < hi and self.is_type_tok(toks[j]):
            j += 1
        if j == i:
            return None
        # a lone identifier typedef must be followed by a declarator-ish token to be a type (not a variable use)
        return j

    def _scan_casts_decls(self):
        """mark casts '(' TYPE '*'* ')' followed by an operand, and declarations TYPE '*'* IDENT.  Result tables:
        self.casts: list of dict(lp, rp, ts, te (type token span, excl qualifiers handled later), ptr)
        self.decls: list of dict(ts, te, ptr, name_i)"""
        toks, mp = self.toks, self.mp
        self.casts, self.decls = [], []
        for f in self.funcs.values():
            i = f.head_i + 1
            while i < f.rb:
                t = toks[i]
                # cast
                if t.kind == "p" and t.text == "(" and i + 1 < f.rb:
                    j = self.type_group_at(i + 1, hi=f.rb)
                    if j is not None and i > 0 and toks[i - 1].text != "sizeof":
                        k = j
                        while k < f.rb and toks[k].text == "*":
                            k += 1
                        if k < f.rb and toks[k].text == ")" and mp.get(i) == k and k + 1 < len(toks):
                            nx = toks[k + 1]
                            prev = toks[i - 1]
                            prev_operand = (prev.kind in ("id", "num") and prev.text not in KEYWORDS) or (prev.text == ")" and (i - 1) not in self.cast_close) or prev.text == "]"
                            is_call_param = prev.kind == "id" and prev.text not in KEYWORDS and i > f.head_i and toks[i - 1].i == f.head_i
                            if (nx.kind in ("id", "num", "chr", "str", "float") or nx.text in ("(", "-", "~", "!", "*", "&", "+")) and not prev_operand:
                                self.casts.append(dict(lp=i, rp=k, ts=i + 1, te=j, nptr=k - j, func=f.name))
                                self.cast_close[k] = i
                # declaration: TYPE '*'* IDENT  at a statement start / after '(' , ',' (params) in for-init
                if self.is_type_tok(t) and t.kind == "id" and (i == f.head_i + 1 or i - 1 <= f.lp or toks[i - 1].text in (";", "{", "}", ",", "(", ")") or self.is_type_tok(toks[i - 1]) is False and toks[i - 1].text in ("for",)):
                    j = self.type_group_at(i, hi=f.rb)
                    if j is not None and not (i - 1 >= 0 and self.is_type_tok(toks[i - 1])):
                        k = j
                        while k < f.rb and toks[k].text == "*":
                            k += 1
                        if k < f.rb and toks[k].kind == "id" and not self.is_type_tok(toks[k]) and k + 1 <= f.rb and toks[k + 1].text in (";", "=", ",", "[", ")") \
                                and not (toks[i - 1].text == "(" and i - 1 != f.lp and toks[i - 2].kind == "id" and toks[i - 2].text not in ("for",) ):
                            self.decls.append(dict(ts=i, te=j, nptr=k - j, name_i=k, func=f.name, param=(i <= f.rp)))
                            if k - j:
                                self.funcs[f.name].ptr_vars.add(toks[k].text)
                i += 1
        for c in self.casts:
            self.funcs[c["func"]]  # noqa


# ====================================================================================================================
# 3. mutant generation
# ====================================================================================================================
class Mutant:
    __slots__ = ("cls", "tags", "func", "line", "edits", "note", "id", "direct", "desc", "site", "need")

    def __init__(self, cls, tags, func, line, edits, note, direct="", site=None, need=None):
        self.cls, self.tags, self.func, self.line, self.edits, self.note, self.direct = cls, list(tags), func, line, edits, note, direct
        self.site, self.need = site, need
        h = hashlib.sha1(("%s|%s|%s" % (cls, func, json.dumps(edits))).encode()).hexdigest()[:10]
        self.id = h

    def to_json(self):
        return dict(id=self.id, cls=self.cls, tags=self.tags, func=self.func, line=self.line, edits=self.edits, note=self.note,
                    direct=self.direct, site=self.site, need=self.need)


def apply_edits(src, edits):
    out, pos = [], 0
    for s, e, t in sorted(edits, key=lambda x: (x[0], x[1])):
        out.append(src[pos:s])
        out.append(t)
        pos = e
    out.append(src[pos:])
    return "".join(out)


def operand_end_tok(m, i):
    """True if token i can END an operand (so a following + - * & is binary)."""
    t = m.toks[i]
    if t.kind in ("num", "float", "str", "chr"):
        return True
    if t.kind == "id":
        return t.text not in KEYWORDS and not (t.text in TYPE_KW or t.text in QUAL_KW) or t.text in ("sizeof",) and False
    if t.kind == "p" and t.text == ")":
        return i not in m.cast_close
    if t.kind == "p" and t.text in ("]",):
        return True
    return False


STOP_LEFT = {";", "{", "}", ",", "?", ":", "&&", "||", "return"} | ASSIGN_OPS
def span_of(m, opi, side, level):
    """operand span (first token index, last token index) of the binary operator at token index opi.
    level: 'rel' (operand of < <= > >=), 'shl' (operand of << / >>), 'mul' (operand of / %)"""
    toks, mp = m.toks, m.mp
    lower = {"rel": EQ_OPS | REL_OPS | {"&", "|", "^"}, "shl": REL_OPS | EQ_OPS | {"&", "|", "^", "<<", ">>"}, "mul": REL_OPS | EQ_OPS | {"&", "|", "^", "<<", ">>", "+", "-", "*", "/", "%"}}[level]
    if side < 0:
        j = opi - 1
        while j >= 0:
            t = toks[j]
            if t.kind == "p" and t.text in (")", "]") and j in mp:
                j = mp[j] - 1
                continue
            if t.kind == "p" and t.text in ("(", "[", "{"):
                break
            tx = t.text
            if (t.kind == "p" and (tx in STOP_LEFT or tx in EQ_OPS or tx in REL_OPS or tx in ("<<", ">>"))) or (t.kind == "id" and tx in ("return", "case")):
                if tx in lower or tx in STOP_LEFT or tx == "return":
                    break
            if t.kind == "p" and tx in ("&", "|", "^", "+", "-", "*", "/", "%") and tx in lower and j > 0 and operand_end_tok(m, j - 1):
                break
            j -= 1
        return (j + 1, opi - 1)
    j = opi + 1
    n = len(toks)
    while j < n:
        t = toks[j]
        tx = t.text
        if t.kind == "p" and tx in ("(", "[") and j in mp:
            j = mp[j] + 1
            continue
        if t.kind == "p" and tx in (")", "]", "}", ";", ",", "?", ":", "&&", "||") or (t.kind == "p" and tx in ASSIGN_OPS):
            break
        if t.kind == "p" and tx in lower and (tx in EQ_OPS or tx in REL_OPS or tx in ("<<", ">>") or (j > opi + 1 and operand_end_tok(m, j - 1))):
            if tx in ("+", "-", "*", "/", "%", "&", "|", "^") and not (j > opi + 1 and operand_end_tok(m, j - 1)):
                j += 1
                continue
            break
        j += 1
    return (opi + 1, j - 1)


CAST_W = {"int8_t": 8, "uint8_t": 8, "s8_t": 8, "u8_t": 8, "char": 8, "unsigned char": 8, "signed char": 8,
          "int16_t": 16, "uint16_t": 16, "s16_t": 16, "u16_t": 16, "short": 16, "unsigned short": 16, "short int": 16, "unsigned short int": 16,
          "int32_t": 32, "uint32_t": 32, "s32_t": 32, "u32_t": 32, "int": 32, "unsigned": 32, "unsigned int": 32, "long": 32, "unsigned long": 32,
          "intptr_t": 32, "uintptr_t": 32, "size_t": 32, "ptrdiff_t": 32}


def unary_end(m, k):
    """index of the last token of the unary expression that starts at token k (prefix ops / casts, a primary, postfix chain)"""
    toks, mp = m.toks, m.mp
    n = len(toks)
    while k < n:
        t = toks[k]
        if t.kind == "p" and t.text in ("-", "+", "!", "~", "*", "&", "++", "--"):
            k += 1
            continue
        if t.kind == "id" and t.text == "sizeof":
            k += 1
            continue
        if t.kind == "p" and t.text == "(" and k in m.cast_close.values():
            k = mp[k] + 1
            continue
        break
    if k >= n:
        return None
    t = toks[k]
    if t.kind in ("num", "float", "str", "chr") or (t.kind == "id"):
        e = k
    elif t.kind == "p" and t.text == "(" and k in mp:
        e = mp[k]
    else:
        return None
    while e + 1 < n:
        nx = toks[e + 1]
        if nx.kind == "p" and nx.text in ("(", "[") and (e + 1) in mp:
            e = mp[e + 1]
        elif nx.kind == "p" and nx.text in (".", "->") and e + 2 < n and toks[e + 2].kind == "id":
            e += 2
        elif nx.kind == "p" and nx.text in ("++", "--"):
            e += 1
        else:
            break
    return e


def lit_dset(op, side, delta):
    """values of (left - right) at a relational for which changing the literal operand by `delta` changes the outcome"""
    mirror = {"<": ">", ">": "<", "<=": ">=", ">=": "<=", "==": "==", "!=": "!="}
    o = op if side == "R" else mirror[op]           # normalise to  x OP c,  t = x - c
    if o in ("<", ">="):
        t = set(range(min(0, delta), max(0, delta)))
    elif o in ("<=", ">"):
        t = set(range(min(1, delta + 1), max(1, delta + 1)))
    else:
        t = {0, delta}
    return t if side == "R" else {-x for x in t}          # left - right


def toggled_type(text_tokens):
    """core type spelling (qualifiers removed) -> sign-flipped spelling or None"""
    core = [t for t in text_tokens if t not in QUAL_KW]
    if len(core) == 1 and core[0] in SIGN_FLIP:
        return SIGN_FLIP[core[0]]
    return SIGN_FLIP_KW.get(" ".join(core))


def type_core_span(m, ts, te):
    toks = m.toks
    idx = [k for k in range(ts, te) if toks[k].text not in QUAL_KW]
    if not idx:
        return None
    return idx[0], idx[-1]


class Generator:
    def __init__(self, m, funcs, evidence=None, lit_deltas=(1, 2), opts=None):
        self.m, self.funcs, self.ev = m, funcs, evidence
        self.lit_deltas = lit_deltas
        self.opts = opts or {}
        self.out = []
        self.sites = {}           # probe sites (see the probe instrumenter)
        self.stats = collections.Counter()
        self._resolve_bases()

    # ---- base-pointer resolution: which parameter + byte delta does each pointer variable denote
    def _strip_casts(self, lo, hi):
        """token range with leading '(' TYPE '*' ')' casts and redundant parens removed"""
        toks = self.m.toks
        while lo <= hi:
            if toks[lo].text == "(" and lo in self.m.cast_close.values():
                lo = self.m.mp[lo] + 1
                continue
            if toks[lo].text == "(" and self.m.mp.get(lo) == hi:
                lo, hi = lo + 1, hi - 1
                continue
            break
        return lo, hi

    def _split_args(self, lp, rp):
        toks, mp = self.m.toks, self.m.mp
        args, cur, k = [], lp + 1, lp + 1
        while k < rp:
            t = toks[k]
            if t.kind == "p" and t.text in "([{" and k in mp:
                k = mp[k] + 1
                continue
            if t.kind == "p" and t.text == ",":
                args.append((cur, k - 1))
                cur = k + 1
            k += 1
        if cur <= rp - 1:
            args.append((cur, rp - 1))
        return args

    def _const_sum(self, lo, hi):
        """value of a +/- sum of integer literals in toks[lo..hi], else None"""
        toks = self.m.toks
        v, sign, k, expect = 0, 1, lo, True
        while k <= hi:
            t = toks[k]
            if expect and t.kind == "num":
                nv = num_value(t.text)
                if nv is None:
                    return None
                v += sign * nv
                expect = False
            elif not expect and t.text in ("+", "-"):
                sign = 1 if t.text == "+" else -1
                expect = True
            elif expect and t.text in ("-", "+") :
                sign = sign * (1 if t.text == "+" else -1)
            else:
                return None
            k += 1
        return None if expect else v

    def base_of(self, f, lo, hi):
        """(param index, delta) for a pointer-valued expression toks[lo..hi] of function f, or None"""
        toks = self.m.toks
        lo, hi = self._strip_casts(lo, hi)
        if lo > hi:
            return None
        if lo == hi and toks[lo].kind == "id":
            nm = toks[lo].text
            if nm in f.vars:
                return f.vars[nm]
            if nm in f.params:
                return (f.params.index(nm), 0)
            return None
        if toks[lo].kind == "id" and toks[lo].text == "OBJ_BYTES" and toks[lo + 1].text == "(" and self.m.mp.get(lo + 1) == hi:
            a = self._split_args(lo + 1, hi)
            if len(a) == 2:
                b = self.base_of(f, *a[0])
                c = self._const_sum(*a[1])
                if b is not None and c is not None:
                    return (b[0], b[1] + c)
            return None
        # X + C  /  C + X (top level)
        d, k = 0, lo
        while k <= hi:
            t = toks[k]
            if t.kind == "p" and t.text in "([{" and k in self.m.mp:
                k = self.m.mp[k] + 1
                continue
            if t.kind == "p" and t.text in ("+", "-") and k > lo:
                b = self.base_of(f, lo, k - 1)
                c = self._const_sum(k, hi) if t.text == "+" else None
                if t.text == "+":
                    c = self._const_sum(k + 1, hi)
                else:
                    c = self._const_sum(k + 1, hi)
                    c = -c if c is not None else None
                if b is not None and c is not None:
                    return (b[0], b[1] + c)
                return None
            k += 1
        return None

    def _resolve_bases(self):
        toks = self.m.toks
        for f in self.m.funcs.values():
            assigns = collections.defaultdict(list)
            for k in range(f.lb + 1, f.rb):
                t = toks[k]
                if t.kind == "id" and k + 1 < f.rb and toks[k + 1].kind == "p" and (toks[k + 1].text in ASSIGN_OPS or toks[k + 1].text in ("++", "--")):
                    if k > 0 and toks[k - 1].text in (".", "->", "*"):
                        pass
                    assigns[t.text].append(k)
                if t.kind == "p" and t.text in ("++", "--") and k + 1 < f.rb and toks[k + 1].kind == "id":
                    assigns[toks[k + 1].text].append(k + 1)
                if t.kind == "p" and t.text == "&" and k + 1 < f.rb and toks[k + 1].kind == "id" and not operand_end_tok(self.m, k - 1):
                    assigns[toks[k + 1].text].append(k + 1)       # address taken: may be assigned through the pointer
            f._assigns = assigns
        # iterate to a fixed point (vars defined from vars)
        for _ in range(3):
            for f in self.m.funcs.values():
                for nm, ks in f._assigns.items():
                    if nm in f.vars or len(ks) != 1 or nm in f.params:
                        continue
                    k = ks[0]
                    if toks[k + 1].text != "=":
                        continue
                    # rhs until ';'
                    e = k + 2
                    while e < f.rb and toks[e].text != ";":
                        if toks[e].kind == "p" and toks[e].text in "([{" and e in self.m.mp:
                            e = self.m.mp[e]
                        e += 1
                    b = self.base_of(f, k + 2, e - 1)
                    if b is not None:
                        f.vars[nm] = b
        # helper functions called from others with a resolvable argument: param -> (caller, caller param index, delta)
        self.helper_maps = {}
        for fn in self.m.funcs.values():
            for k in range(fn.lb + 1, fn.rb):
                t = toks[k]
                if t.kind == "id" and t.text in self.m.funcs and t.text != fn.name and toks[k + 1].text == "(":
                    callee = self.m.funcs[t.text]
                    args = self._split_args(k + 1, self.m.mp[k + 1])
                    for pi, (lo, hi) in enumerate(args):
                        b = self.base_of(fn, lo, hi)
                        self.helper_maps.setdefault((callee.name, pi), []).append((fn.name, b))

    def root_of(self, f, nm):
        """resolve a base variable name in function f to (evidence function, param index, delta) or None"""
        if nm in f.vars:
            pi, dl = f.vars[nm]
        elif nm in f.params:
            pi, dl = f.params.index(nm), 0
        else:
            return None
        return self.root_param(f.name, pi, dl, 0)

    def root_param(self, fname, pi, dl, depth):
        if self.ev is not None and self.ev.has_func(fname):
            return (fname, pi, dl)
        if depth > 4:
            return None
        outs = set()
        sites = self.helper_maps.get((fname, pi), [])
        if not sites:
            return None
        for (caller, b) in sites:
            if b is None:
                return None
            r = self.root_param(caller, b[0], b[1] + dl, depth + 1)
            if r is None:
                return None
            outs.add(r)
        return outs.pop() if len(outs) == 1 else None

    # ---- accessor occurrences
    def accessors(self, f):
        toks, mp = self.m.toks, self.m.mp
        res = []
        for k in range(f.lb + 1, f.rb):
            t = toks[k]
            if t.kind == "id" and t.text in self.m.accessors and k + 1 < len(toks) and toks[k + 1].text == "(":
                rp = mp.get(k + 1)
                if rp is None:
                    continue
                args = self._split_args(k + 1, rp)
                if len(args) != 2:
                    continue
                mo = ACCESSOR_RE.match(t.text)
                sgn, w = (mo.group(1), int(mo.group(2))) if mo.group(1) else ("P", 32)
                nx = toks[rp + 1].text if rp + 1 < len(toks) else ""
                pv = toks[k - 1].text if k > 0 else ""
                is_store = nx in ASSIGN_OPS or nx in ("++", "--") or pv in ("++", "--")
                addr = pv == "&" and not operand_end_tok(self.m, k - 2) if k >= 2 else (pv == "&")
                res.append(dict(k=k, rp=rp, name=t.text, sgn=sgn, w=w, args=args, store=is_store, addr=addr))
        return res

    def acc_key(self, f, a):
        """(root evidence func, param idx, byte offset, is_indexed) or None"""
        toks = self.m.toks
        (l0, h0), (l1, h1) = a["args"]
        l0, h0 = self._strip_casts(l0, h0)
        if not (l0 == h0 and toks[l0].kind == "id"):
            return None
        r = self.root_of(f, toks[l0].text)
        if r is None:
            return None
        # offset: leading literal (+ sum); indexed if other non-literal terms
        c = self._const_sum(l1, h1)
        if c is not None:
            return (r[0], r[1], r[2] + c, False)
        # first top-level constant term
        k, consts, other = l1, 0, False
        sign = 1
        while k <= h1:
            t = toks[k]
            if t.kind == "num" and (k == l1 or toks[k - 1].text in ("+", "-")) and (k == h1 or toks[k + 1].text in ("+", "-")):
                v = num_value(t.text)
                consts += (sign if k == l1 or toks[k - 1].text == "+" else -1) * v if v is not None else 0
            elif t.kind == "p" and t.text in "([{" and k in self.m.mp:
                other = True
                k = self.m.mp[k]
            elif t.text not in ("+", "-"):
                other = True
            k += 1
        return (r[0], r[1], r[2] + consts, True)

    # ---- emit helpers
    def emit(self, cls, tags, f, tokspan_or_pos, edits, note, direct="", site=None, need=None):
        pos = tokspan_or_pos
        line = self.m.line_of(pos)
        self.out.append(Mutant(cls, tags, f.name, line, edits, note, direct, site, need))

    # ---- main
    def run(self):
        m = self.m
        for fname in self.funcs:
            f = m.funcs[fname]
            self.gen_func(f)
        # dedupe by id
        seen, res = set(), []
        for x in self.out:
            if x.id in seen:
                continue
            seen.add(x.id)
            res.append(x)
        self.out = res
        return res

    def in_body(self, f, k):
        return f.lb < k < f.rb

    def gen_func(self, f):
        m, toks = self.m, self.m.toks
        acc = self.accessors(f)
        acc_args_lits = set()            # token indices of literals chosen as accessor offset targets
        # -------- OFF
        for a in acc:
            (l1, h1) = a["args"][1]
            tgt = []
            k = l1
            while k <= h1:
                t = toks[k]
                if t.kind == "num" and (k == l1 or toks[k - 1].text in ("+", "-")) and (k == h1 or toks[k + 1].text in ("+", "-")):
                    tgt.append(k)
                elif t.kind == "p" and t.text in "([" and k in m.mp:
                    k = m.mp[k]
                k += 1
            if tgt:
                k = tgt[0]
                v = num_value(toks[k].text)
                if v is not None:
                    acc_args_lits.add(k)
                    w = a["w"] // 8
                    deltas = {1: (1, -1), 2: (2, -2), 4: (2, -2, 4, -4)}.get(w, (2, -2))
                    for d in deltas:
                        nv = v + d
                        if nv < 0:
                            continue
                        txt = ("0x%x" % nv) if toks[k].text.lower().startswith("0x") else str(nv)
                        self.emit("OFF", ["OFF"], f, toks[k].s, [(toks[k].s, toks[k].e, txt)], "%s offset %s -> %s" % (a["name"], toks[k].text, txt))
        # -------- walk tokens
        litrel = {}           # literal token that is a WHOLE operand of a relational/equality -> (op token, side)
        for k in range(f.lb + 1, f.rb):
            if toks[k].kind == "p" and (toks[k].text in REL_OPS or toks[k].text in EQ_OPS):
                l, r = span_of(m, k, -1, "rel"), span_of(m, k, +1, "rel")
                if l[0] == l[1] and toks[l[0]].kind == "num":
                    litrel[l[0]] = (k, "L")
                if r[0] == r[1] and toks[r[0]].kind == "num":
                    litrel[r[0]] = (k, "R")
        for k in range(f.lb + 1, f.rb):
            t = toks[k]
            # LIT
            if t.kind == "num" and k not in acc_args_lits:
                v = num_value(t.text)
                if v is not None:
                    suf = re.search(r"[uUlL]*$", t.text).group(0)
                    ishex = t.text.lower().startswith("0x")
                    for d in self.lit_deltas:
                        for sg in (1, -1):
                            nv = v + sg * d
                            if nv > 0xffffffff:
                                continue
                            if nv < 0:
                                txt = "(%d%s)" % (nv, "")
                            else:
                                txt = (("0x%x" % nv) if ishex else str(nv)) + suf
                            site = need = None
                            if k in litrel and nv >= 0 and v >= 0:
                                ok_, sd_ = litrel[k]
                                self.add_rel_site(f, ok_)
                                if not self.sites["R%d" % ok_]["ptr"]:
                                    site, need = "R%d" % ok_, "d:" + ",".join(str(x) for x in sorted(lit_dset(toks[ok_].text, sd_, sg * d)))
                            lit_direct, lit_tags = "", ["LIT", "LIT%d" % d]
                            if k > 0 and toks[k - 1].text == "[" and toks[k + 1].text == "]":
                                q = k - 1                      # array size in a declaration (statement starts with a type): not behaviour
                                while q > f.lb and toks[q].text not in (";", "{", "}"):
                                    q -= 1
                                if q + 1 < k and self.m.is_type_tok(toks[q + 1]):
                                    lit_direct, lit_tags = "NOISY", lit_tags + ["DECL-SIZE"]
                            self.emit("LIT", lit_tags, f, t.s, [(t.s, t.e, txt)], "%s -> %s" % (t.text, txt), lit_direct, site, need)
                continue
            if t.kind != "p":
                continue
            tx = t.text
            prev_op = k > 0 and operand_end_tok(m, k - 1)
            # REL
            if tx in REL_OPS or tx in EQ_OPS:
                outs = {"<": [("<=", "bound"), (">=", "negate")], "<=": [("<", "bound"), (">", "negate")], ">": [(">=", "bound"), ("<=", "negate")],
                        ">=": [(">", "bound"), ("<", "negate")], "==": [("!=", "negate")], "!=": [("==", "negate")]}[tx]
                self.add_rel_site(f, k)
                for nt, kind in outs:
                    self.emit("REL", ["REL", "REL-" + kind], f, t.s, [(t.s, t.e, nt)], "%s -> %s" % (tx, nt), "", "R%d" % k, "eq" if kind == "bound" else "n")
            elif tx in ("+", "-", "*", "/", "%") and prev_op:
                # skip pointer stars (prev is a type) - handled by operand_end_tok (type tokens are not operand ends)
                outs = {"+": ["-"], "-": ["+"], "*": ["+"], "/": ["*"], "%": ["/"]}[tx]
                for nt in outs:
                    self.emit("ARITH", ["ARITH"], f, t.s, [(t.s, t.e, nt)], "%s -> %s" % (tx, nt))
            elif tx in ("+=", "-=", "*=", "/=", "%="):
                nt = {"+=": "-=", "-=": "+=", "*=": "+=", "/=": "*=", "%=": "/="}[tx]
                self.emit("ARITH", ["ARITH"], f, t.s, [(t.s, t.e, nt)], "%s -> %s" % (tx, nt))
            elif tx in ("&", "|", "^") and prev_op:
                nt = {"&": "|", "|": "&", "^": "|"}[tx]
                self.emit("BIT", ["BIT"], f, t.s, [(t.s, t.e, nt)], "%s -> %s" % (tx, nt))
            elif tx in ("&=", "|=", "^="):
                nt = {"&=": "|=", "|=": "&=", "^=": "|="}[tx]
                self.emit("BIT", ["BIT"], f, t.s, [(t.s, t.e, nt)], "%s -> %s" % (tx, nt))
            elif tx in ("&&", "||"):
                nt = "||" if tx == "&&" else "&&"
                self.emit("LOGIC", ["LOGIC"], f, t.s, [(t.s, t.e, nt)], "%s -> %s" % (tx, nt))
            elif tx in ("<<", ">>"):
                nt = ">>" if tx == "<<" else "<<"
                self.emit("SHIFT", ["SHIFT"], f, t.s, [(t.s, t.e, nt)], "%s -> %s" % (tx, nt))
            elif tx in ("<<=", ">>="):
                nt = ">>=" if tx == "<<=" else "<<="
                self.emit("SHIFT", ["SHIFT"], f, t.s, [(t.s, t.e, nt)], "%s -> %s" % (tx, nt))
            elif tx == "!" and not prev_op:
                self.emit("NEG", ["NEG", "NEG-not"], f, t.s, [(t.s, t.e, "")], "drop !")
        # -------- NEG: if-condition negation
        for k in range(f.lb + 1, f.rb):
            t = toks[k]
            if t.kind == "id" and t.text == "if" and toks[k + 1].text == "(":
                rp = m.mp[k + 1]
                lps = toks[k + 1]
                self.emit("NEG", ["NEG", "NEG-if"], f, t.s, [(lps.e, lps.e, "!("), (toks[rp].s, toks[rp].s, ")")], "if (c) -> if (!(c))")
        # -------- STMT
        self.gen_stmts(f)
        # -------- sign-sensitive classes
        self.gen_sign(f, acc)

    def add_rel_site(self, f, k):
        key = "R%d" % k
        if key not in self.sites:
            l, r = span_of(self.m, k, -1, "rel"), span_of(self.m, k, +1, "rel")
            self.sites[key] = dict(kind="rel", op=k, l=l, r=r, ptr=self.is_ptr_span(f, l) or self.is_ptr_span(f, r))

    def gen_stmts(self, f):
        m, toks, mp = self.m, self.m.toks, self.m.mp
        hdr_close = set()
        for k in range(f.lb + 1, f.rb):
            t = toks[k]
            if t.kind == "id" and t.text in ("if", "for", "while", "switch") and toks[k + 1].text == "(":
                hdr_close.add(mp[k + 1])
        k = f.lb + 1
        while k < f.rb:
            t = toks[k]
            prev = toks[k - 1]
            starts = (prev.text in (";", "{", "}") and prev.kind == "p") or (k - 1 in hdr_close) or (prev.kind == "id" and prev.text == "else")
            if prev.text == ";" and prev.i > 0 and toks[prev.i - 1].text == ")" and False:
                starts = False
            if starts and not (t.kind == "id" and (t.text in KEYWORDS or self.m.is_type_tok(t))) and not (t.kind == "p" and t.text in (";", "{", "}")):
                # find the ';' at depth 0
                e = k
                ok = True
                while e < f.rb:
                    te = toks[e]
                    if te.kind == "p" and te.text in "([" and e in mp:
                        e = mp[e] + 1
                        continue
                    if te.kind == "p" and te.text in ("{", "}"):
                        ok = False
                        break
                    if te.kind == "p" and te.text == ";":
                        break
                    e += 1
                if ok and e < f.rb and not (t.kind == "id" and toks[k + 1].text == ":"):
                    # declaration of a user typedef we don't know? require an assignment or a call or ++/--
                    stmt = toks[k:e]
                    has_assign = any(x.kind == "p" and (x.text in ASSIGN_OPS or x.text in ("++", "--")) for x in stmt)
                    is_call = toks[k].kind == "id" and toks[k + 1].text == "("
                    if has_assign or is_call:
                        kind = "store" if has_assign else "call"
                        s0, e0 = toks[k].s, toks[e].e
                        site = need = None
                        # simple assignment  LHS = RHS ;  with LHS an identifier or an accessor: probe "value changes"
                        q = k
                        while q < e and not (toks[q].kind == "p" and toks[q].text in ASSIGN_OPS):
                            if toks[q].kind == "p" and toks[q].text in "([" and q in mp:
                                q = mp[q]
                            q += 1
                        pure = all(not (x.kind == "id" and toks[x.i + 1].text == "(" and x.text not in self.m.accessors and x.text not in ("OBJ_BYTES", "sar32", "mul32") and not self.m.is_type_tok(x))
                                   for x in toks[q + 1:e]) if q < e else False
                        # an identifier LHS must be definitely initialised (a parameter, or declared with an initialiser): deleting the
                        # first assignment of an uninitialised local makes the mutant read an indeterminate value (not a value-based difference)
                        if pure and q == k + 1 and not (toks[k].text in f.params or any(
                                d["func"] == f.name and toks[d["name_i"]].text == toks[k].text and toks[d["name_i"] + 1].text == "=" for d in self.m.decls)):
                            pure = False
                        if pure and q < e and toks[q].text == "=" and q > k and (q == k + 1 or (toks[k].text in self.m.accessors and mp.get(k + 1) == q - 1)) \
                                and toks[k].kind == "id" and not any(x.kind == "p" and x.text in ("++", "--") for x in toks[k:e]):
                            site, need = "A%d" % k, "chg"
                            self.sites[site] = dict(kind="asg", lhs=(k, q - 1), rhs=(q + 1, e - 1))
                        self.emit("STMT", ["STMT", "STMT-" + kind], f, s0, [(s0, e0, ";")], "delete statement", "", site, need)
                    k = e
            k += 1

    # ---------------------------------------------------------------------------------------------------
    def gen_sign(self, f, acc):
        m, toks = self.m, self.m.toks
        # spans that consume sign: operands of relationals / >> (left) / / %
        consume = []       # (lo, hi, kind)
        relops = []
        for k in range(f.lb + 1, f.rb):
            t = toks[k]
            if t.kind != "p":
                continue
            if t.text in REL_OPS:
                l = span_of(m, k, -1, "rel")
                r = span_of(m, k, +1, "rel")
                consume.append((l[0], l[1], "rel"))
                consume.append((r[0], r[1], "rel"))
                relops.append((k, l, r))
            elif t.text in ("/", "%") and k > 0 and operand_end_tok(m, k - 1):
                l = span_of(m, k, -1, "mul")
                r = span_of(m, k, +1, "mul")
                consume.append((l[0], l[1], "div"))
                consume.append((r[0], r[1], "div"))
            elif t.text == ">>" and k > 0:
                l = span_of(m, k, -1, "shl")
                consume.append((l[0], l[1], "shr"))

        def in_consume(k):
            return [c[2] for c in consume if c[0] <= k <= c[1]]

        # ---- accessor keys and evidence
        for a in acc:
            a["key"] = self.acc_key(f, a)
            a["ev"] = None
            if a["key"] is not None and self.ev is not None:
                a["ev"] = self.ev.acc_evidence(a["key"][0], a["key"][1], a["key"][2], a["w"], a["key"][3])
        # ---- SIGNA: accessor load sign swaps (blob directed)
        for a in acc:
            if a["store"] or a["addr"] or a["sgn"] == "P":
                continue
            nm = a["name"]
            nn = ("U" if a["sgn"] == "S" else "S") + str(a["w"])
            t = toks[a["k"]]
            ev = a["ev"]
            if ev is None:
                direct, reason = "UNDIRECTED", "no blob mapping"
            elif not ev["found"]:
                direct, reason = "UNDIRECTED", "blob has no access at this offset/width"
            elif ev["sign"] & {"cmp:s", "cmp:u", "sar", "shr", "div", "fild"}:
                direct, reason = "DIR", ",".join(sorted(ev["sign"]))
            elif ev["sign"]:
                direct, reason = "WEAK", "extension only: " + ",".join(sorted(ev["sign"]))
            else:
                direct, reason = "SKIP", "blob: sign-neutral (%s)" % ",".join(sorted(ev["all"]))
            self.sites["C%d" % a["k"]] = dict(kind="acc", k=a["k"], rp=a["rp"], w=a["w"], name=nm)
            # 8/16-bit loads promote to int either way, so the swap changes the VALUE only when the top bit is set (sound);
            # 32-bit swaps also change the type of the other operand's conversions -> no value-based claim
            self.emit("SIGNA", ["SIGN", "SIGNA", direct], f, t.s, [(t.s, t.e, nn)], "%s -> %s (%s)" % (nm, nn, reason), direct, "C%d" % a["k"],
                      "top" if a["w"] in (8, 16) else None)
        # ---- WIDTH: accessor widening/narrowing (noisy, opt-in class)
        for a in acc:
            if a["sgn"] == "P":
                continue
            nn = ACC_SWAP_WIDTH.get((a["sgn"], a["w"]))
            if nn is None:
                continue
            t = toks[a["k"]]
            self.emit("WIDTH", ["WIDTH", "store" if a["store"] else "load"], f, t.s, [(t.s, t.e, nn)], "%s -> %s" % (a["name"], nn), "NOISY")
        # ---- CMPSIGN: compare signedness flips (blob directed; pointer compares always)
        accmap = {}
        for a in acc:
            accmap[a["k"]] = a
        for (k, l, r) in relops:
            t = toks[k]
            for lo, hi in (l, r):
                if lo > hi:
                    break
            else:
                ptr = self.is_ptr_span(f, l) or self.is_ptr_span(f, r)
                classes, basis = set(), []
                for (lo, hi) in (l, r):
                    for a in acc:
                        if lo <= a["k"] <= hi and a["ev"] and a["ev"]["found"]:
                            classes |= a["ev"]["cmp"]
                            basis.append("acc")
                lit_classes = set()
                if self.ev is not None and self.ev.has_func(f.name):
                    for (lo, hi) in (l, r):
                        for q in range(lo, hi + 1):
                            if toks[q].kind == "num":
                                v = num_value(toks[q].text)
                                if v is not None and abs(v) > 2:          # 0/1/2 are compared in every signedness: no evidence
                                    lit_classes |= self.ev.const_classes(f.name, v)
                if ptr:
                    want, direct, reason = ("s", "u"), "PTR", "pointer/address-width compare (always)"
                elif classes:
                    want = {"s": ("u",), "u": ("s",)}.get(next(iter(classes)), ("s", "u")) if len(classes) == 1 else ("s", "u")
                    direct, reason = "DIR", "blob cmp class at accessor: %s" % ",".join(sorted(classes))
                elif lit_classes:
                    want = ("u",) if lit_classes == {"s"} else (("s",) if lit_classes == {"u"} else ("s", "u"))
                    direct, reason = "DIR", "blob cmp class at literal: %s" % ",".join(sorted(lit_classes))
                else:
                    want, direct, reason = ("s", "u"), "UNDIRECTED", "no blob compare evidence"
                for w in want:
                    ty = "uint32_t" if w == "u" else "int32_t"
                    edits = [(toks[l[0]].s, toks[l[0]].s, "(%s)(" % ty), (toks[l[1]].e, toks[l[1]].e, ")"),
                             (toks[r[0]].s, toks[r[0]].s, "(%s)(" % ty), (toks[r[1]].e, toks[r[1]].e, ")")]
                    tags = ["SIGN", "CMPSIGN", direct] + (["PTR"] if ptr else [])
                    self.add_rel_site(f, k)
                    self.emit("CMPSIGN", tags, f, t.s, edits, "compare as %s (%s)" % (ty, reason), direct, "R%d" % k, "sd")
        # ---- casts
        for c in self.m.casts:
            if c["func"] != f.name:
                continue
            ts, te = c["ts"], c["te"]
            core = type_core_span(m, ts, te)
            if core is None:
                continue
            names = [toks[q].text for q in range(ts, te)]
            tg = toggled_type(names)
            is_ptrtype = any(n in PTRISH_TYPES for n in names)
            ctx = in_consume(c["lp"])
            if is_ptrtype:
                tags_extra = ["PTR"]
            else:
                tags_extra = []
            directed = bool(ctx) or is_ptrtype
            cw = CAST_W.get(" ".join(n for n in names if n not in QUAL_KW))
            vsite = vneed_f = vneed_d = None
            if c["nptr"] == 0 and cw is not None and c["rp"] + 1 < len(toks):
                ue = unary_end(m, c["rp"] + 1)
                if ue is not None:
                    vsite = "V%d" % c["lp"]
                    self.sites[vsite] = dict(kind="cv", lp=c["lp"], rp=c["rp"], opnd=(c["rp"] + 1, ue), w=cw, ty=" ".join(names))
                    vneed_f = "top" if cw in (8, 16) else None      # 32-bit flips also retype the other operand (no value-based claim)
                    vneed_d = None                                  # a dropped cast also retypes the expression
            if tg is not None and c["nptr"] == 0:
                cs, ce = toks[core[0]].s, toks[core[1]].e
                self.emit_noisy(f, "CAST", ["CAST", "CAST-flip"] + tags_extra, toks[c["lp"]].s, [(cs, ce, tg)], "cast (%s) -> (%s)" % (" ".join(names), tg), directed, ctx, vsite, vneed_f)
            if c["nptr"] == 0:
                self.emit_noisy(f, "CAST", ["CAST", "CAST-drop"] + tags_extra, toks[c["lp"]].s, [(toks[c["lp"]].s, toks[c["rp"]].e, "")], "drop cast (%s)" % " ".join(names), directed, ctx, vsite, vneed_d)
        # ---- declared-type sign / width flips (noisy, opt-in)
        for d in self.m.decls:
            if d["func"] != f.name or d["nptr"]:
                continue
            ts, te = d["ts"], d["te"]
            core = type_core_span(m, ts, te)
            if core is None:
                continue
            names = [toks[q].text for q in range(ts, te)]
            tg = toggled_type(names)
            nm = toks[d["name_i"]].text
            cs, ce = toks[core[0]].s, toks[core[1]].e
            if tg is not None:
                self.emit("SIGN", ["SIGN", "SIGN-decl"] + (["param"] if d["param"] else []), f, cs, [(cs, ce, tg)], "decl %s %s -> %s" % (" ".join(names), nm, tg), "NOISY")
            if len(names) == 1 and names[0] in WIDTH_FLIP and not d["param"]:
                self.emit("WIDTH", ["WIDTH", "decl"], f, cs, [(cs, ce, WIDTH_FLIP[names[0]])], "decl %s %s -> %s" % (names[0], nm, WIDTH_FLIP[names[0]]), "NOISY")

    def emit_noisy(self, f, cls, tags, pos, edits, note, directed, ctx, site=None, need=None):
        """cast flips / drops: class CAST; direct=DIR when inside a sign-consuming operand (relational, >> left, / %) or
        pointer-width, else NOISY (not run by default)."""
        tags = tags + ["SIGN", ("in-" + ctx[0]) if ctx else "no-sign-context"]
        self.emit(cls, tags, f, pos, edits, note, "DIR" if directed else "NOISY", site, need)

    def is_ptr_span(self, f, span):
        """does the operand span look pointer-valued?  Identifiers declared with a '*' (or params with '*') count unless they are
        dereferenced / indexed / used as an accessor base; `PTR(` accessors and (u)intptr_t / size_t casts count."""
        toks, mp = self.m.toks, self.m.mp
        lo, hi = span
        q = lo
        while q <= hi:
            t = toks[q]
            if t.kind == "id" and t.text in self.m.accessors and t.text != "PTR" and q + 1 <= hi and toks[q + 1].text == "(" and (q + 1) in mp:
                q = mp[q + 1] + 1
                continue
            if t.kind == "id" and t.text in ("OBJ_BYTES", "PTR"):
                return True
            if t.kind == "id" and t.text in PTRISH_TYPES:
                return True
            if t.kind == "id" and t.text in f.ptr_vars:
                nx = toks[q + 1].text if q + 1 < len(toks) else ""
                pv = toks[q - 1].text if q > 0 else ""
                if nx != "[" and not (pv == "*" and not operand_end_tok(self.m, q - 2)):
                    return True
            q += 1
        return False


# ====================================================================================================================
# 4. blob evidence (from dsplibs.o machine code, through asmdiff's symbolic executor)
# ====================================================================================================================
JCC_S = {"jl", "jle", "jg", "jge", "js", "jns", "jnge", "jnl", "jng", "jnle", "setl", "setle", "setg", "setge", "setnl", "setng",
         "setnge", "setnle", "sets", "setns", "cmovl", "cmovle", "cmovg", "cmovge", "cmovs", "cmovns"}
JCC_U = {"jb", "jbe", "ja", "jae", "jc", "jnc", "jnae", "jnb", "jna", "jnbe", "setb", "setbe", "seta", "setae", "setc", "setnc",
         "setnae", "setnb", "setna", "setnbe", "cmovb", "cmovbe", "cmova", "cmovae"}
JCC_E = {"je", "jne", "jz", "jnz", "sete", "setne", "setz", "setnz"}


class BlobEvidence:
    """Per blob function: resolved object accesses (obj+offset, width, extension) with the sign-consuming operations that
    each loaded value reaches (straight-line provenance), and the signed/unsigned jcc class of every compare constant."""

    def __init__(self, blob_path, funcmap):
        import asmdiff as A
        self.A = A
        self.blob_path = blob_path
        self.blob = A.Obj(blob_path)
        self.funcmap = funcmap          # port function name -> blob function name
        self.data = {}
        for bname in sorted(set(funcmap.values())):
            sym = self.blob.func(bname)
            if sym is None:
                continue
            self.data[bname] = self._analyse(sym)

    def has_func(self, pname):
        return self.funcmap.get(pname) in self.data

    def _analyse(self, sym):
        A = self.A
        F = A.extract(self.blob, self.blob_path, sym, follow=None)
        insns = A.disasm_func(self.blob, self.blob_path, sym)
        recs = {}
        for r in F.acc:
            if r["kind"] in ("r", "rw"):
                recs.setdefault(r["ins"], r)
        tags = collections.defaultdict(set)
        consts = collections.defaultdict(set)
        prov = {}

        def reg_of(op):
            return A.PARENT.get(op) if op in A.PARENT else None

        def cls_after(k):
            for j in range(k + 1, min(k + 4, len(insns))):
                m = insns[j].mn
                if m in JCC_S:
                    return "s"
                if m in JCC_U:
                    return "u"
                if m in JCC_E:
                    return "e"
                if not (m.startswith("mov") or m in ("lea", "nop")):
                    break
            return None

        for k, i in enumerate(insns):
            mn = i.mn
            ops = A.split_ops(i.ops)
            r = recs.get(i.addr)
            srcs = set()
            for o in ops:
                rr = reg_of(o)
                if rr is not None and rr in prov:
                    srcs |= prov[rr]
            if r is not None:
                srcs.add(id(r))
                if r["ext"] in ("s", "u"):
                    tags[id(r)].add("ext:" + r["ext"])
                if r["mn"] in ("fild", "fiadd", "fisub", "fimul", "fidiv", "ficom", "ficomp"):
                    tags[id(r)].add("fild")
                tags[id(r)].add("ext:n" if r["ext"] == "n" else "ext:" + r["ext"])
            if mn in ("cmp", "test"):
                c = cls_after(k)
                if c:
                    for s_ in srcs:
                        tags[s_].add("cmp:" + c)
                    for o in ops:
                        v = A.imm_of(o)
                        if v is not None:
                            for w in (8, 16, 32):
                                consts[A.sx(v, 32) if v < 2**31 else v].add(c)
                            consts[v].add(c)
                            consts[v & 0xffffffff].add(c)
                            consts[A.sx(v & 0xffffffff, 32)].add(c)
                continue
            if mn in ("sar", "shr", "sal", "shl", "idiv", "div", "imul", "mul", "cdq", "cwd", "cwde"):
                tag = {"sar": "sar", "shr": "shr", "idiv": "div", "div": "div"}.get(mn)
                if tag:
                    tg = set()
                    if mn in ("idiv", "div"):
                        tg = set(prov.get("eax", set())) | set(prov.get("edx", set())) | srcs
                    else:
                        d = reg_of(ops[0]) if ops else None
                        tg = set(prov.get(d, set())) if d else set()
                    for s_ in tg:
                        tags[s_].add(tag)
            # provenance transfer
            if not ops:
                continue
            d = reg_of(ops[0])
            if mn in ("call",):
                for x in ("eax", "ecx", "edx"):
                    prov.pop(x, None)
                continue
            if d is None:
                continue
            if mn in ("mov", "movzx", "movsx", "movzbl", "movsbl", "movzwl", "movswl", "lea"):
                if mn == "lea":
                    prov.pop(d, None)
                else:
                    prov[d] = set(srcs) - (set(prov.get(d, set())) if len(ops) == 2 and ops[1] == ops[0] else set()) if len(ops) == 2 else set()
            elif mn == "xor" and len(ops) == 2 and ops[0] == ops[1]:
                prov.pop(d, None)
            elif mn in ("add", "sub", "adc", "sbb", "and", "or", "xor", "imul", "shl", "sal", "shr", "sar", "neg", "not", "inc", "dec", "cmovne", "cmove", "cdq", "cwde"):
                prov[d] = set(prov.get(d, set())) | srcs
            elif mn in ("cdq", "cwd"):
                prov["edx"] = set(prov.get("eax", set()))
            elif mn.startswith("set") or mn == "pop":
                prov.pop(d, None)
        acc = []
        for r in F.acc:
            t = set(tags.get(id(r), set()))
            acc.append(dict(base=r["base"], off=r["off"], w=r["width"], idx=r["idx"], kind=r["kind"], ext=r["ext"], tags=t))
        return dict(acc=acc, consts=consts, nunk=F.unk_acc)

    SIGN_TAGS = {"ext:s", "ext:u", "cmp:s", "cmp:u", "sar", "shr", "div", "fild"}

    def acc_evidence(self, pname, pidx, off, width, indexed):
        d = self.data.get(self.funcmap.get(pname))
        if d is None:
            return None
        base = "a%d" % pidx
        exact, near = [], []
        for r in d["acc"]:
            if r["base"] != base:
                continue
            if indexed:
                if r["idx"] is not None and abs(r["off"] - off) <= 64:
                    (exact if r["w"] == width and abs(r["off"] - off) <= 16 else near).append(r)
            else:
                if r["idx"] is None and r["off"] <= off + width // 8 - 1 and off <= r["off"] + max(1, r["w"] // 8) - 1:
                    (exact if (r["off"] == off and r["w"] == width) else near).append(r)
        use = exact or near
        alltags = set()
        for r in use:
            alltags |= r["tags"]
        return dict(found=bool(use), exact=bool(exact), all=alltags, sign=alltags & self.SIGN_TAGS,
                    cmp={t[-1] for t in alltags if t.startswith("cmp:") and t[-1] in "su"})

    def const_classes(self, pname, v):
        d = self.data.get(self.funcmap.get(pname))
        if d is None:
            return set()
        out = set()
        for c in (v, v - 1, v + 1):
            out |= {x for x in d["consts"].get(c, set()) if x in "su"}
        return out


# ====================================================================================================================
# 5. build plumbing: record the chunk's own build script with a gcc wrapper, then replay only the port-dependent steps
# ====================================================================================================================
WRAPPER = r'''#!/usr/bin/env python3
import json, os, sys
with open(os.environ["MUTATE_GCC_LOG"], "a") as f:
    f.write(json.dumps({"cwd": os.getcwd(), "argv": sys.argv[1:], "cc": os.path.basename(sys.argv[0])}) + "\n")
cc = "/usr/bin/" + os.path.basename(sys.argv[0])
os.execv(cc, [cc] + sys.argv[1:])
'''
CXX_EXT = (".cpp", ".cc", ".cxx")


def run(cmd, cwd=None, env=None, timeout=None, input_=None):
    p = subprocess.run(cmd, cwd=cwd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
    return p


def arg_after(argv, flag):
    return argv[argv.index(flag) + 1] if flag in argv else None


def replace_arg(argv, old, new):
    return [new if a == old else a for a in argv]


def canon_asm(text):
    """canonical form of gcc -S output: drop comments / .file / .ident, rename .L labels by first appearance"""
    lines = []
    for ln in text.splitlines():
        s = ln.strip()
        if not s or s.startswith("#") or s.startswith(".file") or s.startswith(".ident"):
            continue
        s = re.sub(r"\s+#.*$", "", s)
        lines.append(s)
    body = "\n".join(lines)
    names = {}

    def rn(mo):
        k = mo.group(0)
        if k not in names:
            names[k] = ".L%d" % len(names)
        return names[k]

    return re.sub(r"\.L[A-Za-z]*\d+", rn, body)


class Plan:
    """Everything needed to build + run one mutant: compile/link templates recovered from a recorded base build."""

    def __init__(self, a, outdir):
        self.a = a
        self.root = os.path.abspath(a.root)
        self.out = os.path.abspath(outdir)
        self.work = os.path.join(self.out, "work")
        os.makedirs(self.work, exist_ok=True)
        self.port_src = os.path.abspath(a.port)
        self.script = a.script if os.path.isabs(a.script) else os.path.join(self.root, a.script)
        self.scenarios = None
        self.info = {}

    # ---- base build (recorded)
    def prepare(self):
        pf = os.path.join(self.out, "plan.json")
        if os.path.exists(pf) and not self.a.rebuild:
            self.info = json.load(open(pf))
            self.scenarios = self.info["scenarios"]
            if all(os.path.exists(p) for p in self.info["outputs_check"]):
                return
        wrap_dir = os.path.join(self.work, "wrap")
        os.makedirs(wrap_dir, exist_ok=True)
        for wn in ("gcc", "g++"):                 # g++: C++ ports (src/*.cpp) are compiled by the chunk script with g++
            wp = os.path.join(wrap_dir, wn)
            open(wp, "w").write(WRAPPER)
            os.chmod(wp, 0o755)
        log = os.path.join(self.work, "gcc.log")
        if os.path.exists(log):
            os.remove(log)
        base = os.path.join(self.work, "base")
        os.makedirs(base, exist_ok=True)
        env = dict(os.environ)
        env.update(PATH=wrap_dir + os.pathsep + env["PATH"], MUTATE_GCC_LOG=log, TPREFIX=os.path.join(base, "chunk"), SCENARIOS=" ")
        env.pop("NOO2", None)
        env[self.a.port_env] = self.port_src
        for kv in self.a.env or []:
            k, v = kv.split("=", 1)
            env[k] = v
        t0 = time.time()
        p = run(["sh", self.script], cwd=self.root, env=env)
        sys.stderr.write("[prepare] base build of %s: rc=%d %.1fs\n" % (os.path.basename(self.script), p.returncode, time.time() - t0))
        if p.returncode != 0 or b"MATCH" not in p.stdout:
            sys.stderr.write(p.stdout.decode(errors="replace")[-2000:] + p.stderr.decode(errors="replace")[-2000:])
            raise SystemExit("base build failed")
        cmds = [json.loads(l) for l in open(log)]
        ptr = os.path.realpath(self.port_src)

        def is_port(a, cwd):
            return a.endswith((".c",) + CXX_EXT) and os.path.realpath(os.path.join(cwd, a)) == ptr

        ccmd = None
        for c in cmds:
            av = c["argv"]
            if "-c" in av and any(is_port(a, c["cwd"]) for a in av) and "-O2" in av:
                # a script may compile the port several times at -O2 (e.g. an extra -m64 build): prefer the one that
                # produces the *_port_o2.o object the -O2 test binary is linked from (the naming convention of the scripts)
                if ccmd is None or (arg_after(av, "-o") or "").endswith("_port_o2.o") or not (arg_after(ccmd["argv"], "-o") or "").endswith("_port_o2.o"):
                    ccmd = c
        if ccmd is None:
            raise SystemExit("could not find the -O2 compile of the port source in the recorded build (is --port-env right / does the script build -O2?)")
        oout = arg_after(ccmd["argv"], "-o")
        lcmd = None
        for c in cmds:
            av = c["argv"]
            if "-c" not in av and "-o" in av and oout in av:
                lcmd = c
        if lcmd is None:
            raise SystemExit("could not find the link command that consumes %s" % oout)
        blobcmd = None
        for c in cmds:
            av = c["argv"]
            if "-c" not in av and "-o" in av and arg_after(av, "-o").endswith("_test_blob"):
                blobcmd = c
        if blobcmd is None:
            raise SystemExit("could not find the blob test binary link")
        self.info = dict(
            cwd=ccmd["cwd"], compile_argv=ccmd["argv"], cc=ccmd.get("cc", "gcc"), port_arg=[a for a in ccmd["argv"] if is_port(a, ccmd["cwd"])][0],
            obj_arg=oout, link_argv=lcmd["argv"], blob_bin=os.path.join(blobcmd["cwd"], arg_after(blobcmd["argv"], "-o")),
            scenarios=self.a.scenarios.split() if self.a.scenarios else self.parse_scenarios(),
            outputs_check=[])
        self.scenarios = self.info["scenarios"]
        # precompile every .c source in the link command once (incremental build: only the mutated object is rebuilt later)
        pre = os.path.join(self.work, "pre")
        os.makedirs(pre, exist_ok=True)
        lav = lcmd["argv"]
        flags = []
        k = 0
        while k < len(lav):
            x = lav[k]
            if x == "-o":
                k += 2
                continue
            if x.startswith("-") and not x.startswith(("-l", "-L")):
                flags.append(x)
            k += 1
        self.info["link_flags"] = flags
        newl = []
        jobs = []
        k = 0
        while k < len(lav):
            x = lav[k]
            if x == "-o":
                newl += [x, lav[k + 1]]
                k += 2
                continue
            if x.endswith(".c"):
                ob = os.path.join(pre, "%s_%d.o" % (os.path.basename(x)[:-2], len(jobs)))
                jobs.append((x, ob))
                newl.append(ob)
            else:
                newl.append(x)
            k += 1
        cflags = [f for f in flags if not f.startswith(("-Wl,", "-static", "-pie", "-no-pie"))]
        with cf.ThreadPoolExecutor(self.a.jobs) as ex:
            futs = [ex.submit(run, ["gcc"] + cflags + ["-c", s, "-o", o], cwd=lcmd["cwd"]) for s, o in jobs]
            for (s, o), fu in zip(jobs, futs):
                r = fu.result()
                if r.returncode != 0:
                    raise SystemExit("precompile of %s failed:\n%s" % (s, r.stderr.decode(errors="replace")[-2000:]))
        self.info["link_argv_pre"] = newl
        # the base build's other artefacts (prefixed table objects, stub_env.o ...) live under work/base: keep
        self.info["outputs_check"] = [o for _, o in jobs] + [self.info["blob_bin"]]
        json.dump(self.info, open(pf, "w"), indent=1)

    def parse_scenarios(self):
        t = open(self.script).read()
        mo = re.search(r'SCENARIOS="\$\{SCENARIOS:-([^}]*)\}"', t)
        if not mo:
            raise SystemExit("cannot parse the scenario list from %s; pass --scenarios" % self.script)
        return mo.group(1).split()

    # ---- per-mutant commands
    def compile_cmd(self, src, obj, asm=False):
        av = list(self.info["compile_argv"])
        av = [src if a == self.info["port_arg"] else a for a in av]
        k = av.index("-o")
        av[k + 1] = obj
        if asm:
            av[av.index("-c")] = "-S"
            av = [a for a in av if not (a.startswith("-g") and len(a) <= 3)] + ["-w"]
        av += ["-I", os.path.dirname(self.port_src)]
        return [self.info.get("cc", "gcc")] + av

    def link_cmd(self, pobj, out):
        av = list(self.info["link_argv_pre"])
        av = [pobj if a == self.info["obj_arg"] else a for a in av]
        k = av.index("-o")
        av[k + 1] = out
        return ["gcc"] + av


def run_scen(binary, scen, cwd, timeout, env=None):
    """-> (status, stdout_bytes, detail).  status: ok | exit | signal | timeout"""
    p = subprocess.Popen([binary, str(scen)], cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, start_new_session=True, env=env)
    try:
        out, _ = p.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        p.communicate()
        return "timeout", b"", "timeout %ds" % timeout
    finally:
        try:
            os.killpg(p.pid, signal.SIGKILL)       # orphaned fork-probe children
        except (ProcessLookupError, PermissionError):
            pass
    if p.returncode < 0:
        return "signal", out, "signal %d" % -p.returncode
    if p.returncode != 0:
        return "exit", out, "exit %d" % p.returncode
    return "ok", out, ""


class Runner:
    def __init__(self, plan, a):
        self.plan, self.a = plan, a
        self.lock = threading.Lock()
        self.blob = {}            # scen -> dict(out=bytes, t=seconds)
        self.kills = collections.Counter()
        self.tries = collections.Counter()
        self.cost = {}

    def blob_outputs(self):
        pl = self.plan
        d = os.path.join(pl.out, "blob")
        os.makedirs(d, exist_ok=True)
        todo = []
        for s in pl.scenarios:
            fp = os.path.join(d, "s%s.out" % s)
            mf = os.path.join(d, "s%s.json" % s)
            if os.path.exists(fp) and os.path.exists(mf) and not self.a.rebuild:
                self.blob[s] = dict(out=open(fp, "rb").read(), t=json.load(open(mf))["t"])
            else:
                todo.append(s)

        def one(s):
            t0 = time.time()
            st, out, det = run_scen(pl.info["blob_bin"], s, pl.info["cwd"], 3600)
            return s, st, out, det, time.time() - t0

        if todo:
            sys.stderr.write("[blob] running %d scenario(s) of the real blob: %s\n" % (len(todo), " ".join(todo)))
            with cf.ThreadPoolExecutor(min(self.a.jobs, len(todo))) as ex:
                for s, st, out, det, t in ex.map(one, todo):
                    if st != "ok":
                        raise SystemExit("blob scenario %s did not exit 0: %s %s" % (s, st, det))
                    open(os.path.join(d, "s%s.out" % s), "wb").write(out)
                    json.dump(dict(t=t, n=len(out)), open(os.path.join(d, "s%s.json" % s), "w"))
                    self.blob[s] = dict(out=out, t=t)
        for s in pl.scenarios:
            self.cost[s] = max(self.blob[s]["t"], 0.05)

    def order(self):
        with self.lock:
            def key(s):
                p = (self.kills[s] + 0.5) / (self.tries[s] + 1.0)
                return self.cost[s] / p
            return sorted(self.plan.scenarios, key=key)

    def timeout_for(self, s):
        return int(max(20, 6 * self.cost[s] + 10))

    def eval_mutant(self, mid, src_text, wdir):
        """build the mutant at -O2 (+ link) and run the scenarios; returns result dict"""
        pl = self.plan
        os.makedirs(wdir, exist_ok=True)
        base = os.path.basename(pl.port_src)
        ms = os.path.join(wdir, base)
        open(ms, "w").write(src_text)
        obj = os.path.join(wdir, "m.o")
        exe = os.path.join(wdir, "m_test")
        t0 = time.time()
        r = run(pl.compile_cmd(ms, obj), cwd=pl.info["cwd"])
        if r.returncode != 0:
            return dict(id=mid, status="invalid", err=r.stderr.decode(errors="replace")[:300])
        r = run(pl.link_cmd(obj, exe), cwd=pl.info["cwd"])
        if r.returncode != 0:
            return dict(id=mid, status="buildfail", err=r.stderr.decode(errors="replace")[:300])
        tb = time.time() - t0
        t1 = time.time()
        killed = None
        ran = 0
        for s in self.order():
            st, out, det = run_scen(exe, s, pl.info["cwd"], self.timeout_for(s))
            ran += 1
            reason = None
            if st != "ok":
                reason = det
            elif out != self.blob[s]["out"]:
                reason = "stdout differs"
            with self.lock:
                self.tries[s] += 1
                if reason:
                    self.kills[s] += 1
            if reason:
                killed = (s, reason)
                break
        for f in (obj, exe):
            try:
                os.remove(f)
            except OSError:
                pass
        res = dict(id=mid, status="killed" if killed else "survived", t_build=round(tb, 2), t_run=round(time.time() - t1, 2), nscen=ran)
        if killed:
            res["by"] = "s%s: %s" % killed
        return res


# ====================================================================================================================
# 6. pipeline
# ====================================================================================================================
DEFAULT_OFF = {"NOISY", "SKIP"}


def policy_ok(mu, a):
    """the default 'directed' selection: everything except blob-sign-neutral accessor swaps and undirected noisy classes"""
    d = mu["direct"]
    if d == "SKIP" and not a.include_skipped:
        return False
    if d == "WEAK" and not getattr(a, "include_weak", False) and not a.include_skipped:
        return False
    if d == "NOISY" and not a.include_noisy:
        return False
    if d == "UNDIRECTED" and a.undirected == "skip":
        return False
    return True


def class_ok(mu, a):
    if not a.classes:
        return True
    want = set(x.strip() for x in a.classes.split(",") if x.strip())
    inc = {w for w in want if not w.startswith("-")}
    exc = {w[1:] for w in want if w.startswith("-")}
    names = set(mu["tags"]) | {mu["cls"]}
    if exc & names:
        return False
    return (not inc) or bool(inc & names)


def read_port(a):
    """The source of --port.  A requested function that the file does not define but a library header it includes does (RS2: the CRC helpers
    crcRun / crcDiffSum / crcOnes moved to src/vpcm/pcm_cp_crc.h, included by v90_cp / v92_cp / v90_mp; RS8: many more helper headers) is made
    visible by expanding that `#include "x.h"` line in place with the header's text (the header's own relative quote-includes are rewritten to
    absolute paths).  Mutants then edit the expanded text; the compile of the mutated copy is unchanged (same declarations, same order).  A
    port that defines all its functions itself is returned untouched."""
    src = open(a.port).read()
    want = list(getattr(a, "func", None) or [])
    if not want:
        return src
    root = os.path.abspath(a.root)
    pdir = os.path.dirname(os.path.abspath(a.port))
    done = set()
    for _ in range(4):
        missing = [f for f in want if f not in Model(src).funcs]
        if not missing:
            break
        changed = False
        for mo in re.finditer(r'^#include "([^"]+)"[^\n]*$', src, re.M):
            inc = mo.group(1)
            for base in (pdir, os.path.join(root, "src")):
                hp = os.path.join(base, inc)
                if os.path.isfile(hp):
                    break
            else:
                continue
            hp = os.path.abspath(hp)
            if hp in done:
                continue
            hs = open(hp).read()
            if not any(f in Model(hs).funcs for f in missing):
                continue
            hdir = os.path.dirname(hp)
            hs = re.sub(r'^#include "([^"]+)"', lambda m2: '#include "%s"' % (os.path.join(hdir, m2.group(1)) if os.path.isfile(os.path.join(hdir, m2.group(1))) else m2.group(1)), hs, flags=re.M)
            src = src[:mo.start()] + "/* mutate.py: the include of %s expanded here (a requested function is defined in it) */\n" % inc + hs + src[mo.end():]
            done.add(hp)
            changed = True
            break
        if not changed:
            break
    return src


def gen_mutants(a, src, quiet=False):
    m = Model(src)
    funcs = a.func
    for f in funcs:
        if f not in m.funcs:
            raise SystemExit("function %s not found in %s (have: %s)" % (f, a.port, ", ".join(m.funcs)))
    ev = None
    if not a.no_evidence:
        fm = {}
        for spec in funcs:
            fm[spec] = spec
        for kv in a.blob_func or []:
            p, b = kv.split("=", 1)
            fm[p] = b
        # only those that exist in the blob
        blob = os.path.abspath(a.blob) if a.blob else os.path.abspath(BLOB_O)
        import asmdiff
        bo = asmdiff.Obj(blob)
        fm = {p: b for p, b in fm.items() if bo.func(b)}
        if fm:
            ev = BlobEvidence(blob, fm)
    # every function in the file that calls/is called also participates in base resolution (helpers without blob namesake)
    g = Generator(m, funcs, ev, lit_deltas=tuple(int(x) for x in a.lit_deltas.split(",")))
    res = g.run()
    return m, g, ev, res


def mutant_display(m, mu):
    """(original line text, mutated line text) for the first edited line"""
    src = m.src
    ms = apply_edits(src, [tuple(e) for e in mu["edits"]])
    ln = mu["line"]
    # mutated source has the same number of lines as the original (edits never add newlines)
    def line_of_text(t, n):
        ls = t.split("\n")
        return ls[n - 1] if 0 < n <= len(ls) else ""
    return line_of_text(src, ln).strip(), line_of_text(ms, ln).strip()


def load_jsonl(p):
    out = []
    if os.path.exists(p):
        for l in open(p):
            l = l.strip()
            if l:
                try:
                    out.append(json.loads(l))
                except ValueError:
                    pass
    return out


def do_run(a):
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    src = read_port(a)
    m, g, ev, muts = gen_mutants(a, src)
    allm = [x.to_json() for x in muts]
    hsrc = hashlib.sha1(src.encode()).hexdigest()[:12]
    sel = [x for x in allm if class_ok(x, a) and policy_ok(x, a)]
    not_sel = len(allm) - len(sel)
    if a.ids:
        want = set(open(a.ids[1:]).read().split()) if a.ids.startswith("@") else set(a.ids.split(","))
        sel = [x for x in sel if x["id"] in want]
    if a.survivors_of:
        prev = a.survivors_of
        if os.path.basename(prev) != "final.jsonl" and os.path.exists(os.path.join(os.path.dirname(prev), "final.jsonl")):
            prev = os.path.join(os.path.dirname(prev), "final.jsonl")
        ids = {r["id"] for r in load_jsonl(prev) if r.get("status") in ("survived", "novalue")}
        sel = [x for x in allm if x["id"] in ids]        # survivors are re-run even if the current policy would exclude them
        sys.stderr.write("[survivors-of] %d survivors in %s -> %d regenerated\n" % (len(ids), prev, len(sel)))
    if a.sample and a.sample < len(sel):
        rnd = random.Random(a.seed)
        bycls = collections.defaultdict(list)
        for x in sel:
            bycls[x["cls"]].append(x)
        pick = []
        for c, l in sorted(bycls.items()):
            k = max(1, round(a.sample * len(l) / len(sel)))
            pick += rnd.sample(l, min(k, len(l)))
        sel = pick
    sys.stderr.write("[gen] %d generated, %d selected by policy/classes/sample\n" % (len(allm), len(sel)))
    # metadata for exactly the selected ones
    meta_f = os.path.join(out, "meta.json")
    json.dump(dict(port=os.path.abspath(a.port), src_hash=hsrc, funcs=a.func, generated=len(allm), selected=len(sel), argv=sys.argv), open(meta_f, "w"), indent=1)
    pl = Plan(a, out)
    pl.prepare()
    rn = Runner(pl, a)
    rn.blob_outputs()
    workd = lambda k: os.path.join(out, "work", "w%d" % k)
    # ---------------- control (unmutated) through the same replay pipeline
    ctl_f = os.path.join(out, "control.json")
    if not os.path.exists(ctl_f) or a.rebuild:
        sys.stderr.write("[control] unmutated port through the replay pipeline ...\n")
        # run ALL scenarios (no early exit on the first kill is needed: it must match everywhere)
        r = rn.eval_mutant("control", src, workd(0))
        if r["status"] != "survived":
            raise SystemExit("CONTROL FAILED (unmutated port is not identical to the blob through the replay pipeline): %s" % r)
        json.dump(r, open(ctl_f, "w"))
    # ---------------- phase A: compile -S, invalid / TCE-equivalent / asm hash
    ma_f = os.path.join(out, "mutants.jsonl")
    done = {r["id"]: r for r in load_jsonl(ma_f)}
    for x in allm:                      # labels are heuristics: refresh them (and site/need) on resumed records
        if x["id"] in done:
            for k in ("cls", "tags", "direct", "note", "site", "need"):
                done[x["id"]][k] = x[k]
    # original asm
    orig_s = os.path.join(out, "work", "orig.s")
    r = run(pl.compile_cmd(os.path.abspath(a.port), orig_s, asm=True), cwd=pl.info["cwd"])
    if r.returncode != 0:
        raise SystemExit("original does not compile -S: " + r.stderr.decode(errors="replace")[:500])
    orig_hash = hashlib.sha1(canon_asm(open(orig_s).read()).encode()).hexdigest()[:16]
    todoA = [x for x in sel if x["id"] not in done]
    t0 = time.time()
    lockw = threading.Lock()
    wq = collections.deque(range(a.jobs))

    def phaseA(x):
        with lockw:
            k = wq.popleft()
        try:
            wd = os.path.join(out, "work", "wa%d" % k)
            os.makedirs(wd, exist_ok=True)
            ms = os.path.join(wd, os.path.basename(a.port))
            open(ms, "w").write(apply_edits(src, [tuple(e) for e in x["edits"]]))
            asm = os.path.join(wd, "m.s")
            r = run(pl.compile_cmd(ms, asm, asm=True), cwd=pl.info["cwd"])
            rec = dict(x)
            if r.returncode != 0:
                rec["status"] = "invalid"
                rec["err"] = (r.stderr.decode(errors="replace").strip().splitlines() or [""])[-1][:160]
            else:
                h = hashlib.sha1(canon_asm(open(asm).read()).encode()).hexdigest()[:16]
                rec["h"] = h
                rec["status"] = "equiv" if h == orig_hash else "live"
            return rec
        finally:
            with lockw:
                wq.append(k)

    if todoA:
        sys.stderr.write("[tce] compiling %d mutants at -O2 -S ...\n" % len(todoA))
        with cf.ThreadPoolExecutor(a.jobs) as ex, open(ma_f, "a") as fo:
            for n, rec in enumerate(ex.map(phaseA, todoA)):
                fo.write(json.dumps(rec) + "\n")
                fo.flush()
                done[rec["id"]] = rec
        sys.stderr.write("[tce] done in %.1fs\n" % (time.time() - t0))
    # ---------------- gcov (optional)
    if a.gcov:
        do_gcov(a, pl, rn, out, src)
    # ---------------- value probes (optional)
    pr = None
    if a.probes:
        pr = do_probes(a, pl, rn, out, src, m, g.sites)
    verdict = {}
    if pr:
        for x in sel:
            verdict[x["id"]] = probe_verdict(done[x["id"]], pr)
    # ---------------- phase B
    res_f = os.path.join(out, "results.jsonl")
    results = {r["id"]: r for r in load_jsonl(res_f)}
    selids = {x["id"] for x in sel}
    live = [done[i] for i in (x["id"] for x in sel) if done[i]["status"] == "live" and not (verdict.get(i) == "NOVALUE" and not a.no_probe_skip)]
    byh = collections.OrderedDict()
    for r in live:
        byh.setdefault(r["h"], []).append(r["id"])
    # a hash already evaluated (by any member of the group) is not re-run
    hres = {}
    for rid, r in results.items():
        if rid in done and "h" in done[rid]:
            hres[done[rid]["h"]] = r
    reps = [(h, ids[0]) for h, ids in byh.items() if h not in hres]
    # optional: validate TCE by running a sample of the TCE-equivalent mutants too (they must all survive)
    tce_check = []
    if a.tce_check:
        eq = [i for i in (x["id"] for x in sel) if done[i]["status"] == "equiv"]
        rnd = random.Random(a.seed + 1)
        tce_check = rnd.sample(eq, min(a.tce_check, len(eq)))
    if a.probe_check and pr:
        nv = [i for i in (x["id"] for x in sel) if done[i]["status"] == "live" and verdict.get(i) == "NOVALUE"]
        rnd = random.Random(a.seed + 2)
        tce_check += rnd.sample(nv, min(a.probe_check, len(nv)))
    sys.stderr.write("[run] %d live mutants in %d distinct object codes; %d to run (+%d TCE-validation)\n" % (len(live), len(byh), len(reps), len(tce_check)))
    lockr = threading.Lock()
    wq2 = collections.deque(range(a.jobs))
    tstart = time.time()
    cnt = [0]

    def phaseB(item):
        h, mid = item
        with lockr:
            k = wq2.popleft()
        try:
            x = done[mid]
            r = rn.eval_mutant(mid, apply_edits(src, [tuple(e) for e in x["edits"]]), workd(k + 1))
            r["h"] = h
            return r
        finally:
            with lockr:
                wq2.append(k)
                cnt[0] += 1
                if cnt[0] % 25 == 0:
                    sys.stderr.write("[run] %d/%d  %.0fs  killed=%d\n" % (cnt[0], len(reps) + len(tce_check), time.time() - tstart, sum(1 for v in rn.kills.values() for _ in range(v))))

    work = list(reps) + [("tce:" + i, i) for i in tce_check]
    if work:
        with cf.ThreadPoolExecutor(a.jobs) as ex, open(res_f, "a") as fo:
            for r in ex.map(phaseB, work):
                if r["h"].startswith("tce:"):
                    r["tce_check"] = True
                fo.write(json.dumps(r) + "\n")
                fo.flush()
                results[r["id"] if not r.get("tce_check") else "tce-" + r["id"]] = r
    sys.stderr.write("[run] phase B wall %.0fs\n" % (time.time() - tstart))
    write_final(a, out, sel, done, pr)
    do_report(a, out)


def do_gcov(a, pl, rn, out, src):
    ef = os.path.join(out, "exec.json")
    if os.path.exists(ef) and not a.rebuild:
        return
    gd = os.path.join(out, "work", "gcov")
    os.makedirs(gd, exist_ok=True)
    info = pl.info
    # -O0 compile of the port from the recorded build (line counts are per source line; -O0 keeps them meaningful)
    base = os.path.basename(a.port)
    shutil.copy(a.port, os.path.join(gd, base))
    cav = [x for x in info["compile_argv"] if x not in ("-O2", "-O1", "-O3", "-Os")]
    cav = [os.path.join(gd, base) if x == info["port_arg"] else x for x in cav]
    k = cav.index("-o")
    stem = os.path.splitext(base)[0]
    cav[k + 1] = os.path.join(gd, stem + ".o")
    cmd = [info.get("cc", "gcc")] + cav + ["-O0", "--coverage", "-I", os.path.dirname(os.path.abspath(a.port))]
    r = run(cmd, cwd=info["cwd"])
    if r.returncode != 0:
        sys.stderr.write("[gcov] compile failed: %s\n" % r.stderr.decode(errors="replace")[:300])
        return
    lav = [os.path.join(gd, stem + ".o") if x == info["obj_arg"] else x for x in info["link_argv_pre"]]
    k = lav.index("-o")
    lav[k + 1] = os.path.join(gd, "g_test")
    r = run(["gcc"] + lav + ["--coverage"], cwd=info["cwd"])
    if r.returncode != 0:
        sys.stderr.write("[gcov] link failed: %s\n" % r.stderr.decode(errors="replace")[:300])
        return
    sys.stderr.write("[gcov] running %d scenarios of the -O0 instrumented port ...\n" % len(rn.plan.scenarios))
    with cf.ThreadPoolExecutor(min(a.jobs, len(rn.plan.scenarios))) as ex:
        list(ex.map(lambda s: run_scen(os.path.join(gd, "g_test"), s, info["cwd"], 7200), rn.plan.scenarios))
    r = run(["gcov", "-o", gd, os.path.join(gd, base)], cwd=gd)
    cg = os.path.join(gd, base + ".gcov")
    if not os.path.exists(cg):
        sys.stderr.write("[gcov] no .gcov produced: %s\n" % r.stderr.decode(errors="replace")[:300])
        return
    cnts = {}
    for ln in open(cg, errors="replace"):
        mo = re.match(r"^\s*([0-9#=\-*]+)\*?:\s*(\d+):", ln)
        if mo and int(mo.group(2)) > 0:
            c = mo.group(1)
            if c == "-":
                continue
            cnts[int(mo.group(2))] = 0 if c in ("#####", "=====") else int(c.replace("*", ""))
    json.dump(cnts, open(ef, "w"))


def write_final(a, out, sel, done, pr=None):
    if pr is None and os.path.exists(os.path.join(out, "probes.json")):
        pr = json.load(open(os.path.join(out, "probes.json")))
    results = {r["id"]: r for r in load_jsonl(os.path.join(out, "results.jsonl")) if not r.get("tce_check")}
    tcechk = {r["id"]: r for r in load_jsonl(os.path.join(out, "results.jsonl")) if r.get("tce_check")}
    hres = {}
    for rid, r in results.items():
        if rid in done and "h" in done[rid]:
            hres[done[rid]["h"]] = r
    exec_f = os.path.join(out, "exec.json")
    cnts = {int(k): v for k, v in json.load(open(exec_f)).items()} if os.path.exists(exec_f) else None
    rows = []
    for x in sel:
        d = done[x["id"]]
        row = dict(id=x["id"], cls=x["cls"], tags=x["tags"], func=x["func"], line=x["line"], direct=x["direct"], note=x["note"])
        if "h" in d:
            row["h"] = d["h"]
        st = d["status"]
        if pr:
            pv = probe_verdict(d, pr)
            if pv:
                row["probe"] = pv
        if st == "live":
            r = hres.get(d["h"])
            if r is None:
                row["status"] = "novalue" if row.get("probe") == "NOVALUE" else "pending"
            else:
                row["status"] = r["status"]
                if r["status"] == "killed":
                    row["by"] = r.get("by")
                if r["id"] != x["id"]:
                    row["dup_of"] = r["id"]
                row["t"] = round(r.get("t_build", 0) + r.get("t_run", 0), 2)
        else:
            row["status"] = {"equiv": "equiv", "invalid": "invalid"}.get(st, st)
        if cnts is not None:
            c = cnts.get(x["line"])
            row["exec"] = "EXECUTED" if (c or 0) > 0 else ("UNREACHED" if c == 0 else "NOCODE")
        rows.append(row)
    with open(os.path.join(out, "final.jsonl"), "w") as fo:
        for r in rows:
            fo.write(json.dumps(r) + "\n")
    chk = list(tcechk.values())
    if chk:
        bad = [r for r in chk if r["status"] == "killed"]
        json.dump(dict(n=len(chk), killed=len(bad), ids=[r["id"] for r in bad]), open(os.path.join(out, "tce_check.json"), "w"))


# ---------------------------------------------------------------------------------------------------------------
def do_report(a, out):
    src = read_port(a) if getattr(a, "port", None) else None
    if a.cmd == "report":
        done = {r["id"]: r for r in load_jsonl(os.path.join(out, "mutants.jsonl"))}
        if a.func and src:
            # labels (class / tags / direct / note) are heuristics: recompute them with the current code, keep the recorded results
            _m, _g, _ev, _mu = gen_mutants(a, src)
            for x in _mu:
                if x.id in done:
                    j = x.to_json()
                    for k in ("cls", "tags", "direct", "note", "site", "need"):
                        done[x.id][k] = j[k]
        write_final(a, out, list(done.values()), done)
    final = load_jsonl(os.path.join(out, "final.jsonl"))
    meta = json.load(open(os.path.join(out, "meta.json")))
    m = Model(src) if src else None
    allm = {x["id"]: x for x in load_jsonl(os.path.join(out, "mutants.jsonl"))}
    P = []
    P.append("mutation report: %s   functions: %s   source hash %s" % (meta["port"], ",".join(meta["funcs"]), meta["src_hash"]))
    ctl = os.path.join(out, "control.json")
    if os.path.exists(ctl):
        P.append("control (unmutated port via the replay pipeline): %s" % json.load(open(ctl))["status"])
    tot = collections.Counter(r["status"] for r in final)
    P.append("generated (by policy/classes/sample selection): %d   invalid %d   TCE-equivalent %d   live %d" % (
        len(final), tot["invalid"], tot["equiv"], tot["killed"] + tot["survived"] + tot["pending"] + tot["novalue"]))
    P.append("  live: killed %d  survived %d  NOVALUE (probe: never run) %d  pending %d" % (tot["killed"], tot["survived"], tot["novalue"], tot["pending"]))
    if any("probe" in r for r in final):
        sv_ = [r for r in final if r["status"] in ("survived", "novalue")]
        pc_ = collections.Counter(r.get("probe", "-") for r in sv_)
        P.append("  survivors+NOVALUE by value probe: NOVALUE %d (distinguishing value never occurs: identical on this harness; harness-gap candidates), "
                 "SEEN %d (value occurred, difference masked downstream: equivalence candidates), no probe %d" % (pc_["NOVALUE"], pc_["SEEN"], pc_["-"]))
    classes = sorted({r["cls"] for r in final})
    P.append("")
    P.append("%-8s %7s %7s %7s %7s %7s %7s  %s" % ("class", "gen", "invalid", "TCE-eq", "killed", "surv", "pend", "surv tags: DIR/UNDIRECTED/PTR/-"))
    for c in classes:
        rs = [r for r in final if r["cls"] == c]
        cc = collections.Counter(r["status"] for r in rs)
        sv = collections.Counter(r["direct"] or "-" for r in rs if r["status"] == "survived")
        P.append("%-8s %7d %7d %7d %7d %7d %7d  %s" % (c, len(rs), cc["invalid"], cc["equiv"], cc["killed"], cc["survived"], cc["pending"],
                                                      " ".join("%s=%d" % kv for kv in sorted(sv.items()))))
    # what the default directed policy would have selected from this (possibly --all) run
    class _D: include_skipped = False; include_noisy = False; include_weak = False; undirected = "keep"; classes = None
    pol = [r for r in final if policy_ok(r, _D)]
    if len(pol) != len(final):
        pc = collections.Counter(r["status"] for r in pol)
        P.append("")
        P.append("default directed policy (skips blob-sign-neutral accessor swaps and undirected noisy classes) keeps %d of %d mutants: "
                 "TCE-equivalent %d, live %d (killed %d, survived %d)" % (len(pol), len(final), pc["equiv"], pc["killed"] + pc["survived"], pc["killed"], pc["survived"]))
        cut = collections.Counter((r["cls"], r["direct"] or "-") for r in final if not policy_ok(r, _D))
        P.append("  excluded by policy: " + ", ".join("%s/%s %d" % (k[0], k[1], v) for k, v in sorted(cut.items())))
        P.append("  (equivalent-mutant pre-filter + policy): live mutants needing a run: all %d -> policy %d" % (
            tot["killed"] + tot["survived"] + tot["pending"], pc["killed"] + pc["survived"] + pc["pending"]))
    def tsum(rows):
        seen, tt = set(), 0.0
        for r in rows:
            if r["status"] in ("killed", "survived") and r.get("h") and r["h"] not in seen:
                seen.add(r["h"])
                tt += r.get("t", 0)
        return tt
    P.append("")
    P.append("CPU seconds spent (build + scenario runs, distinct object codes): all run mutants %.0f s; default-policy subset %.0f s" % (tsum(final), tsum(pol)))
    if any("exec" in r for r in final):
        sv = [r for r in final if r["status"] in ("survived", "novalue")]
        ce = collections.Counter(r.get("exec", "?") for r in sv)
        P.append("")
        P.append("survivors by line execution (gcov, -O0 port over all scenarios): %s" % dict(ce))
    tc = os.path.join(out, "tce_check.json")
    if os.path.exists(tc):
        P.append("TCE validation: %s" % open(tc).read())
    # survivors, grouped by function/line
    sv = [r for r in final if r["status"] in ("survived", "novalue")]
    P.append("")
    P.append("SURVIVORS (%d; incl. NOVALUE), grouped by function / line.  [class direct exec probe] note" % len(sv))
    byf = collections.defaultdict(list)
    for r in sv:
        byf[(r["func"], r["line"])].append(r)
    for (fn, ln), rs in sorted(byf.items()):
        orig = m.line_text(ln).strip() if m else ""
        P.append("  %s:%d  %s" % (fn, ln, orig[:150]))
        for r in sorted(rs, key=lambda r: (r["cls"], r["note"])):
            x = allm.get(r["id"])
            mutl = ""
            if m and x:
                try:
                    mutl = mutant_display(m, x)[1]
                except Exception:
                    pass
            P.append("      %s [%s %s%s%s] %s%s" % (r["id"], r["cls"], r["direct"] or "-", (" " + r["exec"]) if "exec" in r else "", (" " + r["probe"]) if "probe" in r else "", r["note"],
                                                  ("   => " + mutl[:150]) if mutl and mutl != orig else ""))
    txt = "\n".join(P) + "\n"
    open(os.path.join(out, "report.txt"), "w").write(txt)
    # sorted survivor list (diffable)
    with open(os.path.join(out, "survivors.txt"), "w") as fo:
        for r in sorted(sv, key=lambda r: (r["func"], r["line"], r["cls"], r["note"], r["id"])):
            fo.write("%s:%d %s %s %s\n" % (r["func"], r["line"], r["cls"], r["id"], r["note"]))
    print(txt)


def do_compare(a):
    """survivors of run A that run B (an extended harness, re-run with --survivors-of) killed = the real harness weaknesses"""
    A = {r["id"]: r for r in load_jsonl(os.path.join(a.out, "final.jsonl"))}
    B = {r["id"]: r for r in load_jsonl(os.path.join(a.other, "final.jsonl"))}
    real = [A[i] for i, r in B.items() if i in A and A[i]["status"] == "survived" and r["status"] == "killed"]
    still = [A[i] for i, r in B.items() if i in A and A[i]["status"] == "survived" and r["status"] == "survived"]
    print("survivors of %s: %d; re-run in %s: killed %d (real harness weaknesses), survived %d" % (
        a.out, sum(1 for r in A.values() if r["status"] == "survived"), a.other, len(real), len(still)))
    class _D: include_skipped = False; include_noisy = False; include_weak = False; undirected = "keep"; classes = None
    for r in sorted(real, key=lambda r: (r["func"], r["line"])):
        print("  REAL %s %s:%d [%s direct=%s tags=%s] %s   killed by %s   default policy keeps it: %s" % (
            r["id"], r["func"], r["line"], r["cls"], r["direct"] or "-", ",".join(r["tags"]), r["note"], B[r["id"]].get("by"), policy_ok(r, _D)))


def do_list(a):
    src = read_port(a)
    m, g, ev, muts = gen_mutants(a, src)
    allm = [x.to_json() for x in muts]
    c = collections.Counter((x["cls"], x["direct"] or "-") for x in allm)
    print("generated %d mutants for %s" % (len(allm), ",".join(a.func)))
    for k, v in sorted(c.items()):
        print("  %-8s %-11s %d" % (k[0], k[1], v))
    sel = [x for x in allm if class_ok(x, a) and policy_ok(x, a)]
    print("selected by the default policy: %d (excluded: %d)" % (len(sel), len(allm) - len(sel)))
    if a.dump:
        for x in allm:
            o, n = mutant_display(m, x)
            print("%s %s:%d [%s %s] %s" % (x["id"], x["func"], x["line"], x["cls"], x["direct"] or "-", x["note"]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["list", "run", "report", "compare"])
    ap.add_argument("--other", help="compare: the second out-dir (survivors-of re-run against an extended harness)")
    ap.add_argument("--port", required=False, help="port source file to mutate (path)")
    ap.add_argument("--func", action="append", required=False, default=[], help="function to mutate (repeatable)")
    ap.add_argument("--blob-func", action="append", help="PORTNAME=BLOBNAME when the blob symbol differs")
    ap.add_argument("--blob", default=None, help="dsplibs.o (default: BLOB_O, test/blob.sh)")
    ap.add_argument("--no-evidence", action="store_true", help="do not consult the blob (every blob-directed mutant becomes UNDIRECTED)")
    ap.add_argument("--root", default=os.path.join(HERE, ".."), help="dsplibs-port directory (where the build script runs)")
    ap.add_argument("--chunk", help="job id of test/jobs.tsv (TESTREORG stage 10: the planned name, e.g. v34/v34_detectors; an old id such as v34_tier5c resolves via tools/job_aliases.tsv); script = its row's stage-1 script")
    ap.add_argument("--script", help="build script (default test/build_and_diff_<chunk>.sh)")
    ap.add_argument("--port-env", help="env var through which the script takes the port under test (e.g. PORT_T7A)")
    ap.add_argument("--env", action="append", help="extra VAR=VALUE for the build script (e.g. PORT_XFLAGS=...)")
    ap.add_argument("--scenarios", help="scenario list (default: parsed from the script)")
    ap.add_argument("--out", help="output directory (JSONL, report, scratch)")
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--sample", type=int, default=0, help="quick mode: run ~N mutants, stratified by class")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--classes", help="comma list of class names / tags to include; prefix '-' to exclude (e.g. LIT,REL,-SIGN)")
    ap.add_argument("--lit-deltas", default="1,2", help="literal deltas (default 1,2)")
    ap.add_argument("--undirected", choices=["keep", "skip"], default="keep", help="mutants whose blob evidence could not be matched: run (default) or skip")
    ap.add_argument("--include-skipped", action="store_true", help="also run accessor sign swaps the blob marks sign-neutral or extension-only (WEAK) (for validation)")
    ap.add_argument("--include-weak", action="store_true", help="also run accessor sign swaps whose only blob evidence is a movsx/movzx (value never reaches a compare / sar / shr / div)")
    ap.add_argument("--include-noisy", action="store_true", help="also run the undirected noisy classes (decl SIGN, WIDTH, CAST outside sign contexts)")
    ap.add_argument("--all", action="store_true", help="shorthand for --include-skipped --include-noisy (the full, undirected set)")
    ap.add_argument("--ids", help="only these mutant ids (comma list or @file), after the class / policy selection")
    ap.add_argument("--survivors-of", help="only re-run the survivors recorded in this final.jsonl (extended harness)")
    ap.add_argument("--gcov", action="store_true", help="also collect line execution counts (gcov, -m32) to flag UNREACHED survivors")
    ap.add_argument("--tce-check", type=int, default=0, help="also RUN N randomly chosen TCE-equivalent mutants (validation: all must survive)")
    ap.add_argument("--probes", action="store_true", help="value probes: instrument the original port, run all scenarios once, and classify mutants whose distinguishing "
                    "value never occurs as NOVALUE (not run: identical on this harness; harness-gap candidates)")
    ap.add_argument("--no-probe-skip", action="store_true", help="with --probes: still run the NOVALUE mutants (validation)")
    ap.add_argument("--probe-check", type=int, default=0, help="with --probes: also RUN N random NOVALUE mutants (validation: all must survive)")
    ap.add_argument("--rebuild", action="store_true", help="redo the recorded base build / blob outputs / control")
    ap.add_argument("--dump", action="store_true", help="list: print every mutant")
    a = ap.parse_args()
    if a.all:
        a.include_skipped = a.include_noisy = True
    if a.cmd in ("run",):
        if not (a.chunk or a.script) or not a.port_env or not a.out:
            ap.error("run needs --chunk/--script, --port-env and --out")
        if not a.script:
            a.script = "test/build_and_diff_%s.sh" % a.chunk
            mv = subprocess.run(["sh", os.path.join(os.path.dirname(os.path.abspath(__file__)), "job_path.sh"), a.chunk, "script"], capture_output=True, text=True)
            if mv.returncode == 0 and mv.stdout.strip():          # TESTREORG: a job moved into test/<module>/ (test/jobs.tsv)
                a.script = mv.stdout.strip()
    if a.cmd == "compare":
        do_compare(a)
    elif a.cmd == "list":
        do_list(a)
    elif a.cmd == "run":
        do_run(a)
    else:
        do_report(a, os.path.abspath(a.out))



# ====================================================================================================================
# 7. value probes: instrument the ORIGINAL port so that the harness itself tells which mutants can never be observed
# ====================================================================================================================
# A mutant that changes behaviour only when some VALUE occurs (operands equal at a relational, a 16-bit load with bit 15
# set, a stored value differing from the old one, ...) cannot be killed by a harness that never produces that value.
# The probes count, per site and over all scenarios, how often the distinguishing value occurred.  need == 0 means the
# mutant is NOVALUE: behaviourally identical on this harness (so it needs no run) AND a harness-gap candidate; need > 0
# but the mutant survives means the difference is masked downstream (equivalent candidate).
MP_PRELUDE = r'''
#ifdef __cplusplus
extern "C" {
#endif
extern void mp_note(int id, unsigned a, unsigned b, unsigned c);
#ifdef __cplusplus
}
#endif
#define MP_SB(x, w) ((unsigned)(((unsigned long long)(long long)(x) >> ((w) - 1)) & 1u))
#define MP_FL(x) (__builtin_classify_type(x) == 8)
#define MP_REL(id, op, a, b) __extension__({ __typeof__(a) _mp_a = (a); __typeof__(b) _mp_b = (b); \
    long long _mp_d = (long long)(__typeof__(_mp_a - _mp_b))_mp_a - (long long)(__typeof__(_mp_a - _mp_b))_mp_b; \
    mp_note((id), _mp_a == _mp_b, (MP_SB(_mp_a, 32) != MP_SB(_mp_b, 32)) | (2u * (MP_FL(_mp_a) || MP_FL(_mp_b))), \
            _mp_d >= -2 && _mp_d <= 2 && _mp_d != 0 ? (unsigned)(_mp_d < 0 ? _mp_d + 3 : _mp_d + 2) : 0u); \
    _mp_a op _mp_b; })
#define MP_RELP(id, op, a, b) __extension__({ __typeof__(a) _mp_a = (a); __typeof__(b) _mp_b = (b); \
    mp_note((id), _mp_a == _mp_b, (MP_SB(_mp_a, 32) != MP_SB(_mp_b, 32)) | (2u * (MP_FL(_mp_a) || MP_FL(_mp_b))), 0u); \
    _mp_a op _mp_b; })
#define MP_ACC(id, w, acc, o, off) __extension__({ __typeof__(acc(o, off)) _mp_v = acc(o, off); mp_note((id), MP_SB(_mp_v, w), 0, 0); _mp_v; })
#define MP_CV(id, w, x) __extension__({ __typeof__(x) _mp_c = (x); mp_note((id), MP_SB(_mp_c, w), MP_FL(_mp_c) ? 2u : 0u, 0); _mp_c; })
#define MP_ASG(id, lhs, x) __extension__({ __typeof__(x) _mp_r = (x); mp_note((id), (__typeof__(lhs))_mp_r != (lhs), 0, 0); _mp_r; })
'''

MP_RT = r'''
#include <fcntl.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>
static unsigned *mp_tab;
__attribute__((constructor)) static void mp_init(void) {
  const char *p = getenv("MP_OUT");
  if (!p) return;
  int fd = open(p, O_RDWR);
  if (fd < 0) return;
  off_t n = lseek(fd, 0, SEEK_END);
  void *m = mmap(0, (size_t)n, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (m != MAP_FAILED) mp_tab = (unsigned *)m;
  close(fd);
}
/* 8 set-once flags per site: 0 executed, 1 a, 2 sign-bit-differs, 3 float operand, 4..7 (left-right) == -2,-1,+1,+2 */
void mp_note(int id, unsigned a, unsigned b, unsigned c) {
  volatile unsigned *t;
  if (!mp_tab) return;
  t = mp_tab + 8 * id;
  if (!t[0]) t[0] = 1u;
  if (a && !t[1]) t[1] = 1u;
  if ((b & 1u) && !t[2]) t[2] = 1u;
  if ((b & 2u) && !t[3]) t[3] = 1u;
  if (c && !t[3 + c]) t[3 + c] = 1u;
}
'''


def build_instrumented(src, m, sites, keys):
    """-> (instrumented source text, {site key: int id}).  Edits are inserts / replacements in the ORIGINAL text, composed so
    that nested sites (an accessor inside a relational inside a cast ...) nest correctly."""
    toks = m.toks
    ids = {k: i for i, k in enumerate(sorted(keys))}
    E = []          # (pos, isrep, phase, spankey, layer, end, text)

    def ins(pos, phase, span, layer, text):
        # opens (phase 1): outer first = larger span first; closes (phase 0): inner first = smaller span first
        E.append((pos, 0, phase, -span if phase == 1 else span, -layer if phase == 1 else layer, pos, text))

    def rep(s_, e_, text):
        E.append((s_, 1, 0, 0, 0, e_, text))

    for k in keys:
        st = sites[k]
        i = ids[k]
        if st["kind"] == "rel":
            l0, l1 = st["l"]
            r0, r1 = st["r"]
            op = toks[st["op"]]
            whole = toks[r1].e - toks[l0].s
            ins(toks[l0].s, 1, whole, 1, "%s(%d, %s, (" % ("MP_RELP" if st.get("ptr") else "MP_REL", i, op.text))
            ins(toks[l1].e, 0, toks[l1].e - toks[l0].s, 1, ")")
            rep(op.s, op.e, ", (")
            ins(toks[r1].e, 0, whole, 1, "))")
        elif st["kind"] == "acc":
            t, lp = toks[st["k"]], toks[st["k"] + 1]
            rep(t.s, lp.e, "MP_ACC(%d, %d, %s, " % (i, st["w"], st["name"]))
        elif st["kind"] == "cv":
            o0, o1 = st["opnd"]
            sp = toks[o1].e - toks[o0].s
            ins(toks[o0].s, 1, sp, 1, "MP_CV(%d, %d, (" % (i, st["w"]))
            ins(toks[o1].e, 0, sp, 1, "))")
        elif st["kind"] == "asg":
            l0, l1 = st["lhs"]
            r0, r1 = st["rhs"]
            sp = toks[r1].e - toks[r0].s
            lhs = src[toks[l0].s:toks[l1].e]
            ins(toks[r0].s, 1, sp, 1, "MP_ASG(%d, %s, (" % (i, lhs))
            ins(toks[r1].e, 0, sp, 1, "))")
    E.sort(key=lambda x: (x[0], x[1], x[2], x[3], x[4]))
    out, pos = [], 0
    for (p, isrep, phase, sk, ly, end, text) in E:
        out.append(src[pos:p])
        out.append(text)
        pos = end
    out.append(src[pos:])
    body = "".join(out)
    # put the prelude after the last #include at the top (types are used only through __typeof__) - simplest: at the very top
    return MP_PRELUDE + "\n#line 1\n" + body, ids


def do_probes(a, pl, rn, out, src, m, sites):
    pf = os.path.join(out, "probes.json")
    if os.path.exists(pf) and not a.rebuild:
        return json.load(open(pf))
    work = os.path.join(out, "work", "probe")
    os.makedirs(work, exist_ok=True)
    rt = os.path.join(work, "mp_rt.c")
    open(rt, "w").write(MP_RT)
    rto = os.path.join(work, "mp_rt.o")
    r = run(["gcc", "-m32", "-O2", "-c", rt, "-o", rto])
    if r.returncode != 0:
        sys.stderr.write("[probes] runtime failed: %s\n" % r.stderr.decode(errors="replace")[:300])
        return None
    base = os.path.basename(a.port)
    # try all site kinds; drop kinds whose instrumentation does not compile
    kinds = ["rel", "acc", "cv", "asg"]
    use = list(kinds)
    for attempt in range(len(kinds) + 1):
        keys = [k for k, v in sites.items() if v["kind"] in use]
        text, ids = build_instrumented(src, m, sites, keys)
        ms = os.path.join(work, base)
        open(ms, "w").write(text)
        obj = os.path.join(work, "p.o")
        r = run(pl.compile_cmd(ms, obj), cwd=pl.info["cwd"])
        if r.returncode == 0:
            break
        err = r.stderr.decode(errors="replace")
        bad = None
        for kd in use:
            keys1 = [k for k, v in sites.items() if v["kind"] == kd]
            t1, _ = build_instrumented(src, m, sites, keys1)
            open(ms, "w").write(t1)
            r1 = run(pl.compile_cmd(ms, obj), cwd=pl.info["cwd"])
            if r1.returncode != 0:
                bad = kd
                sys.stderr.write("[probes] site kind %s does not compile (%s): dropped\n" % (kd, (r1.stderr.decode(errors='replace').strip().splitlines() or [''])[-1][:160]))
                break
        if bad is None:
            sys.stderr.write("[probes] instrumented build failed: %s\n" % err[:400])
            return None
        use.remove(bad)
    else:
        return None
    exe = os.path.join(work, "p_test")
    lc = pl.link_cmd(obj, exe)
    r = run(lc + [rto], cwd=pl.info["cwd"])
    if r.returncode != 0:
        sys.stderr.write("[probes] link failed: %s\n" % r.stderr.decode(errors="replace")[:400])
        return None
    tab = os.path.join(work, "counters.bin")
    with open(tab, "wb") as fo:
        fo.write(b"\0" * (32 * max(1, len(ids))))
    env = dict(os.environ)
    env["MP_OUT"] = tab

    def one(s):
        st, o, det = run_scen(exe, s, pl.info["cwd"], 3600, env=env)
        return s, st, o, det

    sys.stderr.write("[probes] running %d scenarios of the instrumented port (%d sites: %s)\n" % (len(pl.scenarios), len(ids), ",".join(use)))
    bad = []
    with cf.ThreadPoolExecutor(min(a.jobs, len(pl.scenarios))) as ex:
        for s, st, o, det in ex.map(one, pl.scenarios):
            if st != "ok" or o != rn.blob[s]["out"]:
                bad.append((s, st, det))
    if bad:
        sys.stderr.write("[probes] instrumented port DOES NOT MATCH the blob in %s: probes disabled (instrumentation changes behaviour, e.g. x87 temporaries)\n" % bad)
        return None
    import array
    arr = array.array("I")
    arr.frombytes(open(tab, "rb").read())
    res = {}
    for k, i in ids.items():
        res[k] = [arr[8 * i + j] for j in range(8)]
    res["_kinds"] = use
    json.dump(res, open(pf, "w"))
    return res


def probe_verdict(mu, pr):
    """'NOVALUE' (provably identical on this harness), 'SEEN' (distinguishing value occurred), '' (no probe)"""
    if not pr or not mu.get("site") or not mu.get("need"):
        return ""
    c = pr.get(mu["site"])
    if c is None:
        return ""
    n, a_, b_, f_ = c[:4]
    need = mu["need"]
    if need.startswith("d:"):
        if n == 0:
            return "NOVALUE"
        if f_ > 0:
            return ""
        for v in need[2:].split(","):
            v = int(v)
            if (v == 0 and a_ > 0) or (v in (-2, -1, 1, 2) and c[{-2: 4, -1: 5, 1: 6, 2: 7}[v]] > 0):
                return "SEEN"
        return "NOVALUE"
    if n == 0:
        return "NOVALUE"            # the mutated operation is never executed: identical (same as gcov UNREACHED)
    if need in ("top",) and f_ > 0:
        return ""                   # floating operand seen: no value-based claim
    if need in ("eq", "top", "chg"):
        return "SEEN" if a_ > 0 else "NOVALUE"
    if need == "n":
        return "SEEN"
    if need == "sd":
        if f_ > 0:
            return ""               # floating operands seen: a (u)int32_t wrap is not value-preserving, no claim
        return "SEEN" if b_ > 0 else "NOVALUE"
    return ""


if __name__ == "__main__":
    main()
