#!/usr/bin/env python3
"""stampkeys.py - object-code keys for the differential-test stamps of tools/run_changed.sh  (STAMPKEYS pass).

WHAT A KEY IS
  The old key of a job was the SHA-256 of SOURCE TEXT (script, test files, every library file the script compiles, the gcc -MM header closure).
  A comment edit or a behaviour-neutral refactor (prototypes moved into headers, RL4 / RL5) therefore invalidated ~180 of 184 stamps although the
  compiled code was identical.  The key of this scheme (SCHEME below) hashes the COMPILED CODE of every compile unit the job builds instead:
  a stamp now means "the same compiled port code (and the same harness text / goldens / script) as a run that passed".

  key = sha256 of
      scheme tag, compiler versions, job id,
      the SOURCE hash of everything that is not compiled port code: the job script(s), test files, goldens, harness text, sources.mk, blob ...
      (every file the old key hashed, minus the library .c/.cpp/headers that are covered by an object hash),
      the compile units (src path + normalised flags) and the sorted MULTISET of (flag-set, kind, symbol, hash) of everything they compile to:
        F  per function: the bytes as objdump -d shows them, relocations / call targets replaced by symbol NAMES (tools/o0check.py's
           normalisation: function-relative offsets, section-relative targets resolved to the function / object / string / constant they
           point at; jump tables, .rodata constants resolved by content), alignment padding excluded;
        D  per data object (.data .rodata .bss .tbss ..., vtables, const tables): bytes with relocations as symbol names;
      so the order of the functions in an object, the object a function lives in (within the job's compile set) and the address of anything do not
      matter.  Harness units (test/*.c) are hashed the same way IN ADDITION to their source text.

RECIPES  (how the units are compiled)  are RECORDED, never guessed: `record` runs every stage of a job (stage 1, the pinned re-run, stage 1b,
  stage 2) once in a private sandbox tree with a logging compiler shim (tools/stampkeys_shim.sh) first in PATH: the shim logs the real command
  lines, runs the real compiler, and links every test executable with a constructor that exits at once, so no test really runs.  The recipe is
  cached by the hash of the job's script-like files (scripts, *.txt / *.mk lists, flagshim, flags.sh ...).  A job whose recipe cannot be recorded
  reliably (script failed in the recorder, generated sources, absolute scratch paths in a flag, -S/-E output steps ...) falls back to the old
  source-text key and is listed.

Usage (normally called by tools/run_changed.sh; see tools/README.md "STAMPKEYS"):
  stampkeys.py keys REQUEST_FILE RESULT_FILE [-j N]    one line per request:  id TAB kind(KB|KF) TAB script TAB args TAB g64cmd TAB files...
                                                      result: id TAB kind TAB OBJ TAB key TAB info   |   id TAB kind TAB FALLBACK TAB reason
  stampkeys.py objhash OBJECT.o                        print the (kind, symbol, hash) items of one object file
  stampkeys.py cache-trim                              apply the size cap to the cache
Environment: STAMPKEYS_CACHE (cache dir; default ~/.cache/dsplibs-stampkeys), STAMPKEYS_CAP_MB (default 250), STAMPKEYS_KEYLOG (dir for key input texts).
"""
import argparse, bisect, collections, hashlib, json, os, re, shlex, shutil, subprocess, sys, tempfile, time, zlib
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import o0check
from o0check import Obj, Norm, func_groups, find_in, obj_at, addend_of, is_exec, is_alloc, PCREL, SKIP_DATA

SCHEME = 'stampkeys-v1'
REC_VERSION = '1'
ROOT_TAG = '@R@'
GCC = '/usr/bin/gcc'

# ----------------------------------------------------------------------------------------------------------------- object hashing

_orig_resolve = o0check.resolve
_active = set()


def _bounds(obj):
    """section index -> sorted offsets referenced through section symbols (starts of the anonymous chunks: jump tables, constants)."""
    b = getattr(obj, '_sk_bounds', None)
    if b is not None:
        return b
    b = collections.defaultdict(set)
    for tsx, rels in obj.relocs.items():
        for (ro, rtype, symidx, add) in rels:
            s = obj.syms[symidx]
            if s['type'] == 'SECTION' or (s['name'] == '' and isinstance(s['shndx'], int)):
                A = addend_of(obj, tsx, ro, rtype, add)
                b[s['shndx']].add(s['value'] + (A + 4 if rtype in PCREL else A))
    obj._sk_bounds = {k: sorted(v) for k, v in b.items()}
    return obj._sk_bounds


def _chunk_sig(obj, norm, fg, sx, off):
    """Content signature of the anonymous chunk of section sx starting at off (up to the next referenced offset / object symbol / section end)."""
    sec = obj.secs[sx]
    data = obj.secdata[sx]
    ends = [len(data)]
    bl = _bounds(obj).get(sx, [])
    i = bisect.bisect_right(bl, off)
    if i < len(bl):
        ends.append(bl[i])
    for si in obj.objs.get(sx, []):
        sv = obj.syms[si]['value']
        if sv > off:
            ends.append(sv)
    end = min(ends)
    key = (id(obj), sx, off)
    if key in _active:
        return 'cyc'
    _active.add(key)
    try:
        raw = bytearray(data[off:end])
        rl = []
        for (ro, rtype, symidx, add) in obj.relocs.get(sx, []):
            if off <= ro < end:
                A = addend_of(obj, sx, ro, rtype, add)
                if add is None:
                    raw[ro - off:ro - off + 4] = b'\xee' * 4
                rl.append('%d %s %s' % (ro - off, rtype, o0check.resolve(obj, norm, fg, symidx, A + 4 if rtype in PCREL else A)))
    finally:
        _active.discard(key)
    return hashlib.sha256((bytes(raw).hex() + '|' + ';'.join(rl)).encode()).hexdigest()[:12]


def resolve2(obj, norm, fg, symidx, extra):
    """o0check.resolve + move tolerance: a target the original can only name as <section>+offset (jump tables, constant pools) is named by CONTENT."""
    r = _orig_resolve(obj, norm, fg, symidx, extra)
    s = obj.syms[symidx]
    if not (s['type'] == 'SECTION' or (s['name'] == '' and isinstance(s['shndx'], int))):
        return r
    sx = s['shndx']
    if not isinstance(sx, int) or sx >= len(obj.secs):
        return r
    sec = obj.secs[sx]
    if is_exec(sec):
        return r
    base = norm.sec(sec.name)
    if not (r.startswith(base) and (len(r) == len(base) or r[len(base)] in '+-')):
        return r                                        # resolved by the original (object symbol / string)
    off = s['value'] + extra
    if sx not in obj.secdata or not (0 <= off < len(obj.secdata[sx])):
        return r
    fl = sec['sh_flags']
    ent = sec['sh_entsize']
    if (fl & 0x10) and not (fl & 0x20) and ent > 0:     # SHF_MERGE constants (.rodata.cst4 / cst8 / cst16): the constant itself
        return 'const:' + obj.secdata[sx][off:off + ent].hex()
    return 'anon:%s:%s' % (re.sub(r'\.(cst|str)\d.*$', '', base), _chunk_sig(obj, norm, fg, sx, off))


o0check.resolve = resolve2          # norm_functions looks `resolve` up in o0check's globals: this is the only change, o0check.py itself is untouched

NOP_RE = re.compile(r'^(nop\w*(\s.*)?|data16\b.*|cs nop\w*.*|(ds|es|fs|gs|cs)\s+nop.*|xchg\s+%ax,%ax|int3|lea\s+0x0\(.*\),%\w+|lea\s+0x0\(.*\)|add\s+%al,\(%[re]ax\)|rep\s+ret|\(bad\).*|\.byte.*)$')
CODE_SEC_KEEP = ('.text',)


def _h(s):
    return hashlib.sha256(s.encode('utf-8', 'replace')).hexdigest()[:16]


def secclass(name, symname):
    """Section name without the per-symbol suffix (-fdata-sections / vague linkage: .data.rel.ro._ZTV3Foo -> .data.rel.ro)."""
    if symname and name.endswith('.' + symname):
        return name[:-len(symname) - 1]
    return name


def func_items(obj, norm):
    out = []
    nf = o0check.norm_functions(obj, norm)
    for name, lines in nf.items():
        out.append(('F', re.sub(r'#\d+$', '', name), _h('\n'.join(lines))))
    # code that is not inside a FUNC symbol (hand-written asm, stray bytes) other than alignment padding: part of the key too
    fg = func_groups(obj, norm)
    for sx, sec in enumerate(obj.secs):
        if not is_exec(sec) or sx not in obj.dis:
            continue
        starts = [g[0] for g in fg.get(sx, [])]
        gaps = []
        for a, t in obj.dis[sx]:
            i = bisect.bisect_right(starts, a) - 1
            if i >= 0 and a < fg[sx][i][1]:
                continue
            t2 = re.sub(r'\s+', ' ', t.split('#')[0]).strip()
            if not NOP_RE.match(t2):
                gaps.append('%x %s' % (a, t2))
        if gaps:
            out.append(('G', norm.sec(sec.name), _h('\n'.join(gaps))))
    return out


def data_items(obj, norm):
    out = []
    fg = func_groups(obj, norm)
    # COMMON symbols
    for s in obj.syms:
        if s['shndx'] == 'SHN_COMMON':
            out.append(('D', norm.name(s['name']), _h('COMMON %s %d' % (s['bind'], s['size']))))
    for sx, sec in enumerate(obj.secs):
        if not is_alloc(sec) or is_exec(sec) or sec.name in SKIP_DATA or sec.name.startswith('.note'):
            continue
        nobits = sec['sh_type'] == 'SHT_NOBITS'
        data = obj.secdata.get(sx, b'')
        rels = obj.relocs.get(sx, [])
        raddr = [r[0] for r in rels]
        syms = sorted(obj.objs.get(sx, []), key=lambda i: (obj.syms[i]['value'], obj.syms[i]['name']))
        covered = []
        for i in syms:
            s = obj.syms[i]
            lo, hi = s['value'], s['value'] + s['size']
            covered.append((lo, hi))
            raw = bytearray(b'\0' * s['size'] if nobits else data[lo:hi])
            rl = []
            for r in rels[bisect.bisect_left(raddr, lo):bisect.bisect_left(raddr, max(hi, lo + 1))]:
                ro, rtype, symidx, add = r
                if not (lo <= ro < max(hi, lo + 1)):
                    continue
                A = addend_of(obj, sx, ro, rtype, add)
                if add is None:
                    raw[ro - lo:ro - lo + 4] = b'\xee' * 4
                rl.append('%d %s %s' % (ro - lo, rtype, o0check.resolve(obj, norm, fg, symidx, A + 4 if rtype in PCREL else A)))
            nm = norm.name(s['name'])
            out.append(('D', nm, _h('%s %s %s %s size=%d %s|%s' % (s['type'], s['bind'], s['vis'], secclass(norm.sec(sec.name), s['name']), s['size'], bytes(raw).hex(), ';'.join(rl)))))
        # bytes of the section that no object symbol covers: anonymous rodata is reached (and hashed) through the references to it; anything else
        # (.init_array, symbol-less data) is part of the key as one item per section
        if sec.name.startswith('.rodata') or nobits:
            continue
        covered.sort()
        raw = bytearray()
        rl = []
        pos = 0
        for lo, hi in covered + [(len(data), len(data))]:
            if lo > pos:
                raw += data[pos:lo]
                for r in rels[bisect.bisect_left(raddr, pos):bisect.bisect_left(raddr, lo)]:
                    ro, rtype, symidx, add = r
                    A = addend_of(obj, sx, ro, rtype, add)
                    if add is None:
                        raw[len(raw) - (lo - ro):len(raw) - (lo - ro) + 4] = b'\xee' * 4
                    rl.append('%s %s' % (rtype, o0check.resolve(obj, norm, fg, symidx, A + 4 if rtype in PCREL else A)))
            pos = max(pos, hi)
        if raw:
            out.append(('A', norm.sec(sec.name), _h(bytes(raw).hex() + '|' + ';'.join(rl))))
    return out


def obj_items(path):
    """Sorted list of (kind, name, hash) of one object file."""
    obj = Obj(path)
    norm = Norm({})
    items = func_items(obj, norm) + data_items(obj, norm)
    items.sort()
    return items


# ----------------------------------------------------------------------------------------------------------------- cache

def cache_dir():
    d = os.environ.get('STAMPKEYS_CACHE') or os.path.join(os.environ.get('XDG_CACHE_HOME') or os.path.join(os.path.expanduser('~'), '.cache'), 'dsplibs-stampkeys')
    os.makedirs(d, exist_ok=True)
    return d


def cache_path(kind, key):
    d = os.path.join(cache_dir(), kind, key[:2])
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, key)


def cache_get(kind, key):
    p = cache_path(kind, key)
    try:
        with open(p, 'rb') as f:
            v = json.loads(zlib.decompress(f.read()))
        os.utime(p, None)           # "used now": the trim removes the least recently used first
        return v
    except (OSError, ValueError, zlib.error):
        return None


def cache_put(kind, key, val):
    p = cache_path(kind, key)
    tmp = '%s.tmp.%d' % (p, os.getpid())
    with open(tmp, 'wb') as f:
        f.write(zlib.compress(json.dumps(val, separators=(',', ':')).encode(), 6))
    os.replace(tmp, p)


def cache_trim(quiet=True):
    """Size cap: when the cache exceeds STAMPKEYS_CAP_MB (default 250), delete the least recently used entries (mtime, refreshed on every hit) until it is under 80 % of the cap."""
    cap = int(os.environ.get('STAMPKEYS_CAP_MB', '250')) * 1024 * 1024
    d = cache_dir()
    files = []
    total = 0
    for dp, dn, fn in os.walk(d):
        for n in fn:
            p = os.path.join(dp, n)
            try:
                st = os.stat(p)
            except OSError:
                continue
            files.append((st.st_mtime, st.st_size, p))
            total += st.st_size
    if total <= cap:
        return total, 0
    files.sort()
    gone = 0
    for mt, sz, p in files:
        if total <= cap * 0.8:
            break
        try:
            os.unlink(p)
            total -= sz
            gone += 1
        except OSError:
            pass
    return total, gone


# ----------------------------------------------------------------------------------------------------------------- toolchain / files

_tool_cache = {}


def toolchain():
    if 'tc' not in _tool_cache:
        _tool_cache['tc'] = o0check.gcc_version()
    return _tool_cache['tc']


def tool_hash():
    if 'th' not in _tool_cache:
        h = hashlib.sha256()
        for f in (os.path.abspath(__file__), os.path.join(HERE, 'o0check.py')):
            h.update(open(f, 'rb').read())
        _tool_cache['th'] = h.hexdigest()[:12]
    return _tool_cache['th']


_sha_cache = {}


def sha_file(path):
    v = _sha_cache.get(path)
    if v is None:
        try:
            with open(path, 'rb') as f:
                v = hashlib.sha256(f.read()).hexdigest()
        except OSError:
            v = None
        _sha_cache[path] = v
    return v


# ----------------------------------------------------------------------------------------------------------------- compile units

TWO_ARG = {'-o', '-MF', '-MT', '-MQ', '-include', '-imacros', '-isystem', '-idirafter', '-iquote', '-iprefix', '-x', '-D', '-U', '-I', '-L', '-l',
           '-T', '-Xlinker', '-Xassembler', '-Xpreprocessor', '-z', '-u', '--param', '-aux-info', '-arch', '-target', '-e', '-iwithprefix'}
SRC_EXT = ('.c', '.cc', '.cpp', '.cxx', '.C', '.CPP')
LINK_ONLY_PREFIX = ('-l', '-L', '-Wl,')
SKIP_CMD = ('-E', '-S', '-fsyntax-only', '-M', '-MM', '-v', '--version', '-dumpmachine', '-dumpversion', '-###', '-shared', '-r', '-print-')


def parse_cmd(args):
    """Real compiler command line -> (flags, sources, kind).  kind: 'compile' | 'skip' (no object: -fsyntax-only, -M, version queries) | 'unsupported:<why>'."""
    flags = []
    srcs = []
    other = []
    i = 0
    n = len(args)
    kind = 'compile'
    while i < n:
        a = args[i]
        if a in ('-E', '-S'):
            return [], [], 'unsupported:%s output step' % a
        if a in ('-fsyntax-only', '-M', '-MM', '-v', '--version', '-dumpmachine', '-dumpversion', '-###') or a.startswith('-print-'):
            return [], [], 'skip'
        if a in ('-shared', '-r'):
            return [], [], 'unsupported:%s link' % a
        if a in ('-o', '-MF', '-MT', '-MQ'):
            i += 2
            continue
        if a in ('-MD', '-MMD', '-MP', '-MG', '-c'):
            i += 1
            continue
        if a in ('-Xlinker',):
            i += 2
            continue
        if a.startswith(('-o',)) and len(a) > 2 and not a.startswith('-objc'):
            i += 1
            continue
        if a.startswith(LINK_ONLY_PREFIX):
            i += 2 if a in ('-l', '-L') else 1
            continue
        if a in TWO_ARG:
            flags += [a, args[i + 1]] if i + 1 < n else [a]
            i += 2
            continue
        if a.startswith('-'):
            flags.append(a)
            i += 1
            continue
        if a.endswith(SRC_EXT):
            srcs.append(a)
        elif a.endswith(('.o', '.a', '.so', '.obj')) or re.search(r'\.so\.\d', a):
            other.append(a)
        else:
            return [], [], 'unsupported:input file %s' % a
        i += 1
    if not srcs:
        return [], [], 'skip'                   # link of objects only
    return flags, srcs, kind


class Unit:
    """One compile: cc (gcc|g++|cc|c++), cwd (relative to the tree root), flags (list, ROOT_TAG for the tree root), src (relative to cwd)."""
    __slots__ = ('cc', 'cwd', 'flags', 'src', 'stage', 'path', 'tag')

    def __init__(self, cc, cwd, flags, src, stage):
        self.cc, self.cwd, self.flags, self.src, self.stage = cc, cwd, list(flags), src, stage
        self.path = os.path.normpath(os.path.join(cwd, src))
        self.tag = hashlib.sha1(('\0'.join([cc, cwd] + self.flags)).encode()).hexdigest()[:8]

    def argv(self, root, out):
        fl = [f.replace(ROOT_TAG, root) for f in self.flags]
        return ['/usr/bin/' + self.cc] + fl + ['-c', self.src, '-o', out]

    def mmargv(self, root):
        fl = [f.replace(ROOT_TAG, root) for f in self.flags]
        return ['/usr/bin/' + self.cc] + fl + ['-MM', self.src]

    def describe(self):
        return '%s %s %s %s' % (self.stage, self.cc, self.path, ' '.join(self.flags))


def is_port_path(p):
    return p.startswith('src/') or p.startswith('include/')


# ----------------------------------------------------------------------------------------------------------------- unit hashing

_mm_cache = {}


def closure(unit, root):
    """The header closure of a unit (gcc -MM with the unit's own flags) as a sorted list of paths relative to the tree root, or None if gcc fails."""
    k = (unit.cc, unit.cwd, tuple(unit.flags), unit.src)
    if k in _mm_cache:
        return _mm_cache[k]
    cwd = os.path.join(root, unit.cwd)
    r = subprocess.run(unit.mmargv(root), cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    res = None
    if r.returncode == 0:
        txt = r.stdout.replace('\\\n', ' ')
        toks = txt.split(':', 1)[1].split() if ':' in txt else []
        res = sorted({os.path.relpath(os.path.normpath(os.path.join(cwd, t)), root) for t in toks})
    _mm_cache[k] = res
    return res


def unit_cache_key(unit, root, clo):
    h = hashlib.sha256()
    h.update(('%s|%s|%s|%s|%s|%s\n' % (SCHEME, REC_VERSION, tool_hash(), toolchain(), unit.cc, unit.cwd)).encode())
    h.update(('\0'.join(unit.flags) + '\n' + unit.path + '\n').encode())
    for p in clo:
        h.update(('%s %s\n' % (p, sha_file(os.path.join(root, p)))).encode())
    return h.hexdigest()


def compile_hash(unit, root, scratch):
    """Compile the unit exactly as recorded and return its items, or ('ERR', message)."""
    out = os.path.join(scratch, 'u%d_%d.o' % (os.getpid(), id(unit) & 0xffffff))
    env = dict(os.environ, LC_ALL='C', TMPDIR=scratch)
    r = subprocess.run(unit.argv(root, out), cwd=os.path.join(root, unit.cwd), env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    if r.returncode != 0:
        return ('ERR', 'compile failed (rc=%d): %s' % (r.returncode, r.stderr.strip().splitlines()[0] if r.stderr.strip() else ''))
    try:
        return ('OK', [list(i) for i in obj_items(out)])
    finally:
        try:
            os.unlink(out)
        except OSError:
            pass


# ----------------------------------------------------------------------------------------------------------------- recording

def read_log(path, tree_abs):
    recs = []
    try:
        data = open(path, 'rb').read().decode('utf-8', 'replace')
    except OSError:
        return recs
    for ln in data.split('\n'):
        if not ln.startswith('\x1f'):
            continue
        f = ln.split('\x1f')[1:]
        if len(f) < 3:
            continue
        stage, cc, cwd = f[0], f[1], f[2]
        args = f[3:]
        recs.append((stage, cc, cwd, args))
    return recs


def strace_reads(path):
    """Library files (src/, include/) opened by any process that is not the compiler proper (cc1 / cc1plus): the script itself reads them."""
    exe = {}
    reads = set()
    try:
        lines = open(path, 'rb').read().decode('utf-8', 'replace').splitlines()
    except OSError:
        return reads
    for ln in lines:
        m = re.match(r'^(\d+)\s+(execve|openat)\((.*)$', ln)
        if not m:
            continue
        pid, call, rest = m.group(1), m.group(2), m.group(3)
        if ' = -1 ' in rest or rest.rstrip().endswith('= -1'):
            continue
        if call == 'execve':
            mm = re.match(r'"([^"]*)"', rest)
            if mm:
                exe[pid] = os.path.basename(mm.group(1))
        else:
            mm = re.match(r'[A-Z_]+, "([^"]*)"', rest)
            if not mm:
                continue
            p = mm.group(1)
            if re.search(r'(^|/)(src|include)/', p) and not p.startswith('/usr/') and exe.get(pid, '') not in ('cc1', 'cc1plus'):
                if '.' in os.path.basename(p) and 'O_DIRECTORY' not in rest:
                    reads.add(p)
    return reads


def unit_list(recs, stage, tree_abs, problems):
    units = []
    for (st, cc, cwd, args) in recs:
        if st != stage:
            continue
        try:
            rel = os.path.relpath(cwd, tree_abs)
        except ValueError:
            problems.append('cwd %s' % cwd)
            continue
        if rel.startswith('..'):
            problems.append('compile in %s (outside the tree)' % cwd)
            continue
        flags, srcs, kind = parse_cmd(args)
        if kind == 'skip':
            continue
        if kind != 'compile':
            problems.append(kind[len('unsupported:'):])
            continue
        flags = [f.replace(tree_abs, ROOT_TAG) for f in flags]
        srcs = [s.replace(tree_abs + '/', '') if s.startswith(tree_abs) else s for s in srcs]
        for f in flags:
            if '/tmp/' in f or (f.startswith('/') and ROOT_TAG not in f):
                problems.append('absolute path in a flag: %s' % f)
        for s in srcs:
            units.append(Unit(cc, rel, flags, s, stage))
    return units


def record_job(job, real_tree, scratch, shimdir, timeout=1500):
    """Record all stages of one job.  job = dict(id, script, args, g, stages=[(name, env-extra dict, command)])  ->  recipe dict."""
    jid = fid(job['id'])               # the sandbox directory, log and strace names (the recipe itself keeps the real id)
    sbroot = os.path.join(scratch, 'sb')
    r = subprocess.run(['sh', os.path.join(real_tree, 'tools/job_sandbox.sh'), sbroot, jid] + job.get('seeds', []), cwd=real_tree, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    if r.returncode != 0:
        return dict(ok=False, why='sandbox failed: ' + r.stderr[-300:])
    tree = os.path.realpath(r.stdout.strip().splitlines()[-1])
    tmp = os.path.join(sbroot, jid, 'tmp')
    # the flag shim of stage 1p must reach the logging shim, not /usr/bin directly
    for g in ('gcc', 'g++'):
        p = os.path.join(tree, 'test/flagshim', g)
        if os.path.isfile(p):
            t = open(p).read()
            if 'real=/usr/bin/$(basename "$0")' not in t:
                return dict(ok=False, why='test/flagshim/%s changed: recorder cannot redirect it' % g)
            open(p, 'w').write(t.replace('real=/usr/bin/$(basename "$0")', 'real=$SK_SHIMDIR/$(basename "$0")'))
    log = os.path.join(sbroot, jid + '.cmds')
    recipe = dict(ok=True, why='', id=job['id'], stages={}, reads=[], rc={})
    allreads = set()
    for (name, extra_env, cmd) in job['stages']:
        env = dict(os.environ, SK_LOG=log, SK_DIR=shimdir, SK_SHIMDIR=shimdir, SK_STAGE=name, TMPDIR=tmp, LC_ALL='C')
        env['PATH'] = shimdir + ':' + env['PATH']
        for k, v in extra_env.items():
            env[k] = v.replace('@TREE@', tree).replace('@SHIM@', shimdir)
        st = os.path.join(sbroot, jid + '.' + name + '.strace')
        t0 = time.time()
        if shutil.which('strace') is None:
            return dict(ok=False, why='strace is not installed (the recorder needs it to see which library files the script itself reads)')
        try:
            p = subprocess.run(['strace', '-f', '-qq', '-e', 'trace=openat,execve', '-o', st, '-s', '300', 'sh', '-c', cmd + ' </dev/null'], cwd=tree, env=env, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, universal_newlines=True, timeout=timeout)
            rc, tail = p.returncode, p.stdout[-600:]
        except subprocess.TimeoutExpired:
            rc, tail = 124, 'timeout'
        recipe['rc'][name] = [rc, tail, round(time.time() - t0, 1)]
        allreads |= {os.path.relpath(x, tree) if x.startswith('/') else x for x in strace_reads(st)}
        try:
            os.unlink(st)
        except OSError:
            pass
    recs = read_log(log, tree)
    problems = []
    for (name, _, _) in job['stages']:
        us = unit_list(recs, name, tree, problems)
        recipe['stages'][name] = [dict(cc=u.cc, cwd=u.cwd, flags=u.flags, src=u.src) for u in us]
    recipe['reads'] = sorted(allreads)
    recipe['problems'] = sorted(set(problems))
    shutil.rmtree(os.path.join(sbroot, jid), ignore_errors=True)
    for n in os.listdir(sbroot):
        if n.startswith(jid + '.'):
            try:
                os.unlink(os.path.join(sbroot, n))
            except OSError:
                pass
    return recipe


# ----------------------------------------------------------------------------------------------------------------- recipes (cache) / stages

SCRIPTLIKE = ('.sh', '.py', '.mk', '.txt', '.tsv', '.cfg')
SPECIAL_ONE_STAGE = ('cross/z01_hybrid', 'cross/x1_flagdiff')
KB_STAGES = ('s1', 's1p', 's1b1', 's1b2')


def pin_flags(root):
    m = re.search(r'^PIN_FLAGS="([^"]*)"', open(os.path.join(root, 'test/flags.sh')).read(), re.M)
    return m.group(1) if m else ''


def moved_job(root, jid):
    """TESTREORG: None for a job whose files are still the old flat test/<id>... names, else {'module', 'stem', 'blob32' (path under test/golden/ or '')}
    from test/jobs.tsv (the same rule as tools/job_path.sh: the first script of the row has a '/')."""
    try:
        for ln in open(os.path.join(root, 'test/jobs.tsv')):
            f = ln.rstrip('\n').split('\t')
            if ln.startswith('#') or len(f) < 8 or f[0] != jid:
                continue
            if '/' not in f[3].split(',')[0]:
                return None
            nm = f[2].lstrip('?')
            stem = nm.split('/', 1)[1]
            # only the job's OWN golden <module>/<stem>.blob32.out[.gz] (as tools/job_path.sh blob32): the golden of another job (p29a / p29b list p29's) or a facet golden does not count
            bl = [x for x in f[5].split(',') if re.search(r'(^|/)' + re.escape(stem) + r'\.blob32\.out(\.gz)?$', x)]
            return dict(module=nm.split('/')[0], stem=stem, blob32=bl[0] if bl else '',
                        own_g64=any(x.endswith('golden64.sh') and not x.endswith('tier_golden64.sh') for x in f[3].split(',')))
    except OSError:
        pass
    return None


def job_stages(root, jid, script, args, g):
    cmd = ('sh %s %s' % (script, args)).strip()
    st = []
    if jid == 'cross/z01_hybrid':
        st.append(('pre', {}, 'sh test/vpcm/dp_vpcm.sh'))          # makes test/.build/vpcm/dp_vpcm_harness.o / dp_vpcm_stub_envg.o, the seeds of the job
    st.append(('s1', {}, cmd))
    if jid in SPECIAL_ONE_STAGE:
        return st
    text = open(os.path.join(root, script)).read()
    pf = pin_flags(root)
    if re.search(r'-O2|OPT', text):
        st.append(('s1p', {'PATH': '@TREE@/test/flagshim:@SHIM@:' + os.environ['PATH'], 'PORT_XFLAGS': pf}, cmd))
    mv = moved_job(root, jid)           # TESTREORG: the files of a moved job are listed in test/jobs.tsv (golden under test/golden/<module>/, generic stage 2 in test/lib)
    gold, tier = ('test/golden/' + mv['blob32'], 'test/lib/tier_golden64.sh') if mv else ('test/golden/%s_blob32.out.gz' % jid, 'test/lib/tier_golden64.sh')
    if mv is not None and (not mv['blob32'] or mv['own_g64']):   # a moved job with its OWN <stem>.golden64.sh (g64=own: the dsp C++ chunks) has no plain-C stage 1b: its golden is that script's
        gold = None
    if gold and os.path.exists(os.path.join(root, gold)) and os.path.exists(os.path.join(root, tier)):
        st.append(('s1b1', {'TG64_M': '-m32', 'FP64_FLAGS': '-O2'}, 'sh %s %s' % (tier, jid)))
        st.append(('s1b2', {'TG64_M': '-m32', 'FP64_FLAGS': '-O2 ' + pf}, 'sh %s %s' % (tier, jid)))
    if g:
        st.append(('s2', {}, g))
    return st


def recipe_key(root, jid, files, stages):
    h = hashlib.sha256()
    first = lambda s: s.splitlines()[0] if s else ''
    h.update(('%s|rec%s|%s|%s\n' % (SCHEME, REC_VERSION, ' '.join(toolchain().split(' | ')[:2]), jid)).encode())
    h.update(('\n'.join('%s %s %s' % (n, sorted(e.items()), c) for n, e, c in stages) + '\n').encode())
    for f in sorted(set(files)):
        if f.startswith('test/golden/') or is_port_path(f):
            continue
        if f.endswith(SCRIPTLIKE) or 'flagshim' in f or f.endswith('flags.sh'):
            h.update(('%s %s\n' % (f, sha_file(os.path.join(root, f)))).encode())
    h.update(('golden:' + ' '.join(sorted(n for n in os.listdir(os.path.join(root, 'test/golden')) if n.startswith(jid + '_'))) + '\n').encode())
    mv = moved_job(root, jid)
    if mv:                              # a moved job's goldens: test/golden/<module>/<stem>.*
        gd = os.path.join(root, 'test/golden', mv['module'], os.path.dirname(mv['stem']))   # a nested module (v34/handshake/<stem>): the stem carries the subdirectory
        h.update(('golden-moved:' + ' '.join(sorted(n for n in (os.listdir(gd) if os.path.isdir(gd) else []) if n.startswith(os.path.basename(mv['stem']) + '.'))) + '\n').encode())
    for f in ('tools/stampkeys_shim.sh', 'tools/stampkeys_exit0.c'):
        h.update(('%s %s\n' % (f, sha_file(os.path.join(HERE, os.path.basename(f))))).encode())
    return h.hexdigest()


def reads_hash(root, reads):
    return hashlib.sha256('\n'.join('%s %s' % (p, sha_file(os.path.join(root, p))) for p in reads).encode()).hexdigest()


def recipe_problem(recipe):
    """Why a recorded recipe cannot be trusted (None = usable)."""
    if not recipe.get('ok'):
        return recipe.get('why') or 'recording failed'
    rc = recipe['rc']
    if 's1' in rc and rc['s1'][0] != 0:
        return 'the recorder run of stage 1 exited %d (control flow of the script depends on test output)' % rc['s1'][0]
    if 's1p' in rc and rc['s1p'][0] != 0:
        return 'the recorder run of the pinned stage exited %d' % rc['s1p'][0]
    for s in ('s1', 's1b1', 's1b2', 's2'):
        if s in recipe['stages'] and not any(is_port_path(os.path.normpath(os.path.join(u['cwd'], u['src']))) for u in recipe['stages'][s]):
            return 'stage %s compiled no library source' % s
    if recipe.get('problems'):
        return '; '.join(recipe['problems'][:3])
    for s, us in recipe['stages'].items():
        if s != 'pre' and not us:
            return 'stage %s recorded no compile' % s
    return None


def get_recipe(root, jid, files, stages, shimdir, scratch, shim_args):
    rk = recipe_key(root, jid, files, stages)
    rec = cache_get('recipe', rk)
    if rec is not None and rec.get('reads_hash') == reads_hash(root, rec.get('reads', [])):
        return rec, False
    job = dict(id=jid, stages=stages)
    rec = record_job(job, root, scratch, shimdir)
    rec['reads_hash'] = reads_hash(root, rec.get('reads', []))
    if rec.get('ok'):                   # an environment failure (no sandbox, no strace) is not cached; a script that fails in the recorder is (deterministic)
        cache_put('recipe', rk, rec)
    return rec, True


# ----------------------------------------------------------------------------------------------------------------- keys

def _compile_worker(args):
    (cc, cwd, flags, src), root, scratch = args
    u = Unit(cc, cwd, flags, src, '')
    try:
        return compile_hash(u, root, scratch)
    except Exception as e:              # not cached (see do_keys): the unit is source-hashed this time
        return ('EXC', 'object extraction failed: %r' % (e,))


def unit_of(d, stage):
    return Unit(d['cc'], d['cwd'], d['flags'], d['src'], stage)


def make_shim(scratch):
    shim = os.path.join(scratch, 'shim')
    os.makedirs(shim, exist_ok=True)
    for n in ('gcc', 'g++', 'cc', 'c++', 'cmp', 'diff'):
        p = os.path.join(shim, n)
        if not os.path.islink(p):
            os.symlink(os.path.join(HERE, 'stampkeys_shim.sh'), p)
    for m in ('32', '64'):
        o = os.path.join(shim, 'exit0_%s.o' % m)
        if not os.path.exists(o):
            subprocess.check_call([GCC, '-m' + m, '-fPIC', '-c', os.path.join(HERE, 'stampkeys_exit0.c'), '-o', o])
    return shim


_ALIASES = None


def resolve_id(tok):
    """TESTREORG stage 10: old id / planned name -> the active job id (tools/job_aliases.tsv, the rule of tools/job_resolve.sh); any other token unchanged."""
    global _ALIASES
    if _ALIASES is None:
        _ALIASES = {}
        try:
            for ln in open(os.path.join(HERE, 'job_aliases.tsv')):
                f = ln.rstrip('\n').split('\t')
                if ln.startswith('#') or f[0] == 'old_id' or len(f) < 3:
                    continue
                _ALIASES[f[0]] = _ALIASES[f[1]] = _ALIASES[f[2]] = f[1]
        except OSError:
            pass
    return _ALIASES.get(tok, tok)


def fid(jid):
    """File-safe form of a job id ('/' -> '__'): scratch, sandbox, log and key-log names (ids contain '/' and '.'; no id contains '__', tools/jobs_check.sh)."""
    return jid.replace('/', '__')


def read_requests(path):
    reqs = []
    for ln in open(path):
        ln = ln.rstrip('\n')
        if not ln:
            continue
        f = ln.split('\t')
        f += [''] * (6 - len(f))
        reqs.append(dict(id=resolve_id(f[0]), kind=f[1], script=f[2], args=f[3], g=f[4], files=f[5].split()))
    return reqs


def do_keys(reqfile, resfile, nj, root, gccv):
    t0 = time.time()
    reqs = read_requests(reqfile)
    scratch = tempfile.mkdtemp(prefix='stampkeys.', dir=os.environ.get('STAMPKEYS_SCRATCH') or os.environ.get('TMPDIR') or '/tmp')
    try:
        shim = make_shim(scratch)
        byid = collections.OrderedDict()
        for r in reqs:
            byid.setdefault(r['id'], []).append(r)
        # 1. recipes (recorded on a miss, in parallel)
        recipes = {}
        info = collections.defaultdict(list)

        def one(jid):
            rs = byid[jid]
            files = sorted({f for r in rs for f in r['files']})
            g = next((r['g'] for r in rs if r['g']), '')
            r0 = rs[0]
            stages = job_stages(root, jid, r0['script'], r0['args'], g)
            sub = os.path.join(scratch, 'rec_' + fid(jid))
            os.makedirs(sub, exist_ok=True)
            try:
                return jid, get_recipe(root, jid, files, stages, shim, sub, None), stages
            except Exception as e:      # a broken recorder run must not take the keys of the other jobs down: this job falls back (not cached)
                return jid, (dict(ok=False, why='recorder error: %r' % (e,)), False), stages
        with ThreadPoolExecutor(nj) as ex:
            for jid, (rec, fresh), stages in ex.map(one, list(byid)):
                recipes[jid] = rec
                if fresh:
                    info['recorded'].append(jid)
        # 2. units: closure, cache key, items (compile on a miss, in a process pool)
        need = {}                      # unit key -> (unit tuple)
        ukey = {}                      # (cc, cwd, flags, src) -> (clo, cache key) | None
        allunits = set()
        for jid, rec in recipes.items():
            if recipe_problem(rec):
                continue
            for st, us in rec['stages'].items():
                if st == 'pre':
                    continue
                for d in us:
                    allunits.add((d['cc'], d['cwd'], tuple(d['flags']), d['src']))

        def clo_of(t):
            u = Unit(t[0], t[1], list(t[2]), t[3], '')
            c = closure(u, root)
            return t, c
        with ThreadPoolExecutor(max(nj, 4)) as ex:
            for t, c in ex.map(clo_of, sorted(allunits)):
                if c is None:
                    ukey[t] = None
                else:
                    u = Unit(t[0], t[1], list(t[2]), t[3], '')
                    ukey[t] = (c, unit_cache_key(u, root, c))
        results = {}                   # cache key -> ('OK', items) | ('ERR', msg)
        misses = {}
        for t, v in ukey.items():
            if v is None:
                continue
            r = cache_get('obj', v[1])
            if r is not None:
                results[v[1]] = r
            else:
                misses[v[1]] = t
        from concurrent.futures import ProcessPoolExecutor
        if misses:
            # forkserver: forking after the recorder threads above can deadlock the pool (Python 3.12+)
            import multiprocessing
            with ProcessPoolExecutor(nj, mp_context=multiprocessing.get_context('forkserver')) as ex:
                keys = list(misses)
                for k, res in zip(keys, ex.map(_compile_worker, [(misses[k], root, scratch) for k in keys])):
                    if res[0] == 'EXC':
                        results[k] = ('ERR', res[1])
                        continue
                    results[k] = res
                    cache_put('obj', k, res)
        # 3. one key per request
        out = []
        keylog = os.environ.get('STAMPKEYS_KEYLOG') or None
        for r in reqs:
            rec = recipes[r['id']]
            why = recipe_problem(rec)
            if why:
                out.append((r['id'], r['kind'], 'FALLBACK', why))
                continue
            stnames = list(KB_STAGES) + (['s2'] if r['kind'] == 'KF' else [])
            units = collections.OrderedDict()
            bad = None
            for st in stnames:
                for d in rec['stages'].get(st, []):
                    t = (d['cc'], d['cwd'], tuple(d['flags']), d['src'])
                    units[t] = 1
            S = set(f for f in r['files'] if os.path.isfile(os.path.join(root, f)))
            S |= set(rec.get('reads', []))
            if os.environ.get('SK_BLOB') and os.path.isfile(os.path.join(root, os.environ['SK_BLOB'])):
                S.add(os.environ['SK_BLOB'])      # the oracle blob (the old key listed it too, but its *.o filter dropped it: a blob change now invalidates)
            covered = set()
            forced = set(rec.get('reads', []))      # library files the recorded run read outside the compiler, or whose compile failed: source text
            lines = []
            obj_lines = []
            errs = []
            for t in units:
                v = ukey.get(t)
                if v is None:
                    bad = 'cannot preprocess %s (%s): header closure unavailable' % (t[3], ' '.join(t[2]))
                    break
                clo, ck = v
                u = Unit(t[0], t[1], list(t[2]), t[3], '')
                res = results[ck]
                lines.append('U %s %s' % (u.tag, u.path))
                if res[0] != 'OK':
                    errs.append((u, clo, res[1]))
                    lines.append('E %s %s' % (u.tag, u.path))
                    S |= set(clo) | {u.path}
                    forced |= set(clo) | {u.path}
                    continue
                for p in clo:
                    if is_port_path(p):
                        covered.add(p)
                    else:
                        S.add(p)                         # harness header: source text (the object hash of the harness unit is added to it)
                if not is_port_path(u.path):
                    S.add(u.path)
                for kind, name, h in res[1]:
                    obj_lines.append('O %s %s %s %s' % (u.tag, kind, name, h))
            if bad:
                out.append((r['id'], r['kind'], 'FALLBACK', bad))
                continue
            # a library file the job's script names (or the old key added through test/libfiles_by_token.txt) but that no compile of the recorded run
            # opened and no process read cannot influence the job: dropped; one in a unit's header closure is covered by the object hash
            S2 = sorted(f for f in S if not is_port_path(f) or f in forced)
            parts = ['scheme=' + SCHEME, gccv, 'id=' + r['id'] + ('.g64' if r['kind'] == 'KF' else '')]
            for f in S2:
                parts.append('S %s %s' % (sha_file(os.path.join(root, f)), f))
            parts += sorted(set(lines))
            parts += sorted(obj_lines)
            txt = '\n'.join(parts) + '\n'
            key = hashlib.sha256(txt.encode()).hexdigest()[:32]
            nsrc = sum(1 for f in S2 if is_port_path(f))
            inf = 'units=%d items=%d srchashed=%d portsrc_srchashed=%d%s' % (len(set(lines)) - len(errs), len(obj_lines), len(S2), nsrc,
                                                                        (' partial(compile-error units: %s)' % ','.join(sorted({u.path for u, _, _ in errs}))) if errs else '')
            out.append((r['id'], r['kind'], 'OBJ', key, inf))
            if keylog:
                os.makedirs(keylog, exist_ok=True)
                open(os.path.join(keylog, fid(r['id']) + ('.g64' if r['kind'] == 'KF' else '')), 'w').write(txt)
        with open(resfile, 'w') as f:
            for o in out:
                f.write('\t'.join(o) + '\n')
        total, gone = cache_trim()
        sys.stderr.write('stampkeys: %d requests, %d recipes recorded%s, %d units (%d compiled), %.1fs, cache %d KB%s\n' % (
            len(reqs), len(info['recorded']), (' (' + ' '.join(info['recorded']) + ')') if info['recorded'] else '', len(allunits), len(misses), time.time() - t0, total // 1024,
            ', trimmed %d entries' % gone if gone else ''))
    finally:
        shutil.rmtree(scratch, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd')
    k = sub.add_parser('keys')
    k.add_argument('req')
    k.add_argument('res')
    k.add_argument('-j', type=int, default=os.cpu_count() or 4)
    k.add_argument('--root', default=os.path.dirname(HERE))
    o = sub.add_parser('objhash')
    o.add_argument('obj')
    sub.add_parser('cache-trim')
    a = ap.parse_args()
    if a.cmd == 'objhash':
        for it in obj_items(a.obj):
            print(*it)
    elif a.cmd == 'cache-trim':
        print(cache_trim())
    elif a.cmd == 'keys':
        do_keys(a.req, a.res, a.j, os.path.abspath(a.root), os.environ.get('SK_GCCV', toolchain().split(' | ')[0]))
    else:
        ap.print_help()
        sys.exit(2)


if __name__ == '__main__':
    main()
