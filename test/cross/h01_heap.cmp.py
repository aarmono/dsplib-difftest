#!/usr/bin/env python3
"""H01: compare the heap-block dumps of the blob and port test binaries under two heap fills.

Usage: h01_heap.cmp.py DUMPDIR [--lo 0] [--hi 165] [--expect FILE]
DUMPDIR holds the files written by test/stub_env.c at exit (H01_DUMP=DUMPDIR H01_HEAPFILL=<byte>): <exe>.<argv1>.f<fill>.txt, one per run.
An executable pair is <x>_blob / <x>_port (e.g. v23_test_blob / v23_test_port).  For every pair and scenario argument:
  (1) the allocation sequence (count, sizes, freed flags) must be the same;
  (2) the blocks dumped after the fill-0 runs must be equal, and after the fill-0xa5 runs must be equal (pointers are normalised by stub_env.c);
  (3) the set of words that differ between the fill-0 and the fill-0xa5 run (= never written by the code under test) must be the same for blob and port.
(2) fails when the port leaves a field uninitialised that the blob writes (or writes a different value); (3) is the same statement independent of the
fill.  EXPECT lines ("<exe-prefix|*> S<block size> W<word> <hi|any>  # reason"; hi = only the high 16 bits of the word differ) list known, explained differences; anything not listed fails.
Exit 0 iff all pairs agree (modulo EXPECT)."""
import sys, os, glob, re, collections


def load(path):
    blocks = []
    cur = None
    for line in open(path):
        line = line.rstrip('\n')
        if line.startswith('B '):
            _, idx, size, state = line.split()
            cur = {'idx': int(idx), 'size': int(size), 'freed': state, 'w': []}
            blocks.append(cur)
        elif cur is not None:
            cur['w'] = line.split()
    return blocks


def main():
    args = sys.argv[1:]
    d = args[0]
    lo, hi = '0', '165'
    expect = []   # (pair-prefix or '*', size, word, kind)
    i = 1
    while i < len(args):
        if args[i] == '--lo': lo = args[i + 1]; i += 2
        elif args[i] == '--hi': hi = args[i + 1]; i += 2
        elif args[i] == '--expect':
            for l in open(args[i + 1]):
                l = l.split('#')[0].split()
                if len(l) >= 4: expect.append((l[0], int(l[1][1:]), int(l[2][1:]), l[3]))
            i += 2
        else: i += 1
    files = glob.glob(os.path.join(d, '*.f*.txt'))
    runs = collections.defaultdict(dict)  # (pair, arg) -> {(side, fill): path}
    for f in files:
        b = os.path.basename(f)
        m = re.match(r'(.+)_(blob|port)\.(.*)\.f(\d+)\.txt$', b)
        if not m: continue
        runs[(m.group(1), m.group(3))][(m.group(2), m.group(4))] = f
    npairs = nok = 0
    fails = []
    used = set()

    def allowed(pair, size, wi, x, y):
        for k, (pre, sz, w, kind) in enumerate(expect):
            if (pre == '*' or pair.startswith(pre)) and sz == size and w == wi:
                if kind == 'hi' and not (len(x) == 8 and len(y) == 8 and x[4:] == y[4:]):
                    continue
                used.add(k)
                return True
        return False

    for (pair, arg), r in sorted(runs.items()):
        need = [('blob', lo), ('blob', hi), ('port', lo), ('port', hi)]
        key = '%s.%s' % (pair, arg)
        if not all(k in r for k in need):
            fails.append((key, 'missing dumps %s' % [k for k in need if k not in r])); continue
        npairs += 1
        D = {k: load(r[k]) for k in need}
        problems = []
        sig = lambda bl: [(b['idx'], b['size'], b['freed']) for b in bl]
        for fill in (lo, hi):
            if sig(D[('blob', fill)]) != sig(D[('port', fill)]):
                problems.append('allocation sequence differs at fill %s (blob %d blocks, port %d)' % (fill, len(D[('blob', fill)]), len(D[('port', fill)])))
        if not problems:
            for fill in (lo, hi):
                for bi, (bb, pb) in enumerate(zip(D[('blob', fill)], D[('port', fill)])):
                    for wi, (x, y) in enumerate(zip(bb['w'], pb['w'])):
                        if x != y and not allowed(pair, bb['size'], wi, x, y):
                            problems.append('fill %s block %d (size %d) word %d: blob %s port %s' % (fill, bb['idx'], bb['size'], wi, x, y))
            # words never written: those that differ between the two fills
            for bi, (a0, a1, c0, c1) in enumerate(zip(D[('blob', lo)], D[('blob', hi)], D[('port', lo)], D[('port', hi)])):
                ub = {j for j, (x, y) in enumerate(zip(a0['w'], a1['w'])) if x != y}
                up = {j for j, (x, y) in enumerate(zip(c0['w'], c1['w'])) if x != y}
                for j in sorted(ub ^ up):
                    if not allowed(pair, a0['size'], j, a0['w'][j], c0['w'][j]):
                        problems.append('block %d word %d: %s by the blob, %s by the port (differs between the two fills = never written)' % (a0['idx'], j, 'unwritten' if j in ub else 'written', 'unwritten' if j in up else 'written'))
        if problems:
            fails.append((key, '; '.join(problems[:4]) + (' (+%d more)' % (len(problems) - 4) if len(problems) > 4 else '')))
        else:
            nok += 1
    for key, why in fails:
        print('HEAPCMP DIFFERENCE %s: %s' % (key, why))
    for k, e in enumerate(expect):
        if k not in used and e[3] != 'any':   # 'any' entries (stack garbage that varies run to run) may or may not show
            print('HEAPCMP EXPECTED DIFFERENCE NOT SEEN (stale): %s S%d W%d %s' % e)
            fails.append((str(e), 'stale expectation'))
    if fails:
        print('HEAPCMP MISMATCH: %d pairs compared, %d equal, %d different' % (npairs, nok, len(fails)))
        return 1
    print('HEAPCMP MATCH: %d exe/scenario pairs: allocation sequences, block contents under fills %s/%s and the never-written word sets are identical blob vs port' % (npairs, lo, hi))
    return 0


if __name__ == '__main__':
    sys.exit(main())
