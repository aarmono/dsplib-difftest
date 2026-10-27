#!/usr/bin/env python3
"""p05_vtable_cmp.py BLOB.o PORT.o  -  compare the vtables of the Resampler classes of the blob and of the port object.

For every _ZTV* symbol of the 4 classes: binding, size, the section bytes (the typeinfo / offset-to-top words and every slot that carries
a relocation must be zero before relocation) and the list (word offset -> relocated symbol name) of the slots.  The relocated symbol names are
taken from the relocation sections of the vtable's own section (.gnu.linkonce.r._ZTV* in the blob, .data.rel.ro.local._ZTV* in a modern
g++ object), so the slot order and the function in every slot are compared, not addresses.  Exit status 1 on any difference.
Output: one line per vtable.
"""
import re
import subprocess
import sys

CLASSES = ['_ZTV9Resampler', '_ZTV21ResamplerTimingOffset', '_ZTV15ResamplerTiming', '_ZTV12V90Resampler']


def run(*a):
    return subprocess.run(a, check=True, capture_output=True, text=True).stdout


def read(path):
    syms = {}
    for ln in run('readelf', '-sW', path).split('\n'):
        t = ln.split()
        if len(t) >= 8 and t[3] == 'OBJECT' and t[7] in CLASSES:
            syms[t[7]] = {'size': int(t[2]), 'bind': t[4], 'shndx': int(t[6]) if t[6].isdigit() else -1}
    secs = {}
    for ln in run('readelf', '-SW', path).split('\n'):
        m = re.match(r'\s*\[\s*(\d+)\]\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)', ln)
        if m:
            secs[int(m.group(1))] = (m.group(2), int(m.group(5), 16))
    rel = {}
    cur = None
    for ln in run('readelf', '-rW', path).split('\n'):
        m = re.match(r"Relocation section '(\S+)'", ln)
        if m:
            cur = m.group(1)
            rel[cur] = []
            continue
        t = ln.split()
        if cur and len(t) >= 5 and re.fullmatch(r'[0-9a-f]{8}', t[0]) and t[2].startswith('R_386'):
            rel[cur].append((int(t[0], 16), t[2], t[4] if len(t) > 4 else ''))
    data = open(path, 'rb').read()
    out = {}
    for name, s in syms.items():
        sname, foff = secs[s['shndx']]
        relname = '.rel' + sname
        slots = sorted(rel.get(relname, []))
        words = [int.from_bytes(data[foff + i:foff + i + 4], 'little') for i in range(0, s['size'], 4)]
        out[name] = (s['bind'], s['size'], words, [(o, ty, sy) for o, ty, sy in slots])
    return out


def main():
    a, b = read(sys.argv[1]), read(sys.argv[2])
    bad = 0
    for c in CLASSES:
        if c not in a or c not in b:
            print('%s: MISSING (blob %s, port %s)' % (c, c in a, c in b))
            bad += 1
            continue
        ba, sa, wa, ra = a[c]
        bb, sb, wb, rb = b[c]
        ok = (ba == bb and sa == sb and wa == wb and [(o, ty, sy) for o, ty, sy in ra] == [(o, ty, sy) for o, ty, sy in rb])
        print('%-28s %s  size %d  slots: %s' % (c, 'SAME' if ok else 'DIFFERENT', sa, ', '.join('%d=%s' % (o // 4, sy) for o, ty, sy in ra)))
        if not ok:
            bad += 1
            print('   blob: %s %d %s %s' % (ba, sa, wa, ra))
            print('   port: %s %d %s %s' % (bb, sb, wb, rb))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
