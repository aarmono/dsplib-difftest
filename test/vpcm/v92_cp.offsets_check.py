#!/usr/bin/env python3
"""Checks that every byte offset by which src/vpcm/pcm_leaf_classes.cpp reads a V90MappingParams block (`pw(p, 0x620)`, `pw(p, 0)`, `0x604 + 4 * k`, `0x304u`, `0x4u` in the
V90ConstellationPower functions) appears in test/vpcm/v92_cp.offsets.cpp (which asserts, at compile time on i386, that it is the offset of a named member)."""
import re, sys, os
here = os.path.dirname(os.path.abspath(__file__))
root = os.path.join(here, '..', '..')
asserted = set()
for l in open(os.path.join(here, 'v92_cp.offsets.cpp')):
    m = re.match(r'^AMP\(\w+, (0x[0-9a-f]+),', l)
    if m:
        asserted.add(int(m.group(1), 16))
src = open(os.path.join(root, 'src/vpcm', 'pcm_leaf_classes.cpp')).read()
a = src.index('static inline unsigned mpRadix')
b = src.index('float V90ConstellationPower::getPower')
b = src.index('\n}\n', b)
region = src[a:b]
used = set()
for m in re.finditer(r'\bpw\(p,\s*(0x[0-9a-f]+|0)\)', region):
    used.add(int(m.group(1), 0))
for m in re.finditer(r'0x(604|304|4)u?\b', region):
    used.add(int(m.group(1), 16))
miss = sorted(used - asserted)
print('V90MappingParams: %d byte offsets used by P06, %d asserted members%s' % (len(used), len(asserted), '' if not miss else '  MISSING: ' + ' '.join(hex(x) for x in miss)))
sys.exit(1 if miss else 0)
