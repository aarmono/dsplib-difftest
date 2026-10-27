#!/usr/bin/env python3
"""Checks that every byte offset by which src/dsp/dsp_resampler.cpp (paramWord) and src/vpcm/pcm_leaf_classes.cpp (pw, in the V90 / V92Phase2Info functions) reads a
V90Parameters / V92Parameters block appears in test/vpcm/pcm_parameters.offsets.cpp (which asserts, at compile time on i386, that it is the offset of a named member).
Both files now read the members by name, so the expected result is 0 offsets used; a byte-offset read that comes back is wrong on a 64-bit build."""
import re, sys, os
here = os.path.dirname(os.path.abspath(__file__))
root = os.path.join(here, '..', '..')
asserted = {'V90': set(), 'V92': set()}
for l in open(os.path.join(here, 'pcm_parameters.offsets.cpp')):
    m = re.match(r'^A(90|92)\(\w+, (0x[0-9a-f]+),', l)
    if m:
        asserted['V' + m.group(1)].add(int(m.group(2), 16))
used = {'V90': set(), 'V92': set()}
for l in open(os.path.join(root, 'src/dsp', 'dsp_resampler.cpp')):
    for m in re.finditer(r'paramWord\([^,()]+,\s*(0x[0-9a-f]+)\)', l):
        used['V90'].add(int(m.group(1), 16))
cls = None
for l in open(os.path.join(root, 'src/vpcm', 'pcm_leaf_classes.cpp')):
    m = re.match(r'^[\w:]*?(V90|V92)Phase2Info::', l)
    if m and not l.startswith(' '):
        cls = m.group(1)
    elif re.match(r'^\S', l) and not re.match(r'^(V90|V92)Phase2Info::', l):
        if not l.startswith('}') : cls = None if not re.match(r'^(void|short|int|unsigned|static)\b.*(V90|V92)Phase2Info::', l) else cls
    for m in re.finditer(r'\bpw\(p,\s*(0x[0-9a-f]+)\)', l):
        if cls:
            used[cls].add(int(m.group(1), 16))
bad = 0
for k in ('V90', 'V92'):
    miss = sorted(used[k] - asserted[k])
    print('%sParameters: %d byte offsets used by P05 / P06, %d asserted members%s' % (k, len(used[k]), len(asserted[k]), '' if not miss else '  MISSING: ' + ' '.join(hex(x) for x in miss)))
    bad += len(miss) + len(used[k])  # any remaining byte-offset read is a defect at x86-64
sys.exit(1 if bad else 0)
