#!/usr/bin/env python3
"""FNV-1a 64 of a P16 table, computed from the bytes of the blob's .data (readelf -sW / -SW: no Ghidra), for the check in v92_tx_coding.sh."""
import subprocess, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'tools'))
from blobpath import BLOB_O   # the blob oracle's location: test/blob.sh
obj = BLOB_O
NAMES = {'coset': '_ZN21V92ConvolutionEncoder14cosetMapping4DE', 'subset': '_ZN21V92ConvolutionEncoder16subsetLabelTableE'}
name = NAMES[sys.argv[1]]
syms = [l.split() for l in subprocess.check_output(['readelf', '-sW', obj]).decode().splitlines() if l.split()[-1:] == [name]]
assert len(syms) == 1, syms
val, size, shndx = int(syms[0][1], 16), int(syms[0][2]), int(syms[0][6])
secs = {}
for l in subprocess.check_output(['readelf', '-SW', obj]).decode().splitlines():
    l = l.replace('[', ' ').replace(']', ' ').split()
    if len(l) > 5 and l[0].isdigit() and int(l[0]) == shndx:
        off = int(l[4], 16)
data = open(obj, 'rb').read()[off + val: off + val + size]
h = 0xcbf29ce484222325
for b in data:
    h ^= b
    h = (h * 0x100000001b3) & 0xffffffffffffffff
print('%016x' % h)
