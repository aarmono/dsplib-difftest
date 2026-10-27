#!/usr/bin/env python3
"""H01: content check of every DATA symbol a port object redefines.

For each GLOBAL data symbol (object/untyped symbol in a non-text section) defined in the blob (argv[1]) that a port object also defines strongly under the same
name, compare size, bytes (.bss = zero) and relocations (offset inside the symbol, type, target name + addend) between the blob and the port object.

Why: N01 found two bugs of exactly this class by hand (B103's AGC_DEF_* tables took the name of a different blob global; the port's 8-entry FPM_sin_sign
replaced a 4-entry blob global that unported code still reads).  The hybrid builder only *reports* size differences; this makes the full check a test.

Usage: h01_datasym.py BLOB.o EXPECTED.txt PORT1.o PORT2.o ...
EXPECTED.txt: one line per known difference "<symbol> <kind>  # reason" (kinds: SIZE, BYTES, RELOC).  Exit 0 iff the computed set == the expected set.
Prints "DATASYM MATCH: n symbols compared ..." on success.
"""
import sys
from elftools.elf.elffile import ELFFile
from elftools.elf.relocation import RelocationSection


def load(path):
    f = open(path, 'rb')
    elf = ELFFile(f)
    secs = list(elf.iter_sections())
    symtab = elf.get_section_by_name('.symtab')
    syms = list(symtab.iter_symbols())
    # relocations per target section index
    rels = {}
    for s in secs:
        if isinstance(s, RelocationSection):
            tgt = s['sh_info']
            rels.setdefault(tgt, []).extend(s.iter_relocations())
    return elf, secs, syms, rels


def target_name(secs, syms, r):
    sym = syms[r['r_info_sym']]
    if sym['st_info']['type'] == 'STT_SECTION':
        # section symbol: name the section, the addend (or implicit addend) is compared separately
        return secs[sym['st_shndx']].name
    return sym.name


def data_syms(path, want_weak=False):
    elf, secs, syms, rels = load(path)
    out = {}
    for s in syms:
        if s['st_shndx'] in ('SHN_UNDEF', 'SHN_ABS', 'SHN_COMMON'):
            continue
        t = s['st_info']['type']
        b = s['st_info']['bind']
        if t not in ('STT_OBJECT', 'STT_NOTYPE'):
            continue
        sec = secs[s['st_shndx']]
        if sec['sh_flags'] & 0x4:  # SHF_EXECINSTR
            continue
        if b == 'STB_LOCAL' or (b == 'STB_WEAK' and not want_weak):
            continue
        if not s.name:
            continue
        size = s['st_size']
        off = s['st_value']
        if sec['sh_type'] == 'SHT_NOBITS':
            data = bytes(size)
        else:
            data = sec.data()[off:off + size]
        rl = []
        for r in rels.get(s['st_shndx'], []):
            ro = r['r_offset']
            if off <= ro < off + max(size, 1):
                # REL (no addend): the addend is the word in the section
                if 'r_addend' in r.entry:
                    add = r['r_addend']
                else:
                    add = int.from_bytes(sec.data()[ro:ro + 4], 'little', signed=True)
                rl.append((ro - off, r['r_info_type'], target_name(secs, syms, r), add))
        out[s.name] = (size, data, tuple(sorted(rl)), sec.name)
    return out


def main():
    blob_path, exp_path = sys.argv[1], sys.argv[2]
    blob = data_syms(blob_path, want_weak=True)
    port = {}
    for p in sys.argv[3:]:
        for k, v in data_syms(p).items():
            port[k] = v  # strong definitions only; a duplicate strong would fail the link anyway
    diffs = {}
    n = 0
    for k, (psize, pdata, prel, psec) in sorted(port.items()):
        if k not in blob:
            continue
        n += 1
        bsize, bdata, brel, bsec = blob[k]
        if bsize != psize:
            diffs[(k, 'SIZE')] = f'blob {bsize} port {psize}'
            continue
        # relocated words differ by construction (the implicit addend is in the data): compare the data with relocated words masked
        mask = bytearray(b'\x01' * bsize)
        for (o, t, nm, a) in brel + prel:
            for i in range(o, min(o + 4, bsize)):
                mask[i] = 0
        bd = bytes(x if m else 0 for x, m in zip(bdata, mask))
        pd = bytes(x if m else 0 for x, m in zip(pdata, mask))
        if bd != pd:
            first = next(i for i in range(bsize) if bd[i] != pd[i])
            diffs[(k, 'BYTES')] = f'first diff at +{first}'
        if brel != prel:
            diffs[(k, 'RELOC')] = f'blob {len(brel)} relocs port {len(prel)} relocs'
    exp = {}
    for line in open(exp_path):
        line = line.split('#')[0].split()
        if len(line) >= 2:
            exp[(line[0], line[1])] = 1
    new = sorted(set(diffs) - set(exp))
    gone = sorted(set(exp) - set(diffs))
    for k in new:
        print('DATASYM NEW DIFFERENCE: %s %s (%s)' % (k[0], k[1], diffs[k]))
    for k in gone:
        print('DATASYM EXPECTED DIFFERENCE GONE: %s %s' % k)
    if new or gone:
        print('DATASYM MISMATCH: compared %d, new %d, gone %d' % (n, len(new), len(gone)))
        return 1
    print('DATASYM MATCH: %d replaced data symbols compared with the blob, %d documented differences' % (n, len(diffs)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
