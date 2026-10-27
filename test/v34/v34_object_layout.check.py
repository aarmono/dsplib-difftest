#!/usr/bin/env python3
"""X3: the V.34 module object exists in TWO spellings that must agree byte for byte on EVERY host: V34Obj (src/v34/v34_object_view.h, C: the tier files) and
tagV34Object (src/v34/v34_object.h, C++: V01 - V07, z01_glue).  On i386 both are the blob's 0xac4c-byte object; on x86-64 v34_object_view.h widens every pointer
slot (41 of them) and the C++ view must shift by exactly the same amounts (V34_PAD64 in v34_object.h).  Before X3 they disagreed for 199 of the 229 common members
at -m64 and a real session crashed in VPcmV34Create (the C tiers wrote over obj->params_).  This script compiles a probe at -m32 and -m64 and compares offsetof()
of every member the two views have in common, and sizeof.  Usage: v34_object_layout.check.py   (from slmodemd/dsplibs-port).  Exit 0 and 'MATCH' iff identical."""
import os, re, subprocess, sys, tempfile


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        print(r.stderr[:2000])
        sys.exit(3)
    return r.stdout


def main():
    src = open('src/v34/v34_object_view.h').read()   # EOS: the single definition (RS5) lives in the generated view
    a = src.index('typedef struct tagV34Object {')
    b = src.index('\n} V34Obj;', a)
    names = []
    for l in src[a:b].split('\n')[1:]:
        l = l.split('/*')[0].strip()
        m = re.match(r'.*?([A-Za-z_][A-Za-z0-9_]*)(\[[^\]]*\])?;', l)
        if m and not m.group(1).startswith('pad_'):
            names.append(m.group(1))
    tmp = tempfile.mkdtemp(prefix='x3lay.')
    bad_total = 0
    for bits in ('32', '64'):
        cpp = '#include <stddef.h>\n#include <stdio.h>\n#include "v34/v34_object.h"\nint main(){\n' + ''.join(
            'printf("%s %%zu\\n",offsetof(tagV34Object,%s));\n' % (n, n) for n in names) + 'printf("SIZE %zu\\n",sizeof(tagV34Object));return 0;}'
        open(tmp + '/p.cpp', 'w').write(cpp)
        run(['g++', '-m' + bits, '-w', '-fno-exceptions', '-fno-rtti', '-Isrc', '-c', '-o', tmp + '/p.o', tmp + '/p.cpp'])
        run(['gcc', '-m' + bits, '-o', tmp + '/p', tmp + '/p.o'])
        cp = dict((l.split()[0], int(l.split()[1])) for l in run([tmp + '/p']).split('\n') if l)
        good = list(names)
        while True:   # the members the C view also has (about 20 names exist in one view only)
            c = '#include <stddef.h>\n#include <stdio.h>\n#include "v34/v34_object_view.h"\nint main(){\n' + ''.join(
                'printf("%s %%zu\\n",offsetof(V34Obj,%s));\n' % (n, n) for n in good) + 'printf("SIZE %zu\\n",sizeof(V34Obj));return 0;}'
            open(tmp + '/c.c', 'w').write(c)
            r = subprocess.run(['gcc', '-m' + bits, '-w', '-Isrc', '-o', tmp + '/c', tmp + '/c.c'], capture_output=True, text=True)
            bad = set(re.findall(r"no member named .(\w+).", r.stderr))
            if r.returncode and not bad:
                print(r.stderr[:2000])
                sys.exit(3)
            if not bad:
                break
            good = [g for g in good if g not in bad]
        cc = dict((l.split()[0], int(l.split()[1])) for l in run([tmp + '/c']).split('\n') if l)
        diff = [(n, cp[n], cc[n]) for n in good if cp[n] != cc[n]]
        print('-m%s: %d common members, sizeof C++ %d, C %d, %d differ %s' % (bits, len(good), cp['SIZE'], cc['SIZE'], len(diff), diff[:5]))
        if diff or cp['SIZE'] != cc['SIZE'] or len(good) < 150:
            bad_total += 1
    for f in os.listdir(tmp):
        os.remove(os.path.join(tmp, f))
    os.rmdir(tmp)
    if bad_total:
        print('MISMATCH: the C and C++ views of the V.34 object differ')
        sys.exit(1)
    print('MATCH: V34Obj and tagV34Object have the same member offsets and size at -m32 and -m64')


main()
