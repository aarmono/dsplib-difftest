#!/usr/bin/env python3
"""gen_harness.py - regenerate (or, with --check, verify) the harness-side files that are derived from the library's layout descriptions.

  python3 tools/gen_harness.py           write them
  python3 tools/gen_harness.py --check   exit 1 and list the files that differ from a fresh generation (no file is written)

Inputs: layouts/**/*.layout (symlink into the dsplib submodule) and test/layouts.tsv (one row per description: the arguments of tools/layout_gen.py
for its --out-test mirror header and --out-probe offsetof probe).  The V.34 offset maps (MAPS below) come from tools/tier_rewrite.py mapheader.
The library-side outputs of the same descriptions (src/**/*_layout.inc and the V.34 views) are generated in the library repository (tools/gen_views.sh).
Run from the repository root; both scripts are the library's own (tools/layout_gen.py, tools/tier_rewrite.py are symlinks into ext/dsplib).
"""
import os
import shlex
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAPS = [   # (arguments of tier_rewrite.py mapheader, output)
    ([], 'test/v34/v34objmap.h'),
    (['--layout', 'layouts/vpcm/vpcm_flo_modem.layout', '--class', 'VPcmFloModem', '--name', 'V34CTX', '--inc', '../src/v34/v34_flomodem_view.h'], 'test/v34/v34ctxmap.h'),
    (['--layout', 'layouts/vpcm/pcm_parameters.layout', '--class', 'V90Parameters', '--name', 'V34CFG', '--inc', '../src/v34/v34_v90params_view.h'], 'test/v34/v34cfgmap.h'),
]


def run(cmd):
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    if r.returncode:
        print('gen_harness: failed: %s\n%s' % (' '.join(cmd), (r.stderr or r.stdout).strip()[-400:]), file=sys.stderr)
        sys.exit(2)
    return r


def main():
    check = '--check' in sys.argv[1:]
    bad, n = [], 0
    with tempfile.TemporaryDirectory(prefix='gen_harness.') as tmp:
        for ln in open(os.path.join(ROOT, 'test/layouts.tsv')):
            if not ln.strip() or ln.startswith('#'):
                continue
            desc, args = ln.rstrip('\n').split('\t', 1)
            a, outs = shlex.split(args), []
            for k, x in enumerate(a):
                if x in ('--out-test', '--out-probe'):
                    outs.append(a[k + 1])
                    if check:
                        a[k + 1] = os.path.join(tmp, a[k + 1].replace('/', '__'))
            run([sys.executable, 'tools/layout_gen.py', desc] + a)
            for o in outs:
                n += 1
                if check:
                    g = os.path.join(tmp, o.replace('/', '__'))
                    if not os.path.exists(os.path.join(ROOT, o)) or open(os.path.join(ROOT, o), 'rb').read() != open(g, 'rb').read():
                        bad.append(o)
        for margs, out in MAPS:
            n += 1
            r = subprocess.run([sys.executable, 'tools/tier_rewrite.py', 'mapheader'] + margs + ['-o', out] + (['--check'] if check else []), cwd=ROOT, capture_output=True, text=True)
            if r.returncode:
                if check:
                    bad.append(out)
                else:
                    print('gen_harness: mapheader %s failed:\n%s' % (out, (r.stderr or r.stdout)[-400:]), file=sys.stderr)
                    sys.exit(2)
    if bad:
        print('gen_harness: files differ from a fresh generation:', file=sys.stderr)
        for b in bad:
            print('  ' + b, file=sys.stderr)
        sys.exit(1)
    print('gen_harness: %d files %s' % (n, 'up to date' if check else 'written'))


if __name__ == '__main__':
    main()
