#!/bin/sh
# cfgptr_check.sh   (run from slmodemd/dsplibs-port/)   T03
# The pointer words of the config blocks (src/core/dsp_table_handles.h) must mean the same thing at -m32 (word == address) and x86-64 (word == handle):
# the pointer entries of every `const int X_CFG_raw[]` (found by scanning the table sources for CFGP(sym)) are checked to resolve with cfgp_ptr() to
# exactly `sym`, every non-pointer word and the 6-slot ECCv32_IMAP/QMAP arrays are printed, and the output of the -m32 and the x86-64 build must be
# IDENTICAL.  Also checks cfgp_word(sym) == CFGP(sym) round trips for every registered symbol.  Exit 0 = identical and no BAD.
T="${TPREFIX:-test/.build/cfgptr_check}"
mkdir -p "$(dirname "$T")"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
FILES="src/fpm/fpm_tables.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c src/fsk/fsk_tables.c src/fsk/v23_backchannel_tables.c src/core/dsp_table_handles.c"
python3 - <<'PY' > $T.c || exit 2
import re
arrs=[]
for f in ['src/fpm/fpm_tables.c','src/v32/v32_tables.c','src/fsk/fsk_tables.c','src/fsk/v23_backchannel_tables.c']:
    s=open(f).read()
    for m in re.finditer(r'const int (\w+)\[(\d+)\] = \{(.*?)\};',s,re.S):
        name,n,body=m.groups()
        body=re.sub(r'/\*.*?\*/','',body,flags=re.S)
        els=[e.strip() for e in body.split(',') if e.strip()]
        if any('CFGP(' in e for e in els):
            arrs.append((name,[(re.search(r'CFGP\((\w+)\)',e).group(1) if 'CFGP(' in e else None) for e in els]))
print('#include <stdio.h>\n#include <stdint.h>\n#include "dsp_table_handles.h"\n#include "fpm_tables.h"\n#include "v32_tables.h"\n#include "v32_viterbi_tables.h"\n#include "fsk_tables.h"\n#include "v23_backchannel_tables.h"')
print('static int bad;\nstatic void chk(const char *a, int i, int w, const void *sym, const char *sn){ int ok = cfgp_ptr(w) == sym && cfgp_word(sym) == w; if(!ok) bad++; printf("%s[%d] -> %s %s\\n", a, i, sn, ok ? "ok" : "BAD"); }')
print('int main(void){')
for name,els in arrs:
    for i,e in enumerate(els):
        if e: print('chk("%s", %d, %s[%d], %s, "%s");'%(name,i,name,i,e,e))
        else: print('printf("%s[%d] = 0x%%08x\\n", (unsigned)%s[%d]);'%(name,i,name,i))
for t,k in (('ECCv32_IMAP','IMAP'),('ECCv32_QMAP','QMAP')):
    for i,e in enumerate(['SMCv32_%s16'%k,'SMCv32_%s16'%k,'VTBv32_%s32'%k,'VTBv32_%s16T'%k,'VTBv32_%s64'%k,'VTBv32_%s128'%k]):
        print('chk("%s", %d, cfgp_word(%s[%d]), %s, "%s");'%(t,i,t,i,e,e))
print('printf("bad=%d\\n", bad); return bad != 0; }')
PY
rc=0
for M in -m32 -m64; do
  gcc $M -w -Isrc -Isrc/core -Isrc/fpm -Isrc/v32 -Isrc/fsk -no-pie -o $T$M $T.c $FILES || exit 2
  ${T_RUN}$T$M > $T$M.out; r=$?; [ $r -eq 0 ] || { echo "$M: BAD entries (see $T$M.out)"; rc=1; }
done
if cmp -s $T-m32.out $T-m64.out; then echo "cfgptr_check: IDENTICAL -m32 / -m64 ($(wc -l < $T-m32.out) lines)"; else echo "cfgptr_check: DIFFERENT"; diff $T-m32.out $T-m64.out | head; rc=1; fi
rm -f $T-m32 $T-m64 $T.c
exit $rc
