#!/bin/sh
# X1: how much does the optimiser's alias / lifetime analysis matter in the port?  Compiles every library .c / .cpp (sources.mk) at -m32 -O2 plain, with -fno-strict-aliasing and
# with -fno-lifetime-dse and compares the generated code (objdump, addresses stripped).  Prints the number of objects whose code differs per flag and the list
# (test/golden/cross/x1_flagdiff.txt holds the accepted result: a NEW object appearing in the list is a file whose source should be audited; the check fails if
# -fno-lifetime-dse changes any function body, which is the claim made in test/flags.sh).  Usage: sh test/cross/x1_flagdiff.sh [--update]
set -e
cd "$(dirname "$0")/../.."
D=${TMPDIR:-/tmp}/x1_flagdiff.$$; mkdir -p $D; trap 'rm -rf $D' EXIT
build() { # tag extra
  mkdir -p $D/$1
  # every library source of sources.mk (kind c | cpp | v04 = with V04_DEFS), each object named by the file's stem (RS4: unique, src/<module>/<stem>),
  # so the list in test/golden/cross/x1_flagdiff.txt does not depend on where the files live
  python3 tools/port_objs.py list | while read f k; do b=$(basename ${f%.*})
    case $k in
      c) gcc -m32 -O2 -w -fno-stack-protector -fno-pie -Isrc $2 -c $f -o $D/$1/$b.o;;
      v04) g++ -m32 -O2 -w -fno-exceptions -fno-rtti -fno-stack-protector -fno-pie -Isrc -DV04_REAL_HELPERS -Dgetbit=v04_getbit_impl -DApplyBulkDelay=v04_apply_bulk_delay_impl $2 -c $f -o $D/$1/$b.o;;
      *) g++ -m32 -O2 -w -fno-exceptions -fno-rtti -fno-stack-protector -fno-pie -Isrc $2 -c $f -o $D/$1/$b.o;;
    esac
    objdump -dr -j .text $D/$1/$b.o | sed '1,2d; s/^ *[0-9a-f]*:\t//' > $D/$1/$b.d; done
}
build plain ""; build nsa -fno-strict-aliasing; build nld -fno-lifetime-dse
cat > $D/norm.py <<'P'
import sys,re
def funcs(p):
    d={};cur=None
    for l in open(p):
        m=re.match(r'^[0-9a-f]+ <(.*)>:$',l)
        if m: cur=m.group(1); d[cur]=[]; continue
        if cur is None: continue
        l=re.sub(r'\b[0-9a-f]+ <([^>+]*)(\+0x[0-9a-f]+)?>',r'<\1\2>',l)
        l=re.sub(r'^\s+[0-9a-f]+: R_386',' R_386',l)
        if re.match(r'^[0-9a-f ]{2,}\s+(jmp|j[a-z]+|call)\s',l): l=re.sub(r'^[0-9a-f ]+\t','',l)   # branch encodings depend on the placement
        d[cur].append(l.rstrip())
    return d
a=funcs(sys.argv[1]); b=funcs(sys.argv[2])
print(sum(1 for k in a if k in b and a[k]!=b[k]))
P
nsa=; nld_bodies=0
for d in $D/plain/*.d; do b=$(basename $d .d)
  cmp -s $d $D/nsa/$b.d || nsa="$nsa $b"
  n=$(python3 $D/norm.py $d $D/nld/$b.d); nld_bodies=$((nld_bodies + n))
done
echo "objects whose -m32 -O2 code differs with -fno-strict-aliasing: $(echo $nsa | wc -w) of $(ls $D/plain/*.d | wc -l)"
echo "function bodies that differ with -fno-lifetime-dse (placement / D1-D2 aliasing of empty destructors ignored): $nld_bodies"
echo $nsa | tr ' ' '\n' | LC_ALL=C sort > $D/nsa.txt   # RS4: byte order (the list does not depend on the caller's locale)
if [ "$1" = "--update" ]; then cp $D/nsa.txt test/golden/cross/x1_flagdiff.txt; echo updated; exit 0; fi
if diff -u test/golden/cross/x1_flagdiff.txt $D/nsa.txt; then echo "MATCH: same list as test/golden/cross/x1_flagdiff.txt (-fno-strict-aliasing)"; else echo "MISMATCH: strict-aliasing sensitive object list changed"; exit 1; fi
[ "$nld_bodies" = 0 ] || { echo "MISMATCH: -fno-lifetime-dse changes code"; exit 1; }
