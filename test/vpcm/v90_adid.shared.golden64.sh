#!/bin/sh
# Golden check WITHOUT the blob for a row of class V90AutoDigitalImpDetector (ROW = p20 / p19 / p18; use test/vpcm/v90_adid_study.golden64.sh, p19, p18): builds only the x86-64 port files of the row (g++ -m64,
# -fno-exceptions -fno-rtti), P06 (G.711) and the harness (gcc -m64), runs the scenarios of the row and diffs the concatenated output against the golden produced by the 32-bit run against the
# real dsplibs.o (build_and_diff_<row>.sh, GOLDEN_UPDATE=1): test/golden/<row>_blob32.out.  FP64_FLAGS="-O0", "-O1", "-O3", "-Os" ... select the build flags (default -O2); FP64_FLAGS=-mfpmath=387
# selects the x87 mode (default is SSE with long double).  No dsplibs.o, no 32-bit toolchain needed.
# Usage: sh test/vpcm/v90_adid_study.golden64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p07) echo vpcm/pcm_parameters;; p18) echo vpcm/v90_adid;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
ROW="${ROW:?}"
T="${TPREFIX:-test/.build/vpcm/v90_adid.$ROW.g64}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
case "$ROW" in p18) GS=v90_adid;; p19) GS=v90_adid_pad_gain;; p20) GS=v90_adid_study;; esac   # stem of the row's job (its golden test/golden/vpcm/$GS.blob32.out)
OPT="-O2"
case "$FP64_FLAGS" in *-O*) OPT="";; esac
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -Isrc $OPT $FP64_FLAGS"
case "$ROW" in
  p20) FILES="src/vpcm/v90_adid.cpp"; SC="0 1 2 3" ;;
  p19) FILES="src/vpcm/v90_adid.cpp src/vpcm/v90_adid_pad_gain.cpp"; SC="0 1 2 3 4 5 6 7 8" ;;
  p18) FILES="src/vpcm/v90_adid.cpp src/vpcm/v90_adid_pad_gain.cpp src/vpcm/v90_adid_study.cpp"; SC="0 1 2 3 4 5 6 7 8 9 10 11 12 13" ;;
esac
OBJS=""
n=0
for f in src/vpcm/pcm_leaf_classes.cpp $FILES; do
  n=$((n + 1))
  g++ -m64 $CXXF -fno-stack-protector -c $f -o ${T}_port$n.o
  OBJS="$OBJS ${T}_port$n.o"
done
for c in p07 p18; do g++ -m64 $CXXF -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; done
gcc -m64 -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf -c test/stub_env.c -o ${T}_stub.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness.o
gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -c test/vpcm/v90_adid.main.c -o ${T}_main.o
gcc -m64 -O2 -no-pie -Wl,--unresolved-symbols=ignore-all -o ${T}_test ${T}_main.o ${T}_harness.o ${T}_stub.o ${T}_probe_p07.o ${T}_probe_p18.o $OBJS
: > ${T}.out
for i in $SC; do ${T_RUN}${T}_test $i >> ${T}.out 2>/dev/null; done
rc=0
if diff -u test/golden/vpcm/${GS}.blob32.out ${T}.out > ${T}.diff; then echo "GOLDEN64: the x86-64 port ($FP64_FLAGS $OPT) reproduces the 32-bit blob's canonical dump for scenarios $SC (test/golden/vpcm/${GS}.blob32.out)"
else echo "GOLDEN64 FAIL: the dump differs ($(grep -c '^[<>+-][^+-]' ${T}.diff) lines, see ${T}.diff)"; rc=1; fi
rm -f ${T}_port*.o ${T}_probe_*.o ${T}_stub.o ${T}_harness.o ${T}_main.o ${T}_test ${T}.out
[ $rc = 0 ] && rm -f ${T}.diff
exit $rc
