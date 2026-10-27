#!/bin/sh
# P06 golden check WITHOUT the blob: builds only the x86-64 port (g++ -m64, -fno-exceptions -fno-rtti) and the harness (gcc -m64), runs the scenarios and diffs the
# concatenated output against the golden produced by the 32-bit run against the real dsplibs.o (pcm_leaf_classes.sh, GOLDEN_UPDATE=1):
#   test/golden/vpcm/pcm_leaf_classes.blob32.out      scenarios 0-9, 11, 12   (must reproduce with the default x86-64 SSE flags AND with -mfpmath=387)
#   test/golden/vpcm/pcm_leaf_classes.blob32.x87.out  scenario 10: the EXTENDED getMetric results as 80-bit patterns, readable only from st(0) on i386: never reproducible by an x86-64 build
# FP64_FLAGS="-O0", "-O1", "-O3", "-Os", "-mfpmath=387" ... select the build flags (default -O2).  No dsplibs.o, no 32-bit toolchain needed.
# Usage: sh test/vpcm/pcm_leaf_classes.golden64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
T="${TPREFIX:-test/.build/vpcm/pcm_leaf_classes.g64}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
OPT="-O2"
case "$FP64_FLAGS" in *-O*) OPT="";; esac
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -Isrc $OPT $FP64_FLAGS"
g++ -m64 $CXXF -c src/vpcm/pcm_leaf_classes.cpp -o ${T}_port.o
g++ -m64 $CXXF -c test/vpcm/pcm_leaf_classes_layout_probe.cpp -o ${T}_probe.o
gcc -m64 -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf -c test/stub_env.c -o ${T}_stub.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness.o
gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-misleading-indentation -O2 -c test/vpcm/pcm_leaf_classes.main.c -o ${T}_main.o
gcc -m64 -O2 -no-pie -o ${T}_test ${T}_main.o ${T}_harness.o ${T}_stub.o ${T}_probe.o ${T}_port.o
: > ${T}.out
for i in 0 1 2 3 4 5 6 7 8 9 11 12; do ${T_RUN}${T}_test $i >> ${T}.out 2>/dev/null; done
rc=0
if diff -u test/golden/vpcm/pcm_leaf_classes.blob32.out ${T}.out > ${T}.diff; then echo "GOLDEN64: the x86-64 port ($FP64_FLAGS $OPT, SSE unless -mfpmath=387) reproduces the 32-bit blob's canonical dump for scenarios 0-9, 11, 12 (test/golden/vpcm/pcm_leaf_classes.blob32.out)"
else echo "GOLDEN64 FAIL: the dump differs ($(grep -c '^[<>+-][^+-]' ${T}.diff) lines, see ${T}.diff)"; rc=1; fi
rm -f ${T}_port.o ${T}_probe.o ${T}_stub.o ${T}_harness.o ${T}_main.o ${T}_test ${T}.out
[ $rc = 0 ] && rm -f ${T}.diff
exit $rc
