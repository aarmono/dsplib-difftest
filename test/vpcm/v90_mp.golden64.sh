#!/bin/sh
# P09 golden check WITHOUT the blob: builds only the x86-64 port (g++ -m64, -fno-exceptions -fno-rtti) and the harness (gcc -m64), runs the scenarios and diffs the
# concatenated output against the golden produced by the 32-bit run against the real dsplibs.o (v90_mp.sh, GOLDEN_UPDATE=1): test/golden/vpcm/v90_mp.blob32.out.
# FP64_FLAGS="-O0", "-O1", "-O3", "-Os" ... select the build flags (default -O2).  No dsplibs.o, no 32-bit toolchain needed.  (V90MP is integer code only: no float flags.)
# Usage: sh test/vpcm/v90_mp.golden64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
T="${TPREFIX:-test/.build/vpcm/v90_mp.g64}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
OPT="-O2"
case "$FP64_FLAGS" in *-O*) OPT="";; esac
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -Isrc $OPT $FP64_FLAGS"
g++ -m64 $CXXF -c src/vpcm/v90_mp.cpp -o ${T}_port.o
g++ -m64 $CXXF -c test/vpcm/v90_mp_layout_probe.cpp -o ${T}_probe.o
gcc -m64 -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf -c test/stub_env.c -o ${T}_stub.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness.o
gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-misleading-indentation -O2 -c test/vpcm/v90_mp.main.c -o ${T}_main.o
gcc -m64 -O2 -no-pie -o ${T}_test ${T}_main.o ${T}_harness.o ${T}_stub.o ${T}_probe.o ${T}_port.o
: > ${T}.out
for i in 0 1 2 3 4 5; do ${T_RUN}${T}_test $i >> ${T}.out 2>/dev/null; done
rc=0
if diff -u test/golden/vpcm/v90_mp.blob32.out ${T}.out > ${T}.diff; then echo "GOLDEN64: the x86-64 port ($FP64_FLAGS $OPT) reproduces the 32-bit blob's canonical dump for scenarios 0-5 (test/golden/vpcm/v90_mp.blob32.out)"
else echo "GOLDEN64 FAIL: the dump differs ($(grep -c '^[<>+-][^+-]' ${T}.diff) lines, see ${T}.diff)"; rc=1; fi
rm -f ${T}_port.o ${T}_probe.o ${T}_stub.o ${T}_harness.o ${T}_main.o ${T}_test ${T}.out
[ $rc = 0 ] && rm -f ${T}.diff
exit $rc
