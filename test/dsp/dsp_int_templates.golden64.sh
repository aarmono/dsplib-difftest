#!/bin/sh
# P01 golden check WITHOUT the blob: builds only the x86-64 port (g++ -m64, -fno-exceptions -fno-rtti) and the harness
# (gcc -m64), runs every scenario and diffs the concatenated output against test/golden/dsp/dsp_int_templates.blob32.out, which was produced by the
# 32-bit run against the real dsplibs.o (dsp/dsp_int_templates.sh, GOLDEN_UPDATE=1).  No dsplibs.o, no 32-bit toolchain needed.
# Usage: sh test/dsp/dsp_int_templates.golden64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
T="${TPREFIX:-test/.build/dsp/dsp_int_templates.g64}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -Isrc -O2"
g++ -m64 $CXXF -c src/dsp/dsp_int_templates.cpp -o ${T}_port.o
g++ -m64 $CXXF -c test/p01_layout_probe.cpp -o ${T}_probe.o
gcc -m64 -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -c test/stub_env.c -o ${T}_stub.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -no-pie -o ${T}_test test/dsp/dsp_int_templates.main.c ${T}_harness.o ${T}_stub.o ${T}_probe.o ${T}_port.o
: > ${T}.out
for i in 0 1 2 3 4 5 6 7 8 9 10 11; do ${T_RUN}${T}_test $i >> ${T}.out; done
diff -u test/golden/dsp/dsp_int_templates.blob32.out ${T}.out && echo "GOLDEN64: the x86-64 port reproduces the 32-bit blob's canonical dump (test/golden/dsp/dsp_int_templates.blob32.out)"
rm -f ${T}_port.o ${T}_probe.o ${T}_stub.o ${T}_harness.o ${T}_test ${T}.out
