#!/bin/sh
# P04 golden check WITHOUT the blob: builds only the x86-64 port (g++ -m64, -fno-exceptions -fno-rtti; P04 + the P02 port object that supplies
# the GenericIIR) and the harness (gcc -m64), runs the 7 scenarios and diffs the concatenated output against the golden produced by the 32-bit
# run against the real dsplibs.o (dsp/dsp_tone_detectors.sh, GOLDEN_UPDATE=1): test/golden/dsp/dsp_tone_detectors.blob32.out.
# Default: the SSE build (x86-64 default float math, long double = x87 where the blob keeps extended values); FP64_FLAGS=-mfpmath=387 builds the
# x87 variant.  No dsplibs.o, no 32-bit toolchain needed.
# Usage: sh test/dsp/dsp_tone_detectors.golden64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
T="${TPREFIX:-test/.build/dsp/dsp_tone_detectors.g64}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -Isrc -O2 $FP64_FLAGS"
g++ -m64 $CXXF -c src/dsp/dsp_tone_detectors.cpp -o ${T}_port.o
g++ -m64 $CXXF -c src/dsp/dsp_float_filters.cpp -o ${T}_p02.o
g++ -m64 $CXXF -c test/dsp/dsp_tone_detectors_layout_probe.cpp -o ${T}_probe.o
g++ -m64 $CXXF -c test/p02_layout_probe.cpp -o ${T}_probe02.o
gcc -m64 -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -c test/stub_env.c -o ${T}_stub.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -no-pie -o ${T}_test test/dsp/dsp_tone_detectors.main.c ${T}_harness.o ${T}_stub.o ${T}_probe.o ${T}_probe02.o ${T}_port.o ${T}_p02.o
: > ${T}.out
for i in 0 1 2 3 4 5 6; do ${T_RUN}${T}_test $i >> ${T}.out; done
rc=0
diff -u test/golden/dsp/dsp_tone_detectors.blob32.out ${T}.out && echo "GOLDEN64: the x86-64 port ($FP64_FLAGS, SSE unless -mfpmath=387) reproduces the 32-bit blob's canonical dump for scenarios 0-6 (test/golden/dsp/dsp_tone_detectors.blob32.out)" || rc=1
rm -f ${T}_port.o ${T}_p02.o ${T}_probe.o ${T}_probe02.o ${T}_stub.o ${T}_harness.o ${T}_test ${T}.out
exit $rc
