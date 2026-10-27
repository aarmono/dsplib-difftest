#!/bin/sh
# P02 golden check WITHOUT the blob: builds only the x86-64 port (g++ -m64, -fno-exceptions -fno-rtti) and the harness (gcc -m64), runs the
# scenarios and diffs the concatenated output against the goldens produced by the 32-bit run against the real dsplibs.o
# (dsp/dsp_float_filters.sh, GOLDEN_UPDATE=1):
#   test/golden/dsp/dsp_float_filters.blob32.out      scenarios 0-6, 8-11 (finite data, inf, denormals, signed zeros, one-NaN-source cases)
#   test/golden/dsp/dsp_float_filters.blob32.nan.out  scenario 7      (NaN payloads incl. signalling NaNs: exact only for x87 code)
# Default: the SSE build (x86-64 default float math): scenarios 0-6, 8-11 must reproduce the golden; scenario 7 is run and reported but is
# EXPECTED to differ (SSE picks another NaN when two different NaNs meet).  FP64_FLAGS=-mfpmath=387: all eight scenarios must reproduce.
# No dsplibs.o, no 32-bit toolchain needed.
# Usage: sh test/dsp/dsp_float_filters.golden64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
T="${TPREFIX:-test/.build/dsp/dsp_float_filters.g64}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -Isrc -O2 $FP64_FLAGS"
g++ -m64 $CXXF -c src/dsp/dsp_float_filters.cpp -o ${T}_port.o
g++ -m64 $CXXF -c test/p02_layout_probe.cpp -o ${T}_probe.o
gcc -m64 -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -c test/stub_env.c -o ${T}_stub.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -no-pie -o ${T}_test test/dsp/dsp_float_filters.main.c ${T}_harness.o ${T}_stub.o ${T}_probe.o ${T}_port.o
: > ${T}.out
for i in 0 1 2 3 4 5 6 8 9 10 11; do ${T_RUN}${T}_test $i >> ${T}.out; done
${T_RUN}${T}_test 7 > ${T}_nan.out
rc=0
diff -u test/golden/dsp/dsp_float_filters.blob32.out ${T}.out && echo "GOLDEN64: the x86-64 port ($FP64_FLAGS, SSE unless -mfpmath=387) reproduces the 32-bit blob's canonical dump for scenarios 0-6, 8-11 (test/golden/dsp/dsp_float_filters.blob32.out)" || rc=1
if diff -q test/golden/dsp/dsp_float_filters.blob32.nan.out ${T}_nan.out >/dev/null; then
  echo "GOLDEN64: scenario 7 (NaN payloads) reproduces test/golden/dsp/dsp_float_filters.blob32.nan.out too"
else
  case "$FP64_FLAGS" in *387*) echo "GOLDEN64 FAIL: scenario 7 differs with -mfpmath=387"; rc=1;; *) echo "GOLDEN64: scenario 7 (NaN payload selection) differs, as expected for SSE float math (see NOTES)";; esac
fi
rm -f ${T}_port.o ${T}_probe.o ${T}_stub.o ${T}_harness.o ${T}_test ${T}.out ${T}_nan.out
exit $rc
