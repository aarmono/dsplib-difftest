#!/bin/sh
# P05 golden check WITHOUT the blob: builds only the x86-64 port (g++ -m64, -fno-exceptions -fno-rtti; the P03 port object supplies LowPassFIR / mean / Var) and the
# harness (gcc -m64), runs the scenarios and diffs the concatenated output against the goldens produced by the 32-bit run against the real dsplibs.o
# (dsp/dsp_resampler.sh, GOLDEN_UPDATE=1):
#   test/golden/dsp/dsp_resampler.blob32.out      scenarios 0-6   (must reproduce with the default x86-64 SSE flags)
#   test/golden/dsp/dsp_resampler.blob32.x87.out  scenarios 7, 8  (7: the EXTENDED return values, readable only from st(0) on i386: never reproducible by an x86-64 build;
#                                   8: special values / NaN payloads: reproduced only with FP64_FLAGS=-mfpmath=387; with SSE flags it is run and reported)
# FP64_FLAGS="-O0", "-O1", "-O3", "-Os", "-mfpmath=387" ... select the build flags (default -O2).  No dsplibs.o, no 32-bit toolchain needed.
# Usage: sh test/dsp/dsp_resampler.golden64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
T="${TPREFIX:-test/.build/dsp/dsp_resampler.g64}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
OPT="-O2"
case "$FP64_FLAGS" in *-O*) OPT="";; esac
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -Isrc $OPT $FP64_FLAGS"
g++ -m64 $CXXF -c src/dsp/dsp_resampler.cpp -o ${T}_port.o
g++ -m64 $CXXF -c src/dsp/dsp_window_fft_psd.cpp -o ${T}_p03.o
g++ -m64 $CXXF -c test/p05_layout_probe.cpp -o ${T}_probe.o
gcc -m64 -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf -c test/stub_env.c -o ${T}_stub.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness.o
gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-misleading-indentation -O2 -c test/dsp/dsp_resampler.main.c -o ${T}_main.o
gcc -m64 -O2 -no-pie -o ${T}_test ${T}_main.o ${T}_harness.o ${T}_stub.o ${T}_probe.o ${T}_port.o ${T}_p03.o
: > ${T}.out; : > ${T}_x87.out
for i in 0 1 2 3 4 5 6; do ${T_RUN}${T}_test $i >> ${T}.out 2>/dev/null; done
${T_RUN}${T}_test 8 > ${T}_s8.out 2>/dev/null || true
rc=0
if diff -u test/golden/dsp/dsp_resampler.blob32.out ${T}.out > ${T}.diff; then echo "GOLDEN64: the x86-64 port ($FP64_FLAGS $OPT, SSE unless -mfpmath=387) reproduces the 32-bit blob's canonical dump for scenarios 0-6 (test/golden/dsp/dsp_resampler.blob32.out)"
else echo "GOLDEN64 FAIL: scenarios 0-6 differ ($(grep -c '^[<>+-][^+-]' ${T}.diff) lines, see ${T}.diff)"; rc=1; fi
# scenario 8 is the last scenario of the x87 golden file (scenario 7 comes first)
python3 - <<'PY' > ${T}_x87_ref.out
t=open('test/golden/dsp/dsp_resampler.blob32.x87.out').read().split('\n')
out=[];ns=0
for l in t:
    if ns>=1: out.append(l)
    if l.startswith('HASH'): ns+=1
print('\n'.join(out),end='')
PY
if diff -q ${T}_x87_ref.out ${T}_s8.out >/dev/null; then echo "GOLDEN64: scenario 8 (special values, NaN payloads) reproduces test/golden/dsp/dsp_resampler.blob32.x87.out too"
else case "$FP64_FLAGS" in *387*) echo "GOLDEN64 FAIL: scenario 8 differs with -mfpmath=387"; rc=1;; *) echo "GOLDEN64: scenario 8 (NaN payload selection) differs, as expected for SSE float math (see NOTES)";; esac; fi
rm -f ${T}_port.o ${T}_p03.o ${T}_probe.o ${T}_stub.o ${T}_harness.o ${T}_main.o ${T}_test ${T}.out ${T}_x87.out ${T}_x87_ref.out ${T}_s8.out
[ $rc = 0 ] && rm -f ${T}.diff
exit $rc
