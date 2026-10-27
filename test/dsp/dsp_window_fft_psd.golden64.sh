#!/bin/sh
# P03 golden check WITHOUT the blob: builds only the x86-64 port (g++ -m64, -fno-exceptions -fno-rtti) and the harness (gcc -m64), runs the scenarios and
# diffs the concatenated output against the goldens produced by the 32-bit run against the real dsplibs.o (dsp/dsp_window_fft_psd.sh, GOLDEN_UPDATE=1):
#   test/golden/dsp/dsp_window_fft_psd.blob32.out      scenarios 0-2, 4-10   (windows, sinc, statistics rounded to float, four1 / realfft / Psd / LowPassFIR / Agc / SineWave, the sites)
#   test/golden/dsp/dsp_window_fft_psd.blob32.x87.out  scenarios 3, 11, 12, 13, 14  (3: the EXTENDED return values of sum / mean / sqrSum / Var / Std, readable only from st(0) on i386 and
#                                   never reproducible by an x86-64 build (the float result is returned in xmm0); 11 / 12 / 13 / 14: four1 / realfft / Psd / Agc on data with NaNs,
#                                   x87-exact only: which NaN wins an addition depends on the machine's NaN rules)
# Default: the SSE build (x86-64 default float math): scenarios 0-2, 4-10 must reproduce the golden; 11-14 are run and reported (EXPECTED to differ).
# FP64_FLAGS=-mfpmath=387: scenarios 11-14 must reproduce too.  Transcendentals are the x87 instructions in both builds (src/core/dsp_math.h):
# DSP_MATH_PORTABLE=1 (extra flag -DDSP_MATH_PORTABLE) builds the libm fallback instead and reports how many scenarios still reproduce.
# No dsplibs.o, no 32-bit toolchain needed.
# Usage: sh test/dsp/dsp_window_fft_psd.golden64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
T="${TPREFIX:-test/.build/dsp/dsp_window_fft_psd.g64}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -Isrc -O2 $FP64_FLAGS"
g++ -m64 $CXXF -c src/dsp/dsp_window_fft_psd.cpp -o ${T}_port.o
g++ -m64 $CXXF -c test/p03_layout_probe.cpp -o ${T}_probe.o
gcc -m64 -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -c test/stub_env.c -o ${T}_stub.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness.o
gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-misleading-indentation -O2 -c test/dsp/dsp_window_fft_psd.main.c -o ${T}_main.o
gcc -m64 -O2 -no-pie -o ${T}_test ${T}_main.o ${T}_harness.o ${T}_stub.o ${T}_probe.o ${T}_port.o -lm
: > ${T}.out; : > ${T}_x87.out
for i in 0 1 2 4 5 6 7 8 9 10; do ${T_RUN}${T}_test $i >> ${T}.out 2>/dev/null; done
for i in 11 12 13 14; do ${T_RUN}${T}_test $i >> ${T}_x87.out 2>/dev/null; done
rc=0
if diff -u test/golden/dsp/dsp_window_fft_psd.blob32.out ${T}.out > ${T}.diff; then echo "GOLDEN64: the x86-64 port ($FP64_FLAGS, SSE unless -mfpmath=387) reproduces the 32-bit blob's canonical dump for scenarios 0-2, 4-10 (test/golden/dsp/dsp_window_fft_psd.blob32.out)"
else echo "GOLDEN64 FAIL: scenarios 0-2, 4-10 differ ($(grep -c '^[<>+-][^+-]' ${T}.diff) lines, see ${T}.diff)"; rc=1; fi
# scenarios 11-14 are the last four scenarios of the x87 golden file (scenario 3 comes first)
python3 - "$T" <<'PY' > ${T}_x87_ref.out
import sys
t=open('test/golden/dsp/dsp_window_fft_psd.blob32.x87.out').read().split('\n')
# the golden file is: scenario 3 output, then 11, 12, 13, 14; every scenario ends with a "HASH" line
out=[];ns=0
for l in t:
    if ns>=1: out.append(l)
    if l.startswith('HASH'): ns+=1
print('\n'.join(out),end='')
PY
if diff -q ${T}_x87_ref.out ${T}_x87.out >/dev/null; then echo "GOLDEN64: scenarios 11-14 (NaN payloads) reproduce test/golden/dsp/dsp_window_fft_psd.blob32.x87.out too"
else case "$FP64_FLAGS" in *387*) echo "GOLDEN64 FAIL: scenarios 11-14 differ with -mfpmath=387"; rc=1;; *) echo "GOLDEN64: scenarios 11-14 (NaN payload selection) differ, as expected for SSE float math (see NOTES)";; esac; fi
rm -f ${T}_port.o ${T}_probe.o ${T}_stub.o ${T}_harness.o ${T}_main.o ${T}_test ${T}.out ${T}_x87.out ${T}_x87_ref.out
[ $rc = 0 ] && rm -f ${T}.diff
exit $rc
