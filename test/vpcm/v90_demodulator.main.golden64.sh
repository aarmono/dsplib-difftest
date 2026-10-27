#!/bin/sh
# P44 golden check WITHOUT the blob: builds only the x86-64 port (g++ -m64, -fno-exceptions -fno-rtti: src/vpcm/v90_demodulator.cpp; every callee is a harness recorder) and the harness (gcc -m64), runs the scenarios and
# diffs the concatenated output against the golden produced by the 32-bit run against the real dsplibs.o (v90_demodulator.main.sh, GOLDEN_UPDATE=1): test/golden/vpcm/v90_demodulator.main.blob32.out.
# FP64_FLAGS="-O0", "-O1", "-O3", "-Os" ... select the build flags (default -O2); FP64_FLAGS=-mfpmath=387 selects the x87 mode (the long double print arithmetic is x87 either way: the result must not change).  No dsplibs.o, no 32-bit toolchain needed.
# Usage: sh test/vpcm/v90_demodulator.main.golden64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p07) echo vpcm/pcm_parameters;; p09) echo vpcm/v90_mp;; p11) echo vpcm/v92_cp;; p17) echo vpcm/v90_connection_evaluator;; p18) echo vpcm/v90_adid;; p24) echo vpcm/v90_constellation_noise;; p26) echo vpcm/v90_trn2_designer;; p28) echo vpcm/v90_mapper;; p34) echo vpcm/v90_phase3_demod_v92_decision;; p37) echo vpcm/v90_phase4_demod_decisions;; p40) echo vpcm/v90_equalizer_process;; p43) echo vpcm/v90_demodulator_progress;; p44) echo vpcm/v90_demodulator;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
T="${TPREFIX:-test/.build/vpcm/v90_demodulator.main.g64}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
OPT="-O2"
case "$FP64_FLAGS" in *-O*) OPT="";; esac
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc $OPT $FP64_FLAGS"
g++ -m64 $CXXF -c src/vpcm/v90_demodulator.cpp -o ${T}_p44.o
PROBES=""
for c in p07 p09 p11 p17 p18 p24 p26 p28 p34 p37 p40 p43 p44; do g++ -m64 $CXXF -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
gcc -m64 -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf -c test/stub_env.c -o ${T}_stub.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness.o
gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -c test/vpcm/v90_demodulator.main.main.c -o ${T}_main.o
gcc -m64 -O2 -no-pie -o ${T}_test ${T}_main.o ${T}_harness.o ${T}_stub.o $PROBES ${T}_p44.o
: > ${T}.out
for i in 0 1 2 3 4; do ${T_RUN}${T}_test $i >> ${T}.out 2>/dev/null; done
rc=0
if diff -u test/golden/vpcm/v90_demodulator.main.blob32.out ${T}.out > ${T}.diff; then echo "GOLDEN64: the x86-64 port ($FP64_FLAGS $OPT) reproduces the 32-bit blob's canonical dump for scenarios 0-4 (test/golden/vpcm/v90_demodulator.main.blob32.out)"
else echo "GOLDEN64: FAIL: the dump differs ($(grep -c '^[<>+-][^+-]' ${T}.diff) lines, see ${T}.diff)"; rc=1; fi
rm -f ${T}_*.o ${T}_test ${T}.out
[ $rc = 0 ] && rm -f ${T}.diff
exit $rc
