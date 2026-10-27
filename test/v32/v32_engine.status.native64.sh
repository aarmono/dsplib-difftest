#!/bin/sh
# X5: V32FP_status() on a REAL x86-64 engine (test/v32/v32_engine.status.native64.main.c).  No blob involved.
#   x86-64 : the V.32 files + the FPM files of the engine (the n01_v32status set; REAL C, native) + the test, run natively: the AGC detect state is set
#            through the member of the typed AGC object and the status word must follow it (bugs.md V80)
#   i386   : the test only compiled (-m32 -c): its static asserts pin the i386 offsets; the blob side of the status word is covered by v32 / n01_v32status
# Env: PORT_FLAGS (extra flags of the x86-64 port build, e.g. -O0), TPREFIX (output prefix), X5_ENGINE (alternative source of v32_engine.c, used to show
# that the test fails on old code).  Usage: sh test/v32/v32_engine.status.native64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
T="${TPREFIX:-test/.build/v32/v32_engine.status.native64}"
mkdir -p "$(dirname "$T")"
ENG="${X5_ENGINE:-src/v32/v32_engine.c}"
SRCS="src/core/dsp_table_handles.c src/v32/v32_datapath.c src/v32/v32_scrambler.c src/v32/v32_pulse_shaping.c src/v32/v32_timing.c src/v32/v32_echo_canceller.c src/v32/v32_equalizer.c src/v32/v32_math.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c src/fpm/fpm_mrf.c src/fpm/fpm_agc.c src/fpm/fpm_math.c src/fpm/fpm_tables.c src/fpm/fpm_tone.c src/fpm/fpm_mtd.c src/fpm/fpm_iir.c"
gcc -m32 -w -O2 -Isrc -c test/v32/v32_engine.status.native64.main.c -o ${T}_main32.o
gcc -m64 -w -O2 -no-pie -fno-stack-protector -Isrc $PORT_FLAGS -o ${T}_test test/v32/v32_engine.status.native64.main.c test/stub_env.c $ENG $SRCS
case "$T" in /*) R="";; *) R="./";; esac
rc=0
${R}${T}_test > ${T}.out 2>&1 || rc=$?
cat ${T}.out
rm -f ${T}_main32.o ${T}_test
[ $rc = 0 ] && rm -f ${T}.out
exit $rc
