#!/bin/sh
# N01: noisy-state V32FP_status differential test (test/v32/v32_engine.status.main.c), blob vs port (-O0 and -O2 for the port), rate classes 0..6.
set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/v32/v32_engine.status}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
OPT="${1:--O0}"
SRCS="src/v32/v32_engine.c src/v32/v32_datapath.c src/v32/v32_scrambler.c src/v32/v32_pulse_shaping.c src/v32/v32_timing.c src/v32/v32_echo_canceller.c src/v32/v32_equalizer.c src/v32/v32_math.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c src/fpm/fpm_mrf.c src/fpm/fpm_agc.c src/fpm/fpm_math.c src/fpm/fpm_tables.c src/fpm/fpm_tone.c src/fpm/fpm_mtd.c src/fpm/fpm_iir.c"
gcc -m32 -w -O0 -o ${T}_blob test/v32/v32_engine.status.main.c test/stub_env.c $BLOB_O
gcc -m32 -w $OPT -Isrc -o ${T}_port test/v32/v32_engine.status.main.c test/stub_env.c $SRCS
: > ${T}_blob.out; : > ${T}_port.out
for r in 0 1 2 3 4 5 6; do ${T_RUN}${T}_blob $r >> ${T}_blob.out 2>&1 || echo "blob rc $r exit $?" >> ${T}_blob.out; ${T_RUN}${T}_port $r >> ${T}_port.out 2>&1 || echo "blob rc $r exit $?" >> ${T}_port.out; done
wc -l ${T}_blob.out
if diff -q ${T}_blob.out ${T}_port.out >/dev/null; then echo "MATCH ($OPT)"; else echo "MISMATCH ($OPT): $(diff ${T}_blob.out ${T}_port.out | grep -c '^<') lines differ"; diff ${T}_blob.out ${T}_port.out | head -6 | cut -c1-400; exit 1; fi
