#!/bin/sh
# N04: FSE_decision_16Tpt/32pt/64pt/128pt called directly over the full input range; every output (ret, angle, mag, lock state, VTB state) of the
# 32-bit blob compared with the port built at -O0 and -O2 (the pinned set is covered by tools/run_changed.sh stage 1p through test/flagshim).
# Usage: sh test/v32/fse_decision.sh   (from anywhere).  Exit 0 + "MATCH" = identical.
set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T=${TPREFIX:-test/.build/v32/fse_decision}
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
SRCS="src/v32/v32_engine.c src/v32/v32_datapath.c src/v32/v32_scrambler.c src/v32/v32_pulse_shaping.c src/v32/v32_timing.c src/v32/v32_echo_canceller.c src/v32/v32_equalizer.c src/v32/v32_math.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c src/fpm/fpm_mrf.c src/fpm/fpm_agc.c src/fpm/fpm_math.c src/fpm/fpm_tables.c src/fpm/fpm_tone.c src/fpm/fpm_mtd.c src/fpm/fpm_iir.c"
SCENARIOS="0 1 2 3"
gcc -m32 -Isrc -w -O0 -g -o ${T}_test_blob test/v32/fse_decision.main.c test/stub_env.c $BLOB_O
: > ${T}_blob.out
for s in $SCENARIOS; do ${T_RUN}${T}_test_blob $s >> ${T}_blob.out 2>&1 || echo "blob scenario $s exit $?" >> ${T}_blob.out; done
rc=0
for OPT in -O0 -O2; do
  gcc -m32 -w $OPT -g -Isrc -o ${T}_test_port test/v32/fse_decision.main.c test/stub_env.c $SRCS
  : > ${T}_port.out
  for s in $SCENARIOS; do ${T_RUN}${T}_test_port $s >> ${T}_port.out 2>&1 || echo "blob scenario $s exit $?" >> ${T}_port.out; done
  if diff -q ${T}_blob.out ${T}_port.out >/dev/null; then echo "MATCH ($OPT): $(wc -l < ${T}_blob.out) lines"; else
    echo "MISMATCH ($OPT): $(diff ${T}_blob.out ${T}_port.out | grep -c '^<') lines differ"; diff ${T}_blob.out ${T}_port.out | head -6 | cut -c1-300; rc=1; fi
done
exit $rc
