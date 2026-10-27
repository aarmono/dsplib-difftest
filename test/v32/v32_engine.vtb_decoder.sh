#!/bin/sh
# Builds both sides of the V.32 VTB_decoder (per-symbol Viterbi trellis
# decode step) differential test - one linked against the real dsplibs.o,
# one against the hand-cleaned src/v32/v32_datapath.c (VTB_decoder +
# VTBv32_init) + its already-verified subsystem dependencies (including the
# byte-exact-extracted VTB tables, src/v32/v32_viterbi_tables.c) - as 32-bit
# binaries, drives a 32-call (I,Q) sequence per trellis rate family (see
# test/v32/v32_engine.vtb_decoder.main.c's own header comment for the full
# design), and diffs stdout. Exit status is diff's: 0 means the port is
# behaviorally identical to the blob for this test's inputs.
#
# Usage: sh test/v32/v32_engine.vtb_decoder.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/v32/v32_engine.vtb_decoder}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_blob \
    test/v32/v32_engine.vtb_decoder.main.c test/stub_env.c $BLOB_O

gcc -m32 -Wall -Wextra -O0 -g -Isrc -o ${T}_test_port \
    test/v32/v32_engine.vtb_decoder.main.c test/stub_env.c \
    src/v32/v32_engine.c \
    src/v32/v32_datapath.c src/v32/v32_scrambler.c src/v32/v32_pulse_shaping.c src/v32/v32_timing.c \
    src/v32/v32_echo_canceller.c src/v32/v32_equalizer.c src/v32/v32_math.c \
    src/v32/v32_tables.c src/v32/v32_viterbi_tables.c \
    src/fpm/fpm_mrf.c src/fpm/fpm_agc.c src/fpm/fpm_math.c src/fpm/fpm_tables.c \
    src/fpm/fpm_tone.c src/fpm/fpm_mtd.c src/fpm/fpm_iir.c

${T_RUN}${T}_test_blob > ${T}_blob.out
${T_RUN}${T}_test_port > ${T}_port.out

diff -u ${T}_blob.out ${T}_port.out && \
    echo "MATCH: V.32 VTB_decoder port is behaviorally identical to dsplibs.o for this test"
