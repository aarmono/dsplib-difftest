#!/bin/sh
# Builds both sides of the V.32 two-object end-to-end capstone REALISTIC-
# STAGGER differential test (test/v32/v32_engine.capstone_stagger.main.c - one
# linked against the real dsplibs.o, one against the hand-cleaned src/v32/v32_engine.c
# + its already-verified subsystem dependencies) as 32-bit binaries, runs
# the Answer-head-start-then-full-duplex scenario, and diffs stdout. Exit
# status is diff's: 0 means the port is behaviorally identical to the blob
# for this test's inputs. See test/v32/v32_engine.capstone_stagger.main.c's own
# header comment for why this scenario exists (a real observed 1.18s/283-
# call Answer head start, derived from a real back-to-back slmodemd call's
# own logged V32FP_create timestamps) and NOTES.md's dated entry for this
# chunk for the actual convergence result.
#
# Usage: sh test/v32/v32_engine.capstone_stagger.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/v32/v32_engine.capstone_stagger}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_blob \
    test/v32/v32_engine.capstone_stagger.main.c test/stub_env.c $BLOB_O

gcc -m32 -Wall -Wextra -O0 -g -Isrc -o ${T}_test_port \
    test/v32/v32_engine.capstone_stagger.main.c test/stub_env.c \
    src/v32/v32_engine.c \
    src/v32/v32_datapath.c src/v32/v32_scrambler.c src/v32/v32_pulse_shaping.c src/v32/v32_timing.c \
    src/v32/v32_echo_canceller.c src/v32/v32_equalizer.c src/v32/v32_math.c \
    src/v32/v32_tables.c src/v32/v32_viterbi_tables.c \
    src/fpm/fpm_mrf.c src/fpm/fpm_agc.c src/fpm/fpm_math.c src/fpm/fpm_tables.c \
    src/fpm/fpm_tone.c src/fpm/fpm_mtd.c src/fpm/fpm_iir.c

${T_RUN}${T}_test_blob > ${T}_blob.out
${T_RUN}${T}_test_port > ${T}_port.out

diff -u ${T}_blob.out ${T}_port.out && \
    echo "MATCH: V.32 two-object capstone REALISTIC-STAGGER (Answer head start, then real full-duplex exchange) port is behaviorally identical to dsplibs.o for this test"
