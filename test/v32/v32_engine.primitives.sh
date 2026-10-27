#!/bin/sh
# Builds both sides of the V.32 primitive/DSP toolkit differential test (one
# linked against the real dsplibs.o, one against the hand-cleaned
# src/v32/v32_*.c files) as 32-bit binaries, runs them, and diffs their output.
# Exit status is diff's: 0 means the port is behaviorally identical to the
# blob for this test's inputs.
#
# Single-process test (like build_and_diff_fpm.sh/build_and_diff_v8.sh) -
# every function under test is called directly with a synthetic, harness-
# owned object buffer, not through the (unported, next-chunk) top-level
# V32FP_create/V32LocLoopNextState lifecycle.
#
# Usage: sh test/v32/v32_engine.primitives.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/v32/v32_engine.primitives}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"

gcc -m32 -Wall -Wextra -O0 -g -Isrc -o ${T}_test_blob \
    test/v32/v32_engine.primitives.main.c test/stub_env.c $BLOB_O

gcc -m32 -Wall -Wextra -O0 -g -Isrc -o ${T}_test_port \
    test/v32/v32_engine.primitives.main.c test/stub_env.c \
    src/v32/v32_datapath.c src/v32/v32_scrambler.c src/v32/v32_pulse_shaping.c src/v32/v32_timing.c \
    src/v32/v32_echo_canceller.c src/v32/v32_equalizer.c src/v32/v32_math.c \
    src/v32/v32_tables.c src/v32/v32_viterbi_tables.c \
    src/fpm/fpm_mrf.c src/fpm/fpm_agc.c src/fpm/fpm_math.c src/fpm/fpm_tables.c

# Every test group runs in its OWN process, same convention as
# test/build_and_diff_b103.sh/test/build_and_diff_v23.sh - see
# test/v32/v32_engine.primitives.main.c's own header comment on main() for why
# (running several groups back-to-back in one process made the REAL
# dsplibs.o intermittently segfault, ASLR-dependent).
#
# Scenario 8 is deliberately EXCLUDED (a manual debug trace, not part of the
# automated comparison - see its own header comment in
# test/v32/v32_engine.primitives.main.c). Scenario 9 is the targeted ECC
# real-IMAP/QMAP-table crash repro added during the 2026-09-29 follow-up
# that root-caused and fixed the V.32 ECC crash (see NOTES.md) - listed
# explicitly (not a contiguous range) so scenario 8 stays excluded.
SCENARIOS="0 1 2 3 4 5 6 7 9"

: > ${T}_blob.out
: > ${T}_port.out
for i in $SCENARIOS; do
  ${T_RUN}${T}_test_blob "$i" >> ${T}_blob.out
  ${T_RUN}${T}_test_port "$i" >> ${T}_port.out
done

diff -u ${T}_blob.out ${T}_port.out && \
    echo "MATCH: V.32 primitive/DSP toolkit port is behaviorally identical to dsplibs.o for this test"
