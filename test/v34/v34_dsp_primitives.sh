#!/bin/sh
# Builds both sides of V.34's 25 hand-cleaned leaf DSP primitives
# differential test (one linked against the real dsplibs.o, one against
# src/v34/v34_dsp_primitives.c + src/v34/v34_timing_hilbert_tables.c) as 32-bit binaries, runs them,
# and diffs their output. Exit status is diff's: 0 means the port is
# behaviorally identical to the blob for this test's inputs.
#
# `V34EchoHistoryBackwardClean` is NOT covered - it isn't in
# src/v34/v34_dsp_primitives.c at all (still unresolved, see NOTES.md).
#
# Each test group runs in its OWN process (scenarios 0-5), same convention
# as test/build_and_diff_v32_primitives.sh - see that script's own comment
# for why.
#
# Usage: sh test/v34/v34_dsp_primitives.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/v34/v34_dsp_primitives}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"

gcc -m32 -Wall -Wextra -O0 -g -o ${T}_test_blob \
    test/v34/v34_dsp_primitives.main.c test/stub_env.c $BLOB_O

gcc -m32 -Wall -Wextra -O0 -g -Isrc -o ${T}_test_port \
    test/v34/v34_dsp_primitives.main.c test/stub_env.c \
    src/v34/v34_dsp_primitives.c src/v34/v34_timing_hilbert_tables.c

SCENARIOS="0 1 2 3 4 5"

: > ${T}_blob.out
: > ${T}_port.out
for i in $SCENARIOS; do
  ${T_RUN}${T}_test_blob "$i" >> ${T}_blob.out
  ${T_RUN}${T}_test_port "$i" >> ${T}_port.out
done

diff -u ${T}_blob.out ${T}_port.out && \
    echo "MATCH: V.34 leaf DSP primitives port is behaviorally identical to dsplibs.o for this test"
