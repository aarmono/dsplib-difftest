#!/bin/sh
# Builds both sides of the v8handshakinit/initTxSequence differential test
# (one linked against the real dsplibs.o, one against the hand-cleaned
# src/v8/v8.c + src/v8/v8_tables.c) as 32-bit binaries, runs them, and diffs
# their output. Exit status is diff's: 0 means the port is behaviorally
# identical to the blob for this test's inputs.
#
# Single-process test, same rationale as v8/v8.main.sh: every
# scenario uses a fresh, harness-owned stack buffer standing in for
# V8Create's allocation, so there's no shared-state-across-instances
# concern to isolate into separate processes.
#
# Usage: sh test/v8/v8.init.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/v8/v8.init}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_blob \
    test/v8/v8.init.main.c test/stub_env.c $BLOB_O

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_port \
    test/v8/v8.init.main.c test/stub_env.c src/v8/v8.c src/v8/v8_tables.c

${T_RUN}${T}_test_blob > ${T}_blob.out
${T_RUN}${T}_test_port > ${T}_port.out

diff -u ${T}_blob.out ${T}_port.out && echo "MATCH: v8handshakinit/initTxSequence port is behaviorally identical to dsplibs.o for this test"
