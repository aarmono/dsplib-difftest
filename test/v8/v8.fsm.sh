#!/bin/sh
# Builds both sides of the V.8 handshake FSM (v8handshak) differential test
# - one linked against the real dsplibs.o, one against the hand-cleaned
# src/v8/v8.c + src/v8/v8_tables.c - as 32-bit binaries, runs them, and diffs
# their output. Exit status is diff's: 0 means the port is behaviorally
# identical to the blob for this test's inputs.
#
# v8handshak's own dispatch is now FULLY covered (every tx-phase/rx-phase/
# sub-state combination it can reach - see src/v8/v8.c's header comment on
# that function). What's still not covered by ANY test is
# V8UpdateModemParameters and the object-lifecycle glue (V8Create/V8Delete/
# V8Control/v8_create/v8_delete/v8_process/dp_v8_init/dp_v8_exit) -
# v8handshak itself never calls any of them, so this test's direct-call
# coverage of v8handshak is unaffected by their absence. See NOTES.md.
#
# Usage: sh test/v8/v8.fsm.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/v8/v8.fsm}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_blob \
    test/v8/v8.fsm.main.c test/stub_env.c $BLOB_O

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_port \
    test/v8/v8.fsm.main.c test/stub_env.c src/v8/v8.c src/v8/v8_tables.c

${T_RUN}${T}_test_blob > ${T}_blob.out
${T_RUN}${T}_test_port > ${T}_port.out

diff -u ${T}_blob.out ${T}_port.out && echo "MATCH: V.8 handshake FSM (partial) port is behaviorally identical to dsplibs.o for this test"
