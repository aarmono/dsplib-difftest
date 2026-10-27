#!/bin/sh
# Builds both sides of the FPM_* toolkit differential test (one linked
# against the real dsplibs.o, one against the hand-cleaned src/fpm/fpm_*.c) as
# 32-bit binaries, runs them, and diffs their output. Exit status is diff's:
# 0 means the port is behaviorally identical to the blob for this test's
# inputs.
#
# Usage: sh test/fpm/fpm_toolkit.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/fpm/fpm_toolkit}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_blob \
    test/fpm/fpm_toolkit.main.c test/stub_env.c $BLOB_O

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_port \
    test/fpm/fpm_toolkit.main.c test/stub_env.c \
    src/fpm/fpm_math.c src/fpm/fpm_iir.c src/fpm/fpm_tone.c src/fpm/fpm_mrf.c \
    src/fpm/fpm_agc.c src/fpm/fpm_fsm.c src/fpm/fpm_fsd.c src/fpm/fpm_mtd.c \
    src/fpm/fpm_tables.c

${T_RUN}${T}_test_blob > ${T}_blob.out
${T_RUN}${T}_test_port > ${T}_port.out

diff -u ${T}_blob.out ${T}_port.out && echo "MATCH: FPM_* port is behaviorally identical to dsplibs.o for this test"
