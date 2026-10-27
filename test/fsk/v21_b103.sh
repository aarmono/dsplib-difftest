#!/bin/sh
# Builds both sides of the Bell 103/V.21 (B103FP_*) differential test (one
# linked against the real dsplibs.o, one against the hand-cleaned
# src/fsk/v21_b103.c + its FPM_*/table dependencies) as 32-bit binaries, runs
# each scenario in its own process (see test/fsk/v21_b103.main.c's header
# comment for why), and diffs the concatenated output. Exit status is
# diff's: 0 means the port is behaviorally identical to the blob for this
# test's inputs.
#
# Usage: sh test/fsk/v21_b103.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/fsk/v21_b103}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_blob \
    test/fsk/v21_b103.main.c test/stub_env.c $BLOB_O

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_port \
    test/fsk/v21_b103.main.c test/stub_env.c \
    src/fsk/v21_b103.c \
    src/fpm/fpm_math.c src/fpm/fpm_iir.c src/fpm/fpm_tone.c src/fpm/fpm_mrf.c \
    src/fpm/fpm_agc.c src/fpm/fpm_fsm.c src/fpm/fpm_fsd.c src/fpm/fpm_mtd.c \
    src/fpm/fpm_tables.c src/fsk/fsk_tables.c

NUM_SCENARIOS=6

: > ${T}_blob.out
: > ${T}_port.out
i=0
while [ "$i" -lt "$NUM_SCENARIOS" ]; do
  ${T_RUN}${T}_test_blob "$i" >> ${T}_blob.out
  ${T_RUN}${T}_test_port "$i" >> ${T}_port.out
  i=$((i + 1))
done

diff -u ${T}_blob.out ${T}_port.out && echo "MATCH: B103/V.21 port is behaviorally identical to dsplibs.o for this test"
