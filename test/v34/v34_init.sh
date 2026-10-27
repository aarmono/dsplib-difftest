#!/bin/sh
# Builds both sides of the V.34 "tier 3, groups A+B" hand-cleaned functions
# differential test (real dsplibs.o vs src/v34/v34_init.c + src/v34/v34_agc_demod.c +
# src/v34/v34_dsp_primitives.c + tables) as 32-bit binaries, runs each scenario in its
# OWN process (intermittent real-blob segfault avoidance, see
# test/build_and_diff_v32_primitives.sh), and diffs the output. A scenario exits
# 2 if its own coverage assertions fail (branch never reached), which fails this
# script (set -e) - a "MATCH" is only printed if every scenario both matched
# AND reached its branches.
#
# Scenarios: 0 txrxdmainit, 1 detectorinit, 2 scrambleGPC+scrambleGPA,
# 3 descrambleGPC+descrambleGPA, 4 preinitV34, 5 preinitdigital, 6 txinit,
# 7 rxinit, 8 VPcmV34LogTimingOffset, 9 setTimingStateParameters (all 9 modes,
# both jump tables), 10 setInitialPhase, 11 TimingV34, 12 debug-level-2 output, 13 TimingV34 with the S[] indices inside
# the array (T07: layout independent, see NOTES.md).
#
# Usage: sh test/v34/v34_init.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/v34/v34_init}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"

gcc -m32 -Isrc -Wall -Wextra -O0 -g -DTEST_BLOB -o ${T}_test_blob \
    test/v34/v34_init.main.c test/stub_env.c $BLOB_O

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_port \
    test/v34/v34_init.main.c test/stub_env.c \
    src/v34/v34_init.c src/v34/v34_agc_demod.c src/v34/v34_dsp_primitives.c src/v34/v34_timing_hilbert_tables.c \
    src/v34/v34_init_tables.c src/fpm/fpm_tables.c

SCENARIOS="0 1 2 3 4 5 6 7 8 9 10 11 12 13"

: > ${T}_blob.out
: > ${T}_port.out
for i in $SCENARIOS; do
  ${T_RUN}${T}_test_blob "$i" >> ${T}_blob.out
  ${T_RUN}${T}_test_port "$i" >> ${T}_port.out
done

diff -u ${T}_blob.out ${T}_port.out && \
    echo "MATCH: V.34 tier-3 groups A+B port is behaviorally identical to dsplibs.o for this test"
