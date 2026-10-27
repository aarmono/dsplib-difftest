#!/bin/sh
# Builds both sides of the V.32 Answer NextState differential test (one linked
# against the real dsplibs.o, one against the hand-cleaned src/v32/v32_engine.c + its
# already-verified subsystem dependencies) as 32-bit binaries, runs both
# phase-groups' scenarios, and diffs stdout. Exit status is diff's: 0 means
# the port is behaviorally identical to the blob for this test's inputs.
#
# Phase-group 1: cases 0-4 + the universal 0x21, plus TxHdxTone - organic
# give-up/timeout path through case 2, plus the direct-poke branch proving
# case 2's else branch and cases 3/4.
# Phase-group 2: cases 6-9, plus RxHdxNoSignal - organic case 6
# TX-budget transition, an organic RxHdxNoSignal batch + direct guard-check
# for case 7, then (phase-group 5 below) the real organic loop-back through
# state 0x1d, and an organic case 8->9->10 transition.
# Phase-group 3: cases 0xa-0x12, plus RateToSeq - the real
# TRN/rate-encode/LISTEN segment, driven fully organically (no direct-call
# crash workaround needed, unlike V32OrgNextState's own phase-group 3 -
# every case in this range resets its own budget to a fresh value).
# Phase-group 4: cases 0x15-0x1e, plus CodeFinalRateSeq - the
# rate-negotiation ack/CONNECT tail. Driven fully organically for the
# incompatible-rate (class 6) branch all the way to the terminal CONNECT
# state, plus a direct reg4 poke for the compatible (class 0) happy path.
# Phase-group 5 (this chunk): case 0x1d's own "local retrain" loop-back body
# (needs TxHdxFinishFrame, hand-cleaned this chunk) - drives the REAL organic
# path state 0x1d -> TxHdxFinishFrame -> state 8 -> case 8's own body -> state
# 9, replacing the prior chunk's own direct substate=8 poke that skipped case
# 0x1d's body entirely. This COMPLETES V32AnsNextState's entire main connect
# path with zero remaining stubs.
#
# Usage: sh test/v32/v32_engine.ans.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/v32/v32_engine.ans}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_blob \
    test/v32/v32_engine.ans.main.c test/stub_env.c $BLOB_O

gcc -m32 -Wall -Wextra -O0 -g -Isrc -o ${T}_test_port \
    test/v32/v32_engine.ans.main.c test/stub_env.c \
    src/v32/v32_engine.c \
    src/v32/v32_datapath.c src/v32/v32_scrambler.c src/v32/v32_pulse_shaping.c src/v32/v32_timing.c \
    src/v32/v32_echo_canceller.c src/v32/v32_equalizer.c src/v32/v32_math.c \
    src/v32/v32_tables.c src/v32/v32_viterbi_tables.c \
    src/fpm/fpm_mrf.c src/fpm/fpm_agc.c src/fpm/fpm_math.c src/fpm/fpm_tables.c \
    src/fpm/fpm_tone.c src/fpm/fpm_mtd.c src/fpm/fpm_iir.c

${T_RUN}${T}_test_blob > ${T}_blob.out
${T_RUN}${T}_test_port > ${T}_port.out

diff -u ${T}_blob.out ${T}_port.out && \
    echo "MATCH: V.32 Answer NextState (phase-groups 1-5: cases 0-4,6-0x12,0x15-0x1e,0x1d + 0x21 - the COMPLETE main connect path, zero remaining stubs) port is behaviorally identical to dsplibs.o for this test"
