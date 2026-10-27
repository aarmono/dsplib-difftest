#!/bin/sh
# Builds both sides of the V.32 higher-rate RX pipeline END-TO-END
# integration differential test (one linked against the real dsplibs.o, one
# against the hand-cleaned src/*.c sources), as 32-bit binaries, runs the
# single scenario (2 trellis-coded rate families, each driven through a real
# LocLoop-role object's real state walk + real case-9 rate switch + a 64-call
# real RxHdxData barrage), and diffs stdout. Exit status is diff's: 0 means
# the port is behaviorally identical to the blob for this test's inputs.
#
# Usage: sh test/v32/v32_engine.vtb_e2e.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/v32/v32_engine.vtb_e2e}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_blob \
    test/v32/v32_engine.vtb_e2e.main.c test/stub_env.c $BLOB_O

gcc -m32 -Wall -Wextra -O0 -g -Isrc -o ${T}_test_port \
    test/v32/v32_engine.vtb_e2e.main.c test/stub_env.c \
    src/v32/v32_engine.c \
    src/v32/v32_datapath.c src/v32/v32_scrambler.c src/v32/v32_pulse_shaping.c src/v32/v32_timing.c \
    src/v32/v32_echo_canceller.c src/v32/v32_equalizer.c src/v32/v32_math.c \
    src/v32/v32_tables.c src/v32/v32_viterbi_tables.c \
    src/fpm/fpm_mrf.c src/fpm/fpm_agc.c src/fpm/fpm_math.c src/fpm/fpm_tables.c \
    src/fpm/fpm_tone.c src/fpm/fpm_mtd.c src/fpm/fpm_iir.c

${T_RUN}${T}_test_blob > ${T}_blob.out
${T_RUN}${T}_test_port > ${T}_port.out

diff -u ${T}_blob.out ${T}_port.out && \
    echo "MATCH: V.32 higher-rate RX pipeline end-to-end integration port is behaviorally identical to dsplibs.o for this test"
