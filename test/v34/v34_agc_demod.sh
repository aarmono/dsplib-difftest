#!/bin/sh
# Builds both sides of the V.34 "tier 2" hand-cleaned functions differential
# test (real dsplibs.o vs src/v34/v34_agc_demod.c + src/v34/v34_dsp_primitives.c +
# src/v34/v34_timing_hilbert_tables.c + src/fpm/fpm_tables.c) as 32-bit binaries, runs each
# scenario in its OWN process (intermittent real-blob segfault avoidance, see
# test/build_and_diff_v32_primitives.sh), and diffs the output. A scenario
# exits 2 if its own coverage assertions fail (branch never reached), which
# fails this script (set -e) - a "MATCH" is only printed if every scenario
# both matched AND reached its branches.
#
# Scenarios: 0 bitreverse, 1 V34agc, 2 V34demodulate (LOCAL in blob, reached
# via symbol-address arithmetic from `bitreverse`), 3 V34ModulatorProcess,
# 4 V34GiveProbeResults, 5 V34InitializeImplementationSpecific+v34FreezeEcho,
# 6 V34SetINFO0dBits+V34GiveINFO0dBits, 7 V34GiveINFO1aBits,
# 8 V34SetINFO0aBits, 9 debug-level-2 output (format strings/arg order,
# incl. V34EchoCleanUp/V34EchoReportCoeff strings), 10 V34ModulatorProcess with taps/stride inside the history (T07: layout independent).
#
# Usage: sh test/v34/v34_agc_demod.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/v34/v34_agc_demod}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"

gcc -m32 -Isrc -Wall -Wextra -O0 -g -DTEST_BLOB -o ${T}_test_blob \
    test/v34/v34_agc_demod.main.c test/stub_env.c $BLOB_O

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_port \
    test/v34/v34_agc_demod.main.c test/stub_env.c \
    src/v34/v34_agc_demod.c src/v34/v34_dsp_primitives.c src/v34/v34_timing_hilbert_tables.c src/fpm/fpm_tables.c

SCENARIOS="0 1 2 3 4 5 6 7 8 9 10"

: > ${T}_blob.out
: > ${T}_port.out
for i in $SCENARIOS; do
  ${T_RUN}${T}_test_blob "$i" >> ${T}_blob.out
  ${T_RUN}${T}_test_port "$i" >> ${T}_port.out
done

diff -u ${T}_blob.out ${T}_port.out && \
    echo "MATCH: V.34 tier-2 port is behaviorally identical to dsplibs.o for this test"
