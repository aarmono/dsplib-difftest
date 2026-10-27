#!/bin/sh
# Builds both sides of the V.34 "tier 4a" RX frame-decode chain differential test
# - putFrame, shellDemapper, decodeDepth, demapFrame, decoderv34 - (real dsplibs.o
# vs src/v34/v34_frame_decoder.c) as 32-bit binaries, runs each scenario in its OWN process,
# and diffs the output. A scenario exits 2 if its own coverage assertions fail, 3
# if a harness assumption (table content, in-range oracle state) is violated; either
# fails this script (set -e) - "MATCH" is only printed if every scenario both
# matched AND reached its branches.
#
# The blob binary additionally links the port's extracted tables (tier 3 + tier 4)
# under a `port_` prefix so the harness can content-check the blob's own GLOBAL
# grid/kLookup/kTable/kkNormal/kkInvert/gInvertPat/rxvect4 at start-up.
# Both binaries compile a private "oracle" copy of the PRISTINE src/v34/v34_frame_decoder.c
# into the harness (renamed oracle_*, PORT_TRACE_INDEX defined) to keep every input
# inside grid[529] / gInvertPat[16] (the blob does not bounds-check them).
#
# The port under test is built at -O0 AND -O2 and BOTH are diffed against the blob.
# PORT_T4A (env) overrides the port source under test (mutation testing); the oracle
# always uses the pristine src/v34/v34_frame_decoder.c.
#
# Scenarios: 0 putFrame (recorder), 1 putFrame (real GPC and GPA), 2 shellDemapper
# small/boundary, 3 shellDemapper wild 16-bit, 4 decodeDepth random, 5 decodeDepth
# lattice sweep, 6 demapFrame single calls, 7 demapFrame chains, 8 decoderv34 slicer,
# 9 decoderv34 frame mode, 10 decoderv34 frame mode with debug level 2, 11 shellDemapper
# with parameters inside X/Y/Z (T07: layout independent, see NOTES.md).
#
# Usage: sh test/v34/v34_frame_decoder.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_T4A="${PORT_T4A:-src/v34/v34_frame_decoder.c}"
T="${TPREFIX:-test/.build/v34/v34_frame_decoder}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Isrc -Wall -Wextra -O0 -g"
OTHER="src/v34/v34_frame_tables.c src/v34/v34_init.c src/v34/v34_agc_demod.c src/v34/v34_dsp_primitives.c src/v34/v34_timing_hilbert_tables.c \
    src/v34/v34_init_tables.c src/fpm/fpm_tables.c"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7 8 9 10 11}"

gcc $CFLAGS -c src/v34/v34_init_tables.c -o ${T}_t3tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t3tables_prefixed.o
gcc $CFLAGS -c src/v34/v34_frame_tables.c -o ${T}_t4tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t4tables_prefixed.o

gcc $CFLAGS -DTEST_BLOB -o ${T}_test_blob test/v34/v34_frame_decoder.main.c test/stub_env.c \
    ${T}_t3tables_prefixed.o ${T}_t4tables_prefixed.o $BLOB_O

gcc $CFLAGS -o ${T}_test_port test/v34/v34_frame_decoder.main.c test/stub_env.c $PORT_T4A $OTHER
gcc -m32 -Isrc -Wall -Wextra -O2 -o ${T}_test_port_o2 test/v34/v34_frame_decoder.main.c test/stub_env.c $PORT_T4A $OTHER

# scenarios are independent processes: run them concurrently, then concatenate in order
rm -f ${T}_s*.out
pids=""
for i in $SCENARIOS; do
  ( ${T_RUN}${T}_test_blob "$i" > ${T}_s$i.blob.out ) & pids="$pids $!"
  ( ${T_RUN}${T}_test_port "$i" > ${T}_s$i.port.out ) & pids="$pids $!"
  ( ${T_RUN}${T}_test_port_o2 "$i" > ${T}_s$i.o2.out ) & pids="$pids $!"
done
rc=0
for p in $pids; do wait $p || rc=1; done
: > ${T}_blob.out
: > ${T}_port.out
: > ${T}_port_o2.out
for i in $SCENARIOS; do
  cat ${T}_s$i.blob.out >> ${T}_blob.out
  cat ${T}_s$i.port.out >> ${T}_port.out
  cat ${T}_s$i.o2.out >> ${T}_port_o2.out
done
rm -f ${T}_s*.out
if [ $rc -ne 0 ]; then echo "SCENARIO FAILURE (non-zero exit: coverage, harness assertion or crash)"; exit 1; fi

diff -u ${T}_blob.out ${T}_port.out
diff -u ${T}_blob.out ${T}_port_o2.out && \
    echo "MATCH: V.34 tier-4a RX frame-decode chain port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
