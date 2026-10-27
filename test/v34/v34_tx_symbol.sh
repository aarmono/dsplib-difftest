#!/bin/sh
# Builds both sides of the V.34 "tier 5a" tx-symbol-path differential test - getFrame, txmit,
# txmitdibit, txmitquadbit - (real dsplibs.o vs src/v34/v34_tx_symbol.c) as 32-bit binaries, runs each
# scenario in its OWN process, and diffs the output. A scenario exits 2 if its own coverage
# assertions fail, 3 if a harness assumption (table content, model self-check, recorder call
# count) is violated; either fails this script - "MATCH" is only printed if every scenario both
# matched AND reached its branches.
#
# The blob binary additionally links the port's extracted tables (tier 3 + tier 5) under a
# `port_` prefix so the harness can content-check the blob's own lsbMask / vect4 / vect16 /
# rxvect4 at start-up (incl. the vect16 over-read: blob vect16[16..19] == port rxvect4[0..3]).
#
# The port under test is built at -O0 AND -O2 and BOTH are diffed against the blob.
# PORT_T5A (env) overrides the port source under test (mutation testing).
#
# Scenarios: 0 getFrame (recorder), 1 getFrame (real scrambleGPC and GPA), 2 txmit random,
# 3 tx chain (txmit/txmitdibit/txmitquadbit), 4 txmitdibit, 5 txmitquadbit random, 6 txmitquadbit
# grid, 7 txmit ring/bulk wrap-edge grid, 8 getFrame chains (recorder, GPC, GPA).
#
# Usage: sh test/v34/v34_tx_symbol.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_T5A="${PORT_T5A:-src/v34/v34_tx_symbol.c}"
T="${TPREFIX:-test/.build/v34/v34_tx_symbol}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Isrc -Wall -Wextra -O0 -g"
OTHER="src/v34/v34_symbol_tables.c src/v34/v34_frame_tables.c src/v34/v34_init.c src/v34/v34_agc_demod.c src/v34/v34_dsp_primitives.c \
    src/v34/v34_timing_hilbert_tables.c src/v34/v34_init_tables.c src/fpm/fpm_tables.c"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7 8}"

gcc $CFLAGS -c src/v34/v34_init_tables.c -o ${T}_t3tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t3tables_prefixed.o
gcc $CFLAGS -c src/v34/v34_symbol_tables.c -o ${T}_t5tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t5tables_prefixed.o

gcc $CFLAGS -DTEST_BLOB -o ${T}_test_blob test/v34/v34_tx_symbol.main.c test/stub_env.c \
    ${T}_t3tables_prefixed.o ${T}_t5tables_prefixed.o $BLOB_O

gcc $CFLAGS -o ${T}_test_port test/v34/v34_tx_symbol.main.c test/stub_env.c $PORT_T5A $OTHER
gcc -m32 -Isrc -Wall -Wextra -O2 -o ${T}_test_port_o2 test/v34/v34_tx_symbol.main.c test/stub_env.c $PORT_T5A $OTHER

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
    echo "MATCH: V.34 tier-5a tx symbol path (getFrame/txmit/txmitdibit/txmitquadbit) port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
