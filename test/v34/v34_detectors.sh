#!/bin/sh
# Builds both sides of the V.34 "tier 5c" detector differential test - tone_detect, dftupdate, dftenergy,
# fskdetect, fskdemodulate - (real dsplibs.o vs src/v34/v34_detectors.c) as 32-bit binaries, runs each scenario
# in its OWN process, and diffs the output. A scenario exits 2 if its own coverage assertions fail, 3 if a
# harness assumption (table content, x87 control word, independent model, ...) is violated; either fails this
# script - "MATCH" is only printed if every scenario both matched AND reached its branches.
#
# The blob binary additionally links the port's extracted tables (tier 4/5 + v8) under a `port_` prefix so
# the harness can content-check the blob's own c1200_/c2400_/intcoef1-3/costbl (and, through the GLOBAL
# intcoef1 anchor, the LOCAL fsklpfcoeff600) at start-up.
#
# The port under test is built at -O0 AND -O2 and BOTH are diffed against the blob.
# PORT_T5C (env) overrides the port source under test (mutation testing); PORT_XFLAGS (env) adds compiler flags
# to the two port builds (e.g. "-msse2 -mfpmath=sse" to prove that a pure-double implementation is caught);
# SCENARIOS (env) selects scenarios; NOO2=1 skips the -O2 port build.
#
# Scenarios: 0 tone_detect random single, 1 tone_detect chain, 2 dftupdate designed, 3 dftupdate special
# doubles, 4 dftupdate reachable chain (sums beyond 2^53), 5 dftenergy integers/big sums, 6 dftenergy special
# doubles, 7 dftupdate+dftenergy realistic chain, 8 fskdetect single, 9 fskdetect chain, 10 fskdemodulate
# single, 11 fskdemodulate chain.
#
# Usage: sh test/v34/v34_detectors.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_T5C="${PORT_T5C:-src/v34/v34_detectors.c}"
T="${TPREFIX:-test/.build/v34/v34_detectors}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Isrc -Wall -Wextra -O0 -g"
OTHER="src/v34/v34_symbol_tables.c src/v34/v34_frame_tables.c src/v8/v8_tables.c"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7 8 9 10 11}"

gcc $CFLAGS -c src/v34/v34_frame_tables.c -o ${T}_t4tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t4tables_prefixed.o
gcc $CFLAGS -c src/v34/v34_symbol_tables.c -o ${T}_t5tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t5tables_prefixed.o
gcc $CFLAGS -c src/v8/v8_tables.c -o ${T}_v8tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_v8tables_prefixed.o

gcc $CFLAGS -DTEST_BLOB -o ${T}_test_blob test/v34/v34_detectors.main.c test/stub_env.c \
    ${T}_t4tables_prefixed.o ${T}_t5tables_prefixed.o ${T}_v8tables_prefixed.o $BLOB_O

gcc $CFLAGS $PORT_XFLAGS -c $PORT_T5C -o ${T}_port_o0.o
gcc $CFLAGS -o ${T}_test_port test/v34/v34_detectors.main.c ${T}_port_o0.o $OTHER
if [ -z "$NOO2" ]; then
  gcc -m32 -Isrc -Wall -Wextra -O2 $PORT_XFLAGS -c $PORT_T5C -o ${T}_port_o2.o
  gcc -m32 -Isrc -Wall -Wextra -O2 -o ${T}_test_port_o2 test/v34/v34_detectors.main.c ${T}_port_o2.o $OTHER
fi

# scenarios are independent processes: run them concurrently, then concatenate in order
rm -f ${T}_s*.out
pids=""
for i in $SCENARIOS; do
  ( ${T_RUN}${T}_test_blob "$i" > ${T}_s$i.blob.out ) & pids="$pids $!"
  ( ${T_RUN}${T}_test_port "$i" > ${T}_s$i.port.out ) & pids="$pids $!"
  if [ -z "$NOO2" ]; then ( ${T_RUN}${T}_test_port_o2 "$i" > ${T}_s$i.o2.out ) & pids="$pids $!"; fi
done
rc=0
for p in $pids; do wait $p || rc=1; done
: > ${T}_blob.out
: > ${T}_port.out
: > ${T}_port_o2.out
for i in $SCENARIOS; do
  cat ${T}_s$i.blob.out >> ${T}_blob.out
  cat ${T}_s$i.port.out >> ${T}_port.out
  if [ -z "$NOO2" ]; then cat ${T}_s$i.o2.out >> ${T}_port_o2.out; fi
done
rm -f ${T}_s*.out
if [ $rc -ne 0 ]; then echo "SCENARIO FAILURE (non-zero exit: coverage, harness assertion or crash)"; exit 1; fi

diff -u ${T}_blob.out ${T}_port.out
if [ -z "$NOO2" ]; then diff -u ${T}_blob.out ${T}_port_o2.out; fi
echo "MATCH: V.34 tier-5c detectors (tone_detect/dftupdate/dftenergy/fskdetect/fskdemodulate) port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
