#!/bin/sh
# Builds both sides of the V.34 "tier 10b" differential test (the 16 DEAD plain-C DSP-chain functions: real dsplibs.o vs
# src/v34/v34_unused.c) as 32-bit binaries, runs each scenario in its OWN process, and diffs the output.
# A scenario exits 2 if its own coverage counters fail, 3 if a harness assumption / independent model is violated; either
# fails this script - "MATCH" is only printed if every scenario both matched AND reached its branches.
#
# The five CALLEES (rxinit, detectorinit, V34SetupModulator, dftupdate, dftenergy) are RECORDERS defined in the test main in
# BOTH binaries: the blob binary links a copy of dsplibs.o in which those five definitions are WEAK (objcopy
# --weaken-symbol), so the strong recorders win for every reference.  The harness reads the tables xyz / hsine* / bpv22* by
# NAME: the blob's own in the blob binary, the port's own in the port binary; the blob binary additionally links the port's
# V.8 costbl under a `port_` prefix for the cosread check.
#
# The port under test is built at -O0 AND -O2 and BOTH are diffed against the blob.  PORT_T10B (env) overrides the port
# source under test (mutation testing); SCENARIOS (env) selects scenarios; NOO2=1 skips the -O2 build; TPREFIX (env)
# overrides the binary / output prefix (parallel mutation runs); T10B_SEED (env) selects an alternative input stream.
#
# Scenarios: 0 trivial setters / initialisers / polyValue + cosread (exhaustive) / scaleVector, 1 initG248, 2 decision,
# 3 agcadapt (independent model), 4 preempindex (model), 5 detectRetrainReq (model on the recorders' state), 6 setupreceiver,
# 7 dpskinit, 8 detectRetrainReq with n <= 6 (T07: layout independent, see NOTES.md).
#
# Usage: sh test/v34/v34_unused.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_T10B="${PORT_T10B:-src/v34/v34_unused.c}"
T="${TPREFIX:-test/.build/v34/v34_unused}"
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Isrc -Wall -Wextra -Wno-unused-function -O0 -g"
HCF="-m32 -Isrc -Wall -Wextra -Wno-unused-function -O2 -g"
TABLES="src/v34/v34_init_tables.c src/v34/v34_frame_tables.c src/v34/v34_symbol_tables.c src/v8/v8_tables.c"
PORTDEPS=""
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7 8}"
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"

gcc $CFLAGS -Ddsplibs_debug_printf=stub_env_unused_debug_printf -c test/stub_env.c -o ${T}_stub_env.o
objcopy --weaken-symbol=rxinit --weaken-symbol=detectorinit --weaken-symbol=V34SetupModulator \
        --weaken-symbol=dftupdate --weaken-symbol=dftenergy $BLOB_O ${T}_dsplibs_weak.o
gcc $CFLAGS -c src/v8/v8_tables.c -o ${T}_v8tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_v8tables_prefixed.o

gcc $HCF -DTEST_BLOB -o ${T}_test_blob test/v34/v34_unused.main.c ${T}_stub_env.o ${T}_v8tables_prefixed.o ${T}_dsplibs_weak.o

gcc $CFLAGS -c $PORT_T10B -o ${T}_port_o0.o
gcc $HCF -o ${T}_test_port test/v34/v34_unused.main.c ${T}_stub_env.o ${T}_port_o0.o $TABLES
if [ -z "$NOO2" ]; then
  gcc -m32 -Isrc -Wall -Wextra -Wno-unused-function -O2 -c $PORT_T10B -o ${T}_port_o2.o
  gcc $HCF -o ${T}_test_port_o2 test/v34/v34_unused.main.c ${T}_stub_env.o ${T}_port_o2.o $TABLES
fi

# scenarios are independent processes: run them concurrently, then concatenate in order
rm -f ${T}_s*.out
pids=""
for i in $SCENARIOS; do
  ( ${R}${T}_test_blob "$i" > ${T}_s$i.blob.out ) & pids="$pids $!"
  ( ${R}${T}_test_port "$i" > ${T}_s$i.port.out ) & pids="$pids $!"
  if [ -z "$NOO2" ]; then ( ${R}${T}_test_port_o2 "$i" > ${T}_s$i.o2.out ) & pids="$pids $!"; fi
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
echo "MATCH: V.34 tier-10b dead DSP-chain port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')) is behaviorally identical to dsplibs.o for this test"
