#!/bin/sh
# Builds both sides of the V.34 "tier 10a" differential test (39 VPcm* / V34XF_* wrapper and accessor functions: real
# dsplibs.o vs src/vpcm/v34_api.c) as 32-bit binaries, runs each scenario in its OWN process, and diffs the output.
# A scenario exits 2 if its own coverage counters fail, 3 if a harness assumption / independent model is violated; either
# fails this script - "MATCH" is only printed if every scenario both matched AND reached its branches.
#
# The four CALLEES of the wrappers (v34handshakinit, preinitdigital, v34setuptxmit, edprintf) are RECORDERS defined in the
# test main in BOTH binaries: the blob binary links a copy of dsplibs.o in which those four definitions are WEAK
# (objcopy --weaken-symbol), so the strong recorders win for every reference.
#
# The port under test is built at -O0 AND -O2 and BOTH are diffed against the blob.  PORT_T10A (env) overrides the port
# source under test (mutation testing); SCENARIOS (env) selects scenarios; NOO2=1 skips the -O2 build; TPREFIX (env)
# overrides the binary / output prefix (parallel mutation runs); T10A_SEED (env) selects an alternative input stream.
#
# Scenarios: 0 simple setters / indicators / pointer returns, 1 interpretMohTimeout (exhaustive) + SetMinimumSigLevel (table),
# 2 state getters + GetSNR, 3 GetCurrentTxBitRate (x87; independent integer model), 4 edprintf group, 5 InterpretMohMessageBits
# (independent model), 6 Indicate* / SetV90RateReneg / K56Flex*, 7 InitiateRateRenegotiation / InitiateHangUp.
#
# Usage: sh test/vpcm/v34_api.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_T10A="${PORT_T10A:-src/vpcm/v34_api.c}"
T="${TPREFIX:-test/.build/vpcm/v34_api}"
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -Wno-unused-function -O0 -g -Isrc"
HCF="-m32 -Wall -Wextra -Wno-unused-function -O2 -g -Isrc"
TABLES="src/v34/v34_api_tables.c"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7}"
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"

gcc $CFLAGS -Ddsplibs_debug_printf=stub_env_unused_debug_printf -c test/stub_env.c -o ${T}_stub_env.o
objcopy --weaken-symbol=v34handshakinit --weaken-symbol=preinitdigital --weaken-symbol=v34setuptxmit \
        --weaken-symbol=edprintf $BLOB_O ${T}_dsplibs_weak.o

gcc $HCF -DTEST_BLOB -o ${T}_test_blob test/vpcm/v34_api.main.c ${T}_stub_env.o $TABLES ${T}_dsplibs_weak.o

gcc $CFLAGS -c $PORT_T10A -o ${T}_port_o0.o
gcc $HCF -o ${T}_test_port test/vpcm/v34_api.main.c ${T}_stub_env.o ${T}_port_o0.o $TABLES
if [ -z "$NOO2" ]; then
  gcc -m32 -Wall -Wextra -Wno-unused-function -O2 -Isrc -c $PORT_T10A -o ${T}_port_o2.o
  gcc $HCF -o ${T}_test_port_o2 test/vpcm/v34_api.main.c ${T}_stub_env.o ${T}_port_o2.o $TABLES
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
echo "MATCH: V.34 tier-10a wrapper/accessor port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')) is behaviorally identical to dsplibs.o for this test"
