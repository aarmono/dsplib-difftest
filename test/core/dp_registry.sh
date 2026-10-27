#!/bin/sh
# R31 (id r31_dp): differential test of prop_dp_init / prop_dp_exit: the real dsplibs.o vs src/core/dp_registry.c, 32-bit, each scenario in its OWN process (test/core/dp_registry.main.c documents
# the scenarios).  The harness owns every callee: the 14 dp_{call,b103,v22,v23,v32,v8,vpcm}_{init,exit} are recorders (the blob's copies are made WEAK with objcopy so the harness' win).  The
# port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a harness assertion fails, either fails this script.
# Env: PORT_R31 (port file under test; mutation testing), PORT_XFLAGS, SCENARIOS, NOO2=1, TPREFIX.
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r31_dp writes test/golden/core/dp_registry.blob32.out.gz).
# Usage: sh test/core/dp_registry.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R31="${PORT_R31:-src/core/dp_registry.c}"
T="${TPREFIX:-test/.build/core/dp_registry}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter"
SCENARIOS="${SCENARIOS:-0 1 2 3}"
WEAK=""
for m in call b103 v22 v23 v32 v8 vpcm; do WEAK="$WEAK --weaken-symbol=dp_${m}_init --weaken-symbol=dp_${m}_exit"; done

gcc $CFLAGS -c test/stub_env.c -o ${T}_stub_env.o
objcopy $WEAK $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF -DTEST_BLOB -no-pie -Isrc -o ${T}_test_blob test/core/dp_registry.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

gcc $CFLAGS $PORT_XFLAGS -Isrc -c $PORT_R31 -o ${T}_port0_o0.o
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/core/dp_registry.main.c ${T}_stub_env.o ${T}_port0_o0.o
if [ -z "$NOO2" ]; then
  gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $PORT_R31 -o ${T}_port0_o2.o
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/core/dp_registry.main.c ${T}_stub_env.o ${T}_port0_o2.o
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
echo "MATCH: prop_dp_init / prop_dp_exit over recorder dp_*_init / dp_*_exit (-O0 and -O2) are behaviorally identical to dsplibs.o for this test"
