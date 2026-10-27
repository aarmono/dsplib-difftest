#!/bin/sh
# R24 (2 of 2): the V.17 decision callbacks with the REAL Viterbi decoder (blob: its own VTB_decoder inside FAX_FSE_decision_*; port: src/fax/fax_v17_decision.c -> FAXVTB_decoder (src/fax/fax_v17_viterbi.c) -> the ported VTB_decoder of
# src/v32/v32_datapath.c: at -m32 called directly, on x86-64 through a scratch image of the V.32 sub-object) and the typed front end FAXVTB_decoder itself vs the blob's VTB_decoder on the raw object: scenarios 4 .. 5
# (test/fax/fax_v17_decision.vtb.main.c + test/fax/r24_common.h), 32-bit, each in its OWN process.  The symbols src/v32/v32_datapath.c references but this test never reaches are abort stubs in the harness main.
# test/stub_env.c is built with the symbols the harness defines renamed (dsplibs_debug_printf).  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; exit 2 = a coverage counter is zero, 3 = harness violation.
# Env: PORT_R24 / PORT_R24V / PORT_R24T / PORT_R24P (src/fax/fax_v17_decision.c / fax_v17_viterbi.c / fax_v17_decision_tables.c / v32_datapath.c: the port file under test; mutation testing), PORT_XFLAGS, SCENARIOS, NOO2=1, TPREFIX, R24_SEED, R24_VERBOSE=1.
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r24_vtb writes test/golden/fax/fax_v17_decision.vtb.blob32.out.gz).
# Usage: sh test/fax/fax_v17_decision.vtb.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R24="${PORT_R24:-src/fax/fax_v17_decision.c}"
PORT_R24V="${PORT_R24V:-src/fax/fax_v17_viterbi.c}"
PORT_R24T="${PORT_R24T:-src/fax/fax_v17_decision_tables.c}"
PORT_R24P="${PORT_R24P:-src/v32/v32_datapath.c}"
PORT_SUP="src/v32/v32_viterbi_tables.c"
T="${TPREFIX:-test/.build/fax/fax_v17_decision.vtb}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"
SCENARIOS="${SCENARIOS:-4 5}"

gcc $CFLAGS -Ddsplibs_debug_printf=stub_env_unused_dbg -c test/stub_env.c -o ${T}_stub_env.o

gcc $HCF -DTEST_BLOB -no-pie -Isrc -o ${T}_test_blob test/fax/fax_v17_decision.vtb.main.c ${T}_stub_env.o $BLOB_O

n=0; pobjs=""
for f in $PORT_R24 $PORT_R24V $PORT_R24T $PORT_R24P $PORT_SUP; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; pobjs="$pobjs ${T}_port${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/fax/fax_v17_decision.vtb.main.c ${T}_stub_env.o $pobjs
if [ -z "$NOO2" ]; then
  n=0; pobjs=""
  for f in $PORT_R24 $PORT_R24V $PORT_R24T $PORT_R24P $PORT_SUP; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; pobjs="$pobjs ${T}_port${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/fax/fax_v17_decision.vtb.main.c ${T}_stub_env.o $pobjs
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
echo "MATCH: the V.17 decision callbacks + typed Viterbi front end (real Viterbi decoder, -O0 and -O2) is behaviorally identical to dsplibs.o for this test"
