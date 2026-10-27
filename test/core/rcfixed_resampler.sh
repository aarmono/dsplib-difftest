#!/bin/sh
# R01: differential test of the RcFixed polyphase sample-rate converter (RcFixed_Create / Delete / Reset / Resample / Check_Combination): real dsplibs.o vs
# src/core/rcfixed_resampler.c (+ src/core/rcfixed_resampler_tables.c), 32-bit, each scenario in its OWN process (test/core/rcfixed_resampler.main.c documents the six scenarios).  The harness owns
# sysdep_malloc / sysdep_free / sysdep_memset (callee recorders: allocation order and sizes, the memset of the state, a heap with red zones and id-patterned
# garbage), so test/stub_env.c is built with those three renamed.  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2
# when a coverage counter is zero and 3 when a harness / model assertion fails, either fails this script.  The blob binary also links the port's tables
# (extracted with readelf): the harness identifies the table a blob state points to by CONTENT, so a wrong extraction shows up as `coef=-1`.
# Env: PORT_RCFIXED (port under test; mutation testing), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX, RCFIXED_SEED (other input stream),
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh rcfixed writes test/golden/core/rcfixed_resampler.blob32.out.gz).
# Usage: sh test/core/rcfixed_resampler.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_RCFIXED="${PORT_RCFIXED:-src/core/rcfixed_resampler.c}"
T="${TPREFIX:-test/.build/core/rcfixed_resampler}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5}"

gcc $CFLAGS -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -c test/stub_env.c -o ${T}_stub_env.o

gcc $CFLAGS -no-pie -Isrc -o ${T}_test_blob test/core/rcfixed_resampler.main.c ${T}_stub_env.o src/core/rcfixed_resampler_tables.c $BLOB_O

gcc $CFLAGS $PORT_XFLAGS -Isrc -c $PORT_RCFIXED -o ${T}_port_o0.o
gcc $CFLAGS -no-pie -Isrc -o ${T}_test_port test/core/rcfixed_resampler.main.c ${T}_stub_env.o ${T}_port_o0.o src/core/rcfixed_resampler_tables.c
if [ -z "$NOO2" ]; then
  gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $PORT_RCFIXED -o ${T}_port_o2.o
  gcc -m32 -Wall -Wextra -O2 -no-pie -Isrc -o ${T}_test_port_o2 test/core/rcfixed_resampler.main.c ${T}_stub_env.o ${T}_port_o2.o src/core/rcfixed_resampler_tables.c
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
echo "MATCH: RcFixed (Create / Delete / Reset / Resample / Check_Combination) port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
