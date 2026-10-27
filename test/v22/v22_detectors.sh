#!/bin/sh
# R05 (3 of 3): differential test of the V.22 detectors (Detect_Retrain / Detect_Rmloop2_ACK / Detect_1s / Detect_v22): the real dsplibs.o vs src/v22/v22_detectors.c +
# (no table), 32-bit, each scenario in its OWN process (test/v22/v22_detectors.main.c documents the scenarios, test/r04_common.h the callee recorders:
# Detect_v22 calls the real FPM_AGC_agc / FPM_MTD_detect of each side).  test/stub_env.c is built with the symbols the
# harness defines renamed.  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a harness
# assertion fails, either fails this script.
# Env: PORT_R05DT (port file(s) under test; mutation testing), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX, R05_SEED (other input stream),
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r05_detect writes test/golden/v22/v22_detectors.blob32.out.gz).
# Usage: sh test/v22/v22_detectors.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R05DT="${PORT_R05DT:-src/v22/v22_detectors.c}"
PORT_SUP="src/fpm/fpm_agc.c src/fpm/fpm_mtd.c src/fpm/fpm_math.c src/fpm/fpm_iir.c src/fpm/fpm_tables.c"
T="${TPREFIX:-test/.build/v22/v22_detectors}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"
SCENARIOS="${SCENARIOS:-0 1 2}"

gcc $CFLAGS -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memcpy=stub_env_unused_memcpy -c test/stub_env.c -o ${T}_stub_env.o

gcc $HCF -no-pie -Isrc -o ${T}_test_blob test/v22/v22_detectors.main.c ${T}_stub_env.o $BLOB_O

n=0; pobjs=""
for f in $PORT_R05DT; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_port0${n}_o0.o; pobjs="$pobjs ${T}_port0${n}_o0.o"; n=$((n+1)); done
n=1; objs=""
for f in $PORT_SUP; do gcc -m32 -w -O0 -g $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; objs="$objs ${T}_port${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/v22/v22_detectors.main.c ${T}_stub_env.o $pobjs $objs
if [ -z "$NOO2" ]; then
  n=0; pobjs=""
  for f in $PORT_R05DT; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port0${n}_o2.o; pobjs="$pobjs ${T}_port0${n}_o2.o"; n=$((n+1)); done
  n=1; objs=""
  for f in $PORT_SUP; do gcc -m32 -w -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; objs="$objs ${T}_port${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/v22/v22_detectors.main.c ${T}_stub_env.o $pobjs $objs
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
echo "MATCH: the V.22 detectors port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
