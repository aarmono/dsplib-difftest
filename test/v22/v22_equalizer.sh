#!/bin/sh
# R04 (1 of 2): differential test of the V.22 / V.22bis FSE (V22_FSE_init / _free / _getdiag / _receive, FSEv22_decision12 / _decision24): the real dsplibs.o vs src/v22/v22_equalizer.c
# (+ src/v22/v22_tables.c), 32-bit, each scenario in its OWN process (test/v22/v22_equalizer.main.c documents the six scenarios, test/r04_common.h the callee recorders: sysdep_malloc /
# sysdep_free / sysdep_memcpy are the harness' (ids, sizes and offsets in shorts are logged); FPM_atan / FPM_phasor are the real callees of each side: the blob's own, and
# src/v32/v32_math.c + src/fpm/fpm_math.c + src/fpm/fpm_tables.c).  The decision callback slot of the object is set by the harness (a scripted fake or the real decision12 / 24).
# test/stub_env.c is built with the symbols the harness defines renamed.  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2 when a
# coverage counter is zero and 3 when a harness assertion fails, either fails this script.
# Env: PORT_R04FSE (port file(s) under test; mutation testing), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX, R04_SEED (other input stream),
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r04_fse writes test/golden/v22/v22_equalizer.blob32.out.gz).
# Usage: sh test/v22/v22_equalizer.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R04FSE="${PORT_R04FSE:-src/v22/v22_equalizer.c}"
PORT_SUP="src/v22/v22_tables.c src/fpm/fpm_math.c src/v32/v32_math.c src/fpm/fpm_tables.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c"
T="${TPREFIX:-test/.build/v22/v22_equalizer}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5}"

gcc $CFLAGS -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memcpy=stub_env_unused_memcpy -c test/stub_env.c -o ${T}_stub_env.o

gcc $HCF -no-pie -Isrc -o ${T}_test_blob test/v22/v22_equalizer.main.c ${T}_stub_env.o $BLOB_O

gcc $CFLAGS $PORT_XFLAGS -Isrc -c $PORT_R04FSE -o ${T}_port0_o0.o
n=1; objs=""
for f in $PORT_SUP; do gcc -m32 -w -O0 -g $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; objs="$objs ${T}_port${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/v22/v22_equalizer.main.c ${T}_stub_env.o ${T}_port0_o0.o $objs
if [ -z "$NOO2" ]; then
  gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $PORT_R04FSE -o ${T}_port0_o2.o
  n=1; objs=""
  for f in $PORT_SUP; do gcc -m32 -w -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; objs="$objs ${T}_port${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/v22/v22_equalizer.main.c ${T}_stub_env.o ${T}_port0_o2.o $objs
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
echo "MATCH: the V.22 FSE (V22_FSE_init / _free / _getdiag / _receive, FSEv22_decision12 / _decision24) port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
