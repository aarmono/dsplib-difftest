#!/bin/sh
# R13 (1 of 1): differential test of the caller-ID modem and API (cid_modem / create_cid / create_cid_dtmf / reset_cid / data_*_output / cid_* / CID_create / CID_process / CID_delete): the real
# dsplibs.o vs src/cid/cid_modem.c (+ the R12 detectors src/cid/cid_detect.c + src/fpm/fpm_div32.c + src/cid/cid_tables.c and src/fpm/fpm_mrf.c as the real callees of each side), 32-bit, each scenario in its OWN
# process (test/cid/cid_modem.main.c documents the scenarios and the callee recorders sysdep_malloc / free / memset / strlen, modem_send_to_tty).  test/stub_env.c is built with the
# symbols the harness defines renamed.  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a harness
# assertion fails, either fails this script.
# Env: PORT_R13 (port file under test; mutation testing), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX, R13_SEED (other input stream),
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r13_cid writes test/golden/cid/cid_modem.blob32.out.gz).
# Usage: sh test/cid/cid_modem.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R13="${PORT_R13:-src/cid/cid_modem.c}"
PORT_R12ALL="$PORT_R13 src/cid/cid_detect.c src/fpm/fpm_div32.c src/cid/cid_tables.c"
PORT_SUP="src/fpm/fpm_iir.c src/fpm/fpm_tables.c src/fpm/fpm_mrf.c"
T="${TPREFIX:-test/.build/cid/cid_modem}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"
SCENARIOS="${SCENARIOS:-0 1 2 3 4}"

gcc $CFLAGS -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memcpy=stub_env_unused_memcpy -Dsysdep_memset=stub_env_unused_memset -Dsysdep_strlen=stub_env_unused_strlen -Dmodem_send_to_tty=stub_env_unused_tty -c test/stub_env.c -o ${T}_stub_env.o

gcc $HCF -no-pie -Isrc -o ${T}_test_blob test/cid/cid_modem.main.c ${T}_stub_env.o $BLOB_O

n=0; pobjs=""
for f in $PORT_R12ALL; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_port0${n}_o0.o; pobjs="$pobjs ${T}_port0${n}_o0.o"; n=$((n+1)); done
n=1; objs=""
for f in $PORT_SUP; do gcc -m32 -w -O0 -g $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; objs="$objs ${T}_port${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/cid/cid_modem.main.c ${T}_stub_env.o $pobjs $objs
if [ -z "$NOO2" ]; then
  n=0; pobjs=""
  for f in $PORT_R12ALL; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port0${n}_o2.o; pobjs="$pobjs ${T}_port0${n}_o2.o"; n=$((n+1)); done
  n=1; objs=""
  for f in $PORT_SUP; do gcc -m32 -w -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; objs="$objs ${T}_port${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/cid/cid_modem.main.c ${T}_stub_env.o $pobjs $objs
fi

# scenarios are independent processes: run them concurrently, then concatenate in order
rm -f ${T}_s*.out
pids=""
for i in $SCENARIOS; do
  ( timeout 600 ${T_RUN}${T}_test_blob "$i" > ${T}_s$i.blob.out ) & pids="$pids $!"
  ( timeout 600 ${T_RUN}${T}_test_port "$i" > ${T}_s$i.port.out ) & pids="$pids $!"
  if [ -z "$NOO2" ]; then ( timeout 600 ${T_RUN}${T}_test_port_o2 "$i" > ${T}_s$i.o2.out ) & pids="$pids $!"; fi
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
echo "MATCH: the caller-ID modem and API port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
