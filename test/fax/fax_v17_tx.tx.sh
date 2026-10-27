#!/bin/sh
# R25 (2 of 3): differential test of the fax V.17 transmitter, transmit streams (the whole state machine at both rates, sample for sample):
# the real dsplibs.o vs src/fax/fax_v17_tx.c + src/fax/fax_pulse_shaping.c + src/fax/fax_v17_tx_tables.c and R18's FIFO / SGD / SDM / SMC / wrappers, 32-bit, each scenario in its OWN process (test/fax/fax_v17_tx.main.c documents
# the scenarios 0 .. 6, test/fax/r25_common.h the callee recorders: sysdep_malloc / sysdep_free / dsplibs_debug_printf are the harness' (block ids and i386-mapped sizes are logged)).  test/stub_env.c is built
# with the symbols the harness defines renamed.  The blob's LOCAL init_vmi_v17tx is made global with objcopy for the blob run.  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario
# exits 2 when a coverage counter is zero and 3 when a harness assertion fails, either fails this script.
# Env: PORT_R25 / PORT_R25P (src/fax/fax_v17_tx.c / src/fax/fax_pulse_shaping.c: the port file under test; mutation testing), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX, R25_SEED (other input stream),
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r25_tx writes test/golden/fax/fax_v17_tx.tx.blob32.out.gz).
# Usage: sh test/fax/fax_v17_tx.tx.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R25="${PORT_R25:-src/fax/fax_v17_tx.c}"
PORT_SUP="${PORT_R25P:-src/fax/fax_pulse_shaping.c} src/fax/fax_v17_tx_tables.c src/fax/fax_common_tables.c src/fax/fax_fifo_sgd.c src/fax/fax_scrambler.c src/fax/fax_modem_wrappers.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c"
T="${TPREFIX:-test/.build/fax/fax_v17_tx.tx}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"
SCENARIOS="${SCENARIOS:-2}"

gcc $CFLAGS -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbg -c test/stub_env.c -o ${T}_stub_env.o
objcopy --globalize-symbol=init_vmi_v17tx $BLOB_O ${T}_dsplibs_glob.o

gcc $HCF -DTEST_BLOB -no-pie -Isrc -o ${T}_test_blob test/fax/fax_v17_tx.main.c ${T}_stub_env.o ${T}_dsplibs_glob.o

n=0; pobjs=""
for f in $PORT_R25; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; pobjs="$pobjs ${T}_port${n}_o0.o"; n=$((n+1)); done
n=1; objs=""
for f in $PORT_SUP; do gcc -m32 -w -O0 -g $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o0.o; objs="$objs ${T}_sup${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/fax/fax_v17_tx.main.c ${T}_stub_env.o $pobjs $objs
if [ -z "$NOO2" ]; then
  n=0; pobjs=""
  for f in $PORT_R25; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; pobjs="$pobjs ${T}_port${n}_o2.o"; n=$((n+1)); done
  n=1; objs=""
  for f in $PORT_SUP; do gcc -m32 -w -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o2.o; objs="$objs ${T}_sup${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/fax/fax_v17_tx.main.c ${T}_stub_env.o $pobjs $objs
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
echo "MATCH: the fax V.17 transmitter port (transmit streams, -O0 and -O2) is behaviorally identical to dsplibs.o for this test"
