#!/bin/sh
# R23 (1 of 3): differential test of the fax V.29 receiver, tables, lifecycle and the VMI wrappers: the real dsplibs.o vs src/fax/fax_v29_rx.c + src/fax/fax_equalizer.c + src/fax/fax_timing.c + tables and R18 / R22 / FPM callees, 32-bit, each scenario in its OWN process
# (test/fax/fax_v29_rx.main.c documents the scenarios 0 .. 2, test/fax/r23_common.h the callee recorders: sysdep_malloc / sysdep_free / dsplibs_debug_printf are the harness' (block ids and i386-mapped sizes are logged)).
# test/stub_env.c is built with the symbols the harness defines renamed.  The blob's LOCAL init_vmi_v29rx is made global with objcopy for the blob run.  The port is built at -O0 AND -O2 and BOTH are diffed
# against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a harness assertion fails, either fails this script.  The transmitter of the end-to-end streams is the blob's own V29TX in the blob run
# and src/fax/fax_v29_tx.c in the port runs (R22 proves them identical).
# Env: PORT_R23 / PORT_R23F / PORT_R23S (src/fax/fax_v29_rx.c / src/fax/fax_equalizer.c / src/fax/fax_timing.c: the port file under test; mutation testing), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX, R23_SEED (other input stream),
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r23_life writes test/golden/fax/fax_v29_rx.life.blob32.out.gz).
# Usage: sh test/fax/fax_v29_rx.life.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R23="${PORT_R23:-src/fax/fax_v29_rx.c}"
PORT_R23F="${PORT_R23F:-src/fax/fax_equalizer.c}"
PORT_R23S="${PORT_R23S:-src/fax/fax_timing.c}"
PORT_SUP="src/fax/fax_v29_rx_tables.c src/fax/fax_rx_common_tables.c src/fax/fax_v29_tx.c src/fax/fax_pulse_shaping.c src/fax/fax_v29_tx_tables.c src/fax/fax_common_tables.c src/fax/fax_fifo_sgd.c src/fax/fax_scrambler.c src/fax/fax_modem_wrappers.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c src/v32/v32_math.c src/fax/fax_v21_tables.c src/fpm/fpm_math.c src/fpm/fpm_iir.c src/fpm/fpm_tone.c src/fpm/fpm_mrf.c src/fpm/fpm_agc.c src/fpm/fpm_fsm.c src/fpm/fpm_fsd.c src/fpm/fpm_mtd.c src/fpm/fpm_tables.c src/fsk/fsk_tables.c"
T="${TPREFIX:-test/.build/fax/fax_v29_rx.life}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"
SCENARIOS="${SCENARIOS:-0 1 2}"

gcc $CFLAGS -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbg -c test/stub_env.c -o ${T}_stub_env.o
objcopy --globalize-symbol=init_vmi_v29rx $BLOB_O ${T}_dsplibs_glob.o

gcc $HCF -DTEST_BLOB -no-pie -Isrc -o ${T}_test_blob test/fax/fax_v29_rx.main.c ${T}_stub_env.o ${T}_dsplibs_glob.o

n=0; pobjs=""
for f in $PORT_R23 $PORT_R23F $PORT_R23S; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; pobjs="$pobjs ${T}_port${n}_o0.o"; n=$((n+1)); done
n=1; objs=""
for f in $PORT_SUP; do gcc -m32 -w -O0 -g $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o0.o; objs="$objs ${T}_sup${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/fax/fax_v29_rx.main.c ${T}_stub_env.o $pobjs $objs
if [ -z "$NOO2" ]; then
  n=0; pobjs=""
  for f in $PORT_R23 $PORT_R23F $PORT_R23S; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; pobjs="$pobjs ${T}_port${n}_o2.o"; n=$((n+1)); done
  n=1; objs=""
  for f in $PORT_SUP; do gcc -m32 -w -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o2.o; objs="$objs ${T}_sup${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/fax/fax_v29_rx.main.c ${T}_stub_env.o $pobjs $objs
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
echo "MATCH: the fax V.29 receiver port (tables, lifecycle and the VMI wrappers, -O0 and -O2) is behaviorally identical to dsplibs.o for this test"
