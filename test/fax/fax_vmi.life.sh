#!/bin/sh
# R27 (2 of 3): differential test of the FAXVMI object and the ops tables, the real dsplibs.o vs src/fax/fax_vmi.c, 32-bit, each scenario in its OWN process: wiring of the vxx_* tables (recorder modems),
# FAXVMI_create (NULL / supplied objects, ids 0 .. 12, forked malloc-failure probes) / _delete / _control / _status / _message, null_*, and FAXVMI_process through the recorder modems -- scenarios 0 4 5
# of test/fax/fax_vmi.main.c (REC build).
# The 48 modem wrappers v21tx_* .. v17rx_* and GetT30FrameIDFromBuffer / GetT30FrameNameByID of the blob are WEAKENED with objcopy so the harness' RECORDERS win in the blob run; the port run links only
# src/fax/fax_vmi.c and the recorders.  test/stub_env.c is built with the symbols the harness defines renamed.  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2 when
# a coverage counter is zero and 3 when a harness assertion fails, either fails this script.
# Env: PORT_R27 (src/fax/fax_vmi.c: the port file under test; mutation testing), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX, R27_SEED (other input stream),
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r27_life writes test/golden/fax/fax_vmi.life.blob32.out.gz).
# Usage: sh test/fax/fax_vmi.life.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R27="${PORT_R27:-src/fax/fax_vmi.c}"
T="${TPREFIX:-test/.build/fax/fax_vmi.life}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter"
SCENARIOS="${SCENARIOS:-0 4 5}"
WEAK=""
for m in v21tx v21rx v27tx v27rx v29tx v29rx v17tx v17rx; do for f in create delete process status control message; do WEAK="$WEAK --weaken-symbol=${m}_$f"; done; done
WEAK="$WEAK --weaken-symbol=GetT30FrameIDFromBuffer --weaken-symbol=GetT30FrameNameByID"

gcc $CFLAGS -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbg -c test/stub_env.c -o ${T}_stub_env.o
objcopy $WEAK $BLOB_O ${T}_dsplibs_weak.o

gcc $HCF -DTEST_BLOB -no-pie -Isrc -o ${T}_test_blob test/fax/fax_vmi.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

gcc $CFLAGS $PORT_XFLAGS -Isrc -c $PORT_R27 -o ${T}_port0_o0.o
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/fax/fax_vmi.main.c ${T}_stub_env.o ${T}_port0_o0.o
if [ -z "$NOO2" ]; then
  gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $PORT_R27 -o ${T}_port0_o2.o
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/fax/fax_vmi.main.c ${T}_stub_env.o ${T}_port0_o2.o
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
echo "MATCH: the FAXVMI object / ops tables port (wiring, lifecycle, control, status, message, process, -O0 and -O2) is behaviorally identical to dsplibs.o for this test"
