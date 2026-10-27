#!/bin/sh
# R30 (2 of 3, id r30_glue): differential test of the fax entry points slmodemd links, FAX_create / FAX_delete / FAX_class1_command / FAX_process, with the class 1 layer
# (fax_class1_create / progress / command / delete) and the resamplers (RcFixed_Create / Delete / Resample) replaced by scripted recorders (WEAKENED in a copy of dsplibs.o with objcopy so
# the recorders win in the blob run; also modem_get_sreg / modem_recv_from_tty / modem_send_to_tty / sysdep_* / dsplibs_debug_printf are the harness' own): the real dsplibs.o vs
# src/fax/fax_api.c, 32-bit, each scenario in its OWN process (scenarios 4 .. 5 of test/fax/fax_api.main.c, build via test/fax/fax_api.glue.main.c).  The port is built at -O0 AND -O2 and BOTH are diffed
# against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a harness assertion fails, either fails this script.
# Env: PORT_R30 (src/fax/fax_api.c: the port file under test; mutation testing), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX, R30_SEED (other input stream),
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r30_glue writes test/golden/fax/fax_api.glue.blob32.out.gz).
# Usage: sh test/fax/fax_api.glue.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R30="${PORT_R30:-src/fax/fax_api.c}"
PORT_SUP=""
T="${TPREFIX:-test/.build/fax/fax_api.glue}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter"
SCENARIOS="${SCENARIOS:-4 5}"
WEAK=""
for f in fax_class1_create fax_class1_progress fax_class1_command fax_class1_delete RcFixed_Create RcFixed_Delete RcFixed_Resample; do WEAK="$WEAK --weaken-symbol=$f"; done

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_dbg -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -Dsysdep_memcpy=stub_env_unused_memcpy -Dsysdep_sprintf=stub_env_unused_sprintf -Dmodem_get_sreg=stub_env_unused_mgs -Dmodem_recv_from_tty=stub_env_unused_mrt -Dmodem_send_to_tty=stub_env_unused_mst -c test/stub_env.c -o ${T}_stub_env.o
objcopy $WEAK $BLOB_O ${T}_dsplibs_weak.o

gcc $HCF -DTEST_BLOB -no-pie -Isrc -o ${T}_test_blob test/fax/fax_api.glue.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

n=0; pobjs=""
for f in $PORT_R30 $PORT_SUP; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; pobjs="$pobjs ${T}_port${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/fax/fax_api.glue.main.c ${T}_stub_env.o $pobjs
if [ -z "$NOO2" ]; then
  n=0; pobjs=""
  for f in $PORT_R30 $PORT_SUP; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; pobjs="$pobjs ${T}_port${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/fax/fax_api.glue.main.c ${T}_stub_env.o $pobjs
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
echo "MATCH: the fax entry points (FAX_create / FAX_delete / FAX_class1_command / FAX_process over scripted class 1 and converter recorders, -O0 and -O2) are behaviorally identical to dsplibs.o for this test"
