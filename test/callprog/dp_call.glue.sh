#!/bin/sh
# R11 (3 of 4, id r11_glue): differential test of the call dp glue (call_create / call_run / call_delete / call_GetSRegister through the dp_operations record dp_call_init registers, dp_call_init / dp_call_exit) with
# CALLPROG_Create / _Dial / _Progress / _Delete and RcFixed_Create / _Delete / _Resample replaced by scripted recorders (WEAKENED in a copy of dsplibs.o with objcopy so the recorders win in the blob run;
# also modem_get_param / modem_get_sreg / modem_dp_register / sysdep_* / dsplibs_debug_printf): the real dsplibs.o vs src/callprog/dp_call.c, 32-bit, each scenario in its OWN process
# (test/callprog/dp_call.glue.main.c documents the scenarios 0 .. 3, test/callprog/r11_common.h).  test/stub_env.c is built with the symbols the harness defines renamed.  The port is built at -O0 AND -O2 and BOTH are
# diffed against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a harness assertion fails, either fails this script.
# Env: PORT_R11G (port file(s) under test; mutation testing; PORT_SUP = the other port files the harness links), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1,
#      TPREFIX, R11_SEED (other input stream), R11_VERBOSE=1,
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r11_glue writes test/golden/callprog/dp_call.glue.blob32.out.gz).
# Usage: sh test/callprog/dp_call.glue.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R11G="${PORT_R11G:-src/callprog/dp_call.c}"
PORT_SUP=""
T="${TPREFIX:-test/.build/callprog/dp_call.glue}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"
SCENARIOS="${SCENARIOS:-0 1 2 3}"

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_printf -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -Dsysdep_memcpy=stub_env_unused_memcpy -Dsysdep_strlen=stub_env_unused_strlen -Dsysdep_strcpy=stub_env_unused_strcpy -Dmodem_get_param=stub_env_unused_mgp -Dmodem_get_sreg=stub_env_unused_msr -Dmodem_dp_register=stub_env_unused_mdr -Dmodem_dp_deregister=stub_env_unused_mdd -c test/stub_env.c -o ${T}_stub_env.o

WEAK=""
for f in CALLPROG_Create CALLPROG_Dial CALLPROG_Progress CALLPROG_Delete RcFixed_Create RcFixed_Delete RcFixed_Resample; do WEAK="$WEAK --weaken-symbol=$f"; done
objcopy $WEAK $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF -no-pie -Isrc -o ${T}_test_blob test/callprog/dp_call.glue.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

n=0; pobjs=""
for f in $PORT_R11G; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; pobjs="$pobjs ${T}_port${n}_o0.o"; n=$((n+1)); done
n=1; objs=""
for f in $PORT_SUP; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o0.o; objs="$objs ${T}_sup${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/callprog/dp_call.glue.main.c ${T}_stub_env.o $pobjs $objs
if [ -z "$NOO2" ]; then
  n=0; pobjs=""
  for f in $PORT_R11G; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; pobjs="$pobjs ${T}_port${n}_o2.o"; n=$((n+1)); done
  n=1; objs=""
  for f in $PORT_SUP; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o2.o; objs="$objs ${T}_sup${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/callprog/dp_call.glue.main.c ${T}_stub_env.o $pobjs $objs
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
echo "MATCH: the call glue port (-O0 and -O2) are behaviorally identical to dsplibs.o for this test"
