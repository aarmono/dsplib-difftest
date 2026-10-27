#!/bin/sh
# R11 (2 of 4, id r11_callprog): differential test of CALLPROG_Create / _Dial / _Progress / _Delete / _Status_string with ALL callees replaced by scripted recorders (cadence_*, Dialer*,
# ResetCallingTone / GenerateCallingTone, Dual_TONE_*, _iir_filter_*: WEAKENED in a copy of dsplibs.o with objcopy so the recorders win in the blob run; also modem_get_param, sysdep_*,
# dsplibs_debug_printf): the real dsplibs.o vs src/callprog/callprog.c, 32-bit, each scenario in its OWN process (test/callprog/callprog.main.c documents the scenarios 0 .. 4, test/callprog/r11_common.h).  test/stub_env.c is built with the symbols the harness defines renamed.  The port is built at -O0 AND -O2 and BOTH are
# diffed against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a harness assertion fails, either fails this script.
# Env: PORT_R11P (port file(s) under test; mutation testing; PORT_SUP = the other port files the harness links), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1,
#      TPREFIX, R11_SEED (other input stream), R11_VERBOSE=1,
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r11_callprog writes test/golden/callprog/callprog.blob32.out.gz).
# Usage: sh test/callprog/callprog.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R11P="${PORT_R11P:-src/callprog/callprog.c}"
PORT_SUP="src/callprog/callprog_tables.c"
T="${TPREFIX:-test/.build/callprog/callprog}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"
SCENARIOS="${SCENARIOS:-0 1 2 3 4}"

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_printf -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -Dsysdep_memcpy=stub_env_unused_memcpy -Dsysdep_strlen=stub_env_unused_strlen -Dsysdep_strcpy=stub_env_unused_strcpy -Dmodem_get_param=stub_env_unused_mgp -Dmodem_get_sreg=stub_env_unused_msr -c test/stub_env.c -o ${T}_stub_env.o

WEAK=""
for f in cadence_create cadence_progress cadence_reset cadence_delete Dual_TONE_create Dual_TONE_detect Dual_TONE_delete _iir_filter_create _iir_filter_progress _iir_filter_delete ResetCallingTone GenerateCallingTone DialerCreate DialerProgress DialerAbort; do WEAK="$WEAK --weaken-symbol=$f"; done
objcopy $WEAK $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF -no-pie -Isrc -o ${T}_test_blob test/callprog/callprog.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

n=0; pobjs=""
for f in $PORT_R11P; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; pobjs="$pobjs ${T}_port${n}_o0.o"; n=$((n+1)); done
n=1; objs=""
for f in $PORT_SUP; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o0.o; objs="$objs ${T}_sup${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/callprog/callprog.main.c ${T}_stub_env.o $pobjs $objs
if [ -z "$NOO2" ]; then
  n=0; pobjs=""
  for f in $PORT_R11P; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; pobjs="$pobjs ${T}_port${n}_o2.o"; n=$((n+1)); done
  n=1; objs=""
  for f in $PORT_SUP; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o2.o; objs="$objs ${T}_sup${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/callprog/callprog.main.c ${T}_stub_env.o $pobjs $objs
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
echo "MATCH: the CALLPROG port (-O0 and -O2) are behaviorally identical to dsplibs.o for this test"
