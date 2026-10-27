#!/bin/sh
# R10: differential test of the dialer: DialerCreate / DialerProgress / DialerAbort / GetDialerConfig / IsDialStringInvalid / Dialer_IsDialStringInvalid, PulseDialDigit / IsPulseDialerReady /
# LastPulseDigitDialed / SetPulseBreakTime / SetPulseMakeTime, ResetCallingTone / GenerateCallingTone (and, through them, the blob-LOCAL AnalyseDialString and
# GetNextDigitAndReturnNextState): the real dsplibs.o vs src/callprog/dialer.c, 32-bit, each scenario in its OWN process (test/callprog/dialer.main.c documents the scenarios 0 .. 7, test/callprog/r10_common.h the
# callee recorders: modem_get_param / modem_set_param, sysdep_strlen / strcpy, dsplibs_debug_printf, TONE_read / FP_Pow).  The two R11 callees TONE_read and FP_Pow are WEAKENED in a copy of
# dsplibs.o (objcopy --weaken-symbol; relocations are symbol-relative) so that the harness' recorders win in the blob run; the port run links only the recorders.  test/stub_env.c is
# built with the symbols the harness defines renamed.  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a
# harness assertion fails, either fails this script.
# Env: PORT_R10 (port file(s) under test; mutation testing), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX, R10_SEED (other input stream), R10_VERBOSE=1,
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r10_dialer writes test/golden/callprog/dialer.blob32.out.gz).
# Usage: sh test/callprog/dialer.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R10="${PORT_R10:-src/callprog/dialer.c}"
T="${TPREFIX:-test/.build/callprog/dialer}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7}"

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_printf -Dsysdep_strlen=stub_env_unused_strlen -Dsysdep_strcpy=stub_env_unused_strcpy -Dmodem_get_param=stub_env_unused_mgp -Dmodem_set_param=stub_env_unused_msp -c test/stub_env.c -o ${T}_stub_env.o

objcopy --weaken-symbol=TONE_read --weaken-symbol=FP_Pow $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF -no-pie -Isrc -o ${T}_test_blob test/callprog/dialer.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

n=0; pobjs=""
for f in $PORT_R10; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; pobjs="$pobjs ${T}_port${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/callprog/dialer.main.c ${T}_stub_env.o $pobjs
if [ -z "$NOO2" ]; then
  n=0; pobjs=""
  for f in $PORT_R10; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; pobjs="$pobjs ${T}_port${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/callprog/dialer.main.c ${T}_stub_env.o $pobjs
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
echo "MATCH: the dialer port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
