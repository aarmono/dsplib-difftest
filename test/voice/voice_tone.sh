#!/bin/sh
# R15: differential test of the voice tone / MTK / silence / detector layer (TONE_create / _detect / _delete, MTK_phasor, notch, silence_*, detector_*, _status and the LOCAL GetGain):
# the real dsplibs.o vs src/voice/voice_tone.c + voice_tone_tables.c, 32-bit, each scenario in its OWN process (test/voice/voice_tone.main.c documents the scenarios 0 .. 5, test/voice/r15_common.h the
# callee recorders sysdep_malloc / free / memset, dsplibs_debug_printf).  The R16 / R09 callees (create_dtmf, dtmf_progress, cadence_create / _progress / _delete) are weakened in the blob and
# replaced by the harness' scripted recorders, modem_get_param (stub_env's abort stub is renamed) by a parameter table; the blob's LOCAL GetGain is globalised (objcopy) and called through a
# regparm(2) pointer.  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a harness assertion fails,
# either fails this script.
# Env: PORT_R15T (port file(s) under test; mutation testing; PORT_SUP = the other port files the harness links), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX,
#      R15_SEED (other input stream), R15_VERBOSE=1, R15_SCALE=n (divide every counted loop; debugging), R15_SWEEP=n (divide only the big random sweeps; the mutation runs use 8).
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r15 writes test/golden/voice/voice_tone.blob32.out.gz).
# Usage: sh test/voice/voice_tone.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R15T="${PORT_R15T:-src/voice/voice_tone.c}"
PORT_SUP="${PORT_SUP:-src/voice/voice_tone_tables.c}"
T="${TPREFIX:-test/.build/voice/voice_tone}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5}"

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_printf -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -Dmodem_get_param=stub_env_unused_mgp -c test/stub_env.c -o ${T}_stub_env.o

objcopy --weaken-symbol=create_dtmf --weaken-symbol=dtmf_progress --weaken-symbol=cadence_create --weaken-symbol=cadence_progress --weaken-symbol=cadence_delete --globalize-symbol=GetGain $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF -DTEST_BLOB -no-pie -Isrc -o ${T}_test_blob test/voice/voice_tone.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o -lm

n=0; pobjs=""
for f in $PORT_R15T; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; pobjs="$pobjs ${T}_port${n}_o0.o"; n=$((n+1)); done
n=1; objs=""
for f in $PORT_SUP; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o0.o; objs="$objs ${T}_sup${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/voice/voice_tone.main.c ${T}_stub_env.o $pobjs $objs -lm
if [ -z "$NOO2" ]; then
  n=0; pobjs=""
  for f in $PORT_R15T; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; pobjs="$pobjs ${T}_port${n}_o2.o"; n=$((n+1)); done
  n=1; objs=""
  for f in $PORT_SUP; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o2.o; objs="$objs ${T}_sup${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/voice/voice_tone.main.c ${T}_stub_env.o $pobjs $objs -lm
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
echo "MATCH: the voice tone / MTK / silence / detector port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
