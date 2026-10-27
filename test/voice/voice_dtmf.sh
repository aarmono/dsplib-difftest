#!/bin/sh
# R16: differential test of the voice DTMF detector and the beep / DTMF generator (create_dtmf, dtmf_detect, dtmf_test, dtmf_progress, beepgen_create / delete / start_beep /
# start_dtmf / sample): the real dsplibs.o vs src/voice/voice_dtmf.c + voice_dtmf_tables.c, 32-bit, each scenario in its OWN process (test/voice/voice_dtmf.main.c documents the scenarios 0 .. 4,
# test/voice/r16_common.h the callee recorders sysdep_malloc / free / memset, dsplibs_debug_printf).  The blob runs unmodified (all its own callees: notch, GetGain, pow, cadence_*);
# modem_get_param (stub_env's abort stub is renamed) is the harness' parameter table.  The port build links R15's voice_tone.c / voice_tone_tables.c for the callees notch /
# voice_getgain (validated by test/voice/voice_tone.sh) and gets abort stubs for R15's cadence_* callees from the harness.  The port is built at -O0 AND -O2 and BOTH are diffed
# against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a harness assertion fails, either fails this script.
# Env: PORT_R16T (port file(s) under test; mutation testing; PORT_SUP = the other port files the harness links), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1,
#      TPREFIX, R16_SEED (other input stream), R16_VERBOSE=1, R16_SCALE=n (divide every counted loop; debugging), R16_SWEEP=n (divide only the big random sweeps; the mutation runs use 8).
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r16 writes test/golden/voice/voice_dtmf.blob32.out.gz).
# Usage: sh test/voice/voice_dtmf.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R16T="${PORT_R16T:-src/voice/voice_dtmf.c}"
PORT_SUP="${PORT_SUP:-src/voice/voice_dtmf_tables.c src/voice/voice_tone.c src/voice/voice_tone_tables.c}"
T="${TPREFIX:-test/.build/voice/voice_dtmf}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"
SCENARIOS="${SCENARIOS:-0 1 2 3 4}"

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_printf -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -Dmodem_get_param=stub_env_unused_mgp -c test/stub_env.c -o ${T}_stub_env.o

gcc $HCF -DTEST_BLOB -no-pie -Isrc -o ${T}_test_blob test/voice/voice_dtmf.main.c ${T}_stub_env.o $BLOB_O -lm

n=0; pobjs=""
for f in $PORT_R16T; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; pobjs="$pobjs ${T}_port${n}_o0.o"; n=$((n+1)); done
n=1; objs=""
for f in $PORT_SUP; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o0.o; objs="$objs ${T}_sup${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/voice/voice_dtmf.main.c ${T}_stub_env.o $pobjs $objs -lm
if [ -z "$NOO2" ]; then
  n=0; pobjs=""
  for f in $PORT_R16T; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; pobjs="$pobjs ${T}_port${n}_o2.o"; n=$((n+1)); done
  n=1; objs=""
  for f in $PORT_SUP; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o2.o; objs="$objs ${T}_sup${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/voice/voice_dtmf.main.c ${T}_stub_env.o $pobjs $objs -lm
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
echo "MATCH: the voice DTMF / beep port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
