#!/bin/sh
# R17 (3 of 3, id r17_e2e): END-TO-END differential test of the voice API slmodemd links, VOICE_create / VOICE_process / VOICE_command / VOICE_delete, with nothing replaced below it: the
# blob run is the unmodified dsplibs.o (its own engine, FDSP, FIFO, BeepGen, DTMF / tone / silence detectors, cadences, RcFixed), the port run links src/voice/voice_engine.c + voice_api.c with the
# ported voice_fdsp.c / voice_fifo8.c (R14), voice_tone.c (R15), voice_dtmf.c (R16), tone_cadence.c / tone_iir.c (R09) and rcfixed_resampler.c (R01); only the symbols dsplibs.o imports from slmodemd are the harness'
# (modem_get_param, modem_recv_from_tty / send_to_tty, sysdep_*, dsplibs_debug_*).  32-bit, each scenario in its OWN process (test/voice/voice_engine.e2e.main.c documents the scenarios 0 .. 4,
# test/voice/r17_common.h the recorders).  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a harness
# assertion fails, either fails this script.
# Env: PORT_R17E (port files under test; mutation testing; PORT_SUP = the other port files the harness links), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1,
#      TPREFIX, R17_SEED (other input stream), R17_VERBOSE=1,
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r17_e2e writes test/golden/voice/voice_engine.e2e.blob32.out.gz).
# Usage: sh test/voice/voice_engine.e2e.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R17E="${PORT_R17E:-src/voice/voice_engine.c src/voice/voice_api.c}"
PORT_SUP="${PORT_SUP:-src/voice/voice_fdsp.c src/voice/voice_fifo8.c src/voice/voice_tone.c src/voice/voice_tone_tables.c src/voice/voice_dtmf.c src/voice/voice_dtmf_tables.c src/tone/tone_cadence.c src/tone/tone_iir.c src/tone/tone_tables.c src/tone/tone_table_words.c src/core/rcfixed_resampler.c src/core/rcfixed_resampler_tables.c}"
T="${TPREFIX:-test/.build/voice/voice_engine.e2e}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"
SCENARIOS="${SCENARIOS:-0 1 2 3 4}"

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_printf -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -Dsysdep_memcpy=stub_env_unused_memcpy -Dmodem_get_param=stub_env_unused_mgp -Dmodem_recv_from_tty=stub_env_unused_mrt -Dmodem_send_to_tty=stub_env_unused_mst -c test/stub_env.c -o ${T}_stub_env.o

gcc $HCF -DTEST_BLOB -no-pie -Isrc -o ${T}_test_blob test/voice/voice_engine.e2e.main.c ${T}_stub_env.o $BLOB_O -lm

n=0; pobjs=""
for f in $PORT_R17E; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; pobjs="$pobjs ${T}_port${n}_o0.o"; n=$((n+1)); done
n=1; objs=""
for f in $PORT_SUP; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o0.o; objs="$objs ${T}_sup${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/voice/voice_engine.e2e.main.c ${T}_stub_env.o $pobjs $objs -lm
if [ -z "$NOO2" ]; then
  n=0; pobjs=""
  for f in $PORT_R17E; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; pobjs="$pobjs ${T}_port${n}_o2.o"; n=$((n+1)); done
  n=1; objs=""
  for f in $PORT_SUP; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o2.o; objs="$objs ${T}_sup${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/voice/voice_engine.e2e.main.c ${T}_stub_env.o $pobjs $objs -lm
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
echo "MATCH: the voice API port (-O0 and -O2) is behaviorally identical to dsplibs.o for this end-to-end test"
