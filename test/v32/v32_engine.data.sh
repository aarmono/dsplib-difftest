#!/bin/sh
# R03 (2 of 3): differential test of the V.32 protocol-dispatch targets v32_data / v32_handshake / v32_null_protocol (blob LOCAL functions, reached through the
# blob's GLOBAL V32FP_modem and its V32_PROTOCOL table): the real dsplibs.o vs src/v32/v32_engine.c (+ the V.32 engine files), 32-bit, REAL engine on both sides, each scenario in its OWN
# process (test/v32/v32_engine.data.main.c documents the scenarios).  The port is built at -O0 AND -O2 (the engine files at the same level); both are diffed against the blob.  A scenario
# exits 2 when a coverage counter is zero and 3 on a harness violation; either fails this script.
# Env: PORT_R03V32 (the engine file under test; mutation testing), SCENARIOS, NOO2=1, TPREFIX, PORT_XFLAGS.  GOLDEN: GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r03_data writes test/golden/v32/v32_engine.data.blob32.out.gz.
# Usage: sh test/v32/v32_engine.data.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

T="${TPREFIX:-test/.build/v32/v32_engine.data}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24}"
PORT_R03V32="${PORT_R03V32:-src/v32/v32_engine.c}"
PORTF="$PORT_R03V32 src/v32/v32_datapath.c src/v32/v32_scrambler.c src/v32/v32_pulse_shaping.c src/v32/v32_timing.c src/v32/v32_echo_canceller.c src/v32/v32_equalizer.c src/v32/v32_math.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c src/fpm/fpm_mrf.c src/fpm/fpm_agc.c src/fpm/fpm_math.c src/fpm/fpm_tables.c src/fpm/fpm_tone.c src/fpm/fpm_mtd.c src/fpm/fpm_iir.c"

gcc $CFLAGS -Isrc -o ${T}_test_blob test/v32/v32_engine.data.main.c test/stub_env.c $BLOB_O

build_port() { # level out
  objs=""
  for f in $PORTF; do
    o=${T}_$(basename $f .c)_$2.o
    gcc -m32 -Wall -Wextra $1 -g $PORT_XFLAGS -Isrc -c $f -o $o
    objs="$objs $o"
  done
  gcc $CFLAGS -Isrc -o ${T}_test_$2 test/v32/v32_engine.data.main.c test/stub_env.c $objs
}
build_port -O0 port
if [ -z "$NOO2" ]; then build_port -O2 port_o2; fi

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

diff -u ${T}_blob.out ${T}_port.out | head -60
diff -q ${T}_blob.out ${T}_port.out > /dev/null
if [ -z "$NOO2" ]; then diff -u ${T}_blob.out ${T}_port_o2.out | head -60; diff -q ${T}_blob.out ${T}_port_o2.out > /dev/null; fi
echo "MATCH: the V.32 protocol dispatch targets v32_data / v32_handshake / v32_null_protocol (through V32FP_modem; real engine) port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
