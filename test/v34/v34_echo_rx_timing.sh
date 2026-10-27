#!/bin/sh
# Builds both sides of the V.34 "tier 5b" echo / RX sample path differential test - updateAlpha, adaptecho,
# modem_serrint, V34EchoHistoryBackwardClean, rxreadqueue, txwritequeue, rxtiming, rxtiminginit -
# (real dsplibs.o vs src/v34/v34_echo_rx_timing.c) as 32-bit binaries, runs each scenario in its OWN process, and diffs
# the output. A scenario exits 2 if its own coverage assertions fail, 3 if a harness assumption (echo header
# layout, independent model, tail-slot model, ...) is violated; either fails this script - "MATCH" is only
# printed if every scenario both matched AND reached its branches.
#
# sysdep_memset (event hash) and dsplibs_debug_printf (level flip) live in the test main; test/stub_env.c is
# built with both renamed away. The blob's LOCAL regparm V34demodulate (called by rxtiming) needs no anchor:
# rxtiming reaches it itself; the port's rxtiming calls the verified tier-2 port.
#
# The port under test is built at -O0 AND -O2 and BOTH are diffed against the blob.
# PORT_T5B (env) overrides the port source under test (mutation testing); SCENARIOS (env) selects scenarios;
# NOO2=1 skips the -O2 port build; TPREFIX (env) overrides the binary/output prefix (parallel mutation runs).
#
# Scenarios: 0 updateAlpha, 1 rxreadqueue/txwritequeue, 2 BackwardClean (all n classes, all write-pointer
# positions), 3 adaptecho random, 4 adaptecho chain, 5 modem_serrint random, 6 modem_serrint chain,
# 7 rxtiming random + chain, 8 rxtiminginit (+ rxtiming), 9 debug prints / level flips,
# 10 BackwardClean with the caller's shape (cap = 0, n = 3k), 11 / 12 / 13 = scenarios 2 / 5 / 7 restricted to inputs inside the arrays
# they walk (T07: layout independent, see NOTES.md); scenario id + 100 / + 200 = the same scenario with the fake
# object mmap()ed so that the 2 GiB boundary falls inside the tx history ring / the RX ring (unsigned ring-end compares).
#
# Usage: sh test/v34/v34_echo_rx_timing.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_T5B="${PORT_T5B:-src/v34/v34_echo_rx_timing.c}"
T="${TPREFIX:-test/.build/v34/v34_echo_rx_timing}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Isrc -Wall -Wextra -O0 -g"
OTHER="src/v34/v34_agc_demod.c src/v34/v34_dsp_primitives.c src/v34/v34_timing_hilbert_tables.c src/fpm/fpm_tables.c"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7 8 9 10 101 201 102 202 110 210 103 203 105 205 11 12 13}"

gcc $CFLAGS -c -Dsysdep_memset=stub_env_unused_memset -Ddsplibs_debug_printf=stub_env_unused_debug_printf test/stub_env.c \
    -o ${T}_stub_env.o

gcc $CFLAGS -DTEST_BLOB -o ${T}_test_blob test/v34/v34_echo_rx_timing.main.c ${T}_stub_env.o $BLOB_O

gcc $CFLAGS -c $PORT_T5B -o ${T}_port_o0.o
gcc $CFLAGS -o ${T}_test_port test/v34/v34_echo_rx_timing.main.c ${T}_stub_env.o ${T}_port_o0.o $OTHER
if [ -z "$NOO2" ]; then
  gcc -m32 -Isrc -Wall -Wextra -O2 -c $PORT_T5B -o ${T}_port_o2.o
  gcc -m32 -Isrc -Wall -Wextra -O2 -o ${T}_test_port_o2 test/v34/v34_echo_rx_timing.main.c ${T}_stub_env.o ${T}_port_o2.o $OTHER
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
echo "MATCH: V.34 tier-5b echo/RX sample path (updateAlpha/adaptecho/modem_serrint/V34EchoHistoryBackwardClean/rxreadqueue/txwritequeue/rxtiming/rxtiminginit) port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')) is behaviorally identical to dsplibs.o for this test"
