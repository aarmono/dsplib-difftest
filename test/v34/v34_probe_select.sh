#!/bin/sh
# Builds both sides of the V.34 "tier 8a" differential test - probeselect, chkForceBaudRate - (real dsplibs.o vs
# src/v34/v34_probe_select.c) as 32-bit binaries, runs each scenario in its OWN process, and diffs the output.
# A scenario exits 2 if its own coverage assertions fail, 3 if a harness assumption (table content, independent
# model disagreement, unreachable-print assumption) is violated; either fails this script - "MATCH" is only
# printed if every scenario both matched AND reached its branches.
#
# In the blob binary probeselect / chkForceBaudRate / bitreverse are the BLOB'S OWN; in the port binaries they are
# the port (src/v34/v34_probe_select.c) and the verified bitreverse (src/v34/v34_agc_demod.c).  The blob binary links the port's
# extracted tier-5 tables under a `port_` prefix so the harness can content-check the blob's own scale*/c1600..c2000
# tables (whose addresses probeselect stores into the object) at start-up.  dsplibs_debug_printf lives in the test
# main (hashes every formatted line, level-flip hook), so stub_env.c's own is renamed away.
#
# The port under test is built at -O0 AND -O2 and BOTH are diffed against the blob.  PORT_T8A (env) overrides the
# port source under test (mutation testing); SCENARIOS (env) selects scenarios; NOO2=1 skips the -O2 build;
# TPREFIX (env) overrides the binary / output prefix (parallel mutation runs).
#
# Scenarios: 0 random single calls (level 0), 1 random at debug levels, 2 debug-level flips (cached-vs-fresh level
# reads), 3 chkForceBaudRate direct, 4 cascade boundary sweep, 5 pre-emphasis steering, 6 SNR / gain exhaustive grid
# at level 2, 7 sensitive-RX / power-reduction classes, 8 repeated calls on a persistent object.
#
# Usage: sh test/v34/v34_probe_select.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_T8A="${PORT_T8A:-src/v34/v34_probe_select.c}"
T="${TPREFIX:-test/.build/v34/v34_probe_select}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Isrc -Wall -Wextra -Wno-unused-function -O0 -g"
# the harness itself is compiled -O2 (speed; it is not under test); only the PORT object differs between the two port binaries
HCF="-m32 -Isrc -Wall -Wextra -Wno-unused-function -O2 -g"
OTHER="src/v34/v34_agc_demod.c src/v34/v34_dsp_primitives.c src/v34/v34_timing_hilbert_tables.c src/v34/v34_symbol_tables.c src/v8/v8_tables.c src/fpm/fpm_tables.c"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7 8}"

gcc $CFLAGS -Ddsplibs_debug_printf=stub_env_unused_debug_printf -c test/stub_env.c -o ${T}_stub_env.o
gcc $CFLAGS -c src/v34/v34_symbol_tables.c -o ${T}_t5tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t5tables_prefixed.o
gcc $CFLAGS -c src/v8/v8_tables.c -o ${T}_v8tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_v8tables_prefixed.o

gcc $HCF -DTEST_BLOB -Dv8_costbl=port_v8_costbl -o ${T}_test_blob test/v34/v34_probe_select.main.c ${T}_stub_env.o \
    ${T}_t5tables_prefixed.o ${T}_v8tables_prefixed.o $BLOB_O

gcc $CFLAGS -c $PORT_T8A -o ${T}_port_o0.o
gcc $HCF -o ${T}_test_port test/v34/v34_probe_select.main.c ${T}_stub_env.o ${T}_port_o0.o $OTHER
if [ -z "$NOO2" ]; then
  gcc -m32 -Isrc -Wall -Wextra -Wno-unused-function -O2 -c $PORT_T8A -o ${T}_port_o2.o
  gcc $HCF -o ${T}_test_port_o2 test/v34/v34_probe_select.main.c ${T}_stub_env.o ${T}_port_o2.o $OTHER
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
echo "MATCH: V.34 tier-8a probeselect/chkForceBaudRate port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')) is behaviorally identical to dsplibs.o for this test"
