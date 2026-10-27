#!/bin/sh
# Builds both sides of the V.34 "tier 6a" differential test - modulatevector - (real dsplibs.o vs
# src/v34/v34_modulator.c) as 32-bit binaries, runs each scenario in its OWN process, and diffs the output.
# A scenario exits 2 if its own coverage assertions fail, 3 if a harness assumption (table content,
# independent shell-mapping / tail model, steering) is violated; either fails this script - "MATCH" is
# only printed if every scenario both matched AND reached its branches.
#
# In the blob binary modulatevector's callees getFrame / txmit / V34nlencoder (and the real scramblers,
# initV34 and preinitV34 of scenario 5) are the BLOB'S OWN; in the port binaries they are the already
# verified ports (src/v34/v34_tx_symbol.c, src/v34/v34_dsp_primitives.c, src/v34/v34_init.c, src/v34/v34_setup.c ...).
# The blob binary links the port's extracted tables (tier 3 + tier 6) under a `port_` prefix so the harness
# can content-check the blob's own quarter / smIndex / gInvertPat / xyz at start-up.
#
# The port under test is built at -O0 AND -O2 and BOTH are diffed against the blob.
# PORT_T6A (env) overrides the port source under test (mutation testing; the harness's oracle copy is
# always the pristine src/v34/v34_modulator.c); SCENARIOS (env) selects scenarios; NOO2=1 skips the -O2 build;
# TPREFIX (env) overrides the binary / output prefix (parallel mutation runs).
#
# Scenarios: 0 random single calls (recorder, v steered), 1 random single calls (real scrambleGPC/GPA),
# 2 chains (recorder, GPC, GPA), 3 shell-index boundary sweep (+ exhaustive m = 2), 4 tail / k / e4c block,
# 5 real preinitV34 + initV34 blocks, 6 zero-divisor probe (SIGFPE in a forked child), 7 legal V.34
# rate/baud grid (largest quarter[] index of every legal combination is <= 415), 8 = scenario 4 with k inside the
# 8-point block (T07: layout independent, see NOTES.md).
#
# Usage: sh test/v34/v34_modulator.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_T6A="${PORT_T6A:-src/v34/v34_modulator.c}"
T="${TPREFIX:-test/.build/v34/v34_modulator}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Isrc -Wall -Wextra -O0 -g"
OTHER="src/v34/v34_tx_symbol.c src/v34/v34_modulator_tables.c src/v34/v34_setup.c src/v34/v34_init.c src/v34/v34_tx_level.c src/v34/v34_agc_demod.c \
    src/v34/v34_dsp_primitives.c src/v34/v34_timing_hilbert_tables.c src/v34/v34_init_tables.c src/v34/v34_frame_tables.c src/v34/v34_symbol_tables.c \
    src/fpm/fpm_tables.c"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7 8}"

gcc $CFLAGS -c src/v34/v34_init_tables.c -o ${T}_t3tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t3tables_prefixed.o
gcc $CFLAGS -c src/v34/v34_modulator_tables.c -o ${T}_t6tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t6tables_prefixed.o

gcc $CFLAGS -DTEST_BLOB -o ${T}_test_blob test/v34/v34_modulator.main.c test/stub_env.c \
    ${T}_t3tables_prefixed.o ${T}_t6tables_prefixed.o $BLOB_O

gcc $CFLAGS -c $PORT_T6A -o ${T}_port_o0.o
gcc $CFLAGS -o ${T}_test_port test/v34/v34_modulator.main.c test/stub_env.c ${T}_port_o0.o $OTHER
if [ -z "$NOO2" ]; then
  gcc -m32 -Isrc -Wall -Wextra -O2 -c $PORT_T6A -o ${T}_port_o2.o
  gcc -m32 -Isrc -Wall -Wextra -O2 -o ${T}_test_port_o2 test/v34/v34_modulator.main.c test/stub_env.c ${T}_port_o2.o $OTHER
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
echo "MATCH: V.34 tier-6a modulatevector port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')) is behaviorally identical to dsplibs.o for this test"
