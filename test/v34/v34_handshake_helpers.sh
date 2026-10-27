#!/bin/sh
# Builds both sides of the V.34 "tier 5d" handshake-helper differential test - ApplyBulkDelay, getbit,
# setfinalrate, initdigital - (real dsplibs.o vs src/v34/v34_handshake_helpers.c) as 32-bit binaries, runs each scenario in
# its OWN process, and diffs the output.  A scenario exits 2 if its own coverage assertions fail, 3 if a
# harness assumption (table content, scale-table gaps, LOCAL-function prologue, independent model) is
# violated; either fails this script - "MATCH" is only printed if every scenario both matched AND reached its
# branches.
#
# ApplyBulkDelay and getbit are LOCAL regparm(3) functions: the blob binary reaches them by address arithmetic
# from the GLOBAL neighbours txwritequeue / setfinalrate (prologue bytes verified at run time) and calls them
# through regparm(3) pointers.  The blob binary also links the port's extracted tier-3/5 tables under a `port_`
# prefix so the harness can content-check the blob tables it registers (and the zero alignment gaps at
# scale[-1] / scale[28]).  initV34 and bitreverse are the REAL ones on both sides (blob's own / verified port).
# sysdep_memset and dsplibs_debug_printf live in the test main (event hash / level flip).
#
# The port under test is built at -O0 AND -O2 and BOTH are diffed against the blob.
# PORT_T5D (env) overrides the port source under test (mutation testing); SCENARIOS (env) selects scenarios;
# NOO2=1 skips the -O2 port build; TPREFIX (env) overrides the binary/output prefix (parallel mutation runs).
#
# Scenarios: 0 ApplyBulkDelay random, 1 ApplyBulkDelay debug/level flips, 2 ApplyBulkDelay chain, 3 getbit random,
# 4 getbit streams (bit-sequence model), 5 getbit chain on wild state, 6 getbit infinite-recursion probe (fork),
# 7 setfinalrate exhaustive + random, 8 setfinalrate debug, 9 initdigital random, 10 initdigital grid,
# 11 setfinalrate + initdigital chain, 12 initdigital debug/level flips.
#
# Usage: sh test/v34/v34_handshake_helpers.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_T5D="${PORT_T5D:-src/v34/v34_handshake_helpers.c}"
T="${TPREFIX:-test/.build/v34/v34_handshake_helpers}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Isrc -Wall -Wextra -O0 -g"
OTHER="src/v34/v34_setup.c src/v34/v34_init.c src/v34/v34_tx_level.c src/v34/v34_agc_demod.c src/v34/v34_dsp_primitives.c \
    src/v34/v34_timing_hilbert_tables.c src/v34/v34_init_tables.c src/v34/v34_frame_tables.c src/v34/v34_symbol_tables.c src/fpm/fpm_tables.c"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7 8 9 10 11 12}"

gcc $CFLAGS -c -Dsysdep_memset=stub_env_unused_memset -Ddsplibs_debug_printf=stub_env_unused_debug_printf test/stub_env.c \
    -o ${T}_stub_env.o
gcc $CFLAGS -c src/v34/v34_init_tables.c -o ${T}_t3tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t3tables_prefixed.o
gcc $CFLAGS -c src/v34/v34_symbol_tables.c -o ${T}_t5tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t5tables_prefixed.o

gcc $CFLAGS -DTEST_BLOB -o ${T}_test_blob test/v34/v34_handshake_helpers.main.c ${T}_stub_env.o \
    ${T}_t3tables_prefixed.o ${T}_t5tables_prefixed.o $BLOB_O

gcc $CFLAGS -c $PORT_T5D -o ${T}_port_o0.o
gcc $CFLAGS -o ${T}_test_port test/v34/v34_handshake_helpers.main.c ${T}_stub_env.o ${T}_port_o0.o $OTHER
if [ -z "$NOO2" ]; then
  gcc -m32 -Isrc -Wall -Wextra -O2 -c $PORT_T5D -o ${T}_port_o2.o
  gcc -m32 -Isrc -Wall -Wextra -O2 -o ${T}_test_port_o2 test/v34/v34_handshake_helpers.main.c ${T}_stub_env.o ${T}_port_o2.o $OTHER
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
echo "MATCH: V.34 tier-5d handshake helpers (ApplyBulkDelay/getbit/setfinalrate/initdigital) port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')) is behaviorally identical to dsplibs.o for this test"
