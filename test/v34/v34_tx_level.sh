#!/bin/sh
# Builds both sides of the V.34 "tier 3, group D" differential test - edprintf,
# GetVPcmMinimalTxPowerReduction, settxlevel - (real dsplibs.o vs
# src/v34/v34_tx_level.c) as 32-bit binaries, runs each scenario in its OWN process
# (so both sides' static edprintf state starts identical), and diffs the output.
# A scenario exits 2 if its own coverage assertions fail, 3 if a harness
# assumption (edprintf's static-buffer layout, offsetarr content) is violated;
# either fails this script (set -e) - "MATCH" is only printed if every scenario
# both matched AND reached its branches.
#
# The sysdep_vsnprintf/strlen/strcpy/strcat stubs live in the test main itself
# (real libc behaviour + event logging + optional chaos), so test/stub_env.c is
# built with -DSTUB_ENV_REAL_STRING (its abort stubs for those four are dropped) and
# with its dsplibs_debug_printf renamed away (the test main defines its own, which can
# also flip the debug level after a print, see scenario 8).
# The blob binary additionally links the port's extracted tables under a `port_`
# prefix so the harness can content-check the blob's LOCAL offsetarr (located via
# the GLOBAL .data anchor _a2u + 0x120) against the port's copy at start-up.
#
# The PORT side is built twice (-O0 and -O2) and BOTH are diffed against the blob:
# edprintf's re-entrancy behaviour (scenario 3) depends on the compiler not
# caching iEncodeOffset / the buffers across the external sysdep_* calls.
#
# Scenarios: 0 edprintf formats x levels (traced), 1 edprintf length/class sweep,
# 2 edprintf long random sequence, 3 edprintf chaos (lying strlen, level flips,
# re-entrant nested edprintf, iEncodeOffset poked to arbitrary values), 4 GetVPcm random, 5 settxlevel scale sweep (all
# 65536 scales), 6 settxlevel exhaustive small-input product, 7 debug output,
# 8 settxlevel level re-read while prints flip the level.
#
# Usage: sh test/v34/v34_tx_level.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/v34/v34_tx_level}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"

CFLAGS="-m32 -Isrc -Wall -Wextra -O0 -g"
PORTSRC="src/v34/v34_tx_level.c src/v34/v34_agc_demod.c src/v34/v34_dsp_primitives.c src/v34/v34_timing_hilbert_tables.c \
    src/v34/v34_init_tables.c src/fpm/fpm_tables.c"

gcc $CFLAGS -c -DSTUB_ENV_REAL_STRING -Ddsplibs_debug_printf=stub_env_unused_debug_printf test/stub_env.c \
    -o ${T}_stub_env.o
gcc $CFLAGS -c src/v34/v34_init_tables.c -o ${T}_tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_tables_prefixed.o

gcc $CFLAGS -DTEST_BLOB -o ${T}_test_blob \
    test/v34/v34_tx_level.main.c ${T}_stub_env.o \
    ${T}_tables_prefixed.o $BLOB_O

gcc $CFLAGS -o ${T}_test_port \
    test/v34/v34_tx_level.main.c ${T}_stub_env.o $PORTSRC

gcc -m32 -Isrc -Wall -Wextra -O2 -o ${T}_test_port_o2 \
    test/v34/v34_tx_level.main.c ${T}_stub_env.o $PORTSRC

SCENARIOS="0 1 2 3 4 5 6 7 8"

# scenarios are independent processes: run them concurrently, then concatenate in order
rm -f ${T}_s*.out
pids=""
for i in $SCENARIOS; do
  ( ${T_RUN}${T}_test_blob "$i" > ${T}_s$i.blob.out ) & pids="$pids $!"
  ( ${T_RUN}${T}_test_port "$i" > ${T}_s$i.port.out ) & pids="$pids $!"
  ( ${T_RUN}${T}_test_port_o2 "$i" > ${T}_s$i.o2.out ) & pids="$pids $!"
done
for p in $pids; do wait $p; done
: > ${T}_blob.out
: > ${T}_port.out
: > ${T}_port_o2.out
for i in $SCENARIOS; do
  cat ${T}_s$i.blob.out >> ${T}_blob.out
  cat ${T}_s$i.port.out >> ${T}_port.out
  cat ${T}_s$i.o2.out >> ${T}_port_o2.out
done
rm -f ${T}_s*.out

diff -u ${T}_blob.out ${T}_port.out
diff -u ${T}_blob.out ${T}_port_o2.out && \
    echo "MATCH: V.34 tier-3 group D port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
