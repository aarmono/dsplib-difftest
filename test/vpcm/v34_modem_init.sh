#!/bin/sh
# Builds both sides of the V.34 "tier 4b" differential test - v34modeminit,
# VPcmV34SetTxScale, cEncodeChar, getMPrecvdBits - (real dsplibs.o vs src/vpcm/v34_modem_init.c)
# as 32-bit binaries, runs each scenario in its OWN process, and diffs the output.
# A scenario exits 2 if its own coverage assertions fail, 3 if a harness assumption
# (table content, edprintf static-buffer layout, getMPrecvdBits prologue) is violated;
# either fails this script - "MATCH" is only printed if every scenario both matched AND
# reached its branches.
#
# getMPrecvdBits is a LOCAL regparm(1) function: the blob binary reaches it by address
# arithmetic from the GLOBAL `bitreverse` (prologue verified at run time). The blob binary
# additionally links the port's extracted tables under a `port_` prefix so the harness can
# content-check every blob table it registers (incl. the LOCAL ones located from the
# GLOBAL anchor hsine1200) against the extraction at start-up.
# The sysdep_vsnprintf/strlen/strcpy/strcat stubs and dsplibs_debug_printf live in the test
# main (so it can learn edprintf's static buffers and flip the debug level after a print);
# test/stub_env.c is built with -DSTUB_ENV_REAL_STRING and its debug printer renamed away.
#
# The port under test is built at -O0 AND -O2 and BOTH are diffed against the blob.
# PORT_T4B (env) overrides the port source under test (mutation testing).
#
# Scenarios: 0 VPcmV34SetTxScale, 1 cEncodeChar, 2 getMPrecvdBits random, 3 getMPrecvdBits
# debug/level-flip, 4 v34modeminit random, 5 v34modeminit debug/level-flip, 6 obj+0x25c2
# twin-run probe, 7 getMPrecvdBits boundary grid.
#
# Usage: sh test/vpcm/v34_modem_init.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_T4B="${PORT_T4B:-src/vpcm/v34_modem_init.c}"
T="${TPREFIX:-test/.build/vpcm/v34_modem_init}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g -Isrc"
OTHER="src/v34/v34_setup.c src/v34/v34_init.c src/v34/v34_tx_level.c src/v34/v34_agc_demod.c src/v34/v34_dsp_primitives.c \
    src/v34/v34_timing_hilbert_tables.c src/v34/v34_init_tables.c src/v34/v34_frame_tables.c src/fpm/fpm_tables.c"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7}"

gcc $CFLAGS -c -DSTUB_ENV_REAL_STRING -Ddsplibs_debug_printf=stub_env_unused_debug_printf test/stub_env.c \
    -o ${T}_stub_env.o
gcc $CFLAGS -c src/v34/v34_init_tables.c -o ${T}_t3tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t3tables_prefixed.o
gcc $CFLAGS -c src/v34/v34_frame_tables.c -o ${T}_t4tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t4tables_prefixed.o

gcc $CFLAGS -DTEST_BLOB -o ${T}_test_blob test/vpcm/v34_modem_init.main.c ${T}_stub_env.o \
    ${T}_t3tables_prefixed.o ${T}_t4tables_prefixed.o $BLOB_O

gcc $CFLAGS -o ${T}_test_port test/vpcm/v34_modem_init.main.c ${T}_stub_env.o $PORT_T4B $OTHER
gcc -m32 -Wall -Wextra -O2 -Isrc -o ${T}_test_port_o2 test/vpcm/v34_modem_init.main.c ${T}_stub_env.o $PORT_T4B $OTHER

# scenarios are independent processes: run them concurrently, then concatenate in order
rm -f ${T}_s*.out
pids=""
for i in $SCENARIOS; do
  ( ${T_RUN}${T}_test_blob "$i" > ${T}_s$i.blob.out ) & pids="$pids $!"
  ( ${T_RUN}${T}_test_port "$i" > ${T}_s$i.port.out ) & pids="$pids $!"
  ( ${T_RUN}${T}_test_port_o2 "$i" > ${T}_s$i.o2.out ) & pids="$pids $!"
done
rc=0
for p in $pids; do wait $p || rc=1; done
: > ${T}_blob.out
: > ${T}_port.out
: > ${T}_port_o2.out
for i in $SCENARIOS; do
  cat ${T}_s$i.blob.out >> ${T}_blob.out
  cat ${T}_s$i.port.out >> ${T}_port.out
  cat ${T}_s$i.o2.out >> ${T}_port_o2.out
done
rm -f ${T}_s*.out
if [ $rc -ne 0 ]; then echo "SCENARIO FAILURE (non-zero exit: coverage, harness assertion or crash)"; exit 1; fi

diff -u ${T}_blob.out ${T}_port.out
diff -u ${T}_blob.out ${T}_port_o2.out && \
    echo "MATCH: V.34 tier-4b (v34modeminit/getMPrecvdBits/cEncodeChar/VPcmV34SetTxScale) port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
