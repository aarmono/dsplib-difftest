#!/bin/sh
# Builds both sides of the V.34 "tier 7a" differential test - receiver - (real dsplibs.o vs
# src/v34/v34_receiver.c) as 32-bit binaries, runs each scenario in its OWN process, and diffs the output.
# A scenario exits 2 if its own coverage assertions fail, 3 if a harness assumption (table content,
# independent phase / retrain / energy model, oracle consistency) is violated; either fails this script -
# "MATCH" is only printed if every scenario both matched AND reached its branches.
#
# In the blob binary every callee of receiver (V34TimingFilter, V34TimingPrefilter, V34Equalizer*, V34scrambler,
# TimingV34, the LOCAL V34demodulate, decoderv34 ...) is the BLOB'S OWN; in the port binaries they are the already
# verified ports (src/v34/v34_dsp_primitives.c, v34_init.c, v34_agc_demod.c, v34_frame_decoder.c ...).  The blob binary links the
# port's extracted tables (tier 3, 4, 7 and the V.8 costbl) under a `port_` prefix so the harness can content-check
# the blob's own vectpp / rxvect4 / costbl / kLookup / grid ... at start-up.
#
# The port under test is built at -O0 AND -O2 and BOTH are diffed against the blob.  PORT_T7A (env) overrides the
# port source under test (mutation testing; the harness's oracle copy is always the pristine src/v34/v34_receiver.c and
# src/v34/v34_frame_decoder.c); SCENARIOS (env) selects scenarios; NOO2=1 skips the -O2 build; TPREFIX (env) overrides the
# binary / output prefix (parallel mutation runs).
#
# Scenarios (9..16 = 0..7 with the TimingV34 sample indices inside S[], T07): 0 single calls (fresh random states), 1 the same at debug level 2, 2 chain: retrain / RRN counter,
# 3 chain: PLL / error window / equaliser adaptation, 4 chain: TRN symbol-count sweep (flags&8 clear and set),
# 5 frame mode (oracle-gated decoderv34 frame path), 6 frame mode at debug level 2, 7 S-S1 / signal-energy classes,
# 8 TimingV34 zero-divisor fork probe (the blob traps in a forked child; the port must trap the same way).
#
# Usage: sh test/v34/v34_receiver.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_T7A="${PORT_T7A:-src/v34/v34_receiver.c}"
T="${TPREFIX:-test/.build/v34/v34_receiver}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Isrc -Wall -Wextra -Wno-unused-function -O0 -g"
OTHER="src/v34/v34_receiver_tables.c src/v34/v34_frame_decoder.c src/v34/v34_frame_tables.c src/v34/v34_init.c src/v34/v34_agc_demod.c \
    src/v34/v34_dsp_primitives.c src/v34/v34_timing_hilbert_tables.c src/v34/v34_init_tables.c src/v34/v34_symbol_tables.c src/v8/v8_tables.c \
    src/fpm/fpm_tables.c"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16}"

for t in src/v34/v34_init_tables.c src/v34/v34_frame_tables.c src/v34/v34_receiver_tables.c src/v8/v8_tables.c; do
  b=$(basename $t .c)
  gcc $CFLAGS -c $t -o ${T}_${b}_prefixed.o
  objcopy --prefix-symbols=port_ ${T}_${b}_prefixed.o
done
PREF="${T}_v34_init_tables_prefixed.o ${T}_v34_frame_tables_prefixed.o ${T}_v34_receiver_tables_prefixed.o ${T}_v8_tables_prefixed.o"

# the harness's oracle copy of receiver (pristine src/v34/v34_receiver.c) needs `costbl` (= v8_costbl) and
# FPM_sqrt_table: in the blob binary those come from the port_-prefixed copies / the blob, in the port binaries from the port
# the harness defines dsplibs_debug_printf itself (counts every print by kind), so stub_env.c's own is renamed away
gcc $CFLAGS -Ddsplibs_debug_printf=stub_env_unused_debug_printf -c test/stub_env.c -o ${T}_stub_env.o

gcc $CFLAGS -DTEST_BLOB -Dv8_costbl=port_v8_costbl -o ${T}_test_blob test/v34/v34_receiver.main.c ${T}_stub_env.o \
    $PREF $BLOB_O

gcc $CFLAGS -c $PORT_T7A -o ${T}_port_o0.o
gcc $CFLAGS -o ${T}_test_port test/v34/v34_receiver.main.c ${T}_stub_env.o ${T}_port_o0.o $OTHER
if [ -z "$NOO2" ]; then
  gcc -m32 -Isrc -Wall -Wextra -O2 -c $PORT_T7A -o ${T}_port_o2.o
  gcc $CFLAGS -o ${T}_test_port_o2 test/v34/v34_receiver.main.c ${T}_stub_env.o ${T}_port_o2.o $OTHER
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
echo "MATCH: V.34 tier-7a receiver port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')) is behaviorally identical to dsplibs.o for this test"
