#!/bin/sh
# Builds both sides of the V.34 "tier 9a" differential test - v34handshakinit, VPcmV34SetMohMessageBits - (real
# dsplibs.o vs src/vpcm/v34_handshake_init.c) as 32-bit binaries, runs each scenario in its OWN process, and diffs the output.
# A scenario exits 2 if its own coverage assertions fail, 3 if a harness assumption (independent-model disagreement,
# table content, state-index oracle) is violated; either fails this script - "MATCH" is only printed if every scenario
# both matched AND reached its branches.
#
# The five callees of v34handshakinit (v34modeminit, rxtiminginit, preinitdigital, detectorinit, V34SetINFO0dBits) are
# RECORDERS defined in the test main in BOTH binaries: the blob binary links a copy of dsplibs.o in which those five
# definitions are WEAK (objcopy --weaken-symbol), so the recorders win for every reference.  VPcmV34SetMohMessageBits
# is the blob's own / the port's own.  (Scenario 8 = INTEGRATED, see below, is built by the second half of this script
# with the REAL callees on both sides.)
#
# The port under test is built at -O0 AND -O2 and BOTH are diffed against the blob.  PORT_T9A (env) overrides the port
# source under test (mutation testing); SCENARIOS (env) selects scenarios; NOO2=1 skips the -O2 build; NOINTEG=1 skips the integrated scenario; TPREFIX (env)
# overrides the binary / output prefix (parallel mutation runs).
#
# Scenarios: 0 random single calls (level 0/1), 1 random at debug levels, 2 hook (level flip / reader-pointer poke after
# the k-th print), 3 VPcmV34SetMohMessageBits direct, 4 wild state words at level <= 1, 5 timer-field boundary sweep,
# 6 arm-1 flag grid, 7 persistent objects, 8 = scenario 2 with the poke away from the pointer slots (T07); then the INTEGRATED scenario (real callees, NOINTEG=1 skips it).
#
# Usage: sh test/vpcm/v34_handshake_init.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_T9A="${PORT_T9A:-src/vpcm/v34_handshake_init.c}"
T="${TPREFIX:-test/.build/vpcm/v34_handshake_init}"
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -Wno-unused-function -O0 -g -Isrc"
HCF="-m32 -Wall -Wextra -Wno-unused-function -O2 -g -Isrc"
TABLES="src/v34/v34_init_tables.c src/v34/v34_frame_tables.c"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7 8}"
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"
# the integrated scenario is skipped for NOINTEG=1 and when SCENARIOS is blank (build-only use by the mutation / coverage tools)
INTEG=1; [ -n "$NOINTEG" ] && INTEG=""; [ -z "$(echo $SCENARIOS)" ] && INTEG=""

gcc $CFLAGS -Ddsplibs_debug_printf=stub_env_unused_debug_printf -c test/stub_env.c -o ${T}_stub_env.o
# blob side: tables under a port_ prefix (the blob has its own c1200_/c2400_; StateName is LOCAL there), callees weakened
gcc $CFLAGS -c src/v34/v34_init_tables.c -o ${T}_t3tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t3tables_prefixed.o
gcc $CFLAGS -c src/v34/v34_frame_tables.c -o ${T}_t4tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_t4tables_prefixed.o
objcopy --weaken-symbol=v34modeminit --weaken-symbol=rxtiminginit --weaken-symbol=preinitdigital \
        --weaken-symbol=detectorinit --weaken-symbol=V34SetINFO0dBits $BLOB_O ${T}_dsplibs_weak.o

gcc $HCF -DTEST_BLOB -o ${T}_test_blob test/vpcm/v34_handshake_init.main.c ${T}_stub_env.o \
    ${T}_t3tables_prefixed.o ${T}_t4tables_prefixed.o ${T}_dsplibs_weak.o

gcc $CFLAGS -c $PORT_T9A -o ${T}_port_o0.o
gcc $HCF -o ${T}_test_port test/vpcm/v34_handshake_init.main.c ${T}_stub_env.o ${T}_port_o0.o $TABLES
if [ -z "$NOO2" ]; then
  gcc -m32 -Wall -Wextra -Wno-unused-function -O2 -Isrc -c $PORT_T9A -o ${T}_port_o2.o
  gcc $HCF -o ${T}_test_port_o2 test/vpcm/v34_handshake_init.main.c ${T}_stub_env.o ${T}_port_o2.o $TABLES
fi

# scenarios are independent processes: run them concurrently, then concatenate in order
rm -f ${T}_s*.out
pids=""
for i in $SCENARIOS; do
  ( ${R}${T}_test_blob "$i" > ${T}_s$i.blob.out ) & pids="$pids $!"
  ( ${R}${T}_test_port "$i" > ${T}_s$i.port.out ) & pids="$pids $!"
  if [ -z "$NOO2" ]; then ( ${R}${T}_test_port_o2 "$i" > ${T}_s$i.o2.out ) & pids="$pids $!"; fi
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

# ---- INTEGRATED scenario: real callees on both sides (tier-4b fixture pulled in as a library, see the .c header) ----
# The blob binary uses the UNMODIFIED dsplibs.o; the port binaries link the port of v34handshakinit plus the verified ports
# of its callees (v34modeminit tier4b, rxtiminginit tier5b, preinitdigital/detectorinit tier3ab, V34SetINFO0dBits tier2).
if [ -n "$INTEG" ]; then
  I=${T}_integ
  OTHERI="src/vpcm/v34_modem_init.c src/v34/v34_echo_rx_timing.c src/v34/v34_setup.c src/v34/v34_init.c src/v34/v34_tx_level.c src/v34/v34_agc_demod.c src/v34/v34_dsp_primitives.c \
      src/v34/v34_timing_hilbert_tables.c src/v34/v34_init_tables.c src/v34/v34_frame_tables.c src/v34/v34_symbol_tables.c src/v8/v8_tables.c src/fpm/fpm_tables.c"
  gcc $CFLAGS -c -DSTUB_ENV_REAL_STRING -Ddsplibs_debug_printf=stub_env_unused_debug_printf test/stub_env.c -o ${I}_stub_env.o
  gcc $CFLAGS -Itest -DTEST_BLOB -o ${I}_blob_bin test/vpcm/v34_handshake_init.integ.main.c ${I}_stub_env.o \
      ${T}_t3tables_prefixed.o ${T}_t4tables_prefixed.o $BLOB_O
  gcc $CFLAGS -Itest -o ${I}_port_bin test/vpcm/v34_handshake_init.integ.main.c ${I}_stub_env.o $PORT_T9A $OTHERI
  if [ -z "$NOO2" ]; then
    gcc -m32 -Wall -Wextra -Wno-unused-function -O2 -Itest -Isrc -o ${I}_port_o2_bin test/vpcm/v34_handshake_init.integ.main.c ${I}_stub_env.o $PORT_T9A $OTHERI
  fi
  ( ${R}${I}_blob_bin 0 > ${I}_blob.out ) & ip1=$!
  ( ${R}${I}_port_bin 0 > ${I}_port.out ) & ip2=$!
  if [ -z "$NOO2" ]; then ( ${R}${I}_port_o2_bin 0 > ${I}_port_o2.out ) & ip3=$!; fi
  irc=0
  wait $ip1 || irc=1
  wait $ip2 || irc=1
  if [ -z "$NOO2" ]; then wait $ip3 || irc=1; fi
  if [ $irc -ne 0 ]; then echo "INTEGRATED SCENARIO FAILURE (non-zero exit: coverage, harness assertion or crash)"; exit 1; fi
  diff -u ${I}_blob.out ${I}_port.out
  if [ -z "$NOO2" ]; then diff -u ${I}_blob.out ${I}_port_o2.out; fi
fi
echo "MATCH: V.34 tier-9a v34handshakinit/VPcmV34SetMohMessageBits port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')) is behaviorally identical to dsplibs.o for this test"
