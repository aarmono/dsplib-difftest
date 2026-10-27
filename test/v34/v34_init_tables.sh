#!/bin/sh
# Builds both sides of the V.34 "tier 3, group C" hand-cleaned functions
# differential test (real dsplibs.o vs src/v34/v34_setup.c + src/v34/v34_init.c +
# src/v34/v34_agc_demod.c + src/v34/v34_dsp_primitives.c + tables) as 32-bit binaries, runs each
# scenario in its OWN process (see test/build_and_diff_v32_primitives.sh), and
# diffs the output. A scenario exits 2 if its own coverage assertions fail
# (branch never reached), which fails this script (set -e) - "MATCH" is only
# printed if every scenario both matched AND reached its branches.
#
# v34setuptxmit calls settxlevel (-> GetVPcmMinimalTxPowerReduction -> edprintf), which
# the PORT binary now gets from src/v34/v34_tx_level.c (group D, verified separately by
# test/v34/v34_tx_level.sh); the blob binary runs the real ones. edprintf
# really calls sysdep_vsnprintf / strlen / strcpy / strcat on both sides, provided by
# test/stub_env_str.c (-DSTUB_ENV_REAL_STRING) - those stubs are still needed.
# The BLOB binary additionally links the port's extracted tables under a `port_`
# prefix (objcopy --prefix-symbols) so the harness can content-check every
# derived blob table address against the extraction at start-up.
#
# Scenarios: 0 initV34, 1 V34SetupDemodulator, 2 V34SetupModulator (grid),
# 3 V34SetupModulator (full baud/carrier/preemp sweeps + random), 4 v34setuptxmit,
# 5 debug-level-2 output.
#
# Usage: sh test/v34/v34_init_tables.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/v34/v34_init_tables}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"

CFLAGS="-m32 -Isrc -Wall -Wextra -O0 -g"

# port tables under a port_ prefix (for the blob binary's content check)
gcc $CFLAGS -c src/v34/v34_init_tables.c -o ${T}_tables_prefixed.o
objcopy --prefix-symbols=port_ ${T}_tables_prefixed.o

gcc $CFLAGS -DTEST_BLOB -DSTUB_ENV_REAL_STRING -o ${T}_test_blob \
    test/v34/v34_init_tables.main.c test/stub_env.c test/stub_env_str.c \
    ${T}_tables_prefixed.o $BLOB_O

gcc $CFLAGS -DSTUB_ENV_REAL_STRING -o ${T}_test_port \
    test/v34/v34_init_tables.main.c test/stub_env.c test/stub_env_str.c \
    src/v34/v34_setup.c src/v34/v34_init.c src/v34/v34_agc_demod.c src/v34/v34_dsp_primitives.c \
    src/v34/v34_timing_hilbert_tables.c src/v34/v34_init_tables.c src/fpm/fpm_tables.c src/v34/v34_tx_level.c

SCENARIOS="0 1 2 3 4 5"

: > ${T}_blob.out
: > ${T}_port.out
for i in $SCENARIOS; do
  ${T_RUN}${T}_test_blob "$i" >> ${T}_blob.out
  ${T_RUN}${T}_test_port "$i" >> ${T}_port.out
done

diff -u ${T}_blob.out ${T}_port.out && \
    echo "MATCH: V.34 tier-3 group C port is behaviorally identical to dsplibs.o for this test"
