#!/bin/sh
# R03 (3 of 3): end-to-end differential test of the V.32 dp glue over the REAL V.32 engine, dp_wrapper and RcFixed: the real dsplibs.o (32-bit) vs src/v32/dp_v32.c + src/v32/v32_engine.c (+ its engine files) +
# src/core/dp_wrapper.c + src/core/rcfixed_resampler.c, each scenario in its OWN process (test/v32/dp_v32.e2e.main.c documents the scenarios: two glue instances over 9600 / 8000 Hz).  The harness owns the callees
# (sysdep_*, modem_*, modem_debug_log_data, dp_param_get, dsplibs_debug_printf); test/stub_env.c is built with the symbols the harness defines renamed, and the blob's dp_param_get is made WEAK.
# The port is built at -O0 AND -O2 and BOTH are diffed against the blob.  A scenario exits 2 when a coverage counter is zero and 3 on a harness violation.
# Env: SCENARIOS, NOO2=1, TPREFIX, PORT_XFLAGS, R02_SEED.  GOLDEN: GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r03_e2e writes test/golden/v32/dp_v32.e2e.blob32.out.gz.
# Usage: sh test/v32/dp_v32.e2e.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

T="${TPREFIX:-test/.build/v32/dp_v32.e2e}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"
SCENARIOS="${SCENARIOS:-0 1 2 3}"
PORTF="src/v32/dp_v32.c src/core/dp_wrapper.c src/core/rcfixed_resampler.c src/core/rcfixed_resampler_tables.c src/v32/v32_engine.c src/v32/v32_datapath.c src/v32/v32_scrambler.c src/v32/v32_pulse_shaping.c src/v32/v32_timing.c src/v32/v32_echo_canceller.c src/v32/v32_equalizer.c src/v32/v32_math.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c src/fpm/fpm_mrf.c src/fpm/fpm_agc.c src/fpm/fpm_math.c src/fpm/fpm_tables.c src/fpm/fpm_tone.c src/fpm/fpm_mtd.c src/fpm/fpm_iir.c"

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_printf -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -Dsysdep_memcpy=stub_env_unused_memcpy -Dmodem_debug_log_data=stub_env_unused_mdld -Dmodem_dp_deregister=stub_env_unused_mdd -Dmodem_dp_register=stub_env_unused_mdr -Dmodem_get_bits=stub_env_unused_mgb -Dmodem_get_param=stub_env_unused_mgp -Dmodem_put_bits=stub_env_unused_mpb -Dmodem_set_param=stub_env_unused_msp -c test/stub_env.c -o ${T}_stub_env.o

objcopy --weaken-symbol=dp_param_get $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF -no-pie -Isrc -o ${T}_test_blob test/v32/dp_v32.e2e.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

build_port() { # level out
  objs=""
  for f in $PORTF; do
    o=${T}_$(basename $f .c)_$2.o
    gcc -m32 -Wall -Wextra -Wno-unused-parameter $1 -g $PORT_XFLAGS -Isrc -c $f -o $o 2>/dev/null
    objs="$objs $o"
  done
  gcc $HCF -no-pie -Isrc -o ${T}_test_$2 test/v32/dp_v32.e2e.main.c ${T}_stub_env.o $objs
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
echo "MATCH: the V.32 dp glue over the real V.32 engine / dp_wrapper / RcFixed (two instances, 9600 / 8000 Hz) port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
