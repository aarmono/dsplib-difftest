#!/bin/sh
# R06 (3 of 3): end-to-end differential test of the V.22 dp glue over the REAL V.22 engine, dp_wrapper and RcFixed: the real dsplibs.o (32-bit) vs src/v22/dp_v22.c + src/v22/v22_engine.c (+ its engine files) +
# src/core/dp_wrapper.c + src/core/rcfixed_resampler.c, each scenario in its OWN process (test/v22/dp_v22.e2e.main.c documents the scenarios: two glue instances over 8000 / 9600 Hz).  The harness owns the callees
# (sysdep_*, modem_*, dp_param_get, dsplibs_debug_printf) and, as CALLEE RECORDERS, the seven R07 / R08 handlers (v22_data ... v22_retrain: the blob's copies are made WEAK); test/stub_env.c is built with the symbols the harness defines renamed.
# The port is built at -O0 AND -O2 and BOTH are diffed against the blob.  A scenario exits 2 when a coverage counter is zero and 3 on a harness violation.
# Env: SCENARIOS, NOO2=1, TPREFIX, PORT_XFLAGS, R02_SEED.  GOLDEN: GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r06_e2e.
# Usage: sh test/v22/dp_v22.e2e.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

T="${TPREFIX:-test/.build/v22/dp_v22.e2e}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"
SCENARIOS="${SCENARIOS:-0 1 2 3}"
PORTF="src/v22/dp_v22.c src/core/dp_wrapper.c src/core/rcfixed_resampler.c src/core/rcfixed_resampler_tables.c src/v22/v22_engine.c src/v22/v22_equalizer.c src/v22/v22_timing.c src/v22/v22_pulse_shaping.c src/v22/v22_rx_resampler.c src/v22/v22_rx_bandpass.c src/v22/v22_scrambler.c src/v22/v22_tables.c src/fpm/fpm_tone.c src/fpm/fpm_mtd.c src/fpm/fpm_agc.c src/fpm/fpm_math.c src/fpm/fpm_iir.c src/v32/v32_math.c src/fpm/fpm_tables.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c"

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_printf -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -Dsysdep_memcpy=stub_env_unused_memcpy -Dmodem_dp_deregister=stub_env_unused_mdd -Dmodem_dp_register=stub_env_unused_mdr -Dmodem_get_bits=stub_env_unused_mgb -Dmodem_get_param=stub_env_unused_mgp -Dmodem_put_bits=stub_env_unused_mpb -Dmodem_set_param=stub_env_unused_msp -c test/stub_env.c -o ${T}_stub_env.o

objcopy --weaken-symbol=dp_param_get --weaken-symbol=v22_data --weaken-symbol=v22_originate --weaken-symbol=v22_answer --weaken-symbol=v22_local_loop --weaken-symbol=v22_org_rmloop2 --weaken-symbol=v22_ans_rmloop2 --weaken-symbol=v22_retrain $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF -no-pie -Isrc -o ${T}_test_blob test/v22/dp_v22.e2e.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

build_port() { # level out
  objs=""
  for f in $PORTF; do
    o=${T}_$(basename $f .c)_$2.o
    gcc -m32 -Wall -Wextra -Wno-unused-parameter $1 -g $PORT_XFLAGS -Isrc -c $f -o $o 2>/dev/null
    objs="$objs $o"
  done
  gcc $HCF -no-pie -Isrc -o ${T}_test_$2 test/v22/dp_v22.e2e.main.c ${T}_stub_env.o $objs
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
echo "MATCH: the V.22 dp glue over the real V.22 engine / dp_wrapper / RcFixed (two instances, 9600 / 8000 Hz) port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
