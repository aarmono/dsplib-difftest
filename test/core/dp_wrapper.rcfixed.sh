#!/bin/sh
# R02 (1b of 5): the dp_wrapper composed with the REAL RcFixed: dp_wrapper_create / run / delete over the blob's own RcFixed_* (blob side) vs src/core/dp_wrapper.c over
# src/core/rcfixed_resampler.c + src/core/rcfixed_resampler_tables.c (port side), 32-bit, scenarios 2 and 3 of test/core/dp_wrapper.main.main.c (the six rate pairs with odd chunk sizes; poked ring states
# and asymmetric converters), through test/core/dp_wrapper.rcfixed.main.c (= the same source with R02_REAL_RC: no fake RcFixed).  The harness still owns sysdep_malloc / free /
# memset / memcpy (so the RcFixed state allocations are logged, sizes mapped to i386), the engine callback and dsplibs_debug_printf; only dp_param_get is weakened in the blob.
# The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a harness assertion fails.
# Env: PORT_R02W, PORT_R02RC (port files under test; mutation testing), PORT_XFLAGS, SCENARIOS, NOO2=1, TPREFIX, R02_SEED,
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r02_wrapper_rc writes test/golden/core/dp_wrapper.rcfixed.blob32.out.gz).
# Usage: sh test/core/dp_wrapper.rcfixed.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R02W="${PORT_R02W:-src/core/dp_wrapper.c}"
PORT_R02RC="${PORT_R02RC:-src/core/rcfixed_resampler.c}"
T="${TPREFIX:-test/.build/core/dp_wrapper.rcfixed}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"   # the harness (a header library with helpers some mains do not use)
SCENARIOS="${SCENARIOS:-2 3}"

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_printf -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -Dsysdep_memcpy=stub_env_unused_memcpy -Dmodem_dp_deregister=stub_env_unused_mdd -Dmodem_dp_register=stub_env_unused_mdr -Dmodem_get_bits=stub_env_unused_mgb -Dmodem_get_param=stub_env_unused_mgp -Dmodem_put_bits=stub_env_unused_mpb -Dmodem_set_param=stub_env_unused_msp -c test/stub_env.c -o ${T}_stub_env.o

objcopy --weaken-symbol=dp_param_get $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF -no-pie -Isrc -o ${T}_test_blob test/core/dp_wrapper.rcfixed.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

gcc $CFLAGS $PORT_XFLAGS -Isrc -c $PORT_R02W -o ${T}_port0_o0.o
gcc $CFLAGS $PORT_XFLAGS -Isrc -c $PORT_R02RC -o ${T}_port1_o0.o
gcc $CFLAGS $PORT_XFLAGS -Isrc -c src/core/rcfixed_resampler_tables.c -o ${T}_port2_o0.o
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/core/dp_wrapper.rcfixed.main.c ${T}_stub_env.o ${T}_port0_o0.o ${T}_port1_o0.o ${T}_port2_o0.o
if [ -z "$NOO2" ]; then
  gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $PORT_R02W -o ${T}_port0_o2.o
  gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $PORT_R02RC -o ${T}_port1_o2.o
  gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c src/core/rcfixed_resampler_tables.c -o ${T}_port2_o2.o
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/core/dp_wrapper.rcfixed.main.c ${T}_stub_env.o ${T}_port0_o2.o ${T}_port1_o2.o ${T}_port2_o2.o
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
echo "MATCH: dp_wrapper over the real RcFixed: port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
