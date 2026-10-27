#!/bin/sh
# R06 (1 of 3): differential test of the V.22 / V.22bis / Bell 212A dp glue (v22_create / v22_process / v22_delete = blob LOCAL functions reached through the record dp_v22_init registers, dp_v22_init / dp_v22_exit): the real dsplibs.o vs src/v22/dp_v22.c + src/core/dp_wrapper.c, 32-bit, each scenario in its OWN process (test/v22/dp_v22.glue.main.c documents
# the scenarios, test/v22/r06_common.h the callee recorders and the typed objects of src/v22/dp_v22.h).  The LOCAL glue functions are called through the dp_operations record the harness captures from modem_dp_register.
# The harness owns every callee (sysdep_*, modem_*, RcFixed_* (a fake), the engine CreateV23Modem / V23ModemMain / DeleteV23Modem, dsplibs_debug_printf; dp_wrapper_* is the REAL one of each side), so test/stub_env.c is built with the symbols the harness defines renamed, and the blob's own copies of the global callees
# (RcFixed_Create / RcFixed_Delete / RcFixed_Resample / V22FP_create / _delete / _modem / dp_param_get) are made WEAK (objcopy) so the harness' definitions win.  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2 when a
# coverage counter is zero and 3 when a harness / independent-model assertion fails, either fails this script.
# Env: PORT_R06GLUE, PORT_R02W (port file(s) under test; mutation testing), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX, R02_SEED (other input stream),
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r06_glue writes test/golden/v22/dp_v22.glue.blob32.out.gz).
# Usage: sh test/v22/dp_v22.glue.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R06GLUE="${PORT_R06GLUE:-src/v22/dp_v22.c}"
PORT_R02W="${PORT_R02W:-src/core/dp_wrapper.c}"
T="${TPREFIX:-test/.build/v22/dp_v22.glue}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"   # the harness (a header library with helpers some mains do not use)
SCENARIOS="${SCENARIOS:-0 1 2 3}"

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_printf -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -Dsysdep_memcpy=stub_env_unused_memcpy -Dmodem_dp_deregister=stub_env_unused_mdd -Dmodem_dp_register=stub_env_unused_mdr -Dmodem_get_bits=stub_env_unused_mgb -Dmodem_get_param=stub_env_unused_mgp -Dmodem_put_bits=stub_env_unused_mpb -Dmodem_set_param=stub_env_unused_msp -c test/stub_env.c -o ${T}_stub_env.o

objcopy --weaken-symbol=RcFixed_Create --weaken-symbol=RcFixed_Delete --weaken-symbol=RcFixed_Resample --weaken-symbol=V22FP_create --weaken-symbol=V22FP_delete --weaken-symbol=V22FP_modem --weaken-symbol=dp_param_get $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF -no-pie -Isrc -o ${T}_test_blob test/v22/dp_v22.glue.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

gcc $CFLAGS $PORT_XFLAGS -Isrc -c $PORT_R06GLUE -o ${T}_port0_o0.o
gcc $CFLAGS $PORT_XFLAGS -Isrc -c $PORT_R02W -o ${T}_port1_o0.o
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/v22/dp_v22.glue.main.c ${T}_stub_env.o ${T}_port0_o0.o ${T}_port1_o0.o
if [ -z "$NOO2" ]; then
  gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $PORT_R06GLUE -o ${T}_port0_o2.o
  gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $PORT_R02W -o ${T}_port1_o2.o
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/v22/dp_v22.glue.main.c ${T}_stub_env.o ${T}_port0_o2.o ${T}_port1_o2.o
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
echo "MATCH: the V.22 dp glue (v22_create / v22_process / v22_delete = blob LOCAL functions reached through the record dp_v22_init registers, dp_v22_init / dp_v22_exit) port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
