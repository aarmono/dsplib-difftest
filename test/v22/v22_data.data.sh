#!/bin/sh
# R08: differential test of the V.22 / V.22bis / Bell 212A the DATA state (v22_data): data transfer at 2400 / 1200 / Bell212, V22FP_control status bits, data-mode engines, carrier loss / carrier-back / signal-quality / remote retrain decisions: the real dsplibs.o vs src/v22/v22_data.c + src/v22/v22_loopback.c + src/v22/v22_retrain.c, 32-bit, each
# scenario in its OWN process (test/v22/v22_data.main.c documents the scenarios and the three ids, test/v22/r06_common.h and test/v22/r06_eng_common.h the recorders).  REAL on each side: the whole V.22 engine
# (src/v22/v22_engine.c is the port on the port side: V22FP_modem dispatches to the code under test), the call setup of R07 (src/v22/v22_call_setup.c on the port side), FPM_TONE / MTD / AGC / SDM / SMC, V22_PPS / MRF / SRE /
# FSE / IIR, Detect_v22 / Detect_1s / Detect_Retrain / Detect_Rmloop2_ACK, DemodDataV22 ...; sysdep_* / dsplibs_debug_printf are the harness'.  NO handler is a recorder: all seven V22_PROTOCOL entries
# are real (R08_REAL_HANDLERS / R07_REAL_CALLSETUP).  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a harness / model
# assertion fails, either fails this script.
# Env: PORT_R08 (port file(s) under test; mutation testing), PORT_XFLAGS, SCENARIOS, NOO2=1, TPREFIX, R02_SEED, GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r08_data).
# Usage: sh test/v22/v22_data.data.sh   (run from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R08_DATA="${PORT_R08_DATA:-src/v22/v22_data.c}"
PORT_R08_LOOPS="${PORT_R08_LOOPS:-src/v22/v22_loopback.c}"
PORT_R08_RETRAIN="${PORT_R08_RETRAIN:-src/v22/v22_retrain.c}"
PORT_R08="$PORT_R08_DATA $PORT_R08_LOOPS $PORT_R08_RETRAIN"
PORT_SUP="src/v22/v22_engine.c src/v22/v22_call_setup.c src/v22/v22_detectors.c src/v22/v22_equalizer.c src/v22/v22_timing.c src/v22/v22_pulse_shaping.c src/v22/v22_rx_resampler.c src/v22/v22_rx_bandpass.c src/v22/v22_scrambler.c src/v22/v22_tables.c src/fpm/fpm_tone.c src/fpm/fpm_mtd.c src/fpm/fpm_agc.c src/fpm/fpm_math.c src/fpm/fpm_iir.c src/v32/v32_math.c src/fpm/fpm_tables.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c"
T="${TPREFIX:-test/.build/v22/v22_data.data}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"   # the harness (a header library with helpers some mains do not use)
SCENARIOS="${SCENARIOS:-0 1 5}"

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_printf -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -Dsysdep_memcpy=stub_env_unused_memcpy -Dmodem_dp_deregister=stub_env_unused_mdd -Dmodem_dp_register=stub_env_unused_mdr -Dmodem_get_bits=stub_env_unused_mgb -Dmodem_get_param=stub_env_unused_mgp -Dmodem_put_bits=stub_env_unused_mpb -Dmodem_set_param=stub_env_unused_msp -c test/stub_env.c -o ${T}_stub_env.o

objcopy --weaken-symbol=RcFixed_Create --weaken-symbol=RcFixed_Delete --weaken-symbol=RcFixed_Resample --weaken-symbol=dp_param_get $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF -no-pie -Isrc -o ${T}_test_blob test/v22/v22_data.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

p8=""; for f in $PORT_R08; do b8=$(basename $f .c); gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_port0_${b8}_o0.o; p8="$p8 ${T}_port0_${b8}_o0.o"; done
n=1; objs=""
for f in $PORT_SUP; do gcc -m32 -w -O0 -g $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; objs="$objs ${T}_port${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/v22/v22_data.main.c ${T}_stub_env.o $p8 $objs
if [ -z "$NOO2" ]; then
  p8o=""; for f in $PORT_R08; do b8=$(basename $f .c); gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port0_${b8}_o2.o; p8o="$p8o ${T}_port0_${b8}_o2.o"; done
  n=1; objs=""
  for f in $PORT_SUP; do gcc -m32 -w -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; objs="$objs ${T}_port${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/v22/v22_data.main.c ${T}_stub_env.o $p8o $objs
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
echo "MATCH: the V.22 R08 handlers (r08_data: src/v22/v22_data.c, v22_loopback.c, v22_retrain.c) port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
