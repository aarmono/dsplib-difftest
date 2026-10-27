#!/bin/sh
# R06 (2 of 3): differential test of the V.22 / V.22bis / Bell 212A ENGINE (V22FP_create / _delete / _modem / _control / _GetDiagnostics, V22_status and the helpers SetTxRate, SetRxRate, ScrambleDataV22,
# DescrambleDataV22, ModDataV22, DemodDataV22, ResetRx, SetAdaptEqV22, TxClockSync, MakeTxData, RxTrained1200 / 2400, TxNOP, RxClampV22, ReadGTimer, CarrierDetect, SignalDetect, GetSignalQuality,
# ScramblerOn, DescramblerOn): the real dsplibs.o vs src/v22/v22_engine.c, 32-bit, each scenario in its OWN process (test/v22/v22_engine.main.c documents the scenarios, test/v22/r06_common.h and
# test/v22/r06_eng_common.h the recorders).  REAL callees on each side: FPM_TONE / MTD / AGC / SDM / SMC, V22_PPS / MRF / SRE / FSE / IIR (the blob's own / the ports); sysdep_* are the harness'.  CALLEE
# RECORDERS on both sides: the seven R07 / R08 handlers (v22_data, v22_originate, v22_answer, v22_local_loop, v22_org_rmloop2, v22_ans_rmloop2, v22_retrain): the blob's copies are made WEAK (objcopy)
# so the harness' definitions win.  test/stub_env.c is built with the symbols the harness defines renamed.  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits
# 2 when a coverage counter is zero and 3 when a harness / model assertion fails, either fails this script.
# Env: PORT_R06ENG (port file(s) under test; mutation testing), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX, R02_SEED (other input stream),
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r06_eng writes test/golden/v22/v22_engine.blob32.out.gz).
# Usage: sh test/v22/v22_engine.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R06ENG="${PORT_R06ENG:-src/v22/v22_engine.c}"
PORT_SUP="src/v22/v22_equalizer.c src/v22/v22_timing.c src/v22/v22_pulse_shaping.c src/v22/v22_rx_resampler.c src/v22/v22_rx_bandpass.c src/v22/v22_scrambler.c src/v22/v22_tables.c src/fpm/fpm_tone.c src/fpm/fpm_mtd.c src/fpm/fpm_agc.c src/fpm/fpm_math.c src/fpm/fpm_iir.c src/v32/v32_math.c src/fpm/fpm_tables.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c"
T="${TPREFIX:-test/.build/v22/v22_engine}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable"   # the harness (a header library with helpers some mains do not use)
SCENARIOS="${SCENARIOS:-0 1 2 3}"

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_printf -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -Dsysdep_memcpy=stub_env_unused_memcpy -Dmodem_dp_deregister=stub_env_unused_mdd -Dmodem_dp_register=stub_env_unused_mdr -Dmodem_get_bits=stub_env_unused_mgb -Dmodem_get_param=stub_env_unused_mgp -Dmodem_put_bits=stub_env_unused_mpb -Dmodem_set_param=stub_env_unused_msp -c test/stub_env.c -o ${T}_stub_env.o

objcopy --weaken-symbol=RcFixed_Create --weaken-symbol=RcFixed_Delete --weaken-symbol=RcFixed_Resample --weaken-symbol=dp_param_get --weaken-symbol=v22_data --weaken-symbol=v22_originate --weaken-symbol=v22_answer --weaken-symbol=v22_local_loop --weaken-symbol=v22_org_rmloop2 --weaken-symbol=v22_ans_rmloop2 --weaken-symbol=v22_retrain $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF -no-pie -Isrc -o ${T}_test_blob test/v22/v22_engine.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

gcc $CFLAGS $PORT_XFLAGS -Isrc -c $PORT_R06ENG -o ${T}_port0_o0.o
n=1; objs=""
for f in $PORT_SUP; do gcc -m32 -w -O0 -g $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; objs="$objs ${T}_port${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/v22/v22_engine.main.c ${T}_stub_env.o ${T}_port0_o0.o $objs
if [ -z "$NOO2" ]; then
  gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $PORT_R06ENG -o ${T}_port0_o2.o
  n=1; objs=""
  for f in $PORT_SUP; do gcc -m32 -w -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; objs="$objs ${T}_port${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/v22/v22_engine.main.c ${T}_stub_env.o ${T}_port0_o2.o $objs
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
echo "MATCH: the V.22 engine (V22FP_* and the helpers of src/v22/v22_engine.c) port (-O0 and -O2) is behaviorally identical to dsplibs.o for this test"
