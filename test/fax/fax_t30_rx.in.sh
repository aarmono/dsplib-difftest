#!/bin/sh
# R29 (1 of 4): differential test of the T.30 / HDLC RECEIVE side of dsplibs.o vs src/fax/fax_t30_rx.c (+ the R28 port files the receive code calls: src/fax/fax_t30_tx.c, src/fax/fax_t30_tables.c, the ported FIFO),
# 32-bit, each scenario in its OWN process (scenarios 0 1 of test/fax/fax_t30_rx.main.c, REC build): the DTE input converters _handle_data_input / _handle_hdlc_input(_open / _close), _recieve_silence_state(_init), _hdlc_receive_state_init, _cHDLCrx_init_from_idle (scenario 0 = input, 1 = small functions and the gain threshold table).
# The callees of the blob are WEAKENED with objcopy so the harness' RECORDERS win in the blob run (FAXVMI_create / _delete / _process / _control / _status / _message, FPM_TONE_generate, FPM_rms);
# the blob-LOCAL _idle_state / _send_silence_state / _answer_tone_state / _recieve_silence_state / cDATAtx_counter (the write-only counter `temp.0` of _handle_data_input sits 4 bytes behind it) /
# init_vmi_data_tx_modem / init_vmi_data_rx_modem / GAIN_THRESHOLD_TABLE are made global for the blob run.  The R28 functions the receive code calls (cTOOLS_*, _put_silence, _idle_state_init,
# _handle_data_output) are the real ones on both sides.  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a
# harness assertion fails, either fails this script.
# Env: PORT_R29 (src/fax/fax_t30_rx.c: the port file under test; mutation testing), PORT_DEAD (src/cid/cid_unused.c: the three dead caller-ID helpers), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX, R29_SEED (other input stream),
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r29_in writes test/golden/fax/fax_t30_rx.in.blob32.out.gz).
# Usage: sh test/fax/fax_t30_rx.in.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R29="${PORT_R29:-src/fax/fax_t30_rx.c}"
PORT_DEAD="${PORT_DEAD:-src/cid/cid_unused.c}"
PORT_SUP="$PORT_DEAD src/fax/fax_t30_tx.c src/fax/fax_t30_tables.c src/fax/fax_fifo_sgd.c src/fax/fax_common_tables.c"
T="${TPREFIX:-test/.build/fax/fax_t30_rx.in}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter"
SCENARIOS="${SCENARIOS:-0 1}"
WEAK=""
for f in FAXVMI_create FAXVMI_delete FAXVMI_process FAXVMI_control FAXVMI_status FAXVMI_message FPM_TONE_generate FPM_rms; do WEAK="$WEAK --weaken-symbol=$f"; done
GLOB="--globalize-symbol=_idle_state --globalize-symbol=_send_silence_state --globalize-symbol=_answer_tone_state --globalize-symbol=_recieve_silence_state --globalize-symbol=cDATAtx_counter --globalize-symbol=init_vmi_data_tx_modem --globalize-symbol=init_vmi_data_rx_modem --globalize-symbol=GAIN_THRESHOLD_TABLE"

gcc $CFLAGS -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbg -Dsysdep_sprintf=stub_env_unused_sprintf -c test/stub_env.c -o ${T}_stub_env.o
objcopy $WEAK $GLOB $BLOB_O ${T}_dsplibs_weak.o

gcc $HCF -DTEST_BLOB -no-pie -Isrc -o ${T}_test_blob test/fax/fax_t30_rx.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

gcc $CFLAGS $PORT_XFLAGS -Isrc -c $PORT_R29 -o ${T}_port0_o0.o
n=1; sup0=""
for f in $PORT_SUP; do gcc -m32 -w -O0 -g $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o0.o; sup0="$sup0 ${T}_sup${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/fax/fax_t30_rx.main.c ${T}_stub_env.o ${T}_port0_o0.o $sup0
if [ -z "$NOO2" ]; then
  gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $PORT_R29 -o ${T}_port0_o2.o
  n=1; sup2=""
  for f in $PORT_SUP; do gcc -m32 -w -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o2.o; sup2="$sup2 ${T}_sup${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/fax/fax_t30_rx.main.c ${T}_stub_env.o ${T}_port0_o2.o $sup2
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
echo "MATCH: the DTE input converters and the small receive functions (-O0 and -O2) are behaviorally identical to dsplibs.o for this test"
