#!/bin/sh
# R28 (2 of 4): differential test of the HDLC transmit states of dsplibs.o vs src/fax/fax_t30_tx.c, 32-bit, scenario 2 (single calls of every state with scripted FAXVMI / R29 recorders, and dispatched sequences).
# scenario 0 = T30frames + ctools (GetT30FrameIDFromBuffer over all 0xff,b1,b2, GetT30FrameNameByID, _put_silence, _handle_data_output, cTOOLS_handle_hdlc_output with the debug lines,
# cTOOLS_handle_data_output_reset); scenario 1 = the shared states and small initialisers (_idle_state(_init), _send_silence_state(_init), _answer_tone_state, cHDLCtx_preamble_state_init,
# cHDLCtx_off_init, _send_hdlc_between_buffer_state_init, _init_tx_nulls_state) -- scenarios 0 1 of test/fax/fax_t30_tx.main.c (REC build).
# The callees of the blob are WEAKENED with objcopy so the harness' RECORDERS win in the blob run (FAXVMI_*, FPM_TONE_generate, R29's _handle_data_input / _handle_hdlc_input /
# _handle_hdlc_input_open); the blob-LOCAL _idle_state / _send_silence_state / _answer_tone_state / cDATAtx_counter / init_vmi_data_tx_modem are made global for the blob run.  The port run
# links src/fax/fax_t30_tx.c, src/fax/fax_t30_tables.c and the ported FIFO (src/fax/fax_fifo_sgd.c, src/fax/fax_common_tables.c) with the recorders.  test/stub_env.c is built with the symbols the harness defines
# renamed.  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2 when a coverage counter is zero and 3 when a harness assertion fails, either fails
# this script.
# Env: PORT_R28 (src/fax/fax_t30_tx.c: the port file under test; mutation testing), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX, R28_SEED (other input stream),
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r28_hdlc writes test/golden/fax/fax_t30_tx.hdlc.blob32.out.gz).
# Usage: sh test/fax/fax_t30_tx.hdlc.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R28="${PORT_R28:-src/fax/fax_t30_tx.c}"
PORT_SUP="src/fax/fax_t30_tables.c src/fax/fax_fifo_sgd.c src/fax/fax_common_tables.c"
T="${TPREFIX:-test/.build/fax/fax_t30_tx.hdlc}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter"
SCENARIOS="${SCENARIOS:-2}"
WEAK=""
for f in FAXVMI_create FAXVMI_delete FAXVMI_process FAXVMI_control FAXVMI_status FAXVMI_message FPM_TONE_generate _handle_data_input _handle_hdlc_input _handle_hdlc_input_open; do WEAK="$WEAK --weaken-symbol=$f"; done
GLOB="--globalize-symbol=_idle_state --globalize-symbol=_send_silence_state --globalize-symbol=_answer_tone_state --globalize-symbol=cDATAtx_counter --globalize-symbol=init_vmi_data_tx_modem"

gcc $CFLAGS -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbg -Dsysdep_sprintf=stub_env_unused_sprintf -c test/stub_env.c -o ${T}_stub_env.o
objcopy $WEAK $GLOB $BLOB_O ${T}_dsplibs_weak.o

gcc $HCF -DTEST_BLOB -no-pie -Isrc -o ${T}_test_blob test/fax/fax_t30_tx.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

gcc $CFLAGS $PORT_XFLAGS -Isrc -c $PORT_R28 -o ${T}_port0_o0.o
n=1; sup0=""
for f in $PORT_SUP; do gcc -m32 -w -O0 -g $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o0.o; sup0="$sup0 ${T}_sup${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/fax/fax_t30_tx.main.c ${T}_stub_env.o ${T}_port0_o0.o $sup0
if [ -z "$NOO2" ]; then
  gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $PORT_R28 -o ${T}_port0_o2.o
  n=1; sup2=""
  for f in $PORT_SUP; do gcc -m32 -w -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o2.o; sup2="$sup2 ${T}_sup${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/fax/fax_t30_tx.main.c ${T}_stub_env.o ${T}_port0_o2.o $sup2
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
echo "MATCH: the HDLC transmit states (_t30_silence_before_tx_state, _t30_preabmle_state, _send_hdlc_buffer_state, _send_hdlc_between_buffer_state, cHDLCtx_off; single calls and dispatched sequences; -O0 and -O2) are behaviorally identical to dsplibs.o for this test"
