#!/bin/sh
# R30 (1 of 3, id r30_class1): differential test of the fax class 1 layer, fax_class1_create / fax_class1_command / fax_class1_progress / fax_class1_delete and the state table
# class1_state_functions, with EVERY callee replaced by scripted recorders (FAXVMI_create / _delete, FPM_TONE_create / _delete, FPM_iir_filt_II, FIFO_delete, the 15 global state functions,
# _cHDLCrx_init_from_idle, _hdlc_receive_state_init, _rx_look_carrier_init, _delete_data_rx_modem / _delete_data_tx_modem, _tx_scrambled_ones_init, cHDLCtx_preamble_state_init: WEAKENED in a
# copy of dsplibs.o with objcopy so the recorders win in the blob run): the real dsplibs.o vs src/fax/fax_class1.c (+ src/fax/fax_class1_tables.c), 32-bit, each scenario in its OWN process
# (scenarios 0 .. 3 of test/fax/fax_api.main.c, build via test/fax/fax_class1.main.c).  The blob-LOCAL state functions (_idle_state ...) are made global for the blob run (the harness compares
# the table slots with them and then replaces the slots by recorders).  The port is built at -O0 AND -O2 and BOTH are diffed against the blob; a scenario exits 2 when a coverage
# counter is zero and 3 when a harness assertion fails, either fails this script.
# Env: PORT_R30 (src/fax/fax_class1.c: the port file under test; mutation testing), PORT_XFLAGS (extra flags of the port builds), SCENARIOS, NOO2=1, TPREFIX, R30_SEED (other input stream),
#      GOLDEN: see test/lib/tier_golden64.sh (GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r30_class1 writes test/golden/fax/fax_class1.blob32.out.gz).
# Usage: sh test/fax/fax_class1.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R30="${PORT_R30:-src/fax/fax_class1.c}"
PORT_SUP="src/fax/fax_class1_tables.c"
T="${TPREFIX:-test/.build/fax/fax_class1}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter"
SCENARIOS="${SCENARIOS:-0 1 2 3}"
WEAK=""
for f in FAXVMI_create FAXVMI_delete FPM_TONE_create FPM_TONE_delete FPM_iir_filt_II FIFO_delete _t30_silence_before_tx_state _t30_preabmle_state _send_hdlc_buffer_state _send_hdlc_between_buffer_state _hdlc_receive_look_carrier_state _hdlc_receive_state _hdlc_receive_between_buffers_state _hdlc_emulate_receive_state _tx_scrambled_ones_state _tx_data_state _tx_nulls_state _rx_look_carrier_state _rx_data_state cHDLCtx_off _tx_silence_before_scrm_ones _cHDLCrx_init_from_idle _hdlc_receive_state_init _rx_look_carrier_init _delete_data_rx_modem _delete_data_tx_modem _tx_scrambled_ones_init cHDLCtx_preamble_state_init; do WEAK="$WEAK --weaken-symbol=$f"; done
GLOB="--globalize-symbol=_idle_state --globalize-symbol=_send_silence_state --globalize-symbol=_answer_tone_state --globalize-symbol=_recieve_silence_state"

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_dbg -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -Dsysdep_memcpy=stub_env_unused_memcpy -Dsysdep_sprintf=stub_env_unused_sprintf -Dmodem_get_sreg=stub_env_unused_mgs -Dmodem_recv_from_tty=stub_env_unused_mrt -Dmodem_send_to_tty=stub_env_unused_mst -c test/stub_env.c -o ${T}_stub_env.o
objcopy $WEAK $GLOB $BLOB_O ${T}_dsplibs_weak.o

gcc $HCF -DTEST_BLOB -no-pie -Isrc -o ${T}_test_blob test/fax/fax_class1.main.c ${T}_stub_env.o ${T}_dsplibs_weak.o

n=0; pobjs=""
for f in $PORT_R30 $PORT_SUP; do gcc $CFLAGS $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o0.o; pobjs="$pobjs ${T}_port${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/fax/fax_class1.main.c ${T}_stub_env.o $pobjs
if [ -z "$NOO2" ]; then
  n=0; pobjs=""
  for f in $PORT_R30 $PORT_SUP; do gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_port${n}_o2.o; pobjs="$pobjs ${T}_port${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/fax/fax_class1.main.c ${T}_stub_env.o $pobjs
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
echo "MATCH: the fax class 1 layer (fax_class1_create / command / progress / delete over scripted callees, -O0 and -O2) is behaviorally identical to dsplibs.o for this test"
