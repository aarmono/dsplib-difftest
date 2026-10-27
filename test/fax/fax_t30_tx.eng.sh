#!/bin/sh
# R28 (4 of 4): differential test of the T.30 / HDLC transmit states over the REAL FAXVMI layer and the REAL fax modems: the real dsplibs.o (its own FAXVMI, V.21 / V.27ter / V.29 / V.17 engines,
# FIFO, init_vmi_data_* functions) vs src/fax/fax_t30_tx.c + src/fax/fax_vmi.c + the ported engines, 32-bit, each scenario in its OWN process (scenarios 5 6 7 of test/fax/fax_t30_tx.main.c, ENG build via
# test/fax/fax_t30_tx.eng.main.c): 5 HDLC sessions (silence, preamble with DLE stuffed frames through the shim of R29's _handle_hdlc_input, send buffer, between buffers, receive channel off state),
# 6 data sessions (every modem value: scrambled ones, data, nulls, over the real modems and FIFO), 7 _init_transmitter / _delete_data_tx_modem with the real init_vmi_data_tx_modem functions.
# R29's _handle_data_input / _handle_hdlc_input / _handle_hdlc_input_open of the blob are WEAKENED so the harness SHIMS (transcribed from the blob's machine code) win on both sides; the blob-LOCAL
# _idle_state / _send_silence_state / _answer_tone_state / cDATAtx_counter / init_vmi_data_tx_modem are made global for the blob run.  The port is built at -O0 AND -O2 and BOTH are diffed against the blob.
# Env: PORT_R28 (src/fax/fax_t30_tx.c), PORT_XFLAGS, SCENARIOS, NOO2=1, TPREFIX, R28_SEED; GOLDEN: GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r28_eng.
# Usage: sh test/fax/fax_t30_tx.eng.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R28="${PORT_R28:-src/fax/fax_t30_tx.c}"
PORT_SUP="src/fax/fax_t30_tables.c src/fax/fax_vmi.c src/fax/fax_v21.c src/fax/fax_v27_tx.c src/fax/fax_v27_rx.c src/fax/fax_v29_tx.c src/fax/fax_v29_rx.c src/fax/fax_v17_tx.c src/fax/fax_v17_rx.c src/fax/fax_equalizer.c src/fax/fax_timing.c src/fax/fax_v17_decision.c src/fax/fax_v17_viterbi.c src/fax/fax_pulse_shaping.c src/fax/fax_v21_tables.c src/fax/fax_v27_tx_tables.c src/fax/fax_v27_rx_tables.c src/fax/fax_v29_tx_tables.c src/fax/fax_v29_rx_tables.c src/fax/fax_v17_tx_tables.c src/fax/fax_v17_rx_tables.c src/fax/fax_v17_decision_tables.c src/fax/fax_rx_common_tables.c src/v32/v32_datapath.c src/fax/fax_common_tables.c src/fax/fax_fifo_sgd.c src/fax/fax_scrambler.c src/fax/fax_modem_wrappers.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c src/v32/v32_math.c src/fpm/fpm_math.c src/fpm/fpm_iir.c src/fpm/fpm_tone.c src/fpm/fpm_mrf.c src/fpm/fpm_agc.c src/fpm/fpm_fsm.c src/fpm/fpm_fsd.c src/fpm/fpm_mtd.c src/fpm/fpm_tables.c src/fsk/fsk_tables.c"
T="${TPREFIX:-test/.build/fax/fax_t30_tx.eng}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter"
SCENARIOS="${SCENARIOS:-5 6 7}"
WEAK="--weaken-symbol=_handle_data_input --weaken-symbol=_handle_hdlc_input --weaken-symbol=_handle_hdlc_input_open"
GLOB="--globalize-symbol=_idle_state --globalize-symbol=_send_silence_state --globalize-symbol=_answer_tone_state --globalize-symbol=cDATAtx_counter --globalize-symbol=init_vmi_data_tx_modem --globalize-symbol=init_vmi_data_rx_modem"

gcc $CFLAGS -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbg -Dsysdep_sprintf=stub_env_unused_sprintf -c test/stub_env.c -o ${T}_stub_env.o
objcopy $WEAK $GLOB $BLOB_O ${T}_dsplibs_glob.o

gcc $HCF -DTEST_BLOB -no-pie -Isrc -o ${T}_test_blob test/fax/fax_t30_tx.eng.main.c ${T}_stub_env.o ${T}_dsplibs_glob.o

gcc $CFLAGS $PORT_XFLAGS -Isrc -c $PORT_R28 -o ${T}_port0_o0.o
n=1; objs=""
for f in $PORT_SUP; do gcc -m32 -w -O0 -g $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o0.o; objs="$objs ${T}_sup${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/fax/fax_t30_tx.eng.main.c ${T}_stub_env.o ${T}_port0_o0.o $objs
if [ -z "$NOO2" ]; then
  gcc -m32 -Wall -Wextra -O2 $PORT_XFLAGS -Isrc -c $PORT_R28 -o ${T}_port0_o2.o
  n=1; objs=""
  for f in $PORT_SUP; do gcc -m32 -w -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_sup${n}_o2.o; objs="$objs ${T}_sup${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/fax/fax_t30_tx.eng.main.c ${T}_stub_env.o ${T}_port0_o2.o $objs
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
echo "MATCH: the T.30 / HDLC transmit states over the real FAXVMI layer and the real fax modems (HDLC sessions, data sessions of every modem value, init / delete life cycle; -O0 and -O2) are behaviorally identical to dsplibs.o for this test"
