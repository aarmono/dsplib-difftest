#!/bin/sh
# R30 (3 of 3, id r30_eng): differential test of the whole fax class 1 stack over the REAL FAXVMI layer and the REAL fax modems: the real dsplibs.o (its own FAX_* entry points, class 1 layer,
# T.30 / HDLC states, FAXVMI, V.21 / V.27ter / V.29 / V.17 engines, RcFixed converters) vs src/fax/fax_api.c + src/fax/fax_class1.c + src/fax/fax_t30_tx.c + src/fax/fax_t30_rx.c + the ported engines, 32-bit,
# each scenario in its OWN process (scenarios 6 7 8 9 of test/fax/fax_api.main.c, ENG build via test/fax/fax_api.eng.main.c): 6 HDLC sessions (two FAX objects wired back to back: AT+FTH / FRH style
# commands, DLE stuffed frames, every rate / role / line impairment), 7 data sessions (every modem value, FTM / FRM, FTS / FRS), 8 life cycles (objects deleted in the middle of a session,
# random command sequences incl. rejected ones), 9 malloc failure probes of the whole chain (FAX_create / command / process / FAX_delete with the k-th allocation failing in a forked child: the child statuses go to a side file
# (R30_PROBE_FILE) compared in ONE direction: the port must survive wherever the blob survives).  Nothing of the blob is replaced (the harness only provides the 23 symbols dsplibs.o imports: sysdep_*, modem_*, the debug printer).  The
# port is built at -O0 AND -O2 and BOTH are diffed against the blob.
# Env: PORT_R30 (the port files under test), PORT_XFLAGS, SCENARIOS, NOO2=1, TPREFIX, R30_SEED; GOLDEN: GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r30_eng.
# Usage: sh test/fax/fax_api.eng.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_R30="${PORT_R30:-src/fax/fax_api.c src/fax/fax_class1.c src/fax/fax_class1_tables.c}"
PORT_SUP="src/fax/fax_t30_rx.c src/fax/fax_t30_tx.c src/fax/fax_t30_tables.c src/fax/fax_vmi.c src/fax/fax_v21.c src/fax/fax_v27_tx.c src/fax/fax_v27_rx.c src/fax/fax_v29_tx.c src/fax/fax_v29_rx.c src/fax/fax_v17_tx.c src/fax/fax_v17_rx.c src/fax/fax_equalizer.c src/fax/fax_timing.c src/fax/fax_v17_decision.c src/fax/fax_v17_viterbi.c src/fax/fax_pulse_shaping.c src/fax/fax_v21_tables.c src/fax/fax_v27_tx_tables.c src/fax/fax_v27_rx_tables.c src/fax/fax_v29_tx_tables.c src/fax/fax_v29_rx_tables.c src/fax/fax_v17_tx_tables.c src/fax/fax_v17_rx_tables.c src/fax/fax_v17_decision_tables.c src/fax/fax_rx_common_tables.c src/v32/v32_datapath.c src/fax/fax_common_tables.c src/fax/fax_fifo_sgd.c src/fax/fax_scrambler.c src/fax/fax_modem_wrappers.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c src/v32/v32_math.c src/fpm/fpm_math.c src/fpm/fpm_iir.c src/fpm/fpm_tone.c src/fpm/fpm_mrf.c src/fpm/fpm_agc.c src/fpm/fpm_fsm.c src/fpm/fpm_fsd.c src/fpm/fpm_mtd.c src/fpm/fpm_tables.c src/fsk/fsk_tables.c src/core/rcfixed_resampler.c src/core/rcfixed_resampler_tables.c"
T="${TPREFIX:-test/.build/fax/fax_api.eng}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
CFLAGS="-m32 -Wall -Wextra -O0 -g"
HCF="$CFLAGS -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter"
SCENARIOS="${SCENARIOS:-6 7 8 9}"

gcc $CFLAGS -Ddsplibs_debug_level=stub_env_unused_level -Ddsplibs_debug_printf=stub_env_unused_dbg -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Dsysdep_memset=stub_env_unused_memset -Dsysdep_memcpy=stub_env_unused_memcpy -Dsysdep_sprintf=stub_env_unused_sprintf -Dmodem_get_sreg=stub_env_unused_mgs -Dmodem_recv_from_tty=stub_env_unused_mrt -Dmodem_send_to_tty=stub_env_unused_mst -c test/stub_env.c -o ${T}_stub_env.o

gcc $HCF -DTEST_BLOB -no-pie -Isrc -o ${T}_test_blob test/fax/fax_api.eng.main.c ${T}_stub_env.o $BLOB_O -lm

n=1; objs0=""
for f in $PORT_R30 $PORT_SUP; do gcc -m32 -w -O0 -g $PORT_XFLAGS -Isrc -c $f -o ${T}_p${n}_o0.o; objs0="$objs0 ${T}_p${n}_o0.o"; n=$((n+1)); done
gcc $HCF -no-pie -Isrc -o ${T}_test_port test/fax/fax_api.eng.main.c ${T}_stub_env.o $objs0 -lm
if [ -z "$NOO2" ]; then
  n=1; objs2=""
  for f in $PORT_R30 $PORT_SUP; do gcc -m32 -w -O2 $PORT_XFLAGS -Isrc -c $f -o ${T}_p${n}_o2.o; objs2="$objs2 ${T}_p${n}_o2.o"; n=$((n+1)); done
  gcc $HCF -no-pie -Isrc -o ${T}_test_port_o2 test/fax/fax_api.eng.main.c ${T}_stub_env.o $objs2 -lm
fi

# scenarios are independent processes: run them concurrently, then concatenate in order
rm -f ${T}_s*.out ${T}_probe_blob ${T}_probe_port ${T}_probe_o2
pids=""
for i in $SCENARIOS; do
  ( R30_PROBE_FILE=${T}_probe_blob ${T_RUN}${T}_test_blob "$i" > ${T}_s$i.blob.out ) & pids="$pids $!"
  ( R30_PROBE_FILE=${T}_probe_port ${T_RUN}${T}_test_port "$i" > ${T}_s$i.port.out ) & pids="$pids $!"
  if [ -z "$NOO2" ]; then ( R30_PROBE_FILE=${T}_probe_o2 ${T_RUN}${T}_test_port_o2 "$i" > ${T}_s$i.o2.out ) & pids="$pids $!"; fi
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
# the malloc failure probes: one direction (a port child that fails where the blob child survived is an error; the port surviving more than the blob is by design)
if [ -f ${T}_probe_blob ]; then
  for v in port o2; do
    [ -f ${T}_probe_$v ] || continue
    if ! paste -d' ' ${T}_probe_blob ${T}_probe_$v | awk '{ if ($1 != $5 || $2 != $6 || $3 != $7) { print "probe lists differ: " $0; bad = 1 } else if ($4 == 0 && $8 != 0) { print "PORT CRASHES WHERE THE BLOB SURVIVES: rate " $1 " cmd " $2 " k " $3; bad = 1 } else { n++; if ($4 != 0) bc++; if ($8 != 0) pc++ } } END { printf "malloc failure probes: %d children, blob crashed %d, port crashed %d\n", n, bc, pc; exit bad }'; then echo "PROBE MISMATCH ($v)"; exit 1; fi
  done
fi
echo "MATCH: two FAX objects over the real class 1 layer, FAXVMI and fax modems (HDLC sessions, data sessions of every modem value, life cycles; -O0 and -O2) are behaviorally identical to dsplibs.o for this test"
