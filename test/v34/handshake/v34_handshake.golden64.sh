#!/bin/sh
# V04 golden check WITHOUT the blob: builds only the x86-64 port (g++ -m64, -fno-exceptions -fno-rtti: the src/v34/handshake/v34_hs_*.cpp pieces listed in PORT_V04; every callee is a harness recorder, the dispatch is the
# harness's iso_port) and the harness (gcc -m64), runs the scenarios and diffs the concatenated output against the golden produced by the 32-bit run against the real dsplibs.o (test/v34/handshake/v34_handshake.sh,
# GOLDEN_UPDATE=1): test/golden/v34/handshake/v34_handshake.blob32.out.  FP64_FLAGS="-O0", "-O3" ... select the build flags (default -O2; the V04 flag set is -O0, -O2, -O2 -mfpmath=387 and one -O3; NOT -O1 / -Os / 32-bit
# -O3 / 32-bit -O2 -mfpmath=sse).  No dsplibs.o, no 32-bit toolchain needed.
# Usage: sh test/v34/handshake/v34_handshake.golden64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../../.."
T="${TPREFIX:-test/.build/v34/handshake/v34_handshake.g64}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
PORT_V04="${PORT_V04:-src/v34/handshake/v34_hs_tx_small.cpp src/v34/handshake/v34_hs_tx_mid.cpp src/v34/handshake/v34_hs_tx_exmit_j.cpp src/v34/handshake/v34_hs_rx_dpsk_small.cpp src/v34/handshake/v34_hs_post_loop.cpp src/v34/handshake/v34_hs_tx_silence.cpp src/v34/handshake/v34_hs_rx_infodone_moh.cpp src/v34/handshake/v34_hs_tx_dpsk.cpp src/v34/handshake/v34_hs_tx_xmitmp.cpp src/v34/handshake/v34_hs_rx_moh_tone.cpp src/v34/handshake/v34_hs_tx_trnseg4.cpp src/v34/handshake/v34_hs_rx_answer.cpp src/v34/handshake/v34_hs_tx_phase1_call.cpp src/v34/handshake/v34_hs_tx_l1.cpp src/v34/handshake/v34_hs_rx_phase1_call.cpp src/v34/handshake/v34_hs_rx_phase2_call.cpp src/v34/handshake/v34_hs_tx_phase1_answer.cpp src/v34/handshake/v34_hs_rx_det_ab.cpp src/v34/handshake/v34_hs_rx_det_sync.cpp src/v34/handshake/v34_hs_rx_det_info.cpp src/v34/handshake/v34_hs_tx_trnseg4a.cpp src/v34/handshake/v34_hs_rx_l1.cpp src/v34/handshake/v34_hs_rx_receive.cpp src/v34/handshake/v34_hs_rx_dpsk_dft.cpp src/v34/handshake/v34_hs_getbit_bulkdelay.cpp src/v34/handshake/v34_hs_dispatch.cpp}"
OPT="-O2"
case "$FP64_FLAGS" in *-O*) OPT="";; esac
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc $OPT $FP64_FLAGS"
OBJS=""
for f in $PORT_V04; do b=$(basename $f .cpp); g++ -m64 $CXXF -c $f -o ${T}_$b.o; OBJS="$OBJS ${T}_$b.o"; done
gcc -m64 -Isrc -O2 -fno-stack-protector -c src/v34/v34_symbol_tables.c -o ${T}_tab5.o
gcc -m64 -Isrc -O2 -fno-stack-protector -c src/v34/v34_init_tables.c -o ${T}_tab3.o
gcc -m64 -Isrc -O2 -fno-stack-protector -c src/v34/v34_receiver_tables.c -o ${T}_tab7.o
gcc -m64 -Isrc -O2 -fno-stack-protector -c src/v34/handshake/v34_hs_tables.c -o ${T}_tab4.o
gcc -m64 -Isrc -O2 -fno-stack-protector -c src/v34/v34_frame_tables.c -o ${T}_tab4t.o
PROBES=""
for c in v01; do g++ -m64 $CXXF -c test/v34/${c}_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
gcc -m64 -Isrc -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf -c test/stub_env.c -o ${T}_stub.o
gcc -m64 -Isrc -Wall -Wextra -Wno-unused-function -O2 -Dsysdep_malloc=h_sysdep_malloc -Dsysdep_free=h_sysdep_free -c test/harness/harness.c -o ${T}_harness.o
gcc -m64 -Isrc -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -c test/v34/handshake/v34_handshake.main.c -o ${T}_main.o
gcc -m64 -Isrc -O2 -no-pie -o ${T}_test ${T}_main.o ${T}_harness.o ${T}_stub.o $PROBES $OBJS ${T}_tab5.o ${T}_tab3.o ${T}_tab7.o ${T}_tab4.o ${T}_tab4t.o -lm
: > ${T}.out
G64_SC="${G64_SC:-0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22}"   # G64_SC="16 17" runs a subset (the diff then compares only those scenarios' slices of the golden: not used by the full check)
for i in $G64_SC; do ${T_RUN}${T}_test $i >> ${T}.out 2>/dev/null; done
rc=0
if diff -u test/golden/v34/handshake/v34_handshake.blob32.out ${T}.out > ${T}.diff; then echo "GOLDEN64: the x86-64 port ($FP64_FLAGS $OPT) reproduces the 32-bit blob's canonical dump for scenarios 0-22 (test/golden/v34/handshake/v34_handshake.blob32.out)"
else echo "GOLDEN64: FAIL: the dump differs ($(grep -c '^[<>+-][^+-]' ${T}.diff) lines, see ${T}.diff)"; rc=1; fi
rm -f ${T}_*.o ${T}_test ${T}.out
[ $rc = 0 ] && rm -f ${T}.diff
exit $rc
