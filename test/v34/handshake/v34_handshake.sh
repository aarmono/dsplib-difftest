#!/bin/sh
# Builds the differential test of port chunk V04 (v34handshak, ONE 61,541 B function ported in pieces: see porting/decompiled/dp/v04_v34handshak.txt; pieces so far: V04a = the small transmit-state cases,
# src/v34/handshake/v34_hs_tx_small.cpp, extern "C" C++ compiled with -fno-exceptions -fno-rtti) and runs every scenario in its OWN process.  The unit under test is "the blob's per-sample loop run ONCE": the build
# against the blob patches v34handshak's loop back edge at run time (-DV04_BLOB), the port builds dispatch with the same signed 16-bit state arithmetic to the ported case helper.  Every callee (txmit,
# modulatevector, txwritequeue, V34EchoReportCoeff, v90Phase34, k56FlexPhase34) and dsplibs_debug_printf is interposed by a harness recorder:
#   blob      test/v34/handshake/v34_handshake.main.c (-DV04_BLOB) + a copy of dsplibs.o in which the callees are WEAK (gcc -m32 -no-pie)
#   port -O0  the harness + the port objects under test (+ src/v34/v34_symbol_tables.c: vect4, src/v34/v34_init_tables.c: StateName; the callees are the recorders: nothing else is linked)
#   port -O2  same at -O2
#   port -m64 (NO64=1 skips) the same harness and the port compiled for x86-64; its output is diffed against the 32-bit BLOB's output (the golden dump)
# The ONE harness source has no #ifdef around the code under test.  A scenario exits 2 if its own coverage sites fail, 3 if a harness assumption is violated; either fails this script.
# "MATCH" is only printed if every scenario matched AND reached its sites.  Flag set (user decision from V04 on): 32-bit -O0 and -O2, x86-64 -O2 (+ -O0, -O2 -mfpmath=387, one -O3 via v04_golden64.sh);
# no -O1 / -Os / 32-bit -O3 / 32-bit -O2 -mfpmath=sse.
# Env: PORT_V04 (space separated port sources under test, mutation testing), SCENARIOS, NOO2=1, NO64=1, TPREFIX (output prefix), V04_SEED, V04_TRIALS, GOLDEN_UPDATE=1 (rewrite test/golden/v34/handshake/v34_handshake.blob32.out), FP64_FLAGS.
# Usage: sh test/v34/handshake/v34_handshake.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

ALL_V04="src/v34/handshake/v34_hs_tx_small.cpp src/v34/handshake/v34_hs_tx_mid.cpp src/v34/handshake/v34_hs_tx_exmit_j.cpp src/v34/handshake/v34_hs_rx_dpsk_small.cpp src/v34/handshake/v34_hs_post_loop.cpp src/v34/handshake/v34_hs_tx_silence.cpp src/v34/handshake/v34_hs_rx_infodone_moh.cpp src/v34/handshake/v34_hs_tx_dpsk.cpp src/v34/handshake/v34_hs_tx_xmitmp.cpp src/v34/handshake/v34_hs_rx_moh_tone.cpp src/v34/handshake/v34_hs_tx_trnseg4.cpp src/v34/handshake/v34_hs_rx_answer.cpp src/v34/handshake/v34_hs_tx_phase1_call.cpp src/v34/handshake/v34_hs_tx_l1.cpp src/v34/handshake/v34_hs_rx_phase1_call.cpp src/v34/handshake/v34_hs_rx_phase2_call.cpp src/v34/handshake/v34_hs_tx_phase1_answer.cpp src/v34/handshake/v34_hs_rx_det_ab.cpp src/v34/handshake/v34_hs_rx_det_sync.cpp src/v34/handshake/v34_hs_rx_det_info.cpp src/v34/handshake/v34_hs_tx_trnseg4a.cpp src/v34/handshake/v34_hs_rx_l1.cpp src/v34/handshake/v34_hs_rx_receive.cpp src/v34/handshake/v34_hs_rx_dpsk_dft.cpp src/v34/handshake/v34_hs_getbit_bulkdelay.cpp src/v34/handshake/v34_hs_dispatch.cpp"   # all pieces ported so far; a mutation run sets PORT_V04 to ONE of them (the others are built unmutated)
PORT_V04="${PORT_V04:-$ALL_V04}"
OTHERS=""
for f in $ALL_V04; do
  case " $PORT_V04 " in *" $f "*) ;; *) case "$PORT_V04" in */$(basename $f)|$(basename $f)) ;; *) OTHERS="$OTHERS $f";; esac;; esac
done
T="${TPREFIX:-test/.build/v34/handshake/v34_handshake}"
mkdir -p "$(dirname "$T")"
CF="-m32 -Wall -Wextra -Wno-unused-function -O0 -g"
HCF="-m32 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -O2 -g"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc"
LD32="-no-pie"
ALLSC="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22}"
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"

SRENAME="-Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf"
gcc $CF $SRENAME -c test/stub_env.c -o ${T}_stub_env.o
PROBES=""
for c in v01; do g++ -m32 $CXXF -O2 -c test/v34/${c}_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
gcc $HCF -Dsysdep_malloc=h_sysdep_malloc -Dsysdep_free=h_sysdep_free -c test/harness/harness.c -o ${T}_harness.o     # the shared harness library (test/harness/, plain C)

CALLEES="txmit modulatevector txwritequeue V34EchoReportCoeff v90Phase34 k56FlexPhase34 V34SetupModulator V34agc fskdemodulate rxreadqueue V34SetINFO0aBits tone_detect settxlevel txinit v34handshakinit VPcmV34SetMohMessageBits initdigital rxinit detectorinit indicateJaTransmission VPcmV34ReportStartOfEchoAdapt VPcmV34ReportMiddleOfEchoAdapt rxtiminginit V34SetINFO1aBits V34SetINFO0dBits probeselect V34GiveINFO0dBits V34GiveINFO1aBits V34GiveINFO1dBits V34GiveProbeResults VPcmV34InterpretMohMessageBits setfinalrate bitreverse VPcmV34GetMaxUpstreamRateIndex rxtiming dftenergy dftupdate receiver VPcmV34LogTimingOffset"
WEAKARGS=""
for f in $CALLEES; do WEAKARGS="$WEAKARGS --weaken-symbol=$f"; done
objcopy $WEAKARGS $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF -DV04_BLOB $LD32 -o ${T}_test_blob test/v34/handshake/v34_handshake.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_dsplibs_weak.o -lm

gcc -m32 -Isrc -O2 -fno-stack-protector -c src/v34/v34_symbol_tables.c -o ${T}_tables.o
gcc -m32 -Isrc -O2 -fno-stack-protector -c src/v34/v34_init_tables.c -o ${T}_tables3.o
gcc -m32 -Isrc -O2 -fno-stack-protector -c src/v34/v34_receiver_tables.c -o ${T}_tables7.o
gcc -m32 -Isrc -O2 -fno-stack-protector -c src/v34/v34_frame_tables.c -o ${T}_tables4t.o
OBJ0=""; OBJ2=""
gcc -m32 -Isrc -O2 -fno-stack-protector -c src/v34/handshake/v34_hs_tables.c -o ${T}_tables4.o
for f in $PORT_V04 $OTHERS; do
  b=$(basename $f .cpp)
  g++ -m32 $CXXF -O0 -g -c $f -o ${T}_${b}_port_o0.o; OBJ0="$OBJ0 ${T}_${b}_port_o0.o"
  if [ -z "$NOO2" ]; then g++ -m32 $CXXF -O2 -g -c $f -o ${T}_${b}_port_o2.o; OBJ2="$OBJ2 ${T}_${b}_port_o2.o"; fi
done
gcc $HCF $LD32 -o ${T}_test_port test/v34/handshake/v34_handshake.main.c ${T}_harness.o ${T}_stub_env.o $PROBES $OBJ0 ${T}_tables.o ${T}_tables3.o ${T}_tables4.o ${T}_tables7.o ${T}_tables4t.o -lm
if [ -z "$NOO2" ]; then
  gcc $HCF $LD32 -o ${T}_test_port_o2 test/v34/handshake/v34_handshake.main.c ${T}_harness.o ${T}_stub_env.o $PROBES $OBJ2 ${T}_tables.o ${T}_tables3.o ${T}_tables4.o ${T}_tables7.o ${T}_tables4t.o -lm
fi

# the ported case helpers exist in the port objects (they replace pieces of v34handshak, which has no separate symbol)
HELPERS=$(for o in $OBJ0; do nm --defined-only $o; done | awk '$3 ~ /^v04_/ {print $3}' | sort | tr '\n' ' ')
echo "HELPERS: $HELPERS"

if [ -z "$NO64" ]; then
  OBJ64=""
  for f in $PORT_V04 $OTHERS; do b=$(basename $f .cpp); g++ -m64 $CXXF -O2 $FP64_FLAGS -c $f -o ${T}_${b}_64.o; OBJ64="$OBJ64 ${T}_${b}_64.o"; done
  gcc -m64 -Isrc -O2 -fno-stack-protector -c src/v34/handshake/v34_hs_tables.c -o ${T}_tables4_64.o
  gcc -m64 -Isrc -O2 -fno-stack-protector -c src/v34/v34_receiver_tables.c -o ${T}_tables7_64.o
  gcc -m64 -Isrc -O2 -fno-stack-protector -c src/v34/v34_frame_tables.c -o ${T}_tables4t_64.o
  gcc -m64 -Isrc -O2 -fno-stack-protector -c src/v34/v34_symbol_tables.c -o ${T}_tables64.o
  gcc -m64 -Isrc -O2 -fno-stack-protector -c src/v34/v34_init_tables.c -o ${T}_tables3_64.o
  P64=""
  for c in v01; do g++ -m64 $CXXF -O2 -c test/v34/${c}_layout_probe.cpp -o ${T}_probe64_$c.o; P64="$P64 ${T}_probe64_$c.o"; done
  gcc -m64 -Isrc -Wall -Wextra -Wno-unused-function -O2 $SRENAME -c test/stub_env.c -o ${T}_stub_env64.o
  gcc -m64 -Isrc -Wall -Wextra -Wno-unused-function -O2 -Dsysdep_malloc=h_sysdep_malloc -Dsysdep_free=h_sysdep_free -c test/harness/harness.c -o ${T}_harness64.o
  gcc -m64 -Isrc -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -no-pie -o ${T}_test_port64 test/v34/handshake/v34_handshake.main.c ${T}_harness64.o ${T}_stub_env64.o $P64 $OBJ64 ${T}_tables64.o ${T}_tables3_64.o ${T}_tables4_64.o ${T}_tables7_64.o ${T}_tables4t_64.o -lm
fi

# scenarios are independent processes: run them concurrently in groups, then concatenate in order
rm -f ${T}_s*.out
rc=0
pids=""
njobs=0
for i in $SCENARIOS; do
  ( ${R}${T}_test_blob "$i" > ${T}_s$i.blob.out ) & pids="$pids $!"
  ( ${R}${T}_test_port "$i" > ${T}_s$i.port.out ) & pids="$pids $!"
  if [ -z "$NOO2" ]; then ( ${R}${T}_test_port_o2 "$i" > ${T}_s$i.o2.out ) & pids="$pids $!"; fi
  if [ -z "$NO64" ]; then ( ${R}${T}_test_port64 "$i" > ${T}_s$i.p64.out ) & pids="$pids $!"; fi
  njobs=$((njobs + 1))
  if [ $njobs -ge 2 ]; then for p in $pids; do wait $p || rc=1; done; pids=""; njobs=0; fi
done
for p in $pids; do wait $p || rc=1; done
: > ${T}_blob.out; : > ${T}_port.out; : > ${T}_port_o2.out; : > ${T}_port64.out
for i in $SCENARIOS; do
  cat ${T}_s$i.blob.out >> ${T}_blob.out
  cat ${T}_s$i.port.out >> ${T}_port.out
  if [ -z "$NOO2" ]; then cat ${T}_s$i.o2.out >> ${T}_port_o2.out; fi
  if [ -z "$NO64" ]; then cat ${T}_s$i.p64.out >> ${T}_port64.out; fi
done
rm -f ${T}_s*.out
if [ $rc -ne 0 ]; then echo "SCENARIO FAILURE (non-zero exit: coverage, harness assertion / model violation or crash)"; exit 1; fi

diff -u ${T}_blob.out ${T}_port.out
if [ -z "$NOO2" ]; then diff -u ${T}_blob.out ${T}_port_o2.out; fi
if [ -z "$NO64" ]; then diff -u ${T}_blob.out ${T}_port64.out; fi
if [ -n "$GOLDEN_UPDATE" ]; then mkdir -p test/golden/v34/handshake; cp ${T}_blob.out test/golden/v34/handshake/v34_handshake.blob32.out; echo "golden updated"; fi
if [ -z "$TPREFIX" ] && [ "$SCENARIOS" = "$ALLSC" ] && [ -f test/golden/v34/handshake/v34_handshake.blob32.out ] && [ -z "$V04_SEED" ] && [ -z "$V04_TRIALS" ]; then
  if diff -u test/golden/v34/handshake/v34_handshake.blob32.out ${T}_blob.out; then echo "GOLDEN: the 32-bit blob run equals test/golden/v34/handshake/v34_handshake.blob32.out"
  else echo "STALE GOLDEN: the blob output differs from test/golden/v34/handshake/v34_handshake.blob32.out (GOLDEN_UPDATE=1 rewrites it)"; exit 1; fi
fi
# the whole-function variant (V04z): the pieces call the REAL ports of getbit / ApplyBulkDelay (src/v34/handshake/v34_hs_getbit_bulkdelay.cpp; cdecl, no recorders) and the blob runs its OWN unpatched local helpers
# (V04_WHOLE=1 makes the harness skip the entry patches); scenarios that reach the helpers only: blob vs port (-O2, x86-64) outputs must be identical.  NOWHOLE=1 skips this step.
if [ -z "$NOWHOLE" ] && [ -z "$NOO2" ]; then
  WSC="${WHOLE_SCENARIOS:-9 10 12 13}"
  WF="-DV04_REAL_HELPERS -Dgetbit=v04_getbit_impl -DApplyBulkDelay=v04_apply_bulk_delay_impl"
  OBJZ=""
  for f in $PORT_V04 $OTHERS; do b=$(basename $f .cpp); g++ -m32 $CXXF -O2 -g $WF -c $f -o ${T}_${b}_z.o; OBJZ="$OBJZ ${T}_${b}_z.o"; done
  gcc $HCF $LD32 -o ${T}_test_port_z test/v34/handshake/v34_handshake.main.c ${T}_harness.o ${T}_stub_env.o $PROBES $OBJZ ${T}_tables.o ${T}_tables3.o ${T}_tables4.o ${T}_tables7.o ${T}_tables4t.o -lm
  if [ -z "$NO64" ]; then
    OBJZ64=""
    for f in $PORT_V04 $OTHERS; do b=$(basename $f .cpp); g++ -m64 $CXXF -O2 $FP64_FLAGS $WF -c $f -o ${T}_${b}_z64.o; OBJZ64="$OBJZ64 ${T}_${b}_z64.o"; done
    gcc -m64 -Isrc -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -no-pie -o ${T}_test_port_z64 test/v34/handshake/v34_handshake.main.c ${T}_harness64.o ${T}_stub_env64.o $P64 $OBJZ64 ${T}_tables64.o ${T}_tables3_64.o ${T}_tables4_64.o ${T}_tables7_64.o ${T}_tables4t_64.o -lm
  fi
  rc=0
  for i in $WSC; do
    V04_NOCOV=1 V04_WHOLE=1 ${R}${T}_test_blob "$i" > ${T}_z$i.blob.out 2>/dev/null || rc=1
    V04_NOCOV=1 V04_WHOLE=1 ${R}${T}_test_port_z "$i" > ${T}_z$i.port.out 2>/dev/null || rc=1
    cmp -s ${T}_z$i.blob.out ${T}_z$i.port.out || { echo "WHOLE scenario $i: 32-bit port differs from the blob"; diff -u ${T}_z$i.blob.out ${T}_z$i.port.out | head -20; rc=1; }
    if [ -z "$NO64" ]; then
      V04_NOCOV=1 V04_WHOLE=1 ${R}${T}_test_port_z64 "$i" > ${T}_z$i.p64.out 2>/dev/null || rc=1
      cmp -s ${T}_z$i.blob.out ${T}_z$i.p64.out || { echo "WHOLE scenario $i: x86-64 port differs from the 32-bit blob"; diff -u ${T}_z$i.blob.out ${T}_z$i.p64.out | head -20; rc=1; }
    fi
  done
  rm -f ${T}_z*.out ${T}_*_z.o ${T}_*_z64.o
  if [ $rc -ne 0 ]; then echo "WHOLE FAILURE"; exit 1; fi
  echo "WHOLE: scenarios $WSC with the real getbit / ApplyBulkDelay ports: blob == port (-O2$([ -z "$NO64" ] && echo ' and x86-64'))"
fi

echo "MATCH: V04 C++ port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')$([ -z "$NO64" ] && echo " + x86-64 [$FP64_FLAGS] vs the 32-bit blob output")) is behaviorally identical to dsplibs.o for this test"
