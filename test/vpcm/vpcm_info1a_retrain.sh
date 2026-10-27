#!/bin/sh
# Builds the differential test of port chunk V01 (the first V.34 "blocked" row: V34SetINFO1aBits + VPcmV34InitiateRetrain, 2,807 B; src/vpcm/vpcm_info1a_retrain.cpp, extern "C" C++ compiled with
# -fno-exceptions -fno-rtti, because the two functions call methods of the V.90 classes) and runs every scenario in its OWN process.  Every callee (bitreverse, VPcmFloModem::getUinfoValue / setPcmSessionType,
# V92EchoCanceller::setEchoDelay, K56FlexFloModem::setMinMaxRates, V90ConstellationDesigner::setMinMaxRates, v34handshakinit) and edprintf / dsplibs_debug_printf are interposed by a harness recorder:
#   blob      test/vpcm/vpcm_info1a_retrain.main.c + a copy of dsplibs.o in which the callees are WEAK (gcc -m32 -no-pie)
#   port -O0  the harness + the port object under test (+ src/v34/v34_api_tables.c: V34DisconnectThreshTable; the callees are the recorders: nothing else is linked)
#   port -O2  same at -O2
#   port -m64 (NO64=1 skips) the same harness and the port compiled for x86-64; its output is diffed against the 32-bit BLOB's output (the golden dump)
# The ONE harness source has no #ifdef around the code under test.  A scenario exits 2 if its own coverage sites fail, 3 if a harness assumption / the independent model is violated; either fails this script.
# "MATCH" is only printed if every scenario matched AND reached its sites.
# Env: PORT_V01 (port source under test, mutation testing), SCENARIOS, NOO2=1, NO64=1, TPREFIX (output prefix), V01_SEED, V01_TRIALS, GOLDEN_UPDATE=1 (rewrite test/golden/vpcm/vpcm_info1a_retrain.blob32.out), FP64_FLAGS.
# Usage: sh test/vpcm/vpcm_info1a_retrain.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p07) echo vpcm/pcm_parameters;; p43) echo vpcm/v90_demodulator_progress;; p47) echo vpcm/vpcm_flo_modem;; v01) echo v34/v01;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_V01="${PORT_V01:-src/vpcm/vpcm_info1a_retrain.cpp}"
T="${TPREFIX:-test/.build/vpcm/vpcm_info1a_retrain}"
mkdir -p "$(dirname "$T")"
CF="-m32 -Wall -Wextra -Wno-unused-function -O0 -g"
HCF="-m32 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -O2 -g"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc"
LD32="-no-pie"
ALLSC="0 1 2 3"
SCENARIOS="${SCENARIOS:-0 1 2 3}"
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"

SRENAME="-Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf"
gcc $CF $SRENAME -c test/stub_env.c -o ${T}_stub_env.o
PROBES=""
for c in p07 p43 p47 v01; do g++ -m32 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
gcc $HCF -Dsysdep_malloc=h_sysdep_malloc -Dsysdep_free=h_sysdep_free -c test/harness/harness.c -o ${T}_harness.o     # the shared harness library (test/harness/, plain C)

CALLEES="bitreverse v34handshakinit edprintf _ZN12VPcmFloModem13getUinfoValueEs _ZN12VPcmFloModem17setPcmSessionTypeEi _ZN16V92EchoCanceller12setEchoDelayEj _ZN15K56FlexFloModem14setMinMaxRatesEii _ZN24V90ConstellationDesigner14setMinMaxRatesEjj"
WEAKARGS=""
for f in $CALLEES; do WEAKARGS="$WEAKARGS --weaken-symbol=$f"; done
objcopy $WEAKARGS $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF $LD32 -o ${T}_test_blob test/vpcm/vpcm_info1a_retrain.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_dsplibs_weak.o -lm

gcc -m32 -O2 -fno-stack-protector -c src/v34/v34_api_tables.c -o ${T}_tables.o
g++ -m32 $CXXF -O0 -g -c $PORT_V01 -o ${T}_port_o0.o
gcc $HCF $LD32 -o ${T}_test_port test/vpcm/vpcm_info1a_retrain.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o0.o ${T}_tables.o -lm
if [ -z "$NOO2" ]; then
  g++ -m32 $CXXF -O2 -g -c $PORT_V01 -o ${T}_port_o2.o
  gcc $HCF $LD32 -o ${T}_test_port_o2 test/vpcm/vpcm_info1a_retrain.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o2.o ${T}_tables.o -lm
fi

# symbol check: the functions exist, with the same name and binding / section class, in the port object
DEFAULT_SYMRE='V34SetINFO1aBits|VPcmV34InitiateRetrain'
SYMRE="${V01_SYMRE:-$DEFAULT_SYMRE}"
BLOBSYMS=$(nm --defined-only $BLOB_O | awk -v re="^($SYMRE)\$" '$3 ~ re {print $2" "$3}' | sort)
PORTSYMS=$(nm --defined-only ${T}_port_o2.o 2>/dev/null || nm --defined-only ${T}_port_o0.o)
MISSING=$(echo "$BLOBSYMS" | while read b s; do echo "$PORTSYMS" | grep -q " $b $s\$" || echo "$b $s"; done)
NBLOB=$(echo "$BLOBSYMS" | wc -l)
if [ -n "$MISSING" ]; then echo "SYMBOL MISMATCH (blob symbol missing / different binding in the port object):"; echo "$MISSING"; exit 1; fi
echo "SYMBOLS: all $NBLOB blob symbols (name + binding) are defined by the port object"

if [ -z "$NO64" ]; then
  g++ -m64 $CXXF -O2 $FP64_FLAGS -c $PORT_V01 -o ${T}_port64.o
  gcc -m64 -O2 -fno-stack-protector -c src/v34/v34_api_tables.c -o ${T}_tables64.o
  P64=""
  for c in p07 p43 p47 v01; do g++ -m64 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe64_$c.o; P64="$P64 ${T}_probe64_$c.o"; done
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 $SRENAME -c test/stub_env.c -o ${T}_stub_env64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -Dsysdep_malloc=h_sysdep_malloc -Dsysdep_free=h_sysdep_free -c test/harness/harness.c -o ${T}_harness64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -no-pie -o ${T}_test_port64 test/vpcm/vpcm_info1a_retrain.main.c ${T}_harness64.o ${T}_stub_env64.o $P64 ${T}_port64.o ${T}_tables64.o -lm
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
if [ -n "$GOLDEN_UPDATE" ]; then mkdir -p test/golden; cp ${T}_blob.out test/golden/vpcm/vpcm_info1a_retrain.blob32.out; echo "golden updated"; fi
if [ -z "$TPREFIX" ] && [ "$SCENARIOS" = "$ALLSC" ] && [ -f test/golden/vpcm/vpcm_info1a_retrain.blob32.out ] && [ -z "$V01_SEED" ] && [ -z "$V01_TRIALS" ]; then
  if diff -u test/golden/vpcm/vpcm_info1a_retrain.blob32.out ${T}_blob.out; then echo "GOLDEN: the 32-bit blob run equals test/golden/vpcm/vpcm_info1a_retrain.blob32.out"
  else echo "STALE GOLDEN: the blob output differs from test/golden/vpcm/vpcm_info1a_retrain.blob32.out (GOLDEN_UPDATE=1 rewrites it)"; exit 1; fi
fi
echo "MATCH: V01 C++ port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')$([ -z "$NO64" ] && echo " + x86-64 [$FP64_FLAGS] vs the 32-bit blob output")) is behaviorally identical to dsplibs.o for this test"
