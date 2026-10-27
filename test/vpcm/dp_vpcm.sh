#!/bin/sh
# Builds the differential test of port chunk Z01 (the glue: VPCMXF_Create / Delete / SessionTermination, dp_param_get, the LOCAL vpcm_create / vpcm_delete / vpcm_run reached through vpcm_op, dp_vpcm_init / exit: 9 functions, 3,534 B; src/vpcm/dp_vpcm.cpp, extern "C" C++ compiled with -fno-exceptions -fno-rtti) and runs every scenario in its OWN process.
# Every callee (sysdep_memset / memcpy, modem_get_param / set_param / get_bits / put_bits / debug_log_data / dp_register / dp_deregister, K56FLEX_Create / Delete, VPcmV34Create / Progress / GetCleanedSamples /
# GetCurrentSessionDP / RxBitRate / TxBitRate, the VPcmFloModem constructor, the six member destructors, V90Demodulator::sessionTermination) and dsplibs_debug_printf is interposed by a harness recorder:
# (the blob never calls its own vpcm_create / vpcm_delete / vpcm_run directly: the harness takes them from the record dp_vpcm_init registers, for the blob and the port alike)
#   blob      test/vpcm/dp_vpcm.main.c + a copy of dsplibs.o in which the callees are WEAK (gcc -m32 -no-pie)
#   port -O0  the harness + the port object under test (the callees are the recorders: nothing else is linked)
#   port -O2  same at -O2
#   port -m64 (NO64=1 skips) the same harness and the port compiled for x86-64; its output is diffed against the 32-bit BLOB's output (the golden dump)
# The ONE harness source has no #ifdef around the code under test.  A scenario exits 2 if its own coverage sites fail, 3 if a harness assumption / the independent model is violated; either fails this script.
# "MATCH" is only printed if every scenario matched AND reached its sites.
# Env: PORT_Z01 (port source under test, mutation testing), SCENARIOS, NOO2=1, NO64=1, TPREFIX (output prefix), Z01_SEED, Z01_TRIALS, GOLDEN_UPDATE=1 (rewrite test/golden/vpcm/dp_vpcm.blob32.out), FP64_FLAGS.
# Usage: sh test/vpcm/dp_vpcm.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p47) echo vpcm/vpcm_flo_modem;; v01) echo v34/v01;; z01) echo vpcm/dp_vpcm;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_Z01="${PORT_Z01:-src/vpcm/dp_vpcm.cpp}"
T="${TPREFIX:-test/.build/vpcm/dp_vpcm}"
mkdir -p "$(dirname "$T")"
CF="-m32 -Wall -Wextra -Wno-unused-function -O0 -g"
HCF="-m32 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -O2 -g"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc"
LD32="-no-pie"
ALLSC="0 1 2 3"
SCENARIOS="${SCENARIOS:-0 1 2 3}"
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"

SRENAME="-Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf -Dsysdep_memset=stub_env_unused_memset -Dsysdep_memcpy=stub_env_unused_memcpy -Dmodem_debug_log_data=stub_env_unused_mdld -Dmodem_dp_deregister=stub_env_unused_mdd -Dmodem_dp_register=stub_env_unused_mdr -Dmodem_get_bits=stub_env_unused_mgb -Dmodem_get_param=stub_env_unused_mgp -Dmodem_put_bits=stub_env_unused_mpb -Dmodem_set_param=stub_env_unused_msp"
gcc $CF $SRENAME -c test/stub_env.c -o ${T}_stub_env.o
PROBES=""
for c in p47 v01 z01; do g++ -m32 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
gcc $HCF -Dsysdep_malloc=h_sysdep_malloc -Dsysdep_free=h_sysdep_free -c test/harness/harness.c -o ${T}_harness.o     # the shared harness library (test/harness/, plain C)

CALLEES="K56FLEX_Create K56FLEX_Delete VPcmV34Create VPcmV34Progress VPcmV34GetCleanedSamples VPcmV34GetCurrentSessionDP VPcmV34GetCurrentRxBitRate VPcmV34GetCurrentTxBitRate _ZN12VPcmFloModemC1EPv12V90ModemSideP19_tagModemParametersj20V90ComputationalMode20V92ComputationalMode _ZN10GenericIIRIfdED1Ev _ZN8SineWaveIffED1Ev _ZN17ANSamToneDetectorD1Ev _ZN16V92EchoCancellerD1Ev _ZN8V92ModemD1Ev _ZN8V90ModemD1Ev _ZN14V90Demodulator18sessionTerminationEv"
WEAKARGS=""
for f in $CALLEES; do WEAKARGS="$WEAKARGS --weaken-symbol=$f"; done
objcopy $WEAKARGS $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF $LD32 -o ${T}_test_blob test/vpcm/dp_vpcm.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_dsplibs_weak.o -lm

g++ -m32 $CXXF -O0 -g -c $PORT_Z01 -o ${T}_port_o0.o
gcc $HCF $LD32 -o ${T}_test_port test/vpcm/dp_vpcm.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o0.o -lm
if [ -z "$NOO2" ]; then
  g++ -m32 $CXXF -O2 -g -c $PORT_Z01 -o ${T}_port_o2.o
  gcc $HCF $LD32 -o ${T}_test_port_o2 test/vpcm/dp_vpcm.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o2.o -lm
fi

# symbol check: the functions exist, with the same name and binding / section class, in the port object
DEFAULT_SYMRE='VPCMXF_Create|VPCMXF_Delete|VPCMXF_SessionTermination|dp_vpcm_init|dp_vpcm_exit|dp_param_get'
SYMRE="${Z01_SYMRE:-$DEFAULT_SYMRE}"
BLOBSYMS=$(nm --defined-only $BLOB_O | awk -v re="^($SYMRE)\$" '$3 ~ re {print $2" "$3}' | sort)
PORTSYMS=$(nm --defined-only ${T}_port_o2.o 2>/dev/null || nm --defined-only ${T}_port_o0.o)
MISSING=$(echo "$BLOBSYMS" | while read b s; do echo "$PORTSYMS" | grep -q " $b $s\$" || echo "$b $s"; done)
NBLOB=$(echo "$BLOBSYMS" | wc -l)
if [ -n "$MISSING" ]; then echo "SYMBOL MISMATCH (blob symbol missing / different binding in the port object):"; echo "$MISSING"; exit 1; fi
echo "SYMBOLS: all $NBLOB blob symbols (name + binding) are defined by the port object"

if [ -z "$NO64" ]; then
  g++ -m64 $CXXF -O2 $FP64_FLAGS -c $PORT_Z01 -o ${T}_port64.o
  P64=""
  for c in p47 v01 z01; do g++ -m64 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe64_$c.o; P64="$P64 ${T}_probe64_$c.o"; done
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 $SRENAME -c test/stub_env.c -o ${T}_stub_env64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -Dsysdep_malloc=h_sysdep_malloc -Dsysdep_free=h_sysdep_free -c test/harness/harness.c -o ${T}_harness64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -no-pie -o ${T}_test_port64 test/vpcm/dp_vpcm.main.c ${T}_harness64.o ${T}_stub_env64.o $P64 ${T}_port64.o -lm
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
if [ -n "$GOLDEN_UPDATE" ]; then mkdir -p test/golden; cp ${T}_blob.out test/golden/vpcm/dp_vpcm.blob32.out; echo "golden updated"; fi
# INTEGRATION (32-bit only, NOGRAPH=1 skips): the glue inside the REAL object graph.  "blob" = test/vpcm/dp_vpcm.graph.main.c + the unmodified dsplibs.o; "port over blob" = the same harness + a copy of dsplibs.o whose six GLOBAL
# glue symbols (VPCMXF_Create / Delete / SessionTermination, dp_param_get, dp_vpcm_init / exit) are WEAK + the port's dp_vpcm.o (so the port's vpcm_op replaces the blob's; the blob's LOCAL vpcm_* stay unreferenced).  Two sessions
# are wired back to back through vpcm_run for 6000 calls per DP id (0x22 / 0x5a / 0x5c x which side answers); the canonical outputs must be identical (and equal to test/golden/vpcm/dp_vpcm.graph.blob32.out).
GV="0 1 2 3 4 5"
if [ -z "$NOGRAPH" ]; then
  gcc $CF $SRENAME -Dsysdep_vsnprintf=stub_env_unused_vsn -Dsysdep_sprintf=stub_env_unused_spf -Dsysdep_strcat=stub_env_unused_scat -Dsysdep_strcpy=stub_env_unused_scpy -Dsysdep_strlen=stub_env_unused_slen -c test/stub_env.c -o ${T}_stub_envg.o
  objcopy --weaken-symbol=VPCMXF_Create --weaken-symbol=VPCMXF_Delete --weaken-symbol=VPCMXF_SessionTermination --weaken-symbol=dp_vpcm_init --weaken-symbol=dp_vpcm_exit --weaken-symbol=dp_param_get $BLOB_O ${T}_graph_weak.o
  gcc $HCF $LD32 -o ${T}_gtest_blob test/vpcm/dp_vpcm.graph.main.c ${T}_harness.o ${T}_stub_envg.o $BLOB_O -lm
  GPORT=${T}_port_o2.o; [ -n "$NOO2" ] && GPORT=${T}_port_o0.o
  gcc $HCF $LD32 -o ${T}_gtest_port test/vpcm/dp_vpcm.graph.main.c ${T}_harness.o ${T}_stub_envg.o ${T}_graph_weak.o $GPORT -lm
  : > ${T}_gblob.out; : > ${T}_gport.out
  for v in $GV; do
    ( ${R}${T}_gtest_blob $v > ${T}_g$v.blob.out ) & gp1=$!
    ( ${R}${T}_gtest_port $v > ${T}_g$v.port.out ) & gp2=$!
    wait $gp1 || rc=1; wait $gp2 || rc=1
    cat ${T}_g$v.blob.out >> ${T}_gblob.out; cat ${T}_g$v.port.out >> ${T}_gport.out
  done
  rm -f ${T}_g[0-9].blob.out ${T}_g[0-9].port.out
  if [ $rc -ne 0 ]; then echo "GRAPH SCENARIO FAILURE (non-zero exit)"; exit 1; fi
  diff -u ${T}_gblob.out ${T}_gport.out
  echo "GRAPH: the port inside the blob's own object graph (6 sessions pairs x 6000 calls) reproduces the blob's output"
  if [ -n "$GOLDEN_UPDATE" ]; then cp ${T}_gblob.out test/golden/vpcm/dp_vpcm.graph.blob32.out; echo "graph golden updated"; fi
  if [ -z "$TPREFIX" ] && [ -f test/golden/vpcm/dp_vpcm.graph.blob32.out ] && [ -z "$Z01G_SEED" ]; then
    if diff -u test/golden/vpcm/dp_vpcm.graph.blob32.out ${T}_gblob.out; then echo "GOLDEN: the graph blob run equals test/golden/vpcm/dp_vpcm.graph.blob32.out"; else echo "STALE GRAPH GOLDEN"; exit 1; fi
  fi
fi

if [ -z "$TPREFIX" ] && [ "$SCENARIOS" = "$ALLSC" ] && [ -f test/golden/vpcm/dp_vpcm.blob32.out ] && [ -z "$Z01_SEED" ] && [ -z "$Z01_TRIALS" ]; then
  if diff -u test/golden/vpcm/dp_vpcm.blob32.out ${T}_blob.out; then echo "GOLDEN: the 32-bit blob run equals test/golden/vpcm/dp_vpcm.blob32.out"
  else echo "STALE GOLDEN: the blob output differs from test/golden/vpcm/dp_vpcm.blob32.out (GOLDEN_UPDATE=1 rewrites it)"; exit 1; fi
fi
echo "MATCH: Z01 C++ port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')$([ -z "$NO64" ] && echo " + x86-64 [$FP64_FLAGS] vs the 32-bit blob output")) is behaviorally identical to dsplibs.o for this test"
