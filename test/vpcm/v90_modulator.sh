#!/bin/sh
# Builds the differential test of port chunk P39 (V90Modulator = part A, src/vpcm/v90_modulator.cpp, 19 symbols 3,852 B; V92Modulator = part B, src/vpcm/v92_modulator.cpp, 18 symbols 5,614 B; REAL C++ compiled with
# -fno-exceptions -fno-rtti) and runs every scenario of both halves in its OWN process.  Every callee is interposed by a harness recorder with scripted results / side effects:
#   blob      test/vpcm/v90_modulator.{a,b}.main.c + a copy of dsplibs.o in which the callees (Scrambler<int,uchar>, V90 / V92 BitsToSymbol, the phase 3 / phase 4 modulators, V90MP / V90CP / V92CP infoToBits, Queue<float>, FloatFIR,
#             ResamplerTimingOffset / Resampler, displaySpectralParams) and edprintf are WEAK   (gcc -m32 -no-pie); sysdep_malloc / sysdep_free are the harness' allocator
#   port -O0  the harness + the port object under test (the callees are the recorders: nothing else is linked)
#   port -O2  same at -O2
#   port -m64 (NO64=1 skips) the same harness and the port compiled for x86-64; its output is diffed against the 32-bit BLOB's output (the golden dump)
# The ONE harness source per half has no #ifdef around the code under test: the functions are reached through __asm__ labels with the mangled names.  A scenario exits 2 if its own coverage sites fail, 3 if a harness
# assumption / the independent model is violated; either fails this script.  Also checked: the blob symbols (name + binding) are defined by the port objects; the harness mirrors (tools/layout_gen.py from
# test/vpcm/v90_modulator.layout and the earlier chunks' layouts) equal the port headers' real class layouts (layout probes).  "MATCH" is only printed if every scenario matched AND reached its sites.
# Env: PORT_P39A / PORT_P39B (port source under test, mutation testing), SCENARIOS (A: 0 1 2 3, B: 4 5 6 7 8), HALF (a / b: only that half), NOO2=1, NO64=1, TPREFIX (output prefix, parallel mutation runs),
# P39A_SEED / P39B_SEED, P39A_TRIALS / P39B_TRIALS, GOLDEN_UPDATE=1 (rewrite test/golden/vpcm/v90_modulator.{a,b}.blob32.out from this blob run), FP64_FLAGS (extra flags of the x86-64 port build).
# Usage: sh test/vpcm/v90_modulator.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p07) echo vpcm/pcm_parameters;; p09) echo vpcm/v90_mp;; p10) echo vpcm/v90_cp;; p11) echo vpcm/v92_cp;; p15) echo vpcm/v92_modulus_encoder;; p27) echo vpcm/pcm_phase3_modulators;; p28) echo vpcm/v90_mapper;; p29) echo vpcm/pcm_bits_to_symbol;; p30) echo vpcm/v90_phase4_modulator_symbol;; p32) echo vpcm/v92_phase4_modulator_symbol;; p39) echo vpcm/v90_modulator;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_P39A="${PORT_P39A:-src/vpcm/v90_modulator.cpp}"
PORT_P39B="${PORT_P39B:-src/vpcm/v92_modulator.cpp}"
T="${TPREFIX:-test/.build/vpcm/v90_modulator}"
mkdir -p "$(dirname "$T")"
CF="-m32 -Wall -Wextra -Wno-unused-function -O0 -g"
HCF="-m32 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -O2 -g"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc"
LD32="-no-pie"
HALF="${HALF:-ab}"
SC_A="${SCENARIOS_A:-0 1 2 3}"
SC_B="${SCENARIOS_B:-0 1 2 3 4}"
[ -n "$SCENARIOS" ] && { SC_A="$SCENARIOS"; SC_B="$SCENARIOS"; }
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"

SRENAME="-Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf"
gcc $CF $SRENAME -c test/stub_env.c -o ${T}_stub_env.o
gcc $HCF -c test/harness/harness.c -o ${T}_harness.o     # the shared harness library (test/harness/, plain C)

CALLEES_COMMON="_ZN9ScramblerIihEC1Ejjj _ZN9ScramblerIihED1Ev _ZN9ScramblerIihE5resetEi _ZN9ScramblerIihE7processEPKiPhj displaySpectralParams edprintf"
CALLEES_A="_ZN15V90BitsToSymbolC1EjP13V90Parameters _ZN15V90BitsToSymbolD1Ev _ZN15V90BitsToSymbol19setSymbolsBlockSizeEj _ZN15V90BitsToSymbol18nofBitsForNextTimeEv _ZN15V90BitsToSymbol7processEPhRjPs
 _ZN18V90Phase3ModulatorC1EP13V90Parametersj _ZN18V90Phase3ModulatorD1Ev _ZN18V90Phase3Modulator14setSessionFlagEj _ZN18V90Phase3Modulator5resetE7PcmTypeh20Phase3ModulatorStatejP5V90JdP5V92JdPK19tagV90DILdescriptorj
 _ZN18V90Phase3Modulator14generateSymbolEv _ZN18V90Phase3Modulator6exitJdEv _ZN18V90Phase3Modulator11exitJdPhaseEv _ZN18V90Phase3Modulator7exitDILEv
 _ZN18V90Phase4ModulatorC1EP13V90ParametersjP15V90BitsToSymbolP5V90MPP16V90MappingParamsS7_P5V90CPj _ZN18V90Phase4ModulatorD1Ev _ZN18V90Phase4Modulator14setSessionFlagEj
 _ZN18V90Phase4Modulator5resetE7PcmTypeh20Phase4ModulatorStatejj _ZN18V90Phase4Modulator14generateSymbolEv _ZN18V90Phase4Modulator16setMappingParamsEP16V90MappingParams
 _ZN18V90Phase4Modulator14setRdRtSymbolsEP16V90MappingParams _ZN18V90Phase4Modulator12setRfSymbolsEP16V90MappingParams _ZN18V90Phase4Modulator13resetBeforRRNEv _ZN18V90Phase4Modulator6exitRiEv
 _ZN18V90Phase4Modulator6exitMPEv _ZN18V90Phase4Modulator9exitMPNotEv _ZN18V90Phase4Modulator22setNextStateAfterTRN2dE20Phase4ModulatorState _ZN5V90MP10infoToBitsEv _ZN5V90CP10infoToBitsEv"
CALLEES_B="_ZN15V92BitsToSymbolC1EjP13V92Parameters _ZN15V92BitsToSymbolD1Ev _ZN15V92BitsToSymbol19setSymbolsBlockSizeEj _ZN15V92BitsToSymbol18nofBitsForNextTimeEv _ZN15V92BitsToSymbol7processEPhRjPs
 _ZN18V92Phase3ModulatorC1EP13V92Parameters _ZN18V92Phase3ModulatorD1Ev
 _ZN18V92Phase3Modulator5resetEs23V92Phase3ModulatorStatejP5V92JaPK19tagV90DILdescriptorj _ZN18V92Phase3Modulator14generateSymbolEv _ZN18V92Phase3Modulator6exitJaEv _ZN18V92Phase3Modulator11exitSilenceEv
 _ZN18V92Phase3Modulator12exitSuSecondEv _ZN18V92Phase3Modulator9exitTRN1uEv _ZN18V92Phase4ModulatorC1EP13V92ParametersP15V92BitsToSymbolP5V92CPP16V92MappingParams _ZN18V92Phase4ModulatorD1Ev
 _ZN18V92Phase4Modulator5resetEsh23V92Phase4ModulatorStatejj _ZN18V92Phase4Modulator14generateSymbolEv _ZN18V92Phase4Modulator7exitCPtEv _ZN18V92Phase4Modulator13resetBeforRRNEv
 _ZN18V92Phase4Modulator13resetBeforFPEEv _ZN5V92CP10infoToBitsEv _ZN5QueueIfEC1Ej _ZN5QueueIfED1Ev _ZN5QueueIfE5resetEv _ZN5QueueIfE5writeEf _ZN5QueueIfE5writeEPfj _ZN5QueueIfE4readEPfj
 _ZN8FloatFIRC1EjPfj _ZN8FloatFIRD1Ev _ZN8FloatFIR5resetEv _ZN8FloatFIR7processEPKfPfj _ZN21ResamplerTimingOffsetC1Ejfjffj _ZN21ResamplerTimingOffset15setTimingOffsetEf
 _ZN9Resampler8resampleEPKfjPfRj _ZNK9Resampler18getNormalizedPhaseEv _ZN9Resampler18setNormalizedPhaseEf"

# one weakened copy of dsplibs.o per half
build_half() {
  H=$1; CALLEES="$2"; PORT="$3"; PROBELIST="$4"; SYMRE="$5"
  PROBES=""
  for c in $PROBELIST; do g++ -m32 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_${H}_$c.o; PROBES="$PROBES ${T}_probe_${H}_$c.o"; done
  WEAKARGS=""
  for f in $CALLEES_COMMON $CALLEES; do WEAKARGS="$WEAKARGS --weaken-symbol=$f"; done
  objcopy $WEAKARGS $BLOB_O ${T}${H}_dsplibs_weak.o
  gcc $HCF $LD32 -Itest -o ${T}${H}_test_blob test/vpcm/v90_modulator.${H}.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}${H}_dsplibs_weak.o
  g++ -m32 $CXXF -O0 -g -c $PORT -o ${T}${H}_port_o0.o
  gcc $HCF $LD32 -Itest -o ${T}${H}_test_port test/vpcm/v90_modulator.${H}.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}${H}_port_o0.o
  if [ -z "$NOO2" ]; then
    g++ -m32 $CXXF -O2 -g -c $PORT -o ${T}${H}_port_o2.o
    gcc $HCF $LD32 -Itest -o ${T}${H}_test_port_o2 test/vpcm/v90_modulator.${H}.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}${H}_port_o2.o
  fi
  BLOBSYMS=$(nm --defined-only $BLOB_O | awk -v re="^($SYMRE)\$" '$3 ~ re {print $2" "$3}' | sort)
  PORTSYMS=$(nm --defined-only ${T}${H}_port_o2.o 2>/dev/null || nm --defined-only ${T}${H}_port_o0.o)
  MISSING=$(echo "$BLOBSYMS" | while read b s; do echo "$PORTSYMS" | grep -q " $b $s\$" || echo "$b $s"; done)
  NBLOB=$(echo "$BLOBSYMS" | wc -l)
  if [ -n "$MISSING" ]; then echo "SYMBOL MISMATCH (blob symbol missing / different binding in the port object):"; echo "$MISSING"; exit 1; fi
  echo "SYMBOLS (part $H): all $NBLOB blob symbols (name + binding) are defined by the port object"
  if [ -z "$NO64" ]; then
    g++ -m64 $CXXF -O2 $FP64_FLAGS -c $PORT -o ${T}${H}_port64.o
    P64=""
    for c in $PROBELIST; do g++ -m64 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe64_${H}_$c.o; P64="$P64 ${T}_probe64_${H}_$c.o"; done
    gcc -m64 -Wall -Wextra -Wno-unused-function -O2 $SRENAME -c test/stub_env.c -o ${T}_stub_env64.o
    gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness64.o
    gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -no-pie -Itest -o ${T}${H}_test_port64 test/vpcm/v90_modulator.${H}.main.c ${T}_harness64.o ${T}_stub_env64.o $P64 ${T}${H}_port64.o
  fi
}

run_half() {
  H=$1; SCS=$2; HV=$3
  rm -f ${T}${H}_s*.out
  pids=""; njobs=0
  for i in $SCS; do
    ( ${R}${T}${H}_test_blob "$i" > ${T}${H}_s$i.blob.out ) & pids="$pids $!"
    ( ${R}${T}${H}_test_port "$i" > ${T}${H}_s$i.port.out ) & pids="$pids $!"
    if [ -z "$NOO2" ]; then ( ${R}${T}${H}_test_port_o2 "$i" > ${T}${H}_s$i.o2.out ) & pids="$pids $!"; fi
    if [ -z "$NO64" ]; then ( ${R}${T}${H}_test_port64 "$i" > ${T}${H}_s$i.p64.out ) & pids="$pids $!"; fi
    njobs=$((njobs + 1))
    if [ $njobs -ge 2 ]; then for p in $pids; do wait $p || rc=1; done; pids=""; njobs=0; fi
  done
  for p in $pids; do wait $p || rc=1; done
  : > ${T}${H}_blob.out; : > ${T}${H}_port.out; : > ${T}${H}_port_o2.out; : > ${T}${H}_port64.out
  for i in $SCS; do
    cat ${T}${H}_s$i.blob.out >> ${T}${H}_blob.out
    cat ${T}${H}_s$i.port.out >> ${T}${H}_port.out
    if [ -z "$NOO2" ]; then cat ${T}${H}_s$i.o2.out >> ${T}${H}_port_o2.out; fi
    if [ -z "$NO64" ]; then cat ${T}${H}_s$i.p64.out >> ${T}${H}_port64.out; fi
  done
  rm -f ${T}${H}_s*.out
}

rc=0
case "$HALF" in *a*)
  build_half a "$CALLEES_A" "$PORT_P39A" "p07 p09 p10 p11 p27 p28 p29 p30 p39" '_ZN12V90Modulator(C[12]|D[12]|14setSessionFlag|11enterPhase[34]|6exitJd|11exitJdPhase|6exitRi|22acknowledgeCPReception|25acknowledgeCPNotReception|21acknowledgeEReception|14enterDataPhase|7exitDIL|11initiate(RRN|FPE)|5reset|8progress).*'
esac
case "$HALF" in *b*)
  build_half b "$CALLEES_B" "$PORT_P39B" "p07 p11 p15 p27 p29 p32 p39" '_Z[NK]*12V92Modulator(C[12]|D[12]|19getV92TxFilterDelay|11enterPhase[34]|6exitJa|11exitSilence|12exitSuSecond|15exitTRN1uSecond|7exitCPt|11initiate(RRN|FPE)|14enterDataPhase|17mkResampledSignal|5reset|8progress).*'
esac
case "$HALF" in *a*) run_half a "$SC_A" ;; esac
case "$HALF" in *b*) run_half b "$SC_B" ;; esac
if [ $rc -ne 0 ]; then echo "SCENARIO FAILURE (non-zero exit: coverage, harness assertion / model violation or crash)"; exit 1; fi

for H in a b; do
  case "$HALF" in *$H*) ;; *) continue;; esac
  diff -u ${T}${H}_blob.out ${T}${H}_port.out
  if [ -z "$NOO2" ]; then diff -u ${T}${H}_blob.out ${T}${H}_port_o2.out; fi
  if [ -z "$NO64" ]; then diff -u ${T}${H}_blob.out ${T}${H}_port64.out; fi
  if [ -n "$GOLDEN_UPDATE" ]; then mkdir -p test/golden/vpcm; cp ${T}${H}_blob.out test/golden/vpcm/v90_modulator.${H}.blob32.out; echo "golden p39$H updated"; fi
  if [ -z "$TPREFIX" ] && [ -f test/golden/vpcm/v90_modulator.${H}.blob32.out ] && [ -z "$P39A_SEED$P39B_SEED" ] && [ -z "$P39A_TRIALS$P39B_TRIALS" ] && [ -z "$SCENARIOS$SCENARIOS_A$SCENARIOS_B" ]; then
    if diff -u test/golden/vpcm/v90_modulator.${H}.blob32.out ${T}${H}_blob.out; then echo "GOLDEN (part $H): the 32-bit blob run equals test/golden/vpcm/v90_modulator.${H}.blob32.out"
    else echo "STALE GOLDEN (part $H): the blob output differs from test/golden/vpcm/v90_modulator.${H}.blob32.out (GOLDEN_UPDATE=1 rewrites it)"; exit 1; fi
  fi
done
echo "MATCH: P39 C++ port (halves: $HALF; $([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')$([ -z "$NO64" ] && echo " + x86-64 [$FP64_FLAGS] vs the 32-bit blob output")) is behaviorally identical to dsplibs.o for this test"
