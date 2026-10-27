#!/bin/sh
# P44 INTEGRATION test: P43's V90Demodulator::progress (src/vpcm/v90_demodulator_progress.cpp) together with the REAL P44 methods it calls (enterRRN / enterDataSteadyState / enterDataPhase / exitPhase3;
# src/vpcm/v90_demodulator.cpp): the P43 harness (test/vpcm/v90_demodulator_progress.main.c, built with -DP44_INTEG) no longer interposes those four methods (the blob's own run on the blob side, the P44 port's on the port side);
# their callees are recorders (the timing-history mean / std, the demapper's linear-mapping reset, V90Equalizer::enterPhase4, V90Phase3Demodulator::exitDIL, the TRN2 designer, V90Phase4Demodulator::reset)
# on top of the P43 recorder set.  The independent model of progress scripts the P44 methods and cannot follow real ones: ONLY blob vs port is compared (g_nomodel behaviour: no model run, no coverage
# check).  Outputs: -O0, -O2, x86-64 vs the 32-bit blob run; own golden test/golden/vpcm/v90_demodulator.integ.blob32.integ.out.
# The P44 port object also references the callees of its other methods (constructor, destructor, reset ...), which this harness does not provide: the port links use --unresolved-symbols=ignore-all (a call into one would crash).
# Usage: sh test/vpcm/v90_demodulator.integ.sh   (from slmodemd/dsplibs-port/);  GOLDEN_UPDATE=1 rewrites the golden;  NO64=1 / NOO2=1 skip;  P44I_TRIALS trials per scenario (default 3000)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p07) echo vpcm/pcm_parameters;; p09) echo vpcm/v90_mp;; p11) echo vpcm/v92_cp;; p17) echo vpcm/v90_connection_evaluator;; p18) echo vpcm/v90_adid;; p24) echo vpcm/v90_constellation_noise;; p28) echo vpcm/v90_mapper;; p34) echo vpcm/v90_phase3_demod_v92_decision;; p37) echo vpcm/v90_phase4_demod_decisions;; p40) echo vpcm/v90_equalizer_process;; p43) echo vpcm/v90_demodulator_progress;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_P43="src/vpcm/v90_demodulator_progress.cpp"
PORT_P44="src/vpcm/v90_demodulator.cpp"
T="${TPREFIX:-test/.build/vpcm/v90_demodulator.integ}"
mkdir -p "$(dirname "$T")"
CF="-m32 -Wall -Wextra -Wno-unused-function -O0 -g"
HCF="-m32 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -O2 -g -DP44_INTEG"
TR="${P44I_TRIALS:-3000}"
export P43_NOCOV=1 P43_TRIALS="$TR"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc"
LD32="-no-pie"
ALLSC="0 1 2"
SCENARIOS="${SCENARIOS:-0 1 2}"
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"

SRENAME="-Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf"
gcc $CF $SRENAME -c test/stub_env.c -o ${T}_stub_env.o
PROBES=""
for c in p07 p09 p11 p17 p18 p24 p28 p34 p37 p40 p43; do g++ -m32 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
gcc $HCF -c test/harness/harness.c -o ${T}_harness.o     # the shared harness library (test/harness/, plain C)

CALLEES="displaySpectralParams edprintf _ZN11DescramblerIhiE7processEPKhPij _ZN11V90Demapper7processEPhRj _ZN12V90Equalizer23calcMeanErrorStatisticsEv _ZN12V90Equalizer23restoreEqualizerToFloatEv _ZN12V90Equalizer7processEPfjPsS0_Rj _ZN12V90PreFilter16getV90CapabilityEv _ZN12V90PreFilter9setFilterEj _ZN12V90Resampler11setBllStateE11V90BllStatej _ZN12V90Resampler8resampleEPKfjPfRj _ZN15ResamplerTiming21adjustHalfBaudBpfGainEf _ZN19V90SpectralVerifier17startAccumulationEv _ZN19V90SpectralVerifier7processEPfj _ZN20V90Phase3Demodulator11getMaxUcodeEv _ZN20V90Phase3Demodulator24setDigitalImairmentsInfoEv _ZN20V90Phase4Demodulator13resetBeforRRNEv _ZN20V90Phase4Demodulator14enterWaitForCPEv _ZN20V90Phase4Demodulator14enterWaitForEdEv _ZN20V90Phase4Demodulator14enterWaitForMPEv _ZN22V90ConnectionEvaluator14evaluatePhase3Ev _ZN22V90ConnectionEvaluator14evaluatePhase4Ef _ZN22V90ConnectionEvaluator18evaluateConnectionEv _ZN22V90ConnectionEvaluator20indicateLocalRetrainEv _ZN22V90ConnectionEvaluator21indicateRemoteRetrainEv _ZN22V90ConnectionEvaluator26evaluateMeanErrorStdPhase4Eff _ZN22V90ConnectionEvaluator30updateCurrentConstellationDataEsfff _ZN24V90ConstellationDesigner7processEjP25V90AutoDigitalImpDetectorfiP16V90MappingParamsPA128_sS5_PsPhS7_h23__tHardwareCodecTypes__j28V90SpecialSpectralConditions _ZN25V90AutoDigitalImpDetector17setConnectionTypeEs _ZN25V90AutoDigitalImpDetector21isThereAnyAltRbsPhaseEv _ZN3AgcIfE6freezeEv _ZN3AgcIfE7processEPKfPfj _ZN5V90Jd12getRatesMaskEv _ZN5V90Jd15getMaxLookaheadEv _ZN5V92Jd12getRatesMaskEv _ZN5V92Jd15getMaxLookaheadEv _ZN8FloatFIR7processEPKfPfj _ZNK12V90PreFilter13isV90WithEia6Ev _ZNK21ResamplerTimingOffset18getTimingOffsetPPMEv _ZN12V90Resampler20getTimingHistoryMeanEv _ZN12V90Resampler19getTimingHistoryStdEv _ZN11V90Demapper20resetLinearMappStudyEj _ZN12V90Equalizer11enterPhase4Ev _ZN20V90Phase3Demodulator7exitDILEv _ZN15V90TRN2Designer18setNofUcodesInTrn2Es _ZN15V90TRN2Designer13V90TRN2DesignEP16V90MappingParamsPA128_sS3_PA128_hPs7PcmTypeS7_sPhjh28V90SpecialSpectralConditions _ZN20V90Phase4Demodulator5resetEh22Phase4DemodulatorStatejj"
WEAKARGS=""
for f in $CALLEES; do WEAKARGS="$WEAKARGS --weaken-symbol=$f"; done
objcopy $WEAKARGS $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF $LD32 -o ${T}_test_blob test/vpcm/v90_demodulator_progress.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_dsplibs_weak.o

g++ -m32 $CXXF -O0 -g -c $PORT_P43 -o ${T}_port_o0.o
g++ -m32 $CXXF -O0 -g -c $PORT_P44 -o ${T}_p44_o0.o
gcc $HCF $LD32 -Wl,--unresolved-symbols=ignore-all -o ${T}_test_port test/vpcm/v90_demodulator_progress.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o0.o ${T}_p44_o0.o
if [ -z "$NOO2" ]; then
  g++ -m32 $CXXF -O2 -g -c $PORT_P43 -o ${T}_port_o2.o
  g++ -m32 $CXXF -O2 -g -c $PORT_P44 -o ${T}_p44_o2.o
  gcc $HCF $LD32 -Wl,--unresolved-symbols=ignore-all -o ${T}_test_port_o2 test/vpcm/v90_demodulator_progress.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o2.o ${T}_p44_o2.o
fi

if [ -z "$NO64" ]; then
  g++ -m64 $CXXF -O2 $FP64_FLAGS -c $PORT_P43 -o ${T}_port64.o
  g++ -m64 $CXXF -O2 $FP64_FLAGS -c $PORT_P44 -o ${T}_p44_64.o
  P64=""
  for c in p07 p09 p11 p17 p18 p24 p28 p34 p37 p40 p43; do g++ -m64 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe64_$c.o; P64="$P64 ${T}_probe64_$c.o"; done
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 $SRENAME -c test/stub_env.c -o ${T}_stub_env64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -no-pie -DP44_INTEG -Wl,--unresolved-symbols=ignore-all -o ${T}_test_port64 test/vpcm/v90_demodulator_progress.main.c ${T}_harness64.o ${T}_stub_env64.o $P64 ${T}_port64.o ${T}_p44_64.o
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
if [ -n "$GOLDEN_UPDATE" ]; then mkdir -p test/golden; cp ${T}_blob.out test/golden/vpcm/v90_demodulator.integ.blob32.integ.out; echo "golden updated"; fi
if [ -z "$TPREFIX" ] && [ "$SCENARIOS" = "$ALLSC" ] && [ -f test/golden/vpcm/v90_demodulator.integ.blob32.integ.out ] && [ -z "$P44I_SEED" ] && [ -z "$P44I_TRIALS" ]; then
  if diff -u test/golden/vpcm/v90_demodulator.integ.blob32.integ.out ${T}_blob.out; then echo "GOLDEN: the 32-bit blob run equals test/golden/vpcm/v90_demodulator.integ.blob32.integ.out"
  else echo "STALE GOLDEN: the blob output differs from test/golden/vpcm/v90_demodulator.integ.blob32.integ.out (GOLDEN_UPDATE=1 rewrites it)"; exit 1; fi
fi
echo "MATCH: P44 integration (P43 progress + the real P44 methods) ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')$([ -z "$NO64" ] && echo " + x86-64 [$FP64_FLAGS] vs the 32-bit blob output")) is behaviorally identical to dsplibs.o for this test"
