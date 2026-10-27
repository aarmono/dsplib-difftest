#!/bin/sh
# Builds the differential test of port chunk P44 (the 20 other methods of V90Demodulator: C1 / C2, D1 / D2, sessionTermination, enterPhase3 / 4, enterFPE, enterRRN, enterDataSteadyState, enterDataPhase,
# exitPhase3, reset, reInit, enterChannelVerification, setSessionFlag, getBitRate, getRbsPattern, indicateRemoteRateReneg, getAT_UD; 6,221 B; src/vpcm/v90_demodulator.cpp, REAL C++ compiled with
# -fno-exceptions -fno-rtti) and runs every scenario in its OWN process.  Every callee (the 14 sub-object constructors / 13 destructors and the 36 methods and printers they call) is interposed by a harness
# recorder with scripted results:
#   blob      test/vpcm/v90_demodulator.main.main.c + a copy of dsplibs.o in which the callees (the sub-object constructors, destructors, methods, displaySpectralParams) and edprintf are WEAK   (gcc -m32 -no-pie); sysdep_malloc /
#             sysdep_free are the harness' allocator
#   port -O0  the harness + the port object under test (the callees are the recorders: nothing else is linked; the port calls getTimingOffsetPPM_LD, the extended-precision twin of the P05 export)
#   port -O2  same at -O2
#   port -m64 (NO64=1 skips) the same harness and the port compiled for x86-64; its output is diffed against the 32-bit BLOB's output (the golden dump)
# The ONE harness source has no #ifdef around the code under test: the functions are reached through __asm__ labels with the mangled names.  A scenario exits 2 if its own coverage sites fail, 3 if a harness
# assumption / the independent model is violated; either fails this script.  Also checked: the blob symbols (name + binding) are defined by the port object; the harness mirrors (tools/layout_gen.py from
# test/vpcm/v90_demodulator.layout, p07, p09, p11, p17, p18, p24, p28, p34, p37, p40) equal the port headers' real class layouts (layout probes).  "MATCH" is only printed if every scenario matched AND reached its sites.
# Env: PORT_P44 (port source under test, mutation testing), SCENARIOS, NOO2=1, NO64=1, TPREFIX (output prefix, parallel mutation runs), P44_SEED (alternative input stream), P44_TRIALS (trials per scenario),
# GOLDEN_UPDATE=1 (rewrite test/golden/vpcm/v90_demodulator.main.blob32.out from this blob run), FP64_FLAGS (extra flags of the x86-64 port build).
# Usage: sh test/vpcm/v90_demodulator.main.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p07) echo vpcm/pcm_parameters;; p09) echo vpcm/v90_mp;; p11) echo vpcm/v92_cp;; p17) echo vpcm/v90_connection_evaluator;; p18) echo vpcm/v90_adid;; p24) echo vpcm/v90_constellation_noise;; p26) echo vpcm/v90_trn2_designer;; p28) echo vpcm/v90_mapper;; p34) echo vpcm/v90_phase3_demod_v92_decision;; p37) echo vpcm/v90_phase4_demod_decisions;; p40) echo vpcm/v90_equalizer_process;; p43) echo vpcm/v90_demodulator_progress;; p44) echo vpcm/v90_demodulator;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_P44="${PORT_P44:-src/vpcm/v90_demodulator.cpp}"
T="${TPREFIX:-test/.build/vpcm/v90_demodulator.main}"
mkdir -p "$(dirname "$T")"
CF="-m32 -Wall -Wextra -Wno-unused-function -O0 -g"
HCF="-m32 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -O2 -g"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc"
LD32="-no-pie"
ALLSC="0 1 2 3 4"
SCENARIOS="${SCENARIOS:-0 1 2 3 4}"
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"

SRENAME="-Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf"
gcc $CF $SRENAME -c test/stub_env.c -o ${T}_stub_env.o
PROBES=""
for c in p07 p09 p11 p17 p18 p24 p26 p28 p34 p37 p40 p43 p44; do g++ -m32 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
gcc $HCF -c test/harness/harness.c -o ${T}_harness.o     # the shared harness library (test/harness/, plain C)

CALLEES="edprintf _ZN11DescramblerIhiE5resetEh _ZN3AgcIfE5resetEv _ZN12V90PreFilter5resetEv _ZNK12V90PreFilter13isV90WithEia6Ev _ZN12V90PreFilter12selectFilterEv _ZN12V90PreFilter12setParamEia6Ev _ZN12V90PreFilter16displayParamEia6Ev _ZN12V90Resampler5resetEv _ZN12V90Resampler11setBllStateE11V90BllStatej _ZN12V90Resampler20getTimingHistoryMeanEv _ZN12V90Resampler19getTimingHistoryStdEv _ZN12V90Resampler22getTimingHistoryMeanLDEv _ZN12V90Resampler21getTimingHistoryStdLDEv _ZN21ResamplerTimingOffset15setTimingOffsetEf _ZN19V90SpectralVerifier5resetEv _ZN12V90Equalizer5resetEj _ZN12V90Equalizer11enterPhase3Ev _ZN12V90Equalizer11enterPhase4Ev _ZN12V90Equalizer24enterChannelVerificationEv _ZN20V90Phase3Demodulator23clearVerificationStatusEv _ZN20V90Phase3Demodulator14setSessionFlagEj _ZN20V90Phase3Demodulator5resetE7PcmTypeh22Phase3DemodulatorStatejP5V90JdP5V92JdP19tagV90DILdescriptorssfj _ZN20V90Phase3Demodulator7exitDILEv _ZN20V90Phase3Demodulator11getMaxUcodeEv _ZN20V90Phase4Demodulator14setSessionFlagEj _ZN20V90Phase4Demodulator5resetEh22Phase4DemodulatorStatejj _ZN22V90ConnectionEvaluator5resetEv _ZN22V90ConnectionEvaluator23indicateRemoteRateRenegEv _ZN24V90ConstellationDesigner5resetEv _ZN11V90Demapper20resetLinearMappStudyEj _ZNK13V90Phase2Info9printInfoEv _ZN25V90AutoDigitalImpDetector21isThereAnyAltRbsPhaseEv _ZN15V90TRN2Designer18setNofUcodesInTrn2Es _ZN15V90TRN2Designer13V90TRN2DesignEP16V90MappingParamsPA128_sS3_PA128_hPs7PcmTypeS7_sPhjh28V90SpecialSpectralConditions _ZN5V90Jd15getMaxLookaheadEv _ZN5V92Jd15getMaxLookaheadEv displaySpectralParams _ZN3AgcIfEC1Ev _ZN12V90PreFilterC1E23__tHardwareCodecTypes__P13V90Phase2InfoP13V90Parameters _ZN12V90ResamplerC1EjfjfP13V90Parametersfj _ZN21V90ConstellationPowerC1Ev _ZN11DescramblerIhiEC1Ejjj _ZN19V90SpectralVerifierC1EP13V90Parameters _ZN25V90AutoDigitalImpDetectorC1EP13V90Parameters _ZN22V90ConnectionEvaluatorC1EP13V90Parameters _ZN20V90Phase3DemodulatorC1EP13V90ParametersP19V90SpectralVerifierjP25V90AutoDigitalImpDetector _ZN11V90DemapperC1EjP13V90ParametersP25V90AutoDigitalImpDetector _ZN20V90Phase4DemodulatorC1EP16V90MappingParamsS1_P11V90DemapperP5V90CPP5V90MPP11DescramblerIhiEP22V90ConnectionEvaluatorP13V90ParametersP20V90Phase3DemodulatorP25V90AutoDigitalImpDetectorj _ZN12V90EqualizerC1EjjP20V90Phase3DemodulatorP20V90Phase4DemodulatorP11V90DemapperP22V90ConnectionEvaluatorP19V90SpectralVerifierP13V90ParametersP12V90ResamplerP12V90PreFilter20V90ComputationalMode _ZN15V90TRN2DesignerC1EP13V90ParametersP21V90ConstellationPower _ZN24V90ConstellationDesignerC1EP13V90ParametersP12V90PreFilterP21V90ConstellationPower _ZN12V90PreFilterD1Ev _ZN12V90ResamplerD1Ev _ZN21V90ConstellationPowerD1Ev _ZN11DescramblerIhiED1Ev _ZN19V90SpectralVerifierD1Ev _ZN25V90AutoDigitalImpDetectorD1Ev _ZN22V90ConnectionEvaluatorD1Ev _ZN20V90Phase3DemodulatorD1Ev _ZN11V90DemapperD1Ev _ZN20V90Phase4DemodulatorD1Ev _ZN12V90EqualizerD1Ev _ZN15V90TRN2DesignerD1Ev _ZN24V90ConstellationDesignerD1Ev"
WEAKARGS=""
for f in $CALLEES; do WEAKARGS="$WEAKARGS --weaken-symbol=$f"; done
objcopy $WEAKARGS $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF $LD32 -o ${T}_test_blob test/vpcm/v90_demodulator.main.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_dsplibs_weak.o

g++ -m32 $CXXF -O0 -g -c $PORT_P44 -o ${T}_port_o0.o
gcc $HCF $LD32 -o ${T}_test_port test/vpcm/v90_demodulator.main.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o0.o
if [ -z "$NOO2" ]; then
  g++ -m32 $CXXF -O2 -g -c $PORT_P44 -o ${T}_port_o2.o
  gcc $HCF $LD32 -o ${T}_test_port_o2 test/vpcm/v90_demodulator.main.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o2.o
fi

# symbol check: the function exists, with the same name and binding / section class, in the port object
DEFAULT_SYMRE='_ZN14V90Demodulator(C[12]E|D[12]E|18sessionTermination|11enterPhase[34]|20enterDataSteadyState|8enterRRN|8enterFPE|14setSessionFlag|10exitPhase3|14enterDataPhase|6reInit|5reset|24enterChannelVerification)[A-Za-z0-9_]*|_ZNK14V90Demodulator(10getBitRate|13getRbsPattern|23indicateRemoteRateReneg|8getAT_UD)[A-Za-z0-9_]*'
SYMRE="${P44_SYMRE:-$DEFAULT_SYMRE}"
BLOBSYMS=$(nm --defined-only $BLOB_O | awk -v re="^($SYMRE)\$" '$3 ~ re {print $2" "$3}' | sort)
PORTSYMS=$(nm --defined-only ${T}_port_o2.o 2>/dev/null || nm --defined-only ${T}_port_o0.o)
MISSING=$(echo "$BLOBSYMS" | while read b s; do echo "$PORTSYMS" | grep -q " $b $s\$" || echo "$b $s"; done)
NBLOB=$(echo "$BLOBSYMS" | wc -l)
if [ -n "$MISSING" ]; then echo "SYMBOL MISMATCH (blob symbol missing / different binding in the port object):"; echo "$MISSING"; exit 1; fi
echo "SYMBOLS: all $NBLOB blob symbols (name + binding) are defined by the port object"

if [ -z "$NO64" ]; then
  g++ -m64 $CXXF -O2 $FP64_FLAGS -c $PORT_P44 -o ${T}_port64.o
  P64=""
  for c in p07 p09 p11 p17 p18 p24 p26 p28 p34 p37 p40 p43 p44; do g++ -m64 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe64_$c.o; P64="$P64 ${T}_probe64_$c.o"; done
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 $SRENAME -c test/stub_env.c -o ${T}_stub_env64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -no-pie -o ${T}_test_port64 test/vpcm/v90_demodulator.main.main.c ${T}_harness64.o ${T}_stub_env64.o $P64 ${T}_port64.o
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
if [ -n "$GOLDEN_UPDATE" ]; then mkdir -p test/golden; cp ${T}_blob.out test/golden/vpcm/v90_demodulator.main.blob32.out; echo "golden updated"; fi
if [ -z "$TPREFIX" ] && [ "$SCENARIOS" = "$ALLSC" ] && [ -f test/golden/vpcm/v90_demodulator.main.blob32.out ] && [ -z "$P44_SEED" ] && [ -z "$P44_TRIALS" ]; then
  if diff -u test/golden/vpcm/v90_demodulator.main.blob32.out ${T}_blob.out; then echo "GOLDEN: the 32-bit blob run equals test/golden/vpcm/v90_demodulator.main.blob32.out"
  else echo "STALE GOLDEN: the blob output differs from test/golden/vpcm/v90_demodulator.main.blob32.out (GOLDEN_UPDATE=1 rewrites it)"; exit 1; fi
fi
echo "MATCH: P44 C++ port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')$([ -z "$NO64" ] && echo " + x86-64 [$FP64_FLAGS] vs the 32-bit blob output")) is behaviorally identical to dsplibs.o for this test"
