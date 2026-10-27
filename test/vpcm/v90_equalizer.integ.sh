#!/bin/sh
# P42 INTEGRATION test: P40's process (and the whole V90Equalizer that P40's harness drives) through an equalizer built by the REAL P42 constructor, with the REAL P42 methods as callees:
# the P40 harness (test/vpcm/v90_equalizer_process.main.c, built with -DP42_INTEG) no longer interposes setLinearEquBeta / setDfeBeta / enterFPE / enterRRN / linearEquFadeEdges / calcMeanErrorStatistics (the blob's own
# functions run on the blob side, the P42 port's on the port side) and builds the equalizer with C1 (+ reset, hamming) and frees it with D1.  Still recorders: enterPhase4 / enterDataPhase (P41), the
# phase demodulators, the demapper, the resampler, V90ConnectionEvaluator, V90PreFilter::isV90WithEia6, edprintf.  hamming / mean / Std / Var are the blob's own on the blob side and the P03 port
# (src/dsp/dsp_window_fft_psd.cpp) on the port side.  The independent model of process cannot follow real callees (it scripts them): ONLY blob vs port is compared (P40_NOMODEL behaviour); the process
# port is the P40 port (src/vpcm/v90_equalizer_process.cpp).  Outputs: -O0, -O2, x86-64 vs the 32-bit blob run; own golden test/golden/vpcm/v90_equalizer.integ.blob32.integ.out.
# Usage: sh test/vpcm/v90_equalizer.integ.sh   (from slmodemd/dsplibs-port/);  GOLDEN_UPDATE=1 rewrites the golden;  NO64=1 / NOO2=1 skip;  P42I_TRIALS trials per scenario (default 3000)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p05) echo p05;; p07) echo vpcm/pcm_parameters;; p17) echo vpcm/v90_connection_evaluator;; p21) echo vpcm/v90_spectral_shaper;; p28) echo vpcm/v90_mapper;; p34) echo vpcm/v90_phase3_demod_v92_decision;; p37) echo vpcm/v90_phase4_demod_decisions;; p40) echo vpcm/v90_equalizer_process;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/vpcm/v90_equalizer.integ}"
mkdir -p "$(dirname "$T")"
CF="-m32 -Wall -Wextra -Wno-unused-function -O0 -g"
HCF="-m32 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -O2 -g -DP42_INTEG"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc"
LD32="-no-pie"
SCENARIOS="${SCENARIOS:-0 1 2}"
case "$T" in /*) R="";; *) R="./";; esac
TR="${P42I_TRIALS:-3000}"
export P40_NOCOV=1   # the model coverage sites of the P40 harness need 8000 trials; here only blob vs port is compared
SRENAME="-Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf"
gcc $CF $SRENAME -c test/stub_env.c -o ${T}_stub_env.o
PROBES=""
for c in p05 p07 p17 p21 p28 p34 p37 p40; do g++ -m32 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
gcc -m32 -Wall -Wextra -Wno-unused-function -O2 -g -c test/harness/harness.c -o ${T}_harness.o

CALLEES="_ZN12V90Equalizer11enterPhase4Ev _ZN12V90Equalizer14enterDataPhaseEv _ZN20V90Phase3Demodulator11getDecisionEf _ZN20V90Phase4Demodulator11getDecisionEs
 _ZN20V90Phase4Demodulator9detectRRNEs _ZN20V90Phase4Demodulator9detectFPEEs _ZN11V90Demapper12hardDecisionEs _ZN11V90Demapper18linearMappingStudyEss _ZN15ResamplerTiming13SdHalfBaudDftEf
 _ZN15ResamplerTiming18resetSdHalfBaudDftEv _ZN21ResamplerTimingOffset15setTimingOffsetEf _ZN12V90Resampler11setBllStateE11V90BllStatej _ZN22V90ConnectionEvaluator14updateAvePdsnrEfj
 _ZNK12V90PreFilter13isV90WithEia6Ev edprintf"
WEAKARGS=""
for f in $CALLEES; do WEAKARGS="$WEAKARGS --weaken-symbol=$f"; done
objcopy $WEAKARGS $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF $LD32 -o ${T}_test_blob test/vpcm/v90_equalizer_process.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_dsplibs_weak.o

PORTS="src/vpcm/v90_equalizer_process.cpp src/vpcm/v90_equalizer.cpp src/dsp/dsp_window_fft_psd.cpp"
g++ -m32 $CXXF -O0 -g -c src/vpcm/v90_equalizer_process.cpp -o ${T}_a0.o; g++ -m32 $CXXF -O0 -g -c src/vpcm/v90_equalizer.cpp -o ${T}_b0.o; g++ -m32 $CXXF -O0 -g -c src/dsp/dsp_window_fft_psd.cpp -o ${T}_c0.o
gcc $HCF $LD32 -o ${T}_test_port test/vpcm/v90_equalizer_process.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_a0.o ${T}_b0.o ${T}_c0.o
if [ -z "$NOO2" ]; then
  g++ -m32 $CXXF -O2 -g -c src/vpcm/v90_equalizer_process.cpp -o ${T}_a2.o; g++ -m32 $CXXF -O2 -g -c src/vpcm/v90_equalizer.cpp -o ${T}_b2.o; g++ -m32 $CXXF -O2 -g -c src/dsp/dsp_window_fft_psd.cpp -o ${T}_c2.o
  gcc $HCF $LD32 -o ${T}_test_port_o2 test/vpcm/v90_equalizer_process.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_a2.o ${T}_b2.o ${T}_c2.o
fi
if [ -z "$NO64" ]; then
  P64=""
  for c in p05 p07 p17 p21 p28 p34 p37 p40; do g++ -m64 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe64_$c.o; P64="$P64 ${T}_probe64_$c.o"; done
  for f in src/vpcm/v90_equalizer_process src/vpcm/v90_equalizer src/dsp/dsp_window_fft_psd; do g++ -m64 $CXXF -O2 $FP64_FLAGS -c $f.cpp -o ${T}_x64_${f##*/}.o; done
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 $SRENAME -c test/stub_env.c -o ${T}_stub_env64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -DP42_INTEG -no-pie -o ${T}_test_port64 test/vpcm/v90_equalizer_process.main.c ${T}_harness64.o ${T}_stub_env64.o $P64 ${T}_x64_v90_equalizer_process.o ${T}_x64_v90_equalizer.o ${T}_x64_dsp_window_fft_psd.o
fi
rm -f ${T}_s*.out
rc=0
for i in $SCENARIOS; do
  P40_TRIALS=$TR ${R}${T}_test_blob "$i" > ${T}_s$i.blob.out || rc=1
  P40_TRIALS=$TR ${R}${T}_test_port "$i" > ${T}_s$i.port.out || rc=1
  if [ -z "$NOO2" ]; then P40_TRIALS=$TR ${R}${T}_test_port_o2 "$i" > ${T}_s$i.o2.out || rc=1; fi
  if [ -z "$NO64" ]; then P40_TRIALS=$TR ${R}${T}_test_port64 "$i" > ${T}_s$i.p64.out || rc=1; fi
done
: > ${T}_blob.out; : > ${T}_port.out; : > ${T}_port_o2.out; : > ${T}_port64.out
for i in $SCENARIOS; do
  cat ${T}_s$i.blob.out >> ${T}_blob.out; cat ${T}_s$i.port.out >> ${T}_port.out
  [ -z "$NOO2" ] && cat ${T}_s$i.o2.out >> ${T}_port_o2.out
  [ -z "$NO64" ] && cat ${T}_s$i.p64.out >> ${T}_port64.out
done
rm -f ${T}_s*.out
if [ $rc -ne 0 ]; then echo "SCENARIO FAILURE (non-zero exit)"; exit 1; fi
diff -u ${T}_blob.out ${T}_port.out
if [ -z "$NOO2" ]; then diff -u ${T}_blob.out ${T}_port_o2.out; fi
if [ -z "$NO64" ]; then diff -u ${T}_blob.out ${T}_port64.out; fi
if [ -n "$GOLDEN_UPDATE" ]; then cp ${T}_blob.out test/golden/vpcm/v90_equalizer.integ.blob32.integ.out; echo "golden updated"; fi
if [ -z "$TPREFIX" ] && [ "$SCENARIOS" = "0 1 2" ] && [ -f test/golden/vpcm/v90_equalizer.integ.blob32.integ.out ] && [ -z "$P42I_TRIALS" ]; then
  if diff -u test/golden/vpcm/v90_equalizer.integ.blob32.integ.out ${T}_blob.out; then echo "GOLDEN: the 32-bit blob run equals test/golden/vpcm/v90_equalizer.integ.blob32.integ.out"
  else echo "STALE GOLDEN: the blob output differs from test/golden/vpcm/v90_equalizer.integ.blob32.integ.out (GOLDEN_UPDATE=1 rewrites it)"; exit 1; fi
fi
echo "MATCH: P42 integration (P40 process + real P42 constructor / methods + P03 helpers, -O0 and -O2$([ -z "$NO64" ] && echo ' + x86-64')) is behaviorally identical to dsplibs.o for this test"
