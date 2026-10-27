#!/bin/sh
# Builds the differential test of port chunk P45 (V90Modem: C1 / C2, D1 / D2, printTitle, reset, setSessionFlag, progress; V92Modem: C1 / C2, D1 / D2, printTitle, reset, progress; 4,201 B;
# src/vpcm/pcm_modem.cpp, REAL C++ compiled with -fno-exceptions -fno-rtti) and runs every scenario in its OWN process.  Every callee (the sub-object constructors / destructors, the modulator / demodulator
# methods, setDilDescriptor, V92DILdescriptorPacker, V92create* / V92delete*) and edprintf are interposed by a harness recorder:
#   blob      test/vpcm/pcm_modem.main.c + a copy of dsplibs.o in which the callees are WEAK (gcc -m32 -no-pie); sysdep_malloc / sysdep_free are the harness' allocator
#   port -O0  the harness + the port object under test (the callees are the recorders: nothing else is linked)
#   port -O2  same at -O2
#   port -m64 (NO64=1 skips) the same harness and the port compiled for x86-64; its output is diffed against the 32-bit BLOB's output (the golden dump)
# The ONE harness source has no #ifdef around the code under test.  A scenario exits 2 if its own coverage sites fail, 3 if a harness assumption / the independent model is violated; either fails this script.
# "MATCH" is only printed if every scenario matched AND reached its sites.
# Env: PORT_P45 (port source under test, mutation testing), SCENARIOS, NOO2=1, NO64=1, TPREFIX (output prefix), P45_SEED, P45_TRIALS, GOLDEN_UPDATE=1 (rewrite test/golden/vpcm/pcm_modem.blob32.out), FP64_FLAGS.
# Usage: sh test/vpcm/pcm_modem.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p07) echo vpcm/pcm_parameters;; p45) echo vpcm/pcm_modem;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_P45="${PORT_P45:-src/vpcm/pcm_modem.cpp}"
T="${TPREFIX:-test/.build/vpcm/pcm_modem}"
mkdir -p "$(dirname "$T")"
CF="-m32 -Wall -Wextra -Wno-unused-function -O0 -g"
HCF="-m32 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -O2 -g"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc"
LD32="-no-pie"
ALLSC="0 1 2 3 4 5 6 7"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7}"
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"

SRENAME="-Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf"
gcc $CF $SRENAME -c test/stub_env.c -o ${T}_stub_env.o
PROBES=""
for c in p07 p45; do g++ -m32 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
g++ -m32 $CXXF -O2 -c test/vpcm/pcm_modem.sizes.cpp -o ${T}_sizes.o; PROBES="$PROBES ${T}_sizes.o"
gcc $HCF -Dsysdep_malloc=h_sysdep_malloc -Dsysdep_free=h_sysdep_free -c test/harness/harness.c -o ${T}_harness.o     # the shared harness library (test/harness/, plain C)

CALLEES="edprintf _ZN5V90MPC1Ev _ZN5V90MPD1Ev _ZN5V90CPC1Ev _ZN5V90CPD1Ev _ZN13V90ParametersC1EP19_tagModemParameters _ZN13V90ParametersD1Ev _ZN13V90Phase2InfoC1EP13V90Parameters _ZN5V90JdC1EP13V90Parameters _ZN5V90JdD1Ev _ZN5V92JdC1EP13V90Parameters _ZN5V92JdD1Ev _ZN12V90ModulatorC1EjP13V90Phase2InfoP5V90JdP5V92JdP19tagV90DILdescriptorP16V90MappingParamsS9_P22tagV90AdditionalCPinfoP5V90CPP5V90MPP13V90Parametersj _ZN12V90ModulatorD1Ev _ZN12V90Modulator14setSessionFlagEj _ZN12V90Modulator5resetEv _ZN12V90Modulator8progressEPiRjPfj _ZN14V90DemodulatorC1EjP13V90Phase2InfoP5V90JdP5V92JdP19tagV90DILdescriptorP16V90MappingParamsS9_P22tagV90AdditionalCPinfoP5V90CPP5V90MP23__tHardwareCodecTypes__P13V90Parameters20V90ComputationalModej _ZN14V90DemodulatorD1Ev _ZN14V90Demodulator14setSessionFlagEj _ZN14V90Demodulator5resetEj _ZN14V90Demodulator8progressEPiRjPfj _Z16setDilDescriptorP19tagV90DILdescriptor7DilType _ZN13V92ParametersC1EP19_tagModemParameters _ZN13V92ParametersD1Ev _ZN13V92Phase2InfoC1EP13V92Parameters _ZN5V92CPC1Ev _ZN5V92CPD1Ev V92createConstellations V92createFilterCoefficients V92deleteConstellations V92deleteFilterCoefficients _ZN12V92ModulatorC1EjP13V92Phase2InfoP5V92JaP19tagV90DILdescriptorP5V92CPP16V92MappingParamsP13V92Parameters _ZN12V92ModulatorD1Ev _ZN12V92Modulator8progressEPiRjPfj _ZN12V92Modulator5resetEv _ZN12V92Modulator11enterPhase3Ev _Z22V92DILdescriptorPackerP19tagV90DILdescriptorPhPi"
WEAKARGS=""
for f in $CALLEES; do WEAKARGS="$WEAKARGS --weaken-symbol=$f"; done
objcopy $WEAKARGS $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF $LD32 -o ${T}_test_blob test/vpcm/pcm_modem.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_dsplibs_weak.o

g++ -m32 $CXXF -O0 -g -c $PORT_P45 -o ${T}_port_o0.o
gcc $HCF $LD32 -o ${T}_test_port test/vpcm/pcm_modem.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o0.o
if [ -z "$NOO2" ]; then
  g++ -m32 $CXXF -O2 -g -c $PORT_P45 -o ${T}_port_o2.o
  gcc $HCF $LD32 -o ${T}_test_port_o2 test/vpcm/pcm_modem.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o2.o
fi

# symbol check: the function exists, with the same name and binding / section class, in the port object
DEFAULT_SYMRE='_ZN8V9[02]Modem[A-Za-z0-9_]*'
SYMRE="${P45_SYMRE:-$DEFAULT_SYMRE}"
BLOBSYMS=$(nm --defined-only $BLOB_O | awk -v re="^($SYMRE)\$" '$3 ~ re {print $2" "$3}' | sort)
PORTSYMS=$(nm --defined-only ${T}_port_o2.o 2>/dev/null || nm --defined-only ${T}_port_o0.o)
MISSING=$(echo "$BLOBSYMS" | while read b s; do echo "$PORTSYMS" | grep -q " $b $s\$" || echo "$b $s"; done)
NBLOB=$(echo "$BLOBSYMS" | wc -l)
if [ -n "$MISSING" ]; then echo "SYMBOL MISMATCH (blob symbol missing / different binding in the port object):"; echo "$MISSING"; exit 1; fi
echo "SYMBOLS: all $NBLOB blob symbols (name + binding) are defined by the port object"

if [ -z "$NO64" ]; then
  g++ -m64 $CXXF -O2 $FP64_FLAGS -c $PORT_P45 -o ${T}_port64.o
  P64=""
  for c in p07 p45; do g++ -m64 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe64_$c.o; P64="$P64 ${T}_probe64_$c.o"; done
  g++ -m64 $CXXF -O2 -c test/vpcm/pcm_modem.sizes.cpp -o ${T}_sizes64.o; P64="$P64 ${T}_sizes64.o"
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 $SRENAME -c test/stub_env.c -o ${T}_stub_env64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -Dsysdep_malloc=h_sysdep_malloc -Dsysdep_free=h_sysdep_free -c test/harness/harness.c -o ${T}_harness64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -no-pie -o ${T}_test_port64 test/vpcm/pcm_modem.main.c ${T}_harness64.o ${T}_stub_env64.o $P64 ${T}_port64.o
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
if [ -n "$GOLDEN_UPDATE" ]; then mkdir -p test/golden; cp ${T}_blob.out test/golden/vpcm/pcm_modem.blob32.out; echo "golden updated"; fi
if [ -z "$TPREFIX" ] && [ "$SCENARIOS" = "$ALLSC" ] && [ -f test/golden/vpcm/pcm_modem.blob32.out ] && [ -z "$P45_SEED" ] && [ -z "$P45_TRIALS" ]; then
  if diff -u test/golden/vpcm/pcm_modem.blob32.out ${T}_blob.out; then echo "GOLDEN: the 32-bit blob run equals test/golden/vpcm/pcm_modem.blob32.out"
  else echo "STALE GOLDEN: the blob output differs from test/golden/vpcm/pcm_modem.blob32.out (GOLDEN_UPDATE=1 rewrites it)"; exit 1; fi
fi
echo "MATCH: P45 C++ port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')$([ -z "$NO64" ] && echo " + x86-64 [$FP64_FLAGS] vs the 32-bit blob output")) is behaviorally identical to dsplibs.o for this test"
