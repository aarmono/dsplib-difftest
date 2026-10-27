#!/bin/sh
# Shared build / run / diff script of the three rows of class V90AutoDigitalImpDetector (ROW = p20, p19 or p18; use test/vpcm/v90_adid_study.sh, p19, p18).
# Builds the differential test (ONE harness source, test/vpcm/v90_adid.main.c; the port is REAL C++ compiled with -fno-exceptions -fno-rtti) and runs every scenario of the row in its OWN process:
#   blob      test/vpcm/v90_adid.main.c + a copy of dsplibs.o in which `edprintf` is WEAK (the harness' recorders define edprintf and dsplibs_debug_printf)   (gcc -m32 -no-pie)
#   port -O0  the harness + the port objects of the row (LINKED WITH gcc: no libstdc++): the row's own port file (under test, src/vpcm/v90_adid.cpp, v90_adid_pad_gain.cpp or
#             v90_adid_study.cpp) + the port files of the EARLIER rows and P06 (G.711), which are cached objects rebuilt only when their sources are newer
#   port -O2  same at -O2
#   port -m64 (NO64=1 skips) the same harness and the port compiled for x86-64 (SSE float default; FP64_FLAGS="-mfpmath=387" for the x87 variant); its output is diffed against the
#             32-bit BLOB's output (the golden dump)
# The staged rows link with unresolved symbols allowed: the harness refers to the functions of later rows through WEAK declarations that its row's scenarios never call.
# A scenario exits 2 if its own coverage counters fail, 3 if a harness assumption / model is violated; either fails this script.  Also checked: every blob symbol (name + binding) of the row's
# functions is defined by the row's port object; the harness mirror (tools/layout_gen.py from test/vpcm/v90_adid.layout, test/vpcm/pcm_parameters.layout) equals the port header's real class layout.
# "MATCH" is only printed if every scenario matched AND reached its branches.
# Env: ROW, PORT_P20 / PORT_P19 / PORT_P18 (port sources; the row's own is the one under test: mutation testing), SCENARIOS, NOO2=1, NO64=1, TPREFIX (output prefix, parallel mutation runs),
# ADID_SEED (alternative input stream), GOLDEN_UPDATE=1 (rewrite test/golden/<row>_blob32.out), FP64_FLAGS (extra flags of the x86-64 port build).
# Usage: sh test/vpcm/v90_adid_study.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p07) echo vpcm/pcm_parameters;; p18) echo vpcm/v90_adid;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

ROW="${ROW:?ROW=p20|p19|p18}"
PORT_P20="${PORT_P20:-src/vpcm/v90_adid.cpp}"
PORT_P19="${PORT_P19:-src/vpcm/v90_adid_pad_gain.cpp}"
PORT_P18="${PORT_P18:-src/vpcm/v90_adid_study.cpp}"
PORT_P06="${PORT_P06:-src/vpcm/pcm_leaf_classes.cpp}"
P20_SYMS='C1EP|C2EP|D1Ev|D2Ev|18clearCamulativeValE|21clearCamulativeAltValE|17setConnectionTypeE|5resetE|18resetLinearMappingE|21isThereAnyAltRbsPhaseE|8isAltRbsE|28calculateLinearMeanAndVarAltE|21unitePhasesInfoOfUrefE|23updateLinMappMeanAndVarE|10updateUrefE|26updateLinMappMeanAndVarAltE|13updateUrefAltE|35uniteLinMappInfoOfUnsuspectedPhasesE|30unSuspectedPhaseNearestLinMappE|26addReceivedSampleToStorageE|25calculateLinearMeanAndVarE|21applyPadGainToLinMappE|16setMaxUcodeArrayE|27setPrevSessionLinearMappingE|24adjustUinfoToPhaseOffsetE'
P19_SYMS='15getAltVarThreshE|22updateAltRbsPhaseInDilE|11findPadGainE|17determineMaxUcodeE|18setQcLinearMappingE'
P18_SYMS='16studyUrefHandlerE|21resetStudyUrefHandlerE|17porcessFirstStudyE|18porcessSecondStudyE'
SC20="0 1 2 3"
SC19="4 5 6 7 8"
SC18="9 10 11 12 13"
case "$ROW" in
  p20) OWN="$PORT_P20"; OWNSYMS="$P20_SYMS"; ALLSC="$SC20"; EARLIER="" ;;
  p19) OWN="$PORT_P19"; OWNSYMS="$P19_SYMS"; ALLSC="$SC20 $SC19"; EARLIER="p20" ;;
  p18) OWN="$PORT_P18"; OWNSYMS="$P18_SYMS"; ALLSC="$SC20 $SC19 $SC18"; EARLIER="p20 p19" ;;
  *) echo "unknown ROW $ROW"; exit 1 ;;
esac
T="${TPREFIX:-test/.build/vpcm/v90_adid.$ROW}"
mkdir -p "$(dirname "$T")"
case "$ROW" in p18) GS=v90_adid;; p19) GS=v90_adid_pad_gain;; p20) GS=v90_adid_study;; esac   # stem of the row's job (its golden test/golden/vpcm/$GS.blob32.out)
SCENARIOS="${SCENARIOS:-$ALLSC}"
CF="-m32 -Wall -Wextra -Wno-unused-function -O0 -g"
HCF="-m32 -Wall -Wextra -Wno-unused-function -O2 -g"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc"
LD32="-no-pie -Wl,--unresolved-symbols=ignore-all"
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"

SRENAME="-Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf"
gcc $CF $SRENAME -c test/stub_env.c -o ${T}_stub_env.o
for c in p07 p18; do g++ -m32 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; done
PROBES="${T}_probe_p07.o ${T}_probe_p18.o"
gcc $HCF -c test/harness/harness.c -o ${T}_harness.o     # the shared harness library (test/harness/, plain C)

objcopy --weaken-symbol=edprintf $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF $LD32 -o ${T}_test_blob test/vpcm/v90_adid.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_dsplibs_weak.o

# cached port objects of the other files (G.711 = P06, earlier rows): rebuilt only when their source or a header is newer / the flags changed
CACHE=test/.build/vpcm/v90_adid_cache
srcof() { case "$1" in p06) echo "$PORT_P06";; p20) echo "$PORT_P20";; p19) echo "$PORT_P19";; esac; }
cdep() { [ ! -f "$1" ] || [ "$2" -nt "$1" ] || [ src/vpcm/v90_adid.h -nt "$1" ] || [ src/core/dsp_cxx.h -nt "$1" ] || [ src/core/dsp_math.h -nt "$1" ]; }
CO0=""; CO2=""; C64=""
for c in p06 $EARLIER; do
  s=$(srcof $c)
  cdep ${CACHE}_${c}_o0.o $s && g++ -m32 $CXXF -O0 -g -c $s -o ${CACHE}_${c}_o0.o
  CO0="$CO0 ${CACHE}_${c}_o0.o"
  if [ -z "$NOO2" ]; then cdep ${CACHE}_${c}_o2.o $s && g++ -m32 $CXXF -O2 -g -c $s -o ${CACHE}_${c}_o2.o; CO2="$CO2 ${CACHE}_${c}_o2.o"; fi
  if [ -z "$NO64" ]; then
    if [ ! -f ${CACHE}_${c}_64.o ] || [ "$FP64_FLAGS" != "$(cat ${CACHE}_${c}_64.flags 2>/dev/null)" ] || cdep ${CACHE}_${c}_64.o $s; then g++ -m64 $CXXF -O2 $FP64_FLAGS -c $s -o ${CACHE}_${c}_64.o; echo "$FP64_FLAGS" > ${CACHE}_${c}_64.flags; fi
    C64="$C64 ${CACHE}_${c}_64.o"
  fi
done

g++ -m32 $CXXF -O0 -g -c $OWN -o ${T}_port_o0.o
gcc $HCF $LD32 -o ${T}_test_port test/vpcm/v90_adid.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o0.o $CO0
if [ -z "$NOO2" ]; then
  g++ -m32 $CXXF -O2 -g -c $OWN -o ${T}_port_o2.o
  gcc $HCF $LD32 -o ${T}_test_port_o2 test/vpcm/v90_adid.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o2.o $CO2
fi

# symbol check: every function symbol of the row exists, with the same name and binding / section class, in the row's port object
SYMRE="_ZN25V90AutoDigitalImpDetector($OWNSYMS)"
BLOBSYMS=$(nm --defined-only $BLOB_O | awk -v re="^($SYMRE)" '$3 ~ re {print $2" "$3}' | sort)
PORTSYMS=$(nm --defined-only ${T}_port_o2.o 2>/dev/null || nm --defined-only ${T}_port_o0.o)
MISSING=$(echo "$BLOBSYMS" | while read b s; do echo "$PORTSYMS" | grep -q " $b $s\$" || echo "$b $s"; done)
NBLOB=$(echo "$BLOBSYMS" | wc -l)
if [ -n "$MISSING" ]; then echo "SYMBOL MISMATCH (blob symbol missing / different binding in the port object):"; echo "$MISSING"; exit 1; fi
echo "SYMBOLS: all $NBLOB blob symbols (name + binding) of $ROW are defined by the port object"

if [ -z "$NO64" ]; then
  g++ -m64 $CXXF -O2 $FP64_FLAGS -c $OWN -o ${T}_port64.o
  P64=""
  for c in p07 p18; do g++ -m64 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe64_$c.o; P64="$P64 ${T}_probe64_$c.o"; done
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 $SRENAME -c test/stub_env.c -o ${T}_stub_env64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -no-pie -Wl,--unresolved-symbols=ignore-all -o ${T}_test_port64 test/vpcm/v90_adid.main.c ${T}_harness64.o ${T}_stub_env64.o $P64 ${T}_port64.o $C64
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
if [ -n "$GOLDEN_UPDATE" ]; then mkdir -p test/golden/vpcm; cp ${T}_blob.out test/golden/vpcm/${GS}.blob32.out; echo "golden updated"; fi
if [ -z "$TPREFIX" ] && [ "$SCENARIOS" = "$ALLSC" ] && [ -f test/golden/vpcm/${GS}.blob32.out ] && [ -z "$ADID_SEED" ]; then
  if diff -u test/golden/vpcm/${GS}.blob32.out ${T}_blob.out; then echo "GOLDEN: the 32-bit blob run equals test/golden/vpcm/${GS}.blob32.out"
  else echo "STALE GOLDEN: the blob output differs from test/golden/vpcm/${GS}.blob32.out (GOLDEN_UPDATE=1 rewrites it)"; exit 1; fi
fi
echo "MATCH: $ROW C++ port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')$([ -z "$NO64" ] && echo " + x86-64 [$FP64_FLAGS] vs the 32-bit blob output")) is behaviorally identical to dsplibs.o for this test"
