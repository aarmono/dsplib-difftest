#!/bin/sh
# Builds the differential test of port chunk P29 (classes V92Transmitter (6 functions), V90BitsToSymbol (11) and V92BitsToSymbol (10): the TX bits -> symbols front ends of the V.90 / V.92 modulators;
# src/vpcm/pcm_bits_to_symbol.cpp, REAL C++ compiled with -fno-exceptions -fno-rtti) and runs every scenario in its OWN process.  Every callee of the three classes is interposed by a harness recorder:
#   flavour A (scenarios 0, 1; harness compiled -DP29_BIN_A): V90Mapper (7 functions) and V92Transmitter (6 functions) are WEAK in a copy of dsplibs.o / overridden in the port link (-z muldefs, harness first)
#   flavour B (scenario 2): V90Mapper and the sub-objects V92ModulusEncoder / V92Precoder / V92PreFilter / V92ConvolutionEncoder (all 22 functions the transmitter calls) are WEAK; V92Transmitter is real
#   flavour C (scenario 3; -DP29_BIN_C): the transmitter over the REAL sub-objects (the blob's own; the cached P02 / P15 / P16 port objects in the port builds); only the V90Mapper recorders (link stubs) remain
#   blob      test/vpcm/pcm_bits_to_symbol.main.main.c + the weakened dsplibs.o copy (gcc -m32 -no-pie)
#   port -O0  the harness + the port object under test
#   port -O2  same at -O2
#   port -m64 (NO64=1 skips) the same harness and the port compiled for x86-64 (SSE float default; FP64_FLAGS="-mfpmath=387" for the x87 variant); its output is diffed against the 32-bit BLOB's output
# The ONE harness source has no #ifdef around the code under test: the functions are reached through __asm__ labels with the mangled names (the flavour macro only selects the recorder set / scenarios).  A scenario
# exits 2 if its own coverage counters fail, 3 if a harness assumption / independent model is violated; either fails this script.  Also checked: the blob symbols (name + binding) are defined by the port object; the
# harness mirrors (tools/layout_gen.py from test/vpcm/pcm_bits_to_symbol.layout, p11) equal the port headers' real class layouts (layout probes).  "MATCH" is only printed if every scenario matched AND reached its branches.
# Env: PORT_P29 (port source under test, mutation testing), SCENARIOS, NOO2=1, NO64=1, TPREFIX (output prefix, parallel mutation runs), P29_SEED (alternative input stream), GOLDEN_UPDATE=1 (rewrite
# test/golden/vpcm/pcm_bits_to_symbol.main.blob32.out from this blob run), FP64_FLAGS (extra flags of the x86-64 port build).
# Usage: sh test/vpcm/pcm_bits_to_symbol.main.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p11) echo vpcm/v92_cp;; p29) echo vpcm/pcm_bits_to_symbol;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_P29="${PORT_P29:-src/vpcm/pcm_bits_to_symbol.cpp}"
T="${TPREFIX:-test/.build/vpcm/pcm_bits_to_symbol.main}"
mkdir -p "$(dirname "$T")"
CF="-m32 -Wall -Wextra -Wno-unused-function -O0 -g"
HCF="-m32 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -O2 -g"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc"
LD32="-no-pie"
ALLSC="0 1 2 3"
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"

SRENAME="-Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf"
gcc $CF $SRENAME -c test/stub_env.c -o ${T}_stub_env.o
PROBES=""
for c in p11 p29; do g++ -m32 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
gcc $HCF -c test/harness/harness.c -o ${T}_harness.o     # the shared harness library (test/harness/, plain C)

MAPPER="_ZN9V90MapperC1EP13V90Parameters _ZN9V90MapperC2EP13V90Parameters _ZN9V90MapperD1Ev _ZN9V90MapperD2Ev _ZN9V90Mapper5resetEP16V90MappingParams7PcmType _ZN9V90Mapper15resetNoSpectralEP16V90MappingParams7PcmType _ZN9V90Mapper7processEPhjPsRj"
TX="_ZN14V92TransmitterC1Ev _ZN14V92TransmitterC2Ev _ZN14V92TransmitterD1Ev _ZN14V92TransmitterD2Ev _ZN14V92Transmitter5resetEP16V92MappingParams _ZN14V92Transmitter7processEPhjPsRj"
SUB="_ZN17V92ModulusEncoderC1Ev _ZN17V92ModulusEncoderC2Ev _ZN17V92ModulusEncoder5resetEP16V92MappingParams _ZN17V92ModulusEncoder8progressEPhPj
 _ZN11V92PrecoderC1Ej _ZN11V92PrecoderC2Ej _ZN11V92PrecoderD1Ev _ZN11V92PrecoderD2Ev _ZN11V92Precoder5resetEP16V92MappingParams _ZN11V92Precoder15setCoefficientsEPfS0_jj _ZN11V92Precoder7processEPjiiPiPf
 _ZN12V92PreFilterC1Ej _ZN12V92PreFilterC2Ej _ZN12V92PreFilterD1Ev _ZN12V92PreFilterD2Ev _ZN12V92PreFilter5resetEv _ZN12V92PreFilter15setCoefficientsEPfS0_jj _ZN12V92PreFilter7processEPfS0_
 _ZN21V92ConvolutionEncoderC1Ev _ZN21V92ConvolutionEncoderC2Ev _ZN21V92ConvolutionEncoderD1Ev _ZN21V92ConvolutionEncoderD2Ev _ZN21V92ConvolutionEncoder5resetEi _ZN21V92ConvolutionEncoder7processEPi"
wk() { for f in $*; do printf -- "--weaken-symbol=%s " $f; done; }
objcopy $(wk $MAPPER $TX $SUB) $BLOB_O ${T}_dsplibs_weakA.o
objcopy $(wk $MAPPER $SUB) $BLOB_O ${T}_dsplibs_weakB.o
HARN="${T}_harness.o ${T}_stub_env.o"
# flavours: P29_FLAV=A (scenarios 0, 1) or B (scenario 2) builds ONE flavour with the plain names *_test_blob / *_test_port / *_test_port_o2 / *_test_port64 (the convention of tools/mutate.py: test/vpcm/pcm_bits_to_symbol.a.sh /
# p29b.sh are the two wrappers); without it both flavours are built (suffix A / B) and all three scenarios run.  The port object is NOT weakened: the harness objects come first on the link line and
# -Wl,-z,muldefs makes the first definition win, so the recorders replace the port's own V92Transmitter symbols in flavour A.
FLAVS="${P29_FLAV:-A B C}"
fsuf() { if [ -n "$P29_FLAV" ]; then echo ""; else echo "$1"; fi; }
if [ -z "${SCENARIOS+x}" ]; then
  case "$P29_FLAV" in A) SCENARIOS="0 1";; B) SCENARIOS="2";; C) SCENARIOS="3";; *) SCENARIOS="0 1 2 3";; esac
fi
MD="-Wl,-z,muldefs"
# flavour C (scenario 3: V92Transmitter over the REAL sub-objects): the port builds link the P02 / P15 / P16 port objects (cached; rebuilt when a source or header is newer), the blob is dsplibs.o unchanged (the harness' V90Mapper recorders
# only satisfy the port link: -z muldefs lets them win over the blob's own copies, which the BitsToSymbol classes are not run against in this flavour)
CACHE=test/.build/vpcm/pcm_bits_to_symbol.main_cache
cdep() { [ ! -f "$1" ] || [ "$2" -nt "$1" ] || [ -n "$(find src/ -name '*.h' -newer "$1" | head -1)" ]; }
CO0=""; CO2=""; C64=""
if echo " $FLAVS " | grep -q " C "; then
  PROBES="$PROBES ${T}_probe_p16.o"
  g++ -m32 $CXXF -O2 -c test/vpcm/v92_tx_coding_layout_probe.cpp -o ${T}_probe_p16.o
  for c in src/dsp/dsp_float_filters src/vpcm/v92_modulus_encoder src/vpcm/v92_tx_coding; do
    cdep ${CACHE}_${c##*/}_o0.o $c.cpp && g++ -m32 $CXXF -O0 -g -c $c.cpp -o ${CACHE}_${c##*/}_o0.o
    CO0="$CO0 ${CACHE}_${c##*/}_o0.o"
    if [ -z "$NOO2" ]; then cdep ${CACHE}_${c##*/}_o2.o $c.cpp && g++ -m32 $CXXF -O2 -g -c $c.cpp -o ${CACHE}_${c##*/}_o2.o; CO2="$CO2 ${CACHE}_${c##*/}_o2.o"; fi
    if [ -z "$NO64" ]; then
      if [ ! -f ${CACHE}_${c##*/}_64.o ] || [ "$FP64_FLAGS" != "$(cat ${CACHE}_${c##*/}_64.flags 2>/dev/null)" ] || cdep ${CACHE}_${c##*/}_64.o $c.cpp; then g++ -m64 $CXXF -O2 $FP64_FLAGS -c $c.cpp -o ${CACHE}_${c##*/}_64.o; echo "$FP64_FLAGS" > ${CACHE}_${c##*/}_64.flags; fi
      C64="$C64 ${CACHE}_${c##*/}_64.o"
    fi
  done
fi
g++ -m32 $CXXF -O0 -g -c $PORT_P29 -o ${T}_port_o0.o
[ -z "$NOO2" ] && g++ -m32 $CXXF -O2 -g -c $PORT_P29 -o ${T}_port_o2.o
for F in $FLAVS; do
  S=$(fsuf $F)
  X0=""; X2=""; BM=$MD
  case $F in A) DEF="-DP29_BIN_A"; WB=${T}_dsplibs_weakA.o;; B) DEF=""; WB=${T}_dsplibs_weakB.o;; C) DEF="-DP29_BIN_C"; WB=$BLOB_O; X0="$CO0"; X2="$CO2";; esac
  [ $F = C ] || BM=""
  gcc $HCF $DEF -c test/vpcm/pcm_bits_to_symbol.main.main.c -o ${T}_main$F.o
  gcc $HCF $LD32 $BM -o ${T}_test_blob$S ${T}_main$F.o $HARN $PROBES $WB
  gcc $HCF $LD32 $MD -o ${T}_test_port$S ${T}_main$F.o $HARN $PROBES ${T}_port_o0.o $X0
  if [ -z "$NOO2" ]; then
    gcc $HCF $LD32 $MD -o ${T}_test_port${S}_o2 ${T}_main$F.o $HARN $PROBES ${T}_port_o2.o $X2
  fi
done

# symbol check: every function symbol of the three classes exists, with the same name and binding / section class, in the port object
DEFAULT_SYMRE='_ZN(14V92Transmitter|15V90BitsToSymbol|15V92BitsToSymbol)'
SYMRE="${P29_SYMRE:-$DEFAULT_SYMRE}"
BLOBSYMS=$(nm --defined-only $BLOB_O | awk -v re="^($SYMRE)" '$3 ~ re {print $2" "$3}' | sort)
PORTSYMS=$(nm --defined-only ${T}_port_o2.o 2>/dev/null || nm --defined-only ${T}_port_o0.o)
MISSING=$(echo "$BLOBSYMS" | while read b s; do echo "$PORTSYMS" | grep -q " $b $s\$" || echo "$b $s"; done)
NBLOB=$(echo "$BLOBSYMS" | wc -l)
if [ -n "$MISSING" ]; then echo "SYMBOL MISMATCH (blob symbol missing / different binding in the port object):"; echo "$MISSING"; exit 1; fi
echo "SYMBOLS: all $NBLOB blob symbols (name + binding) are defined by the port object"

if [ -z "$NO64" ]; then
  g++ -m64 $CXXF -O2 $FP64_FLAGS -c $PORT_P29 -o ${T}_port64.o
  P64=""
  for c in p11 p29; do g++ -m64 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe64_$c.o; P64="$P64 ${T}_probe64_$c.o"; done
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 $SRENAME -c test/stub_env.c -o ${T}_stub_env64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness64.o
  W64="-Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2"
  for F in $FLAVS; do
    S=$(fsuf $F)
    X6=""; PR="$P64"
    case $F in A) DEF="-DP29_BIN_A";; B) DEF="";; C) DEF="-DP29_BIN_C"; X6="$C64"; g++ -m64 $CXXF -O2 -c test/vpcm/v92_tx_coding_layout_probe.cpp -o ${T}_probe64_p16.o; PR="$P64 ${T}_probe64_p16.o";; esac
    gcc -m64 $W64 $DEF -c test/vpcm/pcm_bits_to_symbol.main.main.c -o ${T}_main${F}64.o
    gcc -m64 -O2 -no-pie $MD -o ${T}_test_port64$S ${T}_main${F}64.o ${T}_harness64.o ${T}_stub_env64.o $PR ${T}_port64.o $X6
  done
fi

flav() { if [ -n "$P29_FLAV" ]; then echo ""; else case $1 in 3) echo C;; 2) echo B;; *) echo A;; esac; fi; }
# scenarios are independent processes: run them concurrently, then concatenate in order
rm -f ${T}_s*.out
rc=0
pids=""
for i in $SCENARIOS; do
  F=$(flav $i)
  ( ${R}${T}_test_blob$F "$i" > ${T}_s$i.blob.out ) & pids="$pids $!"
  ( ${R}${T}_test_port$F "$i" > ${T}_s$i.port.out ) & pids="$pids $!"
  if [ -z "$NOO2" ]; then ( ${R}${T}_test_port${F}_o2 "$i" > ${T}_s$i.o2.out ) & pids="$pids $!"; fi
  if [ -z "$NO64" ]; then ( ${R}${T}_test_port64$F "$i" > ${T}_s$i.p64.out ) & pids="$pids $!"; fi
  for p in $pids; do wait $p || rc=1; done; pids=""
done
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
if [ -n "$GOLDEN_UPDATE" ]; then mkdir -p test/golden; cp ${T}_blob.out test/golden/vpcm/pcm_bits_to_symbol.main.blob32.out; echo "golden updated"; fi
if [ -z "$TPREFIX" ] && [ "$SCENARIOS" = "$ALLSC" ] && [ -z "$P29_FLAV" ] && [ -f test/golden/vpcm/pcm_bits_to_symbol.main.blob32.out ] && [ -z "$P29_SEED" ]; then
  if diff -u test/golden/vpcm/pcm_bits_to_symbol.main.blob32.out ${T}_blob.out; then echo "GOLDEN: the 32-bit blob run equals test/golden/vpcm/pcm_bits_to_symbol.main.blob32.out"
  else echo "STALE GOLDEN: the blob output differs from test/golden/vpcm/pcm_bits_to_symbol.main.blob32.out (GOLDEN_UPDATE=1 rewrites it)"; exit 1; fi
fi
echo "MATCH: P29 C++ port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')$([ -z "$NO64" ] && echo " + x86-64 [$FP64_FLAGS] vs the 32-bit blob output")) is behaviorally identical to dsplibs.o for this test"
