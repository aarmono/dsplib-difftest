#!/bin/sh
# Builds the differential test of port chunk P33 (V92Phase4Modulator part B: the 35 methods other than generateSymbol, 36 symbols; src/vpcm/v92_phase4_modulator.cpp, REAL C++ compiled with -fno-exceptions
# -fno-rtti) and runs every scenario in its OWN process.  Every callee is interposed by a harness recorder with scripted results:
#   blob      test/vpcm/v92_phase4_modulator.main.c + a copy of dsplibs.o in which the callees (Scrambler<uchar,uchar> ctor / dtor / process(uchar) / processAllOnes / processAllZeros / reset, V92Mapper C1 / C2 / D1 / D2 / reset /
#             process, V92BitsToSymbol nofBitsForNextTime / process x2 / reset / setSymbolsBlockSize, V92CP::infoToBits / getBitVector, the P32 generateSymbol) and edprintf are WEAK   (gcc -m32 -no-pie);
#             sysdep_malloc / sysdep_free are the harness' allocator
#   port -O0  the harness + the port object under test (the callees are the recorders: nothing else is linked)
#   port -O2  same at -O2
#   port -m64 (NO64=1 skips) the same harness and the port compiled for x86-64; its output is diffed against the 32-bit BLOB's output (the golden dump)
# The ONE harness source has no #ifdef around the code under test: the methods are reached through __asm__ labels with the mangled names.  A scenario exits 2 if its own coverage sites fail, 3 if a harness
# assumption / the independent model is violated; either fails this script.  Also checked: the blob symbols (name + binding) are defined by the port object; the harness mirrors (tools/layout_gen.py from
# test/vpcm/v92_phase4_modulator_symbol.layout, p11, p15, p29) equal the port headers' real class layouts (layout probes).  "MATCH" is only printed if every scenario matched AND reached its sites.
# Env: PORT_P33 (port source under test, mutation testing), SCENARIOS, NOO2=1, NO64=1, TPREFIX (output prefix, parallel mutation runs), P33_SEED (alternative input stream), GOLDEN_UPDATE=1 (rewrite
# test/golden/vpcm/v92_phase4_modulator.blob32.out from this blob run), FP64_FLAGS (extra flags of the x86-64 port build).
# Usage: sh test/vpcm/v92_phase4_modulator.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p11) echo vpcm/v92_cp;; p15) echo vpcm/v92_modulus_encoder;; p29) echo vpcm/pcm_bits_to_symbol;; p32) echo vpcm/v92_phase4_modulator_symbol;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_P33="${PORT_P33:-src/vpcm/v92_phase4_modulator.cpp}"
T="${TPREFIX:-test/.build/vpcm/v92_phase4_modulator}"
mkdir -p "$(dirname "$T")"
CF="-m32 -Wall -Wextra -Wno-unused-function -O0 -g"
HCF="-m32 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -O2 -g"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc"
LD32="-no-pie"
ALLSC="0 1 2"
SCENARIOS="${SCENARIOS:-0 1 2}"
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"

SRENAME="-Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf"
gcc $CF $SRENAME -c test/stub_env.c -o ${T}_stub_env.o
PROBES=""
for c in p11 p15 p29 p32; do g++ -m32 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
gcc $HCF -c test/harness/harness.c -o ${T}_harness.o     # the shared harness library (test/harness/, plain C)

CALLEES="_ZN9ScramblerIhhEC1Ejjj _ZN9ScramblerIhhEC2Ejjj _ZN9ScramblerIhhED1Ev _ZN9ScramblerIhhED2Ev _ZN9ScramblerIhhE7processEh _ZN9ScramblerIhhE14processAllOnesEPhj
 _ZN9ScramblerIhhE15processAllZerosEPhj _ZN9ScramblerIhhE5resetEh _ZN9V92MapperC1Ev _ZN9V92MapperC2Ev _ZN9V92MapperD1Ev _ZN9V92MapperD2Ev _ZN9V92Mapper5resetEsh _ZN9V92Mapper7processEPh
 _ZN15V92BitsToSymbol18nofBitsForNextTimeEv _ZN15V92BitsToSymbol7processEPhj _ZN15V92BitsToSymbol7processERjPs _ZN15V92BitsToSymbol5resetEP16V92MappingParams
 _ZN15V92BitsToSymbol19setSymbolsBlockSizeEj _ZN5V92CP10infoToBitsEv _ZN5V92CP12getBitVectorERj _ZN18V92Phase4Modulator14generateSymbolEv edprintf"
WEAKARGS=""
for f in $CALLEES; do WEAKARGS="$WEAKARGS --weaken-symbol=$f"; done
objcopy $WEAKARGS $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF $LD32 -o ${T}_test_blob test/vpcm/v92_phase4_modulator.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_dsplibs_weak.o

g++ -m32 $CXXF -O0 -g -c $PORT_P33 -o ${T}_port_o0.o
gcc $HCF $LD32 -o ${T}_test_port test/vpcm/v92_phase4_modulator.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o0.o
if [ -z "$NOO2" ]; then
  g++ -m32 $CXXF -O2 -g -c $PORT_P33 -o ${T}_port_o2.o
  gcc $HCF $LD32 -o ${T}_test_port_o2 test/vpcm/v92_phase4_modulator.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o2.o
fi

# symbol check: both functions exist, with the same name and binding / section class, in the port object
DEFAULT_SYMRE='_ZN18V92Phase4Modulator([^1].*|1[^4].*|14[^g].*)'
SYMRE="${P33_SYMRE:-$DEFAULT_SYMRE}"
BLOBSYMS=$(nm --defined-only $BLOB_O | awk -v re="^($SYMRE)" '$3 ~ re {print $2" "$3}' | sort)
PORTSYMS=$(nm --defined-only ${T}_port_o2.o 2>/dev/null || nm --defined-only ${T}_port_o0.o)
MISSING=$(echo "$BLOBSYMS" | while read b s; do echo "$PORTSYMS" | grep -q " $b $s\$" || echo "$b $s"; done)
NBLOB=$(echo "$BLOBSYMS" | wc -l)
if [ -n "$MISSING" ]; then echo "SYMBOL MISMATCH (blob symbol missing / different binding in the port object):"; echo "$MISSING"; exit 1; fi
echo "SYMBOLS: all $NBLOB blob symbols (name + binding) are defined by the port object"

if [ -z "$NO64" ]; then
  g++ -m64 $CXXF -O2 $FP64_FLAGS -c $PORT_P33 -o ${T}_port64.o
  P64=""
  for c in p11 p15 p29 p32; do g++ -m64 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe64_$c.o; P64="$P64 ${T}_probe64_$c.o"; done
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 $SRENAME -c test/stub_env.c -o ${T}_stub_env64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -no-pie -o ${T}_test_port64 test/vpcm/v92_phase4_modulator.main.c ${T}_harness64.o ${T}_stub_env64.o $P64 ${T}_port64.o
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
if [ -n "$GOLDEN_UPDATE" ]; then mkdir -p test/golden; cp ${T}_blob.out test/golden/vpcm/v92_phase4_modulator.blob32.out; echo "golden updated"; fi
if [ -z "$TPREFIX" ] && [ "$SCENARIOS" = "$ALLSC" ] && [ -f test/golden/vpcm/v92_phase4_modulator.blob32.out ] && [ -z "$P33_SEED" ]; then
  if diff -u test/golden/vpcm/v92_phase4_modulator.blob32.out ${T}_blob.out; then echo "GOLDEN: the 32-bit blob run equals test/golden/vpcm/v92_phase4_modulator.blob32.out"
  else echo "STALE GOLDEN: the blob output differs from test/golden/vpcm/v92_phase4_modulator.blob32.out (GOLDEN_UPDATE=1 rewrites it)"; exit 1; fi
fi
echo "MATCH: P33 C++ port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')$([ -z "$NO64" ] && echo " + x86-64 [$FP64_FLAGS] vs the 32-bit blob output")) is behaviorally identical to dsplibs.o for this test"
