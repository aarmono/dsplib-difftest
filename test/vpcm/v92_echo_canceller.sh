#!/bin/sh
# Builds the differential test of port chunk P22 (classes V92EchoCanceller and V90PreFilter: 30 functions; src/vpcm/v92_echo_canceller.cpp + src/vpcm/v90_prefilter_tables.cpp, REAL C++ compiled with -fno-exceptions
# -fno-rtti) and runs every scenario in its OWN process:
#   blob      test/vpcm/v92_echo_canceller.main.c + a copy of dsplibs.o in which `edprintf` is WEAK (the harness' recorders define edprintf and dsplibs_debug_printf)   (gcc -m32 -no-pie)
#   port -O0  the harness + the port object under test + the port objects of the callees of earlier chunks (P01 differential encoders, P03 Psd, P06 V90SpectralShapingFilter: cached
#             objects rebuilt only when their sources are newer)   (LINKED WITH gcc: no libstdc++)
#   port -O2  same at -O2
#   port -m64 (NO64=1 skips) the same harness and the port compiled for x86-64 (SSE float default; FP64_FLAGS="-mfpmath=387" for the x87 variant); its output is diffed against the
#             32-bit BLOB's output (the golden dump)
# The ONE harness source has no #ifdef around the code under test: the functions are reached through __asm__ labels with the mangled names.  A scenario exits 2 if its own coverage
# counters fail, 3 if a harness assumption / independent model is violated; either fails this script.  Also checked: the blob symbols (name + binding) of the two classes and the two
# tables are defined by the port object; the harness mirrors (tools/layout_gen.py from test/vpcm/v92_echo_canceller.layout, p07, p06, p02) equal the port headers' real class layouts (layout probes).
# "MATCH" is only printed if every scenario matched AND reached its branches.
# Env: PORT_P22 (port source under test, mutation testing), SCENARIOS, NOO2=1, NO64=1, TPREFIX (output prefix, parallel mutation runs), P22_SEED (alternative input stream),
# GOLDEN_UPDATE=1 (rewrite test/golden/vpcm/v92_echo_canceller.blob32.out from this blob run), FP64_FLAGS (extra flags of the x86-64 port build).
# Usage: sh test/vpcm/v92_echo_canceller.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p02) echo p02;; p06) echo vpcm/pcm_leaf_classes;; p07) echo vpcm/pcm_parameters;; p22) echo vpcm/v92_echo_canceller;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_P22="${PORT_P22:-src/vpcm/v92_echo_canceller.cpp}"
PORT_TAB="${PORT_P22TAB:-src/vpcm/v90_prefilter_tables.cpp}"
T="${TPREFIX:-test/.build/vpcm/v92_echo_canceller}"
mkdir -p "$(dirname "$T")"
CF="-m32 -Wall -Wextra -Wno-unused-function -O0 -g"
HCF="-m32 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -Wno-sign-compare -Wno-parentheses -O2 -g"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc"
LD32="-no-pie"
ALLSC="0 1 2 3 4 5 6 7 8"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7 8}"
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"

SRENAME="-Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf"
gcc $CF $SRENAME -c test/stub_env.c -o ${T}_stub_env.o
PROBES=""
for c in p02 p06 p07 p22; do g++ -m32 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
gcc $HCF -c test/harness/harness.c -o ${T}_harness.o     # the shared harness library (test/harness/, plain C)

objcopy --weaken-symbol=edprintf $BLOB_O ${T}_dsplibs_weak.o
gcc $HCF $LD32 -o ${T}_test_blob test/vpcm/v92_echo_canceller.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_dsplibs_weak.o

# cached port objects of the callees' chunks: rebuilt only when their source or a header is newer / the flags changed
CACHE=test/.build/vpcm/v92_echo_canceller_cache
srcof() { case "$1" in p02) echo src/dsp/dsp_float_filters.cpp;; tab) echo "$PORT_TAB";; esac; }
cdep() { [ ! -f "$1" ] || [ "$2" -nt "$1" ] || [ src/vpcm/v92_echo_canceller.h -nt "$1" ] || [ src/core/dsp_cxx.h -nt "$1" ] || [ src/core/dsp_math.h -nt "$1" ]; }
CO0=""; CO2=""; C64=""
for c in p02 tab; do
  s=$(srcof $c)
  cdep ${CACHE}_${c}_o0.o $s && g++ -m32 $CXXF -O0 -g -c $s -o ${CACHE}_${c}_o0.o
  CO0="$CO0 ${CACHE}_${c}_o0.o"
  if [ -z "$NOO2" ]; then cdep ${CACHE}_${c}_o2.o $s && g++ -m32 $CXXF -O2 -g -c $s -o ${CACHE}_${c}_o2.o; CO2="$CO2 ${CACHE}_${c}_o2.o"; fi
  if [ -z "$NO64" ]; then
    if [ ! -f ${CACHE}_${c}_64.o ] || [ "$FP64_FLAGS" != "$(cat ${CACHE}_${c}_64.flags 2>/dev/null)" ] || cdep ${CACHE}_${c}_64.o $s; then g++ -m64 $CXXF -O2 $FP64_FLAGS -c $s -o ${CACHE}_${c}_64.o; echo "$FP64_FLAGS" > ${CACHE}_${c}_64.flags; fi
    C64="$C64 ${CACHE}_${c}_64.o"
  fi
done

g++ -m32 $CXXF -O0 -g -c $PORT_P22 -o ${T}_port_o0.o
gcc $HCF $LD32 -o ${T}_test_port test/vpcm/v92_echo_canceller.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o0.o $CO0
if [ -z "$NOO2" ]; then
  g++ -m32 $CXXF -O2 -g -c $PORT_P22 -o ${T}_port_o2.o
  gcc $HCF $LD32 -o ${T}_test_port_o2 test/vpcm/v92_echo_canceller.main.c ${T}_harness.o ${T}_stub_env.o $PROBES ${T}_port_o2.o $CO2
fi

# symbol check: every function / object symbol of the two classes and the two tables exists, with the same name and binding / section class, in the port object
DEFAULT_SYMRE='_Z(N|NK)(16V92EchoCanceller|12V90PreFilter)|v92echoPreFilter_'
SYMRE="${P22_SYMRE:-$DEFAULT_SYMRE}"
BLOBSYMS=$(nm --defined-only $BLOB_O | awk -v re="^($SYMRE)" '$3 ~ re {print $2" "$3}' | sort)
PORTSYMS=$( (nm --defined-only ${T}_port_o2.o 2>/dev/null || nm --defined-only ${T}_port_o0.o); nm --defined-only ${CACHE}_tab_o2.o 2>/dev/null || nm --defined-only ${CACHE}_tab_o0.o)
MISSING=$(echo "$BLOBSYMS" | while read b s; do echo "$PORTSYMS" | grep -q " $b $s\$" || echo "$b $s"; done)
NBLOB=$(echo "$BLOBSYMS" | wc -l)
if [ -n "$MISSING" ]; then echo "SYMBOL MISMATCH (blob symbol missing / different binding in the port object):"; echo "$MISSING"; exit 1; fi
echo "SYMBOLS: all $NBLOB blob symbols (name + binding) are defined by the port object"

if [ -z "$NO64" ]; then
  g++ -m64 $CXXF -O2 $FP64_FLAGS -c $PORT_P22 -o ${T}_port64.o
  P64=""
  for c in p02 p06 p07 p22; do g++ -m64 $CXXF -O2 -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe64_$c.o; P64="$P64 ${T}_probe64_$c.o"; done
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 $SRENAME -c test/stub_env.c -o ${T}_stub_env64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -O2 -no-pie -o ${T}_test_port64 test/vpcm/v92_echo_canceller.main.c ${T}_harness64.o ${T}_stub_env64.o $P64 ${T}_port64.o $C64
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
if [ -n "$GOLDEN_UPDATE" ]; then mkdir -p test/golden; cp ${T}_blob.out test/golden/vpcm/v92_echo_canceller.blob32.out; echo "golden updated"; fi
if [ -z "$TPREFIX" ] && [ "$SCENARIOS" = "$ALLSC" ] && [ -f test/golden/vpcm/v92_echo_canceller.blob32.out ] && [ -z "$P22_SEED" ]; then
  if diff -u test/golden/vpcm/v92_echo_canceller.blob32.out ${T}_blob.out; then echo "GOLDEN: the 32-bit blob run equals test/golden/vpcm/v92_echo_canceller.blob32.out"
  else echo "STALE GOLDEN: the blob output differs from test/golden/vpcm/v92_echo_canceller.blob32.out (GOLDEN_UPDATE=1 rewrites it)"; exit 1; fi
fi
echo "MATCH: P22 C++ port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')$([ -z "$NO64" ] && echo " + x86-64 [$FP64_FLAGS] vs the 32-bit blob output")) is behaviorally identical to dsplibs.o for this test"
