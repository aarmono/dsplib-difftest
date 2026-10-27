#!/bin/sh
# Builds the differential test of port chunk P02 (the 27 float filter functions of dsplibs.o: FloatFIR / FloatIIR / FloatARMA /
# GenericIIR<float,double>; src/dsp/dsp_float_filters.cpp, REAL C++ compiled with -fno-exceptions -fno-rtti) and runs every scenario in
# its OWN process:
#   blob      test/dsp/dsp_float_filters.main.c + dsplibs.o                       (gcc -m32 -no-pie)
#   port -O0  test/dsp/dsp_float_filters.main.c + g++ -m32 -O0 port object        (LINKED WITH gcc: no libstdc++, g++ -m32 cannot link here)
#   port -O2  same at -O2
#   port -m64 (NO64=1 skips) the same harness and the port compiled for x86-64 (SSE float default; FP64_FLAGS="-mfpmath=387" for the
#             x87 variant); its output is diffed against the 32-bit BLOB's output (the golden dump)
# The ONE harness source has no #ifdef: the functions are reached through __asm__ labels with the mangled names.  A scenario exits
# 2 if its own coverage counters fail, 3 if a harness assumption / independent model is violated; either fails this script.
# "MATCH" is only printed if every scenario matched AND reached its branches.
#
# Also checked here: the 27 blob symbols (name + binding) are all defined by the port object; the harness mirror structs (generated
# by tools/layout_gen.py from test/p02.layout) equal the port header's real class layout (test/p02_layout_probe.cpp, every binary).
#
# Env: PORT_P02 (port source under test, mutation testing), SCENARIOS, NOO2=1, NO64=1, TPREFIX (output prefix, parallel mutation
# runs), P02_SEED (alternative input stream), GOLDEN_UPDATE=1 (rewrite test/golden/dsp/dsp_float_filters.blob32.out from this blob run), FP64_FLAGS
# (extra flags of the x86-64 port build, e.g. -mfpmath=387).
# Scenarios: 0 FloatFIR vs a delay-line model, 1 FloatIIR vs model, 2 FloatARMA process(float) vs model, 3 FloatARMA block vs model
# (dead in the blob), 4 GenericIIR vs model, 5 special values (NaN payloads / inf / denormals / signed zeros; blob vs port only),
# 6 setCoefficients + odd configurations of FloatFIR / FloatIIR (blob vs port only), 7 NaN payloads incl. signalling NaNs (x87-exact only),
# 8 / 9 / 10 FloatFIR / FloatIIR / FloatARMA with POKED non-multiple-of-4 tap counts (the blob's dot-product remainder loop; vs model).
#
# Usage: sh test/dsp/dsp_float_filters.sh   (run from slmodemd/dsplibs-port/)

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh

PORT_P02="${PORT_P02:-src/dsp/dsp_float_filters.cpp}"
T="${TPREFIX:-test/.build/dsp/dsp_float_filters}"
mkdir -p "$(dirname "$T")"
CF="-m32 -Wall -Wextra -Wno-unused-function -O0 -g"
HCF="-m32 -Wall -Wextra -Wno-unused-function -O2 -g"
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -Isrc"
LD32="-no-pie"
SCENARIOS="${SCENARIOS:-0 1 2 3 4 5 6 7 8 9 10 11}"
case "$T" in /*) R="";; *) R="./";; esac   # absolute TPREFIX (mutation / coverage tools) needs no "./"

# the shared stubs, with the allocator renamed (the harness owns sysdep_malloc / sysdep_free: allocation table + log)
gcc $CF -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -c test/stub_env.c -o ${T}_stub_env.o
g++ -m32 $CXXF -O2 -c test/p02_layout_probe.cpp -o ${T}_probe.o
gcc $HCF -c test/harness/harness.c -o ${T}_harness.o     # the shared harness library (test/harness/, plain C)

gcc $HCF $LD32 -o ${T}_test_blob test/dsp/dsp_float_filters.main.c ${T}_harness.o ${T}_stub_env.o ${T}_probe.o $BLOB_O

g++ -m32 $CXXF -O0 -g -c $PORT_P02 -o ${T}_port_o0.o
gcc $HCF $LD32 -o ${T}_test_port test/dsp/dsp_float_filters.main.c ${T}_harness.o ${T}_stub_env.o ${T}_probe.o ${T}_port_o0.o
if [ -z "$NOO2" ]; then
  g++ -m32 $CXXF -O2 -g -c $PORT_P02 -o ${T}_port_o2.o
  gcc $HCF $LD32 -o ${T}_test_port_o2 test/dsp/dsp_float_filters.main.c ${T}_harness.o ${T}_stub_env.o ${T}_probe.o ${T}_port_o2.o
fi

# symbol check: every WEAK template instance of the 11 classes in the blob exists, with the same name and binding, in the port object
BLOBSYMS=$(nm --defined-only $BLOB_O | grep -E '_ZN(8FloatFIR|8FloatIIR|9FloatARMA|10GenericIIRIfd)' | awk '{print $2" "$3}' | sort)
PORTSYMS=$(nm --defined-only ${T}_port_o2.o 2>/dev/null || nm --defined-only ${T}_port_o0.o)
MISSING=$(echo "$BLOBSYMS" | while read b s; do echo "$PORTSYMS" | grep -q " $b $s\$" || echo "$b $s"; done)
NBLOB=$(echo "$BLOBSYMS" | wc -l)
if [ -n "$MISSING" ]; then echo "SYMBOL MISMATCH (blob symbol missing / different binding in the port object):"; echo "$MISSING"; exit 1; fi
echo "SYMBOLS: all $NBLOB blob symbols (name + binding) are defined by the port object"

if [ -z "$NO64" ]; then
  g++ -m64 $CXXF -O2 $FP64_FLAGS -c $PORT_P02 -o ${T}_port64.o
  g++ -m64 $CXXF -O2 -c test/p02_layout_probe.cpp -o ${T}_probe64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free \
      -c test/stub_env.c -o ${T}_stub_env64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness64.o
  gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -no-pie -o ${T}_test_port64 test/dsp/dsp_float_filters.main.c ${T}_harness64.o ${T}_stub_env64.o ${T}_probe64.o ${T}_port64.o
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
# scenario 7 (NaN payloads, incl. signalling NaNs) is exact only for x87 code: the SSE build of the port (x86-64 default) lets gcc turn an
# explicit extended-precision float subtraction into subss, and SSE picks a different NaN when two different NaNs meet.  It is therefore
# compared for the x86-64 build only with FP64_FLAGS containing -mfpmath=387 (and always for the 32-bit builds, which are x87).
S64="$SCENARIOS"
case "$FP64_FLAGS" in *387*) ;; *) S64=$(for i in $SCENARIOS; do [ "$i" = 7 ] || printf '%s ' $i; done);; esac
: > ${T}_blob.out; : > ${T}_port.out; : > ${T}_port_o2.out; : > ${T}_port64.out; : > ${T}_blob64ref.out; : > ${T}_blob_main.out; : > ${T}_blob_nan.out
for i in $SCENARIOS; do
  cat ${T}_s$i.blob.out >> ${T}_blob.out
  cat ${T}_s$i.port.out >> ${T}_port.out
  if [ -z "$NOO2" ]; then cat ${T}_s$i.o2.out >> ${T}_port_o2.out; fi
  if [ "$i" = 7 ]; then cat ${T}_s$i.blob.out >> ${T}_blob_nan.out; else cat ${T}_s$i.blob.out >> ${T}_blob_main.out; fi
done
for i in $S64; do
  if [ -z "$NO64" ]; then cat ${T}_s$i.p64.out >> ${T}_port64.out; cat ${T}_s$i.blob.out >> ${T}_blob64ref.out; fi
done
rm -f ${T}_s*.out
if [ $rc -ne 0 ]; then echo "SCENARIO FAILURE (non-zero exit: coverage, harness assertion / model violation or crash)"; exit 1; fi

diff -u ${T}_blob.out ${T}_port.out
if [ -z "$NOO2" ]; then diff -u ${T}_blob.out ${T}_port_o2.out; fi
if [ -z "$NO64" ]; then diff -u ${T}_blob64ref.out ${T}_port64.out; fi
if [ -n "$GOLDEN_UPDATE" ]; then mkdir -p test/golden/dsp; cp ${T}_blob_main.out test/golden/dsp/dsp_float_filters.blob32.out; cp ${T}_blob_nan.out test/golden/dsp/dsp_float_filters.blob32.nan.out; echo "golden updated"; fi
if [ -z "$TPREFIX" ] && [ "$SCENARIOS" = "0 1 2 3 4 5 6 7 8 9 10 11" ] && [ -f test/golden/dsp/dsp_float_filters.blob32.out ] && [ -z "$P02_SEED" ]; then
  diff -u test/golden/dsp/dsp_float_filters.blob32.out ${T}_blob_main.out && diff -u test/golden/dsp/dsp_float_filters.blob32.nan.out ${T}_blob_nan.out && echo "GOLDEN: the 32-bit blob runs equal test/golden/dsp/dsp_float_filters.blob32.out (scenarios 0-6, 8-11) and dsp/dsp_float_filters.blob32.nan.out (scenario 7)"
fi
echo "MATCH: P02 C++ port ($([ -z "$NOO2" ] && echo '-O0 and -O2' || echo '-O0')$([ -z "$NO64" ] && echo " + x86-64 [$FP64_FLAGS; scenarios $S64] vs the 32-bit blob output")) is behaviorally identical to dsplibs.o for this test"
