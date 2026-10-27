#!/bin/sh
# P29 golden check WITHOUT the blob: builds only the x86-64 port (g++ -m64, -fno-exceptions -fno-rtti: the P29 file + the P02 / P15 / P16 port files for scenario 3; in scenarios 0 - 2 every callee - V90Mapper, V92Transmitter's sub-objects, V92Transmitter
# itself in flavour A - is a harness recorder (the harness objects come first on the link line and -z muldefs lets them win over the port's own V92Transmitter symbols)) and the harness (gcc -m64), runs the scenarios and diffs the concatenated output against the golden produced by the 32-bit run against the real
# dsplibs.o (pcm_bits_to_symbol.main.sh, GOLDEN_UPDATE=1): test/golden/vpcm/pcm_bits_to_symbol.main.blob32.out.
# FP64_FLAGS="-O0", "-O1", "-O3", "-Os" ... select the build flags (default -O2); FP64_FLAGS=-mfpmath=387 selects the x87 mode (default is SSE with long double).  No dsplibs.o, no 32-bit toolchain needed.
# Usage: sh test/vpcm/pcm_bits_to_symbol.main.golden64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p11) echo vpcm/v92_cp;; p16) echo vpcm/v92_tx_coding;; p29) echo vpcm/pcm_bits_to_symbol;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
T="${TPREFIX:-test/.build/vpcm/pcm_bits_to_symbol.main.g64}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
OPT="-O2"
case "$FP64_FLAGS" in *-O*) OPT="";; esac
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc $OPT $FP64_FLAGS"
g++ -m64 $CXXF -c src/vpcm/pcm_bits_to_symbol.cpp -o ${T}_port.o
OBJS=""
for f in src/dsp/dsp_float_filters src/vpcm/v92_modulus_encoder src/vpcm/v92_tx_coding; do g++ -m64 $CXXF -c $f.cpp -o ${T}_${f##*/}.o; OBJS="$OBJS ${T}_${f##*/}.o"; done
PROBES=""
for c in p11 p16 p29; do g++ -m64 $CXXF -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
gcc -m64 -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf -c test/stub_env.c -o ${T}_stub.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness.o
W64="-Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2"
gcc -m64 $W64 -DP29_BIN_A -c test/vpcm/pcm_bits_to_symbol.main.main.c -o ${T}_mainA.o
gcc -m64 $W64 -c test/vpcm/pcm_bits_to_symbol.main.main.c -o ${T}_mainB.o
gcc -m64 $W64 -DP29_BIN_C -c test/vpcm/pcm_bits_to_symbol.main.main.c -o ${T}_mainC.o
gcc -m64 -O2 -no-pie -Wl,-z,muldefs -o ${T}_testA ${T}_mainA.o ${T}_harness.o ${T}_stub.o $PROBES ${T}_port.o
gcc -m64 -O2 -no-pie -Wl,-z,muldefs -o ${T}_testB ${T}_mainB.o ${T}_harness.o ${T}_stub.o $PROBES ${T}_port.o
gcc -m64 -O2 -no-pie -Wl,-z,muldefs -o ${T}_testC ${T}_mainC.o ${T}_harness.o ${T}_stub.o $PROBES ${T}_port.o $OBJS
: > ${T}.out
for i in 0 1; do ${T_RUN}${T}_testA $i >> ${T}.out 2>/dev/null; done
${T_RUN}${T}_testB 2 >> ${T}.out 2>/dev/null
${T_RUN}${T}_testC 3 >> ${T}.out 2>/dev/null
rc=0
if diff -u test/golden/vpcm/pcm_bits_to_symbol.main.blob32.out ${T}.out > ${T}.diff; then echo "GOLDEN64: the x86-64 port ($FP64_FLAGS $OPT) reproduces the 32-bit blob's canonical dump for scenarios 0-3 (test/golden/vpcm/pcm_bits_to_symbol.main.blob32.out)"
else echo "GOLDEN64 FAIL: the dump differs ($(grep -c '^[<>+-][^+-]' ${T}.diff) lines, see ${T}.diff)"; rc=1; fi
rm -f ${T}_*.o ${T}_testA ${T}_testB ${T}_testC ${T}.out
[ $rc = 0 ] && rm -f ${T}.diff
exit $rc
