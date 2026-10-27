#!/bin/sh
# P28 golden check WITHOUT the blob: builds only the x86-64 port (g++ -m64, -fno-exceptions -fno-rtti: the P28 file + the P01 / P03 / P06 / P21 port files for the sub-objects, the modulus coders, the G.711 coders and the shaper's constructor; the shaper's reset / resetSSFilter / process and the detector's clearCamulativeVal are the harness recorders, so the P21 object has those three symbols WEAK) and the harness (gcc -m64), runs the scenarios and
# diffs the concatenated output against the golden produced by the 32-bit run against the real dsplibs.o (v90_mapper.sh, GOLDEN_UPDATE=1): test/golden/vpcm/v90_mapper.blob32.out.
# FP64_FLAGS="-O0", "-O1", "-O3", "-Os" ... select the build flags (default -O2); FP64_FLAGS=-mfpmath=387 selects the x87 mode (default is SSE with long double).  No dsplibs.o, no 32-bit toolchain needed.
# Usage: sh test/vpcm/v90_mapper.golden64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p07) echo vpcm/pcm_parameters;; p11) echo vpcm/v92_cp;; p18) echo vpcm/v90_adid;; p28) echo vpcm/v90_mapper;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
T="${TPREFIX:-test/.build/vpcm/v90_mapper.g64}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
OPT="-O2"
case "$FP64_FLAGS" in *-O*) OPT="";; esac
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc $OPT $FP64_FLAGS"
OBJS=""
for f in src/vpcm/v90_mapper src/dsp/dsp_int_templates src/dsp/dsp_window_fft_psd src/vpcm/pcm_leaf_classes src/vpcm/v90_spectral_shaper; do g++ -m64 $CXXF -c $f.cpp -o ${T}_${f##*/}.o; OBJS="$OBJS ${T}_${f##*/}.o"; done
objcopy --weaken-symbol=_ZN17V90SpectralShaper5resetEjjffff --weaken-symbol=_ZN17V90SpectralShaper13resetSSFilterEffff --weaken-symbol=_ZN17V90SpectralShaper7processEPsPhS0_ ${T}_v90_spectral_shaper.o
PROBES=""
for c in p07 p11 p18 p28; do g++ -m64 $CXXF -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
gcc -m64 -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf -c test/stub_env.c -o ${T}_stub.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness.o
gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -c test/vpcm/v90_mapper.main.c -o ${T}_main.o
gcc -m64 -O2 -no-pie -o ${T}_test ${T}_main.o ${T}_harness.o ${T}_stub.o $PROBES $OBJS
: > ${T}.out
for i in 0 1 2 3 4; do ${T_RUN}${T}_test $i >> ${T}.out 2>/dev/null; done
rc=0
if diff -u test/golden/vpcm/v90_mapper.blob32.out ${T}.out > ${T}.diff; then echo "GOLDEN64: the x86-64 port ($FP64_FLAGS $OPT) reproduces the 32-bit blob's canonical dump for scenarios 0-4 (test/golden/vpcm/v90_mapper.blob32.out)"
else echo "GOLDEN64 FAIL: the dump differs ($(grep -c '^[<>+-][^+-]' ${T}.diff) lines, see ${T}.diff)"; rc=1; fi
rm -f ${T}_*.o ${T}_test ${T}.out
[ $rc = 0 ] && rm -f ${T}.diff
exit $rc
