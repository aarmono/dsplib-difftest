#!/bin/sh
# P39 golden check WITHOUT the blob: builds only the x86-64 ports (g++ -m64, -fno-exceptions -fno-rtti: src/vpcm/v90_modulator.cpp, src/vpcm/v92_modulator.cpp; every callee is a harness recorder) and the harnesses
# (gcc -m64), runs the scenarios and diffs the concatenated output against the goldens produced by the 32-bit runs against the real dsplibs.o (v90_modulator.sh, GOLDEN_UPDATE=1): test/golden/vpcm/v90_modulator.a.blob32.out
# (scenarios 0-3) and test/golden/vpcm/v90_modulator.b.blob32.out (scenarios 0-4).
# FP64_FLAGS="-O0", "-O1", "-O3", "-Os" ... select the build flags (default -O2); FP64_FLAGS=-mfpmath=387 selects the x87 mode (the long double arithmetic is x87 either way, the float stores of the port are
# the only SSE / x87 difference: the result must not change).  No dsplibs.o, no 32-bit toolchain needed.
# Usage: sh test/vpcm/v90_modulator.golden64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p07) echo vpcm/pcm_parameters;; p09) echo vpcm/v90_mp;; p10) echo vpcm/v90_cp;; p11) echo vpcm/v92_cp;; p15) echo vpcm/v92_modulus_encoder;; p27) echo vpcm/pcm_phase3_modulators;; p28) echo vpcm/v90_mapper;; p29) echo vpcm/pcm_bits_to_symbol;; p30) echo vpcm/v90_phase4_modulator_symbol;; p32) echo vpcm/v92_phase4_modulator_symbol;; p39) echo vpcm/v90_modulator;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
T="${TPREFIX:-test/.build/vpcm/v90_modulator.g64}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
OPT="-O2"
case "$FP64_FLAGS" in *-O*) OPT="";; esac
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc $OPT $FP64_FLAGS"
gcc -m64 -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf -c test/stub_env.c -o ${T}_stub.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -c test/harness/harness.c -o ${T}_harness.o
rc=0
for H in a b; do
  if [ $H = a ]; then PORT=src/vpcm/v90_modulator.cpp; PL="p07 p09 p10 p11 p27 p28 p29 p30 p39"; SC="0 1 2 3"; else PORT=src/vpcm/v92_modulator.cpp; PL="p07 p11 p15 p27 p29 p32 p39"; SC="0 1 2 3 4"; fi
  g++ -m64 $CXXF -c $PORT -o ${T}_p39$H.o
  PROBES=""
  for c in $PL; do g++ -m64 $CXXF -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_${H}_$c.o; PROBES="$PROBES ${T}_probe_${H}_$c.o"; done
  gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -Itest -c test/vpcm/v90_modulator.${H}.main.c -o ${T}_main_$H.o
  gcc -m64 -O2 -no-pie -o ${T}_test_$H ${T}_main_$H.o ${T}_harness.o ${T}_stub.o $PROBES ${T}_p39$H.o
  : > ${T}_$H.out
  for i in $SC; do ${T_RUN}${T}_test_$H $i >> ${T}_$H.out 2>/dev/null; done
  if diff -u test/golden/vpcm/v90_modulator.${H}.blob32.out ${T}_$H.out > ${T}_$H.diff; then echo "GOLDEN64 (part $H): the x86-64 port ($FP64_FLAGS $OPT) reproduces the 32-bit blob's canonical dump for scenarios $SC (test/golden/vpcm/v90_modulator.${H}.blob32.out)"
  else echo "GOLDEN64 (part $H): FAIL: the dump differs ($(grep -c '^[<>+-][^+-]' ${T}_$H.diff) lines, see ${T}_$H.diff)"; rc=1; fi
  rm -f ${T}_p39$H.o ${T}_probe_${H}_*.o ${T}_main_$H.o ${T}_test_$H ${T}_$H.out
  [ $rc = 0 ] && rm -f ${T}_$H.diff
done
rm -f ${T}_stub.o ${T}_harness.o
exit $rc
