#!/bin/sh
# X4: the V.90 / V.92 parameter blocks at the NATIVE layout (test/vpcm/pcm_parameters.native64.main.cpp).  No blob involved.
#   x86-64 : src/vpcm/pcm_leaf_classes.cpp + src/dsp/dsp_resampler.cpp (REAL C++, -fno-exceptions -fno-rtti) + the test, run natively; the objects of the
#            parameter blocks are filled through their named members and the Phase2Info classes and the V90Resampler must read the same members
#   i386   : the test only compiled (-m32 -c): a 32-bit libstdc++ is not available for the link, the static asserts of the file are the check
# Env: PORT_FLAGS (extra flags of the x86-64 port build, e.g. -O0), TPREFIX (output prefix), MUT_SRC (alternative source of pcm_leaf_classes.cpp, used to show
# that the test fails on old code).  Usage: sh test/vpcm/pcm_parameters.native64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
T="${TPREFIX:-test/.build/vpcm/pcm_parameters.native64}"
mkdir -p "$(dirname "$T")"
LEAF="${X4_LEAF:-src/vpcm/pcm_leaf_classes.cpp}"
RES="${X4_RES:-src/dsp/dsp_resampler.cpp}"
CXXF="-Wall -Wextra -Wno-invalid-offsetof -fno-exceptions -fno-rtti -fno-stack-protector -Isrc"
g++ -m32 $CXXF -O2 -c test/vpcm/pcm_parameters.native64.main.cpp -o ${T}_main32.o
g++ -m64 $CXXF -O2 $PORT_FLAGS -c $LEAF -o ${T}_leaf.o
g++ -m64 $CXXF -O2 $PORT_FLAGS -c $RES -o ${T}_res.o
g++ -m64 $CXXF -O2 -c test/vpcm/pcm_parameters.native64.main.cpp -o ${T}_main.o
gcc -m64 -O2 -Wno-unused-result -c test/stub_env.c -o ${T}_stub.o
# the host symbols and the unused window / FFT helpers of the resampler are not needed by the scenarios
g++ -m64 -no-pie -Wl,--unresolved-symbols=ignore-all -o ${T}_test ${T}_main.o ${T}_leaf.o ${T}_res.o ${T}_stub.o
case "$T" in /*) R="";; *) R="./";; esac
rc=0
${R}${T}_test > ${T}.out 2>&1 || rc=$?
cat ${T}.out
rm -f ${T}_main32.o ${T}_leaf.o ${T}_res.o ${T}_main.o ${T}_stub.o ${T}_test
[ $rc = 0 ] && rm -f ${T}.out
exit $rc
