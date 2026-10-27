#!/bin/sh
# Z01 golden check WITHOUT the blob: builds only the x86-64 port (g++ -m64, -fno-exceptions -fno-rtti: src/vpcm/dp_vpcm.cpp; every callee is a harness recorder; the integration scenario against the real object graph is 32-bit only) and the
# harness (gcc -m64), runs the scenarios and diffs the concatenated output against the golden produced by the 32-bit run against the real dsplibs.o (dp_vpcm.sh, GOLDEN_UPDATE=1): test/golden/vpcm/dp_vpcm.blob32.out.
# FP64_FLAGS="-O0", "-O3", "-O2 -mfpmath=387" select the build flags (default -O2; the flag set is -O0, -O2, -O2 -mfpmath=387 and one -O3 run); FP64_FLAGS=-mfpmath=387 selects the x87 mode (the only floating point is the x87 buffer-size computation of VPCMXF_Create and the truncating sample stores of vpcm_run: the result must not change).  No dsplibs.o, no 32-bit toolchain needed.
# Usage: sh test/vpcm/dp_vpcm.golden64.sh   (from slmodemd/dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
lp() { case $1 in p47) echo vpcm/vpcm_flo_modem;; v01) echo v34/v01;; z01) echo vpcm/dp_vpcm;; esac; }   # layout probe of a chunk: test/$(lp CHUNK)_layout_probe.cpp
T="${TPREFIX:-test/.build/vpcm/dp_vpcm.g64}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
OPT="-O2"
case "$FP64_FLAGS" in *-O*) OPT="";; esac
CXXF="-Wall -Wextra -fno-exceptions -fno-rtti -fno-stack-protector -Isrc $OPT $FP64_FLAGS"
g++ -m64 $CXXF -c src/vpcm/dp_vpcm.cpp -o ${T}_z01.o
PROBES=""
for c in p47 v01 z01; do g++ -m64 $CXXF -c test/$(lp $c)_layout_probe.cpp -o ${T}_probe_$c.o; PROBES="$PROBES ${T}_probe_$c.o"; done
gcc -m64 -O2 -Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free -Ddsplibs_debug_printf=stub_env_unused_dbgprintf -Dsysdep_memset=stub_env_unused_memset -Dsysdep_memcpy=stub_env_unused_memcpy -Dmodem_debug_log_data=stub_env_unused_mdld -Dmodem_dp_deregister=stub_env_unused_mdd -Dmodem_dp_register=stub_env_unused_mdr -Dmodem_get_bits=stub_env_unused_mgb -Dmodem_get_param=stub_env_unused_mgp -Dmodem_put_bits=stub_env_unused_mpb -Dmodem_set_param=stub_env_unused_msp -c test/stub_env.c -o ${T}_stub.o
gcc -m64 -Wall -Wextra -Wno-unused-function -O2 -Dsysdep_malloc=h_sysdep_malloc -Dsysdep_free=h_sysdep_free -c test/harness/harness.c -o ${T}_harness.o
gcc -m64 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-misleading-indentation -O2 -c test/vpcm/dp_vpcm.main.c -o ${T}_main.o
gcc -m64 -O2 -no-pie -o ${T}_test ${T}_main.o ${T}_harness.o ${T}_stub.o $PROBES ${T}_z01.o -lm
: > ${T}.out
for i in 0 1 2 3; do ${T_RUN}${T}_test $i >> ${T}.out 2>/dev/null; done
rc=0
if diff -u test/golden/vpcm/dp_vpcm.blob32.out ${T}.out > ${T}.diff; then echo "GOLDEN64: the x86-64 port ($FP64_FLAGS $OPT) reproduces the 32-bit blob's canonical dump for scenarios 0-3 (test/golden/vpcm/dp_vpcm.blob32.out)"
else echo "GOLDEN64: FAIL: the dump differs ($(grep -c '^[<>+-][^+-]' ${T}.diff) lines, see ${T}.diff)"; rc=1; fi
rm -f ${T}_*.o ${T}_test ${T}.out
[ $rc = 0 ] && rm -f ${T}.diff
exit $rc
