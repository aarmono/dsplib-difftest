#!/bin/sh
# Z01 HYBRID DRY RUN (32-bit; a test of the INTEGRATION PLAN in NOTES.md, NOT the production switch-over and not part of the MATCH of vpcm/dp_vpcm.sh): EVERY port object (src/*.c, src/*.cpp:
# the P01 - P47 classes, the V.34 C tiers, V01 - V07, V04a - z, V.32, V.8, B103 / V.21, V.23, FPM, ring detector, Z01) is linked over a copy of dsplibs.o in which every global symbol that a port object
# redefines is WEAK; the integration harness test/vpcm/dp_vpcm.graph.main.c (two real V.34 / V.90 / V.92 sessions wired back to back, 6 variants x 6,000 calls) must print the same output as with the unmodified
# blob (test/golden/vpcm/dp_vpcm.graph.blob32.out).  Needed on top of the ports: src/vpcm/vpcm_getmp_alias.cpp (V02's regparm(1) getMPrecvdBits -> the cdecl function of tier 4b) and the V04 pieces compiled with
# -DV04_REAL_HELPERS -Dgetbit=v04_getbit_impl -DApplyBulkDelay=v04_apply_bulk_delay_impl (otherwise the pieces call the cdecl getbit / ApplyBulkDelay of tier 5d with the blob's regparm(3) convention: crash).
# Usage: sh test/cross/z01_hybrid.sh   (from slmodemd/dsplibs-port/; needs test/.build/vpcm/dp_vpcm_harness.o and dp_vpcm_stub_envg.o from a run of vpcm/dp_vpcm.sh)
set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/cross/z01_hybrid}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
mkdir -p "$(dirname "$T")"
mkdir -p ${T}_objs; rm -f ${T}_objs/*.o
CXXF="-m32 -O2 -w -fno-exceptions -fno-rtti -fno-stack-protector -fno-pie -Isrc"
# every library source of sources.mk (kind c | cpp | v04 = with V04_DEFS), objects named by the file's stem (RS4: unique, src/<module>/<stem>)
python3 tools/port_objs.py list | while read f k; do b=$(basename ${f%.*})
  case $k in
    c) gcc -m32 -O2 -w -fno-stack-protector -fno-pie -Isrc -c $f -o ${T}_objs/$b.o;;
    v04) g++ $CXXF -DV04_REAL_HELPERS -Dgetbit=v04_getbit_impl -DApplyBulkDelay=v04_apply_bulk_delay_impl -c $f -o ${T}_objs/$b.o;;
    *) g++ $CXXF -c $f -o ${T}_objs/$b.o;;
  esac
done
for o in ${T}_objs/*.o; do nm -g --defined-only $o; done | awk 'NF==3 && $2!="W"{print $3}' | sort -u > ${T}_port.txt
nm -g --defined-only $BLOB_O | awk 'NF==3{print $3}' | sort -u > ${T}_blob.txt
comm -12 ${T}_port.txt ${T}_blob.txt > ${T}_both.txt
echo "port objects: $(ls ${T}_objs/*.o | wc -l); blob globals redefined by a port object (weakened): $(wc -l < ${T}_both.txt)"
objcopy $(sed 's/^/--weaken-symbol=/' ${T}_both.txt | tr '\n' ' ') $BLOB_O ${T}_blob_hybrid.o
gcc -m32 -O2 -w -no-pie -o ${T}_test test/vpcm/dp_vpcm.graph.main.c test/.build/vpcm/dp_vpcm_harness.o test/.build/vpcm/dp_vpcm_stub_envg.o ${T}_blob_hybrid.o ${T}_objs/*.o -lm -Itest
: > ${T}.out
for v in 0 1 2 3 4 5; do ${T_RUN}${T}_test $v >> ${T}.out; done
for s in VPcmV34Progress v34handshak VPCMXF_Create; do objdump -t ${T}_test | grep -q " $s\$" && echo "$s: $(objdump -t ${T}_test | grep " $s\$" | awk '{print $1}') (the port's if above the blob's .text)"; done
if cmp ${T}.out test/golden/vpcm/dp_vpcm.graph.blob32.out; then echo "HYBRID: all ports over the blob reproduce the blob's output (test/golden/vpcm/dp_vpcm.graph.blob32.out, 514 lines)"; else echo "HYBRID MISMATCH"; diff ${T}.out test/golden/vpcm/dp_vpcm.graph.blob32.out | head; exit 1; fi
