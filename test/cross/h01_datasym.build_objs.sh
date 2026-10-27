#!/bin/sh
# X3 NOTE: the hybrid slmodemd (make slmodemd-hybrid, n01_link_variant.sh) was REMOVED at the switch-over; this script stays ONLY as the object builder of
# test/cross/h01_datasym.sh (run_changed id h01_datasym: port objects at -O0 + the weakened blob list, compared with the blob).  It was test/n01_build_hybrid.sh.
# N01: build the 32-bit HYBRID dsplibs for a real slmodemd: a copy of ../dsplibs.o in which every global the port objects redefine is WEAK, plus
# the port objects (see NOTES.md, Z01 INTEGRATION PLAN, and test/cross/z01_hybrid.sh, whose recipe this is).
# Output (default dir slmodemd/hybrid/, override with OUT=...): dsplibs_hybrid.o (weakened blob), objs/*.o (port objects), link.txt, report.txt.
# Knobs for bisecting (environment):
#   EXCLUDE='regex'   port source basenames (without extension, e.g. v32 or 'v0[1-7]_') that are NOT built / linked: the blob keeps its own copy
#                     of everything those files would have defined (their symbols are simply not weakened).  Matched with grep -E against the basename.
#   KEEPOBJ='regex'   like EXCLUDE but the objects stay linked (port-only tables / helpers resolve) while every blob-named global they define is weakened
#                     so the blob's copy is used.  KEEPSYM='regex' the same per symbol name.
#   NOPIN=1           build the PLAIN flag set (without test/flags.sh PIN_FLAGS); default is the PINNED set
#   OPT=-O2           optimisation level of the port objects (the dry run and the differential tests used -O2 and -O0).
# R03: src/v32/dp_v32.c is the V.32 dp glue (dp_v32_init / dp_v32_exit, v32_create / delete / process); the blob's LOCAL v32_data / handshake / null_protocol live in src/v32/v32_engine.c.
# i386 only.
set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
OUT="${OUT:-../hybrid}"
OPT="${OPT:--O2}"
. ./test/flags.sh   # X1: PIN_FLAGS (-fno-strict-aliasing, a safety net, see test/flags.sh); NOPIN=1 builds the PLAIN set
[ -n "$NOPIN" ] && PIN_FLAGS=
mkdir -p "$OUT/objs"; rm -f "$OUT"/objs/*.o
CXXF="-m32 $OPT $PIN_FLAGS -w -fno-exceptions -fno-rtti -fno-stack-protector -fno-pie -Isrc"
CF="-m32 $OPT $PIN_FLAGS -w -fno-stack-protector -fno-pie -Isrc"
skip() { [ -n "$EXCLUDE" ] && echo "$1" | grep -Eq "$EXCLUDE"; }
# every library source of sources.mk (kind c | cpp | v04 = with V04_DEFS); objects and EXCLUDE / KEEPOBJ use the file's stem (RS4: the new
# names, e.g. EXCLUDE='^vpcm_' or 'v32_engine'; tools/filemap.tsv gives the old name of a stem)
python3 tools/port_objs.py list | while read f k; do b=$(basename ${f%.*})
  skip $b && continue
  case $k in
    c) gcc $CF -c $f -o $OUT/objs/$b.o;;
    v04) g++ $CXXF -DV04_REAL_HELPERS -Dgetbit=v04_getbit_impl -DApplyBulkDelay=v04_apply_bulk_delay_impl -c $f -o $OUT/objs/$b.o;;
    *) g++ $CXXF -c $f -o $OUT/objs/$b.o;;
  esac
done
# (X3: the shim is src/vpcm/vpcm_getmp_alias.cpp now and is built by the loop above)
nm -g --defined-only $BLOB_O | awk 'NF==3{print $3}' | sort -u > $OUT/blob_syms.txt
# KEEPOBJ='regex' (bisect): the matching port objects stay in the link but every blob-named global they define is made weak in the object, so the
# blob's strong definition wins for those names (and for every call to them from the other port objects).  KEEPSYM='regex' does the same per symbol.
: > $OUT/keep.txt
for o in $OUT/objs/*.o; do
  b=$(basename $o .o)
  nm -g --defined-only $o | awk 'NF==3 && $2!="W"{print $3}' | sort -u > $OUT/tmp_syms.txt
  comm -12 $OUT/tmp_syms.txt $OUT/blob_syms.txt > $OUT/tmp_both.txt
  if [ -n "$KEEPOBJ" ] && echo $b | grep -Eq "$KEEPOBJ"; then cat $OUT/tmp_both.txt > $OUT/tmp_keep.txt
  elif [ -n "$KEEPSYM" ]; then grep -E "$KEEPSYM" $OUT/tmp_both.txt > $OUT/tmp_keep.txt || true
  else : > $OUT/tmp_keep.txt; fi
  if [ -s $OUT/tmp_keep.txt ]; then objcopy $(sed 's/^/--weaken-symbol=/' $OUT/tmp_keep.txt | tr '\n' ' ') $o; cat $OUT/tmp_keep.txt >> $OUT/keep.txt; fi
done
sort -u $OUT/keep.txt -o $OUT/keep.txt
for o in $OUT/objs/*.o; do nm -g --defined-only $o; done | awk 'NF==3 && $2!="W"{print $3}' | sort -u > $OUT/port_syms.txt
comm -12 $OUT/port_syms.txt $OUT/blob_syms.txt > $OUT/weakened.txt  # (symbols weakened in the port objects by KEEPOBJ/KEEPSYM are no longer strong, so they are not in port_syms)
# GUARD 1 (N01 finding): a blob name that exists more than once in dsplibs.o (a GLOBAL and LOCAL symbols of the same name, e.g. AGC_DEF_ALPHA / AGC_DEF_BETA) is
# ambiguous: a port symbol with that name replaces the GLOBAL one although the port may have extracted a LOCAL copy.  Fatal.
readelf -sW $BLOB_O | awk 'NF==8 && $4!="SECTION" && $4!="FILE"{n[$8]++; if($5=="GLOBAL"||$5=="WEAK") g[$8]=1} END{for(k in n) if(n[k]>1 && g[k]) print k}' | sort > $OUT/blob_dupnames.txt
if [ -n "$(comm -12 $OUT/blob_dupnames.txt $OUT/weakened.txt)" ]; then echo "FATAL: port symbols replace blob globals that share their name with blob LOCAL symbols:"; comm -12 $OUT/blob_dupnames.txt $OUT/weakened.txt; exit 1; fi
# GUARD 2 (report only): size of every replaced DATA symbol, blob vs port (a different size means a different table)
nm -S -g --defined-only $BLOB_O | awk 'NF==4 && $3 ~ /^[DdRrBb]$/ {print $4, $2}' | sort > $OUT/blob_dsize.txt
for o in $OUT/objs/*.o; do nm -S -g --defined-only $o | awk 'NF==4 && $3 ~ /^[DdRrBb]$/ {print $4, $2}'; done | sort > $OUT/port_dsize.txt
join $OUT/blob_dsize.txt $OUT/port_dsize.txt | awk '$2!=$3{print "SIZE MISMATCH (blob port):", $0}' > $OUT/size_mismatch.txt || true
[ -s $OUT/size_mismatch.txt ] && { echo "warning: $(wc -l < $OUT/size_mismatch.txt) replaced data symbols differ in size between blob and port (see $OUT/size_mismatch.txt)"; }
objcopy $(sed 's/^/--weaken-symbol=/' $OUT/weakened.txt | tr '\n' ' ') $BLOB_O $OUT/dsplibs_hybrid.o
echo "keepobj '$KEEPOBJ' keepsym '$KEEPSYM': $(wc -l < $OUT/keep.txt) blob-named port symbols left to the blob"
echo "port objects: $(ls $OUT/objs/*.o | wc -l); blob globals redefined by a port object (weakened): $(wc -l < $OUT/weakened.txt); opt $OPT; exclude '${EXCLUDE}'" | tee $OUT/report.txt
