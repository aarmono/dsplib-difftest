#!/bin/sh
# H01: every DATA symbol a port object redefines (the symbols the hybrid weakens in the blob) is compared with the blob's: size, bytes (relocated words masked)
# and relocations.  N01 found two bugs of this class by hand (B103's AGC_DEF_* tables took the name of a different blob global; the port's 8-entry FPM_sin_sign
# replaced the blob's 4-entry global that unported code still reads); the hybrid builder only warned about size differences.  Known differences:
# test/golden/cross/h01_datasym.expect.txt (a listed difference that disappears fails too).
set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
OUT=${TMPDIR:-/tmp}/h01_datasym.$$
trap 'rm -rf $OUT' EXIT
OUT=$OUT OPT=-O0 NOPIN=1 sh test/cross/h01_datasym.build_objs.sh > /dev/null
python3 test/cross/h01_datasym.py $BLOB_O test/golden/cross/h01_datasym.expect.txt $OUT/objs/*.o
echo "MATCH: data symbols replaced by the port objects are the blob's (modulo test/golden/cross/h01_datasym.expect.txt)"
