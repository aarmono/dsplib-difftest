#!/bin/sh
# N02: uninitialised-heap probe of the V.32 constructor.  The differential tests run in a fresh process where malloc() returns zero pages, so a field the
# port forgets to initialise reads as 0 there but as garbage in a live slmodemd (dirty heap): found by replaying a V.32bis session (N02).
# Builds ${P}_{blob,port} (P = test/.build/v32/v32_engine.uninit), runs each with the heap pre-filled with 0x00 and with 0xa5 (setarch -R), and requires that the set of bytes
# that differ between the two fills (= never written by the constructor) is the same for the blob and the port.
set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
P="${TPREFIX:-test/.build/v32/v32_engine.uninit}"
mkdir -p "$(dirname "$P")"
T=/tmp/n02_uninit.$$; mkdir -p $T
gcc -m32 -w -O0 -g -c -Dsysdep_malloc=stub_env_malloc_unused -o $T/stub_env.o test/stub_env.c
gcc -m32 -w -O0 -g -o ${P}_blob test/v32/v32_engine.uninit.main.c $T/stub_env.o $BLOB_O 2>/dev/null
gcc -m32 -w -O0 -g -Isrc -o ${P}_port test/v32/v32_engine.uninit.main.c $T/stub_env.o src/v32/v32_engine.c src/v32/v32_datapath.c src/v32/v32_scrambler.c src/v32/v32_pulse_shaping.c \
    src/v32/v32_timing.c src/v32/v32_echo_canceller.c src/v32/v32_equalizer.c src/v32/v32_math.c src/v32/v32_tables.c src/v32/v32_viterbi_tables.c src/fpm/fpm_mrf.c src/fpm/fpm_agc.c src/fpm/fpm_math.c \
    src/fpm/fpm_tables.c src/fpm/fpm_tone.c src/fpm/fpm_mtd.c src/fpm/fpm_iir.c
for b in blob port; do for f in 0 165; do setarch -R ./${P}_$b $f > $T/u_${b}_$f.txt; done; done
if python3 test/v32/v32_engine.uninit.cmp.py $T; then echo "MATCH: V32FP_create leaves the same heap bytes uninitialised in the port as in dsplibs.o"; rc=0; else echo "MISMATCH"; rc=1; fi
rm -rf $T
exit $rc
