#!/bin/sh
# H01: determinism guard for the V.34 tier-3 group C modulator sweep (test/v34/v34_init_tables.sh scenario 3).  T0P saw ONE parallel run in which the
# port's scenario-3 hash differed from the blob's (all=831877ee vs 65b36a1a); it never reproduced (H01: 24 full-script runs under load in private sandboxes, 48
# stand-alone runs under ASLR / setarch -R / MALLOC_PERTURB_ / environment padding, an ASan+UBSan build and a stack-poison build: all identical).  This job
# re-runs the sweep under those perturbations and requires every output to be byte-identical (and equal to the blob's, apart from the table-check banner,
# which goes to stderr): a latent dependence on heap/stack contents, ASLR or the environment would show up here as a differing md5.
# Variants (port binary, scenario 3 and 2): default; setarch -R; MALLOC_PERTURB_=165; 4 KB of environment padding; H01_POISON=0x00 / 0xa5 (the harness
# overwrites 64 KB of stack with that byte before every call, test/v34/v34_init_tables.main.c).
set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
CFLAGS="-m32 -Isrc -Wall -Wextra -O0 -g"
T=${TMPDIR:-/tmp}/h01_repeat.$$; mkdir -p $T
trap 'rm -rf $T' EXIT
gcc $CFLAGS -DSTUB_ENV_REAL_STRING -o $T/port test/v34/v34_init_tables.main.c test/stub_env.c test/stub_env_str.c \
    src/v34/v34_setup.c src/v34/v34_init.c src/v34/v34_agc_demod.c src/v34/v34_dsp_primitives.c src/v34/v34_timing_hilbert_tables.c src/v34/v34_init_tables.c src/fpm/fpm_tables.c src/v34/v34_tx_level.c
gcc $CFLAGS -c src/v34/v34_init_tables.c -o $T/pt.o; objcopy --prefix-symbols=port_ $T/pt.o
gcc $CFLAGS -DTEST_BLOB -DSTUB_ENV_REAL_STRING -o $T/blob test/v34/v34_init_tables.main.c test/stub_env.c test/stub_env_str.c $T/pt.o $BLOB_O
PAD=$(head -c 4093 /dev/zero | tr '\0' x)
run() { n=$1; shift; "$@" 2>/dev/null | md5sum | cut -c1-32 > $T/$n.md5; }
for sc in 2 3; do
  run blob$sc $T/blob $sc &
  run def$sc $T/port $sc &
  run aslr$sc setarch -R $T/port $sc &
  run pert$sc env MALLOC_PERTURB_=165 $T/port $sc &
  run pad$sc env H01PAD=$PAD $T/port $sc &
  run poi0$sc env H01_POISON=0 $T/port $sc &
  run poi165$sc env H01_POISON=165 $T/port $sc &
  wait
done
bad=0
for sc in 2 3; do
  ref=$(cat $T/blob$sc.md5)
  for v in def aslr pert pad poi0 poi165; do
    [ "$(cat $T/$v$sc.md5)" = "$ref" ] || { echo "DIFFERENT: scenario $sc variant $v: $(cat $T/$v$sc.md5) vs blob $ref"; bad=1; }
  done
done
if [ $bad = 0 ]; then echo "MATCH: V34SetupModulator scenarios 2 and 3: port output identical to the blob's under default / no-ASLR / MALLOC_PERTURB_=165 / 4 KB env / stack poison 0x00 and 0xa5"; else echo "MISMATCH"; exit 1; fi
