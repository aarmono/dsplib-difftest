#!/bin/sh
# H01: heap-pattern test of every constructor / create function reachable by the plain-C differential harnesses (those that use test/stub_env.c's
# sysdep_malloc).  A fresh test process gives malloc() zero pages, so a field a port constructor forgets to write (or writes differently) reads 0, as in the
# blob, and the harness cannot see it; in a live slmodemd the heap is dirty (N02: V32FP_recreate).  This job re-runs each of the scripts below twice, with
# every sysdep_malloc block pre-filled with 0x00 and with 0xa5 (H01_HEAPFILL), while stub_env.c dumps every block's contents when the code frees it and at exit
# (H01_DUMP); test/cross/h01_heap.cmp.py then requires, for every blob/port pair and scenario, the same allocation sequence, the same contents under each fill and the
# same set of never-written words.  Each script must itself still MATCH under both fills (blob vs port with the same garbage heap).
# Not included: v32_org (the BLOB segfaults under a 0xa5 heap: its harness relies on a zero heap), n01_v32status (170 s, >8000 blocks), the V.34 tiers (no
# sysdep_malloc'ed objects), the P/V/Z rows (their harness allocator already patterns every block).  Usage: sh test/cross/h01_heap.sh  (from dsplibs-port/)
set -e
cd "$(dirname "$0")/../.."
D=${TMPDIR:-/tmp}/h01_heap.$$; mkdir -p $D
trap 'rm -rf $D' EXIT
# scripts still named build_and_diff_<id>.sh: none since TESTREORG stage 8 (the V.32, FSE decision and FPM scripts moved)
SCRIPTS=${H01_SCRIPTS:-""}
# TESTREORG stages 3 / 4 / 8: the scripts of moved jobs live in test/<module>/<stem>.sh (b103 = fsk/v21_b103, v23 = fsk/v23, v8 = v8/v8.main, v8_fsm = v8/v8.fsm, v32 = v32/v32_engine.main, n04_fsedec = v32/fse_decision, fpm = fpm/fpm_toolkit, ...)
MOVED=${H01_MOVED:-"fsk/v21_b103 fsk/v23 v8/v8.main v8/v8.fsm v8/v8.init v8/v8.jm v8/v8.lifecycle v32/v32_engine.main v32/v32_engine.ans v32/v32_engine.capstone v32/v32_engine.capstone_stagger v32/v32_engine.loclop v32/v32_engine.primitives v32/v32_engine.rng v32/v32_engine.vtb v32/v32_engine.vtb_decoder v32/v32_engine.vtb_e2e v32/fse_decision fpm/fpm_toolkit"}
n=0
for s in $MOVED $SCRIPTS; do
  case $s in */*) sp=test/$s.sh;; *) sp=test/build_and_diff_$s.sh;; esac
  for f in 0 165; do
    out=$(H01_DUMP=$D H01_HEAPFILL=$f sh $sp 2>&1) || { echo "$out" | tail -5; echo "FAIL: $s under heap fill $f (blob vs port)"; exit 1; }
    echo "$out" | grep -q '^MATCH' || { echo "$out" | tail -5; echo "FAIL: $s under heap fill $f: no MATCH"; exit 1; }
  done
  n=$((n+1))
done
# the plain-C rd harness is test/ring/ring_detector.sh
for f in 0 165; do out=$(H01_DUMP=$D H01_HEAPFILL=$f sh test/ring/ring_detector.sh 2>&1) || { echo "$out" | tail -5; echo "FAIL: rd under heap fill $f"; exit 1; }; done
python3 test/cross/h01_heap.cmp.py $D --expect test/golden/cross/h01_heap.expect.txt
echo "MATCH: $n scripts + rd under heap fill 0x00 and 0xa5"
