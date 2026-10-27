#!/bin/sh
# One job of tools/run_changed.sh: stages 1 / 1p / 1b / 2 (or the special cross/z01_hybrid / cross/x1_flagdiff job), run in the CURRENT directory (= the dsplibs-port root: the real tree
# when run_changed.sh is serial, a private sandbox tree when it runs jobs in parallel, see tools/job_sandbox.sh).  Prints exactly the per-job lines run_changed.sh has always
# printed; exit 0 = job passed (the caller writes the stamp), 1 = failed.
# Env (set by run_changed.sh): JOB_ID JOB_SCRIPT JOB_ARGS JOB_G (the stage-2 command, may be empty) NOPIN
# Not meant to be called by hand.
# TESTREORG stage 10: an old id or planned spelling in JOB_ID resolves to the active id (the callers pass the active id already)
id=$(sh tools/job_resolve.sh "$JOB_ID"); s=$JOB_SCRIPT; a=$JOB_ARGS; g=$JOB_G
. ./test/flags.sh   # PIN_FLAGS
PINENV="PATH=$PWD/test/flagshim:$PATH PORT_XFLAGS='$PIN_FLAGS'"
# H01: on a failure keep the first lines of the output as well as the last (T0P lost which call differed in a one-off tier3c failure)
failshow() { if [ $(echo "$1" | wc -l) -le 40 ]; then echo "$1"; else echo "$1" | head -25; echo "[...]"; echo "$1" | tail -15; fi; }
ok() { echo "$1" | grep -q '^MATCH' && ! echo "$1" | grep -q MISMATCH; }
case "$id" in
cross/z01_hybrid)
  t0=$(date +%s); out=$(sh $s 2>&1); rc=$?; t1=$(date +%s)
  if [ $rc -eq 0 ] && echo "$out" | grep -q '^HYBRID: all ports' && ! echo "$out" | grep -q MISMATCH; then echo "$id: MATCH ($((t1-t0))s)"; exit 0
  else echo "$id: FAIL (rc=$rc)"; failshow "$out"; exit 1; fi;;
cross/x1_flagdiff)
  t0=$(date +%s); out=$(sh $s 2>&1); rc=$?; t1=$(date +%s)
  if [ $rc -eq 0 ] && ok "$out"; then echo "$id: MATCH ($((t1-t0))s)"; exit 0
  else echo "$id: FAIL (rc=$rc)"; failshow "$out"; exit 1; fi;;
esac
t0=$(date +%s)
out=$(sh $s $a 2>&1); rc=$?
if [ $rc -eq 0 ] && ok "$out"; then
  t1=$(date +%s); res="MATCH ($((t1-t0))s)"
  # X1 stage 1p: the same script with the pinned flags (only when the script builds a port file optimised)
  if [ -z "$NOPIN" ] && grep -q -- '-O2\|OPT' $s; then
    outp=$(eval "$PINENV sh $s $a" 2>&1); rcp=$?
    if [ $rcp -eq 0 ] && ok "$outp"; then res="$res + pinned MATCH ($(( $(date +%s) - t1 ))s)"
    else echo "$id: $res but PINNED ($PIN_FLAGS) FAIL (rc=$rcp)"; echo "$outp" | tail -5; exit 1; fi
  fi
  # X1 stage 1b: plain-C tiers: 32-bit -O2 against the blob golden, plain and pinned
  bg=$(sh tools/job_path.sh $id blob32); tg=test/lib/tier_golden64.sh   # a moved job (TESTREORG): golden under test/golden/<module>/, the generic stage 2 in test/lib
  [ -n "$bg" ] || { bg=test/golden/${id}_blob32.out.gz; tg=test/lib/tier_golden64.sh; }
  [ -n "$(sh tools/job_path.sh $id golden64)" ] && bg=   # a moved job with its OWN <stem>.golden64.sh (g64=own, the dsp C++ chunks) has no plain-C stage 1b: its golden is that script's
  if [ -f "$bg" ] && [ -f $tg ]; then
    t1b=$(date +%s); okb=1
    for fl in "-O2" "-O2 $PIN_FLAGS"; do
      [ -n "$NOPIN" ] && [ "$fl" != "-O2" ] && continue
      outb=$(TG64_M=-m32 FP64_FLAGS="$fl" sh $tg $id 2>&1); rcb=$?
      { [ $rcb -eq 0 ] && echo "$outb" | grep -q '^GOLDEN64 PASS' ; } || { okb=; echo "$id: $res but 32-bit '$fl' golden FAIL (rc=$rcb)"; echo "$outb" | tail -4; break; }
    done
    [ -z "$okb" ] && exit 1
    res="$res + m32 -O2 golden ($(( $(date +%s) - t1b ))s)"
  fi
  t1=$(date +%s)
  if [ -n "$g" ]; then
    out2=$($g 2>&1); rc2=$?
    t2=$(date +%s)
    if [ $rc2 -eq 0 ] && echo "$out2" | grep -q 'GOLDEN64' && ! echo "$out2" | grep -qE 'GOLDEN64:? FAIL'; then
      echo "$out2" | grep -q 'XFAIL' && res="$res + g64 XFAIL ($((t2-t1))s)" || res="$res + g64 PASS ($((t2-t1))s)"
    else echo "$id: $res but x86-64 GOLDEN FAIL (rc=$rc2)"; echo "$out2" | tail -4; exit 1; fi
  fi
  echo "$id: $res"; exit 0
else echo "$id: FAIL (rc=$rc)"; failshow "$out"; exit 1; fi
