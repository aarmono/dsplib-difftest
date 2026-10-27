#!/bin/sh
# THE generic stage 2 of the test/ restructuring (docs/testing.md): the x86-64 golden check of every job whose row of test/jobs.tsv names a blob golden test/golden/<module>/<stem>.blob32.out.gz and has no
# own <stem>.golden64.sh.  The script, harness main and golden of a job are looked up in test/jobs.tsv (tools/job_path.sh: <module>/<stem>.sh, <stem>.main.c, golden/<module>/<stem>.blob32.out.gz), the main
# regex accepts a directory, and the work files go to test/.build/<module>/ (the ignored build directory).  It started as a copy of the flat test/tier_golden64.sh, which was deleted by stage 9 (the last
# module's move); the file stays in test/lib/ (about 200 scripts and the tools name it there).
# Generic x86-64 golden check WITHOUT the blob for the plain-C differential tests (V.34 tiers, v34_primitives, v32*, v8*, fpm, b103, rd).
# Usage: sh test/lib/tier_golden64.sh NAME     (from slmodemd/dsplibs-port/; NAME = a job id of test/jobs.tsv, e.g. ring/ring_detector; an old id such as rd resolves, TESTREORG stage 10)
#        GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh NAME   -> runs the 32-bit differential script (blob vs port), requires MATCH, and stores
#                                                           the blob's concatenated stdout as test/golden/<module>/<stem>.blob32.out.gz
#        FP64_FLAGS="-O0" / "-O3" / "-mfpmath=387" select the build flags (default -O2, SSE); no dsplibs.o, no 32-bit toolchain needed.
# What it does: derives from the 32-bit script (so the two cannot drift) the harness main (test/*_diff_main.c), the port files (every
# src/*.c named), the stub -D defines and the scenario list; builds them as x86-64 (gcc -m64 -no-pie, one object per file), runs every
# scenario in its own process and diffs the concatenated stdout against test/golden/NAME_blob32.out.gz.
# Result classes: GOLDEN64 PASS | GOLDEN64 FAIL <class> <detail> where class = compile (detail = the library / harness files that do not
# compile), link, run (a scenario crashed / exited non-zero), diff (output differs).  A failure is an EXPECTED failure (exit 0, printed
# "GOLDEN64 XFAIL") when test/golden/tier_golden64_xfail.txt has a line "NAME class detail" that matches exactly; an expected failure that
# starts to PASS (XPASS) or whose class/detail changes exits 1, so the list is the exact, current worklist of files that still need the
# 64-bit conversion (see NOTES.md "T00 DONE").  Diagnostic: the number of 'cast to/from pointer from/to integer of different size'
# warnings per port file is printed as "WARN64 file=count" lines (informational).
# T07 scenario-level xfails: lines "NAME scenario N reason" in the xfail list (one per scenario) switch NAME to per-scenario mode (x86-64 only; -m32 self-tests
# stay strict).  The golden is split per scenario with test/golden/NAME_blob32.sizes (one "scenario bytes" line per scenario of the blob's own output, written by
# GOLDEN_UPDATE); a scenario FAILS when it crashes or its output differs from its slice.  The id PASSES when exactly the listed scenarios fail: a failing scenario
# that is not listed fails the job, a listed scenario that passes (XPASS, default flags) fails the job, so the list cannot rot (other FP64_FLAGS: a listed scenario
# that passes is only reported).  Diagnostic: TG64_SCEN="13 11" runs only these scenarios and prints "SCEN NAME n same-as-golden|DIFFERS-FROM-GOLDEN" per scenario
# (with TG64_SUBST this is the mutation check: a port file substituted by a mutant must make the new scenarios differ from their golden slice).  The listed scenarios are those whose recorded blob output depends on the i386 memory layout (an index that runs past an array).
set -u
cd "$(dirname "$0")/../.."
NAME="${1:?usage: tier_golden64.sh NAME}"
# TESTREORG stage 10: NAME is the active job id = the planned name (v32/v32_engine.status ...); an old id (r28_misc) or the planned spelling resolves to it.  The xfail list and every
# message use the active id; file names made from it use the file-safe form FID ('/' -> '__').
NAME=$(sh tools/job_resolve.sh "$NAME"); FID=$(printf '%s' "$NAME" | sed 's|/|__|g')
S=$(sh tools/job_path.sh "$NAME" script) || { echo "$NAME: not a moved job (test/jobs.tsv)"; exit 2; }
[ -f "$S" ] || { echo "no such script $S"; exit 2; }
GOLD=$(sh tools/job_path.sh "$NAME" blob32); [ -n "$GOLD" ] || GOLD=test/golden/${NAME}_blob32.out.gz   # gzip -9n of the blob's concatenated stdout (some are 15 MB of text)
XF=test/golden/tier_golden64_xfail.txt

# the script text with continuation lines joined
# (cut at the optional integrated-scenario section "if [ -n "$INTEG" ]" of v34_tier9a: its second main/second file list is not part of the golden)
TXT=$(sed -e ':a' -e '/\\$/N; s/\\\n//; ta' "$S" | grep -v '^ *#' | sed '/^if \[ -n "\$INTEG" \]/,$d')
MAIN=$(echo "$TXT" | grep -o 'test/[A-Za-z0-9_/.]*\(_diff_main\|\.main\)\.c' | head -1)
FILES=$(echo "$TXT" | grep -o 'src/[A-Za-z0-9_/]*\.c' | awk '!s[$0]++')
# T03: src/core/dsp_table_handles.c = the handle registry of src/core/dsp_table_handles.h (empty at -m32), needed by every test that links a pointer-table file
FILES="$FILES
src/core/dsp_table_handles.c"
EXTRA=; echo "$TXT" | grep -q 'test/stub_env_str\.c' && EXTRA=test/stub_env_str.c
# -D defines: taken from the FIRST line of the 32-bit script that mentions stub_env.c (minus -DTEST_BLOB).  If that line is a "-c" compile of the stub
# (renames such as -Dsysdep_malloc=...) they apply to the stub only; if it is a combined compile+link line they apply to every source file.
SLINE=$(echo "$TXT" | grep -m1 'test/stub_env\.c')
DEFS=$(echo "$SLINE" | grep -o ' -D[A-Za-z_0-9]*\(=[A-Za-z_0-9]*\)\?' | grep -v '^ -DTEST_BLOB$' | awk '!s[$0]++' | tr '\n' ' ')
case "$SLINE" in *" -c "*) DEFS_ALL=;; *) DEFS_ALL="$DEFS"; DEFS=;; esac
SC=$(echo "$TXT" | grep -m1 -o '^ *SCENARIOS="[^"]*"' | sed 's/^ *SCENARIOS="\(\${SCENARIOS:-\)\{0,1\}//; s/}\{0,1\}"$//')
if [ -z "$SC" ]; then
  n=$(echo "$TXT" | grep -m1 -o '^ *NUM_SCENARIOS=[0-9]*' | sed 's/.*=//')
  [ -n "$n" ] && SC=$(seq -s ' ' 0 $((n - 1)))
fi
[ -n "$MAIN" ] || { echo "no harness main found in $S"; exit 2; }
FULLSC="$SC"   # all scenarios of the golden (the per-scenario slicing below needs every offset)

if [ -n "${GOLDEN_UPDATE:-}" ]; then
  out=$(sh $S 2>&1) || { echo "$out" | tail -5; echo "GOLDEN_UPDATE: the 32-bit differential script did not MATCH; golden not written"; exit 1; }
  echo "$out" | grep -q '^MATCH' || { echo "$out" | tail -5; echo "GOLDEN_UPDATE: no MATCH line"; exit 1; }
  # the script leaves the blob's concatenated stdout in <T>_blob.out; T is test/<something>
  T=$(echo "$TXT" | grep -m1 -o '^ *T=[^ ]*' | sed 's/^ *T=//; s/"//g; s/\${TPREFIX:-\(.*\)}/\1/')
  if [ -n "$T" ]; then B=${T}_blob.out; else B=$(echo "$TXT" | grep -o 'test/[A-Za-z0-9_]*_blob\.out' | head -1); fi
  [ -f "$B" ] || { echo "GOLDEN_UPDATE: cannot find the blob output ($B)"; exit 1; }
  if awk -v n="$NAME" '$1 == n && $2 == "scenario" { f = 1 } END { exit !f }' $XF 2>/dev/null || [ -n "${GOLDEN_SIZES:-}" ]; then
    # per-scenario sizes of the blob's own output (the blob test binary the script built), checked against the concatenation
    BB=$(echo "$TXT" | grep -o '[^ "]*_test_blob' | head -1)
    case "$BB" in *'$'*) BB=$(echo "$BB" | sed "s#\${\{0,1\}T}\{0,1\}#$T#");; esac
    [ -x "$BB" ] || { echo "GOLDEN_UPDATE: no blob test binary $BB"; exit 1; }
    SZF=$(sh tools/job_path.sh "$NAME" sizes)
    : > $SZF.tmp; tot=0
    for i in $SC; do sz=$(./$BB $i 2>/dev/null | wc -c); echo "$i $sz" >> $SZF.tmp; tot=$((tot + sz)); done
    [ "$tot" = "$(wc -c < $B)" ] || { rm -f $SZF.tmp; echo "GOLDEN_UPDATE: per-scenario sizes ($tot) do not add up to the blob output ($(wc -c < $B))"; exit 1; }
    mv $SZF.tmp $SZF
  fi
  gzip -9nc "$B" > "$GOLD"; echo "GOLDEN_UPDATE: $GOLD written from $B ($(wc -l < $B) lines, $(wc -c < $GOLD) bytes gz)"; exit 0
fi
[ -f "$GOLD" ] || { echo "GOLDEN64 FAIL nogolden $GOLD missing (GOLDEN_UPDATE=1 sh $0 $NAME)"; exit 1; }

# diagnostic only: TG64_SUBST="src/<module>/a.c=/path/b.c ..." builds /path/b.c instead of src/<module>/a.c (e.g. a scratch copy of a table file with the
# 32-bit pointer values zeroed, to see what else would fail once the table file compiles); such a run is never an expected failure
for p in ${TG64_SUBST:-}; do FILES=$(echo "$FILES" | sed "s|^${p%%=*}\$|${p#*=}|"); done
BD=$(sh tools/job_path.sh "$NAME" build); mkdir -p "$(dirname "$BD")"
T="${TPREFIX:-$(dirname "$BD")/tg64_$FID}"
case "$T" in /*) T_RUN= ;; *) T_RUN=./ ;; esac   # an absolute TPREFIX must not get a "./" in front
OPT="-O2"
case "${FP64_FLAGS:-}" in *-O*) OPT="";; esac
M="${TG64_M:--m64}"   # TG64_M=-m32 builds the same harness as 32-bit: a self-test of the derived file lists / defines / scenarios (must PASS everywhere)
CF="$M -w -fno-stack-protector -Isrc $OPT ${FP64_FLAGS:-}"
rm -f ${T}_*.o ${T}_test ${T}.out ${T}.diff ${T}.log
# T07: scenario-level xfail list of this id (x86-64 only, no diagnostic substitution)
SXL=
SCMODE=
if [ -n "${TG64_SCEN:-}" ]; then SCMODE=1; SC="$TG64_SCEN"; fi   # diagnostic (mutation checks): run only these scenarios, report each against its golden slice
if [ "$M" = -m64 ] && [ -z "${TG64_SUBST:-}" ]; then SXL=$(awk -v n="$NAME" '$1 == n && $2 == "scenario" && $3 ~ /^[0-9]+$/ { print $3 }' $XF 2>/dev/null | tr '\n' ',' | sed 's/,$//'); fi
# the port files get the requested flags; the harness main / stub are always built -O0 (v23's harness itself changes its output at -O2 with the 32-bit
# port at -O0: the harnesses are test code, not under test; the 32-bit scripts build them -O0 as well)
HF="$M -w -fno-stack-protector -Isrc -O0"
OBJS=; BAD=; n=0
for f in $FILES $MAIN test/stub_env.c $EXTRA; do
  n=$((n + 1)); o=${T}_$n.o; d="$DEFS_ALL"; flags="$CF"
  case "$f" in test/stub_env.c) d="$DEFS_ALL $DEFS";; esac
  case "$f" in src/*) ;; *) flags="$HF";; esac
  if gcc $flags $d -c $f -o $o 2>>${T}.log; then OBJS="$OBJS $o"; else BAD="$BAD $f"; fi
done
# informational: pointer/integer width warnings per port file (compile-only)
for f in $FILES; do
  c=$(gcc $M -Wint-to-pointer-cast -Wpointer-to-int-cast -Isrc -O0 -fsyntax-only $f 2>&1 | grep -c 'warning: cast \(to pointer from integer\|from pointer to integer\)')
  [ "$c" -gt 0 ] && echo "WARN64 $f=$c"
done
[ -n "$SCMODE" ] && [ -z "$SXL" ] && SXL=-   # diagnostic mode is always per scenario
CLASS=; DETAIL=
if [ -n "$BAD" ]; then CLASS=compile; DETAIL=$(echo $BAD | tr ' ' ','); fi
if [ -z "$CLASS" ]; then
  if ! gcc $M -no-pie -o ${T}_test $OBJS -lm 2>>${T}.log; then CLASS=link; DETAIL=$(grep -o "undefined reference to \`[A-Za-z0-9_]*'" ${T}.log | sort -u | sed "s/.*\`//; s/'//" | head -5 | tr '\n' ',' | sed 's/,$//'); fi
fi
if [ -z "$CLASS" ]; then
  : > ${T}.out
  bad=
  for i in ${SC:-NOARG}; do
    if [ -n "$SXL" ]; then
      timeout 300 ${T_RUN}${T}_test $i > ${T}.sc.$i 2>>${T}.log || { bad="$bad $i"; echo "$i" >> ${T}.crash; }
      cat ${T}.sc.$i >> ${T}.out
    elif [ "$i" = NOARG ]; then timeout 300 ${T_RUN}${T}_test >> ${T}.out 2>>${T}.log || bad="$bad main"
    else timeout 300 ${T_RUN}${T}_test $i >> ${T}.out 2>>${T}.log || bad="$bad $i"; fi
  done
  if [ -n "$bad" ] && [ -z "$SXL" ]; then CLASS=run; DETAIL="scenario$(echo $bad | tr ' ' ',' | sed 's/^/ /')"; fi
fi
if [ -z "$CLASS" ] && [ -n "$SXL" ]; then
  # T07 per-scenario mode: slice the golden with NAME_blob32.sizes, a scenario fails when it crashed or differs from its slice
  SZ=$(sh tools/job_path.sh "$NAME" sizes)
  [ -f "$SZ" ] || { echo "GOLDEN64 FAIL nosizes $SZ missing (GOLDEN_UPDATE=1 sh $0 $NAME)"; exit 1; }
  gzip -dc $GOLD > ${T}.gold
  off=0; FAILS=
  for i in $FULLSC; do
    sz=$(awk -v s=$i '$1 == s {print $2}' $SZ)
    [ -n "$sz" ] || { echo "GOLDEN64 FAIL nosizes scenario $i not in $SZ"; exit 1; }
    tail -c +$((off + 1)) ${T}.gold | head -c $sz > ${T}.g.$i
    off=$((off + sz))
    case " $SC " in *" $i "*) ;; *) continue;; esac   # TG64_SCEN: only the selected scenarios were run
    if grep -qx "$i" ${T}.crash 2>/dev/null || ! cmp -s ${T}.g.$i ${T}.sc.$i; then FAILS="$FAILS $i"; fi
  done
  [ "$off" = "$(wc -c < ${T}.gold)" ] || { echo "GOLDEN64 FAIL nosizes $SZ does not add up to the golden"; exit 1; }
  if [ -n "$SCMODE" ]; then   # diagnostic: one line per selected scenario, never an expected failure / pass
    for i in $SC; do case " $FAILS " in *" $i "*) echo "SCEN $NAME $i DIFFERS-FROM-GOLDEN";; *) echo "SCEN $NAME $i same-as-golden";; esac; done
    [ -n "${TG64_KEEP:-}" ] || rm -f ${T}_*.o ${T}_test
    rm -f ${T}.gold ${T}.g.* ${T}.sc.* ${T}.crash ${T}.out ${T}.log ${T}.diff; exit 0
  fi
  UNEXP=; FIXED=
  for i in $FAILS; do case ",$SXL," in *",$i,"*) ;; *) UNEXP="$UNEXP $i";; esac; done
  for i in $(echo $SXL | tr ',' ' '); do case " $FAILS " in *" $i "*) ;; *) FIXED="$FIXED $i";; esac; done
  [ -n "${TG64_KEEP:-}" ] || rm -f ${T}_*.o ${T}_test
  rm -f ${T}.gold ${T}.g.* ${T}.sc.* ${T}.crash
  if [ -n "$UNEXP" ]; then echo "GOLDEN64 FAIL $NAME scenario-diff${UNEXP} (not in the scenario xfail list of $XF; failing:${FAILS}; log ${T}.log)"; exit 1; fi
  if [ -n "$FIXED" ]; then
    if [ -z "${FP64_FLAGS:-}" ]; then echo "GOLDEN64 XPASS $NAME scenario${FIXED} now passes ($OPT): remove it from $XF"; exit 1; fi
    echo "note: listed scenario${FIXED} passes at ${FP64_FLAGS} (flag dependent, not an error here)"
  fi
  echo "GOLDEN64 PASS: the $M port ($OPT ${FP64_FLAGS:-}) reproduces the 32-bit blob's output for every scenario of ${SC} except the designed scenario xfails ${SXL} (layout dependent, see $XF)"
  rm -f ${T}.out ${T}.log ${T}.diff; exit 0
fi
if [ -z "$CLASS" ]; then
  if gzip -dc $GOLD | diff -u - ${T}.out > ${T}.diff; then CLASS=pass; else CLASS=diff; DETAIL="$(grep -c '^[<>+-][^+-]' ${T}.diff) lines"; fi
fi
[ -n "${TG64_KEEP:-}" ] || rm -f ${T}_*.o ${T}_test   # TG64_KEEP=1 keeps the 64-bit test binary (for gdb)
if [ $CLASS = pass ]; then
  if [ "$M" = -m64 ] && [ -z "${TG64_SUBST:-}" ] && awk -v n="$NAME" '$1 == n { f = 1 } END { exit !f }' $XF 2>/dev/null; then
    if [ -z "${FP64_FLAGS:-}" ]; then echo "GOLDEN64 XPASS $NAME now passes ($OPT): remove it from $XF"; rm -f ${T}.out ${T}.log ${T}.diff; exit 1; fi
    echo "GOLDEN64 PASS (listed as expected failure at the default flags; flag dependent): $NAME ($OPT ${FP64_FLAGS:-})"; rm -f ${T}.out ${T}.log ${T}.diff; exit 0
  fi
  echo "GOLDEN64 PASS: the $M port ($OPT ${FP64_FLAGS:-}) reproduces the 32-bit blob's output (scenarios: ${SC:-none}) ($GOLD)"
  rm -f ${T}.out ${T}.log ${T}.diff; exit 0
fi
# exact match (class AND detail) for the default flags (-O2 SSE); for any other FP64_FLAGS only NAME and CLASS are compared (UB makes the crashing scenarios flag dependent)
XMATCH=false
if [ "$M" = -m64 ] && [ -z "${TG64_SUBST:-}" ]; then
  if [ -z "${FP64_FLAGS:-}" ]; then grep -qxF "$NAME $CLASS $DETAIL" $XF 2>/dev/null && XMATCH=true
  else awk -v n="$NAME" -v c="$CLASS" '$1 == n && $2 == c { f = 1 } END { exit !f }' $XF 2>/dev/null && XMATCH=true; fi
fi
if $XMATCH; then
  echo "GOLDEN64 XFAIL $NAME $CLASS $DETAIL (expected, see $XF)"; rm -f ${T}.out ${T}.log ${T}.diff; exit 0
fi
echo "GOLDEN64 FAIL $NAME $CLASS $DETAIL (not in $XF; log ${T}.log, diff ${T}.diff)"
exit 1
