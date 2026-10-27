#!/bin/sh
# Re-runs only the differential tests whose inputs changed since their last MATCH (hash-stamped; stamps in test/.stamps, ignored by git).
# Usage: sh tools/run_changed.sh [--all] [--dry-run] [--list] [--g64] [--src-keys] [--keys [--key-info]] [id ...]     (from anywhere; output: one line per test)
#   --all      ignore the stamps, run everything     --dry-run   list what would run, run nothing
#   --list     list every job (id, script, golden64 stage) and the scripts that are NOT covered, with the reason; run nothing
#   --stamp    do not run: record the CURRENT stage-1 key of the named ids as verified (for changes proven behaviour-neutral by other evidence, e.g. identical object code; say why in the commit/NOTES)
#   --g64      also run the x86-64 golden stage (stage 2); OFF by default (its XFAILs are unchanged until a job's files are converted to 64-bit).  Use with explicit ids.  A stamp made with stage 2 also satisfies a run without it.
#   id ...     only these jobs.  TESTREORG stage 10: the ids are the planned names of test/jobs.tsv (ring/ring_detector, vpcm/pcm_parameters.main, v32/v32_engine.status.o2 ...); the old ids (rd, p07, v34_tier2, n01_v32status_O2 ...)
#              are still accepted and resolved through tools/job_aliases.tsv.  An id contains '/' and '.': every FILE NAME made from an id (stamp test/.stamps/<fid>, key log, job log, sandbox, mode file) uses the
#              file-safe form fid = id with each '/' replaced by '__' (ring/ring_detector -> ring__ring_detector); jobs_check.sh verifies that no id contains '__' so the form is injective.
#              The stamps of the old ids (test/.stamps/rd ...) are NOT used and not touched: a job under its new id has no stamp until it runs.
# JOBS (one per test/build_and_diff_<id>.sh; the jobs MOVED into test/<module>/ (TESTREORG stage 2, docs/testing.md; today: rd = test/ring/ring_detector.sh) are read from test/jobs.tsv; "adid" is run through its p18/p19/p20 wrappers;
# since stage 8 n01_v32status / n01_v32status_O2 are two moved jobs: v32/v32_engine.status.sh, and v32/v32_engine.status.o2.sh which runs it with -O2).  STAGE 1 = the 32-bit blob-vs-port script (must exit 0, print MATCH, no MISMATCH).
# STAGE 2 = the x86-64 golden (no blob): test/<id>_golden64.sh if it exists, else test/lib/tier_golden64.sh <id> if the job's row names a <module>/<stem>.blob32.out[.gz]
# (default flags -O2 SSE).  Stage 2 passes on "GOLDEN64 PASS" or on a documented expected failure ("GOLDEN64 XFAIL", test/golden/tier_golden64_xfail.txt).
# STAMPKEYS (this scheme replaces the source-text key below as the DEFAULT): a stamp means "the same COMPILED port code (and the same script / harness text / goldens) as a
#   run that passed", not "run on this source text".  The library .c/.cpp files and headers are no longer hashed as text: the key hashes what every compile unit of the
#   job's recorded recipe (tools/stampkeys.py: real compile lines captured with a logging shim, stages 1, 1p, 1b, 2) compiles to - per function / data object, relocations as
#   symbol names, order- and move-tolerant - so a comment edit, a moved prototype or a function moved between files of the job keeps its stamps.  Jobs whose recipe cannot
#   be recorded reliably fall back to the source-text key (marker line keyscheme=stampkeys-v1:fallback-source-text).  --src-keys = the old source-text key exactly (comparison /
#   debugging); --keys --key-info adds "[obj ...]" or "[src (fallback: reason)]" per job.  Cache: $STAMPKEYS_CACHE (default ~/.cache/dsplibs-stampkeys, cap $STAMPKEYS_CAP_MB = 250).
#   The first run after the switch re-keys every job once (new key scheme).  Details and blind spots: tools/README.md "STAMPKEYS".
# Key of a job = sha256 over: gcc version; the script text; test/stub_env.c, test/stub_env_str.c, ../dsplibs.o, test/harness/*; every repo file the script
# names (test/..., src/..., include/...; a src/ path without extension = its .c / .cpp, as the computed loops spell them; scripts it runs, e.g. the adid / p29
# wrappers, are followed recursively); every file test/<tok>_* test/<tok>.layout test/golden/<tok>_* and the library files of the token's line in
# test/libfiles_by_token.txt for each pNN/vNN/zNN token in a followed script; the header closure (gcc -MM) of the .c/.cpp files among them
# (generated outputs - *.out outside test/golden, *.o, *.diff, *.log, *_test*, *_blob, *_port - are left out).  The stage-2 key adds the golden64 script(s),
# the golden file and the xfail list.  The key is conservative: a comment edit re-runs, an unrelated chunk's edit does not.
# X1 stages (flag set defined in test/flags.sh): 1p = the same script again with PORT_XFLAGS=$PIN_FLAGS through test/flagshim (the PINNED set; only for scripts that
# compile a port file with -O2 / OPT - a -O0-only script cannot depend on -fno-strict-aliasing); 1b = jobs with a blob golden (the plain-C tiers): the 32-bit -O2 build of
# the port against the golden (tier_golden64.sh with TG64_M=-m32), plain and pinned.  --matrix additionally runs stage 2 of every job with the x86-64 flag matrix (-O0,
# -O2 -mfpmath=387, and ONE -O3 job: p43) - slow (hours) and stamped separately.  --no-pin skips 1p / 1b-pinned.
# A stamp is written only after stage 1 (and stage 2) passed.
# RS4 (user decision 5): keys are computed on the CURRENT paths and text of this tree (the RS3 canonical mirror is gone): the RS4 move (src/<module>/)
# invalidated every stamp once, by design.  The association job token -> library files that the old flat file names (<tok>_*) gave is test/libfiles_by_token.txt.
# --keys   print "id key" per job (the stage-1 key, or "KB:KF" with --g64) and run nothing; RUN_CHANGED_KEYLOG=DIR writes the hashed lines of every key to DIR/<id>.
# -j N / --jobs N / env JOBS (default 1 = serial, in this tree, exactly as before T0P): run independent jobs in parallel, each in a private sandbox copy of the tree
# (tools/job_sandbox.sh), at most N cores' worth at once (weights, exclusive lane: test/jobclass.txt; scheduler tools/run_pool.py); per-job logs + timing.tsv under
# $RUN_LOGS (kept with RUN_KEEP_LOGS=1, always kept on failure); output order, summary line and stamps as in the serial run.  --matrix stays serial (after the jobs).  --strict: jobs of class 'par' (internally parallel) run alone.  After a parallel run the real tree is audited (nothing but the stamps may have been written).
cd "$(dirname "$0")/.." || exit 1
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
ALL=; DRY=; WANT=; LIST=; NOG=; NOPIN=; MATRIX=; STRICT=; NJOBS=${JOBS:-1}
while [ $# -gt 0 ]; do a=$1; shift; case "$a" in -j|--jobs) NJOBS=$1; shift;; -j[0-9]*) NJOBS=${a#-j};; --all) ALL=1;; --dry-run) DRY=1;; --list) LIST=1;; --no-g64) NOG=1;; --g64) G64=1;; --stamp) STAMP=1;; --no-pin) NOPIN=1;; --matrix) MATRIX=1;; --strict) STRICT=1;; --keys) KEYS=1;; --src-keys) SRC_KEYS=1;; --key-info) KINFO=1;; -*) echo "usage: $0 [-j N] [--all] [--dry-run] [--keys [--key-info]] [--src-keys] [--list] [--g64] [--no-pin] [--strict] [--matrix] [id ...]"; exit 2;; *) WANT="$WANT $a";; esac; done
WANT=$(sh tools/job_resolve.sh $WANT | tr "\n" " ")   # TESTREORG: old ids, new ids and planned names (tools/job_aliases.tsv) all select the same job (since stage 10 the new id = the planned name)
fid() { printf '%s' "$1" | sed 's|/|__|g'; }   # file-safe form of a job id (stamps, key logs, job logs, sandboxes): '/' -> '__'; ids contain no '__' (jobs_check.sh)
mkdir -p test/.stamps
. ./test/flags.sh   # PIN_FLAGS
PINENV="PATH=$PWD/test/flagshim:$PATH PORT_XFLAGS='$PIN_FLAGS'"
GCCV=$(gcc --version | head -1)

# job table: "id|script|args"
jobs() {
  # TESTREORG stage 2: the jobs whose files were moved into test/<module>/ come from test/jobs.tsv (first script of the row has a '/'); the others are found by glob as before
  awk -F'\t' '!/^#/ && $1 != "id" && $1 != "cross/z01_hybrid" && $1 != "cross/x1_flagdiff" { split($4, s, ","); if (s[1] ~ /\//) { a = ""; if (match($8, /args=[^ ;]+/)) a = substr($8, RSTART + 5, RLENGTH - 5); print $1 "|test/" s[1] "|" a } }' test/jobs.tsv
  [ -f test/build_and_diff.sh ] && echo "rd|test/build_and_diff.sh|"
  for s in test/build_and_diff_*.sh; do
    [ -f "$s" ] || continue   # no flat script is left since stage 9: the unmatched glob would become a job called "*" (and then a file name in the unquoted loops below)
    n=$(basename $s .sh); n=${n#build_and_diff_}
    case "$n" in
      adid) continue;;   # needs ROW=: covered through build_and_diff_p18/p19/p20 (they exec it with ROW)
    esac
    echo "$n|$s|"
  done
}
# generated / volatile files that must not be part of a key
volatile() { case "$1" in test/golden/*) return 1;; test/.build/*) return 0;; *.out|*.diff|*.o|*.log|*_test|*_test_*|*_blob|*_port|*.stamp) return 0;; esac; return 1; }
# files an (followed) script depends on
collect() {
  s=$1
  echo $s
  [ "${CDEPTH:-0}" -ge 3 ] && return   # scripts may name each other (p29 / p29a, comments): bounded recursion
  # (the module directories of src/ - v22/ v32/ v34/ - are not tokens: the directory part of a src/ path is dropped before the token scan)
  # (an "@tok path" line of test/libfiles_by_token.txt: a script naming that file also names the token, as its old flat name did: v22.c, v32.c up to RS3)
  for tok in $({ sed -E 's#(^|[^A-Za-z0-9_])src/([A-Za-z0-9_]+/)+#\1#g' $s | grep -o "\b[pvz][0-9][0-9]\b"
                 sed -n 's/^@\([pvz][0-9][0-9]\) /\1 /p' test/libfiles_by_token.txt | while read t p; do grep -q "$p" $s && echo $t; done; } | sort -u); do
    ls test/${tok}_* test/${tok}.layout test/golden/${tok}_* 2>/dev/null
    sed -n "s/^$tok //p" test/libfiles_by_token.txt | tr ' ' '\n'
  done
  for f in $(grep -o '\b\(test\|src\|include\)/[A-Za-z0-9_./-]*' $s | sort -u); do
    case "$f" in src/*.*) ;; src/*) for x in $f.c $f.cpp; do [ -f $x ] && echo $x; done; continue;; esac   # computed loops: src/<module>/<stem>
    [ -f "$f" ] || continue
    echo $f
    case "$f" in test/build_and_diff*.sh|test/*_golden64.sh|test/[a-z0-9_]*/[A-Za-z0-9_.]*.sh) [ "$f" != "$s" ] && ( CDEPTH=$((${CDEPTH:-0}+1)); collect $f );; esac
  done
}
MMC=${TMPDIR:-/tmp}/run_changed_mm.$$; mkdir -p $MMC; trap 'rm -rf $MMC' EXIT   # per-run cache of the header closure of each source file
KROOT=$PWD   # where keys are computed (RS3 used a canonical mirror of the tree here; RS4 re-keyed to the current paths)
mmdeps() { # file -> its gcc -MM closure (cached for this run)
  c=$MMC/$(echo "$1" | tr '/' '_')
  [ -f "$c" ] || { case "$1" in *.cpp) x="-x c++";; *) x=;; esac; gcc -m32 -MM -Isrc -Itest -Itest/harness $x $1 2>/dev/null | tr -d '\\\n' | cut -d: -f2- > $c; }
  cat $c
}
key_files() { # id script [extra files] -> the files the (old, source-text) key of the job hashes, one list (computed in $KROOT)
  s=$2; shift 2
  tg=test/lib/tier_golden64.sh   # the generic stage 2 (the flat test/tier_golden64.sh was deleted with the last module's move, TESTREORG stage 9)
  files="test/flags.sh test/flagshim/gcc test/flagshim/g++ $tg test/stub_env.c test/stub_env_str.c $BLOB_O $(ls test/harness/* 2>/dev/null) $(collect $s) $*"
  for f in $files; do volatile $f || echo $f; done | sort -u
}
# STAMPKEYS: the key of a job is by default the key of its COMPILED CODE (tools/stampkeys.py: object hashes of every compile unit the job builds, recorded recipes,
# source hashes only for what is not port code); --src-keys = the old SOURCE-TEXT key (tools/README.md "STAMPKEYS").  A job whose recipe cannot be recorded reliably
# falls back to the source-text key (with a marker line, so its stamp is not mistaken for an old one); `--keys --key-info` says which.
SKREQ=$MMC/sk.req; SKRES=$MMC/sk.res
key() { # id script [extra files]   (computed in $KROOT)
  if [ -z "$SRC_KEYS" ]; then
    kind=KB; [ "$KEYTAG" = .g64 ] && kind=KF
    if [ -n "$SKPREP" ]; then   # request pass: record what the object-key pass needs (job, stage commands, file list)
      printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$1" $kind "$2" "$KARGS" "$KG" "$( cd $KROOT && key_files "$@" | tr '\n' ' ')" >> $SKREQ; echo "-"; return
    fi
    r=$(awk -F'\t' -v id="$1" -v k=$kind '$1==id && $2==k {print; exit}' $SKRES 2>/dev/null)
    case "$r" in
      *"	OBJ	"*) echo "$r" | cut -f4; echo "obj $(echo "$r" | cut -f5)" > $MMC/mode.$(fid $1)$KEYTAG; return;;
      *"	FALLBACK	"*) echo "src (fallback: $(echo "$r" | cut -f4))" > $MMC/mode.$(fid $1)$KEYTAG; KMARK="keyscheme=stampkeys-v1:fallback-source-text";;
      *) echo "src (no stampkeys result)" > $MMC/mode.$(fid $1)$KEYTAG; KMARK="keyscheme=stampkeys-v1:fallback-source-text";;
    esac
  fi
  ( cd $KROOT && key_here "$@" )
}
key_here() {
  id=$1; s=$2; shift 2
  files=$(key_files $id $s "$@")
  deps=
  for f in $files; do case "$f" in *.c|*.cpp) deps="$deps $(mmdeps $f)";; esac; done
  { [ -n "$KMARK" ] && echo "$KMARK"; echo "$GCCV"; echo "id=$id"; for f in $files $deps; do [ -f "$f" ] && sha256sum "$f"; done | sort -u; } > $MMC/keyin
  [ -n "$RUN_CHANGED_KEYLOG" ] && { mkdir -p "$RUN_CHANGED_KEYLOG"; cp $MMC/keyin "$RUN_CHANGED_KEYLOG/$(fid $id)$KEYTAG"; }
  sha256sum < $MMC/keyin | cut -c1-32
}
g64_cmd() { # id -> the stage-2 command (empty if none)
  if [ -n "$(sh tools/job_path.sh $1 moved)" ]; then   # moved job (TESTREORG): own <stem>.golden64.sh, else the generic stage 2 when the row lists a blob32 golden
    gs=$(sh tools/job_path.sh $1 golden64)
    if [ -n "$gs" ]; then echo "sh $gs"; elif [ -n "$(sh tools/job_path.sh $1 blob32)" ]; then echo "sh test/lib/tier_golden64.sh $1"; fi
  elif [ -f test/$1_golden64.sh ]; then echo "sh test/$1_golden64.sh"
  elif [ -f test/golden/$1_blob32.out.gz ] && [ -f test/lib/tier_golden64.sh ]; then echo "sh test/lib/tier_golden64.sh $1"; fi   # (dead since stage 9: every job is moved)
}
g64_files() { # id -> files the stage-2 outcome depends on (besides the stage-1 key files)
  if [ -n "$(sh tools/job_path.sh $1 moved)" ]; then
    gs=$(sh tools/job_path.sh $1 golden64)
    if [ -n "$gs" ]; then collect $gs; else echo "test/lib/tier_golden64.sh test/golden/tier_golden64_xfail.txt $(sh tools/job_path.sh $1 blob32)"; fi
  elif [ -f test/$1_golden64.sh ]; then collect test/$1_golden64.sh
  else echo "test/lib/tier_golden64.sh test/golden/tier_golden64_xfail.txt test/golden/$1_blob32.out.gz"; fi
}
uncovered() {
  cat <<'EOF'
test/vpcm/v90_adid.shared.sh, v90_adid.shared.golden64.sh   not jobs of their own: need ROW=; run through the wrappers vpcm/v90_adid.sh / v90_adid_pad_gain.sh / v90_adid_study.sh (stage 1: p18 / p19 / p20) and their .golden64.sh (stage 2)
test/cross/h01_datasym.build_objs.sh   object builder of the job h01_datasym (was n01_build_hybrid.sh); not a test itself
test/cross/z01_hybrid.sh           job "z01_hybrid" below (runs after z01, needs z01's objects)  [listed here only when it is skipped]
test/p03_study.sh                  3-4 minute transcendental study (prints statistics, no MATCH criterion)
test/dsp/dsp_resampler.vtable_cmp.py, test/vpcm/pcm_parameters.offsets_check.py, v92_cp.offsets_check.py, v92_tx_coding.table_hash.py, test/v32/v32_engine.uninit.cmp.py, test_member_map.py, test_tier_rewrite.sh   helpers called by other scripts / by hand
EOF
}

if [ -n "$LIST" ]; then
  jobs | while IFS='|' read id s a; do echo "$id: stage1 $s $a; stage2 $(g64_cmd $id | grep . || echo none)"; done
  echo "cross/z01_hybrid: test/cross/z01_hybrid.sh (stage 1 only)"
  echo "cross/x1_flagdiff: test/cross/x1_flagdiff.sh (stage 1 only; planned by run_changed itself, not a build_and_diff script)"
  echo "--- NOT covered by run_changed:"; uncovered
  exit 0
fi

ran=0; skipped=0; failed=0; xf=0
# --- plan: which jobs run (stamp check), in the fixed job order; every entry: id|script|args|g64 command|key (one per line in $PLAN, '|' separated)
PLAN=$MMC/plan; : > $PLAN
plan_job() { # id script args g key
  st=test/.stamps/$(fid $1)
  if [ -n "$KEYS" ]; then echo "$1 $5$([ -n "$KINFO" ] && echo " [$(cat $MMC/mode.$(fid $1) 2>/dev/null)]")"; return; fi
  # stamp = "KB" (stage 1 only) or "KB:KF" (stage 1 + stage 2); KB hashes the stage-1 inputs only, so editing the xfail list never invalidates stage 1
  if [ -z "$ALL" ] && { [ "$(cat $st 2>/dev/null)" = "$5" ] || { [ -z "$4" ] && [ "$(cat $st 2>/dev/null | cut -d: -f1)" = "$5" ]; }; }; then echo "$1: unchanged (skipped)"; skipped=$((skipped+1)); return; fi
  if [ -n "$DRY" ]; then echo "$1: WOULD RUN"; return; fi
  if [ -n "$STAMP" ]; then printf '%s\n' "$5" | cut -d: -f1 > $st; echo "$1: STAMPED (not run)"; return; fi
  printf '%s|%s|%s|%s|%s\n' "$1" "$2" "$3" "$4" "$5" >> $PLAN
}
keypass() { # one pass over the selected jobs: plan_job for each (SKPREP=1: only the key() calls, which write the stampkeys requests)
for j in $(jobs); do
  id=$(echo $j | cut -d'|' -f1); s=$(echo $j | cut -d'|' -f2); a=$(echo $j | cut -d'|' -f3)
  [ -n "$WANT" ] && case " $WANT " in *" $id "*) ;; *) continue;; esac
  g=$(g64_cmd $id); KARGS=$a; KG=$g; kb=$(key $id $s)
  if [ -n "$SKPREP" ]; then [ -n "$g" ] && KEYTAG=.g64 key $id $s $(cd $KROOT && g64_files $id) >/dev/null; continue; fi   # the recipe is keyed on the larger (stage 2) file set whether or not --g64 is given
  if [ -n "$G64" ] && [ -z "$NOG" ] && [ -n "$g" ]; then plan_job "$id" "$s" "$a" "$g" "$kb:$(KEYTAG=.g64 key $id $s $(cd $KROOT && g64_files $id))"
  else plan_job "$id" "$s" "$a" "" "$kb"; fi
done
# z01 hybrid dry run (stage 1 only): needs the objects of the z01 job, so it runs after it
if [ -z "$WANT" ] || case " $WANT " in *" cross/z01_hybrid "*) true;; *) false;; esac; then
  s=test/cross/z01_hybrid.sh; KARGS=; KG=
  k=$(key cross/z01_hybrid $s test/vpcm/dp_vpcm.graph.main.c test/golden/vpcm/dp_vpcm.graph.blob32.out sources.mk $(cd $KROOT && find src/ -type f | LC_ALL=C sort ))
  [ -n "$SKPREP" ] || plan_job cross/z01_hybrid $s "" "" "$k"
fi
# x1_flagdiff (stage 1 only): -fno-strict-aliasing sensitive object list unchanged and -fno-lifetime-dse changes no code (see test/flags.sh)
if [ -z "$WANT" ] || case " $WANT " in *" cross/x1_flagdiff "*) true;; *) false;; esac; then
  s=test/cross/x1_flagdiff.sh; KARGS=; KG=
  if [ -n "$SRC_KEYS" ]; then
    k="$( cd $KROOT && { echo "$GCCV"; sha256sum $s test/golden/cross/x1_flagdiff.txt sources.mk $(find src/ -type f | LC_ALL=C sort) ; } > $MMC/keyin; [ -n "$RUN_CHANGED_KEYLOG" ] && mkdir -p "$RUN_CHANGED_KEYLOG" && cp $MMC/keyin "$RUN_CHANGED_KEYLOG/cross__x1_flagdiff"; sha256sum < $MMC/keyin | cut -c1-32 )"
  else k=$(key cross/x1_flagdiff $s test/golden/cross/x1_flagdiff.txt sources.mk $(cd $KROOT && find src/ -type f | LC_ALL=C sort)); fi
  [ -n "$SKPREP" ] || plan_job cross/x1_flagdiff $s "" "" "$k"
fi
}
if [ -z "$SRC_KEYS" ] && [ -z "$LIST" ]; then   # object keys: request pass, one tools/stampkeys.py run for all selected jobs, then the real pass reads its result
  : > $SKREQ; SKPREP=1; keypass; SKPREP=
  SK_BLOB="$BLOB_O" SK_GCCV="$GCCV" STAMPKEYS_KEYLOG="$RUN_CHANGED_KEYLOG" python3 tools/stampkeys.py keys $SKREQ $SKRES -j ${SK_J:-$(nproc)} || { echo "run_changed: tools/stampkeys.py failed; use --src-keys for the old source-text keys" >&2; exit 2; }
fi
keypass
# --- execute the plan
account() { # id rc  (output of the job already printed)
  if [ "$2" -eq 0 ]; then ran=$((ran+1)); else failed=$((failed+1)); fi
}
if [ -s $PLAN ] && [ "$NJOBS" -le 1 ]; then
  # serial: in this tree, one job after the other (the behaviour before T0P)
  while IFS='|' read -r id s a g k; do
    st=test/.stamps/$(fid $id)
    out=$(JOB_ID=$id JOB_SCRIPT=$s JOB_ARGS=$a JOB_G=$g NOPIN=$NOPIN sh tools/run_job.sh 2>&1 </dev/null); rc=$?
    echo "$out"
    if [ $rc -eq 0 ]; then echo "$k" > $st; else rm -f $st; fi
    echo "$out" | grep -q 'g64 XFAIL' && [ $rc -eq 0 ] && xf=$((xf+1))
    account $id $rc
  done < $PLAN
elif [ -s $PLAN ]; then
  # parallel: tools/run_pool.py runs every job in a private sandbox tree (tools/job_sandbox.sh), weights from test/jobclass.txt, stamps written by the pool
  LOGS=${RUN_LOGS:-${TMPDIR:-/tmp}/run_changed_logs.$$}; SBX=${RUN_SANDBOX:-${TMPDIR:-/tmp}/run_changed_sb.$$}
  rm -rf $LOGS; mkdir -p $LOGS $SBX
  JF=$MMC/jobfile; : > $JF; EST=
  while IFS='|' read -r id s a g k; do
    cls=$(awk -v id="$id" '$1 == id { print; exit }' test/jobclass.txt 2>/dev/null)   # exact id (the '.' of an id is not a regex wildcard)
    w=$(echo "$cls" | awk '{print $2}'); cl=$(echo "$cls" | awk '{print $3}'); e=$(echo "$cls" | awk '{print $4}')
    [ -n "$w" ] || w=1; ex=0; [ "$cl" = excl ] && ex=1; [ -n "$STRICT" ] && [ "$cl" = par ] && ex=1
    seeds=; [ "$id" = cross/z01_hybrid ] && seeds="test/.build/vpcm/dp_vpcm_harness.o test/.build/vpcm/dp_vpcm_stub_envg.o"
    EST="$EST,$id=${e:-0}"
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$id" "$w" "$ex" "$k" "$s" "$a" "$g" "$NOPIN" "$seeds" >> $JF
  done < $PLAN
  touch $MMC/audit_marker; sleep 1
  POOL_EST=$EST python3 tools/run_pool.py -j $NJOBS --root "$PWD" --sandbox $SBX --logs $LOGS $JF
  # isolation audit: the pool and the jobs may touch nothing in the real tree except the stamps (every other write must have gone to a sandbox)
  stray=$(find . $BLOB_O \( -type f -o -type l \) -newer $MMC/audit_marker -not -path './test/.stamps/*' -not -path './.git/*' 2>/dev/null)
  if [ -n "$stray" ]; then echo "ISOLATION VIOLATION: files of the real tree written during the parallel run:"; echo "$stray" | head -20; failed=$((failed+1)); fi
  while IFS='|' read -r id s a g k; do   # deterministic order = plan order
    cat $LOGS/$(fid $id).log 2>/dev/null || echo "$id: FAIL (no log)"
    rc=$(cat $LOGS/$(fid $id).rc 2>/dev/null || echo 1)
    grep -q 'g64 XFAIL' $LOGS/$(fid $id).log 2>/dev/null && [ "$rc" -eq 0 ] && xf=$((xf+1))
    account $id $rc
  done < $PLAN
  [ -n "$RUN_KEEP_LOGS" ] || [ $failed -gt 0 ] || rm -rf $LOGS $SBX
  [ $failed -gt 0 ] && echo "(logs: $LOGS, sandboxes of passed jobs removed, of failed jobs kept: $SBX)"
  [ -n "$RUN_KEEP_LOGS" ] && echo "(logs: $LOGS, timing: $LOGS/timing.tsv)"
fi
# X1 --matrix: stage 2 of every job under the rest of the x86-64 flag set (SSE -O0, -O2 -mfpmath=387; and ONE -O3 job, p43); never run without --matrix (hours)
if [ -n "$MATRIX" ] && [ -z "$DRY" ]; then
  for j in $(jobs); do
    id=$(echo $j | cut -d'|' -f1)
    [ -n "$WANT" ] && case " $WANT " in *" $id "*) ;; *) continue;; esac
    g=$(g64_cmd $id); [ -z "$g" ] && continue
    for fl in "-O0" "-O2 -mfpmath=387" "-O3"; do
      [ "$fl" = "-O3" ] && [ "$id" != vpcm/v90_demodulator_progress ] && continue   # (was p43)
      sc=test/build_and_diff_$id.sh; [ $id = rd ] && sc=test/build_and_diff.sh; [ -n "$(sh tools/job_path.sh $id moved)" ] && sc=$(sh tools/job_path.sh $id script); k=$(SRC_KEYS=1 key $id $sc $(cd $KROOT && g64_files $id))+m"$fl"; st=test/.stamps/matrix_$(echo "$id$fl" | tr -c 'A-Za-z0-9\n' _)
      [ -z "$ALL" ] && [ "$(cat $st 2>/dev/null)" = "$k" ] && { echo "$id [x86-64 $fl]: unchanged (skipped)"; continue; }
      t0=$(date +%s); out2=$(FP64_FLAGS="$fl" $g 2>&1); rc2=$?
      if [ $rc2 -eq 0 ] && echo "$out2" | grep -q 'GOLDEN64' && ! echo "$out2" | grep -qE 'GOLDEN64:? FAIL'; then echo "$k" > $st; echo "$id [x86-64 $fl]: $(echo "$out2" | tail -1 | cut -c1-90) ($(( $(date +%s) - t0 ))s)"
      else rm -f $st; echo "$id [x86-64 $fl]: FAIL (rc=$rc2)"; echo "$out2" | tail -3; failed=$((failed+1)); fi
    done
  done
fi
echo "ran=$ran skipped=$skipped failed=$failed (g64 expected-failures among ran: $xf)"
[ $failed -eq 0 ]
