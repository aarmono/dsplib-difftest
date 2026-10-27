#!/bin/sh
# Consistency check of the job manifest (TESTREORG): test/jobs.tsv against the tree, tools/run_changed.sh --list and tools/job_aliases.tsv.  Runs nothing, builds nothing.
# Usage: sh tools/jobs_check.sh      exit 0 = consistent; every problem is printed as "FAIL: ..." (all of them, not only the first).
# Checks: (1) 8 columns, no empty id / module / new_name / script; (2) ids and new_names unique; (3) the id set = the jobs of run_changed.sh --list (+ x1_flagdiff,
# which run_changed plans itself); (4) every file named in script / harness exists in test/, every golden in test/golden/; (5) every job script of test/
# (build_and_diff*.sh, *_golden64.sh, test/<module>/*.sh, test/<module>/<sub>/*.sh) is named by some row; (6) stage2 = yes exactly when the row has a golden64 script or its
# <id>_blob32.out(.gz) / <module>/<stem>.blob32.out(.gz) golden (the test/lib/tier_golden64.sh case); (7) job_aliases.tsv (TESTREORG stage 10): new_id column = the ids, planned = new_id = new_name without '?', old ids unique and none of them a job id, resolver maps old / new / planned to the new id;
# (10) stage 10: ids = new_name, characters [A-Za-z0-9_./], no '__' (the file-safe form fid = id with '/' -> '__' is injective), jobclass.txt = exactly the ids, the xfail list names ids only,
# h01_heap's MOVED list = ids with an existing script test/<id>.sh.
# (11) a golden of another job is declared in tools/golden_owners.tsv (TOOLFIX).
# (8) a MOVED job (first script has a '/') follows the file convention of docs/testing.md; (9) no script runs a moved job under its old build_and_diff_<id>.sh name.
cd "$(dirname "$0")/.." || exit 1
M=test/jobs.tsv; A=tools/job_aliases.tsv
TMP=${TMPDIR:-/tmp}/jobs_check.$$; mkdir -p $TMP; trap 'rm -rf $TMP' EXIT
rc=0; fail() { echo "FAIL: $*"; rc=1; }
[ -f $M ] || { echo "FAIL: $M missing"; exit 1; }
grep -v '^#' $M > $TMP/m.tsv
[ "$(head -1 $TMP/m.tsv)" = "$(printf 'id\tmodule\tnew_name\tscript\tharness\tgolden\tstage2\tnotes')" ] || fail "$M: header line"
tail -n +2 $TMP/m.tsv > $TMP/rows.tsv
awk -F'\t' 'NF != 8 { printf "FAIL: %s row %d (%s): %d columns, 8 expected\n", FILENAME, NR, $1, NF; bad=1 }
  $1 == "" || $2 == "" || $3 == "" || $4 == "" { printf "FAIL: row %d (%s): empty id / module / new_name / script\n", NR, $1; bad=1 }
  $7 != "yes" && $7 != "no" { printf "FAIL: %s: stage2 must be yes or no\n", $1; bad=1 }
  { if (seen[$1]++) { printf "FAIL: duplicate id %s\n", $1; bad=1 }
    n = $3; sub(/^\?/, "", n); if (nn[n]++) { printf "FAIL: duplicate new_name %s\n", n; bad=1 } }
  END { exit bad }' $TMP/rows.tsv > $TMP/out1 || rc=1; cat $TMP/out1
cut -f1 $TMP/rows.tsv | sort > $TMP/ids.manifest
{ sh tools/run_changed.sh --list | sed -n '/^---/q;s/^\([A-Za-z0-9_./]*\):.*/\1/p'; echo cross/x1_flagdiff; } | sort -u > $TMP/ids.list
diff $TMP/ids.list $TMP/ids.manifest > $TMP/d1 || { fail "job ids differ between run_changed --list (<) and $M (>):"; cat $TMP/d1; }
# files
awk -F'\t' '{ n = split($4, a, ","); for (i = 1; i <= n; i++) print $1 "\ttest/" a[i]
             n = split($5, a, ","); for (i = 1; i <= n; i++) if (a[i] != "") print $1 "\ttest/" a[i]
             n = split($6, a, ","); for (i = 1; i <= n; i++) if (a[i] != "") print $1 "\ttest/golden/" a[i] }' $TMP/rows.tsv > $TMP/files.tsv
while IFS='	' read -r id f; do [ -f "$f" ] || fail "$id: $f does not exist"; done < $TMP/files.tsv
# every job script is named by a row
for s in test/build_and_diff*.sh test/*_golden64.sh test/[a-z0-9_]*/*.sh test/[a-z0-9_]*/[a-z0-9_]*/*.sh; do
  [ -f "$s" ] || continue
  case "$s" in test/lib/*) continue;; esac       # shared helpers (TESTREORG stage 2: test/lib/tier_golden64.sh)
  cut -f2 $TMP/files.tsv | grep -qx "$s" || fail "$s is not named by any row of $M"
done
# stage2
awk -F'\t' '{ nm = $3; sub(/^\?/, "", nm); sub(/^[^\/]*\//, "", nm)
              g = ($4 ~ /(^|,)[A-Za-z0-9_.\/]*golden64\.sh(,|$)/) || ($6 ~ ("(^|,|\\/)" $1 "_blob32\\.out(\\.gz)?(,|$)")) || ($6 ~ ("(^|,)[a-z0-9_]+\\/" nm "\\.blob32\\.out(\\.gz)?(,|$)"))
              if (g && $7 != "yes") printf "FAIL: %s: has a golden64 script / blob32 golden but stage2=%s\n", $1, $7
              if (!g && $7 == "yes") printf "FAIL: %s: stage2=yes but no golden64 script / blob32 golden\n", $1 }' $TMP/rows.tsv > $TMP/out2
[ -s $TMP/out2 ] && { cat $TMP/out2; rc=1; }
# moved jobs (TESTREORG stage 2: the first script has a '/'): files follow the convention <module>/<stem>.sh, .golden64.sh, .main.c, goldens golden/<module>/<stem>.blob32.*
awk -F'\t' '{ split($4, s, ","); if (s[1] !~ /\//) next
              nm = $3; sub(/^\?/, "", nm); split(nm, p, "/"); mod = p[1]; stem = nm; sub(/^[^\/]*\//, "", stem)
              dir = nm; sub(/\/[^\/]*$/, "", dir)   # the module directory: v34/handshake for the nested module (its files are test/v34/handshake/<stem>.sh)
              if (dir != $2) printf "FAIL: %s: module %s <> directory of new_name %s\n", $1, $2, dir
              if (s[1] != mod "/" stem ".sh") printf "FAIL: %s: stage-1 script %s, convention %s/%s.sh\n", $1, s[1], mod, stem
              # (TESTREORG stage 9: the V.34 object support files shared by the jobs of v34, v34/handshake and vpcm live in test/v34/ and may be named by any of them)
              if (mod == "cross") { if (s[1] != "cross/" stem ".sh") printf "FAIL: %s: stage-1 script %s, convention cross/%s.sh\n", $1, s[1], stem; next }   # module cross (TESTREORG stage 9): the job reads files of every module
              n = split($5, a, ","); for (i = 1; i <= n; i++) if (a[i] ~ /^[a-z0-9_]+\// && a[i] !~ ("^" mod "/" stem "\\.") && a[i] !~ ("^" mod "/") && a[i] !~ /^v34\/(v34_img\.h|v34(obj|ctx|cfg|eng)map\.h|v01_common\.h|v01_layout(_probe)?\.(h|cpp))$/) printf "FAIL: %s: harness file %s is not under %s/\n", $1, a[i], mod
              n = split($6, a, ","); for (i = 1; i <= n; i++) if (a[i] != "" && a[i] !~ ("^" mod "/")) printf "FAIL: %s: golden %s is not under golden/%s/\n", $1, a[i], mod }' $TMP/rows.tsv > $TMP/out5
[ -s $TMP/out5 ] && { cat $TMP/out5; rc=1; }
# no script still runs a moved job under its old name (found by the fsk move: the SCRIPTS list of h01_heap named b103 / v23); comment lines are ignored
grep -v '^#' tools/job_aliases.tsv | tail -n +2 | cut -f1 > $TMP/oldids   # (stage 10: the scripts could only still name a job by its OLD id)
for id in $(cat $TMP/oldids); do
  grep -H -v '^ *#' test/*.sh test/lib/*.sh tools/*.sh 2>/dev/null | grep -E "build_and_diff_${id}\.sh" | sed "s/^/FAIL: moved job $id is still run under its old script name: /" > $TMP/out6
  grep -h '^SCRIPTS=' test/cross/h01_heap.sh | grep -qE "[\" ]${id}[\" ]" && echo "FAIL: moved job $id is still in the SCRIPTS list of test/cross/h01_heap.sh" >> $TMP/out6
  [ -s $TMP/out6 ] && { cut -c1-200 $TMP/out6; rc=1; }
done
# aliases
[ -f $A ] || { fail "$A missing"; exit 1; }
grep -v '^#' $A | tail -n +2 > $TMP/al.tsv
awk -F'\t' 'NF != 3 { printf "FAIL: %s row %d: %d columns, 3 expected\n", FILENAME, NR, NF }' $TMP/al.tsv > $TMP/out3
[ -s $TMP/out3 ] && { cat $TMP/out3; rc=1; }
cut -f2 $TMP/al.tsv | sort > $TMP/ids.alias
diff $TMP/ids.alias $TMP/ids.manifest > $TMP/d2 || { fail "new_id column of $A differs from the ids of $M:"; cat $TMP/d2; }
awk -F'\t' 'NR == FNR { n = $3; sub(/^\?/, "", n); plan[$1] = n; job[$1] = 1; next }
            { if (!($2 in job)) { printf "FAIL: %s: new_id %s is not a job id\n", $1, $2; next }
              if ($3 != $2) printf "FAIL: %s: planned %s <> new_id %s\n", $1, $3, $2
              if ($3 != plan[$2]) printf "FAIL: %s: planned %s <> new_name %s of %s\n", $1, $3, plan[$2], $2
              if (($1 in job)) printf "FAIL: old id %s is also a job id\n", $1
              if (seen[$2]++) printf "FAIL: new_id %s appears twice\n", $2 }' $TMP/rows.tsv $TMP/al.tsv > $TMP/out4
[ -s $TMP/out4 ] && { cat $TMP/out4; rc=1; }
# the resolver: old id, new id and planned name all give the new_id
while IFS='	' read -r o n p; do
  r=$(sh tools/job_resolve.sh "$o" "$n" "$p" | tr '\n' ' ')
  [ "$r" = "$n $n $n " ] || fail "resolver: $o $n $p -> $r (expected $n three times)"
done < $TMP/al.tsv
# stage 10: the ids are the new names
awk -F'\t' '{ n = $3; sub(/^\?/, "", n)
              if ($3 ~ /^\?/) printf "FAIL: %s: new_name %s still has the uncertainty marker\n", $1, $3
              if ($1 != n) printf "FAIL: id %s <> new_name %s (ids are the planned names since stage 10)\n", $1, n
              if ($1 !~ /^[A-Za-z0-9_.\/]+$/) printf "FAIL: id %s has a character outside [A-Za-z0-9_./]\n", $1
              if ($1 ~ /__/) printf "FAIL: id %s contains __ (the file-safe form, / -> __, would not be injective)\n", $1
              f = $1; gsub(/\//, "__", f); if (fid[f]++) printf "FAIL: two ids have the file-safe form %s\n", f }' $TMP/rows.tsv > $TMP/out7
[ -s $TMP/out7 ] && { cat $TMP/out7; rc=1; }
# (11) TOOLFIX: a golden that is ANOTHER job's own golden (<module>/<stem of a job>.blob32.out[.gz]; jobs p29a / p29b list p29's) must be declared in
# tools/golden_owners.tsv (job TAB owner job), and a row there must name a job that lists the golden of its owner.  The golden names of facets (`.x87`, `.nan`, `.a`, `.graph` ...)
# and of the `.integ` jobs (`<stem>.blob32.integ.out`) are the job's own and need no row.  (test/jobs.tsv is hashed into the keys of 156 jobs, so the marker is not kept there.)
O=tools/golden_owners.tsv
[ -f $O ] || fail "$O missing"
grep -v '^#' $O | awk 'NF' > $TMP/owners.tsv
awk -F'\t' 'FNR == 1 { f++ }
            f == 1 { n = $3; sub(/^\?/, "", n); own[n] = 1; next }
            f == 2 { if (NF != 2) printf "FAIL: %s: %d columns, 2 expected: %s\n", FILENAME, NF, $0; dec[$1 " " $2] = 1; used[$1 " " $2] = 0; next }
            { n = split($6, a, ","); for (i = 1; i <= n; i++) { g = a[i]; sub(/\.gz$/, "", g)
                if (g ~ /\.blob32\.out$/) { b = g; sub(/\.blob32\.out$/, "", b)
                  if ((b in own) && b != $1) { if (!(($1 " " b) in dec)) printf "FAIL: %s: golden %s is the own golden of job %s: add \"%s<TAB>%s\" to %s\n", $1, a[i], b, $1, b, "tools/golden_owners.tsv"; else used[$1 " " b] = 1 } } } }
            END { for (k in used) if (!used[k]) { split(k, x, " "); printf "FAIL: tools/golden_owners.tsv: %s does not list the golden of %s (or one of them is not a job id)\n", x[1], x[2] } }' $TMP/rows.tsv $TMP/owners.tsv $TMP/rows.tsv > $TMP/out11
[ -s $TMP/out11 ] && { cat $TMP/out11; rc=1; }
# test/jobclass.txt: exactly the ids, once each
grep -v '^#' test/jobclass.txt | awk 'NF { print $1 }' | sort > $TMP/ids.jobclass
diff $TMP/ids.jobclass $TMP/ids.manifest > $TMP/d3 || { fail "test/jobclass.txt ids (<) differ from the job ids (>):"; cat $TMP/d3; }
# the xfail list: every non-comment line starts with a job id
grep -v '^#' test/golden/tier_golden64_xfail.txt | awk 'NF { print $1 }' | sort -u | while read -r x; do grep -qx -- "$x" $TMP/ids.manifest || echo "FAIL: tier_golden64_xfail.txt names $x, which is not a job id"; done > $TMP/out8
[ -s $TMP/out8 ] && { cat $TMP/out8; rc=1; }
# h01_heap: its MOVED list (script stems = planned names = ids) names job ids with an existing script
sed -n 's/^MOVED=\${H01_MOVED:-"\(.*\)"}$/\1/p' test/cross/h01_heap.sh | tr ' ' '\n' | grep . | while read -r x; do
  grep -qx -- "$x" $TMP/ids.manifest || echo "FAIL: test/cross/h01_heap.sh MOVED names $x, which is not a job id"
  [ -f "test/$x.sh" ] || echo "FAIL: test/cross/h01_heap.sh MOVED: test/$x.sh does not exist"; done > $TMP/out9
[ -s $TMP/out9 ] && { cat $TMP/out9; rc=1; }
[ -n "$(sed -n 's/^MOVED=\${H01_MOVED:-"\(.*\)"}$/\1/p' test/cross/h01_heap.sh)" ] || fail "test/cross/h01_heap.sh: MOVED list not found"
if [ $rc -eq 0 ]; then echo "OK: $(wc -l < $TMP/rows.tsv) jobs, $(wc -l < $TMP/files.tsv) file references, $(wc -l < $TMP/al.tsv) aliases consistent"; fi
exit $rc
