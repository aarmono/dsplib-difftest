#!/bin/sh
# Where the files of a job are (TESTREORG stage 2).  Usage: sh tools/job_path.sh ID WHAT      prints a path relative to dsplibs-port/ (or nothing)
#   WHAT = moved     prints "yes" when the job's files were moved into test/<module>/ (the first script of its row in test/jobs.tsv has a '/'), else nothing
#          script    the stage-1 script                          golden64  the job's own stage-2 script (<stem>.golden64.sh), if any
#          blob32    the blob golden test/golden/<module>/<stem>.blob32.out[.gz] (tier_golden64.sh), only that exact name: a facet golden (.nan.out, .integ.out, another job's) does not count
#          sizes     its per-scenario size file (blob32 with .out.gz -> .sizes)   main  the harness main (<stem>.main.c)
#          module    the module directory (ring)                 stem  the file stem (ring_detector)       build  the artifact prefix test/.build/<module>/<stem>
# Only jobs that were moved answer (every other job still uses the old test/<id>... names, which the callers keep spelling themselves); for those the output is empty
# and the exit status is 1.  The data is test/jobs.tsv (columns: id module new_name script harness golden stage2 notes; paths relative to test/ and test/golden/).
# Env: JOBS_TSV (default: test/jobs.tsv next to this script's tree), JOB_ALIASES (default: tools/job_aliases.tsv).
# TESTREORG stage 10: ID may be the active id (= new_name, e.g. ring/ring_detector), the old id (rd) or the planned name: it is resolved through tools/job_aliases.tsv first (same
# rule as tools/job_resolve.sh), so every tool that asks this script (run_job.sh, cover_chunk.sh, mutate.py --chunk, test/lib/tier_golden64.sh, stampkeys.py) accepts all spellings.
T=${JOBS_TSV:-$(dirname "$0")/../test/jobs.tsv}
A=${JOB_ALIASES:-$(dirname "$0")/job_aliases.tsv}
[ $# -eq 2 ] || { echo "usage: $0 ID moved|script|golden64|blob32|sizes|main|module|stem|build" >&2; exit 2; }
awk -F'\t' -v id="$1" -v what="$2" -v tab="$A" '
  BEGIN { while ((getline l < tab) > 0) { if (l ~ /^#/ || l ~ /^old_id\t/) continue
            n = split(l, f, "\t"); if (n < 3) continue
            m[f[1]] = f[2]; m[f[2]] = f[2]; m[f[3]] = f[2] }
          if (id in m) id = m[id] }
  function pick(list, pat,   n, a, i) { n = split(list, a, ","); for (i = 1; i <= n; i++) if (a[i] ~ pat) return a[i]; return "" }
  function ere(s) { gsub(/\./, "\\.", s); return s }
  /^#/ || $1 != id { next }
  { split($4, s, ","); if (s[1] !~ /\//) exit 1
    nm = $3; sub(/^\?/, "", nm); split(nm, p, "/"); mod = p[1]; stem = nm; sub(/^[^\/]*\//, "", stem)
    if (what == "moved") print "yes"
    else if (what == "script") print "test/" s[1]
    else if (what == "golden64") { g = pick($4, "golden64\\.sh$"); if (g != "" && g !~ /tier_golden64\.sh$/) print "test/" g }
    else if (what == "blob32") { g = pick($6, "(^|/)" ere(stem) "\\.blob32\\.out(\\.gz)?$"); if (g != "") print "test/golden/" g }
    else if (what == "sizes") { g = pick($6, "(^|/)" ere(stem) "\\.blob32\\.out(\\.gz)?$"); if (g != "") { sub(/\.out(\.gz)?$/, ".sizes", g); print "test/golden/" g } }
    else if (what == "main") { g = pick($5, "main\\.c(pp)?$"); if (g != "") print "test/" g }
    else if (what == "module") print mod
    else if (what == "stem") print stem
    else if (what == "build") print "test/.build/" mod "/" stem
    found = 1; exit 0 }
  END { exit found ? 0 : 1 }' "$T"
