#!/bin/sh
# Resolves job ids through tools/job_aliases.tsv (TESTREORG).  Usage: sh tools/job_resolve.sh TOKEN ...   prints one resolved token per line, in order.
# A token that is an old_id, a new_id or a planned name of the table prints the ACTIVE id (column new_id); any other token prints unchanged.
# Stage 10: new_id = planned name, so an old id (rd, v34_tier2 ...) prints the new id and a new id / planned name prints itself.
# Env: JOB_ALIASES = the table (default: tools/job_aliases.tsv next to this script).
T=${JOB_ALIASES:-$(dirname "$0")/job_aliases.tsv}
[ $# -gt 0 ] || exit 0
printf '%s\n' "$@" | awk -F'\t' -v tab="$T" '
  BEGIN { while ((getline l < tab) > 0) { if (l ~ /^#/ || l ~ /^old_id\t/) continue
            n = split(l, f, "\t"); if (n < 3) continue
            m[f[1]] = f[2]; m[f[2]] = f[2]; m[f[3]] = f[2] } }
  { print ($0 in m) ? m[$0] : $0 }'
