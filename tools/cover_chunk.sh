#!/bin/sh
# cover_chunk.sh - run blobcov (machine-code coverage of the REAL blob function) under an existing chunk's
# harness WITHOUT editing test/build_and_diff_<chunk>.sh.
#
#   tools/cover_chunk.sh v34/v34_receiver --func receiver [--func X ...] [options]
#
# The chunk name is a job id of test/jobs.tsv (TESTREORG stage 10: the planned name, e.g. v34/v34_receiver; the old id v34_tier7a is resolved through tools/job_aliases.tsv; work files use the
# file-safe form of the id, '/' -> '__').  The blob test binary (${T}_test_blob) and the default
# scenario list are read from that script (its `T="${TPREFIX:-...}"` and `SCENARIOS="${SCENARIOS:-...}"` lines).
#
# options:
#   --func NAME        (repeatable, required) blob symbol to collect/report   (or --addr HEX --size N)
#   --scenarios "0 1"  override scenario ids (default: the script's list)
#   --rebuild          rebuild the blob binary first, into the scratch dir, by running the existing script with
#                      SCENARIOS=" " NOO2=1 TPREFIX=<scratch>/...  (builds only; runs no scenario; ~2 s)
#   --binary PATH      use this blob test binary instead
#   --out DIR          scratch dir for callgrind files (default: ${TMPDIR:-/tmp}/blobcov_<file-safe id>)
#   --jobs N           parallel scenarios (default 6)
#   --timeout S        per-scenario wall-clock cap; on expiry partial counts are dumped (coverage = lower bound)
#   --resume           reuse scenarios already collected in --out
#   --report-only      do not run valgrind; only (re)print the report from --out
#   --clean            delete the callgrind scratch dir after the report (default: keep, so --report-only/--resume work)
#   anything else      passed to blobcov.py (e.g. --full, --brief, --json FILE, --ctx N, --show-pad)
#
# Runtime: valgrind costs ~50x the native scenario time (native times are in the chunk's header comment / a plain run of
# ./<binary> <scenario>); run in the background.  No valgrind process is left behind: blobcov kills its process group
# on timeout and waits for every child.
set -e
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/.." && pwd)
cd "$root"
chunk="$1"; shift || { sed -n '2,30p' "$0"; exit 1; }
chunk=$(sh tools/job_resolve.sh "$chunk"); cfid=$(printf '%s' "$chunk" | sed 's|/|__|g')   # active id / file-safe form
script="test/build_and_diff_${chunk}.sh"
mvs=$(sh tools/job_path.sh "$chunk" script) && script=$mvs   # TESTREORG: a job whose files were moved into test/<module>/ (test/jobs.tsv)
[ -f "$script" ] || { echo "no $script" >&2; exit 1; }
funcs=""; scen=""; rebuild=0; binary=""; out=""; jobs=6; timeout=0; resume=""; reportonly=0; keep=0; extra=""
while [ $# -gt 0 ]; do
  case "$1" in
    --func) funcs="$funcs --func $2"; shift 2;;
    --addr) funcs="$funcs --addr $2"; shift 2;;
    --size) extra="$extra --size $2"; shift 2;;
    --scenarios) scen="$2"; shift 2;;
    --rebuild) rebuild=1; shift;;
    --binary) binary="$2"; shift 2;;
    --out) out="$2"; shift 2;;
    --jobs) jobs="$2"; shift 2;;
    --timeout) timeout="$2"; shift 2;;
    --resume) resume="--resume"; shift;;
    --report-only) reportonly=1; shift;;
    --clean) keep=1; shift;;
    *) extra="$extra $1"; shift;;
  esac
done
[ -n "$funcs" ] || { echo "need at least one --func" >&2; exit 1; }
out="${out:-${TMPDIR:-/tmp}/blobcov_${cfid}}"
mkdir -p "$out"
T=$(sed -n 's/^T="\${TPREFIX:-\(.*\)}"$/\1/p' "$script" | head -1)
[ -n "$scen" ] || scen=$(sed -n 's/^SCENARIOS="\${SCENARIOS:-\(.*\)}"$/\1/p' "$script" | head -1)
if [ -z "$binary" ]; then
  if [ "$rebuild" = 1 ]; then
    mkdir -p "$out/build"
    SCENARIOS=" " NOO2=1 TPREFIX="$out/build/${cfid}" sh "$script" > "$out/build/build.log" 2>&1 || { cat "$out/build/build.log"; exit 1; }
    binary="$out/build/${cfid}_test_blob"
  else
    binary="${T}_test_blob"
  fi
fi
[ -x "$binary" ] || { echo "blob test binary $binary not found (build it with the chunk script, or use --rebuild)" >&2; exit 1; }
echo "chunk=$chunk binary=$binary scenarios='$scen' out=$out" >&2
if [ "$reportonly" = 1 ]; then
  python3 tools/blobcov.py report --binary "$binary" --dir "$out" $funcs $extra
else
  python3 tools/blobcov.py run --binary "$binary" --scenarios "$scen" --out "$out" --jobs "$jobs" --timeout "$timeout" $resume $funcs $extra
fi
if [ "$keep" = 1 ]; then rm -rf "$out"; echo "(removed $out)" >&2
elif [ "$reportonly" = 0 ]; then echo "(callgrind files kept in $out; rm -rf it when done, or use --clean)" >&2; fi
