#!/bin/sh
# Private working tree for one job of a parallel tools/run_changed.sh -j N run.
# Usage: sh tools/job_sandbox.sh <sandbox-root> <id> [seed file ...]   ->  prints the path of <root>/<fid>/tree  (fid = id with '/' replaced by '__')
# (A tree that is not a git checkout - an extracted `git archive`, a copy - is copied with find instead of git ls-files.)
# Layout: <root>/<id>/tree/ = a COPY of every tracked / untracked-not-ignored file of the real tree
# (so tracked inputs stay pristine and every generated file - test/*.out, objects, binaries - is private to the job) except test/golden and ext (the dsplib submodule: src, layouts and the library tools are symlinks into it), which are
# symlinks to the real directories (read-only inputs: only GOLDEN_UPDATE=1 writes there, never in run_changed).  [seed ...] = ignored artifacts of the real tree that
# the job needs from an earlier job's build (z01_hybrid: test/.build/vpcm/dp_vpcm_harness.o test/.build/vpcm/dp_vpcm_stub_envg.o).
root=$1; id=$(printf '%s' "$2" | sed 's|/|__|g'); shift 2   # a job id has '/' (TESTREORG stage 10): the sandbox directory is named by the file-safe form (fid, '/' -> '__'; idempotent)
cd "$(dirname "$0")/.." || exit 1
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
real=$PWD
sb=$root/$id; rm -rf "$sb"; mkdir -p "$sb/tree" "$sb/tmp" || exit 1
{ git ls-files -co --exclude-standard 2>/dev/null || find . -type f -not -path './test/.stamps/*' -not -path './obj*/*' | sed 's|^\./||'; for f in "$@"; do echo "$f"; done; } | grep -v '^test/golden/\|^ext/' | while read -r f; do [ -e "$f" ] && echo "$f"; done | tar -cf - -T - | tar -xf - -C "$sb/tree" || exit 1
mkdir -p "$sb/tree/test/.stamps"
ln -s "$real/test/golden" "$sb/tree/test/golden"
ln -s "$real/ext" "$sb/tree/ext"
echo "$sb/tree"
