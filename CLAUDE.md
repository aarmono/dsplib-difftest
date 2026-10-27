# CLAUDE.md - dsplib-difftest

Differential tests of the library `dsplib` (submodule `ext/dsplib`) against the blob.  Read `docs/testing.md` (evidence), `tools/README.md` (tools) and
`docs/remaining-work.md` (open work).

## Setup

* `BLOB_O` - absolute path of the blob (an i386 relocatable object).  Required; there is no default and the blob is never committed.
* `ext/dsplib` is a submodule.  `src`, `layouts`, `sources.mk`, `dsplib.mk`, `flags.mk`, `tools/port_objs.py`, `tools/layout_gen.py`,
  `tools/tier_rewrite.py`, `tools/o0check.py` and `tools/o0check_*.tsv` are symlinks into it: never replace them by copies.
* The pinned compiler flags live in the library (`flags.mk`); `test/flags.sh` only reads them.

## Rules

* Run only the jobs your change affects: `tools/run_changed.sh -j N [--g64] <job ids>`.  Never `--all` for routine work; never run two test scripts at
  once in one tree.  A stamp (`test/.stamps/<id>`) means "the same COMPILED port code (plus the same script, harness text, goldens) as a run that
  passed" (`tools/stampkeys.py`): a comment edit keeps the stamp, any change of a compiled byte drops it.
* x86-64 behaviour needs a `--g64` job: -m32 identity cannot see pointer-width errors.
* A test change that is not a library change goes here; a library change goes to `dsplib` first and the submodule pointer is bumped here in its own
  commit, with the jobs that ran.
* Harness-side files generated from the library's layouts (`test/*_layout.h`, `*_layout_probe.cpp`, `test/v34/v34*map.h`) come from
  `python3 tools/gen_harness.py` (`--check` verifies them; the arguments are in `test/layouts.tsv`).  Never edit them by hand.
* Mutation runs (`tools/mutate.py`) and coverage runs (`tools/cover_chunk.sh`) only when asked.
* Scratch files: one directory under `/tmp/<branch-name>/`, deleted at the end.  Stop if `/` has under 5 GB free.
* Waiting on background processes: never use `pgrep -f <pattern>` as a completion check (it matches its own wrapper); record the PID and
  `while kill -0 $PID 2>/dev/null; do sleep 10; done`, or poll the job's own output file.
* Commit identity: use the configured user.name and user.email.  One commit per file or small group; the message says what
  changed and the evidence.  Commit messages and pull request descriptions contain no mention of Claude, of any AI model, or of tooling that
  generated the change: no `Co-Authored-By` trailer and no "generated with" line.
