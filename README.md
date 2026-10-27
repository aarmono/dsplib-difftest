# dsplib-difftest

Differential tests of the portable DSP library `dsplib` against the closed-source blob it was decompiled from.  Each job builds the same harness
against the blob and against the port (32-bit, -O0 and -O2, plain and with the pinned flags) and compares what they do; the x86-64 build of the
port is compared with goldens recorded from the blob.

    git clone --recurse-submodules <this repository>
    export BLOB_O=/absolute/path/to/dsplibs.o
    sh tools/run_changed.sh -j 4 callprog/callprog         # one job; ids are <module>/<stem>, see test/jobs.tsv
    sh tools/run_changed.sh -j 4 --g64 ring/ring_detector   # + the x86-64 golden

* `ext/dsplib` - the library under test (submodule).  `src`, `layouts`, `sources.mk`, `dsplib.mk`, `flags.mk` and the library scripts in `tools/`
  are symlinks into it, so the scripts and harnesses keep their short relative paths.  To test a working checkout of the library, point the submodule
  at it (or replace `ext/dsplib` by a symlink to it).
* `test/<module>/<stem>.*` - the jobs (scripts, harness mains, layout mirrors); `test/golden/<module>/` - the recorded outputs;
  `test/jobs.tsv` - the job table.
* `tools/` - runner, parallel pool, stamps, coverage and mutation tools (`tools/README.md`).
* `docs/testing.md` - how the evidence is built; `docs/mutation-results/` - recorded mutation runs.

The blob is NOT in this repository and has no default location: `BLOB_O` must name it (absolute path).
