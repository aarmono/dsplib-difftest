# tools/ - runner, stamps, coverage and mutation tools of the differential tests

| tool | what it does |
|---|---|
| `run_changed.sh` | runs the jobs of `test/jobs.tsv` (all stages: blob vs port at -m32, pinned flags, -m32 -O2 golden, x86-64 golden); stamps skip jobs whose compiled code did not change |
| `run_job.sh`, `run_pool.py`, `job_sandbox.sh`, `job_path.sh`, `job_resolve.sh`, `jobs_check.sh` | one job, the parallel pool with a private tree per job, job table lookups and the consistency check of the table |
| `stampkeys.py`, `stampkeys_shim.sh`, `stampkeys_exit0.c` | the stamp keys: a job is "the same" when the COMPILED port code (plus script, harness text, goldens) is the same |
| `gen_harness.py` | regenerates / checks the harness-side files derived from the library's layout descriptions (`test/layouts.tsv`) |
| `blobcov.py`, `cover_chunk.sh` | which blob instructions / branch edges the harness never executed (callgrind) |
| `mutate.py` | directed mutation testing of a port file against a job |
| `blobpath.py` | `BLOB_O` for the python tools |

`port_objs.py`, `layout_gen.py`, `tier_rewrite.py`, `o0check.py` and the `o0check_*.tsv` tables are SYMLINKS into the library (`ext/dsplib/tools`): the stamp
keys import `o0check.py`, the jobs read the source list with `port_objs.py`, `gen_harness.py` calls the two generators.

Parts of this file were written when the tools lived in one tree with the library and the blob tools; the sections on `asmdiff.py`, `cmp342.py`, the
inventories and `params_gen.py` moved to the blob-tools repository.  Paths such as `src/...` and `layouts/...` are the symlinks into the library.

---------------------------------------------------------------------------------------------------------------

## blobcov.py - dynamic coverage of the blob function under an existing harness

### How it works
`valgrind --tool=callgrind --dump-instr=yes --collect-jumps=yes --compress-pos=no --collect-atstart=no
--toggle-collect=<symbol>` runs the chunk's **blob test binary** (`test/<chunk>_test_blob`, the 32-bit PIE that
links `dsplibs.o`) once per scenario (`argv[1]`, the convention of every harness).  Only the function under test
(and its callees) is collected.  Per-instruction execution counts and, for every conditional jump, *taken* and
*executed* counts (`jcnd=taken/executed`) are parsed, summed over scenarios and mapped onto
`objdump -d -M intel` of the linked binary.  callgrind reports file-relative addresses, so they equal `nm` addresses
of the PIE (no load-base guessing); offsets printed as `+0x..` are **function-relative**, i.e. identical to
`objdump -dr dsplibs.o` offsets minus the function's `.text` offset (the `.o` offset of `+0` is printed in the
header).  Recursion (e.g. `getbit`) is handled (verified: callgrind splits the record as `fn'2` and the parser sums
by address).  Scenarios that fork a probe child (`receiver` s8, `getbit` s6) are traced (`--trace-children=yes`, one
output file per pid).

`blobcov` is purely **observational**: it never changes what the test does.  Coverage is of *whatever the harness
does*, so it is a statement about the harness, not about the port.

### Usage
```
# easiest: through the chunk's existing build script (reads binary + scenario list from it)
tools/cover_chunk.sh v34_tier7a --func receiver                      # reuses test/v34_tier7a_test_blob
tools/cover_chunk.sh v34_tier5d --func ApplyBulkDelay --func getbit --func setfinalrate --func initdigital --rebuild
tools/cover_chunk.sh v34_tier6a --func modulatevector --scenarios "0 3 5" --timeout 900 --out /tmp/x
tools/cover_chunk.sh v34_tier7a --func receiver --out /tmp/x --report-only --full --json /tmp/x/r.json

# directly
tools/blobcov.py run    --binary test/v34_tier7a_test_blob --scenarios "0 1 2" --func receiver --out DIR [--jobs 6] [--timeout S]
tools/blobcov.py report --binary test/v34_tier7a_test_blob --dir DIR --func receiver --json out.json [--json-insns] [--full|--brief]
tools/blobcov.py run ... --addr 0x7cc00 --size 433          # LOCAL / renamed symbol by nm address of the linked binary
```
* `--rebuild` (cover_chunk.sh) runs the existing script as `SCENARIOS=" " NOO2=1 TPREFIX=<scratch>/chunk sh
  test/build_and_diff_<chunk>.sh`: it only **builds** (the empty scenario list makes the script run nothing; the
  resulting blob binary was verified byte-identical to the checked-in build).  Without `--rebuild` the existing
  `${T}_test_blob` is used - make sure it is current.
* Put `--out` under a scratch dir (callgrind files are small with `--toggle-collect`, ~0.1-3 MB, but there is one per
  scenario per pid).  `blobcov` kills its process group on timeout and waits for every child; it never leaves
  valgrind running.
* `--timeout S`: on expiry it asks callgrind to dump (`callgrind_control -d`), then kills.  The partial counts are a
  **lower bound** of coverage and the run is marked `partial` in `meta.json`.  (Verified working.)
* Functions are selected by symbol name (`--func`, repeatable; LOCAL symbols work if they are unique) or by
  `--addr/--size` (must still coincide with a symbol for `--toggle-collect`; `--collect-all` is the huge fallback).
* Some harness scenarios are *meant* to crash (`receiver` s8 traps with SIGFPE in the forked child; `getbit` s6 is an
  infinite-recursion probe): their exit status is recorded, their counts are still used.

### Runtime (measured on this 8-core box, load ~16 from other agents)
valgrind/callgrind is ~50x.  Native scenario times: tier7a s0/s5/s7 ~20 s each (~17 min under callgrind when the CPU is
not contended), tier6a s2 21 s, tier5d s4 26 s.  Scenarios are independent processes, so `--jobs 6` brings a whole
chunk to roughly `50 x (slowest scenario)`; a chunk is 20-40 minutes wall in the background.  There is no
call-count cap in the harnesses; use `--scenarios` to pick scenarios or `--timeout` for a lower-bound answer.

### Report
For every function: entries, instructions executed / total (and excluding never-executed alignment padding), bytes,
conditional branches, edges exercised, per-scenario contribution, then

* **A. never-exercised edges of reached branches**: `BLOCK` = the un-taken direction leads to a block that is
  completely unreached (needs a harness that reaches it or a dead-code argument); `EDGE` = the direction never
  happened but its destination block is reached from elsewhere (a branch-outcome gap only).
* **B. unreached code ranges**: contiguous zero-count runs with size, the first instructions of disassembly, calls made,
  notes (`idiv` = possible #DE, string ops), and the reached branch edge(s) that enter the range (`entered by`).  A
  range with no entering edge is reached only from other unreached code (or via a jump table / is dead).
* **C. switch tables** (indirect `jmp [reg*4+table]` found in the function).
* **D. hottest blocks**.

### Triage (every unreached edge gets exactly one of)
1. **Steer the harness** - a legal or at least reachable input reaches it; add a scenario/probe so the port is actually
   compared there (most `BLOCK` rows).
2. **Provably unreachable** - dead/guard code (e.g. a range check that an earlier check makes impossible, a defensive
   branch on an invariant the caller or `init*` establishes).  Document it in NOTES.md *with the argument* (and, if it is
   a wide-range invariant, the harness oracle that enforces it).
3. **Known crash/hang path** - `idiv` by zero, unbounded recursion/loop, wild pointer: document; make sure the
   port reproduces it the same way (fork probe) or is deliberately different and says so.

`EDGE` rows with a reached destination usually reduce to "branch outcome the random data rarely produces" (e.g. a
16-bit compare that is never true): steer with boundary values.

### Limits / what it cannot see
* Only executed machine code in the collected function is counted; callees are collected but only the named functions
  are reported.  Instruction coverage is necessary, not sufficient: it says nothing about *data* coverage (a path
  reached once with one operand value is reported as covered).
* Counts come from valgrind's translation; `rep`-prefixed string ops and `loop`/`jecxz` are instruction-level only (no
  edge data); indirect jumps are reported through the coverage of their targets and via table detection (the table
  length is taken from the preceding `cmp/ja` bound, else scanned).
* `--toggle-collect` needs the symbol to be in `nm` of the linked binary.

---------------------------------------------------------------------------------------------------------------

### Baseline measured 2026-09-30 (existing tests, blob machine code; callgrind 3.18.1, all scenarios of each chunk)
| function (blob) | entries | insns executed | edges exercised | unexercised |
|---|---|---|---|---|
| `receiver` (t7a, 9 scen.) | 320,002 | 1010/1010 = 100% | 136/136 | none |
| `modulatevector` (t6a, 8 scen.) | 199,537 | 816/820 = 99.5% raw, 100% excl. 4 alignment-pad insns | 78/80 | 2 edges, both provably dead (see below) |
| `ApplyBulkDelay` (t5d, 13 scen.) | 96,000 | 105/108, 100% excl. 3 pad | 24/24 | none |
| `getbit` | 1,306,419 | 127/128, 100% excl. 1 pad | 25/26 | 1 edge, provably dead |
| `setfinalrate` | 138,825 | 176/180, 100% excl. 4 pad | 38/38 | none |
| `initdigital` | 329,376 | 296/299, 100% excl. 3 pad | 60/60 | none |

The unexercised edges: `modulatevector +0xb57` and `+0xb75` are `cmp reg,3 ; jne` after `and reg,3 ; cmp reg,2 ; je ; jg`
(the quarter-turn switch compiled as a decision tree: once `and 3` and `>2` hold, the value is 3, `jne` cannot be taken);
`getbit +0x4c` is `test cx,cx ; jle` right after the identical test at `+0x36` with `cx` unchanged (`cx>0` already
established).  Note the receiver was 95.5% (7 debug-print blocks unreached) with the non-debug scenarios only; scenarios 1/6
(debug level 2) close it - the gap is always a `dsplibs_debug_printf` block behind `cmp dsplibs_debug_level,1`.

## mutate.py - shared, directed mutation-testing driver

Replaces the per-chunk throw-away drivers.  One command per chunk, resumable JSONL output, survivors grouped and
classified.  Python >= 3.8, standard library only (imports `asmdiff.py` for the blob facts).  Nothing in `src/` or
`test/` is touched; all scratch goes under `--out`.

```
# typical chunk (env var name = the PORT_xxx hook of the chunk's own build script)
tools/mutate.py run --chunk v34_tier5c --port src/v34/v34_detectors.c --port-env PORT_T5C \
      --func tone_detect --func dftupdate --func dftenergy --func fskdetect --func fskdemodulate \
      --out /tmp/m5c --jobs 6 --probes --gcov [--probe-check 20 --tce-check 10]
tools/mutate.py run ... --sample 150                       # quick iteration (stratified by class)
tools/mutate.py run ... --classes LIT,REL,-SIGN            # class / tag filter ('-' excludes)
tools/mutate.py run ... --all                              # the full undirected set (validation / retrospectives)
tools/mutate.py run ... --survivors-of /tmp/m5c/final.jsonl --out /tmp/m5c_ext   # after extending the harness
tools/mutate.py report  --port ... --func ... --out /tmp/m5c                     # regenerate report.txt (labels + probes refreshed)
tools/mutate.py compare --out /tmp/m5c_old --other /tmp/m5c_ext                  # survivors killed by the extended harness
tools/mutate.py list --port src/v34/v34_receiver.c --func receiver [--dump]           # generation counts only (no build)
```
`--func` restricts mutation to the named function bodies (include the port's static helpers; they are resolved through
their call sites).  `--script`/`--chunk` = the chunk's `test/build_and_diff_<chunk>.sh`, `--root` = the `dsplibs-port`
directory (default: this tree; a `git archive` of another commit works the same way), `--env VAR=VAL` passes extra
variables (e.g. `PORT_XFLAGS=...`), `--scenarios` overrides the list parsed from the script.
A function the port file does not define but a library header it includes does (the shared helper headers since RS2 / RS8: `--port src/vpcm/v92_cp.cpp --func crcRun` finds
`crcRun` in `pcm_cp_crc.h`) is found by expanding that `#include` line in place (`read_port`, RS8): `list` generates the header function's mutants; a `run` would mutate
the expanded copy of the port (not exercised in RS8: no mutation run).

### How it works
1. **Recorded replay, no per-chunk glue.**  The chunk's own script is run once (`SCENARIOS=" "`, `TPREFIX=<out>/work/base/chunk`,
   `PORT_xxx=<port>`) with a `gcc` wrapper that logs every compiler call.  From the log the tool takes the `-O2` compile of
   the port and the link of `*_test_port_o2`.  Every `.c` in that link (the harness main, the table files, the other ports) is
   **pre-compiled once**; a mutant then costs one `gcc -O2 -c` of the port file plus one link of ready objects
   (~0.3-0.4 s).  The unmutated port goes through exactly the same replay as a **control** and must match the blob.
2. **Blob outputs cached**: each scenario of the real blob is run once (stdout + time stored under `<out>/blob/`).
   *killed* = the mutant's stdout differs, or the scenario exits non-zero (assertion = exit 2/3, signal, timeout).
   Scenarios are tried in adaptive order (cost / observed kills) and stop at the first kill; timeout per scenario =
   max(20 s, 6 x blob time + 10 s); process groups are killed (fork probes leave no orphans).
3. **Invalid mutants** (do not compile) are detected by the `-O2 -S` compile of phase A and counted.
4. **Trivial compiler equivalence (TCE)**: phase A compiles original and every mutant to assembly (`-O2 -S`, the tests'
   flags, no debug info) and compares the whole translation unit after stripping comments and renaming `.L` labels by
   first appearance.  Identical => the mutant is the same program: counted, not run.  Mutants with identical code among
   themselves are run once (`dup_of`).  Test hooks compiled out (`COV(id)`, `#ifdef` blocks) fall out here too.
   Validation: `--tce-check N` actually runs N TCE-equivalent mutants (all must survive; 0 of the validation runs killed so far, see section (a)-(c) below).
5. **Results**: `mutants.jsonl` (generation + phase-A status), `results.jsonl` (one line per distinct object code),
   `final.jsonl` (one line per mutant: status, killer, tags, probe, exec), `survivors.txt` (sorted, diffable),
   `report.txt`.  Rerunning the same command resumes; `--rebuild` redoes the recorded build / blob outputs.

### Operator classes (tags) and what is directed
| class | what | default |
|---|---|---|
| `LIT` | integer literal +-1, +-2 (accessor offset literals are `OFF`; array sizes in declarations are `NOISY`) | run |
| `REL` | `< <= > >=` boundary flip + negation, `== !=` | run |
| `ARITH BIT SHIFT LOGIC` | `+ - * / %` (and `op=`), `& \| ^`, `<< >>`, `&& \|\|` swaps | run |
| `OFF` | accessor byte offset +-width (and +-2, +-4 for dwords) | run |
| `STMT` | delete an assignment / call statement | run |
| `NEG` | drop `!`, `if (c)` -> `if (!(c))` | run |
| `SIGNA` | `S16<->U16`, `S32<->U32`, `S8<->U8` on accessor LOADS | directed (below) |
| `CMPSIGN` | compare both operands of `< <= > >=` as `(int32_t)` / `(uint32_t)` | directed, pointer compares always |
| `CAST` | cast flips / drops | run only inside sign-consuming contexts (operand of a relational, left of `>>`, of `/ %`) or pointer-width casts; else `NOISY` |
| `SIGN` (decl), `WIDTH` | declaration sign flips, accessor / declaration width swaps | `NOISY`: opt-in (`--include-noisy` / `--all`) |

`direct` labels: `DIR` (blob evidence), `PTR` (pointer / address-width compare: always run), `UNDIRECTED` (no blob mapping:
**fallback, generated and run**, `--undirected skip` drops them), `WEAK` (only a movsx/movzx evidence), `SKIP` (blob is
sign-neutral there), `NOISY`.  The default policy runs everything except `SKIP`, `WEAK`, `NOISY`.

**Blob-directed SIGNA / CMPSIGN.**  `BlobEvidence` runs `asmdiff`'s symbolic executor on the blob function, then a
straight-line provenance pass: which loaded values reach a `cmp`/`test` followed by a signed (`jl..`) or unsigned (`jb..`)
jcc, a `sar`, `shr` or `div`, plus movsx/movzx extensions.  Port accessors are mapped to `obj+offset` through the
single-assignment base pointers of the source (`b = obj + 0x264`, `OBJ_BYTES(o, c)`; helpers through their call sites).  An
accessor sign swap is `DIR` if the blob value reaches a sign-consuming operation, `WEAK` if only an extension is seen,
`SKIP` if the blob never extends it, `UNDIRECTED` if the offset cannot be mapped.  `CMPSIGN` is directed by the compare
classes of the accessors in the operands (literals > 2 as weaker evidence).  **Limits (measured, see below):** straight-line
provenance only (no spill slots), `asmdiff`'s 64-byte window for indexed accesses, GCC 3.4 extends almost every 16-bit load
(so `SKIP`/`WEAK` cut only ~16% of the accessor swaps of `receiver`), and the evidence says where the blob cares about the
sign, **not** whether the port's value later masks it - that is what TCE and the probes are for.

### Value probes (`--probes`) - the most useful filter
A mutant that changes behaviour only when a specific VALUE occurs cannot be killed by a harness that never produces it.
`--probes` instruments the *original* port (inserted macros around relationals, 8/16-bit accessor loads, 8/16-bit casts
and simple assignments; set-once flags in a MAP_SHARED file, so crashes / forks lose nothing), runs every scenario once,
**requires the instrumented binary to match the blob**, and then labels each probe-able mutant:
* `NOVALUE`: the distinguishing value never occurred (`<`->`<=` with operands never equal; `S16->U16` with bit 15 never set;
  LIT +-d on a compared literal with `x - c` never in the affected window; signed-vs-unsigned compare with operand signs never
  differing; a deleted assignment that never changes the stored value; the site never executed).  The mutant is identical on
  this harness, so it is **not run** (`status novalue`) and listed as a **harness-gap candidate**: either steer the harness to
  the value or argue it cannot occur (then it is an equivalence / dead-code note).
* `SEEN`: the value occurred but the mutant survives => the difference is masked downstream (equivalence candidate).
Supported conditions are exactly the sound ones: relational boundary / negation, literal-operand of a relational (window
`x - c` in -2..2), 8/16-bit sign swaps and cast flips (32-bit swaps also retype the other operand, cast drops retype the
expression: no claim), `chg` for `lhs = pure-rhs;` with an initialised lhs, `sd` for CMPSIGN (skipped if a floating operand
occurs).  Measured soundness: over the ~2,400 mutants of `receiver` that were actually run, **0** killed mutants carry `NOVALUE`
(an early version had 27; they exposed a flag-slot bug and an uninitialised-local case, both fixed).  Not probe-able: ARITH, BIT,
SHIFT, LOGIC, OFF, NEG, most LIT, WIDTH, cast drops, 32-bit sign swaps (they get no label and are always run).

### gcov (`--gcov`)
`-O0 --coverage -m32` build of the port, all scenarios, `gcov` line counts: survivors on a line no scenario executed are
`UNREACHED` (harness gap), on executed lines `EXECUTED`, on lines without code `NOCODE`.  Works in this environment.
Line level only (an executed line with a never-taken branch is `EXECUTED`; the probes cover the relational cases).
Counters of a scenario that crashes before `exit` (the fork probes) are lost - use the probes for those.

### Retrospective measurements (2026-09-30, 8-core box shared with another agent, load 8-15)

**(a) `receiver` (tier 7a), harness of commit a17bf09 (before the 6 steering fixes), port code identical to 4198759**
(`git archive` of both commits; the port differs only in a header comment).  Earlier hand-rolled driver: 2,332 mutants, 526 survivors.

| | mutants | notes |
|---|---|---|
| generated (all classes, current generator) | 2,846 | LIT 960, OFF 460, WIDTH 280, CAST 344 (30 directed + 314 noisy), STMT 171, ARITH 151, SIGNA 118, REL 95, SIGN-decl 90, CMPSIGN 72, NEG 50, BIT 31, SHIFT 19, LOGIC 5 |
| invalid (do not compile) | 4 | lexer never touches digits inside identifiers / pointer stars (the old driver: 402 invalid in 5c) |
| TCE-equivalent (not run) | 456 (16%) | CAST 244, WIDTH 64, SIGN-decl 59, SIGNA 39, CMPSIGN 37, LIT 8, OFF 3, STMT 2, REL 1, NEG 1; 66 validation runs (TCE-equivalent or NOVALUE mutants) over the three chunks: 0 killed |
| excluded by the directed policy | 703 (25%) | WIDTH 280, CAST-noisy 314, SIGN-decl 90, SIGNA skip/weak 19; of these 369 are TCE-equivalent anyway |
| **default policy set** | 2,143 | TCE 87, run 1,874 distinct object codes: **1,952 killed, 100 survivors** (97 run + 3 NOVALUE not run) |
| survivors by class | | SIGNA 33, LIT 31, STMT 12, CMPSIGN 8, CAST 7, REL 6, OFF 2, LOGIC 1 |
| survivors by probe | | NOVALUE 41, SEEN 28, no probe 31 |
| noisy set (703), run against the extended harness' scenarios 0/1 (the only scenarios the extension changed) | 703 | 369 TCE, 211 killed, 123 survive s0/s1 (69 of them known to survive all old scenarios) |

**Key number - real findings preserved.**  Survivors of the a17bf09 harness (all 100) re-run against the 4198759 harness
(`--survivors-of`, all 9 scenarios): **10 killed = the real weaknesses**, all in the default set: `rx_tail:247`
`(int32_t)ecx < 0` (`<`->`<=`, literal 0->1, 0->2; published error sum exactly 0), `rx_tail:259` `S16(b,0x124) <= 0x7530`
(`<=`->`<`, literal 0x7530 -> 0x752e/0x752f/0x7531/0x7532), and `receiver:449` `s1 = 0, s2 = 0` (0->1, 0->-1: shifted-TRN2
sums differing by one).  (The older count "6" merges the literal variants.)  **No real survivor was lost by the filters:** the
10 are REL / LIT classes that the policy never filters; of the 703 policy-excluded noisy mutants the 211 killed by the extended
scenarios 0/1 were *all* already killed by the old harness, i.e. none of the noisy classes contained a real finding.  The value
probes (recomputed on the a17bf09 harness) label **8 of the 10 `NOVALUE`** - they point straight at "operands never equal at
this compare" - and the remaining 2 (the `s1 = 0` initialisers) carry no probe.  No killed mutant of the whole run was labelled
`NOVALUE` (soundness check over ~2,400 run mutants).

| cost (CPU seconds, 6 workers on a box at load 8-15) | |
|---|---|
| phase A (compile 2,824 mutants `-O2 -S`, TCE, dedupe) | 160 s wall |
| blob outputs (9 scenarios) / control (9 scenarios, port -O2) | 30 s wall / 93 s |
| phase B, default set: build 917 s + scenario runs 27,753 s (incl. noisy ones run early) | kills 1,994 x 2.9 s, survivors 148 x 148 s (survivors = 79% of the time) |
| default-policy subset only | 19,124 CPU-s (= ~53 min at 6 jobs, ~1.3 h observed under load) |
| unfiltered estimate (no TCE, no policy: every TCE-equivalent / noisy survivor also pays a full 93-148 s pass) | >= 70,000 CPU-s (about 3-4x) |

**(b) tier-5c `detectors`, current tree** (earlier: 843 valid mutants, 402 invalid, 30 survivors, all equivalent):
1,414 generated (invalid 6), 1,121 default set, TCE 52, 965 distinct object codes run in **404 s wall** (CPU 2,088 s; whole command
incl. blob, control, gcov, probes ~9 min), 1,013 killed, **50 survivors** (45 + 5 NOVALUE): probes NOVALUE 5 / SEEN 11 / none 34.  Survivor
count is not smaller than the old 30 because the generated set is broader (every CAST in a sign context, LIT on every loop
init / bound, CMPSIGN); the survivors are the same equivalence families the NOTES list (logical vs arithmetic `>>14` before
a `(short)` truncation, `k < 12` -> `13` = compiler-UB drop, `m = 0 -> 1`, abs shift 31 -> 29/30 ...) plus 6 array-size literals (now `NOISY`).
**The two real pointer-compare mutants are generated and killed under the current harness**: `CMPSIGN/PTR` at `tone_detect:123` and
`:157` (both operands compared as `(int32_t)` instead of unsigned) are killed by scenario 0.  Note the single-cast flips
`(uintptr_t)p < (uintptr_t)end` -> `(intptr_t)p < (uintptr_t)end` are **TCE-equivalent** (mixed operands convert back to unsigned);
only flipping both casts changes the program, which is what `CMPSIGN` does.

**(c) tier-6a `modulatevector`, current tree** (earlier: 1,316 mutants, 416 survivors): 1,955 generated, 1,303 default set (invalid 14,
TCE 182), 993 distinct object codes run in **649 s wall** (CPU 3,210 s), 1,007 killed, **100 survivors** (84 + 16 NOVALUE): probes NOVALUE 16 / SEEN 21 /
none 63, gcov `UNREACHED` 8 (the documented dead `quarter_at` / `ginv_at` range-guard returns).  The two real weaknesses of
that chunk are generated and killed: `SIGNA U16->S16` on the limit word `0x25e2` (`tier6a:468`) and `S16->U16` on the short limit
`0x2620` (`:475`), both labelled `DIR` by the blob evidence (`cmp:s,ext:u` / `cmp:s`).  (The pre-extension 6a harness is not in
history, so no before/after was possible there.)

### What the filters cost / risk
* **TCE**: sound (same machine code).  Only compares at `-O2`; the tests also diff `-O0`, which mutation never did.
* **Probes**: sound for the conditions listed above *given the harness is deterministic and the instrumented build matches the blob*
  (checked on every run).  They do not cover ARITH/BIT/SHIFT/OFF/NEG/most LIT/WIDTH/cast drops (no label => always run).
  `NOVALUE` is **not** "equivalent": it is exactly the list of harness-gap candidates to read first (all 8 probe-able real
  findings of (a) are in it, 8 of 41).
* **Directed SIGNA (blob evidence)**: weak.  GCC 3.4 emits movsx/movzx on almost every 16-bit load that feeds 32-bit code, so
  only 19 of 118 `receiver` accessor swaps were `SKIP`/`WEAK` and 33 of the 99 `DIR` ones survive (all equivalent: flags words,
  values that fit, `& mask` afterwards).  The real 6a sign findings were `DIR`; no real finding in 7a was SIGNA.  The heuristic
  cannot see downstream masking - keep SIGNA but expect it to be ~1/3 of the survivors; drop `WEAK`/`SKIP` only if time matters.
* **Noisy classes (WIDTH, decl SIGN, non-sign-context CAST)**: 25% of the mutants, >50% TCE-equivalent, survivors all equivalent,
  zero real findings in (a) (measured with the extended harness' scenarios 0/1, see above).  Risk: a real finding hidden in these
  classes in another chunk is not generated by default; `--all` runs them.
* **UNDIRECTED fallback**: 50 of 72 `CMPSIGN` and 0 accessor swaps in `receiver` (the base-pointer mapping resolved every accessor
  there); `--undirected skip` would drop them, but most are `for (i < N)` loop compares that probes label `NOVALUE` anyway.

### Where the time goes, and how to cut it further
Build is 3-5% of the CPU (incremental: one object + link, ~0.3-0.4 s); scenario runs are 95%, and **survivors dominate**
(a survivor runs every scenario: 93-148 s for `receiver`, 20 s for 5c).  Already cut: TCE, dedupe, `NOVALUE`, policy.
Ideas, not implemented: (1) per-scenario probes (one counter file per scenario) would restrict a `SEEN` mutant to the scenarios
that produce its distinguishing value (sound); (2) per-scenario gcov line sets would skip scenarios that never execute the mutated
line (sound except for fork-probe scenarios); (3) `--survivors-of` with `--scenarios` limited to the scenarios the harness
extension touched (what (a) did for the noisy set); (4) an "all mutants in one binary with a runtime selector" build is **not**
worth it - compile is 3-5% of the cost and a selector changes the `-O2` code generation (TCE and the optimisation-sensitive
equivalences would no longer describe the mutant).

### Recommended workflow (per chunk)
1. `tools/mutate.py run ... --probes --gcov --probe-check 20` (default policy).  Read `report.txt` top-down:
   (a) **`NOVALUE` survivors first** - each is either a harness gap (steer the harness so the distinguishing value occurs, e.g.
   operands equal, bit 15 set, `x == c+-1`) or a documented unreachable / dead-code equivalence; (b) `UNREACHED`; (c) `SEEN` and
   un-probed survivors are almost always equivalence families (write them up by family, not by mutant).
2. Extend the harness for (a)/(b); rerun **only the survivors**: `--survivors-of <old>/final.jsonl --out <new>` (NOVALUE ones are included);
   `tools/mutate.py compare --out <old> --other <new>` lists what the extension killed.  Restrict `--scenarios` to the changed ones
   when the rest of the harness is untouched.
3. Always run: LIT, REL, STMT, NEG, ARITH/BIT/SHIFT/LOGIC, OFF, CMPSIGN (pointer compares: `PTR`), SIGNA (`DIR`), CAST in sign contexts.
   Skip unless you have spare time: WIDTH, decl SIGN, CAST outside sign contexts (`--all` once per tool change / new kind of code).
4. Record in NOTES: generated / invalid / TCE / run / killed / survivors by probe, the NOVALUE list with its disposition, and
   `--tce-check`/`--probe-check` results.
5. Clean up `--out` (it holds compiled objects); the tool leaves no background processes (scenario process groups are killed).

### Known limits
* Mutation is at `-O2` only; the lexer is not a full C parser (spans of operands are found by precedence scanning; a construct it
  mis-spans yields an invalid mutant, never a wrong result); accessor macros are recognised by name (`S8 U8 S16 U16 S32 U32 PTR`);
  one port file per run; chunks whose script does not follow the `PORT_xxx` / `TPREFIX` / `NOO2` / `SCENARIOS` convention (the
  older tiers) are not supported.
* Probes instrument with GNU statement-expressions and `__typeof__`; a site kind that does not compile is dropped automatically and
  an instrumented binary that does not match the blob disables the probes (both reported on stderr).
* C++ (measured at P42 / P44): `--func Class::method/N` (N = argument count) finds only NON-const member functions; a `const` member function (P44: getBitRate, getRbsPattern, indicateRemoteRateReneg, getAT_UD)
  and a constructor with a member-initialiser list are not listed ("function ... not found (have: ...)"): put the body into a `static inline` helper (`getAT_UDBody`, `ctorBody`) that the method calls and name the
  helper in `--func`; the initialiser list itself (constants handed to the sub-object constructors) is then not mutated: check those by hand (the harness' constructor scenario compares every argument).

---------------------------------------------------------------------------------------------------------------

## C++ port chunks (first used for P01, 2026-09-30): what changed in the tools, and the per-chunk command set

The remaining V.90 / V.92 / K56Flex classes are ported as REAL C++ (`src/*.cpp`, `-fno-exceptions -fno-rtti`, no libstdc++).  Every tool
keeps working on the plain-C ports exactly as before (verified: `cmp342.py --all` gives the identical verdict for all 356 functions
with a blob namesake, before and after the change); what was added:

* **asmdiff.py**: no change.  Functions are named by their MANGLED symbol (`--func _ZN5QueueIfE4readEPfj`); the blob's
  `.gnu.linkonce.t.<mangled>` template instances are found by symbol, a `.cpp` port is compiled by the `gcc` driver (by extension).
  Look at section (5) for a float class: `fld DWORD` / `fstp DWORD` only in the port means the port copies a float through the x87 where
  the blob uses integer moves (signalling-NaN payloads are quieted on i386).
* **cmp342.py**: reads EVERY executable section (`.text`, `.gnu.linkonce.t.*`, COMDAT `.text.*`) of both objects into one virtual text
  (16-aligned, symbols and relocation offsets shifted by the section base; cache key `v4`); `*.cpp` / `*.cc` / `*.cxx` ports are compiled
  with `g++` 3.4.2 in the container with `-std=gnu99` removed and `-fno-exceptions -fno-rtti` added; `--all` includes `src/*.cpp`.
  `porting/tools/cmp342.py src/dsp/dsp_int_templates.cpp --summary` (P01: 12 IDENTICAL, 1 IDENT-REGS, 12 SAME-SHAPE, 1 CLOSE, 6 NEAR, 19 FAR).
  The port sources must stay parseable by g++ 3.4.2 (C++98; use `DSP_STATIC_ASSERT` from `src/core/dsp_cxx.h`, no `nullptr` / `override` / `auto`).
* **mutate.py**: `tools/mutate.py run --chunk p01 --port src/dsp/dsp_int_templates.cpp --port-env PORT_P01 --func 'Queue::write/2' ...`.
  C++ function keys are `Class::method/nparams` (template arguments dropped, destructors `Class::~Class/0`, overloads told apart by the
  parameter count; the list is `python3 -c "import sys; sys.path.insert(0,'tools'); import mutate; print(*mutate.Model(open('src/dsp/dsp_int_templates.cpp').read()).funcs, sep='\n')"`);
  the lexer knows `::`; template parameter names (`template <class H, class O>`) are type names; the recorded replay wraps `g++` as well as `gcc`
  (`cc` is stored in `plan.json`) and prefers the compile whose output is `*_port_o2.o` (a script may build the port several times at -O2, e.g.
  an extra `-m64` object); the probe prelude is `extern "C"` and uses `a - b` for the common type (pointer operands compile).  Only
  bodies in the `.cpp` are mutated: keep the definitions out of the headers (declarations in the header, definitions + explicit member
  instantiations in the `.cpp`), otherwise `--func` cannot reach them.  Measured on P01: 693 mutants, 1.7 min wall with 6 jobs (the
  harness scenarios take 0.02-1 s).  The `ARITH` / `SIGN` classes produce ~7% invalid mutants on template-heavy code (lexer view of `<`, `*`).
* **blobcov.py / cover_chunk.sh**: valgrind is started with `--demangle=no`, so `--func` takes the mangled symbol
  (`tools/cover_chunk.sh p01 --rebuild --func _ZN5QueueIfE4readEPfj ...`).  Caveat seen on `jmp`-into-the-condition loops (`reset`,
  `copyHistoryTail`): the tool counts one of the two loop edges as unexercised although `never-exercised edges of REACHED branches` is 0
  and both directions ran thousands of times; instruction coverage is the reliable number.  A callee that is itself a `--func` is
  attributed to the scenarios in which it is called directly, not through its caller (per-scenario columns, not the totals).
* **Harness convention for C++ chunks** (`test/p01_diff_main.c`, `test/build_and_diff_p01.sh`): ONE C source linked against `dsplibs.o` or the
  port object, functions reached by `__asm__("_Z...")` labels, object layouts mirrored as C structs with real pointer members,
  `sysdep_malloc/free` defined by the harness (deterministic bump allocator mapped around 2^31, red zones, allocation table, log), every
  canonical line hashed, sampled lines printed; the 32-bit blob run is the golden (`test/golden/`), `test/p01_golden64.sh` checks an
  x86-64 build of the port against it without any blob.  The build script is also what `mutate.py` replays (`PORT_P01`, `TPREFIX`, `SCENARIOS`,
  `NOO2`, `P01_SEED`).

## Shared C++-chunk harness library and the layout-table generator (P02 onwards, 2026-09-30)

**`test/harness/harness.[ch]`** (plain C99, links with `-m32 -no-pie` against `dsplibs.o` / the port and with `-m64`): the ~300 lines of
machinery that `test/p01_diff_main.c` carried, extracted without changing one output byte (`sh test/build_and_diff_p01.sh` prints MATCH,
the 32-bit blob run is still byte-identical to `test/golden/p01_blob32.out`, `sh test/p01_golden64.sh` still reproduces it).  Build it once
per architecture (`gcc <harness flags> -c test/harness/harness.c`) and link it into the blob, port and 64-bit harness binaries; it DEFINES
`sysdep_malloc` / `sysdep_free`, so `test/stub_env.c` must be compiled with `-Dsysdep_malloc=stub_env_unused_malloc -Dsysdep_free=stub_env_unused_free`
(as in the P01 script).  API (all `h_*`, header comment has the details):
* PRNG `h_rnd/h_rmod/h_rrange/h_rpct` (+ `h_seed_xor` for the `<CHUNK>_SEED` hook); the object fill of `h_obj_new` has its OWN generator (the
  random stream must not depend on `sizeof`, which differs on x86-64);
* canonical output `h_emit(show, fmt, ...)` (every line hashed with FNV-1a 64, printed when `show`), `h_cov_finish()` prints `COV ...` lines + `HASH <h> lines <n>`
  and exits 2 if a REQUIRED counter (`h_cov_define`, cleared by `h_covreq[id] = 0`) is zero; `h_die3` = model / harness violation (exit 3);
  `h_sample`, `h_garbage_hi`, float helpers (`h_bits_of`, `h_float_from_bits[_quiet]`: print floats as bit patterns, never `%f`);
* allocator: deterministic bump arena mmap-ed so that 0x80000000 lies 4 MB inside it (`h_arena_init`; every trial's first block starts just below
  the boundary: signed pointer compares), 48-byte red zones, freed blocks poisoned, allocation table (`h_al`, `h_last_alloc`, `h_nmalloc/h_nfree`) and the
  hashed `M id size` / `F id` log; `h_scenario_begin(elem4)` (1: blocks 4-byte aligned, 0: any byte offset, 2: 2-byte aligned, for 16-bit tables), `h_trial_end(what)` (red zones + leak report + next trial), `h_check_not_wild`,
  `h_maybe_null_first_ptr` (dtor-with-NULL-block probe), `h_fmt_ptr` (`A<id>+<off>`, never a raw address);
* field-wise dump `h_dump_obj(show, tag, obj, HFld table, n)` and the malloc-time store-order probe `H_CTOR_PROBE(obj, table, n, sizeof, call)`.  Field kinds:
  `b h u i` integers, `f` float32 and `d` double (bit patterns), `p` pointer (normalised), `o` OWNING heap pointer (the block it points to is appended:
  hex up to 192 B, else a hash; `heap.<name>` when an object has several owners), `e` pointer into the object (`E+<off>`), `v` vptr (`V:<name>+<off>`
  via `h_register_vtable`), `x` unknown region (hex / hash); `count` > 1 = array;
* `h_layout_compare(cls, mirror, probe, n)`;
* `H_COV_MAX` is 128 counters (ids 60 / 61 stay reserved); a `v` (vptr) field prints `V:<name>+<off>` of the registered table with the LARGEST base <= the vptr (vtables of different classes lie close together),
* `h_log_next_size` (one-shot): the next `sysdep_malloc` writes this size into its `M id size` log line instead of the real one, for an allocation whose `sizeof` differs between i386 and x86-64 (an object that holds pointers); default 0 = off.
* `h_log_map_from` / `h_log_map_to` (persistent, P16): every `sysdep_malloc` of exactly `h_log_map_from` bytes is logged as `h_log_map_to` bytes (several heap objects of one pointer-holding class, e.g. the two heap FloatFIR of P16 / the reference stages of its harness: `sizeof` 0x14 on i386, 0x28 on x86-64); default 0 = off (keep every other block's size distinct from `h_log_map_from`).
* `h_log_map_add(from, to)` / `h_log_map_clear()` (P29): up to 8 further persistent log-size pairs (the P16 `h_log_map_from` / `h_log_map_to` is ONE pair): P29 allocates four classes that hold pointers (`sizeof` differs on x86-64: V90Mapper, V92Transmitter, V92Precoder, V92PreFilter) in one process; default none (every earlier chunk unchanged: P01 .. P28 re-verified MATCH with unchanged goldens).
Not implemented (not needed so far): a recorder for class-callee interposition (when a chunk needs one, weaken the callee in a copy of `dsplibs.o`
as in tiers 9a / 10a and define the recorder with the callee's mangled `__asm__` label, taking `this` first).

**`tools/layout_gen.py`**: one text description per chunk (`test/<chunk>.layout`: `class <Name> size <n> cxx <typedef> mirror <C struct>`, then
`<offset> <kind> <member> [<count>]` lines; kinds `u8 u16 u32 i32 float32 double ptr-to-heap ptr-into-heap ptr-to-embedded vptr unknown-region`) ->
`test/<chunk>_layout.h` (C mirror structs with REAL pointer members, `HFld` tables, i386 `_Static_assert`s, `<chunk>_layout_check()` comparing every mirror
against the real class), `src/<module>/<name>_layout.inc` (the `DSP_ASSERT_LAYOUT_I386` text for the end of the port header, guarded by `__i386__`) and
`test/<chunk>_layout_probe.cpp` (sizeof / offsetof of the real classes).  A generated `--out-test` header outside `test/` (a moved job: `test/<module>/`) needs `--harness-include ../harness/harness.h` (default `harness/harness.h`, relative to the header; TESTREORG stage 4).  The description is self-checking: the generator lays the members out with the
i386 rules and refuses a wrong offset or size (a gap must be an `unknown-region`).  A member written `name=a.b` (P21, embedded sub-objects) or `name=b` (P22, a member inherited from a base class) keeps the mirror / dump name `name` and uses the designator `a.b` / `b` of the real class in the generated `offsetof` asserts and the probe (the mirror itself stays flat: no embedded struct kind is needed).  The kind `u32-ptrslot` (added with P27) is a 32-bit value that is the LAST member of an embedded sub-object that also holds pointers (`Scrambler<uchar,int>`: 7 pointers + `tail_`): on x86-64 the sub-object is padded to pointer alignment and the owner's next member starts after the padding, so the mirror member is a pointer-sized integer (`uintptr_t`, printed as a `u32` = the low word); `member_map.py` / `member_map_source.py` know it (width 4 on i386, 8 / 8-aligned on x86-64). The kind `align8` (added with P28; kind letter `a`) is a ZERO-SIZE marker that exists only in the C mirror: `char name[0] __attribute__((aligned(sizeof(void *))))`, i.e. the next member starts pointer-aligned (nothing changes on i386 where every offset is 4-aligned already, 8 on x86-64).  Put one right before the first flattened member of an embedded sub-object that holds a pointer (such a sub-object is pointer-aligned on x86-64 even when its own first member is a 32-bit word) and one right after its last member (its size is padded to pointer alignment, the owner's next member starts after the padding): the flat mirror then has the real class's x86-64 offsets.  It is not a member of the real class: no HFld entry, no `offsetof` assert, no probe entry, no mirror offset; `member_map.py` ignores it (zero width, not reported as untouched), `member_map_source.py` aligns the x86-64 layout at it.  Used by `test/vpcm/v90_mapper.layout` (V90SpectralShaper inside V90Mapper, V90SignBitsExtractor inside V90Demapper).  The kind `vptr` is the IMPLICIT vptr of a polymorphic class (not a declared member): the mirror has a real `void *` member, the generated port asserts skip it and the probe reports offset 0 (the C++ classes need `#pragma GCC diagnostic ignored "-Winvalid-offsetof"` before the `.inc`).  Regenerate with the command in the script's docstring; the outputs are committed.  TESTREORG6: two options reproduce committed wording: `--inc-banner chunk` (first line of the `--out-port` file `i386 layout asserts of chunk <chunk>;`, the banner of 28 src/vpcm `.inc` files; default: the generic text) and `--src-label NAME` (the description name printed in the banners instead of its path: test/vpcm/pcm_bits_to_symbol.layout was committed as `p29.layout`).  The committed generated files are the truth: a member renamed by hand in a generated file is carried into the `.layout` (`old=new`, the mirror / dump name stays `old`).  **`tools/layout_check.py`** (called by `sh tools/gen_views.sh --check`, ~2 s) runs the `# regenerate:` line (ONE physical line) of every `test/**/*.layout` into a temporary directory and compares each output byte for byte with the committed file; its SKIP set names test/v01.layout (gen_views.sh checks it with `--paths`) and test/v34obj.layout (not an input), its ALLOW map (empty) the tolerated drift, one comment each.

Float support in the harness library: `h_x87_check()` (asserts the x87 control word is the process default), `H_SHOWALL=1` in the environment prints EVERY canonical line of a scenario (debugging a blob / port difference; the hash line then differs from the golden), the field kinds `f` / `d` print floats and doubles as bit patterns.  `mutate.py` note: its `-O2` 32-bit replay cannot see mutants that only change SSE semantics (dropping a `(long double)` cast on a float product is equivalent on x87 at -O2); the x86-64 golden run is the check for those.

## jobs.tsv, job_aliases.tsv, job_resolve.sh, jobs_check.sh - the job manifest (TESTREORG stage 1)

`test/jobs.tsv` (tab separated, `#` header explains the columns): one row per job id that `run_changed.sh` runs (185: `--list` plus `x1_flagdiff`).  Columns: `id`, `module`
(src/ module of the code under test; `cross` = no single one), `new_name` (planned id `<module>/<stem>[.<facet>]`), `script` (stage-1 script, wrapper / shared scripts, own `<id>_golden64.sh`),
`harness` (mains, quote-included headers, layout files, probes, python checkers; the files every job shares are not listed), `golden` (files of `test/golden/` the job owns), `stage2` (yes/no),
`notes` (`args=...`, `g64=own|tier`, reasons for a `?`).  A `?` in front of module / new_name marks a judgement.  Paths are relative to `test/` (goldens to `test/golden/`).  The file was derived from the
scripts once; the lists of harness files follow the `test/...` names in the non-comment lines of the scripts (followed through the wrapper scripts they run), the quote-includes of those files and the
`pNN_layout_probe.cpp` of the probe loops, so a computed name that no script spells out may be missing.

`tools/job_aliases.tsv` (`old_id`, `new_id`, `planned`): stage 1 had `new_id == old_id`; since stage 10 `new_id` = `planned` = the active id (jobs.tsv `id`, `jobclass.txt`, the xfail list use it; stamps are `test/.stamps/<fid>`, fid = id with `/` -> `__`; see docs/testing.md "Job ids since TESTREORG stage 10"). `tools/job_path.sh`, `stampkeys.py` and `tier_golden64.sh` resolve through the table as well, so `cover_chunk.sh` / `mutate.py --chunk` / `run_job.sh` accept old ids.  `jobs_check.sh` also checks `jobclass.txt` = the ids, the xfail names, `h01_heap`'s MOVED list and the fid rules.
`sh tools/job_resolve.sh TOKEN ...` prints the active id of every token (old id, new id or planned name; other tokens pass through); `run_changed.sh` runs its id arguments through it.
`sh tools/jobs_check.sh` (no job is run, about a second): column count, unique ids and new names, id set = `run_changed.sh --list` + `x1_flagdiff`, every named file exists, every job script of `test/`
is named by a row, `stage2` agrees with the golden64 script / `<id>_blob32.out(.gz)`, the alias table agrees with the manifest and the resolver maps old, new and planned names.  Exit 0 and `OK: ...` when consistent.

## job_path.sh, test/lib/tier_golden64.sh - moved jobs (TESTREORG stage 2)

`sh tools/job_path.sh ID WHAT` prints a file of a job that was MOVED into `test/<module>/` (the first script of its `test/jobs.tsv` row has a `/`), read from the manifest: WHAT = `moved` (`yes`), `script`, `golden64`
(own `<stem>.golden64.sh`, empty if none), `blob32` / `sizes` (`test/golden/<module>/<stem>.blob32.out[.gz]` / `.sizes`; ONLY that exact name - since stage 7 the golden of another job or a facet golden listed in the row does not count, `stampkeys.py moved_job` agrees), `main`, `module`, `stem`, `build` (`test/.build/<module>/<stem>`).  A job not moved prints nothing and
exits 1: the callers keep spelling the old `test/<id>...` names for those.  Users: `run_changed.sh` (job list, `g64_cmd`, `g64_files`, key file list), `run_job.sh` (stage 1b), `stampkeys.py` (`moved_job`: the 1b stages and the golden listing of
the recipe key), `cover_chunk.sh <chunk>`, `mutate.py --chunk`.  `test/lib/tier_golden64.sh NAME` is the generic stage 2 / `GOLDEN_UPDATE=1` for the moved jobs (it began as a copy of the flat `test/tier_golden64.sh`, deleted by stage 9; see its header and the
section "Files of a moved job" of docs/testing.md); run it from anywhere.  `jobs_check.sh` additionally checks that the rows of moved jobs follow the naming convention.  Today moved: `rd` (module `ring`), since stage 3 the 18 jobs of `tone`, `cid`, `fsk`, `callprog`, `core`, since stage 5 the 40 jobs of `fax` (r18 .. r30; shared mains named after the id family, `.sizes` branch exercised by `r24_vtb`; journal section TESTREORG5), since stage 7 the 56 jobs of `vpcm` (p06 .. p47, v01 .. v07, z01, v34_tier4b / 9a / 10a, the four former `?vpcm` rows; the layout-unit, shared-script, two-main and `.sizes` + `.integ` golden branches; journal section TESTREORG7), since stage 8 the 20 jobs of `v32` (19, incl. the `?v32` rows and the two-row `n01_v32status` script + `-O2` wrapper) and `fpm` (1; h01_heap now runs 12 moved scripts; journal section TESTREORG8), since stage 4 the 29 jobs of `v22` (12), `voice` (6), `v8` (6), `dsp` (5; their own `<stem>.golden64.sh` and the `.layout` unit of p04 are the branches stage 4 exercised, see the journal section TESTREORG4).  Since stage 9 (the last) EVERY job is moved: the 16 jobs of `v34` (module files, the V.34 object support files `test/v34/v34_img.h`, `v34*map.h`, the v01 unit, `v34obj.layout` ...; `gen_views.sh`, `tier_rewrite.py`, `split_region.py` (glob `test/**/*.c`), `layout_check.py` follow), `v04` of the nested module `v34/handshake` (`job_path.sh` splits `new_name` at the first `/`: module `v34`, stem `handshake/v34_handshake`; `jobs_check.sh` compares the module column with the directory of `new_name`) and the four jobs of module `cross` (`h01_datasym`, `h01_heap`, `z01_hybrid`, `x1_flagdiff`; `run_changed.sh` plans the last two itself; journal section TESTREORG9).  The flat `test/tier_golden64.sh` is gone and the tools' branches for a job that is not moved are dead (kept).  `jobs_check.sh` check (9) fails when a script still runs a moved job under its old `build_and_diff_<id>.sh` name or `h01_heap`'s `SCRIPTS=` list names a moved id (h01_heap takes its `MOVED=` list separately).  Verified for stage 3 (R, TESTREORG3): `job_path.sh` `sizes` / `golden64` / `main` with a synthetic `JOBS_TSV` and the real `.sizes` path (`GOLDEN_SIZES=1 GOLDEN_UPDATE=1 sh test/lib/tier_golden64.sh r09_toneiir`, then `TG64_SCEN="0 1 2"`), `cover_chunk.sh r09_toneiir --rebuild`, `mutate.py run --chunk rcfixed --ids NOSUCHMUTANT` (base build + control only, no mutant run).

## TOOLFIX - tooling defects found by the test move

* **params_gen.py is a faithful generator again** (`python3 tools/params_gen.py [--write | --check]`, needs `../dsplibs.o`, about 4 s).  The committed `pcm_parameters.h` / `_load.cpp` / `.cpp`, `test/vpcm/pcm_parameters.layout` and `test/vpcm/p07_model.h` are what it produces.  Two things had drifted from it and are now inputs: (1) the hand-written names and member notes of the words that no loadParams key reaches (the 4 `inferred` and 23 `rename` rows: `tools/params_names.tsv`; a `rename` row keeps `unkNN=name` in the layout, the generated name without a row is `unkNN`); (2) the output of the `src/` files is passed through `clang-format` before it is compared / written (the files are formatted by the pre-commit hook), and the member comment style is `///<` as the committed files have it.  `sh tools/gen_views.sh --check` runs `params_gen.py --check` (exit 1 = the generator would change a file).  A new name or note for an `unkNN` word: add a row to `params_names.tsv`, run `--write`.
* **test/test_member_map.py passes again and runs in the gate** (`sh tools/gen_views.sh --check` runs it, about 25 s more).  Two TOOL-side defects: `member_map.py validate` did not know the layout kind `pad64` (a zero-size marker in the i386 layout; `layout_members` skips it like `align8`; `member_map_source.layout64` counts its `count` bytes on x86-64).  Two TEST-side defects: the real-file test looked for `paramWord` accesses in `src/dsp/dsp_resampler.cpp`, which X64FIX converted to named members (it now pins that no raw access bound to V90Parameters is left there), and the `validate` rows of the moved layouts are named by the layout FILE (`vpcm_session_wrappers.layout`, `dp_vpcm.layout`), not by the chunk id.
* **jobs_check.sh and goldens of another job**: a job whose golden column lists the OWN golden of another job (`<module>/<stem>.blob32.out[.gz]` with `<module>/<stem>` a job id; today `vpcm/pcm_bits_to_symbol.a` / `.b` list `.main`'s) is declared in `tools/golden_owners.tsv` (job, owner); `jobs_check.sh` fails when the row is missing or names a job that does not list the owner's golden.  The marker is a separate file because `test/jobs.tsv` is hashed into the stamp keys of 156 jobs.  The `.blob32.integ.out` goldens of the two `.integ` jobs and the facet goldens (`.x87`, `.nan`, `.graph`, `.a` / `.b` of `v90_modulator`) are the job's own and need no row.
* **GOLDEN_UPDATE=1 through a p29 flavour wrapper** (`test/vpcm/pcm_bits_to_symbol.a.sh` / `.b.sh`) exits 1 with a message: the golden is the main job's (both flavours); update it with `GOLDEN_UPDATE=1 sh test/vpcm/pcm_bits_to_symbol.main.sh`.
* **Absolute TPREFIX**: the scripts that start a built binary as `./${T}_test...` set `T_RUN` (empty when `$T` is absolute, `./` otherwise) right after their `T=` line and run `${T_RUN}${T}_test...`; before, `.//abs/path` was looked up relative to `./abs` (rc 127).  `test/jobclass.txt` is maintained by hand (its header said it was generated by a tool that never existed).
