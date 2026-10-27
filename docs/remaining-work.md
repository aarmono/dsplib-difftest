# Remaining work

One list of what is open, in progress or waiting on a decision.  Every pass updates it in the same commit as its journal section: move the item it finished to
"Done", add what it found or left over.  Details of finished passes live in `porting-journal.md`; known defects in `bugs.md`.

Status: **open** (nobody working), **running** (an agent has it, branch given), **waiting** (needs a user decision), **hold** (user said not yet).

## In progress and next

| Id | Item | Status | Notes |
|----|------|--------|-------|
| TESTREORG | `test/` restructuring (layout B: `test/<module>/` flat, files named by job stem, goldens in `test/golden/<module>/`, artifacts in `test/.build/`, ids renamed to the planned names) | done | Stages 1-10 merged (journal sections TESTREORG1-TESTREORG10; conventions in `docs/testing.md` and `tools/README.md` sections 27-29): all 185 jobs are moved, `test/tier_golden64.sh` is deleted (`test/lib/tier_golden64.sh` serves all), old ids and planned names resolve through `tools/job_aliases.tsv`.  The final rerun (`run_changed.sh -j 4 --g64 --all`, 185 jobs, failed=0) ran after stage 10; TOOLFIX then re-keyed every job again (see "Stamps are stale after TOOLFIX" below).  Open follow-ups: section "TESTREORG follow-ups". |

## Held or waiting for a decision

| Item | Status | Notes |
|------|--------|-------|
| Run the jobs V34REL and V34ACCESS skipped. V34ACCESS: `v04`, `v01`-`v07`, `v34_tier5b`, `v34_tier5d`, `v34_tier10b`, `z01`, `p47`, `x1_flagdiff` (the C++ handshake -O2 objects changed, so `v04` / `v01`-`v07` are the priority); V34REL: (`h01_*`, `v34_tier2`, `v34_tier3ab`, `v34_tier10b`, `v34_primitives`, `p05`, `p16`, `p21`-`p23`, `p25`-`p33`, `p37`, `p39`-`p44`, `p46`) | hold | Do in capped batches, outside agent tasks; `v34_tier2` first (modulator history index) |
| Rebuild and record the production hashes in `CLAUDE.md` (x86-64 `slmodemd` / `modem_test` changed with FLAGSFIX; both widths changed with the -O2 differences of V34REL / TYPEDPARAM / V34ACCESS) | open | Do with the next build-touching pass |
| Retype `VPcmFloModem::ctx_` and the `void *` entry points that take it | open | Changes the C++ mangled constructor name, so `dp_vpcm.o` and `vpcm_flo_modem.o` differ at -O2; allowed under the relaxed rules |
| The 50 dead functions (`v34_unused.c`) | hold | |
| Stamps are stale after TOOLFIX | waiting | TOOLFIX edited the text of 168 scripts (absolute `TPREFIX`) and `test/lib/tier_golden64.sh`, so all 185 jobs have a new key (compiled `O` items identical in all 355 key logs; 173 jobs changed through their own script, 12 only through `tier_golden64.sh`); only 6 jobs ran since (`fsk/v21_b103`, `core/rcfixed_resampler`, `fpm/fpm_toolkit`, `dsp/dsp_int_templates`, `vpcm/pcm_bits_to_symbol.a` / `.b`).  Decision for the user: another full `run_changed.sh -j 4 --g64 --all` (hours), or revert the three absolute-`TPREFIX` commits `a785766`, `e1105e1`, `419328b` (then 56 jobs re-key: the 54 golden64 jobs and the 3 p29 wrappers net out) and rerun those.  No stamp or golden was touched. |
| `include/dsplib/dsplib.h` | hold | |
| Split the signal-chain section out of `architecture.md` | waiting | |
| Delete the o0check cache (`/tmp/o0check-$USER`, about 594 MB) and `/tmp/claude-1000` (about 2.8 GB) | waiting | Needs the user's approval |

## TESTREORG follow-ups (the move and the id renames are DONE; all optional, none is started)

| Item | Status | Notes |
|------|--------|-------|
| Dedup of the harness helpers | open | About 25 harnesses carry their own copies of `die3`, `emit`, `fnv_*`, `hash_bytes`, `rnd`, `find_blk`, `alarm_handler`.  Move them into `test/harness/`; each changed harness needs a real job run (a harness object changes, so the key changes); do it in capped batches by module. |
| Dead "unmoved job" code paths | open | All 185 jobs are moved, so these branches can never run: `run_changed.sh` (flat-glob loop, `rd` special case in `jobs()`, `test/<id>_golden64.sh` / `${id}_blob32` branches in `g64_cmd` / `g64_files`, the `--matrix` `sc=` default), `run_job.sh` (`${id}_blob32` fallback), `stampkeys.py` (`moved_job` returning None, the flat `<id>_*` golden listing), `job_path.sh` (the exit-1 "not moved" path), `jobs_check.sh` (checks 5, 6 and 9 for flat names), `cover_chunk.sh` / `mutate.py` (flat `build_and_diff_<chunk>.sh` default), `filemap.py` `GOLDEN_LISTS` (historical).  Removing them re-keys jobs that hash those scripts, so they need a rerun of the affected jobs. |
| Stale `src/` comments naming moved test files | done | 30 references in 21 `src/` files now name the files at their new `test/<module>/` paths (comment-only; `tools/objcmp.sh HEAD` IDENTICAL, 848 objects).  Left as they are on purpose: the `test/p02.layout` / `p03.layout` / `p05.layout` names in the three `src/dsp/*_layout.inc` banners (those units still live in `test/`). |
| Home of the shared support files left in `test/` | waiting | `p02` / `p03` / `p05` layout units, `p01_layout_probe.cpp`, `r02_common.h`, `r04_common.h` (all users have moved; they belong to no single module); also `test/lib/tier_golden64.sh` (about 200 scripts and the tools name that path). |
| Stale comments in test scripts naming old ids and files | open | About 40 `GOLDEN_UPDATE=1 sh ... r28_misc` usage lines still show old ids (they work through the alias table); four v34 scripts name `build_and_diff_v32_primitives.sh`; mains of v8, fsk and v34 name `test/fpm_diff_main.c`; `docs/bugs.md` rows N02-B and N04 name old script names; `tools/README.md` still uses flat `test/p0N_*` names for the dsp jobs; (fixed in TOOLFIX2: `v34ctxmap.h` / `v34cfgmap.h` are generated and checked by `gen_views.sh`, the `v34obj.layout` generator comment, the `V90CP::extendEu_` comment of the two vpcm_flo_modem layouts, the three fax tx create comments).  Editing script text re-keys those jobs. |
| Tooling defects found by the move | open (TOOLFIX fixed 6 of them, see the journal section TOOLFIX) | Left: the recipe key includes the job id, so another id change costs about 18 minutes of key recording (an id-free recipe key would also change the cache layout); `n02_vclock.c` and `p03_transcendental_study.cpp` are not jobs; 36 scripts have no `TPREFIX`; `test/x3_fax_pair.py` is not a job but is the documented fax smoke driver (testing.md, architecture.md): keep it.  `test/cfgptr_check.sh` (not a job, nothing runs it) was repaired: include paths for the module headers, artifacts under `test/.build/`, `-m32` and `-m64` outputs IDENTICAL (106 lines).  `pcm_bits_to_symbol.main.sh` already creates `test/.build/vpcm` (a clean checkout run passed; the reported failure does not reproduce).  Fixed by TOOLFIX: `params_gen.py` (reproduces the committed files, `--check` in `gen_views.sh --check`), `test_member_map.py` (repaired, run by `gen_views.sh --check`), absolute `TPREFIX` in the `./${T}_test` scripts, `GOLDEN_UPDATE=1` through the p29 wrappers (refused), the `jobclass.txt` header, `jobs_check.sh` and goldens of another job (`tools/golden_owners.tsv`). **The TOOLFIX script edits re-key jobs: the stamps of those jobs need a rerun (journal TOOLFIX lists how many).** |
| Mutation-run output | done | The run directories of `test/p35_mut_run1` .. `p47_mut_run2` (about 168 MB, untracked) were reduced, with the user's approval, to their `report.txt` / `survivors.txt` / `meta.json` and the logs, kept in `docs/mutation-results/` (about 1 MB, tracked, see its README); the scratch was deleted. |
| Untested paths after the id rename | open | `--matrix`, `GOLDEN_UPDATE=1` of `tier_golden64.sh` under the new ids, `cover_chunk.sh --rebuild`, `mutate.py run --chunk` (only the id resolution was checked).  The first user of each should check it. |

## Optional typed-parameter leftovers (TYPEDPARAM kept `void *` / `char *` here)

* `VPcmFloModem::ctx_` is `void *` (its type is in the mangled constructor symbol; retyping changed the -O2 objects); `VPcmV34SetV90RateReneg`, `V34XF_Indicate{Jd,Dil,Trn2d}Received`, `VPcmV34Indicate{Local,Remote}RRN`, `VPcmV34SetIndicationOfRemoteRetrain`, `VPcmV34GetMaxUpstreamRateIndex`, `V34XF_Get*` stay `void *` for that reason.
* V.32 (after TYPEDPARAM2): `VTB_decoder` stays `char *` (shared with the fax V.17 modem, which hands its own `FaxVtb`), `FPM_lmsupd` (`char *historyBase`, shared with fax), the V.32 handler buffers (`void *rawBits` / `out` / `inSamples` / `outBits`: bytes or shorts depending on the state) and `V32FP_modem`'s buffers, `V32FP_GetCleanedSamples` (returns the sample buffer as `char *`); V32ENG (done): the engine and handshake locals are `V32Eng *` / `V32Hdx *`, V32Eng embeds its sub-objects; still raw there: the `agc[28]` read (candidate bug), the MRF / AGC byte arrays, the address slots (real pointer members would change the layout), and the MEMBER_IDX / MEMBER_AS_OF views inside the sub-object files.
* Fax wrappers (`fax_modem_wrappers.c` and the `faxw_*` accessors: the opaque modem object of three modem types through offset accessors), `fax_init_vmi_*` (callback `user`), `V27RX_status` / `v27rx_status` / `null_status` (`void *st`, one slot for several status types), `Dialer_IsDialStringInvalid` (the r10 harness passes a fake CallProg image), `v8_phase_rev_*` (`phaseRev` is an untyped byte record), `V8Delete`, `dp_runtime.c` / `fax_api.c` / `voice_api.c` / `cid` entry points (host ABI), the voice state handlers and the `put16` helper (byte or short buffers), `voice_engine` buffers.

## Optional V.34 leftovers

* `V34Rx` `smp_[8]` (V34TRELLIS): the stores `smp[1 + i]` are unbounded by the count `w38c_` (4 at every library writer, up to 21 in tier5b); `@warning` in receiver / rxtiming.
* Never-accessed gaps: `unk04d4_` (8 bytes), `unk0426_`, `unk045a_`, `unk048a_`, `unk0490_`, `V34Mod` `unk2104_`, `unkaba0_`.
* Raw accessors left after V34TRELLIS: the FSK state block of `v34_detectors.c`, the sample rings and window of `v34_echo_rx_timing.c` (9 `U16` uses), the stride-8 word tables `w2a34_ ..` and the `symPts_` pair read of `v34_modulator.c` (5), `demapFrame`'s `const uint32_t *sym` parameter; in the C++ handshake the `tag_bitQ` `unk00_` words (`ld16` / `st16`) (the DFT bins were typed by V34DFT, the trellis area and the 32-bit pairs by V34TRELLIS).
* Type the `AS_U16` sites; tighten slice sizes.
* Sole-definition treatment (as V34SUB) for the V.90 / V.92 views.
* `test/` and `tools/` are not clang-formatted yet.

## Done (recent, newest last; see the journal)

CM1-CM13 comments, PM1-PM7 plain member access, RL1-RL6, `CID_process` return value (`c895058`), declaration audit (`declaration-audit.md`), `bugs.md` section V,
V34SUB, V34CLEAN, V34REL (`843bf8d`), HOUSE (test_tier_rewrite fixed, `bugs.md` V.12 V75-V79, old worktrees and branches removed), `UINT_MAX` / `USHRT_MAX` constants (`bef55f2`), TYPEDPARAM (86 functions take typed object parameters, journal section TYPEDPARAM), V34ACCESS (the `S16T` / `U16T` / `S32T` / `U32T` / `PTRT` / `V34OFF*` / `V34SUBP` / `OBJ_BYTES` uses of the live V.34 / vpcm files become plain members, 27 unknown regions become 34 members, V34demodulate / V34SetupModulator / detectorinit / initV34 / V34EchoHistoryBackwardClean / resetRetrainDetector take typed objects; journal section V34ACCESS), FLAGSFIX (`chkForceBaudRate` reads `ctx_->v34Flags_` instead of `ctx + 0x217`, wrong at x86-64; `bugs.md` VR3; journal section FLAGSFIX)., X64FIX (V90Phase2Info / V92Phase2Info, V90Resampler and vpcm_delete use the named members / `struct dsp_info`; new native job `x4_params64`; `bugs.md` VR4-VR6, V47 text corrected; journal section X64FIX; open follow-ups there: the stale `@bug` text of the three fax tx create functions, `v90_equalizer.cpp` `params + 0x5c`). TYPEDPARAM2 (118 functions outside V.34 / vpcm take their object type: the V.32 wrapper chain `V32Wrap *`, V32Fse / V32Pps / V32Ecc / V32Sre / V32Sdm / V32Smc / V32Vtb, V22Status, V23Cfg, V8_setFilters, voice_getgain; -O2 identical at both widths; journal section TYPEDPARAM2).
V32ENG (V32Eng embeds sdm / smc / pps / circ / ecc / sre / fse / hist(.vtb) / rxSdm, `opt[10]`, `V32Eng *` / `V32Hdx *` locals; journal section V32ENG; three -O2 functions differ).
UNIONFIX (`_tagModemParameters` is one flat struct, one member per word; journal section UNIONFIX; three -O2 objects differ, listed there).
V80FIX (`V32FP_status` reads the AGC detect state through `V32E_AGCF`; new job `x5_v32status64`; `bugs.md` VR7; journal section V80FIX; V81 stays an open candidate).
V90TIE (`V90ParObj` tied to `V90Parameters`: 342 members + sizeof at both widths, no defect; `K56HandleObj` size / offsets tied, `K56FLEX_Create` allocates `sizeof`; journal section V90TIE; -O2 identical).
V34DFT (`DftBin` 0x2c-byte DFT bin record: `probe.bin_[25]`, `dftB_[4]`, `dftC_[3]`, `wa256_` + `probeEnergy_[25]`; 25 records fill V34Probe exactly - the old "reaches past the struct" notes were a miscount; probeselect / chkForceBaudRate / dftupdate / dftenergy / V34GiveProbeResults / v34modeminit and the C++ RX_L1 / RX_DPSK users typed; journal section V34DFT; x86-64 layout unchanged; job v04 not run, `x1_flagdiff` not run).

V34 batch 1 (p06 p45 p47 z01 v34_tier3d/4b/5b v34_primitives h01_repeat passed with --g64; x1_flagdiff golden updated: v34_hs_rx_l1 in, v34_probe_select out, benign dead stores).

V34 batches 2-3 (g64 PASS: v34_tier3ab, p21-p23, p25-p28, p37, p40-p44, p46, v34_tier5a, v34_tier8a, v34_tier10b; p29-p33 and r04_sre / r05_* / r06_glue unchanged by stamp; p39a/b not run, no job of that name).

V34 batch 4 (g64 PASS: p18-p20, h01_heap, n01_v32status, r03_e2e, r03_glue, r02_v23, v02, v03, v05, v06, v07; r17_* unchanged by stamp). All previously skipped jobs have now run.

TESTREORG stages 1-10 (the `test/<module>/` layout, 185 jobs moved, ids renamed to `<module>/<stem>`; layout-drift fix with `tools/layout_check.py`; final `--all --g64` rerun passed; stray build artifacts deleted; stale `src/` comments updated, `d2efe23`) and TOOLFIX (`params_gen.py`, `test_member_map.py`, absolute `TPREFIX`, p29 flavour wrappers, `jobclass.txt`, `golden_owners.tsv`; `cfgptr_check.sh` repaired, `98a0261`).

## Standing rules

Test runs are capped: no long jobs per work item; one small `run_changed` run at the end of a task (3-6 jobs, about 30 minutes) and the skipped jobs listed in the report;
never `--all` for routine chunks.  No production slmodemd C changes without agreement (report candidates).  One code agent at a time.
V34BITQ (`V34BitQTail`: the six bit-queue records share the 28-byte tail `tail_`; 433 uses renamed; getbit / V34SetINFO0dBits take `tag_bitQ *`; journal section V34BITQ; v04 / v34_tier5d / v34_tier2 + x3 passed).
V34TRELLIS (`rx.trellis_` TrellisMem, `fbHist_[6]`, `smp_[8]`, `rxShellIdx_` / `decPt_` / `eqPt_` pairs, `fdFields_[16]`, `ringNo_[8]`; journal section V34TRELLIS; tier4a / 3ab / 3c / 5a / 9a + x3 passed, tier5b UNTESTED).
