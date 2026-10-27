# Mutation-run results (kept from the old `test/p*_mut_run*` directories)

Results of `tools/mutate.py run --chunk ...` runs on the vpcm chunks p35 .. p47 (chunk ids = the old job ids; see `tools/job_aliases.tsv` for the module/stem names).
Per run directory: `report.txt` (counts and the surviving mutants with their source lines), `survivors.txt` (the survivors only) and `meta.json` (the run's parameters).
The run directories' scratch (mutant lists, per-mutant results, compile trees, about 168 MB) was deleted with the user's approval; a run is reproduced with `mutate.py`.
`p43_mut_run3` has no report (an interrupted run).  The `.log` files are the run logs, `p43_mut_time.txt` / `v02_mut1.log` / `p47_seeds.log` small timing and seed notes.
The headline numbers of each chunk are in `porting-journal.md` (sections "Mutation").
