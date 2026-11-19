# M01 Replay repeatability: the floor every comparison is judged against

**Report:** 3.4 (method), and the "against the floor" statements in 4.1–4.3. **Question:** when the same recorded
input is replayed twice into the same configuration, how much do the arm's path and every metric differ? A
difference between conditions only counts if it clearly exceeds this.

**Prior claims to test:** two V5 replay sets matched within 1 % (lag 62/116 vs 62/114 ms; runs since overwritten);
RCM replays repeat to ~0.1 mm.

## Data

Every replay pair or triple of identical input and configuration (same META settings and code_version):
`V9b/NoTool/*` (19 pairs), `V9b/Singularity/*` (2 triples), V8/V9/V9b RCM replays (triples, with NDI), V7 RCM
replays, V5 benchmark vs Tuned are *not* pairs (different settings). Build the groups from META + REPLAY
(identical `replay_of`, settings hash, kinematics hash, code_version), not from names.

## Extraction (the clever bits)

- **Path-level repeatability, not just metric-level:** align replays by the stylus input (cross-correlation),
  then take the arm-path difference sample by sample (rms and max, translation and rotation). This is the most
  direct statement of "deterministic", and it does not depend on any metric's definition.
- **Input fidelity:** the replayed stylus should be bit-identical; measure the difference of the *recorded*
  STYLUS_RAW between replays (timing jitter of the replay player itself).
- **Metric-level:** each metric of folders L07, L08, R02 per replay → within-group spread.
- **Time drift:** the order of replays within a session (Replay1 → 3) as a covariate, to catch warm-up.

## Method

Per group: path rms/max difference; per metric: range, and coefficient of repeatability (Bland–Altman, 1.96·√2·s_w);
pooled within-group SD across groups per metric. Report as "the floor for metric X is Y".

## Pseudocode

```
groups = group_by(replay_of, settings_hash, kin_hash, code_version)
for g in groups:
    align replays by xcorr(stylus speed)
    path_diff = |arm_i(t) − arm_j(t)| → rms, max
    input_diff = |stylus_i − stylus_j|, timing jitter
    for metric in [lag, err, jitter, band_power, rcm_rms]: values[g, metric] = [...]
s_w[metric] = pooled within-group SD; CR = 2.77 s_w
```

## Figures

1. **"Replays overlap"**: one representative pair, arm path vs time for one axis, both replays drawn, with
   the difference trace magnified ×100 underneath on its own axis (labelled). The magnification makes the point
   that the difference is tiny at full scale and still structured, not noise.
2. **Bland–Altman per metric** (small multiples): mean of the pair vs difference, limits of agreement, with
   the effect sizes from 01/05/09 drawn as reference lines at the edge, so the reader sees floor vs effect.

## Outputs

A floor table (metric → within-group SD, CR) consumed by every other folder; the input-fidelity number for 3.4.

## Status (2026-11-13)

Code: `analysis.py` in this folder (run from `thesis/results` with ROS and the workspace sourced, or use `run_all.sh`). Results, figures and the check of every prior claim are written by the code to [`RESULTS.md`](RESULTS.md) and `figures/`; no number in RESULTS.md is typed by hand. Read RESULTS.md for what was done, what was excluded and why, and what is still open.
