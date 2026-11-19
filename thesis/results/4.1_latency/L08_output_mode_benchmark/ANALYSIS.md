# L08 Same-day output-mode benchmark: lag, tracking and smoothness

**Report:** 4.1.4, Table 4.1, F5.5 (signature figure). **Question:** with identical input on the same day, how do
trajectory, position, filtered velocity and unfiltered velocity output compare in lag, tracking error and
smoothness, and is the difference larger than the replay floor?

**Prior claims to test:** 1:1 lag 66–78 ms (position, filtered velocity) vs 170–248 ms (trajectory), paired
difference ~120 ms; rotation lag higher at 1:1.

## Data

`data/recordings/V9b/NoTool/{TrajectoryMode,PositionMode,VelocityOptimised,VelocityUnfiltered}/*_Replay{1,2}`: six
trajectories (Long, Short, NoBuzzArea, LargeVibrations at 1:1; Scaled5 Slow/Fast at 1:5), 38 replays. Bare flange,
calibrated kinematics (check META per run). The run table is built from META/REPLAY, including `code_version`
per mode.

## Extraction (the clever bits)

- **Write the estimator, then validate it on synthetic data:** shift a recorded target by a known delay, add
  the measured jitter, run the estimator, recover the delay. Do this before trusting any number (the inherited
  `plot_paths.compute_metrics` is a candidate, not a given; this folder reimplements and cross-checks it).
- **Lag from velocity cross-correlation as well as position best-shift.** Position best-shift is biased by
  constant offsets and by the P-loop's slow correction; velocity cross-correlation is not. Report both; a
  disagreement is itself informative.
- **Frequency-resolved lag:** the phase of the cross-spectrum between target and arm velocity divided by 2πf
  gives lag as a function of frequency. A single number hides that the arm may be fast for slow motion and slow
  for fast motion; this curve is the honest version and it feeds folder L06.
- **Gap check before metrics:** detect gaps in `UR_PATH.csv` (> 3× median dt) and in the bag; any run with a gap
  inside a stroke is masked or excluded with the reason printed (this is how the known 334 ms gap in one
  VelocityOptimised ShortBenchmark replay must be rediscovered, not looked up).
- **Smoothness that cannot be gamed by lag:** jitter = rms of the arm path above 5 Hz *relative to the input's
  own* above-5-Hz content (ratio), plus angular jerk p95 from the measured joint velocities.

## Method

1. Per replay: translation and rotation lag (both estimators), lag-removed rms error, jitter ratio, jerk p95,
   frequency-resolved lag.
2. Unit of analysis = trajectory: mean of its replays per mode.
3. Paired comparison across trajectories: differences with 95 % CI (t) and Wilcoxon signed-rank; also
   ratio (trajectory / position) per trajectory.
4. Every difference reported against the replay floor (folder M01).

## Pseudocode

```
for run in NoTool:
    check_gaps(run) -> exclude/mask with reason
    tgt = TOUCH_PATH; arm = UR_PATH(source == rtde)
    lag_pos  = argmin_shift rms(arm(t+s) - tgt(t))
    lag_vel  = argmax_shift xcorr(d arm, d tgt)
    lag_f    = phase(cross_spectrum(d tgt, d arm)) / (2π f)
    err      = rms(arm(t+lag) - tgt(t)); jit = hp5(arm)/hp5(tgt); jerk = p95(|d³ angle|)
per traj: mean over replays; paired diffs across trajectories; CI, Wilcoxon
synthetic validation of lag estimators first
```

## Figures

1. **Signature (F5.5), "Fast or smooth: you can have both"**: x = lag (ms), y = jitter ratio (arm / input). One
   mark per replay, colour + marker per mode. Replays of the same trajectory across modes joined by a thin
   grey line (a slope graph inside a scatter), so each line shows what switching mode does to *that* input.
   Horizontal line at ratio 1 (= the arm as rough as the hand), vertical band at 50 ms. Trajectory output sits
   bottom-right, the shipped modes bottom-left.
2. **Lag vs frequency**: frequency-resolved lag per mode (median over trajectories, IQR ribbon), 0.2–8 Hz. Shows
   where position output gains its lead and where rotation lags.
3. **Paired differences**: one row per trajectory, dot = trajectory − position lag, CI bar for the mean. Simple,
   and it carries the statistics.

## Outputs

Table 4.1 values (rebuilt), paired difference with CI and Wilcoxon p, the excluded runs with reasons, both lag
estimators, the frequency-resolved curves (for folder L06).

## Status (2026-11-13)

Code: `analysis.py` in this folder (run from `thesis/results` with ROS and the workspace sourced, or use `run_all.sh`). Results, figures and the check of every prior claim are written by the code to [`RESULTS.md`](RESULTS.md) and `figures/`; no number in RESULTS.md is typed by hand. Read RESULTS.md for what was done, what was excluded and why, and what is still open.
