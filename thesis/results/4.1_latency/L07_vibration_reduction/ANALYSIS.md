# L07 Vibration reduction against the unfiltered baseline (threshold (i))

**Report:** 4.1.3, 4.1.4, scorecard (i). **Question:** with identical input, how much less does the arm vibrate
in the tremor band with the final filtering than with none, and is that ≥ 30 %?

**Prior claim to test (not an input):** 15–25 Hz joint-velocity power down 76–88 % (position) and 64–68 %
(filtered velocity) vs unfiltered.

## Data

| Condition | Runs (`data/recordings/V9b/NoTool/`) | Command topic |
|---|---|---|
| Unfiltered baseline | `VelocityUnfiltered/Scaled5_{Slow,Fast}_Replay{1,2}` | `/forward_velocity_controller/commands` |
| Position (final) | `PositionMode/Scaled5_{Slow,Fast}_Replay{1,2}` | `/joint_velocity_limiter/commands` |
| Velocity, filtered | `VelocityOptimised/Scaled5_{Slow,Fast}_Replay{1,2}` | `/joint_velocity_limiter/commands` |
| Trajectory (reference) | `TrajectoryMode/Scaled5_{Slow,Fast}_Replay{1,2}` | `/scaled_joint_trajectory_controller/joint_trajectory` |

Only the 1:5 runs have a baseline, so the comparison is 2 trajectories × 2 replays per condition. Confirm from
each META.yaml (not the folder name) that `TOUCH_COMMAND_FILTER` and the Kalman flag differ only where expected,
and that kinematics, scale and tool match.

Signals, all from the bags at 500 Hz:
- **Measured joint velocity** `/joint_states.velocity` (the arm, what threshold (i) is about).
- **Commanded joint velocity** (the topic above, or differentiate the trajectory set-points), to separate "the
  command was rough" from "the arm amplified it".
- **Stylus** `/phantom/state` (the input, identical across conditions by construction; checks the replay).
- **Motor current** `/joint_states.effort` as a second, independent channel.

## Extraction (the clever bits)

- **Same input, so align by input.** The replays play the same stylus file, so cross-correlate the stylus
  speed between runs to align them to < 2 ms, then compare segment by segment. Differences then come from the
  stack, not from where a window happened to fall.
- **"While moving" from the input, not from the arm.** Moving = stylus target speed above a threshold (pick
  it from the target-speed histogram's valley, report it) for ≥ 0.5 s, with 0.25 s trimmed at each end.
  Defined on the input, the window is identical across conditions, so a mode that vibrates at rest cannot
  shrink its own window.
- **Tool-space vibration without a tracker:** map joint velocities through the Jacobian (calibrated kinematics
  from META) to tool linear and angular velocity. Threshold (i) speaks of distal-tip vibration. This gives a
  tool-space number from joint data, at the tool0 flange (stated plainly as not the tip).
- **Detrend motion, keep vibration:** within each moving window subtract a 3 Hz zero-phase low-pass of the
  signal (offline, so zero-phase is fine here) before the PSD, so the large voluntary motion does not leak into
  15–25 Hz through window edges.

## Method

1. Welch PSD per joint per moving window (Hann, 1 s segments at 500 Hz, 50 % overlap); pool windows per run
   weighted by duration.
2. Band power in 15–25 Hz (the band fixed **before** looking at the result; also report 8–12 Hz tremor and
   30–60 Hz as secondary bands), summed over joints after weighting each joint by its tool-space lever (from the
   Jacobian), and in tool space directly.
3. Reduction = 1 − P_filtered / P_unfiltered, per trajectory (paired), then across the two trajectories.
4. Uncertainty: block bootstrap over Welch segments (blocks of 1 s, 2000 resamples) → 95 % CI of each
   reduction. Replay-to-replay spread shown separately (it is the repeatability floor, folder M01).
5. Repeat on the commanded signal: if command and arm reductions agree, the arm is passive in this band and the
   filter is the cause.

## Pseudocode

```
runs = run_table(V9b/NoTool, scale=1:5)
for run in runs:
    js   = bag(run, /joint_states)            # t, q, qd, current (by name)
    cmd  = bag(run, command_topic(run.mode))
    sty  = bag(run, /phantom/state)
align every run to the first unfiltered replay by xcorr(|stylus velocity|)
moving = windows(|target velocity| > v_thr, min 0.5 s, trim 0.25 s)       # from input, shared
for run, window in moving:
    x = qd[window] - lowpass_zero_phase(qd[window], 3 Hz)
    v_tool = J(q[window]) @ x                                              # calibrated kinematics
    psd[run] += welch(x, v_tool, current)
band_power[run] = integrate(psd[run], 15..25 Hz)
reduction[mode, traj] = 1 - mean(band_power[mode, traj]) / mean(band_power[unfiltered, traj])
ci = block_bootstrap(segments, 2000)
```

## Statistics

Paired by trajectory; effect vs floor (`planning/10` §10.2). No t-test on 2 replays. Report: reduction [95 % CI]
per trajectory per mode, the replay spread, and whether the lower CI bound clears 30 %.

## Figures

1. **"Where the filter removes energy"**: the PSD *ratio* (filtered / unfiltered, log y, 1–100 Hz) per mode,
   one line per trajectory with its bootstrap ribbon, the 15–25 Hz band shaded, a horizontal line at 0.7
   (= the 30 % threshold) so the verdict is the gap between the curve and the line. A ratio plot shows *where*
   the reduction happens, which raw overlaid spectra hide.
2. **Spectra chain (F5.3)**: three stacked panels sharing the frequency axis: stylus → command → arm, unfiltered
   vs filtered, so the reader sees tremor enter at the stylus, get multiplied by 2πf in the command, and be cut
   by the filter. Annotate the tremor peak and any 19.5 / 39 Hz rate lines.
3. **Paired dot plot** of band power: one line per trajectory joining unfiltered → filtered, log y, with the
   −30 % reference drawn as the slope a line must beat.

## Outputs

`RESULTS.md`: per trajectory × mode reduction with CI, the moving-window definition and threshold, and the
joint-side vs tool-space numbers. Figures to `figures/`.

## Pitfalls

- 500 Hz `/joint_states` cannot see > 250 Hz; fine for this band.
- Velocity output at 1:5 only: the baseline cannot be extended to 1:1 (it was never replayed there, by rule).
- If replays of the same condition differ by more than the effect, say so; do not average it away.

## Status (2026-11-13)

Code: `analysis.py` in this folder (run from `thesis/results` with ROS and the workspace sourced, or use `run_all.sh`). Results, figures and the check of every prior claim are written by the code to [`RESULTS.md`](RESULTS.md) and `figures/`; no number in RESULTS.md is typed by hand. Read RESULTS.md for what was done, what was excluded and why, and what is still open.
