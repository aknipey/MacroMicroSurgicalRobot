# Thesis plotting area

Comparison figures built from `logs/`, which is only read. Everything lands in `out/` (PNG to look at, PDF for
LaTeX). One style, and **one colour and marker per condition everywhere**:

| Condition | Folder | Colour / marker |
|---|---|---|
| V7 nominal kinematics, velocity output (10-26) | `logs/V7` | blue ● |
| V8 calibrated kinematics, velocity (11-03) | `logs/V8/CalibratedKinematics` | orange ■ |
| V9 calibrated kinematics, position (11-03) | `logs/V9/PositionMode` | green ▲ |
| V9 calibrated kinematics, trajectory (11-03) | `logs/V9/TrajectoryMode` | yellow ◆ |

Trajectory sets: RCMOnly, WholeRun(DrinkBottle), FastChecking, SlowChecking (`*SimpleHolder_ReplayN`, NDI
required). Excluded runs and their reasons are in `common.EXCLUDE`. Right now that is only
V8 FastChecking_Replay1, where the camera moved mid-run (see E0). Excluded runs still show, hollow, in the strip plot.

```
cd ~/ros2_ws/src/MacroMicroSurgicalRobot
python3 thesis/plotting/make_all.py                 # everything (first run ~18 min for tracking metrics, then cached)
python3 thesis/plotting/rcm_plots.py target ecdf    # some RCM figures (~5 s)
python3 thesis/plotting/tracking_plots.py
python3 thesis/plotting/compare.py <run> <run> ...  # ad-hoc sheet for any NDI runs; --group pools folders
```

To add a condition (e.g. V9b), add one entry to `CONDITIONS` in `common.py` with the next palette slot
(`#e87ba4` ▼, `#008300`, `#6250d6`, `#e34948`). Every figure picks it up.

## Figures and what they show now

| File | What | Reading of the current data |
|---|---|---|
| `rcm_replay_strip` | every replay's NDI RCM rms as a dot, median bar, 1 mm target line | Replays repeat to ~0.1 mm, so differences between conditions are real. Only RCMOnly is under 1 mm in every condition; trajectory mode just makes it on FastChecking (0.96). |
| `rcm_target` | where the shaft crosses the RCM plane, sets × conditions, coloured by time | SlowChecking is an **offset**, not a wobble: the cloud sits 2.6 mm (V7) / 1.1-1.3 mm (11-03) away from the latched point. FastChecking is broad scatter. |
| `rcm_bias_spread` | each replay's rms split into bias (latched point vs real pivot) and spread | Puts the E0 latch finding on one axis: SlowChecking is bias-dominated, RCMOnly is under 0.5 on both axes, and the output modes sit on top of each other. |
| `rcm_ecdf` | share of RCM time within X mm (marker = median) | Reads "within 1 / 2 mm" directly. Calibrated SlowChecking holds ≥ 96 % within 2 mm (V7: ~20 %). |
| `rcm_timeline` | error vs time since latch: replay median and range | The replay bands are hair-thin: the arm is deterministic, so the error is systematic, not noise. All conditions start near 0 and rise as the arm leaves the latch pose. |
| `rcm_tilt` | error vs shaft tilt from the latch direction | V7 WholeRun grows almost linearly with tilt (to 2.2 mm at 40°), which is pose-dependent error. On 11-03 it stays flat at ~0.7-1 mm. |
| `tracking_tradeoff` | lag vs > 5 Hz jitter per replay; dashed = the stylus input's own jitter | The mode trade-off in one picture. Trajectory mode lags ~160-190 ms but is the smoothest; velocity and position both lag ~56 ms. Every mode stays below the input's jitter. |
| `tracking_profile` | dot matrix: lag, shape error, jitter, rotation lag, RCM rms per set × condition | Position mode ≈ velocity mode on every metric (the first real-arm position runs: no penalty). Trajectory trades 3× the lag and 2× the shape error on FastChecking for lower jitter and lower RCM error. |
| `compare_*` | `compare.py` sheet: targets, error vs time, ECDF, numbers table | `compare_slowchecking` = Replay1 of each condition. |

Metric definitions for tracking are those of `scripts/plot_paths.py` (`compute_metrics`), imported and run in
memory. Its own CLI writes METRICS.yaml into the run folders; this area doesn't. The cache is
`out/cache/tracking.json` (keyed on file times; delete it to recompute).

## Notes

- Target plane: through the NDI-latched point, normal to the mean shaft, u along camera +y. The camera moved
  between days, so u/v directions are comparable within a day, not across days. Distances are comparable
  across days.
- rms² = bias² + spread² holds exactly for the crossing points. Crossing distance ≈ rcm_error / cos(tilt).
- Colours are slots 1-4 of a validated palette; the markers carry identity for colour-blind and print readers.
