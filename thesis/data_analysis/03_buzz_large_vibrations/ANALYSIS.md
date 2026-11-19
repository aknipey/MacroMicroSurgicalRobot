# 03 The high-frequency buzz: where, in which modes, and what it tracks

**Report:** 4.1.4 residuals, 4.5.4 limitations. **Question:** is the audible > 80 Hz buzz a property of the
pose, the output mode, or the command, and is it present in position output?

**Prior claims to test:** speedj-only and pose-dependent (V7); in V9b position output shows 0.32–0.35 A vs
trajectory 0.055 A in 80–249 Hz; earlier "follows acceleration" correlations held only within a buzzing pose.

## Data

| Runs | Why |
|---|---|
| `logs/V7/LargeVibrations`, `V7/NoBuzzArea`, `V7/WeirdVibrationSounds` | the original buzz (velocity) and a no-buzz control |
| `logs/V9b/NoTool/{PositionMode,TrajectoryMode}/LargeVibrations_Replay{1,2}` | same input, two modes, same day |
| `logs/V9b/NoTool/*/NoBuzzArea_Replay*` | control region, three modes |
| every other bag with `/joint_states` | to map the buzz across the whole workspace for free |

Never replay LargeVibrations in velocity output (project rule); do not request new runs.

## Extraction (the clever bits)

- **Short-time band energy, not one number per run.** Spectrogram of `/joint_states.effort` (current) per
  joint: 0.25 s Hann windows, 50 % overlap, band 80–249 Hz (below Nyquist 250). One number per window gives a
  time series of buzz that can be joined to pose, speed and command at the same instant.
- **Pose map from every bag.** Each window has a mean joint configuration. Pooling all runs from V5 onwards
  (thousands of windows) gives buzz energy as a function of where the arm is, independent of which experiment
  the window came from. That tests "pose-dependent" with far more coverage than the two dedicated runs.
- **Separate command from response.** For the same windows, compute the 80–249 Hz content of the *command*
  (velocity commands, or differentiated position set-points). Buzz with a quiet command = the arm/drive;
  buzz with a rough command = the stack.
- **Aliasing check.** Anything above 250 Hz folds into the band. Look for spectral lines that move with joint
  speed (mechanical orders) vs fixed lines (electrical or aliased).
- **Baseline per joint at rest** from still periods (target speed ≈ 0) to get the floor of each joint's
  current noise.

## Method

1. Fix the metric before computing: band 80–249 Hz, per joint rms current in A, windows labelled moving/still
   from the input (as folder 01).
2. Per window: band rms per joint, joint configuration, joint speeds, tool speed, command band content, mode,
   run.
3. Mixed model: log(buzz) ~ mode + pose features (wrist angles, elbow angle, configuration distance to the
   LargeVibrations region) + speed + (1 | run). Report which terms matter.
4. Same-input comparison: V9b LargeVibrations position vs trajectory, aligned by stylus, window by window.

## Pseudocode

```
for run in runs_with_joint_states(V5..V9b):
    js = bag(run, /joint_states); cmd = bag(run, command_topic)
    for w in windows(0.25 s, 50 %):
        buzz[w, joint] = rms(bandpass(current[w, joint], 80..249))
        cmdhf[w]       = rms(bandpass(cmd_velocity[w], 80..249))
        pose[w], speed[w], mode[w], moving[w] = ...
table = concat(windows)
fit mixed model; same-input paired windows for V9b LargeVibrations
```

## Figures

1. **"Where the arm buzzes"**: pose map. Axes = the two joint angles that separate buzz from no-buzz (chosen
   by the model, likely wrist_2 vs elbow); each window a small dot coloured by buzz energy (sequential, one hue),
   shape by mode. The LargeVibrations and NoBuzzArea paths drawn as lines across it.
2. **Same input, two modes**: spectrogram pair (position above, trajectory below, same colour scale, 0–250 Hz)
   for V9b LargeVibrations, with the input speed trace on top. The reader sees whether the buzz switches on at
   the same moments in both modes.
3. **Command vs response scatter**: x = command 80–249 Hz content, y = current 80–249 Hz, per window,
   coloured by mode. Points above the diagonal band = the arm adds energy the command did not contain.

## Outputs

The fixed metric definition; buzz by mode in the same pose with CI; the pose terms that predict it; a clear
statement of whether position output buzzes (and how much relative to the floor).

## Pitfalls

- Current depends on load and gravity torque; compare band power, not mean current.
- Effort units are the driver's current; check the driver docs and state it.
- If the evidence stays ambiguous, the report says "unresolved" and this folder supplies the limitation text.

## Status (2026-11-13)

Code: `analysis.py` in this folder (run from `thesis/data_analysis` with ROS and the workspace sourced). Results, figures and the check of every prior claim are written by the code to [`RESULTS.md`](RESULTS.md) and `figures/`; no number in RESULTS.md is typed by hand. Read RESULTS.md for what was done, what was excluded and why, and what is still open.
