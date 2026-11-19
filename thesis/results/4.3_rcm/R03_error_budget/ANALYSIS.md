# R03 RCM error budget and variance decomposition

**Report:** 4.3.3, F7.6. **Question:** how much of the NDI-measured RCM error comes from the controller, the
kinematic model, the latch pose, the measurement chain (camera placement, tool seat, tracker), and the output
mode, and which factors explain the variance across runs?

**Prior claims to test:** kinematics explain only 0.1–0.35 mm (E0); a ~2.5 mm latch-pose offset dominates
SlowChecking; trajectory and camera dominate, mode is negligible.

## Data

Folder R02's per-run outputs (all RCM replays, three days), plus:
- `/joint_states` from each bag and both kinematic models: nominal (ur_description) and calibrated
  (`config/calibrations/ur16e_kinematics_2026-11-03.yaml`, hash in META);
- the hand–eye recordings in `src/ndi_tracking/calibrations/raw/` named in each day's calibration file;
- `data/recordings/V9b/NDIStaticFloor` (arm still, 30 s): the tracker's per-sample noise on the shaft line. Its
  `error_rms_mm` (0.73) is distance to an old latched point, a constant offset, not noise; use the scatter.

## Extraction (the clever bits)

- **Counterfactual kinematics:** push each run's joint angles through *both* models with that day's hand–eye
  tool; the model shaft lines under each give the RCM error each model alone would produce (the controller held
  its model's line). The difference is the kinematic term, per run, per sample. Validate: for the model a run
  actually used, the prediction must reproduce the measured error (report how closely).
- **Latch term:** error about the latched point minus error about the fitted pivot (from 09), in quadrature.
- **Static floor on the RCM metric itself:** pass the static run's shaft lines through the same error
  computation about their own mean → the RCM error a perfectly still arm would show.
- **Camera/day as a factor without guessing:** use the per-run fitted camera pose (R02) as a continuous covariate
  (distance and angle from a reference placement), not just a day label.

## Method

1. Per run, a stacked budget: controller (bag `rcm_error`) → + kinematic model (counterfactual) → + latch offset
   → + residual (measurement chain), each in quadrature where independent; check the sum against the measured
   rms.
2. Variance decomposition over runs: log(rms) ~ set + mode + day (or camera pose) + set:day, type-II sums of
   squares, partial η² per factor with bootstrap CI. On 11-03 the design is balanced (4 sets × 3 modes × 3
   replays); 11-06 adds the cross-day bridge (position and velocity on RCMOnly and SlowChecking both days).
3. Replay variance (residual) = the floor; anything smaller than it is not interpretable.

## Pseudocode

```
for run in rcm_runs:
    for model in [nominal, calibrated]:
        line_model = FK_model(q) ∘ X_handeye(day, model)
        pred[model] = dist(shift_to_ndi(line_model), latched point)    # validated on run.model
    kin = rms(pred[nominal]) vs rms(pred[calibrated])
    latch = sqrt(rms_latched² − rms_fitted²)
    budget[run] = [ctrl, kin, latch, residual]
floor = rcm_error(static run about its own mean)
anova: log(rms) ~ set * day + mode ; bootstrap η²
```

## Figures

1. **"Where the millimetre goes" (F7.6)**: one horizontal stacked bar per trajectory set (median run), segments
   controller → kinematics → latch → measurement chain, a 1 mm line, the NDI static floor as a hatched sliver at
   the start. Ordered by total. It reads as an error budget a reviewer recognises.
2. **Variance shares**: dot-and-interval plot of partial η² per factor (set, day/camera, mode, interaction,
   replay), so "mode is negligible" is a visible near-zero dot with its CI.
3. **Counterfactual check**: measured vs predicted rms per run, for the model actually used (validation) and the
   other model (counterfactual), with the identity line.

## Outputs

Budget table per set, η² per factor with CI, the validation residuals, the static-floor RCM noise.

## Pitfalls

Quadrature assumes independence; report the arithmetic version too if they disagree materially. The
counterfactual inherits hand–eye error; state the hand–eye residual alongside.

## Status (2026-11-13)

Code: `analysis.py` in this folder (run from `thesis/results` with ROS and the workspace sourced, or use `run_all.sh`). Results, figures and the check of every prior claim are written by the code to [`RESULTS.md`](RESULTS.md) and `figures/`; no number in RESULTS.md is typed by hand. Read RESULTS.md for what was done, what was excluded and why, and what is still open.
