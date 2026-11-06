# E0: calibrated-kinematics prediction from existing bags (2026-11-03)

Offline analysis, no lab time. Plan: `claude-notes/07_remaining_experiments.md` §1.4. Everything here
reads `logs/` and writes only into this folder (kept apart from the experiment records).

```
source /opt/ros/jazzy/setup.bash && cd ~/ros2_ws/src/MacroMicroSurgicalRobot
python3 thesis/planning/analysis/E0_calibrated_kinematics/e0_offline.py    # ~1 min, results/e0_results.yaml
python3 thesis/planning/analysis/E0_calibrated_kinematics/e0_figures.py    # results/*.png
```

## Short answer

**The calibrated kinematics are not the main cause of the 10-26 NDI RCM error.** Run through the arm's own
calibration, the 10-26 replays would have measured about **0.25 mm less** (0.1-0.35 mm, depending on the set),
not the 1-3 mm that R7.9 assumed. SlowChecking would still be **2.7 mm**, not "towards RCMOnly's level".

What sets SlowChecking's 2.9 mm is a **pose-dependent offset of about 2.5 mm between the start/approach region
(where the NDI latches its RCM point) and the working region**. It is present under both models, so it is not
kinematic. It halves on 11-03 (1.3 mm) for almost identical joint paths, which points at the tool, wand seat or
camera placement, not at the arm.

The 10-26 → 11-03 change in SlowChecking (2.87 → 1.30 mm) splits into roughly **¼ kinematics** (0.25-0.45 mm)
and **¾ everything else that changed** (tool/seat, camera placement: 1.0-1.2 mm).

## Method

- Joint angles: `/joint_states` from each bag (500 Hz RTDE), mapped to recorder time through the controller TCP
  rows of `UR_PATH.csv` (same RTDE packet). They go through **two FK models**: nominal (ur_description UR16e)
  and calibrated (`config/calibrations/ur16e_kinematics_2026-11-03.yaml`). FK is the code of
  `scripts/kinematics_check.py`.
- **Tool (tool0 → wand, X) per model, from that day's independent wrist-jog hand-eye recording**
  (`ndi_tracking/calibrations/raw/robot_tcp_20261026_162504.npz`, `..._20261103_094111.npz`). Those files
  hold TF tool0, not joints, so the joints are recovered by IK through the model TF used that day (exact to
  <1e-3 mm), then pushed through both models.
  *First attempt fitted X on the replays themselves: rejected, because on pivoting motion X absorbs the very
  error under test (the tip moved 5 mm and the nominal prediction no longer matched the measurement).*
- **Camera (base → camera, Y) and NDI delay fitted per replay**, with X fixed. A constant camera error cannot
  change an RCM error, because the NDI latches its point in camera frame.
- In the RCM segment, per sample: the **offset** between the measured shaft line (NDI through Y) and the model
  shaft line (FK through X), perpendicular to the shaft, at the model's pivot. If a controller held the model
  lines exactly through its point, the NDI would see `|offset − offset at latch|` (perpendicular to the shaft).
  That is the **predicted** RCM error, from that model alone.
  - **best**: the same, about the best constant offset instead of the latched one (latch effect removed).
  - **latch shift**: how far the offset at the latch sits from that best constant.

**Validation.** For the model each run actually used, the prediction must reproduce the measured NDI error,
because the controller held that model's lines (its own `rcm_error` is 0.05-0.23 mm). It does:

| Set | 10-26 measured / predicted (nominal) | 11-03 measured / predicted (calibrated) |
|---|---|---|
| RCMOnly | 0.43 / 0.52 | 0.51 / 0.79 |
| WholeRun | 0.90 / 0.96 | 1.26 / 1.40 |
| FastChecking | 1.70 / 1.37 | 1.27 / 0.86 |
| SlowChecking | 2.87 / 2.96 | 1.30 / 1.51 |

The prediction is within about 0.1-0.4 mm. FastChecking is under-predicted, probably because of timing at
30-50 °/s (the error depends on a ~10-20 ms NDI delay fitted per run). The recomputed measured values equal
NDI_META's to 0.01 mm.

## Results (median of replays; mm rms)

![budget](results/e0_budget.png)

| Set (swing 10-26) | 10-26 measured | 10-26 predicted: nominal → **calibrated** | latch shift (cal.) | best (cal.) | 11-03 measured | 11-03 predicted: nominal → calibrated |
|---|---|---|---|---|---|---|
| RCMOnly (40°) | 0.43 | 0.52 → **0.34** | 0.10 | 0.31 | 0.51 | 1.20 → 0.79 |
| WholeRun (70°) | 0.90 | 0.96 → **0.61** | 0.21 | 0.50 | 1.26 | 1.28 → 1.40 |
| FastChecking (47°) | 1.70 | 1.37 → **1.38** | 1.11 | 0.84 | 1.27 | 1.49 → 0.86 |
| SlowChecking (16°) | 2.87 | 2.96 → **2.71** | 2.49 | 1.09 | 1.30 | 1.95 → 1.51 |

11-03 FastChecking excludes Replay1 (camera moved mid-run, see Data hygiene).

1. **The calibrated model fits the arm better, but only by a little.** The whole-run hand-eye residual drops in
   all 8 day×set cells (10-26: 0.47-1.44 vs 0.48-1.54 mm; 11-03: 0.63-1.15 vs 0.81-1.44 mm). The wrist-jog fits
   are identical (0.78/0.79 and 0.62/0.62 mm), as expected, because jogging the wrist barely exercises where
   the models differ. This independently confirms the 11-03 calibration file, with an effect of 0.1-0.3 mm.
2. **On 10-26, threshold (iii) (≤ 1 mm) would still have failed with calibrated kinematics** for FastChecking
   (1.38) and SlowChecking (2.71). RCMOnly and WholeRun pass with either model.
3. **SlowChecking is a latch effect.** About the best point it is 1.1 mm; the latched point is 2.5 mm off. The
   offset is ~2.5 mm during the whole approach while the arm is moving (3-12 s), so it is **not settling or
   hysteresis**. It stays through the latch and the still period, then falls to ~0.3 mm over 16-23 s as the arm
   moves into the working region (figure below). It is reversible and tracks the pose (wrist_3 alone explains
   R² = 0.86, but that joint just follows trajectory progress: the jog residual has R² ≈ 0 against wrist_3 over
   112°). It is not a rigid tool error either: the offset varies as much in the tool0 frame as in base.
4. **What it might be** (untested): NDI accuracy in that part of the volume, or wand orientation accuracy (the
   pivot is 160-250 mm from the markers, so 0.3° is about 1 mm); compliance; the wand seat. On 11-03 the joint
   paths were nearly identical but the offset halved, while the camera was ~90 mm elsewhere and the
   tool/seat had changed. That argues against arm compliance, which would repeat.
5. **Pivot offsets (R7.9's 1.33 vs 3.7 mm) don't collapse under the calibrated model.** NDI-vs-model pivot
   distance, 10-26: RCMOnly 0.16 → 0.43, SlowChecking 2.65 → 2.76 mm.
6. **On 11-03 the calibration did help.** The counterfactual of running the 11-03 replays on nominal kinematics
   would have been worse in 3 of 4 sets: RCMOnly 1.20 vs 0.79, FastChecking 1.49 vs 0.86, SlowChecking 1.95 vs
   1.51. The kinematic part depends on where the tool and RCM point sit, so it isn't a fixed number per arm.

![latch](results/e0_latch_offset.png)

## Data hygiene found on the way

- **The NDI camera moved twice.** Per-run camera fits agree to 0.1-1.5 mm at the RCM points across all
  trajectories of a day, except FastChecking_Replay1 on both days. On 10-26 it sits 7 mm off: the camera
  moved between Replay1 and Replay2, and the jog's own camera pose fits Replay1 to 1.4 mm but Replay2 not at
  all (7 mm). Between runs this is harmless, because each run re-latches in camera frame.
- **11-03 FastChecking_Replay1 (the 5.08 mm outlier) is a measurement artifact.** At t ≈ 28 s, with the joints
  smooth, the robot-vs-NDI residual steps from ~2 to 13 mm and stays there. The camera (or the wand) moved
  during the RCM segment. Later runs fit the jog tool with a camera ~12 mm away, which points at the camera.
  **Exclude it from the 11-03 statistics** with this reason.

  ![camera](results/e0_camera_move.png)
- The fitted NDI delay is +5 to +20 ms (10-15 ms typical), consistent with the 10 ms of the hand-eye files.

## What this changes in the plan

- **R7.9 / C7 wording:** "the misses are explained by factory kinematics" is not supported. Factory kinematics
  account for ~0.1-0.4 mm. The rest is a pose-dependent, non-kinematic measurement-chain or tool error, and the
  **latch-at-start protocol** amplifies it (the RCM point is taken in a pose region the trajectory then leaves).
- **RD1 expectation** (calibrated rerun of the 10-26 set *with the same tool*): SlowChecking ≈ 2.7 mm,
  FastChecking ≈ 1.4, WholeRun ≈ 0.6, RCMOnly ≈ 0.35. If RD1 instead shows ~1.3 mm everywhere, as 11-03 did,
  then the 11-03 improvement came from the tool, seat or camera.
- **F7.6 error budget** gets a different shape. Per set: controller (0.05-0.23) | kinematic model (this table,
  nominal − calibrated predicted) | pose-dependent non-kinematic (best, calibrated) | latch (latch shift).
- **Cheap lab test (~10 min, settles point 4):** with the wand still in the holder, move in joint space between
  the SlowChecking start pose and a working pose, several times, pausing 3 s at each. Do it twice, with the
  camera in two places. If the NDI-vs-calibrated-FK offset follows the pose and changes with camera placement,
  it is NDI or wand accuracy. If it follows the pose regardless of the camera, it is arm or seat.
- **Protocol fix to consider:** latch the RCM point after the arm is in the working region, or report the
  error about the fitted pivot alongside the latched one (NDI_META already has `error_rms_about_fitted_pivot_mm`).

## Caveats

- One wand (8700340) and one tip and axis calibration (`8700340.json`, pivot rms 0.93 mm). Its own error is
  rigid in the wand frame, so the fit absorbs it, but any view-dependent part is not separated here.
- The jog X for the calibrated model on 10-26 was fitted from joints recovered through the nominal model. That
  is exact for the joints, because nominal FK is what TF published.
- Per-run Y absorbs constant errors only, and those cannot affect an RCM error. It could absorb a small part of
  a pose-dependent error, so treat the predictions as slight underestimates.
- "Predicted" assumes the controller holds the model lines perfectly. Add its own 0.05-0.23 mm.

## Files

- `e0_offline.py`: the analysis. `results/e0_results.yaml` holds per-run and per-model numbers, the hand-eye
  fits and the camera spread. `results/e0_offsets.npy` holds the offset time series.
- `e0_figures.py`: the figures in `results/`.
