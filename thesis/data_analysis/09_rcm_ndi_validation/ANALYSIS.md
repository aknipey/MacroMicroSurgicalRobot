# 09 RCM error measured independently with the NDI tracker

**Report:** 4.3.2, Table 4.2, strip figure. **Question:** where does the tracked shaft actually pass relative to
the RCM point, for each recorded trajectory, output mode, kinematic model and day, and how does that compare with
the controller's own error?

**Prior claims to test:** controller 0.05–0.23 mm; NDI RCMOnly 0.42–0.50 mm in every condition, other sets
0.86–2.92 mm depending on day; replays repeat to ~0.1 mm.

## Data

| Condition | Folder | Date |
|---|---|---|
| velocity, nominal | `logs/V7/*SimpleHolder_Replay*` | 10-26 |
| velocity, calibrated | `logs/V8/CalibratedKinematics/*_Replay*` | 11-03 |
| position / trajectory, calibrated | `logs/V9/{PositionMode,TrajectoryMode}/*_Replay*` | 11-03 |
| position, velocity filtered/unfiltered | `logs/V9b/{PositionBridge,VelocityOptimised,VelocityUnfiltered}/*_Replay*` | 11-06 |

Originals (no `_Replay`) kept apart. `*_WRONG_SCALE_1to1` folders: confirm the scale from META and exclude
by that check. Per run: `NDI_PATH.csv` (60 Hz, camera frame), the bag's `/…/rcm_error`, `/…/rcm_point`,
`/…/rcm_active`, and META (kinematics, tool, camera day).

## Extraction (the clever bits)

- **Recompute the error from geometry**, not from `NDI_META.yaml`: per NDI sample, the shaft line from
  `tip_*` and `axis_*`, its distance to the latched point (`rcm_*` columns), in the RCM segment (`rcm_active`).
  Then check the result equals `error_rms_mm` to 0.01 mm. This also gives per-sample errors for distributions.
- **Crossing points:** intersect each sample's shaft with the plane through the latched point normal to the mean
  shaft; the 2D cloud separates a constant offset (bias) from wobble (spread). rms² = bias² + spread² exactly.
- **Error about the fitted pivot:** the least-squares point closest to all shaft lines in the segment, the
  best fixed point the shaft actually pivoted about.
- **Camera-moved detector:** fit the camera pose per run from robot FK vs NDI tip (hand–eye tool from the day's
  calibration file); a step in the residual mid-run flags a moved camera (how the 11-03 FastChecking Replay 1
  outlier must be found).
- **Controller error from the bag** on the same time base (`rcm_error` topic), so both errors are per sample
  and can be plotted together.

## Method

Per run: NDI rms, p95, max, % within 1 / 2 mm, bias, spread, fitted-pivot rms, swing, controller rms. Per
condition × set: replay range. Exclusions printed with reasons.

## Pseudocode

```
for run in rcm_runs:
    ndi = NDI_PATH; seg = rcm_active
    e = dist(line(tip, axis), rcm_point)[seg]; assert |rms(e) - NDI_META.error_rms_mm| < 0.01
    cross = intersect(line, plane(rcm_point, mean_axis)) → bias, spread
    pivot* = argmin Σ dist(line_i, x)² → e_fit
    cam_resid = |T_cam·tip_ndi − FK(q)·tool| over time → step? → exclude
    e_ctrl = bag(rcm_error)[seg]
```

## Figures

1. **Strip (T4.2 companion), "Only RCMOnly is under 1 mm every time"**: x = trajectory set, dots = replays,
   colour/marker = condition, 1 mm line, controller error as a grey band at the bottom of every column (the
   "what the robot thinks" floor). The gap between grey band and dots is the paper's point.
2. **Crossing-point targets**: small multiples (set × day), each a 2D scatter of shaft crossings around the
   latched point (origin), 1 mm circle, colour = time in the segment. A constant offset shows as a cloud away
   from the centre, wobble as a spread cloud. One glance distinguishes the two failure types.
3. **ECDF**: share of RCM time within x mm, one line per condition, faceted by set; 1 mm reference.

## Outputs

Table 4.2 (rebuilt), per-sample error files in `cache/`, exclusion list, crossing and pivot results for folder 10.

## Status (2026-11-13)

Code: `analysis.py` in this folder (run from `thesis/data_analysis` with ROS and the workspace sourced). Results, figures and the check of every prior claim are written by the code to [`RESULTS.md`](RESULTS.md) and `figures/`; no number in RESULTS.md is typed by hand. Read RESULTS.md for what was done, what was excluded and why, and what is still open.
