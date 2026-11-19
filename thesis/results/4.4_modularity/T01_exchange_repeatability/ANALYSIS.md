# T01 Quick-change coupling: re-seat repeatability and exchange time, per version

**Report:** 4.4.1, scorecard (ii), G3 box. **Question:** for each coupling version (V1, V2, V3), how repeatably
does the tool re-seat, in which directions (seating depth, lateral, tilt, clocking), how long does an exchange
take, and how much of the measured spread belongs to the coupling rather than the measurement chain?

**Prior claims to test** (from the first exchange script `exchange_6dof.py`, deleted 2026-11-19 and in git history, and the draft; reproduce, don't reuse):
pooled n = 60 re-seat ≤ 0.19 mm, rms 0.043 mm; tracker 5 s floor 0.003–0.006 mm; successive seats correlated
(cosine +0.62 to +0.70); independent swap-to-swap part 0.009–0.043 mm; set 3 drifts 36 µm/min; rotation ≤ 0.03°;
median exchange 4.8 s.

## Data

| Version | Folder | Date | Hardware (user, 2026-11-13) |
|---|---|---|---|
| V1 | `data/recordings/Exchange/20261027_095016_QC_V1` | 10-27 09:50 | White 3D-printed prototype coupling (photos `qc_printed_prototype_v1_*`) |
| V2 | `data/recordings/Exchange/20261027_100249_QC_V2` | 10-27 10:02 | Tool side a full disc; balls sit in the **middle** of the radial rod V-grooves (`qc_tool_side_full_disc_v2_mass_193g`) |
| V3 | `data/recordings/Exchange/20261030_115609_QC_V3` | 10-30 11:56 | Minimal 3-lug tool side; balls on the **outer half** of the grooves for a wider base (`qc_tool_side_lightened_3lug_v3_mass_124g`) |

Kinematic-coupling expectation to test: a wider ball base (V3) should reduce *tilt* for the same contact
error (angle ≈ contact error / base radius), and so the tip error at a long tool; the printed V1 should
be worst (polymer contacts).

Each: `NDI_PATH.csv` (60 Hz; tip, shaft axis, marker origin, quaternion, `fit_error`, `visible`), `NDI_EVENTS.csv`,
`NDI_META.yaml` (wand 8700340, pivot calibration). Protocol (`data/recordings/Exchange/PROTOCOL.md`): 30 s still, 20 swaps
(lever open, wand out of view, refit, lever closed, ~5 s seated), 30 s still. Arm and camera assumed still; only
the wand is tracked.

**Confound to state up front:** each version is one session, so a difference between versions is version +
day + camera pose + handling. Version comparisons are descriptive unless the effect is far larger than the
within-session floors.

## Extraction (the clever bits)

- **Segment every exchange into phases from the data**, not just "seated vs not": seated (still rule) →
  unlock (tip moves < 1 mm while still visible: the lever opening) → lift (fast motion) → out of view
  (`visible = 0`) → approach (visible, moving) → seat-and-lock (decelerating to still) → settle (still rule
  satisfied, residual decay). Exchange time then splits into handling time and seating time, and settling
  time is measured, not assumed.
- **Express re-seat in the coupling's own frame.** The wand sits in the holder with its shaft along tool0 z
  (holder `direction_tool0` ≈ [−0.004, 0.045, 0.999] in META of the holder tool), so the tracked shaft axis is the
  coupling axis to within ~3°. Decompose each seat offset into **axial** (seating depth: balls riding
  higher or lower in the grooves) and **radial** (lateral), and each rotation into **tilt** (about axes ⊥ shaft)
  and **clocking** (about the shaft). Kinematic-coupling theory predicts different signatures for preload
  variation (axial), friction hysteresis (radial, direction-dependent) and a ball not fully seated (tilt);
  this frame makes the data speak that language.
- **Seat = robust median of the settled part** (after the measured settling time), with a trim-sensitivity
  check (0–2 s per end) reported, so end-bleed (hand returning to the lever) cannot bias it.
- **Floors from the same recording:**
  1. *Precision floor*: 5 s medians inside each 30 s still period, each end separately (different seats).
  2. *Wander floor*: slope of the measured point in the still periods (µm/min) with nothing touched.
  3. *Marker-fit quality*: `fit_error` per seat as a covariate; a seat with a poor fit is flagged, not dropped.
- **Separate per-swap scatter from slow motion:** successive-difference estimator (rms(Δ)/√2) vs total spread;
  linear drift fit; change-point scan with a permutation test; direction correlation of successive offsets vs a
  shuffled-order null. Only the per-swap part is attributed to the coupling; the slow part is reported as an
  upper bound component.
- **Bumps:** within each seat (both ends trimmed), the largest step between adjacent 0.75 s medians, compared
  with the same statistic inside the still periods.
- **Tip error at a real tool:** apply each seat's measured rigid rotation and translation to the micro tool's
  tip (`config/calibrations/micro_tool_2026-11-06_134732.json`) and the RCM holder tool's tip (from tools.yaml),
  so the result is in "mm at the tip that matters", not at the wand.

## Method (per version, then pooled)

1. Phases and exchange time per swap; settling time distribution.
2. Seat poses → offsets in the coupling frame (axial, radial, tilt, clocking) from the version's median.
3. Floors; spread components (per-swap, slow, drift); direction correlation; change-points; bumps.
4. Tip-equivalent error at the micro and RCM tools.
5. Versions compared on the drift-robust per-swap component: ratio of per-swap rms between versions with
   bootstrap CI (resampling successive differences), Brown–Forsythe test as a secondary check. Bedding-in (first
   3 vs rest) per version.
6. Against threshold (ii): median and max exchange time; max and p95 tip-equivalent error; stated as an upper
   bound.

## Pseudocode

```
for v in [V1, V2, V3]:
    ndi = NDI_PATH(v)
    phases = segment(ndi): seated / unlock / lift / out_of_view / approach / lock / settle
    t_settle = per swap: time from first still sample until |p - final seat| < floor_sample_sd
    seats = [robust_median(pose[after settle]) for each seated]
    z = mean shaft axis; offsets → (axial, radial_x, radial_y), rotation → (tilt_x, tilt_y, clock)
    floors = still-period 5 s medians, still-period wander, fit_error
    fast = rms(diff(seats))/√2; slow = sqrt(total² − fast²); drift = linfit(seats ~ t)
    corr = mean cos(off_k, off_k+1) vs shuffle null; cp = changepoint permutation
    bumps = maxstep(trimmed seats) vs maxstep(still periods)
    tip_err[tool] = |R_seat·tip_tool + t_seat − tip_tool|
compare versions: bootstrap ratio of fast; bedding-in
```

## Figures

1. **"Every exchange, phase by phase"**: a swimlane timeline, one row per swap (60 rows, grouped by version),
   time on x from lever-open, coloured bars for unlock / lift / out-of-view / approach / lock / settle. Medians
   per version as a bold row at the bottom of each group. Shows where the 5 s goes and that seating is a small
   part of it.
2. **"How the coupling re-seats", per version**: a 2D radial-plane scatter (camera-independent coupling frame)
   with axial offset as a small vertical bar on each point. Successive seats joined by faint arrows (makes the
   direction correlation visible), the precision-floor ellipse at the centre, and a 1 mm circle drawn to scale
   at the edge of an inset so the reader sees the cloud is a speck at threshold scale. Three panels, one per
   version, identical axes.
3. **Sequence plot (current figure, rebuilt)**: seat offset vs seat number per version, with the drift line, any
   change-point marked, the bedding-in region shaded and the precision floor band.
4. **Spread components**: per version, a stacked horizontal bar: tracker floor | per-swap (coupling) |
   slow (unattributed), with the tip-equivalent at the micro tool printed at the end. It turns the statistical
   argument of 4.4.1 into one picture.
5. **Settling**: median |tip − final seat| vs time since the still rule fired, per version, with the
   precision floor, showing when a seat is truly settled.

## Outputs

Per version and pooled: exchange time (phases), settling time, seat offsets in the coupling frame, floors,
per-swap / slow / drift components, correlation and change-point tests, bump check, tip-equivalent errors,
version comparison with CI. RESULTS.md lists which prior claims held.

## Pitfalls

- The holder tool's axis is not exactly the coupling axis (~2.6° from tool0 z): state it; it mixes a little
  axial into radial (sin 2.6° ≈ 4.5 %).
- Out-of-view time depends on how far the wand was carried, not on the coupling: keep it out of "seating time".
- n = 20 per version, correlated: no CI on per-version percentiles; CIs only on drift-robust statistics.

## Status (2026-11-13)

Code: `analysis.py` in this folder (run from `thesis/results` with ROS and the workspace sourced, or use `run_all.sh`). Results, figures and the check of every prior claim are written by the code to [`RESULTS.md`](RESULTS.md) and `figures/`; no number in RESULTS.md is typed by hand. Read RESULTS.md for what was done, what was excluded and why, and what is still open.
