# R04 Pose-offset test: does the latch offset follow the camera or the pose?

**Report:** 4.3.3 (the direct evidence for "measurement chain, not arm"). **Question:** the arm was cycled in
joint space between the SlowChecking latch pose and its working pose, with the camera in two placements. Does
the NDI-vs-robot offset between the two poses (tip *and shaft axis*) change with the camera? If it does, the
offset is in the measurement chain; if it follows the pose regardless of camera, it is the arm or the seat.

**Prior claims to test:** tip distance between poses 76.32 mm (robot) vs 76.46 / 76.47 mm (NDI, two cameras);
holds repeat to ~0.02 mm; Camera 2 is 185 mm and 15.9° from Camera 1; axis analysis never done.

## Data

`data/recordings/V9b/PoseOffset_Camera1`, `data/recordings/V9b/PoseOffset_Camera2` (6 cycles each, 4 s holds; trajectory player:
`TRAJECTORY.yaml` joint-space samples), each with `NDI_PATH.csv` and a bag (`/joint_states`). Exclude by data,
not by name, but the `_FAILED_catchup` and `_INCOMPLETE_` folders should fail the validity check (motion not
completed) and be reported as such.

## Extraction (the clever bits)

- **Holds from the joints:** a hold = joint speeds below a threshold for ≥ 3 s (from `/joint_states`), labelled
  pose A or B by nearest commanded configuration. Take the median NDI tip, axis and full pose per hold
  (trim 0.5 s at each end, check the trim does not change the result, as in folder T01).
- **Compare displacement, not position.** The camera frame differs between placements, so compare the *rigid
  motion between A and B*: in the NDI it is T_A⁻¹T_B (wand pose at A to B); from the robot, FK(B)·FK(A)⁻¹
  through the hand–eye tool. Both are frame-independent. Their difference, ΔT, is the chain's error on that
  motion, a 6-DOF quantity: translation of the tip, rotation of the axis.
- **Map to RCM error:** apply ΔT to the latched shaft line to get the RCM error this offset alone would create
  at the working pose, so the result is in the units of 4.3.3.
- **Cycle-to-cycle spread** of ΔT is the floor; the camera effect must exceed it.

## Method

1. Per hold: median pose (NDI), FK pose (calibrated), validity checks (visibility, completed motion).
2. Per cycle: ΔT for the A→B motion; tip-distance difference, axis-angle difference, RCM-error equivalent.
3. Camera 1 vs Camera 2: difference of mean ΔT with bootstrap CI over cycles.

## Pseudocode

```
holds = still(joint_speed < thr, ≥ 3 s) → label A/B
for hold: T_ndi = median pose; T_fk = FK(q)·X_tool
for cycle: dN = T_ndi[A]^-1 T_ndi[B]; dR = T_fk[A]^-1 T_fk[B]; Δ = dR^-1 dN
           tip_err = |Δ.t|; axis_err = angle(Δ.R); rcm_equiv = dist(Δ·shaft_line, latch_point)
camera effect = mean_C2(Δ) − mean_C1(Δ), bootstrap over cycles
```

## Figures

1. **"Does the offset move with the camera?"**: two panels (tip offset vector, axis offset), each showing the
   six cycles per camera as small arrows from a common origin in the tool frame, with each camera's mean arrow
   bold and its cycle-spread ellipse. Arrows that rotate or resize between cameras = chain; arrows that
   coincide = arm/seat.
2. **RCM-equivalent error** per camera as dot + CI, against the SlowChecking measured error on each day as
   reference lines. It answers whether this offset is the size of what 4.3.2 measured.

## Outputs

Tip and axis offset per camera with CI, the camera effect, the RCM-equivalent, and a one-paragraph verdict for
4.3.3 (including "inconclusive" if the effect is inside the floor).

## Status (2026-11-13)

Code: `analysis.py` in this folder (run from `thesis/results` with ROS and the workspace sourced, or use `run_all.sh`). Results, figures and the check of every prior claim are written by the code to [`RESULTS.md`](RESULTS.md) and `figures/`; no number in RESULTS.md is typed by hand. Read RESULTS.md for what was done, what was excluded and why, and what is still open.
