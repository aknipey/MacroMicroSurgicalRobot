# L03 Displacement delivery: how much of Servo's command the arm actually covers

**Report:** 4.1.1 (and a `\TODO` there). **Question:** over a run, what fraction of the displacement in
MoveIt Servo's commanded twist does the arm deliver, per output mode? Is the trajectory route's shortfall a
property of the route, and does the position route (which also ends in `servoj`) avoid it?

**Prior claims to test:** trajectory 0.60 (real and emulator, independent of lookahead), velocity 1.04; position
mode not measured.

## Data

`/servo_node/delta_twist_cmds` (TwistStamped, `frame_id` base_link) and the arm's measured motion in every bag
that has both: V1 (position, crawl), V2–V3 (trajectory), V5, V7–V9b (velocity and position), V9b NoTool (all
modes, same input), and `data/recordings/SIM_latency/*` (emulator: lookahead variants B_look10/13/20).

## Extraction (the clever bits)

- **Compare like with like.** The twist is the *input to Servo*, a velocity of tool0 in base_link. Integrate
  it per stroke (clutch engaged: `/phantom/button`) to a commanded displacement path. Get the delivered path from
  forward kinematics of `/joint_states` positions (calibrated or nominal model per META), the same frame and
  point (tool0). Do not use the TCP pose here: it is the same data through the pendant, and FK keeps the
  comparison inside one clock (header stamps).
- **Ratio as a slope, not an end-point.** Plot cumulative |delivered| against cumulative |commanded| arc length
  per stroke. The slope is the delivery ratio, and its stability over the stroke shows whether the shortfall is
  constant (a gain) or grows with speed (a rate limit). End-point ratios are dominated by the last moment of the
  stroke and by the P-loop correction, so avoid them.
- **Separate feedforward from correction.** The commanded twist is k_ff·v_target + K_p·error. The P term
  re-injects whatever the arm missed, so the ratio of *commanded* to *delivered* understates the route's loss.
  Report both: delivered / commanded twist, and delivered / target (the stylus motion the operator asked for).
- **Projection, not magnitude:** use the component of delivered displacement along the commanded direction
  (dot product per sample), so lateral noise cannot inflate "delivered".

## Method

1. Per stroke: integrate twist (trapezoid on header stamps), FK path, align (best lag from folder L08, to stop the
   lag itself counting as shortfall), cumulative arc lengths, robust linear fit (Theil–Sen) of delivered vs
   commanded.
2. Per run: duration-weighted median slope; per mode: across runs and replays.
3. Speed dependence: slope in bins of commanded speed.
4. Emulator: the same on SIM_latency to test "same on real arm and emulator" and "independent of lookahead".

## Pseudocode

```
for run in runs:
    tw = bag(run, /servo_node/delta_twist_cmds)            # base_link, header stamps
    js = bag(run, /joint_states); fk = FK(js.q, kinematics(run))
    for stroke in clutch_strokes(run):
        cmd = cumsum(tw.linear * dt); dlv = fk.p - fk.p[stroke.start]
        dlv = shift(dlv, best_lag)                         # lag is not loss
        s_cmd = cumsum(|d cmd|); s_dlv = cumsum(dot(d dlv, unit(d cmd)))
        slope[stroke] = theil_sen(s_dlv ~ s_cmd)
    ratio[run] = weighted_median(slope)
```

## Figures

1. **"Delivered vs commanded"**: cumulative delivered vs commanded arc length, one thin line per stroke,
   coloured by mode, with reference lines at slope 1.0 and 0.6. Trajectory strokes fan out below 1, velocity and
   position sit on it. One look shows the route effect.
2. **Ratio vs commanded speed** per mode (binned medians with IQR), real arm solid, emulator hollow. Flat =
   gain loss; falling = rate limit.

## Outputs

Delivery ratio per mode (real, emulator), its speed dependence, and the position-mode number the report needs.

## Pitfalls

- Twist frame: confirm `frame_id` on every run (base_link in V9b; check V1–V3).
- Angular part: do the same for rotation (rotation-vector increments), since rotation lag is larger.
- V1 ran on UR5e kinematics in some runs (check `tcp_offset.check_max_mm`); exclude by that check.

## Status (2026-11-13)

Code: `analysis.py` in this folder (run from `thesis/results` with ROS and the workspace sourced, or use `run_all.sh`). Results, figures and the check of every prior claim are written by the code to [`RESULTS.md`](RESULTS.md) and `figures/`; no number in RESULTS.md is typed by hand. Read RESULTS.md for what was done, what was excluded and why, and what is still open.
