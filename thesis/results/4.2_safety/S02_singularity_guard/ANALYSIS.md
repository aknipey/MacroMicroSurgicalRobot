# S02 Singularity guard on the real arm

**Report:** 4.2.1, F6.1. **Question:** driven into a wrist singularity and back with identical input, does the
arm return with the guard on and stay stuck with it off, and what exactly happens in between?

**Prior claims to test:** guard on: peak c 60.3–60.4, return within 0.5–0.6 mm; guard off: c ≈ 204, stuck
171.5–171.7 mm from start, 3 of 3; the mock's guard-off run escaped.

## Data

`data/recordings/V9b/Singularity/{GuardOn,GuardOff}/SingularityInput_Replay{1,2,3}` (real arm), the input
`data/recordings/V9b/SingularityInput` (trajectory-player YAML), and `data/recordings/V9b/SingularityMock` (mock). Bags carry
`/ur16e_touch_teleop/condition_number`, `/servo_node/status`, `/servo_node/delta_twist_cmds`, `/joint_states`,
the teleop target. `V9b/Singularity/METRICS.csv` exists but is a claim to reproduce, not an input.

## Extraction (the clever bits)

- **Recompute c from the joints** with the calibrated kinematics (Jacobian condition number as Servo defines
  it; check the definition in Servo's source) and compare with the published topic. That validates the topic and
  gives c at 500 Hz.
- **Decompose the commanded twist** into the component along ∇c (into the singularity) and the rest, per
  sample. The guard's claim is that it removes only the first; this shows it directly.
- **Servo status as an event track:** decode `/servo_node/status` codes to mark when Servo scales, halts, or
  reports "no warnings".
- **Return gap and travel** from FK of `/joint_states`, the tool0 point relative to its start.

## Method

1. Per replay: c(t) (topic and recomputed), status events, twist components along/against ∇c, tool path, return
   gap, max joint speed.
2. Replay agreement (max difference between replays of the same condition) as the floor.
3. Mock vs real for guard off: same metrics on the mock run.

## Pseudocode

```
for run in GuardOn/* + GuardOff/* + SingularityMock:
    js = bag(/joint_states); c_topic = bag(condition_number); st = bag(/servo_node/status)
    c = cond(J(q)) (Servo's definition); grad = ∇_q c → twist space
    tw = bag(delta_twist_cmds); into = max(0, tw · ĝ); along = tw - into ĝ
    p = FK(q); gap = |p_end - p_start|; travel = max |p - p_start|
compare replays; compare mock
```

## Figures

1. **"Guard off: the arm cannot be brought back" (F6.1)**: two stacked panels sharing time. Top: c(t) on a log
   axis for all six replays (on = one colour, off = another; replays as thin overlapping lines, which shows how
   identical they are), Servo's thresholds as horizontal lines, Servo-halt intervals shaded. Bottom: tool distance
   from start, ending at 0.5 mm (on) vs 172 mm (off), with the final values labelled at the right edge.
2. **Phase portrait**: x = c, y = distance from start, one trajectory per replay, arrows for direction. Guard on
   loops out and back; guard off goes out and stops at c ≈ 200. A compact picture of "stuck".
3. Inset or appendix: the mock's guard-off run on the same axes, escaping, to show what simulation would have
   reported.

## Outputs

Six-run table (peak c, time above thresholds, return gap, travel, max joint speed), mock comparison.

## Pitfalls

One stroke, one singularity type: state it. If recomputed c disagrees with the topic, report both and find out
why before plotting.

## Status (2026-11-13)

Code: `analysis.py` in this folder (run from `thesis/results` with ROS and the workspace sourced, or use `run_all.sh`). Results, figures and the check of every prior claim are written by the code to [`RESULTS.md`](RESULTS.md) and `figures/`; no number in RESULTS.md is typed by hand. Read RESULTS.md for what was done, what was excluded and why, and what is still open.
