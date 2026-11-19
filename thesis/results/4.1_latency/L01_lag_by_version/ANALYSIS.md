# L01 Lag by version: the chain of diagnoses in one figure

**Report:** opener of 4.1 (F5.1), Appendix A. **Question:** how did lag, tracking and faults change from V1 to
V9b, and which change caused each step?

**Prior claims to test:** V1 640–760 ms translation / > 1.5 s rotation and ~10 % of commanded speed; V2 128–158 ms
angle lag; V3 ~140 ms at every scale; V5 62/58 ms; V9b 66–78 ms (position, 1:1).

## Data: one input through the whole history

`REPLAY.yaml` shows the same recorded inputs were replayed across versions:

| Input (original) | Replayed in |
|---|---|
| `V3/LongBenchmark` (trajectory output) | `V4/LongBenchmark_failed` (velocity, faulted), `V5/LongBenchmark`, `V5/LongBenchmarkTuned`, `V9b/NoTool/*/LongBenchmark_Replay*` (all modes) |
| `V3/ShortBenchmark` | `V4/ShortBenchmark_failed`, `V5/ShortBenchmark(Tuned)`, `V9b/NoTool/*/ShortBenchmark_Replay*` |
| `V3/Scaled5_Slow` | `V9b/NoTool/*/Scaled5_Slow_Replay*` |

V1 and V2 used different inputs (10 cm moves, 90° rotations: `V1/*`, `V2/*`), so they are a separate panel,
compared on the same move types (`V1_Forwards_10CM` vs `V2/IN_10CM` etc.). Confirm each pairing from the files.

## Extraction (the clever bits)

- **Same input = same x-axis.** For the LongBenchmark chain, plot the arm path of every version against *the
  same target time base*: lag shows as horizontal shift, the crawl as a squashed curve, faults as an abrupt end.
- **Faults from the data, not the notes:** a V4 replay "failed" if `/joint_states` stops changing or
  `speed_scaling` drops to 0 before the input ends; record the time and the faulting joint (largest current
  spike in the preceding 100 ms).
- **Achieved speed fraction for V1:** median ratio of arm tool speed to target speed during strokes, which is the
  crawl in one number.
- **What changed between versions** from META `code_version` + settings diff (automated), so each step in the
  figure is labelled by an actual setting change, not by memory.

## Method

1. Per run: translation and rotation lag (folder L08 estimator), lag-removed error, speed fraction, fault time.
2. Settings diff between consecutive versions' META files → one-line label per step.
3. For V1 vs V2: per move type, lag and speed fraction.

## Pseudocode

```
chain = [V3/LongBenchmark, V4/LongBenchmark_failed, V5/LongBenchmark, V5/LongBenchmarkTuned,
         V9b/NoTool/{Trajectory,Velocity*,Position}/LongBenchmark_Replay*]
for run in chain:
    metrics[run] = folderL08.metrics(run); fault[run] = detect_fault(run)
    label[run]   = diff(META.settings(prev), META.settings(run)) + mode
V1/V2 panel: per move type, lag + speed fraction
```

## Figures

1. **"Same hand, five stacks" (F5.1)**: small multiples, one row per version, sharing a time axis: a 4 s excerpt of
   the LongBenchmark target (grey) and the arm (colour) for one axis. The reader *sees* lag shrink and the V4 trace
   stop at its fault (marked ×). Right margin: lag number per row. Far more convincing than a bar chart of lags.
2. **Lag ladder**: one row per version (y), lag on a log x-axis (V1's ~700 ms and V9b's ~70 ms on one scale),
   translation ● and rotation ○, each row annotated with the change that produced it ("trajectory output",
   "velocity output + filter", "500 Hz + limiter", "position output with lead"). The log axis makes each step's
   *ratio* visible.

## Outputs

Per-version metrics table (Appendix A), the two figures, the settings diffs.

## Pitfalls

- V1–V3 ran under the `ur5e_*` names; some V3 runs used UR5e kinematics (check `tcp_offset.check_max_mm`).
- V2 contains mixed-version runs (planning notes say so): verify `code_version` per run, do not trust folders.

## Status (2026-11-13)

Code: `analysis.py` in this folder (run from `thesis/results` with ROS and the workspace sourced, or use `run_all.sh`). Results, figures and the check of every prior claim are written by the code to [`RESULTS.md`](RESULTS.md) and `figures/`; no number in RESULTS.md is typed by hand. Read RESULTS.md for what was done, what was excluded and why, and what is still open.
