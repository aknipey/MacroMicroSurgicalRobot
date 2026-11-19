# R01 The depth-invariant sideways law

**Report:** 4.3.1, F7.2 (signature figure). **Question:** compared with a fixed tip-space scale and the V6 fade,
does the law keep flange motion per millimetre of stylus constant at every insertion depth, and what does each
law do to pivot rate and flange acceleration near the port, offline and on the arm?

**Prior claims to test:** fixed scale gives flange/tip ≈ 30× at 10 mm, 9× at 30 mm, 1.4× at 130 mm; the law gives
0.50 mm/mm at every depth (k = 0.5), pivot 0.09–0.16°/mm vs 5.7°/mm peak for the V6 fade; continuity fix cut the
largest flange step from 11 to 0.03 mm.

## Data

- **The code itself, offline:** `macro_micro_robot/ur16e_common.py` (`lever`, `pivot_slope`, `target_shaft`,
  `twist`) at the commits that ran V6, V7 and V9b (`git show <code_version>:...`), with each run's tool geometry
  from META (`tip_tool0_mm`, `direction_tool0`, `shaft_length_mm`).
- **The arm:** `data/recordings/V6/{BasicInRCM,ScalingInRCM,ExitingRCM}` (fade) and the V7–V9b RCM runs (law); topics
  `/…/insertion_depth`, `/…/rcm_point`, `/…/target_pose`, `/joint_states`.

## Extraction (the clever bits)

- **Drive the real functions, not a re-derivation.** Import `pivot_slope` and `target_shaft` from each commit
  and sweep: depth 0–150 mm × sideways stylus offset, giving flange displacement, pivot angle and their
  derivatives. The figure is then the shipped code's behaviour, exactly.
- **Replay recorded stylus motion through each law offline:** take V7 RCM inputs (TOUCH_PATH + depth) and run
  them through fixed scale, V6 fade, smootherstep, cubic fade and the law, to get flange speed and acceleration
  for the *same* hand motion. That is the fair comparison the real arm could not give (each law ran on
  different days with different inputs).
- **On-arm check:** for V6 and V7+ runs, compute the flange speed per stylus speed vs depth from the logs and
  overlay it on the offline curves.

## Pseudocode

```
for law in [fixed, v6_fade, smootherstep, cubic_fade, law(k=0.5)] (from git at each commit):
    for depth in 0..150 mm, s in small sideways offsets:
        axis, d = target_shaft(tip_target(depth, s), rcm, axis0, max_depth, law.pivot_slope)
        flange = tool0_from(axis, d, tool_geometry)
    gain[law, depth] = d|flange|/ds; pivot_rate[law, depth] = dθ/ds
    for rec in V7 RCM inputs: flange_path = simulate(rec, law) → speed, accel p95, max step at depth 0
arm overlay: V6 (fade), V7-V9b (law) measured flange speed / stylus speed vs depth
```

## Figures

1. **Signature (F7.2), "Same feel at every depth"**: two aligned panels vs insertion depth (0–150 mm, the port
   at 0 marked): top = flange mm per stylus mm (log y), bottom = pivot °/mm. One line per law; the law is a
   flat line at 0.5 on top, fixed scale climbs to ~30× near the port. Measured on-arm points overlaid as dots
   in the same colours. A small schematic in the corner (shaft through the port, lever r(d) and depth d labelled)
   so the reader sees why the lever matters.
2. **Same hand, five laws**: flange acceleration over time for one recorded RCM input passed through each law
   (small multiples, shared y), with depth as a grey band underneath. The kink of the fade and the calm of the
   law are visible together.
3. **Zero crossing**: flange position vs target depth through 0 for the branched and continuous versions
   (inset), showing the 11 mm step disappear.

## Outputs

Gain and pivot-rate tables at 10/30/130 mm per law; p95 flange acceleration per law for the shared inputs; on-arm
agreement.

## Status (2026-11-13)

Code: `analysis.py` in this folder (run from `thesis/results` with ROS and the workspace sourced, or use `run_all.sh`). Results, figures and the check of every prior claim are written by the code to [`RESULTS.md`](RESULTS.md) and `figures/`; no number in RESULTS.md is typed by hand. Read RESULTS.md for what was done, what was excluded and why, and what is still open.
