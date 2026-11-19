# 02 Closed-loop latency model: what the filter really costs, and what would meet 50 ms

**Report:** 4.1.3, 4.5.1. **Question:** the command filter sits inside the proportional loop. How much lag does
it add there, can a simple model of the loop predict the measured lags across every configuration the project
ran, and what cutoff (or predictor) would meet both 50 ms and ≥ 30 % tremor-band reduction?

**Prior claims to test:** 46–48 ms unfiltered vs 52–62 ms filtered at 1:5 (V9b); arm ≈ 16 ms dead time + 15 ms
first-order lag (emulator fit); open-loop Butterworth delay ≈ 0.225 / f_c.

## Data: the version history as a natural experiment

Every run's `META.yaml` records the settings it ran with. Build one table from all of them:

| Field | From |
|---|---|
| output mode, `TOUCH_COMMAND_FILTER` (f_c), Kalman on/off, `TOUCH_POSITION_GAIN` (K_p), `TOUCH_FEEDFORWARD` (k_ff), `SERVO_POSITION_LEAD`, publish rates, scale | `META.yaml settings` |
| measured translation and rotation lag | recomputed here (same estimator as folder 05) from `UR_PATH.csv` vs `TOUCH_PATH.csv` |
| arm response | `/joint_states` vs the command topic |

Candidate runs: `logs/V5/*`, `V7/*`, `V8/*`, `V9/*`, `V9b/NoTool/*`, `V9b/*` RCM runs (real arm), and
`logs/SIM_latency/*` (17 emulator variants: lookahead, feedforward, 250 Hz, FK pose, velocity variants), which
add points the arm never ran. Keep real and sim points apart in the fit, and use the sim points to test
the model, not to fit it.

## Extraction

- **Identify the arm block from the arm itself,** not from the emulator: on every real run, fit
  `qd_measured(t) ≈ G_arm * qd_commanded(t)` per joint with G_arm = e^{-sT_d} / (τs + 1) by least squares on
  segments with motion, giving T_d and τ with spread across runs, modes and joints. This replaces the inherited
  16 ms / 15 ms with measured values and their uncertainty.
- **Identify the operator input spectrum** from `TOUCH_PATH.csv` (target speed PSD): the "lag" metric depends
  on what frequencies the input contains, so the model must be driven by the real input, not a sine.
- **Lag the same way everywhere:** the best-shift estimator of folder 05, applied to model output and to
  measurements alike, so model and data are compared on one definition.

## Model

Per Cartesian axis, the teleop loop (from `ur16e_touch_teleop.py`, check against the code at each run's
`code_version`):

```
v_cmd = F(s) · [ k_ff · s·p_target + K_p · (p_target − p_robot) ]       F = 2nd-order Butterworth at f_c (or 1)
p_robot = G_servo(s) · G_arm(s) · v_cmd / s                             G_servo: Servo + limiter + mode (lead for position)
```

Closed-loop transfer T(s) = p_robot / p_target. Simulate it in discrete time at the run's loop rate, driven by
that run's own recorded `p_target`, and compute the lag with the folder-05 estimator. Free parameters: only the
G_servo delay per mode (everything else is set from META or identified above). Fit it on V9b; predict the rest.

## Pseudocode

```
table = [settings(run) | measured_lag(run) for run in real_runs + sim_runs]
arm = fit_fopdt(qd_cmd, qd_meas)  for real runs → T_d, tau (median, IQR)
def predict(run, servo_delay):
    p = simulate_loop(run.target, f_c=run.f_c, kff, Kp, lead, arm, servo_delay, rate)
    return best_lag(p, run.target)
servo_delay[mode] = argmin Σ_V9b (predict − measured)²
residuals = predict(other runs) − measured                 # held-out check
open_loop_cost(f_c)  = 0.225/f_c
closed_loop_cost(f_c) = predict(f_c) − predict(no filter)   # same input
design map: for f_c in 2..20 Hz, Kp in range, lead in 0..40 ms:
    lag(f_c, Kp, lead), tremor_reduction(f_c) from folder 01's input spectrum
```

## Statistics

Fit with bootstrap over runs (parameters with 95 % CI). Report held-out residuals (rms, max) by version. The
model earns its place only if held-out error is below the replay repeatability (folder 12); say so either way.

## Figures

1. **"Filter cost: open loop vs inside the loop"**: lag added by the filter vs f_c. Dashed curve = 0.225/f_c,
   solid curve = model, measured points with replay ranges. The gap between the curves is the finding.
2. **Design map (signature for 4.5.1)**: x = f_c, y = K_p (or position lead); colour = predicted end-to-end lag;
   contour at 50 ms; hatched region where tremor-band reduction < 30 %. The feasible region (if any) is the
   unhatched area under the 50 ms contour. Mark the shipped setting and the measured points.
3. **Held-out check**: predicted vs measured lag, one mark per run, coloured by version, with the identity line
   and the ±repeatability band.

## Outputs

Fitted T_d, τ, servo delays with CI; closed-loop filter cost at 5 Hz with CI; the feasible set (or the proof it is
empty); held-out residual table.

## Pitfalls

- Settings changed between code versions under the same name: read the code at `code_version` (git show) for
  the loop structure of each version, not only META.
- Rotation and translation lag differ (rotation is larger at 1:1); model both or say which.
- The 1:5 vs 1:1 rotation exponent (`TOUCH_SCALE_ROTATION_EXPONENT`) changes the rotation input.

## Status (2026-11-13)

Code: `analysis.py` in this folder (run from `thesis/data_analysis` with ROS and the workspace sourced). Results, figures and the check of every prior claim are written by the code to [`RESULTS.md`](RESULTS.md) and `figures/`; no number in RESULTS.md is typed by hand. Read RESULTS.md for what was done, what was excluded and why, and what is still open.
