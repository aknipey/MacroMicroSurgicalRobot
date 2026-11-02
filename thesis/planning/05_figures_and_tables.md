# 5. Figures and tables to make

Plan the figures before writing: a results chapter is mostly "figure → what it shows → why it matters". Save
vector PDFs into `../latex/figures/<chapter>/` and CSVs for pgfplots into `../latex/data/`. Keep the scripts that
make them in `scripts/thesis_figures/` (create it) so every figure can be rebuilt from `logs/`.

**Style:** one consistent matplotlib style (serif font matching the thesis, 0.8\linewidth ≈ 13 cm wide, 9–10 pt
labels, colour-blind safe palette, the same colour per output mode everywhere: trajectory / velocity / position).

⚠ = the analysis existed only as a scratch script during development and must be re-created.

---

## Signature figures (make these first)

| # | Figure | Data | How |
|---|---|---|---|
| **F5.5** | **Pareto plot: lag vs smoothness** (5–15 Hz jitter or angular jerk p95) for every configuration, real (filled) vs sim (hollow), V1→final annotated | METRICS_SUMMARY.csv in logs/V3, V5, SIM_latency; VERSIONS.md tables | `plot_paths.py` metrics + a small plotting script |
| **F7.2** | **RCM law: tool0-per-mm and pivot-per-mm vs depth**, V6 fade / smootherstep / fade³ / final law | analytic + `RcmTool` | script calling `macro_micro_robot.ur16e_common.RcmTool` directly |
| **T9.1** | **Symptom → obvious suspect → actual cause (interface)** | `01_story_and_novelty.md` §1.2 | table |
| **F7.4 / T7.1** | NDI RCM error per replay set + controller's own error | `logs/V7/*_Replay*/NDI_META.yaml`, bag `rcm_error` | parse YAML + bag |
| **F1.1** | Annotated photo of the whole setup | **take photos** | – |

## Chapter 3–4

| # | Figure / table | Data / how |
|---|---|---|
| F3.1 | Version timeline V1–V8 (date, change, lag) | VERSIONS.md summary table |
| F4.1 | Hardware with frames (base, tool0, camera, stylus) | photo + overlay |
| F4.2 | Software architecture / signal chain with rates | `docs/system_architecture.drawio` (export PDF) |
| F4.3 | Replay pipeline (original run → STYLUS_RAW → replay → new run → metrics) | draw.io / TikZ |
| T4.1 | Output modes (controller, UR command, rate, versions) | VERSIONS.md |
| T4.2 | Metric definitions | – |
| T4.3 | Hardware specifications | datasheets + measured |

## Chapter 5

| # | Figure / table | Data / how |
|---|---|---|
| F5.1 | Lag (position/angle) by version, real vs sim | VERSIONS.md + METRICS |
| F5.2 | Commanded vs delivered displacement: ratio 0.60 (trajectory) vs 1.04 (velocity), real and sim | ⚠ re-create from V3 logs + SIM_latency bags (integrate twist vs FK displacement) |
| T5.1 | Sim latency study (configurations → lag / RMS / jitter) | VERSIONS.md V4 table |
| F5.3 | Spectra: stylus (tremor 7–13 Hz), command (feedforward vs gain terms), tool, before/after | ⚠ from bags (`/phantom/pose`, twist topic, `/joint_states` FK) |
| F5.4 | Wrist torque-chatter spectra at 100 / 250 / 500 Hz command rates | ⚠ `/joint_states` effort from V4 failed runs + 10:35 run (if kept) + V5 |
| T5.2 | Faults: joint, speed, chatter vs run median | VERSIONS.md |
| T5.3 | Command-filter cutoff → residual band content → added lag (+ group-delay formula column) | VERSIONS.md V5 + formula |
| T5.4 | Kalman vs raw noise / lag per run; One Euro comparison | VERSIONS.md V5 |
| T5.5 | Benchmark: trajectory / V5 / V5 tuned / sim prediction | VERSIONS.md V5 tuned table; METRICS_SUMMARY.csv |
| F5.6 | Stick-slip: robot tool speed dips aligned with Touch pitch speed (2–3 annotated events) | bag `/phantom/joint_states` + FK, 11:33 run ⚠ (check the run was kept; else describe with the recorded table) |
| F5.7 | Buzz: big-joint current spectrogram, LargeVibrations vs NoBuzzArea, with mean poses | `logs/V7/LargeVibrations` + `logs/V7/NoBuzzArea` (the 16:43 run) |
| F5.8 | Position vs velocity output (sim): robot-behind-Servo histogram, lag | SIM runs 2026-10-30 (if kept) / re-run |

## Chapter 6

| # | Figure / table | Data / how |
|---|---|---|
| F6.1 | Condition number vs time, guarded vs unguarded; joint speeds | re-run the kinematic sim (offline script) |
| F6.2 | Guard geometry: into-singularity component removed, slide along edge | TikZ sketch |
| T6.1 | OpenHaptics lift-test matrix | VERSIONS.md (copy verbatim) |
| F6.3 | touch_server ↔ touch_bridge process split (UDP) | TikZ |
| F6.4 | Force staircase: pen displacement vs time for 0.001–0.03 N steps | bench data (check if the lift/staircase recordings were saved; else table) |
| T6.2 | Safeguards: what, failure prevented, test | R6.8 |

## Chapter 7

| # | Figure / table | Data / how |
|---|---|---|
| F7.1 | RCM geometry (tip, point c, tool0, lever r(d), θ, sideways s) | TikZ |
| F7.3 | tool0 sideways speed during in/out with sideways offset: fade vs law | offline RcmTool sim |
| F7.4 | NDI rcm_error vs time, one replay per set | NDI_PATH.csv |
| F7.6 | **RCM error budget** per trajectory set: controller / kinematic model / hand-eye + NDI floor / residual, factory vs calibrated | E0 + RD1 (`07_remaining_experiments.md` §1.6) |
| F7.5 | Teleop-latched vs NDI-latched point and pivots in base_link (3D or 2 projections) | `scripts/rcm_offset_check.py` output |
| T7.1 | Replay results (rms, p95, within 2 mm, swing; controller's own error) | REPLAYS_2026-10-26.md |
| T7.2 | Twist-construction variants → offline error (5.8 / 2.6 / 4.2 / 0.45 mm) | VERSIONS.md V6 part 2 |

## Chapter 8

| # | Figure / table | Data / how |
|---|---|---|
| F8.1 | Coupling exploded view + photo | Onshape render (doc e933569c…, V2/V3 studios) |
| F8.2 | Cam / preload free-body diagram (107 N, 1.2 N·m) | TikZ |
| F8.3 | Tip re-seat scatter (3 sessions × 20 swaps), with a 1 mm circle | `logs/Exchange/*/NDI_PATH.csv` via `planning/analysis/exchange_repeatability.py` |
| F8.4 | Swap-time histogram vs the 20 s threshold | same |
| T8.1 | Calibration methods: method, residual, check | R4.6, R8.4 |
| F8.5 | Micro module step response (sim): before/after damping fix | re-run the fake-Arduino test |
| F8.6 | Mode-switch state machine (macro / micro / recording / RCM chord) | TikZ |

## Appendix

| # | Content |
|---|---|
| T-A1 | Full version table |
| T-C1 | Final settings |
| F-F* | All NDI_PLOT.png, per-run METRICS tables |
| F-D* | Coupling drawings |
